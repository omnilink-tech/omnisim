// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#include "OmProtoManager.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmApplicationInfo.hpp"
#include "OmFieldModel.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmMultipleValue.hpp"
#include "OmNetwork.hpp"
#include "OmNode.hpp"
#include "OmParser.hpp"
#include "OmProject.hpp"
#include "OmProtoModel.hpp"
#include "OmProtoTreeItem.hpp"
#include "OmSFNode.hpp"
#include "OmStandardPaths.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmUrl.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QCryptographicHash>
#include <QtCore/QFile>
#include <QtCore/QRegularExpression>
#include <QtCore/QUrl>
#include <QtCore/QXmlStreamReader>

static OmProtoManager *gInstance = NULL;

namespace {
  QByteArray protoFingerprint(const QString &url) {
    const QString path = OmUrl::isWeb(url) ? OmNetwork::instance()->get(url) : url;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
      return QByteArray();

    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file))
      return QByteArray();
    return hash.result();
  }
}  // namespace

OmProtoManager *OmProtoManager::instance() {
  if (!gInstance)
    gInstance = new OmProtoManager();
  return gInstance;
}

OmProtoManager::OmProtoManager() {
  mTreeRoot = NULL;

  mImportedFromSupervisor = false;

  loadOmniSimProtoMap();

  // set 1/1/1970 by default to force a generation of the OmProtoInfos the first time
  mProtoInfoGenerationTime.insert(PROTO_WORLD, QDateTime::fromSecsSinceEpoch(0));
  mProtoInfoGenerationTime.insert(PROTO_PROJECT, QDateTime::fromSecsSinceEpoch(0));
  mProtoInfoGenerationTime.insert(PROTO_EXTRA, QDateTime::fromSecsSinceEpoch(0));
}

// we do not delete the PROTO models here: each PROTO model is automatically deleted when its last PROTO instance is deleted
OmProtoManager::~OmProtoManager() {
  cleanup();

  if (gInstance == this)
    gInstance = NULL;
}

OmProtoModel *OmProtoManager::readModel(const QString &url, const QString &worldPath, const QString &prefix,
                                        const QStringList &baseTypeList) const {
  OmTokenizer tokenizer;
  const QString path = OmUrl::isWeb(url) ? OmNetwork::instance()->get(url) : url;
  int errors = tokenizer.tokenize(path, prefix);
  if (errors > 0)
    return NULL;

  OmParser parser(&tokenizer);
  if (!parser.parseProtoInterface(worldPath))
    return NULL;

  tokenizer.rewind();
  while (tokenizer.peekWord() == "EXTERNPROTO" || tokenizer.peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
    parser.skipExternProto();

  const bool prevInstantiateMode = OmNode::instantiateMode();
  try {
    OmNode::setInstantiateMode(false);
    OmProtoModel *model = new OmProtoModel(&tokenizer, worldPath, url, prefix, baseTypeList);
    OmNode::setInstantiateMode(prevInstantiateMode);
    return model;
  } catch (...) {
    OmNode::setInstantiateMode(prevInstantiateMode);
    return NULL;
  }
}

OmProtoModel *OmProtoManager::findModel(const QString &modelName, const QString &worldPath, const QString &parentFilePath,
                                        const QStringList &baseTypeList) {
  if (modelName.isEmpty())
    return NULL;

  assert(!parentFilePath.isEmpty());  // cannot find a model unless we know where to look

  QString protoDeclaration;

  // nodes imported from a supervisor should only check the IMPORTABLE list
  if (!mImportedFromSupervisor) {
    // check the clipboard buffer
    if (protoDeclaration.isEmpty() && !mExternProtoClipboardBuffer.isEmpty()) {
      foreach (const OmExternProto *item, mExternProtoClipboardBuffer) {
        if (item->name() == modelName)
          protoDeclaration = item->url();
      }
    }

    // determine the location of the PROTO based on the EXTERNPROTO declaration in the parent file
    if (protoDeclaration.isEmpty())
      protoDeclaration = findExternProtoDeclarationInFile(parentFilePath, modelName);
  }

  // for IMPORTABLE proto nodes the declaration is in the EXTERNPROTO list, nodes added with add-node follow a different pipe
  if (protoDeclaration.isEmpty()) {
    foreach (const OmExternProto *proto, mExternProto) {
      if (proto->name() == modelName && proto->isImportable())
        protoDeclaration = proto->url();
    }
    // for supervisor imported nodes, only the first level should be exclusively checked in the IMPORTABLE list
    mImportedFromSupervisor = false;
  }

  // based on the declaration found in the file or in the mExternProto list, check if it's a known model
  if (!protoDeclaration.isEmpty()) {
    foreach (OmProtoModel *model, mModels) {
      // if the resolved url is one among the known ones, return the model
      if (OmUrl::resolveUrl(model->url()) == OmUrl::combinePaths(protoDeclaration, parentFilePath))
        return model;
    }
  }

  // if the protoDeclaration is still empty then none was given so find a valid one using the backwards compatibility mechanism
  if (protoDeclaration.isEmpty()) {
    protoDeclaration = injectDeclarationByBackwardsCompatibility(modelName);
    bool foundProtoVersion = false;
    const OmVersion protoVersion = checkProtoVersion(parentFilePath, &foundProtoVersion);
    if (foundProtoVersion && protoVersion < OmVersion(2022, 1, 0)) {
      const QString backwardsCompatibilityMessage = tr("Please adapt your project to R2025a following these instructions: "
                                                       "https://github.com/omnilink-tech/omnisim/blob/main/docs/guide/upgrading-omnisim.md");
      const QString outdatedProtoMessage =
        tr("'%1' must be converted because EXTERNPROTO declarations are missing.").arg(parentFilePath);
      displayMissingDeclarations(backwardsCompatibilityMessage);
      displayMissingDeclarations(outdatedProtoMessage);
    } else {
      QString url;
      if (protoDeclaration.isEmpty() && isProtoInCategory(modelName, PROTO_OMNISIM))
        url = mOmniSimProtoList.value(modelName)->url();
      else if (OmUrl::isWeb(protoDeclaration))
        url = protoDeclaration;
      else
        url = QDir(QFileInfo(mCurrentWorld).absolutePath()).relativeFilePath(protoDeclaration);
      const QString errorMessage =
        (!protoDeclaration.isEmpty() || isProtoInCategory(modelName, PROTO_OMNISIM)) ?
          tr("Missing declaration for '%1', add: 'EXTERNPROTO \"%2\"' to '%3'.").arg(modelName).arg(url).arg(parentFilePath) :
          tr("Missing declaration for '%1', unknown node.").arg(modelName);

      displayMissingDeclarations(errorMessage);
    }
    if (protoDeclaration.isEmpty())
      return NULL;
  }

  // a PROTO declaration is provided, enforce it
  QString modelPath;  // how the PROTO is referenced
  if (OmUrl::isWeb(protoDeclaration)) {
    // The PROTO is a remote asset, check if it's already cached
    if (OmNetwork::instance()->isCachedWithMapUpdate(protoDeclaration))
      modelPath = protoDeclaration;
  } else if (OmUrl::isLocalUrl(protoDeclaration) || QDir::isRelativePath(protoDeclaration)) {
    // two possibitilies arise if the declaration is local (omnisim://)
    // 1. the parent PROTO is in the cache (all its references are always 'omnisim://'): it may happen if a PROTO references
    // another PROTO (both being cached)
    // 2. the PROTO is actually locally available
    // option (1) needs to be checked first, otherwise in the webots development environment the declarations aren't
    // respected (since a local version of the PROTO exists virtually every time)
    QString parentFile;
    if (OmFileUtil::isLocatedInDirectory(parentFilePath, OmStandardPaths::cachedAssetsPath()))
      // reverse lookup the file in order to establish its original remote path
      parentFile = OmNetwork::instance()->getUrlFromEphemeralCache(parentFilePath);
    else
      parentFile = parentFilePath;

    // extract the prefix from the parent so that we can build the child's path accordingly
    if (OmUrl::isWeb(parentFile)) {
      const QRegularExpression re(OmUrl::remoteAssetRegex(true));
      const QRegularExpressionMatch match = re.match(parentFile);
      if (match.hasMatch()) {
        if (OmUrl::isLocalUrl(protoDeclaration))  // replace the prefix (omnisim://) based on the parent's prefix
          modelPath = protoDeclaration.replace("omnisim://", match.captured(0));
        else  // if it's a relative url, then manufacture a remote url based on the relative path and the parent's path
          modelPath = OmUrl::combinePaths(protoDeclaration, parentFile);
        // if the PROTO tree was built correctly, by definition the child must be cached already too
        assert(OmNetwork::instance()->isCachedNoMapUpdate(modelPath));
      } else {
        OmLog::error(tr("The cascaded URL inferring mechanism is supported only for official OmniSim assets."));
        return NULL;
      }
    }
  }

  if (modelPath.isEmpty()) {
    assert(QFileInfo(parentFilePath).exists());
    modelPath = OmUrl::combinePaths(protoDeclaration, parentFilePath);
  }
  // determine prefix and disk location from modelPath
  const QString modelDiskPath = OmUrl::isWeb(modelPath) && OmNetwork::instance()->isCachedWithMapUpdate(modelPath) ?
                                  OmNetwork::instance()->get(modelPath) :
                                  modelPath;
  const QString prefix = OmUrl::computePrefix(modelPath);  // used to retrieve remote assets (replaces omnisim:// in the body)

  if (!modelPath.isEmpty() && QFileInfo(modelDiskPath).exists()) {
    OmProtoModel *model = readModel(modelPath, worldPath, prefix, baseTypeList);
    if (model == NULL)  // can occur if the PROTO contains errors
      return NULL;
    mModels << model;
    model->ref();
    mModelFingerprints.insert(model, protoFingerprint(model->url()));
    return model;
  }

  return NULL;
}

QString OmProtoManager::findExternProtoDeclarationInFile(const QString &url, const QString &modelName) {
  if (url.isEmpty())
    return QString();

  QFile file(OmUrl::isWeb(url) ? OmNetwork::instance()->get(url) : url);
  if (!file.open(QIODevice::ReadOnly)) {
    OmLog::error(tr("Could not search for EXTERNPROTO declarations in '%1' because the file is not readable.").arg(url));
    return QString();
  }
  QString identifier = modelName;
  identifier.replace("+", "\\+").replace("-", "\\-").replace("_", "\\_");
  const QString regex = QString("^\\s*(?:IMPORTABLE\\s+)?EXTERNPROTO\\s+\"(.*(?:/|\\\\|(?<=\"))%1\\.proto)\"").arg(identifier);
  const QRegularExpression re(regex, QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    if (match.hasMatch())
      return match.captured(1);
  }

  return QString();
}

QMap<QString, QString> OmProtoManager::undeclaredProtoNodes(const QString &filename) {
  QMap<QString, QString> protoNodeList;

  if (!OmWorldFileFormat::isWorldFile(filename))
    return protoNodeList;

  OmTokenizer tokenizer;
  tokenizer.tokenize(filename);
  OmParser parser(&tokenizer);

  // only apply mechanism to worlds prior to R2022b
  if (tokenizer.fileVersion() >= OmVersion(2022, 1, 0))
    return protoNodeList;
  // fill queue with nodes referenced by the world file
  QStringList queue;
  queue << parser.protoNodeList();

  displayMissingDeclarations(tr("Please adapt your project to R2025a following these instructions: "
                                "https://github.com/omnilink-tech/omnisim/blob/main/docs/guide/upgrading-omnisim.md"));

  // list all PROTO nodes which are known
  QMap<QString, QString> localProto;
  foreach (QString path, listProtoInCategory(PROTO_PROJECT))
    localProto.insert(QFileInfo(path).baseName(), path);
  QMap<QString, QString> extraProto;
  foreach (QString path, listProtoInCategory(PROTO_EXTRA))
    extraProto.insert(QFileInfo(path).baseName(), path);
  QMap<QString, QString> omniSimProto;
  foreach (QString path, listProtoInCategory(PROTO_OMNISIM))
    omniSimProto.insert(QUrl(path).fileName().replace(".proto", "", Qt::CaseInsensitive), path);

  QStringList knownProto;
  knownProto << localProto.keys() << extraProto.keys() << omniSimProto.keys();
  knownProto.removeDuplicates();

  while (!queue.empty()) {
    const QString proto = queue.front();
    queue.pop_front();

    // determine PROTO location
    QString url;
    if (localProto.contains(proto))  // check if it's a PROTO local to the project
      url = localProto.value(proto);
    else if (extraProto.contains(proto))  // check if it's a PROTO local to the extra projects
      url = extraProto.value(proto);
    else if (omniSimProto.contains(proto))  // if all else fails, use the official OmniSim proto
      url = omniSimProto.value(proto);
    else
      continue;

    url = OmUrl::resolveUrl(url);
    assert(url.endsWith(".proto", Qt::CaseInsensitive));

    if (!protoNodeList.contains(proto))
      protoNodeList.insert(proto, url);

    if (!OmUrl::isWeb(url)) {
      // open the PROTO file and extract all PROTO nodes it refrerences by brute force (comparing against the known nodes)
      QFile file(url);
      if (file.open(QIODevice::ReadOnly)) {
        const QByteArray &contents = file.readAll();
        foreach (const QString &item, knownProto) {
          QString identifier = item;
          identifier.replace("+", "\\+").replace("-", "\\-").replace("_", "\\_");
          const QRegularExpression re(identifier + "\\s*\\{");
          const QRegularExpressionMatch match = re.match(contents);
          if (match.hasMatch() && !protoNodeList.contains(item))
            queue << item;  // these nodes need to further be analyzed to see what they depend on
        }
      }
    }
  }

  return protoNodeList;
}

void OmProtoManager::retrieveExternProto(const QString &filename, bool reloading) {
  // clear current project related variables
  cleanup();
  mCurrentWorld = filename;
  mReloading = reloading;

  // set the world file as the root of the tree
  mTreeRoot = new OmProtoTreeItem(filename, NULL, false);
  connect(mTreeRoot, &OmProtoTreeItem::finished, this, &OmProtoManager::loadWorld);

  // backwards compatibility mechanism: populate the tree with urls which are not declared by EXTERNPROTO (worlds < R2022b)
  QMapIterator<QString, QString> it(undeclaredProtoNodes(filename));
  while (it.hasNext())
    mTreeRoot->insert(it.next().value());

  // root node of the tree is fully populated, trigger cascaded download
  mTreeRoot->download();
}

void OmProtoManager::retrieveExternProto(const QString &filename) {
  // set the proto file as the root of the tree
  mTreeRoot = new OmProtoTreeItem(filename, NULL, false);
  connect(mTreeRoot, &OmProtoTreeItem::finished, this, &OmProtoManager::protoRetrievalCompleted);
  // trigger download
  mTreeRoot->download();
}

void OmProtoManager::retrieveLocalProtoDependencies() {
  QStringList dependencies;
  // current project
  QDirIterator it(OmProject::current()->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext())
    dependencies << it.next();
  // extra projects
  foreach (const OmProject *project, *OmProject::extraProjects()) {
    QDirIterator i(project->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
    while (i.hasNext())
      dependencies << i.next();
  }

  // create an empty root and populate its children with the dependencies to be downloaded
  mTreeRoot = new OmProtoTreeItem("", NULL, false);
  foreach (const QString &proto, dependencies)
    mTreeRoot->insert(proto);

  connect(mTreeRoot, &OmProtoTreeItem::finished, this, &OmProtoManager::dependenciesAvailable);
  // trigger parallel download
  mTreeRoot->download();
}

void OmProtoManager::protoRetrievalCompleted() {
  disconnect(mTreeRoot, &OmProtoTreeItem::finished, this, &OmProtoManager::protoRetrievalCompleted);

  mTreeRoot->generateSessionProtoList(mSessionProto);
  mTreeRoot->deleteLater();

  emit retrievalCompleted();
}

void OmProtoManager::loadWorld() {
  disconnect(mTreeRoot, &OmProtoTreeItem::finished, this, &OmProtoManager::loadWorld);
  if (!mTreeRoot->error().isEmpty()) {
    foreach (const QString &error, mTreeRoot->error())
      OmLog::error(error);
  }

  // generate mSessionProto based on the resulting tree
  mTreeRoot->generateSessionProtoList(mSessionProto);

  // determine what changed from the previous session (and therefore what is no longer needed)
  QSet<QString> difference = QSet<QString>(mPreviousSessionProto.begin(), mPreviousSessionProto.end())
                               .subtract(QSet<QString>(mSessionProto.begin(), mSessionProto.end()));

  QList<OmProtoModel *>::iterator modelIt = mModels.begin();
  while (modelIt != mModels.end()) {
    OmProtoModel *model = *modelIt;
    const QByteArray currentFingerprint = protoFingerprint(model->url());
    const bool changed = currentFingerprint.isEmpty() || mModelFingerprints.value(model) != currentFingerprint;
    // Parsed models are immutable.  Keep them when their content is unchanged,
    // including local project PROTOs; release the manager's cache reference as
    // soon as the file changed or left the session.  The old world keeps its
    // own reference until normal teardown.
    if (changed || difference.contains(model->url())) {
      mModelFingerprints.remove(model);
      modelIt = mModels.erase(modelIt);
      model->unref();
    } else
      ++modelIt;
  }

  // declare all root PROTO defined at the world level, and inferred by backwards compatibility, to the list of EXTERNPROTO
  foreach (const OmProtoTreeItem *const child, mTreeRoot->children())
    declareExternProto(child->name(), child->url(), child->isImportable());

  // cleanup and load world at last
  mTreeRoot->deleteLater();
  emit worldLoadCompleted(mCurrentWorld, mReloading, true);
}

void OmProtoManager::loadOmniSimProtoMap() {
  if (!mOmniSimProtoList.empty())
    return;  // Webots proto list already generated

  const QString filename(OmStandardPaths::resourcesPath() + QString("proto-list.xml"));
  QFile file(filename);
  if (!file.open(QFile::ReadOnly | QFile::Text)) {
    OmLog::error(tr("Cannot read file '%1'.").arg(filename));
    return;
  }

  QXmlStreamReader reader(&file);
  if (reader.readNextStartElement()) {
    if (reader.name().toString() == "proto-list") {
      while (reader.readNextStartElement()) {
        if (reader.name().toString() == "proto") {
          bool needsRobotAncestor = false;
          QString name, url, baseType, license, licenseUrl, documentationUrl, description, slotType;
          QStringList tags, parameters, parents;
          while (reader.readNextStartElement()) {
            if (reader.name().toString() == "name") {
              name = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "base-type") {
              baseType = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "url") {
              url = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "license") {
              license = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "license-url") {
              licenseUrl = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "documentation-url") {
              documentationUrl = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "description") {
              description = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "slot-type") {
              slotType = reader.readElementText();
              reader.readNext();
            }
            if (reader.name().toString() == "tags") {
              tags = reader.readElementText().split(',', Qt::SkipEmptyParts);
              reader.readNext();
            }
            if (reader.name().toString() == "parameters") {
              parameters = reader.readElementText().split("\\n", Qt::SkipEmptyParts);
              reader.readNext();
            }
            if (reader.name().toString() == "parents") {
              parents = reader.readElementText().split(",", Qt::SkipEmptyParts);
              reader.readNext();
            }
            if (reader.name().toString() == "needs-robot-ancestor") {
              needsRobotAncestor = reader.readElementText() == "true";
              reader.readNext();
            }
          }
          description = description.replace("\\n", "\n");
          // cppcheck-suppress constVariablePointer
          OmProtoInfo *const info = new OmProtoInfo(url, baseType, license, licenseUrl, documentationUrl, description, slotType,
                                                    tags, parameters, parents, needsRobotAncestor);
          mOmniSimProtoList.insert(name, info);
        } else
          reader.raiseError(tr("Expected 'proto' element."));
      }
    } else
      reader.raiseError(tr("Expected 'proto-list' element."));
  } else
    reader.raiseError(tr("The format of 'proto-list.xml' is invalid."));
}

void OmProtoManager::generateProtoInfoMap(int category, bool regenerate) {
  if (!regenerate)
    return;

  QMap<QString, OmProtoInfo *> *map;
  switch (category) {
    case PROTO_WORLD:
      map = &mWorldFileProtoList;
      break;
    case PROTO_PROJECT:
      map = &mProjectProtoList;
      break;
    case PROTO_EXTRA:
      map = &mExtraProtoList;
      break;
    case PROTO_OMNISIM:
      return;  // note: mOmniSimProtoList is loaded, not generated
    default:
      OmLog::error(tr("Cannot select proto list, unknown category '%1'.").arg(category));
      return;
  }

  // flag all as dirty
  QMapIterator<QString, OmProtoInfo *> it(*map);
  while (it.hasNext())
    it.next().value()->setDirty(true);

  // find all proto and instantiate the nodes to build OmProtoInfo (if necessary)
  const QStringList protos = listProtoInCategory(category);
  const QDateTime lastGenerationTime = mProtoInfoGenerationTime.value(category);
  foreach (const QString &protoPath, protos) {
    if (!QFileInfo(protoPath).exists())
      continue;  // PROTO was deleted

    QString protoInferredPath;
    QString protoName;
    const bool isCachedWithMapUpdateProto = OmFileUtil::isLocatedInDirectory(protoPath, OmStandardPaths::cachedAssetsPath());
    if (isCachedWithMapUpdateProto) {  // cached file, infer name from reverse lookup
      protoInferredPath = OmNetwork::instance()->getUrlFromEphemeralCache(protoPath);
      protoName = QUrl(protoInferredPath).fileName().replace(".proto", "", Qt::CaseInsensitive);
    } else {
      protoInferredPath = protoPath;
      protoName = QFileInfo(protoPath).baseName();
    }

    if (!map->contains(protoName) || (QFileInfo(protoPath).lastModified() > lastGenerationTime)) {
      // if it exists but is just out of date, remove previous information
      if (map->contains(protoName)) {
        const OmProtoInfo *info = map->take(protoName);  // remove element from map
        delete info;
      }

      OmProtoInfo *info;
      const bool isOmniSimProto = isProtoInCategory(protoName, PROTO_OMNISIM) &&
                                 (OmUrl::resolveUrl(protoUrl(protoName, PROTO_OMNISIM)) == OmUrl::resolveUrl(protoInferredPath));
      // for distributions, the official PROTO can be used only if it is in the cache, which is not the case in the development
      // environment
      if (isOmniSimProto && (isCachedWithMapUpdateProto || OmUrl::isLocalUrl(protoPath)))
        // the proto is an official one, both in name and url, so copy the info from the one provided in proto-list.xml
        // note: a copy is necessary because other categories can be deleted, but the webots one can't and shouldn't
        info = new OmProtoInfo(*protoInfo(protoName, PROTO_OMNISIM));
      else
        // generate from file and insert it
        info = generateInfoFromProtoFile(protoPath);

      if (info) {
        info->setDirty(false);
        map->insert(protoName, info);
      }
    } else  // no info change necessary
      map->value(protoName)->setDirty(false);
  }

  // delete anything that is still flagged as dirty (it might happen if the PROTO no longer exists in the folders)
  it.toFront();
  while (it.hasNext()) {
    if (it.next().value()->isDirty()) {
      const OmProtoInfo *info = map->take(it.key());  // remove element
      delete info;
    }
  }

  mProtoInfoGenerationTime.remove(category);
  mProtoInfoGenerationTime.insert(category, QDateTime::currentDateTime());
}

QStringList OmProtoManager::listProtoInCategory(int category) const {
  QStringList protos;

  switch (category) {
    case PROTO_WORLD: {
      for (int i = 0; i < mExternProto.size(); ++i) {
        QString protoPath(mExternProto[i]->url());
        // mExternProto contains raw paths, retrieve corresponding disk file
        if (OmUrl::isWeb(protoPath) && OmNetwork::instance()->isCachedWithMapUpdate(protoPath))
          protoPath = OmNetwork::instance()->get(protoPath);

        protos << protoPath;
      }
      break;
    }
    case PROTO_PROJECT: {
      QDirIterator it(OmProject::current()->protosPath(), QStringList() << "*.proto", QDir::Files,
                      QDirIterator::Subdirectories);

      while (it.hasNext())
        protos << it.next();

      break;
    }
    case PROTO_EXTRA: {
      foreach (const OmProject *project, *OmProject::extraProjects()) {
        QDirIterator it(project->protosPath(), QStringList() << "*.proto", QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext())
          protos << it.next();
      }
      break;
    }
    case PROTO_OMNISIM: {
      QMapIterator<QString, OmProtoInfo *> it(mOmniSimProtoList);
      while (it.hasNext())
        protos << it.next().value()->url();
      break;
    }
    default:
      OmLog::error(tr("Cannot list protos, unknown category '%1'.").arg(category));
  }

  return protos;
}

const QMap<QString, OmProtoInfo *> &OmProtoManager::protoInfoMap(int category) const {
  // cppcheck-suppress unassignedVariable
  static QMap<QString, OmProtoInfo *> empty;

  switch (category) {
    case PROTO_WORLD:
      return mWorldFileProtoList;
    case PROTO_PROJECT:
      return mProjectProtoList;
    case PROTO_EXTRA:
      return mExtraProtoList;
    case PROTO_OMNISIM:
      return mOmniSimProtoList;
    default:
      OmLog::error(tr("Cannot retrieve proto info list, unknown category '%1'.").arg(category));
      return empty;
  }
}

const OmProtoInfo *OmProtoManager::protoInfo(const QString &protoName, int category) {
  const QMap<QString, OmProtoInfo *> &map = protoInfoMap(category);
  if (!map.contains(protoName)) {
    OmLog::error(tr("PROTO '%1' does not belong to category '%2'.").arg(protoName).arg(category));
    return NULL;
  }

  return map.value(protoName);
}

bool OmProtoManager::isProtoInCategory(const QString &protoName, int category) const {
  // note: this function should only be called if the category is known to be up to date, otherwise the update will loop
  // forever
  switch (category) {
    case PROTO_WORLD:
      return mWorldFileProtoList.contains(protoName);
    case PROTO_PROJECT:
      return mProjectProtoList.contains(protoName);
    case PROTO_EXTRA:
      return mExtraProtoList.contains(protoName);
    case PROTO_OMNISIM:
      return mOmniSimProtoList.contains(protoName);
    default:
      OmLog::error(tr("Cannot check if '%1' exists because category '%2' is unknown.").arg(protoName).arg(category));
  }

  return false;
}

QString OmProtoManager::protoUrl(const QString &protoName, int category) const {
  const QMap<QString, OmProtoInfo *> &map = protoInfoMap(category);
  if (map.contains(protoName))
    return map.value(protoName)->url();

  return QString();
}

OmProtoInfo *OmProtoManager::generateInfoFromProtoFile(const QString &protoFileName) {
  assert(QFileInfo(protoFileName).exists());
  OmTokenizer tokenizer;
  const int errors = tokenizer.tokenize(protoFileName);
  if (errors > 0)
    return NULL;  // invalid PROTO file

  OmParser parser(&tokenizer);
  if (!parser.parseProtoInterface(mCurrentWorld))
    return NULL;  // invalid PROTO file

  const QString url = OmFileUtil::isLocatedInDirectory(protoFileName, OmStandardPaths::cachedAssetsPath()) ?
                        OmNetwork::instance()->getUrlFromEphemeralCache(protoFileName) :
                        protoFileName;

  tokenizer.rewind();
  OmProtoModel *protoModel = NULL;
  const bool previousInstantiateMode = OmNode::instantiateMode();
  OmNode *previousParent = OmNode::globalParentNode();
  try {
    OmNode::setGlobalParentNode(NULL);
    OmNode::setInstantiateMode(false);
    protoModel = new OmProtoModel(&tokenizer, mCurrentWorld, url);
    OmNode::setInstantiateMode(previousInstantiateMode);
    OmNode::setGlobalParentNode(previousParent);
  } catch (...) {
    OmNode::setInstantiateMode(previousInstantiateMode);
    OmNode::setGlobalParentNode(previousParent);
    return NULL;
  }

  tokenizer.rewind();

  bool needsRobotAncestor = false;
  // establish if it requires a Robot ancestor by checking if it contains devices
  while (tokenizer.hasMoreTokens()) {
    const OmToken *token = tokenizer.nextToken();
    if (token->isIdentifier() && mNeedsRobotAncestorCallback(token->word())) {
      needsRobotAncestor = true;
      break;
    }
  }

  // generate field string (needed by PROTO wizard)
  QStringList parameters;
  foreach (const OmFieldModel *model, protoModel->fieldModels()) {
    const OmValue *defaultValue = model->defaultValue();
    QString vrmlDefaultValue;

    if (defaultValue->type() == WB_SF_NODE) {
      const OmSFNode *sfn = dynamic_cast<const OmSFNode *>(defaultValue);
      if (sfn->value()) {
        QString nodeContent = OmVrmlNodeUtilities::exportNodeToString(sfn->value());
        vrmlDefaultValue = nodeContent.replace(QRegularExpression("[\\s\\n]+"), " ");
      }
    } else
      vrmlDefaultValue = defaultValue->toString();

    const OmMultipleValue *mv = dynamic_cast<const OmMultipleValue *>(defaultValue);
    if (defaultValue->type() == WB_MF_NODE && mv) {
      vrmlDefaultValue = "[ ";
      for (int j = 0; j < mv->size(); ++j)
        vrmlDefaultValue += mv->itemToString(j) + "{} ";
      vrmlDefaultValue += "]";
    }

    QString field;
    field += model->isW3d() ? "w3dField " : "field ";
    field += defaultValue->vrmlTypeName() + " ";
    field += model->name() + " ";
    field += vrmlDefaultValue;
    parameters << field;
  }

  // generate parents string (needed by PROTO wizard)
  QStringList parents = protoModel->parentProtoNames();

  OmProtoInfo *info = new OmProtoInfo(url, protoModel->baseType(), protoModel->license(), protoModel->licenseUrl(),
                                      protoModel->documentationUrl(), protoModel->info(), protoModel->slotType(),
                                      protoModel->tags(), parameters, parents, needsRobotAncestor);

  protoModel->destroy();
  return info;
}

QString OmProtoManager::declareExternProto(const QString &protoName, const QString &protoPath, bool importable,
                                           bool forceUpdate) {
  QString previousUrl;
  const QString expandedProtoPath(OmUrl::resolveUrl(protoPath));
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName) {
      mExternProto[i]->setImportable(mExternProto[i]->isImportable() || importable);
      if (mExternProto[i]->url() != expandedProtoPath) {
        previousUrl = mExternProto[i]->url();
        if (forceUpdate)
          mExternProto[i]->setUrl(expandedProtoPath);
      }
      return previousUrl;
    }
  }

  mExternProto.push_back(new OmExternProto(protoName, expandedProtoPath, importable));
  return previousUrl;
}

void OmProtoManager::purgeUnusedExternProtoDeclarations(const QSet<QString> &protoNamesInUse) {
  for (int i = mExternProto.size() - 1; i >= 0; --i) {
    if (!protoNamesInUse.contains(mExternProto[i]->name()) && !mExternProto[i]->isImportable()) {
      // delete non-importable nodes that have no remaining visible instances
      delete mExternProto[i];
      mExternProto.remove(i);
    }
  }
}

QString OmProtoManager::externProtoUrl(const OmNode *node, bool formatted) const {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == node->modelName()) {
      if (formatted)
        return formatExternProtoPath(mExternProto[i]->url());
      return mExternProto[i]->url();
    }
  }

  // PROTO might be declared in PROTO file instead of world file
  // for example for default PROTO parameter nodes
  if (node->proto()) {
    if (formatted)
      return formatExternProtoPath(node->proto()->url());
    return node->proto()->url();
  }

  assert(false);
  return QString();
}

void OmProtoManager::saveToExternProtoClipboardBuffer(const QString &url) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->url() == url) {
      mExternProtoClipboardBuffer << new OmExternProto(*mExternProto[i]);
      return;
    }
  }
}

void OmProtoManager::saveToExternProtoClipboardBuffer(const QList<const OmNode *> &nodes) {
  foreach (const OmNode *node, nodes) {
    if (!node->proto())
      continue;

    saveToExternProtoClipboardBuffer(node->proto()->url());
  }
}

void OmProtoManager::clearExternProtoClipboardBuffer() {
  qDeleteAll(mExternProtoClipboardBuffer);
  mExternProtoClipboardBuffer.clear();
}

QList<QString> OmProtoManager::externProtoClipboardBufferUrls() const {
  QList<QString> list;
  foreach (const OmExternProto *proto, mExternProtoClipboardBuffer)
    list << proto->url();
  return list;
}

void OmProtoManager::resetExternProtoClipboardBuffer(const QList<QString> &bufferUrls) {
  clearExternProtoClipboardBuffer();
  foreach (const QString &url, bufferUrls)
    saveToExternProtoClipboardBuffer(url);
}

void OmProtoManager::removeImportableExternProto(const QString &protoName, OmNode *root) {
  for (int i = mExternProto.size() - 1; i >= 0; --i) {
    if (mExternProto[i]->name() == protoName) {
      assert(mExternProto[i]->isImportable());
      // only IMPORTABLE nodes should be removed using this function, instantiated nodes are removed when deleting the node
      mExternProto[i]->setImportable(false);
      if (!OmVrmlNodeUtilities::existsVisibleProtoNodeNamed(protoName, root)) {
        delete mExternProto[i];
        mExternProto.remove(i);
      }
      return;  // we can stop since the list is supposed to contain unique elements, and a match was found
    }
  }
}

void OmProtoManager::updateExternProto(const QString &protoName, const QString &url) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName) {
      mExternProto[i]->setUrl(OmUrl::resolveUrl(url));
      // loaded model still refers to previous file, it will be updated on world reload
      return;  // we can stop since the list is supposed to contain unique elements, and a match was found
    }
  }

  assert(false);  // should not be requesting to change something that doesn't exist
}

QString OmProtoManager::formatExternProtoPath(const QString &url) const {
  QString path = url;
  if (OmFileUtil::isLocatedInInstallationDirectory(path, true))
    path.replace(OmStandardPaths::omniSimHomePath(), "omnisim://");
  if (path.startsWith(OmProject::current()->protosPath()))
    path = QDir(QFileInfo(mCurrentWorld).absolutePath()).relativeFilePath(path);

  return path;
}

bool OmProtoManager::isImportableExternProtoDeclared(const QString &protoName) {
  for (int i = 0; i < mExternProto.size(); ++i) {
    if (mExternProto[i]->name() == protoName && mExternProto[i]->isImportable())
      return true;
  }

  return false;
}

void OmProtoManager::cleanup() {
  qDeleteAll(mWorldFileProtoList);
  qDeleteAll(mProjectProtoList);
  qDeleteAll(mExtraProtoList);
  qDeleteAll(mExternProto);

  mUniqueErrorMessages.clear();
  mWorldFileProtoList.clear();
  mProjectProtoList.clear();
  mExtraProtoList.clear();
  mExternProto.clear();

  mPreviousSessionProto = mSessionProto;
  mSessionProto.clear();
}

QString OmProtoManager::injectDeclarationByBackwardsCompatibility(const QString &modelName) {
  // check if it's the current project
  const QStringList projectProto = listProtoInCategory(PROTO_PROJECT);
  foreach (const QString &proto, projectProto) {
    if (proto.contains(modelName + ".proto", Qt::CaseInsensitive))
      return QFileInfo(proto).absoluteFilePath();
  }
  // check if it's in the EXTRA projects
  const QStringList extraProto = listProtoInCategory(PROTO_EXTRA);
  foreach (const QString &proto, extraProto) {
    if (proto.contains(modelName + ".proto", Qt::CaseInsensitive))
      return QFileInfo(proto).absoluteFilePath();
  }
  // check among the  official ones
  if (isProtoInCategory(modelName, PROTO_OMNISIM)) {
    QString url = mOmniSimProtoList.value(modelName)->url();
    if (OmUrl::isWeb(url)) {
      if (OmNetwork::instance()->isCachedWithMapUpdate(url))
        return url;
    }

    if (OmUrl::isLocalUrl(url)) {
      url = QDir::cleanPath(url.replace("omnisim://", OmStandardPaths::omniSimHomePath()));
      if (QFileInfo(url).exists())
        return url;
    }
  }

  return QString();
}

void OmProtoManager::displayMissingDeclarations(const QString &message) {
  if (!mUniqueErrorMessages.contains(message)) {
    mUniqueErrorMessages << message;
    OmLog::error(message);
  }
}

OmVersion OmProtoManager::checkProtoVersion(const QString &protoUrl, bool *foundProtoVersion) {
  QFile protoFile(protoUrl);
  OmVersion protoVersion;
  if (protoFile.open(QIODevice::ReadOnly)) {
    const QByteArray &contents = protoFile.readAll();
    // non-capturing alternation: OmVersion's own regex owns capture groups 1-6
    *foundProtoVersion = protoVersion.fromString(contents, "^(?:OMNISIM|VRML_SIM)", " utf8$");
  }
  return protoVersion;
}
