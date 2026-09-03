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

#include "OmProtoModel.hpp"

#include "OmField.hpp"
#include "OmFieldModel.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmNetwork.hpp"
#include "OmNode.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeReader.hpp"
#include "OmParser.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoTemplateEngine.hpp"
#include "OmStandardPaths.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmUrl.hpp"
#include "OmValue.hpp"

#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QHash>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>


namespace {
  // Per-field regexes for the constructor's two template-regenerator scans, compiled once per
  // (model, field) instead of once per (token, field). Keyed by the OmFieldModel pointer, which is
  // stable for the model's lifetime; cleared when the model is destroyed (see the destructor).
  QHash<const OmFieldModel *, QRegularExpression> &statementFieldRegexCache() {
    static QHash<const OmFieldModel *, QRegularExpression> cache;
    return cache;
  }
  QHash<const OmFieldModel *, QRegularExpression> &stringFieldRegexCache() {
    static QHash<const OmFieldModel *, QRegularExpression> cache;
    return cache;
  }
  const QRegularExpression &statementFieldRegex(const OmFieldModel *model) {
    QHash<const OmFieldModel *, QRegularExpression> &cache = statementFieldRegexCache();
    auto it = cache.find(model);
    if (it == cache.end())
      it = cache.insert(model, QRegularExpression(QString("(^|\\W)fields\\.%1($|\\W)").arg(QRegularExpression::escape(model->name()))));
    return it.value();
  }
  const QRegularExpression &stringFieldRegex(const OmFieldModel *model, const QString &open, const QString &close) {
    QHash<const OmFieldModel *, QRegularExpression> &cache = stringFieldRegexCache();
    auto it = cache.find(model);
    if (it == cache.end())
      it = cache.insert(model, QRegularExpression(QString("%1(?:(?!%2|\").)*fields\\.%3(?:(?!%4|\").)*%5")
                                                     .arg(open)
                                                     .arg(close)
                                                     .arg(QRegularExpression::escape(model->name()))
                                                     .arg(close)
                                                     .arg(close)));
    return it.value();
  }
  void forgetFieldRegexes(const OmFieldModel *model) {
    statementFieldRegexCache().remove(model);
    stringFieldRegexCache().remove(model);
  }

  // Token cache for generateRoot(): the lexed tokens of a PROTO body, keyed by the PROTO url and
  // the exact body text. Every instance of a non-template PROTO, and every instance of a
  // deterministic template with the same parameter key, hands tokenizeString the SAME text and
  // got the same tokens back after a character-by-character lex through a QTextStream --
  // measured 2026-09-02 on city_traffic: 1.0 s for 476 instances of 60 distinct bodies. Tokens
  // carry only word/line/column, so deep copies of the cached vector are indistinguishable from
  // a fresh lex. Bodies that lex with errors are never cached. Bounded: cleared when it grows
  // past 512 entries. OMNISIM_PROTO_TOKEN_CACHE=0 (value-parsed) lexes every instance as before.
  struct CachedTokens {
    QString content;  // the exact body text, compared on lookup so a qHash collision can never serve foreign tokens
    QVector<OmToken *> tokens;
  };
  QHash<QString, CachedTokens> &protoTokenCache() {
    static QHash<QString, CachedTokens> cache;
    return cache;
  }
  bool protoTokenCacheEnabled() {
    static const bool on = []() {
      const QByteArray v = qgetenv("OMNISIM_PROTO_TOKEN_CACHE").trimmed().toLower();
      return v.isEmpty() || (v != "0" && v != "false" && v != "off");
    }();
    return on;
  }
  void clearProtoTokenCache() {
    QHash<QString, CachedTokens> &cache = protoTokenCache();
    for (auto it = cache.begin(); it != cache.end(); ++it)
      qDeleteAll(it.value().tokens);
    cache.clear();
  }
}  // namespace

OmProtoLoadProfile &OmProtoLoadProfile::instance() {
  static OmProtoLoadProfile p;
  return p;
}

bool OmProtoLoadProfile::enabled() {
  static const bool on = !qEnvironmentVariableIsEmpty("OMNISIM_RELOAD_PROFILE");
  return on;
}

QString OmProtoLoadProfile::report() const {
  return QString("proto models read=%1 (%2 ms) instances=%3: template %4 ms + tokenize %5 ms + syntax %6 ms "
                 "+ readNode %7 ms + aliasing %8 ms")
    .arg(modelsRead)
    .arg(modelReadNs / 1000000)
    .arg(instances)
    .arg(templateNs / 1000000)
    .arg(tokenizeNs / 1000000)
    .arg(syntaxNs / 1000000)
    .arg(readNodeNs / 1000000)
    .arg(aliasNs / 1000000);
}

#include <QtCore/QStringList>
#include <QtCore/QTemporaryFile>
#include <QtCore/QTextStream>
#include <QtCore/QUrl>

#include <cassert>

OmProtoModel::OmProtoModel(OmTokenizer *tokenizer, const QString &worldPath, const QString &url, const QString &prefix,
                           QStringList baseTypeList) {
  // nodes in proto parameters or proto body should not be instantiated
  assert(!OmNode::instantiateMode());

  mDerived = false;
  QString baseTypeSlotType;

  mFileVersion = tokenizer->fileVersion();

  mInfo.clear();
  const QString &tokenizerInfo = tokenizer->info();
  if (!tokenizerInfo.isEmpty() && !tokenizerInfo.trimmed().isEmpty()) {
    const QStringList headerInfo = tokenizerInfo.split("\n");  // # comments
    for (int i = 0; i < headerInfo.size(); ++i) {
      if (!headerInfo.at(i).startsWith("tags:") && !headerInfo.at(i).startsWith("license:") &&
          !headerInfo.at(i).startsWith("license url:") && !headerInfo.at(i).startsWith("documentation url:") &&
          !headerInfo.at(i).startsWith("template language:") && !headerInfo.at(i).startsWith("keywords:"))
        mInfo += headerInfo.at(i) + "\n";
    }
    mInfo.chop(1);
  }
  mTags = tokenizer->tags();
  mLicense = tokenizer->license();
  mLicenseUrl = tokenizer->licenseUrl();
  mDocumentationUrl = tokenizer->documentationUrl();
  mIsDeterministic = !mTags.contains("nonDeterministic");
  mHasIndirectFieldAccess = mTags.contains("indirectFieldAccess");

  OmParser parser(tokenizer);
  while (tokenizer->peekWord() == "EXTERNPROTO" || tokenizer->peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
    parser.skipExternProto();

  while (tokenizer->hasMoreTokens() && tokenizer->peekWord() != "PROTO")
    tokenizer->nextToken();
  tokenizer->skipToken("PROTO");
  mName = tokenizer->nextWord();
  // check recursive definition
  if (baseTypeList.contains(mName)) {
    tokenizer->reportError(tr("Recursive definition of PROTO node '%1' is not supported").arg(baseTypeList.first()));
    throw 0;
  }
  // check if PROTO name is not an existing base node name
  if (OmNodeModel::baseModelNames().contains(mName)) {
    tokenizer->reportError(tr("PROTO node '%1' cannot have a base node name").arg(mName));
    throw 0;
  }

  mRefCount = 0;
  mAncestorRefCount = 0;

  mPrefix = prefix;
  mUrl = url;

  assert(mUrl.endsWith(".proto", Qt::CaseInsensitive));      // mUrl needs to be the full reference, including file name
  assert(OmUrl::isWeb(mUrl) || QDir::isAbsolutePath(mUrl));  // by this point, all urls must be resolved

  if (!mUrl.endsWith(mName + ".proto", Qt::CaseInsensitive)) {
    tokenizer->reportFileError(tr("'%1' PROTO identifier does not match filename").arg(mName));
    throw 0;
  }

  // start proto parameters list
  tokenizer->skipToken("[");

  // read proto parameters
  while (tokenizer->hasMoreTokens() && tokenizer->peekWord() != "]") {
    OmFieldModel *parameter = NULL;
    try {
      parameter = new OmFieldModel(tokenizer, worldPath);
    } catch (...) {
      tokenizer->reportError(tr("Errors when parsing the PROTO parameters"), tokenizer->peekToken());
      throw 0;
    }
    // qDebug() << parameter->name() << ((parameter->defaultValue() != NULL) ? parameter->defaultValue()->vrmlTypeName() :
    // "NULL");
    if (findFieldModel(parameter->name())) {
      tokenizer->reportError(tr("Ignoring duplicate '%1' PROTO parameter declaration").arg(parameter->name()),
                             parameter->nameToken());
      parameter->destroy();
    } else {
      parameter->ref();
      mFieldModels.append(parameter);
    }
  }

  tokenizer->skipToken("]");
  tokenizer->skipToken("{");

  const OmToken *token = tokenizer->peekToken();
  int contentLine = token->line() - 1;
  mContentStartingLine = contentLine;
  int contentColumn = token->column() - 1;

  const QString &open = OmProtoTemplateEngine::openingToken();
  const QString &close = OmProtoTemplateEngine::closingToken();

  QFile file(diskPath());
  if (file.open(QIODevice::ReadOnly)) {
    for (int i = 0; i < contentLine; i++)
      file.readLine();
    file.read(contentColumn);

    mContent.clear();

    QTextStream in(&file);
    bool insideTemplateStatement = false;
    while (!in.atEnd()) {
      QString line = in.readLine();

      // Remove comments in order to filter out the template statements using VRML comments.
      // Typical cases to support:
      //   #"my string" # my comment
      //   "my string" # my comment
      //   "my string" # my "quoted comment"
      //   "my string with # character" # my comment
      //   "my string with protected \"quote\"." # my comment
      QString lineWithoutComments;
      bool insideDoubleQuotes = false;
      QChar pc;
      for (int i = 0; i < line.size(); ++i) {
        const QChar c = line[i];
        if (c == open[1] && pc == open[0])
          insideTemplateStatement = true;
        else if (c == close[1] && pc == close[0])
          insideTemplateStatement = false;
        else if (c == '"' && pc != '\\')
          insideDoubleQuotes = !insideDoubleQuotes;
        else if (!insideTemplateStatement && c == '#' && !insideDoubleQuotes)
          // ignore VRML comments
          break;
        lineWithoutComments.append(c);
        pc = c;
      }

      mContent += lineWithoutComments + "\n";
    }
    int lastClosingBracket = mContent.lastIndexOf('}');
    if (lastClosingBracket > 0)
      mContent = mContent.left(lastClosingBracket);
    else {
      tokenizer->reportFileError(tr("Bracket issue"));
      throw 0;
    }
    file.close();
  }

  // Inject the prefix prior to tokenizing the content, replacing the
  // local "omnisim://" scheme.
  if (!mPrefix.isEmpty() && mPrefix != "omnisim://")
    mContent.replace(QString("omnisim://").toUtf8(), mPrefix.toUtf8());

  // read the remaining tokens in order to
  // - determine if it's a template
  // - check which parameter need to regenerate the template instance
  mTemplate = false;
  mAncestorProtoModel = NULL;
  token = NULL;
  const OmToken *previousToken = NULL;
  bool readBaseType = true;
  QStringList sharedParameterNames;
  QString previousRedirectedFieldName;
  bool hasPreviousSlotTypeFieldName = false;
  int insideBrackets = -1;
  while (tokenizer->hasMoreTokens()) {
    previousToken = token;
    token = tokenizer->nextToken();

    if (mHasIndirectFieldAccess) {
      foreach (OmFieldModel *model, mFieldModels)
        model->setTemplateRegenerator(true);
    }

    if (token->isTemplateStatement()) {
      mTemplate = true;

      if (!mHasIndirectFieldAccess) {  // If the proto has indirect field access, we've already set the fields as template
                                       // regenerators
        // A field can only be named through "fields.<name>", so a statement without that substring
        // can mark nothing -- skip the per-field regexes entirely (2026-09-02: they were compiled
        // once per token x field, ~60k QRegularExpression constructions on a 2,000-token template;
        // 31 models read cost 530 ms of the city's parse stage).
        if (token->word().contains(QLatin1String("fields."))) {
          foreach (OmFieldModel *model, mFieldModels) {
            if (model->isTemplateRegenerator())
              continue;
            // condition explanation: if (token contains modelName and not an identifier containing modelName such as
            // "my_awesome_modelName") or (token contains fields and not an identifier containing fields such as "my_fields")
            if (token->word().contains(statementFieldRegex(model)))
              model->setTemplateRegenerator(true);
          }
        }
      }
    } else if (readBaseType) {
      // read base type
      if (token->word() == "DEF") {
        // skip DEF name
        tokenizer->nextToken();
        // read base type
        token = tokenizer->nextToken();
      }

      mBaseType = token->word();
      mDerived = !OmNodeModel::isBaseModelName(mBaseType);
      mAncestorProtoName = "";
      readBaseType = false;

      if (mDerived) {
        bool error = false;
        try {
          baseTypeList.append(mName);
          OmProtoModel *baseProtoModel = OmProtoManager::instance()->findModel(mBaseType, worldPath, url, baseTypeList);
          mAncestorProtoModel = baseProtoModel;
          if (baseProtoModel) {
            mAncestorProtoName = mBaseType;
            mBaseType = baseProtoModel->baseType();
            QStringList derivedParameterNames = parameterNames();
            QStringList baseParameterNames = baseProtoModel->parameterNames();
            baseTypeSlotType = baseProtoModel->slotType();
            foreach (const QString &derivedName, derivedParameterNames) {
              if (baseParameterNames.contains(derivedName))
                sharedParameterNames.append(derivedName);
            }
          } else
            error = true;
        } catch (...) {
          error = true;
        }

        if (error) {
          tokenizer->reportError(tr("Errors when parsing the base PROTO"), token);
          throw 0;
        }

        baseTypeList.removeLast();
      }
    } else if (sharedParameterNames.contains(token->word()) && !previousRedirectedFieldName.isEmpty()) {
      // check that derived parameter is only redirected to corresponding base parameter
      // cppcheck-suppress variableScope
      QString parameterName = token->word();
      if (previousRedirectedFieldName != token->word()) {
        tokenizer->reportError(
          tr("Derived and base PROTO can use the same parameter name only if it is linked directly, like '%1 IS %1'")
            .arg(parameterName));
        throw 0;
      }
    } else if (token->isString()) {
      if (!mHasIndirectFieldAccess) {  // If the proto has indirect field access, we've already set the fields as template
                                       // regenerators
        // check which parameter need to regenerate the template instance from inside a string
          // regex test cases:
          // "You know nothing, John Snow."  => false
          // "%<=fields.model->name()>%"  => true
          // "%<= fields.model->name().value.x >% %<= fields.model->name().value.y >%"  => true
          // "abc %<= fields.model->name().value.y >% def"  => true
          // "%<= 17 % fields.model->name().value.y * 88 >%"  => true
          // "fields.model->name().value.y"  => false
          // "%<>% fields.model->name().value.y %<>%"  => false
          // "%< a = \"fields.model->name().value.y\" >%"  => false
          // "%<= \"fields.model->name().value.y\" >%"  => false
          // "%<= fields.model->name().value.y >%"  => true
        if (token->word().contains(QLatin1String("fields."))) {
          foreach (OmFieldModel *model, mFieldModels) {
            if (model->isTemplateRegenerator())
              continue;
            if (token->word().contains(stringFieldRegex(model, open, close)))
              model->setTemplateRegenerator(true);
          }
        }
      }
    }

    if (mSlotType.isEmpty() && mBaseType == "Slot") {
      if (hasPreviousSlotTypeFieldName) {
        if (token->isString())
          mSlotType = token->toString();
        hasPreviousSlotTypeFieldName = false;
      } else if (insideBrackets == 0 && token->word() == "type") {
        hasPreviousSlotTypeFieldName = true;
      } else if (token->word() == "{") {
        ++insideBrackets;
      } else if (token->word() == "}") {
        --insideBrackets;
      }
    }

    if (token->word() == "IS") {
      assert(previousToken);
      previousRedirectedFieldName = previousToken->word();
    } else {
      previousRedirectedFieldName.clear();
    }
  }

  if (mSlotType.isEmpty() && mBaseType == "Slot" && mDerived)
    mSlotType = baseTypeSlotType;

  if (mDocumentationUrl.isEmpty()) {
    const QStringList &bookAndPage = documentationBookAndPage(mBaseType == "Robot", true);
    if (!bookAndPage.isEmpty())
      mDocumentationUrl =
        QString("%1/%2/%3.md").arg(OmStandardPaths::omniSimDocsBaseUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);
  }
}

OmProtoModel::~OmProtoModel() {
  foreach (const OmFieldModel *model, mFieldModels)
    forgetFieldRegexes(model);
  foreach (const OmFieldModel *model, mFieldModels)
    model->unref();
  mFieldModels.clear();
  mDeterministicContentMap.clear();
}

OmNode *OmProtoModel::generateRoot(const QVector<OmField *> &parameters, const QString &worldPath, int uniqueId) {
  if (mContent.isEmpty())
    return NULL;

  assert(!OmNode::instantiateMode());

  OmTokenizer tokenizer;
  tokenizer.setErrorOffset(mContentStartingLine);

  int rootUniqueId = -1;
  QString content = mContent;
  if (mTemplate) {
    QString key;
    if (mIsDeterministic) {
      foreach (const OmField *parameter, parameters) {
        if (parameter->isTemplateRegenerator()) {
          QString statement = OmProtoTemplateEngine::convertFieldValueToJavaScriptStatement(parameter);
          key += statement;
        }
      }
    }
    if (!mIsDeterministic || (!mDeterministicContentMap.contains(key) || mDeterministicContentMap.value(key).isEmpty())) {
      OmProtoTemplateEngine te(mContent);
      rootUniqueId = uniqueId >= 0 ? uniqueId : OmNode::getFreeUniqueId();
      QElapsedTimer teTimer;
      if (OmProtoLoadProfile::enabled())
        teTimer.start();
      const bool generated = te.generate(name() + ".proto", parameters, mUrl, worldPath, rootUniqueId);
      if (OmProtoLoadProfile::enabled())
        OmProtoLoadProfile::instance().templateNs += teTimer.nsecsElapsed();
      if (!generated) {
        tokenizer.setReferralFile(mUrl);
        tokenizer.reportFileError(tr("Template engine error: %1").arg(te.error()));
        return NULL;
      }
      content = te.result();
      if (mIsDeterministic)
        mDeterministicContentMap.insert(key, content);
    } else
      content = mDeterministicContentMap.value(key);
  } else
    mIsDeterministic = true;

  const bool profiling = OmProtoLoadProfile::enabled();
  QElapsedTimer phase;
  if (profiling)
    phase.start();
  tokenizer.setReferralFile(mUrl);
  const QString tokenKey = protoTokenCacheEnabled()
                             ? mUrl + QLatin1Char('#') + QString::number(qHash(content)) + QLatin1Char('#') +
                                 QString::number(content.size())
                             : QString();
  const auto cachedTokens = tokenKey.isEmpty() ? protoTokenCache().constEnd() : protoTokenCache().constFind(tokenKey);
  if (cachedTokens != protoTokenCache().constEnd() && cachedTokens->content == content) {
    tokenizer.adoptTokens(cachedTokens->tokens);
  } else {
    if (tokenizer.tokenizeString(content) > 0) {
      tokenizer.reportFileError(tr("Failed to load due to syntax error(s)"));
      return NULL;
    }
    if (!tokenKey.isEmpty()) {
      if (protoTokenCache().size() >= 512)
        clearProtoTokenCache();
      CachedTokens entry;
      entry.content = content;
      entry.tokens.reserve(tokenizer.tokens().size());
      foreach (const OmToken *token, tokenizer.tokens())
        entry.tokens.append(new OmToken(*token));
      protoTokenCache().insert(tokenKey, entry);
    }
  }
  if (profiling)
    { OmProtoLoadProfile::instance().tokenizeNs += phase.nsecsElapsed(); phase.restart(); }

  // parse generated PROTO
  OmParser parser(&tokenizer);
  if (!parser.parseProtoBody(worldPath))
    return NULL;
  tokenizer.rewind();
  if (profiling)
    { OmProtoLoadProfile::instance().syntaxNs += phase.nsecsElapsed(); phase.restart(); }

  // read node in a local DEF/USE scope
  OmNode *root = NULL;

  try {
    OmNodeReader reader(OmNodeReader::PROTO_MODEL);
    root = reader.readNode(&tokenizer, worldPath);
  } catch (...) {
    tokenizer.reportFileError(tr("Cannot create the '%1' PROTO").arg(mName));
    return NULL;
  }
  if (profiling) {
    { OmProtoLoadProfile::instance().readNodeNs += phase.nsecsElapsed(); phase.restart(); }
    ++OmProtoLoadProfile::instance().instances;
  }

  if (!root) {
    tokenizer.reportFileError(tr("PROTO has unknown base type"));
    return NULL;
  }

  // aliasing error reports are based on the header, so the error offset has no sense here
  tokenizer.setErrorOffset(0);

  verifyAliasing(root, &tokenizer);
  if (profiling)
    { OmProtoLoadProfile::instance().aliasNs += phase.nsecsElapsed(); phase.restart(); }

  if (mTemplate) {
    root->setProtoInstanceTemplateContent(content.toUtf8());
    if (rootUniqueId >= 0 && rootUniqueId != uniqueId)
      root->setUniqueId(rootUniqueId);
  }

  return root;
}

QStringList OmProtoModel::parentProtoNames() const {
  QStringList parents;
  const OmProtoModel *parentProtoModel = this;
  while ((parentProtoModel = parentProtoModel->ancestorProtoModel()))
    parents << parentProtoModel->name();
  return parents;
}

void OmProtoModel::ref(bool isFromProtoInstanceCreation) {
  mRefCount++;
  if (isFromProtoInstanceCreation)
    mAncestorRefCount++;
}

void OmProtoModel::unref() {
  mRefCount--;
  if (mRefCount < mAncestorRefCount) {
    mAncestorRefCount--;
    if (isDerived() && mAncestorProtoModel)
      mAncestorProtoModel->unref();
  }
  if (mRefCount == 0)
    delete this;
}

void OmProtoModel::destroy() {
  assert(mRefCount == 0);
  delete this;
}

OmFieldModel *OmProtoModel::findFieldModel(const QString &fieldName) const {
  foreach (OmFieldModel *fieldModel, mFieldModels)
    if (fieldModel->name() == fieldName)
      return fieldModel;

  return NULL;  // not found
}

const QString OmProtoModel::projectPath() const {
  QString protoPath = path();

  if (!protoPath.isEmpty()) {
    if (OmUrl::isWeb(protoPath))
      protoPath.replace(QRegularExpression(OmUrl::remoteAssetRegex(false)), OmStandardPaths::omniSimHomePath());
#ifdef __APPLE__
    if (OmFileUtil::isLocatedInInstallationDirectory(protoPath, true))
      protoPath.insert(OmStandardPaths::omniSimHomePath().length(), "Contents/");
#endif

    QDir protoProjectDir(protoPath);
    while (protoProjectDir.dirName() != "protos") {
      QString dir = protoProjectDir.path();
      // cd up (we don't use QDir::cdUp() as it doesn't cd up if the upper folder doesn't exist which may happen here)
      dir.chop(protoProjectDir.dirName().size());
      assert(!dir.isEmpty());
      protoProjectDir.setPath(dir);
      if (protoProjectDir.isRoot())
        return QString();
    }
    // cppcheck-suppress ignoredReturnErrorCode
    protoProjectDir.cdUp();
    return protoProjectDir.absolutePath();
  }
  return QString();
}

QStringList OmProtoModel::parameterNames() const {
  QStringList names;
  foreach (const OmFieldModel *fieldModel, mFieldModels)
    names.append(fieldModel->name());
  return names;
}

void OmProtoModel::setIsTemplate(bool value) {
  mTemplate = value;
  if (mTemplate && mIsDeterministic)
    // if ancestor is nonDeterministic this proto can't be either
    mIsDeterministic = mAncestorProtoModel->isDeterministic();
}

void OmProtoModel::verifyNodeAliasing(OmNode *node, OmFieldModel *param, OmTokenizer *tokenizer, bool searchInParameters,
                                      bool &ok) const {
  QVector<OmField *> fields;
  if (searchInParameters)
    fields = node->parameters();
  else
    fields = node->fields();

  // search self
  foreach (const OmField *field, fields) {
    if (field->alias() == param->name()) {
      if (field->type() == param->type())
        ok = true;
      else
        tokenizer->reportError(
          tr("Type mismatch between '%1' PROTO parameter and field '%2'").arg(param->name(), field->name()),
          param->nameToken());
    }
  }

  // recursively search sub nodes
  const QList<OmNode *> &l = node->subNodes(false, !searchInParameters, true);
  foreach (OmNode *subnode, l) {
    if (subnode->isProtoInstance())
      // search only in parameters of sub protos: fields are in sub proto parameter scope
      verifyNodeAliasing(subnode, param, tokenizer, true, ok);
    else
      verifyNodeAliasing(subnode, param, tokenizer, false, ok);
  }
}

// verify that each proto parameter has at least one matching IS parameter
void OmProtoModel::verifyAliasing(OmNode *root, OmTokenizer *tokenizer) const {
  if (!root)
    return;

  foreach (OmFieldModel *param, mFieldModels) {
    if (param->isUnconnected())
      continue;
    bool ok = false;
    verifyNodeAliasing(root, param, tokenizer, isDerived(), ok);
    if (!isTemplate() && !ok)
      tokenizer->reportError(tr("PROTO parameter '%1' has no matching IS field").arg(param->name()), param->nameToken());
  }
}

QStringList OmProtoModel::documentationBookAndPage(bool isRobot, bool skipProtoTag) const {
  QStringList bookAndPage;
  if (isRobot) {
    // check for robot doc
    const QString &nodeName = mName.toLower();

    const QString page("guide/" + nodeName + ".md");
    if (checkIfDocumentationPageExist(page)) {
      bookAndPage << "guide" << nodeName;
      return bookAndPage;
    }
  } else {
    // check for object doc
    const QDir &objectsDir(OmStandardPaths::projectsPath() + "objects");
    QDir dir(projectPath());
    QString directoryName = dir.dirName().replace('_', '-');
    while (!dir.isRoot()) {
      if (dir == objectsDir) {
        const QString page("guide/object-" + directoryName + ".md");
        if (checkIfDocumentationPageExist(page)) {
          bookAndPage << "guide"
                      << "object-" + directoryName;
          return bookAndPage;
        }
        break;
      }
      directoryName = dir.dirName().replace('_', '-');
      if (!dir.cdUp())
        break;
    }
  }
  if (!skipProtoTag) {
    const QString &rawDocumentationUrl = mDocumentationUrl;
    if (!rawDocumentationUrl.isEmpty()) {
      const QStringList &splittedPath = rawDocumentationUrl.split("doc/");
      if (splittedPath.size() == 2) {
        const QString file(splittedPath[1].split('#')[0]);
        const QString page(file + ".md");
        if (checkIfDocumentationPageExist(page)) {
          bookAndPage = file.split('/');
          if (splittedPath[1].contains('#'))
            bookAndPage[1] += '#' + splittedPath[1].split('#')[1];
          return bookAndPage;
        }
      }
    }
  }

  return bookAndPage;  // return empty
}

bool OmProtoModel::checkIfDocumentationPageExist(const QString &page) const {
  bool exist = false;
  QFile file(OmStandardPaths::omniSimHomePath() + "docs/list.txt");
  if (!file.open(QIODevice::ReadOnly))
    return false;
  QTextStream in(&file);
  QString line = in.readLine();
  while (!line.isNull()) {
    if (line.contains(page, Qt::CaseSensitive)) {
      exist = true;
      break;
    }
    line = in.readLine();
  }

  file.close();

  return exist;
}

const QString OmProtoModel::diskPath() const {
  if (OmUrl::isWeb(mUrl))
    return OmNetwork::instance()->get(mUrl);

  return mUrl;
}

const QString OmProtoModel::path() const {
  if (OmUrl::isWeb(mUrl))
    return QUrl(mUrl).adjusted(QUrl::RemoveFilename).toString();

  return QFileInfo(mUrl).absolutePath() + "/";
}
