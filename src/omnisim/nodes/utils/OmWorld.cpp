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

#include "OmWorld.hpp"

#include "OmApplication.hpp"
#include "OmBackground.hpp"
#include "OmBallJointParameters.hpp"
#include "OmBasicJoint.hpp"
#include "OmBoundingSphere.hpp"
#include "OmFileUtil.hpp"
#include "OmGeometry.hpp"
#include "OmGroup.hpp"
#include "OmHingeJointParameters.hpp"
#include "OmImageTexture.hpp"
#include "OmJoint.hpp"
#include "OmJointDevice.hpp"
#include "OmJointParameters.hpp"
#include "OmLed.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmMFString.hpp"
#include "OmMotor.hpp"
#include "OmNetwork.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeReader.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPbrAppearance.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPerspective.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmPropeller.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmQuaternion.hpp"
#include "OmRenderingDevice.hpp"
#include "OmRobot.hpp"
#include "OmSimulationState.hpp"
#include "OmSlot.hpp"
#include "OmSolid.hpp"
#include "OmStandardPaths.hpp"
#include "OmTemplateManager.hpp"
#include "OmTokenizer.hpp"
#include "OmUrl.hpp"
#include "OmViewpoint.hpp"
#include "OmWorldInfo.hpp"
#include "OmDeformableFrameListener.hpp"
#include "OmWriter.hpp"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QTextStream>


#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>

static OmWorld *gInstance = NULL;
bool OmWorld::cW3dStreaming = false;
bool OmWorld::cPrintExternUrls = false;

OmWorld *OmWorld::instance() {
  return gInstance;
}

OmWorld::OmWorld(OmTokenizer *tokenizer) :
  mWorldLoadingCanceled(false),
  mResetRequested(false),
  mRestartControllers(false),
  mIsModified(false),
  mIsModifiedFromSceneTree(false),
  mWorldInfo(NULL),
  mViewpoint(NULL),
  mViewpointAutoInserted(false),
  mPerspective(NULL),
  mLastAwakeningTime(0.0),
  mIsLoading(true),
  mIsCleaning(false),
  mIsVideoRecording(false) {
  OmDeformableFrameListener::resetLastUpdateTime();
  gInstance = this;
  OmNode::setInstantiateMode(true);
  OmNode::setGlobalParentNode(NULL);
  mRoot = new OmGroup();
  mRoot->setUniqueId(0);
  OmNode::setGlobalParentNode(mRoot);
  mRadarTargets.clear();
  mCameraRecognitionObjects.clear();

  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->startMeasure(OmPerformanceLog::LOADING);

  if (tokenizer) {
    mFileName = tokenizer->fileName();
    mPerspective = new OmPerspective(mFileName);
    mPerspective->load();

    // read/create nodes
    if (log)
      log->startMeasure(OmPerformanceLog::LOADING_PARSE);
    OmNodeReader reader;
    OmApplication::instance()->setWorldLoadingStatus(tr("Parsing nodes"));
    connect(&reader, &OmNodeReader::readNodesHasProgressed, OmApplication::instance(), &OmApplication::setWorldLoadingProgress);
    connect(OmApplication::instance(), &OmApplication::worldLoadingWasCanceled, &reader, &OmNodeReader::cancelReadNodes);
    QList<OmNode *> nodes = reader.readNodes(tokenizer, mFileName);
    disconnect(OmApplication::instance(), &OmApplication::worldLoadingWasCanceled, &reader, &OmNodeReader::cancelReadNodes);
    disconnect(&reader, &OmNodeReader::readNodesHasProgressed, OmApplication::instance(),
               &OmApplication::setWorldLoadingProgress);
    if (log)
      log->stopMeasure(OmPerformanceLog::LOADING_PARSE);
    if (OmApplication::instance()->wasWorldLoadingCanceled()) {
      mWorldLoadingCanceled = true;
      return;
    }
    if (log)
      log->startMeasure(OmPerformanceLog::LOADING_INSTANTIATE);
    OmTemplateManager::instance()->blockRegeneration(true);
    const OmField *childrenField = mRoot->findField("children");
    int index = 0;
    OmApplication::instance()->setWorldLoadingStatus(tr("Creating nodes"));
    foreach (OmNode *node, nodes) {
      index++;
      OmApplication::instance()->setWorldLoadingProgress(index * 100 / nodes.size());
      if (OmApplication::instance()->wasWorldLoadingCanceled()) {
        mWorldLoadingCanceled = true;
        if (log)
          log->stopMeasure(OmPerformanceLog::LOADING_INSTANTIATE);
        return;
      }
      QString errorMessage;
      if (OmNodeUtilities::isAllowedToInsert(childrenField, mRoot, errorMessage, OmNode::STRUCTURE_USE,
                                             OmNodeUtilities::slotType(node), node)) {
        node->validate();
        mRoot->addChild(node);
      } else
        mRoot->parsingWarn(errorMessage);
    }
    OmTemplateManager::instance()->blockRegeneration(false);

    // ensure a minimal set of nodes for a functional world
    checkPresenceOfMandatoryNodes();
    if (log)
      log->stopMeasure(OmPerformanceLog::LOADING_INSTANTIATE);
  } else {
    mFileName = OmProject::newWorldPath();
    mPerspective = new OmPerspective(mFileName);
    mPerspective->load();

    // create default nodes
    mWorldInfo = new OmWorldInfo();
    mViewpoint = new OmViewpoint();
    mViewpointAutoInserted = true;
    mRoot->addChild(mWorldInfo);
    mRoot->addChild(mViewpoint);
  }

  OmUrl::setWorldFileName(mFileName);

  OmNode::setGlobalParentNode(NULL);
  updateTopLevelLists();

  // world loading stuff
  connect(root(), &OmGroup::childFinalizationHasProgressed, OmApplication::instance(), &OmApplication::setWorldLoadingProgress);
  connect(root(), &OmGroup::worldLoadingStatusHasChanged, OmApplication::instance(),
          &OmApplication::worldLoadingStatusHasChanged);
  connect(this, &OmWorld::worldLoadingStatusHasChanged, OmApplication::instance(), &OmApplication::setWorldLoadingStatus);
  connect(this, &OmWorld::worldLoadingHasProgressed, OmApplication::instance(), &OmApplication::setWorldLoadingProgress);
  connect(OmApplication::instance(), &OmApplication::worldLoadingWasCanceled, root(), &OmGroup::cancelFinalization);

  OmProtoManager::instance()->setNeedsRobotAncestorCallback(
    [](const QString &nodeType) { return OmNodeUtilities::isDeviceTypeName(nodeType) && nodeType != "Connector"; });
}

void OmWorld::finalize() {
  disconnect(OmApplication::instance(), &OmApplication::worldLoadingWasCanceled, root(), &OmGroup::cancelFinalization);
  disconnect(root(), &OmGroup::worldLoadingStatusHasChanged, OmApplication::instance(),
             &OmApplication::worldLoadingStatusHasChanged);
  disconnect(this, &OmWorld::worldLoadingStatusHasChanged, OmApplication::instance(), &OmApplication::setWorldLoadingStatus);
  disconnect(this, &OmWorld::worldLoadingHasProgressed, OmApplication::instance(), &OmApplication::setWorldLoadingProgress);
  disconnect(root(), &OmGroup::childFinalizationHasProgressed, OmApplication::instance(),
             &OmApplication::setWorldLoadingProgress);
  if (OmApplication::instance()->wasWorldLoadingCanceled())
    mWorldLoadingCanceled = true;

  connect(mRoot, &OmGroup::topLevelListsUpdateRequested, this, &OmWorld::updateTopLevelLists);
  connect(mWorldInfo, &OmWorldInfo::globalPhysicsPropertiesChanged, this, &OmWorld::awake);
  connect(OmNodeOperations::instance(), &OmNodeOperations::nodeAdded, this, &OmWorld::storeAddedNodeIfNeeded);

  if (OmProject::current())
    connect(OmProject::current(), &OmProject::pathChanged, this, &OmWorld::updateProjectPath);

  // check for Solid name clash
  QSet<const QString> topSolidNameSet;
  foreach (OmSolid *s, mTopSolids)
    s->resolveNameClashIfNeeded(false, true, mTopSolids, &topSolidNameSet);

  // Frame the camera on the scene when -- and only when -- the world authored no Viewpoint.
  // This is called from OmSimulationWorld's constructor immediately after root()->finalize(),
  // i.e. after every node's postFinalize(), which is what builds the OmBoundingSphere tree
  // (OmGroup::postFinalize -> OmGroup::recomputeBoundingSphere). The spheres are therefore
  // readable here; earlier -- e.g. in checkPresenceOfMandatoryNodes() -- they are not.
  frameViewpointOnScene();

  // OMNISIM_PROBE_GATE coverage meter (newton-ode-replacement-plan.md W0.2): the world is now fully
  // finalized (every node's postFinalize is done, so the capability gate's subtree walk is DECISIVE here).
  // Record each top-level articulation's resolved physics backend + capability-gate verdict, then exit --
  // authoritative (the REAL OmSolid resolution, not a Python re-derivation that would drift) and COMPLETE
  // (every topSolid, including the pinned/explicit robots a gate-only probe never sees). Inert unless the
  // env var is set. scripts/dev/newton_coverage.py aggregates this across the world corpus.
  if (qEnvironmentVariableIsSet("OMNISIM_PROBE_GATE")) {
    QFile gf(qEnvironmentVariable("OMNISIM_PROBE_GATE"));
    if (gf.open(QIODevice::Append | QIODevice::Text)) {
      foreach (OmSolid *s, mTopSolids) {
        const QString backend = s->effectivePhysicsBackendName();  // "newton" | "auto" (both Newton)
        const char *reason = "capable";
        s->articulationNewtonCapable(&reason);  // capability verdict: "capable" | "mesh" | "joint"
        gf.write(QString("articulation=%1 backend=%2 gate=%3\n")
                   .arg(s->name())
                   .arg(backend)
                   .arg(QString::fromLatin1(reason))
                   .toUtf8());
      }
      gf.close();
    }
    // Terminate IMMEDIATELY without running static destructors: the wgpu/Qt/Newton teardown can hang on
    // heavy worlds, and a hung-exiting process holds resources (port/GPU) that race the meter's next
    // back-to-back launch. The report is already flushed (close() above), so a hard _Exit is clean here.
    std::_Exit(0);
  }
}

OmWorld::~OmWorld() {
  delete mRoot;
  OmNode::cleanup();
  gInstance = NULL;

  delete mPerspective;
}

bool OmWorld::needSaving() const {
  if (mIsModifiedFromSceneTree)
    return true;
  if (OmPreferences::instance()->value("General/disableSaveWarning").toBool())
    return false;
  else
    return mIsModified;
}

void OmWorld::setModifiedFromSceneTree() {
  if (!mIsModifiedFromSceneTree) {
    mIsModifiedFromSceneTree = true;
    setModified();
  }
}

void OmWorld::setModified(bool isModified) {
  if (mIsModified != isModified) {
    mIsModified = isModified;
    emit modificationChanged(isModified);
  }
}

bool OmWorld::saveAs(const QString &fileName) {
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
    return false;

  OmWriter writer(&file, fileName);
  writer.writeHeader(fileName);

  writer << "\n";  // leave one space between header and body regardless of whether there are EXTERNPROTO or not

  // prior to saving the EXTERNPROTO entries to file, purge the unused entries
  OmNodeOperations::instance()->purgeUnusedExternProtoDeclarations();
  const QVector<OmExternProto *> &externProto = OmProtoManager::instance()->externProto();
  for (int i = 0; i < externProto.size(); ++i) {
    const QString &url = OmProtoManager::instance()->formatExternProtoPath(externProto[i]->url());
    writer << QString("%1EXTERNPROTO \"%2\"\n").arg(externProto[i]->isImportable() ? "IMPORTABLE " : "").arg(url);
    if (i == externProto.size() - 1)
      writer << "\n";  // add additional empty line after the last EXTERNPROTO entry
  }

  for (int i = 0; i < mRoot->childCount(); ++i) {
    mRoot->child(i)->write(writer);
    writer << "\n";
  }

  writer.writeFooter();

  mFileName = fileName;
  OmUrl::setWorldFileName(mFileName);

  mIsModified = false;
  mIsModifiedFromSceneTree = false;
  emit modificationChanged(false);

  storeLastSaveTime();

  mRoot->save("__init__");
  return true;
}

bool OmWorld::save() {
  return saveAs(mFileName);
}

bool OmWorld::exportAsHtml(const QString &fileName, bool animation) const {
  assert(fileName.endsWith(".html", Qt::CaseInsensitive));

  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  QString w3dFilename = fileName;
  w3dFilename.replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".w3d");

  QString cssFileName = fileName;
  cssFileName.replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".css");

  bool success = true;

  try {
    // export w3d file
    success = exportAsW3d(w3dFilename);
    if (!success)
      throw tr("Cannot export the w3d file to '%1'").arg(w3dFilename);

    // export css file
    QString typeString = (animation) ? "Animation" : "Scene";
    QString titleString(OmWorld::instance()->worldInfo()->title());
    titleString = titleString.toHtmlEscaped();

    QList<std::pair<QString, QString>> cssTemplateValues;
    cssTemplateValues << std::pair<QString, QString>("%title%", titleString);
    cssTemplateValues << std::pair<QString, QString>("%type%", typeString);

    success = OmFileUtil::copyAndReplaceString(OmStandardPaths::resourcesWebPath() + "templates/w3d_playback.css", cssFileName,
                                               cssTemplateValues);
    if (!success)
      throw tr("Cannot copy the 'w3d_playback.css' file to '%1'").arg(cssFileName);

    // export html file
    QString infoString;
    const OmMFString &info = OmWorld::instance()->worldInfo()->info();
    for (int i = 0; i < info.size(); ++i) {
      QString line = info.itemToString(i, OmPrecision::DOUBLE_MAX);
      line.replace(QRegularExpression("^\""), "");
      line.replace(QRegularExpression("\"$"), "");
      infoString += line + "\n";
    }

    infoString = infoString.toHtmlEscaped();
    infoString.replace("\n", "<br/>");

    QList<std::pair<QString, QString>> templateValues;
    templateValues << std::pair<QString, QString>("%w3dFilename%", QFileInfo(w3dFilename).fileName());
    templateValues << std::pair<QString, QString>("%type%", typeString);
    templateValues << std::pair<QString, QString>("%title%", titleString);
    templateValues << std::pair<QString, QString>("%description%", infoString);
    templateValues << std::pair<QString, QString>(
      "%w3dName%",
      fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".w3d"));
    templateValues << std::pair<QString, QString>(
      "%jpgName%",
      fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".jpg"));
    templateValues << std::pair<QString, QString>(
      "%cssName%",
      fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".css"));
    if (animation)
      templateValues << std::pair<QString, QString>(
        "%jsonName%",
        fileName.split('/').last().replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".json"));
    else
      templateValues << std::pair<QString, QString>("%jsonName%", "");

    success = OmFileUtil::copyAndReplaceString(OmStandardPaths::resourcesWebPath() + "templates/w3d_playback.html", fileName,
                                               templateValues);
    if (!success)
      throw tr("Cannot copy 'w3d_playback.html' to '%1'").arg(fileName);

  } catch (const QString &e) {
    OmLog::error(tr("Cannot export html: '%1'").arg(e), true);
  }

  simulationState->resumeSimulation();
  return success;
}

bool OmWorld::exportAsW3d(const QString &fileName) const {
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
    return false;

  OmWriter writer(&file, fileName);
  write(writer);

  return true;
}

void OmWorld::write(OmWriter &writer) const {
  writer.writeHeader(worldInfo()->title());

  // write nodes
  const int count = mRoot->childCount();
  for (int i = 0; i < count; ++i) {
    mRoot->child(i)->write(writer);
    writer << "\n";
  }
  QStringList list;
  const OmMFString &info = worldInfo()->info();
  const int n = info.size();
  for (int i = 0; i < n; ++i)
    list << info.item(i);
  writer.writeFooter(&list);
}

OmNode *OmWorld::findTopLevelNode(const QString &modelName, int preferredPosition) const {
  OmNode *result = NULL;

  OmMFNode::Iterator it(mRoot->children());
  int position = 0;
  while (it.hasNext()) {
    OmNode *const node = it.next();
    if (node->nodeModelName() == modelName) {
      if (result)
        OmLog::warning(tr("'%1': found duplicate %2 node.").arg(mFileName, modelName), false, OmLog::PARSING);
      else {
        result = node;
        if (position != preferredPosition)
          OmLog::warning(tr("'%1': %2 node should be preferably included at position %3 instead of position %4.")
                           .arg(mFileName)
                           .arg(modelName)
                           .arg(preferredPosition + 1)
                           .arg(position + 1),
                         false, OmLog::PARSING);
      }
    }
    ++position;
  }

  if (!result)
    OmLog::warning(tr("'%1': added missing %2 node.").arg(mFileName, modelName), false, OmLog::PARSING);

  return result;
}

void OmWorld::checkPresenceOfMandatoryNodes() {
  mWorldInfo = static_cast<OmWorldInfo *>(findTopLevelNode("WorldInfo", 0));
  if (!mWorldInfo) {
    mWorldInfo = new OmWorldInfo();
    mRoot->insertChild(0, mWorldInfo);
  }

  mViewpoint = static_cast<OmViewpoint *>(findTopLevelNode("Viewpoint", 1));
  if (!mViewpoint) {
    mViewpoint = new OmViewpoint();
    // The world file authored no Viewpoint at all, so this one carries the bare schema defaults
    // (position -10 0 0, orientation 0 0 1 0) -- a grazing, ground-level, fixed-scale view.
    // Flag it so finalize() can frame it on the scene instead. An AUTHORED Viewpoint never sets
    // this flag and is therefore never touched.
    mViewpointAutoInserted = true;
    mRoot->insertChild(1, mViewpoint);
  }
}

// Load-time auto-framing constants.
//
// These MUST stay in sync with src/python/omniworld/viewpoint.py, the repo's single reference
// implementation of the viewpoint convention (docs/developer/viewpoint-convention.md). That
// module bakes framings into generated/retrofitted .wbt files; this code produces the same
// framing for a world that has no Viewpoint at all.
namespace {
  // Canonical hero direction, expressed in the default Z-up (ENU) frame: the eye sits along
  // this vector *from* the subject. Front-right, ~33 deg elevation. Only the direction matters.
  const OmVector3 gHeroDirection(0.62, -0.66, 0.58);

  // Keeps some air between the subject and the frame borders.
  const double gFramingMargin = 1.3;

  // Viewport aspect (width/height) assumed when the real 3D view has not reported one yet.
  // OmViewpoint::mAspectRatio only becomes meaningful once updateAspectRatio() runs off a render
  // window resize, which has typically not happened at load time (and never happens headless);
  // it is 1.0 until then. 16:9 is the conservative windowed default, matching
  // omniworld.viewpoint.DEFAULT_ASPECT.
  const double gDefaultAspect = 16.0 / 9.0;

  // Sanity bounds on the framing radius, in metres. Below the lower bound the camera would sit
  // inside its own near plane; above the upper bound the world extent is almost certainly
  // garbage (a runaway procedural mesh) and the stock default is the safer answer.
  const double gMinFramingRadius = 0.05;
  const double gMaxFramingRadius = 1.0e4;

  bool isFinite(const OmVector3 &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  // Grow the world-coordinates sphere (c, r) so that it also encloses (c2, r2).
  void encloseSphere(OmVector3 &c, double &r, const OmVector3 &c2, double r2) {
    const OmVector3 delta = c2 - c;
    const double d = delta.length();
    if (d + r2 <= r)  // (c2, r2) is already inside
      return;
    if (d + r <= r2) {  // (c, r) is inside (c2, r2)
      c = c2;
      r = r2;
      return;
    }
    const double newRadius = 0.5 * (d + r + r2);
    if (d > 1e-12)
      c += delta * ((newRadius - r) / d);
    r = newRadius;
  }

  // Reads a finalized node's bounding sphere in world coordinates. Returns false when the sphere
  // is absent, empty or non-finite -- callers must treat that as "no extent", never as (0, 0).
  bool globalSphereOf(OmBaseNode *node, OmVector3 &center, double &radius) {
    if (node == NULL)
      return false;
    OmBoundingSphere *const sphere = node->boundingSphere();
    if (sphere == NULL)
      return false;
    // Force a full (not dirty-only) recompute, as OmViewpoint::moveViewpointToObject does: for
    // graphical uses the cached values are not invalidated on every change.
    sphere->recomputeIfNeeded(false);
    if (sphere->isEmpty())
      return false;
    OmVector3 c;
    double r = 0.0;
    sphere->computeSphereInGlobalCoordinates(c, r);
    if (!std::isfinite(r) || r < 0.0 || !isFinite(c))
      return false;
    center = c;
    radius = r;
    return true;
  }
}  // namespace

void OmWorld::frameViewpointOnScene() {
  // HARD GATE. An authored Viewpoint is a deliberate composition (cinematic worlds, sky
  // showcases, and the projects/samples/demos/worlds/rendering/** render-oracle fixtures whose
  // camera IS the test instrument). Only the auto-inserted default is ever touched.
  if (!mViewpointAutoInserted || mViewpoint == NULL || mWorldInfo == NULL || mWorldLoadingCanceled)
    return;

  // --- 1. what to frame ---------------------------------------------------------------------
  // Prefer the robots: that is what a user opening a world wants to see. Fall back to the whole
  // scene (OmGroup::boundingSphere() on the root already aggregates every child).
  OmVector3 center;
  double radius = 0.0;
  bool valid = false;
  foreach (OmRobot *const robot, mRobots) {
    OmVector3 c;
    double r = 0.0;
    if (!globalSphereOf(robot, c, r))
      continue;
    if (valid)
      encloseSphere(center, radius, c, r);
    else {
      center = c;
      radius = r;
      valid = true;
    }
  }
  if (!valid)
    valid = globalSphereOf(mRoot, center, radius);

  // MANDATORY FALLBACK: an empty or nonsensical extent leaves the schema default (-10 0 0)
  // untouched rather than emitting a NaN or zero-radius camera.
  if (!valid || !std::isfinite(radius) || radius > gMaxFramingRadius)
    return;
  radius = std::max(radius, gMinFramingRadius);

  // --- 2. up axis ---------------------------------------------------------------------------
  // Respect WorldInfo.coordinateSystem rather than hardcoding +Z, and rotate the canonical
  // (Z-up) hero direction into the world's frame the same way OmViewpoint::orbitTo builds its
  // space quaternion.
  OmVector3 up = mWorldInfo->upVector();
  if (!isFinite(up) || up.isNull())
    up = OmVector3(0, 0, 1);
  up.normalize();

  const OmVector3 defaultUp(0, 0, 1);
  const double upDot = up.dot(defaultUp);
  OmQuaternion spaceQuaternion;  // identity
  if (upDot < -0.9999)
    // antiparallel: a vertical flip about +X (about +Z, as orbitTo does, would not flip at all)
    spaceQuaternion = OmQuaternion(OmVector3(1, 0, 0), M_PI);
  else if (upDot < 0.9999) {
    spaceQuaternion = OmQuaternion(defaultUp.cross(up), defaultUp.angle(up));
    spaceQuaternion.normalize();
  }
  OmVector3 direction = spaceQuaternion * gHeroDirection;
  if (!isFinite(direction) || direction.isNull())
    return;
  direction.normalize();

  // --- 3. distance --------------------------------------------------------------------------
  // fieldOfView carries VRML semantics: it is the angle on the LARGER viewport dimension (cf.
  // OmViewpoint::updateFieldOfViewY). The binding constraint is therefore the TIGHT axis, which
  // must be derived from the aspect ratio -- omitting this is what makes tall subjects overflow
  // vertically on a wide window. radius/sin(half) (not tan) is the tangent fit to the sphere.
  //
  // This is the exact form; OmViewpoint::moveViewpointToObject uses radius/(sin(fov/2)*min(a,1/a)),
  // a small-angle approximation of the same quantity.
  const double fov = std::min(std::max(mViewpoint->fieldOfView()->value(), 1e-3), M_PI - 1e-3);
  double aspect = mViewpoint->aspectRatio();
  if (!std::isfinite(aspect) || aspect <= 0.0)
    aspect = gDefaultAspect;
  // long/short viewport ratio, floored at the assumed default: at load time the reported aspect
  // is usually the unset 1.0, and a square framing crops on any wider window.
  const double ratio = std::max((aspect >= 1.0) ? aspect : (1.0 / aspect), gDefaultAspect);
  const double halfTight = atan(tan(fov / 2.0) / ratio);
  if (!std::isfinite(halfTight) || sin(halfTight) < 1e-6)
    return;
  double distance = radius * gFramingMargin / sin(halfTight);

  // never place the eye inside the near plane
  const double nearPlane = mViewpoint->nearField()->value();
  if (std::isfinite(nearPlane) && distance < nearPlane + radius)
    distance = nearPlane + radius;
  if (!std::isfinite(distance) || distance <= 0.0)
    return;

  // --- 4. apply -----------------------------------------------------------------------------
  const OmVector3 eye = center + direction * distance;
  if (!isFinite(eye))
    return;
  mViewpoint->setPosition(eye);
  mViewpoint->lookAt(center, up);
  // OmViewpoint::postFinalize() has already snapshotted the (schema-default) pose for reset;
  // re-snapshot so that a simulation reset restores the framed camera, not `position -10 0 0`.
  mViewpoint->save(mViewpoint->stateId());
}

OmSolid *OmWorld::findSolid(const QString &name) const {
  OmMFNode::Iterator it(mRoot->children());
  while (it.hasNext()) {
    OmSolid *const solidChild = dynamic_cast<OmSolid *>(it.next());
    if (solidChild) {
      OmSolid *const found = solidChild->findSolid(name);
      if (found)
        return found;
    }
  }
  return NULL;
}

QList<OmSolid *> OmWorld::findSolids(bool visibleNodes) const {
  const QList<OmNode *> &allNodes = mRoot->subNodes(true, !visibleNodes, false);
  QList<OmSolid *> allSolids;

  foreach (OmNode *const node, allNodes) {
    // cppcheck-suppress constVariablePointer
    OmSolid *const solid = dynamic_cast<OmSolid *>(node);
    if (solid)
      allSolids.append(solid);
  }

  return allSolids;
}

QList<std::pair<QString, OmMFString *>> OmWorld::listTextureFiles() const {
  QList<std::pair<QString, OmMFString *>> list = mRoot->listTextureFiles();
  return list;
}

// update the list of robots and top level solids
void OmWorld::updateTopLevelLists() {
  mTopSolids.clear();
  mTopSolids = OmNodeUtilities::findSolidDescendants(mRoot);
}

void OmWorld::removeRobotIfPresent(OmRobot *robot) {
  if (!robot)
    return;

  mRobots.removeAll(robot);
  emit robotRemoved(robot);
}

void OmWorld::addRobotIfNotAlreadyPresent(OmRobot *robot) {
  assert(robot);

  // don't add a robot that's already in the global list
  if (mRobots.contains(robot))
    return;

  mRobots.append(robot);
  setUpControllerForNewRobot(robot);
  emit robotAdded(robot);
}

void OmWorld::updateProjectPath(const QString &oldPath, const QString &newPath) {
  const QFileInfo infoPath(mFileName);
  const QFileInfo infoNewPath(newPath);
  const QString newFilename = infoNewPath.absolutePath() + "/worlds/" + infoPath.fileName();
  if (QFile::exists(newFilename)) {
    mFileName = newFilename;
    OmUrl::setWorldFileName(mFileName);
  }
}

void OmWorld::setViewpoint(OmViewpoint *viewpoint) {
  bool viewpointHasChanged = mViewpoint != viewpoint;
  mViewpoint = viewpoint;
  if (viewpointHasChanged)
    emit viewpointChanged();
}

double OmWorld::orthographicViewHeight() const {
  return mViewpoint->orthographicViewHeight();
}

void OmWorld::setOrthographicViewHeight(double ovh) const {
  mViewpoint->setOrthographicViewHeight(ovh);
}

bool OmWorld::reloadPerspective() {
  delete mPerspective;
  mPerspective = new OmPerspective(mFileName);
  return mPerspective->load();
}

void OmWorld::awake() {
  double currentSimulationTime = OmSimulationState::instance()->time();
  if (currentSimulationTime > mLastAwakeningTime) {  // we don't want to awake all the world several times in the same step
    mLastAwakeningTime = currentSimulationTime;
    OmMFNode::Iterator it(mRoot->children());
    while (it.hasNext()) {
      OmGroup *const group = dynamic_cast<OmGroup *>(it.next());
      if (group)
        OmSolid::awakeSolids(group);
    }
  }
}

void OmWorld::retrieveNodeNamesWithOptionalRendering(QStringList &centerOfMassNodeNames, QStringList &centerOfBuoyancyNodeNames,
                                                     QStringList &supportPolygonNodeNames) const {
  centerOfMassNodeNames.clear();
  centerOfBuoyancyNodeNames.clear();
  supportPolygonNodeNames.clear();

  const OmSolid *solid = NULL;
  const QList<OmNode *> &allNodes = mRoot->subNodes(true);
  for (int i = 0; i < allNodes.size(); ++i) {
    solid = dynamic_cast<OmSolid *>(allNodes[i]);
    if (solid && (solid->globalCenterOfMassRepresentationEnabled() || solid->centerOfBuoyancyRepresentationEnabled() ||
                  solid->supportPolygonRepresentationEnabled())) {
      const QString name = solid->computeUniqueName();
      if (solid->globalCenterOfMassRepresentationEnabled())
        centerOfMassNodeNames << name;
      if (solid->centerOfBuoyancyRepresentationEnabled())
        centerOfBuoyancyNodeNames << name;
      if (solid->supportPolygonRepresentationEnabled())
        supportPolygonNodeNames << name;
    }
  }
}

QString OmWorld::logWorldMetrics() const {
  int solidCount = 0;
  int jointCount = 0;
  int geomCount = 0;
  const QList<OmNode *> &allNodes = mRoot->subNodes(true);
  foreach (const OmNode *node, allNodes) {
    if (dynamic_cast<const OmBasicJoint *>(node)) {
      jointCount++;
      continue;
    }
    const OmSolid *solid = dynamic_cast<const OmSolid *>(node);
    if (solid && (solid->isKinematic() || solid->isSolidMerger())) {
      solidCount++;
      continue;
    }
    const OmGeometry *geometry = dynamic_cast<const OmGeometry *>(node);
    if (geometry && !geometry->isInBoundingObject())
      geomCount++;
  }

  return QString("%1 solids, %2 joints, %3 graphical geometries").arg(solidCount).arg(jointCount).arg(geomCount);
}
