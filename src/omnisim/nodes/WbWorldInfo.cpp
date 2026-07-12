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

#include "WbWorldInfo.hpp"

#include "WbApplicationInfo.hpp"
#include "WbContactProperties.hpp"
#include "WbDamping.hpp"
#include "WbField.hpp"
#include "WbFieldChecker.hpp"
#include "WbGroup.hpp"
#include "WbMFNode.hpp"
#include "WbMFString.hpp"
#include "WbMathsUtilities.hpp"
#include "WbOdeContext.hpp"
#include "WbParser.hpp"
#include "WbPreferences.hpp"
#include "WbProtoTemplateEngine.hpp"
#include "WbReceiver.hpp"
#include "WbSFNode.hpp"
#include "WbSFVector3.hpp"
#include "WbSolid.hpp"
#include "WbTokenizer.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <QtCore/QRegularExpression>

void WbWorldInfo::init(const WbVersion *version) {
  mInfo = findMFString("info");
  mTitle = findSFString("title");
  mWindow = findSFString("window");
  mCfm = findSFDouble("CFM");
  mErp = findSFDouble("ERP");
  mPhysics = findSFString("physics");
  mBroadphase = findSFString("broadphase");
  mNewtonSolver = findSFString("newtonSolver");
  mNewtonSubsteps = findSFInt("newtonSubsteps");
  mNewtonStatics = findSFBool("newtonStatics");
  mNewtonRobotColliders = findSFBool("newtonRobotColliders");
  mNewtonCompoundColliders = findSFBool("newtonCompoundColliders");
  mDefaultPhysicsBackend = findSFString("defaultPhysicsBackend");
  mDefaultRenderBackend = findSFString("defaultRenderBackend");
  mBasicTimeStep = findSFDouble("basicTimeStep");
  mFps = findSFDouble("FPS");
  mOptimalThreadCount = findSFInt("optimalThreadCount");
  mPhysicsDisableTime = findSFDouble("physicsDisableTime");
  mPhysicsDisableLinearThreshold = findSFDouble("physicsDisableLinearThreshold");
  mPhysicsDisableAngularThreshold = findSFDouble("physicsDisableAngularThreshold");
  mDefaultDamping = findSFNode("defaultDamping");
  mInkEvaporation = findSFDouble("inkEvaporation");
  mGravity = findSFDouble("gravity");
  mCoordinateSystem = findSFString("coordinateSystem");
  WbField *northDirectionField = findField("northDirection");
  const WbSFVector3 *const northDirection = findSFVector3("northDirection");
  if (version && *version < WbVersion(2020, 1, 0, true)) {
    mGravity->setValue(WbParser::legacyGravity());
    mCoordinateSystem->setValue("NUE");  // default value for Webots < R2020b
    if (northDirection->value() == WbVector3(1.0, 0.0, 0.0))
      northDirectionField->reset();
    else if (northDirection->value() == WbVector3(0.0, 0.0, 1.0)) {
      northDirectionField->reset();
      mCoordinateSystem->setValue("EUN");
    } else if (!northDirectionField->isDefault())
      parsingWarn(tr("The 'northDirection' field is deprecated, according to the 'coordinateSystem' field, the north is "
                     "aligned along the x-axis."));
  } else if (northDirection->value() == WbVector3(0.0, 0.0, 1.0) && mCoordinateSystem->value() == "NUE") {
    northDirectionField->reset();
    mCoordinateSystem->setValue("EUN");
  } else if (!northDirectionField->isDefault())
    parsingWarn(tr("The 'northDirection' field is deprecated, please use the 'coordinateSystem' field instead."));

  WbProtoTemplateEngine::setCoordinateSystem(mCoordinateSystem->value());
  mGpsCoordinateSystem = findSFString("gpsCoordinateSystem");
  mGpsReference = findSFVector3("gpsReference");
  mLineScale = findSFDouble("lineScale");
  mDragForceScale = findSFDouble("dragForceScale");
  mDragTorqueScale = findSFDouble("dragTorqueScale");
  mRandomSeed = findSFInt("randomSeed");
  mContactProperties = findMFNode("contactProperties");

  mPhysicsReceiver = NULL;

  if (findSFString("fast2d")->value() != "")
    parsingWarn(tr("fast2d plugin are not supported anymore, if you don't want to simulate dynamic, you can use the built-in "
                   "kinematic mode of OmniSim."));
}

WbWorldInfo::WbWorldInfo(WbTokenizer *tokenizer) : WbBaseNode("WorldInfo", tokenizer) {
  init(tokenizer ? &tokenizer->fileVersion() : &WbApplicationInfo::version());
}

WbWorldInfo::WbWorldInfo(const WbWorldInfo &other) : WbBaseNode(other) {
  init();
}

WbWorldInfo::WbWorldInfo(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbWorldInfo::~WbWorldInfo() {
  delete mPhysicsReceiver;
}

void WbWorldInfo::downloadAssets() {
  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    WbContactProperties *const cp = static_cast<WbContactProperties *>(mContactProperties->item(i));
    cp->downloadAssets();
  }
}

void WbWorldInfo::preFinalize() {
  WbBaseNode::preFinalize();

  if (defaultDamping())
    defaultDamping()->preFinalize();

  if (!mPhysics->value().isEmpty() || mPhysics->value() != "<none>")
    mPhysicsReceiver = WbReceiver::createPhysicsReceiver();

  updateGravity();
  updateCfm();
  updateErp();
  updateBasicTimeStep();
  updateFps();
  updateLineScale();
  updateDragForceScale();
  updateDragTorqueScale();
  updateRandomSeed();
  updateDefaultDamping();
  updateGpsCoordinateSystem();
  WbProtoTemplateEngine::setCoordinateSystem(mCoordinateSystem->value());

  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    WbContactProperties *const cp = static_cast<WbContactProperties *>(mContactProperties->item(i));
    cp->preFinalize();
  }
}

void WbWorldInfo::postFinalize() {
  WbBaseNode::postFinalize();

  if (defaultDamping())
    defaultDamping()->postFinalize();

  connect(mTitle, &WbSFString::changed, this, &WbWorldInfo::titleChanged);
  connect(mGravity, &WbSFDouble::changed, this, &WbWorldInfo::updateGravity);
  connect(mCfm, &WbSFDouble::changed, this, &WbWorldInfo::updateCfm);
  connect(mErp, &WbSFDouble::changed, this, &WbWorldInfo::updateErp);
  connect(mBasicTimeStep, &WbSFDouble::changed, this, &WbWorldInfo::updateBasicTimeStep);
  connect(mOptimalThreadCount, &WbSFInt::changed, this, &WbWorldInfo::updateOptimalThreadCount);
  connect(mOptimalThreadCount, &WbSFInt::changed, this, &WbWorldInfo::displayOptimalThreadCountWarning);
  connect(mFps, &WbSFDouble::changed, this, &WbWorldInfo::updateFps);
  connect(mLineScale, &WbSFDouble::changed, this, &WbWorldInfo::updateLineScale);
  connect(mDragForceScale, &WbSFDouble::changed, this, &WbWorldInfo::updateDragForceScale);
  connect(mDragTorqueScale, &WbSFDouble::changed, this, &WbWorldInfo::updateDragTorqueScale);
  connect(mRandomSeed, &WbSFInt::changed, this, &WbWorldInfo::updateRandomSeed);
  connect(mPhysicsDisableTime, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableLinearThreshold, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableAngularThreshold, &WbSFDouble::changed, this, &WbWorldInfo::physicsDisableChanged);
  connect(mDefaultDamping, &WbSFNode::changed, this, &WbWorldInfo::updateDefaultDamping);
  connect(mCoordinateSystem, &WbSFString::changed, this, &WbWorldInfo::updateCoordinateSystem);
  connect(mCoordinateSystem, &WbSFString::changed, this, &WbWorldInfo::updateGravity);

  connect(mGpsCoordinateSystem, &WbSFString::changed, this, &WbWorldInfo::updateGpsCoordinateSystem);
  connect(mGpsReference, &WbSFString::changed, this, &WbWorldInfo::gpsReferenceChanged);

  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    WbContactProperties *const cp = static_cast<WbContactProperties *>(mContactProperties->item(i));
    cp->postFinalize();
    connect(cp, &WbContactProperties::valuesChanged, this, &WbWorldInfo::updateContactProperties);
  }

  connect(mContactProperties, &WbMFNode::changed, this, &WbWorldInfo::updateContactProperties);

  WbWorld::instance()->setWorldInfo(this);
}

void WbWorldInfo::reset(const QString &id) {
  WbBaseNode::reset(id);

  for (int i = 0; i < mContactProperties->size(); ++i)
    mContactProperties->item(i)->reset(id);
  WbNode *const d = mDefaultDamping->value();
  if (d)
    d->reset(id);
}

double WbWorldInfo::lineScale() const {
  return mLineScale->value();
}

int WbWorldInfo::contactPropertiesCount() const {
  return mContactProperties->size();
}

WbContactProperties *WbWorldInfo::contactProperties(int index) const {
  return static_cast<WbContactProperties *>(mContactProperties->item(index));
}

WbDamping *WbWorldInfo::defaultDamping() const {
  return static_cast<WbDamping *>(mDefaultDamping->value());
}

void WbWorldInfo::createWrenObjects() {
  WbBaseNode::createWrenObjects();
  WbWrenRenderingContext::instance()->setLineScale(static_cast<float>(lineScale()));
}

void WbWorldInfo::createOdeObjects() {
  WbBaseNode::createOdeObjects();

  // §8.4 of rendering-roadmap.md (landed): apply the broadphase
  // field. Runs before any Solid's createOdeObjects, so
  // the ODE collision space is still empty and the switch is safe.
  // Worlds that don't set the field stay on "simple" (the default),
  // which is the current behavior -- byte-equivalent.
  //
  // "auto" gets resolved here -- not down in WbOdeContext -- because
  // the decision depends on world geometry (the top-level Solids
  // are populated by WbWorld::updateTopLevelLists during the world
  // ctor, so we can read their positions). WbOdeContext gets handed
  // the already-resolved value ("sap" or "quadtree"); it still
  // accepts "auto" as a defensive fallback (→ "sap") for any caller
  // that hasn't done the resolution.
  WbOdeContext::instance()->setBroadphaseType(resolveBroadphaseChoice().toUtf8().constData());

  applyToOdeGravity();
  applyToOdeCfm();
  applyToOdeErp();
  applyToOdeGlobalDamping();
  applyToOdePhysicsDisableTime();
}

QString WbWorldInfo::resolveBroadphaseChoice() const {
  // Pass-through for explicit choices. Only "auto" needs geometric
  // resolution.
  const QString &requested = mBroadphase->value();
  if (requested != "auto")
    return requested;

  // Auto-mode rule from rendering-roadmap.md §8.4:
  // quadtree when the world spans > 500 m on either horizontal axis,
  // else sap. The horizontal axes are X and Y under any coordinate
  // system OmniSim supports (ENU/NUE/EUN) -- the up axis varies but
  // the horizontal plane is always two of {x, y, z}, so taking the
  // two-largest-span axes from the bounding box gets it right
  // regardless of "up".
  //
  // We sample positions from top-level Solids only -- nested Solids
  // can't displace their containing top-level much further than
  // their parent, so the top-level extent is a good proxy for total
  // world extent. Top-level translations are world-space (no parent
  // transform above them by definition).
  const QList<WbSolid *> &topSolids = WbWorld::instance()->topSolids();
  if (topSolids.isEmpty())
    return QStringLiteral("sap");  // empty world -> default modern broadphase

  double minX = topSolids.first()->translation().x();
  double maxX = minX;
  double minY = topSolids.first()->translation().y();
  double maxY = minY;
  double minZ = topSolids.first()->translation().z();
  double maxZ = minZ;
  for (const WbSolid *const s : topSolids) {
    const double x = s->translation().x();
    const double y = s->translation().y();
    const double z = s->translation().z();
    if (x < minX) minX = x; else if (x > maxX) maxX = x;
    if (y < minY) minY = y; else if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; else if (z > maxZ) maxZ = z;
  }

  // The plan's threshold is "horizontal extent > 500 m". Determining
  // which axes are horizontal needs the coordinate system (ENU has
  // up=Z, NUE has up=Y, EUN has up=Y). The simpler coordinate-system-
  // agnostic substitute is: largest single-axis span > 500 m. That
  // rule errs slightly on the side of "use quadtree" for very tall
  // narrow worlds (a 1 km tower in a 100 m yard would pick quadtree),
  // but quadtree at the 2 km default extents handles a tall tower
  // fine and the only cost of a false positive is a slightly fancier
  // broadphase than necessary -- not a correctness issue.
  const double spanX = maxX - minX;
  const double spanY = maxY - minY;
  const double spanZ = maxZ - minZ;
  const double maxSpan = spanX > spanY ? (spanX > spanZ ? spanX : spanZ) : (spanY > spanZ ? spanY : spanZ);

  return maxSpan > 500.0 ? QStringLiteral("quadtree") : QStringLiteral("sap");
}

void WbWorldInfo::updateBasicTimeStep() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mBasicTimeStep, 32.0);
}

void WbWorldInfo::updateFps() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mFps, 60.0);
}

void WbWorldInfo::displayOptimalThreadCountWarning() {
  int threadPreferenceNumber = WbPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > 1 and threadPreferenceNumber > 1)
    parsingWarn(
      tr("Physics multi-threading is enabled. "
         "This can have a noticeable impact on the simulation speed (negative or positive depending on the simulated world). "
         "In case of multi-threading, simulation replicability is not guaranteed. "));
}

void WbWorldInfo::updateOptimalThreadCount() {
  // this function is called only on field update,
  // loading a world where the 'optimalThreadCount' field is higher than the limit set in the preferences will therefore not
  // raise any warning
  int threadPreferenceNumber = WbPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > threadPreferenceNumber)
    parsingWarn(tr("A limit of '%1' threads is set in the preferences.").arg(threadPreferenceNumber));
  else if (!WbFieldChecker::resetIntIfNonPositive(this, mOptimalThreadCount, 1))
    emit optimalThreadCountChanged();
}

void WbWorldInfo::updateLineScale() {
  if (WbFieldChecker::resetDoubleIfNegative(this, mLineScale, 0.0))
    return;

  if (areWrenObjectsInitialized())
    applyLineScaleToWren();
}

void WbWorldInfo::applyLineScaleToWren() {
  WbWrenRenderingContext::instance()->setLineScale(static_cast<float>(mLineScale->value()));
}

void WbWorldInfo::updateDragForceScale() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mDragForceScale, 30.0);
}

void WbWorldInfo::updateDragTorqueScale() {
  WbFieldChecker::resetDoubleIfNonPositive(this, mDragTorqueScale, 5.0);
}

void WbWorldInfo::updateRandomSeed() {
  WbFieldChecker::resetIntIfNegativeAndNotDisabled(this, mRandomSeed, 0, -1);
  emit randomSeedChanged();
}

void WbWorldInfo::applyToOdePhysicsDisableTime() {
  WbOdeContext::instance()->setPhysicsDisableTime(mPhysicsDisableTime->value());
}

void WbWorldInfo::updateGravity() {
  updateGravityBasis();
  if (areOdeObjectsCreated())
    applyToOdeGravity();
}

void WbWorldInfo::applyToOdeGravity() {
  WbOdeContext::instance()->setGravity(mGravityVector.x(), mGravityVector.y(), mGravityVector.z());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateCfm() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mCfm, 0.00001))
    return;

  if (areOdeObjectsCreated())
    applyToOdeCfm();
}

void WbWorldInfo::applyToOdeCfm() {
  WbOdeContext::instance()->setCfm(mCfm->value());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateErp() {
  if (WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mErp, 0.0, 1.0, 0.2))
    return;

  if (areOdeObjectsCreated())
    applyToOdeErp();
}

void WbWorldInfo::applyToOdeErp() {
  WbOdeContext::instance()->setErp(mErp->value());
  emit globalPhysicsPropertiesChanged();
}

void WbWorldInfo::updateDefaultDamping() {
  if (areOdeObjectsCreated())
    applyToOdeGlobalDamping();
}

void WbWorldInfo::applyToOdeGlobalDamping() {
  const WbDamping *const damping = defaultDamping();
  if (damping) {
    connect(damping, &WbDamping::changed, this, &WbWorldInfo::updateDefaultDamping, Qt::UniqueConnection);

    // convert damping per second (specified in the Scene Tree) in damping per step (for ODE)
    const double ts = basicTimeStep() * 0.001;
    const double linear = 1.0 - pow(1.0 - damping->linear(), ts);
    const double angular = 1.0 - pow(1.0 - damping->angular(), ts);
    WbOdeContext::instance()->setDamping(linear, angular);
  } else {
    // default ODE damping is 0.0
    WbOdeContext::instance()->setDamping(0.0, 0.0);
  }

  emit globalPhysicsPropertiesChanged();
}

// Computes an orthonormal basis whose 'yaw unit vector' is the opposite of the normalized gravity vector
void WbWorldInfo::updateGravityBasis() {
  const QString &system = mCoordinateSystem->value();
  assert(system.size() == 3);
  mNorthVector = WbVector3(system[0] == 'N' ? 1 : 0, system[1] == 'N' ? 1 : 0, system[2] == 'N' ? 1 : 0);
  mEastVector = WbVector3(system[0] == 'E' ? 1 : 0, system[1] == 'E' ? 1 : 0, system[2] == 'E' ? 1 : 0);
  mUpVector = WbVector3(system[0] == 'U' ? 1 : 0, system[1] == 'U' ? 1 : 0, system[2] == 'U' ? 1 : 0);
  mGravityUnitVector = -mUpVector;
  mGravityVector = mGravityUnitVector * mGravity->value();
}

void WbWorldInfo::updateCoordinateSystem() {
  warn(tr("Please save and revert the world so that the change of coordinate system is taken into account when reloading "
          "procedural PROTO nodes."));
}

void WbWorldInfo::updateGpsCoordinateSystem() {
  if (mGpsCoordinateSystem->value() != "local" && mGpsCoordinateSystem->value() != "WGS84") {
    mGpsCoordinateSystem->setValue("local");
    parsingWarn(tr("'gpsCoordinateSystem' must either be 'local' or 'WGS84'. Reset to default value 'local'."));
  }
  emit gpsCoordinateSystemChanged();
}

void WbWorldInfo::updateContactProperties() {
  if (areOdeObjectsCreated())
    emit globalPhysicsPropertiesChanged();
}
