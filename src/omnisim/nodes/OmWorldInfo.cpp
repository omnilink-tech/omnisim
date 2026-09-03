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

#include "OmWorldInfo.hpp"

#include "OmApplicationInfo.hpp"
#include "OmContactProperties.hpp"
#include "OmDamping.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmGroup.hpp"
#include "OmMFNode.hpp"
#include "OmMFString.hpp"
#include "OmMathsUtilities.hpp"
#include "OmParser.hpp"
#include "OmPreferences.hpp"
#include "OmProtoTemplateEngine.hpp"
#include "OmSFNode.hpp"
#include "OmSFVector3.hpp"
#include "OmSolid.hpp"
#include "OmTokenizer.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"

#include <QtCore/QRegularExpression>

void OmWorldInfo::init(const OmVersion *version) {
  mInfo = findMFString("info");
  mTitle = findSFString("title");
  mWindow = findSFString("window");
  mCfm = findSFDouble("CFM");
  mErp = findSFDouble("ERP");
  mPhysics = findSFString("physics");
  mBroadphase = findSFString("broadphase");
  mNewtonSolver = findSFString("newtonSolver");
  mNewtonSubsteps = findSFInt("newtonSubsteps");
  mNewtonCone = findSFString("newtonCone");
  mNewtonImpratio = findSFDouble("newtonImpratio");
  mNewtonCondim = findSFInt("newtonCondim");
  mNewtonNoslipIterations = findSFInt("newtonNoslipIterations");
  mNewtonClothSelfContact = findSFInt("newtonClothSelfContact");
  mNewtonGroundMu = findSFDouble("newtonGroundMu");
  mNewtonContactKe = findSFDouble("newtonContactKe");
  mNewtonContactKd = findSFDouble("newtonContactKd");
  mNewtonIterations = findSFInt("newtonIterations");
  mNewtonLsIterations = findSFInt("newtonLsIterations");
  mNewtonNjmax = findSFInt("newtonNjmax");
  mNewtonNconmax = findSFInt("newtonNconmax");
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
  OmField *northDirectionField = findField("northDirection");
  const OmSFVector3 *const northDirection = findSFVector3("northDirection");
  if (version && *version < OmVersion(2020, 1, 0, true)) {
    mGravity->setValue(OmParser::legacyGravity());
    mCoordinateSystem->setValue("NUE");  // default value for Webots < R2020b
    if (northDirection->value() == OmVector3(1.0, 0.0, 0.0))
      northDirectionField->reset();
    else if (northDirection->value() == OmVector3(0.0, 0.0, 1.0)) {
      northDirectionField->reset();
      mCoordinateSystem->setValue("EUN");
    } else if (!northDirectionField->isDefault())
      parsingWarn(tr("The 'northDirection' field is deprecated, according to the 'coordinateSystem' field, the north is "
                     "aligned along the x-axis."));
  } else if (northDirection->value() == OmVector3(0.0, 0.0, 1.0) && mCoordinateSystem->value() == "NUE") {
    northDirectionField->reset();
    mCoordinateSystem->setValue("EUN");
  } else if (!northDirectionField->isDefault())
    parsingWarn(tr("The 'northDirection' field is deprecated, please use the 'coordinateSystem' field instead."));

  OmProtoTemplateEngine::setCoordinateSystem(mCoordinateSystem->value());
  mGpsCoordinateSystem = findSFString("gpsCoordinateSystem");
  mGpsReference = findSFVector3("gpsReference");
  mLineScale = findSFDouble("lineScale");
  mDragForceScale = findSFDouble("dragForceScale");
  mDragTorqueScale = findSFDouble("dragTorqueScale");
  mRandomSeed = findSFInt("randomSeed");
  mContactProperties = findMFNode("contactProperties");

  if (findSFString("fast2d")->value() != "")
    parsingWarn(tr("fast2d plugin are not supported anymore, if you don't want to simulate dynamic, you can use the built-in "
                   "kinematic mode of OmniSim."));

  // WorldInfo.physics named a user-compiled ODE physics plugin
  // (webots_physics_init/collide/step). The feature was REMOVED from OmniSim
  // with the ODE retirement; the field stays DECLARED in WorldInfo.wrl on
  // purpose -- an undeclared field is an ERROR that takes a headless run's
  // exit code to 1, so deleting it would make every legacy world read as a
  // crash. Parsed, warned about once, and ignored.
  const QString &physicsPluginName = mPhysics->value();
  if (!physicsPluginName.isEmpty() && physicsPluginName != "<none>")
    parsingWarn(tr("Physics plugins were removed from OmniSim: the WorldInfo.physics field is ignored (this world asks for "
                   "'%1'). Delete the field from WorldInfo to silence this warning.")
                  .arg(physicsPluginName));
}

OmWorldInfo::OmWorldInfo(OmTokenizer *tokenizer) : OmBaseNode("WorldInfo", tokenizer) {
  init(tokenizer ? &tokenizer->fileVersion() : &OmApplicationInfo::version());
}

OmWorldInfo::OmWorldInfo(const OmWorldInfo &other) : OmBaseNode(other) {
  init();
}

OmWorldInfo::OmWorldInfo(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmWorldInfo::~OmWorldInfo() {
}

void OmWorldInfo::downloadAssets() {
  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    OmContactProperties *const cp = static_cast<OmContactProperties *>(mContactProperties->item(i));
    cp->downloadAssets();
  }
}

void OmWorldInfo::preFinalize() {
  OmBaseNode::preFinalize();

  if (defaultDamping())
    defaultDamping()->preFinalize();

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
  OmProtoTemplateEngine::setCoordinateSystem(mCoordinateSystem->value());

  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    OmContactProperties *const cp = static_cast<OmContactProperties *>(mContactProperties->item(i));
    cp->preFinalize();
  }
}

void OmWorldInfo::postFinalize() {
  OmBaseNode::postFinalize();

  if (defaultDamping())
    defaultDamping()->postFinalize();

  connect(mTitle, &OmSFString::changed, this, &OmWorldInfo::titleChanged);
  connect(mGravity, &OmSFDouble::changed, this, &OmWorldInfo::updateGravity);
  connect(mCfm, &OmSFDouble::changed, this, &OmWorldInfo::updateCfm);
  connect(mErp, &OmSFDouble::changed, this, &OmWorldInfo::updateErp);
  connect(mBasicTimeStep, &OmSFDouble::changed, this, &OmWorldInfo::updateBasicTimeStep);
  connect(mOptimalThreadCount, &OmSFInt::changed, this, &OmWorldInfo::updateOptimalThreadCount);
  connect(mOptimalThreadCount, &OmSFInt::changed, this, &OmWorldInfo::displayOptimalThreadCountWarning);
  connect(mFps, &OmSFDouble::changed, this, &OmWorldInfo::updateFps);
  connect(mLineScale, &OmSFDouble::changed, this, &OmWorldInfo::updateLineScale);
  connect(mDragForceScale, &OmSFDouble::changed, this, &OmWorldInfo::updateDragForceScale);
  connect(mDragTorqueScale, &OmSFDouble::changed, this, &OmWorldInfo::updateDragTorqueScale);
  connect(mRandomSeed, &OmSFInt::changed, this, &OmWorldInfo::updateRandomSeed);
  connect(mPhysicsDisableTime, &OmSFDouble::changed, this, &OmWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableLinearThreshold, &OmSFDouble::changed, this, &OmWorldInfo::physicsDisableChanged);
  connect(mPhysicsDisableAngularThreshold, &OmSFDouble::changed, this, &OmWorldInfo::physicsDisableChanged);
  connect(mDefaultDamping, &OmSFNode::changed, this, &OmWorldInfo::updateDefaultDamping);
  connect(mCoordinateSystem, &OmSFString::changed, this, &OmWorldInfo::updateCoordinateSystem);
  connect(mCoordinateSystem, &OmSFString::changed, this, &OmWorldInfo::updateGravity);

  connect(mGpsCoordinateSystem, &OmSFString::changed, this, &OmWorldInfo::updateGpsCoordinateSystem);
  connect(mGpsReference, &OmSFString::changed, this, &OmWorldInfo::gpsReferenceChanged);

  const int size = mContactProperties->size();
  for (int i = 0; i < size; ++i) {
    OmContactProperties *const cp = static_cast<OmContactProperties *>(mContactProperties->item(i));
    cp->postFinalize();
    connect(cp, &OmContactProperties::valuesChanged, this, &OmWorldInfo::updateContactProperties);
  }

  connect(mContactProperties, &OmMFNode::changed, this, &OmWorldInfo::updateContactProperties);

  OmWorld::instance()->setWorldInfo(this);
}

void OmWorldInfo::reset(const QString &id) {
  OmBaseNode::reset(id);

  for (int i = 0; i < mContactProperties->size(); ++i)
    mContactProperties->item(i)->reset(id);
  OmNode *const d = mDefaultDamping->value();
  if (d)
    d->reset(id);
}

double OmWorldInfo::lineScale() const {
  return mLineScale->value();
}

int OmWorldInfo::contactPropertiesCount() const {
  return mContactProperties->size();
}

OmContactProperties *OmWorldInfo::contactProperties(int index) const {
  return static_cast<OmContactProperties *>(mContactProperties->item(index));
}

OmDamping *OmWorldInfo::defaultDamping() const {
  return static_cast<OmDamping *>(mDefaultDamping->value());
}

void OmWorldInfo::createWrenObjects() {
  OmBaseNode::createWrenObjects();
  OmWrenRenderingContext::instance()->setLineScale(static_cast<float>(lineScale()));
}

void OmWorldInfo::createOdeObjects() {
  OmBaseNode::createOdeObjects();

}

void OmWorldInfo::updateBasicTimeStep() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mBasicTimeStep, 32.0);
}

void OmWorldInfo::updateFps() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mFps, 60.0);
}

void OmWorldInfo::displayOptimalThreadCountWarning() {
  int threadPreferenceNumber = OmPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > 1 and threadPreferenceNumber > 1)
    parsingWarn(
      tr("Physics multi-threading is enabled. "
         "This can have a noticeable impact on the simulation speed (negative or positive depending on the simulated world). "
         "In case of multi-threading, simulation replicability is not guaranteed. "));
}

void OmWorldInfo::updateOptimalThreadCount() {
  // this function is called only on field update,
  // loading a world where the 'optimalThreadCount' field is higher than the limit set in the preferences will therefore not
  // raise any warning
  int threadPreferenceNumber = OmPreferences::instance()->value("General/numberOfThreads", 1).toInt();
  if (mOptimalThreadCount->value() > threadPreferenceNumber)
    parsingWarn(tr("A limit of '%1' threads is set in the preferences.").arg(threadPreferenceNumber));
  else if (!OmFieldChecker::resetIntIfNonPositive(this, mOptimalThreadCount, 1))
    emit optimalThreadCountChanged();
}

void OmWorldInfo::updateLineScale() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mLineScale, 0.0))
    return;

  if (areWrenObjectsInitialized())
    applyLineScaleToWren();
}

void OmWorldInfo::applyLineScaleToWren() {
  OmWrenRenderingContext::instance()->setLineScale(static_cast<float>(mLineScale->value()));
}

void OmWorldInfo::updateDragForceScale() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mDragForceScale, 30.0);
}

void OmWorldInfo::updateDragTorqueScale() {
  OmFieldChecker::resetDoubleIfNonPositive(this, mDragTorqueScale, 5.0);
}

void OmWorldInfo::updateRandomSeed() {
  OmFieldChecker::resetIntIfNegativeAndNotDisabled(this, mRandomSeed, 0, -1);
  emit randomSeedChanged();
}

void OmWorldInfo::updateGravity() {
  updateGravityBasis();
  if (areOdeObjectsCreated())
    emit globalPhysicsPropertiesChanged();
}

void OmWorldInfo::updateCfm() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mCfm, 0.00001))
    return;

  if (areOdeObjectsCreated())
    emit globalPhysicsPropertiesChanged();
}

void OmWorldInfo::updateErp() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mErp, 0.0, 1.0, 0.2))
    return;

  if (areOdeObjectsCreated())
    emit globalPhysicsPropertiesChanged();
}

void OmWorldInfo::updateDefaultDamping() {
  if (!areOdeObjectsCreated())
    return;
  // The per-step damping conversion that used to be pushed into ODE here is
  // gone; the Damping node is read by the Newton registration path directly.
  // Keep listening to the node's own field changes and wake sleeping bodies.
  const OmDamping *const damping = defaultDamping();
  if (damping)
    connect(damping, &OmDamping::changed, this, &OmWorldInfo::updateDefaultDamping, Qt::UniqueConnection);
  emit globalPhysicsPropertiesChanged();
}

// Computes an orthonormal basis whose 'yaw unit vector' is the opposite of the normalized gravity vector
void OmWorldInfo::updateGravityBasis() {
  const QString &system = mCoordinateSystem->value();
  assert(system.size() == 3);
  mNorthVector = OmVector3(system[0] == 'N' ? 1 : 0, system[1] == 'N' ? 1 : 0, system[2] == 'N' ? 1 : 0);
  mEastVector = OmVector3(system[0] == 'E' ? 1 : 0, system[1] == 'E' ? 1 : 0, system[2] == 'E' ? 1 : 0);
  mUpVector = OmVector3(system[0] == 'U' ? 1 : 0, system[1] == 'U' ? 1 : 0, system[2] == 'U' ? 1 : 0);
  mGravityUnitVector = -mUpVector;
  mGravityVector = mGravityUnitVector * mGravity->value();
}

void OmWorldInfo::updateCoordinateSystem() {
  warn(tr("Please save and revert the world so that the change of coordinate system is taken into account when reloading "
          "procedural PROTO nodes."));
}

void OmWorldInfo::updateGpsCoordinateSystem() {
  if (mGpsCoordinateSystem->value() != "local" && mGpsCoordinateSystem->value() != "WGS84") {
    mGpsCoordinateSystem->setValue("local");
    parsingWarn(tr("'gpsCoordinateSystem' must either be 'local' or 'WGS84'. Reset to default value 'local'."));
  }
  emit gpsCoordinateSystemChanged();
}

void OmWorldInfo::updateContactProperties() {
  if (areOdeObjectsCreated())
    emit globalPhysicsPropertiesChanged();
}
