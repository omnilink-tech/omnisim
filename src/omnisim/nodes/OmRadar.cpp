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

#include "OmRadar.hpp"

#include "OmAffinePlane.hpp"
#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmGroup.hpp"
#include "OmLog.hpp"
#include "OmObjectDetection.hpp"
#include "OmRandom.hpp"
#include "OmRobot.hpp"
#include "OmSensor.hpp"
#include "OmSolid.hpp"
#include "OmSolidUtilities.hpp"
#include "OmWorld.hpp"
#include "../physics/OmNewtonBackend.hpp"
#include "../physics/OmPhysicsBackend.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QVector>

#include <cassert>

// Newton raycast service gate (kernel blocker #1, ode-retirement-campaign.md).
// Same value-parsed read as OmDistanceSensor::refreshRaysFromNewton.
static bool isNewtonRaycastEnabled() {
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_RAYCAST")).trimmed().toLower();
    // DEFAULT ON since 2026-08-07: every ray consumer answers from the
    // Newton service (mj_ray on the live mjModel), incl. the LASER
    // transparency re-cast and the joint-descending exclusion walk.
    // =0 turns the service off; with ODE deleted there is no other ray
    // provider, so occlusion cannot be evaluated at all in that case.
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return sGate == 1;
}

// One-shot, loud-by-design: `occlusion TRUE` was asked for but no ray could be
// cast this tick. Silence here is exactly the defect class that made every
// target read as unoccluded after the ODE ray carrier was removed, so the
// device reports its PREVIOUS occlusion verdicts rather than pretending the
// line of sight is clear -- and says so once.
static void warnRadarOcclusionUnavailableOnce(const char *why) {
  static bool warned = false;
  if (warned)
    return;
  warned = true;
  OmLog::warning(QString::fromUtf8("Radar 'occlusion' is TRUE but the Newton raycast service could not answer (") +
                 QString::fromUtf8(why) +
                 QString::fromUtf8("). The previous occlusion verdicts are kept -- targets are NOT re-tested this tick, "
                                   "and none is silently promoted to \"visible\"."));
}

// Collects the Newton body indices of every Solid under `robot` (same walk as
// OmDistanceSensor builds its mNewtonExcludeBodies).
static void collectRobotNewtonBodies(const OmRobot *robot, QVector<int> &out) {
  // shared walk: descends joint/slot endPoints too (the original per-file
  // OmGroup-only walk missed Solids behind joints, so an articulated robot
  // could see its own arm where ODE's same-robot rule excluded it)
  OmSolidUtilities::collectNewtonBodies(const_cast<OmRobot *>(robot), out);
}

class OmRadarTarget : public OmObjectDetection {
public:
  OmRadarTarget(OmRadar *radar, OmSolid *solidTarget, const bool needToCheckCollision, const double maxRange) :
    OmObjectDetection(radar, solidTarget, needToCheckCollision ? OmObjectDetection::ONE_RAY : OmObjectDetection::NONE, maxRange,
                      radar->horizontalFieldOfView()) {
    mTargetDistance = 0.0;
    mReceivedPower = 0.0;
    mSpeed = 0.0;
    mAzimuth = 0.0;
  };

  virtual ~OmRadarTarget() {}

  double targetDistance() const { return mTargetDistance; }
  double receivedPower() const { return mReceivedPower; }
  double speed() const { return mSpeed; }
  double azimuth() const { return mAzimuth; }
  void setTargetDistance(double distance) { mTargetDistance = distance; }
  void setReceivedPower(double receivedPower) { mReceivedPower = receivedPower; }
  void setSpeed(double speed) { mSpeed = speed; }
  void setAzimuth(double azimuth) { mAzimuth = azimuth; }

protected:
  double distance() override { return objectRelativePosition().length(); }

  double mTargetDistance;
  double mReceivedPower;
  double mSpeed;
  double mAzimuth;
};

void OmRadar::init() {
  mMinRange = findSFDouble("minRange");
  mMaxRange = findSFDouble("maxRange");
  mHorizontalFieldOfView = findSFDouble("horizontalFieldOfView");
  mVerticalFieldOfView = findSFDouble("verticalFieldOfView");
  mMinAbsoluteRadialSpeed = findSFDouble("minAbsoluteRadialSpeed");
  mMinRadialSpeed = findSFDouble("minRadialSpeed");
  mMaxRadialSpeed = findSFDouble("maxRadialSpeed");
  mCellDistance = findSFDouble("cellDistance");
  mCellSpeed = findSFDouble("cellSpeed");
  mRangeNoise = findSFDouble("rangeNoise");
  mSpeedNoise = findSFDouble("speedNoise");
  mAngularNoise = findSFDouble("angularNoise");
  mAntennaGain = findSFDouble("antennaGain");
  mFrequency = findSFDouble("frequency");
  mTransmittedPower = findSFDouble("transmittedPower");
  mOcclusion = findSFBool("occlusion");
  mMinDetectableSignal = findSFDouble("minDetectableSignal");

  mSensorElapsedTime = 0;
  mSensor = NULL;

  mTransmittedPowerInW = 0.0;
  mRealGain = 0.0;
  mReceivedPowerThreshold = 0.0;
  mReceivedPowerFactor = 1.0;
}

OmRadar::OmRadar(OmTokenizer *tokenizer) : OmSolidDevice("Radar", tokenizer) {
  init();
}

OmRadar::OmRadar(const OmRadar &other) : OmSolidDevice(other) {
  init();
}

OmRadar::OmRadar(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmRadar::~OmRadar() {
  delete mSensor;
  qDeleteAll(mRadarTargets);
  mRadarTargets.clear();
}

void OmRadar::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();
}

void OmRadar::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mMinRange, &OmSFDouble::changed, this, &OmRadar::updateMinRange);
  connect(mMaxRange, &OmSFDouble::changed, this, &OmRadar::updateMaxRange);
  connect(mHorizontalFieldOfView, &OmSFDouble::changed, this, &OmRadar::updateHorizontalFieldOfView);
  connect(mVerticalFieldOfView, &OmSFDouble::changed, this, &OmRadar::updateVerticalFieldOfView);
  connect(mMinAbsoluteRadialSpeed, &OmSFDouble::changed, this, &OmRadar::updateMinAbsoluteRadialSpeed);
  connect(mMinRadialSpeed, &OmSFDouble::changed, this, &OmRadar::updateMinAndMaxRadialSpeed);
  connect(mMaxRadialSpeed, &OmSFDouble::changed, this, &OmRadar::updateMinAndMaxRadialSpeed);
  connect(mCellDistance, &OmSFDouble::changed, this, &OmRadar::updateCellDistance);
  connect(mCellSpeed, &OmSFDouble::changed, this, &OmRadar::updateCellSpeed);
  connect(mRangeNoise, &OmSFDouble::changed, this, &OmRadar::updateRangeNoise);
  connect(mSpeedNoise, &OmSFDouble::changed, this, &OmRadar::updateSpeedNoise);
  connect(mAngularNoise, &OmSFDouble::changed, this, &OmRadar::updateAngularNoise);
  connect(mFrequency, &OmSFDouble::changed, this, &OmRadar::updateFrequency);
  connect(mAntennaGain, &OmSFDouble::changed, this, &OmRadar::updateAntennaGain);
  connect(mTransmittedPower, &OmSFDouble::changed, this, &OmRadar::updateTransmittedPower);
  connect(mMinDetectableSignal, &OmSFDouble::changed, this, &OmRadar::updateMinDetectableSignal);

  updateMinRange();
  updateMaxRange();
  updateHorizontalFieldOfView();
  updateVerticalFieldOfView();
  updateMinAbsoluteRadialSpeed();
  updateMinAndMaxRadialSpeed();
  updateCellDistance();
  updateCellSpeed();
  updateRangeNoise();
  updateSpeedNoise();
  updateAngularNoise();
  updateFrequency();
  updateAntennaGain();
  updateTransmittedPower();
  updateMinDetectableSignal();
}

void OmRadar::updateMinRange() {
  OmFieldChecker::resetDoubleIfNegative(this, mMinRange, 0.0);
  if (mMaxRange->value() <= mMinRange->value()) {
    if (mMaxRange->value() == 0.0) {
      double newMaxRange = mMinRange->value() + 1.0;
      parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
      mMaxRange->setValueNoSignal(newMaxRange);
    } else {
      double newMinRange = mMaxRange->value() - 1.0;
      if (newMinRange < 0.0)
        newMinRange = 0.0;
      parsingWarn(tr("'minRange' is greater or equal to 'maxRange'. Setting 'minRange' to %1.").arg(newMinRange));
      mMinRange->setValueNoSignal(newMinRange);
    }
    return;
  }
}

void OmRadar::updateMaxRange() {
  OmFieldChecker::resetDoubleIfNegative(this, mMaxRange, mMinRange->value() + 1.0);

  if (mMaxRange->value() <= mMinRange->value()) {
    double newMaxRange = mMinRange->value() + 1.0;
    parsingWarn(tr("'maxRange' is less or equal to 'minRange'. Setting 'maxRange' to %1.").arg(newMaxRange));
    mMaxRange->setValueNoSignal(newMaxRange);
    return;
  }
}

void OmRadar::updateHorizontalFieldOfView() {
  OmFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(this, mHorizontalFieldOfView, 0.0, M_PI, 0.78);
}

void OmRadar::updateVerticalFieldOfView() {
  OmFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(this, mVerticalFieldOfView, 0.0, M_PI_2, 0.1);
}

void OmRadar::updateMinAbsoluteRadialSpeed() {
  OmFieldChecker::resetDoubleIfNegative(this, mMinAbsoluteRadialSpeed, 0.0);
}

void OmRadar::updateMinAndMaxRadialSpeed() {
  if (mMinRadialSpeed->value() == 0.0 && mMaxRadialSpeed->value() == 0.0)
    // no limits
    return;

  if (mMaxRadialSpeed->value() <= mMinRadialSpeed->value()) {
    double newMaxRadialSpeed = mMinRadialSpeed->value() + 1.0;
    parsingWarn(
      tr("'maxRadialSpeed' is less than or equal to 'minRadialSpeed'. Setting 'maxRadialSpeed' to %1.").arg(newMaxRadialSpeed));
    mMaxRadialSpeed->setValueNoSignal(newMaxRadialSpeed);
    return;
  }
}

void OmRadar::updateCellDistance() {
  OmFieldChecker::resetDoubleIfNegative(this, mCellDistance, 0.0);
}

void OmRadar::updateCellSpeed() {
  OmFieldChecker::resetDoubleIfNegative(this, mCellSpeed, 0.0);
}

void OmRadar::updateRangeNoise() {
  OmFieldChecker::resetDoubleIfNegative(this, mRangeNoise, 0.0);
}

void OmRadar::updateSpeedNoise() {
  OmFieldChecker::resetDoubleIfNegative(this, mSpeedNoise, 0.0);
}

void OmRadar::updateAngularNoise() {
  OmFieldChecker::resetDoubleIfNegative(this, mAngularNoise, 0.0);
}

void OmRadar::updateFrequency() {
  OmFieldChecker::resetDoubleIfNegative(this, mFrequency, 24.0);
  updateReceivedPowerFactor();
}

void OmRadar::updateTransmittedPower() {
  // convert transmittedPower from dBm to W
  mTransmittedPowerInW = 0.001 * pow(10.0, mTransmittedPower->value() / 10.0);
  updateReceivedPowerFactor();
}

void OmRadar::updateAntennaGain() {
  // convert antennaGain from dBi to to real gain
  mRealGain = pow(10.0, mAntennaGain->value() / 10.0);
  updateReceivedPowerFactor();
}

void OmRadar::updateMinDetectableSignal() {
  // convert minDetectableSignal from dBm into W
  mReceivedPowerThreshold = 0.001 * pow(10.0, mMinDetectableSignal->value() / 10.0);
}

void OmRadar::updateReceivedPowerFactor() {
  // compute the received power factor
  //(part of the received power formula only dependent on the radar caracteristics)
  mReceivedPowerFactor = mTransmittedPowerInW * pow(mRealGain, 2.0) * pow(wavelength(), 2.0) / pow(4 * M_PI, 3.0);
}

void OmRadar::prePhysicsStep(double ms) {
  OmSolidDevice::prePhysicsStep(ms);

  qDeleteAll(mRadarTargets);
  mRadarTargets.clear();

  if (isPowerOn() && mSensor->isEnabled() && mSensor->needToRefreshInMs(ms) && mOcclusion->value()) {
    // Create the candidate targets (and their occlusion rays). They are aimed
    // and filtered later, in refreshSensorIfNeeded(): ODE used to re-aim them
    // mid-step through OmSolidDevice's static dirty-sensor list (since removed: nothing drained it) and collide them in the
    // same pass, but the Newton raycast service answers AFTER the step, so there
    // is nothing to subscribe to any more.
    computeTargets(false, true);
  }
}

void OmRadar::postPhysicsStep() {
  OmSolidDevice::postPhysicsStep();

  // delete the OmRadarTargets that dropped out of the frustum
  // it is preferable to not delete them during the physics step to avoid
  // possible inconsistencies in other clusters
  qDeleteAll(mInvalidRadarTargets);
  mInvalidRadarTargets.clear();
}

// Aims every candidate target's occlusion ray at the target's CURRENT pose and
// re-runs the frustum + target-property pass, dropping the candidates that no
// longer qualify. ODE called this from the middle of the physics step (through
// OmSolidDevice's static dirty-sensor list (since removed: nothing drained it)) because its collision pass ran there;
// refreshSensorIfNeeded() calls it directly now, immediately before the Newton
// raycast, so the rays that get cast match the poses the raycast service sees.
void OmRadar::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();

  // compute the radar position, rotation, axis and plane
  const OmVector3 radarPosition = position();
  const OmMatrix3 radarRotation = rotationMatrix();
  const OmVector3 radarAxis = radarRotation * OmVector3(1.0, 0.0, 0.0);
  const OmAffinePlane radarPlane(radarRotation * OmVector3(0.0, 0.0, 1.0), radarAxis);
  const OmAffinePlane *frustumPlanes =
    OmObjectDetection::computeFrustumPlanes(this, verticalFieldOfView(), horizontalFieldOfView(), maxRange(), true);
  foreach (OmRadarTarget *target, mRadarTargets) {
    target->object()->updateTransformForPhysicsStep();
    if (!target->recomputeRayDirection(frustumPlanes) ||
        !setTargetProperties(radarPosition, radarRotation, radarAxis, radarPlane, target)) {
      mRadarTargets.removeAll(target);
      mInvalidRadarTargets.append(target);
    }
  }

  delete[] frustumPlanes;
}

void OmRadar::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;
    default:
      assert(0);
  }
}

void OmRadar::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());

  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)minRange();
  stream << (double)maxRange();
  stream << (double)horizontalFieldOfView();
  stream << (double)verticalFieldOfView();
}

void OmRadar::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_RADAR_DATA;
    int numberOfTargets = mRadarTargets.size();
    stream << (int)numberOfTargets;
    for (int i = 0; i < numberOfTargets; ++i) {
      stream << (double)mRadarTargets.at(i)->targetDistance();
      stream << (double)mRadarTargets.at(i)->receivedPower();
      stream << (double)mRadarTargets.at(i)->speed();
      stream << (double)mRadarTargets.at(i)->azimuth();
    }
    mSensor->resetPendingValue();
  }
}

void OmRadar::computeTargets(bool finalSetup, bool needCollisionDetection) {
  // compute the radar position, rotation, axis and plane
  const OmVector3 radarPosition = position();
  const OmMatrix3 radarRotation = rotationMatrix();
  const OmVector3 radarAxis = radarRotation * OmVector3(1.0, 0.0, 0.0);
  const OmAffinePlane radarPlane(radarRotation * OmVector3(0.0, 0.0, 1.0), radarAxis);
  const OmAffinePlane *frustumPlanes =
    OmObjectDetection::computeFrustumPlanes(this, verticalFieldOfView(), horizontalFieldOfView(), maxRange(), true);

  // loop for each possible target to check if it is visible
  QList<OmSolid *> targets = OmWorld::instance()->radarTargetSolids();
  for (int i = 0; i < targets.size(); i++) {
    OmSolid *target = targets.at(i);
    if (target == this)
      continue;
    // create target
    OmRadarTarget *generatedTarget = new OmRadarTarget(this, target, needCollisionDetection, maxRange());
    if (finalSetup) {
      if (!generatedTarget->isContainedInFrustum(frustumPlanes) ||
          !setTargetProperties(radarPosition, radarRotation, radarAxis, radarPlane, generatedTarget)) {
        delete generatedTarget;
        continue;
      }
    }
    mRadarTargets.append(generatedTarget);
  }

  delete[] frustumPlanes;
}

bool OmRadar::setTargetProperties(const OmVector3 &radarPosition, const OmMatrix3 &radarRotation, const OmVector3 &radarAxis,
                                  const OmAffinePlane &radarPlane, OmRadarTarget *radarTarget) {
  assert(radarTarget);

  const OmVector3 targetPosition = radarTarget->object()->position();
  const OmVector3 targetToRadarVector = targetPosition - radarPosition;

  double distance = radarTarget->objectRelativePosition().length() + mRangeNoise->value() * OmRandom::nextGaussian();

  // check that target is not too close
  if (distance < (minRange() - radarTarget->objectSize().x() / 2.0))
    return false;

  if (distance > maxRange())
    distance = maxRange();
  else if (distance < minRange())
    distance = minRange();

  // compute received power and check if it is above the threshold
  // the received power is converted to dBm after the targets are merged
  // this is done to avoid converting back and forth when merging two targets
  double receivedPower = mReceivedPowerFactor * radarTarget->object()->radarCrossSection() / pow(distance, 4.0);
  if (receivedPower < mReceivedPowerThreshold)
    return false;

  // compute speed using (distance / time).
  OmVector3 targetVelocity =
    (targetPosition - mRadarTargetsPreviousTranslations[radarTarget->object()]) * (1000 / mSensorElapsedTime);
  OmVector3 radarVelocity = (radarPosition - mPreviousRadarPosition) * (1000 / mSensorElapsedTime);
  OmVector3 relativeVelocity = targetVelocity - radarVelocity;

  double relativeSpeed = targetToRadarVector.normalized().dot(relativeVelocity);
  relativeSpeed += mSpeedNoise->value() * OmRandom::nextGaussian();
  // check speed is in the radial speed range
  if (mMinRadialSpeed->value() != 0.0 || mMaxRadialSpeed->value() != 0.0) {
    if (relativeSpeed < mMinRadialSpeed->value())
      return false;
    else if (relativeSpeed > mMaxRadialSpeed->value())
      return false;
  }

  // check that absolute speed is bigger than 'mMinAbsoluteRadialSpeed'
  if (mMinAbsoluteRadialSpeed->value() > 0 && fabs(relativeSpeed) < mMinAbsoluteRadialSpeed->value())
    return false;

  // compute horizontal angle
  OmVector3 projectedTargetToRadarVector = radarPlane.vectorProjection(targetToRadarVector);
  projectedTargetToRadarVector.normalize();
  double azimuth = radarAxis.angle(projectedTargetToRadarVector);
  if (projectedTargetToRadarVector.dot(radarRotation * OmVector3(0.0, 1.0, 0.0)) > 0.0)
    azimuth = -azimuth;

  // checks that azimuth is not out of the detection frustum,
  // this can happen if the object's center is outside but a part of the object is inside.
  // In that case a future improvement would be to adapt the received power.
  if (azimuth > (mHorizontalFieldOfView->value() / 2.0))
    azimuth = mHorizontalFieldOfView->value() / 2.0;
  else if (azimuth < -(mHorizontalFieldOfView->value() / 2.0))
    azimuth = -mHorizontalFieldOfView->value() / 2.0;

  azimuth += mAngularNoise->value() * OmRandom::nextGaussian();

  // update target
  radarTarget->setTargetDistance(distance);
  radarTarget->setReceivedPower(receivedPower);
  radarTarget->setSpeed(relativeSpeed);
  radarTarget->setAzimuth(azimuth);
  return true;
}

bool OmRadar::refreshTargetOcclusionFromNewton() {
  // Kernel blocker #1 (ode-retirement-campaign.md): answer every target's
  // occlusion ray from the Newton raycast service (mj_ray over the live
  // mjModel). This is the ONLY ray provider left, so a gate-off or an
  // unavailable runtime means the occlusion question is UNANSWERED -- returning
  // false makes the caller keep its previous verdicts instead of treating
  // "no answer" as "clear line of sight".
  if (mRadarTargets.isEmpty())
    return true;  // nothing to test
  if (!isNewtonRaycastEnabled()) {
    warnRadarOcclusionUnavailableOnce("OMNISIM_NEWTON_RAYCAST is disabled");
    return false;
  }
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable()) {
    warnRadarOcclusionUnavailableOnce("the Newton runtime is not available");
    return false;
  }
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);

  // Own-robot exclusion, replicating odeNearCallback's same-robot rule
  // (waived by selfCollision); cached once like OmDistanceSensor.
  if (mNewtonExcludeBodies.isEmpty()) {
    const OmRobot *const r = robot();
    if (r != nullptr && !r->selfCollision())
      collectRobotNewtonBodies(r, mNewtonExcludeBodies);
    if (mNewtonExcludeBodies.isEmpty())
      mNewtonExcludeBodies.append(-999);  // sentinel: "built, nothing to exclude"
  }
  const QVector<int> deviceExclude = (mNewtonExcludeBodies.size() == 1 && mNewtonExcludeBodies[0] == -999)
                                       ? QVector<int>()
                                       : mNewtonExcludeBodies;
  // one raycastBatch per target: the exclude list carries the target's own
  // bodies, so targets cannot share a batch
  bool answered = true;
  foreach (OmRadarTarget *target, mRadarTargets) {
    if (!target->refreshCollisionDepthsFromNewton(newton, deviceExclude))
      answered = false;
  }
  if (!answered)
    warnRadarOcclusionUnavailableOnce("the raycast service returned no answer");
  return answered;
}

bool OmRadar::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  if (!mOcclusion->value())
    // no ray casting needed: targets can be created here, at the end of the
    // step, when all the body positions are up-to-date
    computeTargets(true, false);
  else {
    // The candidates were created in prePhysicsStep(); aim their rays at the
    // post-step poses and compute their properties (frustum, range, power,
    // speed, azimuth) -- ODE did this mid-step, we do it here.
    updateRaysSetupIfNeeded();
    // Then cast the rays through the Newton service and drop what it says is
    // blocked. If the service could not answer, the target list is reported
    // unfiltered rather than being silently declared fully visible: the
    // one-shot warning above names the reason.
    if (refreshTargetOcclusionFromNewton())
      removeOccludedTargets();
  }

  if (mCellDistance->value() > 0.0)
    mergeTargets();

  // convert the received power into dBm
  for (int i = 0; i < mRadarTargets.size(); ++i)
    mRadarTargets.at(i)->setReceivedPower(10.0 * log10(mRadarTargets.at(i)->receivedPower() / 0.001));

  mRadarTargetsPreviousTranslations.clear();
  QList<OmSolid *> targets = OmWorld::instance()->radarTargetSolids();
  for (int i = 0; i < targets.size(); ++i) {
    // cppcheck-suppress constVariablePointer
    OmSolid *target = targets.at(i);
    if (target != this)
      mRadarTargetsPreviousTranslations.insert(target, target->position());
  }

  // cache the sensor timestep for use in velocity calculations
  mPreviousRadarPosition = position();
  mSensorElapsedTime = mSensor->elapsedTime();
  mSensor->updateTimer();
  return true;
}

void OmRadar::reset(const QString &id) {
  OmSolidDevice::reset(id);

  qDeleteAll(mRadarTargets);
  mRadarTargets.clear();
  qDeleteAll(mInvalidRadarTargets);
  mInvalidRadarTargets.clear();
  mRadarTargetsPreviousTranslations.clear();
}

void OmRadar::removeOccludedTargets() {
  for (int i = mRadarTargets.size() - 1; i >= 0; --i) {
    if (mRadarTargets.at(i)->hasCollided()) {
      delete mRadarTargets[i];
      mRadarTargets.removeAt(i);
    }
  }
}

void OmRadar::mergeTargets(int startingIndex) {
  // Merge the targets recursively if the distance is < 'cellDistance'
  int numberOfTargets = mRadarTargets.size();
  if (startingIndex >= numberOfTargets)
    return;
  double cellDistance = mCellDistance->value();
  double cellSpeed = mCellSpeed->value();
  bool targetMerged = false;
  int i = 0;
  for (i = startingIndex; i < numberOfTargets - 1; ++i) {
    QVector<int> targetsToMerge;
    for (int j = i + 1; j < numberOfTargets; ++j) {
      if ((fabs(mRadarTargets.at(i)->targetDistance() - mRadarTargets.at(j)->targetDistance()) < cellDistance) &&
          ((cellSpeed <= 0.0) || (fabs(mRadarTargets.at(i)->speed() - mRadarTargets.at(j)->speed()) < cellSpeed))) {
        targetsToMerge.append(j);
        targetMerged = true;
      }
    }
    if (targetMerged) {
      // use a weighted average (weight is the received power) to compute resulting target
      double resultingReceivedPower = mRadarTargets.at(i)->receivedPower();
      double resultingAzimuth = mRadarTargets.at(i)->azimuth() * resultingReceivedPower;
      double resultingSpeed = mRadarTargets.at(i)->speed() * resultingReceivedPower;
      double resultingDistance = mRadarTargets.at(i)->targetDistance() * resultingReceivedPower;
      for (int j = 0; j < targetsToMerge.size(); ++j) {
        resultingReceivedPower += mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
        resultingAzimuth +=
          mRadarTargets.at(targetsToMerge.at(j))->azimuth() * mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
        resultingSpeed +=
          mRadarTargets.at(targetsToMerge.at(j))->speed() * mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
        resultingDistance +=
          mRadarTargets.at(targetsToMerge.at(j))->targetDistance() * mRadarTargets.at(targetsToMerge.at(j))->receivedPower();
      }
      resultingAzimuth /= resultingReceivedPower;
      resultingSpeed /= resultingReceivedPower;
      resultingDistance /= resultingReceivedPower;
      mRadarTargets[i]->setReceivedPower(resultingReceivedPower);
      mRadarTargets[i]->setAzimuth(resultingAzimuth);
      mRadarTargets[i]->setSpeed(resultingSpeed);
      mRadarTargets[i]->setTargetDistance(resultingDistance);
      // remove the merged target
      for (int j = targetsToMerge.size() - 1; j >= 0; --j) {
        delete mRadarTargets[targetsToMerge.at(j)];
        mRadarTargets.removeAt(targetsToMerge.at(j));
      }
      break;
    }
  }
  if (targetMerged)
    mergeTargets(i);
}

