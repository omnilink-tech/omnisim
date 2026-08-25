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

#include "OmLightSensor.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmGroup.hpp"
#include "OmLight.hpp"
#include "OmLookupTable.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmOdeContext.hpp"
#include "OmRobot.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSensor.hpp"
#include "OmSolid.hpp"
#include "OmSolidUtilities.hpp"
#include "../physics/OmNewtonBackend.hpp"
#include "../physics/OmPhysicsBackend.hpp"

#include "../../controller/c/messages.h"

#include <limits>

#include <QtCore/QDataStream>

// TODO: remove these dependencies :
// implementing generic functionalities on OmLight would be more modular
#include "OmDirectionalLight.hpp"
#include "OmPointLight.hpp"
#include "OmSpotLight.hpp"

// Newton raycast service gate (kernel blocker #1, ode-retirement-campaign.md).
// Same value-parsed read as OmDistanceSensor::refreshRaysFromNewton: DEFAULT
// OFF until the parity suite covers every ray consumer; =1 opts a run in,
// =0/unset keeps the ODE ray path.
static bool isNewtonRaycastEnabled() {
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_RAYCAST")).trimmed().toLower();
    // DEFAULT ON since 2026-08-07: every ray consumer answers from the
    // Newton service (mj_ray on the live mjModel), incl. the LASER
    // transparency re-cast and the joint-descending exclusion walk.
    // =0 reverts to the ODE ray verdicts while src/ode still ships.
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return sGate == 1;
}

// Collects the Newton body indices of every Solid under `robot` (same walk as
// OmDistanceSensor builds its mNewtonExcludeBodies).
static void collectRobotNewtonBodies(const OmRobot *robot, QVector<int> &out) {
  // shared walk: descends joint/slot endPoints too (the original per-file
  // OmGroup-only walk missed Solids behind joints, so an articulated robot
  // could see its own arm where ODE's same-robot rule excluded it)
  OmSolidUtilities::collectNewtonBodies(const_cast<OmRobot *>(robot), out);
}

class LightRay {
public:
  LightRay(OmLightSensor *sensor, const OmVector3 &lightDirection, double distance, const OmLight *light, double direct,
           double attenuation, dSpaceID spaceId) :
    mLight(light),
    mSensor(sensor),
    mDirect(direct),
    mAttenuation(attenuation),
    mCollided(false) {
    // ODE is gone: no ray geom exists; carry the segment in plain
    // members so the Newton raycast service (the live occlusion answer in
    // this build) still gets the exact sensor->light ray.
    (void)spaceId;
    mGeom = NULL;
    mStart = sensor->matrix().translation();
    mSegmentDir = lightDirection;
    mSegmentLength = distance;
  };

  ~LightRay() {
  }

  bool hasCollided() const { return mCollided; }
  void setCollided() { mCollided = true; }
  // The Newton raycast service owns the answer under OMNISIM_NEWTON_RAYCAST:
  // it may both set and CLEAR the flag (the ODE collision pass already ran).
  void setNewtonVerdict(bool collided) { mCollided = collided; }
  const OmLight *light() const { return mLight; }
  double directIntensity() const { return mDirect; }
  double attenuation() const { return mAttenuation; }
  dGeomID geom() const { return mGeom; }
  // Native segment accessors (the ODE ray geom used to be the carrier)
  const OmVector3 &segmentStart() const { return mStart; }
  const OmVector3 &segmentDirection() const { return mSegmentDir; }
  double segmentLength() const { return mSegmentLength; }

  bool recomputeRayDirection() {
    const OmVector3 &sensorPos = mSensor->matrix().translation();
    const OmVector3 &sensorDirection = mSensor->matrix().xAxis();

    OmVector3 L;
    double distance = 0.0;
    mSensor->computeLightMeasurement(mLight, sensorDirection, sensorPos, L, distance, mDirect, mAttenuation);
    if (mDirect <= 0.0)
      return false;

    // recompute ray length and direction
    mStart = sensorPos;
    mSegmentDir = L;
    mSegmentLength = distance;
    return true;
  }

private:
  const OmLight *mLight;
  const OmLightSensor *mSensor;
  double mDirect;
  double mAttenuation;
  bool mCollided;  // the geom has collided yet
  dGeomID mGeom;   // geom that checks collision of this packet
  OmVector3 mStart;       // native segment carrier (see ctor)
  OmVector3 mSegmentDir;
  double mSegmentLength;
};

void OmLightSensor::init() {
  mValue = 0.0;
  mLut = NULL;
  mSensor = NULL;
  mRgbIntensity = new OmRgb();
  mSensorUpdateRequested = false;

  mLookupTable = findMFVector3("lookupTable");
  mColorFilter = findSFColor("colorFilter");
  mOcclusion = findSFBool("occlusion");
  mResolution = findSFDouble("resolution");

  mNeedToReconfigure = false;
}

OmLightSensor::OmLightSensor(OmTokenizer *tokenizer) : OmSolidDevice("LightSensor", tokenizer) {
  init();
}

OmLightSensor::OmLightSensor(const OmLightSensor &other) : OmSolidDevice(other) {
  init();
}

OmLightSensor::OmLightSensor(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmLightSensor::~OmLightSensor() {
  delete mLut;
  delete mSensor;
  delete mRgbIntensity;

  qDeleteAll(mRayList);
}

void OmLightSensor::preFinalize() {
  OmSolidDevice::preFinalize();
  mSensor = new OmSensor();
  updateLookupTable();
}

void OmLightSensor::postFinalize() {
  OmSolidDevice::postFinalize();
  connect(mLookupTable, &OmMFVector3::changed, this, &OmLightSensor::updateLookupTable);
  connect(mResolution, &OmSFDouble::changed, this, &OmLightSensor::updateResolution);
}

void OmLightSensor::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      break;
    default:
      assert(0);
  }
}

void OmLightSensor::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_LIGHT_SENSOR_DATA;
    stream << mValue;

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmLightSensor::addConfigure(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mLookupTable->size();
  for (int i = 0; i < mLookupTable->size(); i++) {
    stream << (double)mLookupTable->item(i).x();
    stream << (double)mLookupTable->item(i).y();
    stream << (double)mLookupTable->item(i).z();
  }
  mNeedToReconfigure = false;
}

void OmLightSensor::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void OmLightSensor::updateLookupTable() {
  // rebuild the lookup table
  delete mLut;
  mLut = new OmLookupTable(*mLookupTable);
  mValue = mLut->minValue();

  mNeedToReconfigure = true;
}

void OmLightSensor::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmLightSensor::prePhysicsStep(double ms) {
  OmSolidDevice::prePhysicsStep(ms);

  mSensorUpdateRequested = isPowerOn() && mSensor->needToRefreshInMs(ms);
  if (mSensorUpdateRequested) {
    // in case postProcessLightMeasurement() was not called since the
    // previous prePhysicsStep() we need to clear the list at this point
    qDeleteAll(mRayList);
    mRayList.clear();

    // reset before accumulation
    mRgbIntensity->setValue(0.0, 0.0, 0.0);

    if (mOcclusion->isTrue()) {
      // add ambient light contribution and create rays for occlusion testing.
      // The rays are cast by the Newton raycast service after the step, so there
      // is no mid-step re-aim to subscribe to any more.
      setupRaysAndComputeDirectContributions(false);
    }
  }
}

void OmLightSensor::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();
  foreach (LightRay *ray, mRayList)
    ray->recomputeRayDirection();
}

void OmLightSensor::setupRaysAndComputeDirectContributions(bool finalSetup) {
  const OmVector3 &sensorAxis = matrix().xAxis();       // sensor normal vector (x-axis)
  const OmVector3 &sensorPos = matrix().translation();  // sensor position (world coordinate)

  QList<const OmLight *> lights = OmLight::lights();
  foreach (const OmLight *light, lights) {
    if (light->isOn()) {
      OmVector3 lightDirection;
      double distance, attenuation, direct;
      computeLightMeasurement(light, sensorAxis, sensorPos, lightDirection, distance, direct, attenuation);
      if (areOdeObjectsCreated() && mOcclusion->isTrue()) {
        if (finalSetup && direct <= 0.0)
          continue;

        // we need collision detection if the light source is in front of the sensor plate
        // and if occlusion check is required by the user
        mRayList.append(
          new LightRay(this, lightDirection, distance, light, direct, attenuation, OmOdeContext::instance()->space()));
      } else
        // otherwise, this light's contribution can be added immediately
        addSingleLightContribution(light, direct, attenuation);
    }
  }
}

void OmLightSensor::computeLightMeasurement(const OmLight *light,
                                            const OmVector3 &sensorAxis,  // sensor normal vector (x-axis)
                                            const OmVector3 &sensorPos,   // (world coordinate)
                                            OmVector3 &lightDirection, double &distance, double &direct,
                                            double &attenuation) const {
  double spotFactor = 1.0;

  const OmPointLight *pointLight = dynamic_cast<const OmPointLight *>(light);
  const OmDirectionalLight *directionalLight = dynamic_cast<const OmDirectionalLight *>(light);
  const OmSpotLight *spotLight = dynamic_cast<const OmSpotLight *>(light);

  if (pointLight) {
    const OmVector3 &lightPos = pointLight->computeAbsoluteLocation();
    lightDirection = lightPos - sensorPos;
    distance = lightDirection.length();  // sensor->light distance
    lightDirection /= distance;          // normalize lightDirection
    attenuation = pointLight->computeAttenuation(distance);
  } else if (directionalLight) {
    lightDirection = -directionalLight->direction();
    distance = 1000.0;  // assume DirectionalLight is 1 kilometer away
    lightDirection.normalize();
    attenuation = 1.0;  // no attenuation for DirectionalLight
  } else if (spotLight) {
    const OmVector3 &lightPos = spotLight->computeAbsoluteLocation();

    lightDirection = lightPos - sensorPos;
    distance = lightDirection.length();  // sensor->light distance
    lightDirection /= distance;          // normalize lightDirection
    attenuation = spotLight->computeAttenuation(distance);

    // compute spot's beam effect
    const OmVector3 &dir = spotLight->direction();
    const OmPose *up = spotLight->upperPose();
    OmVector3 R = up ? -(up->rotation().toMatrix3() * dir) : -dir;
    R.normalize();
    double alpha = OmMathsUtilities::clampedAcos(lightDirection.dot(R));  // both lightDirection and R are normalized
    assert(!std::isnan(alpha));
    if (alpha > spotLight->cutOffAngle())
      spotFactor = 0.0;
    else if (alpha <= spotLight->beamWidth())
      spotFactor = 1.0;
    else
      spotFactor = (alpha - spotLight->cutOffAngle()) / (spotLight->beamWidth() - spotLight->cutOffAngle());
  } else {
    assert(0);
    distance = 0.0;
    direct = 0.0;
    attenuation = 0.0;
    return;  // make compiler happy in release mode
  }

  // compute direct light intensity according to rays incidence angle
  // dot(sensorAxis, lightDirection) == cos(phi) because both sensorAxis and
  // lightDirection have been normalized above
  direct = 0.0;
  double cosine = sensorAxis.dot(lightDirection);
  if (cosine > 0.0)
    // light is in front of sensor
    direct = light->intensity() * cosine * spotFactor;
}

bool OmLightSensor::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmLightSensor::computeValue() {
  if (mOcclusion->isTrue())
    // add direct light contribution after collision detection
    postProcessLightMeasurement();
  else
    // add direct and ambient light contribution
    setupRaysAndComputeDirectContributions(true);

  // apply color filter
  const OmRgb &rgbFilter = mColorFilter->value();
  double finalIntensity = rgbFilter.red() * mRgbIntensity->red() + rgbFilter.green() * mRgbIntensity->green() +
                          rgbFilter.blue() * mRgbIntensity->blue();
  finalIntensity /= 3.0;

  // apply lookup table
  mValue = mLut->lookup(finalIntensity);

  // apply resolution if needed
  if (mResolution->value() != -1.0)
    mValue = OmMathsUtilities::discretize(mValue, mResolution->value());
}

void OmLightSensor::addSingleLightContribution(const OmLight *light, double direct, double attenuation) {
  const OmRgb &color = light->color();
  double attenuatedLight = attenuation * (light->ambientIntensity() + direct);

  mRgbIntensity->setRed(mRgbIntensity->red() + attenuatedLight * color.red());
  mRgbIntensity->setGreen(mRgbIntensity->green() + attenuatedLight * color.green());
  mRgbIntensity->setBlue(mRgbIntensity->blue() + attenuatedLight * color.blue());
}

void OmLightSensor::refreshRayCollisionsFromNewton() {
  // Kernel blocker #1 (ode-retirement-campaign.md): answer the sensor->light
  // occlusion rays from the Newton raycast service (mj_ray over the live
  // mjModel) instead of the ODE collision pass. Same contract as
  // OmDistanceSensor::refreshRaysFromNewton: gate off (the default for now)
  // or service unavailable leaves the ODE verdicts intact.
  if (!isNewtonRaycastEnabled() || mRayList.isEmpty())
    return;
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
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
  const int excludeCount =
    (mNewtonExcludeBodies.size() == 1 && mNewtonExcludeBodies[0] == -999) ? 0 : mNewtonExcludeBodies.size();

  const int n = mRayList.size();
  QVector<double> rays;
  rays.reserve(n * 7);
  for (int i = 0; i < n; ++i) {
    // ODE is gone: the LightRay carries the segment itself
    const OmVector3 &start = mRayList[i]->segmentStart();
    const OmVector3 &dir = mRayList[i]->segmentDirection();
    rays << start.x() << start.y() << start.z() << dir.x() << dir.y() << dir.z() << mRayList[i]->segmentLength();
  }
  QVector<OmNewtonRayHit> hits(n);
  if (newton->raycastBatch(n, rays.constData(), hits.data(), excludeCount ? mNewtonExcludeBodies.constData() : nullptr,
                           excludeCount) != n)
    return;  // service unavailable this tick -- keep the ODE verdicts
  for (int i = 0; i < n; ++i)
    // ODE semantics: ANY hit along the sensor->light segment occludes
    mRayList[i]->setNewtonVerdict(hits[i].dist >= 0.0);
}

void OmLightSensor::postProcessLightMeasurement() {
  assert(mOcclusion->isTrue());

  // Newton raycast service: overrides the ODE occlusion verdicts under the
  // OMNISIM_NEWTON_RAYCAST gate; no-op otherwise
  refreshRayCollisionsFromNewton();

  foreach (LightRay *ray, mRayList) {
    // in case of occlusion the direct intensity is zero
    double direct = ray->hasCollided() ? 0.0 : ray->directIntensity();

    // add this light contribution of measured intensity
    addSingleLightContribution(ray->light(), direct, ray->attenuation());
    delete ray;
  }

  // clear list
  mRayList.clear();
}

void OmLightSensor::rayCollisionCallback(dGeomID geom) {
  foreach (LightRay *ray, mRayList) {
    if (ray->geom() == geom) {
      ray->setCollided();
      return;
    }
  }

  assert(0);  // should never be reached
}

