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

#include "OmDistanceSensor.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmGeometry.hpp"
#include "OmLookupTable.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmNodeUtilities.hpp"
#include "OmOdeContext.hpp"
#include "OmGroup.hpp"
#include "OmRobot.hpp"
#include "../physics/OmNewtonBackend.hpp"
#include "../physics/OmPhysicsBackend.hpp"
#include "OmRay.hpp"
#include "OmRgb.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSensor.hpp"
#include "OmSolidUtilities.hpp"
#include "OmShape.hpp"
#include "OmSimulationState.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

#include <limits>

// LASER transparency walk over a boundingObject (Group/Pose/Shape/Geometry --
// note OmNodeUtilities::findDescendantNodesOfType does NOT descend OmShape,
// so a bounding Shape's geometry needs this dedicated walk).
static bool boundingObjectHasTransparentGeometry(OmBaseNode *root) {
  QList<OmBaseNode *> queue;
  queue.append(root);
  while (!queue.isEmpty()) {
    OmBaseNode *const n = queue.takeLast();
    if (n == nullptr)
      continue;
    if (const OmGeometry *const g = dynamic_cast<const OmGeometry *>(n)) {
      if (g->isTransparent())
        return true;
      continue;
    }
    if (const OmShape *const sh = dynamic_cast<const OmShape *>(n)) {
      queue.append(sh->geometry());
      continue;
    }
    if (const OmGroup *const grp = dynamic_cast<const OmGroup *>(n)) {  // covers OmPose
      const OmMFNode &kids = grp->children();
      for (int i = 0; i < kids.size(); ++i)
        queue.append(dynamic_cast<OmBaseNode *>(kids.item(i)));
    }
  }
  return false;
}

// circle parts
static const double HALF = M_PI;
static const double THIRD = 2 * M_PI / 3;
static const double QUARTER = M_PI_2;
static const double FIFTH = 2 * M_PI / 5;
static const double SIXTH = M_PI / 3;
static const double SEVENTH = 2 * M_PI / 7;

// number of predefined configurations
static const int NUM_PREDEFINED = 10;

// definition of predefined combinations
static const double POLAR[NUM_PREDEFINED][NUM_PREDEFINED][2] = {
  {{0, 0}},
  {{QUARTER, 1}, {-QUARTER, 1}},
  {{0, 1}, {THIRD, 1}, {-THIRD, 1}},
  {{0, 0}, {0, 1}, {THIRD, 1}, {-THIRD, 1}},
  {{0, 0}, {0, 1}, {QUARTER, 1}, {HALF, 1}, {-QUARTER, 1}},
  {{0, 0}, {0, 1}, {FIFTH, 1}, {2 * FIFTH, 1}, {3 * FIFTH, 1}, {4 * FIFTH, 1}},
  {{0, 0}, {0, 1}, {SIXTH, 1}, {2 * SIXTH, 1}, {3 * SIXTH, 1}, {4 * SIXTH, 1}, {5 * SIXTH, 1}},
  {{0, 0}, {0, 1}, {SEVENTH, 1}, {2 * SEVENTH, 1}, {3 * SEVENTH, 1}, {4 * SEVENTH, 1}, {5 * SEVENTH, 1}, {6 * SEVENTH, 1}},
  {{0, 0.3}, {THIRD, 0.3}, {-THIRD, 0.3}, {0, 1}, {SIXTH, 1}, {2 * SIXTH, 1}, {3 * SIXTH, 1}, {4 * SIXTH, 1}, {5 * SIXTH, 1}},
  {{0, 0},
   {0, 0.5},
   {THIRD, 0.5},
   {-THIRD, 0.5},
   {0, 1},
   {SIXTH, 1},
   {2 * SIXTH, 1},
   {3 * SIXTH, 1},
   {4 * SIXTH, 1},
   {5 * SIXTH, 1}}};

class SensorRay {
public:
  SensorRay() {
    mDistance = std::numeric_limits<double>::infinity();
    mGeom = NULL;
    mCollidedGeometry = NULL;
    mNewtonHit = false;
    mWeight = 1.0;
    mContactPosition[0] = 0.0;
    mContactPosition[1] = 0.0;
    mContactPosition[2] = 0.0;
    mContactNormal[0] = 0.0;
    mContactNormal[1] = 0.0;
    mContactNormal[2] = 0.0;
  };

  ~SensorRay() {
  }

  // for GENERIC
  void setCollision(OmGeometry *geometry, double depth) {
    if (depth < mDistance) {
      mCollidedGeometry = geometry;
      mDistance = depth;
    }
  }

  // for LASER and SONAR
  void setCollision(OmGeometry *geometry, const dContactGeom *contact) {
    if (contact->depth < mDistance) {
      mCollidedGeometry = geometry;
      // the meaning of ODE contact depth for a ray collision is "distance from ray's origin to the contact point"
      mDistance = contact->depth;
      memcpy(mContactPosition, contact->pos, sizeof(dVector3));
      memcpy(mContactNormal, contact->normal, sizeof(dVector3));

      // according to ODE, in a dContactGeom, the normal vector points "in" g1
      // this means that it point "out of" g2, this is fine if g2 is the primitive
      // however if g2 is the sensor's ray then we must invert the normal
    }
  }
  void resetCollision() {
    mCollidedGeometry = NULL;
    mNewtonHit = false;
    mDistance = std::numeric_limits<double>::infinity();
  }
  // A hit answered by the Newton raycast service (no ODE geom involved).
  // mCollidedGeometry stays NULL -- a Newton hit on static world geometry has
  // no OmGeometry to point at -- so hit tests must use hasHit(), never the
  // geometry pointer alone. The normal may be all-zero when the bundled
  // mujoco binding does not expose mj_ray's normal; consumers treat that as
  // "absent".
  void setNewtonHit(double distance, const double normal[3]) {
    mNewtonHit = true;
    mDistance = distance;
    mContactNormal[0] = normal[0];
    mContactNormal[1] = normal[1];
    mContactNormal[2] = normal[2];
  }
  bool hasHit() const { return mNewtonHit || mCollidedGeometry != NULL; }

  // getters
  double distance() const { return mDistance; }
  double weight() const { return mWeight; }
  dGeomID geom() const { return mGeom; }
  OmGeometry *collidedGeometry() const { return mCollidedGeometry; }
  const OmVector3 &direction() const { return mDirection; }
  const dReal *contactPosition() const { return mContactPosition; }
  const dReal *contactNormal() const { return mContactNormal; }

  // setters
  void setGeom(dGeomID geom) {
    mGeom = geom;
  }
  void setDirection(double x, double y, double z) { mDirection.setXyz(x, y, z); }
  void setDistance(double distance) { mDistance = distance; }
  void setWeight(double weight) { mWeight = weight; }

protected:
  OmVector3 mDirection;
  double mWeight;
  double mDistance;

  // ODE ray tracing
  dGeomID mGeom;
  OmGeometry *mCollidedGeometry;
  bool mNewtonHit;
  dVector3 mContactPosition;
  dVector3 mContactNormal;
};

void OmDistanceSensor::init() {
  mValue = 0.0;
  mDistance = std::numeric_limits<double>::infinity();
  mRayType = SONAR;
  mNRays = 1;
  mSensor = NULL;
  mRays = NULL;
  mLut = NULL;
  mIsSubscribedToRayTracing = false;
  mNeedToReconfigure = false;

  mLookupTable = findMFVector3("lookupTable");
  mType = findSFString("type");
  mAperture = findSFDouble("aperture");
  mNumberOfRays = findSFInt("numberOfRays");
  mGaussianWidth = findSFDouble("gaussianWidth");
  mResolution = findSFDouble("resolution");
  mRedColorSensitivity = findSFDouble("redColorSensitivity");
}

OmDistanceSensor::OmDistanceSensor(OmTokenizer *tokenizer) : OmSolidDevice("DistanceSensor", tokenizer) {
  init();
}

OmDistanceSensor::OmDistanceSensor(const OmDistanceSensor &other) : OmSolidDevice(other) {
  init();
}

OmDistanceSensor::OmDistanceSensor(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmDistanceSensor::~OmDistanceSensor() {
  delete mSensor;
  delete mLut;
  delete[] mRays;

  if (mIsSubscribedToRayTracing) {
    mIsSubscribedToRayTracing = false;
    OmSimulationState::instance()->unsubscribeToRayTracing();
  }
}

void OmDistanceSensor::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();
  updateRaySetup();
}

void OmDistanceSensor::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mLookupTable, &OmMFVector3::changed, this, &OmDistanceSensor::updateRaySetup);
  connect(mType, &OmSFString::changed, this, &OmDistanceSensor::updateRaySetup);
  connect(mAperture, &OmSFDouble::changed, this, &OmDistanceSensor::updateRaySetup);
  connect(mNumberOfRays, &OmSFInt::changed, this, &OmDistanceSensor::updateRaySetup);
  connect(mGaussianWidth, &OmSFDouble::changed, this, &OmDistanceSensor::updateRaySetup);
  connect(mResolution, &OmSFDouble::changed, this, &OmDistanceSensor::updateRaySetup);
  connect(mRedColorSensitivity, &OmSFDouble::changed, this, &OmDistanceSensor::updateRaySetup);
}

void OmDistanceSensor::updateRaySetup() {
  // update type
  if (mType->value().compare("generic", Qt::CaseInsensitive) == 0)
    mRayType = GENERIC;
  else if (mType->value().compare("infra-red", Qt::CaseInsensitive) == 0 ||
           mType->value().compare("infrared", Qt::CaseInsensitive) == 0)
    mRayType = INFRA_RED;
  else if (mType->value().compare("laser", Qt::CaseInsensitive) == 0)
    mRayType = LASER;
  else
    mRayType = SONAR;

  // correct invalid input values
  if (OmFieldChecker::resetDoubleIfNegative(this, mAperture, -mAperture->value()))
    return;  // in order to avoiding passing twice in this function
  if (OmFieldChecker::resetIntIfLess(this, mNumberOfRays, 1, 1))
    return;  // in order to avoiding passing twice in this function
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mGaussianWidth, 1.0))
    return;  // in order to avoiding passing twice in this function
  if (OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1, -1))
    return;  // in order to avoiding passing twice in this function
  if (OmFieldChecker::resetDoubleIfNegative(this, mRedColorSensitivity, -mRedColorSensitivity->value()))
    return;  // in order to avoiding passing twice in this function
  if (mRayType == LASER && mNumberOfRays->value() > 1) {
    parsingWarn(tr("'type' \"laser\" must have one single ray."));
    mNumberOfRays->setValue(1);
    return;  // in order to avoiding passing twice in this function
  }

  // rebuild the lookup table
  delete mLut;
  mLut = new OmLookupTable(*mLookupTable);

  // if a null aperture is specified together with multiple rays
  // then improve performance by casting one single ray only
  mNRays = (mNumberOfRays->value() > 1 && mAperture->isZero()) ? 1 : mNumberOfRays->value();

  // alloc/realloc buffers
  delete[] mRays;
  mRays = new SensorRay[mNRays];

  // compute the rays directions and weights
  setupRayDirs();
  setupRayWeights();

  // notify ode (if already created)
  if (areOdeObjectsCreated() && mRayType != INFRA_RED)
    createOdeRays();

  // subscribe for ray tracing if needed
  if (mRayType == INFRA_RED) {
    if (!mIsSubscribedToRayTracing) {
      mIsSubscribedToRayTracing = true;
      OmSimulationState::instance()->subscribeToRayTracing();
    }
  } else {
    if (mIsSubscribedToRayTracing) {
      mIsSubscribedToRayTracing = false;
      OmSimulationState::instance()->unsubscribeToRayTracing();
    }
  }

  mNeedToReconfigure = true;
}

void OmDistanceSensor::polarTo3d(double alpha, double theta, int i) {
  // rotate the cone so that its initial angle
  // looks upwards, that ways we obtain a left/right symmetry
  alpha += QUARTER;

  // rescale the cone tip angle to fit all rays
  // in the user-defined aperture angle
  theta *= mAperture->value() / 2;

  // first rotate around x-axis which is the sensors central ray axis
  const double x = cos(theta);
  double y = -sin(theta);

  // then rotate around z-axis
  const double z = -y * sin(alpha);
  y *= cos(alpha);

  mRays[i].setDirection(x, y, z);
}

void OmDistanceSensor::setupRayDirs() {
  if (mNRays == 1)
    mRays[0].setDirection(1, 0, 0);
  else {
    if (mNRays > NUM_PREDEFINED) {
      // Not a predefined configuration: arrange rays in a 3d
      // cone oriented towards x. The cone is further divided in a
      // number of thinner cones that will accomodate some rays
      int left = mNRays;  // number of rays left to be assigned
      int ncone;          // number of orbits
      int s = 0;          // number of so far assigned rays

      double *const alphas = new double[mNRays];
      double *const thetas = new double[mNRays];

      // as long as there are rays left to be assigned
      for (ncone = 0; left > 0; ncone++) {
        const int capacity = 3 * ncone + 1;
        const int m = (left > capacity) ? capacity : left;

        const double twoPiOverM = 2.0 * M_PI / m;
        // within a cone arrange the rays in a circle
        for (int i = 0; i < m; i++) {
          alphas[s] = s * twoPiOverM;
          thetas[s] = ncone;
          s++;
        }

        left -= capacity;
      }

      // for each ray ...
      for (int i = 0; i < mNRays; i++) {
        // rescale the cones to fit all rays
        thetas[i] /= (ncone - 1);

        // convert from polar to 3d coordinate system
        polarTo3d(alphas[i], thetas[i], i);
      }

      delete[] alphas;
      delete[] thetas;
    } else {
      // for each ray ...
      for (int i = 0; i < mNRays; i++)
        // convert from polar to 3d coordinate system
        polarTo3d(POLAR[mNRays - 1][i][0], POLAR[mNRays - 1][i][1], i);
    }
  }
}

void OmDistanceSensor::setupRayWeights() {
  // avoid calculations for the most common case: mNRays == 1
  if (mNRays == 1) {
    mRays[0].setWeight(1.0);
    return;
  }
  // create gaussian distribution
  double sum = 0;
  for (int i = 0; i < mNRays; i++) {
    const OmVector3 &dir = mRays[i].direction();
    const double theta = acos(dir.x() / dir.length());
    assert(!std::isnan(theta));
    const double temp = theta / (mAperture->value() * mGaussianWidth->value());
    const double w = exp(-(temp * temp));
    mRays[i].setWeight(w);
    sum += w;
  }

  // normalize such that the sum of all rays equals 1.0
  for (int i = 0; i < mNRays; i++) {
    double w = mRays[i].weight();
    mRays[i].setWeight(w / sum);
  }
}

void OmDistanceSensor::createOdeObjects() {
  OmSolidDevice::createOdeObjects();

  if (mRayType != INFRA_RED) {
    createOdeRays();
  }
}

void OmDistanceSensor::createOdeRays() {
  // ODE is gone: rays stay geomless; the Newton raycast service
  // (refreshRaysFromNewton, which builds its segments from the sensor
  // matrix, not from ODE geoms) is the answer path.
}

void OmDistanceSensor::setSensorRays() {
  const OmMatrix4 &m = matrix();
  const OmVector3 &trans = m.translation();
  for (int i = 0; i < mNRays; i++)
    if (mRays[i].geom()) {  // NOT INFRA_RED
      // get ray direction
      const OmVector3 &dir = mRays[i].direction();
      assert(!dir.isNull());

      // apply sensor's coordinate system transformation to rays
      OmVector3 r = m.sub3x3MatrixDot(dir);
      if (r.isNull())  // Prevent ODE from crashing on zero direction vector
        r.setXyz(1.0, 0.0, 0.0);
      // setup ray position and direction for ODE collision detection
      (void)r;  // unreachable: geom() is always null now
    }
}

void OmDistanceSensor::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();
  setSensorRays();
}

void OmDistanceSensor::prePhysicsStep(double ms) {
  OmSolidDevice::prePhysicsStep(ms);

  if (isPowerOn() && mSensor->needToRefreshInMs(ms) && mNRays > 0) {
    // reset collisions -- refreshRaysFromNewton() fills them back in after the
    // step, from mj_ray on the live mjModel
    for (int i = 0; i < mNRays; ++i)
      mRays[i].resetCollision();
  }
}

bool OmDistanceSensor::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmDistanceSensor::refreshRaysFromNewton() {
  // Kernel blocker #1 (ode-retirement-campaign.md): answer this sensor's rays
  // from the Newton raycast service (mj_ray over the live mjModel) instead of
  // the ODE collision pass. Value-parsed gate, DEFAULT OFF until the parity
  // suite covers every ray consumer; =1 opts a run in, =0/unset keeps ODE.
  static int sGate = -1;
  if (sGate < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_RAYCAST")).trimmed().toLower();
    // DEFAULT ON since 2026-08-07: every ray consumer answers from the
    // Newton service (mj_ray on the live mjModel), incl. the LASER
    // transparency re-cast and the joint-descending exclusion walk.
    // =0 reverts to the ODE ray verdicts while src/ode still ships.
    sGate = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  if (!sGate || mNRays <= 0 || mLut == nullptr)
    return;
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);

  // Own-robot exclusion, replicating odeNearCallback's same-robot rule
  // (OmSimulationCluster: a sensor never sees its own robot unless
  // selfCollision is on). Newton body indices are stable after registration;
  // rebuild only if the cache is empty and the robot has bodies.
  if (mNewtonExcludeBodies.isEmpty()) {
    const OmRobot *const r = robot();
    if (r != nullptr && !r->selfCollision())
      // shared walk: descends joint/slot endPoints too, so a jointed arm is
      // excluded exactly like ODE's same-robot rule excludes it
      OmSolidUtilities::collectNewtonBodies(const_cast<OmRobot *>(r), mNewtonExcludeBodies);
    if (mNewtonExcludeBodies.isEmpty())
      mNewtonExcludeBodies.append(-999);  // sentinel: "built, nothing to exclude"
  }

  const double lutMaxRange = mLut->maxMetricsRange();
  const OmMatrix4 &m = matrix();
  const OmVector3 &trans = m.translation();
  QVector<double> rays;
  rays.reserve(mNRays * 7);
  for (int i = 0; i < mNRays; ++i) {
    OmVector3 r = m.sub3x3MatrixDot(mRays[i].direction());
    if (r.isNull())
      r.setXyz(1.0, 0.0, 0.0);
    rays << trans.x() << trans.y() << trans.z() << r.x() << r.y() << r.z() << lutMaxRange;
  }
  QVector<OmNewtonRayHit> hits(mNRays);
  const int excludeCount = (mNewtonExcludeBodies.size() == 1 && mNewtonExcludeBodies[0] == -999)
                             ? 0 : mNewtonExcludeBodies.size();
  const int n = newton->raycastBatch(mNRays, rays.constData(), hits.data(),
                                     excludeCount ? mNewtonExcludeBodies.constData() : nullptr, excludeCount);
  if (n != mNRays)
    return;  // service unavailable this tick -- keep whatever the ODE pass wrote
  for (int i = 0; i < mNRays; ++i) {
    mRays[i].resetCollision();
    if (hits[i].dist >= 0.0)
      mRays[i].setNewtonHit(hits[i].dist, hits[i].normal);
  }

  // LASER transparency skip: ODE's rayCollisionCallback ignores a LASER hit
  // on a transparent geometry (the ray passes through and the per-geom
  // callbacks deliver whatever lies behind). Replicate by re-casting past
  // transparent-bounded bodies. Body-level approximation: the hit comes back
  // as a Newton body index, so a body is "transparent" when the Solid that
  // registered it has a transparent geometry in its boundingObject -- exact
  // for the transparent helper boxes this feature exists for, coarser than
  // ODE's per-geom verdict on a mixed compound (documented divergence).
  // INFRA_RED never reaches this function (color readback keeps it on ODE).
  if (mRayType == LASER) {
    QVector<int> extendedExcludes = excludeCount ? mNewtonExcludeBodies : QVector<int>();
    for (int round = 0; round < 4; ++round) {
      QVector<int> recastIdx;
      for (int i = 0; i < mNRays; ++i) {
        if (hits[i].dist < 0.0)
          continue;
        const OmSolid *const hitSolid = OmSolid::findSolidByNewtonBodyIndex(hits[i].newtonBody);
        if (hitSolid == nullptr || hitSolid->boundingObject() == nullptr)
          continue;
        if (boundingObjectHasTransparentGeometry(hitSolid->boundingObject())) {
          if (!extendedExcludes.contains(hits[i].newtonBody))
            extendedExcludes.append(hits[i].newtonBody);
          recastIdx.append(i);
        }
      }
      if (recastIdx.isEmpty())
        break;
      const int rn = recastIdx.size();
      QVector<double> rrays;
      rrays.reserve(rn * 7);
      foreach (const int i, recastIdx)
        for (int k = 0; k < 7; ++k)
          rrays.append(rays[i * 7 + k]);
      QVector<OmNewtonRayHit> rhits(rn);
      if (newton->raycastBatch(rn, rrays.constData(), rhits.data(), extendedExcludes.constData(),
                               extendedExcludes.size()) != rn)
        break;  // service hiccup: keep the current (first-hit) answers
      for (int j = 0; j < rn; ++j) {
        const int i = recastIdx[j];
        hits[i] = rhits[j];
        mRays[i].resetCollision();
        if (rhits[j].dist >= 0.0)
          mRays[i].setNewtonHit(rhits[j].dist, rhits[j].normal);
      }
    }
  }
}

void OmDistanceSensor::computeValue() {
  static const double PI_OVER_8 = 0.392699082;  // used for sonar type

  // Newton raycast service (kernel blocker #1, ode-retirement-campaign.md):
  // when active, answer the rays from the live mjModel instead of the ODE
  // collision pass. Runs AFTER the ODE pass filled mRays, so it owns the
  // final answer under the gate; with the gate off (default for now, until
  // the parity suite covers every consumer) nothing changes.
  if (mRayType != INFRA_RED)
    refreshRaysFromNewton();

  // value when no object collide
  if (mLut->isEmpty()) {
    mValue = 0.0;
    mDistance = std::numeric_limits<double>::infinity();
    return;
  }

  const double lutMaxRange = mLut->maxMetricsRange();

  if (mRayType == GENERIC) {
    // average all ray collision distances using ray weights
    mDistance = 0.0;
    for (int i = 0; i < mNRays; i++) {
      const double dist = (mRays[i].hasHit()) ? mRays[i].distance() : lutMaxRange;
      mDistance += dist * mRays[i].weight();
    }
  } else if (mRayType == INFRA_RED) {
    double averageInfraRedFactor = 0.0;
    mDistance = 0.0;
    for (int i = 0; i < mNRays; i++) {
      double distance = 0.0;
      // apply sensor's coordinate system transformation to rays
      const OmMatrix4 &m = matrix();
      const OmVector3 &trans = m.translation();
      OmVector3 r = m.sub3x3MatrixDot(mRays[i].direction());
      r.normalize();
      const OmShape *const shape = OmNodeUtilities::findIntersectingShape(OmRay(trans, r), lutMaxRange, distance);

      if (shape) {
        mRays[i].setDistance(distance);

        OmRgb pickedColor;
        double roughness, occlusion;
        shape->pickColor(OmRay(trans, r), pickedColor, &roughness, &occlusion);

        const double infraRedFactor = 0.8 * pickedColor.red() * (1 - 0.5 * roughness) * (1 - 0.5 * occlusion) + 0.2;
        averageInfraRedFactor += infraRedFactor * mRays[i].weight();
      } else
        averageInfraRedFactor += mRays[i].weight();

      mDistance += distance * mRays[i].weight();
    }

    // apply infrared reflection factor and red color sensitivity
    // before adding of red color sensitivity factor it was calculated with mDistance = mDistance / averageInfraRedFactor
    mDistance = mDistance + (mDistance / averageInfraRedFactor - mDistance) * mRedColorSensitivity->value();
  } else if (mRayType == SONAR) {
    // use only the nearest ray collision, ignore ray weight
    mDistance = lutMaxRange;
    for (int i = 0; i < mNRays; ++i)
      if (mRays[i].hasHit() && mRays[i].distance() < mDistance) {
        // compute angle between ray and contact normal
        // ODE is gone: recompute the same world-frame direction the ODE
        // ray geom would have carried (matrix() x local ray direction).
        const OmVector3 direction(matrix().sub3x3MatrixDot(mRays[i].direction()));
        OmVector3 normal(mRays[i].contactNormal());  // the contact normal is expressed in global coordinates
        // A zero normal means "normal unavailable" (the Newton raycast path
        // when the bundled mujoco binding does not expose mj_ray's normal):
        // accept the hit rather than reject it on an angle we cannot compute.
        if (normal.isNull()) {
          mDistance = mRays[i].distance();
          continue;
        }
        const double angle = direction.angle(-normal);
        // ignore contact outside of a reflection cone with 45 degrees aperture (experimental value)
        if (angle < PI_OVER_8)
          mDistance = mRays[i].distance();
      }
  } else if (mRayType == LASER)
    // consider only one ray (there should be only one)
    mDistance = (mRays[0].hasHit()) ? mRays[0].distance() : lutMaxRange;

  mValue = mLut->lookup(mDistance);
  if (mResolution->value() != -1.0)
    mValue = OmMathsUtilities::discretize(mValue, mResolution->value());
}

void OmDistanceSensor::rayCollisionCallback(OmGeometry *object, dGeomID rayGeom, const dContactGeom *contact) {
  if (!mSensor->isEnabled())
    return;

  if (object->isTransparent()) {
    if (mRayType == LASER || mRayType == INFRA_RED)
      return;
  }

  for (int i = 0; i < mNRays; i++)
    if (rayGeom == mRays[i].geom()) {
      if (mRayType == GENERIC)
        mRays[i].setCollision(object, contact->depth);
      else  // SONAR and LASER
        mRays[i].setCollision(object, contact);
      return;
    }

  assert(0);  // should never be reached
}

void OmDistanceSensor::handleMessage(QDataStream &stream) {
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

void OmDistanceSensor::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_DISTANCE_SENSOR_DATA;
    stream << mValue;

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmDistanceSensor::addConfigure(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mRayType;
  stream << (double)mLut->minValue();
  stream << (double)mLut->maxValue();
  stream << (double)mAperture->value();
  stream << (int)mLookupTable->size();
  for (int i = 0; i < mLookupTable->size(); i++) {
    stream << (double)mLookupTable->item(i).x();
    stream << (double)mLookupTable->item(i).y();
    stream << (double)mLookupTable->item(i).z();
  }
  mNeedToReconfigure = false;
}

void OmDistanceSensor::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

