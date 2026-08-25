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

#include "OmTouchSensor.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmLookupTable.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSFDouble.hpp"
#include "OmSensor.hpp"
#include "OmSolidMerger.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only
#include <QtCore/QDataStream>
#include <cassert>
#include "../../controller/c/messages.h"

// OMNISIM_NEWTON_TOUCH_FORCE (value-parsed, DEFAULT OFF): route "force" /
// "force-3d" reads to the Newton backend's cfrc_int mount-wrench service
// (_scratch/design_weld_touch.md T1). Must match the un-fold gate in
// OmSolid.cpp byte-for-byte -- the read is only meaningful for a sensor that
// was un-folded into its own Newton body at registration.
static bool newtonTouchForceEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_TOUCH_FORCE")).trimmed().toLower();
    // DEFAULT ON since 2026-08-08: ODE is being deleted, so there is no
    // fallback left to degrade to -- an unset gate must mean the native
    // path, not silence. "0"/"false"/"off"/"no" still opt out.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

// OMNISIM_NEWTON_BUMPER (value-parsed, DEFAULT OFF): answer a "bumper"
// TouchSensor from the Newton backend's NATIVE contact set instead of the ODE
// near-callback's setTouching(). Kernel gap found by the deletion audit: a
// bumper is fed ONLY by odeNearCallback (OmSimulationCluster), which is dead
// for Newton bodies -- so every bumper on a Newton robot silently reads 0.
static bool newtonBumperEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_BUMPER")).trimmed().toLower();
    // DEFAULT ON since 2026-08-08: ODE is being deleted, so there is no
    // fallback left to degrade to -- an unset gate must mean the native
    // path, not silence. "0"/"false"/"off"/"no" still opt out.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

// The available Newton backend, or NULL (runtime absent). The forced-ODE
// early-out this used to start with is gone with the OMNISIM_FORCE_ODE switch.
static OmNewtonBackend *availableNewtonBackend() {
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return nullptr;
  return static_cast<OmNewtonBackend *>(raw);
}

void OmTouchSensor::init() {
  mIsTouching = false;
  mWarnedNewtonBumperInert = false;
  mIsGuiTouch = false;
  mFeedback = NULL;
  mLut = NULL;
  mSensor = NULL;
  mDeviceType = BUMPER;
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;
  mWarnedNewtonInert = false;

  mLookupTable = findMFVector3("lookupTable");
  mType = findSFString("type");
  mResolution = findSFDouble("resolution");

  mNeedToReconfigure = false;
}

OmTouchSensor::OmTouchSensor(OmTokenizer *tokenizer) : OmSolidDevice("TouchSensor", tokenizer) {
  init();
}

OmTouchSensor::OmTouchSensor(const OmTouchSensor &other) : OmSolidDevice(other) {
  init();
}

OmTouchSensor::OmTouchSensor(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmTouchSensor::~OmTouchSensor() {
  delete mLut;
  delete mSensor;
}

void OmTouchSensor::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();

  updateLookupTable();
  updateType();
}

void OmTouchSensor::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mLookupTable, &OmMFVector3::changed, this, &OmTouchSensor::updateLookupTable);
  connect(mType, &OmSFString::changed, this, &OmTouchSensor::updateType);
  connect(mResolution, &OmSFDouble::changed, this, &OmTouchSensor::updateResolution);
}

void OmTouchSensor::updateLookupTable() {
  // rebuild lookup table
  delete mLut;
  mLut = new OmLookupTable(*mLookupTable);

  mValues[0] = mLut->minValue();
  mValues[1] = mLut->minValue();
  mValues[2] = mLut->minValue();

  mNeedToReconfigure = true;
}

void OmTouchSensor::updateType() {
  if (mType->value() == "bumper")
    mDeviceType = BUMPER;
  else if (mType->value() == "force")
    mDeviceType = FORCE;
  else if (mType->value() == "force-3d")
    mDeviceType = FORCE3D;
  else
    mDeviceType = BUMPER;

  if (mDeviceType == BUMPER && mType->value() != "bumper")
    parsingWarn(tr("Unknown 'type': \"%1\". Set to \"bumper\"").arg(mType->value()));

  if ((mDeviceType == FORCE || mDeviceType == FORCE3D) && !physics())
    parsingWarn(tr("\"force\" and \"force-3d\" 'type' requires 'physics' to be functional."));
}

void OmTouchSensor::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmTouchSensor::handleMessage(QDataStream &stream) {
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

void OmTouchSensor::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    if (mDeviceType != FORCE3D) {  // BUMPER or FORCE
      stream << (unsigned char)C_TOUCH_SENSOR_DATA;
      stream << (double)mValues[0];
    } else {  // FORCE_3D
      stream << (unsigned char)C_TOUCH_SENSOR_DATA_3D;
      stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];
    }
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmTouchSensor::addConfigure(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mDeviceType;
  stream << (int)mLookupTable->size();
  for (int i = 0; i < mLookupTable->size(); i++) {
    stream << (double)mLookupTable->item(i).x();
    stream << (double)mLookupTable->item(i).y();
    stream << (double)mLookupTable->item(i).z();
  }
  mNeedToReconfigure = false;
}

bool OmTouchSensor::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

bool OmTouchSensor::newtonMountForce(OmVector3 &f) const {
  // Meaningful only for a sensor un-folded into its OWN Newton body (the
  // registration-side half of OMNISIM_NEWTON_TOUCH_FORCE): newtonBodyIndex()
  // is this sensor's own body, never the merge leader's.
  if (!newtonTouchForceEnabled() || newtonBodyIndex() < 0)
    return false;
  OmNewtonBackend *const newton = availableNewtonBackend();
  if (newton == nullptr || !newton->isWorldRunning())
    return false;
  double w[6];
  if (newton->touchForce(newtonBodyIndex(), w) != 0)
    return false;
  // w[0..2] is already ODE-f1-compatible (negated cfrc_int = force on the
  // PARENT side of the mount), so the math below runs verbatim on it.
  f = OmVector3(w[0], w[1], w[2]);
  return true;
}

bool OmTouchSensor::newtonBumperTouching(bool &touching) const {
  // A bumper asks one binary question: is anything in contact with ME? Under
  // the P3.10c fold a device Solid owns no body, so a folded bumper's contacts
  // are indistinguishable from its chassis's -- the registration side un-folds
  // bumpers into their own body under this same gate, and newtonBodyIndex() is
  // then this sensor's own body. Without that, refuse rather than answer with
  // the parent's contacts (a bumper that reports the chassis touching a wall
  // is worse than one that says it cannot see).
  if (!newtonBumperEnabled() || newtonBodyIndex() < 0)
    return false;
  OmNewtonBackend *const newton = availableNewtonBackend();
  if (newton == nullptr || !newton->isWorldRunning())
    return false;
  std::vector<OmNewtonContact> contacts;
  newton->getContacts(contacts);
  const int me = newtonBodyIndex();
  touching = false;
  for (size_t i = 0; i < contacts.size(); ++i) {
    if (contacts[i].bodyA == me || contacts[i].bodyB == me) {
      touching = true;
      break;
    }
  }
  return true;
}

void OmTouchSensor::computeValue() {
  // Newton mount-wrench source (OMNISIM_NEWTON_TOUCH_FORCE, default ON). The
  // ODE mount-joint feedback that used to be the fall-back source is gone, so
  // when the native readback does not answer there is NO force source at all and
  // mValues[0] is a lookup of 0.0. Say so once rather than reporting a silent
  // 0 N for ever (W0 of the design).
  //
  // The gate condition is newtonBodyIndex() < 0, i.e. the sensor was never
  // un-folded into a Newton body of its own -- the STRUCTURAL reason, permanent
  // for the run. It deliberately does not fire on the transient reasons
  // (world not running yet, touchForce() error), because mWarnedNewtonInert is
  // one-shot and a spurious early fire would consume the real warning.
  OmVector3 newtonF1;
  const bool useNewton = forceBehavior() && newtonMountForce(newtonF1);
  if (forceBehavior() && !useNewton && !mWarnedNewtonInert && newtonBodyIndex() < 0 &&
      nearestNewtonBodyIndex() >= 0) {
    mWarnedNewtonInert = true;
    parsingWarn(tr("This \"%1\" TouchSensor has NO force source and will read 0 N: it was not un-folded into a "
                   "Newton body of its own, so the native mount-force readback cannot answer, and the ODE "
                   "mount-joint feedback it used to fall back to has been removed. Do not read this 0 as a "
                   "measured force.")
                  .arg(mType->value()));
  }

  if (mDeviceType == FORCE) {  // "force" sensors
    // TODO: check why mFeedback is NULL
    if (!useNewton && !mFeedback) {
      mValues[0] = mLut->lookup(0.0);
      return;
    }

    const OmVector3 f1 = newtonF1;  // mFeedback never exists here; !useNewton already returned above

    // by convention, TouchSensor's x-axis is the sensitive axis
    // the orientation of the x-axis can be read directly from the rotation matrix
    OmVector3 xaxis = matrix().xAxis();

    // compute how much of the force is aligned with the TouchSensors's x-axis
    // use dot product: |force| * cos(theta) = dot(xaxis, sum) / |xaxis|
    // we know that: |xaxis| == 1.0 (approximatively), therefore it can be ignored
    double force = xaxis.dot(f1);

    // ignore negative forces because they would represent a pull rather than a push on the sensor
    force = force < 0.0 ? -force : 0.0;

    // lookup and add noise
    mValues[0] = mLut->lookup(force);

    // apply resolution
    if (mResolution->value() != -1.0)
      mValues[0] = OmMathsUtilities::discretize(mValues[0], mResolution->value());
  } else if (mDeviceType == FORCE3D) {
    // TODO: check why mFeedback is NULL
    if (!useNewton && !mFeedback) {
      mValues[0] = mLut->lookup(0.0);
      mValues[1] = mLut->lookup(0.0);
      mValues[2] = mLut->lookup(0.0);
      return;
    }

    const OmVector3 f1 = newtonF1;  // mFeedback never exists here; !useNewton already returned above

    // take into account the sensor's current orientation
    OmVector3 vec = f1 * matrix();

    // lookup
    mValues[0] = mLut->lookup(vec[0]);
    mValues[1] = mLut->lookup(vec[1]);
    mValues[2] = mLut->lookup(vec[2]);

    // apply resolution
    if (mResolution->value() != -1.0) {
      mValues[0] = OmMathsUtilities::discretize(mValues[0], mResolution->value());
      mValues[1] = OmMathsUtilities::discretize(mValues[1], mResolution->value());
      mValues[2] = OmMathsUtilities::discretize(mValues[2], mResolution->value());
    }
  } else {  // BUMPER
    // for "bumpers" we don't need to do the math, we return a binary value.
    // The only source now is the native Newton contact set (OMNISIM_NEWTON_BUMPER,
    // default ON): the ODE near-callback's setTouching() that used to feed
    // mIsTouching went with ODE, so if newtonBumperTouching() cannot answer,
    // nothing sets the flag and the sensor reads a silent 0. Warn on the
    // STRUCTURAL reason -- no Newton body of its own, permanent for the run --
    // and not on the transient ones, because the warning is one-shot.
    bool nativeTouching = false;
    if (newtonBumperTouching(nativeTouching))
      mIsTouching = nativeTouching;
    else if (!mWarnedNewtonBumperInert && newtonBodyIndex() < 0 && nearestNewtonBodyIndex() >= 0) {
      mWarnedNewtonBumperInert = true;
      parsingWarn(tr("This \"bumper\" TouchSensor has NO touch source and will read 0: it was not un-folded into "
                     "a Newton body of its own, so the native contact set cannot be asked about it, and the ODE "
                     "collision callback that used to set its flag has been removed."));
    }
    mValues[0] = (mIsTouching != mIsGuiTouch) ? 1 : 0;
    mIsTouching = false;  // reset once it was read
  }
}

void OmTouchSensor::setODEDynamicFlag(const OmBaseNode *_node) {
  const OmGeometry *geom = dynamic_cast<const OmGeometry *>(_node);

  if (!geom) {
    const OmShape *shape = dynamic_cast<const OmShape *>(_node);
    if (shape)
      geom = shape->geometry();
  }
  // A geom used to be marked dGeomSetDynamicFlag() here; ODE is gone, so there
  // is nothing to flag and only the Group recursion is left to do.
  if (!geom) {
    const OmGroup *group = dynamic_cast<const OmGroup *>(_node);
    if (group) {
      for (int i = 0; i < group->childCount(); i++)
        setODEDynamicFlag(group->child(i));
    }
  }
}

void OmTouchSensor::createOdeObjects() {
  OmSolidDevice::createOdeObjects();
  const OmBaseNode *node = OmSolidDevice::boundingObject();
  setODEDynamicFlag(node);
}

dJointID OmTouchSensor::createJoint(dBodyID body, dBodyID parentBody, dWorldID world) const {
  (void)body;
  (void)parentBody;
  (void)world;
  return NULL;  // ODE is gone: the newtonMountForce readback is the force source
}

void OmTouchSensor::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

bool OmTouchSensor::forceBehavior() const {
  return (mDeviceType == FORCE || mDeviceType == FORCE3D);
}

void OmTouchSensor::setSolidMerger() {
  mSolidMerger = physics() ? new OmSolidMerger(this) : NULL;
}
