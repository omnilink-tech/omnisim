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

#include "OmAccelerometer.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmLookupTable.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSensor.hpp"
#include "OmWorld.hpp"

#include <QtCore/QDataStream>
#include <cassert>
#include "../../controller/c/messages.h"

void OmAccelerometer::init() {
  for (int i = 0; i < 3; i++) {
    mVelocity[i] = 0.0;
    mValues[i] = 0.0;
  }

  mLut = NULL;
  mSensor = NULL;

  mLookupTable = findMFVector3("lookupTable");
  mXAxis = findSFBool("xAxis");
  mYAxis = findSFBool("yAxis");
  mZAxis = findSFBool("zAxis");
  mResolution = findSFDouble("resolution");

  mNeedToReconfigure = false;
  mWarningWasPrinted = false;
}

OmAccelerometer::OmAccelerometer(OmTokenizer *tokenizer) : OmSolidDevice("Accelerometer", tokenizer) {
  init();
}

OmAccelerometer::OmAccelerometer(const OmAccelerometer &other) : OmSolidDevice(other) {
  init();
}

OmAccelerometer::OmAccelerometer(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmAccelerometer::~OmAccelerometer() {
  delete mLut;
  delete mSensor;
}

void OmAccelerometer::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();
  updateLookupTable();
}

void OmAccelerometer::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mLookupTable, &OmMFVector3::changed, this, &OmAccelerometer::updateLookupTable);
  connect(mResolution, &OmSFDouble::changed, this, &OmAccelerometer::updateResolution);
}

void OmAccelerometer::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new OmLookupTable(*mLookupTable);

  mNeedToReconfigure = true;
}

void OmAccelerometer::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmAccelerometer::handleMessage(QDataStream &stream) {
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

void OmAccelerometer::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void OmAccelerometer::addConfigure(OmDataStream &stream) {
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

void OmAccelerometer::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_ACCELEROMETER_DATA;
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

bool OmAccelerometer::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmAccelerometer::computeValue() {
  // set acceleration due to gravity
  const OmVector3 &gravity = OmWorld::instance()->worldInfo()->gravityVector();
  OmVector3 acceleration(-gravity);

  // add other acceleration (computed from changes in velocity)
  // P1.5 + Newton integration: dispatch through whichever backend
  // owns the upper Solid's body. ODE-backed Solids hit the ODE
  // override; Newton-backed Solids now get real point-velocity
  // reads via the Newton override (was previously reading bridge-
  // proxy state via the ODE backend, which was stale or zero).
  // carrierBodyHandle, not bodyHandle: an accelerometer on a FOLDED carrier
  // (nested Solid, no joint to its parent -- the URDF importer's IMU emission
  // pattern) owns no body of its own; the fold leader's body is the physically
  // correct read, and the point-velocity query below passes the SENSOR's own
  // world position, so the lever arm inside the rigid fold is right.
  OmSolid *const us = upperSolid();
  const OmBodyHandle upperHandle = us ? us->carrierBodyHandle() : nullptr;
  if (upperHandle) {
    // ⚠ THE RETURN VALUE IS LOAD-BEARING AND WAS BEING DISCARDED. getBodyPointVel
    // returns -1 WITHOUT WRITING v[] whenever the Newton world is not running --
    // and unconditionally on a NEWTON=OFF build, whose stub is a bare
    // `return -1;` (OmNewtonBackend.cpp). With the array left uninitialised that
    // published raw stack memory as an acceleration: not a wrong number, an
    // arbitrary one, and different every run.
    double newVelocity[3] = {0.0, 0.0, 0.0};
    const OmVector3 &t = position();
    const double pt[3] = {t.x(), t.y(), t.z()};
    if (us->physicsBackend()->getBodyPointVel(upperHandle, pt, newVelocity) == 0) {
      const double inverseDt = 1000.0 / mSensor->elapsedTime();

      for (int i = 0; i < 3; ++i) {
        acceleration[i] += (newVelocity[i] - mVelocity[i]) * inverseDt;
        mVelocity[i] = newVelocity[i];
      }
    }
    // On failure `acceleration` keeps the gravity term computed above and
    // mVelocity is left alone, so the next successful read differences against
    // the last real velocity rather than against zero -- which would otherwise
    // manufacture a spike of |v|/dt out of nothing.
  } else {
    if (!mWarningWasPrinted) {
      warn(tr("Parent of Accelerometer node has no physics: measurements may be wrong. Only the gravity term is published (there is no body to difference velocity against), so the reading is at best -g. Add a Physics node to the Solid carrying this sensor (or to the ancestor it is folded into) -- see docs/reference/accelerometer.md."));
      mWarningWasPrinted = true;
    }
    // No body to difference velocities against, but the gravity term computed
    // above is still real: fall through and publish it via the same
    // sensor-frame projection the success path uses. A body-less accelerometer
    // at rest must read -g in its own frame, not NaN.
  }

  const OmVector3 &result = acceleration * matrix();

  // apply lookup table
  mValues[0] = mXAxis->isTrue() ? mLut->lookup(result.x()) : NAN;
  mValues[1] = mYAxis->isTrue() ? mLut->lookup(result.y()) : NAN;
  mValues[2] = mZAxis->isTrue() ? mLut->lookup(result.z()) : NAN;

  // apply resolution if needed
  if (mResolution->value() != -1.0) {
    if (mXAxis->isTrue())
      mValues[0] = OmMathsUtilities::discretize(mValues[0], mResolution->value());
    if (mYAxis->isTrue())
      mValues[1] = OmMathsUtilities::discretize(mValues[1], mResolution->value());
    if (mZAxis->isTrue())
      mValues[2] = OmMathsUtilities::discretize(mValues[2], mResolution->value());
  }
}
