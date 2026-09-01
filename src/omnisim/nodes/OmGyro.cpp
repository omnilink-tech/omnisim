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

#include "OmGyro.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmLookupTable.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSFDouble.hpp"
#include "OmSensor.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

void OmGyro::init() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;
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

OmGyro::OmGyro(OmTokenizer *tokenizer) : OmSolidDevice("Gyro", tokenizer) {
  init();
}

OmGyro::OmGyro(const OmGyro &other) : OmSolidDevice(other) {
  init();
}

OmGyro::OmGyro(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmGyro::~OmGyro() {
  delete mLut;
  delete mSensor;
}

void OmGyro::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();
  updateLookupTable();
}

void OmGyro::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mLookupTable, &OmMFVector3::changed, this, &OmGyro::updateLookupTable);
  connect(mResolution, &OmSFDouble::changed, this, &OmGyro::updateResolution);
}

void OmGyro::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new OmLookupTable(*mLookupTable);

  mNeedToReconfigure = true;
}

void OmGyro::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmGyro::handleMessage(QDataStream &stream) {
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

void OmGyro::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_GYRO_DATA;
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmGyro::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void OmGyro::addConfigure(OmDataStream &stream) {
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

bool OmGyro::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmGyro::computeValue() {
  // P1.5 + Newton integration: dispatch through whichever backend owns
  // the gyro's parent Solid. Newton-backed Solids now read true Newton
  // angular velocity instead of the bridge-proxy ODE state.
  // carrierBodyHandle, not bodyHandle: a gyro on a FOLDED carrier (nested
  // Solid, no joint to its parent -- the URDF importer's IMU emission
  // pattern) owns no body of its own; the fold leader's body is the
  // physically correct read (omega is identical throughout a rigid fold).
  OmSolid *const us = upperSolid();
  const OmBodyHandle bodyHandle = us ? us->carrierBodyHandle() : nullptr;

  if (bodyHandle) {
    // ⚠ CHECK THE RETURN -- it was being discarded. getBodyAngularVel returns -1
    // WITHOUT WRITING v[] when the Newton world is not running, and always on a
    // NEWTON=OFF build. Uninitialised, this published raw stack memory as an
    // angular velocity. Zero is the honest reading for "no body to ask".
    double v[3] = {0.0, 0.0, 0.0};
    if (us->physicsBackend()->getBodyAngularVel(bodyHandle, v) != 0)
      v[0] = v[1] = v[2] = 0.0;
    OmVector3 globalVelocity(v[0], v[1], v[2]);

    // from global to Gyro's local coordinate system
    OmVector3 localVelocity = globalVelocity * matrix();

    // lookup
    mValues[0] = mXAxis->isTrue() ? mLut->lookup(localVelocity.x()) : NAN;
    mValues[1] = mYAxis->isTrue() ? mLut->lookup(localVelocity.y()) : NAN;
    mValues[2] = mZAxis->isTrue() ? mLut->lookup(localVelocity.z()) : NAN;

    // apply resolution if needed
    if (mResolution->value() != -1.0) {
      if (mXAxis->isTrue())
        mValues[0] = OmMathsUtilities::discretize(mValues[0], mResolution->value());
      if (mYAxis->isTrue())
        mValues[1] = OmMathsUtilities::discretize(mValues[1], mResolution->value());
      if (mZAxis->isTrue())
        mValues[2] = OmMathsUtilities::discretize(mValues[2], mResolution->value());
    }
  } else {
    // Latched like OmAccelerometer's no-body warning: computeValue runs every
    // sensor refresh, so an unlatched warn floods the log at the refresh rate.
    if (!mWarningWasPrinted) {
      parsingWarn(tr("this node or its parents requires a 'physics' field to be functional."));
      mWarningWasPrinted = true;
    }
  }
}
