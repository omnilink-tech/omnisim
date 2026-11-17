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

#include "OmAltimeter.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmMathsUtilities.hpp"
#include "OmRandom.hpp"
#include "OmSFDouble.hpp"
#include "OmSensor.hpp"
#include "OmWorld.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

#include <cassert>

void OmAltimeter::init() {
  mType = findSFString("type");
  mAccuracy = findSFDouble("accuracy");
  mResolution = findSFDouble("resolution");
  mSensor = NULL;
}

OmAltimeter::OmAltimeter(OmTokenizer *tokenizer) : OmSolidDevice("Altimeter", tokenizer) {
  init();
}

OmAltimeter::OmAltimeter(const OmAltimeter &other) : OmSolidDevice(other) {
  init();
}

OmAltimeter::OmAltimeter(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmAltimeter::~OmAltimeter() {
  delete mSensor;
}

void OmAltimeter::preFinalize() {
  OmSolidDevice::preFinalize();
  mSensor = new OmSensor();
}

void OmAltimeter::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mResolution, &OmSFDouble::changed, this, &OmAltimeter::updateResolution);
}

void OmAltimeter::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

bool OmAltimeter::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  const OmVector3 &t = matrix().translation();

  // compute current altitude
  double accuracy = mAccuracy->value();

  const OmVector3 reference = OmWorld::instance()->worldInfo()->gpsReference();
  const QString &coordinateSystem = OmWorld::instance()->worldInfo()->coordinateSystem();
  const int upIndex = coordinateSystem.indexOf('U');
  if (OmWorld::instance()->worldInfo()->gpsCoordinateSystem() == "WGS84")
    mMeasuredAltitude = reference[2];
  else
    mMeasuredAltitude = reference[upIndex];

  mMeasuredAltitude += t[upIndex];  // get exact altitude
  // add noise if necessary
  if (accuracy != 0.0)
    mMeasuredAltitude += accuracy * OmRandom::nextGaussian();
  // apply resolution if necessary
  if (mResolution->value() != -1.0)
    mMeasuredAltitude = OmMathsUtilities::discretize(mMeasuredAltitude, mResolution->value());
  mSensor->updateTimer();
  return true;
}

void OmAltimeter::reset(const QString &id) {
  OmSolidDevice::reset(id);
}

void OmAltimeter::handleMessage(QDataStream &stream) {
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

void OmAltimeter::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_ALTIMETER_DATA;
    stream << (double)mMeasuredAltitude;

    mSensor->resetPendingValue();
  }
}

void OmAltimeter::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
}
