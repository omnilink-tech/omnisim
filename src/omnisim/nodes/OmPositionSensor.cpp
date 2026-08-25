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

//
//  OmPositionSensor.cpp
//
#include "OmPositionSensor.hpp"

#include "OmDataStream.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmJoint.hpp"
#include "OmMathsUtilities.hpp"
#include "OmPropeller.hpp"
#include "OmRandom.hpp"
#include "OmSolid.hpp"

#include "../../../include/controller/c/omnisim/position_sensor.h"  // position sensor types
#include "../../controller/c/messages.h"  // contains the definitions for the macros C_SET_SAMPLING_PERIOD, C_CONFIGURE

#include <QtCore/QDataStream>

#include <cassert>

OmPositionSensor::OmPositionSensor(const QString &modelName, OmTokenizer *tokenizer) : OmJointDevice(modelName, tokenizer) {
  init();
}

OmPositionSensor::OmPositionSensor(OmTokenizer *tokenizer) : OmJointDevice("PositionSensor", tokenizer) {
  init();
}

OmPositionSensor::OmPositionSensor(const OmPositionSensor &other) : OmJointDevice(other) {
  init();
}

OmPositionSensor::OmPositionSensor(const OmNode &other) : OmJointDevice(other) {
  init();
}

void OmPositionSensor::init() {
  mSensor = new OmSensor();
  mValue = 0.0;
  mRequestedDeviceTag = NULL;
  mNoise = findSFDouble("noise");
  mResolution = findSFDouble("resolution");
}

void OmPositionSensor::postFinalize() {
  OmJointDevice::postFinalize();
  connect(mNoise, &OmSFDouble::changed, this, &OmPositionSensor::updateNoise);
  connect(mResolution, &OmSFDouble::changed, this, &OmPositionSensor::updateResolution);
}

void OmPositionSensor::updateNoise() {
  OmFieldChecker::resetDoubleIfNegative(this, mNoise, 0.0);
}

void OmPositionSensor::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmPositionSensor::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());

  stream << (unsigned short)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)type();
}

void OmPositionSensor::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;
  if (command & C_SET_SAMPLING_PERIOD) {
    short rate;
    stream >> rate;
    mSensor->setRefreshRate(rate);
  }
  if (command & C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE) {
    short deviceType;
    stream >> deviceType;
    assert(mRequestedDeviceTag == NULL);
    mRequestedDeviceTag = new WbDeviceTag[1];
    const OmLogicalDevice *device = getSiblingDeviceByType(deviceType);
    if (!device && deviceType == WB_NODE_ROTATIONAL_MOTOR)
      // check both motor types
      device = getSiblingDeviceByType(WB_NODE_LINEAR_MOTOR);
    mRequestedDeviceTag[0] = device ? device->tag() : 0;
  }
}

double OmPositionSensor::position() const {
  // get exact position
  double pos = OmJointDevice::position();
  // apply noise if needed
  if (mNoise->value() > 0.0)
    pos += mNoise->value() * OmRandom::nextGaussian();
  // apply resolution if needed
  if (mResolution->value() != -1.0)
    pos = OmMathsUtilities::discretize(pos, mResolution->value());
  return pos;
}

void OmPositionSensor::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_POSITION_SENSOR_DATA;
    stream << mValue;
    mSensor->resetPendingValue();
  }
  if (mRequestedDeviceTag != NULL) {
    stream << tag();
    stream << (unsigned char)C_POSITION_SENSOR_GET_ASSOCIATED_DEVICE;
    stream << (unsigned short)mRequestedDeviceTag[0];
    delete[] mRequestedDeviceTag;
    mRequestedDeviceTag = NULL;
  }
}

bool OmPositionSensor::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    mValue = position();
    mSensor->updateTimer();
    return true;
  }
  return false;
}
