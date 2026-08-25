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

#include "OmInertialUnit.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmRandom.hpp"
#include "OmSensor.hpp"
#include "OmWorld.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

void OmInertialUnit::init() {
  mSensor = NULL;

  mNoise = findSFDouble("noise");
  mXAxis = findSFBool("xAxis");
  mYAxis = findSFBool("yAxis");
  mZAxis = findSFBool("zAxis");
  mResolution = findSFDouble("resolution");

  mNeedToReconfigure = false;
}

OmInertialUnit::OmInertialUnit(OmTokenizer *tokenizer) : OmSolidDevice("InertialUnit", tokenizer) {
  init();
}

OmInertialUnit::OmInertialUnit(const OmInertialUnit &other) : OmSolidDevice(other) {
  init();
}

OmInertialUnit::OmInertialUnit(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmInertialUnit::~OmInertialUnit() {
  delete mSensor;
}

void OmInertialUnit::preFinalize() {
  OmSolidDevice::preFinalize();
  mSensor = new OmSensor();
  updateNoise();
}

void OmInertialUnit::postFinalize() {
  OmSolidDevice::postFinalize();
  connect(mResolution, &OmSFDouble::changed, this, &OmInertialUnit::updateResolution);
  connect(mNoise, &OmMFVector3::changed, this, &OmInertialUnit::updateNoise);
}

void OmInertialUnit::updateNoise() {
  mNeedToReconfigure = true;
}

void OmInertialUnit::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmInertialUnit::handleMessage(QDataStream &stream) {
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

void OmInertialUnit::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_INERTIAL_UNIT_DATA;
    stream << (double)mQuaternion.x() << (double)mQuaternion.y() << (double)mQuaternion.z() << (double)mQuaternion.w();

    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmInertialUnit::addConfigure(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)mNoise->value();
  const QByteArray &s = OmWorld::instance()->worldInfo()->coordinateSystem().toUtf8();
  stream.writeRawData(s.constData(), s.size() + 1);
  mNeedToReconfigure = false;
}

void OmInertialUnit::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

bool OmInertialUnit::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmInertialUnit::computeValue() {
  OmMatrix3 e = rotationMatrix();

  if (mNoise->value() != 0.0) {
    const double noise = mNoise->value() * M_PI;
    e *= OmMatrix3(noise * OmRandom::nextGaussian(), noise * OmRandom::nextGaussian(), noise * OmRandom::nextGaussian());
  }

  if (!mXAxis->isTrue() || !mYAxis->isTrue() || !mZAxis->isTrue()) {
    OmRotation rotation(e);
    if (!mXAxis->isTrue())
      rotation.setX(0);
    if (!mYAxis->isTrue())
      rotation.setZ(0);
    if (!mZAxis->isTrue())
      rotation.setY(0);
    rotation.normalizeAxis();
    e = rotation.toMatrix3();
  }

  mQuaternion = e.toQuaternion();

  // apply resolution if needed
  if (mResolution->value() != -1.0) {
    mQuaternion.setX(OmMathsUtilities::discretize(mQuaternion.x(), mResolution->value()));
    mQuaternion.setY(OmMathsUtilities::discretize(mQuaternion.y(), mResolution->value()));
    mQuaternion.setZ(OmMathsUtilities::discretize(mQuaternion.z(), mResolution->value()));
    mQuaternion.setW(OmMathsUtilities::discretize(mQuaternion.w(), mResolution->value()));
  }
}
