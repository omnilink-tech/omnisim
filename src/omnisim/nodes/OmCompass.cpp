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

#include "OmCompass.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmLookupTable.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmSensor.hpp"
#include "OmWorld.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

void OmCompass::init() {
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
}

OmCompass::OmCompass(OmTokenizer *tokenizer) : OmSolidDevice("Compass", tokenizer) {
  init();
}

OmCompass::OmCompass(const OmCompass &other) : OmSolidDevice(other) {
  init();
}

OmCompass::OmCompass(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmCompass::~OmCompass() {
  delete mLut;
  delete mSensor;
}

void OmCompass::preFinalize() {
  OmSolidDevice::preFinalize();

  mSensor = new OmSensor();
  updateLookupTable();
}

void OmCompass::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(mLookupTable, &OmMFVector3::changed, this, &OmCompass::updateLookupTable);
  connect(mResolution, &OmSFDouble::changed, this, &OmCompass::updateResolution);
}

void OmCompass::updateLookupTable() {
  mValues[0] = 0.0;
  mValues[1] = 0.0;
  mValues[2] = 0.0;

  // create the lookup table
  delete mLut;
  mLut = new OmLookupTable(*mLookupTable);

  mNeedToReconfigure = true;
}

void OmCompass::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmCompass::handleMessage(QDataStream &stream) {
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

void OmCompass::writeAnswer(OmDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_COMPASS_DATA;
    stream << (double)mValues[0] << (double)mValues[1] << (double)mValues[2];
    mSensor->resetPendingValue();
  }

  if (mNeedToReconfigure)
    addConfigure(stream);
}

void OmCompass::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigure(stream);
}

void OmCompass::addConfigure(OmDataStream &stream) {
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

bool OmCompass::refreshSensorIfNeeded() {
  if (isPowerOn() && mSensor->needToRefresh()) {
    computeValue();
    mSensor->updateTimer();
    return true;
  }
  return false;
}

void OmCompass::computeValue() {
  // get global north
  assert(OmWorld::instance()->worldInfo());
  const OmVector3 &globalNorth = OmWorld::instance()->worldInfo()->northVector();

  // convert from global to Compass local coordinate system
  OmVector3 localNorth = globalNorth * matrix();

  // normalize
  localNorth.normalize();

  // lookup
  mValues[0] = mXAxis->isTrue() ? mLut->lookup(localNorth.x()) : NAN;
  mValues[1] = mYAxis->isTrue() ? mLut->lookup(localNorth.y()) : NAN;
  mValues[2] = mZAxis->isTrue() ? mLut->lookup(localNorth.z()) : NAN;

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
