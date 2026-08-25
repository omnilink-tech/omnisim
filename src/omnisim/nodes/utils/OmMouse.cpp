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

#include "OmMouse.hpp"

#include "OmSensor.hpp"

#include <cmath>

QList<OmMouse *> OmMouse::mMouses;

OmMouse *OmMouse::create() {
  OmMouse *m = new OmMouse();
  mMouses << m;
  return m;
}

void OmMouse::destroy(OmMouse *mouse) {
  mMouses.removeOne(mouse);
  delete mouse;
}

OmMouse::OmMouse() : mSensor(NULL), mHasMoved(false), mHasClicked(false), mIsTracked(false), mIs3dPositionEnabled(false) {
  reset();
}

OmMouse::~OmMouse() {
  delete mSensor;
}

void OmMouse::setRefreshRate(int rate) {
  if (mSensor == NULL)
    mSensor = new OmSensor();
  mSensor->setRefreshRate(rate);
}

int OmMouse::refreshRate() const {
  if (mSensor)
    return mSensor->refreshRate();
  return 0.0;
}

bool OmMouse::hasPendingValue() {
  return mSensor != NULL ? mSensor->hasPendingValue() : false;
}

void OmMouse::reset() {
  mLeft = false;
  mMiddle = false;
  mRight = false;
  mU = NAN;
  mV = NAN;
  mX = NAN;
  mY = NAN;
  mZ = NAN;
}

bool OmMouse::refreshSensorIfNeeded() {
  if (mSensor->needToRefresh()) {
    mSensor->updateTimer();
    return true;
  }
  return false;
}

bool OmMouse::needToRefresh() const {
  if (mSensor->needToRefresh() || mIsTracked)
    return true;
  return false;
}
