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

#include "OmSensor.hpp"

#include "OmRobot.hpp"
#include "OmSimulationState.hpp"

#include <cassert>

OmSensor::OmSensor() :
  mRefreshRate(0),  // disabled
  mLastUpdate(-std::numeric_limits<double>::infinity()),
  mIsRemoteMode(false),
  mIsFirstValueReady(false),
  mHasPendingValue(false) {
}

void OmSensor::setRefreshRate(int rate) {
  mRefreshRate = rate;
  // first value available after sampling period elapsed
  mLastUpdate = OmSimulationState::instance()->time();
  if (mRefreshRate == 0)
    emit stateChanged();
  // else state will change when first value is read
}

bool OmSensor::needToRefresh() {
  if (mRefreshRate == 0 || elapsedTime() < mRefreshRate)
    return false;

  if (!mIsFirstValueReady) {
    mIsFirstValueReady = true;
    emit stateChanged();
  }
  return !mIsRemoteMode;
}

bool OmSensor::needToRefreshInMs(int ms) {
  if (mRefreshRate != 0 && !mIsRemoteMode)
    return (elapsedTime() + ms) >= mRefreshRate;
  return false;
}

void OmSensor::updateTimer() {
  mLastUpdate = OmSimulationState::instance()->time();
  mHasPendingValue = true;
}

double OmSensor::elapsedTime() const {
  return OmSimulationState::instance()->time() - mLastUpdate;
}

void OmSensor::toggleRemoteMode(bool enabled) {
  mIsRemoteMode = enabled;
}

void OmSensor::connectToRobotSignal(const OmRobot *robot, bool connectRemoteMode) {
  if (connectRemoteMode)
    connect(robot, &OmRobot::toggleRemoteMode, this, &OmSensor::toggleRemoteMode, Qt::UniqueConnection);
  connect(robot, &OmRobot::wasReset, this, &OmSensor::reset);
}

void OmSensor::reset() {
  mLastUpdate = -std::numeric_limits<double>::infinity();
  mIsFirstValueReady = false;
  mHasPendingValue = false;
}
