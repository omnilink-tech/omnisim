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

#include "OmGuiRefreshOracle.hpp"
#include "OmSimulationState.hpp"

#include <QtCore/QCoreApplication>

OmGuiRefreshOracle *OmGuiRefreshOracle::cInstance = NULL;

OmGuiRefreshOracle *OmGuiRefreshOracle::instance() {
  if (!cInstance)
    cInstance = new OmGuiRefreshOracle();
  return cInstance;
}

void OmGuiRefreshOracle::cleanup() {
  if (cInstance) {
    delete cInstance;
    cInstance = NULL;
  }
}

OmGuiRefreshOracle::OmGuiRefreshOracle() : mCanRefreshNow(true) {
  qAddPostRoutine(OmGuiRefreshOracle::cleanup);

  OmSimulationState *state = OmSimulationState::instance();
  connect(state, &OmSimulationState::modeChanged, this, &OmGuiRefreshOracle::updateFlags);
  connect(state, &OmSimulationState::controllerReadRequestsCompleted, this, &OmGuiRefreshOracle::updateFlags);
  connect(&mGlobalRefreshTimer, &QTimer::timeout, this, &OmGuiRefreshOracle::updateFlags);

  // The global refresh timer should not timeout faster than one refresh period
  // so we use the smallest value bigger than the refresh period to check if
  // we should update but haven't yet
  mGlobalRefreshTimer.start(301);
  mLastRefreshTimer.start();
}

OmGuiRefreshOracle::~OmGuiRefreshOracle() {
}

void OmGuiRefreshOracle::updateFlags() {
  // set the mCanRefreshNow to true at frequency 3.33 Hz
  if (!OmSimulationState::instance()->isPaused() && mLastRefreshTimer.elapsed() < 300) {
    mCanRefreshNow = false;
    // restart global refresh timer
    mGlobalRefreshTimer.start(301);
    return;
  }

  bool canRefreshNowHasChanged = !mCanRefreshNow;
  mCanRefreshNow = true;

  if (canRefreshNowHasChanged)
    emit canRefreshActivated();
  emit canRefreshUpdated();
  mLastRefreshTimer.restart();
}
