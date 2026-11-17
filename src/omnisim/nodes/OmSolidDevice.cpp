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

#include "OmSolidDevice.hpp"
#include "OmNodeUtilities.hpp"
#include "OmRobot.hpp"
#include "OmSolidMerger.hpp"

OmSolidDevice::OmSolidDevice(const QString &modelName, OmTokenizer *tokenizer) : OmDevice(), OmSolid(modelName, tokenizer) {
}

OmSolidDevice::OmSolidDevice(const OmSolidDevice &other) : OmDevice(other), OmSolid(other) {
}

OmSolidDevice::OmSolidDevice(const OmNode &other) : OmDevice(), OmSolid(other) {
}

OmSolidDevice::~OmSolidDevice() {
}

QString OmSolidDevice::computeShortUniqueName() const {
  const OmSolid *robot = OmNodeUtilities::findRobotAncestor(this);
  if (robot)
    return QString("%1:%2").arg(robot->computeUniqueName()).arg(OmSolid::name());
  return computeUniqueName();
}
