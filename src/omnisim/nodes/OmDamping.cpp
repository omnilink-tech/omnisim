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

#include "OmDamping.hpp"
#include "OmWorld.hpp"

void OmDamping::init() {
  mLinear = findSFDouble("linear");
  mAngular = findSFDouble("angular");
}

OmDamping::OmDamping(OmTokenizer *tokenizer) : OmBaseNode("Damping", tokenizer) {
  init();
}

OmDamping::OmDamping(const OmDamping &other) : OmBaseNode(other) {
  init();
}

OmDamping::OmDamping(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmDamping::~OmDamping() {
}

void OmDamping::preFinalize() {
  OmBaseNode::preFinalize();

  updateLinear();
  updateAngular();
}

void OmDamping::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mLinear, &OmSFDouble::changed, this, &OmDamping::updateLinear);
  connect(mAngular, &OmSFDouble::changed, this, &OmDamping::updateAngular);
}

void OmDamping::updateLinear() {
  if (mLinear->value() < 0.0) {
    parsingWarn(tr("'linear' must be greater than or equal to zero."));
    mLinear->setValue(0.0);
    return;
  }

  if (mLinear->value() > 1.0) {
    parsingWarn(tr("'linear' must be less than or equal to one."));
    mLinear->setValue(1.0);
    return;
  }

  emit changed();
}

void OmDamping::updateAngular() {
  if (mAngular->value() < 0.0) {
    parsingWarn(tr("'angular' must be greater than or equal to zero."));
    mAngular->setValue(0.0);
    return;
  }

  if (mAngular->value() > 1.0) {
    parsingWarn(tr("'angular' must be less than or equal to one."));
    mAngular->setValue(1.0);
    return;
  }
  emit changed();
}
