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

#include "OmDirectionalLight.hpp"
#include "OmField.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector3.hpp"

void OmDirectionalLight::init() {
  mDirection = findSFVector3("direction");
}

OmDirectionalLight::OmDirectionalLight(OmTokenizer *tokenizer) : OmLight("DirectionalLight", tokenizer) {
  init();
  if (tokenizer == NULL)
    mDirection->setValueNoSignal(0, -1, 0);
}

OmDirectionalLight::OmDirectionalLight(const OmDirectionalLight &other) : OmLight(other) {
  init();
}

OmDirectionalLight::OmDirectionalLight(const OmNode &other) : OmLight(other) {
  init();
}

void OmDirectionalLight::preFinalize() {
  OmLight::preFinalize();

  updateDirection();
}

void OmDirectionalLight::postFinalize() {
  OmLight::postFinalize();

  connect(mDirection, &OmSFVector3::changed, this, &OmDirectionalLight::updateDirection);
}

OmDirectionalLight::~OmDirectionalLight() {
  // D1.4: nothing to release since the WREN light object was excised.
}

void OmDirectionalLight::updateDirection() {
  emit directionChanged();
}

const OmVector3 &OmDirectionalLight::direction() const {
  return mDirection->value();
}

QStringList OmDirectionalLight::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "direction" << OmLight::fieldsToSynchronizeWithW3d();

  return fields;
}
