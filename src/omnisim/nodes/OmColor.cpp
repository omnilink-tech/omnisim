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

#include "OmColor.hpp"

#include "OmFieldChecker.hpp"
#include "OmMFColor.hpp"

void OmColor::init() {
  mColor = findMFColor("color");
}

OmColor::OmColor(OmTokenizer *tokenizer) : OmBaseNode("Color", tokenizer) {
  init();
}

OmColor::OmColor(const OmColor &other) : OmBaseNode(other) {
  init();
}

OmColor::OmColor(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmColor::~OmColor() {
}

void OmColor::preFinalize() {
  OmBaseNode::preFinalize();

  updateColor();
}

void OmColor::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mColor, &OmMFColor::changed, this, &OmColor::updateColor);
}

void OmColor::updateColor() {
  if (OmFieldChecker::resetMultipleColorIfInvalid(this, mColor))
    return;
  emit changed();
}

void OmColor::copyValuesToArray(double array[][3]) const {
  int nColors = mColor->size();
  for (int i = 0; i < nColors; i++) {
    OmRgb r = mColor->item(i);
    array[i][0] = r.red();
    array[i][1] = r.green();
    array[i][2] = r.blue();
  }
}

QStringList OmColor::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "color";
  return fields;
}
