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

#include "OmLens.hpp"

void OmLens::init() {
  mCenter = findSFVector2("center");
  mRadialCoefficients = findSFVector2("radialCoefficients");
  mTangentialCoefficients = findSFVector2("tangentialCoefficients");
}

OmLens::OmLens(OmTokenizer *tokenizer) : OmBaseNode("Lens", tokenizer) {
  init();
}

OmLens::OmLens(const OmLens &other) : OmBaseNode(other) {
  init();
}

OmLens::OmLens(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmLens::~OmLens() {
}

void OmLens::preFinalize() {
  OmBaseNode::preFinalize();

  updateCenter();
  updateRadialCoefficients();
  updateTangentialCoefficients();
}

void OmLens::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mCenter, &OmSFVector2::changed, this, &OmLens::updateCenter);
  connect(mRadialCoefficients, &OmSFVector2::changed, this, &OmLens::updateRadialCoefficients);
  connect(mTangentialCoefficients, &OmSFVector2::changed, this, &OmLens::updateTangentialCoefficients);
}

void OmLens::updateCenter() {
  if (mCenter->value().x() < 0.0) {
    parsingWarn(tr("Invalid 'center.x' changed to 0. The value should be in the range [0;1]."));
    mCenter->setX(0.0);
  }
  if (mCenter->value().y() < 0.0) {
    parsingWarn(tr("Invalid 'center.y' changed to 0. The value should be in the range [0;1]."));
    mCenter->setY(0.0);
  }
  if (mCenter->value().x() > 1.0) {
    parsingWarn(tr("Invalid 'center.x' changed to 1. The value should be in the range [0;1]."));
    mCenter->setX(1.0);
  }
  if (mCenter->value().y() > 1.0) {
    parsingWarn(tr("Invalid 'center.y' changed to 1. The value should be in the range [0;1]."));
    mCenter->setX(1.0);
  }

  emit centerChanged();
}

void OmLens::updateRadialCoefficients() {
  emit radialCoefficientsChanged();
}

void OmLens::updateTangentialCoefficients() {
  emit tangentialCoefficientsChanged();
}
