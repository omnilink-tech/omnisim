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

#include "OmRecognition.hpp"

#include "OmFieldChecker.hpp"

void OmRecognition::init() {
  mMaxRange = findSFDouble("maxRange");
  mMaxObjects = findSFInt("maxObjects");
  mOcclusion = findSFInt("occlusion");
  mFrameColor = findSFColor("frameColor");
  mFrameThickness = findSFInt("frameThickness");
  mSegmentation = findSFBool("segmentation");
}

OmRecognition::OmRecognition(OmTokenizer *tokenizer) : OmBaseNode("Recognition", tokenizer) {
  init();
}

OmRecognition::OmRecognition(const OmRecognition &other) : OmBaseNode(other) {
  init();
}

OmRecognition::OmRecognition(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmRecognition::~OmRecognition() {
}

void OmRecognition::preFinalize() {
  OmBaseNode::preFinalize();

  updateMaxRange();
  updateMaxObjects();
  updateOcclusion();
}

void OmRecognition::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mMaxRange, &OmSFDouble::changed, this, &OmRecognition::updateMaxRange);
  connect(mMaxObjects, &OmSFInt::changed, this, &OmRecognition::updateMaxObjects);
  connect(mOcclusion, &OmSFInt::changed, this, &OmRecognition::updateOcclusion);
  connect(mFrameThickness, &OmSFInt::changed, this, &OmRecognition::updateFrameThickness);
  connect(mSegmentation, &OmSFBool::changed, this, &OmRecognition::segmentationChanged);
}

void OmRecognition::updateMaxRange() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mMaxRange, 100.0))
    return;
}

void OmRecognition::updateMaxObjects() {
  if (OmFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mMaxObjects, -1, -1))
    return;
}

void OmRecognition::updateOcclusion() {
  if (mOcclusion->value() < 0 || mOcclusion->value() > 2) {
    parsingWarn(tr("Invalid 'occlusion' changed to 1. The value should be 0, 1 or 2."));
    mOcclusion->setValue(1);
  }
}

void OmRecognition::updateFrameThickness() {
  if (OmFieldChecker::resetIntIfNegative(this, mFrameThickness, 0))
    return;
}
