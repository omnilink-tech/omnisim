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

#include "OmFocus.hpp"

#include "OmFieldChecker.hpp"

void OmFocus::init() {
  mFocalDistance = findSFDouble("focalDistance");
  mFocalLength = findSFDouble("focalLength");
  mMaxFocalDistance = findSFDouble("maxFocalDistance");
  mMinFocalDistance = findSFDouble("minFocalDistance");
}

OmFocus::OmFocus(OmTokenizer *tokenizer) : OmBaseNode("Focus", tokenizer) {
  init();
}

OmFocus::OmFocus(const OmFocus &other) : OmBaseNode(other) {
  init();
}

OmFocus::OmFocus(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmFocus::~OmFocus() {
}

void OmFocus::preFinalize() {
  OmBaseNode::preFinalize();

  updateFocalDistance();
  updateFocalLength();
  updateMinFocalDistance();
  updateMaxFocalDistance();
}

void OmFocus::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mFocalDistance, &OmSFDouble::changed, this, &OmFocus::updateFocalDistance);
  connect(mFocalLength, &OmSFDouble::changed, this, &OmFocus::updateFocalLength);
  connect(mMinFocalDistance, &OmSFDouble::changed, this, &OmFocus::updateMinFocalDistance);
  connect(mMaxFocalDistance, &OmSFDouble::changed, this, &OmFocus::updateMaxFocalDistance);
}

void OmFocus::updateFocalDistance() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mFocalDistance, 0.0))
    return;
  emit focusSettingsChanged();
}

void OmFocus::updateFocalLength() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mFocalLength, 0.0))
    return;
  emit focusSettingsChanged();
}

void OmFocus::updateMinFocalDistance() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mMinFocalDistance, 0.0))
    return;
  if (mMinFocalDistance->value() > mMaxFocalDistance->value()) {
    parsingWarn(tr("Invalid 'minFocalDistance' changed to %1. The value should be smaller or equal to 'maxFocalDistance'.")
                  .arg(mMaxFocalDistance->value()));
    mMinFocalDistance->setValue(mMaxFocalDistance->value());
    return;
  }
}

void OmFocus::updateMaxFocalDistance() {
  if (mMaxFocalDistance->value() < mMinFocalDistance->value()) {
    parsingWarn(tr("Invalid 'maxFocalDistance' changed to %1. The value should be bigger or equal to 'minFocalDistance'.")
                  .arg(mMinFocalDistance->value() + 0.1));
    mMaxFocalDistance->setValue(mMinFocalDistance->value() + 0.1);
    return;
  }
}
