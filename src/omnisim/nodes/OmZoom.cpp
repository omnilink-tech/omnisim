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

#include "OmZoom.hpp"

#include "OmFieldChecker.hpp"

void OmZoom::init() {
  mMaxFieldOfView = findSFDouble("maxFieldOfView");
  mMinFieldOfView = findSFDouble("minFieldOfView");
}

OmZoom::OmZoom(OmTokenizer *tokenizer) : OmBaseNode("Zoom", tokenizer) {
  init();
}

OmZoom::OmZoom(const OmZoom &other) : OmBaseNode(other) {
  init();
}

OmZoom::OmZoom(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmZoom::~OmZoom() {
}

void OmZoom::preFinalize() {
  OmBaseNode::preFinalize();

  updateMinFieldOfView();
  updateMaxFieldOfView();
}

void OmZoom::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mMinFieldOfView, &OmSFDouble::changed, this, &OmZoom::updateMinFieldOfView);
  connect(mMaxFieldOfView, &OmSFDouble::changed, this, &OmZoom::updateMaxFieldOfView);
}

void OmZoom::updateMinFieldOfView() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mMinFieldOfView, 0.0))
    return;
  if (mMinFieldOfView->value() > mMaxFieldOfView->value()) {
    parsingWarn(tr("Invalid 'minFieldOfView' changed to %1. The value should be smaller or equal to 'maxFieldOfView'.")
                  .arg(mMaxFieldOfView->value()));
    mMinFieldOfView->setValue(mMaxFieldOfView->value());
    return;
  }
}

void OmZoom::updateMaxFieldOfView() {
  if (mMaxFieldOfView->value() < mMinFieldOfView->value()) {
    parsingWarn(tr("Invalid 'maxFieldOfView' changed to %1. The value should be bigger or equal to 'minFieldOfView'.")
                  .arg(mMinFieldOfView->value() + 0.1));
    mMaxFieldOfView->setValue(mMinFieldOfView->value() + 0.1);
    return;
  }
}
