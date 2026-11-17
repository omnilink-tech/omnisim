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

#include "OmLensFlare.hpp"

#include "OmFieldChecker.hpp"
#include "OmLight.hpp"
#include "OmMFVector2.hpp"
#include "OmNodeUtilities.hpp"
#include "OmRay.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmWorld.hpp"

void OmLensFlare::init() {
  mTransparency = findSFDouble("transparency");
  mScale = findSFDouble("scale");
  mBias = findSFDouble("bias");
  mDispersal = findSFDouble("dispersal");
  mHaloWidth = findSFDouble("haloWidth");
  mChromaDistortion = findSFDouble("chromaDistortion");
  mSamples = findSFInt("samples");
  mBlurIterations = findSFInt("blurIterations");
}

OmLensFlare::OmLensFlare(OmTokenizer *tokenizer) : OmBaseNode("LensFlare", tokenizer) {
  init();
}

OmLensFlare::OmLensFlare(const OmLensFlare &other) : OmBaseNode(other) {
  init();
}

OmLensFlare::OmLensFlare(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmLensFlare::~OmLensFlare() {
}

void OmLensFlare::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mTransparency, &OmSFDouble::changed, this, &OmLensFlare::updateTransparency);
  connect(mScale, &OmSFDouble::changed, this, &OmLensFlare::updateScale);
  connect(mBias, &OmSFDouble::changed, this, &OmLensFlare::updateBias);
  connect(mDispersal, &OmSFDouble::changed, this, &OmLensFlare::updateDispersal);
  connect(mHaloWidth, &OmSFDouble::changed, this, &OmLensFlare::updateHaloWidth);
  connect(mChromaDistortion, &OmSFDouble::changed, this, &OmLensFlare::updateChromaDistortion);
  connect(mSamples, &OmSFInt::changed, this, &OmLensFlare::updateSamples);
  connect(mBlurIterations, &OmSFInt::changed, this, &OmLensFlare::updateBlur);
}

void OmLensFlare::detachFromViewport() {
  // D1.4: the WREN lens-flare post-processing effect is retired; nothing to detach.
}

void OmLensFlare::updateTransparency() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.5))
    return;
}

void OmLensFlare::updateScale() {
  // D1.4: field state only; the WREN lens-flare effect is retired.
}

void OmLensFlare::updateBias() {
  // D1.4: field state only; the WREN lens-flare effect is retired.
}

void OmLensFlare::updateDispersal() {
  // D1.4: field state only; the WREN lens-flare effect is retired.
}

void OmLensFlare::updateHaloWidth() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mHaloWidth, 0.5))
    return;
}

void OmLensFlare::updateChromaDistortion() {
  // D1.4: field state only; the WREN lens-flare effect is retired.
}

void OmLensFlare::updateSamples() {
  if (OmFieldChecker::resetIntIfNegative(this, mSamples, 1))
    return;
}

void OmLensFlare::updateBlur() {
  if (OmFieldChecker::resetIntIfNegative(this, mBlurIterations, 2))
    return;
}
