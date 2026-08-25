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

#include "OmWrenAbstractManipulator.hpp"

#include "OmVector2.hpp"
#include "OmWrenRenderingContext.hpp"

OmWrenAbstractManipulator::OmWrenAbstractManipulator(int numberOfHandles) :
  mIsVisible(false),
  mOriginScaleFactorNeeded(false),
  mScale(1.0),
  mNumberOfHandles(numberOfHandles),
  mIsActive(false) {
  mUnscale[0] = 1.0f;
  mUnscale[1] = 1.0f;
  mUnscale[2] = 1.0f;
}

OmWrenAbstractManipulator::~OmWrenAbstractManipulator() {
}

void OmWrenAbstractManipulator::show() {
  mIsVisible = true;
  computeHandleScaleFromViewportSize();
}

void OmWrenAbstractManipulator::hide() {
  mIsVisible = false;
}

void OmWrenAbstractManipulator::showNormal() {
  computeHandleScaleFromViewportSize();
}

void OmWrenAbstractManipulator::updateHandleScale(const double *scale) {
  mUnscale[0] = static_cast<float>(1.0 / scale[0]);
  mUnscale[1] = static_cast<float>(1.0 / scale[1]);
  mUnscale[2] = static_cast<float>(1.0 / scale[2]);
}

void OmWrenAbstractManipulator::computeHandleScaleFromViewportSize() {
  // WREN's own formula (handles at a constant ~100 px), fed from the view dimensions the
  // rendering context tracks (OmView3D updates them on every resize).
  const float sizeOnScreen = 100;
  const OmWrenRenderingContext *const ctx = OmWrenRenderingContext::instance();
  const float width = ctx ? ctx->width() : 1;
  const float height = ctx ? ctx->height() : 1;
  const float maxDimension = height > width ? height : width;
  mScale = 2 * sizeOnScreen / (maxDimension > 0 ? maxDimension : 1);
}
