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

#include "OmWrenRenderingContext.hpp"

OmWrenRenderingContext *OmWrenRenderingContext::cRenderingContext = NULL;

void OmWrenRenderingContext::setWrenRenderingContext(int width, int height) {
  cleanup();
  cRenderingContext = new OmWrenRenderingContext(width, height);
}

void OmWrenRenderingContext::cleanup() {
  delete cRenderingContext;
}

const double OmWrenRenderingContext::SOLID_LINE_SCALE_FACTOR = 0.1;

OmWrenRenderingContext::OmWrenRenderingContext(int width, int height) :
  QObject(),
  mWidth(width),
  mHeight(height),
  mLineScale(0.0),
  mSolidLineScale(0.0),
  mRenderingMode(RM_PLAIN),
  mProjectionMode(PM_PERSPECTIVE),
  mOptionalRenderingsMask(VM_MAIN) {
  assert(VM_OMNISIM_RANGE_CAMERA == VM_REGULAR + (unsigned int)VF_LASER_BEAM);
}

OmWrenRenderingContext::~OmWrenRenderingContext() {
  if (cRenderingContext)
    cRenderingContext = NULL;
}

void OmWrenRenderingContext::setDimension(int width, int height) {
  mWidth = width;
  mHeight = height;
  emit dimensionChanged();
}

void OmWrenRenderingContext::setLineScale(float lineScale) {
  mSolidLineScale = SOLID_LINE_SCALE_FACTOR * lineScale;

  emit lineScaleChanged();
}

bool OmWrenRenderingContext::isOptionalRenderingEnabled(int optionalRendering) const {
  return (optionalRendering & mOptionalRenderingsMask) != 0;
}

unsigned int OmWrenRenderingContext::visibilityMask() const {
  int visibilityMask = mOptionalRenderingsMask;
  if (mRenderingMode == RM_WIREFRAME && isOptionalRenderingEnabled(VF_ALL_BOUNDING_OBJECTS))
    visibilityMask &= ~VM_REGULAR;
  return visibilityMask;
}

void OmWrenRenderingContext::enableOptionalRendering(int optionalRendering, bool enable, bool userAction) {
  if (enable)
    mOptionalRenderingsMask |= optionalRendering;
  else
    mOptionalRenderingsMask &= ~optionalRendering;

  emit optionalRenderingChanged(optionalRendering);
  if (userAction)
    emit view3dRefreshRequired();
}

void OmWrenRenderingContext::setRenderingMode(int renderingMode, bool userAction) {
  mRenderingMode = renderingMode;
  emit renderingModeChanged();
  if (userAction)
    emit view3dRefreshRequired();
}

void OmWrenRenderingContext::setProjectionMode(int projectionMode, bool userAction) {
  mProjectionMode = projectionMode;
  emit projectionModeChanged();
  if (userAction)
    emit view3dRefreshRequired();
}

void OmWrenRenderingContext::requestView3dRefresh() {
  emit view3dRefreshRequired();
}
