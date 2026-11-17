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

#include "OmAbstractDragEvent.hpp"

#include "OmVector2.hpp"
#include "OmVector3.hpp"
#include "OmWrenLabelOverlay.hpp"

// Main abstract class //
/////////////////////////
const float OmDragEvent::cFloatMax = std::numeric_limits<float>::max();

OmDragEvent::OmDragEvent() : QObject() {
}

bool OmDragEvent::exceedsFloatMax(const OmVector3 &v) {
  return (fabs(v.x()) >= cFloatMax || fabs(v.y()) >= cFloatMax || fabs(v.z()) >= cFloatMax);
}

bool OmDragEvent::exceedsFloatMax(double x) {
  return fabs(x) >= cFloatMax;
}

bool OmDragEvent::exceedsFloatMax(float x) {
  return fabs(x) >= cFloatMax;
}

//
//  View 3D drag event classes (dragging the mouse modifies the content of the 3D view and requires OmViewpoint)
//

// Abstract classes //
//////////////////////

// OmDragView3DEvent constructor
OmDragView3DEvent::OmDragView3DEvent(OmViewpoint *viewpoint) :
  OmDragEvent(),
  mViewpoint(viewpoint),
  mViewDistanceUnscaling(1.0f) {
}

OmVector2 OmDragView3DEvent::clampLabelPosition(const float x, const float y, const OmWrenLabelOverlay *overlay) {
  OmVector2 clamped(x, y);
  const float labelWidth = overlay->width();
  const float labelHeight = overlay->height();

  if (x < 0.0f)
    clamped[0] = 0.0f;
  else if (x > 1.0f - labelWidth)
    clamped[0] = 1.0f - labelWidth;

  if (y < 0.0f)
    clamped[1] = 0.0f;
  else if (y > 1.0f - labelHeight)
    clamped[1] = 1.0f - labelHeight;

  return clamped;
}

// Kinematics drag events

// OmDragKinematicsEvent constructor
OmDragKinematicsEvent::OmDragKinematicsEvent(OmViewpoint *viewpoint) : OmDragView3DEvent(viewpoint) {
}
