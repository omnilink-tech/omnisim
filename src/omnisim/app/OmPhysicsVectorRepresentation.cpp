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

#include "OmPhysicsVectorRepresentation.hpp"

#include "OmRotation.hpp"
#include "OmVector3.hpp"

// D1.4: the WREN arrow/tail/coil scene objects were excised with the renderer.
// The drag-arrow VISUAL is drawn by the wgpu overlay (gui/OmDragArrowLines),
// which reads the drag event's own begin/end state -- so every method here is
// a kept, state-free stub preserving the drag events' lifecycle calls
// (construct, updatePosition, setScale, delete).

// Abstract class //
////////////////////

// OmPhysicsVectorRepresentation functions

OmPhysicsVectorRepresentation::~OmPhysicsVectorRepresentation() {
}

void OmPhysicsVectorRepresentation::initializeTailAndArrow(const float * /*materialColor*/) {
  // D1.4: WREN arrow/tail meshes retired; OmDragArrowLines owns the geometry.
}

void OmPhysicsVectorRepresentation::setScale(float /*scale*/) {
  // D1.4: WREN head-transform scaling retired.
}

void OmPhysicsVectorRepresentation::updatePosition(const OmVector3 & /*begin*/, const OmVector3 & /*end*/,
                                                   const OmRotation & /*orientation*/) {
  // D1.4: WREN transform updates retired; OmDragArrowLines reads the drag
  // event's begin/end directly every frame.
}

// Implemented classes //
/////////////////////////

// OmForceRepresentation functions

OmForceRepresentation::OmForceRepresentation() {
  const float forceColor[3] = {1.0f, 0.5f, 0.0f};
  initializeTailAndArrow(forceColor);
}

// OmTorqueRepresentation functions

OmTorqueRepresentation::OmTorqueRepresentation() {
  const float torqueColor[3] = {1.0f, 0.85f, 0.0f};
  initializeTailAndArrow(torqueColor);
}

OmTorqueRepresentation::~OmTorqueRepresentation() {
}
