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

#include "OmWrenAbstractResizeManipulator.hpp"

const int OmWrenAbstractResizeManipulator::STANDARD_COORDINATES[3] = {X, Y, Z};
const OmVector3 OmWrenAbstractResizeManipulator::STANDARD_COORDINATE_VECTORS[3] = {
  OmVector3(1.0, 0.0, 0.0), OmVector3(0.0, 1.0, 0.0), OmVector3(0.0, 0.0, 1.0)};

OmWrenAbstractResizeManipulator::OmWrenAbstractResizeManipulator(ResizeConstraint constraint) :
  OmWrenAbstractManipulator(3),
  mConstraint(constraint),
  mUniformMaterialIndex(0) {
}

OmWrenAbstractResizeManipulator::~OmWrenAbstractResizeManipulator() {
}

void OmWrenAbstractResizeManipulator::setResizeConstraint(ResizeConstraint constraint) {
  mConstraint = constraint;
}

void OmWrenAbstractResizeManipulator::highlightAxis(int index) {
  OmWrenAbstractManipulator::highlightAxis(index);
}

void OmWrenAbstractResizeManipulator::showNormal() {
  OmWrenAbstractManipulator::showNormal();
}

void OmWrenAbstractResizeManipulator::updateHandleDimensions(const float scaleFactor, const float viewDistanceScale) {
  // State-only (D1.4): the WREN handle/axis transforms this used to move are gone; a future
  // wgpu resize-handle drawer recomputes its geometry from the manipulator + geometry state.
  (void)scaleFactor;
  (void)viewDistanceScale;
}
