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

#include "OmTranslateRotateManipulator.hpp"

#include "OmAbstractPose.hpp"
#include "OmMatrix4.hpp"

namespace {
  // The WREN handle transforms' LOCAL pose, verbatim from the old initializeHandlesEntities():
  // a tiny per-axis offset so translucent handles sorted deterministically, and the per-axis
  // rotation that aims the +Y-authored meshes down X/Y/Z.
  const float kHandleOffset[3][3] = {{0.0001f, 0.0f, 0.0f}, {0.0f, 0.0001f, 0.0f}, {0.0f, 0.0f, 0.0001f}};
  // (angle, x, y, z). Axis 1 (Y) is the identity: WREN passed a zero axis with angle 0, which
  // its transform treated as identity -- spell it with a safe unit axis so the matrix ctor
  // cannot normalize a null vector.
  const float kHandleRotation[3][4] = {
    {1.570796327f, 0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {1.570796327f, 1.0f, 0.0f, 0.0f}};

  void toColumnMajor(const OmMatrix4 &m, float out16[16]) {
    for (int c = 0; c < 4; ++c)
      for (int r = 0; r < 4; ++r)
        out16[c * 4 + r] = static_cast<float>(m(r, c));
  }
}  // namespace

const OmVector3 OmTranslateRotateManipulator::STANDARD_COORDINATE_VECTORS[3] = {
  OmVector3(1.0, 0.0, 0.0), OmVector3(0.0, 1.0, 0.0), OmVector3(0.0, 0.0, 1.0)};

OmTranslateRotateManipulator::OmTranslateRotateManipulator(bool isTranslationAvailable, bool isRotationAvailable) :
  OmWrenAbstractManipulator(3),
  mHasRotationHandles(isRotationAvailable),
  mHasTranslationHandles(isTranslationAvailable),
  mAttachedPose(NULL),
  mRotationLineVisible(false) {
  for (int i = 0; i < 3; ++i) {
    mTranslationHandleVisible[i] = true;
    mRotationHandleVisible[i] = true;
    mInfiniteAxisVisible[i] = false;
  }
}

OmTranslateRotateManipulator::~OmTranslateRotateManipulator() {
}

void OmTranslateRotateManipulator::attachTo(const OmAbstractPose *pose) {
  mAttachedPose = pose;
  markAttached();
}

void OmTranslateRotateManipulator::highlightAxis(int index) {
  OmWrenAbstractManipulator::highlightAxis(index);

  for (int i = 0; i < 3; ++i) {
    mRotationHandleVisible[i] = false;
    mTranslationHandleVisible[i] = false;
  }

  const int handleIndex = index % 3;
  if (index < 3 && mHasTranslationHandles)
    mTranslationHandleVisible[handleIndex] = true;
  else if (mHasRotationHandles)
    mRotationHandleVisible[handleIndex] = true;

  if (mHasTranslationHandles || mHasRotationHandles)
    mInfiniteAxisVisible[handleIndex] = true;
}

void OmTranslateRotateManipulator::showNormal() {
  OmWrenAbstractManipulator::showNormal();

  for (int i = 0; i < 3; ++i) {
    mRotationHandleVisible[i] = true;
    mTranslationHandleVisible[i] = true;
    mInfiniteAxisVisible[i] = false;
  }
}

void OmTranslateRotateManipulator::showRotationLine(bool show) {
  if (!mHasRotationHandles)
    return;
  mRotationLineVisible = show;
}

void OmTranslateRotateManipulator::updateRotationLine(const OmVector3 &begin, const OmVector3 &end,
                                                      const OmRotation &orientation, float arrowScale) {
  // State only (the WREN line + double-arrow renderables are gone; P8's open leftover).
  (void)orientation;
  (void)arrowScale;
  mRotationLineBegin = begin;
  mRotationLineEnd = end;
}

OmVector3 OmTranslateRotateManipulator::relativeHandlePosition(int handleNumber) const {
  int axis = handleNumber % 3;
  OmVector3 position = STANDARD_COORDINATE_VECTORS[axis];
  if (handleNumber > 2)
    position[axis] -= 0.1f;

  return position * mScale;
}

bool OmTranslateRotateManipulator::handleMatrix(int axis, float out16[16]) const {
  if (!mAttachedPose || axis < 0 || axis > 2)
    return false;
  const float *un = handleUnscale();
  // poseWorld * S(unscale) * (T(offset) R(rot)): OmMatrix4's VRML constructor composes
  // T * R * S, so build the local as two factors to keep the ordering exact.
  const OmMatrix4 unscale(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, un[0], un[1], un[2]);
  const OmMatrix4 local(kHandleOffset[axis][0], kHandleOffset[axis][1], kHandleOffset[axis][2],
                        kHandleRotation[axis][1], kHandleRotation[axis][2], kHandleRotation[axis][3],
                        kHandleRotation[axis][0], 1.0, 1.0, 1.0);
  const OmMatrix4 world = mAttachedPose->matrix() * unscale * local;
  toColumnMajor(world, out16);
  return true;
}

bool OmTranslateRotateManipulator::translationHandleMatrix(int axis, float out16[16]) const {
  if (!mHasTranslationHandles)
    return false;
  return handleMatrix(axis, out16);
}

bool OmTranslateRotateManipulator::rotationHandleMatrix(int axis, float out16[16]) const {
  if (!mHasRotationHandles)
    return false;
  return handleMatrix(axis, out16);
}

bool OmTranslateRotateManipulator::axesMatrix(float out16[16]) const {
  if (!mAttachedPose)
    return false;
  const float *un = handleUnscale();
  const OmMatrix4 unscale(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, un[0], un[1], un[2]);
  const OmMatrix4 world = mAttachedPose->matrix() * unscale;
  toColumnMajor(world, out16);
  return true;
}
