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

#ifndef OM_TRANSLATE_ROTATE_MANIPULATOR_HPP
#define OM_TRANSLATE_ROTATE_MANIPULATOR_HPP

//
// Description: the object translate/rotate manipulator (the main-view gizmo).
//
// D1.4 (WREN deletion): STATE + MATRIX MATH only. The WREN renderables that used to hang
// off the attached node's transform are gone; OmGizmoLines draws the handles and
// OmScenePicker hit-tests them, both from the matrices this class computes. The matrix
// reproduces what wr_transform_get_matrix() returned for each handle transform:
//     poseWorld * S(unscale) * T(sortOffset[axis]) * R(axisRotation[axis])
// pose chain, manipulator unscale, per-axis rotation and the translucency sort offset all
// included -- which is why the drawn handle cannot drift from the draggable one.
//

#include "OmRotation.hpp"
#include "OmVector3.hpp"
#include "OmWrenAbstractManipulator.hpp"

class OmAbstractPose;

class OmTranslateRotateManipulator : public OmWrenAbstractManipulator {
  Q_OBJECT

public:
  OmTranslateRotateManipulator(bool isTranslationAvailable, bool isRotationAvailable);
  virtual ~OmTranslateRotateManipulator() override;

  // Attachment: the pose whose matrix() anchors the gizmo.
  void attachTo(const OmAbstractPose *pose);
  const OmAbstractPose *attachedPose() const { return mAttachedPose; }

  void showRotationLine(bool show);

  // Visibility
  void highlightAxis(int index) override;
  void showNormal() override;

  // Others
  void updateRotationLine(const OmVector3 &begin, const OmVector3 &end, const OmRotation &orientation, float arrowScale);
  int coordinate(int handleNumber) const override { return handleNumber % 3; }
  int coordinateToHandleNumber(int coord) override { return coord; };
  const OmVector3 &coordinateVector(int handleNumber) const override { return STANDARD_COORDINATE_VECTORS[handleNumber % 3]; }
  OmVector3 relativeHandlePosition(int handleNumber) const override;

  bool hasTranslationHandles() const { return mHasTranslationHandles; }
  bool hasRotationHandles() const { return mHasRotationHandles; }

  // Per-handle visibility (WREN parity: during a drag only the highlighted axis' handle is
  // visible; showNormal() restores all of them). OmGizmoLines skips hidden handles.
  bool isTranslationHandleVisible(int axis) const {
    return mHasTranslationHandles && axis >= 0 && axis < 3 && mTranslationHandleVisible[axis];
  }
  bool isRotationHandleVisible(int axis) const {
    return mHasRotationHandles && axis >= 0 && axis < 3 && mRotationHandleVisible[axis];
  }

  // World matrices, column-major float16 -- exactly what wr_transform_get_matrix() used to
  // return for the corresponding handle transform. Return false when the handle set does not
  // exist or no pose is attached.
  bool translationHandleMatrix(int axis, float out16[16]) const;
  bool rotationHandleMatrix(int axis, float out16[16]) const;
  bool axesMatrix(float out16[16]) const;

private:
  // Utility constants
  static const OmVector3 STANDARD_COORDINATE_VECTORS[3];

  bool handleMatrix(int axis, float out16[16]) const;

  bool mHasRotationHandles;
  bool mHasTranslationHandles;

  const OmAbstractPose *mAttachedPose;

  bool mTranslationHandleVisible[3];
  bool mRotationHandleVisible[3];
  bool mInfiniteAxisVisible[3];

  // Mid-drag rotation-line state (begin/end/orientation/arrowScale). Stored for a future
  // overlay drawer; the WREN line + double-arrow renderables are gone (P8's open leftover).
  bool mRotationLineVisible;
  OmVector3 mRotationLineBegin;
  OmVector3 mRotationLineEnd;
};

#endif  // OM_TRANSLATE_ROTATE_MANIPULATOR_HPP
