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

#ifndef OM_WREN_ABSTRACT_RESIZE_MANIPULATOR_HPP
#define OM_WREN_ABSTRACT_RESIZE_MANIPULATOR_HPP

// D1.4 (WREN deletion): STATE-ONLY. The resize/scale handle renderables died with WREN and,
// unlike the translate/rotate gizmo (drawn by OmGizmoLines, picked by OmScenePicker), they
// have no wgpu drawer or hit test yet -- under the wgpu main view they were already invisible
// (P8's documented leftover), and they are now unpickable too. The class survives because the
// geometry nodes, drag events and the scene-tree editor drive its state, and a future drawer
// draws from this state. Resizing via the scene-tree fields is unaffected.

#include "OmVector3.hpp"
#include "OmWrenAbstractManipulator.hpp"

class OmWrenAbstractResizeManipulator : public OmWrenAbstractManipulator {
  Q_OBJECT

public:
  enum ResizeConstraint { NO_CONSTRAINT, UNIFORM, X_EQUAL_Y };

  virtual ~OmWrenAbstractResizeManipulator() override;

  // Attachment (the geometry marks it attached; there is no render parent any more).
  void attachTo() { markAttached(); }

  // Setters
  virtual void setResizeConstraint(ResizeConstraint constraint);

  ResizeConstraint resizeConstraint() { return mConstraint; }

  // Visibility
  void highlightAxis(int index) override;
  void showNormal() override;

  // Utility constants
  static const int STANDARD_COORDINATES[3];
  static const OmVector3 STANDARD_COORDINATE_VECTORS[3];

  // Others
  int coordinate(int handleNumber) const override { return STANDARD_COORDINATES[handleNumber]; }
  int coordinateToHandleNumber(int coord) override { return coord; };
  const OmVector3 &coordinateVector(int handleNumber) const override { return STANDARD_COORDINATE_VECTORS[handleNumber]; }
  OmVector3 relativeHandlePosition(int handleNumber) const override {
    return mScale * STANDARD_COORDINATE_VECTORS[coordinate(handleNumber)];
  }
  void updateHandleDimensions(const float scaleFactor, const float viewDistanceScale);

protected:
  explicit OmWrenAbstractResizeManipulator(ResizeConstraint constraint);

  enum { X, Y, Z };

  ResizeConstraint mConstraint;
  int mUniformMaterialIndex;

private:
  OmWrenAbstractResizeManipulator(const OmWrenAbstractResizeManipulator &original);
  OmWrenAbstractResizeManipulator &operator=(const OmWrenAbstractResizeManipulator &original);
};

#endif  // OM_WREN_ABSTRACT_RESIZE_MANIPULATOR_HPP
