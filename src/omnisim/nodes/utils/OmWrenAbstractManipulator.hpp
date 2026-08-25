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

#ifndef OM_WREN_ABSTRACT_MANIPULATOR_HPP
#define OM_WREN_ABSTRACT_MANIPULATOR_HPP

//
// Description: abstract class implementing basic function for a manipulator
//              that acts on 3 dimensions.
//
// D1.4 (WREN deletion): this class family is STATE-ONLY now. It used to own the
// WREN renderables/transforms for the gizmo handles; the drawing moved to
// OmGizmoLines (wgpu overlay lines) and the hit test to OmScenePicker (wgpu ID
// pick), both of which READ this state. What survives here is exactly what those
// two and the drag events consume: attachment, activity, the handle screen scale,
// per-axis highlight/visibility, and the coordinate/axis bookkeeping.
//

#include <QtCore/QObject>
#include <QtCore/QVector>

class OmVector3;

class OmWrenAbstractManipulator : public QObject {
  Q_OBJECT

public:
  virtual ~OmWrenAbstractManipulator();

  // Getters
  bool isAttached() const { return mIsVisible; }
  bool isActive() const { return mIsActive; }
  // The `screenScale` scalar the WREN handles shader used to receive: what decides how big
  // one object-space unit of handle geometry is on screen. OmGizmoLines and OmScenePicker
  // both read it, so the drawn gizmo and the hit test cannot diverge.
  float handleScreenScale() const { return mScale; }
  // The manipulator "unscale" (1 / the attached transform's absolute scale), applied so
  // handles keep their metric size under a scaled Transform. Identity until
  // updateHandleScale() is called.
  const float *handleUnscale() const { return mUnscale; }

  // Setters
  void setActive(bool b) { mIsActive = b; }
  void updateHandleScale(const double *scale);

  // Visibility
  virtual void show();
  virtual void highlightAxis(int index) {}
  virtual void showNormal();

  // Others
  virtual int coordinate(int handleNumber) const = 0;
  virtual int coordinateToHandleNumber(int coord) = 0;
  virtual const OmVector3 &coordinateVector(int handleNumber) const = 0;
  virtual OmVector3 relativeHandlePosition(int handleNumber) const = 0;

  void computeHandleScaleFromViewportSize();

public slots:
  virtual void hide();

protected:
  explicit OmWrenAbstractManipulator(int numberOfHandles);

  // Mark this manipulator attached (the caller decides what it is attached to; the concrete
  // subclasses that need a matrix store their own attachment target).
  void markAttached() { mIsVisible = true; }

  bool mIsVisible;
  bool mOriginScaleFactorNeeded;
  float mScale;
  float mUnscale[3];
  int mNumberOfHandles;

private:
  OmWrenAbstractManipulator(const OmWrenAbstractManipulator &original);
  OmWrenAbstractManipulator &operator=(const OmWrenAbstractManipulator &original);

  bool mIsActive;
};

#endif  // OM_WREN_ABSTRACT_MANIPULATOR_HPP
