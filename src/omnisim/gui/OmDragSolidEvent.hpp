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

#ifndef OM_DRAG_SOLID_EVENT_HPP
#define OM_DRAG_SOLID_EVENT_HPP

//
// Description: classes allowing to store data related with the solid mouse dragging
//

#include "OmAffinePlane.hpp"
#include "OmDragPoseEvent.hpp"
#include "OmRay.hpp"
#include "OmVector3.hpp"

#include <QtCore/QPointer>
#include <QtCore/QSize>

class OmPhysicsVectorRepresentation;
class OmWrenLabelOverlay;
class OmSolid;
class OmSolidMerger;
class OmViewpoint;

// Special translation and rotation drag event for Solid nodes that reset the physics
///////////////////////////////////////////////////////////////////////////////////////
class OmDragHorizontalSolidEvent : public OmDragHorizontalEvent {
public:
  OmDragHorizontalSolidEvent(const QPoint &initialPosition, OmViewpoint *viewpoint, OmSolid *selectedSolid);
  virtual ~OmDragHorizontalSolidEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmSolid *mSelectedSolid;
};

class OmDragVerticalSolidEvent : public OmDragVerticalEvent {
public:
  OmDragVerticalSolidEvent(const QPoint &initialPosition, OmViewpoint *viewpoint, OmSolid *selectedSolid);
  virtual ~OmDragVerticalSolidEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmSolid *mSelectedSolid;
};

class OmDragTranslateAlongAxisSolidEvent : public OmDragTranslateAlongAxisEvent {
  Q_OBJECT;

public:
  OmDragTranslateAlongAxisSolidEvent(const QPoint &initialMousePosition, const QSize &widgetSize, OmViewpoint *viewpoint,
                                     int handleNumber, OmSolid *selectedSolid);
  virtual ~OmDragTranslateAlongAxisSolidEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmSolid *mSelectedSolid;
};

class OmDragRotateAroundWorldVerticalAxisSolidEvent : public OmDragRotateAroundWorldVerticalAxisEvent {
public:
  OmDragRotateAroundWorldVerticalAxisSolidEvent(const QPoint &initialPosition, OmViewpoint *viewpoint, OmSolid *selectedSolid);
  virtual ~OmDragRotateAroundWorldVerticalAxisSolidEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmSolid *mSelectedSolid;
};

class OmDragRotateAroundAxisSolidEvent : public OmDragRotateAroundAxisEvent {
  Q_OBJECT;

public:
  OmDragRotateAroundAxisSolidEvent(const QPoint &initialMousePosition, const QSize &widgetSize, OmViewpoint *viewpoint,
                                   int handleNumber, OmSolid *selectedSolid);
  virtual ~OmDragRotateAroundAxisSolidEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmSolid *mSelectedSolid;
};

// Abstract class for drag events involving physics changes only: user-defined forces and torques //
///////////////////////////////////////////////////////////////////////////////////////////////////

// OmDragPhysicsEvent class (abstract)
///////////////////////////////////////
class OmDragPhysicsEvent : public OmDragView3DEvent {
  Q_OBJECT;

public:
  OmDragPhysicsEvent(const QSize &widgetSize, OmViewpoint *viewpoint, OmSolid *selectedSolid);
  virtual ~OmDragPhysicsEvent() override;
  void apply(const QPoint &currentMousePosition) override;
  void lock();
  // Accessor
  virtual bool isLocked() const { return mIsLocked; }

  // W4a overlay port (lane E4): the wgpu main view redraws the force/torque arrow through
  // OmDragArrowLines, which must read the SAME state the WREN representation reads
  // (updatePosition(mOrigin, mEnd, ...) / setScale(mViewDistanceScaling)) -- these expose it.
  const OmVector3 &dragOrigin() const { return mOrigin; }
  const OmVector3 &dragEnd() const { return mEnd; }
  float viewDistanceScaling() const { return mViewDistanceScaling; }
  virtual bool isTorqueDrag() const { return false; }

signals:
  void aborted();  // triggers drag destruction in OmView3D

public slots:
  virtual void updateRenderingAndPhysics();

protected:
  void init();
  void applyChangesToWren();
  virtual void applyToOde() = 0;
  virtual void updateOrigin() = 0;
  virtual QString magnitudeString() const = 0;
  OmSolid *mSelectedSolid;
  OmPhysicsVectorRepresentation *mRepresentation;
  OmVector3 mOrigin;
  OmVector3 mEnd;
  OmVector3 mVector;
  double mScalingFactor;
  OmAffinePlane mDragPlane;
  OmRay mMouseRay;
  std::pair<bool, double> mIntersectionOutput;
  bool mIsLocked;
  OmWrenLabelOverlay *mTextOverlay;
  QSize mWidgetSize;
  float mViewDistanceScaling;
};

// OmDragForceEvent class
class OmDragForceEvent : public OmDragPhysicsEvent {
  Q_OBJECT;

public:
  OmDragForceEvent(const QSize &widgetSize, OmViewpoint *viewpoint, OmSolid *selectedSolid);

public slots:
  void applyToOde() override;

private:
  static const double FORCE_SCALING_FACTOR;
  OmVector3 mRelativeOrigin;
  void updateOrigin() override;
  QString magnitudeString() const override;
};

// OmDragTorqueEvent class
class OmDragTorqueEvent : public OmDragPhysicsEvent {
  Q_OBJECT;

public:
  OmDragTorqueEvent(const QSize &widgetSize, OmViewpoint *viewpoint, OmSolid *selectedSolid);
  bool isTorqueDrag() const override { return true; }

public slots:
  void applyToOde() override;

private:
  QPointer<OmSolidMerger> mSolidMerger;
  static const double TORQUE_SCALING_FACTOR;
  void updateOrigin() override;
  QString magnitudeString() const override;
};

#endif
