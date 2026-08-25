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

#ifndef OM_DRAG_POSE_EVENT_HPP
#define OM_DRAG_POSE_EVENT_HPP

//
// Description: classes allowing to store data related with the mouse dragging of a Pose node
//

#include "OmAbstractDragEvent.hpp"

#include "OmAffinePlane.hpp"
#include "OmMatrix4.hpp"
#include "OmQuaternion.hpp"
#include "OmRay.hpp"
#include "OmVector2.hpp"
#include "OmVector3.hpp"

#include <QtCore/QSize>

class OmAbstractPose;
class OmWrenLabelOverlay;
class OmTranslateRotateManipulator;
class OmViewpoint;

// OmDragPoseEvent class (abstract) : change the position or the orientation of a Pose node
class OmDragPoseEvent : public OmDragKinematicsEvent {
public:
  OmDragPoseEvent(OmViewpoint *viewpoint, OmAbstractPose *selectedPose);
  virtual ~OmDragPoseEvent() override;
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  OmAbstractPose *mSelectedPose;
};

// another abstract layer:
class OmTranslateEvent : public OmDragPoseEvent {
public:
  OmTranslateEvent(OmViewpoint *viewpoint, OmAbstractPose *selectedPose);
  virtual ~OmTranslateEvent() override;
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  OmVector3 mScaleFromParents;
  const OmVector3 mInitialPosition;
  const OmVector3 mUpWorldVector;
  OmAffinePlane mDragPlane;
  OmRay mMouseRay;
  std::pair<bool, double> mIntersectionOutput;
};

// Implemented classes:

// OmDragHorizontalEvent class
class OmDragHorizontalEvent : public OmTranslateEvent {
public:
  OmDragHorizontalEvent(const QPoint &initialPosition, OmViewpoint *viewpoint, OmAbstractPose *selectedPose);
  virtual ~OmDragHorizontalEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmVector3 mTranslationOffset;
  OmQuaternion mCoordinateTransform;
  bool mIsMouseRayValid;
};

// OmDragVerticalEvent class
class OmDragVerticalEvent : public OmTranslateEvent {
public:
  OmDragVerticalEvent(const QPoint &initialPosition, OmViewpoint *viewpoint, OmAbstractPose *selectedPose);
  virtual ~OmDragVerticalEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  OmVector3 mNormal;
  OmVector3 mTranslationOffset;
};

// OmDragTranslateAlongAxisEvent class
class OmDragTranslateAlongAxisEvent : public OmDragPoseEvent {
  Q_OBJECT;

public:
  OmDragTranslateAlongAxisEvent(const QPoint &initialMousePosition, const QSize &widgetSize, OmViewpoint *viewpoint,
                                int handleNumber, OmAbstractPose *selectedPose);
  virtual ~OmDragTranslateAlongAxisEvent() override;
  void apply(const QPoint &currentMousePosition) override;

protected:
  const OmVector3 mInitialMatterPosition;
  double mTranslationOffset;
  int mHandleNumber;
  OmTranslateRotateManipulator *mManipulator;
  OmWrenLabelOverlay *mTextOverlay;

  enum { X, Y, Z };
  int mCoordinate;
  double mMouseOffset;
  OmVector3 mHandleOffset;
  OmVector2 mWidgetSizeFactor;
  double mStepSize;
  OmVector2 mDirectionOnScreen;
  double mAbsoluteScale;
};

class OmDragRotateAroundWorldVerticalAxisEvent : public OmDragPoseEvent {
  Q_OBJECT;

public:
  OmDragRotateAroundWorldVerticalAxisEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint,
                                           OmAbstractPose *selectedPose);
  virtual ~OmDragRotateAroundWorldVerticalAxisEvent() override;
  void apply(const QPoint &currentMousePosition) override;

protected:
  const OmQuaternion mInitialQuaternionRotation;
  double mPreviousAngle;
  double mInitialMouseXPosition;
  const OmVector3 mUpWorldVector;
};

// OmDragRotateAroundAxisEvent class
class OmDragRotateAroundAxisEvent : public OmDragPoseEvent {
  Q_OBJECT;

public:
  OmDragRotateAroundAxisEvent(const QPoint &initialMousePosition, const QSize &widgetSize, OmViewpoint *viewpoint,
                              int handleNumber, OmAbstractPose *selectedPose);
  virtual ~OmDragRotateAroundAxisEvent() override;
  void apply(const QPoint &currentMousePosition) override;

protected:
  OmTranslateRotateManipulator *mManipulator;
  OmWrenLabelOverlay *mTextOverlay;

  int mHandleNumber;
  int mCoordinate;
  const OmQuaternion mInitialQuaternionRotation;
  OmMatrix4 mInitialMatrix;
  OmVector3 mInitialPosition;
  double mZEye;
  double mStepSize;
  int mStepFractionNumerator;
  int mStepFractionDenominator;
  double mPreviousAngle;
  double mInitialAngle;
  OmVector2 mObjectScreenPosition;
  static const double RAD_TO_DEG;
};

#endif
