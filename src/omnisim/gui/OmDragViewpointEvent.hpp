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

#ifndef OM_DRAG_VIEWPOINT_EVENT_HPP
#define OM_DRAG_VIEWPOINT_EVENT_HPP

//
// Description: classes allowing to store data related with viewpoint dragging
//

#include "OmAbstractDragEvent.hpp"
#include "OmVector3.hpp"

#include <QtCore/QPoint>

class OmViewpoint;

// OmDragViewpointEvent class (abstract) : change camera's position or orientation
///////////////////////////////////////////////////////////////////////////////////
class OmDragViewpointEvent : public OmDragKinematicsEvent {
public:
  virtual ~OmDragViewpointEvent() override {}
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  explicit OmDragViewpointEvent(OmViewpoint *viewpoint);
};

// Implemented classes:

// OmTranslateViewpointEvent class
class OmTranslateViewpointEvent : public OmDragViewpointEvent {
public:
  OmTranslateViewpointEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, double scale);
  virtual ~OmTranslateViewpointEvent() override;
  void apply(const QPoint &currentMousePosition) override;

private:
  const QPoint mInitialMousePosition;
  QPoint mDifference;
  const OmVector3 mInitialCameraPosition;
  const double mScaleFactor;
};

// OmRotateViewpointEvent class
class OmRotateViewpointEvent : public OmDragViewpointEvent {
public:
  OmRotateViewpointEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, bool objectPicked);
  virtual ~OmRotateViewpointEvent() override;
  void apply(const QPoint &currentMousePosition) override;

  static void applyToViewpoint(const QPoint &delta, const OmVector3 &rotationCenter, const OmVector3 &worldUpVector,
                               bool objectPicked, OmViewpoint *viewpoint);

private:
  QPoint mPreviousMousePosition;
  QPoint mDelta;
  const OmVector3 mWorldUpVector;
  bool mIsObjectPicked;
};

// OmZoomAndRotateViewpointEvent class
class OmZoomAndRotateViewpointEvent : public OmDragViewpointEvent {
public:
  OmZoomAndRotateViewpointEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, const double scale);
  virtual ~OmZoomAndRotateViewpointEvent() override;
  void apply(const QPoint &currentMousePosition) override;

  static void applyToViewpoint(double tiltAngle, double zoom, double scaleFactor, OmViewpoint *viewpoint);

private:
  QPoint mPreviousMousePosition;
  QPoint mDelta;
  const double mZscaleFactor;
};

#endif
