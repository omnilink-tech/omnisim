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

#ifndef OM_ABSTRACT_DRAG_EVENT_HPP
#define OM_ABSTRACT_DRAG_EVENT_HPP

//
// Description: abstract classes allowing to store data related with the different kind of mouse dragging
//              (so far used only for mouse dragging impacting the 3D view, i.e. OmDragViewpointEvent)
//

#include <QtCore/QObject>

#include <QtCore/QPoint>

class OmVector2;
class OmVector3;
class OmWrenLabelOverlay;
class OmViewpoint;

// OmDragEvent class
class OmDragEvent : public QObject {
  Q_OBJECT;

public:
  virtual ~OmDragEvent() {}
  virtual void apply(const QPoint &currentMousePosition) = 0;

  // test numerical limits supported by bounding boxes
  static const float cFloatMax;
  static bool exceedsFloatMax(const OmVector3 &v);
  static bool exceedsFloatMax(double x);
  static bool exceedsFloatMax(float x);

protected:
  OmDragEvent();
};

// Abstract class for drag events impacting the 3D-View //
//////////////////////////////////////////////////////////

// OmDragView3DEvent class
class OmDragView3DEvent : public OmDragEvent {
public:
  virtual ~OmDragView3DEvent() override {}
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  explicit OmDragView3DEvent(OmViewpoint *viewpoint);

  static OmVector2 clampLabelPosition(const float x, const float y, const OmWrenLabelOverlay *overlay);

  OmViewpoint *mViewpoint;
  float mViewDistanceUnscaling;
};

// Abstract class for drag events involving kinematic changes only: camera moves, non-physical solid or fluid displacement //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// OmDragKinematicsEvent class (abstract)
/////////////////////////////////////////
class OmDragKinematicsEvent : public OmDragView3DEvent {
public:
  virtual ~OmDragKinematicsEvent() override {}
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  explicit OmDragKinematicsEvent(OmViewpoint *viewpoint);
};

#endif
