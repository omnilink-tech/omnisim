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

#ifndef OM_DRAG_SCALE_EVENT_HPP
#define OM_DRAG_SCALE_EVENT_HPP

//
// Description: classes allowing to store data related with rescale mouse dragging
//

#include "OmDragResizeEvent.hpp"
#include "OmVariant.hpp"
#include "OmVector2.hpp"
#include "OmVector3.hpp"

#include <QtCore/QObject>
#include <QtCore/QPoint>

class OmTransform;
class OmCone;
class OmCylinder;
class OmGeometry;
class OmScaleManipulator;
class OmViewpoint;

// Scale Cylinder
class OmRescaleCylinderEvent : public OmRegularResizeEvent {
public:
  OmRescaleCylinderEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                         OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  OmCylinder *mCylinder;
};

// Scale Capsule
class OmRescaleCapsuleEvent : public OmResizeCapsuleEvent {
public:
  OmRescaleCapsuleEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                        OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale Box
class OmRescaleBoxEvent : public OmResizeBoxEvent {
public:
  OmRescaleBoxEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber, OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale Plane
class OmRescalePlaneEvent : public OmResizePlaneEvent {
public:
  OmRescalePlaneEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                      OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale Cone
class OmRescaleConeEvent : public OmRegularResizeEvent {
public:
  OmRescaleConeEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                     OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  OmCone *mCone;
};

// Scale ElevationGrid
class OmRescaleElevationGridEvent : public OmResizeElevationGridEvent {
public:
  OmRescaleElevationGridEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                              OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale IndexedFaceSet
class OmRescaleIndexedFaceSetEvent : public OmResizeIndexedFaceSetEvent {
public:
  OmRescaleIndexedFaceSetEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                               OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

#endif
