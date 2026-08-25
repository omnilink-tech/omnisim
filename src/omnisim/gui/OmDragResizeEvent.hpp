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

#ifndef OM_DRAG_RESIZE_EVENT_HPP
#define OM_DRAG_RESIZE_EVENT_HPP

//
// Description: classes allowing to store data related with resize mouse dragging
//
// This class allows geometries fields to be modified individually,
// whereas OmDragScaleEvent scales all the fields at the same time (uniform scale).
//

#include "OmAbstractDragEvent.hpp"

#include <QtCore/QPoint>

class OmBox;
class OmCapsule;
class OmCone;
class OmCylinder;
class OmElevationGrid;
class OmIndexedFaceSet;
class OmGeometry;
class OmWrenAbstractResizeManipulator;
class OmPlane;
class OmSphere;
class OmVector3;

// OmDragResizeHandleEvent class (abstract) //
//////////////////////////////////////////////
class OmDragResizeHandleEvent : public OmDragView3DEvent {
  Q_OBJECT;

public:
  OmDragResizeHandleEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                          OmGeometry *selectedGeometry);
  virtual ~OmDragResizeHandleEvent() override;
  void apply(const QPoint &currentMousePosition) override = 0;
  virtual void addActionInUndoStack() = 0;

signals:
  void aborted();  // triggers drag destruction in OmView3D

protected:
  QPoint mInitialMousePosition;
  OmGeometry *mSelectedGeometry;
  int mHandleNumber;
  OmWrenAbstractResizeManipulator *mManipulator;
  double mResizeRatio;
  double mTotalScaleRatio;
  double mMouseOffset;
  double mGeomCenterOffset;
  double mSizeValue;
  int mCoordinate;
  enum { X, Y, Z };

  void computeRatio(const QPoint &currentMousePosition);
  OmVector3 computeLocalMousePosition(const QPoint &currentMousePosition);
  double sizeValue() const { return mSizeValue; }
};

// OmRegularResizeEvent class (another abstract layer) : resize spheres, boxes, cylinders, capsules and cones
class OmRegularResizeEvent : public OmDragResizeHandleEvent {
public:
  OmRegularResizeEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                       OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override = 0;
};

// Resize Sphere
class OmResizeSphereEvent : public OmRegularResizeEvent {
public:
  OmResizeSphereEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                      OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  OmSphere *mSphere;
};

// Resize Cylinder
class OmResizeCylinderEvent : public OmRegularResizeEvent {
public:
  OmResizeCylinderEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                        OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  OmCylinder *mCylinder;
};

// Resize Capsule
class OmResizeCapsuleEvent : public OmRegularResizeEvent {
public:
  OmResizeCapsuleEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                       OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  OmCapsule *mCapsule;
};

// Resize Box
class OmResizeBoxEvent : public OmRegularResizeEvent {
public:
  OmResizeBoxEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber, OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  OmBox *mBox;
};

// Resize Plane
class OmResizePlaneEvent : public OmDragResizeHandleEvent {
public:
  OmResizePlaneEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                     OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  OmPlane *mPlane;
};

// Resize Cone
class OmResizeConeEvent : public OmRegularResizeEvent {
public:
  OmResizeConeEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber, OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  OmCone *mCone;
};

// Resize ElevationGrid
class OmResizeElevationGridEvent : public OmRegularResizeEvent {
public:
  OmResizeElevationGridEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                             OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  OmElevationGrid *mElevationGrid;
};

// Resize IndexedFaceSet
class OmResizeIndexedFaceSetEvent : public OmRegularResizeEvent {
public:
  OmResizeIndexedFaceSetEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                              OmGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  OmIndexedFaceSet *mIndexedFaceSet;
};

#endif
