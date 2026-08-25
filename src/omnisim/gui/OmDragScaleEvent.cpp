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

#include "OmDragScaleEvent.hpp"

#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmCone.hpp"
#include "OmCylinder.hpp"
#include "OmEditCommand.hpp"
#include "OmElevationGrid.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmPlane.hpp"
#include "OmResizeAndTranslateCommand.hpp"
#include "OmResizeCommand.hpp"
#include "OmResizeManipulator.hpp"
#include "OmUndoStack.hpp"
#include "OmViewpoint.hpp"
#include "OmWrenRenderingContext.hpp"

// Drags scaling the cylinder

OmRescaleCylinderEvent::OmRescaleCylinderEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                               OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCylinder(static_cast<OmCylinder *>(selectedGeometry)) {
}

void OmRescaleCylinderEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(
    new OmResizeCommand(mCylinder, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescaleCylinderEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double resizedRadius = mCylinder->radius() * mResizeRatio;

  if (exceedsFloatMax(resizedRadius)) {
    emit aborted();
    return;
  }

  mCylinder->setRadius(resizedRadius);

  const double resizedHeight = mCylinder->height() * mResizeRatio;

  if (exceedsFloatMax(resizedHeight)) {
    emit aborted();
    return;
  }

  mCylinder->setHeight(resizedHeight);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the capsule

OmRescaleCapsuleEvent::OmRescaleCapsuleEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                             OmGeometry *selectedGeometry) :
  OmResizeCapsuleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void OmRescaleCapsuleEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(new OmResizeCommand(mCapsule, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescaleCapsuleEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double resizedRadius = mCapsule->radius() * mResizeRatio;

  if (exceedsFloatMax(resizedRadius)) {
    emit aborted();
    return;
  }

  mCapsule->setRadius(resizedRadius);

  const double resizedHeight = mCapsule->height() * mResizeRatio;

  if (exceedsFloatMax(resizedHeight)) {
    emit aborted();
    return;
  }

  mCapsule->setHeight(resizedHeight);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the box

OmRescaleBoxEvent::OmRescaleBoxEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                     OmGeometry *selectedGeometry) :
  OmResizeBoxEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void OmRescaleBoxEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(new OmResizeCommand(mBox, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescaleBoxEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const OmVector3 &rescaledSize = mResizeRatio * mBox->size();
  mBox->setSize(rescaledSize);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the plane

OmRescalePlaneEvent::OmRescalePlaneEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                         OmGeometry *selectedGeometry) :
  OmResizePlaneEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void OmRescalePlaneEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(new OmResizeCommand(mPlane, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescalePlaneEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const OmVector2 &rescaledSize = mResizeRatio * mPlane->size();

  if (exceedsFloatMax(rescaledSize.x()) || exceedsFloatMax(rescaledSize.y())) {
    emit aborted();
    return;
  }

  mPlane->setSize(rescaledSize);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the cone

OmRescaleConeEvent::OmRescaleConeEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                       OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCone(static_cast<OmCone *>(selectedGeometry)) {
}

void OmRescaleConeEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(new OmResizeCommand(mCone, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescaleConeEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double resizedBottomRadius = fabs(mCone->bottomRadius() * mResizeRatio);

  if (exceedsFloatMax(resizedBottomRadius)) {
    emit aborted();
    return;
  }

  mCone->setBottomRadius(resizedBottomRadius);
  const double resizedHeight = fabs(mCone->height() * mResizeRatio);

  if (exceedsFloatMax(resizedHeight)) {
    emit aborted();
    return;
  }

  mCone->setHeight(resizedHeight);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the elevation grid

OmRescaleElevationGridEvent::OmRescaleElevationGridEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint,
                                                         int handleNumber, OmGeometry *selectedGeometry) :
  OmResizeElevationGridEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void OmRescaleElevationGridEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(
    new OmResizeCommand(mElevationGrid, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescaleElevationGridEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  const double resizedXspacing = mElevationGrid->xSpacing() * mResizeRatio;
  if (exceedsFloatMax(resizedXspacing)) {
    emit aborted();
    return;
  }

  mElevationGrid->setXspacing(resizedXspacing);

  const double resizedYspacing = mElevationGrid->ySpacing() * mResizeRatio;
  if (exceedsFloatMax(resizedYspacing)) {
    emit aborted();
    return;
  }

  mElevationGrid->setYspacing(resizedYspacing);

  if (exceedsFloatMax(mResizeRatio * mElevationGrid->heightRange())) {
    emit aborted();
    return;
  }

  mElevationGrid->setHeightScaleFactor(mResizeRatio);

  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the indexed face set

OmRescaleIndexedFaceSetEvent::OmRescaleIndexedFaceSetEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint,
                                                           int handleNumber, OmGeometry *selectedGeometry) :
  OmResizeIndexedFaceSetEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void OmRescaleIndexedFaceSetEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(
    new OmResizeCommand(mIndexedFaceSet, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmRescaleIndexedFaceSetEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  if (exceedsFloatMax(mResizeRatio * mIndexedFaceSet->range(mCoordinate))) {
    emit aborted();
    return;
  }

  mIndexedFaceSet->rescale(OmVector3(mResizeRatio, mResizeRatio, mResizeRatio));

  // update global resize values
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}
