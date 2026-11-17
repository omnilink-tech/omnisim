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

#include "OmDragResizeEvent.hpp"

#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmCone.hpp"
#include "OmCylinder.hpp"
#include "OmElevationGrid.hpp"
#include "OmGeometry.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmMatrix4.hpp"
#include "OmPlane.hpp"
#include "OmResizeAndTranslateCommand.hpp"
#include "OmResizeCommand.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSphere.hpp"
#include "OmUndoStack.hpp"
#include "OmViewpoint.hpp"
#include "OmWrenRenderingContext.hpp"

// Moves a resize handle by dragging the mouse and changes the geometry size accordingly //
///////////////////////////////////////////////////////////////////////////////////////////

OmDragResizeHandleEvent::OmDragResizeHandleEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                                 OmGeometry *selectedGeometry) :
  OmDragView3DEvent(viewpoint),
  mInitialMousePosition(initialMousePosition),
  mSelectedGeometry(selectedGeometry),
  mHandleNumber(handleNumber),
  mManipulator(selectedGeometry->resizeManipulator()),
  mResizeRatio(1.0),
  mTotalScaleRatio(1.0),
  mGeomCenterOffset(0.0),
  mSizeValue(0.0) {
  mCoordinate = handleNumber;
  mManipulator->highlightAxis(mManipulator->coordinate(mHandleNumber));
  mManipulator->setActive(true);
  mViewDistanceUnscaling = mViewpoint->viewDistanceUnscaling(selectedGeometry->matrix().translation());
  mSizeValue = mViewDistanceUnscaling * mManipulator->relativeHandlePosition(mHandleNumber)[mCoordinate];
  const OmVector3 mouse3dPosition = computeLocalMousePosition(initialMousePosition);
  mMouseOffset = mouse3dPosition[mCoordinate] - mSizeValue;
  mViewpoint->lock();
}

OmDragResizeHandleEvent::~OmDragResizeHandleEvent() {
  mManipulator->setActive(false);
  mManipulator->showNormal();
  mManipulator->updateHandleDimensions(1.0f, 1.0f);

  mViewpoint->unlock();
}

OmVector3 OmDragResizeHandleEvent::computeLocalMousePosition(const QPoint &currentMousePosition) {
  const OmMatrix4 &matrix = mSelectedGeometry->matrix();

  OmMatrix3 unscaledMatrix = matrix.extracted3x3Matrix();
  const OmVector3 &scale = mSelectedGeometry->absoluteScale();
  unscaledMatrix.scale(1.0f / scale.x(), 1.0f / scale.y(), 1.0f / scale.z());

  OmVector3 attachedHandlePosition(mTotalScaleRatio * mViewDistanceUnscaling *
                                   mManipulator->relativeHandlePosition(mHandleNumber));
  attachedHandlePosition = unscaledMatrix * attachedHandlePosition;

  const float zEye = mViewpoint->zEye(attachedHandlePosition);
  OmVector3 localMousePosition = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), zEye);
  localMousePosition = matrix.pseudoInversed(localMousePosition);
  localMousePosition /= scale;
  return localMousePosition;
}

void OmDragResizeHandleEvent::computeRatio(const QPoint &currentMousePosition) {
  OmVector3 localMousePosition = computeLocalMousePosition(currentMousePosition);
  const double newSizeValue = localMousePosition[mCoordinate] - mMouseOffset;
  mResizeRatio = newSizeValue / mSizeValue;

  if (abs(mResizeRatio) <= 0.01) {
    mResizeRatio = mResizeRatio < 0.0 ? -1.0 : 1.0;
    return;
  }

  mSizeValue = newSizeValue;
}

// Regular resize event: sphere, box, cylinder, capsule and cone

OmRegularResizeEvent::OmRegularResizeEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                           OmGeometry *selectedGeometry) :
  OmDragResizeHandleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

// Drag resizing the sphere

OmResizeSphereEvent::OmResizeSphereEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                         OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mSphere(static_cast<OmSphere *>(selectedGeometry)) {
}

void OmResizeSphereEvent::addActionInUndoStack() {
  OmUndoStack::instance()->push(new OmResizeCommand(mSphere, OmVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void OmResizeSphereEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double currentRadius = mSphere->radius() * mResizeRatio;

  if (exceedsFloatMax(currentRadius)) {
    emit aborted();
    return;
  }

  mSphere->setRadius(currentRadius);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the cylinder

OmResizeCylinderEvent::OmResizeCylinderEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                             OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCylinder(static_cast<OmCylinder *>(selectedGeometry)) {
}

void OmResizeCylinderEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mCylinder, scale));
}

void OmResizeCylinderEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  if (mCoordinate != Z) {
    // Resizing the radius
    const double resizedRadius = mCylinder->radius() * mResizeRatio;

    if (exceedsFloatMax(resizedRadius)) {
      emit aborted();
      return;
    }

    mCylinder->setRadius(resizedRadius);
  } else {
    // Resizing the height
    const double resizedHeight = mCylinder->height() * mResizeRatio;

    if (exceedsFloatMax(resizedHeight)) {
      emit aborted();
      return;
    }

    mCylinder->setHeight(resizedHeight);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the capsule

OmResizeCapsuleEvent::OmResizeCapsuleEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                           OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCapsule(static_cast<OmCapsule *>(selectedGeometry)) {
}

void OmResizeCapsuleEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mCapsule, scale));
}

void OmResizeCapsuleEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  if (mCoordinate != Z) {
    // Resizing the radius
    const double resizedRadius = mCapsule->radius() * mResizeRatio;

    if (exceedsFloatMax(resizedRadius)) {
      emit aborted();
      return;
    }

    mCapsule->setRadius(resizedRadius);
  } else {
    // Resizing the height
    const float resizedHeight = mCapsule->height() * mResizeRatio;

    if (exceedsFloatMax(resizedHeight)) {
      emit aborted();
      return;
    }

    mCapsule->setHeight(resizedHeight);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the box

OmResizeBoxEvent::OmResizeBoxEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                   OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mBox(static_cast<OmBox *>(selectedGeometry)) {
}

void OmResizeBoxEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mBox, scale));
}

void OmResizeBoxEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const OmVector3 &size = mBox->size();
  double currentValue;
  switch (mCoordinate) {
    case X:
      currentValue = size.x() * mResizeRatio;

      if (exceedsFloatMax(currentValue)) {
        emit aborted();
        return;
      }

      mBox->setX(currentValue);
      break;
    case Y:
      currentValue = size.y() * mResizeRatio;

      if (exceedsFloatMax(currentValue)) {
        emit aborted();
        return;
      }

      mBox->setY(currentValue);
      break;
    case Z:
      currentValue = size.z() * mResizeRatio;

      if (exceedsFloatMax(currentValue)) {
        emit aborted();
        return;
      }

      mBox->setZ(currentValue);
      break;
    default:
      assert(0);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the plane

OmResizePlaneEvent::OmResizePlaneEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                       OmGeometry *selectedGeometry) :
  OmDragResizeHandleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mPlane(static_cast<OmPlane *>(selectedGeometry)) {
}

void OmResizePlaneEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mPlane, scale));
}

void OmResizePlaneEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const OmVector2 &size = mPlane->size();
  if (mCoordinate == X) {
    double currentX = size.x() * mResizeRatio;

    if (exceedsFloatMax(currentX)) {
      emit aborted();
      return;
    }

    mPlane->setX(currentX);
  } else {
    double currentZ = size.y() * mResizeRatio;

    if (exceedsFloatMax(currentZ)) {
      emit aborted();
      return;
    }

    mPlane->setY(currentZ);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the cone

OmResizeConeEvent::OmResizeConeEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, int handleNumber,
                                     OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCone(static_cast<OmCone *>(selectedGeometry)) {
}

void OmResizeConeEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mCone, scale));
}

void OmResizeConeEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  if (mCoordinate != Z) {
    // Resizing the radius
    const double resizedBottomRadius = mCone->bottomRadius() * mResizeRatio;

    if (exceedsFloatMax(resizedBottomRadius)) {
      emit aborted();
      return;
    }

    mCone->setBottomRadius(resizedBottomRadius);
  } else {
    // Resizing the height
    const double resizedHeight = mCone->height() * mResizeRatio;

    if (exceedsFloatMax(resizedHeight)) {
      emit aborted();
      return;
    }

    mCone->setHeight(resizedHeight);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the elevation grid

OmResizeElevationGridEvent::OmResizeElevationGridEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint,
                                                       int handleNumber, OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mElevationGrid(static_cast<OmElevationGrid *>(selectedGeometry)) {
}

void OmResizeElevationGridEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mElevationGrid, scale));
}

void OmResizeElevationGridEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  switch (mCoordinate) {
    case X: {
      const double resizedXspacing = mElevationGrid->xSpacing() * mResizeRatio;

      if (exceedsFloatMax(resizedXspacing)) {
        emit aborted();
        return;
      }

      mElevationGrid->setXspacing(resizedXspacing);
      break;
    }
    case Y: {
      const double resizedYspacing = mElevationGrid->ySpacing() * mResizeRatio;
      if (exceedsFloatMax(resizedYspacing)) {
        emit aborted();
        return;
      }
      mElevationGrid->setYspacing(resizedYspacing);
      break;
    }
    case Z:
      if (exceedsFloatMax(mResizeRatio * mElevationGrid->heightRange())) {
        emit aborted();
        return;
      }
      mElevationGrid->setHeightScaleFactor(mResizeRatio);
      break;
    default:
      assert(false);
  }

  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the indexed face set

OmResizeIndexedFaceSetEvent::OmResizeIndexedFaceSetEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint,
                                                         int handleNumber, OmGeometry *selectedGeometry) :
  OmRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mIndexedFaceSet(static_cast<OmIndexedFaceSet *>(selectedGeometry)) {
  mSizeValue = mViewDistanceUnscaling * mManipulator->relativeHandlePosition(mHandleNumber)[mCoordinate];
}

void OmResizeIndexedFaceSetEvent::addActionInUndoStack() {
  OmVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  OmUndoStack::instance()->push(new OmResizeCommand(mIndexedFaceSet, scale));
}

void OmResizeIndexedFaceSetEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  if (exceedsFloatMax(mResizeRatio * mIndexedFaceSet->range(mCoordinate))) {
    emit aborted();
    return;
  }

  OmVector3 scale(1.0f, 1.0f, 1.0f);
  scale[mCoordinate] = mResizeRatio;
  mIndexedFaceSet->rescale(scale);

  // update global resize values
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}
