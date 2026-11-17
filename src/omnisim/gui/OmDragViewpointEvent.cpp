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

#include "OmDragViewpointEvent.hpp"

#include "OmQuaternion.hpp"
#include "OmSFRotation.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"

// OmDragViewpointEvent constructor
OmDragViewpointEvent::OmDragViewpointEvent(OmViewpoint *viewpoint) : OmDragKinematicsEvent(viewpoint) {
}

// Translate viewpoint //
/////////////////////////

// OmTranslateViewpointEvent functions

OmTranslateViewpointEvent::OmTranslateViewpointEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, double scale) :
  OmDragViewpointEvent(viewpoint),
  mInitialMousePosition(initialMousePosition),
  mInitialCameraPosition(viewpoint->position()->value()),
  mScaleFactor(scale) {
}

OmTranslateViewpointEvent::~OmTranslateViewpointEvent() {
  if (mInitialCameraPosition != mViewpoint->position()->value())
    OmWorld::instance()->setModified();
}

void OmTranslateViewpointEvent::apply(const QPoint &currentMousePosition) {
  if (mViewpoint->isLocked())
    return;
  mDifference = currentMousePosition - mInitialMousePosition;
  const double targetRight = mScaleFactor * mDifference.x();
  const double targetUp = mScaleFactor * mDifference.y();
  const OmRotation &orientation = mViewpoint->orientation()->value();
  const OmVector3 target = targetRight * orientation.right() + targetUp * orientation.up();
  mViewpoint->position()->setValue(mInitialCameraPosition + target);
}

// Rotate Viewpoint //
//////////////////////

// OmRotateViewpointEvent functions

OmRotateViewpointEvent::OmRotateViewpointEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint, bool objectPicked) :
  OmDragViewpointEvent(viewpoint),
  mPreviousMousePosition(initialMousePosition),
  mDelta(),
  mWorldUpVector(OmWorld::instance()->worldInfo()->upVector()),
  mIsObjectPicked(objectPicked) {
  mViewpoint->lockRotationCenter();
}

OmRotateViewpointEvent::~OmRotateViewpointEvent() {
  mViewpoint->unlockRotationCenter();
  if (!mDelta.isNull())
    OmWorld::instance()->setModified();
}

void OmRotateViewpointEvent::apply(const QPoint &currentMousePosition) {
  if (mViewpoint->isLocked())
    return;

  mDelta = currentMousePosition - mPreviousMousePosition;
  mPreviousMousePosition = currentMousePosition;
  applyToViewpoint(mDelta, mViewpoint->rotationCenter(), mWorldUpVector, mIsObjectPicked, mViewpoint);
}

void OmRotateViewpointEvent::applyToViewpoint(const QPoint &delta, const OmVector3 &rotationCenter,
                                              const OmVector3 &worldUpVector, bool objectPicked, OmViewpoint *viewpoint) {
  double halfPitchAngle = 0.005 * delta.y();
  double halfYawAngle = -0.005 * delta.x();
  if (!objectPicked) {
    halfPitchAngle /= -8;
    halfYawAngle /= -8;
  }
  const double sinusYaw = sin(halfYawAngle);
  const double sinusPitch = sin(halfPitchAngle);
  OmSFRotation *orientation = viewpoint->orientation();
  OmSFVector3 *position = viewpoint->position();
  const OmRotation &orientationValue = orientation->value();
  const OmVector3 pitch = orientationValue.right();
  const OmQuaternion pitchRotation(cos(halfPitchAngle), sinusPitch * pitch.x(), sinusPitch * pitch.y(), sinusPitch * pitch.z());
  const OmQuaternion yawRotation(cos(halfYawAngle), sinusYaw * worldUpVector.x(), sinusYaw * worldUpVector.y(),
                                 sinusYaw * worldUpVector.z());
  // Updates camera's position and orientation
  const OmQuaternion deltaRotation(yawRotation * pitchRotation);
  const OmVector3 currentPosition(deltaRotation * (position->value() - rotationCenter) + rotationCenter);
  const OmQuaternion currentOrientation(deltaRotation * orientationValue.toQuaternion());
  position->setValue(currentPosition);
  orientation->setValue(OmRotation(currentOrientation));
}

// Zoom and rotate Viewpoint //
///////////////////////////////

// OmZoomAndRotateViewpointEvent functions

OmZoomAndRotateViewpointEvent::OmZoomAndRotateViewpointEvent(const QPoint &initialMousePosition, OmViewpoint *viewpoint,
                                                             double scale) :
  OmDragViewpointEvent(viewpoint),
  mPreviousMousePosition(initialMousePosition),
  mDelta(),
  mZscaleFactor(scale) {
}

OmZoomAndRotateViewpointEvent::~OmZoomAndRotateViewpointEvent() {
  if (!mDelta.isNull())
    OmWorld::instance()->setModified();
}

void OmZoomAndRotateViewpointEvent::apply(const QPoint &currentMousePosition) {
  if (mViewpoint->isLocked())
    return;

  mDelta = currentMousePosition - mPreviousMousePosition;
  mPreviousMousePosition = currentMousePosition;
  applyToViewpoint(0.01 * mDelta.x(), mDelta.y(), mZscaleFactor, mViewpoint);
}

void OmZoomAndRotateViewpointEvent::applyToViewpoint(double tiltAngle, double zoom, double scaleFactor,
                                                     OmViewpoint *viewpoint) {
  if (viewpoint->projectionMode() == OmWrenRenderingContext::PM_ORTHOGRAPHIC) {
    if (zoom > 0.0)
      viewpoint->incOrthographicViewHeight();
    else
      viewpoint->decOrthographicViewHeight();
  }
  OmSFVector3 *position = viewpoint->position();
  OmSFRotation *orientation = viewpoint->orientation();
  const OmRotation &orientationValue = orientation->value();
  const OmVector3 rollVector = orientationValue.direction();
  const OmVector3 zDisplacement = (scaleFactor * zoom) * rollVector;
  const OmQuaternion roll(rollVector, tiltAngle);
  position->setValue(position->value() + zDisplacement);
  orientation->setValue(OmRotation(roll * orientationValue.toQuaternion()));
}
