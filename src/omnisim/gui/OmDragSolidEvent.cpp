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

#include "OmDragSolidEvent.hpp"

#include "OmEditCommand.hpp"
#include "OmLog.hpp"
#include "OmMathsUtilities.hpp"
#include "OmPhysicsVectorRepresentation.hpp"
#include "OmRotation.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmSolidMerger.hpp"
#include "OmStandardPaths.hpp"
#include "OmTranslateRotateManipulator.hpp"
#include "OmUndoStack.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"
#include "OmWrenLabelOverlay.hpp"
#include "OmWrenRenderingContext.hpp"

#include <QtCore/QSize>

OmDragHorizontalSolidEvent::OmDragHorizontalSolidEvent(const QPoint &initialPosition, OmViewpoint *viewpoint,
                                                       OmSolid *selectedSolid) :
  OmDragHorizontalEvent(initialPosition, viewpoint, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

OmDragHorizontalSolidEvent::~OmDragHorizontalSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void OmDragHorizontalSolidEvent::apply(const QPoint &currentMousePosition) {
  OmDragHorizontalEvent::apply(currentMousePosition);
}

OmDragVerticalSolidEvent::OmDragVerticalSolidEvent(const QPoint &initialPosition, OmViewpoint *viewpoint,
                                                   OmSolid *selectedSolid) :
  OmDragVerticalEvent(initialPosition, viewpoint, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

OmDragVerticalSolidEvent::~OmDragVerticalSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void OmDragVerticalSolidEvent::apply(const QPoint &currentMousePosition) {
  OmDragVerticalEvent::apply(currentMousePosition);
}

OmDragTranslateAlongAxisSolidEvent::OmDragTranslateAlongAxisSolidEvent(const QPoint &initialMousePosition,
                                                                       const QSize &widgetSize, OmViewpoint *viewpoint,
                                                                       int handleNumber, OmSolid *selectedSolid) :
  OmDragTranslateAlongAxisEvent(initialMousePosition, widgetSize, viewpoint, handleNumber, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

OmDragTranslateAlongAxisSolidEvent::~OmDragTranslateAlongAxisSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void OmDragTranslateAlongAxisSolidEvent::apply(const QPoint &currentMousePosition) {
  OmDragTranslateAlongAxisEvent::apply(currentMousePosition);
}

OmDragRotateAroundWorldVerticalAxisSolidEvent::OmDragRotateAroundWorldVerticalAxisSolidEvent(const QPoint &initialPosition,
                                                                                             OmViewpoint *viewpoint,
                                                                                             OmSolid *selectedSolid) :
  OmDragRotateAroundWorldVerticalAxisEvent(initialPosition, viewpoint, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

OmDragRotateAroundWorldVerticalAxisSolidEvent::~OmDragRotateAroundWorldVerticalAxisSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void OmDragRotateAroundWorldVerticalAxisSolidEvent::apply(const QPoint &currentMousePosition) {
  OmDragRotateAroundWorldVerticalAxisEvent::apply(currentMousePosition);
}

OmDragRotateAroundAxisSolidEvent::OmDragRotateAroundAxisSolidEvent(const QPoint &initialMousePosition, const QSize &widgetSize,
                                                                   OmViewpoint *viewpoint, int handleNumber,
                                                                   OmSolid *selectedSolid) :
  OmDragRotateAroundAxisEvent(initialMousePosition, widgetSize, viewpoint, handleNumber, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

OmDragRotateAroundAxisSolidEvent::~OmDragRotateAroundAxisSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void OmDragRotateAroundAxisSolidEvent::apply(const QPoint &currentMousePosition) {
  OmDragRotateAroundAxisEvent::apply(currentMousePosition);
}

// Physics drag events //
/////////////////////////

// Abstract class

// OmDragPhysicsEvent constructor
OmDragPhysicsEvent::OmDragPhysicsEvent(const QSize &widgetSize, OmViewpoint *viewpoint, OmSolid *selectedSolid) :
  OmDragView3DEvent(viewpoint),
  mSelectedSolid(selectedSolid),
  mIsLocked(false),
  mWidgetSize(widgetSize),
  mViewDistanceScaling(1.0f) {
  // init label
  mTextOverlay = OmWrenLabelOverlay::createOrRetrieve(OmWrenLabelOverlay::dragCaptionOverlayId(),
                                                      OmStandardPaths::fontsPath() + "LiberationSans-Regular.ttf");
  mTextOverlay->setColor(0x00FFFFFF);
  mTextOverlay->setBackgroundColor(0x70000000);
  mTextOverlay->setSize(0.1);
  mTextOverlay->setText("0");
  mTextOverlay->applyChangesToWren();

  connect(OmSimulationWorld::instance(), &OmSimulationWorld::physicsStepStarted, this,
          &OmDragPhysicsEvent::updateRenderingAndPhysics, Qt::UniqueConnection);
  mViewpoint->lock();
}

// OmDragPhysicsEvent destructor
OmDragPhysicsEvent::~OmDragPhysicsEvent() {
  delete mRepresentation;
  // destroy translation offset label
  OmWrenLabelOverlay::removeLabel(OmWrenLabelOverlay::dragCaptionOverlayId());
  mViewpoint->unlock();
}

void OmDragPhysicsEvent::init() {
  // Set size so that the arrow head has a length of 50px
  const float screenSize = 50.0f;  // px
  mViewDistanceScaling = mViewpoint->viewDistanceUnscaling(mSelectedSolid->position()) * screenSize;
}

void OmDragPhysicsEvent::lock() {
  mIsLocked = true;
  disconnect(OmSimulationWorld::instance(), &OmSimulationWorld::physicsStepStarted, this,
             &OmDragPhysicsEvent::updateRenderingAndPhysics);
}

void OmDragPhysicsEvent::apply(const QPoint &currentMousePosition) {
  // If the mouse button was released in PAUSE mode, skip updates
  if (mIsLocked)
    return;

  // Updates the World Coordinates of Force or the Torque's end
  mViewpoint->viewpointRay(currentMousePosition.x(), currentMousePosition.y(), mMouseRay);
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  mEnd = mMouseRay.point(mIntersectionOutput.second);

  // Updates the World Coordinates of Force or Torque's origin
  updateOrigin();

  if (exceedsFloatMax(mEnd) || exceedsFloatMax(mVector) || exceedsFloatMax(mOrigin)) {
    emit aborted();
    return;
  }

  // Updates graphical representation
  applyChangesToWren();

  // display the magnitude of the generated vector in the overlay label
  const OmVector2 labelPosition = clampLabelPosition((25.0 + currentMousePosition.x()) / mWidgetSize.width(),
                                                     (-20.0 + currentMousePosition.y()) / mWidgetSize.height(), mTextOverlay);
  mTextOverlay->moveToPosition(labelPosition.x(), labelPosition.y());
  mTextOverlay->updateText(magnitudeString());
}

void OmDragPhysicsEvent::applyChangesToWren() {
  mRepresentation->updatePosition(mOrigin, mEnd, mViewpoint->orientation()->value());
}

void OmDragPhysicsEvent::updateRenderingAndPhysics() {
  updateOrigin();

  if (exceedsFloatMax(mEnd) || exceedsFloatMax(mVector) || exceedsFloatMax(mOrigin)) {
    emit aborted();
    return;
  }

  mTextOverlay->updateText(magnitudeString());
  applyChangesToWren();
  applyToOde();
}

//
// OmDragPhysicsEvent implemented functions
//

// Add a force by dragging the mouse  //
////////////////////////////////////////

// OmDragForceEvent functions

OmDragForceEvent::OmDragForceEvent(const QSize &widgetSize, OmViewpoint *viewpoint, OmSolid *selectedSolid) :
  OmDragPhysicsEvent(widgetSize, viewpoint, selectedSolid) {
  mRepresentation = new OmForceRepresentation();
  mOrigin = viewpoint->rotationCenter();
  mRelativeOrigin = selectedSolid->matrix().pseudoInversed(mOrigin);
  mEnd = mOrigin;
  mDragPlane = OmAffinePlane(viewpoint->orientation()->value().direction(), mOrigin);
  mScalingFactor = OmWorld::instance()->worldInfo()->dragForceScale() * selectedSolid->mass();
  init();
  mRepresentation->setScale(mViewDistanceScaling);
}

void OmDragForceEvent::updateOrigin() {
  mOrigin = mSelectedSolid->matrix() * mRelativeOrigin;
  mVector = mEnd - mOrigin;
}

void OmDragForceEvent::applyToOde() {
  // ODE
  mSelectedSolid->awake();
  mSelectedSolid->addForceAtPosition(mScalingFactor * mVector.length2() * mVector, mOrigin);
}

QString OmDragForceEvent::magnitudeString() const {
  return OmPrecision::doubleToString(mScalingFactor * mVector.length2() * mVector.length(), OmPrecision::GUI_MEDIUM) + " N";
}

// Add a torque by dragging the mouse  //
/////////////////////////////////////////

// OmDragTorqueEvent functions

OmDragTorqueEvent::OmDragTorqueEvent(const QSize &widgetSize, OmViewpoint *viewpoint, OmSolid *selectedSolid) :
  OmDragPhysicsEvent(widgetSize, viewpoint, selectedSolid->solidMerger()->solid()),
  mSolidMerger(selectedSolid->solidMerger()) {
  mRepresentation = new OmTorqueRepresentation();
  mOrigin = mSelectedSolid->matrix() * mSolidMerger->centerOfMass();
  mEnd = mOrigin;
  mDragPlane = OmAffinePlane(viewpoint->orientation()->value().direction(), mOrigin);
  mScalingFactor = OmWorld::instance()->worldInfo()->dragTorqueScale() * selectedSolid->mass();
  init();
  mRepresentation->setScale(mViewDistanceScaling);
}

void OmDragTorqueEvent::updateOrigin() {
  mOrigin = mSelectedSolid->matrix() * mSolidMerger->centerOfMass();
  mVector = mEnd - mOrigin;
}

void OmDragTorqueEvent::applyToOde() {
  // ODE
  mSelectedSolid->awake();
  mSelectedSolid->addTorque(mScalingFactor * mVector.length2() * mVector);
}

QString OmDragTorqueEvent::magnitudeString() const {
  return OmPrecision::doubleToString(mScalingFactor * mVector.length2() * mVector.length(), OmPrecision::GUI_MEDIUM) + " Nm";
}
