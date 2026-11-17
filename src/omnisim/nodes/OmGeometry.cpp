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

#include "OmGeometry.hpp"

#include "OmBoundingSphere.hpp"
#include "OmNode.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPrecision.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSimulationState.hpp"
#include "OmSlot.hpp"
#include "OmSolid.hpp"
#include "OmTransform.hpp"
#include "OmVector4.hpp"
#include "OmWorld.hpp"


// Constant used to scale down the line scale property
const float OmGeometry::LINE_SCALE_FACTOR = 250.0f;

const int gMaxIndexNumberToCastShadows = (1 << 16) - 1;  // 2^16 - 1 (16-bit resolution)

int OmGeometry::maxIndexNumberToCastShadows() {
  return gMaxIndexNumberToCastShadows;
}

void OmGeometry::init() {
  mCollisionTime = -std::numeric_limits<float>::infinity();
  mPreviousCollisionTime = -std::numeric_limits<float>::infinity();
  mIs90DegreesRotated = false;
  mResizeManipulator = NULL;
  mResizeManipulatorInitialized = false;
  mResizeConstraint = OmWrenAbstractResizeManipulator::NO_CONSTRAINT;
  mBoundingSphere = NULL;
  mPickable = false;
  mIsTransparent = false;
}

OmGeometry::OmGeometry(const QString &modelName, OmTokenizer *tokenizer) : OmBaseNode(modelName, tokenizer) {
  init();
}

OmGeometry::OmGeometry(const OmGeometry &other) : OmBaseNode(other) {
  init();
}

OmGeometry::OmGeometry(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmGeometry::~OmGeometry() {
  delete mResizeManipulator;
  delete mBoundingSphere;
}

void OmGeometry::postFinalize() {
  OmBaseNode::postFinalize();

  mBoundingSphere = new OmBoundingSphere(this);
  recomputeBoundingSphere();

}

////////////////////////
// Create ODE Objects //
////////////////////////

void OmGeometry::createOdeObjects() {
  OmBaseNode::createOdeObjects();
}

// Default creation method (overriden in every OmGeometry inherited class)
bool OmGeometry::createOdeGeom() {
  parsingWarn(tr("This type of geometry node cannot be placed in 'boundingObject'."));
  return false;
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void OmGeometry::setPickable(bool pickable) {
  if (isInBoundingObject())
    return;

  mPickable = pickable && isShadedGeometryPickable();
}

///////////////////
// Apply Methods //
///////////////////

////////////
// Others //
////////////

void OmGeometry::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  // D1.4: the WREN bounding-object collision recolour died with WREN; the collision TIME
  // state (setColliding) survives for any future overlay recolour.
  (void)triggerChange;
  (void)onSelection;
}

void OmGeometry::setColliding() {
  if (mCollisionTime != OmSimulationState::instance()->time()) {
    mPreviousCollisionTime = mCollisionTime;
    mCollisionTime = OmSimulationState::instance()->time();
  }
}

void OmGeometry::setSleepMaterial() {
}

// Selection management
void OmGeometry::propagateSelection(bool selected) {
  // D1.4: selection feedback is the wgpu selection tint/box (OmView3D), not a WREN
  // visibility flip.
  (void)selected;
}

bool OmGeometry::isSelected() {
  const OmMatter *const matter = OmNodeUtilities::findUpperMatter(this);
  if (matter)
    return matter->isSelected();
  return false;
}

//////////////
// Setters  //
//////////////

void OmGeometry::setTransparent(bool isTransparent) {
  mIsTransparent = isTransparent;
}

void OmGeometry::showResizeManipulator(bool enabled) {
  if (enabled)
    attachResizeManipulator();
  else
    detachResizeManipulator();

  emit visibleHandlesChanged(enabled);
}

void OmGeometry::updateResizeHandlesSize() {
  createResizeManipulatorIfNeeded();
  if (mResizeManipulator && !OmNodeUtilities::isNodeOrAncestorLocked(this))
    mResizeManipulator->computeHandleScaleFromViewportSize();
}

void OmGeometry::createResizeManipulatorIfNeeded() {
  if (!mResizeManipulatorInitialized) {
    mResizeManipulatorInitialized = true;
    if (!mResizeManipulator && hasResizeManipulator()) {
      createResizeManipulator();
      if (mResizeManipulator)
        mResizeManipulator->attachTo();
    }
  }
}

OmWrenAbstractResizeManipulator *OmGeometry::resizeManipulator() {
  createResizeManipulatorIfNeeded();
  return mResizeManipulator;
}

bool OmGeometry::isResizeManipulatorAttached() const {
  return mResizeManipulator ? mResizeManipulator->isAttached() : false;
}

void OmGeometry::attachResizeManipulator() {
  createResizeManipulatorIfNeeded();
  if (mResizeManipulator && !mResizeManipulator->isAttached()) {
    setResizeManipulatorDimensions();
    mResizeManipulator->show();
  }
}

void OmGeometry::detachResizeManipulator() const {
  if (mResizeManipulator && mResizeManipulator->isAttached())
    mResizeManipulator->hide();
}

void OmGeometry::setUniformConstraintForResizeHandles(bool enabled) {
  createResizeManipulatorIfNeeded();
  if (!mResizeManipulator || !mResizeManipulator->isAttached())
    return;

  if (enabled)
    mResizeManipulator->setResizeConstraint(OmScaleManipulator::UNIFORM);
  else
    mResizeManipulator->setResizeConstraint((OmWrenAbstractResizeManipulator::ResizeConstraint)mResizeConstraint);
}

// ODE setters

// Utility functions

OmBaseNode *OmGeometry::transformedGeometry() {  // returns an upper OmPose lying in the same boundingObject if it does
                                                 // exist, otherwise the OmGeometry itself
  OmPose *const up = upperPose();
  return up->isInBoundingObject() ? static_cast<OmBaseNode *>(up) : static_cast<OmBaseNode *>(this);
}

const OmVector3 OmGeometry::absoluteScale() const {
  const OmTransform *const up = upperTransform();
  return up ? up->absoluteScale() : OmVector3(1.0, 1.0, 1.0);
}

OmVector3 OmGeometry::absolutePosition() const {
  const OmPose *const up = upperPose();
  return up ? up->position() : OmVector3();
}

void OmGeometry::setOdePosition(const OmVector3 &translation) {
  mOdePositionSet = translation;
  mOdeOffsetTranslation = translation + mOdeOffsetRotation * mLocalOdeGeomOffsetPosition;
}

void OmGeometry::setOdeRotation(const OmMatrix3 &rotation) {
  mOdeOffsetRotation = rotation;

  if (!mLocalOdeGeomOffsetPosition.isNull())
    // the position should be recomputed because the local offset is influenced by the global rotation
    setOdePosition(mOdePositionSet);

  if (mIs90DegreesRotated) {
    // append 90 deg rotation
    static const OmMatrix3 localRotation = OmRotation(1.0, 0.0, 0.0, M_PI_2).toMatrix3();
    mOdeOffsetRotation *= localRotation;
  }
}

bool OmGeometry::isAValidBoundingObject(bool checkOde, bool warning) const {
  if (!isInBoundingObject())
    return false;

  const OmPose *const up = upperPose();
  if (up && up->isInBoundingObject() && up->geometry() != this)
    return false;

  // checkOde asked whether an ODE collision geom existed; none ever does now,
  // so that question is answered "no" exactly as it was after ODE went away.
  if (checkOde)
    return false;

  return true;
}

int OmGeometry::triangleCount() const {
  // D1.4: the WREN mesh this used to measure is gone; subclasses with a CPU triangle mesh
  // override (OmTriangleMeshGeometry).
  return 0;
}

bool OmGeometry::exportNodeHeader(OmWriter &writer) const {
  if (writer.isUrdf())
    return true;
  return OmBaseNode::exportNodeHeader(writer);
}

////////////////////////////////
//  Position and orientation  //
////////////////////////////////

OmMatrix4 OmGeometry::matrix() const {
  const OmPose *up = upperPose();
  if (!up)
    return OmMatrix4();
  if (!up->isInBoundingObject())
    return up->matrix();
  else {
    const OmMatrix4 &matrix4 = up->vrmlMatrix();
    up = up->upperPose();
    return up->matrix() * matrix4;
  }
}

///////////////////////////////
// Scale Handles Constraints //
///////////////////////////////

int OmGeometry::constraintType() const {
  const int geometryType = nodeType();
  int constraint = OmWrenAbstractResizeManipulator::NO_CONSTRAINT;
  if (geometryType == WB_NODE_SPHERE || geometryType == WB_NODE_CAPSULE)
    constraint = OmWrenAbstractResizeManipulator::UNIFORM;
  else if (geometryType == WB_NODE_CYLINDER)
    constraint = OmWrenAbstractResizeManipulator::X_EQUAL_Y;

  return constraint;
}

////////////
// Export //
////////////

void OmGeometry::exportBoundingObjectToW3d(OmWriter &writer) const {
  assert(writer.isW3d());
  assert(isInBoundingObject());
  this->write(writer);
}
