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

#include "OmAbstractPose.hpp"

#include "OmBaseNode.hpp"
#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmMatter.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmSimulationState.hpp"
#include "OmTransform.hpp"
#include "OmTranslateRotateManipulator.hpp"
#include "OmVrmlNodeUtilities.hpp"

void OmAbstractPose::init(OmBaseNode *node) {
  mBaseNode = node;
  assert(mBaseNode);

  mTranslation = node->findSFVector3("translation");
  mRotation = node->findSFRotation("rotation");
  mTranslationStep = node->findSFDouble("translationStep");
  mRotationStep = node->findSFDouble("rotationStep");

  mMatrix = NULL;
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
  mTranslateRotateManipulator = NULL;

  mTranslateRotateManipulatorInitialized = false;

  mIsTranslationFieldVisible = true;
  mIsRotationFieldVisible = true;
  mIsTranslationFieldVisibleReady = false;
  mIsRotationFieldVisibleReady = false;
  mCanBeTranslated = false;
  mCanBeRotated = false;
}

OmAbstractPose::~OmAbstractPose() {
  deleteWrenObjects();
  delete mMatrix;
}

void OmAbstractPose::deleteWrenObjects() {
  delete mTranslateRotateManipulator;
  mTranslateRotateManipulator = NULL;
  mTranslateRotateManipulatorInitialized = false;
}

void OmAbstractPose::updateTranslation() {
  mBaseNode->setMatrixNeedUpdate();

  if (OmSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();
}

void OmAbstractPose::updateRotation() {
  mBaseNode->setMatrixNeedUpdate();
  mRelativeQuaternion = mRotation->value().toQuaternion();

  if (OmSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

void OmAbstractPose::updateTranslationAndRotation() {
  mBaseNode->setMatrixNeedUpdate();
  mRelativeQuaternion = mRotation->value().toQuaternion();

  if (OmSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere() && !mBaseNode->isInBoundingObject())
    mBaseNode->boundingSphere()->setOwnerMoved();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

void OmAbstractPose::updateTranslationFieldVisibility() const {
  if (mIsTranslationFieldVisibleReady)
    return;
  mIsTranslationFieldVisible = OmVrmlNodeUtilities::isVisible(mBaseNode->findField("translation", true));
  mCanBeTranslated =
    !OmNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("translation", true)) && mIsTranslationFieldVisible;
  mIsTranslationFieldVisibleReady = true;
}

void OmAbstractPose::updateRotationFieldVisibility() const {
  if (mIsRotationFieldVisibleReady)
    return;
  mIsRotationFieldVisible = OmVrmlNodeUtilities::isVisible(mBaseNode->findField("rotation", true));
  mCanBeRotated =
    !OmNodeUtilities::isTemplateRegeneratorField(mBaseNode->findField("rotation", true)) && mIsRotationFieldVisible;
  mIsRotationFieldVisibleReady = true;
}

bool OmAbstractPose::isTranslationFieldVisible() const {
  updateTranslationFieldVisibility();
  return mIsTranslationFieldVisible;
}

bool OmAbstractPose::isRotationFieldVisible() const {
  updateRotationFieldVisibility();
  return mIsRotationFieldVisible;
}

bool OmAbstractPose::canBeTranslated() const {
  updateTranslationFieldVisibility();
  return mCanBeTranslated;
}

bool OmAbstractPose::canBeRotated() const {
  updateRotationFieldVisibility();
  return mCanBeRotated;
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void OmAbstractPose::createTranslateRotateManipulatorIfNeeded() {
  if (!mTranslateRotateManipulatorInitialized) {
    mTranslateRotateManipulatorInitialized = true;
    if (!canBeTranslated() && !canBeRotated())
      return;
    mTranslateRotateManipulator = new OmTranslateRotateManipulator(canBeTranslated(), canBeRotated());
    if (mTranslateRotateManipulator) {
      mTranslateRotateManipulator->attachTo(this);
      updateTranslateRotateHandlesSize();
    }
  }
}

OmMatrix3 OmAbstractPose::rotationMatrix() const {
  const OmMatrix4 &m = matrix();
  OmMatrix3 rm = m.extracted3x3Matrix();
  const OmTransform *t = dynamic_cast<OmTransform *>(mBaseNode);
  if (!t)
    t = mBaseNode->upperTransform();
  if (t) {
    const OmVector3 s = t->absoluteScale();
    rm.scale(1.0 / s.x(), 1.0 / s.y(), 1.0 / s.z());
  }
  return rm;
}

// Matrix 4-by-4

const OmMatrix4 &OmAbstractPose::matrix() const {
  // ensure that the matrix value is not used and computed before the node is finalized
  // because in some cases field values are only set after the node construction
  // (for example in case of PROTO instances)
  assert(mBaseNode->isPreFinalizedCalled());

  if (!mMatrix) {
    mMatrix = new OmMatrix4();
    updateMatrix();
  } else if (mMatrixNeedUpdate)
    updateMatrix();

  return *mMatrix;
}

void OmAbstractPose::updateMatrix() const {
  assert(mMatrix);

  // combine with upper matrix if any
  const OmPose *const pose = mBaseNode->upperPose();
  OmVector3 t, s;
  OmRotation r;
  if (pose) {
    // to prevent shear effect in case of non-uniform scaling, it is not possible to multiply the transform matrix directly
    // note that this computation matches the one in WREN
    const OmTransform *transform = dynamic_cast<const OmTransform *>(pose);
    if (!transform)
      transform = pose->upperTransform();
    s = transform ? transform->absoluteScale() : OmVector3(1.0, 1.0, 1.0);
    OmQuaternion q = pose->rotationMatrix().toQuaternion();
    t = pose->position() + q * (s * mTranslation->value());
    mRelativeQuaternion = mRotation->value().toQuaternion();
    q = q * mRelativeQuaternion;
    q.normalize();
    r.fromQuaternion(q);
  } else {
    t = mTranslation->value();
    r = mRotation->value();
    s = OmVector3(1.0, 1.0, 1.0);
  }

  mMatrix->fromVrml(t.x(), t.y(), t.z(), r.x(), r.y(), r.z(), r.angle(), s.x(), s.y(), s.z());
  mMatrixNeedUpdate = false;
}

void OmAbstractPose::setMatrixNeedUpdateFlag() const {
  mMatrixNeedUpdate = true;
  mVrmlMatrixNeedUpdate = true;
}

const OmMatrix4 &OmAbstractPose::vrmlMatrix() const {
  if (mVrmlMatrixNeedUpdate) {
    mVrmlMatrix.fromVrml(translation(), rotation(), OmVector3(1, 1, 1));
    mVrmlMatrixNeedUpdate = false;
  }

  return mVrmlMatrix;
}

// Position and orientation setters

void OmAbstractPose::setTranslationAndRotation(double tx, double ty, double tz, double rx, double ry, double rz, double angle) {
  mTranslation->setValue(tx, ty, tz);
  mRotation->setValue(rx, ry, rz, angle);
}

void OmAbstractPose::setTranslationAndRotation(const OmVector3 &v, const OmRotation &r) {
  mTranslation->setValue(v);
  mRotation->setValue(r);
}

void OmAbstractPose::setTranslation(double tx, double ty, double tz) {
  mTranslation->setValue(tx, ty, tz);
}

void OmAbstractPose::rotate(const OmVector3 &v) {
  OmMatrix3 rotation(v.normalized(), v.length());
  OmRotation newRotation = OmRotation(rotation * mRotation->value().toMatrix3());
  newRotation.normalize();
  setRotation(newRotation);
}

void OmAbstractPose::setRotation(double x, double y, double z, double angle) {
  mRotation->setValue(x, y, z, angle);
}

void OmAbstractPose::setRotationAngle(double angle) {
  mRotation->setAngle(angle);
}

void OmAbstractPose::setRotation(const OmRotation &r) {
  mRotation->setValue(r);
}

///////////////////////////////////////////////////////
//  WREN methods related to OmPose manipulators //
///////////////////////////////////////////////////////

void OmAbstractPose::attachTranslateRotateManipulator() {
  createTranslateRotateManipulatorIfNeeded();

  if (mTranslateRotateManipulator) {
    if (!mTranslateRotateManipulator->isAttached()) {
      // get size
      OmBaseNode *node = baseNode();
      if (node->isInBoundingObject())
        node = OmNodeUtilities::findBoundingObjectAncestor(node);
      OmBoundingSphere *bs = node->boundingSphere();
      if (bs)
        bs->recomputeIfNeeded(false);

      mTranslateRotateManipulator->show();
    }
  }
}

void OmAbstractPose::detachTranslateRotateManipulator() {
  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    mTranslateRotateManipulator->hide();
}

void OmAbstractPose::updateTranslateRotateHandlesSize() {
  createTranslateRotateManipulatorIfNeeded();
  if (!mTranslateRotateManipulator)
    return;

  const OmTransform *transform = dynamic_cast<OmTransform *>(mBaseNode);
  if (!transform)
    transform = mBaseNode->upperTransform();
  if (transform)
    mTranslateRotateManipulator->updateHandleScale(transform->absoluteScale().ptr());

  if (!OmNodeUtilities::isNodeOrAncestorLocked(mBaseNode))
    mTranslateRotateManipulator->computeHandleScaleFromViewportSize();
}
