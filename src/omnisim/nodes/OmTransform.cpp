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

#include "OmTransform.hpp"

#include "OmBoundingSphere.hpp"
#include "OmSimulationState.hpp"
#include "OmTranslateRotateManipulator.hpp"

void OmTransform::init() {
  mScale = findSFVector3("scale");
  mPreviousXscaleValue = 1.0;
  mAbsoluteScaleNeedUpdate = true;
}

OmTransform::OmTransform(OmTokenizer *tokenizer) : OmPose("Transform", tokenizer) {
  init();
}

OmTransform::OmTransform(const OmTransform &other) : OmPose(other) {
  init();
}

OmTransform::OmTransform(const OmNode &other) : OmPose(other) {
  init();
}

void OmTransform::preFinalize() {
  OmPose::preFinalize();

  sanitizeScale();
}

void OmTransform::postFinalize() {
  OmPose::postFinalize();

  connect(mScale, SIGNAL(changed()), this, SLOT(updateScale()));
}

void OmTransform::applyToScale() {
  mBaseNode->setMatrixNeedUpdate();
  mBaseNode->setScaleNeedUpdate();

  if (OmSimulationState::instance()->isRayTracingEnabled() && mBaseNode->boundingSphere())
    mBaseNode->boundingSphere()->setOwnerSizeChanged();

  if (mTranslateRotateManipulator && mTranslateRotateManipulator->isAttached())
    updateTranslateRotateHandlesSize();
}

void OmTransform::updateScale(bool warning) {
  sanitizeScale();

  applyToScale();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void OmTransform::sanitizeScale() {
  OmVector3 sanitizedScale = mScale->value();
  bool invalid = false;

  if (sanitizedScale.x() == 0.0) {
    sanitizedScale.setX(1.0);
    mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be non-zero: x is set to 1.0."));
    invalid = true;
  }

  if (sanitizedScale.y() == 0.0) {
    sanitizedScale.setY(1.0);
    mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be non-zero: y is set to 1.0."));
    invalid = true;
  }

  if (sanitizedScale.z() == 0.0) {
    sanitizedScale.setZ(1.0);
    mBaseNode->parsingWarn(QObject::tr("All 'scale' coordinates must be non-zero: z is set to 1.0."));
    invalid = true;
  }

  if (invalid)
    mScale->setValue(sanitizedScale);
}

void OmTransform::applyToOdeScale() {
  geometry()->applyToOdeData();
}

QStringList OmTransform::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "scale" << OmPose::fieldsToSynchronizeWithW3d();
  return fields;
}

void OmTransform::updateAbsoluteScale() const {
  mAbsoluteScale = mScale->value();
  // multiply with upper transform scale if any
  const OmTransform *const up = mBaseNode->upperTransform();
  if (up)
    mAbsoluteScale *= up->absoluteScale();

  mAbsoluteScaleNeedUpdate = false;
}

const OmVector3 &OmTransform::absoluteScale() const {
  if (mAbsoluteScaleNeedUpdate)
    updateAbsoluteScale();

  return mAbsoluteScale;
}

const OmMatrix4 &OmTransform::vrmlMatrix() const {
  if (mVrmlMatrixNeedUpdate) {
    mVrmlMatrix.fromVrml(translation(), rotation(), scale());
    mVrmlMatrixNeedUpdate = false;
  }

  return mVrmlMatrix;
}

void OmTransform::setScaleNeedUpdate() {
  setScaleNeedUpdateFlag();
  OmGroup::setScaleNeedUpdate();
}

void OmTransform::setScaleNeedUpdateFlag() const {
  // optimisation: it's useless to call the function recursively if scalarScaleNeedUpdate is true,
  // because all the children's scalarNeedUpdate are already true.
  if (mAbsoluteScaleNeedUpdate)
    return;

  mAbsoluteScaleNeedUpdate = true;
}

void OmTransform::updateMatrix() const {
  assert(mMatrix);

  // combine with upper matrix if any
  const OmPose *const pose = upperPose();
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
    s = absoluteScale();
  } else {
    t = mTranslation->value();
    r = mRotation->value();
    s = mScale->value();
  }

  mMatrix->fromVrml(t.x(), t.y(), t.z(), r.x(), r.y(), r.z(), r.angle(), s.x(), s.y(), s.z());
  mMatrixNeedUpdate = false;
}
