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

#include "OmSphere.hpp"

#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatter.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSFBool.hpp"
#include "OmSFInt.hpp"
#include "OmSimulationState.hpp"
#include "OmTokenizer.hpp"
#include "OmTransform.hpp"
#include "OmVersion.hpp"
#include "OmVrmlNodeUtilities.hpp"


#include <cmath>

void OmSphere::init() {
  mRadius = findSFDouble("radius");
  mSubdivision = findSFInt("subdivision");
  mIco = findSFBool("ico");
  mResizeConstraint = OmWrenAbstractResizeManipulator::UNIFORM;
}

OmSphere::OmSphere(OmTokenizer *tokenizer) : OmGeometry("Sphere", tokenizer) {
  init();
  if (tokenizer == NULL)
    mRadius->setValueNoSignal(0.1);
}

OmSphere::OmSphere(const OmSphere &other) : OmGeometry(other) {
  init();
}

OmSphere::OmSphere(const OmNode &other) : OmGeometry(other) {
  init();
}

OmSphere::~OmSphere() {
}

void OmSphere::postFinalize() {
  OmGeometry::postFinalize();

  connect(mRadius, &OmSFDouble::changed, this, &OmSphere::updateRadius);
  connect(mSubdivision, &OmSFInt::changed, this, &OmSphere::updateMesh);
  connect(mIco, &OmSFBool::changed, this, &OmSphere::updateMesh);
}

void OmSphere::createWrenObjects() {
  OmGeometry::createWrenObjects();

  sanitizeFields();

  emit wrenObjectsCreated();
}

void OmSphere::setResizeManipulatorDimensions() {
  OmVector3 scale(radius(), radius(), radius());

  const OmTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void OmSphere::createResizeManipulator() {
  mResizeManipulator =
    new OmRegularResizeManipulator(uniqueId(), (OmWrenAbstractResizeManipulator::ResizeConstraint)mResizeConstraint);
}

bool OmSphere::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const radiusField = findField("radius", true);
  return OmVrmlNodeUtilities::isVisible(radiusField) && !OmNodeUtilities::isTemplateRegeneratorField(radiusField);
}

bool OmSphere::sanitizeFields() {
  bool invalidValue;
  if (mIco->value()) {
    invalidValue = OmFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mSubdivision, 1, 5, 1);
  } else
    invalidValue = OmFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mSubdivision, 3, 32, 24);
  if (invalidValue)
    return false;

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mRadius, 1.0))
    return false;

  return true;
}

void OmSphere::updateRadius() {
  if (!sanitizeFields())
    return;

  if (isInBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmSphere::updateMesh() {
  if (!sanitizeFields())
    return;

  emit changed();
}

void OmSphere::rescale(const OmVector3 &scale) {
  if (scale.x() != 1.0)
    setRadius(radius() * scale.x());
  else if (scale.y() != 1.0)
    setRadius(radius() * scale.y());
  else if (scale.z() != 1.0)
    setRadius(radius() * scale.z());
}

QStringList OmSphere::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "radius"
         << "ico"
         << "subdivision";
  return fields;
}

/////////////////
// ODE objects //
/////////////////

bool OmSphere::createOdeGeom() {
  if (mRadius->value() <= 0.0) {
    parsingWarn(tr("'radius' must be positive when used in 'boundingObject'. The Sphere is rejected as a collider, so the Solid does not collide through it; set 'radius' to a positive value in metres."));
    return NULL;
  }

  return true;
}

double OmSphere::scaledRadius() const {
  const OmVector3 &scale = absoluteScale();
  return fabs(mRadius->value() * std::max(std::max(scale.x(), scale.y()), scale.z()));
}

bool OmSphere::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidRadius = mRadius->value() <= 0.0;
  if (warning && invalidRadius)
    parsingWarn(tr("'radius' must be positive when used in 'boundingObject'. The Sphere is rejected as a collider, so the Solid does not collide through it; set 'radius' to a positive value in metres."));
  return !invalidRadius;
}

bool OmSphere::isAValidBoundingObject(bool checkOde, bool warning) const {
  const bool admissible = OmGeometry::isAValidBoundingObject(checkOde, warning);
  return admissible && isSuitableForInsertionInBoundingObject(admissible && warning);
}
/////////////////
// Ray Tracing //
/////////////////

bool OmSphere::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 collisionPoint;
  bool collisionExists = computeCollisionPoint(collisionPoint, ray);
  if (!collisionExists)
    return false;

  OmVector3 pointOnTexture(collisionPoint);
  const OmPose *const up = upperPose();
  if (up) {
    pointOnTexture = up->matrix().pseudoInversed(collisionPoint);
    pointOnTexture /= absoluteScale();
  }

  const double u = 0.5 + atan2(pointOnTexture.x(), -pointOnTexture.y()) * 0.5 * M_1_PI;
  const double v = 0.5 - OmMathsUtilities::clampedAsin(pointOnTexture.z() / scaledRadius()) * M_1_PI;

  // result
  uv.setXy(u, v);
  return true;
}

double OmSphere::computeDistance(const OmRay &ray) const {
  OmVector3 collisionPoint;
  bool collisionExists = computeCollisionPoint(collisionPoint, ray);
  if (!collisionExists)
    return -1;

  OmVector3 d = ray.origin() - collisionPoint;
  return d.length();
}

bool OmSphere::computeCollisionPoint(OmVector3 &point, const OmRay &ray) const {
  OmVector3 center;
  const OmPose *const up = upperPose();
  if (up)
    center = up->matrix().translation();
  const double r = scaledRadius();

  // distance from sphere
  const std::pair<bool, double> result = ray.intersects(center, r, true);

  point = ray.origin() + result.second * ray.direction();
  return result.first;
}

void OmSphere::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->set(OmVector3(), radius());
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmSphere::computeFrictionDirection(const OmVector3 &normal) const {
  parsingWarn(
    tr("A Sphere is used in a Bounding object using an asymmetric friction. Sphere does not support asymmetric friction"));
  return OmVector3(0, 0, 0);
}
