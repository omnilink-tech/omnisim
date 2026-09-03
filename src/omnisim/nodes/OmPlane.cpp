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

#include "OmPlane.hpp"

#include "OmAffinePlane.hpp"
#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSFVector2.hpp"
#include "OmSimulationState.hpp"
#include "OmTransform.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWrenAbstractResizeManipulator.hpp"
#include "OmWriter.hpp"


#include <algorithm>

void OmPlane::init() {
  mSize = findSFVector2("size");
}

OmPlane::OmPlane(OmTokenizer *tokenizer) : OmGeometry("Plane", tokenizer) {
  init();
}

OmPlane::OmPlane(const OmPlane &other) : OmGeometry(other) {
  init();
}

OmPlane::OmPlane(const OmNode &other) : OmGeometry(other) {
  init();
}

OmPlane::~OmPlane() {
}

void OmPlane::postFinalize() {
  OmGeometry::postFinalize();

  connect(mSize, &OmSFVector2::changed, this, &OmPlane::updateSize);
}

const OmVector2 &OmPlane::size() const {
  return mSize->value();
}

void OmPlane::setSize(const OmVector2 &size) {
  mSize->setValue(size);
}

void OmPlane::setSize(double x, double y) {
  mSize->setValue(x, y);
}

void OmPlane::setX(double x) {
  mSize->setX(x);
}

void OmPlane::setY(double y) {
  mSize->setY(y);
}

const OmVector2 OmPlane::scaledSize() const {
  const OmVector2 &s1 = mSize->value();
  const OmVector3 &s2 = absoluteScale();
  return OmVector2(fabs(s2.x() * s1.x()), fabs(s2.y() * s1.y()));
}

void OmPlane::createWrenObjects() {
  OmGeometry::createWrenObjects();

  sanitizeFields();
  updateSize();

  emit wrenObjectsCreated();
}

void OmPlane::createResizeManipulator() {
  mResizeManipulator = new OmPlaneResizeManipulator(uniqueId());
}

void OmPlane::setResizeManipulatorDimensions() {
  OmVector3 scale(size().x(), size().y(), 0.1f * std::min(mSize->value().x(), mSize->value().y()));

  const OmTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

bool OmPlane::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const sizeField = findField("size", true);
  return OmVrmlNodeUtilities::isVisible(sizeField) && !OmNodeUtilities::isTemplateRegeneratorField(sizeField);
}

bool OmPlane::sanitizeFields() {
  if (OmFieldChecker::resetVector2IfNonPositive(this, mSize, OmVector2(1.0, 1.0)))
    return false;

  return true;
}

void OmPlane::rescale(const OmVector3 &scale) {
  OmVector2 resizedSize = size();
  if (scale.x() != 1.0)
    resizedSize[0] *= scale.x();
  if (scale.y() != 1.0)
    resizedSize[1] *= scale.y();
  setSize(resizedSize);
}

void OmPlane::updateSize() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

bool OmPlane::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidDimensions = (mSize->x() <= 0.0 || mSize->y() <= 0.0);
  if (warning && invalidDimensions)
    parsingWarn(tr("All 'size' components must be positive for a Plane used in a 'boundingObject'. The Plane is rejected as a collider, so nothing rests on it; set both 'size' components to positive metres."));

  return !invalidDimensions;
}

QStringList OmPlane::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "size";
  return fields;
}

/////////////////
// ODE objects //
/////////////////

bool OmPlane::createOdeGeom() {
  double d;
  OmVector3 n;
  computePlaneParams(n, d);
  return true;
}

bool OmPlane::isAValidBoundingObject(bool checkOde, bool warning) const {
  const bool admissible = OmGeometry::isAValidBoundingObject(checkOde, warning);
  return admissible && isSuitableForInsertionInBoundingObject(admissible && warning);
}

void OmPlane::setOdePosition(const OmVector3 &translation) {
  updateOdePlanePosition();
}

void OmPlane::setOdeRotation(const OmMatrix3 &matrix) {
  updateOdePlanePosition();
}

void OmPlane::updateOdePlanePosition() {
}

void OmPlane::computePlaneParams(OmVector3 &n, double &d) {
  const OmPose *pose = upperPose();

  // initial values with identity matrices
  n.setXyz(0.0, 0.0, 1.0);  // plane normal

  if (pose) {
    const OmMatrix3 &m3 = pose->rotationMatrix();
    // Applies this pose's rotation to plane normal
    n = m3 * n;

    // Computes the d parameter in the plane equation
    d = pose->position().dot(n);
  } else
    d = 0.0;
}

/////////////////
// Ray tracing //
/////////////////

bool OmPlane::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 collisionPoint;
  const bool intersectionExists = computeCollisionPoint(collisionPoint, ray);
  if (!intersectionExists)
    // no valid collision
    return false;

  // transform intersection point to plane coordinates
  OmVector3 pointOnTexture(collisionPoint);
  const OmPose *const pose = upperPose();
  if (pose) {
    pointOnTexture = pose->matrix().pseudoInversed(collisionPoint);
    pointOnTexture /= absoluteScale();
  }

  // transform point into texture coordinates in range [0..1]
  const double sx = scaledSize().x();
  const double sy = scaledSize().y();

  const double u = pointOnTexture.x() / sx + 0.5;
  const double v = -pointOnTexture.y() / sy + 0.5;

  // result
  uv.setXy(u, v);
  return true;
}

double OmPlane::computeDistance(const OmRay &ray) const {
  OmVector3 collisionPoint;
  const bool collisionExists = computeCollisionPoint(collisionPoint, ray);
  if (!collisionExists)
    // no valid collision
    return -1;

  // distance
  const OmVector3 &d = ray.origin() - collisionPoint;
  return d.length();
}

bool OmPlane::computeCollisionPoint(OmVector3 &point, const OmRay &ray) const {
  // 1. Compute the 4 plane vertices in world coordinates.
  const double planeWidth = size().x();
  const double planeHeight = size().y();
  const OmMatrix4 &upperMatrix = upperPose()->matrix();
  const OmVector3 p1 = upperMatrix * OmVector3(0.5 * planeWidth, -0.5 * planeHeight, 0.0);
  const OmVector3 p2 = upperMatrix * OmVector3(0.5 * planeWidth, 0.5 * planeHeight, 0.0);
  const OmVector3 p3 = upperMatrix * OmVector3(-0.5 * planeWidth, 0.5 * planeHeight, 0.0);
  const OmVector3 p4 = upperMatrix * OmVector3(-0.5 * planeWidth, -0.5 * planeHeight, 0.0);

  // 2. Check if the ray intersects one of the two oriented triangle.
  // Compute the intersection point in such case.
  double u, v;
  const std::pair<bool, double> intersection1 = ray.intersects(p1, p2, p3, true, u, v);
  if (intersection1.first && intersection1.second > 0.0) {
    point = ray.origin() + intersection1.second * ray.direction();
    return true;
  }

  const std::pair<bool, double> intersection2 = ray.intersects(p1, p3, p4, true, u, v);
  if (intersection2.first && intersection2.second > 0.0) {
    point = ray.origin() + intersection2.second * ray.direction();
    return true;
  }

  // 3. The ray does not intersect the plane.
  return false;
}

void OmPlane::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->set(OmVector3(), mSize->value().length() / 2.0);
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmPlane::computeFrictionDirection(const OmVector3 &normal) const {
  return OmVector3(1, 0, 0);
}
