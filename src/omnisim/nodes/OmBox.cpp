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

#include "OmBox.hpp"

#include "OmAffinePlane.hpp"
#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmMatter.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSimulationState.hpp"
#include "OmTransform.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"
#include "OmWrenAbstractResizeManipulator.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

void OmBox::init() {
  mSize = findSFVector3("size");
}

OmBox::OmBox(OmTokenizer *tokenizer) : OmGeometry("Box", tokenizer) {
  init();
  if (tokenizer == NULL)
    mSize->setValueNoSignal(0.1, 0.1, 0.1);
}

OmBox::OmBox(const OmBox &other) : OmGeometry(other) {
  init();
}

OmBox::OmBox(const OmNode &other) : OmGeometry(other) {
  init();
}

OmBox::~OmBox() {
}

void OmBox::postFinalize() {
  OmGeometry::postFinalize();

  connect(mSize, &OmSFVector3::changed, this, &OmBox::updateSize);
}

const OmVector3 &OmBox::size() const {
  return mSize->value();
}

void OmBox::createWrenObjects() {
  OmGeometry::createWrenObjects();

  sanitizeFields();
  updateSize();

  emit wrenObjectsCreated();
}

void OmBox::setResizeManipulatorDimensions() {
  OmVector3 scale = size().abs();
  const OmTransform *const transform = upperTransform();
  if (transform)
    scale *= transform->absoluteScale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void OmBox::createResizeManipulator() {
  mResizeManipulator = new OmRegularResizeManipulator(uniqueId());
}

bool OmBox::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const sizeField = findField("size", true);
  return OmVrmlNodeUtilities::isVisible(sizeField) && !OmNodeUtilities::isTemplateRegeneratorField(sizeField);
}

void OmBox::setSize(const OmVector3 &size) {
  mSize->setValue(size);
}

void OmBox::setSize(double x, double y, double z) {
  mSize->setValue(x, y, z);
}

void OmBox::setX(double x) {
  mSize->setX(x);
}

void OmBox::setY(double y) {
  mSize->setY(y);
}

void OmBox::setZ(double z) {
  mSize->setZ(z);
}

void OmBox::rescale(const OmVector3 &scale) {
  OmVector3 resizedSize = size();
  resizedSize *= scale;
  setSize(resizedSize);
}

bool OmBox::sanitizeFields() {
  if (OmFieldChecker::resetVector3IfNonPositive(this, mSize, OmVector3(1.0, 1.0, 1.0)))
    return false;

  return true;
}

void OmBox::updateSize() {
  if (!sanitizeFields())
    return;

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

QStringList OmBox::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "size";
  return fields;
}

/////////////////
// ODE objects //
/////////////////

dGeomID OmBox::createOdeGeom(dSpaceID space) {
  const OmVector3 &s1 = mSize->value();
  if (s1.x() <= 0.0 || s1.y() <= 0.0 || s1.z() <= 0.0) {
    parsingWarn(tr("'size' must be positive: construction of the Box in 'boundingObject' failed."));
    return NULL;
  }

  (void)space;
  return NULL;  // ODE is gone: no collision geoms
}

void OmBox::applyToOdeData(bool correctSolidMass) {
  if (mOdeGeom == NULL)
    return;

  if (correctSolidMass)
    applyToOdeMass();
}

const OmVector3 OmBox::scaledSize() const {
  return (mSize->value() * absoluteScale()).abs();
}

bool OmBox::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidDimensions = (mSize->x() <= 0.0 || mSize->y() <= 0.0 || mSize->z() <= 0.0);
  if (warning && invalidDimensions)
    parsingWarn(tr("All 'size' components must be positive for a Box used in a 'boundingObject'."));

  return !invalidDimensions;
}

bool OmBox::isAValidBoundingObject(bool checkOde, bool warning) const {
  const bool admissible = OmGeometry::isAValidBoundingObject(checkOde, warning);
  return admissible && isSuitableForInsertionInBoundingObject(admissible && warning);
}

/////////////////
// Ray Tracing //
/////////////////
enum IntersectedFace { FRONT_FACE, BACK_FACE, LEFT_FACE, RIGHT_FACE, TOP_FACE, BOTTOM_FACE };

int OmBox::findIntersectedFace(const OmVector3 &minBound, const OmVector3 &maxBound, const OmVector3 &intersectionPoint) {
  static const double TOLERANCE = 1e-9;

  // determine intersected face
  if (fabs(intersectionPoint.x() - maxBound.x()) < TOLERANCE)
    return FRONT_FACE;
  else if (fabs(intersectionPoint.x() - minBound.x()) < TOLERANCE)
    return BACK_FACE;
  else if (fabs(intersectionPoint.z() - minBound.z()) < TOLERANCE)
    return BOTTOM_FACE;
  else if (fabs(intersectionPoint.z() - maxBound.z()) < TOLERANCE)
    return TOP_FACE;
  else if (fabs(intersectionPoint.y() - maxBound.y()) < TOLERANCE)
    return LEFT_FACE;
  else if (fabs(intersectionPoint.y() - minBound.y()) < TOLERANCE)
    return RIGHT_FACE;

  return -1;
}

OmVector2 OmBox::computeTextureCoordinate(const OmVector3 &minBound, const OmVector3 &maxBound, const OmVector3 &point,
                                          bool nonRecursive, int intersectedFace) {
  double u, v;
  if (intersectedFace < 0)
    intersectedFace = findIntersectedFace(minBound, maxBound, point);

  OmVector3 vertex = point - minBound;
  OmVector3 s = maxBound - minBound;
  switch (intersectedFace) {
    case TOP_FACE:
      u = vertex.x() / s.x();
      v = 1 - vertex.y() / s.y();
      if (nonRecursive) {
        u = 0.25 * u + 0.50;
        v = 0.50 * v;
      }
      break;
    case BOTTOM_FACE:
      u = vertex.x() / s.x();
      v = vertex.y() / s.y();
      if (nonRecursive) {
        u = 0.25 * u;
        v = 0.50 * v;
      }
      break;
    case FRONT_FACE:
      u = vertex.y() / s.y();
      v = 1 - vertex.z() / s.z();
      if (nonRecursive) {
        u = 0.25 * u + 0.75;
        v = 0.50 * v + 0.50;
      }
      break;
    case BACK_FACE:
      u = 1 - vertex.y() / s.y();
      v = 1 - vertex.z() / s.z();
      if (nonRecursive) {
        u = 0.25 * u + 0.25;
        v = 0.50 * v + 0.50;
      }
      break;
    case LEFT_FACE:
      u = 1 - vertex.x() / s.x();
      v = 1 - vertex.z() / s.z();
      if (nonRecursive) {
        u = 0.25 * u;
        v = 0.50 * v + 0.50;
      }
      break;
    case RIGHT_FACE:
      u = vertex.x() / s.x();
      v = 1 - vertex.z() / s.z();
      if (nonRecursive) {
        u = 0.25 * u + 0.50;
        v = 0.50 * v + 0.50;
      }
      break;
    default:
      v = 0;
      u = 0;
      assert(false);
      break;
  }

  return OmVector2(u, v);
}

bool OmBox::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 localCollisionPoint;
  double collisionDistance = computeLocalCollisionPoint(localCollisionPoint, ray);
  if (collisionDistance < 0)
    // no valid collision
    return false;

  // transform point into texture coordinates in range [0..1]
  OmVector3 halfSize = scaledSize() * 0.5;
  uv = computeTextureCoordinate(-halfSize, +halfSize, localCollisionPoint, textureCoordSet == 1);
  return true;
}

double OmBox::computeDistance(const OmRay &ray) const {
  OmVector3 collisionPoint;
  return computeLocalCollisionPoint(collisionPoint, ray);
}

double OmBox::computeLocalCollisionPoint(OmVector3 &point, const OmRay &ray) const {
  OmRay localRay(ray);
  const OmPose *const pose = upperPose();
  if (pose) {
    localRay.setDirection(ray.direction() * pose->matrix());
    OmVector3 origin = pose->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
    localRay.setOrigin(origin);
    localRay.normalize();
  }

  OmVector3 halfSize = scaledSize() * 0.5;
  double tmin, tmax;
  std::pair<bool, double> result = localRay.intersects(-halfSize, halfSize, tmin, tmax);

  if (result.first && tmin >= 0) {
    // collision detected and ray origin not inside the box
    point = localRay.origin() + result.second * localRay.direction();
    return result.second;
  }

  return -1;
}

void OmBox::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->set(OmVector3(), mSize->value().length() / 2.0);
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmBox::computeFrictionDirection(const OmVector3 &normal) const {
  OmVector3 localNormal = normal * matrix().extracted3x3Matrix();
  // Find most probable face and return first friction direction in the local coordinate system
  if ((fabs(localNormal[2]) > fabs(localNormal[0])) && (fabs(localNormal[2]) > fabs(localNormal[1])))
    return OmVector3(1, 0, 0);
  else
    return OmVector3(0, 0, 1);
}
