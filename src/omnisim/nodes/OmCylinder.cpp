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

#include "OmCylinder.hpp"

#include "OmAffinePlane.hpp"
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
#include "OmTransform.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

#include <cmath>
#include <limits>

void OmCylinder::init() {
  mBottom = findSFBool("bottom");
  mRadius = findSFDouble("radius");
  mHeight = findSFDouble("height");

  mSide = findSFBool("side");
  mTop = findSFBool("top");
  mSubdivision = findSFInt("subdivision");

  mResizeConstraint = OmWrenAbstractResizeManipulator::X_EQUAL_Y;
}

OmCylinder::OmCylinder(OmTokenizer *tokenizer) : OmGeometry("Cylinder", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mRadius->setValueNoSignal(0.05);
    mHeight->setValueNoSignal(0.1);
  }
}

OmCylinder::OmCylinder(const OmCylinder &other) : OmGeometry(other) {
  init();
}

OmCylinder::OmCylinder(const OmNode &other) : OmGeometry(other) {
  init();
}

OmCylinder::~OmCylinder() {
}

void OmCylinder::postFinalize() {
  OmGeometry::postFinalize();

  connect(mBottom, &OmSFBool::changed, this, &OmCylinder::updateBottom);
  connect(mRadius, &OmSFDouble::changed, this, &OmCylinder::updateRadius);
  connect(mHeight, &OmSFDouble::changed, this, &OmCylinder::updateHeight);
  connect(mSide, &OmSFBool::changed, this, &OmCylinder::updateSide);
  connect(mTop, &OmSFBool::changed, this, &OmCylinder::updateTop);
  connect(mSubdivision, &OmSFInt::changed, this, &OmCylinder::updateSubdivision);
}

void OmCylinder::createWrenObjects() {
  OmGeometry::createWrenObjects();

  if (isInBoundingObject()) {
    if (mSubdivision->value() < MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION && !OmVrmlNodeUtilities::hasAUseNodeAncestor(this))
      // silently reset the subdivision on node initialization
      mSubdivision->setValue(MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION);
  }
  sanitizeFields();

  emit wrenObjectsCreated();
}

void OmCylinder::setResizeManipulatorDimensions() {
  OmVector3 scale(mRadius->value(), mRadius->value(), mHeight->value());

  const OmTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void OmCylinder::createResizeManipulator() {
  mResizeManipulator =
    new OmRegularResizeManipulator(uniqueId(), (OmWrenAbstractResizeManipulator::ResizeConstraint)mResizeConstraint);
}

bool OmCylinder::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const heightField = findField("height", true);
  const OmField *const radiusField = findField("radius", true);
  return OmVrmlNodeUtilities::isVisible(heightField) && OmVrmlNodeUtilities::isVisible(radiusField) &&
         !OmNodeUtilities::isTemplateRegeneratorField(heightField) && !OmNodeUtilities::isTemplateRegeneratorField(radiusField);
}

bool OmCylinder::sanitizeFields() {
  if (OmFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mSubdivision, 3, 1000, 3))
    return false;
  if (mSubdivision->value() < MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION && isInBoundingObject() &&
      !OmVrmlNodeUtilities::hasAUseNodeAncestor(this)) {
    parsingWarn(tr("'subdivision' value has no effect to physical 'boundingObject' geometry. "
                   "A minimum value of %2 is used for the representation.")
                  .arg(MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION));
    mSubdivision->setValue(MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION);
    return false;
  }

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mRadius, 1.0))
    return false;

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mHeight, 1.0))
    return false;

  return true;
}

void OmCylinder::rescale(const OmVector3 &scale) {
  // the cylinder is Z-aligned: the radius spans x/y, the height spans z.
  // This matches scaledRadius()/scaledHeight() (which feed dCreateCylinder) and
  // setResizeManipulatorDimensions(), all of which map (radius, radius, height) -> (x, y, z).
  if (scale.x() != 1.0)
    setRadius(radius() * scale.x());
  else if (scale.y() != 1.0)
    setRadius(radius() * scale.y());

  if (scale.z() != 1.0)
    setHeight(height() * scale.z());
}

void OmCylinder::updateBottom() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCylinder::updateRadius() {
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

void OmCylinder::updateHeight() {
  sanitizeFields();

  if (isAValidBoundingObject())
    applyToOdeData();

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCylinder::updateSide() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCylinder::updateTop() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCylinder::updateSubdivision() {
  if (!sanitizeFields())
    return;

  emit changed();
}

QStringList OmCylinder::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "radius"
         << "height"
         << "subdivision"
         << "bottom"
         << "side"
         << "top";
  return fields;
}

/////////////////
// ODE Objects //
/////////////////

dGeomID OmCylinder::createOdeGeom(dSpaceID space) {
  if (mRadius->value() <= 0.0) {
    parsingWarn(tr("'radius' must be positive when used in a 'boundingObject'."));
    return NULL;
  }

  if (mHeight->value() <= 0.0) {
    parsingWarn(tr("'height' must be positive when used in a 'boundingObject'."));
    return NULL;
  }

  (void)space;
  return NULL;  // ODE is gone: no collision geoms
}

void OmCylinder::applyToOdeData(bool correctSolidMass) {
  if (mOdeGeom == NULL)
    return;

  if (correctSolidMass)
    applyToOdeMass();
}

double OmCylinder::scaledRadius() const {
  const OmVector3 &scale = absoluteScale();
  return fabs(mRadius->value() * std::max(scale.x(), scale.y()));
}

double OmCylinder::scaledHeight() const {
  return fabs(mHeight->value() * absoluteScale().z());
}

bool OmCylinder::isSuitableForInsertionInBoundingObject(bool warning) const {
  const bool invalidRadius = mRadius->value() <= 0.0;
  const bool invalidHeight = mHeight->value() <= 0.0;
  if (warning) {
    if (invalidRadius)
      parsingWarn(tr("'radius' must be positive when used in a 'boundingObject'."));

    if (invalidHeight)
      parsingWarn(tr("'height' must be positive when used in a 'boundingObject'."));
  }

  return (!invalidHeight && !invalidRadius);
}

bool OmCylinder::isAValidBoundingObject(bool checkOde, bool warning) const {
  const bool admissible = OmGeometry::isAValidBoundingObject(checkOde, warning);
  return admissible && isSuitableForInsertionInBoundingObject(admissible && warning);
}
/////////////////
// Ray Tracing //
/////////////////

bool OmCylinder::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 collisionPoint;
  int faceIndex;
  double collisionDistance = computeLocalCollisionPoint(collisionPoint, faceIndex, ray);
  if (collisionDistance < 0)
    return false;

  // cppcheck-suppress variableScope
  double h = scaledHeight();
  double r = scaledRadius();

  double u, v;
  if (faceIndex > 0) {
    // top face or bottom face
    if (collisionPoint.x() * collisionPoint.x() + collisionPoint.y() * collisionPoint.y() > r * r)
      return false;

    u = (collisionPoint.x() + r) / (2 * r);
    v = (-collisionPoint.y() + r) / (2 * r);

    if (collisionPoint.z() < 0) {
      v = 1 - v;
    }

    if (textureCoordSet == 1) {
      u = u * 0.5;
      v = v * 0.5;
      if (faceIndex == 1)  // TOP
        u += 0.5;
      else  // BOTTOM
        v += 0.5;
    }
  } else {
    // body
    double theta = OmMathsUtilities::clampedAsin(-collisionPoint.x() / r);
    assert(!std::isnan(theta));
    if (-collisionPoint.y() > 0)
      theta = M_PI - theta;

    theta = theta - floor(theta / (2 * M_PI)) * 2 * M_PI;
    u = theta / (2 * M_PI);
    v = 1 - (collisionPoint.z() + h / 2) / h;

    if (textureCoordSet == 1) {
      u = u * 0.5;
      v = v * 0.5;
    }
  }

  uv.setXy(u, v);
  return true;
}

double OmCylinder::computeDistance(const OmRay &ray) const {
  OmVector3 collisionPoint;
  int faceIndex;
  return computeLocalCollisionPoint(collisionPoint, faceIndex, ray);
}

double OmCylinder::computeLocalCollisionPoint(OmVector3 &point, int &faceIndex, const OmRay &ray) const {
  OmVector3 direction(ray.direction());
  OmVector3 origin(ray.origin());
  const OmPose *const up = upperPose();
  if (up) {
    direction = ray.direction() * up->matrix();
    direction.normalize();
    origin = up->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
  }

  double r = scaledRadius();
  double r2 = r * r;
  double h = scaledHeight();
  double d = std::numeric_limits<double>::infinity();
  faceIndex = -1;

  // distance from body
  if (mSide->value()) {
    double a = direction.x() * direction.x() + direction.y() * direction.y();
    double b = 2 * (origin.x() * direction.x() + origin.y() * direction.y());
    double c = origin.x() * origin.x() + origin.y() * origin.y() - r2;
    double discriminant = b * b - 4 * a * c;

    // if c < 0: ray origin is inside cylinder body
    if (c >= 0 && discriminant > 0) {
      // ray intersects the sphere in two points
      discriminant = sqrt(discriminant);
      double t1 = (-b - discriminant) / (2 * a);
      double t2 = (-b + discriminant) / (2 * a);
      double z1 = origin.z() + t1 * direction.z();
      double z2 = origin.z() + t2 * direction.z();
      if (t1 > 0 && z1 >= -h / 2 && z1 <= h / 2) {
        d = t1;
        faceIndex = 0;
      } else if (t2 > 0 && z2 >= -h / 2 && z2 <= h / 2) {
        d = t2;
        faceIndex = 0;
      }
    }
  }

  // distance from top face
  if (mTop->value()) {
    std::pair<bool, double> intersection =
      OmRay(origin, direction).intersects(OmAffinePlane(OmVector3(0, 0, 1), OmVector3(0, 0, h / 2)), true);
    if (intersection.first && intersection.second > 0 && intersection.second < d) {
      OmVector3 p = origin + intersection.second * direction;
      if (p.x() * p.x() + p.y() * p.y() <= r2) {
        d = intersection.second;
        faceIndex = 1;
      }
    }
  }

  // distance from bottom face
  if (mBottom->value()) {
    std::pair<bool, double> intersection =
      OmRay(origin, direction).intersects(OmAffinePlane(OmVector3(0, 0, -1), OmVector3(0, 0, -h / 2)), true);
    if (intersection.first && intersection.second > 0 && intersection.second < d) {
      OmVector3 p = origin + intersection.second * direction;
      if (p.x() * p.x() + p.y() * p.y() <= r2) {
        d = intersection.second;
        faceIndex = 2;
      }
    }
  }

  if (d == std::numeric_limits<double>::infinity())
    return -1;

  point = origin + d * direction;
  return d;
}

void OmCylinder::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  const bool top = mTop->value();
  const bool side = mSide->value();
  const bool bottom = mBottom->value();
  const double halfHeight = mHeight->value() / 2.0;
  const double r = mRadius->value();

  if ((top + side + bottom) == 0)  // it is empty
    mBoundingSphere->empty();
  else if ((top + side + bottom) == 1 && !side) {  // just one disk
    const double center = top ? halfHeight : -halfHeight;
    mBoundingSphere->set(OmVector3(0, 0, center), r);
  } else
    mBoundingSphere->set(OmVector3(), OmVector3(r, halfHeight, 0).length());
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmCylinder::computeFrictionDirection(const OmVector3 &normal) const {
  OmVector3 localNormal = normal * matrix().extracted3x3Matrix();
  // Find most probable face and return first friction direction in the local coordinate system
  if ((fabs(localNormal[2]) > fabs(localNormal[0])) && (fabs(localNormal[2]) > fabs(localNormal[1])))  // top or bottom face
    return OmVector3(1, 0, 0);
  else  // side
    return OmVector3(0, 0, 1);
}
