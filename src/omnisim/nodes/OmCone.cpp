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

#include "OmCone.hpp"

#include "OmAffinePlane.hpp"
#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSFBool.hpp"
#include "OmSFInt.hpp"
#include "OmSimulationState.hpp"
#include "OmTransform.hpp"
#include "OmVector2.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include <cmath>

void OmCone::init() {
  mBottomRadius = findSFDouble("bottomRadius");
  mHeight = findSFDouble("height");
  mSide = findSFBool("side");
  mBottom = findSFBool("bottom");
  mSubdivision = findSFInt("subdivision");

  mResizeConstraint = OmWrenAbstractResizeManipulator::X_EQUAL_Y;
}

OmCone::OmCone(OmTokenizer *tokenizer) : OmGeometry("Cone", tokenizer) {
  init();
  if (tokenizer == NULL) {
    mBottomRadius->setValueNoSignal(0.05);
    mHeight->setValueNoSignal(0.1);
  }
}

OmCone::OmCone(const OmCone &other) : OmGeometry(other) {
  init();
}

OmCone::OmCone(const OmNode &other) : OmGeometry(other) {
  init();
}

OmCone::~OmCone() {
}

void OmCone::postFinalize() {
  OmGeometry::postFinalize();

  connect(mBottomRadius, &OmSFDouble::changed, this, &OmCone::updateBottomRadius);
  connect(mHeight, &OmSFDouble::changed, this, &OmCone::updateHeight);
  connect(mSide, &OmSFBool::changed, this, &OmCone::updateSide);
  connect(mBottom, &OmSFBool::changed, this, &OmCone::updateBottom);
  connect(mSubdivision, &OmSFInt::changed, this, &OmCone::updateSubdivision);
}

void OmCone::createWrenObjects() {
  OmGeometry::createWrenObjects();

  sanitizeFields();

  emit wrenObjectsCreated();
}

void OmCone::setResizeManipulatorDimensions() {
  OmVector3 scale(mBottomRadius->value(), mBottomRadius->value(), mHeight->value());

  const OmTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void OmCone::createResizeManipulator() {
  mResizeManipulator = new OmRegularResizeManipulator(uniqueId(), OmWrenAbstractResizeManipulator::ResizeConstraint::X_EQUAL_Y);
}

bool OmCone::areSizeFieldsVisibleAndNotRegenerator() const {
  const OmField *const heightField = findField("height", true);
  const OmField *const radiusField = findField("bottomRadius", true);
  return OmVrmlNodeUtilities::isVisible(heightField) && OmVrmlNodeUtilities::isVisible(radiusField) &&
         !OmNodeUtilities::isTemplateRegeneratorField(heightField) && !OmNodeUtilities::isTemplateRegeneratorField(radiusField);
}

bool OmCone::sanitizeFields() {
  if (isInBoundingObject())
    return false;

  if (OmFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mSubdivision, 3, 1000, 3))
    return false;

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mBottomRadius, 1.0))
    return false;

  if (OmFieldChecker::resetDoubleIfNonPositive(this, mHeight, 1.0))
    return false;

  return true;
}

void OmCone::rescale(const OmVector3 &scale) {
  // the cone is Z-aligned: the bottom radius spans x/y, the height spans z.
  // Cf. setResizeManipulatorDimensions(), which maps
  // (bottomRadius, bottomRadius, height) -> (x, y, z), recomputeBoundingSphere(), whose
  // centre slides along z, and computeLocalCollisionPoint(), whose cone equation is
  // x^2 + y^2 = k * z^2.
  // NOTE: scaledHeight()/scaledBottomRadius() below still use the pre-R2022b Y-up axes and
  // therefore disagree with the geometry above; that is a separate bug on the ray-tracing
  // path, deliberately left untouched here (see report).
  if (scale.x() != 1.0)
    setBottomRadius(bottomRadius() * scale.x());
  else if (scale.y() != 1.0)
    setBottomRadius(bottomRadius() * scale.y());

  if (scale.z() != 1.0)
    setHeight(height() * scale.z());
}

double OmCone::height() const {
  return mHeight->value();
}

double OmCone::bottomRadius() const {
  return mBottomRadius->value();
}

void OmCone::setHeight(double h) {
  mHeight->setValue(h);
}

void OmCone::setBottomRadius(double r) {
  mBottomRadius->setValue(r);
}

void OmCone::updateBottomRadius() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCone::updateHeight() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCone::updateSide() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCone::updateBottom() {
  if (!sanitizeFields())
    return;

  if (mBoundingSphere && !isInBoundingObject())
    mBoundingSphere->setOwnerSizeChanged();

  if (resizeManipulator() && resizeManipulator()->isAttached())
    setResizeManipulatorDimensions();

  emit changed();
}

void OmCone::updateSubdivision() {
  if (!sanitizeFields())
    return;

  emit changed();
}

double OmCone::scaledHeight() const {
  return fabs(mHeight->value() * absoluteScale().y());
}

double OmCone::scaledBottomRadius() const {
  const OmVector3 &scale = absoluteScale();
  return fabs(mBottomRadius->value() * std::max(scale.x(), scale.z()));
}

QStringList OmCone::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "bottomRadius"
         << "height"
         << "subdivision"
         << "bottom"
         << "side";
  return fields;
}

/////////////////
// Ray Tracing //
/////////////////

bool OmCone::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 localCollisionPoint;
  const double collisionDistance = computeLocalCollisionPoint(localCollisionPoint, ray);
  if (collisionDistance < 0.0)
    return false;

  const double h = scaledHeight();
  const double r = scaledBottomRadius() * (0.5 - localCollisionPoint.z() / h);

  double u, v;
  if (localCollisionPoint.z() == -h / 2.0) {
    // bottom face
    if (localCollisionPoint.x() * localCollisionPoint.x() + localCollisionPoint.y() * localCollisionPoint.y() > r * r)
      return false;

    u = (localCollisionPoint.x() + r) / (2.0 * r);
    v = 1.0 - (-localCollisionPoint.y() + r) / (2.0 * r);

    if (textureCoordSet == 1) {
      u = u * 0.5 + 0.5;
    }

  } else {
    // body
    double theta = atan(localCollisionPoint.x() / -localCollisionPoint.y());
    theta -= floor(theta / (2.0 * M_PI)) * 2.0 * M_PI;
    if (theta < M_PI && localCollisionPoint.x() > 0 && -localCollisionPoint.y() > 0.0) {
      theta += M_PI;
    } else if (theta < M_PI && localCollisionPoint.x() > 0.0 && -localCollisionPoint.y() < 0.0) {
      theta += M_PI;
    } else if (theta > M_PI && localCollisionPoint.x() < 0.0 && -localCollisionPoint.y() > 0.0) {
      theta -= M_PI;
    } else if (theta > M_PI && localCollisionPoint.x() < 0.0 && -localCollisionPoint.y() < 0.0) {
      theta -= M_PI;
    }
    u = theta / (2.0 * M_PI);
    v = 1.0 - (localCollisionPoint.z() + h / 2.0) / h;

    if (textureCoordSet == 1)
      u *= 0.5;
  }

  uv.setXy(u, v);
  return true;
}

double OmCone::computeDistance(const OmRay &ray) const {
  OmVector3 collisionPoint;
  return computeLocalCollisionPoint(collisionPoint, ray);
}

double OmCone::computeLocalCollisionPoint(OmVector3 &point, const OmRay &ray) const {
  OmVector3 direction(ray.direction());
  OmVector3 origin(ray.origin());

  const OmPose *const up = upperPose();
  if (up) {
    direction = ray.direction() * up->matrix();
    direction.normalize();
    origin = up->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
  }

  const double radius = scaledBottomRadius();
  const double radius2 = radius * radius;
  const double h = scaledHeight();
  double d = std::numeric_limits<double>::infinity();

  // distance from body
  if (mSide->value()) {
    const double k = radius2 / (h * h);
    const double halfH = h / 2.0;
    const double o = origin.z() - halfH;
    const double a = direction.x() * direction.x() + direction.y() * direction.y() - k * direction.z() * direction.z();
    const double b = 2.0 * (origin.x() * direction.x() + origin.y() * direction.y() - k * o * direction.z());
    const double c = origin.x() * origin.x() + origin.y() * origin.y() - k * o * o;
    double discriminant = b * b - 4.0 * a * c;

    // if c < 0: ray origin is inside the cone body
    if (c >= 0 && discriminant > 0) {
      // ray intersects the sphere in two points
      discriminant = sqrt(discriminant);
      const double t1 = (-b - discriminant) / (2 * a);
      const double t2 = (-b + discriminant) / (2 * a);
      const double z1 = origin.z() + t1 * direction.z();
      const double z2 = origin.z() + t2 * direction.z();
      if (t1 > 0.0 && z1 >= -halfH && z1 <= halfH)
        d = t1;
      else if (t2 > 0.0 && z2 >= -halfH && z2 <= halfH)
        d = t2;
    }
  }

  // distance from bottom face
  if (mBottom->value()) {
    std::pair<bool, double> intersection =
      OmRay(origin, direction).intersects(OmAffinePlane(OmVector3(0.0, 0.0, -1.0), OmVector3(0.0, 0.0, -h / 2.0)), true);
    if (intersection.first && intersection.second > 0.0 && intersection.second < d) {
      const OmVector3 &p = origin + intersection.second * direction;
      if (p.x() * p.x() + p.y() * p.y() <= radius2) {
        d = intersection.second;
      }
    }
  }

  if (d == std::numeric_limits<double>::infinity())
    return -1;

  point = origin + d * direction;
  return d;
}

void OmCone::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  const bool side = mSide->value();
  const double r = mBottomRadius->value();
  const double h = mHeight->value();
  const double halfHeight = h / 2.0;

  if (!side && !mBottom->value())  // it is empty
    mBoundingSphere->empty();
  else if (!side || h <= r)  // consider it as disk
    mBoundingSphere->set(OmVector3(0, 0, -halfHeight), r);
  else {
    const double newRadius = halfHeight + r * r / (2 * h);
    mBoundingSphere->set(OmVector3(0, 0, halfHeight - newRadius), newRadius);
  }
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmCone::computeFrictionDirection(const OmVector3 &normal) const {
  parsingWarn(
    tr("A Cone is used in a Bounding object using an asymmetric friction. Cone does not support asymmetric friction"));
  return OmVector3(0, 0, 0);
}
