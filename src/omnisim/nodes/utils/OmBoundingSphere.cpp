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

#include "OmBoundingSphere.hpp"

#include "OmBaseNode.hpp"
#include "OmMatrix3.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmRotation.hpp"
#include "OmShape.hpp"
#include "OmTransform.hpp"

#include <cassert>

bool gUpdatesEnabled = false;
bool gRayTracingEnabled = false;

// cppcheck-suppress constParameterPointer
void OmBoundingSphere::enableUpdates(bool enabled, OmBoundingSphere *root) {
  gUpdatesEnabled = enabled;
  if (enabled && root) {
    QList<OmBoundingSphere *> spheres;
    spheres << root;
    while (!spheres.isEmpty()) {
      OmBoundingSphere *bs = spheres.takeFirst();
      bs->mBoundSpaceDirty = true;
      bs->mParentCoordinatesDirty = true;
      spheres << bs->mSubBoundingSpheres;
    }
  }
}

OmBoundingSphere::OmBoundingSphere(const OmBaseNode *owner) :
  mRadius(0.0),
  mParentBoundingSphere(NULL),
  mOwner(NULL),
  mGeomOwner(NULL),
  mPoseOwner(NULL),
  mBoundSpaceDirty(false),
  mParentCoordinatesDirty(true),
  mRadiusInParentCoordinates(0.0),
  mGlobalCoordinatesUpdateTime(-1.0),
  mRadiusInGlobalCoordinates(0.0) {
  setOwner(owner);
}

OmBoundingSphere::OmBoundingSphere(const OmBaseNode *owner, const OmVector3 &center, double radius) :
  mCenter(center),
  mRadius(radius),
  mParentBoundingSphere(NULL),
  mOwner(NULL),
  mGeomOwner(NULL),
  mPoseOwner(NULL),
  mBoundSpaceDirty(true),
  mParentCoordinatesDirty(true),
  mRadiusInParentCoordinates(0.0),
  mGlobalCoordinatesUpdateTime(-1.0),
  mRadiusInGlobalCoordinates(0.0) {
  setOwner(owner);
}

OmBoundingSphere::~OmBoundingSphere() {
  foreach (OmBoundingSphere *sub, mSubBoundingSpheres)
    sub->setParentBoundingSphere(NULL);
  if (mParentBoundingSphere)
    mParentBoundingSphere->removeSubBoundingSphere(this);
}

void OmBoundingSphere::setOwner(const OmBaseNode *owner) {
  mOwner = owner;
  mPoseOwner = dynamic_cast<const OmPose *>(mOwner);
  mGeomOwner = dynamic_cast<const OmGeometry *>(mOwner);
}

double OmBoundingSphere::scaledRadius() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  double globalRadius;
  OmVector3 globalCenter;
  computeSphereInGlobalCoordinates(globalCenter, globalRadius);
  return globalRadius;
}

double OmBoundingSphere::radiusInParentCoordinates() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  if (mParentCoordinatesDirty)
    recomputeSphereInParentCoordinates();
  return mRadiusInParentCoordinates;
}

const OmVector3 &OmBoundingSphere::center() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  return mCenter;
}

const OmVector3 &OmBoundingSphere::centerInParentCoordinates() {
  if (mBoundSpaceDirty)
    recomputeIfNeeded();
  if (mParentCoordinatesDirty)
    recomputeSphereInParentCoordinates();
  return mCenterInParentCoordinates;
}

void OmBoundingSphere::empty() {
  set(OmVector3(), 0.0);
}

bool OmBoundingSphere::isEmpty() const {
  return mRadius == 0.0 && mCenter == OmVector3();
}

void OmBoundingSphere::set(const OmVector3 &center, const double radius) {
  if (mCenter == center && mRadius == radius)
    return;
  mCenter = center;
  mRadius = radius;
  if (!gRayTracingEnabled && !mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void OmBoundingSphere::addSubBoundingSphereToParentNode(const OmBaseNode *node) {
  const OmBaseNode *parent = dynamic_cast<const OmBaseNode *>(node->parentNode());
  while (parent) {
    if (parent->boundingSphere()) {
      parent->boundingSphere()->addSubBoundingSphere(node->boundingSphere());
      return;
    }
    parent = dynamic_cast<const OmBaseNode *>(parent->parentNode());
  }
}

void OmBoundingSphere::addSubBoundingSphere(OmBoundingSphere *subBoundingSphere) {
  if (!subBoundingSphere || mSubBoundingSpheres.contains(subBoundingSphere))
    return;
  mSubBoundingSpheres.append(subBoundingSphere);
  subBoundingSphere->setParentBoundingSphere(this);
  if (!mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void OmBoundingSphere::removeSubBoundingSphere(OmBoundingSphere *boundingSphere) {
  if (!mSubBoundingSpheres.contains(boundingSphere))
    return;
  mSubBoundingSpheres.removeOne(boundingSphere);
  if (mSubBoundingSpheres.isEmpty())
    empty();
  else if (!mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void OmBoundingSphere::enclose(const OmVector3 &point) {
  if (isEmpty()) {
    set(point, 0);
    return;
  }
  // Test if the sphere contains the point.
  if ((point - mCenter).length() <= mRadius)
    return;

  const OmVector3 delta = mCenter - point;
  const double newRadius = (delta.length() + mRadius) / 2.0;
  set(point + delta.normalized() * newRadius, newRadius);
}

bool OmBoundingSphere::enclose(const OmBoundingSphere *other) {
  if (other->isEmpty())
    return false;

  const OmVector3 &otherCenter = const_cast<OmBoundingSphere *>(other)->centerInParentCoordinates();
  const double otherRadius = const_cast<OmBoundingSphere *>(other)->radiusInParentCoordinates();
  if (isEmpty()) {
    set(otherCenter, otherRadius);
    return true;
  }

  // Test matching centers
  if (mCenter == otherCenter) {
    if (otherRadius > mRadius) {
      set(mCenter, otherRadius);
      return true;
    }
    return false;
  }

  const OmVector3 &distanceVector = otherCenter - mCenter;
  const double distance = distanceVector.length();
  const double sum = mRadius + distance + otherRadius;

  // Other is inside the instance
  if (sum <= mRadius * 2)
    return false;
  // Other contains the instance
  if (sum <= otherRadius * 2) {
    set(otherCenter, otherRadius);
    return true;
  }

  // General case
  // compute radius of the sphere which includes the two spheres.
  const double newRadius = sum / 2.0;
  set(mCenter + distanceVector.normalized() * (newRadius - mRadius), newRadius);
  return true;
}

void OmBoundingSphere::recomputeSphereInParentCoordinates() {
  if (!mParentCoordinatesDirty)
    return;

  if (mPoseOwner != NULL) {
    const OmTransform *const t = dynamic_cast<const OmTransform *const>(mPoseOwner);
    const OmVector3 &scale = t ? t->scale() : OmVector3(1.0, 1.0, 1.0);
    mRadiusInParentCoordinates = std::max(std::max(scale.x(), scale.y()), scale.z()) * mRadius;
    mCenterInParentCoordinates = mPoseOwner->vrmlMatrix() * mCenter;
  } else {
    mRadiusInParentCoordinates = mRadius;
    mCenterInParentCoordinates = mCenter;
  }
  mParentCoordinatesDirty = false;
}

void OmBoundingSphere::computeSphereInGlobalCoordinates(OmVector3 &center, double &radius) const {
  const OmPose *upperPose = dynamic_cast<const OmPose *>(mPoseOwner);
  if (upperPose == NULL)
    upperPose = OmNodeUtilities::findUpperPose(mOwner);
  if (upperPose) {
    const OmTransform *t = dynamic_cast<const OmTransform *>(upperPose);
    if (t == NULL)
      t = upperPose->upperTransform();
    if (t) {
      const OmVector3 &scale = t->absoluteScale();
      radius = std::max(std::max(scale.x(), scale.y()), scale.z()) * mRadius;
    } else
      radius = mRadius;
    center = upperPose->matrix() * mCenter;
  } else {
    radius = mRadius;
    center = mCenter;
  }
}

void OmBoundingSphere::recomputeIfNeeded(bool dirtyOnly) {
  QSet<const OmBoundingSphere *> visited;
  recomputeIfNeededInternal(dirtyOnly, visited);
}

void OmBoundingSphere::recomputeIfNeededInternal(bool dirtyOnly, QSet<const OmBoundingSphere *> &visited) {
  // prevent infinite loop in case of SolidReference nodes
  if (visited.contains(this))
    return;
  visited << this;

  if ((dirtyOnly || gUpdatesEnabled) && !mBoundSpaceDirty)
    return;

  if (mSubBoundingSpheres.empty()) {
    // geometry or empty bounding sphere
    if (mGeomOwner)
      mGeomOwner->recomputeBoundingSphere();
    mBoundSpaceDirty = false;
    return;
  }
  const OmVector3 prevCenter = mCenter;
  const double prevRadius = mRadius;
  mCenter = OmVector3();
  mRadius = 0.0;
  bool prevState = gRayTracingEnabled;
  gRayTracingEnabled = true;
  foreach (OmBoundingSphere *sub, mSubBoundingSpheres) {
    sub->recomputeIfNeededInternal(true, visited);
    if (sub->isEmpty())
      continue;
    enclose(sub);
  }
  gRayTracingEnabled = prevState;

  if (mParentBoundingSphere && (mCenter != prevCenter || mRadius != prevRadius))
    mParentCoordinatesDirty = true;
  mBoundSpaceDirty = false;
}

void OmBoundingSphere::parentUpdateNotification() const {
  if (mParentBoundingSphere) {
    OmBoundingSphere *parent = mParentBoundingSphere;
    while (parent != NULL) {
      parent->mBoundSpaceDirty = true;
      parent->mParentCoordinatesDirty = true;
      parent = parent->mParentBoundingSphere;
    }
  }
}

void OmBoundingSphere::setOwnerMoved() {
  if (mParentBoundingSphere && !mParentCoordinatesDirty) {
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

void OmBoundingSphere::setOwnerSizeChanged() {
  assert(mGeomOwner || mPoseOwner);
  if (!mBoundSpaceDirty) {
    mBoundSpaceDirty = true;
    mParentCoordinatesDirty = true;
    if (gUpdatesEnabled)
      parentUpdateNotification();
  }
}

OmBoundingSphere::IntersectingShape OmBoundingSphere::computeIntersection(const OmRay &ray, double timeStep) {
  recomputeIfNeeded();
  if (mGlobalCoordinatesUpdateTime < timeStep) {
    computeSphereInGlobalCoordinates(mCenterInGlobalCoordinates, mRadiusInGlobalCoordinates);
    mGlobalCoordinatesUpdateTime = timeStep;
  }

  IntersectingShape res(0.0, NULL);
  if (isEmpty())
    return res;

  std::pair<bool, double> intersectionPair = ray.intersects(mCenterInGlobalCoordinates, mRadiusInGlobalCoordinates);

  if (!intersectionPair.first)
    return res;

  // This sphere is intersected, therefore, test if one sub sphere is intersected
  if (mSubBoundingSpheres.isEmpty()) {
    if (mGeomOwner != NULL) {
      const double d = mGeomOwner->computeDistance(ray);
      if (d > 0.0) {
        res.shape = dynamic_cast<OmShape *>(mGeomOwner->parentNode());
        res.distance = d;
      }
    }
  } else {
    foreach (OmBoundingSphere *sub, mSubBoundingSpheres) {
      IntersectingShape intersection = sub->computeIntersection(ray, timeStep);
      if (intersection.shape != NULL && intersection.distance > 0 &&
          (res.shape == NULL || intersection.distance < res.distance)) {
        res.shape = intersection.shape;
        res.distance = intersection.distance;
      }
    }
  }
  return res;
}
