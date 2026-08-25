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

#include "OmObjectDetection.hpp"

#include "OmAffinePlane.hpp"
#include "OmBoundingSphere.hpp"
#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmCoordinate.hpp"
#include "OmCylinder.hpp"
#include "OmElevationGrid.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmSolid.hpp"
#include "OmSphere.hpp"
#include "../../physics/OmNewtonBackend.hpp"

OmObjectDetection::OmObjectDetection(OmSolid *device, OmSolid *object, const int occlusion, const double maxRange,
                                     const double horizontalFieldOfView) :
  mDevice(device),
  mObject(object),
  mObjectRelativePosition(0.0, 0.0, 0.0),
  mObjectSize(0.0, 0.0, 0.0),
  mUseBoundingSphereOnly(true),
  mMaxRange(maxRange),
  mHorizontalFieldOfView(horizontalFieldOfView),
  mIsOmniDirectional(mHorizontalFieldOfView > M_PI),
  mOcclusion(occlusion) {
  if (mOcclusion == ONE_RAY) {
    const OmVector3 devicePosition = mDevice->position();
    const OmVector3 direction = object->position() - devicePosition;
    createRays(devicePosition, QList<OmVector3>() << OmVector3(), direction);
  }
  assert(mOcclusion == NONE || mOcclusion == ONE_RAY || mOcclusion == MULTIPLE_RAYS);
};

OmObjectDetection::~OmObjectDetection() {
  deleteRays();
}

bool OmObjectDetection::hasCollided() const {
  assert(!mRaysCollisionDepth.isEmpty());
  for (int i = 0; i < mRaysCollisionDepth.size(); i++) {
    // `int(0.5 * i)` maps a ray to the object axis its slack is measured along.
    // That is exactly right for the 6-ray BOUNDING-SPHERE case (two extremities
    // per axis: 0,0,1,1,2,2) and it is simply wrong for the 8-ray BOUNDING-BOX
    // case, where corners do not belong to a single axis: i=6 and i=7 both give
    // 3, and OmVector3::operator[] is unchecked pointer arithmetic
    // (`*(&mX + index)`), so those two rays read PAST the vector.
    //
    // The out-of-bounds read is inherited verbatim from the pre-deletion ODE
    // implementation, where it was unreachable in practice because the ray
    // carrier had stopped producing rays at all. Restoring the carrier makes it
    // reachable again (Camera recognition, `occlusion 2`, a box-bounded target),
    // so it is clamped rather than shipped: the 6-ray path is unchanged
    // bit-for-bit, and the two corner rays that used to read garbage now use the
    // z extent. A principled per-corner slack (half the box diagonal, say) is a
    // behaviour question and deliberately NOT decided here.
    const int axis = qMin(2, int(0.5 * i));
    if (mRaysCollisionDepth[i] <= 0.5 * mObjectSize[axis])
      return false;
  }
  return true;
}

void OmObjectDetection::createRays(const OmVector3 &origin, const QList<OmVector3> &directions, const OmVector3 &offset) {
  QListIterator<OmVector3> it(directions);
  while (it.hasNext()) {
    // Same geometry the ODE carrier held: origin at the device, direction
    // (target centre + this corner offset) - device, length |direction|. ONE_RAY
    // passes a single zero direction so the ray aims at the target centre;
    // MULTIPLE_RAYS passes computeCorners(), i.e. 6 bounding-sphere axis
    // extremities or 8 bounding-box corners.
    RaySegment ray;
    ray.start = origin;
    ray.direction = it.next() + offset;
    ray.length = ray.direction.length();
    mRays << ray;
    mRaysCollisionDepth << 0.0;
  }
}

void OmObjectDetection::deleteRays() {
  mRays.clear();
  mRaysCollisionDepth.clear();
}

bool OmObjectDetection::refreshCollisionDepthsFromNewton(const OmNewtonBackend *newton,
                                                         const QVector<int> &deviceExcludeBodies) {
  const int n = mRays.size();
  if (n == 0)
    return true;  // occlusion NONE, or the bounds are not known yet: nothing to test
  assert(mRaysCollisionDepth.size() == mRays.size());  // createRays/deleteRays keep them index-parallel

  // replicate the device rayCollisionCallback filter at the exclusion level:
  // hits on the target itself or on its DIRECT solid children never count as
  // an occlusion (OmCamera/OmRadar::rayCollisionCallback)
  QVector<int> exclude = deviceExcludeBodies;
  if (mObject->newtonBodyIndex() >= 0)
    exclude.append(mObject->newtonBodyIndex());
  foreach (const OmSolid *child, mObject->solidChildren())
    if (child->newtonBodyIndex() >= 0)
      exclude.append(child->newtonBodyIndex());

  QVector<double> rays;
  rays.reserve(n * 7);
  QVector<double> lengths(n);
  for (int i = 0; i < n; ++i) {
    // the segment the carrier holds IS the device->target ray (re-aimed by
    // updateRayDirection() right before this call)
    const RaySegment &ray = mRays.at(i);
    lengths[i] = ray.length;
    rays << ray.start.x() << ray.start.y() << ray.start.z() << ray.direction.x() << ray.direction.y() << ray.direction.z()
         << lengths[i];
  }
  QVector<OmNewtonRayHit> hits(n);
  if (newton->raycastBatch(n, rays.constData(), hits.data(), exclude.isEmpty() ? nullptr : exclude.constData(),
                           exclude.size()) != n)
    return false;  // service could not answer -- leave the depths (and so the previous verdict) untouched
  for (int i = 0; i < n; ++i)
    // same bookkeeping the ODE near-callback did: the obstruction depth is the
    // hit's remaining distance to the target end of the segment (nearest hit ==
    // largest depth, and the ODE callback kept the largest); a miss is 0, which
    // hasCollided() reads as "clear" against its half-object-size slack.
    // raycastBatch already returns only the NEAREST non-excluded hit, so no
    // max() is needed here.
    mRaysCollisionDepth[i] = hits[i].dist >= 0.0 ? lengths[i] - hits[i].dist : 0.0;
  return true;
}

void OmObjectDetection::updateRayDirection() {
  const OmVector3 &devicePosition = mDevice->position();
  const OmVector3 offset = mObject->position() - devicePosition;
  QList<OmVector3> directions;
  if (mOcclusion == ONE_RAY)
    directions << OmVector3();
  else
    directions << computeCorners();
  if (directions.size() != mRays.size()) {
    deleteRays();
    createRays(devicePosition, directions, offset);
  } else {
    // re-aim in place, exactly as dGeomRaySet/dGeomRaySetLength did: the
    // per-ray collision depths are deliberately NOT reset here (the ODE carrier
    // kept them across a re-aim too, and the device clears them by rebuilding
    // its detection objects each refresh)
    for (int i = 0; i < mRays.size(); ++i) {
      const OmVector3 dir = directions[i] + offset;
      mRays[i].start = devicePosition;
      mRays[i].direction = dir;
      mRays[i].length = dir.length();
    }
  }
}

bool OmObjectDetection::recomputeRayDirection(const OmAffinePlane *frustumPlanes) {
  if (mOcclusion == NONE)
    return true;

  mObject->updateTransformForPhysicsStep();
  if (!isContainedInFrustum(frustumPlanes))
    return false;

  updateRayDirection();
  return true;
}

void OmObjectDetection::mergeBounds(OmVector3 &referenceObjectSize, OmVector3 &referenceObjectRelativePosition,
                                    const OmVector3 &addedObjectSize, const OmVector3 &addedObjectRelativePosition) {
  const double minX = qMin(referenceObjectRelativePosition.x() - 0.5 * referenceObjectSize.x(),
                           addedObjectRelativePosition.x() - 0.5 * addedObjectSize.x());
  const double minY = qMin(referenceObjectRelativePosition.y() - 0.5 * referenceObjectSize.y(),
                           addedObjectRelativePosition.y() - 0.5 * addedObjectSize.y());
  const double minZ = qMin(referenceObjectRelativePosition.z() - 0.5 * referenceObjectSize.z(),
                           addedObjectRelativePosition.z() - 0.5 * addedObjectSize.z());
  const double maxX = qMax(referenceObjectRelativePosition.x() + 0.5 * referenceObjectSize.x(),
                           addedObjectRelativePosition.x() + 0.5 * addedObjectSize.x());
  const double maxY = qMax(referenceObjectRelativePosition.y() + 0.5 * referenceObjectSize.y(),
                           addedObjectRelativePosition.y() + 0.5 * addedObjectSize.y());
  const double maxZ = qMax(referenceObjectRelativePosition.z() + 0.5 * referenceObjectSize.z(),
                           addedObjectRelativePosition.z() + 0.5 * addedObjectSize.z());
  referenceObjectSize.setX(maxX - minX);
  referenceObjectSize.setY(maxY - minY);
  referenceObjectSize.setZ(maxZ - minZ);
  referenceObjectRelativePosition.setX((maxX + minX) / 2.0);
  referenceObjectRelativePosition.setY((maxY + minY) / 2.0);
  referenceObjectRelativePosition.setZ((maxZ + minZ) / 2.0);
}

bool OmObjectDetection::doesChildrenHaveBoundingObject(const OmSolid *solid) {
  if (solid->boundingObject())
    return true;
  else {
    foreach (const OmSolid *sc, solid->solidChildren()) {
      if (doesChildrenHaveBoundingObject(sc))
        return true;
    }
  }
  return false;
}

bool OmObjectDetection::isWithinBounds(const OmAffinePlane *frustumPlanes, const OmBaseNode *boundingObject,
                                       OmVector3 &objectSize, OmVector3 &objectRelativePosition, const OmBaseNode *rootObject) {
  int nodeType = WB_NODE_NO_NODE;
  bool useBoundingSphere = false;
  if (boundingObject)
    nodeType = boundingObject->nodeType();
  else if (!rootObject)
    return false;
  else if (doesChildrenHaveBoundingObject(dynamic_cast<const OmSolid *>(rootObject)))
    return false;
  else
    useBoundingSphere = true;

  const OmBaseNode *referenceObject = boundingObject;
  if (useBoundingSphere)
    referenceObject = rootObject;
  const OmPose *pose = dynamic_cast<const OmPose *>(referenceObject);
  if (!pose)
    pose = referenceObject->upperPose();
  assert(pose);
  const OmMatrix3 objectRotation = pose->rotationMatrix();
  OmVector3 objectPosition = pose->position();

  if (nodeType == WB_NODE_SHAPE) {
    const OmShape *shape = static_cast<const OmShape *>(boundingObject);
    boundingObject = shape->geometry();
    return isWithinBounds(frustumPlanes, boundingObject, objectSize, objectRelativePosition);
  } else if (nodeType == WB_NODE_GROUP || nodeType == WB_NODE_POSE) {
    bool visible = false;
    const OmGroup *group = static_cast<const OmGroup *>(boundingObject);
    for (int i = 0; i < group->childCount(); ++i) {
      boundingObject = group->child(i);
      if (!visible) {
        if (isWithinBounds(frustumPlanes, boundingObject, objectSize, objectRelativePosition))
          visible = true;
      } else {
        OmVector3 newObjectSize, newObjectRelativePosition;
        if (isWithinBounds(frustumPlanes, boundingObject, newObjectSize, newObjectRelativePosition))
          mergeBounds(objectSize, objectRelativePosition, newObjectSize, newObjectRelativePosition);
      }
    }
    return visible;
  }

  const OmVector3 devicePosition = mDevice->position();
  const OmMatrix3 deviceInverseRotation = mDevice->rotationMatrix().transposed();
  if (boundingObject &&
      (nodeType == WB_NODE_BOX || nodeType == WB_NODE_INDEXED_FACE_SET || nodeType == WB_NODE_ELEVATION_GRID)) {
    QVector<OmVector3> points;
    switch (nodeType) {
      case WB_NODE_BOX: {
        const OmBox *box = static_cast<const OmBox *>(boundingObject);
        const OmVector3 size = 0.5 * box->scaledSize();
        points.append(objectRotation * OmVector3(size.x(), size.y(), size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(size.x(), size.y(), -size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(size.x(), -size.y(), size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(size.x(), -size.y(), -size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(-size.x(), size.y(), size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(-size.x(), size.y(), -size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(-size.x(), -size.y(), size.z()) + objectPosition);
        points.append(objectRotation * OmVector3(-size.x(), -size.y(), -size.z()) + objectPosition);
        break;
      }
      case WB_NODE_INDEXED_FACE_SET: {
        const OmIndexedFaceSet *indexedFaceSet = static_cast<const OmIndexedFaceSet *>(boundingObject);
        const OmCoordinate *coordinates = indexedFaceSet->coord();
        for (int i = 0; i < coordinates->pointSize(); ++i) {
          points.append(objectRotation * coordinates->point(i) + objectPosition);
        }
        break;
      }
      case WB_NODE_ELEVATION_GRID: {
        const OmElevationGrid *elevationGrid = static_cast<const OmElevationGrid *>(boundingObject);
        const double xSpacing = elevationGrid->xSpacing();
        const double ySpacing = elevationGrid->ySpacing();
        const int xDimension = elevationGrid->xDimension();
        const int yDimension = elevationGrid->yDimension();
        for (int i = 0; i < xDimension; ++i) {
          for (int j = 0; j < yDimension; ++j)
            points.append(objectRotation * OmVector3(xSpacing * i, elevationGrid->height(i + j * xDimension), ySpacing * j) +
                          objectPosition);
        }
        break;
      }
      default:
        assert(false);
    }

    // Remove points not in the frustum
    QList<OmVector3> pointsInFrustum;
    QList<OmVector3> pointsAtBack;
    int pointsInside = 0;
    bool isOnePointOutsidePlane[4] = {false, false, false, false};
    bool isOnePointOnCorrectSide[4] = {false, false, false, false};
    for (int i = 0; i < points.size(); ++i) {
      bool inside;
      if (mIsOmniDirectional) {
        if (frustumPlanes[PARALLEL].distance(points[i]) > 0) {
          // object is in front of the omnidirectional device
          inside = true;
          for (int j = 0; j < 4; ++j)
            isOnePointOnCorrectSide[j] = true;
        } else {
          // object is at the back of the omnidirectional device
          inside = false;
          double minDistance = 0.0;
          int minIndex = -1;
          for (int j = 0; j < PARALLEL; ++j) {
            const double d = frustumPlanes[j].distance(points[i]);
            if (d < 0) {
              inside = true;
              break;
            } else if (minIndex < 0 || d < minDistance) {
              minDistance = d;
              minIndex = j;
            }
          }
          if (inside) {
            for (int j = 0; j < PARALLEL; ++j)
              isOnePointOnCorrectSide[j] = true;
          } else {
            for (int j = 0; j < PARALLEL; ++j)
              isOnePointOutsidePlane[j] = true;
            points[i] = devicePosition + frustumPlanes[minIndex].vectorProjection(points[i] - devicePosition);
          }
        }
      } else if (frustumPlanes[PARALLEL].distance(points[i]) > 0) {  // object is in front of the planar device
        inside = true;
        for (int j = 0; j < PARALLEL; ++j) {
          if (frustumPlanes[j].distance(points[i]) < 0) {
            points[i] = devicePosition + frustumPlanes[j].vectorProjection(points[i] - devicePosition);
            isOnePointOutsidePlane[j] = true;
            inside = false;
          } else
            isOnePointOnCorrectSide[j] = true;
        }
      } else {
        pointsAtBack.append(points[i]);
        continue;  // use points at the back of the planar device only partly inside the frustum
      }

      if (inside)
        pointsInside++;
      pointsInFrustum.append(points[i]);
    }

    // no points in front of the device
    if (pointsInFrustum.size() == 0)
      return false;
    // no point inside the frustum
    if (pointsInside == 0) {
      // no point is in the frustum, we need to make sure the object is 'crossing' the frustum
      // either horizontally or vertically
      if (!(isOnePointOutsidePlane[RIGHT] && isOnePointOutsidePlane[LEFT] && isOnePointOnCorrectSide[BOTTOM] &&
            isOnePointOnCorrectSide[TOP]) &&
          !(isOnePointOutsidePlane[BOTTOM] && isOnePointOutsidePlane[TOP] && isOnePointOnCorrectSide[RIGHT] &&
            isOnePointOnCorrectSide[LEFT])) {
        return false;
      }
    }
    // add points at the back of the device to ensure the whole object is detected
    pointsInFrustum << pointsAtBack;
    // move the points in the device referential
    for (int i = 0; i < pointsInFrustum.size(); ++i)
      pointsInFrustum[i] = deviceInverseRotation * (pointsInFrustum[i] - devicePosition);

    double minX = pointsInFrustum[0].x();
    double maxX = minX;
    double minY = pointsInFrustum[0].y();
    double maxY = minY;
    double minZ = pointsInFrustum[0].z();
    double maxZ = minZ;
    for (int i = 1; i < pointsInFrustum.size(); ++i) {
      minX = qMin(minX, pointsInFrustum[i].x());
      maxX = qMax(maxX, pointsInFrustum[i].x());
      minY = qMin(minY, pointsInFrustum[i].y());
      maxY = qMax(maxY, pointsInFrustum[i].y());
      minZ = qMin(minZ, pointsInFrustum[i].z());
      maxZ = qMax(maxZ, pointsInFrustum[i].z());
    }
    objectRelativePosition = OmVector3((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
    objectSize.setX(maxX - minX);
    objectSize.setY(maxY - minY);
    objectSize.setZ(maxZ - minZ);
    mUseBoundingSphereOnly = false;
  } else if (useBoundingSphere ||
             (boundingObject && (nodeType == WB_NODE_SPHERE || nodeType == WB_NODE_CYLINDER || nodeType == WB_NODE_CAPSULE))) {
    double outsidePart[4] = {0.0, 0.0, 0.0, 0.0};
    if (useBoundingSphere) {
      OmBoundingSphere *boundingSphere = rootObject->boundingSphere();
      const double size = 2 * boundingSphere->scaledRadius();
      objectSize.setXyz(size, size, size);
      // correct the object center
      objectPosition = pose->matrix() * boundingSphere->center();
    } else {
      double height = 0;
      double radius = 0;
      switch (nodeType) {
        case WB_NODE_SPHERE: {
          const OmSphere *sphere = static_cast<const OmSphere *>(boundingObject);
          radius = sphere->scaledRadius();
          objectSize.setX(2 * radius);
          objectSize.setY(2 * radius);
          objectSize.setZ(2 * radius);
          break;
        }
        case WB_NODE_CYLINDER: {
          const OmCylinder *cylinder = static_cast<const OmCylinder *>(boundingObject);
          height = cylinder->scaledHeight();
          radius = cylinder->scaledRadius();
          break;
        }
        case WB_NODE_CAPSULE: {
          const OmCapsule *capsule = static_cast<const OmCapsule *>(boundingObject);
          radius = capsule->scaledRadius();
          height = capsule->scaledHeight() + 2 * radius;
          break;
        }
        default:
          assert(false);
      }
      if (nodeType == WB_NODE_CYLINDER || nodeType == WB_NODE_CAPSULE) {
        const OmMatrix3 rotation = deviceInverseRotation * objectRotation;
        const double xRange =
          fabs(rotation(0, 2) * height) + 2 * radius * sqrt(qMax(0.0, 1.0 - rotation(0, 2) * rotation(0, 2)));
        const double yRange =
          fabs(rotation(1, 2) * height) + 2 * radius * sqrt(qMax(0.0, 1.0 - rotation(1, 2) * rotation(1, 2)));
        const double zRange =
          fabs(rotation(2, 2) * height) + 2 * radius * sqrt(qMax(0.0, 1.0 - rotation(2, 2) * rotation(2, 2)));
        objectSize = OmVector3(xRange, yRange, zRange);
        mUseBoundingSphereOnly = false;
      }
    }
    // check distance between center and frustum planes
    if (!mIsOmniDirectional || frustumPlanes[PARALLEL].distance(objectPosition) < 0.0) {
      // if omnidirectional frustum, then check only if objects are in the back of the device
      bool inside = false;
      for (int j = 0; j < PARALLEL; ++j) {
        const double d = frustumPlanes[j].distance(objectPosition);
        const double halfObjectSize = objectSize[j % 2 + 1] / 2.0;
        if (mIsOmniDirectional) {
          if (d < -halfObjectSize) {  // object is completely inside
            inside = true;
            break;
          }
        } else {
          if (d < -halfObjectSize)  // object is completely outside
            return false;
          if (d < halfObjectSize)  // a part of the object is outside
            outsidePart[j] = halfObjectSize - d;
          inside = true;
        }
      }

      if (!inside)
        return false;  // object not visible in case of omnidirectional device
    }

    objectRelativePosition = deviceInverseRotation * (objectPosition - devicePosition);
    if (mHorizontalFieldOfView <= M_PI_2) {
      // do not recompute the object size and position if partly outside in case of fovX > PI
      // (a more complete computation will be needed and currently it seems to work quite well as-is)
      objectSize.setY(objectSize.y() - outsidePart[RIGHT] - outsidePart[LEFT]);
      objectSize.setZ(objectSize.z() - outsidePart[BOTTOM] - outsidePart[TOP]);
      objectRelativePosition +=
        0.5 * OmVector3(0, outsidePart[RIGHT] - outsidePart[LEFT], outsidePart[BOTTOM] - outsidePart[TOP]);
    }
  }
  return true;
}

bool OmObjectDetection::recursivelyCheckIfWithinBounds(const OmSolid *solid, const bool boundsInitialized,
                                                       const OmAffinePlane *frustumPlanes) {
  bool initialized = boundsInitialized;
  if (initialized) {
    OmVector3 newObjectSize, newObjectRelativePosition;
    if (isWithinBounds(frustumPlanes, solid->boundingObject(), newObjectSize, newObjectRelativePosition))
      mergeBounds(mObjectSize, mObjectRelativePosition, newObjectSize, newObjectRelativePosition);
  } else
    initialized = isWithinBounds(frustumPlanes, solid->boundingObject(), mObjectSize, mObjectRelativePosition, solid);
  foreach (const OmSolid *s, solid->solidChildren())
    initialized = recursivelyCheckIfWithinBounds(s, initialized, frustumPlanes);
  return initialized;
}

bool OmObjectDetection::isContainedInFrustum(const OmAffinePlane *frustumPlanes) {
  assert(mObject);
  const bool useBoundingSphereOnly = mUseBoundingSphereOnly;
  if (!recursivelyCheckIfWithinBounds(mObject, false, frustumPlanes))
    return false;
  // check distance
  if (distance() > (mMaxRange + mObjectSize.x() / 2.0))
    return false;

  if (mOcclusion != NONE && (mRays.isEmpty() || useBoundingSphereOnly != mUseBoundingSphereOnly))
    updateRayDirection();
  return true;
}

OmAffinePlane *OmObjectDetection::computeFrustumPlanes(const OmSolid *device, const double verticalFieldOfView,
                                                       const double horizontalFieldOfView, const double maxRange,
                                                       const bool isPlanarProjection) {
  const OmVector3 devicePosition = device->position();
  const OmMatrix3 deviceRotation = device->rotationMatrix();
  // construct the 4 planes defining the sides of the frustum
  const float halfFovX = horizontalFieldOfView / 2.0;
  const float halfFovY = verticalFieldOfView / 2.0;
  double x, y, z;
  if (isPlanarProjection || halfFovX < M_PI_2) {
    x = maxRange;
    y = maxRange * tan(halfFovX);
    z = maxRange * tan(halfFovY);
  } else {
    // omnidirectional sensor with horizontal FOV > PI
    const float angleY[4] = {-halfFovY, -halfFovY, halfFovY, halfFovY};
    const float angleX[4] = {halfFovX, -halfFovX, -halfFovX, halfFovX};
    for (int k = 0; k < 4; ++k) {
      const float helper = cosf(angleY[k]);
      // get x, y and z from the spherical coordinates
      if (angleY[k] > M_PI_4 || angleY[k] < -M_PI_4)
        y = maxRange * cosf(angleY[k] + M_PI_2) * sinf(angleX[k]);
      else
        y = maxRange * helper * sinf(angleX[k]);
      z = maxRange * sinf(angleY[k]);
      x = maxRange * helper * cosf(angleX[k]);
    }
  }
  const OmVector3 topRightCorner = devicePosition + deviceRotation * OmVector3(x, -y, z);
  const OmVector3 topLeftCorner = devicePosition + deviceRotation * OmVector3(x, y, z);
  const OmVector3 bottomRightCorner = devicePosition + deviceRotation * OmVector3(x, -y, -z);
  const OmVector3 bottomLeftCorner = devicePosition + deviceRotation * OmVector3(x, y, -z);
  OmAffinePlane *planes = new OmAffinePlane[PLANE_NUMBER];
  planes[RIGHT] = OmAffinePlane(devicePosition, topRightCorner, bottomRightCorner);              // right plane
  planes[BOTTOM] = OmAffinePlane(devicePosition, bottomRightCorner, bottomLeftCorner);           // bottom plane
  planes[LEFT] = OmAffinePlane(devicePosition, bottomLeftCorner, topLeftCorner);                 // left plane
  planes[TOP] = OmAffinePlane(devicePosition, topLeftCorner, topRightCorner);                    // top plane
  planes[PARALLEL] = OmAffinePlane(deviceRotation * OmVector3(maxRange, 0, 0), devicePosition);  // device plane
  return planes;
}

QList<OmVector3> OmObjectDetection::computeCorners() const {
  QList<OmVector3> points;
  OmVector3 size = 0.5 * mObjectSize;
  if (mUseBoundingSphereOnly) {  // 6 rays for bounding sphere
    points << OmVector3(size.x(), 0, 0);
    points << OmVector3(-size.x(), 0, 0);
    points << OmVector3(0, size.y(), 0);
    points << OmVector3(0, -size.y(), 0);
    points << OmVector3(0, 0, size.z());
    points << OmVector3(0, 0, -size.z());
  } else {  // 8 rays for bounding box
    points << OmVector3(size.x(), size.y(), size.z());
    points << OmVector3(-size.x(), size.y(), size.z());
    points << OmVector3(size.x(), -size.y(), size.z());
    points << OmVector3(-size.x(), -size.y(), size.z());
    points << OmVector3(size.x(), size.y(), -size.z());
    points << OmVector3(-size.x(), size.y(), -size.z());
    points << OmVector3(size.x(), -size.y(), -size.z());
    points << OmVector3(-size.x(), -size.y(), -size.z());
  }
  return points;
}
