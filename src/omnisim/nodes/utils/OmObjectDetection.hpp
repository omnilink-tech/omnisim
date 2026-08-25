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

#ifndef OM_OBJECT_DETECTION_HPP
#define OM_OBJECT_DETECTION_HPP

#include "OmMatrix3.hpp"
#include "OmVector3.hpp"

#include <QtCore/QList>
#include <QtCore/QVector>

class OmAffinePlane;
class OmBaseNode;
class OmNewtonBackend;
class OmSolid;

class OmObjectDetection {
public:
  enum FrustumPlane { LEFT = 0, BOTTOM, RIGHT, TOP, PARALLEL, PLANE_NUMBER };
  // Occlusion:
  // - NONE = occlusion ignored
  // - ONE_RAY = only one ray pointing at the center
  // - MULTIPLE_RAYS = multiple rays pointing at the bounding box or bounding sphere corners
  //                    (created once object size is determined)
  enum Occlusion { NONE = 0, ONE_RAY = 1, MULTIPLE_RAYS = 2 };

  OmObjectDetection(OmSolid *device, OmSolid *object, const int occlusion, const double maxRange,
                    const double horizontalFieldOfView);
  virtual ~OmObjectDetection();

  bool hasCollided() const;
  OmSolid *device() const { return mDevice; }
  OmSolid *object() const { return mObject; }
  const OmVector3 &objectSize() const { return mObjectSize; }
  const OmVector3 &objectRelativePosition() const { return mObjectRelativePosition; }
  // Number of occlusion ray segments currently carried: 0 when occlusion is
  // NONE, 1 for ONE_RAY, and 6 (bounding sphere) or 8 (bounding box) for
  // MULTIPLE_RAYS once the object's bounds are known.
  int rayCount() const { return mRays.size(); }

  // Newton raycast service consumer (OMNISIM_NEWTON_RAYCAST, default ON): answer
  // this object's occlusion rays from the live mjModel via
  // OmNewtonBackend::raycastBatch and rewrite the per-ray collision depths.
  // deviceExcludeBodies = the casting robot's Newton bodies (may be empty); the
  // target's own body and its direct solid children are excluded here,
  // replicating the device rayCollisionCallback target filter.
  // Returns false when the service could not answer -- the depths are then left
  // exactly as they were (the previous verdict stands; a caller must NOT read
  // that as "unoccluded").
  bool refreshCollisionDepthsFromNewton(const OmNewtonBackend *newton, const QVector<int> &deviceExcludeBodies);

  void deleteRays();

  // Recomputes ray position and direction and returns if the current ray is valid or can be removed.
  bool recomputeRayDirection(const OmAffinePlane *frustumPlanes);

  // Computes whether the object is contained in the given frustum.
  bool isContainedInFrustum(const OmAffinePlane *frustumPlanes);

  // Computes the frustum plane for the given device ray.
  static OmAffinePlane *computeFrustumPlanes(const OmSolid *device, const double verticalFieldOfView,
                                             const double horizontalFieldOfView, const double maxRange,
                                             const bool isPlanarProjection);

  // Return corners of the bounding box/sphere of the object
  QList<OmVector3> computeCorners() const;

private:
  static void mergeBounds(OmVector3 &referenceObjectSize, OmVector3 &referenceObjectRelativePosition,
                          const OmVector3 &addedObjectSize, const OmVector3 &addedObjectRelativePosition);
  static bool doesChildrenHaveBoundingObject(const OmSolid *solid);

  // Checks whether the object is in the bounds of the `frustumPlanes` frustum.
  //
  // @param[in] frustumPlanes Frustum of the device.
  // @param[in] boundingObject Bounding object of the target object.
  // @param[out] objectSize AABB of the target object.
  // @param[out] objectRelativePosition The object's position in respect to the device. The center of the object is calculated
  // from AABB points.
  // @param[in] rootObject If `rootObject` and `boundingObject` are not defined the method returns false.
  // @return Returns `true` if the object is inside the frustum, `false` otherwise.
  bool isWithinBounds(const OmAffinePlane *frustumPlanes, const OmBaseNode *boundingObject, OmVector3 &objectSize,
                      OmVector3 &objectRelativePosition, const OmBaseNode *rootObject = NULL);
  // Checks whether the object and its solid children are inside the `frustumPlanes` frustum.
  bool recursivelyCheckIfWithinBounds(const OmSolid *solid, const bool boundsInitialized, const OmAffinePlane *frustumPlanes);
  virtual double distance() = 0;

  void createRays(const OmVector3 &origin, const QList<OmVector3> &directions, const OmVector3 &offset);
  void updateRayDirection();

  // One occlusion ray segment, device -> target bounding volume. The carrier
  // used to be an ODE ray geom (dCreateRay + dGeomRaySet + dGeomRaySetLength,
  // read back with dGeomRayGet); it is now three plain members holding the same
  // three quantities, so the Newton raycast service casts the identical ray.
  // `direction` is NOT normalized -- neither was the vector handed to
  // dGeomRaySet, and raycastBatch normalizes internally while clamping the hit
  // to `length`.
  struct RaySegment {
    OmVector3 start;
    OmVector3 direction;
    double length = 0.0;
  };

  OmSolid *mDevice;
  OmSolid *mObject;
  OmVector3 mObjectRelativePosition;
  OmVector3 mObjectSize;
  bool mUseBoundingSphereOnly;  // used by OmCamera recognition functionality to compute a more tight fitting bounding box
  double mMaxRange;
  QList<double> mRaysCollisionDepth;  // per-ray collision depth, index-parallel to mRays
  QList<RaySegment> mRays;            // the occlusion rays checked for this object
  double mHorizontalFieldOfView;
  bool mIsOmniDirectional;  // is sensor omnidirectional (horizontal FOV > PI)
  int mOcclusion;
};

#endif
