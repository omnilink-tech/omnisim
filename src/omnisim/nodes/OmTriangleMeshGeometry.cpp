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

#include "OmTriangleMeshGeometry.hpp"

#include "OmBoundingSphere.hpp"
#include "OmField.hpp"
#include "OmMatter.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmRay.hpp"
#include "OmResizeManipulator.hpp"
#include "OmSimulationState.hpp"
#include "OmTransform.hpp"
#include "OmTriangleMesh.hpp"
#include "OmWorld.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

OmTriangleMeshMap OmTriangleMeshGeometry::cTriangleMeshMap;

void OmTriangleMeshGeometry::init() {
  mTrimeshData = NULL;
  mTriangleMesh = NULL;
  mScaledCoordinatesNeedUpdate = true;
  mCorrectSolidMass = true;
  mIsOdeDataApplied = false;
  mCcw = true;
}

OmTriangleMeshGeometry::OmTriangleMeshGeometry(const QString &modelName, OmTokenizer *tokenizer) :
  OmGeometry(modelName, tokenizer) {
  init();
}

OmTriangleMeshGeometry::OmTriangleMeshGeometry(const OmTriangleMeshGeometry &other) :
  OmGeometry(other),
  mTriangleMeshError(other.mTriangleMeshError),
  mMeshKey(other.mMeshKey) {
  init();
}

OmTriangleMeshGeometry::OmTriangleMeshGeometry(const OmNode &other) : OmGeometry(other) {
  init();
}

OmTriangleMeshGeometry::~OmTriangleMeshGeometry() {
  if (mTriangleMesh) {
    OmTriangleMeshCache::releaseTriangleMesh(this);
    clearTrimeshResources();
  }
}

void OmTriangleMeshGeometry::preFinalize() {
  if (isPreFinalizedCalled())
    return;

  OmGeometry::preFinalize();

  mMeshKey.set(this);
  OmTriangleMeshCache::useTriangleMesh(this);
}

OmTriangleMeshCache::TriangleMeshInfo OmTriangleMeshGeometry::createTriangleMesh() {
  delete mTriangleMesh;
  mTriangleMesh = new OmTriangleMesh();
  updateTriangleMesh(false);
  return OmTriangleMeshCache::TriangleMeshInfo(mTriangleMesh);
}

void OmTriangleMeshGeometry::clearTrimeshResources() {
}

void OmTriangleMeshGeometry::createWrenObjects() {
  foreach (const QString &warning, mTriangleMesh->warnings())
    parsingWarn(warning);

  if (!mTriangleMeshError.isEmpty())
    parsingWarn(tr("Cannot create %1 because: \"%2\".").arg(nodeModelName()).arg(mTriangleMeshError));

  OmGeometry::createWrenObjects();

  const OmShape *shape = dynamic_cast<const OmShape *>(parentNode());
  if (shape && shape->isCastShadowsEnabled() && 3 * mTriangleMesh->numberOfTriangles() > maxIndexNumberToCastShadows()) {
    warn(tr("Too many triangles (%1) in mesh: unable to cast shadows, please reduce "
            "the number of triangles below %2 or set Shape.castShadows to "
            "FALSE to disable this warning.")
           .arg(mTriangleMesh->numberOfTriangles())
           .arg((int)(maxIndexNumberToCastShadows() / 3)));
  }
  emit wrenObjectsCreated();
}

void OmTriangleMeshGeometry::setResizeManipulatorDimensions() {
  OmVector3 scale(1.0f, 1.0f, 1.0f);

  const OmTransform *const up = upperTransform();
  if (up)
    scale *= up->absoluteScale();
  else
    return;

  resizeManipulator()->updateHandleScale(scale.ptr());
  updateResizeHandlesSize();
}

void OmTriangleMeshGeometry::setCcw(bool ccw) {
  mCcw = ccw;
}

void OmTriangleMeshGeometry::buildWrenMesh(bool updateCache) {
  // D1.4: the WREN render-mesh rebuild is gone; only the shared CPU triangle-mesh cache
  // refresh survives (it re-runs updateTriangleMesh() through the cache when the key changes).
  if (updateCache) {
    OmTriangleMeshCache::releaseTriangleMesh(this);
    mMeshKey.set(this);
    OmTriangleMeshCache::useTriangleMesh(this);
  }
}

/////////////////
// ODE objects //
/////////////////

// works only for meshes made up of triangles
dGeomID OmTriangleMeshGeometry::createOdeGeom(dSpaceID space) {
  if (!isPreFinalizedCalled())  // needed because preFinalize comes after insertion and insertion triggers ODE dGeom creation
    preFinalize();

  if (!mTriangleMesh->isValid()) {
    clearTrimeshResources();  // delete trimesh resources if any
    return NULL;
  }

  setOdeTrimeshData();

  (void)space;
  return NULL;  // ODE is gone: no collision geoms
}

void OmTriangleMeshGeometry::setOdeTrimeshData() {
  assert(mTriangleMesh->isValid());
  clearTrimeshResources();  // delete trimesh resources if any
}

// works only for meshes made up of triangles
void OmTriangleMeshGeometry::applyToOdeData(bool correctSolidMass) {
  mCorrectSolidMass = correctSolidMass;

  if (mTriangleMesh->isValid() == false)
    return;

  setOdeTrimeshData();
  if (mOdeGeom == NULL) {
    if (areOdeObjectsCreated())
      emit validTriangleMeshGeometryInserted();

    return;
  }

  if (mCorrectSolidMass)
    applyToOdeMass();

  mIsOdeDataApplied = true;
}

void OmTriangleMeshGeometry::updateOdeData() {
  if (mIsOdeDataApplied)
    applyToOdeData(mCorrectSolidMass);
}

bool OmTriangleMeshGeometry::isSuitableForInsertionInBoundingObject(bool warning) const {
  return true;
}

bool OmTriangleMeshGeometry::isAValidBoundingObject(bool checkOde, bool warning) const {
  assert(mTriangleMesh);
  return mTriangleMesh->isValid() && OmGeometry::isAValidBoundingObject(checkOde, warning);
}
/////////////////
// Ray tracing //
/////////////////

OmVector2 OmTriangleMeshGeometry::nonRecursiveTextureSizeFactor() const {
  if (mTriangleMesh->areTextureCoordinatesValid())
    // user-specified mapping
    return OmVector2(1, 1);

  // default
  return OmVector2(4, 2);
}

bool OmTriangleMeshGeometry::pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet) const {
  OmVector3 localCollisionPoint;
  int t;
  bool collisionExists = computeLocalCollisionPoint(localCollisionPoint, t, ray);
  if (!collisionExists)
    return false;

  OmVector3 v0(mTriangleMesh->vertex(t, 0, 0), mTriangleMesh->vertex(t, 0, 1), mTriangleMesh->vertex(t, 0, 2));
  OmVector3 v1(mTriangleMesh->vertex(t, 1, 0), mTriangleMesh->vertex(t, 1, 1), mTriangleMesh->vertex(t, 1, 2));
  OmVector3 v2(mTriangleMesh->vertex(t, 2, 0), mTriangleMesh->vertex(t, 2, 1), mTriangleMesh->vertex(t, 2, 2));

  const OmPose *const up = upperPose();
  if (up) {
    const OmMatrix4 &m = up->matrix();
    v0 = m * v0;
    v1 = m * v1;
    v2 = m * v2;
  }

  double u, v;
  ray.intersects(v0, v1, v2, false, u, v);
  OmVector2 tc0, tc1, tc2;
  if (textureCoordSet == 0 || mTriangleMesh->areTextureCoordinatesValid()) {
    tc0.setXy(mTriangleMesh->textureCoordinate(t, 0, 0), mTriangleMesh->textureCoordinate(t, 0, 1));
    tc1.setXy(mTriangleMesh->textureCoordinate(t, 1, 0), mTriangleMesh->textureCoordinate(t, 1, 1));
    tc2.setXy(mTriangleMesh->textureCoordinate(t, 2, 0), mTriangleMesh->textureCoordinate(t, 2, 1));
  } else {  // textureCoordSet == 1 and no user-specified mapping
    tc0.setXy(mTriangleMesh->nonRecursiveTextureCoordinate(t, 0, 0), mTriangleMesh->nonRecursiveTextureCoordinate(t, 0, 1));
    tc1.setXy(mTriangleMesh->nonRecursiveTextureCoordinate(t, 1, 0), mTriangleMesh->nonRecursiveTextureCoordinate(t, 1, 1));
    tc2.setXy(mTriangleMesh->nonRecursiveTextureCoordinate(t, 2, 0), mTriangleMesh->nonRecursiveTextureCoordinate(t, 2, 1));
  }
  uv = (1 - u - v) * tc0 + u * tc1 + v * tc2;

  return true;
}

double OmTriangleMeshGeometry::computeDistance(const OmRay &ray) const {
  OmVector3 localCollisionPoint;
  int triangleIndex;
  return computeLocalCollisionPoint(localCollisionPoint, triangleIndex, ray);
}

double OmTriangleMeshGeometry::computeLocalCollisionPoint(OmVector3 &point, int &triangleIndex, const OmRay &ray) const {
  if (!mTriangleMesh)
    return false;

  OmRay localRay(ray);
  const OmPose *const up = upperPose();
  if (up) {
    localRay.setDirection(ray.direction() * up->matrix());
    OmVector3 origin = up->matrix().pseudoInversed(ray.origin());
    origin /= absoluteScale();
    localRay.setOrigin(origin);
    localRay.normalize();
  }

  int nTriangles = mTriangleMesh->numberOfTriangles();
  double closestDistance = std::numeric_limits<double>::infinity();
  bool found = false;
  updateScaledCoordinates();
  for (int t = 0; t < nTriangles; ++t) {
    OmVector4 v0(mTriangleMesh->scaledVertex(t, 0, 0), mTriangleMesh->scaledVertex(t, 0, 1),
                 mTriangleMesh->scaledVertex(t, 0, 2), 1.0);
    OmVector4 v1(mTriangleMesh->scaledVertex(t, 1, 0), mTriangleMesh->scaledVertex(t, 1, 1),
                 mTriangleMesh->scaledVertex(t, 1, 2), 1.0);
    OmVector4 v2(mTriangleMesh->scaledVertex(t, 2, 0), mTriangleMesh->scaledVertex(t, 2, 1),
                 mTriangleMesh->scaledVertex(t, 2, 2), 1.0);

    double u, v;
    std::pair<bool, double> result = localRay.intersects(v0.toVector3(), v1.toVector3(), v2.toVector3(), true, u, v);
    if (result.first && result.second > 0.0 && result.second < closestDistance) {
      found = true;
      closestDistance = result.second;
      triangleIndex = t;
    }
  }

  if (found) {
    point = localRay.origin() + closestDistance * localRay.direction();
    return closestDistance;
  }

  return -1;
}

void OmTriangleMeshGeometry::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();
  if (mTriangleMesh->numberOfTriangles() == 0)
    return;

  // Ritter's bounding sphere approximation:
  // 1. Pick a point x from P, search a point y in P, which has the largest distance from x;
  // 2. Search a point z in P, which has the largest distance from y. set up an
  //    initial sphere B, with its centre as the midpoint of y and z, the radius as
  //    half of the distance between y and z;
  // 3. If all points in P are within sphere B, then we get a bounding sphere.
  //    Otherwise, let p be a point outside the sphere, construct a new sphere covering
  //    both point p and previous sphere. Repeat this step until all points are covered.
  // Note that steps 1. and 2. help in computing a better fitting (smaller) sphere by
  // estimating the center of the final sphere and thus reducing the bias due to the enclosed
  // vertices order.
  const int nbTriangles = mTriangleMesh->numberOfTriangles();
  OmVector3 p2(mTriangleMesh->vertex(0, 0, 0), mTriangleMesh->vertex(0, 0, 1), mTriangleMesh->vertex(0, 0, 2));
  OmVector3 p1;
  double maxDistance;  // squared distance
  for (int i = 0; i < 2; ++i) {
    maxDistance = 0.0;
    p1 = p2;
    for (int t = 0; t < nbTriangles; ++t) {
      for (int v = 0; v < 3; ++v) {
        const OmVector3 point(mTriangleMesh->vertex(t, v, 0), mTriangleMesh->vertex(t, v, 1), mTriangleMesh->vertex(t, v, 2));
        const double d = p1.distance2(point);
        if (d > maxDistance) {
          maxDistance = d;
          p2 = point;
        }
      }
    }
  }
  mBoundingSphere->set((p2 + p1) * 0.5, sqrt(maxDistance) * 0.5);

  for (int t = 0; t < nbTriangles; ++t) {
    for (int v = 0; v < 3; ++v) {
      const OmVector3 point(mTriangleMesh->vertex(t, v, 0), mTriangleMesh->vertex(t, v, 1), mTriangleMesh->vertex(t, v, 2));
      mBoundingSphere->enclose(point);
    }
  }
}

void OmTriangleMeshGeometry::updateScaledCoordinates() const {
  if (mScaledCoordinatesNeedUpdate) {
    const OmVector3 &s = absoluteScale();
    mTriangleMesh->updateScaledCoordinates(s.x(), s.y(), s.z());
    mScaledCoordinatesNeedUpdate = false;
    return;
  }
}

void OmTriangleMeshGeometry::setScaleNeedUpdate() {
  mScaledCoordinatesNeedUpdate = true;
}

/////////////////////////////////////////////////////////////
//  Resizing by pulling handles                            //
/////////////////////////////////////////////////////////////

double OmTriangleMeshGeometry::max(int coordinate) const {
  return mTriangleMesh->max(coordinate);
}

double OmTriangleMeshGeometry::min(int coordinate) const {
  return mTriangleMesh->min(coordinate);
}

////////////////////////
// Friction Direction //
////////////////////////

OmVector3 OmTriangleMeshGeometry::computeFrictionDirection(const OmVector3 &normal) const {
  parsingWarn(tr("A %1 is used in a Bounding object using an asymmetric friction. %1 does not support "
                 "asymmetric friction")
                .arg(nodeModelName()));
  return OmVector3(0, 0, 0);
}
