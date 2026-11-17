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

#ifndef OM_TRIANGLE_MESH_GEOMETRY_HPP
#define OM_TRIANGLE_MESH_GEOMETRY_HPP

#include "OmGeometry.hpp"
#include "OmTriangleMeshCache.hpp"

#include <unordered_map>

class OmTriangleMesh;
class OmVector3;

typedef struct dxTriMeshData *dTriMeshDataID;

typedef std::unordered_map<OmTriangleMeshCache::TriangleMeshGeometryKey, OmTriangleMeshCache::TriangleMeshInfo,
                           OmTriangleMeshCache::TriangleMeshGeometryKeyHasher>
  OmTriangleMeshMap;

class OmTriangleMeshGeometry : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  virtual ~OmTriangleMeshGeometry() override;

  // reimplemented public functions
  void preFinalize() override;
  void createWrenObjects() override;
  void setScaleNeedUpdate() override;
  bool createOdeGeom() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;

  // Rescaling and translating
  double max(int coordinate) const;
  double min(int coordinate) const;
  double range(int coordinate) const { return max(coordinate) - min(coordinate); }

  virtual void updateTriangleMesh(bool issueWarnings = true) = 0;

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const OmRay &ray) const override;

  // friction
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override;

  // Non-recursive texture mapping
  OmVector2 nonRecursiveTextureSizeFactor() const override;

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  // OmTriangleMesh management (see OmTriangleMeshCache.hpp)
  virtual uint64_t computeHash() const = 0;

  OmTriangleMeshCache::TriangleMeshInfo createTriangleMesh();
  virtual void setTriangleMesh(OmTriangleMesh *triangleMesh) { mTriangleMesh = triangleMesh; }
  // Read access to the underlying mesh -- the Newton-backend registrar
  // walks the vertices to compute an AABB for URDF links whose collision
  // geometry is a mesh (Newton's helper module has no mesh primitive).
  OmTriangleMesh *triangleMesh() const { return mTriangleMesh; }
  // D1.4 TODO(excision): OmGeometry::triangleCount() now returns 0 (its WREN mesh is gone) and
  // is NOT virtual, so it cannot be overridden here; callers holding an OmTriangleMeshGeometry
  // can use mTriangleMesh->numberOfTriangles() via triangleMesh() instead.

  virtual void updateOdeData();

  OmTriangleMeshMap &getTriangleMeshMap() { return cTriangleMeshMap; }
  OmTriangleMeshCache::TriangleMeshGeometryKey &getMeshKey() { return mMeshKey; }
  static OmTriangleMeshMap &triangleMeshMap() { return cTriangleMeshMap; }

signals:
  void validTriangleMeshGeometryInserted();

protected:
  // All constructors are reserved for derived classes only
  OmTriangleMeshGeometry(const QString &modelName, OmTokenizer *tokenizer);
  OmTriangleMeshGeometry(const OmTriangleMeshGeometry &other);
  OmTriangleMeshGeometry(const OmNode &other);

  virtual int indexSize() const { return 0; }

  // D1.4: buildWrenMesh() no longer builds any render mesh; it survives as the shared
  // CPU triangle-mesh cache refresh hook (subclasses call buildWrenMesh(true) on field updates).
  void buildWrenMesh(bool updateCache);
  void setCcw(bool ccw);

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // triangle mesh
  OmTriangleMesh *mTriangleMesh;
  QString mTriangleMeshError;
  dTriMeshDataID mTrimeshData;

  // Hashmap containing triangle meshes, shared by all instances
  static OmTriangleMeshMap cTriangleMeshMap;

  // Hashmap key for this instance's mesh
  OmTriangleMeshCache::TriangleMeshGeometryKey mMeshKey;

private:
  OmTriangleMeshGeometry &operator=(const OmTriangleMeshGeometry &);  // non copyable
  // Only derived classes can be cloned
  OmNode *clone() const override = 0;

  void init();

  bool mCcw;  // winding order of the authored faces (CPU state)

  // ODE
  void setOdeTrimeshData();
  void clearTrimeshResources();
  bool mCorrectSolidMass;
  bool mIsOdeDataApplied;

  // ray tracing
  // compute local collision point and return the distance
  double computeLocalCollisionPoint(OmVector3 &point, int &triangleIndex, const OmRay &ray) const;
  void updateScaledCoordinates() const;
  mutable bool mScaledCoordinatesNeedUpdate;
};

#endif
