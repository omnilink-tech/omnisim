// Copyright 2026 OmniLink
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

#ifndef OM_CLOTH_HPP
#define OM_CLOTH_HPP

//
// Description: a rectangular patch of simulated fabric, driven by newton's
//   SolverVBD; the wgpu renderer re-streams its particle positions once per
//   frame via wgpuVertexStream() (indices are fixed, uploaded once).
//
//   THE TWO THINGS THAT MAKE THIS NODE DIFFERENT FROM EVERY OTHER ONE HERE:
//
//   1. It is the first node backed by PARTICLES rather than rigid bodies.
//      Newton keeps particles in their own Model arrays with their own solver,
//      so a Cloth needs the COUPLED solver -- WorldInfo.newtonSolver
//      "mujoco+vbd". In a world that does not declare it there is no SolverVBD
//      to integrate the particles, the backend declines the registration, and
//      this node reports that as a parse warning naming the field. It still
//      renders, at its authored rest pose, so the patch can be positioned in
//      the editor -- but it never moves, and the warning is the only thing
//      that distinguishes "misconfigured" from "broken physics".
//
//   2. Its geometry changes every frame, so it does NOT go through
//      OmIndexedFaceSet / MFVec3f: the deformed surface lives in this node's
//      own CPU arrays (mPositions / mVertexNormals / mTexCoords / mIndices),
//      refreshed per step by animateMesh() and drained by the wgpu draw path.
//
//   POSE. The patch's world placement comes from its OWN `translation` /
//   `rotation` fields, not from an ancestor transform: particle positions come
//   back from Newton in world space, so the draw's model matrix is identity
//   (the same choice OmGranularGroup makes, for the same reason). A Cloth
//   nested under a Pose/Group therefore ignores that ancestor. It is
//   registered as top-level-insertable for exactly this reason.
//
//   See resources/nodes/Cloth.wrl for the authored field contract.
//

#include "OmBaseNode.hpp"

#include <vector>

class OmAbstractAppearance;
class OmAppearance;
class OmMFString;
class OmPbrAppearance;
class OmSFBool;
class OmSFColor;
class OmSFDouble;
class OmSFInt;
class OmSFNode;
class OmSFRotation;
class OmSFVector3;
// The value type, not the field type -- worldBounds() hands back plain vectors.
// OmBaseNode.hpp already forward-declares it, but this header names it in its own
// public signatures, so it declares it itself rather than inheriting the include.
class OmVector3;

class OmCloth : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmCloth(OmTokenizer *tokenizer = NULL);
  OmCloth(const OmCloth &other);
  explicit OmCloth(const OmNode &other);
  ~OmCloth() override;

  int nodeType() const override { return WB_NODE_CLOTH; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void reset(const QString &id) override;

  // The `appearance` child, resolved to whichever of the two appearance node
  // types it actually is. All three return NULL when the field is empty, which
  // is the signal to keep the legacy engine-owned phong material.
  OmAppearance *appearance() const;
  OmPbrAppearance *pbrAppearance() const;
  OmAbstractAppearance *abstractAppearance() const;

  // Read-only accessors (also used by tests).
  int dimX() const;
  int dimY() const;
  double cellX() const;
  double cellY() const;
  double mass() const;
  // TRUE when `url` names a mesh that loaded, i.e. this is a GARMENT rather
  // than a rectangular patch. The two modes are mutually exclusive and this is
  // the single predicate every mode-dependent branch below reads, so there is
  // one place where "which kind of cloth is this" is decided.
  bool isMeshCloth() const { return !mMeshVertices.empty(); }
  // Mesh cloth: one particle per mesh vertex. Grid cloth: (dimX + 1) *
  // (dimY + 1), because newton's add_cloth_grid counts CELLS, so the particle
  // grid is one larger in each direction.
  int particleCount() const;
  // True once addClothGrid() has succeeded, i.e. the patch is actually being
  // simulated. False means "rendering the authored rest pose only".
  bool isSimulated() const { return mParticleStart >= 0; }

  // ---- WORLD-SPACE BOUNDS -------------------------------------------------
  //
  // ⚠ THIS EXISTS BECAUSE AN AGENT CANNOT OTHERWISE AIM A CAMERA AT CLOTH.
  // Every other node's extent is reachable by walking `children` / `boundingObject`
  // down to a Geometry node and reading its fields -- which is exactly what the
  // harness supervisor's bounds walk does. A Cloth has NEITHER: its surface is
  // engine-owned and exists only as mPositions, so `GET /scene/tree?bounds=1`
  // reports `bounds: null` for it and `POST /scene/frame {"def": "SHEET"}` fails
  // with SUBJECT_BOUNDS_NOT_FOUND. Every viewpoint in a cloth world has had to be
  // hand-computed from the gripper's coordinates instead.
  //
  // These return the box around the CURRENT, DEFORMED surface -- the same
  // world-space particle positions the renderer drew this frame, not the authored
  // rest pose -- so a draped garment measures where it hangs, not where it was
  // spawned. Both return FALSE (and leave the outputs untouched) when there is
  // nothing to measure: no particles, or a geometry that was never built because
  // the run has no GL context. Non-finite particles are excluded rather than
  // propagated, because one NaN would otherwise make the whole box NaN and send a
  // framing computation to a NaN camera pose.
  //
  // The result is CACHED and recomputed lazily, so calling these per node per
  // scene-tree walk costs one O(particles) pass per SIMULATION STEP at worst,
  // not one per call.
  bool worldBounds(OmVector3 &minCorner, OmVector3 &maxCorner) const;
  // Centre of that box, and the radius of the sphere that circumscribes it --
  // i.e. exactly the {center, radius, bbox_min, bbox_max} quadruple the harness
  // reports for every other node.
  bool worldCenterAndRadius(OmVector3 &center, double &radius) const;

  // Per-frame entry point, called by OmDeformableFrameListener exactly as
  // OmMuscle::animateMesh / OmTrack::animateMesh are. Reads this frame's
  // particle positions back from Newton in ONE packed FFI crossing and
  // recomputes the per-vertex normals the wgpu stream reads.
  void animateMesh();

  // ---- wgpu MAIN-VIEW DRAW SOURCE ------------------------------------------
  //
  // ⚠ WHY A STREAM-FILLER RATHER THAN FOUR RAW ACCESSORS. The obvious shape --
  // hand the renderer const refs to mPositions / mVertexNormals / mTexCoords /
  // mIndices -- would ALSO hand it two facts it has no business knowing and would
  // get wrong: (1) mVertexNormals holds the RAW angle-weighted ACCUMULATION, not
  // unit vectors (uploadPositions() normalises at upload time, with a +Z fallback
  // for a fully degenerate fan), and (2) a Cloth indexes every array by PARTICLE
  // while a SoftBody indexes positions by particle and normals/UVs by RENDER
  // VERTEX. Both nodes therefore expose the same two calls and each owns its own
  // layout, so the collector has exactly one code path and no per-node special
  // case to keep in sync.
  //
  // Fills `out` with the CURRENT deformed surface in the wgpu vertex layout --
  // pos3 + norm3 + uv2, all float, 32-byte stride -- in WORLD space (which is
  // what Newton hands back and why the draw's model matrix is IDENTITY).
  // Returns false, leaving `out` untouched, when there is nothing renderable
  // yet (no particles, no topology, or arrays not yet sized).
  bool wgpuVertexStream(std::vector<unsigned char> &out) const;
  // The triangle list, addressing the vertices wgpuVertexStream() writes. Fixed
  // for the node's lifetime -- VBD moves particles, it never re-tessellates --
  // which is exactly what lets the renderer upload it once and re-stream only
  // the vertices.
  const std::vector<unsigned int> &wgpuIndices() const { return mIndices; }
  // The `castShadows` field, defaulting TRUE like the WREN renderable does.
  bool wgpuCastShadows() const;
  // The legacy engine-owned phong colour, for a Cloth that declares no
  // `appearance`. Same field and same (0.8, 0.2, 0.2) default the legacy
  // engine-owned material used.
  void wgpuFallbackDiffuse(float rgb3[3]) const;

private slots:
  // One-shot Newton registration. Deferred out of postFinalize on purpose --
  // see the long comment in the .cpp: the Newton world is opened and configured
  // lazily on the first tick, and registering earlier would open it before
  // WorldInfo's contact/coordinate declarations have been cached, silently
  // costing every Cloth-bearing world its declared friction and up axis.
  void onPhysicsStepStarted();

  // Re-runs the (possibly swapped) appearance node's setup chain so its texture
  // CPU loads are fresh for the wgpu material path. Connected to the
  // appearance's own changed() signal, exactly as OmShape connects
  // OmShape::updateAppearance, so editing a texture or a roughness in the scene
  // tree repaints the fabric without a reload.
  void updateAppearance();

private:
  OmCloth &operator=(const OmCloth &);  // non copyable
  OmNode *clone() const override { return new OmCloth(*this); }

  void init();

  // Packs the fix{Left,Right,Bottom,Top} booleans into the backend's
  // OmNewtonClothFix bitmask.
  int fixFlags() const;

  // Loads `url` through assimp into mMeshVertices / mMeshIndices, then cleans
  // it (weld duplicates, drop degenerate + duplicate triangles, drop orphan
  // vertices) and warns with counts. Called once from preFinalize. Returns
  // false and leaves both vectors empty when there is nothing usable, which is
  // what keeps isMeshCloth() honest. See the .cpp for why cloth needs a
  // DIFFERENT assimp configuration from OmMesh's.
  bool loadMeshFromUrl();

  // Fills mPositions with the UNSIMULATED rest pose -- for a mesh, the loaded
  // vertices; for a grid, local (x*cellX, y*cellY, 0) -- rotated by `rotation`
  // and offset by `translation`. Used for the initial upload, and it is all a
  // Cloth in a non-VBD world will ever show.
  void computeRestPositions();

  // Builds the triangle index list ONCE. For a MESH it is the asset's own index
  // list, verbatim; for a GRID the winding mirrors newton's
  // ModelBuilder.add_cloth_grid exactly (two triangles per cell, v0/v1/v3 and
  // v1/v2/v3, over a row-major Y-outer/X-inner vertex layout). Either way it is
  // derived from the SAME data the solver is given, which is what guarantees the
  // rendered surface and the simulated particles cannot drift out of
  // correspondence.
  void buildTopology();

  // Recomputes per-vertex normals from mPositions by ANGLE-weighted accumulation
  // over the triangle list (see the .cpp for why the weighting is not the
  // customary area one). The result stays RAW (un-normalised);
  // wgpuVertexStream() normalises at interleave time.
  void uploadPositions();

  // Rescans mPositions into the cached world-space AABB. Called lazily by the
  // two accessors above, never on the hot path.
  void refreshWorldBounds() const;

  // Fills mTexCoords (2 floats per particle). Mesh cloth: the asset's own UV
  // set when it carried one, else a planar XY projection of the rest pose.
  // Grid cloth: the (dimX, dimY) lattice normalised to [0, 1]. Called once, from
  // createWrenObjects, because the parameterisation never changes -- only the
  // positions do.
  void buildTextureCoordinates();

  // user-accessible fields
  OmSFVector3 *mTranslation;
  OmSFRotation *mRotation;
  OmMFString *mUrl;
  OmSFDouble *mDensity;
  OmSFDouble *mMeshScale;
  OmSFDouble *mPinTopBand;
  OmSFInt *mDimX;
  OmSFInt *mDimY;
  OmSFDouble *mCellX;
  OmSFDouble *mCellY;
  OmSFDouble *mMass;
  OmSFDouble *mParticleRadius;
  OmSFDouble *mTriKe;
  OmSFDouble *mTriKa;
  OmSFDouble *mTriKd;
  OmSFDouble *mEdgeKe;
  OmSFBool *mFixLeft;
  OmSFBool *mFixRight;
  OmSFBool *mFixBottom;
  OmSFBool *mFixTop;
  OmSFColor *mDiffuseColor;
  OmSFNode *mAppearance;
  OmSFBool *mCastShadows;

  // This patch's half-open particle range [mParticleStart, mParticleEnd) in the
  // Newton world's particle arrays; mParticleStart is -1 when the patch is not
  // simulated (no runtime, no VBD solver, or the registration failed). Newton
  // appends, so a second Cloth in the same world gets a non-zero start, and the
  // range is exactly what the readback slices on.
  int mParticleStart;
  int mParticleEnd;
  // Guards the one-shot registration so a failed attempt is not retried (and
  // its warning not re-emitted) on all ~30 ticks per second that follow.
  bool mRegistrationDone;

  // 3 floats per particle (tightly packed xyz, matching
  // OmNewtonBackend::snapshotParticlePositions' stride -- no repack).
  std::vector<float> mPositions;
  std::vector<float> mVertexNormals;
  std::vector<unsigned int> mIndices;
  // 2 floats per particle, uploaded ONCE (the parameterisation is fixed even
  // though the positions are not).
  std::vector<float> mTexCoords;

  // MESH CLOTH ONLY, empty for a grid. The loaded garment in its OWN local
  // frame (not world -- `translation`/`rotation` place it, and the solver
  // applies that placement itself), 3 doubles per vertex and 3 ints per
  // triangle: exactly the layout OmNewtonBackend::addClothMesh takes, so the
  // registration passes .data() with no repack.
  //
  // ⚠ THIS IS THE AUTHORITATIVE VERTEX ORDER and it is why these survive past
  // registration rather than being freed. The renderer must draw the mesh
  // before any particle readback exists, so it needs the rest pose; and the
  // particle at index i IS mesh vertex i, an identity the runtime preserves
  // (its own cleaning pass is order-preserving) and which onPhysicsStepStarted
  // asserts by comparing counts. Free these and that correspondence becomes an
  // assumption instead of a check.
  std::vector<double> mMeshVertices;
  std::vector<int> mMeshIndices;
  // MESH CLOTH ONLY, and EMPTY when the asset carried no UV set -- which is the
  // signal to fall back to the planar projection. 2 doubles per SURVIVING
  // vertex, i.e. index-aligned with mMeshVertices.
  //
  // ⚠ A UV-seam vertex is SPLIT in the file and WELDED here, because the weld is
  // on POSITION only and must stay that way: an unwelded seam has no stretch and
  // no bending across it and the garment splits open there (see Cloth.wrl).
  // The surviving particle keeps the FIRST UV it was seen with, so a seam
  // ends up one texel wide of smear rather than a tear.
  std::vector<double> mMeshUvs;

  // Cached world-space AABB of mPositions, in metres. `mutable` because the two
  // accessors are const reads that may have to refresh a stale cache -- the
  // ordinary lazy-cache shape, and the reason a scene-tree walk over a
  // cloth-heavy world does not turn into one full rescan per query.
  //
  // DIRTY is set wherever mPositions changes and NOWHERE ELSE: buildTopology()
  // (resized and zeroed), computeRestPositions() (rest pose written) and
  // animateMesh() (a frame's particles read back). VALID is the separate
  // question of whether the last refresh found anything finite to measure, and
  // it is what the accessors return.
  mutable bool mWorldBoundsDirty;
  mutable bool mWorldBoundsValid;
  mutable double mWorldBoundsMin[3];
  mutable double mWorldBoundsMax[3];
};

#endif
