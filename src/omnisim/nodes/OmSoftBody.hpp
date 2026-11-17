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

#ifndef OM_SOFT_BODY_HPP
#define OM_SOFT_BODY_HPP

//
// Description: a rectangular block of volumetric (tetrahedral FEM) soft matter,
//   driven by newton's SolverVBD; the wgpu renderer re-streams its particle
//   positions once per frame via wgpuVertexStream().
//
//   THE TWIN OF OmCloth. It is deliberately built the same way -- same coupled
//   solver ("mujoco+vbd"), same one-shot deferred registration, same per-frame
//   vertex streaming, same identity model matrix (world-space particles), same
//   authored `appearance` overriding the same legacy engine-owned phong
//   `diffuseColor` -- and OmCloth.hpp/.cpp is the place to read the reasoning
//   behind all five. Only what DIFFERS is written out here:
//
//   1. ELASTICITY LIVES IN THE TETS, not in the surface. kMu / kLambda are the
//      Lame parameters (Pa) and kDamp the viscous damping; the surface triangles
//      newton generates carry zero stiffness and exist for collision and
//      rendering only. So there is no triKe/triKa/triKd/edgeKe here, and no
//      `mass` field either: per-particle mass is DERIVED as
//      cellX * cellY * cellZ * density.
//
//   2. ⚠ THE SURFACE TOPOLOGY CANNOT BE DERIVED, so it is FETCHED ONCE AT
//      REGISTRATION and that is the whole index buffer. A cloth sheet's winding
//      is computed analytically from dimX/dimY (OmCloth::buildTopology), which is
//      what lets a Cloth render its rest pose even when it never registers. A
//      soft block's surface is the set of OPEN FACES of its tet mesh: it exists
//      only inside newton's ModelBuilder, which is CONSUMED at finalize(), so the
//      runtime snapshots it at authoring time and hands it back through
//      OmNewtonBackend::softSurfaceTriangles(). Two consequences that shape this
//      class:
//        - the render surface cannot be built in createWrenObjects() the way
//          OmCloth's is, because the indices are not known until the first
//          physics step (buildWrenMesh() is therefore called from BOTH,
//          whichever runs later);
//        - an unsimulated SoftBody renders NOTHING. There is no rest-pose
//          fallback to fall back to. It warns once, naming the field to set.
//
//   3. ⚠ IT NEEDS ITS OWN GRID INDEX, which nothing hands it. addSoftGrid()
//      returns a PARTICLE index; softSurfaceTriangles() is keyed by the block's
//      position in the runtime's append-ordered soft_grids list. The two are
//      different numbers. See countRegisteredSoftGrids() for how that is
//      resolved without a static counter.
//
//   4. ⚠ A PARTICLE IS NOT A RENDER VERTEX HERE, and OmCloth's are the same
//      thing. A sheet has no hard edges, so one smoothed normal per particle is
//      right; a block is six flat faces meeting at 90 deg, and one normal per
//      particle smears them into a bar of soap (measured on the reference
//      sponge: a bottom-edge vertex read (0, -0.625, -0.781), a 51 deg smear
//      across a hard right angle). So the surface is SPLIT by `creaseAngle` --
//      once, at registration, since the topology is fixed from then on -- into
//      one render vertex per (particle, smoothing cluster). mRenderToParticle is
//      the resulting fan-out table and mIndices addresses RENDER vertices, not
//      particles. The per-frame path is unchanged in shape: it still gathers one
//      particle position per render vertex and re-streams two buffers.
//
//   POSE. As for Cloth, the block's world placement comes from its OWN
//   `translation` / `rotation` fields and not from an ancestor transform, because
//   particle positions come back from Newton in world space and the draw's model
//   matrix is identity. ⚠ `translation` is the block's MINIMUM CORNER, not its
//   centre -- newton's convention, the opposite of Solid/Box.
//
//   ⚠ fixTop / fixBottom pin the LOCAL +Y / -Y faces, not world up and down, and
//   nothing pins the Z faces. Pin the world-top face of a Z-up block by rotating
//   +90 deg about X so local +Y maps to world +Z.
//
//   See resources/nodes/SoftBody.wrl for the authored field contract and the
//   measured numbers, and projects/samples/demos/worlds/physics/
//   newton_softbody_drop.omniworld for the reference world.
//

#include "OmBaseNode.hpp"

#include <vector>

class OmAbstractAppearance;
class OmAppearance;
class OmNewtonBackend;
class OmPbrAppearance;
class OmSFBool;
class OmSFColor;
class OmSFDouble;
class OmSFInt;
class OmSFNode;
class OmSFRotation;
class OmSFVector3;

class OmSoftBody : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmSoftBody(OmTokenizer *tokenizer = NULL);
  OmSoftBody(const OmSoftBody &other);
  explicit OmSoftBody(const OmNode &other);
  ~OmSoftBody() override;

  int nodeType() const override { return WB_NODE_SOFT_BODY; }
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
  int dimZ() const;
  double cellX() const;
  double cellY() const;
  double cellZ() const;
  double density() const;
  // Radians, clamped to [0, pi] in preFinalize. Two adjacent surface triangles
  // keep a SHARED smoothed normal while the angle between them is strictly less
  // than this -- the identical rule OmTriangleMesh::finalPass applies to an
  // IndexedFaceSet, so the field means the same thing on both nodes.
  double creaseAngle() const;
  // (dimX + 1) * (dimY + 1) * (dimZ + 1) -- newton's add_soft_grid counts CELLS,
  // so the particle lattice is one larger on every axis.
  int particleCount() const;
  // True once addSoftGrid() has succeeded, i.e. the block is actually being
  // simulated. False means "inert AND invisible" -- unlike a Cloth, there is no
  // rest-pose surface to draw without newton (see point 2 above).
  bool isSimulated() const { return mParticleStart >= 0; }
  // This block's half-open particle range [start, end) in the Newton world's
  // SHARED particle arrays (start is -1 when not simulated). Consumed by the
  // supervisor particle-stats verb (C_SUPERVISOR_NODE_PARTICLE_STATS).
  int particleRangeStart() const { return mParticleStart; }
  int particleRangeEnd() const { return mParticleEnd; }

  // Per-frame entry point, called by OmDeformableFrameListener exactly as
  // OmCloth::animateMesh is. Reads this frame's particle positions back from
  // Newton in ONE packed FFI crossing and recomputes the per-render-vertex
  // normals the wgpu stream reads.
  void animateMesh();

  // ---- wgpu MAIN-VIEW DRAW SOURCE ------------------------------------------
  //
  // Same two calls OmCloth exposes, with the same contract (see the long note on
  // OmCloth::wgpuVertexStream for WHY this is a stream-filler and not four raw
  // array accessors). The SoftBody's own layout differences are handled HERE and
  // are invisible to the renderer: positions are indexed by PARTICLE and gathered
  // through mRenderToParticle, while normals and UVs are already per-RENDER-VERTEX
  // -- and unlike OmCloth's, this node's normals are stored already normalised.
  //
  // Fills `out` with the CURRENT deformed surface in the wgpu vertex layout
  // (pos3 + norm3 + uv2 float, 32-byte stride) in WORLD space, one record per
  // render vertex. Returns false when there is nothing renderable yet.
  bool wgpuVertexStream(std::vector<unsigned char> &out) const;
  // The triangle list, addressing render vertices (i.e. the records
  // wgpuVertexStream() writes). Fixed unless `creaseAngle` is edited, which
  // rebuilds it -- and changes the vertex count, which is what the renderer's
  // size check keys on.
  const std::vector<unsigned int> &wgpuIndices() const { return mIndices; }
  // The `castShadows` field, defaulting TRUE like the WREN renderable does.
  bool wgpuCastShadows() const;
  // The legacy engine-owned phong colour, for a SoftBody that declares no
  // `appearance`. Same field and same (0.85, 0.35, 0.3) default the legacy
  // engine-owned material used.
  void wgpuFallbackDiffuse(float rgb3[3]) const;

private slots:
  // One-shot Newton registration. Deferred out of postFinalize on purpose --
  // see the long comment in the .cpp: the Newton world is opened and configured
  // lazily on the first tick, and registering earlier would open it before
  // WorldInfo's contact/coordinate declarations have been cached, silently
  // costing every deformable-bearing world its declared friction and up axis.
  void onPhysicsStepStarted();

  // Re-runs the (possibly swapped) appearance node's setup chain so its texture
  // CPU loads are fresh for the wgpu material path. Connected to the
  // appearance's own changed() signal, exactly as OmCloth and OmShape connect
  // theirs, so editing a texture or a roughness in the scene tree repaints the
  // block without a reload.
  void updateAppearance();

  // Re-splits the surface at the new crease angle and rebuilds the render
  // topology. ⚠ Unlike updateAppearance() this cannot be a re-bind: the split
  // changes the VERTEX COUNT, so the index/UV/normal buffers are re-derived
  // from mSurfaceTriangles, which is kept for exactly this reason.
  void updateCreaseAngle();

private:
  OmSoftBody &operator=(const OmSoftBody &);  // non copyable
  OmNode *clone() const override { return new OmSoftBody(*this); }

  void init();

  // Packs the fix{Left,Right,Bottom,Top} booleans into the backend's
  // OmNewtonClothFix bitmask, which the soft path reuses verbatim.
  int fixFlags() const;

  // How many soft blocks the runtime already holds, i.e. the grid index THIS
  // block will get from the next successful addSoftGrid(). See the .cpp for why
  // this is counted rather than kept in a static.
  static int countRegisteredSoftGrids(const OmNewtonBackend *newton);

  // Fills mPositions with the UNSIMULATED lattice: local
  // (x*cellX, y*cellY, z*cellZ) rotated by `rotation` and offset by
  // `translation`, in newton's Z-outer / Y-middle / X-inner order. Used for the
  // first uploaded frame, before the first readback exists.
  void computeRestPositions();

  // Pulls this block's render surface -- the open faces of its tet mesh, as
  // BLOCK-LOCAL triples -- out of the runtime ONCE into mSurfaceTriangles and
  // validates it against the particle count, then derives the render topology
  // from it. Returns false (and warns) if it cannot be trusted, in which case no
  // mesh is built. Must run AFTER a successful registration.
  bool fetchSurfaceTopology(const OmNewtonBackend *newton);

  // Turns mSurfaceTriangles (particle indices) into the RENDER topology:
  // mRenderToParticle (one entry per render vertex, naming the particle it
  // follows), mIndices (the same triangles re-addressed to render vertices) and
  // mTexCoords. Two triangles meeting at a particle share a render vertex only
  // while the angle between their REST normals is below `creaseAngle`; above it
  // the vertex is split so each side keeps its own flat normal.
  //
  // ⚠ REST POSE, ONCE. The split has to be fixed for the node's lifetime (the
  // index buffer is static-draw and the vertex count cannot move per frame), and
  // the authored lattice is the one geometry guaranteed to exist, to be
  // undeformed, and to have the block's real 90 deg edges. A split recomputed
  // from a squashed pose would make the silhouette a function of the strain.
  bool buildRenderTopology();

  // Fills mTexCoords (2 floats per RENDER vertex) with a box projection in the
  // block's own local frame -- each render vertex mapped by the face its
  // smoothing cluster belongs to. Takes the LOCAL rest lattice that
  // buildRenderTopology() has already built, both because the projection wants
  // the block's own axes and so the lattice is not walked twice. Called only from
  // there, because the parameterisation never changes; only the positions do.
  void buildTextureCoordinates(const std::vector<float> &localRest);

  // ANGLE-weighted per-render-vertex normals from a particle-indexed position
  // array, normalised in place. Angle- and not area-weighted on purpose: a
  // soft-body solver deforms triangle AREAS non-uniformly, so area weighting
  // makes a vertex normal a function of the local strain -- the quantity that
  // changes most per frame -- and the shading swims. The interior angle a face
  // subtends at a vertex is far more stable under deformation (Thurmer &
  // Wuthrich 1998; Max, JGT 4(2) 1999).
  void computeVertexNormals(const std::vector<float> &positions, std::vector<float> &normals) const;

  // Builds the render-surface buffers (mPositions sized, rest pose, normals)
  // once BOTH preconditions hold: createWrenObjects() has run, and
  // fetchSurfaceTopology() has produced an index buffer. Called from both, and
  // is a no-op until the second one lands.
  void buildWrenMesh();

  // Recomputes the per-vertex normals from mPositions (normalised in place,
  // ready for the wgpu stream).
  void uploadPositions();

  // user-accessible fields
  OmSFVector3 *mTranslation;
  OmSFRotation *mRotation;
  OmSFInt *mDimX;
  OmSFInt *mDimY;
  OmSFInt *mDimZ;
  OmSFDouble *mCellX;
  OmSFDouble *mCellY;
  OmSFDouble *mCellZ;
  OmSFDouble *mDensity;
  OmSFDouble *mKMu;
  OmSFDouble *mKLambda;
  OmSFDouble *mKDamp;
  OmSFDouble *mParticleRadius;
  OmSFBool *mFixLeft;
  OmSFBool *mFixRight;
  OmSFBool *mFixBottom;
  OmSFBool *mFixTop;
  OmSFColor *mDiffuseColor;
  OmSFNode *mAppearance;
  OmSFDouble *mCreaseAngle;
  OmSFBool *mCastShadows;

  // This block's half-open particle range [mParticleStart, mParticleEnd) in the
  // Newton world's particle arrays; mParticleStart is -1 when the block is not
  // simulated (no runtime, no VBD solver, or the registration failed). Newton
  // appends, and cloth and soft bodies share ONE particle list, so a soft block
  // in a world that also holds a Cloth gets a non-zero start.
  int mParticleStart;
  int mParticleEnd;
  // This block's position in the runtime's soft_grids list -- the key
  // softSurfaceTriangles() wants, which is NOT mParticleStart. -1 until
  // registered.
  int mGridIndex;
  // Guards the one-shot registration so a failed attempt is not retried (and
  // its warning not re-emitted) on all ~30 ticks per second that follow.
  bool mRegistrationDone;

  // 3 floats per PARTICLE (tightly packed xyz, matching
  // OmNewtonBackend::snapshotParticlePositions' stride -- no repack).
  //
  // ⚠ Sized to EVERY particle, including the interior ones no surface triangle
  // references. That is deliberate: it is the READBACK target, and the runtime
  // fills our whole half-open particle range in one packed FFI crossing, so the
  // array has to be that range for the transfer to stay a straight memcpy.
  // Nothing forces the DRAWN vertices to be that set -- mRenderToParticle is the
  // indirection that lets the two differ, so an interior particle now costs 12
  // bytes of staging and no vertex at all.
  std::vector<float> mPositions;

  // The runtime's surface, verbatim: 3 PARTICLE indices per triangle, block
  // local. ⚠ Kept for the node's lifetime because it CANNOT be re-fetched --
  // newton's ModelBuilder is consumed at finalize -- and it is what
  // buildRenderTopology() re-splits when `creaseAngle` is edited.
  std::vector<int> mSurfaceTriangles;

  // RENDER VERTICES. mRenderToParticle[v] is the particle whose position render
  // vertex v follows; several render vertices share a particle wherever a crease
  // split them. mIndices addresses THESE, not particles, and mVertexNormals /
  // mTexCoords are sized to them.
  std::vector<unsigned int> mRenderToParticle;
  std::vector<float> mVertexNormals;
  std::vector<unsigned int> mIndices;
  // 2 floats per render vertex, uploaded ONCE (the parameterisation is fixed
  // even though the positions are not).
  std::vector<float> mTexCoords;
};

#endif
