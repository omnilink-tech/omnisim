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

#ifndef OM_GRANULAR_BED_HPP
#define OM_GRANULAR_BED_HPP

//
// Description: a rectangular volume of GRANULAR MATTER (sand, snow, gravel,
//   soil) simulated by newton's SolverImplicitMPM and two-way coupled to the
//   rigid world through SolverCoupledProxy. The wgpu renderer draws one
//   instanced sphere per particle from wgpuParticles().
//
//   See resources/nodes/GranularBed.wrl for the authored field contract, the
//   measured traps and the two reference worlds.
//
//   BUILT AS A THIRD SIBLING OF OmCloth / OmSoftBody, and the shape is
//   deliberately theirs: the same one-shot registration DEFERRED to the first
//   physics step, the same reason for deferring it, the same world-space
//   particle range, the same top-level-only placement rule and the same
//   authored `appearance` overriding the same legacy engine-owned phong
//   `diffuseColor`. OmSoftBody.hpp/.cpp is where the reasoning behind those
//   lives. What DIFFERS is written out here.
//
//   1. A DIFFERENT SOLVER, NOT A DIFFERENT PRIMITIVE ON THE SAME ONE. Cloth and
//      SoftBody are elastic continua and both ride SolverVBD. A bed's particles
//      are bound to nothing at all; they are integrated by SolverImplicitMPM
//      against a background grid under a Drucker-Prager yield surface. The
//      runtime therefore builds a SECOND particle solver alongside (not instead
//      of) the VBD one when a world carries both, and a bed's fields are a
//      friction coefficient and two yield thresholds rather than a stiffness.
//
//   2. ⛔ IT REQUIRES CUDA AND THE RUNTIME REFUSES TO FINALIZE WITHOUT IT.
//      MEASURED on CPU: 42-67 ms/step at ~3k particles (0.15-0.25x realtime),
//      351 ms/step at 8192; CUDA is nearly FLAT in N (30.9 ms at 2197, 67.4 ms
//      at 405224). A refusal, not a degradation, because a world silently
//      running at 0.2x realtime reads as a physics bug rather than as a wrong
//      device. OMNISIM_MPM_ALLOW_CPU=1 overrides.
//
//   3. THE PARTICLE COUNT IS DERIVED AND `count` IS ONLY A BUDGET. Resolution
//      comes from voxelSize and particlesPerCell; `count` caps the result and
//      the runtime bisects for the finest grid under it, coarsening voxelSize
//      and never refining it. So this node CANNOT know its own particle count
//      before registration -- unlike OmSoftBody, whose lattice is authored
//      exactly -- which is why every buffer here is sized from the range the
//      runtime returns rather than from the fields.
//
//   4. ⚠ THERE IS NO SURFACE AND NO MESH. A bed has no topology to fetch, no
//      crease angle, no UVs and no vertex normals: it is drawn as N instanced
//      unit spheres, exactly as OmGranularGroup is. Consequently `appearance`
//      supplies COLOUR ONLY -- baseColor, roughness, emissiveColor,
//      transparency -- and its texture maps are NOT sampled, because the bed
//      has no UV parameterisation for them to be sampled against. That is a
//      real limit of drawing a volume as spheres, not an unfinished path.
//
//   5. ⚠ THE BED IS INVISIBLE UNTIL THE FIRST PHYSICS STEP, ON PURPOSE. Its
//      positions are read back from the solver, and the readback needs a
//      RUNNING world (a built Model with a live State); at t=0, and on a
//      simulation paused before its first step, there is nothing to read. The
//      node declines to draw rather than inventing a rest pose -- the same
//      choice OmGranularGroup makes for a CUDA-less box, and for the same
//      reason: a phantom heap at the world origin is a scene nobody authored.
//      Storing an authored copy would cost 12 bytes per particle held for the
//      node's lifetime (4.8 MB on a 400k bed) to improve one frame.
//
//   ⚠ NOT A RE-BACKING OF OmGranularGroup, AND THE TWO MUST NOT BE MERGED.
//   GranularGroup is the older CUDA-kernel particle node: its own device
//   buffer, its own integrator, its own one-way collider sweep, and NO
//   `translation` field at all (its particles are placed in world coordinates
//   by its kernel -- a live bug that silently broke a shipped probe). This node
//   shares none of that machinery; they coexist in one world and nothing is
//   passed between them.
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
class OmSFString;
class OmSFVector3;

class OmGranularBed : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmGranularBed(OmTokenizer *tokenizer = NULL);
  OmGranularBed(const OmGranularBed &other);
  explicit OmGranularBed(const OmNode &other);
  ~OmGranularBed() override;

  int nodeType() const override { return WB_NODE_GRANULAR_BED; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  // The `appearance` child, resolved to whichever of the two appearance node
  // types it actually is. All three return NULL when the field is empty, which
  // is the signal to keep the legacy engine-owned phong `diffuseColor`.
  OmAppearance *appearance() const;
  OmPbrAppearance *pbrAppearance() const;
  OmAbstractAppearance *abstractAppearance() const;

  // Read-only field accessors (also used by tests).
  double voxelSize() const;
  double particlesPerCell() const;
  int countBudget() const;
  double density() const;
  // The particle count the SOLVER actually built, which is DERIVED (see point 3
  // in the header note) and is 0 until registration succeeds. There is
  // deliberately no "expected" counterpart: the fields do not determine it.
  int particleCount() const { return (mParticleStart >= 0) ? (mParticleEnd - mParticleStart) : 0; }
  // True once addGranularBed() has succeeded, i.e. the bed is being simulated.
  // False means inert AND invisible -- there is no rest pose to draw.
  bool isSimulated() const { return mParticleStart >= 0; }

  // ---- THE wgpu RENDER SURFACE --------------------------------------------
  //
  // THE REGISTRY, and it is OmGranularGroup's verbatim (file-static vector,
  // maintained by the ctor/dtor pair) for the same reason: OmGranularBed
  // derives from OmBaseNode, not OmSolid, so collectWorldDraws ("walk the top
  // Solids, then the root's non-Solid children") cannot reach it -- and a scene
  // walk to discover that a world has no bed would tax every world that has
  // none. anyGranularBeds() is an empty-test on a vector that already exists
  // and constructs nothing.
  static bool anyGranularBeds();
  static const std::vector<OmGranularBed *> &liveBeds();

  // World-space particle centres, TIGHTLY PACKED xyz (3 floats per particle,
  // stride 12), pointing straight at the readback buffer -- no copy -- plus the
  // one radius every particle in this bed shares.
  //
  // ⚠ STRIDE 3, NOT OmGranularGroup's 4. That node's kernel writes a per-particle
  // radius into .w; a bed's radius is uniform (it is half the final lattice
  // spacing, which the count budget may have coarsened, so it is FETCHED from
  // the runtime once at registration rather than read from the field). Handing
  // back the packed xyz the FFI already produced avoids a per-particle repack
  // every frame for a value that is the same for all of them.
  //
  // Returns FALSE, and touches no output, when this bed has no readback yet --
  // not registered, or not stepped (see point 5 in the header note).
  bool wgpuParticles(const float *&xyzOut, int &countOut, float &radiusOut) const;

  // Copy this step's particle positions solver->host into the buffer
  // wgpuParticles() hands out. PUBLIC because the wgpu collector calls it, once
  // per simulation step, exactly as it does for OmGranularGroup. Returns true
  // iff the buffer now holds this step's positions.
  bool refreshHostPositions();

  // The `castShadows` field. ⚠ Defaults FALSE here, unlike Cloth and SoftBody:
  // a bed is routinely 10^4-10^5 draws and every one of them would be
  // re-rasterised into the shadow map per light per frame, for a silhouette
  // that is already the top surface of an opaque volume.
  bool wgpuCastShadows() const;
  // The legacy engine-owned phong colour, for a bed that declares no
  // `appearance`. Same field and same default the GranularGroup draw used.
  void wgpuFallbackDiffuse(float rgb3[3]) const;

private slots:
  // One-shot Newton registration, DEFERRED out of postFinalize for exactly the
  // reason OmSoftBody defers its own -- see the long comment in the .cpp: the
  // Newton world is opened and configured lazily on the first tick, and
  // registering earlier would open it before WorldInfo's contact / coordinate
  // declarations have been cached.
  void onPhysicsStepStarted();

  // Re-runs the (possibly swapped) appearance node's setup chain so its CPU
  // texture loads are fresh for the wgpu material path. Connected to the
  // appearance's own changed() signal, as OmCloth / OmSoftBody / OmShape do.
  void updateAppearance();

private:
  OmGranularBed &operator=(const OmGranularBed &);  // non copyable
  OmNode *clone() const override { return new OmGranularBed(*this); }

  void init();

  // How many beds the runtime already holds, i.e. the index THIS bed will get
  // from the next successful addGranularBed().
  //
  // ⚠ ASKED DIRECTLY, NOT PROBED. OmSoftBody has to discover its own key by
  // calling softSurfaceTriangles() on ascending indices until one answers zero,
  // because a soft block's key is its position in a list the runtime never
  // reports the length of. A bed's list does report it, so this is one FFI call
  // and it cannot confuse "the runtime declined the query" with "index i is one
  // past the end" -- which the probe form can, and which would silently pair
  // this bed with another bed's radius.
  static int countRegisteredBeds(const OmNewtonBackend *newton);

  // user-accessible fields
  OmSFVector3 *mTranslation;
  OmSFRotation *mRotation;
  OmSFVector3 *mSize;
  OmSFDouble *mVoxelSize;
  OmSFDouble *mParticlesPerCell;
  OmSFInt *mCount;
  OmSFDouble *mDensity;
  OmSFDouble *mFriction;
  OmSFDouble *mYieldPressure;
  OmSFDouble *mYieldStress;
  OmSFDouble *mYoungModulus;
  OmSFDouble *mPoissonRatio;
  OmSFDouble *mViscosity;
  OmSFDouble *mParticleRadius;
  OmSFInt *mRigidSubsteps;
  OmSFInt *mProxyIterations;
  OmSFInt *mMaxIterations;
  OmSFDouble *mTolerance;
  OmSFString *mGridType;
  OmSFInt *mGridPadding;
  OmSFColor *mDiffuseColor;
  OmSFNode *mAppearance;
  OmSFBool *mCastShadows;

  // This bed's half-open particle range [mParticleStart, mParticleEnd) in the
  // Newton world's particle arrays; mParticleStart is -1 when the bed is not
  // simulated. Beds, cloth sheets and soft blocks all append into ONE particle
  // list, so a bed in a world that also holds a Cloth gets a non-zero start.
  int mParticleStart;
  int mParticleEnd;
  // This bed's position in the runtime's granular_beds list. -1 until registered.
  int mBedIndex;
  // The uniform per-particle radius the runtime settled on, in metres. -1 until
  // registered. NOT the `particleRadius` field: that field may be 0 (unset), and
  // the derived value depends on the FINAL lattice spacing after any coarsening
  // the `count` budget forced.
  double mParticleRadiusEffective;
  // Guards the one-shot registration so a failed attempt is not retried (and
  // its warning not re-emitted) on every tick that follows.
  bool mRegistrationDone;
  // True once refreshHostPositions() has filled mHostPositions at least once.
  // Until then the bed declines to draw (header note, point 5).
  bool mHavePositions;

  // 3 floats per particle, tightly packed xyz -- the exact stride
  // OmNewtonBackend::snapshotParticlePositions writes, so the readback is a
  // straight memcpy with no repack.
  std::vector<float> mHostPositions;
};

#endif
