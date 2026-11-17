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

#include "OmSoftBody.hpp"

#include "OmAppearance.hpp"
#include "OmLog.hpp"
#include "OmMatrix3.hpp"
// OmNewtonBackend.hpp also pulls in OmPhysicsBackend.hpp, which is where the
// OmPhysicsBackendRegistry namespace lives -- there is no separate registry
// header, and OmCloth.cpp / OmSolid.cpp get it the same way.
#include "OmNewtonBackend.hpp"
#include "OmPbrAppearance.hpp"
#include "OmQuaternion.hpp"
#include "OmRotation.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFNode.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmVector3.hpp"
#include "OmDeformableFrameListener.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

// What a 0 in the .wrl becomes. ⚠ These are the RUNTIME's own defaults and must
// stay in step with omnisim_newton_runtime.py's World.add_soft_grid signature --
// they are duplicated here rather than passed through as 0 because 0 is a
// perfectly legal Lame parameter on the runtime side (see the block comment in
// onPhysicsStepStarted).
static const double DEFAULT_DENSITY = 1000.0;
static const double DEFAULT_K_MU = 1.0e4;
static const double DEFAULT_K_LAMBDA = 1.0e4;
static const double DEFAULT_K_DAMP = 1.0;
static const double DEFAULT_PARTICLE_RADIUS = 0.01;

// Union-find over a SMALL local set -- the triangle corners meeting at ONE
// particle, which on a lattice is a handful and on any surface is bounded by the
// vertex valence. Flat parent array plus path halving is the whole
// implementation; nothing here is worth a class.
static int softBodyFindRoot(std::vector<int> &parent, int i) {
  while (parent[i] != i) {
    parent[i] = parent[parent[i]];
    i = parent[i];
  }
  return i;
}

void OmSoftBody::init() {
  mTranslation = findSFVector3("translation");
  mRotation = findSFRotation("rotation");
  mDimX = findSFInt("dimX");
  mDimY = findSFInt("dimY");
  mDimZ = findSFInt("dimZ");
  mCellX = findSFDouble("cellX");
  mCellY = findSFDouble("cellY");
  mCellZ = findSFDouble("cellZ");
  mDensity = findSFDouble("density");
  mKMu = findSFDouble("kMu");
  mKLambda = findSFDouble("kLambda");
  mKDamp = findSFDouble("kDamp");
  mParticleRadius = findSFDouble("particleRadius");
  mFixLeft = findSFBool("fixLeft");
  mFixRight = findSFBool("fixRight");
  mFixBottom = findSFBool("fixBottom");
  mFixTop = findSFBool("fixTop");
  mDiffuseColor = findSFColor("diffuseColor");
  mAppearance = findSFNode("appearance");
  mCreaseAngle = findSFDouble("creaseAngle");
  mCastShadows = findSFBool("castShadows");

  mParticleStart = -1;
  mParticleEnd = -1;
  mGridIndex = -1;
  mRegistrationDone = false;
  mSurfaceTriangles.clear();
  mRenderToParticle.clear();
  mTexCoords.clear();
}

OmAppearance *OmSoftBody::appearance() const {
  return mAppearance ? dynamic_cast<OmAppearance *>(mAppearance->value()) : NULL;
}

OmPbrAppearance *OmSoftBody::pbrAppearance() const {
  return mAppearance ? dynamic_cast<OmPbrAppearance *>(mAppearance->value()) : NULL;
}

OmAbstractAppearance *OmSoftBody::abstractAppearance() const {
  return mAppearance ? dynamic_cast<OmAbstractAppearance *>(mAppearance->value()) : NULL;
}

OmSoftBody::OmSoftBody(OmTokenizer *tokenizer) : OmBaseNode("SoftBody", tokenizer) {
  init();
}

OmSoftBody::OmSoftBody(const OmSoftBody &other) : OmBaseNode(other) {
  init();
}

OmSoftBody::OmSoftBody(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmSoftBody::~OmSoftBody() {
  OmDeformableFrameListener::instance()->unsubscribeSoftBody(this);
}

int OmSoftBody::dimX() const {
  return mDimX ? mDimX->value() : 0;
}

int OmSoftBody::dimY() const {
  return mDimY ? mDimY->value() : 0;
}

int OmSoftBody::dimZ() const {
  return mDimZ ? mDimZ->value() : 0;
}

double OmSoftBody::cellX() const {
  return mCellX ? mCellX->value() : 0.0;
}

double OmSoftBody::cellY() const {
  return mCellY ? mCellY->value() : 0.0;
}

double OmSoftBody::cellZ() const {
  return mCellZ ? mCellZ->value() : 0.0;
}

double OmSoftBody::density() const {
  return mDensity ? mDensity->value() : 0.0;
}

double OmSoftBody::creaseAngle() const {
  // 0.8 rad is the .wrl default and the value 2168 nodes in this tree already
  // carry; it is repeated here only for the missing-field case, which is the
  // same shape as every other accessor above.
  return mCreaseAngle ? mCreaseAngle->value() : 0.8;
}

int OmSoftBody::particleCount() const {
  // newton's add_soft_grid counts CELLS, so the particle lattice is one larger
  // on every axis. Getting this wrong is the classic off-by-one, and it is worse
  // here than it is for a sheet: the surface indices come back BLOCK-LOCAL and
  // are validated against this number, so an undersized count silently rejects a
  // perfectly good mesh.
  const int nx = dimX();
  const int ny = dimY();
  const int nz = dimZ();
  if (nx <= 0 || ny <= 0 || nz <= 0)
    return 0;
  return (nx + 1) * (ny + 1) * (nz + 1);
}

int OmSoftBody::fixFlags() const {
  // The soft path reuses OmNewtonClothFix verbatim -- the backend expands the
  // same four bits into add_soft_grid's four booleans. ⚠ Top/Bottom are the
  // LOCAL +Y / -Y faces, not world up and down; nothing pins the Z faces.
  int flags = OmClothFixNone;
  if (mFixLeft && mFixLeft->value())
    flags |= OmClothFixLeft;
  if (mFixRight && mFixRight->value())
    flags |= OmClothFixRight;
  if (mFixBottom && mFixBottom->value())
    flags |= OmClothFixBottom;
  if (mFixTop && mFixTop->value())
    flags |= OmClothFixTop;
  return flags;
}

void OmSoftBody::downloadAssets() {
  OmBaseNode::downloadAssets();
  // The appearance owns ImageTexture children whose `url` may be remote; without
  // this the first frame renders before the texture exists and never repaints.
  if (abstractAppearance())
    abstractAppearance()->downloadAssets();
}

void OmSoftBody::preFinalize() {
  OmBaseNode::preFinalize();

  // Same forwarding OmCloth::preFinalize and OmShape::preFinalize do, and for
  // the same reason: an appearance is an ordinary node that has to run its own
  // finalize chain (OmPbrAppearance::preFinalize is where the BRDF LUT is baked
  // and every map is resolved).
  if (abstractAppearance())
    abstractAppearance()->preFinalize();

  // Clamp to a lattice that can actually be built. Every one of these would
  // otherwise reach newton as a degenerate block (zero tetrahedra, zero-volume
  // cells, massless particles), where the failure surfaces as a solver exception
  // naming none of these fields.
  if (mDimX && mDimX->value() < 1) {
    parsingWarn(tr("'dimX' must be at least 1 (it counts CELLS, not vertices); got %1.").arg(mDimX->value()));
    mDimX->setValue(1);
  }
  if (mDimY && mDimY->value() < 1) {
    parsingWarn(tr("'dimY' must be at least 1 (it counts CELLS, not vertices); got %1.").arg(mDimY->value()));
    mDimY->setValue(1);
  }
  if (mDimZ && mDimZ->value() < 1) {
    parsingWarn(tr("'dimZ' must be at least 1 (it counts CELLS, not vertices); got %1.").arg(mDimZ->value()));
    mDimZ->setValue(1);
  }
  if (mCellX && mCellX->value() <= 0.0) {
    parsingWarn(tr("'cellX' must be strictly positive; got %1.").arg(mCellX->value()));
    mCellX->setValue(0.05);
  }
  if (mCellY && mCellY->value() <= 0.0) {
    parsingWarn(tr("'cellY' must be strictly positive; got %1.").arg(mCellY->value()));
    mCellY->setValue(0.05);
  }
  if (mCellZ && mCellZ->value() <= 0.0) {
    parsingWarn(tr("'cellZ' must be strictly positive; got %1.").arg(mCellZ->value()));
    mCellZ->setValue(0.05);
  }

  // ⚠ NEGATIVE IS NOT A SENTINEL HERE, unlike on the cloth path. A cloth's
  // triKa/triKd go to the runtime as -1 meaning "derive from the matching ke";
  // add_soft_grid has nothing to derive from and forwards these straight into
  // the FEM material, so a negative Lame parameter is simply an unstable solve
  // with no message naming the field. Clamp to 0, which this node's own
  // convention reads as "leave it to the runtime".
  if (mDensity && mDensity->value() < 0.0) {
    parsingWarn(tr("'density' cannot be negative; got %1. Using the runtime default (0 means unset).")
                  .arg(mDensity->value()));
    mDensity->setValue(0.0);
  }
  if (mKMu && mKMu->value() < 0.0) {
    parsingWarn(tr("'kMu' is a Lame parameter and cannot be negative; got %1. Using the runtime default "
                   "(0 means unset).")
                  .arg(mKMu->value()));
    mKMu->setValue(0.0);
  }
  if (mKLambda && mKLambda->value() < 0.0) {
    parsingWarn(tr("'kLambda' is a Lame parameter and cannot be negative; got %1. Using the runtime default "
                   "(0 means unset).")
                  .arg(mKLambda->value()));
    mKLambda->setValue(0.0);
  }
  if (mKDamp && mKDamp->value() < 0.0) {
    parsingWarn(tr("'kDamp' cannot be negative (that is energy injection, not damping); got %1. Using the "
                   "runtime default (0 means unset).")
                  .arg(mKDamp->value()));
    mKDamp->setValue(0.0);
  }
  if (mParticleRadius && mParticleRadius->value() < 0.0) {
    parsingWarn(tr("'particleRadius' cannot be negative; got %1. Using the runtime default (0 means unset).")
                  .arg(mParticleRadius->value()));
    mParticleRadius->setValue(0.0);
  }

  // ⚠ 0 IS NOT "UNSET" ON THIS ONE, and it is the only field on the node where
  // it is not. 0 is a meaningful, legal crease angle -- it means "split every
  // shared vertex", i.e. fully faceted -- and it is exactly what IndexedFaceSet
  // defaults to. So it is clamped to the range an angle between two normals can
  // occupy and nothing else: below 0 no pair can ever be smoothed (which 0
  // already expresses), and above pi every pair is smoothed (which pi already
  // expresses), so a value outside is a typo rather than an intent.
  if (mCreaseAngle && mCreaseAngle->value() < 0.0) {
    parsingWarn(tr("'creaseAngle' is an angle in RADIANS and cannot be negative; got %1. Clamped to 0, which "
                   "renders every surface triangle flat.")
                  .arg(mCreaseAngle->value()));
    mCreaseAngle->setValue(0.0);
  }
  if (mCreaseAngle && mCreaseAngle->value() > M_PI) {
    parsingWarn(tr("'creaseAngle' is an angle in RADIANS between two face normals, so it cannot exceed pi; got "
                   "%1. Clamped to pi, which shares every vertex normal -- a rectangular block then renders as a "
                   "rounded bar of soap.")
                  .arg(mCreaseAngle->value()));
    mCreaseAngle->setValue(M_PI);
  }
}

void OmSoftBody::postFinalize() {
  OmBaseNode::postFinalize();

  if (abstractAppearance())
    abstractAppearance()->postFinalize();
  // Replacing or editing the appearance repaints the block in place -- no
  // reload, no rebuilt mesh, no touched particles. mAppearance's own changed()
  // covers "a different node was plugged in"; the appearance's covers "a field
  // inside it moved", which is what a texture swap or a roughness tweak is.
  if (mAppearance)
    connect(mAppearance, &OmSFNode::changed, this, &OmSoftBody::updateAppearance);
  updateAppearance();

  // ⚠ creaseAngle gets its OWN slot, not updateAppearance's: it changes the
  // vertex count, so it cannot be answered by re-binding a material.
  if (mCreaseAngle)
    connect(mCreaseAngle, &OmSFDouble::changed, this, &OmSoftBody::updateCreaseAngle);

  // Registration is DEFERRED to the first physics step, not done here. See the
  // long comment in onPhysicsStepStarted() for why -- doing it in postFinalize
  // would open the Newton world before WorldInfo's declarations reach the
  // backend, silently costing this world its declared friction and up axis.
  OmSimulationWorld *const world = OmSimulationWorld::instance();
  if (world != NULL)
    connect(world, &OmSimulationWorld::physicsStepStarted, this, &OmSoftBody::onPhysicsStepStarted);
}

int OmSoftBody::countRegisteredSoftGrids(const OmNewtonBackend *newton) {
  // ⚠ WHY THIS IS COUNTED RATHER THAN KEPT IN A STATIC COUNTER.
  //
  // Two different indices meet in this node. addSoftGrid() returns a PARTICLE
  // index (the block's offset into one shared particle array that cloth also
  // appends into), while softSurfaceTriangles() is keyed by the block's position
  // in the runtime's append-ordered World.soft_grids list. Nothing converts one
  // into the other, so the only honest way to learn our own key is to ask how
  // many blocks are already there before we add ours.
  //
  // A static counter would be wrong in three separate ways that all read as "the
  // wrong block's surface, or none": World.soft_grids is reset with the world
  // (World.__init__), so it would have to be told about every world load and
  // every /world/load reload; a FAILED registration consumes no slot but would
  // still have to not consume a count; and anything else that authors a soft
  // grid without an OmSoftBody node would desynchronise it silently. Counting is
  // self-describing and cannot drift.
  //
  // The probe is exact because soft_surface_packed() answers b"" -- zero
  // triangles -- for an out-of-range index, and every real block has surface
  // faces (dimX/dimY/dimZ are clamped to >= 1 in preFinalize, and a single cell
  // already contributes open faces). So the first zero is one past the end.
  //
  // Cost is registration-time only and quadratic in the number of soft blocks in
  // the world, which is a handful; the per-frame path never calls this.
  if (newton == NULL)
    return -1;
  const int kMaxSoftGrids = 4096;  // a ceiling, not a limit: nothing may loop unbounded over an FFI answer
  for (int i = 0; i < kMaxSoftGrids; ++i) {
    const int triangles = newton->softSurfaceTriangles(i, NULL, 0);
    if (triangles < 0)
      return -1;  // the runtime declined the query -- we cannot know our own key
    if (triangles == 0)
      return i;  // out of range: i is one past the last, i.e. the slot we are about to fill
  }
  return -1;
}

void OmSoftBody::onPhysicsStepStarted() {
  if (mRegistrationDone)
    return;
  // One shot whatever the outcome: a failure that re-fires every tick would
  // flood the log (and GET /sim/events) at the basic time step.
  mRegistrationDone = true;

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == NULL || !raw->isAvailable()) {
    OmLog::warning(tr("SoftBody '%1' is inert: the Newton runtime is not available, so there is no solver to "
                      "simulate its particles. Unlike a Cloth it cannot even render its rest pose -- its "
                      "surface is generated by newton -- so it is invisible as well as motionless. Run `python -m omnisim doctor`; on Windows stage the runtime with `make -C src/omnisim bundle-newton-runtime`, on Linux pip newton/warp/mujoco into the system python3 (docs/developer/newton-runtime-bundle.md).")
                     .arg(usefulName()));
    return;
  }
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);

  // ⚠ DO NOT REPLACE THIS WITH A BARE ensureWorldOpen().
  //
  // The Newton world is opened LAZILY, from inside
  // OmSolid::flushPendingNewtonRegistrations(), and that function does one thing
  // first that nothing else does: it caches WorldInfo's coordinateSystem and its
  // contact/solver declaration (newtonGroundMu / newtonContactKe /
  // newtonContactKd / iterations) into the backend BEFORE the world is
  // constructed. newton bakes the up axis into the implicit ground plane's normal
  // and copies mu/ke/kd into every shape AT ADD TIME, so a world opened ahead of
  // that cache can never be given them afterwards -- the exact defect that made a
  // declared newtonGroundMu 2.0 slide at the default on the 55-degree ramp
  // comparator.
  //
  // So: drive the ordinary flush first and let it open + configure the world. It
  // is idempotent (per-Solid registration is one-shot and the function is called
  // every tick by OmSimulationWorld::step anyway), and this runs from
  // physicsStepStarted, which is emitted EARLIER in the same step than the
  // ordinary flush/finalize seam -- so the block lands in the same build the
  // solids do, before finalizeWorld() closes it.
  OmSolid::flushPendingNewtonRegistrations();
  if (!newton->isWorldOpenForBuild()) {
    // A deformable-only world: no Solid opened it. The flush above still cached
    // the WorldInfo declarations, so opening it now applies them in the right
    // order. (The solver preference is plumbed by the flush's tail on the next
    // pass, which happens later in this same step, and finalizeWorld()
    // re-asserts it -- so "mujoco+vbd" still reaches the runtime in time.)
    if (newton->ensureWorldOpen() != 0) {
      OmLog::warning(tr("SoftBody '%1' is inert: the Newton world could not be opened.").arg(usefulName()));
      return;
    }
  }

  // Ask BEFORE registering: the answer is the slot this block is about to fill.
  const int gridIndex = countRegisteredSoftGrids(newton);

  // By value, not by const reference: the null-guard ternary yields a prvalue,
  // and binding a reference to it is a lifetime subtlety this code does not need
  // to have.
  const OmVector3 t = mTranslation ? mTranslation->value() : OmVector3();
  const OmRotation r = mRotation ? mRotation->value() : OmRotation();
  const OmQuaternion q = r.toQuaternion();
  // ⚠ The block's MINIMUM CORNER, not its centre -- newton's convention for
  // add_soft_grid, and the opposite of Solid/Box. Passed through untouched so the
  // .wbt number and the solver's number are the same number.
  const double pos[3] = {t.x(), t.y(), t.z()};
  // xyzw on the wire -- the house body_q layout, w LAST. OmQuaternion stores w
  // FIRST, so this reorder is load-bearing, not cosmetic.
  const double quat[4] = {q.x(), q.y(), q.z(), q.w()};

  // ⚠ TWO DIFFERENT "UNSET" CONVENTIONS MEET HERE, so this block is a
  // translation, not a set of defaults duplicated for convenience.
  //   .wrl side:     0 means "leave it to the runtime" (the convention every
  //                  WorldInfo newton* field uses, so authors already know it).
  //   runtime side:  every one of these is forwarded VERBATIM into the FEM
  //                  material. ZERO IS A LITERAL ZERO -- a 0 kMu really is a
  //                  block with no shear rigidity, and a 0 density really is
  //                  massless particles.
  // So an unset field must never be forwarded as 0; it is replaced by the
  // runtime's own default value, kept in the constants at the top of this file.
  // Note there is no negative "derive from" sentinel the way the cloth path has
  // one -- add_soft_grid has nothing to derive these from.
  const double densityValue = (mDensity && mDensity->value() > 0.0) ? mDensity->value() : DEFAULT_DENSITY;
  const double kMu = (mKMu && mKMu->value() > 0.0) ? mKMu->value() : DEFAULT_K_MU;
  const double kLambda = (mKLambda && mKLambda->value() > 0.0) ? mKLambda->value() : DEFAULT_K_LAMBDA;
  const double kDamp = (mKDamp && mKDamp->value() > 0.0) ? mKDamp->value() : DEFAULT_K_DAMP;
  const double radius =
    (mParticleRadius && mParticleRadius->value() > 0.0) ? mParticleRadius->value() : DEFAULT_PARTICLE_RADIUS;

  int particleEnd = -1;
  const int particleStart = newton->addSoftGrid(pos, quat, dimX(), dimY(), dimZ(), cellX(), cellY(), cellZ(),
                                                densityValue, kMu, kLambda, kDamp, radius, fixFlags(), &particleEnd);
  if (particleStart < 0) {
    // By far the most likely cause, and the one an author can fix, is the solver:
    // SolverVBD only exists on the coupled path. Name the field and the value
    // rather than reporting a bare failure.
    OmLog::warning(tr("SoftBody '%1' registered no particles and will neither move nor render. The usual cause "
                      "is that this world does not declare WorldInfo { newtonSolver \"mujoco+vbd\" } -- that is "
                      "the only solver profile that builds the SolverVBD a SoftBody needs.")
                     .arg(usefulName()));
    return;
  }
  mParticleStart = particleStart;
  mParticleEnd = particleEnd;

  // The invariant the whole render path rests on: the surface indices come back
  // BLOCK-LOCAL, so they are only meaningful if our particle range really is the
  // lattice we sized our buffers for. Refuse to draw rather than index a vertex
  // array that does not correspond.
  if (mParticleEnd - mParticleStart != particleCount()) {
    OmLog::warning(tr("SoftBody '%1' is inert: Newton returned %2 particles for a %3 x %4 x %5 cell lattice, "
                      "which needs %6. The block will not be rendered.")
                     .arg(usefulName())
                     .arg(mParticleEnd - mParticleStart)
                     .arg(dimX())
                     .arg(dimY())
                     .arg(dimZ())
                     .arg(particleCount()));
    mParticleStart = -1;
    mParticleEnd = -1;
    return;
  }

  OmLog::info(tr("SoftBody '%1': registered %2 particles (%3 x %4 x %5 cells) at Newton particle offset %6.")
                .arg(usefulName())
                .arg(particleCount())
                .arg(dimX())
                .arg(dimY())
                .arg(dimZ())
                .arg(mParticleStart));

  // Physics is live from here on whatever happens below; only the rendering can
  // still fail, and it fails visibly (an invisible but colliding block) rather
  // than silently.
  if (gridIndex < 0) {
    OmLog::warning(tr("SoftBody '%1' simulates but will not be rendered: its position in the runtime's soft-body "
                      "list could not be determined, so its surface triangles cannot be fetched.")
                     .arg(usefulName()));
    return;
  }
  mGridIndex = gridIndex;
  if (!fetchSurfaceTopology(newton))
    return;  // fetchSurfaceTopology() has already warned with the specifics
  buildWrenMesh();
}

bool OmSoftBody::fetchSurfaceTopology(const OmNewtonBackend *newton) {
  mSurfaceTriangles.clear();
  mIndices.clear();
  mRenderToParticle.clear();
  if (newton == NULL || mGridIndex < 0)
    return false;

  // ⚠ ONE FETCH, FOR EVER. This is the single structural difference from
  // OmCloth::buildTopology(), which derives a sheet's winding analytically from
  // dimX/dimY and could therefore recompute it at any time. A soft block's
  // surface is the set of OPEN FACES of its tet mesh: it is produced while
  // newton's ModelBuilder is being filled, the builder is CONSUMED at finalize(),
  // and the runtime snapshots the faces at authoring time precisely because they
  // cannot be recovered afterwards. What comes back here IS the surface -- there
  // is no second source to reconcile it against. buildRenderTopology() then
  // RE-ADDRESSES it from particles to render vertices (the crease split), and it
  // is that index buffer which is uploaded once as GL_STATIC_DRAW while only the
  // positions stream per tick.
  const int nTriangles = newton->softSurfaceTriangles(mGridIndex, NULL, 0);
  if (nTriangles <= 0) {
    OmLog::warning(tr("SoftBody '%1' simulates but will not be rendered: Newton reported no surface triangles "
                      "for soft-body index %2.")
                     .arg(usefulName())
                     .arg(mGridIndex));
    return false;
  }

  // Kept as int, which is what the backend writes AND what these are: PARTICLE
  // indices, the authoritative surface. The widening to the unsigned render
  // indices WREN wants happens once, per corner, in buildRenderTopology() --
  // which needs the particle numbers themselves to know which corners meet.
  mSurfaceTriangles.assign(static_cast<std::size_t>(nTriangles) * 3, 0);
  const int got = newton->softSurfaceTriangles(mGridIndex, mSurfaceTriangles.data(), nTriangles);
  if (got != nTriangles) {
    OmLog::warning(tr("SoftBody '%1' simulates but will not be rendered: asked Newton for %2 surface triangles "
                      "and got %3.")
                     .arg(usefulName())
                     .arg(nTriangles)
                     .arg(got));
    mSurfaceTriangles.clear();
    return false;
  }

  // Validate before a single index reaches WREN. These are BLOCK-LOCAL, so every
  // one must land inside this block's own particle lattice; an out-of-range index
  // would be read by the driver as a vertex-array offset and fault somewhere with
  // no OmniSim frame on the stack (that is exactly how the cloth shadow-buffer
  // crash presented). ⚠ It is ALSO what makes the incidence table below safe to
  // build without a second bounds check per corner.
  const int n = particleCount();
  for (std::size_t k = 0; k < mSurfaceTriangles.size(); ++k) {
    const int index = mSurfaceTriangles[k];
    if (index < 0 || index >= n) {
      OmLog::warning(tr("SoftBody '%1' simulates but will not be rendered: Newton's surface references particle "
                        "%2, outside this block's %3 particles.")
                       .arg(usefulName())
                       .arg(index)
                       .arg(n));
      mSurfaceTriangles.clear();
      return false;
    }
  }
  return buildRenderTopology();
}

bool OmSoftBody::buildRenderTopology() {
  mIndices.clear();
  mRenderToParticle.clear();
  mVertexNormals.clear();
  mTexCoords.clear();

  const int n = particleCount();
  const std::size_t cornerCount = mSurfaceTriangles.size();
  const std::size_t triangleCount = cornerCount / 3;
  if (n <= 0 || triangleCount == 0)
    return false;

  // 1. THE REST LATTICE, in the block's OWN LOCAL frame -- not mPositions.
  //
  // ⚠ Deliberately independent of whatever the solver is currently doing. This
  // runs at registration, but it also re-runs when `creaseAngle` is edited mid
  // simulation, and a split derived from a squashed pose would make the block's
  // silhouette a function of its strain. Local rather than world because the
  // crease test is rotation-invariant (it compares two normals to each other)
  // while the box projection in buildTextureCoordinates() is not -- it wants the
  // block's own axes, which is exactly what this is.
  std::vector<float> rest(static_cast<std::size_t>(n) * 3, 0.0f);
  {
    // Z OUTER, Y MIDDLE, X INNER -- newton's own lattice order, the same one
    // computeRestPositions() copies, so index i here is particle i there.
    const int nx = dimX() + 1;
    const int ny = dimY() + 1;
    const int nz = dimZ() + 1;
    const double dx = cellX();
    const double dy = cellY();
    const double dz = cellZ();
    int i = 0;
    for (int z = 0; z < nz; ++z) {
      for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x, ++i) {
          rest[i * 3 + 0] = static_cast<float>(x * dx);
          rest[i * 3 + 1] = static_cast<float>(y * dy);
          rest[i * 3 + 2] = static_cast<float>(z * dz);
        }
      }
    }
  }

  // 2. One UNIT normal per surface triangle. A degenerate face is left at zero
  // and is then never merged with anything (the acos test below skips it), which
  // is the honest answer: a triangle with no area has no normal to compare.
  std::vector<float> faceNormals(triangleCount * 3, 0.0f);
  for (std::size_t t = 0; t < triangleCount; ++t) {
    const float *const p0 = &rest[static_cast<std::size_t>(mSurfaceTriangles[t * 3 + 0]) * 3];
    const float *const p1 = &rest[static_cast<std::size_t>(mSurfaceTriangles[t * 3 + 1]) * 3];
    const float *const p2 = &rest[static_cast<std::size_t>(mSurfaceTriangles[t * 3 + 2]) * 3];
    const float e1[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
    const float e2[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
    const float cx = e1[1] * e2[2] - e1[2] * e2[1];
    const float cy = e1[2] * e2[0] - e1[0] * e2[2];
    const float cz = e1[0] * e2[1] - e1[1] * e2[0];
    const float len = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (len <= 1e-20f)
      continue;
    faceNormals[t * 3 + 0] = cx / len;
    faceNormals[t * 3 + 1] = cy / len;
    faceNormals[t * 3 + 2] = cz / len;
  }

  // 3. Which triangle CORNERS meet at each particle, as a CSR: counting pass,
  // prefix sum, fill pass. One allocation each, no per-particle vector, and the
  // corner id is its own offset into mSurfaceTriangles / mIndices.
  std::vector<int> cornerStart(static_cast<std::size_t>(n) + 1, 0);
  for (std::size_t k = 0; k < cornerCount; ++k)
    ++cornerStart[static_cast<std::size_t>(mSurfaceTriangles[k]) + 1];
  for (int p = 0; p < n; ++p)
    cornerStart[p + 1] += cornerStart[p];
  std::vector<int> cornerList(cornerCount, 0);
  {
    std::vector<int> cursor(cornerStart.begin(), cornerStart.end() - 1);
    for (std::size_t k = 0; k < cornerCount; ++k)
      cornerList[cursor[mSurfaceTriangles[k]]++] = static_cast<int>(k);
  }

  // 4. THE SPLIT. Per particle, cluster the triangles meeting there by the crease
  // test and emit one render vertex per cluster.
  //
  // The test is `angle(ni, nj) < creaseAngle`, strictly -- character for
  // character the rule OmTriangleMesh::finalPass applies to an IndexedFaceSet, so
  // a creaseAngle carried over from a Shape means the same thing here. It is
  // TRANSITIVE by construction (union-find, not a greedy nearest-cluster scan),
  // which is what makes a smoothly curving surface stay one cluster all the way
  // round instead of tearing wherever the walk happened to start.
  //
  // Cost is O(k^2) in the valence k of one particle -- 4 to 6 on a lattice
  // surface, 12 at a corner of a heavily subdivided one -- and it is paid ONCE,
  // at registration. The per-frame path never comes here.
  const double crease = creaseAngle();
  int surfaceParticles = 0;
  mIndices.assign(cornerCount, 0u);
  std::vector<int> parent;
  std::vector<int> renderId;
  for (int p = 0; p < n; ++p) {
    const int lo = cornerStart[p];
    const int k = cornerStart[p + 1] - lo;
    // ⚠ An INTERIOR particle has k == 0 and gets NO render vertex at all. That is
    // the one place where the drawn vertex set is smaller than the particle set,
    // and it is why mRenderToParticle exists rather than the old
    // one-vertex-per-particle identity: a 4^3 block used to upload 125 vertices
    // of which 27 were never referenced by any triangle.
    if (k == 0)
      continue;
    ++surfaceParticles;
    parent.resize(k);
    for (int i = 0; i < k; ++i)
      parent[i] = i;
    for (int i = 0; i < k; ++i) {
      const float *const ni = &faceNormals[static_cast<std::size_t>(cornerList[lo + i] / 3) * 3];
      if (ni[0] == 0.0f && ni[1] == 0.0f && ni[2] == 0.0f)
        continue;  // degenerate face: merges with nothing
      for (int j = i + 1; j < k; ++j) {
        const float *const nj = &faceNormals[static_cast<std::size_t>(cornerList[lo + j] / 3) * 3];
        if (nj[0] == 0.0f && nj[1] == 0.0f && nj[2] == 0.0f)
          continue;
        const float dot = ni[0] * nj[0] + ni[1] * nj[1] + ni[2] * nj[2];
        const double angle = std::acos(static_cast<double>(std::max(-1.0f, std::min(1.0f, dot))));
        if (angle >= crease)
          continue;
        const int ri = softBodyFindRoot(parent, i);
        const int rj = softBodyFindRoot(parent, j);
        if (ri != rj)
          parent[ri] = rj;
      }
    }
    renderId.assign(k, -1);
    for (int i = 0; i < k; ++i) {
      const int root = softBodyFindRoot(parent, i);
      if (renderId[root] < 0) {
        renderId[root] = static_cast<int>(mRenderToParticle.size());
        mRenderToParticle.push_back(static_cast<unsigned int>(p));
      }
      mIndices[cornerList[lo + i]] = static_cast<unsigned int>(renderId[root]);
    }
  }

  mVertexNormals.assign(mRenderToParticle.size() * 3, 0.0f);
  buildTextureCoordinates(rest);

  // The one line that says whether the crease split did anything. Render
  // vertices == surface particles means every vertex was smoothed (creaseAngle
  // too large, or a genuinely smooth surface); == 3 * triangles means every one
  // was split (creaseAngle 0). The reference sponge reads 104 / 54 / 94.
  OmLog::info(tr("SoftBody '%1': surface is %2 triangles over %3 particles, split at creaseAngle %4 rad into %5 "
                 "render vertices.")
                .arg(usefulName())
                .arg(static_cast<int>(triangleCount))
                .arg(surfaceParticles)
                .arg(crease, 0, 'f', 3)
                .arg(static_cast<int>(mRenderToParticle.size())));
  return !mRenderToParticle.empty();
}

void OmSoftBody::buildTextureCoordinates(const std::vector<float> &localRest) {
  const std::size_t vertexCount = mRenderToParticle.size();
  mTexCoords.assign(vertexCount * 2, 0.0f);
  if (vertexCount == 0)
    return;

  // A BOX PROJECTION in the block's own local frame: each render vertex is
  // mapped by the face its smoothing cluster faces, which is only possible
  // BECAUSE of the crease split -- before it, a corner particle was one vertex
  // and could carry only one UV, so any mapping but a single planar one was
  // unrepresentable. (The node's original placeholder was that single planar
  // projection along local Z, which collapses the four side faces to a line. It
  // did not matter while the material was an untextured engine-owned phong; the
  // `appearance` field is what makes it matter.)
  //
  // Normalised over the block's AUTHORED extent, so a texture covers each face
  // exactly once. ⚠ Faces of different aspect ratio therefore stretch it
  // differently; TextureTransform.scale is the knob, and this is documented in
  // SoftBody.wrl rather than guessed at here.
  const float span[3] = {static_cast<float>(dimX() * cellX()), static_cast<float>(dimY() * cellY()),
                         static_cast<float>(dimZ() * cellZ())};
  float inv[3] = {1.0f, 1.0f, 1.0f};
  for (int c = 0; c < 3; ++c)
    inv[c] = span[c] > 1e-9f ? 1.0f / span[c] : 1.0f;

  // The cluster's REST normal is what picks the face. Same accumulation the
  // per-frame path uses, run once against the local rest lattice.
  std::vector<float> restNormals;
  computeVertexNormals(localRest, restNormals);
  if (restNormals.size() != vertexCount * 3)
    return;

  for (std::size_t v = 0; v < vertexCount; ++v) {
    const float *const nrm = &restNormals[v * 3];
    int axis = 0;
    float best = std::fabs(nrm[0]);
    if (std::fabs(nrm[1]) > best) {
      axis = 1;
      best = std::fabs(nrm[1]);
    }
    if (std::fabs(nrm[2]) > best)
      axis = 2;
    // The two axes that are NOT the dominant one, in a fixed order so adjacent
    // faces of the same orientation share a continuous parameterisation.
    const int uAxis = (axis == 0) ? 1 : 0;
    const int vAxis = (axis == 2) ? 1 : 2;
    const std::size_t p = static_cast<std::size_t>(mRenderToParticle[v]) * 3;
    mTexCoords[v * 2 + 0] = localRest[p + uAxis] * inv[uAxis];
    mTexCoords[v * 2 + 1] = localRest[p + vAxis] * inv[vAxis];
  }
}

void OmSoftBody::computeVertexNormals(const std::vector<float> &positions, std::vector<float> &normals) const {
  const std::size_t vertexCount = mRenderToParticle.size();
  normals.assign(vertexCount * 3, 0.0f);
  if (vertexCount == 0 || positions.size() < static_cast<std::size_t>(particleCount()) * 3)
    return;

  // ANGLE-WEIGHTED accumulation, and the weighting is the whole point of this
  // function rather than an implementation detail.
  //
  // The obvious thing -- and what this node did before -- is to sum the RAW cross
  // products, whose magnitude is 2*area, so each face is weighted by its own area
  // for free. On a rigid mesh that is fine. On a soft body it is not: VBD deforms
  // triangle AREAS non-uniformly under load, so an area-weighted vertex normal is
  // a function of the local strain -- the quantity that changes most, and most
  // abruptly, from frame to frame -- and the shading swims over a block that is
  // barely moving. The INTERIOR ANGLE a face subtends at the vertex is far more
  // stable under the same deformation, and it is also the weighting with the
  // right limit behaviour under refinement (Thurmer & Wuthrich 1998; Max, JGT
  // 4(2) 1999).
  for (std::size_t k = 0; k + 2 < mIndices.size(); k += 3) {
    const unsigned int r[3] = {mIndices[k], mIndices[k + 1], mIndices[k + 2]};
    const float *p[3];
    for (int c = 0; c < 3; ++c)
      p[c] = &positions[static_cast<std::size_t>(mRenderToParticle[r[c]]) * 3];

    const float e1[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]};
    const float e2[3] = {p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]};
    float fn[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
    const float twiceArea = std::sqrt(fn[0] * fn[0] + fn[1] * fn[1] + fn[2] * fn[2]);
    if (twiceArea <= 1e-20f)
      continue;  // collapsed triangle: it has no normal to contribute
    fn[0] /= twiceArea;
    fn[1] /= twiceArea;
    fn[2] /= twiceArea;

    for (int c = 0; c < 3; ++c) {
      const float *const a = p[c];
      const float *const b = p[(c + 1) % 3];
      const float *const d = p[(c + 2) % 3];
      const float u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
      const float w[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
      const float lu = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
      const float lw = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
      if (lu <= 1e-20f || lw <= 1e-20f)
        continue;
      const float cosTheta = std::max(-1.0f, std::min(1.0f, (u[0] * w[0] + u[1] * w[1] + u[2] * w[2]) / (lu * lw)));
      const float weight = std::acos(cosTheta);
      normals[r[c] * 3 + 0] += fn[0] * weight;
      normals[r[c] * 3 + 1] += fn[1] * weight;
      normals[r[c] * 3 + 2] += fn[2] * weight;
    }
  }

  for (std::size_t v = 0; v < vertexCount; ++v) {
    float *const nrm = &normals[v * 3];
    const float len = std::sqrt(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
    if (len > 1e-12f) {
      nrm[0] /= len;
      nrm[1] /= len;
      nrm[2] /= len;
    } else {
      // Every render vertex is referenced by at least one triangle -- that is how
      // it came to exist -- so this is now only reachable through a fully
      // collapsed fan, not through the interior particles that used to land here.
      // Any unit vector beats a zero normal, which renders as a black vertex.
      nrm[0] = 0.0f;
      nrm[1] = 0.0f;
      nrm[2] = 1.0f;
    }
  }
}

void OmSoftBody::computeRestPositions() {
  const int nx = dimX() + 1;
  const int ny = dimY() + 1;
  const int nz = dimZ() + 1;
  const OmVector3 t = mTranslation ? mTranslation->value() : OmVector3();
  const OmRotation r = mRotation ? mRotation->value() : OmRotation();
  const OmMatrix3 m = r.toMatrix3();
  const double dx = cellX();
  const double dy = cellY();
  const double dz = cellZ();

  // ⚠ Z OUTER, Y MIDDLE, X INNER -- the identical lattice order newton's
  // add_soft_grid produces (its own grid_index(x, y, z) is
  // z*(dimX+1)*(dimY+1) + y*(dimX+1) + x), so index i here is particle i there.
  // The surface indices are block-local against exactly this ordering, which is
  // why it is copied from the source rather than chosen.
  int i = 0;
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x, ++i) {
        const OmVector3 world = t + m * OmVector3(x * dx, y * dy, z * dz);
        mPositions[i * 3 + 0] = static_cast<float>(world.x());
        mPositions[i * 3 + 1] = static_cast<float>(world.y());
        mPositions[i * 3 + 2] = static_cast<float>(world.z());
      }
    }
  }
}

void OmSoftBody::uploadPositions() {
  const int n = particleCount();
  const std::size_t vertexCount = mRenderToParticle.size();
  if (n <= 0 || vertexCount == 0 || mPositions.size() < static_cast<std::size_t>(n) * 3)
    return;

  computeVertexNormals(mPositions, mVertexNormals);
  // D1.4: WREN's per-frame coordinate/normal re-stream is gone. The normals
  // (already normalised in place) and positions are drained per frame by
  // wgpuVertexStream(), which does the render-vertex gather through
  // mRenderToParticle itself.
}

// ---- wgpu MAIN-VIEW DRAW SOURCE ---------------------------------------------
//
// The interleave is the wgpu vertex layout (pos3 + norm3 + uv2 float, stride 32):
// walk RENDER VERTICES, gather the position through mRenderToParticle, take the
// normal and UV straight (both are already per-render-vertex, and
// computeVertexNormals() already normalised).
bool OmSoftBody::wgpuVertexStream(std::vector<unsigned char> &out) const {
  const std::size_t vertexCount = mRenderToParticle.size();
  const std::size_t particles = mPositions.size() / 3;
  if (vertexCount == 0 || particles == 0 || mIndices.empty() ||
      mVertexNormals.size() < vertexCount * 3 || mTexCoords.size() < vertexCount * 2)
    return false;
  out.resize(vertexCount * 32);
  unsigned char *const dst = out.data();
  for (std::size_t v = 0; v < vertexCount; ++v) {
    const std::size_t p = static_cast<std::size_t>(mRenderToParticle[v]);
    if (p >= particles)
      return false;  // a render vertex naming a particle we do not have is not a surface
    float rec[8];
    rec[0] = mPositions[p * 3 + 0];
    rec[1] = mPositions[p * 3 + 1];
    rec[2] = mPositions[p * 3 + 2];
    rec[3] = mVertexNormals[v * 3 + 0];
    rec[4] = mVertexNormals[v * 3 + 1];
    rec[5] = mVertexNormals[v * 3 + 2];
    rec[6] = mTexCoords[v * 2 + 0];
    rec[7] = mTexCoords[v * 2 + 1];
    std::memcpy(dst + v * 32, rec, 32);
  }
  return true;
}

bool OmSoftBody::wgpuCastShadows() const {
  return mCastShadows ? mCastShadows->value() : true;
}

void OmSoftBody::wgpuFallbackDiffuse(float rgb3[3]) const {
  rgb3[0] = static_cast<float>(mDiffuseColor ? mDiffuseColor->red() : 0.85);
  rgb3[1] = static_cast<float>(mDiffuseColor ? mDiffuseColor->green() : 0.35);
  rgb3[2] = static_cast<float>(mDiffuseColor ? mDiffuseColor->blue() : 0.3);
}

void OmSoftBody::animateMesh() {
  // Not simulated: there is nothing to read back, and (unlike a Cloth) nothing
  // static to keep showing either -- with no registration the surface buffers
  // were never built.
  if (mParticleStart < 0)
    return;

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == NULL || !raw->isAvailable())
    return;
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;

  const int n = particleCount();
  if (n <= 0 || mParticleEnd - mParticleStart != n || mPositions.size() < static_cast<std::size_t>(n) * 3)
    return;

  // Straight into the vertex array: the runtime slices OUR half-open range
  // server-side, so there is no staging buffer and no discarded transfer even in
  // a world holding several blocks (and cloth appends into the same particle
  // list, which is exactly why the slice has to be a range and not a count).
  const int got = newton->snapshotParticlePositions(mParticleStart, mParticleEnd, mPositions.data());
  if (got != n)
    return;  // short read -- keep the previous frame's geometry rather than tearing

  uploadPositions();
}

void OmSoftBody::createWrenObjects() {
  OmBaseNode::createWrenObjects();

  if (particleCount() <= 0)
    return;

  // Normally a no-op at this point -- the index buffer does not exist until the
  // first physics step registers the block. It does the work when the ordering is
  // the other way round, i.e. node re-finalized on an already-registered block.
  buildWrenMesh();
}

void OmSoftBody::buildWrenMesh() {
  // Both preconditions, in either order: the node is fully set up
  // (areWrenObjectsInitialized() -- the base hook has run), and newton has told
  // us what the surface is. Whichever of createWrenObjects() /
  // onPhysicsStepStarted() runs second is the one that actually builds.
  // D1.4: a non-empty mPositions is the "already built" latch that the WREN
  // dynamic-mesh pointer used to provide.
  if (!areWrenObjectsInitialized() || !mPositions.empty() || mIndices.empty() || mRenderToParticle.empty())
    return;

  const int n = particleCount();
  if (n <= 0)
    return;

  mPositions.assign(static_cast<std::size_t>(n) * 3, 0.0f);
  // ⚠ PARTICLES for the positions, RENDER VERTICES for the normals. The two are
  // different counts as soon as a crease splits anything, and mPositions is sized
  // to the former because it is the readback target, not the vertex array.
  mVertexNormals.assign(mRenderToParticle.size() * 3, 0.0f);
  // The first frame is drawn before any readback exists (the world is still open
  // for build at registration time, so snapshotParticlePositions would refuse),
  // and the authored lattice is exactly what the solver is about to start from.
  computeRestPositions();

  uploadPositions();

  // The appearance is an ordinary node and drives its own texture CPU loads
  // from its createWrenObjects() chain; the wgpu material path reads what it
  // latches. Same order OmCloth::createWrenObjects and
  // OmShape::createWrenObjects use.
  if (abstractAppearance())
    abstractAppearance()->createWrenObjects();

  // D1.4: the WREN mesh/material/renderable build that lived here is gone; the
  // wgpu renderer drains wgpuVertexStream()/wgpuIndices() instead. The face
  // culling measurement notes (a soft block is a closed volume, all surface
  // faces wind outward -- CCW seen from outside, 104/104 on the reference
  // sponge) now concern only the wgpu draw path.

  // Subscribe for the per-frame vertex stream, and only now: an inert SoftBody
  // never gets here, so it costs no per-frame callback at all. The listener fires
  // only when sim time has actually advanced, so a paused simulation is free too.
  OmDeformableFrameListener::instance()->subscribeSoftBody(this);
}

void OmSoftBody::updateAppearance() {
  // Field edits inside the appearance repaint without a reload; the connection
  // is re-made here (rather than only in postFinalize) because the appearance
  // NODE itself can be swapped at runtime.
  if (pbrAppearance())
    connect(pbrAppearance(), &OmPbrAppearance::changed, this, &OmSoftBody::updateAppearance, Qt::UniqueConnection);
  else if (appearance())
    connect(appearance(), &OmAppearance::changed, this, &OmSoftBody::updateAppearance, Qt::UniqueConnection);

  if (!areWrenObjectsInitialized())
    return;

  // A swapped-in appearance node still has to run its own setup chain (texture
  // CPU loads in particular); the wgpu renderer reads whatever it latches.
  if (abstractAppearance())
    abstractAppearance()->createWrenObjects();
}

void OmSoftBody::updateCreaseAngle() {
  // Nothing to re-split until the surface has been fetched. The value is read
  // fresh by buildRenderTopology(), so an edit made before registration is simply
  // the one that gets used.
  if (mSurfaceTriangles.empty())
    return;

  buildRenderTopology();

  // ⚠ A FULL TOPOLOGY REBUILD, not a re-stream: the split changes the VERTEX
  // COUNT, so mIndices / mTexCoords / mVertexNormals were all re-derived above
  // (the renderer's size check keys on the new counts). mPositions is untouched
  // -- the particles did not move -- so only the normals need recomputing for
  // the new render-vertex set.
  if (mPositions.empty())
    return;
  uploadPositions();
}

void OmSoftBody::reset(const QString &id) {
  OmBaseNode::reset(id);
  if (abstractAppearance())
    abstractAppearance()->reset(id);
}
