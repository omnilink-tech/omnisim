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

#include "OmGranularBed.hpp"

#include "OmAppearance.hpp"
#include "OmLog.hpp"
// OmNewtonBackend.hpp also pulls in OmPhysicsBackend.hpp, which is where the
// OmPhysicsBackendRegistry namespace lives -- there is no separate registry
// header, and OmCloth.cpp / OmSoftBody.cpp / OmSolid.cpp get it the same way.
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
#include "OmSFString.hpp"
#include "OmSFVector3.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmVector3.hpp"

#include <cstddef>

void OmGranularBed::init() {
  mTranslation = findSFVector3("translation");
  mRotation = findSFRotation("rotation");
  mSize = findSFVector3("size");
  mVoxelSize = findSFDouble("voxelSize");
  mParticlesPerCell = findSFDouble("particlesPerCell");
  mCount = findSFInt("count");
  mDensity = findSFDouble("density");
  mFriction = findSFDouble("friction");
  mYieldPressure = findSFDouble("yieldPressure");
  mYieldStress = findSFDouble("yieldStress");
  mYoungModulus = findSFDouble("youngModulus");
  mPoissonRatio = findSFDouble("poissonRatio");
  mViscosity = findSFDouble("viscosity");
  mParticleRadius = findSFDouble("particleRadius");
  mRigidSubsteps = findSFInt("rigidSubsteps");
  mProxyIterations = findSFInt("proxyIterations");
  mMaxIterations = findSFInt("maxIterations");
  mTolerance = findSFDouble("tolerance");
  mGridType = findSFString("gridType");
  mGridPadding = findSFInt("gridPadding");
  mDiffuseColor = findSFColor("diffuseColor");
  mAppearance = findSFNode("appearance");
  mCastShadows = findSFBool("castShadows");

  mParticleStart = -1;
  mParticleEnd = -1;
  mBedIndex = -1;
  mParticleRadiusEffective = -1.0;
  mRegistrationDone = false;
  mHavePositions = false;
  mHostPositions.clear();
}

// THE REGISTRY. File-static rather than a class member so nothing has to be
// constructed to ask "does this world have a bed?" -- anyGranularBeds() is an
// empty-test on an already-existing vector. Membership is maintained by the
// ctor/dtor pair, so it cannot drift from the set of live nodes the way a
// lazily-populated list would. Verbatim OmGranularGroup's, which is verbatim
// the deformables' -- three nodes, one pattern, deliberately.
static std::vector<OmGranularBed *> gLiveGranularBeds;

bool OmGranularBed::anyGranularBeds() {
  return !gLiveGranularBeds.empty();
}

const std::vector<OmGranularBed *> &OmGranularBed::liveBeds() {
  return gLiveGranularBeds;
}

OmGranularBed::OmGranularBed(OmTokenizer *tokenizer) : OmBaseNode("GranularBed", tokenizer) {
  init();
  gLiveGranularBeds.push_back(this);
}

OmGranularBed::OmGranularBed(const OmGranularBed &other) : OmBaseNode(other) {
  init();
  gLiveGranularBeds.push_back(this);
}

OmGranularBed::OmGranularBed(const OmNode &other) : OmBaseNode(other) {
  init();
  gLiveGranularBeds.push_back(this);
}

OmGranularBed::~OmGranularBed() {
  for (std::size_t i = 0; i < gLiveGranularBeds.size(); ++i)
    if (gLiveGranularBeds[i] == this) {
      gLiveGranularBeds.erase(gLiveGranularBeds.begin() + i);
      break;
    }
}

OmAppearance *OmGranularBed::appearance() const {
  return mAppearance ? dynamic_cast<OmAppearance *>(mAppearance->value()) : NULL;
}

OmPbrAppearance *OmGranularBed::pbrAppearance() const {
  return mAppearance ? dynamic_cast<OmPbrAppearance *>(mAppearance->value()) : NULL;
}

OmAbstractAppearance *OmGranularBed::abstractAppearance() const {
  return mAppearance ? dynamic_cast<OmAbstractAppearance *>(mAppearance->value()) : NULL;
}

double OmGranularBed::voxelSize() const {
  return mVoxelSize ? mVoxelSize->value() : 0.0;
}

double OmGranularBed::particlesPerCell() const {
  return mParticlesPerCell ? mParticlesPerCell->value() : 0.0;
}

int OmGranularBed::countBudget() const {
  return mCount ? mCount->value() : 0;
}

double OmGranularBed::density() const {
  return mDensity ? mDensity->value() : 0.0;
}

bool OmGranularBed::wgpuCastShadows() const {
  // ⚠ FALSE, not TRUE. See the header: a bed is 10^4-10^5 draws and the shadow
  // pass would re-rasterise every one of them per light per frame.
  return mCastShadows ? mCastShadows->value() : false;
}

void OmGranularBed::wgpuFallbackDiffuse(float rgb3[3]) const {
  if (rgb3 == NULL)
    return;
  // The .wrl default, restated so a node whose field is somehow absent still
  // draws sand rather than white.
  rgb3[0] = 0.85f;
  rgb3[1] = 0.70f;
  rgb3[2] = 0.40f;
  if (mDiffuseColor) {
    rgb3[0] = static_cast<float>(mDiffuseColor->red());
    rgb3[1] = static_cast<float>(mDiffuseColor->green());
    rgb3[2] = static_cast<float>(mDiffuseColor->blue());
  }
}

bool OmGranularBed::wgpuParticles(const float *&xyzOut, int &countOut, float &radiusOut) const {
  // ⚠ DECLINES RATHER THAN DRAWING A PLACEHOLDER, in all three of its failure
  // cases: not registered, not yet stepped, and a radius the runtime would not
  // report. Same posture as OmGranularGroup::wgpuParticles and for the same
  // reason -- N spheres piled at the world origin, or drawn at a guessed size,
  // is not a scene anyone authored.
  if (mParticleStart < 0 || !mHavePositions)
    return false;
  const int n = mParticleEnd - mParticleStart;
  if (n <= 0 || mHostPositions.size() < static_cast<std::size_t>(n) * 3)
    return false;
  if (!(mParticleRadiusEffective > 0.0))
    return false;
  xyzOut = mHostPositions.data();
  countOut = n;
  radiusOut = static_cast<float>(mParticleRadiusEffective);
  return true;
}

bool OmGranularBed::refreshHostPositions() {
  if (mParticleStart < 0)
    return false;
  const int n = mParticleEnd - mParticleStart;
  if (n <= 0)
    return false;
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == NULL || !raw->isAvailable())
    return false;
  const OmNewtonBackend *const newton = static_cast<const OmNewtonBackend *>(raw);
  const std::size_t expected = static_cast<std::size_t>(n) * 3;
  if (mHostPositions.size() != expected)
    mHostPositions.assign(expected, 0.0f);
  // ⚠ ONE FFI CROSSING FOR THE WHOLE BED, and the destination stride is the
  // runtime's own (3 float32, tightly packed), so this is a memcpy on both
  // sides of the boundary rather than a per-particle loop. At 4 x 10^5
  // particles a per-particle crossing would not be a rendering path.
  const int got = newton->snapshotParticlePositions(mParticleStart, mParticleEnd,
                                                    mHostPositions.data());
  if (got != n) {
    // Before the first step the world is not RUNNING and the readback declines
    // (-1). That is the ordinary t=0 case, not an error, so it is silent and
    // the bed simply keeps declining to draw until a step has happened.
    return false;
  }
  mHavePositions = true;
  return true;
}

void OmGranularBed::downloadAssets() {
  OmBaseNode::downloadAssets();
  // The appearance owns ImageTexture children whose `url` may be remote. Only
  // its COLOUR reaches a particle draw (the header explains why the maps
  // cannot), but the node still has to run its own asset chain -- an
  // appearance shared with a Shape elsewhere in the scene would otherwise be
  // resolved by whichever owner happened to finalize first.
  if (abstractAppearance())
    abstractAppearance()->downloadAssets();
}

void OmGranularBed::preFinalize() {
  OmBaseNode::preFinalize();

  if (abstractAppearance())
    abstractAppearance()->preFinalize();

  // Clamp to a bed that can actually be built. Each of these would otherwise
  // reach newton as a degenerate request -- a zero-volume box, a zero-spacing
  // lattice, massless particles -- where the failure surfaces as a solver
  // exception naming none of these fields.
  if (mSize) {
    const OmVector3 s = mSize->value();
    if (!(s.x() > 0.0) || !(s.y() > 0.0) || !(s.z() > 0.0)) {
      parsingWarn(tr("'size' must be strictly positive on every axis (it is an extent in METRES, "
                     "not a cell count); got %1 %2 %3. Using 1 1 0.3.")
                    .arg(s.x())
                    .arg(s.y())
                    .arg(s.z()));
      mSize->setValue(1.0, 1.0, 0.3);
    }
  }
  if (mVoxelSize && mVoxelSize->value() < 0.0) {
    parsingWarn(tr("'voxelSize' cannot be negative; got %1. Using the runtime default "
                   "(0 means unset).")
                  .arg(mVoxelSize->value()));
    mVoxelSize->setValue(0.0);
  }
  if (mParticlesPerCell && mParticlesPerCell->value() < 0.0) {
    parsingWarn(tr("'particlesPerCell' cannot be negative; got %1. Using the runtime default "
                   "(0 means unset).")
                  .arg(mParticlesPerCell->value()));
    mParticlesPerCell->setValue(0.0);
  }
  if (mCount && mCount->value() < 0) {
    parsingWarn(tr("'count' is a particle BUDGET and cannot be negative; got %1. Using 0, which "
                   "disables the budget and uses the authored voxelSize exactly.")
                  .arg(mCount->value()));
    mCount->setValue(0);
  }
  // ⚠ NEGATIVE IS NOT A SENTINEL ON ANY MATERIAL FIELD HERE, unlike the cloth
  // path where triKa/triKd go to the runtime as -1 meaning "derive from the
  // matching ke". These are forwarded into a Drucker-Prager material, where a
  // negative friction or yield threshold is simply an unstable solve with no
  // message naming the field. Clamp to 0, which this node's convention reads as
  // "leave it to newton".
  OmSFDouble *const material[7] = {mDensity,      mFriction,     mYieldPressure, mYieldStress,
                                   mYoungModulus, mPoissonRatio, mViscosity};
  const char *const materialNames[7] = {"density",      "friction",     "yieldPressure",
                                        "yieldStress",  "youngModulus", "poissonRatio",
                                        "viscosity"};
  for (int i = 0; i < 7; ++i) {
    if (material[i] && material[i]->value() < 0.0) {
      parsingWarn(tr("'%1' cannot be negative; got %2. Using 0, which leaves the value to newton.")
                    .arg(materialNames[i])
                    .arg(material[i]->value()));
      material[i]->setValue(0.0);
    }
  }
  if (mParticleRadius && mParticleRadius->value() < 0.0) {
    parsingWarn(tr("'particleRadius' cannot be negative; got %1. Using 0, which derives it from "
                   "the lattice spacing.")
                  .arg(mParticleRadius->value()));
    mParticleRadius->setValue(0.0);
  }
  // ⚠ rigidSubsteps 1 is MEASURABLY WRONG rather than merely cheap: a rigid
  // cube RESTING on the bed GAINS energy and climbs, 0.27 -> 0.99 m. Not
  // clamped -- 1 is a legitimate diagnostic setting and silently overriding an
  // authored value is worse -- but it does not go past unremarked.
  if (mRigidSubsteps && mRigidSubsteps->value() < 1) {
    parsingWarn(tr("'rigidSubsteps' must be at least 1; got %1. Using the default 4.")
                  .arg(mRigidSubsteps->value()));
    mRigidSubsteps->setValue(4);
  } else if (mRigidSubsteps && mRigidSubsteps->value() < 4) {
    parsingWarn(tr("'rigidSubsteps' is %1, below the default 4. MEASURED at 1: a rigid body "
                   "RESTING on the bed gains energy and climbs (0.27 -> 0.99 m). Honouring the "
                   "authored value, but treat anything below 4 as a diagnostic setting.")
                  .arg(mRigidSubsteps->value()));
  }
  if (mProxyIterations && mProxyIterations->value() < 0) {
    parsingWarn(tr("'proxyIterations' cannot be negative; got %1. Using 0 (the runtime default, 1).")
                  .arg(mProxyIterations->value()));
    mProxyIterations->setValue(0);
  }
  if (mMaxIterations && mMaxIterations->value() < 0) {
    parsingWarn(tr("'maxIterations' cannot be negative; got %1. Using 0 (the runtime default, 50).")
                  .arg(mMaxIterations->value()));
    mMaxIterations->setValue(0);
  }
  if (mTolerance && mTolerance->value() < 0.0) {
    parsingWarn(tr("'tolerance' cannot be negative; got %1. Using 0 (the runtime default, 1e-4).")
                  .arg(mTolerance->value()));
    mTolerance->setValue(0.0);
  }
  if (mGridPadding && mGridPadding->value() < 0) {
    parsingWarn(tr("'gridPadding' cannot be negative; got %1. Using 0.").arg(mGridPadding->value()));
    mGridPadding->setValue(0);
  }
  if (mGridType) {
    const QString g = mGridType->value().trimmed().toLower();
    if (g != "sparse" && g != "fixed") {
      parsingWarn(tr("'gridType' must be \"sparse\" or \"fixed\"; got \"%1\". Using \"sparse\".")
                    .arg(mGridType->value()));
      mGridType->setValue("sparse");
    }
  }
  // ⚠ THE ONE COMBINATION THAT FAILS SILENTLY AT RUNTIME. A "fixed" grid is
  // sized ONCE from the particle bounds at construction; material that leaves
  // that box produces NaN positions with no exception, no warning and no log
  // line of any kind. Upstream's own coupled example pairs "fixed" with a
  // padding of 50. Say so here, at parse time, where the field is visible.
  if (mGridType && mGridPadding && mGridType->value().trimmed().toLower() == "fixed" &&
      mGridPadding->value() <= 0) {
    parsingWarn(tr("'gridType' is \"fixed\" with 'gridPadding' 0. A fixed MPM grid is allocated "
                   "once from the particle bounds and anything that leaves that box NaNs "
                   "SILENTLY -- no error, no warning, just NaN positions from some step onward. "
                   "Declare a gridPadding (upstream uses 50) or use \"sparse\"."));
  }
}

void OmGranularBed::postFinalize() {
  OmBaseNode::postFinalize();

  if (abstractAppearance())
    abstractAppearance()->postFinalize();
  // Replacing or editing the appearance recolours the bed in place -- no
  // reload, no touched particles. mAppearance's own changed() covers "a
  // different node was plugged in"; the appearance's covers "a field inside it
  // moved".
  if (mAppearance)
    connect(mAppearance, &OmSFNode::changed, this, &OmGranularBed::updateAppearance);
  updateAppearance();

  // Registration is DEFERRED to the first physics step, not done here. See the
  // long comment in onPhysicsStepStarted() for why -- doing it in postFinalize
  // would open the Newton world before WorldInfo's declarations reach the
  // backend, silently costing this world its declared friction and up axis.
  OmSimulationWorld *const world = OmSimulationWorld::instance();
  if (world != NULL)
    connect(world, &OmSimulationWorld::physicsStepStarted, this,
            &OmGranularBed::onPhysicsStepStarted);
}

int OmGranularBed::countRegisteredBeds(const OmNewtonBackend *newton) {
  if (newton == NULL)
    return -1;
  return newton->granularBedCount();
}

void OmGranularBed::onPhysicsStepStarted() {
  if (mRegistrationDone)
    return;
  // One shot whatever the outcome: a failure that re-fires every tick would
  // flood the log (and GET /sim/events) at the basic time step.
  mRegistrationDone = true;

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == NULL || !raw->isAvailable()) {
    OmLog::warning(tr("GranularBed '%1' is inert: the Newton runtime is not available, so there is "
                      "no MPM solver to simulate its particles. It is invisible as well as "
                      "motionless -- a bed's positions come back from the solver and there is no "
                      "rest pose to draw without one.")
                     .arg(usefulName()));
    return;
  }
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);

  // ⚠ DO NOT REPLACE THIS WITH A BARE ensureWorldOpen(). Identical reasoning to
  // OmSoftBody::onPhysicsStepStarted, restated because getting it wrong is
  // silent: the Newton world is opened LAZILY from inside
  // OmSolid::flushPendingNewtonRegistrations(), which first caches WorldInfo's
  // coordinateSystem and its contact/solver declaration into the backend. newton
  // bakes the up axis into the implicit ground plane's normal and copies
  // mu/ke/kd into every shape AT ADD TIME, so a world opened ahead of that cache
  // can never be given them afterwards.
  OmSolid::flushPendingNewtonRegistrations();
  if (!newton->isWorldOpenForBuild()) {
    // A bed-only world: no Solid opened it. The flush above still cached the
    // WorldInfo declarations, so opening it now applies them in the right order.
    if (newton->ensureWorldOpen() != 0) {
      OmLog::warning(
        tr("GranularBed '%1' is inert: the Newton world could not be opened.").arg(usefulName()));
      return;
    }
  }

  // Ask BEFORE registering: the answer is the slot this bed is about to fill.
  const int bedIndex = countRegisteredBeds(newton);

  // By value, not by const reference: the null-guard ternary yields a prvalue,
  // and binding a reference to it is a lifetime subtlety this code does not
  // need to have.
  const OmVector3 t = mTranslation ? mTranslation->value() : OmVector3();
  const OmVector3 s = mSize ? mSize->value() : OmVector3(1.0, 1.0, 0.3);
  const OmRotation r = mRotation ? mRotation->value() : OmRotation();
  const OmQuaternion q = r.toQuaternion();
  // ⚠ The bed's MINIMUM CORNER, not its centre -- newton's convention for
  // add_particle_grid, and the opposite of Solid/Box. Passed through untouched
  // so the .omniworld number and the solver's number are the same number.
  const double pos[3] = {t.x(), t.y(), t.z()};
  const double size[3] = {s.x(), s.y(), s.z()};
  // xyzw on the wire, w LAST -- the house body_q layout. OmQuaternion stores w
  // FIRST, so this reorder is load-bearing, not cosmetic.
  const double quat[4] = {q.x(), q.y(), q.z(), q.w()};

  // ⚠ EVERY MATERIAL VALUE IS FORWARDED VERBATIM, INCLUDING 0, and that is the
  // opposite of what OmSoftBody.cpp does. There, 0 means unset and this file's
  // constants are substituted; here, 0 means unset and THE RUNTIME omits the
  // per-particle custom attribute so newton's own default applies. The reason
  // for the difference is that MPM's defaults are numerous (twelve
  // per-particle attributes) and live inside newton, where a version bump can
  // move them -- duplicating them on this side is a second source of truth that
  // goes stale without anything failing.
  // gridType / gridPadding do NOT ride the positional call: they configure the
  // ONE SolverImplicitMPM every bed in this world shares, so they go through
  // their own setter, where a disagreement between two beds is reported rather
  // than dropped.
  const QByteArray gridType =
    (mGridType ? mGridType->value().trimmed().toLower() : QString("sparse")).toUtf8();
  newton->setGranularGrid(gridType.constData(), mGridPadding ? mGridPadding->value() : 0);

  int particleEnd = -1;
  const int particleStart = newton->addGranularBed(
    pos, quat, size, mVoxelSize ? mVoxelSize->value() : 0.0,
    mParticlesPerCell ? mParticlesPerCell->value() : 0.0, mCount ? mCount->value() : 0,
    mDensity ? mDensity->value() : 0.0, mFriction ? mFriction->value() : 0.0,
    mYieldPressure ? mYieldPressure->value() : 0.0, mYieldStress ? mYieldStress->value() : 0.0,
    mYoungModulus ? mYoungModulus->value() : 0.0, mPoissonRatio ? mPoissonRatio->value() : 0.0,
    mViscosity ? mViscosity->value() : 0.0, mParticleRadius ? mParticleRadius->value() : 0.0,
    mRigidSubsteps ? mRigidSubsteps->value() : 4, mProxyIterations ? mProxyIterations->value() : 0,
    mMaxIterations ? mMaxIterations->value() : 0, mTolerance ? mTolerance->value() : 0.0,
    &particleEnd);
  if (particleStart < 0) {
    OmLog::warning(tr("GranularBed '%1' registered no particles and will neither move nor render. "
                      "The two causes worth checking first are a refused particle count (a bed "
                      "declaring no `count` budget is capped at 4 million particles -- see the "
                      "engine log for the exact figure) and a Newton runtime whose newton package "
                      "predates SolverImplicitMPM.")
                     .arg(usefulName()));
    return;
  }
  mParticleStart = particleStart;
  mParticleEnd = particleEnd;
  mBedIndex = bedIndex;

  // ⚠ THE RADIUS IS FETCHED, NOT READ OFF THE FIELD. `particleRadius` may be 0
  // (unset), and even when it is set the DERIVED value depends on the final
  // lattice spacing after any coarsening the `count` budget forced. Drawing the
  // bed at the authored number would show particles at a size the solver is not
  // simulating -- which is exactly the sort of "looks about right" rendering
  // that hides a coarsened grid.
  if (mBedIndex >= 0)
    mParticleRadiusEffective = newton->granularParticleRadius(mBedIndex);
  if (!(mParticleRadiusEffective > 0.0)) {
    OmLog::warning(tr("GranularBed '%1' simulates but will not be rendered: the runtime did not "
                      "report a particle radius for bed index %2, so there is no honest size to "
                      "draw its particles at.")
                     .arg(usefulName())
                     .arg(mBedIndex));
  }

  mHostPositions.assign(static_cast<std::size_t>(mParticleEnd - mParticleStart) * 3, 0.0f);
  mHavePositions = false;

  OmLog::info(tr("GranularBed '%1': registered %2 particles at Newton particle offset %3 "
                 "(bed index %4, particle radius %5 m). The bed is invisible until the first "
                 "physics step -- its positions are read back from the solver.")
                .arg(usefulName())
                .arg(mParticleEnd - mParticleStart)
                .arg(mParticleStart)
                .arg(mBedIndex)
                .arg(mParticleRadiusEffective, 0, 'g', 4));
}

void OmGranularBed::updateAppearance() {
  // Same shape as OmCloth / OmSoftBody: rebind to whichever appearance node is
  // plugged in now, so editing a field inside it repaints without a reload.
  // Qt::UniqueConnection because this runs on every swap and the previous
  // connection to the same object must not be duplicated.
  if (pbrAppearance())
    connect(pbrAppearance(), &OmPbrAppearance::changed, this, &OmGranularBed::updateAppearance,
            Qt::UniqueConnection);
  else if (appearance())
    connect(appearance(), &OmAppearance::changed, this, &OmGranularBed::updateAppearance,
            Qt::UniqueConnection);
}

void OmGranularBed::reset(const QString &id) {
  OmBaseNode::reset(id);
  // ⚠ NOTHING IS RESTORED HERE, AND THAT IS NOT AN OVERSIGHT. A reset rewinds
  // the ENGINE's saved node state; the particles live in the Newton model,
  // which OmniSim rebuilds on a world load rather than rewinding in place, and
  // there is no MPM state-restore path to call. Clearing the readback flag is
  // the honest half: the bed stops drawing a pose that no longer describes the
  // solver, and picks up again on the next step.
  mHavePositions = false;
}
