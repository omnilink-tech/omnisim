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

#ifndef OM_WORLD_INFO_HPP
#define OM_WORLD_INFO_HPP

#include "OmBaseNode.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFString.hpp"
#include "OmSFVector3.hpp"
#include "OmVector3.hpp"

class OmDamping;
class OmContactProperties;
class OmVersion;

class OmWorldInfo : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmWorldInfo(OmTokenizer *tokenizer = NULL);
  OmWorldInfo(const OmWorldInfo &other);
  explicit OmWorldInfo(const OmNode &other);
  virtual ~OmWorldInfo() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_WORLD_INFO; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  // field accessors
  const OmMFString &info() const { return *mInfo; }
  const QString &title() const { return mTitle->value(); }
  const QString &window() const { return mWindow->value(); }
  double gravity() const { return mGravity->value(); }
  double cfm() const { return mCfm->value(); }
  double erp() const { return mErp->value(); }
  // ODE broadphase selection (§8.4 of rendering-roadmap.md; landed).
  // Returns the raw field value: "simple" (the default, matches today's
  // dSimpleSpace behavior), "sap", "quadtree", or "auto". For the
  // already-resolved value (auto → sap or quadtree depending on top-
  // level Solid positions), use resolveBroadphaseChoice().
  const QString &broadphase() const { return mBroadphase->value(); }
  // Newton solver choice (WorldInfo.newtonSolver). ⚠ THIS COMMENT USED TO
  // DESCRIBE AN XPBD DEFAULT THAT NO LONGER EXISTS -- XPBD was removed
  // 2026-08-07 and SolverMuJoCo is the only rigid solver, so the field does
  // not choose a solver FAMILY any more, it chooses where that solver runs:
  //   "" / "auto" / "mujoco" -> the reference CPU mj_step (deterministic)
  //   "mujoco_warp"          -> the same solver batched on the GPU
  //   "mujoco+vbd"           -> CPU mj_step COUPLED with newton's SolverVBD,
  //                             the only value under which a `Cloth` node
  //                             simulates (a Cloth in any other world is
  //                             registered nowhere and neither falls nor
  //                             renders -- see OmCloth.cpp).
  // Passed VERBATIM to the runtime; the string-to-solver mapping lives there,
  // not here. Read by OmSolid's Newton flush and plumbed to the backend
  // before finalize.
  const QString &newtonSolver() const { return mNewtonSolver->value(); }
  // Newton sub-steps per tick + static-collider registration (WorldInfo
  // newtonSubsteps / newtonStatics; default-flip-plan.md §4.2 N3). Fold the
  // OMNISIM_NEWTON_SUBSTEPS / OMNISIM_NEWTON_STATICS launch knobs into the
  // world file; the defaults (1 / false) preserve today's exact behavior, and
  // the env vars still override. Read by OmSolid's Newton flush and plumbed to
  // the backend before finalize.
  int newtonSubsteps() const { return mNewtonSubsteps->value(); }
  // Newton/MuJoCo friction-cone type + impratio (WorldInfo newtonCone /
  // newtonImpratio). "" / 0 = MuJoCo stock (pyramidal cone, impratio 1),
  // preserving today's contact physics for every existing world.
  // "elliptic" + impratio 10 removes the inscribed-pyramid creep near the
  // friction-cone boundary (omnibench T2). The OMNISIM_NEWTON_CONE /
  // OMNISIM_NEWTON_IMPRATIO env vars still override. Read by OmSolid's
  // Newton flush and plumbed to the backend before finalize.
  const QString &newtonCone() const { return mNewtonCone->value(); }
  double newtonImpratio() const { return mNewtonImpratio->value(); }
  // Newton/MuJoCo contact dimensionality (WorldInfo newtonCondim). 0 (default)
  // = unset, leaving whatever newton built -- MEASURED to be condim 3 on every
  // geom of every OmniSim world, i.e. sliding friction only, with the
  // torsional and rolling coefficients newton writes into geom_friction[1:2]
  // never consulted. 1 = frictionless, 3 = sliding, 4 = + torsional (what a
  // two-finger pinch wants: without it the part spins freely about the contact
  // normal), 6 = + rolling. The OMNISIM_NEWTON_CONDIM env var still overrides.
  // Read by OmSolid's Newton flush and plumbed to the backend before finalize.
  int newtonCondim() const { return mNewtonCondim->value(); }
  // Newton/MuJoCo NOSLIP post-solve iterations (WorldInfo
  // newtonNoslipIterations -> mjOption.noslip_iterations). 0 (default) = unset
  // == MuJoCo's own stock 0, so every existing world is byte-identical.
  // It is a Gauss-Seidel pass over the FRICTION constraints ONLY, run after
  // the main solve, whose purpose is removing the tangential drift a soft
  // friction constraint accumulates under sustained load -- i.e. exactly the
  // "the pinch holds at the commanded normal force and the part still creeps
  // out" failure. MEASURED on ladder0 rung 8: 0 creeps 56 mm and drops the
  // part; >=1 carries it, insensitive to the count from 3 up, which is what
  // makes it a solver BUDGET rather than a tuned number.
  // ⚠ CPU ONLY -- mujoco_warp has no noslip field at all and its put_model
  // RAISES on a non-zero value, so the backend refuses to apply it there and
  // warns once. The OMNISIM_NEWTON_NOSLIP env var still overrides (value
  // parsed, so =0 is the exact-revert hatch). Read by OmSolid's Newton flush
  // and plumbed to the backend before finalize.
  int newtonNoslipIterations() const { return mNewtonNoslipIterations->value(); }
  // Whether cloth/soft PARTICLES collide with each other (WorldInfo
  // newtonClothSelfContact -> SolverVBD particle_enable_self_contact and the
  // self-contact radius/margin derived from the authored particleRadius).
  // -1 (the default) = unset, leaving the runtime's own default, which is ON.
  // 0 = OFF, 1 = ON.
  // ⚠ There is no single right answer and the gap is 24x. DRAPING needs it on
  // -- newton's own default is OFF and with it off a fold passes through
  // itself silently, which is wrong for the normal cloth case. GRASPING needs
  // it off: measured on the patch world a pinched fold tracks to -22.11 mm
  // with it on and -0.92 mm with it off, because fabric gathered between the
  // pads pushes ITSELF back out of the jaws.
  // Reachable ONLY through OMNISIM_CLOTH_SELF_CONTACT until 2026-08-15, so a
  // deformable-grasp world could not state its own physics and a forgotten
  // env var did not fail -- it slipped 24x in silence. The env var still
  // overrides (env > field > default, value-parsed, so =1 exact-reverts a
  // world declaring 0). Read by OmSolid's Newton flush and plumbed to the
  // backend before finalize.
  int newtonClothSelfContact() const { return mNewtonClothSelfContact->value(); }
  // Per-world Newton contact friction / compliance / solver iteration counts
  // (WorldInfo newtonGroundMu / newtonContactKe / newtonContactKd /
  // newtonIterations / newtonLsIterations). 0 on any of them = unset, keeping
  // the runtime defaults (mu 1.0, ke 2500, kd 100, solver's own iterations),
  // so every existing world is byte-identical. These five were reachable ONLY
  // through OMNISIM_NEWTON_* environment variables until 2026-08-02, which
  // made a .wbt an incomplete description of its own physics -- a tuned
  // friction grasp could not be reproduced from the file alone. The env vars
  // still override. Read by OmSolid's Newton flush, plumbed before finalize.
  double newtonGroundMu() const { return mNewtonGroundMu->value(); }
  double newtonContactKe() const { return mNewtonContactKe->value(); }
  double newtonContactKd() const { return mNewtonContactKd->value(); }
  int newtonIterations() const { return mNewtonIterations->value(); }
  int newtonLsIterations() const { return mNewtonLsIterations->value(); }
  // Newton/MuJoCo constraint-row + contact buffer caps (WorldInfo newtonNjmax /
  // newtonNconmax). 0 (default) = unset, keeping the engine's built-in 256 and
  // so every existing world byte-identical. A positive value raises the cap;
  // -1 asks newton for its own auto-estimate. Needed by multi-robot fleet
  // worlds: a 4-wheel-drive Husky rests on 8 wheel-ground contacts = 32
  // constraint rows (measured), so ten of them peak at nefc=320 and exceed
  // 256, and mujoco_warp then per-tick-printfs "nefc overflow" from inside
  // the kernel AND silently truncates the constraint vector. The
  // OMNISIM_NEWTON_NJMAX / OMNISIM_NEWTON_NCONMAX env vars still override.
  // Read by OmSolid's Newton flush and plumbed to the backend before finalize.
  int newtonNjmax() const { return mNewtonNjmax->value(); }
  int newtonNconmax() const { return mNewtonNconmax->value(); }
  bool newtonStatics() const { return mNewtonStatics->value(); }
  // Give robot-wrapper bodies (a URDFRobot's chassis Solid) their OWN
  // boundingObject as a Newton collider, so the robot BODY -- not just its
  // wheels/feet -- collides with scene geometry (walls, racks, crates). The
  // default skips it: a chassis envelope box that engulfs the wheel space
  // would pin the body on the ground and starve the wheels of load (P3.10i/j
  // in OmSolid.cpp). FALSE preserves that wheel-only default everywhere; TRUE
  // folds the OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE launch knob into the world
  // file (the env var still forces it on) for chassis whose collider clears
  // the wheels (e.g. the warehouse Husky, whose box floats ~0.13 m up).
  bool newtonRobotColliders() const { return mNewtonRobotColliders->value(); }
  // Register EVERY collider in a compound boundingObject (a Group of offset
  // primitives on ONE rigid body -- e.g. a movable bin's floor + 4 walls) as its
  // own Newton shape, not just the first child. FALSE preserves the first-child-
  // only default for every existing world; TRUE folds the
  // OMNISIM_NEWTON_COMPOUND_COLLIDERS launch knob into the .wbt (the env var still
  // forces it on). Read by OmSolid's Newton shape attach (per-world, not static,
  // so a world switched in via the launcher reload reads ITS OWN field).
  bool newtonCompoundColliders() const { return mNewtonCompoundColliders->value(); }
  // World-level default backends (default-flip-plan.md §3.2). When non-empty ("ode"/"newton" and
  // "wren"/"wgpu" respectively) they supply the choice for any node still on the "auto"/unspecified
  // sentinel — i.e. pin a whole world to one backend without editing every Solid/Viewpoint. An
  // explicit per-node (or, for physics, per-articulation-ancestor) backend still wins. Empty = inert.
  const QString &defaultPhysicsBackend() const { return mDefaultPhysicsBackend->value(); }
  const QString &defaultRenderBackend() const { return mDefaultRenderBackend->value(); }
  // Computes the effective broadphase: "auto" is resolved here using
  // the world AABB heuristic from rendering-roadmap.md §8.4
  // (quadtree when world spans > 500 m on the largest axis, else sap).
  // Any other field value is returned unchanged. Safe to call from
  // createOdeObjects time -- OmWorld::topSolids is populated by then.
  QString resolveBroadphaseChoice() const;
  double basicTimeStep() const { return mBasicTimeStep->value(); }
  double fps() const { return mFps->value(); }
  int optimalThreadCount() const { return mOptimalThreadCount->value(); }
  double physicsDisableTime() const { return mPhysicsDisableTime->value(); }
  double physicsDisableLinearThreshold() const { return mPhysicsDisableLinearThreshold->value(); }
  double physicsDisableAngularThreshold() const { return mPhysicsDisableAngularThreshold->value(); }
  OmDamping *defaultDamping() const;
  double lineScale() const;
  double dragForceScale() const { return mDragForceScale->value(); };
  double dragTorqueScale() const { return mDragTorqueScale->value(); };
  const QString &coordinateSystem() const { return mCoordinateSystem->value(); }
  const QString &gpsCoordinateSystem() const { return mGpsCoordinateSystem->value(); }
  const OmVector3 &gpsReference() const { return mGpsReference->value(); }
  int randomSeed() const { return mRandomSeed->value(); }
  int contactPropertiesCount() const;
  const OmMFNode &contactProperties() const { return *mContactProperties; }
  OmContactProperties *contactProperties(int index) const;
  double inkEvaporation() const { return mInkEvaporation->value(); }

  // Enums
  enum { X, Y, Z };

  // other accessors

  const OmVector3 &eastVector() const { return mEastVector; }
  const OmVector3 &northVector() const { return mNorthVector; }
  const OmVector3 &upVector() const { return mUpVector; }
  // returns the gravity vector (oriented along the down axis)
  const OmVector3 &gravityVector() const { return mGravityVector; }
  // returns a unit vector with the direction and orientation of the gravity
  const OmVector3 &gravityUnitVector() const { return mGravityUnitVector; }

  void createOdeObjects() override;
  void createWrenObjects() override;

signals:
  void gpsCoordinateSystemChanged();
  void gpsReferenceChanged();
  void physicsDisableChanged();
  void titleChanged();
  void globalPhysicsPropertiesChanged();
  void optimalThreadCountChanged();
  void randomSeedChanged();

private:
  OmWorldInfo &operator=(const OmWorldInfo &);  // non copyable
  OmNode *clone() const override { return new OmWorldInfo(*this); }
  void init(const OmVersion *version = NULL);

  // User accessible fields
  OmMFString *mInfo;
  OmSFString *mTitle;
  OmSFString *mWindow;
  OmSFDouble *mGravity;
  OmSFDouble *mCfm;
  OmSFDouble *mErp;
  OmSFString *mPhysics;
  OmSFString *mBroadphase;
  OmSFString *mNewtonSolver;
  OmSFInt *mNewtonSubsteps;
  OmSFString *mNewtonCone;
  OmSFDouble *mNewtonImpratio;
  OmSFInt *mNewtonCondim;
  OmSFInt *mNewtonNoslipIterations;
  OmSFInt *mNewtonClothSelfContact;
  OmSFDouble *mNewtonGroundMu;
  OmSFDouble *mNewtonContactKe;
  OmSFDouble *mNewtonContactKd;
  OmSFInt *mNewtonIterations;
  OmSFInt *mNewtonLsIterations;
  OmSFInt *mNewtonNjmax;
  OmSFInt *mNewtonNconmax;
  OmSFBool *mNewtonStatics;
  OmSFBool *mNewtonRobotColliders;
  OmSFBool *mNewtonCompoundColliders;
  OmSFString *mDefaultPhysicsBackend;
  OmSFString *mDefaultRenderBackend;
  OmSFDouble *mBasicTimeStep;
  OmSFDouble *mFps;
  OmSFInt *mOptimalThreadCount;
  OmSFDouble *mPhysicsDisableTime;
  OmSFDouble *mPhysicsDisableLinearThreshold;
  OmSFDouble *mPhysicsDisableAngularThreshold;
  OmSFNode *mDefaultDamping;
  OmSFDouble *mInkEvaporation;
  OmSFString *mCoordinateSystem;
  OmSFString *mGpsCoordinateSystem;
  OmSFVector3 *mGpsReference;
  OmSFDouble *mLineScale;
  OmSFDouble *mDragForceScale;
  OmSFDouble *mDragTorqueScale;
  OmSFInt *mRandomSeed;
  OmMFNode *mContactProperties;

  // Gravity variables
  OmVector3 mEastVector;
  OmVector3 mNorthVector;
  OmVector3 mUpVector;
  OmVector3 mGravityVector;
  OmVector3 mGravityUnitVector;

  // Apply methods
  void applyLineScaleToWren();
  void applyToOdeGravity();
  void applyToOdeCfm();
  void applyToOdeErp();
  // Non-slot update methods
  void applyToOdeGlobalDamping();
  void applyToOdePhysicsDisableTime();
  void updateGravityBasis();

private slots:
  void updateBasicTimeStep();
  void updateFps();
  void updateOptimalThreadCount();
  void updateLineScale();
  void updateDragForceScale();
  void updateDragTorqueScale();
  void updateRandomSeed();
  void updateGravity();
  void updateCfm();
  void updateErp();
  void updateDefaultDamping();
  void updateCoordinateSystem();
  void updateGpsCoordinateSystem();
  void updateContactProperties();
  void displayOptimalThreadCountWarning();
};

#endif
