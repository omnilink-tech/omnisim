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

#ifndef WB_WORLD_INFO_HPP
#define WB_WORLD_INFO_HPP

#include "WbBaseNode.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFString.hpp"
#include "WbSFVector3.hpp"
#include "WbVector3.hpp"

class WbReceiver;
class WbDamping;
class WbContactProperties;
class WbVersion;

class WbWorldInfo : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbWorldInfo(WbTokenizer *tokenizer = NULL);
  WbWorldInfo(const WbWorldInfo &other);
  explicit WbWorldInfo(const WbNode &other);
  virtual ~WbWorldInfo() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_WORLD_INFO; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  // field accessors
  const WbMFString &info() const { return *mInfo; }
  const QString &title() const { return mTitle->value(); }
  const QString &window() const { return mWindow->value(); }
  double gravity() const { return mGravity->value(); }
  double cfm() const { return mCfm->value(); }
  double erp() const { return mErp->value(); }
  const QString &physics() const { return mPhysics->value(); }
  // ODE broadphase selection (§8.4 of rendering-roadmap.md; landed).
  // Returns the raw field value: "simple" (the default, matches today's
  // dSimpleSpace behavior), "sap", "quadtree", or "auto". For the
  // already-resolved value (auto → sap or quadtree depending on top-
  // level Solid positions), use resolveBroadphaseChoice().
  const QString &broadphase() const { return mBroadphase->value(); }
  // Newton solver choice (WorldInfo.newtonSolver): "" / "auto" / "xpbd" keep
  // the default GPU XPBD path; "mujoco" selects SolverMuJoCo (robust frictional
  // contact, required for pinch grasps). Read by WbSolid's Newton flush and
  // plumbed to the backend before finalize.
  const QString &newtonSolver() const { return mNewtonSolver->value(); }
  // Newton sub-steps per tick + static-collider registration (WorldInfo
  // newtonSubsteps / newtonStatics; default-flip-plan.md §4.2 N3). Fold the
  // OMNISIM_NEWTON_SUBSTEPS / OMNISIM_NEWTON_STATICS launch knobs into the
  // world file; the defaults (1 / false) preserve today's exact behavior, and
  // the env vars still override. Read by WbSolid's Newton flush and plumbed to
  // the backend before finalize.
  int newtonSubsteps() const { return mNewtonSubsteps->value(); }
  bool newtonStatics() const { return mNewtonStatics->value(); }
  // Give robot-wrapper bodies (a URDFRobot's chassis Solid) their OWN
  // boundingObject as a Newton collider, so the robot BODY -- not just its
  // wheels/feet -- collides with scene geometry (walls, racks, crates). The
  // default skips it: a chassis envelope box that engulfs the wheel space
  // would pin the body on the ground and starve the wheels of load (P3.10i/j
  // in WbSolid.cpp). FALSE preserves that wheel-only default everywhere; TRUE
  // folds the OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE launch knob into the world
  // file (the env var still forces it on) for chassis whose collider clears
  // the wheels (e.g. the warehouse Husky, whose box floats ~0.13 m up).
  bool newtonRobotColliders() const { return mNewtonRobotColliders->value(); }
  // Register EVERY collider in a compound boundingObject (a Group of offset
  // primitives on ONE rigid body -- e.g. a movable bin's floor + 4 walls) as its
  // own Newton shape, not just the first child. FALSE preserves the first-child-
  // only default for every existing world; TRUE folds the
  // OMNISIM_NEWTON_COMPOUND_COLLIDERS launch knob into the .wbt (the env var still
  // forces it on). Read by WbSolid's Newton shape attach (per-world, not static,
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
  // createOdeObjects time -- WbWorld::topSolids is populated by then.
  QString resolveBroadphaseChoice() const;
  double basicTimeStep() const { return mBasicTimeStep->value(); }
  double fps() const { return mFps->value(); }
  int optimalThreadCount() const { return mOptimalThreadCount->value(); }
  double physicsDisableTime() const { return mPhysicsDisableTime->value(); }
  double physicsDisableLinearThreshold() const { return mPhysicsDisableLinearThreshold->value(); }
  double physicsDisableAngularThreshold() const { return mPhysicsDisableAngularThreshold->value(); }
  WbDamping *defaultDamping() const;
  double lineScale() const;
  double dragForceScale() const { return mDragForceScale->value(); };
  double dragTorqueScale() const { return mDragTorqueScale->value(); };
  const QString &coordinateSystem() const { return mCoordinateSystem->value(); }
  const QString &gpsCoordinateSystem() const { return mGpsCoordinateSystem->value(); }
  const WbVector3 &gpsReference() const { return mGpsReference->value(); }
  int randomSeed() const { return mRandomSeed->value(); }
  int contactPropertiesCount() const;
  const WbMFNode &contactProperties() const { return *mContactProperties; }
  WbContactProperties *contactProperties(int index) const;
  double inkEvaporation() const { return mInkEvaporation->value(); }

  // Enums
  enum { X, Y, Z };

  // other accessors

  const WbVector3 &eastVector() const { return mEastVector; }
  const WbVector3 &northVector() const { return mNorthVector; }
  const WbVector3 &upVector() const { return mUpVector; }
  // returns the gravity vector (oriented along the down axis)
  const WbVector3 &gravityVector() const { return mGravityVector; }
  // returns a unit vector with the direction and orientation of the gravity
  const WbVector3 &gravityUnitVector() const { return mGravityUnitVector; }

  const WbReceiver *physicsReceiver() const { return mPhysicsReceiver; }

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
  WbWorldInfo &operator=(const WbWorldInfo &);  // non copyable
  WbNode *clone() const override { return new WbWorldInfo(*this); }
  void init(const WbVersion *version = NULL);

  // User accessible fields
  WbMFString *mInfo;
  WbSFString *mTitle;
  WbSFString *mWindow;
  WbSFDouble *mGravity;
  WbSFDouble *mCfm;
  WbSFDouble *mErp;
  WbSFString *mPhysics;
  WbSFString *mBroadphase;
  WbSFString *mNewtonSolver;
  WbSFInt *mNewtonSubsteps;
  WbSFBool *mNewtonStatics;
  WbSFBool *mNewtonRobotColliders;
  WbSFBool *mNewtonCompoundColliders;
  WbSFString *mDefaultPhysicsBackend;
  WbSFString *mDefaultRenderBackend;
  WbSFDouble *mBasicTimeStep;
  WbSFDouble *mFps;
  WbSFInt *mOptimalThreadCount;
  WbSFDouble *mPhysicsDisableTime;
  WbSFDouble *mPhysicsDisableLinearThreshold;
  WbSFDouble *mPhysicsDisableAngularThreshold;
  WbSFNode *mDefaultDamping;
  WbSFDouble *mInkEvaporation;
  WbSFString *mCoordinateSystem;
  WbSFString *mGpsCoordinateSystem;
  WbSFVector3 *mGpsReference;
  WbSFDouble *mLineScale;
  WbSFDouble *mDragForceScale;
  WbSFDouble *mDragTorqueScale;
  WbSFInt *mRandomSeed;
  WbMFNode *mContactProperties;

  // physics receiver node
  WbReceiver *mPhysicsReceiver;

  // Gravity variables
  WbVector3 mEastVector;
  WbVector3 mNorthVector;
  WbVector3 mUpVector;
  WbVector3 mGravityVector;
  WbVector3 mGravityUnitVector;

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
