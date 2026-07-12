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

#ifndef WB_NEWTON_BACKEND_HPP
#define WB_NEWTON_BACKEND_HPP

#include <QtCore/QString>
#include <vector>

//
// WbNewtonBackend — opt-in GPU-resident physics backend, wraps NVIDIA
// Newton (Apache 2.0, https://github.com/newton-physics/newton). Lives
// alongside WbOdeBackend; world authors opt a WbSolid into this
// backend via the (forthcoming) `physicsBackend "newton"` field.
//
// Build modes
// -----------
//
// OMNISIM_WITH_NEWTON=ON (default since the Stage 3 build-flag flip,
// 2026-06-06 -- see src/omnisim/Makefile + docs/developer/default-flip-plan.md):
//   - Newton library code is compiled in. isAvailable() actually brings up
//     the Newton runtime (warp + newton Python packages, embedded via the
//     stable CPython API) and returns true on success; on a box without the
//     runtime (it is NOT bundled) or without NVIDIA hardware it returns
//     false and the dispatcher falls back to WbOdeBackend -- safe, but not
//     actually Newton. The upstream PyPI package was renamed from
//     `newton-physics` to `newton` in late 2025; install with
//     `pip install "newton[examples]" warp-lang`.
//
// OMNISIM_WITH_NEWTON=OFF:
//   - Pure-ODE legacy build: WbNewtonBackend exists but isAvailable()
//     returns false, every Newton request falls back to WbOdeBackend at the
//     dispatcher layer, and zero Newton library code is linked into the
//     binary.
//
// This file is included on every build, regardless of
// OMNISIM_WITH_NEWTON, because the dispatcher needs to know that
// "newton" is a recognised backend kind even when the runtime is
// unavailable. The .cpp toggles the actual runtime hookup.
//

#include "WbPhysicsBackend.hpp"

// Opaque Python-state holder; defined in WbNewtonBackend.cpp where the
// embedded CPython headers can be safely included. Keeping the
// declaration opaque means no header in the rest of the tree pulls in
// <Python.h>, which would conflict with Qt's `slots`/`signals` macros.
struct WbNewtonRuntimeState;

// W4.1/W4.2: one native Newton rigid contact. bodyA/bodyB are Newton body indices (the same space as
// mNewtonBodyIndex / the pose readback / body_f writes; -1 = static world geometry). point + normal are world
// frame; depth is the penetration along the normal; forceMag is 0 under XPBD (positional solve).
struct WbNewtonContact {
  int bodyA;
  int bodyB;
  double point[3];
  double normal[3];
  double depth;
  double forceMag;
};

class WbNewtonBackend : public WbPhysicsBackend {
public:
  WbNewtonBackend();
  ~WbNewtonBackend() override;
  WbPhysicsBackendKind kind() const override { return WbPhysicsBackendKind::Newton; }
  const char *name() const override { return "newton"; }
  bool isAvailable() const override { return mAvailable; }

  // World-build-phase API (called once per WbWorld load, before sim
  // starts). Each returns 0 on success, -1 on failure; failures emit
  // a single WbLog::warning identifying the step. Calling these on a
  // backend with isAvailable()==false is a no-op success: the registry
  // never routes solids here in that state, so the methods exist only
  // to keep call-sites simple.

  // Resets any prior world state and starts a fresh ModelBuilder.
  // Idempotent: safe to call between world loads.
  int beginWorld();
  // Idempotent: opens a world (with a default ground plane) if one
  // isn't already open. Solids that opt into Newton can call this in
  // postFinalize without coordinating with siblings; the first call
  // wins and subsequent calls are no-ops. Returns 0 on a freshly-opened
  // world, 0 if already open, -1 on error.
  int ensureWorldOpen();
  // True iff a world has been begun and is in the build phase
  // (addBody/addShapeSphere accept). Becomes false once finalizeWorld()
  // succeeds (the world is then in the run phase).
  bool isWorldOpenForBuild() const;
  // True iff the world has been finalized (step/getBodyPosition accept).
  bool isWorldRunning() const;
  // Adds an infinite static ground plane to the model under construction.
  int addGroundPlane();
  // Adds a dynamic rigid body with the given mass at world position
  // (x, y, z) and orientation given by the (qx, qy, qz, qw) quaternion
  // (xyzw layout matching Newton's body_q convention). Returns the
  // body's index (>= 0) on success, -1 on error.
  // hasCom (default false): when false the link's center of mass defaults to
  // the link origin (legacy behavior, every existing Newton robot is validated
  // against it). When true the (cx, cy, cz) link-frame COM offset is passed
  // through to the runtime's add_body. Opt-in via OMNISIM_NEWTON_USE_LINK_COM.
  int addBody(double mass, double x, double y, double z,
              double qx, double qy, double qz, double qw,
              double ixx = 0.0, double iyy = 0.0, double izz = 0.0,
              double ixy = 0.0, double ixz = 0.0, double iyz = 0.0,
              bool hasCom = false, double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // P8.1 statics-on-Newton: adds a STATIC rigid body (mass=0) at world
  // pose (x, y, z) + quaternion (qx, qy, qz, qw). Static bodies have
  // their xform pinned by the solver — `step()` won't move them, but
  // collision shapes attached via addShape* are tested against
  // dynamic-body shapes in the contact phase. Returns the body's
  // index (>= 0) on success, -1 on error. See
  // docs/developer/physics-p8-statics-design.md §4.
  int addStaticBody(double x, double y, double z,
                    double qx, double qy, double qz, double qw);
  // Attaches a sphere collision/visual shape to bodyIdx. Returns 0/-1.
  // (cx, cy, cz) is an offset in the body's local frame -- default
  // (0,0,0) places the sphere at the body origin.
  int addShapeSphere(int bodyIdx, double radius,
                     double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // Attaches a box collision/visual shape to bodyIdx. (hx, hy, hz) are
  // half-extents along each local axis (Newton's convention -- WbBox
  // uses full size, so callers must divide by 2). (cx, cy, cz) is the
  // box centre offset in the body's local frame (default origin).
  int addShapeBox(int bodyIdx, double hx, double hy, double hz,
                  double cx = 0.0, double cy = 0.0, double cz = 0.0,
                  double ke = -1.0);
  // Cylinder + capsule. halfHeight is total length / 2 -- WbCylinder and
  // WbCapsule both expose `height()` as the full length, so callers must
  // divide. Newton's capsule/cylinder extend along LOCAL Z; a Webots
  // Cylinder bounding object extends along its body-local Y, so addShapeCylinder
  // (W1.2) rotates the substituted capsule -90 deg about X (Z->Y) and accepts a
  // local offset (cx,cy,cz). NOTE: native Newton cylinder narrow-phase locks
  // wheels (probe 7), so addShapeCylinder substitutes a same-radius/half-height
  // capsule (line contact, robust). addShapeCapsule still uses Newton's default
  // Z axis (a known latent mismatch for non-Z capsule bounding objects -- not
  // yet exercised in the corpus). Returns 0/-1.
  int addShapeCylinder(int bodyIdx, double radius, double halfHeight,
                       double cx = 0.0, double cy = 0.0, double cz = 0.0);
  int addShapeCapsule(int bodyIdx, double radius, double halfHeight);
  // Infinite static ground plane (newton-ode-replacement-plan.md W1.1) -- e.g. a Floor's Plane
  // boundingObject. Local normal +Z (the WbPlane convention); (cx,cy,cz) is the local offset, the body's
  // transform orients it. Returns 0/-1.
  int addShapePlane(int bodyIdx, double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // Native triangle-mesh collision (newton-ode-replacement-plan.md W1) -- replaces the AABB-box
  // approximation. vertices = flat 3*nVertices doubles, indices = flat 3*nTriangles vertex indices;
  // (cx,cy,cz) is the mesh's local offset in the body frame. compute_inertia is OFF (the body already
  // carries the Solid's mass + inertia). Returns 0/-1.
  int addShapeMesh(int bodyIdx, const double *vertices, int nVertices,
                   const int *indices, int nTriangles,
                   double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // W3.1 external-wrench injection (newton-ode-replacement-plan.md): queue a per-tick WORLD-frame force
  // (fx,fy,fz) + torque (tx,ty,tz) on a Newton body (about the body's reference point). step() sums these
  // into state.body_f and clears them after the tick -- ODE addBodyForce semantics (re-applied each tick).
  // Valid during simulation (no openForBuild). Returns 0/-1.
  int addBodyForce(int bodyIdx, double fx, double fy, double fz,
                   double tx, double ty, double tz);
  // W3.2 mid-step velocity set: write a Newton body's linear (angular=0) or angular (angular=1) velocity
  // straight into body_qd ([vx,vy,vz, wx,wy,wz], world frame). Persistent state (not re-applied per tick).
  // Valid during simulation. Returns 0/-1.
  int setBodyVel(int bodyIdx, double x, double y, double z, int angular);
  // W4.1/W4.2 native contact readback: snapshot this step's rigid contacts (filled by model.collide every
  // substep) into `out`. Replaces the ODE collision bridge as the contact source for the damage tracker +
  // sensors. Returns the contact count (>=0) or -1 if unavailable. World-frame points/normals.
  int getContacts(std::vector<WbNewtonContact> &out) const;
  // Adds a 1-DoF revolute (hinge) joint between parent and child bodies.
  //   - axis (ax, ay, az): rotation axis in local space (length 1).
  //   - parentAnchor (parent_x/y/z): hinge attachment in parent's frame.
  //   - childAnchor (child_x/y/z):   hinge attachment in child's frame.
  //   - targetKe / targetKd: PD gains for joint actuation. Pass 0 to
  //     get a free-spinning hinge (no motor); pass non-zero to enable
  //     velocity targets via setJointTargetVelocity. Defaults are 0.
  // Both bodies must already exist via addBody. Returns the joint index
  // (>= 0) on success, -1 on error.
  // childRot* = the child link's authored rotation relative to its joint
  // parent (quaternion of R_child^T * R_parent), baked into the joint's
  // child_xform so a revolute constraint preserves the child Solid's
  // off-axis `rotation` instead of projecting it away. Identity default
  // keeps axis-aligned children (URDF wheels) byte-unchanged.
  int addJointRevolute(int parentIdx, int childIdx,
                       double ax, double ay, double az,
                       double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                       double childAnchorX, double childAnchorY, double childAnchorZ,
                       double targetKe = 0.0, double targetKd = 0.0,
                       double limitLower = 0.0, double limitUpper = 0.0,
                       double effortLimit = 0.0, double velocityLimit = 0.0,
                       double childRotX = 0.0, double childRotY = 0.0,
                       double childRotZ = 0.0, double childRotW = 1.0);
  // Prismatic (linear/slider) joint -- e.g. parallel-gripper fingers. Same
  // queue/topo-sort/gain path as addJointRevolute; the slot it returns is
  // used with setJointTarget{Position,Velocity} exactly like a revolute.
  int addJointPrismatic(int parentIdx, int childIdx,
                        double ax, double ay, double az,
                        double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                        double childAnchorX, double childAnchorY, double childAnchorZ,
                        double targetKe = 0.0, double targetKd = 0.0,
                        double limitLower = 0.0, double limitUpper = 0.0,
                        double effortLimit = 0.0, double velocityLimit = 0.0);
  // Hinge2 / universal joint -- 2-DoF rotation about two axes sharing one anchor (a caster's steer + roll,
  // or a car front wheel). Built natively as a Newton d6 joint with two FREE angular DoF (passive); the
  // capability gate admits it alongside Hinge/Slider (newton-ode-replacement-plan.md W2).
  int addJointHinge2(int parentIdx, int childIdx,
                     double ax1, double ay1, double az1,
                     double ax2, double ay2, double az2,
                     double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                     double childAnchorX, double childAnchorY, double childAnchorZ);
  // Ball / spherical joint -- 3-DoF rotation about a shared anchor, zero relative translation (common in
  // legged/soft rigs). Built natively as a Newton ball joint (JointType.BALL: a quaternion-based spherical
  // constraint, gimbal-free, not a d6 Euler triple). PASSIVE: ball target pos/vel control is MuJoCo-only
  // upstream, so a motorised ball joint is a follow-up; the capability gate admits it alongside
  // Hinge/Slider/Hinge2 (newton-ode-replacement-plan.md W2.2).
  int addJointBall(int parentIdx, int childIdx,
                   double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                   double childAnchorX, double childAnchorY, double childAnchorZ);
  // Sets the per-step velocity target (rad/s) for a previously-added
  // revolute joint. Only honored by SolverXPBD when the joint was
  // created with non-zero targetKe. Idempotent across ticks; reset
  // by calling with vel=0. Returns 0/-1.
  int setJointTargetVelocity(int jointIdx, double vel);
  // Sets the per-step position target (rad) for a revolute joint with
  // target_ke > 0 (position-spring + damping config). Drives the
  // joint toward this angle each tick. 0/-1.
  int setJointTargetPosition(int jointIdx, double pos);
  // Sets a per-step raw joint TORQUE (Nm) via control.joint_f (applied
  // generalized force). Additive over the PD; for pure torque control build
  // the joint in EFFORT mode (OMNISIM_NEWTON_TORQUE_MODE). Re-send every tick.
  int setJointForce(int jointIdx, double tau);
  // Read the current Newton-tracked angle (rad) for a revolute slot.
  // 0.0 if the model hasn't been finalised. Used by WbBasicJoint's
  // position-control bridge -- the ODE-side hinge->position() doesn't
  // reflect Newton state, so the bridge has to query Newton directly.
  double getJointAngle(int jointIdx) const;
  // Warp the Newton-side body pose to the given world transform and
  // zero its velocity. Called when a Supervisor write hits the
  // Solid's translation/rotation field -- without this, ODE moves
  // but Newton stays put and the two diverge over many resets.
  void resetBodyPose(int bodyIdx, double x, double y, double z,
                     double qx, double qy, double qz, double qw);
  // Reset every revolute joint angle + velocity to the builder's
  // initial values, then eval_fk to propagate the change into
  // body_q. Companion to resetBodyPose: a chassis reset isn't useful
  // if the legs still hold their previous angles via joint_q.
  void resetJointsToDefaults();
  // Diagnostic: dump joint_q from both state_a and model so we can
  // see whether the solver is actually writing joint angles.
  QString diagDumpJointQ() const;
  // Finalises the model: builder -> Model -> Solver + state ping-pong.
  // After this call, addBody / addShapeSphere are no-ops returning -1
  // until the next beginWorld(). 0/-1.
  int finalizeWorld();

  // World-teardown hook: releases the embedded Python World and clears
  // the running/openForBuild flags so the NEXT world load starts from a
  // fresh beginWorld(). Without this, a WORLD RELOAD (Ctrl+Shift+R) left
  // the process-singleton's `running` flag true -> the reloaded world's
  // ensureWorldOpen() no-opped, every addBody/addShape registration
  // silently failed, and the reloaded robots ran on ODE (a Newton-tuned
  // humanoid explodes within ~3 s there) while the ORPHANED old Newton
  // world kept stepping invisibly. Called by ~WbSimulationWorld.
  void teardownWorld();

  // Sets the per-world Newton solver preference (WorldInfo.newtonSolver)
  // BEFORE finalizeWorld(): "mujoco" -> SolverMuJoCo (robust frictional
  // contact, required for pinch grasps); "" / "auto" / "xpbd" -> default GPU
  // XPBD. Must be called during the build phase. 0/-1.
  int setSolverPreference(const QString &name);

  // Folds the WorldInfo.newtonSubsteps choice into the runtime BEFORE the
  // first step (default-flip-plan.md §4.2 N3): N>1 splits each tick into N
  // sub-steps so high closing-speed XPBD contact stays convergent, without a
  // launch env var. The OMNISIM_NEWTON_SUBSTEPS env var still overrides.
  // n<=1 is the unchanged single-step default. 0/-1.
  int setNewtonSubsteps(int n);

  // Runtime-phase API (called every physics tick after the world is
  // finalised). 0/-1.

  // Advances the Newton solver by dt seconds.
  int step(double dt);
  // Reads the world-space pose of bodyIdx into xform[7]:
  //   xform = [x, y, z, qx, qy, qz, qw]
  // matching Newton's native body_q layout. Quaternion is [xyz, w] with
  // w as the scalar component (newton/warp convention). Returns 0/-1.
  int getBodyXform(int bodyIdx, double xform[7]) const;
  // Body spatial velocity from Newton: vel = [vx,vy,vz, wx,wy,wz] in WORLD
  // frame (matches Webots getVelocity()). Returns 0/-1. Without this,
  // getVelocity() reads 0 for Newton-backed solids.
  int getBodyVelocity(int bodyIdx, double vel[6]) const;

  // R3.7b: bulk snapshot of every registered body's translation as
  // 4-float (xyzw, with w = 0 padding) records. Writes up to
  // maxBodies records into `xyzw`; returns the number of bodies
  // actually written (= min(maxBodies, currently registered)) on
  // success, -1 if Newton isn't running or the call into Python
  // failed. The output is laid out tightly: out[4*i+0..2] = body i's
  // (x, y, z), out[4*i+3] = 0.0. Layout matches
  // WbWgpuRenderTarget::clearAndDrawInstanced's `bodyOffsetsXyz0`
  // parameter so the future R3.4-step-4 Camera scene walk can pass
  // the buffer through unchanged.
  //
  // Single Python call + numpy slice — costs one GPU->CPU transfer
  // of body_q regardless of body count (and that transfer is
  // already cached per-step by the body_xform path).
  int snapshotBodyTranslations(int maxBodies, float *xyzw) const;

  // ---- WbPhysicsBackend dispatcher overrides ---------------------------
  //
  // Bridge between the abstract opaque WbBodyHandle interface and Newton's
  // integer-index API. Body handles for Newton-backed solids are packed as
  // (void*)(uintptr_t)(idx + 1) so a NULL handle is clearly invalid (idx=0
  // is a valid Newton body, so the +1 offset keeps 0 reserved). Callsites
  // that go through the registry and resolve to this backend get full
  // polymorphism for the read methods below; write paths stay on the
  // Newton-specific API for now because the abstract write methods don't
  // map 1:1 (Newton drives via joint targets, not direct body force/pose
  // writes outside of resetBodyPose).
  //
  // Newton functionally overrides 7 of WbPhysicsBackend's ~42 dispatcher
  // operation virtuals (the six body/joint reads below + reset); the
  // remaining ~35 -- every write-side virtual -- inherit the -1 default,
  // which callers treat as "this backend doesn't support that op" -- correct
  // for Newton's design (it drives writes via its own integer-index API).
  static WbBodyHandle handleFromIndex(int idx);
  static int indexFromHandle(WbBodyHandle h);

  int getBodyPosition(WbBodyHandle body, double pos[3]) const override;
  int getBodyQuaternion(WbBodyHandle body, double q[4]) const override;
  int getBodyLinearVel(WbBodyHandle body, double v[3]) const override;
  int getBodyAngularVel(WbBodyHandle body, double v[3]) const override;
  // Point velocity computed from body-origin state: v_point = v + omega x r,
  // with r = point - body_origin (both in world frame). Newton doesn't expose
  // a native point-velocity primitive, so this stages through getBodyXform +
  // getBodyVelocity. Two Python round-trips per call; sensors call this once
  // per tick, not per inner step.
  int getBodyPointVel(WbBodyHandle body, const double point[3], double v[3]) const override;
  // P1.6 hinge-angle override: routes the abstract joint-handle read to
  // the existing integer-index getJointAngle(idx). Joint handles share
  // the body-handle packing (idx+1 in a void*) so indexFromHandle does
  // the unpack. Rate intentionally inherits the -1 default -- Newton
  // doesn't expose joint angular rate today, and the post-physics-step
  // rate-based normalisation in WbHingeJoint is ODE-only anyway (Newton
  // reads angle from joint_q directly without renormalising).
  int getJointHingeAngle(WbJointHandle joint, double *angleOut) const override;

  // P7 reset hook: called by WbSimulationWorld::reset after the
  // per-Solid Newton-sync cascade has fired. Re-runs eval_fk so the
  // articulation body_q reflects the freshly-zeroed joint_q values
  // for any Solids whose syncNewtonPoseFromFields didn't get called
  // (e.g. fixed-base robots where the chassis didn't move, so no
  // positionChangedArtificially signal fires). Cheap, idempotent.
  int reset() override;

private:
  // Set true at construction iff:
  //   - OMNISIM_WITH_NEWTON is defined (build flag was ON), AND
  //   - the Newton/Warp runtime was successfully initialised, AND
  //   - the FFI smoke check + helper-module exec succeeded.
  bool mAvailable;

  // Owns the Python-side handles (helper-module namespace dict + the
  // `World` instance for the current world). Allocated when the
  // backend brings Newton up; freed when the backend is destroyed or
  // beginWorld() rolls a fresh one.
  WbNewtonRuntimeState *mRuntime;

  // Sticky copy of the last WorldInfo.newtonSolver request. The GUI
  // rebuilds the world several times per load and each rebuild rolls a
  // fresh Python `world` whose `_solver_pref` resets to None, so a build
  // that finalizes before the flush re-applies the preference would
  // silently fall back to XPBD. finalizeWorld() re-asserts this cached
  // value onto the world it is about to finalize, making the choice
  // build-order-independent.
  QString mSolverPref;

  WbNewtonBackend(const WbNewtonBackend &) = delete;
  WbNewtonBackend &operator=(const WbNewtonBackend &) = delete;
};

#endif
