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

#ifndef OM_PHYSICS_BACKEND_HPP
#define OM_PHYSICS_BACKEND_HPP

#include "OmPhysicsHandles.hpp"

//
// OmPhysicsBackend -- the abstract physics interface the engine's nodes talk
// to, and the registry that hands out the one implementation: OmNewtonBackend
// (GPU/CPU Newton + MuJoCo). It began life (cuda-newton-physics-plan.md P0) as
// a dispatcher between ODE and Newton; ODE was deleted in bdc02139 and its
// inert stub layer was removed on 2026-09-02, so there is exactly one backend
// and this header no longer dispatches anything. The abstraction is kept
// because every node reaches physics through it (handle-keyed body/joint ops
// with a uniform "0 = ok, -1 = unsupported" contract), which is what keeps
// Newton-specific headers out of the node TUs.
//
// This header is intentionally pure-virtual: no inline implementations, no Qt
// dependencies. It can be included from any TU. The concrete implementation
// lives in OmNewtonBackend.{hpp,cpp}.
//

#include <cstddef>

class OmVector3;

// Opaque per-backend body / joint handles (OmPhysicsHandles.hpp).
// OmNewtonBackend packs a body index as `(void*)(uintptr_t)(idx+1)` (see
// OmNewtonBackend::handleFromIndex) so 0 stays reserved as "no body"; joints
// use the same scheme. Most Newton-resident code routes through the integer-
// index API directly and only sensors / supervisor helpers use the
// handle-keyed accessors below.

enum class OmPhysicsBackendKind {
  Newton,   // the only backend
  Auto,     // the schema default ("auto" / ""): resolves to Newton
  Unknown,  // any other value in a world file. "ode" lands here since ODE's
            // stub layer was removed (2026-09-02): it is a RETIRED SELECTOR,
            // and the Solid runs on Newton like every other. OmSolid warns once
            // per world when it sees one. A Solid can no longer be registered
            // with NO solver by naming a backend that does not exist.
};

// String-to-kind helper, defined in the .cpp.
OmPhysicsBackendKind OmPhysicsBackendKindFromString(const char *name);
const char *OmPhysicsBackendKindToString(OmPhysicsBackendKind kind);

class OmPhysicsBackend {
public:
  virtual ~OmPhysicsBackend() = default;
  virtual OmPhysicsBackendKind kind() const = 0;
  // Best-effort name for diagnostic logging. Concrete implementations
  // typically return OmPhysicsBackendKindToString(kind()).
  virtual const char *name() const = 0;
  // Returns true if this backend is currently usable (OmNewtonBackend returns
  // false when the Newton runtime failed to import or was never bundled).
  virtual bool isAvailable() const = 0;

  // Handle-keyed body pose accessors. Each returns 0 on success, -1 if the
  // backend cannot answer for this handle (no body registered, runtime down).
  //
  // The default impls return -1 so a backend opts into each op explicitly.
  // Quaternion convention: q[0]=w, q[1..3]=xyz.
  virtual int getBodyPosition(OmBodyHandle body, double pos[3]) const;
  virtual int setBodyPosition(OmBodyHandle body, const double pos[3]) const;
  virtual int getBodyQuaternion(OmBodyHandle body, double q[4]) const;
  virtual int setBodyQuaternion(OmBodyHandle body, const double q[4]) const;
  // Velocity accessors, same handle + return-code conventions. Linear and
  // angular velocity are independent state so each gets its own get/set pair.
  virtual int getBodyLinearVel(OmBodyHandle body, double v[3]) const;
  virtual int setBodyLinearVel(OmBodyHandle body, const double v[3]) const;
  virtual int getBodyAngularVel(OmBodyHandle body, double v[3]) const;
  virtual int setBodyAngularVel(OmBodyHandle body, const double v[3]) const;
  // Force/torque application. AtPos variants take a world-space (Pos) or
  // body-relative (RelPos) attachment point; addBodyTorque applies pure
  // angular force. These accumulate for the current step and clear after it.
  virtual int addBodyForceAtPos(OmBodyHandle body, const double force[3], const double pos[3]) const;
  virtual int addBodyForceAtRelPos(OmBodyHandle body, const double force[3], const double pos[3]) const;
  virtual int addBodyTorque(OmBodyHandle body, const double torque[3]) const;
  // Replace the body's accumulated force/torque buffer (vs add* which
  // accumulates into it). Most common use: reset to zero on physics reset /
  // world reload.
  virtual int setBodyForce(OmBodyHandle body, const double force[3]) const;
  virtual int setBodyTorque(OmBodyHandle body, const double torque[3]) const;
  // Point velocity: linear velocity of a specific world-space point on the
  // body, combining linear and angular contributions. Used by sensors
  // (Accelerometer, GPS) and aerodynamic actuators (Propeller).
  virtual int getBodyPointVel(OmBodyHandle body, const double point[3], double v[3]) const;
  // Sleep/wake state. enabled=true wakes the body; enabled=false puts it to
  // sleep. One method so callsites that branch on a condition need one call.
  virtual int setBodyEnabled(OmBodyHandle body, bool enabled) const;
  // Returns 1 if the body is awake, 0 if sleeping, -1 if the backend does not
  // support the query (Newton has no body sleep). An int tri-state, not a
  // bool, so callers can distinguish "sleeping" from "unsupported".
  virtual int isBodyEnabled(OmBodyHandle body) const;

  // Body-config setup operations -- called once at body creation, not in the
  // per-step hot path. getBodyMass returns just the scalar mass (kg).
  virtual int getBodyMass(OmBodyHandle body, double *mass) const;
  virtual int setBodyMaxAngularSpeed(OmBodyHandle body, double speed) const;
  virtual int setBodyDamping(OmBodyHandle body, double linear, double angular) const;
  virtual int setBodyDampingDefaults(OmBodyHandle body) const;
  virtual int setBodyAutoDisableFlag(OmBodyHandle body, bool enabled) const;
  virtual int setBodyAutoDisableLinearThreshold(OmBodyHandle body, double threshold) const;
  virtual int setBodyAutoDisableAngularThreshold(OmBodyHandle body, double threshold) const;
  virtual int setBodyAutoDisableTime(OmBodyHandle body, double time) const;

  // Joint reads. getJointHingeAngle / getJointHingeAngleRate read the current
  // angle (rad) and angular rate (rad/s) of a 1-DoF revolute joint. Same
  // convention as the body ops: 0 on success, -1 if the backend cannot answer.
  // Newton answers the angle (joint_q) and inherits the -1 default for the
  // rate; OmHingeJoint derives the rate itself.
  virtual int getJointHingeAngle(OmJointHandle joint, double *angleOut) const;
  virtual int getJointHingeAngleRate(OmJointHandle joint, double *rateOut) const;

  // Slider-position read for prismatic joints. Same -1 default.
  virtual int getJointSliderPosition(OmJointHandle joint, double *positionOut) const;

  // 3-DoF angle/angle-rate reads (ball joint, hinge-2 joint). `axis` is 0, 1
  // or 2. Same -1 default; the multi-DoF joints read their angles back through
  // OmBasicJoint's Newton-specific path (registerNewtonMultiDof).
  virtual int getJointAMotorAngle(OmJointHandle joint, int axis, double *angleOut) const;
  virtual int getJointAMotorAngleRate(OmJointHandle joint, int axis, double *rateOut) const;

  // Per-step add-torque / add-force write side: user-defined torque/force
  // injection into a joint's accumulator, cleared after the next step. Same
  // -1 default; Newton's motors are driven through OmBasicJoint's control
  // bridge (target position / velocity / torque mode), not these methods.
  virtual int addJointHingeTorque(OmJointHandle joint, double torque) const;
  virtual int addJointSliderForce(OmJointHandle joint, double force) const;
  virtual int addJointAMotorTorques(OmJointHandle joint, double t0, double t1, double t2) const;
  // Hinge2 takes only two torques (the two axes of a universal joint).
  virtual int addJointHinge2Torques(OmJointHandle joint, double t1, double t2) const;

  // Joint parameter family. The abstract surface names each parameter through
  // OmJointParam rather than any solver's own constants. Multi-axis joints
  // (AMotor, LMotor, Hinge2) take an `axis` argument.
  enum OmJointParam {
    WB_JP_FMAX = 0,
    WB_JP_VELOCITY = 1,
    WB_JP_LO_STOP = 2,
    WB_JP_HI_STOP = 3,
    WB_JP_STOP_CFM = 4,
    WB_JP_STOP_ERP = 5,
    WB_JP_SUSPENSION_ERP = 6,  // Hinge only
    WB_JP_SUSPENSION_CFM = 7,  // Hinge only
  };
  virtual int setJointHingeParam(OmJointHandle joint, OmJointParam param, double value) const;
  virtual int setJointSliderParam(OmJointHandle joint, OmJointParam param, double value) const;
  // AMotor / LMotor variants carry an `axis` index. axis in {0,1,2}.
  virtual int setJointAMotorParam(OmJointHandle joint, int axis, OmJointParam param, double value) const;
  virtual int setJointLMotorParam(OmJointHandle joint, int axis, OmJointParam param, double value) const;
  // Hinge2 carries two axes natively (steering + drive in vehicle hinges), so
  // it takes the same axis-multiplexed shape as A/LMotor.
  virtual int setJointHinge2Param(OmJointHandle joint, int axis, OmJointParam param, double value) const;
  // Ball joint has its own param family, used only by OmSolid's
  // zero-on-disable path; same axis multiplex.
  virtual int setJointBallParam(OmJointHandle joint, int axis, OmJointParam param, double value) const;

  // Joint lifecycle enable / disable. isJointEnabled returns the tri-state:
  // 1 enabled, 0 disabled, -1 unsupported. Newton inherits the -1 defaults:
  // joints exist for the lifetime of the articulation model. When Newton grows
  // mid-run joint deactivation (damage / break-away), the override slot is
  // ready.
  virtual int setJointEnabled(OmJointHandle joint, bool enabled) const;
  virtual int isJointEnabled(OmJointHandle joint) const;

  // Reset hook. Called by OmSimulationWorld::reset after the per-Solid reset
  // cascade has fired and before the simulation resumes. Backends that carry
  // world-scope state outside the per-Solid level (Newton's articulation
  // joint_q buffer persists across resets if not explicitly cleared) implement
  // this to flush. Default is a no-op return 0.
  //
  // Distinct from `setJointEnabled` / `setBodyEnabled` (per-entity state) and
  // from the Newton-specific `beginWorld()` / `finalizeWorld()` (world-build
  // lifecycle). reset() is "the physics state at the start of a fresh episode."
  virtual int reset();
};

// Process-wide registry. Returns the long-lived backend instance. Owns the
// lifetime of the singleton; safe to call from any thread.
//
// resolve(kind) returns the Newton backend for EVERY kind -- there is nothing
// else to return. When the Newton runtime is unavailable it still returns the
// (non-null) Newton object, whose isAvailable() is false and whose every op
// returns -1 because no body was ever registered; it logs one ERROR per process
// saying the world will load and stand still. Newton availability is therefore
// load-bearing, and newtonEnforced() (below) is what turns its absence into a
// visible failure at the registration sites.
namespace OmPhysicsBackendRegistry {
  // Overlap the one-time CPython/warp/newton import with asset retrieval and
  // world parsing. Both calls are idempotent. Set
  // OMNISIM_NEWTON_ASYNC_PRELOAD=0 for the synchronous diagnostic path.
  void startNewtonRuntimePreload();
  void waitForNewtonRuntimePreload();
  OmPhysicsBackend *newtonBackend();
  OmPhysicsBackend *resolve(OmPhysicsBackendKind kind);

  // Newton-enforcement policy. When this returns true, a Solid or joint that
  // resolved to Newton but never registered a Newton body is a HARD ERROR
  // (OmLog::fatal at world load), not a quiet omission -- the fix for "part of
  // the world silently dropped out of the solver and a demo broke with no
  // visible cause" (user-facing default chosen 2026-06-29: hard-fail loudly).
  // Precedence, highest first:
  //   1. OMNISIM_REQUIRE_NEWTON -> true (explicit assertion; PRESENCE-gated).
  //   2. Default                -> true iff the Newton runtime actually
  //      initialised (newtonBackend()->isAvailable()).
  // Call only from world-load enforcement sites (never per-step): the
  // availability probe lazily constructs the Newton backend, which those
  // sites have already done.
  bool newtonEnforced();
}

#endif
