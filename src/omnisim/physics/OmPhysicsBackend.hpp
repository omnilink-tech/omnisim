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
// OmPhysicsBackend — abstract dispatcher between ODE (default, single-
// threaded CPU) and the GPU-resident Newton solver (additive, opt-in
// per-Solid). Lives at the foundation of the Newton physics-backend
// migration plan; see docs/developer/cuda-newton-physics-plan.md.
//
// Design contract (P0 of the plan; non-negotiable):
//
//   - ODE remains the default backend on every Solid that doesn't
//     explicitly opt out. Every existing world stays byte-equivalent
//     to today's behaviour.
//   - This header is included on every build, including
//     OMNISIM_WITH_NEWTON=OFF. The Newton-specific impl is gated on
//     the build flag at .cpp level; ODE-only builds carry zero
//     Newton code.
//   - At P0, the only concrete implementation is OmOdeBackend (a thin
//     wrapper that forwards every call to the existing ODE code path).
//     Adding the abstraction with one impl is a no-op refactor, which
//     is exactly the point: P0 lands the dispatcher with no behaviour
//     change, P1+ extends with OmNewtonBackend behind the build flag.
//   - Per-Solid dispatch keys off the (forthcoming) `physicsBackend`
//     SFString field on OmSolid. Default value "ode" -> OmOdeBackend.
//     Value "newton" -> OmNewtonBackend (or fall-back to ODE when
//     unavailable). Future backends slot in via OmPhysicsBackendKind.
//
// This header is intentionally pure-virtual: no inline implementations,
// no Qt dependencies, no <ode/ode.h>. It can be included from any TU.
// Concrete implementations live in OmOdeBackend.{hpp,cpp} and
// OmNewtonBackend.{hpp,cpp}.
//

#include <cstddef>

class OmVector3;

// Opaque per-backend body handle. Each concrete backend reinterprets
// the value in its own way:
//   - OmOdeBackend: reinterprets as `dBodyID` (ODE's opaque body
//     pointer).
//   - OmNewtonBackend: doesn't use this accessor path -- it routes
//     reads through its own integer-index API (getBodyXform). The
//     default impl on OmPhysicsBackend returns -1 for handle-keyed
//     ops so Newton inherits "unsupported" until it ever needs to
//     interop with the ODE-style handle.
// Crossing handles between backends is undefined: the caller is
// responsible for routing a handle only to the backend that produced
// it. P1.5 widens the interface one method at a time -- callers that
// move to backend-routed access pass the dBodyID they already hold
// straight through to OmOdeBackend; only when more backends grow
// handle-shaped accessors does the dispatching layer get richer.

// Opaque per-backend joint handle. Same opaque-pointer pattern as
// OmBodyHandle but for joint-side accessors (P1.6 joint-op widening):
//   - OmOdeBackend: reinterprets as `dJointID`.
//   - OmNewtonBackend: packs as `(void*)(uintptr_t)(idx+1)` matching
//     the body-handle scheme so 0 stays reserved as "no joint".
// As with body handles, crossing between backends is undefined --
// callsites that already know which backend owns a joint (every
// OmBasicJoint subclass tracks mNewtonJointIndex vs mJoint) just route
// to the right backend.

enum class OmPhysicsBackendKind {
  // RETIRED SELECTOR, NOT A BACKEND. ODE was deleted in bdc02139; the value
  // survives because world files may still contain `physicsBackend "ode"` and
  // the parser has to recognise it rather than reject the world. It resolves to
  // OmOdeBackend, an inert stub whose isAvailable() is false and whose every
  // verb returns -1 -- so a Solid pinned here is NOT simulated by a legacy
  // engine, it is not simulated at all. OmSolid warns once per such Solid that
  // declares a boundingObject or physics.
  //
  // ⚠ It is also USED DELIBERATELY as a "do not simulate this" switch: the 16
  // G1 ghost worlds pin their hologram robot to "ode" so its ~30 collider-less
  // links stay out of the solver entirely. Removing those pins is not a
  // cleanup -- it puts every link on the kinematic mocap path and costs the
  // flagship measurable wall clock (measured: t=8721 vs ~9700 in a 900 s
  // budget). If you want to retire the idiom, give it its own field first.
  Ode,
  Newton,   // the only real backend
  Auto,     // resolves to Newton when the runtime is available. When it is NOT,
            // there is nothing left to fall back to: resolve() hands out the
            // inert stub and logs one error saying the world will run with no
            // physics. "auto" no longer means "whatever is fastest", it means
            // "Newton, and say so loudly if it is missing".
  Unknown,  // unrecognised value in a world file; resolves to the inert stub
};

// String-to-kind helper, defined in the .cpp.
OmPhysicsBackendKind OmPhysicsBackendKindFromString(const char *name);
const char *OmPhysicsBackendKindToString(OmPhysicsBackendKind kind);

// At P0 this interface is intentionally minimal: just enough for the
// dispatcher table to exist. P1 widens it to wrap getVelocity /
// setVelocity / getPosition / addForce / contact iteration etc., one
// method at a time, each with an existing ODE call as the default
// implementation. That gradual widening is what keeps every commit
// behaviour-preserving.
class OmPhysicsBackend {
public:
  virtual ~OmPhysicsBackend() = default;
  virtual OmPhysicsBackendKind kind() const = 0;
  // Best-effort name for diagnostic logging. Concrete implementations
  // typically return OmPhysicsBackendKindToString(kind()).
  virtual const char *name() const = 0;
  // Returns true if this backend is currently usable (e.g. OmNewtonBackend
  // returns false when the runtime fell back to ODE because
  // OMNISIM_WITH_NEWTON=OFF or no NVIDIA GPU is present).
  virtual bool isAvailable() const = 0;

  // P1.5: handle-keyed body pose accessors. Each returns 0 on success,
  // -1 if the backend doesn't support handle-keyed lookup (Newton
  // inherits -1 today; its Newton-resident solids use getBodyXform on
  // the concrete type).
  //
  // The default impls return -1 so future backends can opt in
  // gradually without touching every TU. OmOdeBackend overrides each
  // to call the matching dBody* function on the handle reinterpreted
  // as dBodyID -- the very ODE calls these methods displaced at the
  // callsites (OmConnector).
  //
  // Quaternion convention follows ODE: q[0]=w, q[1..3]=xyz. dReal is
  // typically double in OmniSim's build; the interface fixes the type
  // at `double` so the abstract surface stays ODE-header-free.
  virtual int getBodyPosition(OmBodyHandle body, double pos[3]) const;
  virtual int setBodyPosition(OmBodyHandle body, const double pos[3]) const;
  virtual int getBodyQuaternion(OmBodyHandle body, double q[4]) const;
  virtual int setBodyQuaternion(OmBodyHandle body, const double q[4]) const;
  // Velocity accessors, same handle + return-code conventions as the
  // position/quaternion ops. Linear and angular velocity are independent
  // ODE state so each gets its own get/set pair.
  virtual int getBodyLinearVel(OmBodyHandle body, double v[3]) const;
  virtual int setBodyLinearVel(OmBodyHandle body, const double v[3]) const;
  virtual int getBodyAngularVel(OmBodyHandle body, double v[3]) const;
  virtual int setBodyAngularVel(OmBodyHandle body, const double v[3]) const;
  // Force/torque application. AtPos variants take a world-space (Pos)
  // or body-relative (RelPos) attachment point; addBodyTorque applies
  // pure angular force. These accumulate into ODE's force/torque
  // buffer for the current step and clear automatically after dWorldStep.
  virtual int addBodyForceAtPos(OmBodyHandle body, const double force[3], const double pos[3]) const;
  virtual int addBodyForceAtRelPos(OmBodyHandle body, const double force[3], const double pos[3]) const;
  virtual int addBodyTorque(OmBodyHandle body, const double torque[3]) const;
  // Replace the body's accumulated force/torque buffer (vs add* which
  // accumulates into it). Most common use: reset to zero on physics
  // reset / world reload.
  virtual int setBodyForce(OmBodyHandle body, const double force[3]) const;
  virtual int setBodyTorque(OmBodyHandle body, const double torque[3]) const;
  // Point velocity: linear velocity of a specific world-space point on
  // the body, combining linear and angular contributions. Used by
  // sensors (Accelerometer, GPS) and aerodynamic actuators (Propeller).
  virtual int getBodyPointVel(OmBodyHandle body, const double point[3], double v[3]) const;
  // Sleep/wake state. enabled=true wakes the body (dBodyEnable);
  // enabled=false puts it to sleep (dBodyDisable). Combined into one
  // method so callsites that branch on a condition don't need two
  // dispatcher lookups -- and so the dispatcher surface stays one
  // method narrower.
  virtual int setBodyEnabled(OmBodyHandle body, bool enabled) const;
  // Returns 1 if the body is awake, 0 if sleeping. Returns -1 if the
  // backend doesn't support the query (Newton's solver tracks awake
  // state differently). Note this is an int tri-state, not a bool, so
  // callers can distinguish "sleeping" from "unsupported".
  virtual int isBodyEnabled(OmBodyHandle body) const;

  // Body-config setup operations -- called once at body creation, not
  // in the per-step hot path. dBodySetMass intentionally stays direct
  // for now: it takes a full dMass struct (mass + center + inertia
  // tensor), and a generic dispatcher version would need a parallel
  // struct mirror. Low leverage given it fires only at world load.
  //
  // getBodyMass returns just the scalar mass (in kg) -- the only field
  // existing callers actually read.
  virtual int getBodyMass(OmBodyHandle body, double *mass) const;
  virtual int setBodyMaxAngularSpeed(OmBodyHandle body, double speed) const;
  virtual int setBodyDamping(OmBodyHandle body, double linear, double angular) const;
  virtual int setBodyDampingDefaults(OmBodyHandle body) const;
  virtual int setBodyAutoDisableFlag(OmBodyHandle body, bool enabled) const;
  virtual int setBodyAutoDisableLinearThreshold(OmBodyHandle body, double threshold) const;
  virtual int setBodyAutoDisableAngularThreshold(OmBodyHandle body, double threshold) const;
  virtual int setBodyAutoDisableTime(OmBodyHandle body, double time) const;

  // P1.6: joint-op widening, read side first.
  //
  // getJointHingeAngle / getJointHingeAngleRate read the current angle
  // (rad) and angular rate (rad/s) of a 1-DoF revolute joint. Same
  // convention as the body ops: 0 on success, -1 if the backend can't
  // answer (default impl). Callers that get -1 fall back to whatever
  // they did before the widening -- i.e. no caller is forced onto the
  // dispatch path until it's wired in.
  //
  // Newton overrides angle (routes to the existing integer-index
  // getJointAngle(idx)) but inherits the rate default -- the rate isn't
  // exposed through the Newton runtime today, and the ODE-side
  // angle-rate correction in OmHingeJoint::postPhysicsStep is moot for
  // Newton-backed joints (the angle comes from Newton's joint_q
  // directly, no normalisation needed).
  virtual int getJointHingeAngle(OmJointHandle joint, double *angleOut) const;
  virtual int getJointHingeAngleRate(OmJointHandle joint, double *rateOut) const;

  // Slider-position read for prismatic joints. Same -1 default as the
  // hinge ops. Newton doesn't support slider/prismatic joints today (the
  // Newton arm wires only revolute hinges) so it inherits -1 and the
  // callsite stays on the ODE path; this matches Featherstone's expected
  // joint surface where prismatic motion is rare in robotics URDFs.
  virtual int getJointSliderPosition(OmJointHandle joint, double *positionOut) const;

  // 3-DoF AMotor angle/angle-rate reads (ball joint, hinge-2 joint).
  // `axis` is 0, 1, or 2 corresponding to ODE's AMotor axis index. Same
  // -1 default; Newton inherits because ball joints aren't a supported
  // articulation primitive in the Newton arm today.
  virtual int getJointAMotorAngle(OmJointHandle joint, int axis, double *angleOut) const;
  virtual int getJointAMotorAngleRate(OmJointHandle joint, int axis, double *rateOut) const;

  // Per-step add-torque / add-force write-side. The mirror of the
  // read-side ops above: user-defined torque/force injection into the
  // joint's accumulator buffer. Cleared automatically after the next
  // dWorldStep. Same -1 default; Newton inherits because the Featherstone
  // solver doesn't accept arbitrary per-joint torque writes outside of
  // its PD-control path (target velocity / target position go through
  // OmBasicJoint's Newton-aware control bridge, not these methods).
  virtual int addJointHingeTorque(OmJointHandle joint, double torque) const;
  virtual int addJointSliderForce(OmJointHandle joint, double force) const;
  virtual int addJointAMotorTorques(OmJointHandle joint, double t0, double t1, double t2) const;
  // Hinge2 add-torque uses ODE's dJointAddHinge2Torques(j, t1, t2) which
  // takes only two torques (the two axes of a universal joint).
  virtual int addJointHinge2Torques(OmJointHandle joint, double t1, double t2) const;

  // setJointParam abstraction for ODE's joint parameter family. The
  // abstract surface stays ODE-header-free by representing the
  // parameter through OmJointParam below instead of ODE's `int
  // dParamFMax` / `int dParamLoStop` constants. The enum starts with
  // the hot-path pair (FMax / Velocity) and grew in slice 6 to cover
  // the full set used by spring/damper + joint-limit + suspension
  // config callsites. Multi-axis joints (AMotor, LMotor) use the
  // `axis` arg below + ODE's dParamGroup = 0x100 offset.
  enum OmJointParam {
    WB_JP_FMAX = 0,             // dParamFMax
    WB_JP_VELOCITY = 1,         // dParamVel
    WB_JP_LO_STOP = 2,          // dParamLoStop
    WB_JP_HI_STOP = 3,          // dParamHiStop
    WB_JP_STOP_CFM = 4,         // dParamStopCFM
    WB_JP_STOP_ERP = 5,         // dParamStopERP
    WB_JP_SUSPENSION_ERP = 6,   // dParamSuspensionERP (Hinge only)
    WB_JP_SUSPENSION_CFM = 7,   // dParamSuspensionCFM (Hinge only)
  };
  virtual int setJointHingeParam(OmJointHandle joint, OmJointParam param, double value) const;
  virtual int setJointSliderParam(OmJointHandle joint, OmJointParam param, double value) const;
  // AMotor / LMotor variants carry an `axis` index because ODE
  // multiplexes their params by axis via the dParamGroup = 0x100 offset
  // (dParamFMax / dParamFMax2 / dParamFMax3, etc.). axis ∈ {0,1,2}.
  virtual int setJointAMotorParam(OmJointHandle joint, int axis, OmJointParam param, double value) const;
  virtual int setJointLMotorParam(OmJointHandle joint, int axis, OmJointParam param, double value) const;
  // Hinge2 carries two axes natively (steering + drive in vehicle
  // hinges), so it takes the same axis-multiplexed shape as A/LMotor.
  virtual int setJointHinge2Param(OmJointHandle joint, int axis, OmJointParam param, double value) const;
  // Ball joint has its own param family (dJointSetBallParam) used only
  // by OmSolid's zero-on-disable path; same axis multiplex.
  virtual int setJointBallParam(OmJointHandle joint, int axis, OmJointParam param, double value) const;

  // Joint lifecycle enable / disable. setJointEnabled(true) wakes the
  // joint (dJointEnable); setJointEnabled(false) puts it to sleep
  // (dJointDisable). isJointEnabled returns the tri-state: 1 enabled,
  // 0 disabled, -1 unsupported.
  //
  // Newton inherits the -1 defaults today: the Newton arm doesn't have
  // a per-joint enable/disable knob (joints exist for the lifetime of
  // the articulation model). When Newton grows mid-run joint deactivation
  // (e.g. for damage / break-away mechanics), the override slot is ready.
  virtual int setJointEnabled(OmJointHandle joint, bool enabled) const;
  virtual int isJointEnabled(OmJointHandle joint) const;

  // P7 reset hook. Called by OmSimulationWorld::reset after the
  // per-Solid reset cascade has fired and before the simulation
  // resumes. Backends that carry world-scope state outside the
  // per-Solid level (e.g. Newton's articulation joint_q buffer that
  // persists across resets if not explicitly cleared) implement this
  // to flush. Default is a no-op return-0 because ODE keeps no
  // world-scope state outside what OmOdeContext clears -- ODE's
  // override below just inherits the default.
  //
  // Distinct from `setJointEnabled` / `setBodyEnabled` (per-entity
  // state) and from the Newton-specific `beginWorld()` /
  // `finalizeWorld()` (world-build lifecycle). reset() is "the
  // physics state at the start of a fresh episode."
  virtual int reset();
};

// Process-wide registry. Returns a long-lived backend instance for a
// given kind. Owns the lifetime of the singletons; safe to call from
// any thread after OmPhysicsBackend::initialise() has been called
// (idempotent first-call wins).
//
// resolve(kind) returns the requested backend if it's available and
// the ODE backend otherwise. ⚠ The historical "transparently get ODE on
// builds without Newton support" guarantee IS GONE: src/ode was deleted, so
// OmOdeBackend is an inert dispatcher stub whose every op returns -1. Landing
// on it means the world has no physics, not legacy physics. Newton
// availability is therefore load-bearing, and newtonEnforced() (below) is the
// only thing that turns its absence into a visible failure.
namespace OmPhysicsBackendRegistry {
  void initialise();
  // Overlap the one-time CPython/warp/newton import with asset retrieval and
  // world parsing. Both calls are idempotent. Set
  // OMNISIM_NEWTON_ASYNC_PRELOAD=0 for the synchronous diagnostic path.
  void startNewtonRuntimePreload();
  void waitForNewtonRuntimePreload();
  OmPhysicsBackend *odeBackend();
  OmPhysicsBackend *newtonBackend();
  OmPhysicsBackend *resolve(OmPhysicsBackendKind kind);

  // Newton-enforcement policy. When this returns true, a silent
  // Newton->ODE downgrade is a HARD ERROR (OmLog::fatal at world load),
  // not a quiet fall-back -- the fix for "Newton silently dropped part
  // of the world onto ODE and a demo broke with no visible cause"
  // (user-facing default chosen 2026-06-29: hard-fail loudly, on by
  // default for Newton-capable builds). Precedence, highest first:
  //   1. OMNISIM_REQUIRE_NEWTON -> true (explicit assertion).
  //   2. Default                -> true iff the Newton runtime actually
  //      initialised (newtonBackend()->isAvailable()).
  // OMNISIM_FORCE_ODE / OMNISIM_LEGACY / OMNISIM_ALLOW_ODE_FALLBACK used to
  // sit above both and return false. They are RETIRED: each named a legitimate
  // route onto ODE, and there is none any more, so relaxing enforcement on
  // their behalf would only buy a physics-free world. They now warn and are
  // ignored.
  // Call only from world-load enforcement sites (never per-step): the
  // availability probe lazily constructs the Newton backend, which those
  // sites have already done.
  bool newtonEnforced();
}

#endif
