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

#ifndef OM_ODE_BACKEND_HPP
#define OM_ODE_BACKEND_HPP

//
// OmOdeBackend -- a TOMBSTONE for a deleted backend. It dispatches nothing.
//
// ODE was removed in bdc02139 (106,283 lines). This class survives for exactly
// one reason: world files in the wild still say `physicsBackend "ode"`, and the
// engine must be able to name that choice and refuse it honestly instead of
// either rejecting the world or silently substituting a backend nobody asked
// for. So every verb returns -1 and isAvailable() is false.
//
// The consequence is worth stating plainly, because the class name suggests the
// opposite: a Solid that resolves here has NO PHYSICS. Not legacy physics, not
// slower physics -- it is absent from the solver. OmSolid warns once per such
// Solid that declares a boundingObject or physics, and resolve() logs one error
// when it has no real backend to hand out at all.
//
// Do not add an implementation to any method here. If a caller needs an
// operation to work, route it through the resolved backend; if it needs to know
// whether it CAN work, ask isAvailable() rather than assuming a non-null
// pointer means a live engine. (That assumption is what made ~113 call sites
// silently dead physics operations after the deletion; they were removed rather
// than fixed, because none of them had a live counterpart.)
//

#include "OmPhysicsBackend.hpp"

class OmOdeBackend : public OmPhysicsBackend {
public:
  OmOdeBackend() = default;
  ~OmOdeBackend() override = default;
  OmPhysicsBackendKind kind() const override { return OmPhysicsBackendKind::Ode; }
  const char *name() const override { return "ode"; }
  // ODE has been deleted; this class is an inert stub and the dispatcher must
  // never route a Solid here. Mirrors OmNewtonBackend's mAvailable=false mode.
  bool isAvailable() const override { return false; }

  // P1.5 widened ops: the body handle is reinterpreted as `dBodyID`.
  // Implementations live in OmOdeBackend.cpp so callers of this header
  // don't have to pull in <ode/ode.h>.
  int getBodyPosition(OmBodyHandle body, double pos[3]) const override;
  int setBodyPosition(OmBodyHandle body, const double pos[3]) const override;
  int getBodyQuaternion(OmBodyHandle body, double q[4]) const override;
  int setBodyQuaternion(OmBodyHandle body, const double q[4]) const override;
  int getBodyLinearVel(OmBodyHandle body, double v[3]) const override;
  int setBodyLinearVel(OmBodyHandle body, const double v[3]) const override;
  int getBodyAngularVel(OmBodyHandle body, double v[3]) const override;
  int setBodyAngularVel(OmBodyHandle body, const double v[3]) const override;
  int addBodyForceAtPos(OmBodyHandle body, const double force[3], const double pos[3]) const override;
  int addBodyForceAtRelPos(OmBodyHandle body, const double force[3], const double pos[3]) const override;
  int addBodyTorque(OmBodyHandle body, const double torque[3]) const override;
  int setBodyForce(OmBodyHandle body, const double force[3]) const override;
  int setBodyTorque(OmBodyHandle body, const double torque[3]) const override;
  int getBodyPointVel(OmBodyHandle body, const double point[3], double v[3]) const override;
  int setBodyEnabled(OmBodyHandle body, bool enabled) const override;
  int isBodyEnabled(OmBodyHandle body) const override;
  int getBodyMass(OmBodyHandle body, double *mass) const override;
  int setBodyMaxAngularSpeed(OmBodyHandle body, double speed) const override;
  int setBodyDamping(OmBodyHandle body, double linear, double angular) const override;
  int setBodyDampingDefaults(OmBodyHandle body) const override;
  int setBodyAutoDisableFlag(OmBodyHandle body, bool enabled) const override;
  int setBodyAutoDisableLinearThreshold(OmBodyHandle body, double threshold) const override;
  int setBodyAutoDisableAngularThreshold(OmBodyHandle body, double threshold) const override;
  int setBodyAutoDisableTime(OmBodyHandle body, double time) const override;

  // P1.6 joint-op widening: hinge-angle reads. The OmJointHandle
  // reinterprets as dJointID; values come straight from the ODE solver
  // state via dJointGetHingeAngle / dJointGetHingeAngleRate.
  int getJointHingeAngle(OmJointHandle joint, double *angleOut) const override;
  int getJointHingeAngleRate(OmJointHandle joint, double *rateOut) const override;
  int getJointSliderPosition(OmJointHandle joint, double *positionOut) const override;
  int getJointAMotorAngle(OmJointHandle joint, int axis, double *angleOut) const override;
  int getJointAMotorAngleRate(OmJointHandle joint, int axis, double *rateOut) const override;

  int addJointHingeTorque(OmJointHandle joint, double torque) const override;
  int addJointSliderForce(OmJointHandle joint, double force) const override;
  int addJointAMotorTorques(OmJointHandle joint, double t0, double t1, double t2) const override;
  int addJointHinge2Torques(OmJointHandle joint, double t1, double t2) const override;
  int setJointHingeParam(OmJointHandle joint, OmJointParam param, double value) const override;
  int setJointSliderParam(OmJointHandle joint, OmJointParam param, double value) const override;
  int setJointAMotorParam(OmJointHandle joint, int axis, OmJointParam param, double value) const override;
  int setJointLMotorParam(OmJointHandle joint, int axis, OmJointParam param, double value) const override;
  int setJointHinge2Param(OmJointHandle joint, int axis, OmJointParam param, double value) const override;
  int setJointBallParam(OmJointHandle joint, int axis, OmJointParam param, double value) const override;
  int setJointEnabled(OmJointHandle joint, bool enabled) const override;
  int isJointEnabled(OmJointHandle joint) const override;
};

#endif
