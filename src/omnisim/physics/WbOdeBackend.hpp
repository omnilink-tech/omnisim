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

#ifndef WB_ODE_BACKEND_HPP
#define WB_ODE_BACKEND_HPP

//
// WbOdeBackend — concrete WbPhysicsBackend that delegates every
// operation to the existing ODE code path. This is the canonical
// backend; every WbSolid that doesn't explicitly opt into another
// backend uses this one.
//
// Note: at P1 of cuda-newton-physics-plan.md, this class is just a
// "marker" implementation that reports kind/name/available. Real ODE
// dispatching still happens through the existing call-site code; the
// backend is only consulted at WbSolid::physicsBackend() lookup time
// to decide whether to short-circuit to a non-ODE path.
//
// As later phases widen WbPhysicsBackend with virtual methods
// (getVelocity, setVelocity, addForce, ...), each new method gets a
// WbOdeBackend implementation that calls the existing ODE code
// verbatim — this is by design, so the no-op refactor stays no-op.
//

#include "WbPhysicsBackend.hpp"

class WbOdeBackend : public WbPhysicsBackend {
public:
  WbOdeBackend() = default;
  ~WbOdeBackend() override = default;
  WbPhysicsBackendKind kind() const override { return WbPhysicsBackendKind::Ode; }
  const char *name() const override { return "ode"; }
  bool isAvailable() const override { return true; }

  // P1.5 widened ops: the body handle is reinterpreted as `dBodyID`.
  // Implementations live in WbOdeBackend.cpp so callers of this header
  // don't have to pull in <ode/ode.h>.
  int getBodyPosition(WbBodyHandle body, double pos[3]) const override;
  int setBodyPosition(WbBodyHandle body, const double pos[3]) const override;
  int getBodyQuaternion(WbBodyHandle body, double q[4]) const override;
  int setBodyQuaternion(WbBodyHandle body, const double q[4]) const override;
  int getBodyLinearVel(WbBodyHandle body, double v[3]) const override;
  int setBodyLinearVel(WbBodyHandle body, const double v[3]) const override;
  int getBodyAngularVel(WbBodyHandle body, double v[3]) const override;
  int setBodyAngularVel(WbBodyHandle body, const double v[3]) const override;
  int addBodyForceAtPos(WbBodyHandle body, const double force[3], const double pos[3]) const override;
  int addBodyForceAtRelPos(WbBodyHandle body, const double force[3], const double pos[3]) const override;
  int addBodyTorque(WbBodyHandle body, const double torque[3]) const override;
  int setBodyForce(WbBodyHandle body, const double force[3]) const override;
  int setBodyTorque(WbBodyHandle body, const double torque[3]) const override;
  int getBodyPointVel(WbBodyHandle body, const double point[3], double v[3]) const override;
  int setBodyEnabled(WbBodyHandle body, bool enabled) const override;
  int isBodyEnabled(WbBodyHandle body) const override;
  int getBodyMass(WbBodyHandle body, double *mass) const override;
  int setBodyMaxAngularSpeed(WbBodyHandle body, double speed) const override;
  int setBodyDamping(WbBodyHandle body, double linear, double angular) const override;
  int setBodyDampingDefaults(WbBodyHandle body) const override;
  int setBodyAutoDisableFlag(WbBodyHandle body, bool enabled) const override;
  int setBodyAutoDisableLinearThreshold(WbBodyHandle body, double threshold) const override;
  int setBodyAutoDisableAngularThreshold(WbBodyHandle body, double threshold) const override;
  int setBodyAutoDisableTime(WbBodyHandle body, double time) const override;

  // P1.6 joint-op widening: hinge-angle reads. The WbJointHandle
  // reinterprets as dJointID; values come straight from the ODE solver
  // state via dJointGetHingeAngle / dJointGetHingeAngleRate.
  int getJointHingeAngle(WbJointHandle joint, double *angleOut) const override;
  int getJointHingeAngleRate(WbJointHandle joint, double *rateOut) const override;
  int getJointSliderPosition(WbJointHandle joint, double *positionOut) const override;
  int getJointAMotorAngle(WbJointHandle joint, int axis, double *angleOut) const override;
  int getJointAMotorAngleRate(WbJointHandle joint, int axis, double *rateOut) const override;

  int addJointHingeTorque(WbJointHandle joint, double torque) const override;
  int addJointSliderForce(WbJointHandle joint, double force) const override;
  int addJointAMotorTorques(WbJointHandle joint, double t0, double t1, double t2) const override;
  int addJointHinge2Torques(WbJointHandle joint, double t1, double t2) const override;
  int setJointHingeParam(WbJointHandle joint, WbJointParam param, double value) const override;
  int setJointSliderParam(WbJointHandle joint, WbJointParam param, double value) const override;
  int setJointAMotorParam(WbJointHandle joint, int axis, WbJointParam param, double value) const override;
  int setJointLMotorParam(WbJointHandle joint, int axis, WbJointParam param, double value) const override;
  int setJointHinge2Param(WbJointHandle joint, int axis, WbJointParam param, double value) const override;
  int setJointBallParam(WbJointHandle joint, int axis, WbJointParam param, double value) const override;
  int setJointEnabled(WbJointHandle joint, bool enabled) const override;
  int isJointEnabled(WbJointHandle joint) const override;
};

#endif
