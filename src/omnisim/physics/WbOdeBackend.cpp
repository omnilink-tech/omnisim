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

#include "WbOdeBackend.hpp"

// P1.5 of engine-migration-plan.md: gradual widening of the
// WbPhysicsBackend interface, starting with one read accessor
// (getBodyPosition) routed at one callsite (WbConnector::snapOrigins).
//
// Why this file exists now (and not at P1 when WbOdeBackend was born):
// pre-P1.5 WbOdeBackend was a header-only marker class -- kind()/name()/
// isAvailable() are all trivial inline returns. The first widened op
// that wraps an actual ODE call (dBodyGetPosition) needs <ode/ode.h>,
// which we deliberately keep out of WbOdeBackend.hpp so the rest of the
// tree doesn't accidentally start including ODE headers transitively
// through the dispatcher abstraction (the whole point of the
// abstraction is to *contain* the ODE dependency, not propagate it).
// Hence a dedicated .cpp.

#include <ode/ode.h>

int WbOdeBackend::getBodyPosition(WbBodyHandle body, double pos[3]) const {
  // The WbBodyHandle contract says: ODE-backend handles reinterpret
  // as dBodyID. dBodyGetPosition returns a pointer into ODE's internal
  // storage; we copy out to decouple the caller from ODE's lifetime
  // (otherwise a subsequent dBodySetPosition on the same body would
  // invalidate the read, which is exactly what would happen at the
  // first call site that adopts this -- WbConnector::snapOrigins
  // reads then writes the same body in two adjacent lines).
  const dReal *const p = dBodyGetPosition(static_cast<dBodyID>(body));
  pos[0] = p[0];
  pos[1] = p[1];
  pos[2] = p[2];
  return 0;
}

int WbOdeBackend::setBodyPosition(WbBodyHandle body, const double pos[3]) const {
  dBodySetPosition(static_cast<dBodyID>(body), pos[0], pos[1], pos[2]);
  return 0;
}

int WbOdeBackend::getBodyQuaternion(WbBodyHandle body, double q[4]) const {
  // Same copy-out rationale as getBodyPosition: dBodyGetQuaternion
  // returns a pointer into ODE's storage, which a subsequent
  // dBodySetQuaternion would invalidate. Quaternion layout follows ODE:
  // q[0]=w, q[1..3]=xyz.
  const dReal *const src = dBodyGetQuaternion(static_cast<dBodyID>(body));
  q[0] = src[0];
  q[1] = src[1];
  q[2] = src[2];
  q[3] = src[3];
  return 0;
}

int WbOdeBackend::setBodyQuaternion(WbBodyHandle body, const double q[4]) const {
  // dBodySetQuaternion takes `dQuaternion` (= dReal[4]). When dReal is
  // double (the OmniSim default) this is byte-equivalent; when float we
  // need an explicit narrow. Stage through a local dQuaternion so the
  // code stays correct either way.
  dQuaternion tmp = {static_cast<dReal>(q[0]), static_cast<dReal>(q[1]),
                     static_cast<dReal>(q[2]), static_cast<dReal>(q[3])};
  dBodySetQuaternion(static_cast<dBodyID>(body), tmp);
  return 0;
}

int WbOdeBackend::getBodyLinearVel(WbBodyHandle body, double v[3]) const {
  // Same copy-out rationale as the position/quaternion getters:
  // dBodyGetLinearVel returns a pointer into ODE's storage, which a
  // subsequent dBodySetLinearVel would invalidate.
  const dReal *const src = dBodyGetLinearVel(static_cast<dBodyID>(body));
  v[0] = src[0];
  v[1] = src[1];
  v[2] = src[2];
  return 0;
}

int WbOdeBackend::setBodyLinearVel(WbBodyHandle body, const double v[3]) const {
  dBodySetLinearVel(static_cast<dBodyID>(body), v[0], v[1], v[2]);
  return 0;
}

int WbOdeBackend::getBodyAngularVel(WbBodyHandle body, double v[3]) const {
  const dReal *const src = dBodyGetAngularVel(static_cast<dBodyID>(body));
  v[0] = src[0];
  v[1] = src[1];
  v[2] = src[2];
  return 0;
}

int WbOdeBackend::setBodyAngularVel(WbBodyHandle body, const double v[3]) const {
  dBodySetAngularVel(static_cast<dBodyID>(body), v[0], v[1], v[2]);
  return 0;
}

int WbOdeBackend::addBodyForceAtPos(WbBodyHandle body, const double force[3], const double pos[3]) const {
  dBodyAddForceAtPos(static_cast<dBodyID>(body), force[0], force[1], force[2], pos[0], pos[1], pos[2]);
  return 0;
}

int WbOdeBackend::addBodyForceAtRelPos(WbBodyHandle body, const double force[3], const double pos[3]) const {
  dBodyAddForceAtRelPos(static_cast<dBodyID>(body), force[0], force[1], force[2], pos[0], pos[1], pos[2]);
  return 0;
}

int WbOdeBackend::addBodyTorque(WbBodyHandle body, const double torque[3]) const {
  dBodyAddTorque(static_cast<dBodyID>(body), torque[0], torque[1], torque[2]);
  return 0;
}

int WbOdeBackend::setBodyForce(WbBodyHandle body, const double force[3]) const {
  dBodySetForce(static_cast<dBodyID>(body), force[0], force[1], force[2]);
  return 0;
}

int WbOdeBackend::setBodyTorque(WbBodyHandle body, const double torque[3]) const {
  dBodySetTorque(static_cast<dBodyID>(body), torque[0], torque[1], torque[2]);
  return 0;
}

int WbOdeBackend::getBodyPointVel(WbBodyHandle body, const double point[3], double v[3]) const {
  // dBodyGetPointVel writes into a dVector3 (= dReal[4], the 4th
  // element is padding). Use a local dVector3 to match ODE's API and
  // copy out the first three values to the dispatcher's double[3].
  dVector3 tmp;
  dBodyGetPointVel(static_cast<dBodyID>(body), point[0], point[1], point[2], tmp);
  v[0] = tmp[0];
  v[1] = tmp[1];
  v[2] = tmp[2];
  return 0;
}

int WbOdeBackend::setBodyEnabled(WbBodyHandle body, bool enabled) const {
  if (enabled)
    dBodyEnable(static_cast<dBodyID>(body));
  else
    dBodyDisable(static_cast<dBodyID>(body));
  return 0;
}

int WbOdeBackend::isBodyEnabled(WbBodyHandle body) const {
  return dBodyIsEnabled(static_cast<dBodyID>(body)) != 0 ? 1 : 0;
}

int WbOdeBackend::getBodyMass(WbBodyHandle body, double *mass) const {
  // Only the scalar mass is exposed through the dispatcher -- the full
  // dMass (mass + center + inertia tensor) is set once at world load
  // and the four existing callers read just the scalar field.
  dMass m;
  dBodyGetMass(static_cast<dBodyID>(body), &m);
  *mass = m.mass;
  return 0;
}

int WbOdeBackend::setBodyMaxAngularSpeed(WbBodyHandle body, double speed) const {
  dBodySetMaxAngularSpeed(static_cast<dBodyID>(body), speed);
  return 0;
}

int WbOdeBackend::setBodyDamping(WbBodyHandle body, double linear, double angular) const {
  dBodySetDamping(static_cast<dBodyID>(body), linear, angular);
  return 0;
}

int WbOdeBackend::setBodyDampingDefaults(WbBodyHandle body) const {
  dBodySetDampingDefaults(static_cast<dBodyID>(body));
  return 0;
}

int WbOdeBackend::setBodyAutoDisableFlag(WbBodyHandle body, bool enabled) const {
  dBodySetAutoDisableFlag(static_cast<dBodyID>(body), enabled ? 1 : 0);
  return 0;
}

int WbOdeBackend::setBodyAutoDisableLinearThreshold(WbBodyHandle body, double threshold) const {
  dBodySetAutoDisableLinearThreshold(static_cast<dBodyID>(body), threshold);
  return 0;
}

int WbOdeBackend::setBodyAutoDisableAngularThreshold(WbBodyHandle body, double threshold) const {
  dBodySetAutoDisableAngularThreshold(static_cast<dBodyID>(body), threshold);
  return 0;
}

int WbOdeBackend::setBodyAutoDisableTime(WbBodyHandle body, double time) const {
  dBodySetAutoDisableTime(static_cast<dBodyID>(body), time);
  return 0;
}

int WbOdeBackend::getJointHingeAngle(WbJointHandle joint, double *angleOut) const {
  *angleOut = dJointGetHingeAngle(static_cast<dJointID>(joint));
  return 0;
}

int WbOdeBackend::getJointHingeAngleRate(WbJointHandle joint, double *rateOut) const {
  *rateOut = dJointGetHingeAngleRate(static_cast<dJointID>(joint));
  return 0;
}

int WbOdeBackend::getJointSliderPosition(WbJointHandle joint, double *positionOut) const {
  *positionOut = dJointGetSliderPosition(static_cast<dJointID>(joint));
  return 0;
}

int WbOdeBackend::getJointAMotorAngle(WbJointHandle joint, int axis, double *angleOut) const {
  *angleOut = dJointGetAMotorAngle(static_cast<dJointID>(joint), axis);
  return 0;
}

int WbOdeBackend::getJointAMotorAngleRate(WbJointHandle joint, int axis, double *rateOut) const {
  *rateOut = dJointGetAMotorAngleRate(static_cast<dJointID>(joint), axis);
  return 0;
}

int WbOdeBackend::addJointHingeTorque(WbJointHandle joint, double torque) const {
  dJointAddHingeTorque(static_cast<dJointID>(joint), torque);
  return 0;
}

int WbOdeBackend::addJointSliderForce(WbJointHandle joint, double force) const {
  dJointAddSliderForce(static_cast<dJointID>(joint), force);
  return 0;
}

int WbOdeBackend::addJointAMotorTorques(WbJointHandle joint, double t0, double t1, double t2) const {
  dJointAddAMotorTorques(static_cast<dJointID>(joint), t0, t1, t2);
  return 0;
}

int WbOdeBackend::addJointHinge2Torques(WbJointHandle joint, double t1, double t2) const {
  dJointAddHinge2Torques(static_cast<dJointID>(joint), t1, t2);
  return 0;
}

// Map WbJointParam to ODE's dParamXxx constants. Single source of
// truth for the parameter set across slice 4 (FMax / Velocity) and
// slice 6 (the spring/damper + joint-limit + suspension family).
static int wbJointParamToOde(WbPhysicsBackend::WbJointParam p) {
  switch (p) {
    case WbPhysicsBackend::WB_JP_FMAX:           return dParamFMax;
    case WbPhysicsBackend::WB_JP_VELOCITY:       return dParamVel;
    case WbPhysicsBackend::WB_JP_LO_STOP:        return dParamLoStop;
    case WbPhysicsBackend::WB_JP_HI_STOP:        return dParamHiStop;
    case WbPhysicsBackend::WB_JP_STOP_CFM:       return dParamStopCFM;
    case WbPhysicsBackend::WB_JP_STOP_ERP:       return dParamStopERP;
    case WbPhysicsBackend::WB_JP_SUSPENSION_ERP: return dParamSuspensionERP;
    case WbPhysicsBackend::WB_JP_SUSPENSION_CFM: return dParamSuspensionCFM;
  }
  return dParamFMax;  // unreachable for current enum domain; keeps the compiler happy
}

// AMotor axis multiplexer: ODE reserves dParamGroup1 = 0x100 / Group2 = 0x200
// for axis-2/3 versions of every param. dParamFMax + axis*256 = dParamFMax2
// (axis=1) or dParamFMax3 (axis=2). Same offset works for dParamVel and all
// the LoStop/HiStop/StopCFM/StopERP family once they grow into the enum.
static int wbJointParamToOdeAxis(WbPhysicsBackend::WbJointParam p, int axis) {
  return wbJointParamToOde(p) + axis * dParamGroup;
}

int WbOdeBackend::setJointHingeParam(WbJointHandle joint, WbJointParam param, double value) const {
  dJointSetHingeParam(static_cast<dJointID>(joint), wbJointParamToOde(param), value);
  return 0;
}

int WbOdeBackend::setJointSliderParam(WbJointHandle joint, WbJointParam param, double value) const {
  dJointSetSliderParam(static_cast<dJointID>(joint), wbJointParamToOde(param), value);
  return 0;
}

int WbOdeBackend::setJointAMotorParam(WbJointHandle joint, int axis, WbJointParam param, double value) const {
  dJointSetAMotorParam(static_cast<dJointID>(joint), wbJointParamToOdeAxis(param, axis), value);
  return 0;
}

int WbOdeBackend::setJointLMotorParam(WbJointHandle joint, int axis, WbJointParam param, double value) const {
  dJointSetLMotorParam(static_cast<dJointID>(joint), wbJointParamToOdeAxis(param, axis), value);
  return 0;
}

int WbOdeBackend::setJointHinge2Param(WbJointHandle joint, int axis, WbJointParam param, double value) const {
  dJointSetHinge2Param(static_cast<dJointID>(joint), wbJointParamToOdeAxis(param, axis), value);
  return 0;
}

int WbOdeBackend::setJointBallParam(WbJointHandle joint, int axis, WbJointParam param, double value) const {
  dJointSetBallParam(static_cast<dJointID>(joint), wbJointParamToOdeAxis(param, axis), value);
  return 0;
}

int WbOdeBackend::setJointEnabled(WbJointHandle joint, bool enabled) const {
  if (enabled)
    dJointEnable(static_cast<dJointID>(joint));
  else
    dJointDisable(static_cast<dJointID>(joint));
  return 0;
}

int WbOdeBackend::isJointEnabled(WbJointHandle joint) const {
  return dJointIsEnabled(static_cast<dJointID>(joint)) != 0 ? 1 : 0;
}
