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

#include "OmOdeBackend.hpp"

// Why this file still exists: the header declares the widened dispatcher ops
// out-of-line, and something has to define them or the tree does not link. It
// used to hold the real ODE calls behind <ode/ode.h> -- kept out of the header
// so the dependency stayed contained rather than propagating through the
// dispatcher abstraction. There is no longer an ODE to contain; what is left is
// the set of definitions that keeps the tombstone linkable.

// ⚠ ODE HAS BEEN DELETED AND THIS IS NOW AN INERT STUB, NOT A BACKEND.
// Every symbol the header declares survives so the tree links, but every verb
// returns -1 ("not supported") and getters zero their outputs so a caller that
// ignores the return code never reads uninitialized memory. isAvailable() is
// false, so resolve() will hand this object out only when Newton is
// unavailable -- and a world routed here has NO physics, not legacy physics.
//
// The call sites that used to bypass resolve() and isAvailable() to reach
// odeBackend() directly -- 113 of them, each a silently-dead physics operation
// after the deletion -- have been removed. Three of those removals fixed live
// corruption rather than dead code (a per-step mPosition clobber, a velocity
// zeroing, and a divide-by-zero in OmTrack), which is the argument for never
// adding another: a dispatch to this object cannot fail loudly.
//
// If you are adding one, you want the resolved backend, not this one.

int OmOdeBackend::getBodyPosition(OmBodyHandle, double pos[3]) const {
  pos[0] = pos[1] = pos[2] = 0.0;
  return -1;
}
int OmOdeBackend::setBodyPosition(OmBodyHandle, const double[3]) const {
  return -1;
}
int OmOdeBackend::getBodyQuaternion(OmBodyHandle, double q[4]) const {
  q[0] = 1.0;
  q[1] = q[2] = q[3] = 0.0;
  return -1;
}
int OmOdeBackend::setBodyQuaternion(OmBodyHandle, const double[4]) const {
  return -1;
}
int OmOdeBackend::getBodyLinearVel(OmBodyHandle, double v[3]) const {
  v[0] = v[1] = v[2] = 0.0;
  return -1;
}
int OmOdeBackend::setBodyLinearVel(OmBodyHandle, const double[3]) const {
  return -1;
}
int OmOdeBackend::getBodyAngularVel(OmBodyHandle, double v[3]) const {
  v[0] = v[1] = v[2] = 0.0;
  return -1;
}
int OmOdeBackend::setBodyAngularVel(OmBodyHandle, const double[3]) const {
  return -1;
}
int OmOdeBackend::addBodyForceAtPos(OmBodyHandle, const double[3], const double[3]) const {
  return -1;
}
int OmOdeBackend::addBodyForceAtRelPos(OmBodyHandle, const double[3], const double[3]) const {
  return -1;
}
int OmOdeBackend::addBodyTorque(OmBodyHandle, const double[3]) const {
  return -1;
}
int OmOdeBackend::setBodyForce(OmBodyHandle, const double[3]) const {
  return -1;
}
int OmOdeBackend::setBodyTorque(OmBodyHandle, const double[3]) const {
  return -1;
}
int OmOdeBackend::getBodyPointVel(OmBodyHandle, const double[3], double v[3]) const {
  v[0] = v[1] = v[2] = 0.0;
  return -1;
}
int OmOdeBackend::setBodyEnabled(OmBodyHandle, bool) const {
  return -1;
}
int OmOdeBackend::isBodyEnabled(OmBodyHandle) const {
  return -1;
}
int OmOdeBackend::getBodyMass(OmBodyHandle, double *mass) const {
  *mass = 0.0;
  return -1;
}
int OmOdeBackend::setBodyMaxAngularSpeed(OmBodyHandle, double) const {
  return -1;
}
int OmOdeBackend::setBodyDamping(OmBodyHandle, double, double) const {
  return -1;
}
int OmOdeBackend::setBodyDampingDefaults(OmBodyHandle) const {
  return -1;
}
int OmOdeBackend::setBodyAutoDisableFlag(OmBodyHandle, bool) const {
  return -1;
}
int OmOdeBackend::setBodyAutoDisableLinearThreshold(OmBodyHandle, double) const {
  return -1;
}
int OmOdeBackend::setBodyAutoDisableAngularThreshold(OmBodyHandle, double) const {
  return -1;
}
int OmOdeBackend::setBodyAutoDisableTime(OmBodyHandle, double) const {
  return -1;
}
int OmOdeBackend::getJointHingeAngle(OmJointHandle, double *angleOut) const {
  *angleOut = 0.0;
  return -1;
}
int OmOdeBackend::getJointHingeAngleRate(OmJointHandle, double *rateOut) const {
  *rateOut = 0.0;
  return -1;
}
int OmOdeBackend::getJointSliderPosition(OmJointHandle, double *positionOut) const {
  *positionOut = 0.0;
  return -1;
}
int OmOdeBackend::getJointAMotorAngle(OmJointHandle, int, double *angleOut) const {
  *angleOut = 0.0;
  return -1;
}
int OmOdeBackend::getJointAMotorAngleRate(OmJointHandle, int, double *rateOut) const {
  *rateOut = 0.0;
  return -1;
}
int OmOdeBackend::addJointHingeTorque(OmJointHandle, double) const {
  return -1;
}
int OmOdeBackend::addJointSliderForce(OmJointHandle, double) const {
  return -1;
}
int OmOdeBackend::addJointAMotorTorques(OmJointHandle, double, double, double) const {
  return -1;
}
int OmOdeBackend::addJointHinge2Torques(OmJointHandle, double, double) const {
  return -1;
}
int OmOdeBackend::setJointHingeParam(OmJointHandle, OmJointParam, double) const {
  return -1;
}
int OmOdeBackend::setJointSliderParam(OmJointHandle, OmJointParam, double) const {
  return -1;
}
int OmOdeBackend::setJointAMotorParam(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}
int OmOdeBackend::setJointLMotorParam(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}
int OmOdeBackend::setJointHinge2Param(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}
int OmOdeBackend::setJointBallParam(OmJointHandle, int, OmJointParam, double) const {
  return -1;
}
int OmOdeBackend::setJointEnabled(OmJointHandle, bool) const {
  return -1;
}
int OmOdeBackend::isJointEnabled(OmJointHandle) const {
  return -1;
}
