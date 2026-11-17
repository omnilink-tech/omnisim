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

#include "OmHingeJoint.hpp"

#include "OmBrake.hpp"
#include "OmHingeJointParameters.hpp"
#include "OmMathsUtilities.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmRotationalMotor.hpp"
#include "OmSolid.hpp"
#include "OmWorld.hpp"

#include <cassert>

void OmHingeJoint::init() {
  // D1.4: nothing left to initialise -- the WREN anchor visuals died with the
  // WREN renderer.
}

// Constructors
OmHingeJoint::OmHingeJoint(const QString &modelName, OmTokenizer *tokenizer) : OmJoint(modelName, tokenizer) {
  init();
}

OmHingeJoint::OmHingeJoint(OmTokenizer *tokenizer) : OmJoint("HingeJoint", tokenizer) {
  init();
}

OmHingeJoint::OmHingeJoint(const OmHingeJoint &other) : OmJoint(other) {
  init();
}

OmHingeJoint::OmHingeJoint(const OmNode &other) : OmJoint(other) {
  init();
}

OmHingeJoint::~OmHingeJoint() {
  // D1.4: WREN anchor visuals gone; nothing to clean up.
}

OmHingeJointParameters *OmHingeJoint::hingeJointParameters() const {
  return dynamic_cast<OmHingeJointParameters *>(mParameters->value());
}

OmRotationalMotor *OmHingeJoint::rotationalMotor() const {
  OmRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    motor = dynamic_cast<OmRotationalMotor *>(mDevice->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

void OmHingeJoint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  OmRotation ir;
  OmVector3 it;
  retrieveEndPointSolidTranslationAndRotation(it, ir);

  OmQuaternion qMinus;
  const double angle = mPosition;
  if (OmMathsUtilities::isZeroAngle(angle)) {
    // In case of a zero angle, the quaternion axis is undefined, so we keep track of the original one
    mEndPointZeroRotation = ir;
  } else {
    const OmVector3 &ax = axis().normalized();
    qMinus = OmQuaternion(ax, -angle);
    const OmQuaternion &q = ir.toQuaternion();
    OmQuaternion qNormalized = qMinus * q;
    if (qNormalized.w() != 1.0)
      qNormalized.normalize();
    mEndPointZeroRotation = OmRotation(qNormalized);
    if (mEndPointZeroRotation.angle() == 0.0)
      mEndPointZeroRotation = OmRotation(ax.x(), ax.y(), ax.z(), 0.0);
  }
  const OmVector3 &an = anchor();
  mEndPointZeroTranslation = qMinus * (it - an) + an;
}

void OmHingeJoint::computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const {
  const OmVector3 &ax = axis().normalized();
  const OmQuaternion q(ax, mPosition);
  const OmQuaternion iq(mEndPointZeroRotation.toQuaternion());
  OmQuaternion qp(q * iq);
  if (qp.w() != 1.0)
    qp.normalize();
  rotation.fromQuaternion(qp);
  if (rotation.angle() == 0.0)
    rotation = OmRotation(ax.x(), ax.y(), ax.z(), 0.0);
  const OmVector3 &a = anchor();
  translation = q * (mEndPointZeroTranslation - a) + a;
}

bool OmHingeJoint::setJoint() {
  if (!OmBasicJoint::setJoint())
    return false;

  setOdeJoint();

  return true;
}

void OmHingeJoint::setOdeJoint() {
  OmJoint::setOdeJoint();
  // compute and set the anchor point and suspension
  applyToOdeAnchor();
  applyToOdeSuspension();
}

void OmHingeJoint::applyToOdeMinAndMaxStop() {
  // ODE removed: hard stops reach physics through the Newton joint's own
  // per-axis limits, set once at registration (registerNewton* -> newtonAxisSpec),
  // so there is nothing to push per update here. Unreachable in any case --
  // every caller gates on `mJoint`, which is now permanently NULL.
}

void OmHingeJoint::applyToOdeStopErp() {
  // ODE removed: HingeJointParameters.stopErp is now UNIMPLEMENTED -- it named
  // ODE's constraint error-reduction parameter, which has no Newton analogue
  // (Newton's joint/contact stiffness comes from newtonContactKe & friends).
}

void OmHingeJoint::applyToOdeStopCfm() {
  // ODE removed: HingeJointParameters.stopCfm is now UNIMPLEMENTED -- see
  // applyToOdeStopErp above.
}

void OmHingeJoint::applyToOdeAxis() {
  // Only the position-offset bookkeeping survives; the axis itself reaches
  // Newton through the joint registration in OmBasicJoint, not through here.
  updateOdePositionOffset();
}

void OmHingeJoint::applyToOdeSuspensionAxis() {
  // suspension along the suspension axis
  const OmHingeJointParameters *const hp = hingeJointParameters();
  (void)hp;  // suspension is UNIMPLEMENTED on Newton (see applyToOdeSuspension)
}

void OmHingeJoint::applyToOdeAnchor() {
  updateOdePositionOffset();
}

void OmHingeJoint::applyToOdeSuspension() {
  // ODE removed: HingeJointParameters' suspensionSpringConstant /
  // suspensionDampingConstant / suspensionAxis are now UNIMPLEMENTED -- they
  // were an ODE suspension-ERP/CFM pair on the hinge (or hinge2 axis 0) and
  // Newton has no suspension concept. applyToOdeSuspensionAxis() is retained
  // because it is also called on axis updates, but it too does nothing now
  // (it re-checks hingeJointParameters() itself).
  applyToOdeSuspensionAxis();
}

void OmHingeJoint::prePhysicsStep(double ms) {
  assert(solidEndPoint());
  OmRotationalMotor *const rm = rotationalMotor();
  OmJointParameters *const p = parameters();
  if (isEnabled()) {
    if (rm && rm->userControl()) {
      // ODE removed: the user torque reaches physics through OmBasicJoint's
      // per-tick Newton push (pushNewtonJointTargets -> setJointForce), which
      // owns motor control for every Newton-registered hinge.
      const double torque = rm->rawInput();
      if (rm->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(torque / rm->maxForceOrTorque(), false, 1);
    } else {
      // ODE removed: the FMax + Vel motor pair is gone -- OmBasicJoint's
      // per-tick Newton push owns position/velocity control. The call below is
      // KEPT FOR ITS SIDE EFFECT, not its return value: it advances the motor's
      // PID state and mCurrentVelocity, which the motor-sound manager, the
      // kinematic differential-wheels model and the muscle visuals read.
      if (rm)
        rm->computeCurrentDynamicVelocity(ms, mPosition);
    }
    // eventually add spring and damping forces
  } else if (rm && rm->runKinematicControl(ms, mPosition)) {  // kinematic mode
    if (p)
      p->setPosition(mPosition);
    else
      updatePosition(mPosition);
    if (rm->hasMuscles()) {
      double velocityPercentage = rm->currentVelocity() / rm->maxVelocity();
      if (rm->kinematicVelocitySign() == -1)
        velocityPercentage = -velocityPercentage;
      emit updateMuscleStretch(velocityPercentage, true, 1);
    }
  }
  mTimeStep = ms;
}

void OmHingeJoint::postPhysicsStep() {
  const OmRotationalMotor *const rm = rotationalMotor();

  // Newton is the only truth for the joint angle now. The ODE branch that used
  // to follow (dJointGetHingeAngleRate + dJointGetHingeAngle, renormalised
  // against mOdePositionOffset) is gone; against the inert stub it read zeros
  // and CLOBBERED mPosition with mOdePositionOffset every step, so dropping it
  // is what preserves the angle. A joint with no Newton registration now simply
  // keeps its last mPosition -- reading it back is UNIMPLEMENTED.
  if (mNewtonJointIndex >= 0) {
    OmPhysicsBackend *const newton = OmPhysicsBackendRegistry::newtonBackend();
    if (newton != nullptr && newton->isAvailable()) {
      double angle = 0.0;
      if (newton->getJointHingeAngle(OmNewtonBackend::handleFromIndex(mNewtonJointIndex), &angle) == 0)
        mPosition = angle;
    }
  }

  OmJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);

  if (isEnabled() && rm && rm->hasMuscles() && !rm->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false, 1);
}

void OmHingeJoint::updatePosition() {
  // Update triggered by an artificial move, i.e. a move caused by the user or a Supervisor
  const OmJointParameters *const p = parameters();

  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p ? p->position() : mPosition);

  emit updateMuscleStretch(0.0, true, 1);
}

void OmHingeJoint::updatePosition(double position) {
  OmSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move
  mPosition = position;
  OmMotor *m = motor();
  if (m && !m->isConfigureDone())
    m->setTargetPosition(position);
  OmVector3 translation;
  OmRotation rotation;
  computeEndPointSolidPositionFromParameters(translation, rotation);
  if (!translation.almostEquals(s->translation()) || !rotation.almostEquals(s->rotation())) {
    mIsEndPointPositionChangedByJoint = true;
    s->setTranslationAndRotation(translation, rotation);
    s->resetPhysics();
    mIsEndPointPositionChangedByJoint = false;
  }
}

// Updates

void OmHingeJoint::updateParameters() {
  OmJoint::updateParameters();
  const OmHingeJointParameters *const p = hingeJointParameters();
  if (p) {
    connect(p, &OmHingeJointParameters::anchorChanged, this, &OmHingeJoint::updateAnchor, Qt::UniqueConnection);
    connect(p, &OmHingeJointParameters::suspensionChanged, this, &OmHingeJoint::updateSuspension, Qt::UniqueConnection);
    connect(p, &OmHingeJointParameters::stopErpChanged, this, &OmHingeJoint::updateStopErp, Qt::UniqueConnection);
    connect(p, &OmHingeJointParameters::stopCfmChanged, this, &OmHingeJoint::updateStopCfm, Qt::UniqueConnection);
  }
}

void OmHingeJoint::updateSuspension() {
  if (isEnabled())
    applyToOdeSuspension();
}

void OmHingeJoint::updateMinAndMaxStop(double min, double max) {
  const OmJointParameters *const p = dynamic_cast<OmJointParameters *>(sender());
  if (min <= -M_PI)
    p->parsingWarn(tr("HingeJoint 'minStop' must be greater than -pi to be effective."));

  if (max >= M_PI)
    p->parsingWarn(tr("HingeJoint 'maxStop' must be less than pi to be effective."));

  const OmRotationalMotor *const rm = rotationalMotor();
  if (rm) {
    const double minPos = rm->minPosition();
    const double maxPos = rm->maxPosition();
    if (min != max && minPos != maxPos) {
      if (minPos < min)
        p->parsingWarn(tr("HingeJoint 'minStop' must be less or equal to RotationalMotor 'minPosition'."));

      if (maxPos > max)
        p->parsingWarn(tr("HingeJoint 'maxStop' must be greater or equal to RotationalMotor 'maxPosition'."));
    }
  }
}

void OmHingeJoint::updateStopErp() {
}

void OmHingeJoint::updateStopCfm() {
}

void OmHingeJoint::updateAnchor() {
  // update the current endPoint pose based on the new anchor value
  // but do not modify the initial endPoint pose
  updatePosition();
}

OmVector3 OmHingeJoint::axis() const {
  static const OmVector3 DEFAULT_AXIS(1.0, 0.0, 0.0);
  const OmJointParameters *const p = parameters();
  return p ? p->axis() : DEFAULT_AXIS;
}

OmVector3 OmHingeJoint::anchor() const {
  const OmHingeJointParameters *const p = hingeJointParameters();
  return p ? p->anchor() : OmBasicJoint::anchor();
}
