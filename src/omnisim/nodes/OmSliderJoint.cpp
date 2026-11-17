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

#include "OmSliderJoint.hpp"

#include "OmBrake.hpp"
#include "OmJointParameters.hpp"
#include "OmLinearMotor.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmSolid.hpp"
#include "OmSolidMerger.hpp"
#include "OmWorld.hpp"

#include <cassert>

// Constructors

OmSliderJoint::OmSliderJoint(OmTokenizer *tokenizer) : OmJoint("SliderJoint", tokenizer) {
}

OmSliderJoint::OmSliderJoint(const OmSliderJoint &other) : OmJoint(other) {
}

OmSliderJoint::OmSliderJoint(const OmNode &other) : OmJoint(other) {
}

OmSliderJoint::~OmSliderJoint() {
}

OmLinearMotor *OmSliderJoint::linearMotor() const {
  OmLinearMotor *motor = NULL;
  for (int i = 0; i < mDevice->size(); ++i) {
    motor = dynamic_cast<OmLinearMotor *>(mDevice->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

void OmSliderJoint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  OmRotation ir;
  OmVector3 it;

  retrieveEndPointSolidTranslationAndRotation(it, ir);

  mEndPointZeroRotation = ir;
  mEndPointZeroTranslation = it - mPosition * axis();
}

void OmSliderJoint::computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const {
  translation = mEndPointZeroTranslation + mPosition * axis();
  rotation = mEndPointZeroRotation;
}

// Update methods: they check validity and correct if necessary

bool OmSliderJoint::setJoint() {
  if (!OmBasicJoint::setJoint())
    return false;

  setOdeJoint();

  return true;
}

void OmSliderJoint::applyToOdeMinAndMaxStop() {
  // ODE removed: hard stops reach physics through the Newton prismatic joint's
  // own limits, set once at registration (newtonAxisSpec), so there is nothing
  // to push per update here. Unreachable in any case -- the caller
  // (updateMinAndMaxStop) gates on `mJoint`, which is now permanently NULL.
}

void OmSliderJoint::applyToOdeAxis() {
  updateOdePositionOffset();
}

void OmSliderJoint::updatePosition() {
  // Update triggered by an artificial move, i.e. a move caused by the user or a Supervisor
  const OmJointParameters *const p = parameters();

  if (solidReference() == NULL && solidEndPoint())
    updatePosition(p ? p->position() : mPosition);

  emit updateMuscleStretch(0.0, true, 1);
}

void OmSliderJoint::updatePosition(double position) {
  OmSolid *const s = solidEndPoint();
  assert(s);
  // called after a special artificial move, i.e. a statically based robot was moved
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

void OmSliderJoint::updateMinAndMaxStop(double min, double max) {
  const OmJointParameters *const p = dynamic_cast<OmJointParameters *>(sender());
  const OmLinearMotor *const lm = linearMotor();
  if (lm) {
    const double minPos = lm->minPosition();
    const double maxPos = lm->maxPosition();
    if (min != max && minPos != maxPos) {
      if (minPos < min)
        p->parsingWarn(tr("SliderJoint 'minStop' must be less or equal to LinearMotor 'minPosition'."));

      if (maxPos > max)
        p->parsingWarn(tr("SliderJoint 'maxStop' must be greater or equal to LinearMotor 'maxPosition'."));
    }
  }
}

void OmSliderJoint::updateParameters() {
  OmJoint::updateParameters();
}

void OmSliderJoint::prePhysicsStep(double ms) {
  OmJointParameters *const p = parameters();
  OmLinearMotor *const lm = linearMotor();

  if (isEnabled()) {
    if (lm && lm->userControl()) {
      // ODE removed: the user force reaches physics through OmBasicJoint's
      // per-tick Newton push (pushNewtonJointTargets -> setJointForce), which
      // handles prismatic joints alongside hinges.
      const double force = lm->rawInput();
      if (lm->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(force / lm->maxForceOrTorque(), false, 1);
    } else {
      // ODE removed: the FMax + Vel motor pair is gone -- OmBasicJoint's
      // per-tick Newton push owns position/velocity control. The call below is
      // KEPT FOR ITS SIDE EFFECT, not its return value: it advances the motor's
      // PID state and mCurrentVelocity, which the motor-sound manager and the
      // muscle visuals read.
      if (lm)
        lm->computeCurrentDynamicVelocity(ms, mPosition);
    }
  } else if (lm) {
    if (lm->runKinematicControl(ms, mPosition)) {
      if (p)
        p->setPosition(mPosition);
      else
        updatePosition(mPosition);
    }
    if (lm->hasMuscles()) {
      double velocityPercentage = lm->currentVelocity() / lm->maxVelocity();
      if (lm->kinematicVelocitySign() == -1)
        velocityPercentage = -velocityPercentage;
      emit updateMuscleStretch(velocityPercentage, true, 1);
    }
  }
}

void OmSliderJoint::postPhysicsStep() {
  // Newton-backed prismatic joints (gripper fingers) expose their slider
  // offset through the same joint_q readback as hinges -- get_joint_angle
  // returns the generalized coordinate, which is a linear distance for a
  // prismatic. That is the only readback now: the ODE fallback
  // (dJointGetSliderPosition) is gone, and against the inert stub it read zero
  // and CLOBBERED mPosition with mOdePositionOffset every step, so dropping it
  // is what preserves the offset. A slider with no Newton registration keeps its
  // last mPosition -- reading it back is UNIMPLEMENTED.
  if (mNewtonJointIndex >= 0) {
    OmPhysicsBackend *const newton = OmPhysicsBackendRegistry::newtonBackend();
    if (newton != nullptr && newton->isAvailable()) {
      double pos = 0.0;
      if (newton->getJointHingeAngle(OmNewtonBackend::handleFromIndex(mNewtonJointIndex), &pos) == 0)
        mPosition = pos;
    }
  }
  OmJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);

  if (isEnabled()) {
    const OmLinearMotor *const lm = linearMotor();
    if (lm && lm->hasMuscles() && !lm->userControl())
      // dynamic position or velocity control
      emit updateMuscleStretch(-lm->computeFeedback() / lm->maxForceOrTorque(), false, 1);
  }
}

OmVector3 OmSliderJoint::axis() const {
  static const OmVector3 DEFAULT_AXIS(0.0, 0.0, 1.0);
  const OmJointParameters *const p = parameters();
  return p ? p->axis().normalized() : DEFAULT_AXIS;
}

OmVector3 OmSliderJoint::anchor() const {
  const OmSolid *const s = solidEndPoint();
  const OmSolidReference *const sr = solidReference();
  if (s && !sr)
    return mEndPointZeroTranslation;
  else if (s) {
    const OmVector3 &a = s->position();
    const OmPose *const up = upperPose();
    return up->matrix().pseudoInversed(a);
  }

  return OmBasicJoint::anchor();
}

void OmSliderJoint::writeExport(OmWriter &writer) const {
  if (writer.isUrdf() && solidEndPoint()) {
    const OmNode *const parentRoot = findUrdfLinkRoot();
    const OmVector3 currentOffset = solidEndPoint()->translation() - anchor();
    const OmVector3 translation = solidEndPoint()->translationFrom(parentRoot) - currentOffset + writer.jointOffset();
    writer.setJointOffset(solidEndPoint()->rotationMatrixFrom(parentRoot).transposed() * currentOffset);
    const OmVector3 eulerRotation = solidEndPoint()->rotationMatrixFrom(parentRoot).toEulerAnglesZYX();
    const OmVector3 rotationAxis = axis() * solidEndPoint()->rotationMatrixFrom(parentRoot);

    writer.increaseIndent();
    writer.indent();
    writer << QString("<joint name=\"%1\" type=\"prismatic\">\n").arg(urdfName());

    writer.increaseIndent();
    writer.indent();
    writer << QString("<parent link=\"%1\"/>\n").arg(parentRoot->urdfName());
    writer.indent();
    writer << QString("<child link=\"%1\"/>\n").arg(solidEndPoint()->urdfName());
    writer.indent();
    writer << QString("<axis xyz=\"%1\"/>\n").arg(rotationAxis.toString(OmPrecision::FLOAT_ROUND_6));
    writer.indent();
    writer << QString("<origin xyz=\"%1\" rpy=\"%2\"/>\n")
                .arg(translation.toString(OmPrecision::FLOAT_ROUND_6))
                .arg(eulerRotation.toString(OmPrecision::FLOAT_ROUND_6));
    writer.indent();
    const OmMotor *m = motor();
    if (m) {
      if (m->minPosition() != 0.0 || m->maxPosition() != 0.0)
        writer << QString("<limit effort=\"%1\" lower=\"%2\" upper=\"%3\" velocity=\"%4\"/>\n")
                    .arg(m->maxForceOrTorque())
                    .arg(m->minPosition())
                    .arg(m->maxPosition())
                    .arg(m->maxVelocity());
      else
        writer << QString("<limit effort=\"%1\" velocity=\"%2\"/>\n").arg(m->maxForceOrTorque()).arg(m->maxVelocity());
    }
    writer.decreaseIndent();

    writer.indent();
    writer << QString("</joint>\n");
    writer.decreaseIndent();

    OmNode::exportNodeSubNodes(writer);
    return;
  }

  OmNode::writeExport(writer);
}
