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
#include "OmHinge2Joint.hpp"

#include "OmBrake.hpp"
#include "OmHingeJointParameters.hpp"
#include "OmJointParameters.hpp"
#include "OmLog.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMotor.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmPositionSensor.hpp"
#include "OmQuaternion.hpp"
#include "OmRobot.hpp"
#include "OmRotationalMotor.hpp"
#include "OmSolid.hpp"
#include "OmWorld.hpp"

// Constructors

void OmHinge2Joint::init() {
  mParameters2 = findSFNode("jointParameters2");
  mDevice2 = findMFNode("device2");

  // spring and dampingConstant

  // hidden field
  mPosition2 = findSFDouble("position2")->value();
  mOdePositionOffset2 = mPosition2;
  mSavedPositions2[stateId()] = mPosition2;
}

OmHinge2Joint::OmHinge2Joint(const QString &modelName, OmTokenizer *tokenizer) : OmHingeJoint(modelName, tokenizer) {
  init();
}

OmHinge2Joint::OmHinge2Joint(OmTokenizer *tokenizer) : OmHingeJoint("Hinge2Joint", tokenizer) {
  init();
}

OmHinge2Joint::OmHinge2Joint(const OmHinge2Joint &other) : OmHingeJoint(other) {
  init();
}

OmHinge2Joint::OmHinge2Joint(const OmNode &other) : OmHingeJoint(other) {
  init();
}

OmHinge2Joint::~OmHinge2Joint() {
}

void OmHinge2Joint::preFinalize() {
  OmHingeJoint::preFinalize();

  for (int i = 0; i < devices2Number(); ++i) {
    if (device2(i) && !device2(i)->isPreFinalizedCalled())
      device2(i)->preFinalize();
  }

  OmBaseNode *const p2 = dynamic_cast<OmBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPreFinalizedCalled())
    p2->preFinalize();

  updateParameters2();

  mSavedPositions2[stateId()] = mPosition2;
}

void OmHinge2Joint::postFinalize() {
  OmHingeJoint::postFinalize();

  for (int i = 0; i < devices2Number(); ++i) {
    if (device2(i) && !device2(i)->isPostFinalizedCalled())
      device2(i)->postFinalize();
  }

  OmBaseNode *const p2 = dynamic_cast<OmBaseNode *>(mParameters2->value());
  if (p2 && !p2->isPostFinalizedCalled())
    p2->postFinalize();

  connect(mDevice2, &OmMFNode::itemChanged, this, &OmHinge2Joint::addDevice2);
  connect(mDevice2, &OmMFNode::itemInserted, this, &OmHinge2Joint::addDevice2);
  connect(mParameters2, &OmSFNode::changed, this, &OmHinge2Joint::updateParameters);
  if (brake2())
    connect(brake2(), &OmBrake::brakingChanged, this, &OmHinge2Joint::updateSpringAndDampingConstants, Qt::UniqueConnection);
}

bool OmHinge2Joint::setJoint() {
  if (!OmBasicJoint::setJoint())
    return false;

  // The "can only connect Solid nodes that have a Physics node" warning that
  // used to fire here keyed on ODE bodies, which never existed after ODE went
  // away, so it fired for EVERY Hinge2Joint. Newton registers the joint
  // separately (OmBasicJoint flush) and reports a missing endpoint there.
  setOdeJoint();

  return true;
}

double OmHinge2Joint::position(int index) const {
  switch (index) {
    case 1:
      return mPosition;
    case 2:
      return mPosition2;
    default:
      return NAN;
  }
}

double OmHinge2Joint::initialPosition(int index) const {
  switch (index) {
    case 1:
      return mSavedPositions[stateId()];
    case 2:
      return mSavedPositions2[stateId()];
    default:
      return NAN;
  }
}

void OmHinge2Joint::setPosition(double position, int index) {
  OmJoint::setPosition(position, index);

  if (index != 2)
    return;

  mPosition2 = position;
  mOdePositionOffset2 = position;
  OmJointParameters *const p2 = parameters2();
  if (p2)
    p2->setPosition(mPosition2);
  // Axis 2 of a Hinge2 is the free-spinning roll axis of a steered wheel, so
  // this is the same reset-cascade writer as OmJoint::setPosition -- see the
  // comment there.
  OmMotor *const m2 = motor2();
  if (m2 && OmMotor::resetMayOverwriteMotorCommand())
    m2->setTargetPosition(position);
}

bool OmHinge2Joint::resetJointPositions() {
  mOdePositionOffset2 = 0.0;
  return OmJoint::resetJointPositions();
}

void OmHinge2Joint::updateOdePositionOffset() {
  OmJoint::updateOdePositionOffset();
  mOdePositionOffset2 = position(2);
}

void OmHinge2Joint::applyToOdeAxis() {
  updateOdePositionOffset();

  const OmMatrix4 &m4 = upperPose()->matrix();
  // compute orientation of rotation axis
  const OmVector3 &a1 = m4.sub3x3MatrixDot(axis());
  OmVector3 a2;
  if (mPosition == 0.0)
    a2 = m4.sub3x3MatrixDot(axis2());
  else {
    // compute axis2 based on axis1 rotation
    OmMatrix3 a1Matrix(axis(), mPosition);
    a2 = (m4.extracted3x3Matrix() * a1Matrix) * axis2();
  }
  const OmVector3 &c = a1.cross(a2);
  (void)c;  // ODE is gone: no ODE joint exists
}

void OmHinge2Joint::applyToOdeMinAndMaxStop() {
  // ODE removed: the two axes' hard stops were clamped onto the ODE hinge2's
  // per-axis LoStop/HiStop. On the (default-OFF) Newton d6 path the limits are
  // set once at registration from newtonAxisSpec, so nothing is pushed per
  // update; with that path off, Hinge2 hard stops are UNIMPLEMENTED.
}

void OmHinge2Joint::prePhysicsStep(double ms) {
  assert(solidEndPoint());
  OmRotationalMotor *const rm = rotationalMotor();
  OmRotationalMotor *const rm2 = rotationalMotor2();
  OmJointParameters *const p = parameters();
  OmJointParameters *const p2 = parameters2();

  if (isEnabled()) {
    // ⚠ MOTORISED Hinge2Joint ACTUATION IS UNIMPLEMENTED. ODE realised it as
    // per-axis hinge2 torques, or an FMax + Vel pair per axis; all of that is
    // gone. The Newton d6 path (registerNewtonMultiDof /
    // pushNewtonMultiDofTargets, reached from OmBasicJoint's per-tick push)
    // exists but is gated OFF by default behind OMNISIM_NEWTON_BALL_HINGE2,
    // because driving d6 POSITION targets through SolverMuJoCo is an upstream
    // newton defect, not ours (verified 2026-08-08). Do not "fix" it here.
    //
    // The muscle-stretch emits and the computeCurrentDynamicVelocity calls are
    // kept: the latter are called FOR THEIR SIDE EFFECT (they advance each
    // motor's PID state and mCurrentVelocity, read by the motor-sound manager
    // and the muscle visuals), not for a return value.
    if (rm && rm->userControl()) {
      if (rm->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(rm->rawInput() / rm->maxForceOrTorque(), false, 1);
    } else if (rm) {
      rm->computeCurrentDynamicVelocity(ms, mPosition);
    }

    if (rm2 && rm2->userControl()) {
      if (rm2->hasMuscles())
        // force is directly applied to the bodies and not included in joint motor feedback
        emit updateMuscleStretch(rm2->rawInput() / rm2->maxForceOrTorque(), false, 2);
    } else if (rm2) {
      rm2->computeCurrentDynamicVelocity(ms, mPosition2);
    }
    // eventually add spring and damping forces
  } else {
    const bool run1 = rm && rm->runKinematicControl(ms, mPosition);
    if (run1) {
      if (p)
        p->setPosition(mPosition);
      if (rm->hasMuscles()) {
        double velocityPercentage = rm->currentVelocity() / rm->maxVelocity();
        if (rm->kinematicVelocitySign() == -1)
          velocityPercentage = -velocityPercentage;
        emit updateMuscleStretch(velocityPercentage, true, 1);
      }
    }

    const bool run2 = rm2 && rm2->runKinematicControl(ms, mPosition2);
    if (run2) {
      if (p2)
        p2->setPosition(mPosition2);
      if (rm2->hasMuscles()) {
        double velocityPercentage = rm2->currentVelocity() / rm2->maxVelocity();
        if (rm2->kinematicVelocitySign() == -1)
          velocityPercentage = -velocityPercentage;
        emit updateMuscleStretch(velocityPercentage, true, 2);
      }
    }

    if (run1 || run2)
      updatePositions(mPosition, mPosition2);
  }
  mTimeStep = ms;
}

//////////////////////////////////////////////////////////////////////////////
// Newton multi-DoF registration + drive (OMNISIM_NEWTON_BALL_HINGE2)       //
//////////////////////////////////////////////////////////////////////////////

QStringList OmHinge2Joint::unmappedNewtonFields(const OmJointParameters *params, const OmBrake *brake,
                                                const QString &axisLabel) {
  QStringList out;
  if (params != nullptr) {
    if (params->springConstant() != 0.0 || params->dampingConstant() != 0.0)
      out << QString("%1 springConstant/dampingConstant").arg(axisLabel);
    if (params->staticFriction() != 0.0)
      out << QString("%1 staticFriction").arg(axisLabel);
  }
  if (brake != nullptr && brake->getBrakingDampingConstant() != 0.0)
    out << QString("%1 Brake dampingConstant").arg(axisLabel);
  return out;
}

void OmHinge2Joint::warnUnmappedNewtonFeatures(const QStringList &extra) const {
  QStringList unmapped = extra;
  // Axis-1 parameters of a Hinge2Joint are HingeJointParameters, which carry two
  // more ODE-solver knobs than the plain JointParameters of axis 2/3. (A
  // BallJoint's are BallJointParameters, so this cast is null there and neither
  // entry can fire -- correctly, since those fields don't exist on it.)
  const OmHingeJointParameters *const hp = hingeJointParameters();
  if (hp != nullptr) {
    if (hp->suspensionSpringConstant() != 0.0 || hp->suspensionDampingConstant() != 0.0)
      unmapped << QStringLiteral("suspensionSpringConstant/suspensionDampingConstant");
    if (hp->stopErp() != -1.0 || hp->stopCfm() != -1.0)
      // Newton's joint limits are a spring, tuned by OMNISIM_NEWTON_LIMIT_KE/KD,
      // not by ODE's error-reduction/constraint-force-mixing pair.
      unmapped << QStringLiteral("stopErp/stopCfm");
  }
  if (unmapped.isEmpty())
    return;
  // ONE warning per joint naming exactly what will not happen. The alternative
  // -- registering the joint and quietly dropping these fields -- is the failure
  // mode this whole gate exists to remove: a spring that does not push reads as
  // a physics bug, not as a missing feature.
  OmLog::warning(tr("%1 '%2': the Newton physics backend does not implement %3, so %4 no effect on this "
                    "joint. The joint CONSTRAINT is honoured -- the bodies stay connected -- but do NOT read that "
                    "as working actuation: a MOTORISED Hinge2Joint or BallJoint does not move at all, and its "
                    "position sensors read 0. There is no ODE path "
                    "left to fall back to (OMNISIM_FORCE_ODE is retired), so a load-bearing field here needs "
                    "a Newton-native implementation.")
                   .arg(nodeModelName())
                   .arg(endPointName())
                   .arg(unmapped.join(QStringLiteral(", ")))
                   .arg(unmapped.size() == 1 ? tr("it has") : tr("they have")),
                 false, OmLog::ODE);
}

bool OmHinge2Joint::hasAnyNewtonMotor() const {
  return motor() != nullptr || motor2() != nullptr;
}

int OmHinge2Joint::registerNewtonMultiDof(OmNewtonBackend *newton, int parentIdx, int childIdx,
                                          const OmVector3 &parentAnchor, const OmVector3 &childAnchor,
                                          const OmQuaternion &childRelRot) {
  if (newton == nullptr)
    return -1;
  // Axis frame. Both axes are expressed in the joint's PARENT anchor frame, which
  // is the parent Solid's frame (the registration passes an identity rotation for
  // parent_xform) -- exactly the frame Webots authors `axis` / `axis2` in, and the
  // same convention the revolute path uses. axis2 co-rotates with axis1 for free:
  // newton's d6 emits the two angular DoF as two MuJoCo hinge elements on the
  // child body, and MuJoCo composes a body's joints in declaration order, so
  // axis2's rotation is carried by axis1's. That is ODE's Hinge2 semantics
  // (axis2 attached to body2) without an intermediate body.
  OmVector3 ax1 = axis();
  OmVector3 ax2 = axis2();
  if (ax1.cross(ax2).isNull()) {
    // Same substitution + wording ODE's applyToOdeAxis uses: two parallel angular
    // axes make the second DoF redundant instead of universal.
    parsingWarn(tr("Hinge axes are aligned: using x and z axes instead."));
    ax1 = OmVector3(1.0, 0.0, 0.0);
    ax2 = OmVector3(0.0, 0.0, 1.0);
  }
  ax1.normalize();
  ax2.normalize();

  double ke1, kd1, lo1, hi1, e1, v1;
  double ke2, kd2, lo2, hi2, e2, v2;
  // positionByConstruction = false for both axes: a Hinge2's canonical shape is a
  // steered wheel (axis1 limited => position servo, axis2 limitless => velocity
  // wheel), so the revolute path's limits-based discriminator is the right one
  // here and a limitless axis2 correctly becomes a velocity wheel.
  newtonAxisSpec(motor(), parameters(), false, &ke1, &kd1, &lo1, &hi1, &e1, &v1);
  newtonAxisSpec(motor2(), parameters2(), false, &ke2, &kd2, &lo2, &hi2, &e2, &v2);
  warnUnmappedNewtonFeatures(unmappedNewtonFields(parameters(), brake(), tr("axis 1")) +
                             unmappedNewtonFields(parameters2(), brake2(), tr("axis 2")));

  const double axis1[3] = {ax1.x(), ax1.y(), ax1.z()};
  const double axis2v[3] = {ax2.x(), ax2.y(), ax2.z()};
  const double pAnchor[3] = {parentAnchor.x(), parentAnchor.y(), parentAnchor.z()};
  const double cAnchor[3] = {childAnchor.x(), childAnchor.y(), childAnchor.z()};
  const double cRot[4] = {childRelRot.x(), childRelRot.y(), childRelRot.z(), childRelRot.w()};
  const double gains[4] = {ke1, kd1, ke2, kd2};
  const double limits[4] = {lo1, hi1, lo2, hi2};
  const double efforts[2] = {e1, e2};
  const double velLimits[2] = {v1, v2};
  return newton->addJointHinge2Motorized(parentIdx, childIdx, axis1, axis2v, pAnchor, cAnchor, cRot,
                                         gains, limits, efforts, velLimits);
}

void OmHinge2Joint::pushNewtonMultiDofTargets(OmNewtonBackend *newton) {
  if (newton == nullptr || mNewtonJointIndex < 0)
    return;
  OmMotor *const motors[2] = {motor(), motor2()};
  for (int dof = 0; dof < 2; ++dof) {
    OmMotor *const m = motors[dof];
    if (m == nullptr)
      continue;
    pushNewtonAxisTarget(newton, mNewtonJointIndex, dof, m);
  }
}

void OmHinge2Joint::postPhysicsStep() {
  // Newton-backed multi-DoF readback (OMNISIM_NEWTON_BALL_HINGE2). The ODE hinge2
  // below is not advanced in a Newton-driven world, so without this the position
  // sensors report a frozen 0 while the bodies visibly move -- and the scene tree
  // would render the chain at ODE's angles while the Solid poses come from Newton
  // (the "body parts falling off" mismatch OmHingeJoint::postPhysicsStep
  // documents). d6 coordinates are one scalar per DoF in declaration order, so
  // dof 0 is axis1 and dof 1 is axis2, both already wrapped by the solver and
  // both signed in Webots' own right-hand-rule sense about the authored axes --
  // hence no negation and no angle-rate decorrelation dance, unlike ODE.
  if (newtonBallHinge2Enabled() && mNewtonJointIndex >= 0) {
    OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
    if (raw != nullptr && raw->isAvailable()) {
      OmNewtonBackend *const nb = static_cast<OmNewtonBackend *>(raw);
      mPosition = nb->getJointAngleDof(mNewtonJointIndex, 0);
      mPosition2 = nb->getJointAngleDof(mNewtonJointIndex, 1);
      OmJointParameters *const np = parameters();
      if (np)
        np->setPositionFromOde(mPosition);
      OmJointParameters *const np2 = parameters2();
      if (np2)
        np2->setPositionFromOde(mPosition2);
      const OmRotationalMotor *const nrm = rotationalMotor();
      const OmRotationalMotor *const nrm2 = rotationalMotor2();
      if (nrm && nrm->hasMuscles() && !nrm->userControl())
        emit updateMuscleStretch(nrm->computeFeedback() / nrm->maxForceOrTorque(), false, 1);
      if (nrm2 && nrm2->hasMuscles() && !nrm2->userControl())
        emit updateMuscleStretch(nrm2->computeFeedback() / nrm2->maxForceOrTorque(), false, 2);
      return;
    }
  }

  assert(mNewtonJointIndex >= 0);
  const OmRotationalMotor *const rm = rotationalMotor();
  const OmRotationalMotor *const rm2 = rotationalMotor2();

  OmJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);
  if (isEnabled() && rm && rm->hasMuscles() && !rm->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm->computeFeedback() / rm->maxForceOrTorque(), false, 1);

  OmJointParameters *const p2 = parameters2();
  if (p2)
    p2->setPositionFromOde(mPosition2);
  if (isEnabled() && rm2 && rm2->hasMuscles() && !rm2->userControl())
    // dynamic position or velocity control
    emit updateMuscleStretch(rm2->computeFeedback() / rm2->maxForceOrTorque(), false, 2);
}

void OmHinge2Joint::reset(const QString &id) {
  OmJoint::reset(id);

  for (int i = 0; i < mDevice2->size(); ++i)
    mDevice2->item(i)->reset(id);

  OmNode *const p = mParameters2->value();
  if (p)
    p->reset(id);

  setPosition(mSavedPositions2[id], 2);
}

void OmHinge2Joint::resetPhysics() {
  OmJoint::resetPhysics();

  OmMotor *const m = motor2();
  if (m)
    m->resetPhysics();
}

void OmHinge2Joint::save(const QString &id) {
  OmJoint::save(id);

  for (int i = 0; i < mDevice2->size(); ++i)
    mDevice2->item(i)->save(id);

  OmNode *const p = mParameters2->value();
  if (p)
    p->save(id);

  mSavedPositions2[id] = mPosition2;
}

void OmHinge2Joint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  OmRotation ir;
  OmVector3 it;
  retrieveEndPointSolidTranslationAndRotation(it, ir);

  OmQuaternion qp;
  if (OmMathsUtilities::isZeroAngle(mPosition) && OmMathsUtilities::isZeroAngle(mPosition2))
    mEndPointZeroRotation = ir;  // Keeps track of the original axis if the angle is zero as it defines the second DoF axis
  else {
    const OmQuaternion q(axis(), -mPosition);
    const OmQuaternion q2(axis2(), -mPosition2);
    qp = q2 * q;
    const OmQuaternion &iq = ir.toQuaternion();
    OmQuaternion qr = qp * iq;
    qr.normalize();
    mEndPointZeroRotation = OmRotation(qr);
  }
  const OmVector3 &a = anchor();
  const OmVector3 t(it - a);
  mEndPointZeroTranslation = qp * t + a;
}

void OmHinge2Joint::computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const {
  OmQuaternion qp;
  const OmQuaternion q(axis(), mPosition);
  const OmQuaternion q2(axis2(), mPosition2);
  OmQuaternion qi = mEndPointZeroRotation.toQuaternion();
  qp = q * q2;
  const OmVector3 &a = anchor();
  const OmVector3 t(mEndPointZeroTranslation - a);
  translation = qp * t + a;
  qp = qp * qi;
  qp.normalize();
  rotation.fromQuaternion(qp);
}

void OmHinge2Joint::updatePosition() {
  const OmJointParameters *const p = parameters();
  const OmJointParameters *const p2 = parameters2();

  if (solidReference() == NULL && solidEndPoint())
    updatePositions(p ? p->position() : mPosition, p2 ? p2->position() : mPosition2);
  emit updateMuscleStretch(0.0, true, 1);
  emit updateMuscleStretch(0.0, true, 2);
}

void OmHinge2Joint::updatePositions(double position, double position2) {
  OmSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move (user or Supervisor move) or in kinematic mode
  mPosition = position;
  mPosition2 = position2;
  OmMotor *m1 = motor();
  OmMotor *m2 = motor2();
  if (m1 && !m1->isConfigureDone())
    m1->setTargetPosition(position);
  if (m2 && !m2->isConfigureDone())
    m2->setTargetPosition(position2);
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

void OmHinge2Joint::updatePosition(double position) {
  updatePositions(mPosition, mPosition2);
}

void OmHinge2Joint::updateMinAndMaxStop(double min, double max) {
  const OmJointParameters *const p = dynamic_cast<OmJointParameters *>(sender());

  const OmRotationalMotor *rm = NULL;
  if (p == parameters2())
    rm = rotationalMotor2();
  else if (p == parameters())
    rm = rotationalMotor();

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

void OmHinge2Joint::updateParameters() {
  OmHingeJoint::updateParameters();
  updateParameters2();
}

// Update methods

void OmHinge2Joint::addDevice2(int index) {
  const OmSolid *const s = upperSolid();
  if (s) {
    OmRobot *const r = s->robot();
    assert(r);
    OmBaseNode *decendant = dynamic_cast<OmBaseNode *>(mDevice2->item(index));
    r->descendantNodeInserted(decendant);
  }
  OmBrake *brake = dynamic_cast<OmBrake *>(mDevice2->item(index));
  if (brake)
    connect(brake, &OmBrake::brakingChanged, this, &OmHinge2Joint::updateSpringAndDampingConstants, Qt::UniqueConnection);
}

void OmHinge2Joint::updateParameters2() {
  const OmJointParameters *const p2 = parameters2();
  if (p2) {
    mOdePositionOffset2 = p2->position();
    mPosition2 = mOdePositionOffset2;
    connect(p2, &OmJointParameters::minAndMaxStopChanged, this, &OmHinge2Joint::updateMinAndMaxStop, Qt::UniqueConnection);
    connect(p2, &OmJointParameters::springAndDampingConstantsChanged, this, &OmHinge2Joint::updateSpringAndDampingConstants,
            Qt::UniqueConnection);
    connect(p2, &OmJointParameters::axisChanged, this, &OmHinge2Joint::updateAxis, Qt::UniqueConnection);
    connect(p2, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
  }
}

OmJointParameters *OmHinge2Joint::parameters2() const {
  return dynamic_cast<OmJointParameters *>(mParameters2->value());
}

OmMotor *OmHinge2Joint::motor2() const {
  OmMotor *motor = NULL;
  for (int i = 0; i < mDevice2->size(); ++i) {
    motor = dynamic_cast<OmMotor *>(mDevice2->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

OmPositionSensor *OmHinge2Joint::positionSensor2() const {
  OmPositionSensor *sensor = NULL;
  for (int i = 0; i < mDevice2->size(); ++i) {
    sensor = dynamic_cast<OmPositionSensor *>(mDevice2->item(i));
    if (sensor)
      return sensor;
  }

  return NULL;
}

OmBrake *OmHinge2Joint::brake2() const {
  OmBrake *brake = NULL;
  for (int i = 0; i < mDevice2->size(); ++i) {
    brake = dynamic_cast<OmBrake *>(mDevice2->item(i));
    if (brake)
      return brake;
  }

  return NULL;
}

OmRotationalMotor *OmHinge2Joint::rotationalMotor2() const {
  OmRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice2->size(); ++i) {
    motor = dynamic_cast<OmRotationalMotor *>(mDevice2->item(i));
    if (motor)
      return motor;
  }

  return NULL;
}

OmVector3 OmHinge2Joint::axis2() const {
  static const OmVector3 DEFAULT_AXIS_2(0.0, 0.0, 1.0);
  const OmJointParameters *const p2 = parameters2();
  return p2 ? p2->axis() : DEFAULT_AXIS_2;
}

OmJointDevice *OmHinge2Joint::device2(int index) const {
  if (index >= 0 && mDevice2->size() > index)
    return dynamic_cast<OmJointDevice *>(mDevice2->item(index));
  else
    return NULL;
}

int OmHinge2Joint::devices2Number() const {
  return mDevice2->size();
}

QVector<OmLogicalDevice *> OmHinge2Joint::devices() const {
  QVector<OmLogicalDevice *> devices;
  int i = 0;
  for (i = 0; i < devicesNumber(); ++i)
    devices.append(device(i));
  for (i = 0; i < devices2Number(); ++i)
    devices.append(device2(i));

  return devices;
}

void OmHinge2Joint::createWrenObjects() {
  // D1.4: WREN axis visuals gone; only the device recursion survives.
  OmHingeJoint::createWrenObjects();

  // create objects for Muscle devices
  for (int i = 0; i < devices2Number(); ++i) {
    if (device2(i))
      device2(i)->createWrenObjects();
  }
}

void OmHinge2Joint::writeExport(OmWriter &writer) const {
  if (writer.isUrdf() && solidEndPoint()) {
    warn(tr("Exporting 'Hinge2Joint' nodes to URDF is currently not supported"));
    return;
  }
  OmBasicJoint::writeExport(writer);
}
