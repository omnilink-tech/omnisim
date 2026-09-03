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

#include "OmBallJoint.hpp"
#include "OmBallJointParameters.hpp"
#include "OmBrake.hpp"
#include "OmJointParameters.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMotor.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmPositionSensor.hpp"
#include "OmQuaternion.hpp"
#include "OmRobot.hpp"
#include "OmRotationalMotor.hpp"
#include "OmSFNode.hpp"
#include "OmSolid.hpp"
#include "OmWorld.hpp"

#include <QtCore/QStringList>
#include <cmath>


// Constructors

void OmBallJoint::init() {
  mParameters3 = findSFNode("jointParameters3");
  mDevice3 = findMFNode("device3");

  // hidden field
  mPosition3 = findSFDouble("position3")->value();
  mOdePositionOffset3 = mPosition3;
  mSavedPositions3[stateId()] = mPosition3;

}

OmBallJoint::OmBallJoint(OmTokenizer *tokenizer) : OmHinge2Joint("BallJoint", tokenizer) {
  init();
}

OmBallJoint::OmBallJoint(const OmBallJoint &other) : OmHinge2Joint(other) {
  init();
}

OmBallJoint::OmBallJoint(const OmNode &other) : OmHinge2Joint(other) {
  init();
}

OmBallJoint::~OmBallJoint() {
}

OmBallJointParameters *OmBallJoint::ballJointParameters() const {
  return dynamic_cast<OmBallJointParameters *>(mParameters->value());
}

OmJointParameters *OmBallJoint::parameters3() const {
  return dynamic_cast<OmJointParameters *>(mParameters3->value());
}

OmMotor *OmBallJoint::motor3() const {
  OmMotor *motor = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    motor = dynamic_cast<OmMotor *>(mDevice3->item(i));
    if (motor)
      return motor;
  }
  return NULL;
}

OmPositionSensor *OmBallJoint::positionSensor3() const {
  OmPositionSensor *sensor = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    sensor = dynamic_cast<OmPositionSensor *>(mDevice3->item(i));
    if (sensor)
      return sensor;
  }
  return NULL;
}

OmBrake *OmBallJoint::brake3() const {
  OmBrake *brake = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    brake = dynamic_cast<OmBrake *>(mDevice3->item(i));
    if (brake)
      return brake;
  }
  return NULL;
}

OmRotationalMotor *OmBallJoint::rotationalMotor3() const {
  OmRotationalMotor *motor = NULL;
  for (int i = 0; i < mDevice3->size(); ++i) {
    motor = dynamic_cast<OmRotationalMotor *>(mDevice3->item(i));
    if (motor)
      return motor;
  }
  return NULL;
}

OmJointDevice *OmBallJoint::device3(int index) const {
  if (index >= 0 && mDevice3->size() > index)
    return dynamic_cast<OmJointDevice *>(mDevice3->item(index));
  return NULL;
}

int OmBallJoint::devices3Number() const {
  return mDevice3->size();
}

QVector<OmLogicalDevice *> OmBallJoint::devices() const {
  QVector<OmLogicalDevice *> devices;
  int i = 0;
  for (i = 0; i < devicesNumber(); ++i)
    devices.append(device(i));
  for (i = 0; i < devices2Number(); ++i)
    devices.append(device2(i));
  for (i = 0; i < devices3Number(); ++i)
    devices.append(device3(i));

  return devices;
}

OmVector3 OmBallJoint::anchor() const {
  static const OmVector3 ZERO(0.0, 0.0, 0.0);
  const OmBallJointParameters *const p = ballJointParameters();
  return p ? p->anchor() : ZERO;
}

OmVector3 OmBallJoint::axis() const {
  const OmJointParameters *const p2 = parameters2();
  const OmJointParameters *const p3 = parameters3();
  if (!p2) {
    if (!p3)
      return OmVector3(1.0, 0.0, 0.0);
    else if (p3->axis().cross(OmVector3(0.0, 0.0, 1.0)).isNull())
      return p3->axis().cross(OmVector3(1.0, 0.0, 0.0));
    else
      return p3->axis().cross(OmVector3(0.0, 0.0, 1.0));
  }
  return p2->axis();
}

OmVector3 OmBallJoint::axis2() const {
  return axis3().cross(axis());
}

OmVector3 OmBallJoint::axis3() const {
  const OmJointParameters *const p2 = parameters2();
  const OmJointParameters *const p3 = parameters3();
  if (!p3) {
    if (!p2)
      return OmVector3(0.0, 0.0, 1.0);
    else if (p2->axis().cross(OmVector3(1.0, 0.0, 0.0)).isNull())
      return p2->axis().cross(OmVector3(0.0, 0.0, 1.0));
    else
      return p2->axis().cross(OmVector3(1.0, 0.0, 0.0));
  }
  return p3->axis();
}

void OmBallJoint::updateEndPointZeroTranslationAndRotation() {
  if (solidEndPoint() == NULL)
    return;

  OmRotation ir;
  OmVector3 it;
  retrieveEndPointSolidTranslationAndRotation(it, ir);

  OmQuaternion qp;
  if (OmMathsUtilities::isZeroAngle(mPosition) && OmMathsUtilities::isZeroAngle(mPosition2) &&
      OmMathsUtilities::isZeroAngle(mPosition3))
    mEndPointZeroRotation = ir;  // Keeps track of the original axis if the angle is zero as it defines the second DoF axis
  else {
    const OmQuaternion q(axis(), -mPosition);
    const OmQuaternion q2(axis2(), -mPosition2);
    const OmQuaternion q3(axis3(), -mPosition3);
    qp = q3 * q2 * q;
    const OmQuaternion &iq = ir.toQuaternion();
    OmQuaternion qr = qp * iq;
    qr.normalize();
    mEndPointZeroRotation = OmRotation(qr);
  }
  const OmVector3 &a = anchor();
  const OmVector3 t(it - a);
  mEndPointZeroTranslation = qp * t + a;
}

void OmBallJoint::computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const {
  OmQuaternion qp;
  const OmQuaternion q(axis(), mPosition);
  const OmQuaternion q2(axis2(), mPosition2);
  const OmQuaternion q3(axis3(), mPosition3);
  OmQuaternion qi = mEndPointZeroRotation.toQuaternion();
  qp = q * q2 * q3;
  const OmVector3 &a = anchor();
  const OmVector3 t(mEndPointZeroTranslation - a);
  translation = qp * t + a;
  qp = qp * qi;
  qp.normalize();
  rotation.fromQuaternion(qp);
}

void OmBallJoint::updatePosition() {
  const OmJointParameters *const p = parameters();
  const OmJointParameters *const p2 = parameters2();
  const OmJointParameters *const p3 = parameters3();

  if (solidReference() == NULL && solidEndPoint())
    updatePositions(p ? p->position() : mPosition, p2 ? p2->position() : mPosition2, p3 ? p3->position() : mPosition3);
}

void OmBallJoint::updatePositions(double position, double position2, double position3) {
  OmSolid *const s = solidEndPoint();
  assert(s);
  // called after an artificial move (user or Supervisor move) or in kinematic mode
  mPosition = position;
  mPosition2 = position2;
  mPosition3 = position3;
  OmMotor *m1 = motor();
  OmMotor *m2 = motor2();
  OmMotor *m3 = motor3();
  if (m1 && !m1->isConfigureDone())
    m1->setTargetPosition(position);
  if (m2 && !m2->isConfigureDone())
    m2->setTargetPosition(position2);
  if (m3 && !m3->isConfigureDone())
    m3->setTargetPosition(position3);
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

void OmBallJoint::updatePosition(double position) {
  updatePositions(mPosition, mPosition2, mPosition3);
}

void OmBallJoint::updateParameters() {
  OmHinge2Joint::updateParameters();
  updateParameters3();
  if (ballJointParameters())
    connect(ballJointParameters(), &OmBallJointParameters::anchorChanged, this, &OmBallJoint::updateAnchor,
            Qt::UniqueConnection);
}

void OmBallJoint::checkMotorLimit() {
  OmMotor *motor = motor2();

  if (!motor)
    return;

  if (motor->minPosition() == 0.0 && motor->maxPosition() == 0.0) {
    motor->setMinPosition(-M_PI_2);
    motor->setMaxPosition(M_PI_2);
  } else {
    if (motor->minPosition() < -M_PI_2) {
      motor->setMinPosition(-M_PI_2);
      parsingWarn(tr("The lower limit of the motor associated to the second axis shouldn't be smaller than -pi/2."));
    }
    if (motor->maxPosition() > M_PI_2) {
      motor->setMaxPosition(M_PI_2);
      parsingWarn(tr("The upper limit of the motor associated to the second axis shouldn't be greater than pi/2."));
    }
  }
}

void OmBallJoint::addDevice2(int index) {
  OmHinge2Joint::addDevice2(index);
  const OmMotor *const motor = dynamic_cast<OmMotor *>(mDevice2->item(index));
  if (motor) {
    checkMotorLimit();
    connect(motor, &OmMotor::minPositionChanged, this, &OmBallJoint::checkMotorLimit, Qt::UniqueConnection);
    connect(motor, &OmMotor::maxPositionChanged, this, &OmBallJoint::checkMotorLimit, Qt::UniqueConnection);
  }
}

void OmBallJoint::addDevice3(int index) {
  const OmSolid *const s = upperSolid();
  if (s) {
    OmRobot *const r = s->robot();
    assert(r);
    OmBaseNode *decendant = dynamic_cast<OmBaseNode *>(mDevice3->item(index));
    r->descendantNodeInserted(decendant);
  }
  const OmBrake *const brake = dynamic_cast<OmBrake *>(mDevice3->item(index));
  if (brake)
    connect(brake, &OmBrake::brakingChanged, this, &OmBallJoint::updateSpringAndDampingConstants, Qt::UniqueConnection);
}

void OmBallJoint::updateParameters3() {
  const OmJointParameters *const p3 = parameters3();
  if (p3) {
    mOdePositionOffset3 = p3->position();
    mPosition3 = mOdePositionOffset3;
    connect(p3, &OmJointParameters::minAndMaxStopChanged, this, &OmBallJoint::updateMinAndMaxStop, Qt::UniqueConnection);
    connect(p3, &OmJointParameters::springAndDampingConstantsChanged, this, &OmBallJoint::updateSpringAndDampingConstants,
            Qt::UniqueConnection);
    connect(p3, &OmJointParameters::axisChanged, this, &OmBallJoint::updateAxis, Qt::UniqueConnection);
    connect(p3, SIGNAL(positionChanged()), this, SLOT(updatePosition()), Qt::UniqueConnection);
  }
}

bool OmBallJoint::setJoint() {
  if (!OmBasicJoint::setJoint())
    return false;

  setOdeJoint();

  return true;
}

double OmBallJoint::position(int index) const {
  switch (index) {
    case 1:
      return mPosition;
    case 2:
      return mPosition2;
    case 3:
      return mPosition3;
    default:
      return NAN;
  }
}

double OmBallJoint::initialPosition(int index) const {
  switch (index) {
    case 1:
      return mSavedPositions[stateId()];
    case 2:
      return mSavedPositions2[stateId()];
    case 3:
      return mSavedPositions3[stateId()];
    default:
      return NAN;
  }
}

void OmBallJoint::setPosition(double position, int index) {
  OmHinge2Joint::setPosition(position, index);

  if (index != 3)
    return;

  mPosition3 = position;
  mOdePositionOffset3 = position;
  OmJointParameters *const p3 = parameters3();
  if (p3)
    p3->setPosition(mPosition3);

  // Same reset-cascade writer as OmJoint::setPosition -- see the comment there.
  OmMotor *const m3 = motor3();
  if (m3 && OmMotor::resetMayOverwriteMotorCommand())
    m3->setTargetPosition(position);
}

bool OmBallJoint::resetJointPositions() {
  mOdePositionOffset3 = 0.0;
  return OmHinge2Joint::resetJointPositions();
}

void OmBallJoint::updateOdePositionOffset() {
  OmHinge2Joint::updateOdePositionOffset();
  mOdePositionOffset3 = position(3);
}

void OmBallJoint::preFinalize() {
  OmHinge2Joint::preFinalize();

  OmBallJointParameters *const p = ballJointParameters();
  if (p && !p->isPreFinalizedCalled())
    p->preFinalize();

  for (int i = 0; i < devices3Number(); ++i) {
    if (device3(i) && !device3(i)->isPreFinalizedCalled())
      device3(i)->preFinalize();
  }

  OmBaseNode *const p3 = dynamic_cast<OmBaseNode *>(mParameters3->value());
  if (p3 && !p3->isPreFinalizedCalled())
    p3->preFinalize();

  updateParameters3();
  checkMotorLimit();

  mSavedPositions3["__init__"] = mPosition3;
}

void OmBallJoint::postFinalize() {
  OmHinge2Joint::postFinalize();

  OmBallJointParameters *const p = ballJointParameters();

  if (p && !p->isPostFinalizedCalled())
    p->postFinalize();
  for (int i = 0; i < devices3Number(); ++i) {
    if (device3(i) && !device3(i)->isPostFinalizedCalled())
      device3(i)->postFinalize();
  }

  OmBaseNode *const p3 = dynamic_cast<OmBaseNode *>(mParameters3->value());
  if (p3 && !p3->isPostFinalizedCalled())
    p3->postFinalize();

  connect(mDevice3, &OmMFNode::itemChanged, this, &OmBallJoint::addDevice3);
  connect(mDevice3, &OmMFNode::itemInserted, this, &OmBallJoint::addDevice3);
  connect(mParameters3, &OmSFNode::changed, this, &OmBallJoint::updateParameters);
  if (p)
    connect(p, &OmBallJointParameters::anchorChanged, this, &OmBallJoint::updateAnchor, Qt::UniqueConnection);
  if (brake3())
    connect(brake3(), &OmBrake::brakingChanged, this, &OmBallJoint::updateSpringAndDampingConstants, Qt::UniqueConnection);
  if (motor2()) {
    connect(motor2(), &OmMotor::minPositionChanged, this, &OmBallJoint::checkMotorLimit, Qt::UniqueConnection);
    connect(motor2(), &OmMotor::maxPositionChanged, this, &OmBallJoint::checkMotorLimit, Qt::UniqueConnection);
  }
}

void OmBallJoint::prePhysicsStep(double ms) {
  assert(solidEndPoint());
  OmRotationalMotor *const rm = rotationalMotor();
  OmRotationalMotor *const rm2 = rotationalMotor2();
  OmRotationalMotor *const rm3 = rotationalMotor3();
  OmJointParameters *const p = parameters();
  OmJointParameters *const p2 = parameters2();
  OmJointParameters *const p3 = parameters3();

  if (isEnabled()) {
    // ⚠ MOTORISED BallJoint ACTUATION IS UNIMPLEMENTED. ODE realised it as an
    // AMotor triple (per-axis torque, or an FMax + Vel pair); all of that is
    // gone. The Newton d6 path (registerNewtonMultiDof / pushNewtonMultiDofTargets,
    // reached from OmBasicJoint's per-tick push) exists but is gated OFF by
    // default behind OMNISIM_NEWTON_BALL_HINGE2, because driving d6 POSITION
    // targets through SolverMuJoCo is an upstream newton defect, not ours
    // (verified 2026-08-08: our d6 reports model_ke/model_kd and mode=3
    // correctly). Do not "fix" it here.
    //
    // The computeCurrentDynamicVelocity calls below are KEPT FOR THEIR SIDE
    // EFFECT, not their return value: they advance each motor's PID state and
    // mCurrentVelocity, which the motor-sound manager and muscle visuals read.
    if (rm && !rm->userControl())
      rm->computeCurrentDynamicVelocity(ms, mPosition);
    if (rm2 && !rm2->userControl())
      rm2->computeCurrentDynamicVelocity(ms, mPosition2);
    if (rm3 && !rm3->userControl())
      rm3->computeCurrentDynamicVelocity(ms, mPosition3);
  } else {
    const bool run1 = rm && rm->runKinematicControl(ms, mPosition);
    if (run1 && p)
      p->setPosition(mPosition);

    const bool run2 = rm2 && rm2->runKinematicControl(ms, mPosition2);
    if (run2 && p2)
      p2->setPosition(mPosition2);

    const bool run3 = rm3 && rm3->runKinematicControl(ms, mPosition3);
    if (run3 && p3)
      p3->setPosition(mPosition3);

    if (run1 || run2 || run3)
      updatePositions(mPosition, mPosition2, mPosition3);
  }
  mTimeStep = ms;
}

//////////////////////////////////////////////////////////////////////////////
// Newton multi-DoF registration + drive (OMNISIM_NEWTON_BALL_HINGE2)       //
//////////////////////////////////////////////////////////////////////////////

OmMatrix3 OmBallJoint::newtonJointBasis() const {
  OmVector3 e1 = axis();
  OmVector3 e3 = axis3();
  if (e1.cross(e3).isNull()) {
    // Same substitution ODE's applyToOdeAxis makes (it warns there; warning here
    // too would double up on every registration, so this stays silent and the
    // ODE-side warning remains the single report).
    e1 = OmVector3(1.0, 0.0, 0.0);
    e3 = OmVector3(0.0, 0.0, 1.0);
  }
  e1.normalize();
  e3.normalize();
  // Gram-Schmidt e3 against e1 so the triad is exactly orthonormal even when the
  // author's two axes are merely non-parallel: a non-orthogonal basis quaternion
  // would shear the joint frame and the readback's Euler decomposition would not
  // be the inverse of computeEndPointSolidPositionFromParameters any more.
  e3 = (e3 - e1 * e1.dot(e3));
  if (e3.isNull())
    e3 = OmVector3(0.0, 0.0, 1.0);
  e3.normalize();
  const OmVector3 e2 = e3.cross(e1);  // matches axis2() == axis3().cross(axis())
  return OmMatrix3(e1, e2, e3);       // columns -> joint frame to parent frame
}

bool OmBallJoint::hasAnyNewtonMotor() const {
  return motor() != nullptr || motor2() != nullptr || motor3() != nullptr;
}

int OmBallJoint::registerNewtonMultiDof(OmNewtonBackend *newton, int parentIdx, int childIdx,
                                        const OmVector3 &parentAnchor, const OmVector3 &childAnchor,
                                        const OmQuaternion &childRelRot) {
  if (newton == nullptr)
    return -1;
  // JOINT FRAME = the authored axis triad. This is NOT cosmetic: newton's MuJoCo
  // conversion emits a spherical joint as ONE mjJNT_BALL element plus three
  // actuators whose gear vectors are the JOINT frame's x/y/z, so per-DoF axis
  // vectors are ignored and the only way to actuate about `axis` / `axis2` /
  // `axis3` is to make them that frame. The same rotation makes joint_q's
  // quaternion decompose (Rx*Ry*Rz) straight into position/position2/position3.
  const OmQuaternion basis = newtonJointBasis().toQuaternion();
  // child anchor frame must coincide with the parent anchor frame at the authored
  // pose, so it carries the child's authored relative rotation as well.
  const OmQuaternion childFrame = childRelRot * basis;

  double ke1, kd1, lo1, hi1, e1, v1;
  double ke2, kd2, lo2, hi2, e2, v2;
  double ke3, kd3, lo3, hi3, e3, v3;
  // positionByConstruction = true on all three axes: a BallJoint is a 3-DoF
  // wrist, never a free-spinning wheel, so a limitless motor here is a servo (the
  // same reasoning the slider path uses). Without this, motor / motor3 -- which
  // Webots does NOT auto-limit, unlike motor2 (see checkMotorLimit) -- would be
  // classified as velocity wheels and setPosition on them would do nothing.
  newtonAxisSpec(motor(), parameters(), true, &ke1, &kd1, &lo1, &hi1, &e1, &v1);
  newtonAxisSpec(motor2(), parameters2(), true, &ke2, &kd2, &lo2, &hi2, &e2, &v2);
  newtonAxisSpec(motor3(), parameters3(), true, &ke3, &kd3, &lo3, &hi3, &e3, &v3);

  QStringList unmapped = unmappedNewtonFields(parameters(), brake(), tr("axis 1")) +
                         unmappedNewtonFields(parameters2(), brake2(), tr("axis 2")) +
                         unmappedNewtonFields(parameters3(), brake3(), tr("axis 3"));
  // Per-axis position LIMITS are the one thing a Newton ball joint accepts but
  // does not enforce: the MuJoCo converter emits its ball element with
  // `limited: False`, and the post-step clamp cannot help either because a ball
  // joint's coordinates are a quaternion, not three angles. Say so rather than
  // let a hard stop silently disappear.
  if (lo1 != hi1 || lo2 != hi2 || lo3 != hi3)
    unmapped << tr("per-axis position limits (minStop/maxStop, min/maxPosition)");
  warnUnmappedNewtonFeatures(unmapped);

  const double pAnchor[3] = {parentAnchor.x(), parentAnchor.y(), parentAnchor.z()};
  const double cAnchor[3] = {childAnchor.x(), childAnchor.y(), childAnchor.z()};
  // xyzw across the FFI: warp/newton quaternions are xyzw, OmQuaternion is wxyz.
  const double pQuat[4] = {basis.x(), basis.y(), basis.z(), basis.w()};
  const double cQuat[4] = {childFrame.x(), childFrame.y(), childFrame.z(), childFrame.w()};
  const double gains[6] = {ke1, kd1, ke2, kd2, ke3, kd3};
  const double limits[6] = {lo1, hi1, lo2, hi2, lo3, hi3};
  const double efforts[3] = {e1, e2, e3};
  const double velLimits[3] = {v1, v2, v3};
  return newton->addJointBallMotorized(parentIdx, childIdx, pAnchor, pQuat, cAnchor, cQuat,
                                       gains, limits, efforts, velLimits);
}

void OmBallJoint::pushNewtonMultiDofTargets(OmNewtonBackend *newton) {
  if (newton == nullptr || mNewtonJointIndex < 0)
    return;
  OmMotor *const motors[3] = {motor(), motor2(), motor3()};
  for (int dof = 0; dof < 3; ++dof) {
    if (motors[dof] == nullptr)
      continue;
    pushNewtonAxisTarget(newton, mNewtonJointIndex, dof, motors[dof]);
  }
}

void OmBallJoint::postPhysicsStep() {
  // Newton-backed readback (OMNISIM_NEWTON_BALL_HINGE2). newton stores a
  // spherical joint's position as a QUATERNION (4 coordinates), not as three
  // angles, so decompose it into Webots' triple. The joint frame IS the authored
  // (axis, axis2, axis3) triad (see registerNewtonMultiDof), so in that frame the
  // relative rotation is exactly Rx(p1) * Ry(p2) * Rz(p3) -- the inverse of
  // computeEndPointSolidPositionFromParameters' q * q2 * q3 -- and the
  // decomposition is the standard XYZ one:
  //     p2 = asin(R02),  p1 = atan2(-R12, R22),  p3 = atan2(-R01, R00)
  // ⚠ It gimbal-locks at p2 = +/-pi/2, which is precisely why Webots clamps a
  // BallJoint's axis-2 motor to +/-pi/2 (checkMotorLimit) -- the same singularity
  // ODE's dAMotorEuler has, so this is a shared limitation, not a new one.
  if (newtonBallHinge2Enabled() && mNewtonJointIndex >= 0) {
    OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
    if (raw != nullptr && raw->isAvailable()) {
      OmNewtonBackend *const nb = static_cast<OmNewtonBackend *>(raw);
      double q[4] = {0.0, 0.0, 0.0, 1.0};  // xyzw
      if (nb->getJointBallQuat(mNewtonJointIndex, q) == 0) {
        const OmMatrix3 m(OmQuaternion(q[3], q[0], q[1], q[2]));
        double r02 = m(0, 2);
        if (r02 > 1.0)
          r02 = 1.0;
        else if (r02 < -1.0)
          r02 = -1.0;
        mPosition = std::atan2(-m(1, 2), m(2, 2));
        mPosition2 = std::asin(r02);
        mPosition3 = std::atan2(-m(0, 1), m(0, 0));
      }
      OmJointParameters *const np = parameters();
      if (np)
        np->setPositionFromOde(mPosition);
      OmJointParameters *const np2 = parameters2();
      if (np2)
        np2->setPositionFromOde(mPosition2);
      OmJointParameters *const np3 = parameters3();
      if (np3)
        np3->setPositionFromOde(mPosition3);
      return;
    }
  }

  // ⚠ BallJoint POSITION READBACK IS UNIMPLEMENTED off the (default-OFF) Newton
  // d6 path handled above. ODE read the three angles + rates back from the
  // control AMotor; that is gone, and against the inert stub it read zeros and
  // CLOBBERED all three positions with their mOdePositionOffset every step, so
  // dropping it is what preserves them. The three fields are still refreshed
  // from whatever mPosition* currently hold, so the scene tree stays consistent.
  OmJointParameters *const p = parameters();
  if (p)
    p->setPositionFromOde(mPosition);
  OmJointParameters *const p2 = parameters2();
  if (p2)
    p2->setPositionFromOde(mPosition2);
  OmJointParameters *const p3 = parameters3();
  if (p3)
    p3->setPositionFromOde(mPosition3);
}

void OmBallJoint::reset(const QString &id) {
  OmHinge2Joint::reset(id);

  for (int i = 0; i < mDevice3->size(); ++i)
    mDevice3->item(i)->reset(id);

  OmNode *const p = mParameters3->value();
  if (p)
    p->reset(id);

  setPosition(mSavedPositions3[id], 3);
}

void OmBallJoint::resetPhysics() {
  OmHinge2Joint::resetPhysics();

  OmMotor *const m = motor3();
  if (m)
    m->resetPhysics();
}

void OmBallJoint::save(const QString &id) {
  OmHinge2Joint::save(id);

  for (int i = 0; i < mDevice3->size(); ++i)
    mDevice3->item(i)->save(id);

  OmNode *const p = mParameters3->value();
  if (p)
    p->save(id);

  mSavedPositions3[id] = mPosition3;
}

void OmBallJoint::applyToOdeAxis() {
  OmVector3 referenceAxis = axis();
  OmVector3 referenceAxis3 = axis3();

  if (referenceAxis.cross(referenceAxis3).isNull()) {
    parsingWarn(tr("Axes are aligned: using x and z axes instead."));
    referenceAxis = OmVector3(1.0, 0.0, 0.0);
    referenceAxis3 = OmVector3(0.0, 0.0, 1.0);
  }

  (void)referenceAxis;
  (void)referenceAxis3;

  updateOdePositionOffset();
}

void OmBallJoint::applyToOdeMinAndMaxStop() {
  // ⚠ BallJoint per-axis HARD STOPS ARE UNIMPLEMENTED. ODE clamped them onto the
  // control AMotor's three axes; that is gone. Newton cannot take over even on
  // the d6 path: a spherical joint's coordinates are a quaternion, so the MuJoCo
  // converter emits the ball element `limited: False` -- registerNewtonMultiDof
  // already warns the user about exactly this (see warnUnmappedNewtonFeatures).
}

void OmBallJoint::writeExport(OmWriter &writer) const {
  if (writer.isUrdf() && solidEndPoint()) {
    this->warn("Exporting 'BallJoint' nodes to URDF is currently not supported");
    return;
  }
  OmBasicJoint::writeExport(writer);
}
