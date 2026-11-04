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

#include "OmBasicJoint.hpp"

#include "OmBoundingSphere.hpp"
#include "OmHinge2Joint.hpp"
#include "OmHingeJoint.hpp"
#include "OmJointParameters.hpp"
#include "OmLinearMotor.hpp"
#include "OmLog.hpp"
#include "OmMotor.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmQuaternion.hpp"
#include "OmRotationalMotor.hpp"
#include "OmSliderJoint.hpp"
#include "OmSlot.hpp"
#include "OmSolid.hpp"
#include "OmSolidMerger.hpp"
#include "OmSolidReference.hpp"

#include <QtCore/QPointer>
#include <QtCore/QSet>
#include <algorithm>
#include <cassert>
#include <cmath>

namespace {
  // P3.7: deferred-registration queue. Joints push themselves here from
  // postFinalize when they look like they should be Newton-backed; the
  // actual addJointRevolute call happens in flushPendingNewtonRegistrations
  // (driven by OmSimulationWorld::step right before finalizeWorld), by
  // which point the parent and child Solids' postFinalize blocks have run
  // and their mNewtonBodyIndex values are populated.
  QList<QPointer<OmBasicJoint>> &pendingNewtonJoints() {
    static QList<QPointer<OmBasicJoint>> list;
    return list;
  }

  // P3.8.b: registry of joints that successfully landed in the Newton
  // model AND have an associated motor. Walked each tick by
  // pushNewtonMotorTargets to push controller-side velocity commands
  // through to the solver.
  QList<QPointer<OmBasicJoint>> &registeredNewtonMotorizedJoints() {
    static QList<QPointer<OmBasicJoint>> list;
    return list;
  }

  // W1.7: these registries are keyed by NEWTON JOINT INDICES, which a mid-run
  // physics rebuild invalidates wholesale. File-scope (not function-local
  // statics) so requeueAllNewtonJointsForRebuild can clear them; stale indices
  // from the previous Newton world would otherwise misfire on reused slots.
  QSet<int> &forceModeJointIndices() {
    static QSet<int> s;
    return s;
  }
  QSet<int> &forceModeAxes() {
    static QSet<int> s;
    return s;
  }
  QSet<int> &loggedNonZeroJointIndices() {
    static QSet<int> s;
    return s;
  }

  // W1.4 servo promotion. limitlessNewtonJointIndices: 1-DoF joints the
  // classifier built as VELOCITY WHEELS (ke=0, kd=500) because they declare no
  // position limits -- the ONLY population promotion may touch.
  // promotedServoJointIndices: the subset currently holding servo gains
  // because the controller sent a finite setPosition over the wire. Both are
  // keyed by NEWTON JOINT INDICES -> cleared by the W1.7 rebuild requeue.
  QSet<int> &limitlessNewtonJointIndices() {
    static QSet<int> s;
    return s;
  }
  QSet<int> &promotedServoJointIndices() {
    static QSet<int> s;
    return s;
  }

  // A range at or beyond +-1e6 at BOTH ends is the ROS/Gazebo spelling of "no
  // limit" (`<limit lower="-1e+16" upper="1e+16">` on a type="revolute" wheel --
  // Neobotix ROX diff and MP-400), not a travel range. Classifying it as a
  // position servo (ke = effort*10 held at the motor's start angle) pinned those
  // wheels at 0.15 % of their commanded rate under setPosition(inf) +
  // setVelocity(). Same threshold and hatch as the runtime's
  // _normalise_unbounded_limits (omnisim_newton_runtime.py), which drops the
  // range on the solver side. OMNISIM_NEWTON_UNBOUNDED_LIMIT_AS_CONTINUOUS=0
  // reverts (value-parsed).
  bool isUnboundedLimitRange(double lower, double upper) {
    // OMNISIM_NEWTON_UNBOUNDED_LIMIT_AS_CONTINUOUS=0 keeps a +-1e16 hinge a position servo (pre-2026-10-03).
    static const bool on = []() {
      const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_UNBOUNDED_LIMIT_AS_CONTINUOUS")).trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return on && lower <= -1.0e6 && upper >= 1.0e6;
  }

  // setAvailableTorque/Force() reaching the solver (2026-10-03). Until then the
  // available torque only gated OmMotor's own bookkeeping: isPIDPositionControl()
  // went false and the push below sent a velocity target of 0 to a full-strength
  // servo -- a BRAKE, not a passive joint. Measured on Ekumen's Andino (a URDF
  // swivel caster, imported with a motor like every revolute/continuous joint):
  // identical to 9 significant figures with and without setAvailableTorque(0).
  // OMNISIM_NEWTON_AVAILABLE_TORQUE=0 restores that (value-parsed).
  bool newtonAvailableTorqueEnabled() {
    static const bool on = []() {
      const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_AVAILABLE_TORQUE")).trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return on;
  }

  // Last available torque pushed per Newton joint index; absent = never pushed
  // (the joint still carries the effort it was registered with).
  QHash<int, double> &pushedAvailableTorque() {
    static QHash<int, double> sPushed;
    return sPushed;
  }

  // OMNISIM_NEWTON_PROMOTE_SERVO -- value-parsed, default ON. "0/false/off/no"
  // restores the pre-2026-09-01 behaviour (setPosition() on a limit-less
  // motor is ignored for ever).
  bool newtonPromoteServoEnabled() {
    static const bool on = []() {
      const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_PROMOTE_SERVO")).trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    return on;
  }

  // Internal parity plan, item W1.4. A motor whose joint declares no position limits
  // is configured as a VELOCITY WHEEL (ke=0, kd=500), so setPosition() on it
  // does nothing. That is a defensible default -- husky/jackal/rover wheels are
  // genuine limit-less motors and must not get a position spring -- but for a
  // limit-less SERVO it is a silent wrong answer, and the most common robotics
  // primitive there is.
  //
  // ⚠ THE WARNING USED TO FIRE ONCE PER PROCESS, VIA TWO `static bool` GUARDS
  // (one here for 1-DoF, one in newtonAxisSpec for hinge2/ball axes). On any
  // world with more than one such motor -- i.e. essentially every wheeled robot
  // world -- the FIRST wheel consumed the warning and every genuinely-affected
  // servo after it was configured silently. Our own OmniBench author hit this
  // and published a `broken` verdict against the engine. Per DEVICE now, keyed
  // on the motor node's uniqueId (exact, and re-warns after a world reload
  // because reload builds fresh nodes) rather than on the device NAME, which is
  // not unique across two identical robots.
  //
  // The GUI rebuilds a world several times per load and each rebuild re-runs
  // registration on the SAME nodes, so process lifetime is the right scope for
  // the set: same id, warned once.
  //
  // This converts a silent wrong answer into a named one. It does NOT fix the
  // classification -- see the comment at the 1-DoF call site for why the full
  // fix is a separate change.
  bool shouldWarnLimitlessServoOnce(const OmMotor *motor) {
    if (motor == nullptr)
      return false;
    static QSet<int> warnedMotorIds;
    const int id = motor->uniqueId();
    if (warnedMotorIds.contains(id))
      return false;
    warnedMotorIds.insert(id);
    return true;
  }
}  // namespace

// OMNISIM_NEWTON_BALL_HINGE2 (value-parsed, DEFAULT ON since 2026-08-17) -- the
// motorised + angle-readback layer for BallJoint / Hinge2Joint. Value-parsed
// exactly like OmSolid::newtonKinematicNativeEnabled: "0"/"false"/"off"/"no" is
// OFF and anything else is ON, so "=0" means off rather than "the name is
// present, therefore on". Read once per process.
bool OmBasicJoint::newtonBallHinge2Enabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_BALL_HINGE2")).trimmed().toLower();
    // WHY THE DEFAULT FLIPPED (internal parity plan, item W1.3).
    //
    // This gate was DEFAULT OFF because the motorised path registered and
    // enrolled correctly but did not ACTUATE: axis 1 read 0.0000 rad when
    // commanded 0.4. Everything engine-side was verified wired
    // (registerNewtonMultiDof under the gate, enrollment in
    // registeredNewtonMotorizedJoints, the per-tick per-axis target push, the
    // (slot, dof) target keys, and OMNISIM_NEWTON_JOINTDIAG showing the d6
    // built with mode=3 POSITION_VELOCITY and per-axis gains), so the defect
    // was attributed BELOW us -- to newton's d6 -> MuJoCo actuator mapping for
    // multi-DoF position control. That attribution was correct, and it was
    // made against the then-vendored newton 1.2.0.
    //
    // The runtime was upgraded to newton 1.5.0 (b56be84a0, 2026-08-09) and the
    // upstream mapping works. Re-measured on the current binary: the hinge2 arm
    // of tests/test_newton_ball_hinge2.py XPASSes -- both axes reach their
    // commanded angles inside 0.05 rad, axis 1 does not drift when only axis 2
    // is re-commanded, and axis 2 travels the full 0.6 rad between its two
    // commands. So the reason for the OFF default no longer exists.
    //
    // ⚠ REMAINING CAVEAT, and it is real: the BALL element is built with
    // `limited: False`, so a BallJointParameters' per-axis minStop/maxStop are
    // NOT enforced by the solver. A ball joint still swings freely past its
    // authored stops. Hinge2 axes ARE limited (the d6 carries per-axis limits;
    // JOINTDIAG reads limits=(-1.4,1.4,-1.4,1.4)).
    //
    // "=0" is the exact-revert hatch back to the PASSIVE constraint that
    // shipped for months.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

void OmBasicJoint::newtonAxisSpec(const OmMotor *motor, const OmJointParameters *params,
                                  bool positionByConstruction, double *ke, double *kd,
                                  double *lower, double *upper, double *effort, double *velLimit) {
  *ke = 0.0;
  *kd = 0.0;
  *effort = 0.0;
  *velLimit = 0.0;
  *lower = 0.0;
  *upper = 0.0;
  if (motor != nullptr) {
    *effort = motor->maxForceOrTorque();
    *velLimit = motor->maxVelocity();
    const double minP = motor->minPosition();
    const double maxP = motor->maxPosition();
    if (minP != maxP) {
      *lower = minP;
      *upper = maxP;
    }
  }
  if (*lower == *upper && params != nullptr) {
    // URDF <limit lower/upper> and hand-authored mechanical stops arrive as the
    // joint's minStop/maxStop, not as the motor's control range (see the revolute
    // path's note: every Newton joint used to read lim=[0,0] because of this).
    const double mn = params->minStop();
    const double mx = params->maxStop();
    if (mn != mx) {
      *lower = mn;
      *upper = mx;
    }
  }
  if (motor == nullptr)
    return;  // passive axis: constrained, never driven
  if (positionByConstruction || *lower != *upper) {
    *ke = (*effort > 0.0) ? *effort * 10.0 : 20.0;
    *kd = (*effort > 0.0) ? *effort * 0.5 : 3.0;
    return;
  }
  *ke = 0.0;
  *kd = 500.0;
  if (shouldWarnLimitlessServoOnce(motor))
    OmLog::warning(QObject::tr(
                     "Joint motor '%1' declares no position limits, so this physics backend configures it as a "
                     "VELOCITY wheel: setPosition() on it will be IGNORED and setVelocity() is what drives it. "
                     "If it is meant to be a servo, give its motor a minPosition/maxPosition (or its joint a "
                     "minStop/maxStop) and it will be position-controlled.")
                     .arg(motor->deviceName()),
                   false, OmLog::ODE);
}

void OmBasicJoint::pushNewtonAxisTarget(OmNewtonBackend *newton, int jointIdx, int dof, OmMotor *motor) {
  if (newton == nullptr || motor == nullptr || jointIdx < 0 || dof < 0)
    return;
  // Which (joint, DoF) pairs currently carry a controller-set force. Packed as
  // jointIdx * 8 + dof: no joint here has more than 3 DoF, so 8 leaves headroom
  // and keeps this a plain int set (the 1-DoF registry next door is keyed by the
  // bare joint index and is a separate set, so the two never collide).
  QSet<int> &sForceModeAxes = forceModeAxes();
  const int axisKey = jointIdx * 8 + dof;
  if (motor->userControl()) {
    newton->setJointForceDof(jointIdx, dof, motor->rawInput());
    sForceModeAxes.insert(axisKey);
    return;
  }
  if (sForceModeAxes.remove(axisKey))
    newton->setJointForceDof(jointIdx, dof, 0.0);
  if (motor->availableForceOrTorque() == 0.0) {  // motor off: never maxVelocity (see the 1-DoF push)
    newton->setJointTargetVelocityDof(jointIdx, dof, 0.0);
    return;
  }
  if (motor->isPIDPositionControl()) {
    newton->setJointTargetPositionDof(jointIdx, dof, motor->targetPosition());
    newton->setJointTargetVelocityDof(jointIdx, dof, 0.0);
    return;
  }
  newton->setJointTargetVelocityDof(jointIdx, dof, motor->targetVelocity());
}

int OmBasicJoint::registerNewtonMultiDof(OmNewtonBackend *, int, int, const OmVector3 &,
                                         const OmVector3 &, const OmQuaternion &) {
  // Base: this joint type has no multi-DoF Newton registration. Only
  // OmHinge2Joint and OmBallJoint override it.
  return -1;
}

void OmBasicJoint::init() {
  mIsEndPointPositionChangedByJoint = false;
  mNewtonJointIndex = -1;

  mParameters = findSFNode("jointParameters");
  mEndPoint = findSFNode("endPoint");
}
// Constructors

OmBasicJoint::OmBasicJoint(const QString &modelName, OmTokenizer *tokenizer) : OmBaseNode(modelName, tokenizer) {
  init();
}

OmBasicJoint::OmBasicJoint(const OmBasicJoint &other) : OmBaseNode(other) {
  init();
}

OmBasicJoint::OmBasicJoint(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmBasicJoint::~OmBasicJoint() {
  OmSolid *const s = solidEndPoint();
  if (s && !s->isBeingDeleted())
    s->removeJointParent(this);


}

void OmBasicJoint::downloadAssets() {
  OmBaseNode *const e = dynamic_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->downloadAssets();
}

void OmBasicJoint::preFinalize() {
  OmBaseNode::preFinalize();

  // set endPoint initial position
  updateParameters();
  updateEndPointZeroTranslationAndRotation();

  OmBaseNode *const p = dynamic_cast<OmBaseNode *>(mParameters->value());
  OmBaseNode *const e = dynamic_cast<OmBaseNode *>(mEndPoint->value());
  if (p && !p->isPreFinalizedCalled())
    p->preFinalize();
  if (e && !e->isPreFinalizedCalled())
    e->preFinalize();
}

void OmBasicJoint::setMatrixNeedUpdate() {
  OmSolid *const s = solidEndPoint();
  if (s && solidReference() == NULL)
    s->setMatrixNeedUpdate();
}

void OmBasicJoint::postFinalize() {
  OmBaseNode::postFinalize();

  OmBaseNode *const p = dynamic_cast<OmBaseNode *>(mParameters->value());
  OmBaseNode *const e = dynamic_cast<OmBaseNode *>(mEndPoint->value());
  if (p && !p->isPostFinalizedCalled())
    p->postFinalize();
  if (e && !e->isPostFinalizedCalled())
    e->postFinalize();

  connect(mEndPoint, &OmSFNode::changed, this, &OmBasicJoint::updateEndPoint);
  const OmGroup *pg = dynamic_cast<OmGroup *>(parentNode());
  if (pg)
    connect(this, &OmBasicJoint::endPointChanged, pg, &OmGroup::insertChildFromSlotOrJoint);
  else {
    const OmSlot *slot = dynamic_cast<OmSlot *>(parentNode());
    if (slot)
      connect(this, &OmBasicJoint::endPointChanged, slot, &OmSlot::endPointInserted);
  }
  connect(mParameters, &OmSFNode::changed, this, &OmBasicJoint::updateParameters);

  OmSolid *const s = solidEndPoint();
  if (s) {
    connect(s, &OmSolid::positionChangedArtificially, this, &OmBasicJoint::updateEndPointPosition);
    updateEndPointPosition();
  }

  // P3.7 + P3.10 (cuda-newton-physics-plan.md): if this is a hinge
  // joint between two Solids that both effectively opt into Newton
  // (own field set, OR inherited from any ancestor Solid), queue self
  // for deferred registration. effectivePhysicsBackendName() lets a
  // single `physicsBackend "newton"` on a URDFRobot's outer Robot
  // propagate to all generated child Solids -- the husky URDF expands
  // into chassis + 4 wheel Solids whose own fields are default-"ode",
  // so we'd miss them if we only checked the local field.
  // OMNISIM_NEWTON_KINEMATIC: a joint whose endPoint has NO Physics node is a
  // KINEMATIC joint -- the ENGINE animates it (OmMotor::runKinematicControl ->
  // updatePosition writes the endpoint's fields; there is no dJoint under ODE
  // either) and the endpoint registers as a Newton KINEMATIC (mocap) body the
  // engine drives via setKinematicPose. Registering a Newton joint on that
  // world-welded body would both fight the mocap pin and (without the flag's
  // endpoint registration) trip enforceNewtonJointEndpoints' FATAL, so the
  // joint is simply never queued. Flag OFF = unchanged: such articulations
  // are gated to ODE upstream and never resolve to Newton anyway.
  // (The full eligibility predicate, endpoint checks included, lives in
  // wantsNewtonRegistration so the W1.7 rebuild requeue shares it.)
  if (wantsNewtonRegistration()) {
    pendingNewtonJoints().append(QPointer<OmBasicJoint>(this));
    // P5 hang fix 2026-05-28: the per-joint OmLog::info that the
    // concurrent G1 session added accumulates per-message cost during
    // world load. At 40+ huskies (160+ joints) the cost grows until the
    // load stalls. Suppressed here; the queue size is visible via the
    // "draining N entries" message in flushPendingNewtonRegistrations,
    // which is the right place for that breadcrumb.
  }

  if (protoParameterNode()) {
    const QVector<OmNode *> nodes = protoParameterNode()->protoParameterNodeInstances();
    if (nodes.size() > 1 && nodes.at(0) == this)
      parsingWarn(tr("Joint node defined in PROTO field is used multiple times. "
                     "OmniSim doesn't fully support this because the multiple node instances cannot be identical."));
  }

}

void OmBasicJoint::createOdeObjects() {
  OmBaseNode::createOdeObjects();
  OmSolid *const s = solidEndPoint();
  if (s && solidReference() == NULL)
    s->createOdeObjects();
}

// Newton enforcement (2026-06-29 default: no silent drop-out). A joint whose articulation resolved to
// Newton but whose endpoint body never registered a Newton body is "silently inert" -- the joint drops
// out and that limb is simulated by nothing while the rest of the robot is on Newton. Under enforcement
// that is a hard error. Fires ONLY when the MISSING endpoint actually wanted Newton (the §4.1 capability
// gate did not keep its articulation off Newton -- that case is named by OmSolid's own sweep). Body
// registration runs before joint registration (OmSimulationWorld), so a negative index here is a genuine
// miss, not a transient ordering artifact.
static void enforceNewtonJointEndpoints(const OmSolid *parent, const OmSolid *child,
                                        int parentIdx, int childIdx) {
  if (!OmPhysicsBackendRegistry::newtonEnforced())
    return;
  const bool parentMissing = parentIdx < 0 && parent != nullptr && !parent->gatedAwayFromNewton();
  const bool childMissing = childIdx < 0 && child != nullptr && !child->gatedAwayFromNewton();
  if (!parentMissing && !childMissing)
    return;
  const OmSolid *const missing = parentMissing ? parent : child;
  OmLog::fatal(
      QString("[newton-enforce] A joint's %1 body '%2' resolved to Newton but never registered a "
              "Newton body, so the joint would be silently inert -- and Newton is the only backend, so "
              "that part of the articulation would run with no physics at all. Fix the model so the "
              "body registers with Newton.")
          .arg(parentMissing ? QStringLiteral("parent") : QStringLiteral("child"))
          .arg(missing != nullptr ? missing->name() : QStringLiteral("?")));
}

bool OmBasicJoint::wantsNewtonRegistration() const {
  // The exact predicate postFinalize used inline before W1.7 (see the
  // comment there): endpoint + parent Solids present, not a kinematic
  // (no-Physics) endpoint under native-kinematic mode, a Hinge/Slider
  // family joint (the OmHingeJoint cast deliberately captures Hinge2 and
  // Ball, which inherit from it), and neither side kept off Newton by the
  // capability gate.
  OmSolid *const s = solidEndPoint();
  OmSolid *const parent = solidParent();
  const bool kinematicEndpoint =
      s != nullptr && s->physics() == nullptr && OmSolid::newtonKinematicNativeEnabled();
  return parent != nullptr && s != nullptr && !kinematicEndpoint &&
         (dynamic_cast<const OmHingeJoint *>(this) != nullptr ||
          dynamic_cast<const OmSliderJoint *>(this) != nullptr) &&
         !parent->gatedAwayFromNewton() && !s->gatedAwayFromNewton();
}

void OmBasicJoint::requeueAllNewtonJointsForRebuild() {
  pendingNewtonJoints().clear();
  registeredNewtonMotorizedJoints().clear();
  forceModeJointIndices().clear();
  forceModeAxes().clear();
  loggedNonZeroJointIndices().clear();
  limitlessNewtonJointIndices().clear();
  promotedServoJointIndices().clear();
  pushedAvailableTorque().clear();  // the rebuilt joints carry their registered effort again
  for (const OmSolid *solid : OmSolid::solids()) {
    for (OmBasicJoint *const j : solid->jointChildren()) {
      if (j == nullptr)
        continue;
      j->mNewtonJointIndex = -1;
      if (j->wantsNewtonRegistration())
        pendingNewtonJoints().append(QPointer<OmBasicJoint>(j));
    }
  }
}

// Spawn-at-position (2026-09-08). A 1-DoF joint authored with a non-zero
// `position` (a URDF <rest> crouch, a midpoint-seeded Panda joint, a
// hand-posed alife limb) used to start the first physics step at q=0: the
// frame was registered at the child's CURRENT pose, newton's coordinate began
// at 0, and only the motor TARGET carried the authored value -- so the servos
// yanked the joint there from wherever the scene had it (the Deep Robotics
// Lite3 read hip-pitch -0.015 / knee 0.524 at t=8 ms with targets -1.0 / 1.8
// and drove its feet through the floor). Now the frame is defined at the
// child's ZERO pose -- Webots' own bookkeeping (zeroEndPointTranslation /
// zeroEndPointRotation) already computes it from the authored pose and
// `position` -- and the authored `position` seeds newton's joint coordinate,
// so the first forward kinematics lands exactly on the authored pose. Limits
// and the position readback are absolute, as before. Value-parsed hatch:
// OMNISIM_NEWTON_SPAWN_AT_POSITION=0 restores the old registration.
static bool newtonSpawnAtPositionEnabled() {
  static int cached = -1;
  if (cached < 0) {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_SPAWN_AT_POSITION")).trimmed().toLower();
    cached = (v == "0" || v == "false" || v == "off" || v == "no") ? 0 : 1;
  }
  return cached == 1;
}

void OmBasicJoint::flushPendingNewtonRegistrations() {
  // Drain the queue regardless of whether registration succeeds, so
  // we don't repeatedly re-attempt failed entries on subsequent ticks.
  // (Failures are unrecoverable at this level: missing body index, no
  // backend, etc. The joint is then silently inert -- and since ODE's
  // removal there is no second backend to carry it, which is why
  // enforceNewtonJointEndpoints above turns that into a hard error.)
  QList<QPointer<OmBasicJoint>> queue;
  std::swap(queue, pendingNewtonJoints());

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);
  QList<QPointer<OmBasicJoint>> mimicFollowers;  // resolved after the loop

  for (const QPointer<OmBasicJoint> &p : queue) {
    if (p.isNull())
      continue;
    // Ball / spherical joint (newton-ode-replacement-plan.md W2.2): MUST be handled before BOTH the Hinge2
    // and the OmHingeJoint casts below -- OmBallJoint is-a OmHinge2Joint is-a OmHingeJoint, so either cast
    // would wrongly capture it and register only 1-2 of its 3 rotational DoF. nodeType() is the exact type
    // (the same discriminator the capability gate uses). Self-contained (mirrors the hinge/slider anchor
    // convention): resolve the two body indices + the shared anchor, then register a native 3-DoF ball
    // joint. PASSIVE by default; OMNISIM_NEWTON_BALL_HINGE2=1 registers it MOTORISED instead (per-axis
    // actuation about the authored axis triad + Newton-side angle readback) via the subclass hook. Ball
    // target pos/vel control needs SolverMuJoCo, which is the default solver since 2026-08-07.
    if (p->nodeType() == WB_NODE_BALL_JOINT) {
      const OmSolid *const parent = p->solidParent();
      const OmSolid *const child = p->solidEndPoint();
      if (parent == nullptr || child == nullptr)
        continue;
      // Merger-aware, and walks up through Physics-less frames (a URDF link with no
      // <inertial> on a fixed joint) to the body that carries them -- see
      // OmSolid::newtonJointParentBodySolid().
      const OmSolid *const parentBody = parent->newtonJointParentBodySolid();
      const int parentIdx = parentBody != nullptr ? parentBody->newtonBodyIndex() : -1;
      const int childIdx = child->effectiveNewtonBodyIndex();
      if (parentIdx < 0 || childIdx < 0) {
        enforceNewtonJointEndpoints(parent, child, parentIdx, childIdx);
        continue;
      }
      const OmVector3 anchor = p->anchor();
      const OmVector3 jointWorld = parent->matrix().translation() + parent->rotationMatrix() * anchor;
      const OmVector3 childAnchor = (jointWorld - child->matrix().translation()) * child->rotationMatrix();
      OmVector3 parentAnchor = anchor;  // re-express in the merger LEADER's frame if the parent merged away
      OmMatrix3 jointParentRot = parent->rotationMatrix();
      if (parentBody != parent) {
        const OmSolid *const leader = parentBody;
        parentAnchor = (jointWorld - leader->matrix().translation()) * leader->rotationMatrix();
        jointParentRot = leader->rotationMatrix();
      }
      int idx;
      if (newtonBallHinge2Enabled()) {
        // MOTORISED ball joint: the subclass owns the axis triad + the three
        // RotationalMotors, so it builds the spec (OmBallJoint::registerNewtonMultiDof).
        // No silent fall-back to the passive constraint on failure -- a motor the
        // controller can command but that never moves is the failure mode this whole
        // flag exists to remove, so a -1 stays a -1 and is reported below.
        const OmQuaternion childRelRot = (child->rotationMatrix().transposed() * jointParentRot).toQuaternion();
        idx = p->registerNewtonMultiDof(newton, parentIdx, childIdx, parentAnchor, childAnchor, childRelRot);
      } else {
        idx = newton->addJointBall(parentIdx, childIdx,
                                   parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
                                   childAnchor.x(), childAnchor.y(), childAnchor.z());
      }
      p->mNewtonJointIndex = idx;
      if (idx >= 0) {
        OmLog::info(QString("[OmNewtonBackend] ball joint %1 (parent=body %2, child=body %3) anchor=(%4,%5,%6)%7")
                        .arg(idx).arg(parentIdx).arg(childIdx)
                        .arg(parentAnchor.x()).arg(parentAnchor.y()).arg(parentAnchor.z())
                        .arg(p->hasAnyNewtonMotor() ? QStringLiteral(" [motorized]") :
                                                      QStringLiteral(" [passive]")));
        if (newtonBallHinge2Enabled() && p->hasAnyNewtonMotor())
          registeredNewtonMotorizedJoints().append(p);
      } else if (newtonBallHinge2Enabled()) {
        OmLog::warning(QObject::tr("BallJoint '%1' could not be registered with the Newton physics backend, so "
                                   "its motors will not move it and its position sensors will read 0. This is a "
                                   "backend bug, not a model error -- re-run without OMNISIM_NEWTON_BALL_HINGE2 "
                                   "for the passive (unmotorised) constraint. There is no ODE path left to fall "
                                   "back to: OMNISIM_FORCE_ODE is retired.")
                         .arg(p->endPointName()),
                       false, OmLog::ODE);
      }
      continue;
    }
    // Hinge2 / universal (newton-ode-replacement-plan.md W2): MUST be handled before the OmHingeJoint cast
    // below -- OmHinge2Joint is-a OmHingeJoint, so that cast would wrongly accept it and register only
    // axis1. Self-contained (mirrors the hinge/slider anchor convention): resolve the two body indices +
    // the shared anchor, then register a native 2-DoF d6 joint. PASSIVE by default (the cart casters are
    // free-spinning); OMNISIM_NEWTON_BALL_HINGE2=1 registers it MOTORISED instead, each axis taking its
    // own motor, limits and ceilings, via the subclass hook.
    if (OmHinge2Joint *const h2 = dynamic_cast<OmHinge2Joint *>(p.data())) {
      const OmSolid *const parent = h2->solidParent();
      const OmSolid *const child = h2->solidEndPoint();
      if (parent == nullptr || child == nullptr)
        continue;
      // Merger-aware, and walks up through Physics-less frames (a URDF link with no
      // <inertial> on a fixed joint) to the body that carries them -- see
      // OmSolid::newtonJointParentBodySolid().
      const OmSolid *const parentBody = parent->newtonJointParentBodySolid();
      const int parentIdx = parentBody != nullptr ? parentBody->newtonBodyIndex() : -1;
      const int childIdx = child->effectiveNewtonBodyIndex();
      if (parentIdx < 0 || childIdx < 0) {
        enforceNewtonJointEndpoints(parent, child, parentIdx, childIdx);
        continue;
      }
      const OmVector3 anchor = p->anchor();
      const OmVector3 jointWorld = parent->matrix().translation() + parent->rotationMatrix() * anchor;
      const OmVector3 childAnchor = (jointWorld - child->matrix().translation()) * child->rotationMatrix();
      OmVector3 parentAnchor = anchor;  // re-express in the merger LEADER's frame if the parent merged away
      OmMatrix3 jointParentRot = parent->rotationMatrix();
      if (parentBody != parent) {
        const OmSolid *const leader = parentBody;
        parentAnchor = (jointWorld - leader->matrix().translation()) * leader->rotationMatrix();
        jointParentRot = leader->rotationMatrix();
      }
      const OmVector3 a1 = h2->axis();
      // axis2() is protected; derive it from the public parameters2() exactly as the protected accessor
      // does (parameters2()->axis(), defaulting to +Z when the second parameters node is absent).
      const OmVector3 a2 = h2->parameters2() ? h2->parameters2()->axis() : OmVector3(0.0, 0.0, 1.0);
      int idx;
      if (newtonBallHinge2Enabled()) {
        // MOTORISED universal joint: axis1 and axis2 each take their own motor,
        // limits and ceilings. Built by OmHinge2Joint::registerNewtonMultiDof (it
        // owns motor/motor2 + parameters/parameters2). Same no-silent-fall-back
        // rule as the ball branch above.
        const OmQuaternion childRelRot = (child->rotationMatrix().transposed() * jointParentRot).toQuaternion();
        idx = p->registerNewtonMultiDof(newton, parentIdx, childIdx, parentAnchor, childAnchor, childRelRot);
      } else {
        idx = newton->addJointHinge2(parentIdx, childIdx, a1.x(), a1.y(), a1.z(), a2.x(), a2.y(),
                                     a2.z(), parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
                                     childAnchor.x(), childAnchor.y(), childAnchor.z());
      }
      p->mNewtonJointIndex = idx;
      if (idx >= 0) {
        OmLog::info(QString("[OmNewtonBackend] hinge2 joint %1 (parent=body %2, child=body %3) "
                            "axis1=(%4,%5,%6) axis2=(%7,%8,%9)%10")
                        .arg(idx).arg(parentIdx).arg(childIdx)
                        .arg(a1.x()).arg(a1.y()).arg(a1.z()).arg(a2.x()).arg(a2.y()).arg(a2.z())
                        .arg(p->hasAnyNewtonMotor() ? QStringLiteral(" [motorized]") :
                                                      QStringLiteral(" [passive]")));
        if (newtonBallHinge2Enabled() && p->hasAnyNewtonMotor())
          registeredNewtonMotorizedJoints().append(p);
      } else if (newtonBallHinge2Enabled()) {
        OmLog::warning(QObject::tr("Hinge2Joint '%1' could not be registered with the Newton physics backend, so "
                                   "its motors will not move it and its position sensors will read 0. This is a "
                                   "backend bug, not a model error -- re-run without OMNISIM_NEWTON_BALL_HINGE2 "
                                   "for the passive (unmotorised) constraint. There is no ODE path left to fall "
                                   "back to: OMNISIM_FORCE_ODE is retired.")
                         .arg(p->endPointName()),
                       false, OmLog::ODE);
      }
      continue;
    }
    OmHingeJoint *const hinge = dynamic_cast<OmHingeJoint *>(p.data());
    OmSliderJoint *const slider =
        (hinge == nullptr) ? dynamic_cast<OmSliderJoint *>(p.data()) : nullptr;
    // Newton handles revolute (hinge) and prismatic (slider) joints. The
    // parallel-gripper fingers ride on sliders; without this branch their
    // bodies dropped out of the articulation as orphans and never closed.
    if (hinge == nullptr && slider == nullptr)
      continue;
    const OmSolid *const parent = hinge ? hinge->solidParent() : slider->solidParent();
    const OmSolid *const child = hinge ? hinge->solidEndPoint() : slider->solidEndPoint();
    if (parent == nullptr || child == nullptr)
      continue;
    // P3.10c: walk to merger leader. URDF fixed-joint children share an
    // ODE body via OmSolidMerger; the matching Newton body lives on the
    // leader. A wheel hinge whose parent Solid is e.g. `top_chassis_link`
    // (a non-leader fixed-merger participant) needs to attach to the
    // chassis leader's Newton body, not to a non-existent body of the
    // merged-away child.
    // Merger-aware, and walks up through Physics-less frames (a URDF link with no
    // <inertial> on a fixed joint) to the body that carries them -- see
    // OmSolid::newtonJointParentBodySolid().
    const OmSolid *const parentBody = parent->newtonJointParentBodySolid();
    const int parentIdx = parentBody != nullptr ? parentBody->newtonBodyIndex() : -1;
    const int childIdx = child->effectiveNewtonBodyIndex();
    if (parentIdx < 0 || childIdx < 0) {
      enforceNewtonJointEndpoints(parent, child, parentIdx, childIdx);
      continue;
    }

    // Authored in the parent SOLID's frame; re-expressed below when that Solid
    // is merged into a leader body with a different orientation.
    OmVector3 axisLocal = hinge ? hinge->axis() : slider->axis();
    // Webots' anchor is in the parent's LOCAL frame; Newton's
    // parent_xform takes that directly. For child_xform (joint
    // frame in the child's LOCAL space), we project:
    //
    //   joint_world  = parent_world + R_parent * parent_anchor
    //   child_anchor = R_child^T * (joint_world - child_world)
    //
    // `v * M` on OmVector3 returns M^T * v (see OmMatrix3.hpp), so the
    // final line below is the R_child^T multiplication without an
    // explicit transpose.
    const OmVector3 anchor = p->anchor();  // virtual; public on OmBasicJoint
    const OmVector3 parentWorld = parent->matrix().translation();
    const OmMatrix3 parentRot = parent->rotationMatrix();
    // The child pose the joint frame is defined at: its ZERO pose when the
    // joint is authored at a non-zero `position` (see newtonSpawnAtPositionEnabled
    // above), else its current pose. The zero pose is the current world pose
    // with the endPoint's own local pose swapped for the zero one, which keeps
    // any Pose/Group nodes between the parent Solid and the joint intact.
    OmVector3 childWorld = child->matrix().translation();
    OmMatrix3 childRot = child->rotationMatrix();
    double initialPosition = 0.0;
    {
      const OmJointParameters *const jp = hinge ? hinge->parameters() : slider->parameters();
      const double authored = (jp != nullptr) ? jp->position() : 0.0;
      if (authored != 0.0 && newtonSpawnAtPositionEnabled()) {
        const OmMatrix3 ancestorRot = childRot * child->rotation().toMatrix3().transposed();
        const OmVector3 ancestorWorld = childWorld - ancestorRot * child->translation();
        childRot = ancestorRot * p->zeroEndPointRotation().toMatrix3();
        childWorld = ancestorWorld + ancestorRot * p->zeroEndPointTranslation();
        initialPosition = authored;
      }
    }
    const OmVector3 jointWorld = parentWorld + parentRot * anchor;
    const OmVector3 childAnchor = (jointWorld - childWorld) * childRot;

    // P3.10c follow-up: `anchor` (p->anchor()) is expressed in the PARENT
    // SOLID's local frame, but the joint attaches to `parentIdx`, which is
    // the merger LEADER when `parent` is a fixed-jointed participant merged
    // away (URDF fixed joints share the leader's Newton body). If we pass the
    // raw parent-frame anchor, the leader->participant offset is silently
    // dropped. Husky's fixed-merged links sit at ZERO offset from their
    // chassis leader, so this was invisible -- but the Robotiq 2F-85 mounts
    // on link6 via a fixed joint with xyz="0 0 0.1655", so the gripper
    // fingers (sliders hanging off the merged-away gripper base) were
    // registered 0.1655 m too close to the wrist. The arm then reaches its
    // IK pose perfectly yet the gripper stops ~0.16 m above the cube (XPBD's
    // sloppy tracking masked it; SolverMuJoCo tracks exactly and exposed it).
    // Re-express the parent anchor in the leader's frame, mirroring the
    // world-space childAnchor computation above. A non-merged parent (its own
    // Newton body) leaves `parentAnchor == anchor`, so every existing joint is
    // byte-unchanged; only fixed-merged parents with a non-zero mount offset
    // shift, which is exactly the correction needed.
    OmVector3 parentAnchor = anchor;
    OmMatrix3 jointParentRot = parentRot;
    if (parentBody != parent) {
      const OmSolid *const leader = parentBody;
      const OmVector3 leaderWorld = leader->matrix().translation();
      const OmMatrix3 leaderRot = leader->rotationMatrix();
      parentAnchor = (jointWorld - leaderWorld) * leaderRot;
      // The AXIS needs the same re-expression as the anchor (2026-09-27).
      // Newton reads it in the joint frame, whose parent side is the leader
      // body's frame (parent_xform carries no rotation), while `axis` is
      // authored in the merged-away parent Solid's frame. Passing it raw rotated
      // every joint whose parent sits below a ROTATED fixed frame about the wrong
      // axis: an external user's probe (moving joint -> fixed frame rotated 90 deg
      // about X -> fixed bracket -> moving joint) read 0.353 rad of orientation
      // error at q = 0.25 rad, while a correct pose at q = 0 hid it. Leader-frame
      // axis = R_leader^T * R_parent * axis (`v * M` is M^T * v). An unrotated
      // chain (R_parent == R_leader) is unchanged to rounding.
      axisLocal = (parentRot * axisLocal) * leaderRot;
      jointParentRot = leaderRot;
    }
    // Child link's authored rotation relative to its joint PARENT body
    // (R_child^T * R_parent): baked into Newton's child_xform so the
    // constraint preserves an off-axis child `rotation` (e.g. battlebox
    // wheels' `rotation 1 0 0 1.5708`) instead of projecting it to the
    // parent's orientation. Identity for axis-aligned children (URDF wheels,
    // huskies) -> those joints stay byte-unchanged. Passed for SLIDERS too
    // since 2026-09-27: before that only revolutes carried it, so a slider
    // child authored at 90 deg to its parent was snapped to the parent's
    // orientation at registration (1.5708 rad error on a minimal probe while
    // the slider readback still said the correct 0.1 m).
    const OmQuaternion childRelRot =
        (childRot.transposed() * jointParentRot).toQuaternion();

    OmMotor *const motor = hinge ? hinge->motor() : slider->motor();
    // URDF <mimic> follower (2026-10-06, OmUrdfImporter appendMimicFields): the
    // joint is coupled to its leader by a MuJoCo joint-equality constraint after
    // this loop, so it is registered WITHOUT a drive -- a servo of its own would
    // fight the constraint. Its motor's min/maxPosition still describe the stops.
    const bool mimicFollower = motor != nullptr && !motor->mimicMotor().isEmpty();
    OmMotor *const driveMotor = mimicFollower ? nullptr : motor;
    // ke/kd are decided AFTER the position limits are computed below: the
    // presence of finite limits is what distinguishes a position-controlled
    // limb from a velocity-driven wheel. See the control-mode-aware block
    // following the limit computation.
    double targetKe = 0.0;
    bool limitlessWheel = false;  // W1.4: set by the velocity-wheel branch below
    double targetKd = 0.0;
    // P3.10m: forward URDF effort + velocity + position limits to
    // Newton so the solver clips actuator force/velocity at physically
    // realistic levels regardless of our PD gain. This is the
    // equivalent of ODE's `maxForceOrTorque` capping; with it in
    // place, ke/kd values that would otherwise punch the body across
    // the floor get clamped at the real motor's torque ceiling.
    double effortLimit = 0.0;
    double velocityLimit = 0.0;
    double limitLower = 0.0;
    double limitUpper = 0.0;
    if (motor != nullptr) {
      if (driveMotor != nullptr) {
        effortLimit = motor->maxForceOrTorque();
        velocityLimit = motor->maxVelocity();
      }
      const double minP = motor->minPosition();
      const double maxP = motor->maxPosition();
      if (minP != maxP) {
        limitLower = minP;
        limitUpper = maxP;
      }
    }
    // URDF <limit lower/upper> is imported as the joint's mechanical stops
    // (minStop/maxStop), NOT the motor's control min/maxPosition (those
    // stay 0 -- the Newton joint diag showed lim=[0,0] on every joint, so
    // a knee could hyperextend past its -0.01 upper bound to +0.35 and
    // collapse the leg). Prefer the parameter stops when present.
    if (limitLower == limitUpper) {
      if (const OmJointParameters *const jp = hinge ? hinge->parameters() : slider->parameters()) {
        const double mn = jp->minStop();
        const double mx = jp->maxStop();
        if (mn != mx) {
          limitLower = mn;
          limitUpper = mx;
        }
      }
    }

    // Control-mode-aware ke/kd (regression fix 2026-05-29). A motorized
    // hinge with FINITE position limits is a position-controlled limb
    // (URDF "revolute": G1/Spot/Atlas legs+arms) and needs a position
    // spring to hold its setpoint -> ke=20/kd=3, the gains the deployed RL
    // policies were trained against and the Newton position default before
    // commit 46f2d9ba. A motorized hinge with NO limits is a velocity-
    // driven wheel (URDF "continuous": husky/jackal/rover) and keeps the
    // probe-verified ke=0/kd=500 pure-velocity config (see file header).
    //
    // Why this is the fix: commit 46f2d9ba correctly made per-joint ke/kd
    // authoritative (huskies need ke=0/kd=500), but this function hardcoded
    // the wheel config for EVERY motor, so every position-controlled limb
    // got kp=0 and could not hold its pose -- the canonical
    // g1_stand_deploy.omniworld faceplanted at 0.93s (knee commanded 0.42 rad,
    // actual ~0). Keying ke/kd on control mode restores G1/Spot/Atlas while
    // leaving wheels untouched. OMNISIM_NEWTON_TARGET_KE/KD still override
    // both branches in the Python builder (the Spot recipe sweeps 250/60).
    //
    // Full-range revolute joints (URDF |limit| >= pi-0.01, e.g. manipulator
    // arm joints with +/-2pi range) once reached here with no limits and
    // were mislabeled velocity wheels (kp=0). RESOLVED 2026-05-29 in the
    // importer (`OmUrdfImporter.cpp`): a full-range *revolute* now records its
    // URDF limits as the motor's min/maxPosition, so it arrives here with finite
    // `limitLower != limitUpper` and is correctly position-controlled. A motor
    // that still reaches the else-branch is a genuine `continuous`/limit-less
    // joint (husky/jackal/rover wheels) -- a true velocity wheel, no warning
    // needed.
    // A slider is a position-controlled actuator (gripper finger / linear
    // stage), never a free-spinning wheel -- classify it as position-
    // controlled even when the URDF recorded no finite travel limit, so it
    // gets a position spring (ke>0) instead of the ke=0/kd=500 velocity-wheel
    // config that left the gripper fingers limp and unable to close.
    // An unbounded (+-1e16-style) range on a HINGE is limit-less for this
    // purpose: it joins the velocity-wheel branch, and with it the W1.4 servo
    // promotion, exactly as a `continuous` joint does (isUnboundedLimitRange).
    const bool unboundedHinge = slider == nullptr && isUnboundedLimitRange(limitLower, limitUpper);
    const bool positionControlled =
        (driveMotor != nullptr) && ((limitLower != limitUpper && !unboundedHinge) || slider != nullptr);
    if (driveMotor != nullptr) {
      if (positionControlled) {
        // Position-spring stiffness scaled to the joint's torque capacity.
        // The legacy flat ke=20 left a heavy arm sagging ~0.8 rad under
        // gravity (force = ke*error only reached the gravity torque at a
        // large error) and a gripper finger clamping at ~0 N. The solver
        // clamps actuator force at effortLimit, so a high ke means "use the
        // available torque to track the setpoint", not runaway force.
        // effortLimit*10 holds a 194 N*m arm shoulder to <0.02 rad and
        // lets a 50 N finger clamp at its full force. Falls back to the
        // legacy 20/3 when the URDF declares no effort limit.
        // OMNISIM_NEWTON_TARGET_KE/KD still override both in the builder, so
        // the Spot/G1 RL recipes keep their explicitly-swept gains.
        targetKe = (effortLimit > 0.0) ? effortLimit * 10.0 : 20.0;
        targetKd = (effortLimit > 0.0) ? effortLimit * 0.5 : 3.0;
        // ⚠ WARN WHEN WE SUBSTITUTE, BECAUSE THE SUBSTITUTION MAKES A BROKEN
        // DESCRIPTION LOOK HEALTHY *HERE* AND NOWHERE ELSE.
        //
        // effortLimit <= 0 covers TWO different authoring cases that URDF
        // distinguishes and this branch cannot:
        //   * the attribute is OMITTED  -- URDF's way of spelling "unlimited";
        //   * effort="0" is DECLARED    -- a declared inability to apply torque.
        // The second is a real and common defect: a published 918-star hand
        // ships all 16 of its actuated joints at effort="0" velocity="0", and
        // measured in PyBullet not one of them moves (mean |err| = the full
        // commanded 0.5 rad). Here the same file gets ke = 20.0 and behaves
        // plausibly, so OmniSim is the one simulator that would NOT reveal it.
        //
        // scripts/dev/urdf_import.py already warns on the import side that
        // "consumers will give this joint zero authority". The engine saying
        // nothing means our own two halves disagree about the same file.
        //
        // This registration path runs once per joint per world load, so the
        // warning is naturally once per joint -- no latch needed.
        //
        // ⚠ Since 2026-10-10 the warning below is FALSE for a declared zero:
        // maxTorque 0 is an OFF motor, which commands nothing, and its zero
        // effort limit reaches the solver on the first tick (the available-
        // torque push further down), so the ke/kd here act through a zero
        // force range and the joint is passive. The importer has already
        // said so ("imports as a motor with no torque ... passive"), and the
        // old line followed it with "OmniSim drives it" -- two messages about
        // one joint that contradicted each other (Ti5 lane, 2026-10-19). Say
        // what happens instead, at info level; the warning stays for the one
        // case where it is still true, OMNISIM_NEWTON_AVAILABLE_TORQUE=0.
        if (effortLimit <= 0.0 && motor != nullptr && newtonAvailableTorqueEnabled()) {
          OmLog::info(QObject::tr(
              "Motor '%1': maxTorque/maxForce 0, so the joint is passive: its zero effort limit is applied "
              "on the first step, and the placeholder position gain it registers with (ke=%2 kd=%3) "
              "delivers no torque.")
              .arg(motor->deviceName()).arg(targetKe).arg(targetKd));
        } else if (effortLimit <= 0.0 && motor != nullptr) {
          OmLog::warning(QObject::tr(
              "Motor '%1': no positive effort limit declared, so a SYNTHETIC position gain "
              "ke=%2 kd=%3 is being used. If the source URDF declared effort=\"0\" rather "
              "than omitting it, that is a declaration that the joint cannot apply torque -- "
              "PyBullet, Gazebo and ros2_control will leave it inert while OmniSim drives it, "
              "so a defective description looks correct here and nowhere else. Declare a real "
              "effort limit, or omit the attribute if unlimited is intended.")
              .arg(motor->deviceName()).arg(targetKe).arg(targetKd));
        }
      } else {
        // VELOCITY-WHEEL CONFIG: no position spring, strong velocity damping.
        //
        // ⚠ THIS DISCRIMINATOR IS WRONG FOR A LIMITLESS SERVO, AND IT FAILS
        // SILENTLY. A motor whose joint declares no position limits lands here
        // and setPosition() then does NOTHING -- measured on a hinge commanded
        // to 0.5 rad with gravity off:
        //
        //   ODE                      0.500000 rad
        //   Newton, no limits        0.000000 rad
        //   Newton, minPosition/maxPosition declared   0.500000 rad
        //
        // In Webots the real signal for velocity control is setPosition(inf),
        // which is a RUNTIME fact, while this decision is made at joint
        // registration before any controller has spoken. Until the gains can be
        // re-configured when the first finite setPosition arrives, say so:
        // a caller otherwise sees a motor that accepts a target, reports no
        // error, and never moves.
        //
        // ⚠ WHY THE FULL FIX IS NOT DONE HERE (internal parity plan, item W1.4, and
        // this is a deliberate stop, not an oversight). The real signal for
        // velocity control is setPosition(inf), a RUNTIME fact; the natural fix
        // is to promote ke when OmMotor::isPIDPositionControl() first goes true
        // -- the signal exists (OmMotor.hpp) and pushNewtonMotorTargets already
        // branches on it every tick. What stops it being a one-liner is blast
        // radius: 1680 limit-less motorised joints across 189 files currently
        // get ke=0 by this branch, and promoting ke changes the dynamics of
        // every one of them. The 49 controllers using the setPosition(inf)
        // idiom are safe by construction (inf never makes isPIDPositionControl
        // true), but "safe by construction for the idiom we grepped" is not the
        // same as "safe", and a wrong default change is worse than a loud
        // warning. The warning below is the shipped half.
        targetKe = 0.0;
        targetKd = 500.0;
        limitlessWheel = true;
        if (shouldWarnLimitlessServoOnce(motor))
          OmLog::warning(QObject::tr(
                           "Joint motor '%1' declares no position limits, so this physics backend configures it as a "
                           "VELOCITY wheel: setPosition() on it will be IGNORED and setVelocity() is what drives it. "
                           "If it is meant to be a servo, give its motor a minPosition/maxPosition (or its joint a "
                           "minStop/maxStop) and it will be position-controlled. (Since 2026-09-01 the first finite "
                           "setPosition() FROM THE CONTROLLER promotes it to a position servo automatically; "
                           "OMNISIM_NEWTON_PROMOTE_SERVO=0 disables the promotion.)")
                           .arg(motor->deviceName()),
                         false, OmLog::ODE);
      }
    }

    const int idx = (slider != nullptr)
        ? newton->addJointPrismatic(
              parentIdx, childIdx,
              axisLocal.x(), axisLocal.y(), axisLocal.z(),
              parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
              childAnchor.x(), childAnchor.y(), childAnchor.z(),
              targetKe, targetKd,
              limitLower, limitUpper,
              effortLimit, velocityLimit,
              childRelRot.x(), childRelRot.y(), childRelRot.z(), childRelRot.w(),
              initialPosition)
        : newton->addJointRevolute(
              parentIdx, childIdx,
              axisLocal.x(), axisLocal.y(), axisLocal.z(),
              parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
              childAnchor.x(), childAnchor.y(), childAnchor.z(),
              targetKe, targetKd,
              limitLower, limitUpper,
              effortLimit, velocityLimit,
              childRelRot.x(), childRelRot.y(), childRelRot.z(), childRelRot.w(),
              initialPosition);
    p->mNewtonJointIndex = idx;
    if (idx >= 0 && motor != nullptr && limitlessWheel)
      limitlessNewtonJointIndices().insert(idx);
    // PASSIVE joint dynamics (2026-10-03). JointParameters.dampingConstant
    // (viscous, N*m*s/rad) and .staticFriction (Coulomb, N*m) -- where
    // OmUrdfImporter puts a URDF <dynamics damping friction> -- used to stop at
    // updateSpringAndDampingConstants(), an empty ODE-era sink, so they never
    // reached the solver: a published Raspberry Pi Mouse declaring
    // damping="1.0" friction="1.0" on both wheels compiled to dof_damping 0 and
    // dof_frictionloss 0. They now ride a separate runtime verb (so a binary and
    // a runtime of different vintages cannot shift a positional argument),
    // called only for a joint that declares a non-zero value -- every other
    // joint is registered exactly as before. The runtime applies them only with
    // OMNISIM_NEWTON_JOINT_DYNAMICS=1 (value-parsed, default OFF, read there) and
    // says which: 0 applied, 1 queued but off. Until 2026-10-09 this logged every
    // declared value as "passive damping ..." although the default applies none.
    if (idx >= 0) {
      if (const OmJointParameters *const jp = hinge ? hinge->parameters() : slider->parameters()) {
        const double damping = jp->dampingConstant();
        const double friction = jp->staticFriction();
        if (damping > 0.0 || friction > 0.0) {
          const int rc = newton->setJointPassiveDynamics(idx, damping, friction);
          if (rc == 0)
            OmLog::info(QString("[OmNewtonBackend] joint %1 '%2': passive damping %3, Coulomb friction %4 applied")
                            .arg(idx).arg(p->endPointName()).arg(damping).arg(friction));
          else if (rc == 1)
            OmLog::info(QString("[OmNewtonBackend] joint %1 '%2': declares damping %3, Coulomb friction %4 -- NOT "
                                "applied (OMNISIM_NEWTON_JOINT_DYNAMICS=1 applies them)")
                            .arg(idx).arg(p->endPointName()).arg(damping).arg(friction));
        }
      }
    }
    // Name every joint the 2026-09-27 frame fixes register differently from the
    // code before them, so a changed behaviour is attributable from the log:
    //  * an axis re-expressed into a merged leader whose frame is rotated;
    //  * a slider child whose authored rotation is now kept (it was identity).
    if (idx >= 0) {
      const OmVector3 authoredAxis = hinge ? hinge->axis() : slider->axis();
      if ((axisLocal - authoredAxis).length() > 1e-9)
        OmLog::info(QString("[OmNewtonBackend] joint %1 '%2': axis re-expressed in its merged leader's frame "
                            "(%3, %4, %5) -> (%6, %7, %8)")
                        .arg(idx).arg(p->endPointName())
                        .arg(authoredAxis.x()).arg(authoredAxis.y()).arg(authoredAxis.z())
                        .arg(axisLocal.x()).arg(axisLocal.y()).arg(axisLocal.z()));
      if (slider != nullptr) {
        const double w = std::min(1.0, std::abs(childRelRot.w()));
        const double angleDeg = 2.0 * std::acos(w) * 180.0 / M_PI;
        if (angleDeg > 1e-6)
          OmLog::info(QString("[OmNewtonBackend] slider joint %1 '%2': child keeps its authored rotation "
                              "relative to the parent body (%3 deg)")
                          .arg(idx).arg(p->endPointName()).arg(angleDeg));
      }
    }
    if (idx >= 0) {
      OmLog::info(QString("[OmNewtonBackend] hinge joint %1 (parent=body %2, child=body %3) "
                          "axis=(%4, %5, %6) anchor=(%7, %8, %9) "
                          "effort=%10 velLim=%11 lim=[%12, %13] %14")
                      .arg(idx)
                      .arg(parentIdx)
                      .arg(childIdx)
                      .arg(axisLocal.x())
                      .arg(axisLocal.y())
                      .arg(axisLocal.z())
                      .arg(anchor.x())
                      .arg(anchor.y())
                      .arg(anchor.z())
                      .arg(effortLimit)
                      .arg(velocityLimit)
                      .arg(limitLower)
                      .arg(limitUpper)
                      .arg(driveMotor != nullptr ?
                               QString("[motorized: kd=%1]").arg(targetKd) :
                               mimicFollower ? QString("[mimic of '%1']").arg(motor->mimicMotor()) :
                                               QStringLiteral("[free-spinning]")));
      if (driveMotor != nullptr)
        registeredNewtonMotorizedJoints().append(p);
      else if (mimicFollower)
        mimicFollowers.append(QPointer<OmBasicJoint>(p));
    }
  }

  // URDF <mimic> coupling: q_follower = offset + multiplier * q_leader as a MuJoCo
  // joint-equality constraint, between the follower and the joint driven by the
  // motor it names IN THE SAME ROBOT. A leader that cannot be found, or that has
  // no Newton joint, leaves the follower undriven and says so.
  for (const QPointer<OmBasicJoint> &f : mimicFollowers) {
    if (f.isNull())
      continue;
    const OmMotor *const fm = dynamic_cast<const OmHingeJoint *>(f.data()) ?
                                  static_cast<const OmHingeJoint *>(f.data())->motor() :
                                  dynamic_cast<const OmSliderJoint *>(f.data()) ?
                                  static_cast<const OmSliderJoint *>(f.data())->motor() : nullptr;
    if (fm == nullptr)
      continue;
    const OmBasicJoint *leader = nullptr;
    for (const OmSolid *solid : OmSolid::solids()) {
      for (OmBasicJoint *const j : solid->jointChildren()) {
        const OmMotor *const lm = dynamic_cast<const OmHingeJoint *>(j) ? static_cast<const OmHingeJoint *>(j)->motor() :
                                  dynamic_cast<const OmSliderJoint *>(j) ? static_cast<const OmSliderJoint *>(j)->motor() :
                                                                           nullptr;
        if (lm != nullptr && lm != fm && lm->robot() == fm->robot() && lm->deviceName() == fm->mimicMotor()) {
          leader = j;
          break;
        }
      }
      if (leader != nullptr)
        break;
    }
    const int rc = (leader != nullptr && leader->mNewtonJointIndex >= 0 && f->mNewtonJointIndex >= 0) ?
                       newton->addJointMimic(f->mNewtonJointIndex, leader->mNewtonJointIndex, fm->mimicMultiplier(),
                                             fm->mimicOffset()) :
                       -1;
    if (rc == 0)
      OmLog::info(QString("[OmNewtonBackend] joint %1 '%2' mimics joint %3 ('%4'): q = %5 + %6 * q_leader")
                      .arg(f->mNewtonJointIndex)
                      .arg(fm->deviceName())
                      .arg(leader->mNewtonJointIndex)
                      .arg(fm->mimicMotor())
                      .arg(fm->mimicOffset())
                      .arg(fm->mimicMultiplier()));
    else
      OmLog::warning(QObject::tr("Motor '%1' mimics '%2' (URDF <mimic>), but %3, so the joint is NOT coupled and "
                                 "is left undriven. OMNISIM_URDF_MIMIC=0 restores the old independent motor.")
                         .arg(fm->deviceName())
                         .arg(fm->mimicMotor())
                         .arg(leader == nullptr ? QObject::tr("no motor of that name exists in the same robot") :
                                                  QObject::tr("the physics backend could not add the coupling")));
  }
}

void OmBasicJoint::pushNewtonMotorTargets() {
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;

  // Diagnostic: log the first time we see a non-zero motor target on
  // each registered joint. One-shot per joint for the lifetime of the
  // process; helps verify the controller -> motor -> Newton chain is
  // wired without spamming the log every tick.
  QSet<int> &sLoggedNonZeroJointIndices = loggedNonZeroJointIndices();
  // Newton joint indices currently driven by a controller-set force. Tracked
  // so that LEAVING force mode can withdraw that force: nothing else clears
  // control.joint_f, and a stale one silently overpowers the position
  // controller that replaced it.
  QSet<int> &sForceModeJointIndices = forceModeJointIndices();

  for (const QPointer<OmBasicJoint> &p : registeredNewtonMotorizedJoints()) {
    if (p.isNull())
      continue;
    if (p->mNewtonJointIndex < 0)
      continue;
    // MULTI-DoF joints (Hinge2 / Ball) own their own per-axis push: each axis has
    // its own motor, its own control mode and its own DoF index inside the slot.
    // nodeType() is the exact type -- a dynamic_cast<OmHingeJoint*> would capture
    // both of them (OmBallJoint is-a OmHinge2Joint is-a OmHingeJoint) and drive
    // only axis1. Only reachable under the flag: nothing registers these joints in
    // this list otherwise.
    const int nt = p->nodeType();
    if (nt == WB_NODE_HINGE_2_JOINT || nt == WB_NODE_BALL_JOINT) {
      p->pushNewtonMultiDofTargets(newton);
      continue;
    }
    OmHingeJoint *const hinge = dynamic_cast<OmHingeJoint *>(p.data());
    OmSliderJoint *const slider =
        (hinge == nullptr) ? dynamic_cast<OmSliderJoint *>(p.data()) : nullptr;
    if (hinge == nullptr && slider == nullptr)
      continue;
    OmMotor *const motor = hinge ? hinge->motor() : slider->motor();
    if (motor == nullptr)
      continue;
    // TORQUE MODE: if the controller used setForceOrTorque (motor->userControl),
    // route the raw joint torque to Newton's control.joint_f and skip the
    // pos/vel push for this joint. Sign follows the newton/MuJoCo joint_f
    // convention (+tau -> +angle; verified via probe_newton_joint_torque),
    // which can differ from the ODE hinge-axis sign -- a torque-mode balance
    // law tunes to this convention directly. Pair with OMNISIM_NEWTON_TORQUE_MODE
    // (EFFORT-mode build) so no PD fights the commanded torque.
    if (motor->userControl()) {
      newton->setJointForce(p->mNewtonJointIndex, motor->rawInput());
      sForceModeJointIndices.insert(p->mNewtonJointIndex);
      continue;
    }
    // ...and when the controller LEAVES force mode, the force it last set has
    // to be withdrawn, or it keeps being applied underneath the position
    // controller that replaced it.
    //
    // Measured 2026-08-02: an agent closed a two-finger gripper with
    // setForce(+30), then called setPosition() to open it, and the fingers did
    // not move. Its workaround was setForce(-N), and it wrote down "switching
    // a force-mode motor back to setPosition() does NOT reopen it in this
    // engine" -- true as observed, and this is why. OmMotor::setTargetPosition
    // clears mUserControl correctly, so the mode DID switch; nothing cleared
    // control.joint_f, so a +30 N.m squeeze went on being applied while the
    // position target quietly lost the argument. Silent, and the symptom
    // (a gripper that will not open) looks nothing like its cause.
    if (sForceModeJointIndices.remove(p->mNewtonJointIndex))
      newton->setJointForce(p->mNewtonJointIndex, 0.0);
    // Available torque: push only CHANGES. The first push is skipped while the
    // motor still has its full maxTorque (the joint was registered with it), so
    // a world that never calls setAvailableTorque() never reaches the runtime.
    // A maxTorque of 0 is the exception: the runtime cannot register a zero
    // effort limit (0 there means "unlimited"), so it is pushed on the first tick.
    if (newtonAvailableTorqueEnabled()) {
      const double avail = motor->availableForceOrTorque();
      QHash<int, double> &pushed = pushedAvailableTorque();
      const auto it = pushed.constFind(p->mNewtonJointIndex);
      const bool changed = (it == pushed.constEnd()) ? (avail != motor->maxForceOrTorque() || avail == 0.0)
                                                     : (it.value() != avail);
      if (changed && newton->setJointEffortLimit(p->mNewtonJointIndex, 0, avail) == 0)
        pushed.insert(p->mNewtonJointIndex, avail);
    }
    // P3.10g: Newton's helper module drives every motorized revolute via
    // a velocity actuator (POSITION_VELOCITY mode w/ kd=500). When the
    // controller uses position control (setPosition() instead of
    // setVelocity()), `motor->targetVelocity()` just returns the
    // hardware max velocity each tick -- useless for tracking a position
    // target. Bridge it here: compute a PD-style velocity command from
    // the position error so the Newton joint actually converges on the
    // requested angle.
    // W1.4 servo promotion (2026-09-01): a limit-less joint was built as a
    // velocity wheel (ke=0), so a controller finite setPosition() had no
    // authority. The wire-level latch (OmMotor::controllerCommandedFinitePosition,
    // set ONLY by C_MOTOR_SET_POSITION -- never by reset or pre-configure
    // moves) is the promotion trigger; setPosition(inf) demotes back to the
    // exact wheel gains. The gain write goes into mj_model actuator
    // gainprm/biasprm directly (the model-array route is baked at conversion
    // and does nothing post-finalize).
    if (newtonPromoteServoEnabled() && limitlessNewtonJointIndices().contains(p->mNewtonJointIndex)) {
      const int jIdx = p->mNewtonJointIndex;
      const bool promoted = promotedServoJointIndices().contains(jIdx);
      const bool wants = motor->controllerCommandedFinitePosition();
      if (wants && !promoted) {
        const double effort = motor->maxForceOrTorque();
        const double servoKe = (effort > 0.0) ? effort * 10.0 : 20.0;
        const double servoKd = (effort > 0.0) ? effort * 0.5 : 3.0;
        const int rc = newton->setJointGains(jIdx, 0, servoKe, servoKd);
        if (rc == 0) {
          promotedServoJointIndices().insert(jIdx);
          // Un-cached push: the IfChanged cache may already hold this target.
          newton->setJointTargetPosition(jIdx, motor->targetPosition());
          OmLog::info(QString("[OmNewtonBackend] motor '%1' (joint %2) received a finite setPosition from its "
                              "controller; re-configuring it with position-servo gains ke=%3 kd=%4 "
                              "(was a ke=0 velocity wheel; OMNISIM_NEWTON_PROMOTE_SERVO=0 disables).")
                          .arg(motor->deviceName())
                          .arg(jIdx)
                          .arg(servoKe)
                          .arg(servoKd));
        } else {
          static QSet<int> sWarnedUnsupported;
          if (!sWarnedUnsupported.contains(jIdx)) {
            sWarnedUnsupported.insert(jIdx);
            OmLog::warning(QString("[OmNewtonBackend] motor '%1' (joint %2): servo promotion is not supported on "
                                   "this solver path (rc=%3; the GPU path bakes gains) -- setPosition() on this "
                                   "limit-less motor stays IGNORED. Declare minPosition/maxPosition instead.")
                               .arg(motor->deviceName())
                               .arg(jIdx)
                               .arg(rc));
          }
        }
      } else if (!wants && promoted) {
        if (newton->setJointGains(jIdx, 0, 0.0, 500.0) == 0)
          promotedServoJointIndices().remove(jIdx);
      }
    }
    // ⚠ A motor with no available torque is OFF (OmMotor: "no control"), and
    // isPIDPositionControl() is false for it -- which used to land in the
    // velocity branch and send targetVelocity(), i.e. maxVelocity until a
    // controller says otherwise. Measured 2026-10-10: a URDF caster declared
    // effort="0" (the Gazebo way to leave a joint passive) span at ~112 rad/s
    // from t=0, with the runtime's default (unlimited) effort behind it, and
    // the robot drove in random directions. An off motor commands nothing.
    double target;
    if (motor->availableForceOrTorque() == 0.0) {
      target = 0.0;
    } else if (motor->isPIDPositionControl()) {
      newton->setJointTargetPositionIfChanged(p->mNewtonJointIndex, motor->targetPosition());
      target = 0.0;
    } else {
      target = motor->targetVelocity();
    }
    // Change-detected (physics-step-cost-optimization-plan.md §3 item 5): the
    // target dicts on the Python side PERSIST across ticks, so re-sending an
    // unchanged value is a pure FFI round trip (PyGILState_Ensure +
    // PyObject_CallMethod, once or twice per motorised joint per tick). A
    // driving Husky holds a constant wheel-velocity setpoint for thousands of
    // ticks; a held arm pose likewise. Measured 0.025 ms/tick on the 8-Husky
    // world. Correctness rests on the setters being the ONLY writers of those
    // dict entries, and on the cache being dropped whenever the solver world
    // is rebuilt (see OmNewtonBackend::clearJointTargetCache).
    newton->setJointTargetVelocityIfChanged(p->mNewtonJointIndex, target);

    if (target != 0.0 && !sLoggedNonZeroJointIndices.contains(p->mNewtonJointIndex)) {
      sLoggedNonZeroJointIndices.insert(p->mNewtonJointIndex);
      OmLog::info(QString("[OmNewtonBackend] motor target_vel reached joint %1: %2 rad/s "
                          "(controller -> OmRotationalMotor -> backend chain verified)")
                      .arg(p->mNewtonJointIndex)
                      .arg(target));
    }
  }
}

bool OmBasicJoint::setJoint() {
  OmSolidReference *const sr = solidReference();
  if (sr)
    sr->updateName();
  const OmSolid *const s = solidEndPoint();
  const bool invalidEndPoint = s == NULL && (sr == NULL || !sr->pointsToStaticEnvironment());
  if (invalidEndPoint || upperSolid() == NULL || (s && s->physics() == NULL) || (s && s->solidMerger().isNull())) {
    // ODE removed: this used to disable the dJoint (and its spring/damper AMotor
    // companion) for an invalid end point. Both handles are permanently NULL now,
    // and a Newton joint is never registered for an invalid end point in the
    // first place, so returning false is the whole of it.
    return false;
  }

  return true;
}

void OmBasicJoint::setOdeJoint() {
  assert(upperSolid() && (solidEndPoint() || (solidReference() && solidReference()->pointsToStaticEnvironment())));
  // The ODE dJoint attach / enable that lived here is gone. Joint enablement
  // is a Newton question -- see OmBasicJoint::isEnabled(), which reports
  // mNewtonJointIndex. Subclasses hang their axis / anchor / stop bookkeeping
  // (the position offsets the Newton path reads) off this call.
}

void OmBasicJoint::reset(const QString &id) {
  OmBaseNode::reset(id);
  OmNode *const p = mParameters->value();
  OmNode *const e = mEndPoint->value();
  if (p)
    p->reset(id);
  if (e)
    e->reset(id);
}

void OmBasicJoint::save(const QString &id) {
  OmBaseNode::save(id);
  OmNode *const p = mParameters->value();
  OmNode *const e = mEndPoint->value();
  if (p)
    p->save(id);
  if (e)
    e->save(id);
}

void OmBasicJoint::updateSegmentationColor(const OmRgb &color) {
  OmBaseNode *const e = dynamic_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->updateSegmentationColor(color);
}

// Update methods: they check validity and correct if necessary

void OmBasicJoint::updateAfterParentPhysicsChanged() {
  OmSolid *const s = solidEndPoint();
  if (s) {
    s->appendJointParent(this);
    if (s->isKinematic())
      updateEndPointZeroTranslationAndRotation();
  }
}

void OmBasicJoint::updateEndPoint() {
  OmSolidReference *const r = solidReference();
  if (r)
    r->updateName();

  OmSolid *const s = solidEndPoint();
  if (s) {
    connect(s, &OmSolid::positionChangedArtificially, this, &OmBasicJoint::updateEndPointPosition, Qt::UniqueConnection);
    s->appendJointParent(this);
  }

  updateEndPointPosition();

  if (r)
    connect(r, &OmSolidReference::changed, this, &OmBasicJoint::setJoint, Qt::UniqueConnection);

  if (s == NULL || s->isPostFinalizedCalled()) {
    emit endPointChanged(s);
    if (s != NULL && isPostFinalizedCalled())
      OmBoundingSphere::addSubBoundingSphereToParentNode(this);
  } else {
    connect(s, &OmBaseNode::finalizationCompleted, this, &OmBasicJoint::endPointChanged, Qt::UniqueConnection);
    connect(s, &OmBaseNode::finalizationCompleted, this, &OmBasicJoint::updateBoundingSphere, Qt::UniqueConnection);
  }
}

void OmBasicJoint::updateBoundingSphere(OmBaseNode *subNode) {
  disconnect(subNode, &OmBaseNode::finalizationCompleted, this, &OmBasicJoint::updateBoundingSphere);
  OmBoundingSphere::addSubBoundingSphereToParentNode(this);
}

void OmBasicJoint::updateEndPointPosition() {
  if (mIsEndPointPositionChangedByJoint)
    return;

  const OmSolid *const s = solidEndPoint();
  if (s)
    updateEndPointZeroTranslationAndRotation();

  if (areOdeObjectsCreated())
    setJoint();
}

void OmBasicJoint::updateSpringAndDampingConstants() {
  // JointParameters.springConstant and Brake damping were realised as an ODE
  // AMotor companion joint; no Newton equivalent is wired, so they are
  // UNIMPLEMENTED and this signal sink does nothing. dampingConstant and
  // staticFriction DO reach Newton since 2026-10-03, but only at registration
  // (flushPendingNewtonRegistrations -> setJointPassiveDynamics): a runtime
  // edit of either field is still not pushed into a finalized model.
}

// Utility functions

void OmBasicJoint::setSolidEndPoint(OmSolid *solid) {
  mEndPoint->removeValue();
  mEndPoint->setValue(solid);
  updateEndPoint();
}

void OmBasicJoint::setSolidEndPoint(OmSolidReference *solid) {
  mEndPoint->removeValue();
  mEndPoint->setValue(solid);
  updateEndPoint();
}

void OmBasicJoint::setSolidEndPoint(OmSlot *slot) {
  mEndPoint->removeValue();
  mEndPoint->setValue(slot);
  updateEndPoint();
}

OmSolid *OmBasicJoint::solidEndPoint() const {
  const OmSlot *slot = dynamic_cast<OmSlot *>(mEndPoint->value());
  if (slot) {
    const OmSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot) {
      OmSolid *solid = childrenSlot->solidEndPoint();
      if (solid)
        return solid;

      const OmSolidReference *s = childrenSlot->solidReferenceEndPoint();
      if (s)
        return s->solid();
    }
  } else {
    OmSolid *solid = dynamic_cast<OmSolid *>(mEndPoint->value());
    if (solid)
      return solid;

    const OmSolidReference *const s = dynamic_cast<OmSolidReference *>(mEndPoint->value());
    if (s)
      return s->solid();
  }

  return NULL;
}

OmSolidReference *OmBasicJoint::solidReference() const {
  const OmSlot *slot = dynamic_cast<OmSlot *>(mEndPoint->value());
  if (slot) {
    const OmSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot)
      return childrenSlot->solidReferenceEndPoint();
    else
      return NULL;
  } else
    return dynamic_cast<OmSolidReference *>(mEndPoint->value());
}

OmSolid *OmBasicJoint::solidParent() const {
  return dynamic_cast<OmSolid *>(parentNode());
}

OmVector3 OmBasicJoint::anchor() const {
  static const OmVector3 ZERO(0.0, 0.0, 0.0);
  return ZERO;
}

bool OmBasicJoint::isEnabled() const {
  // ODE is gone, so no dJoint ever exists, but a Newton-registered joint is
  // the live articulation: it must keep its per-step Newton readback
  // (postPhysicsStep) and motor path, which callers gate on isEnabled().
  // (The ODE branch that used to follow -- dJointIsEnabled on mJoint -- was
  // already unreachable: mJoint is permanently NULL, so it returned false.)
  return mNewtonJointIndex >= 0;
}

//////////
// WREN //
//////////

void OmBasicJoint::createWrenObjects() {
  // D1.4: the WREN joint-axis line renderable died with the WREN renderer; the
  // wgpu overlay path (OmWgpuView collectors) draws joint axes now. Only the
  // endpoint recursion below survives.
  OmBaseNode::createWrenObjects();

  if (solidReference())  // don't create twice
    return;

  OmSlot *slot = dynamic_cast<OmSlot *>(mEndPoint->value());
  if (slot) {
    slot->createWrenObjects();
    return;
  }

  OmSolid *const s = solidEndPoint();
  if (s)
    s->createWrenObjects();
}

/////////////////
// Handle jerk //
/////////////////

bool OmBasicJoint::resetJointPositions() {
  OmSolid *const s = solidEndPoint();
  if (s == NULL)
    return false;

  const OmVector3 t(s->translation());
  const OmRotation r(s->rotation());

  const OmSolidReference *const sr = solidReference();
  // set ODE joint in initial position to avoid offset
  if (sr == NULL) {
    s->setTranslationAndRotation(mEndPointZeroTranslation, mEndPointZeroRotation);
    s->resetPhysics();
  }
  setJoint();

  // back to current position
  if (sr == NULL) {
    s->setTranslationAndRotation(t, r);
    s->resetPhysics();
  }

  return true;
}

void OmBasicJoint::retrieveEndPointSolidTranslationAndRotation(OmVector3 &it, OmRotation &ir) const {
  const OmSolid *const s = solidEndPoint();
  assert(s);

  if (solidReference()) {
    OmMatrix4 m = upperPose()->matrix().pseudoInversed() * s->matrix();
    ir = OmRotation(m.extracted3x3Matrix());
    it = m.translation();
  } else {
    ir = s->rotation();
    it = s->translation();
  }
}

void OmBasicJoint::write(OmWriter &writer) const {
  OmSolid *const s = solidEndPoint();
  OmVector3 translation;
  OmRotation rotation;

  if (s && nodeType() != WB_NODE_BALL_JOINT) {
    // remove unquantified ODE effects on the endPoint Solid translation and rotation fields
    translation = s->translation();
    rotation = s->rotation();
    OmVector3 computedTranslation;
    OmRotation computedRotation;
    const OmBasicJoint *instance = NULL;
    if (isProtoParameterNode())
      instance = dynamic_cast<OmBasicJoint *>(getFirstFinalizedProtoInstance());
    if (instance == NULL)
      instance = this;
    instance->computeEndPointSolidPositionFromParameters(computedTranslation, computedRotation);
    s->blockSignals(true);
    if (!translation.almostEquals(computedTranslation))
      s->setTranslationFromOde(computedTranslation);
    if (!rotation.almostEquals(computedRotation))
      s->setRotationFromOde(computedRotation);
    s->blockSignals(false);
  }

  if (writer.isOmniSim() || writer.isUrdf())
    OmBaseNode::write(writer);
  else {
    // we should not export any SolidReference Solid here,
    // otherwise they will appear duplicate in the W3D/VRML file,
    // this is why we don't use the solidEndPoint() method
    const OmSolid *solid = dynamic_cast<const OmSolid *>(mEndPoint->value());
    if (solid)
      solid->write(writer);
    else {
      const OmSlot *slot = dynamic_cast<const OmSlot *>(mEndPoint->value());
      if (slot) {
        const OmSlot *childrenSlot = slot->slotEndPoint();
        if (childrenSlot) {
          const OmSolid *solidInSlot = childrenSlot->solidEndPoint();
          if (solidInSlot)
            solidInSlot->write(writer);
        }
      }
    }
  }

  if (s && nodeType() != WB_NODE_BALL_JOINT) {
    // restore previous endPoint Solid status
    s->blockSignals(true);
    s->setTranslationFromOde(translation);
    s->setRotationFromOde(rotation);
    s->blockSignals(false);
  }
}

OmBoundingSphere *OmBasicJoint::boundingSphere() const {
  if (solidReference())
    return NULL;
  const OmSolid *const solid = solidEndPoint();
  if (solid)
    return solid->boundingSphere();
  return NULL;
}

QList<const OmBaseNode *> OmBasicJoint::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  if (mEndPoint->value())
    list << static_cast<OmBaseNode *>(mEndPoint->value())->findClosestDescendantNodesWithDedicatedWrenNode();
  return list;
}

QString OmBasicJoint::endPointName() const {
  if (!mEndPoint->value())
    return QString();

  QString name = mEndPoint->value()->computeName();
  if (name.isEmpty())
    name = mEndPoint->value()->endPointName();
  return name;
}
