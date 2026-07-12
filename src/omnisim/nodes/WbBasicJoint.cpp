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

#include "WbBasicJoint.hpp"

#include "WbBoundingSphere.hpp"
#include "WbHinge2Joint.hpp"
#include "WbHingeJoint.hpp"
#include "WbJointParameters.hpp"
#include "WbLinearMotor.hpp"
#include "WbLog.hpp"
#include "WbMotor.hpp"
#include "WbNewtonBackend.hpp"
#include "WbPhysicsBackend.hpp"
#include "WbQuaternion.hpp"
#include "WbRotationalMotor.hpp"
#include "WbSliderJoint.hpp"
#include "WbSlot.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/ode.h>
#include <QtCore/QPointer>
#include <QtCore/QSet>
#include <cassert>

namespace {
  // P3.7: deferred-registration queue. Joints push themselves here from
  // postFinalize when they look like they should be Newton-backed; the
  // actual addJointRevolute call happens in flushPendingNewtonRegistrations
  // (driven by WbSimulationWorld::step right before finalizeWorld), by
  // which point the parent and child Solids' postFinalize blocks have run
  // and their mNewtonBodyIndex values are populated.
  QList<QPointer<WbBasicJoint>> &pendingNewtonJoints() {
    static QList<QPointer<WbBasicJoint>> list;
    return list;
  }

  // P3.8.b: registry of joints that successfully landed in the Newton
  // model AND have an associated motor. Walked each tick by
  // pushNewtonMotorTargets to push controller-side velocity commands
  // through to the solver.
  QList<QPointer<WbBasicJoint>> &registeredNewtonMotorizedJoints() {
    static QList<QPointer<WbBasicJoint>> list;
    return list;
  }
}  // namespace

void WbBasicJoint::init() {
  mJoint = NULL;
  mIsReverseJoint = false;
  mSpringAndDamperMotor = NULL;
  mIsEndPointPositionChangedByJoint = false;
  mNewtonJointIndex = -1;

  mTransform = NULL;
  mRenderable = NULL;
  mMesh = NULL;
  mMaterial = NULL;

  mParameters = findSFNode("jointParameters");
  mEndPoint = findSFNode("endPoint");
}
// Constructors

WbBasicJoint::WbBasicJoint(const QString &modelName, WbTokenizer *tokenizer) : WbBaseNode(modelName, tokenizer) {
  init();
}

WbBasicJoint::WbBasicJoint(const WbBasicJoint &other) : WbBaseNode(other) {
  init();
}

WbBasicJoint::WbBasicJoint(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbBasicJoint::~WbBasicJoint() {
  WbSolid *const s = solidEndPoint();
  if (s && !s->isBeingDeleted())
    s->removeJointParent(this);

  if (mJoint)
    dJointDestroy(mJoint);
  mJoint = NULL;

  if (mSpringAndDamperMotor)
    dJointDestroy(mSpringAndDamperMotor);
  mSpringAndDamperMotor = NULL;

  if (areWrenObjectsInitialized()) {
    wr_static_mesh_delete(mMesh);
    wr_material_delete(mMaterial);
    wr_node_delete(WR_NODE(mRenderable));
    wr_node_delete(WR_NODE(mTransform));
  }
}

void WbBasicJoint::downloadAssets() {
  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->downloadAssets();
}

void WbBasicJoint::preFinalize() {
  WbBaseNode::preFinalize();

  // set endPoint initial position
  updateParameters();
  updateEndPointZeroTranslationAndRotation();

  WbBaseNode *const p = dynamic_cast<WbBaseNode *>(mParameters->value());
  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mEndPoint->value());
  if (p && !p->isPreFinalizedCalled())
    p->preFinalize();
  if (e && !e->isPreFinalizedCalled())
    e->preFinalize();
}

void WbBasicJoint::setMatrixNeedUpdate() {
  WbSolid *const s = solidEndPoint();
  if (s && solidReference() == NULL)
    s->setMatrixNeedUpdate();
}

void WbBasicJoint::postFinalize() {
  WbBaseNode::postFinalize();

  WbBaseNode *const p = dynamic_cast<WbBaseNode *>(mParameters->value());
  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mEndPoint->value());
  if (p && !p->isPostFinalizedCalled())
    p->postFinalize();
  if (e && !e->isPostFinalizedCalled())
    e->postFinalize();

  connect(mEndPoint, &WbSFNode::changed, this, &WbBasicJoint::updateEndPoint);
  const WbGroup *pg = dynamic_cast<WbGroup *>(parentNode());
  if (pg)
    connect(this, &WbBasicJoint::endPointChanged, pg, &WbGroup::insertChildFromSlotOrJoint);
  else {
    const WbSlot *slot = dynamic_cast<WbSlot *>(parentNode());
    if (slot)
      connect(this, &WbBasicJoint::endPointChanged, slot, &WbSlot::endPointInserted);
  }
  connect(mParameters, &WbSFNode::changed, this, &WbBasicJoint::updateParameters);

  WbSolid *const s = solidEndPoint();
  if (s) {
    connect(s, &WbSolid::positionChangedArtificially, this, &WbBasicJoint::updateEndPointPosition);
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
  WbSolid *const parent = solidParent();
  if (parent != nullptr && s != nullptr &&
      (dynamic_cast<WbHingeJoint *>(this) != nullptr ||
       dynamic_cast<WbSliderJoint *>(this) != nullptr) &&
      parent->effectivePhysicsBackendName() != QStringLiteral("ode") &&
      s->effectivePhysicsBackendName() != QStringLiteral("ode")) {
    pendingNewtonJoints().append(QPointer<WbBasicJoint>(this));
    // P5 hang fix 2026-05-28: the per-joint WbLog::info that the
    // concurrent G1 session added accumulates per-message cost during
    // world load. At 40+ huskies (160+ joints) the cost grows until the
    // load stalls. Suppressed here; the queue size is visible via the
    // "draining N entries" message in flushPendingNewtonRegistrations,
    // which is the right place for that breadcrumb.
  }

  if (protoParameterNode()) {
    const QVector<WbNode *> nodes = protoParameterNode()->protoParameterNodeInstances();
    if (nodes.size() > 1 && nodes.at(0) == this)
      parsingWarn(tr("Joint node defined in PROTO field is used multiple times. "
                     "OmniSim doesn't fully support this because the multiple node instances cannot be identical."));
  }

}

void WbBasicJoint::createOdeObjects() {
  WbBaseNode::createOdeObjects();
  WbSolid *const s = solidEndPoint();
  if (s && solidReference() == NULL)
    s->createOdeObjects();
}

// Newton enforcement (2026-06-29 default: no silent Newton->ODE). A joint whose articulation resolved
// to Newton but whose endpoint body never registered a Newton body is "silently inert" -- the joint
// drops out and that limb runs on ODE while the rest of the robot is on Newton. Under enforcement that is
// a hard error. Fires ONLY when the MISSING endpoint actually wanted Newton (its effective backend is not
// an explicit "ode"); a joint that legitimately belongs to an ODE articulation has no Newton body by
// design and is left alone. Body registration runs before joint registration (WbSimulationWorld), so a
// negative index here is a genuine miss, not a transient ordering artifact.
static void enforceNewtonJointEndpoints(const WbSolid *parent, const WbSolid *child,
                                        int parentIdx, int childIdx) {
  if (!WbPhysicsBackendRegistry::newtonEnforced())
    return;
  static const QString kOde = QStringLiteral("ode");
  const bool parentMissing =
      parentIdx < 0 && parent != nullptr && parent->effectivePhysicsBackendName() != kOde;
  const bool childMissing =
      childIdx < 0 && child != nullptr && child->effectivePhysicsBackendName() != kOde;
  if (!parentMissing && !childMissing)
    return;
  const WbSolid *const missing = parentMissing ? parent : child;
  WbLog::fatal(
      QString("[newton-enforce] A joint's %1 body '%2' resolved to Newton but never registered a "
              "Newton body, so the joint would be silently inert and that part of the articulation "
              "would fall back to ODE. Newton enforcement is on (the default on a Newton-capable "
              "build); a silent Newton->ODE downgrade is refused. Fix the model so the body "
              "registers, or set the articulation's physicsBackend to \"ode\" explicitly, or allow "
              "the graceful fall-back with OMNISIM_ALLOW_ODE_FALLBACK=1.")
          .arg(parentMissing ? QStringLiteral("parent") : QStringLiteral("child"))
          .arg(missing != nullptr ? missing->name() : QStringLiteral("?")));
}

void WbBasicJoint::flushPendingNewtonRegistrations() {
  // Drain the queue regardless of whether registration succeeds, so
  // we don't repeatedly re-attempt failed entries on subsequent ticks.
  // (Failures are unrecoverable at this level: missing body index, no
  // backend, etc. The corresponding Solids fall back to ODE-only
  // dynamics and the joint is silently inert.)
  QList<QPointer<WbBasicJoint>> queue;
  std::swap(queue, pendingNewtonJoints());

  WbPhysicsBackend *const raw = WbPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  WbNewtonBackend *const newton = static_cast<WbNewtonBackend *>(raw);

  for (const QPointer<WbBasicJoint> &p : queue) {
    if (p.isNull())
      continue;
    // Ball / spherical joint (newton-ode-replacement-plan.md W2.2): MUST be handled before BOTH the Hinge2
    // and the WbHingeJoint casts below -- WbBallJoint is-a WbHinge2Joint is-a WbHingeJoint, so either cast
    // would wrongly capture it and register only 1-2 of its 3 rotational DoF. nodeType() is the exact type
    // (the same discriminator the capability gate uses). Self-contained (mirrors the hinge/slider anchor
    // convention): resolve the two body indices + the shared anchor, then register a native 3-DoF ball
    // joint. PASSIVE -- ball target control is MuJoCo-only upstream, so a motorised ball joint is a follow-up.
    if (p->nodeType() == WB_NODE_BALL_JOINT) {
      const WbSolid *const parent = p->solidParent();
      const WbSolid *const child = p->solidEndPoint();
      if (parent == nullptr || child == nullptr)
        continue;
      const int parentIdx = parent->effectiveNewtonBodyIndex();
      const int childIdx = child->effectiveNewtonBodyIndex();
      if (parentIdx < 0 || childIdx < 0) {
        enforceNewtonJointEndpoints(parent, child, parentIdx, childIdx);
        continue;
      }
      const WbVector3 anchor = p->anchor();
      const WbVector3 jointWorld = parent->matrix().translation() + parent->rotationMatrix() * anchor;
      const WbVector3 childAnchor = (jointWorld - child->matrix().translation()) * child->rotationMatrix();
      WbVector3 parentAnchor = anchor;  // re-express in the merger LEADER's frame if the parent merged away
      if (parent->newtonBodyIndex() < 0 && parent->solidMerger() != nullptr &&
          parent->solidMerger()->solid() != nullptr && parent->solidMerger()->solid() != parent) {
        const WbSolid *const leader = parent->solidMerger()->solid();
        parentAnchor = (jointWorld - leader->matrix().translation()) * leader->rotationMatrix();
      }
      const int idx = newton->addJointBall(parentIdx, childIdx,
                                           parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
                                           childAnchor.x(), childAnchor.y(), childAnchor.z());
      p->mNewtonJointIndex = idx;
      if (idx >= 0)
        WbLog::info(QString("[WbNewtonBackend] ball joint %1 (parent=body %2, child=body %3) anchor=(%4,%5,%6)")
                        .arg(idx).arg(parentIdx).arg(childIdx)
                        .arg(parentAnchor.x()).arg(parentAnchor.y()).arg(parentAnchor.z()));
      continue;
    }
    // Hinge2 / universal (newton-ode-replacement-plan.md W2): MUST be handled before the WbHingeJoint cast
    // below -- WbHinge2Joint is-a WbHingeJoint, so that cast would wrongly accept it and register only
    // axis1. Self-contained (mirrors the hinge/slider anchor convention): resolve the two body indices +
    // the shared anchor, then register a native passive 2-DoF d6 joint. The cart casters are free-
    // spinning, so no motor path -- a motorised hinge2 is a follow-up.
    if (WbHinge2Joint *const h2 = dynamic_cast<WbHinge2Joint *>(p.data())) {
      const WbSolid *const parent = h2->solidParent();
      const WbSolid *const child = h2->solidEndPoint();
      if (parent == nullptr || child == nullptr)
        continue;
      const int parentIdx = parent->effectiveNewtonBodyIndex();
      const int childIdx = child->effectiveNewtonBodyIndex();
      if (parentIdx < 0 || childIdx < 0) {
        enforceNewtonJointEndpoints(parent, child, parentIdx, childIdx);
        continue;
      }
      const WbVector3 anchor = p->anchor();
      const WbVector3 jointWorld = parent->matrix().translation() + parent->rotationMatrix() * anchor;
      const WbVector3 childAnchor = (jointWorld - child->matrix().translation()) * child->rotationMatrix();
      WbVector3 parentAnchor = anchor;  // re-express in the merger LEADER's frame if the parent merged away
      if (parent->newtonBodyIndex() < 0 && parent->solidMerger() != nullptr &&
          parent->solidMerger()->solid() != nullptr && parent->solidMerger()->solid() != parent) {
        const WbSolid *const leader = parent->solidMerger()->solid();
        parentAnchor = (jointWorld - leader->matrix().translation()) * leader->rotationMatrix();
      }
      const WbVector3 a1 = h2->axis();
      // axis2() is protected; derive it from the public parameters2() exactly as the protected accessor
      // does (parameters2()->axis(), defaulting to +Z when the second parameters node is absent).
      const WbVector3 a2 = h2->parameters2() ? h2->parameters2()->axis() : WbVector3(0.0, 0.0, 1.0);
      const int idx = newton->addJointHinge2(parentIdx, childIdx, a1.x(), a1.y(), a1.z(), a2.x(), a2.y(),
                                             a2.z(), parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
                                             childAnchor.x(), childAnchor.y(), childAnchor.z());
      p->mNewtonJointIndex = idx;
      if (idx >= 0)
        WbLog::info(QString("[WbNewtonBackend] hinge2 joint %1 (parent=body %2, child=body %3) "
                            "axis1=(%4,%5,%6) axis2=(%7,%8,%9)")
                        .arg(idx).arg(parentIdx).arg(childIdx)
                        .arg(a1.x()).arg(a1.y()).arg(a1.z()).arg(a2.x()).arg(a2.y()).arg(a2.z()));
      continue;
    }
    WbHingeJoint *const hinge = dynamic_cast<WbHingeJoint *>(p.data());
    WbSliderJoint *const slider =
        (hinge == nullptr) ? dynamic_cast<WbSliderJoint *>(p.data()) : nullptr;
    // Newton handles revolute (hinge) and prismatic (slider) joints. The
    // parallel-gripper fingers ride on sliders; without this branch their
    // bodies dropped out of the articulation as orphans and never closed.
    if (hinge == nullptr && slider == nullptr)
      continue;
    const WbSolid *const parent = hinge ? hinge->solidParent() : slider->solidParent();
    const WbSolid *const child = hinge ? hinge->solidEndPoint() : slider->solidEndPoint();
    if (parent == nullptr || child == nullptr)
      continue;
    // P3.10c: walk to merger leader. URDF fixed-joint children share an
    // ODE body via WbSolidMerger; the matching Newton body lives on the
    // leader. A wheel hinge whose parent Solid is e.g. `top_chassis_link`
    // (a non-leader fixed-merger participant) needs to attach to the
    // chassis leader's Newton body, not to a non-existent body of the
    // merged-away child.
    const int parentIdx = parent->effectiveNewtonBodyIndex();
    const int childIdx = child->effectiveNewtonBodyIndex();
    if (parentIdx < 0 || childIdx < 0) {
      enforceNewtonJointEndpoints(parent, child, parentIdx, childIdx);
      continue;
    }

    const WbVector3 axisLocal = hinge ? hinge->axis() : slider->axis();
    // Webots' anchor is in the parent's LOCAL frame; Newton's
    // parent_xform takes that directly. For child_xform (joint
    // frame in the child's LOCAL space), we project:
    //
    //   joint_world  = parent_world + R_parent * parent_anchor
    //   child_anchor = R_child^T * (joint_world - child_world)
    //
    // `v * M` on WbVector3 returns M^T * v (see WbMatrix3.hpp), so the
    // final line below is the R_child^T multiplication without an
    // explicit transpose.
    const WbVector3 anchor = p->anchor();  // virtual; public on WbBasicJoint
    const WbVector3 parentWorld = parent->matrix().translation();
    const WbVector3 childWorld = child->matrix().translation();
    const WbMatrix3 parentRot = parent->rotationMatrix();
    const WbMatrix3 childRot = child->rotationMatrix();
    const WbVector3 jointWorld = parentWorld + parentRot * anchor;
    const WbVector3 childAnchor = (jointWorld - childWorld) * childRot;

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
    WbVector3 parentAnchor = anchor;
    WbMatrix3 jointParentRot = parentRot;
    if (parent->newtonBodyIndex() < 0 && parent->solidMerger() != nullptr &&
        parent->solidMerger()->solid() != nullptr &&
        parent->solidMerger()->solid() != parent) {
      const WbSolid *const leader = parent->solidMerger()->solid();
      const WbVector3 leaderWorld = leader->matrix().translation();
      const WbMatrix3 leaderRot = leader->rotationMatrix();
      parentAnchor = (jointWorld - leaderWorld) * leaderRot;
      jointParentRot = leaderRot;
    }
    // Child link's authored rotation relative to its joint PARENT body
    // (R_child^T * R_parent): baked into Newton's child_xform so a revolute
    // constraint preserves an off-axis child `rotation` (e.g. battlebox
    // wheels' `rotation 1 0 0 1.5708`) instead of projecting it to the
    // parent's orientation. Identity for axis-aligned children (URDF wheels,
    // huskies) -> those joints stay byte-unchanged.
    const WbQuaternion childRelRot =
        (childRot.transposed() * jointParentRot).toQuaternion();

    WbMotor *const motor = hinge ? hinge->motor() : slider->motor();
    // ke/kd are decided AFTER the position limits are computed below: the
    // presence of finite limits is what distinguishes a position-controlled
    // limb from a velocity-driven wheel. See the control-mode-aware block
    // following the limit computation.
    double targetKe = 0.0;
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
      effortLimit = motor->maxForceOrTorque();
      velocityLimit = motor->maxVelocity();
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
      if (const WbJointParameters *const jp = hinge ? hinge->parameters() : slider->parameters()) {
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
    // g1_stand_deploy.wbt faceplanted at 0.93s (knee commanded 0.42 rad,
    // actual ~0). Keying ke/kd on control mode restores G1/Spot/Atlas while
    // leaving wheels untouched. OMNISIM_NEWTON_TARGET_KE/KD still override
    // both branches in the Python builder (the Spot recipe sweeps 250/60).
    //
    // Full-range revolute joints (URDF |limit| >= pi-0.01, e.g. manipulator
    // arm joints with +/-2pi range) once reached here with no limits and
    // were mislabeled velocity wheels (kp=0). RESOLVED 2026-05-29 in the
    // importer (`WbUrdfImporter.cpp`): a full-range *revolute* now records its
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
    const bool positionControlled =
        (motor != nullptr) && (limitLower != limitUpper || slider != nullptr);
    if (motor != nullptr) {
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
      } else {
        targetKe = 0.0;
        targetKd = 500.0;
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
              effortLimit, velocityLimit)
        : newton->addJointRevolute(
              parentIdx, childIdx,
              axisLocal.x(), axisLocal.y(), axisLocal.z(),
              parentAnchor.x(), parentAnchor.y(), parentAnchor.z(),
              childAnchor.x(), childAnchor.y(), childAnchor.z(),
              targetKe, targetKd,
              limitLower, limitUpper,
              effortLimit, velocityLimit,
              childRelRot.x(), childRelRot.y(), childRelRot.z(), childRelRot.w());
    p->mNewtonJointIndex = idx;
    if (idx >= 0) {
      WbLog::info(QString("[WbNewtonBackend] hinge joint %1 (parent=body %2, child=body %3) "
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
                      .arg(motor != nullptr ?
                               QString("[motorized: kd=%1]").arg(targetKd) :
                               QStringLiteral("[free-spinning]")));
      if (motor != nullptr)
        registeredNewtonMotorizedJoints().append(p);
    }
  }
}

void WbBasicJoint::pushNewtonMotorTargets() {
  WbPhysicsBackend *const raw = WbPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  WbNewtonBackend *const newton = static_cast<WbNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;

  // Diagnostic: log the first time we see a non-zero motor target on
  // each registered joint. One-shot per joint for the lifetime of the
  // process; helps verify the controller -> motor -> Newton chain is
  // wired without spamming the log every tick.
  static QSet<int> sLoggedNonZeroJointIndices;

  for (const QPointer<WbBasicJoint> &p : registeredNewtonMotorizedJoints()) {
    if (p.isNull())
      continue;
    if (p->mNewtonJointIndex < 0)
      continue;
    WbHingeJoint *const hinge = dynamic_cast<WbHingeJoint *>(p.data());
    WbSliderJoint *const slider =
        (hinge == nullptr) ? dynamic_cast<WbSliderJoint *>(p.data()) : nullptr;
    if (hinge == nullptr && slider == nullptr)
      continue;
    WbMotor *const motor = hinge ? hinge->motor() : slider->motor();
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
      continue;
    }
    // P3.10g: Newton's helper module drives every motorized revolute via
    // a velocity actuator (POSITION_VELOCITY mode w/ kd=500). When the
    // controller uses position control (setPosition() instead of
    // setVelocity()), `motor->targetVelocity()` just returns the
    // hardware max velocity each tick -- useless for tracking a position
    // target. Bridge it here: compute a PD-style velocity command from
    // the position error so the Newton joint actually converges on the
    // requested angle.
    double target;
    if (motor->isPIDPositionControl()) {
      newton->setJointTargetPosition(p->mNewtonJointIndex, motor->targetPosition());
      target = 0.0;
    } else {
      target = motor->targetVelocity();
    }
    newton->setJointTargetVelocity(p->mNewtonJointIndex, target);

    if (target != 0.0 && !sLoggedNonZeroJointIndices.contains(p->mNewtonJointIndex)) {
      sLoggedNonZeroJointIndices.insert(p->mNewtonJointIndex);
      WbLog::info(QString("[WbNewtonBackend] motor target_vel reached joint %1: %2 rad/s "
                          "(controller -> WbRotationalMotor -> backend chain verified)")
                      .arg(p->mNewtonJointIndex)
                      .arg(target));
    }
  }
}

bool WbBasicJoint::setJoint() {
  WbSolidReference *const sr = solidReference();
  if (sr)
    sr->updateName();
  const WbSolid *const s = solidEndPoint();
  const bool invalidEndPoint = s == NULL && (sr == NULL || !sr->pointsToStaticEnvironment());
  if (invalidEndPoint || upperSolid() == NULL || (s && s->physics() == NULL) || (s && s->solidMerger().isNull())) {
    // P1.6 slice 5: disable routes through the dispatcher.
    WbPhysicsBackend *const ode = WbPhysicsBackendRegistry::odeBackend();
    if (mJoint) {
      dJointAttach(mJoint, NULL, NULL);
      ode->setJointEnabled(reinterpret_cast<WbJointHandle>(mJoint), false);
    }
    if (mSpringAndDamperMotor) {
      dJointAttach(mSpringAndDamperMotor, NULL, NULL);
      ode->setJointEnabled(reinterpret_cast<WbJointHandle>(mSpringAndDamperMotor), false);
    }
    return false;
  }

  return true;
}

void WbBasicJoint::setOdeJoint(dBodyID body, dBodyID parentBody) {
  assert(mJoint && upperSolid() && (solidEndPoint() || (solidReference() && solidReference()->pointsToStaticEnvironment())));
  // linked to static environment: ODE internally requires the first body to be not NULL and switches the two bodies
  mIsReverseJoint = parentBody == NULL;
  dJointAttach(mJoint, parentBody, body);
  // P1.6 slice 5: enable/disable routes through the dispatcher.
  const bool enable = !(parentBody == NULL && body == NULL);
  WbPhysicsBackendRegistry::odeBackend()->setJointEnabled(
    reinterpret_cast<WbJointHandle>(mJoint), enable);

  applyToOdeSpringAndDampingConstants(body, parentBody);
}

void WbBasicJoint::reset(const QString &id) {
  WbBaseNode::reset(id);
  WbNode *const p = mParameters->value();
  WbNode *const e = mEndPoint->value();
  if (p)
    p->reset(id);
  if (e)
    e->reset(id);
}

void WbBasicJoint::save(const QString &id) {
  WbBaseNode::save(id);
  WbNode *const p = mParameters->value();
  WbNode *const e = mEndPoint->value();
  if (p)
    p->save(id);
  if (e)
    e->save(id);
}

void WbBasicJoint::updateSegmentationColor(const WbRgb &color) {
  WbBaseNode *const e = dynamic_cast<WbBaseNode *>(mEndPoint->value());
  if (e)
    e->updateSegmentationColor(color);
}

// Update methods: they check validity and correct if necessary

void WbBasicJoint::updateAfterParentPhysicsChanged() {
  WbSolid *const s = solidEndPoint();
  if (s) {
    s->appendJointParent(this);
    if (s->isKinematic())
      updateEndPointZeroTranslationAndRotation();
  }
}

void WbBasicJoint::updateEndPoint() {
  WbSolidReference *const r = solidReference();
  if (r)
    r->updateName();

  WbSolid *const s = solidEndPoint();
  if (s) {
    connect(s, &WbSolid::positionChangedArtificially, this, &WbBasicJoint::updateEndPointPosition, Qt::UniqueConnection);
    s->appendJointParent(this);
  }

  updateEndPointPosition();

  if (r)
    connect(r, &WbSolidReference::changed, this, &WbBasicJoint::setJoint, Qt::UniqueConnection);

  if (s == NULL || s->isPostFinalizedCalled()) {
    emit endPointChanged(s);
    if (s != NULL && isPostFinalizedCalled())
      WbBoundingSphere::addSubBoundingSphereToParentNode(this);
  } else {
    connect(s, &WbBaseNode::finalizationCompleted, this, &WbBasicJoint::endPointChanged, Qt::UniqueConnection);
    connect(s, &WbBaseNode::finalizationCompleted, this, &WbBasicJoint::updateBoundingSphere, Qt::UniqueConnection);
  }
}

void WbBasicJoint::updateBoundingSphere(WbBaseNode *subNode) {
  disconnect(subNode, &WbBaseNode::finalizationCompleted, this, &WbBasicJoint::updateBoundingSphere);
  WbBoundingSphere::addSubBoundingSphereToParentNode(this);
}

void WbBasicJoint::updateEndPointPosition() {
  if (mIsEndPointPositionChangedByJoint)
    return;

  const WbSolid *const s = solidEndPoint();
  if (s)
    updateEndPointZeroTranslationAndRotation();

  if (areOdeObjectsCreated())
    setJoint();
}

void WbBasicJoint::updateSpringAndDampingConstants() {
  const WbSolid *const s = solidEndPoint();
  const WbSolid *const us = upperSolid();
  if (areOdeObjectsCreated() && s && us)
    applyToOdeSpringAndDampingConstants(s->body(), us->bodyMerger());
}

// Utility functions

void WbBasicJoint::setSolidEndPoint(WbSolid *solid) {
  mEndPoint->removeValue();
  mEndPoint->setValue(solid);
  updateEndPoint();
}

void WbBasicJoint::setSolidEndPoint(WbSolidReference *solid) {
  mEndPoint->removeValue();
  mEndPoint->setValue(solid);
  updateEndPoint();
}

void WbBasicJoint::setSolidEndPoint(WbSlot *slot) {
  mEndPoint->removeValue();
  mEndPoint->setValue(slot);
  updateEndPoint();
}

WbSolid *WbBasicJoint::solidEndPoint() const {
  const WbSlot *slot = dynamic_cast<WbSlot *>(mEndPoint->value());
  if (slot) {
    const WbSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot) {
      WbSolid *solid = childrenSlot->solidEndPoint();
      if (solid)
        return solid;

      const WbSolidReference *s = childrenSlot->solidReferenceEndPoint();
      if (s)
        return s->solid();
    }
  } else {
    WbSolid *solid = dynamic_cast<WbSolid *>(mEndPoint->value());
    if (solid)
      return solid;

    const WbSolidReference *const s = dynamic_cast<WbSolidReference *>(mEndPoint->value());
    if (s)
      return s->solid();
  }

  return NULL;
}

WbSolidReference *WbBasicJoint::solidReference() const {
  const WbSlot *slot = dynamic_cast<WbSlot *>(mEndPoint->value());
  if (slot) {
    const WbSlot *childrenSlot = slot->slotEndPoint();
    if (childrenSlot)
      return childrenSlot->solidReferenceEndPoint();
    else
      return NULL;
  } else
    return dynamic_cast<WbSolidReference *>(mEndPoint->value());
}

WbSolid *WbBasicJoint::solidParent() const {
  return dynamic_cast<WbSolid *>(parentNode());
}

WbVector3 WbBasicJoint::anchor() const {
  static const WbVector3 ZERO(0.0, 0.0, 0.0);
  return ZERO;
}

bool WbBasicJoint::isEnabled() const {
  if (!mJoint)
    return false;
  // P1.6 slice 5: isJointEnabled returns 1 (enabled) / 0 (disabled) / -1
  // (unsupported). On ODE this is always 0 or 1; if a future backend
  // returns -1, treat it as "enabled" -- the safer default for callers
  // that gate behaviour on this (e.g. WbHingeJoint::prePhysicsStep
  // skipping motor control when !isEnabled).
  const int r = WbPhysicsBackendRegistry::odeBackend()->isJointEnabled(
    reinterpret_cast<WbJointHandle>(mJoint));
  return r != 0;
}

//////////
// WREN //
//////////

void WbBasicJoint::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  const float color[3] = {0.0f, 0.0f, 0.0f};
  mMaterial = wr_phong_material_new();
  wr_phong_material_set_color(mMaterial, color);
  wr_material_set_default_program(mMaterial, WbWrenShaders::lineSetShader());

  mRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRenderable, false);
  wr_renderable_set_receive_shadows(mRenderable, false);
  wr_renderable_set_material(mRenderable, mMaterial, NULL);
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_JOINT_AXES);
  wr_renderable_set_drawing_mode(mRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);

  mTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mTransform), false);
  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mTransform));

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbBasicJoint::updateOptionalRendering);

  if (solidReference())  // don't create twice
    return;

  WbSlot *slot = dynamic_cast<WbSlot *>(mEndPoint->value());
  if (slot) {
    slot->createWrenObjects();
    return;
  }

  WbSolid *const s = solidEndPoint();
  if (s)
    s->createWrenObjects();
}

void WbBasicJoint::updateOptionalRendering(int option) {
  if (option == WbWrenRenderingContext::VF_JOINT_AXES) {
    if (WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option)) {
      updateJointAxisRepresentation();
      wr_node_set_visible(WR_NODE(mTransform), true);
    } else
      wr_node_set_visible(WR_NODE(mTransform), false);
  }
}
/////////////////
// Handle jerk //
/////////////////

bool WbBasicJoint::resetJointPositions() {
  WbSolid *const s = solidEndPoint();
  if (s == NULL)
    return false;

  const WbVector3 t(s->translation());
  const WbRotation r(s->rotation());

  const WbSolidReference *const sr = solidReference();
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

void WbBasicJoint::retrieveEndPointSolidTranslationAndRotation(WbVector3 &it, WbRotation &ir) const {
  const WbSolid *const s = solidEndPoint();
  assert(s);

  if (solidReference()) {
    WbMatrix4 m = upperPose()->matrix().pseudoInversed() * s->matrix();
    ir = WbRotation(m.extracted3x3Matrix());
    it = m.translation();
  } else {
    ir = s->rotation();
    it = s->translation();
  }
}

void WbBasicJoint::write(WbWriter &writer) const {
  WbSolid *const s = solidEndPoint();
  WbVector3 translation;
  WbRotation rotation;

  if (s && nodeType() != WB_NODE_BALL_JOINT) {
    // remove unquantified ODE effects on the endPoint Solid translation and rotation fields
    translation = s->translation();
    rotation = s->rotation();
    WbVector3 computedTranslation;
    WbRotation computedRotation;
    const WbBasicJoint *instance = NULL;
    if (isProtoParameterNode())
      instance = dynamic_cast<WbBasicJoint *>(getFirstFinalizedProtoInstance());
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

  if (writer.isWebots() || writer.isUrdf())
    WbBaseNode::write(writer);
  else {
    // we should not export any SolidReference Solid here,
    // otherwise they will appear duplicate in the W3D/VRML file,
    // this is why we don't use the solidEndPoint() method
    const WbSolid *solid = dynamic_cast<const WbSolid *>(mEndPoint->value());
    if (solid)
      solid->write(writer);
    else {
      const WbSlot *slot = dynamic_cast<const WbSlot *>(mEndPoint->value());
      if (slot) {
        const WbSlot *childrenSlot = slot->slotEndPoint();
        if (childrenSlot) {
          const WbSolid *solidInSlot = childrenSlot->solidEndPoint();
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

WbBoundingSphere *WbBasicJoint::boundingSphere() const {
  if (solidReference())
    return NULL;
  const WbSolid *const solid = solidEndPoint();
  if (solid)
    return solid->boundingSphere();
  return NULL;
}

QList<const WbBaseNode *> WbBasicJoint::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const WbBaseNode *> list;
  if (mEndPoint->value())
    list << static_cast<WbBaseNode *>(mEndPoint->value())->findClosestDescendantNodesWithDedicatedWrenNode();
  return list;
}

QString WbBasicJoint::endPointName() const {
  if (!mEndPoint->value())
    return QString();

  QString name = mEndPoint->value()->computeName();
  if (name.isEmpty())
    name = mEndPoint->value()->endPointName();
  return name;
}
