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

// Abstract node class representing the most general mechanical joint

#ifndef OM_BASIC_JOINT_HPP
#define OM_BASIC_JOINT_HPP

#include "OmBaseNode.hpp"
#include "OmOdeTypes.hpp"
#include "OmRotation.hpp"
#include "OmVector3.hpp"

class OmBoundingSphere;
class OmJointParameters;
class OmMotor;
class OmNewtonBackend;
class OmQuaternion;
class OmSlot;
class OmSolid;
class OmSolidReference;

class OmBasicJoint : public OmBaseNode {
  Q_OBJECT

public:
  virtual ~OmBasicJoint() override;

  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  virtual void prePhysicsStep(double ms) {}
  virtual void postPhysicsStep() {}
  virtual bool setJoint();
  void write(OmWriter &writer) const override;
  virtual bool resetJointPositions();
  void setMatrixNeedUpdate() override;
  virtual void updateOdeWorldCoordinates() {}
  virtual void computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const = 0;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  void updateSegmentationColor(const OmRgb &color) override;

  void setSolidEndPoint(OmSolid *solid);
  void setSolidEndPoint(OmSolidReference *solid);
  void setSolidEndPoint(OmSlot *slot);

  OmSolid *solidEndPoint() const;
  OmSolidReference *solidReference() const;
  OmSolid *solidParent() const;
  virtual dJointID jointID() const { return mJoint; }
  // endPoint Solid translation and rotation if joint position is 0
  const OmVector3 &zeroEndPointTranslation() const { return mEndPointZeroTranslation; }
  const OmRotation &zeroEndPointRotation() const { return mEndPointZeroRotation; }
  bool isEnabled() const;

  // ray tracing
  OmBoundingSphere *boundingSphere() const override;

  void updateAfterParentPhysicsChanged();
  virtual void updateEndPointZeroTranslationAndRotation() = 0;

  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;
  QString endPointName() const override;

  // P3.7 of cuda-newton-physics-plan.md: queue this joint for Newton-side
  // registration if both endpoints opt into the Newton backend. The actual
  // registry handshake is deferred to flushPendingNewtonRegistrations(),
  // which runs from OmSimulationWorld::step right before the Newton model
  // is finalised. Deferral is necessary because joint postFinalize fires
  // *during* the parent Solid's children traversal, before the parent's
  // own Newton body index has been allocated.
  static void flushPendingNewtonRegistrations();

  // P3.8.b: per-tick drive for any registered Newton hinge with a motor.
  // Reads each motor's targetVelocity() and pushes it through
  // backend->setJointTargetVelocity. Called from OmSimulationWorld::step
  // before newton->step. No-op if no motorized hinges are Newton-backed.
  static void pushNewtonMotorTargets();

  // P3.7 status accessor: 0/-1 from addJointRevolute, or -1 if not a
  // hinge or one of the endpoints isn't Newton-backed.
  int newtonJointIndex() const { return mNewtonJointIndex; }

  // OMNISIM_NEWTON_BALL_HINGE2 (value-parsed, DEFAULT OFF). Turns on the
  // MOTORISED + angle-readback layer for BallJoint and Hinge2Joint: their
  // per-axis motors reach the solver, their per-axis limits/effort ceilings are
  // built into the constraint, and their position sensors read Newton's own
  // angles instead of an ODE joint that a Newton-driven world never advances.
  // OFF keeps the passive W2 / W2.2 registration exactly as it ships -- the
  // articulation is still admitted to Newton and still constrained, it simply
  // cannot be driven and reads back through ODE.
  static bool newtonBallHinge2Enabled();

  // Newton registration hook for the MULTI-DoF joints (Hinge2 / Ball), called
  // from flushPendingNewtonRegistrations under the flag above. Implemented by
  // the subclasses because only they can reach their own axis derivation
  // (OmHinge2Joint::axis2, OmBallJoint::axis3 and their cross-product
  // fallbacks are protected) and their own per-axis motors/parameters.
  //   parentAnchor: the joint anchor in the frame of the body `parentIdx`
  //                 actually names (the merger LEADER when the parent Solid was
  //                 merged away), already resolved by the caller.
  //   childAnchor:  the same point in the child body's frame.
  //   childRelRot:  the child link's authored rotation relative to its joint
  //                 parent (R_child^T * R_parent), as in addJointRevolute.
  // Returns the Newton joint slot, or -1 when this joint type has no
  // multi-DoF registration (the base implementation).
  virtual int registerNewtonMultiDof(OmNewtonBackend *newton, int parentIdx, int childIdx,
                                     const OmVector3 &parentAnchor, const OmVector3 &childAnchor,
                                     const OmQuaternion &childRelRot);
  // True when this joint has at least one motor on any of its axes, i.e. it
  // belongs in the per-tick motor-target registry.
  virtual bool hasAnyNewtonMotor() const { return false; }
  // Per-tick push of every axis' motor command for a registered multi-DoF
  // joint. Called from pushNewtonMotorTargets under the flag.
  virtual void pushNewtonMultiDofTargets(OmNewtonBackend *newton) {}

public slots:
  void updateEndPoint();

signals:
  void endPointChanged(OmBaseNode *node);

protected:
  OmBasicJoint(const QString &modelName, OmTokenizer *tokenizer = NULL);
  OmBasicJoint(const OmBasicJoint &other);
  OmBasicJoint(const OmNode &other);

  virtual void setOdeJoint(dBodyID body, dBodyID parentBody);
  // anchor point on an hinge axis; also defined for a slider axis to set its graphical represention position
  virtual OmVector3 anchor() const;

  // Per-axis Newton actuator spec for ONE (motor, jointParameters) pair of a
  // multi-DoF joint. Shared by OmHinge2Joint and OmBallJoint so the two agree
  // with each other AND with the revolute path in
  // flushPendingNewtonRegistrations, whose long control-mode comment is the
  // authority on the ke/kd classification:
  //   position-controlled -> ke = effort*10, kd = effort*0.5 (20/3 with no
  //                          declared effort limit)
  //   velocity wheel      -> ke = 0, kd = 500
  // and whose limit precedence this mirrors: the motor's min/maxPosition first,
  // then the joint parameters' minStop/maxStop.
  // `positionByConstruction` forces the position branch for an axis that can
  // never be a free-spinning wheel (every axis of a BallJoint -- it is a 3-DoF
  // wrist, exactly as a slider is always a linear servo). With it false, an axis
  // with NO declared limits is classified as a velocity wheel, which is right for
  // a Hinge2's roll axis (a car front wheel steers on axis1 and free-spins on
  // axis2) and warns once, because setPosition on such an axis is ignored.
  // A NULL motor yields ke = kd = 0: a genuinely passive axis.
  static void newtonAxisSpec(const OmMotor *motor, const OmJointParameters *params,
                             bool positionByConstruction, double *ke, double *kd,
                             double *lower, double *upper, double *effort, double *velLimit);

  // Push ONE axis' motor command for a registered multi-DoF joint. Same three
  // modes, in the same order, as the 1-DoF path in pushNewtonMotorTargets:
  // torque mode (setForceOrTorque) -> control.joint_f for that DoF; PID position
  // control -> a position target plus a zero velocity target; otherwise a raw
  // velocity target. It also WITHDRAWS a force the controller previously set when
  // that controller leaves torque mode -- nothing else clears control.joint_f, and
  // a stale one silently overpowers the position controller that replaced it.
  static void pushNewtonAxisTarget(OmNewtonBackend *newton, int jointIdx, int dof, OmMotor *motor);

  dJointID mJoint;
  // joint attached to the static environment (internally in ODE the first body cannot be NULL)
  bool mIsReverseJoint;
  // the second end of the joint, the first being its parent; this is is either a Solid or a SolidReference
  OmSFNode *mEndPoint;
  // axis, anchor, initial position, damping and spring constants, min and max stop
  OmSFNode *mParameters;

  // variables and methods about the endPoint Solid translation and rotation when joint position is 0
  OmVector3 mEndPointZeroTranslation;
  OmRotation mEndPointZeroRotation;
  void retrieveEndPointSolidTranslationAndRotation(OmVector3 &it, OmRotation &ir) const;
  dJointID mSpringAndDamperMotor;  // ODE linear motor used to simulate spring and damper effects by means of stops
  virtual void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) = 0;

  bool mIsEndPointPositionChangedByJoint;

  // P3.7: Newton joint index in the active Model, or -1 when the joint
  // isn't (yet) registered with OmNewtonBackend (default-ODE world,
  // mismatched endpoints, non-hinge joint type, etc.).
  int mNewtonJointIndex;

  const bool isJoint() const override { return true; }

protected slots:
  virtual void updateParameters() = 0;
  void updateSpringAndDampingConstants();

private:
  OmBasicJoint &operator=(const OmBasicJoint &);  // non copyable
  void init();

private slots:
  void updateEndPointPosition();
  void updateBoundingSphere(OmBaseNode *subNode);
};
#endif
