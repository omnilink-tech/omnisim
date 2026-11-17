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

// Implemented node class representing an hinge (1 DOF, rotation along a choosen axis)

#ifndef OM_HINGE_JOINT_HPP
#define OM_HINGE_JOINT_HPP

#include "OmJoint.hpp"

class OmRotationalMotor;
class OmHingeJointParameters;

class OmHingeJoint : public OmJoint {
  Q_OBJECT

public:
  explicit OmHingeJoint(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmHingeJoint(OmTokenizer *tokenizer = NULL);
  OmHingeJoint(const OmHingeJoint &other);
  explicit OmHingeJoint(const OmNode &other);
  virtual ~OmHingeJoint() override;

  int nodeType() const override { return WB_NODE_HINGE_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const override;

  OmVector3 anchor() const override;
  // return the axis of the joint with coordinates relative to the parent Solid; defaults to unit x-axis
  OmVector3 axis() const override;
  void updateEndPointZeroTranslationAndRotation() override;

  // P3.8.b of cuda-newton-physics-plan.md: needed by
  // OmBasicJoint::flushPendingNewtonRegistrations + pushNewtonMotorTargets
  // so the dispatcher can detect a motor on the hinge and drive its
  // velocity target through the Newton solver each tick. Promoted from
  // protected for that explicit consumer.
  OmRotationalMotor *rotationalMotor() const;

public slots:
  bool setJoint() override;
  void updatePosition() override;

protected:
  void setOdeJoint() override;
  void updatePosition(double position) override;  // position change caused by the jerk of a statically based robot
  OmHingeJointParameters *hingeJointParameters() const;

protected slots:
  void updateParameters() override;
  void updateMinAndMaxStop(double min, double max) override;
  void updateStopErp();
  void updateStopCfm();
  virtual void updateAnchor();

private slots:
  void updateSuspension();

private:
  OmHingeJoint &operator=(const OmHingeJoint &);  // non copyable
  void init();

  void applyToOdeMinAndMaxStop() override;
  virtual void applyToOdeSuspension();
  void applyToOdeAxis() override;
  virtual void applyToOdeSuspensionAxis();
  void applyToOdeAnchor();
  void applyToOdeStopErp();
  void applyToOdeStopCfm();
};

#endif
