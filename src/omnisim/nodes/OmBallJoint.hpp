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

// Implemented node class representing a ball-and-socket joint (3 angular DOF)

#ifndef OM_BALL_JOINT_HPP
#define OM_BALL_JOINT_HPP

class OmBallJointParameters;
class OmVector3;

#include <cassert>
#include "OmHinge2Joint.hpp"
#include "OmMatrix3.hpp"

class OmBallJoint : public OmHinge2Joint {
  Q_OBJECT

public:
  virtual ~OmBallJoint() override;
  explicit OmBallJoint(OmTokenizer *tokenizer = NULL);
  OmBallJoint(const OmBallJoint &other);
  explicit OmBallJoint(const OmNode &other);
  void preFinalize() override;
  void postFinalize() override;
  int nodeType() const override { return WB_NODE_BALL_JOINT; }
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  void reset(const QString &id) override;
  void resetPhysics() override;
  void save(const QString &id) override;
  QVector<OmLogicalDevice *> devices() const override;
  bool resetJointPositions() override;
  void setPosition(double position, int index = 1) override;
  double position(int index = 1) const override;
  double initialPosition(int index = 1) const override;
  OmBallJointParameters *ballJointParameters() const;
  OmJointParameters *parameters3() const override;
  void computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const override;
  OmMotor *motor3() const override;
  OmPositionSensor *positionSensor3() const;
  OmBrake *brake3() const;
  OmJointDevice *device3(int index) const;
  virtual int devices3Number() const;

  OmVector3 axis() const override;
  void updateEndPointZeroTranslationAndRotation() override;

  // Newton multi-DoF registration + drive (OMNISIM_NEWTON_BALL_HINGE2). A ball
  // joint is registered as a native 3-DoF spherical joint whose JOINT FRAME is
  // the (axis, axis2, axis3) triad, so its up-to-three RotationalMotors drive
  // the solver about Webots' own axes; see OmBasicJoint for the contract.
  int registerNewtonMultiDof(OmNewtonBackend *newton, int parentIdx, int childIdx,
                             const OmVector3 &parentAnchor, const OmVector3 &childAnchor,
                             const OmQuaternion &childRelRot) override;
  bool hasAnyNewtonMotor() const override;
  void pushNewtonMultiDofTargets(OmNewtonBackend *newton) override;

public slots:
  bool setJoint() override;
  void updatePosition() override;

protected:
  OmVector3 axis2() const override;
  OmVector3 axis3() const;
  // The orthonormal right-handed basis [axis | axis2 | axis3] as columns, i.e.
  // the rotation that takes a vector from the JOINT frame to the parent Solid's
  // frame. Both the Newton registration (which has to make that triad BE the
  // joint frame, because MuJoCo's ball actuators are gear vectors in the joint
  // frame) and the readback (which decomposes joint_q in it) go through this one
  // definition so they cannot drift apart. Falls back to the x/z substitution
  // ODE's applyToOdeAxis uses when the authored axes are aligned.
  OmMatrix3 newtonJointBasis() const;
  OmMFNode *mDevice3;  // JointDevices: logical position sensor device, a motor and brake, only one per type is allowed
  double mOdePositionOffset3;
  double mPosition3;  // Keeps track of the joint position3 if JointParameters3 don't exist.

  OmVector3 anchor() const override;  // defaults to the center of the Solid parent, i.e. (0, 0, 0) in relative coordinates
  void updateOdePositionOffset() override;
  void updatePosition(double position) override;
  void updatePositions(double position, double position2, double position3);
  void writeExport(OmWriter &writer) const override;

protected slots:
  void addDevice2(int index) override;
  virtual void addDevice3(int index);
  void updateParameters() override;
  void checkMotorLimit();

private:
  OmBallJoint &operator=(const OmBallJoint &);  // non copyable
  OmRotationalMotor *rotationalMotor3() const;
  void updateParameters3();
  OmSFNode *mParameters3;
  QMap<QString, double> mSavedPositions3;
  void applyToOdeAxis() override;
  void applyToOdeMinAndMaxStop() override;
  void init();
};

#endif
