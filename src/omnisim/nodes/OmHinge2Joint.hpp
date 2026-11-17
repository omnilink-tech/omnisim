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

// Implemented node class representing an hinge2 (2 DOF, rotation along two choosen intersecting axes)

#ifndef OM_HINGE_2_JOINT_HPP
#define OM_HINGE_2_JOINT_HPP

#include "OmHingeJoint.hpp"

#include <QtCore/QMap>
#include <QtCore/QStringList>

class QString;

class OmHinge2Joint : public OmHingeJoint {
  Q_OBJECT

public:
  explicit OmHinge2Joint(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmHinge2Joint(OmTokenizer *tokenizer = NULL);
  OmHinge2Joint(const OmHinge2Joint &other);
  explicit OmHinge2Joint(const OmNode &other);
  virtual ~OmHinge2Joint() override;

  void preFinalize() override;
  void postFinalize() override;
  int nodeType() const override { return WB_NODE_HINGE_2_JOINT; }
  void createWrenObjects() override;
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
  OmJointParameters *parameters2() const override;
  void computeEndPointSolidPositionFromParameters(OmVector3 &translation, OmRotation &rotation) const override;

  OmMotor *motor2() const override;
  OmPositionSensor *positionSensor2() const;
  OmBrake *brake2() const;
  OmJointDevice *device2(int index) const;
  virtual int devices2Number() const;
  void updateEndPointZeroTranslationAndRotation() override;

  // Newton multi-DoF registration + drive (OMNISIM_NEWTON_BALL_HINGE2). A
  // universal joint is registered as a native d6 with TWO independently
  // actuated angular axes; see OmBasicJoint for the contract.
  int registerNewtonMultiDof(OmNewtonBackend *newton, int parentIdx, int childIdx,
                             const OmVector3 &parentAnchor, const OmVector3 &childAnchor,
                             const OmQuaternion &childRelRot) override;
  bool hasAnyNewtonMotor() const override;
  void pushNewtonMultiDofTargets(OmNewtonBackend *newton) override;

public slots:
  bool setJoint() override;

protected:
  virtual OmVector3 axis2() const;  // return the axis of the joint with coordinates relative to the parent Solid; defaults to
                                    // the rotation axis of the solid endpoint
  // ONE warning naming every field of this joint that the Newton backend cannot
  // honour (springs/dampers, brake damping, staticFriction, suspension, stop
  // ERP/CFM). Emitted once per joint at registration -- never a silent partial
  // behaviour. `extra` lets OmBallJoint append its own (its axis limits).
  void warnUnmappedNewtonFeatures(const QStringList &extra = QStringList()) const;
  // The unmappable-field list for one (parameters, brake) pair, prefixed with
  // `axisLabel`. Shared with OmBallJoint, which has three of them.
  static QStringList unmappedNewtonFields(const OmJointParameters *params, const OmBrake *brake,
                                          const QString &axisLabel);
  OmQuaternion endPointRotation() const;
  OmRotationalMotor *rotationalMotor2() const;
  OmMFNode *mDevice2;  // JointDevices: logical position sensor device, a motor and brake, only one per type is allowed
  double mOdePositionOffset2;
  double mPosition2;                       // Keeps track of the joint position2 if JointParameters2 don't exist.
  QMap<QString, double> mSavedPositions2;
  void updatePosition(double position) override;
  void updatePositions(double position, double position2);
  void updateOdePositionOffset() override;
  void writeExport(OmWriter &writer) const override;

protected slots:
  virtual void addDevice2(int index);
  void updateParameters() override;
  void updatePosition() override;
  void updateMinAndMaxStop(double min, double max) override;

private:
  OmHinge2Joint &operator=(const OmHinge2Joint &);  // non copyable
  void updateParameters2();
  void init();
  OmSFNode *mParameters2;
  void applyToOdeAxis() override;
  void applyToOdeMinAndMaxStop() override;
};

#endif
