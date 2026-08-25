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

// Abstract node class representing the most general mechanical joint supporting position sensor or motor

#ifndef OM_JOINT_HPP
#define OM_JOINT_HPP

#include "OmBasicJoint.hpp"

#include <QtCore/QMap>

class QString;

class OmBrake;
class OmJointDevice;
class OmJointParameters;
class OmLogicalDevice;
class OmMotor;
class OmPositionSensor;

class OmJoint : public OmBasicJoint {
  Q_OBJECT

public:
  virtual ~OmJoint() override;

  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void reset(const QString &id) override;
  virtual void resetPhysics();
  void save(const QString &id) override;
  virtual QVector<OmLogicalDevice *> devices() const;

  // R4 wgpu optional-rendering (VF_JOINT_AXES): world-space anchor + normalized
  // axis for drawing the joint-axis representation in the wgpu main view. Returns
  // false if there is no upper pose. Same transform as applyToOdeAnchor/applyToOdeAxis
  // (upperPose()->matrix() applied to the protected local anchor()/axis()).
  bool axisRepresentation(OmVector3 &worldAnchor, OmVector3 &worldAxis) const;

  OmJointParameters *parameters() const;
  virtual double position(int index = 1) const { return (index == 1) ? mPosition : NAN; }
  virtual double initialPosition(int index = 1) const { return (index == 1) ? mSavedPositions[stateId()] : NAN; }
  virtual void setPosition(double position, int index = 1);
  bool resetJointPositions() override;
  virtual OmJointParameters *parameters2() const { return NULL; }
  virtual OmJointParameters *parameters3() const { return NULL; }

  OmPositionSensor *positionSensor() const;
  OmMotor *motor() const;
  virtual OmMotor *motor2() const { return NULL; }
  virtual OmMotor *motor3() const { return NULL; }
  OmBrake *brake() const;

  OmJointDevice *device(int index) const;
  virtual int devicesNumber() const;

signals:
  void updateMuscleStretch(double forcePercentage, bool immediateUpdate, int motorIndex);

public slots:
  virtual void updatePosition() {}

protected:
  void writeExport(OmWriter &writer) const override;

  OmJoint(const QString &modelName, OmTokenizer *tokenizer = NULL);
  OmJoint(const OmJoint &other);
  OmJoint(const OmNode &other);

  void setOdeJoint(dBodyID body, dBodyID parentBody) override;
  virtual OmVector3 axis()
    const;  // return the axis of the joint with coordinates relative to the parent Solid; defaults to z-axis
  virtual void updatePosition(double position) = 0;  // position change caused by the jerk of a statically based robot

  OmMFNode *mDevice;  // JointDevices: logical position sensor device, a motor and brake, only one per type is allowed
  double mPosition;   // Keeps track of the joint position if JointParameters doesn't exist.
  QMap<QString, double> mSavedPositions;  // position loaded from a .wbt file
  double mTimeStep;  // keep track of the argument of the last call of 'prePhysicsStep' (wich is then used in 'postPhysicsStep'
                     // to update the mPosition

  // Joint position offset between ODE and Webots
  // ODE automatically resets the joint offset to 0 when setting the axis and anchor values
  // so we keep track of the current status by storing it in Webots
  double mOdePositionOffset;
  virtual void updateOdePositionOffset();

protected slots:
  virtual void addDevice(int index);
  void updateParameters() override;
  virtual void updateMinAndMaxStop(double min, double max);
  virtual void updateAxis();
  const QString urdfName() const override;

private:
  OmJoint &operator=(const OmJoint &);  // non copyable
  void init();
  virtual void applyToOdeMinAndMaxStop() = 0;
  virtual void applyToOdeAxis() = 0;
};
#endif
