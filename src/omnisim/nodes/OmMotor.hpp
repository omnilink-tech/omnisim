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

//
//  OmMotor.hpp
//

// Abstract class representing motors used to move mechanical joints

#ifndef OM_MOTOR_HPP
#define OM_MOTOR_HPP

#include "OmJointDevice.hpp"
#include "OmMFNode.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector3.hpp"

class OmDownloader;
class OmSensor;
class OmSoundClip;

class OmMotor : public OmJointDevice {
  Q_OBJECT

public:
  virtual ~OmMotor() override;

  // Accessors
  bool userControl() const { return mUserControl; }
  double rawInput() const { return mRawInput; }
  double acceleration() const { return mAcceleration->value(); }
  const OmVector3 &controlPID() const { return mControlPID->value(); }
  double maxForceOrTorque() const { return mMaxForceOrTorque->value(); }
  double maxVelocity() const { return mMaxVelocity->value(); }
  double minPosition() const { return mMinPosition->value(); }
  double maxPosition() const { return mMaxPosition->value(); }
  double multiplier() const { return mMultiplier->value(); }
  void setMinPosition(double position) { mMinPosition->setValue(position); }
  void setMaxPosition(double position) { mMaxPosition->setValue(position); }
  const QString &sound() const { return mSound->value(); }
  const OmSoundClip *soundClip() const { return mSoundClip; }
  double computeCurrentDynamicVelocity(double ms, double position);
  bool runKinematicControl(double ms, double &position);
  double currentVelocity() const { return mCurrentVelocity; }
  // P3.8.b: signed target velocity (rad/s for rotational, m/s for linear)
  // currently commanded by the controller via wb_motor_set_velocity, or
  // 0 if no command has been issued. Read by OmBasicJoint each physics
  // tick to drive the Newton solver's joint actuator.
  double targetVelocity() const { return mTargetVelocity; }
  // P3.10g: position target reached by the controller via
  // wb_motor_set_position. Read by OmBasicJoint when isPIDPositionControl
  // is true so the Newton motor-push can compute target velocity from
  // position error (Newton's helper module drives revolutes via a
  // velocity-actuator, not a position spring, so the PD step happens
  // outside the solver).
  double targetPosition() const { return mTargetPosition; }
  int kinematicVelocitySign() const { return mKinematicVelocitySign; }
  void setTargetPosition(double position);
  void resetPhysics();
  double energyConsumption() const override;
  void powerOn(bool) override;

  bool isPIDPositionControl() const { return (!mUserControl && mMotorForceOrTorque != 0.0 && !std::isinf(mTargetPosition)); }
  // W1.4 servo promotion: TRUE only after the CONTROLLER sent a finite
  // wb_motor_set_position over the wire (cleared by setPosition(inf), i.e.
  // wheel mode). Deliberately NOT derived from isPIDPositionControl(), which
  // is true from world load on every motor (preFinalize seeds a finite
  // target) and flips true again on reset -- neither is a controller intent.
  bool controllerCommandedFinitePosition() const { return mControllerCommandedFinitePosition; }
  bool isConfigureDone() const;

  bool hasMuscles() const { return !mMuscles->isEmpty(); }

  // inherited from OmDevice
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void writeConfigure(OmDataStream &stream) override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(OmDataStream &stream) override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  static const QList<const OmMotor *> &motors() { return cMotors; }

  // --- RESET POLICY: may a reset overwrite what the controller commanded? ---
  //
  // A reset restores the AUTHORED scene. A motor's control MODE and setpoint
  // are not authored anywhere: they are the live controller's commands, and on
  // the Newton path the ONLY in-engine record that a wheel is in velocity
  // control is `isinf(mTargetPosition)` (see isPIDPositionControl below). So
  // "restoring" a motor to its authored state does not merely reset a number,
  // it silently changes the motor's MODE -- and on a limit-less wheel, which
  // Newton registers with the velocity-wheel actuator config (ke=0, kd=500),
  // the resulting position mode makes OmBasicJoint push target VELOCITY 0 into
  // a joint with strong velocity damping. That is a hard brake.
  //
  // That is only safe when the controllers are being restarted, because then
  // every controller re-issues its commands within a few ticks. Two reset
  // paths do NOT restart controllers, by explicit design:
  //   * wb_supervisor_simulation_reset()  (OmSimulationWorld.cpp passes
  //     restartControllers=false; libController's own handler only drops the
  //     supervisor's cached field values)
  //   * wb_supervisor_node_load_state()   (no controller-restart concept)
  // and MEASURED 2026-08-12, each reproduces the freeze on its own: a rover
  // driving 7.009 m in 150 steps reads 0.000 m net and 0.000 m path in the 150
  // steps after either call, with wheel angular velocity exactly 0, while the
  // clock advances at full speed. Re-issuing setPosition(inf)/setVelocity()
  // from the controller restores 6.048 m, which is what proves it is the
  // command state and not the physics.
  //
  // So inside those cascades the command tuple below survives; everywhere else
  // this reads true and behaviour is unchanged.
  static bool resetMayOverwriteMotorCommand() { return cResetMayOverwriteCommand; }

  // RAII: set the policy for the duration of one reset cascade. Nested and
  // exception-safe, so a cascade that throws cannot leave the flag latched --
  // which would silently disable setPosition() for the rest of the session.
  class ResetPolicy {
  public:
    explicit ResetPolicy(bool mayOverwriteCommand) : mPrevious(cResetMayOverwriteCommand) {
      cResetMayOverwriteCommand = mayOverwriteCommand;
    }
    ~ResetPolicy() { cResetMayOverwriteCommand = mPrevious; }
    ResetPolicy(const ResetPolicy &) = delete;
    ResetPolicy &operator=(const ResetPolicy &) = delete;

  private:
    bool mPrevious;
  };

  void setupJointFeedback();

signals:
  void minPositionChanged();
  void maxPositionChanged();

protected:
  OmMotor(const QString &modelName, OmTokenizer *tokenizer = NULL);
  OmMotor(const OmMotor &other);
  OmMotor(const OmNode &other);

  // fields
  OmSFDouble *mMaxForceOrTorque;
  OmSFDouble *mMinPosition;
  OmSFDouble *mMaxPosition;

  virtual void turnOffMotor() = 0;
  double mMotorForceOrTorque;
  void enableMotorFeedback(int rate);
  virtual double computeFeedback() const = 0;

  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;

protected slots:
  void updateMaxForceOrTorque();
  void updateMinAndMaxPosition();

private:
  static QList<const OmMotor *> cMotors;
  // See resetMayOverwriteMotorCommand(). Default true == the pre-2026-08-12
  // behaviour, so any reset caller not explicitly scoped keeps it.
  static bool cResetMayOverwriteCommand;

  void addConfigureToStream(OmDataStream &stream);
  void inferMotorCouplings();
  void enforceMotorLimitsInsideJointLimits();
  void removeFromCoupledMotors(OmMotor *motor) { mCoupledMotors.removeAll(motor); };
  void addToCoupledMotors(OmMotor *motor);

  void checkMinAndMaxPositionAcrossCoupledMotors();
  void checkMaxVelocityAcrossCoupledMotors();
  void checkMultiplierAcrossCoupledMotors();

  // the effect of these functions depends on the current control strategy
  void setVelocity(double velocity);
  void setAcceleration(double acceleration);
  void setForceOrTorque(double forceOrTorque);
  void setAvailableForceOrTorque(double availableForceOrTorque);

  bool isPositionUnlimited() { return minPosition() == 0.0 && maxPosition() == 0.0; }

  OmMotor &operator=(const OmMotor &);  // non copyable
  void init();
  OmSFDouble *mAcceleration;
  OmSFVector3 *mControlPID;
  OmSFDouble *mConsumptionFactor;
  OmSFDouble *mMaxVelocity;
  OmSensor *mForceOrTorqueSensor;
  double mForceOrTorqueLastValue;
  OmSFString *mSound;
  OmMFNode *mMuscles;
  OmSoundClip *mSoundClip;
  double mTargetVelocity;
  double mTargetPosition;
  double mCurrentVelocity;
  double mRawInput;
  bool mUserControl;
  bool mControllerCommandedFinitePosition;
  bool mHasAllocatedJointFeedback;
  void setMaxAcceleration(double acc);
  void setMaxVelocity(double v);
  void awake() const;
  double mErrorIntegral;
  double mPreviousError;
  bool mNeedToConfigure;
  int mKinematicVelocitySign;
  QList<OmJointDevice *> mChangedAssociatedDevices;
  WbDeviceTag *mRequestedDeviceTag;
  OmDownloader *mDownloader;
  OmSFDouble *mMultiplier;
  QList<OmMotor *> mCoupledMotors;

private slots:
  void updateSound();
  void updateMaxVelocity();
  void updateMaxAcceleration();
  void updateControlPID();
  void updateMuscles();
  void updateMultiplier();
};

#endif
