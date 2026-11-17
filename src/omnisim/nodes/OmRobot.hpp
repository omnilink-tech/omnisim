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

#ifndef OM_ROBOT_HPP
#define OM_ROBOT_HPP

#include "OmMFString.hpp"
#include "OmSFBool.hpp"
#include "OmSFString.hpp"
#include "OmSolid.hpp"
#include "OmVector3.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QList>
#include <QtCore/QVarLengthArray>
#include <QtCore/QVector>

class OmAbstractCamera;
class OmDataStream;
class OmDevice;
class OmJoystickInterface;
class OmKinematicDifferentialWheels;
class OmMFDouble;
class OmMouse;
class OmRenderingDevice;
class OmSensor;
class OmSupervisorUtilities;

class QByteArray;
class QDataStream;
class QTimer;

class OmRobot : public OmSolid {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmRobot(OmTokenizer *tokenizer = NULL);
  OmRobot(const OmRobot &other);
  explicit OmRobot(const OmNode &other);
  virtual ~OmRobot() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ROBOT; }
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  // controller
  void notifyExternControllerChanged();
  void newRemoteExternController();
  void removeRemoteExternController();
  bool isControllerExtern() const { return controllerName() == "<extern>"; }
  bool isControllerStarted() const { return mControllerStarted; }
  void startController();
  void setControllerStarted(bool started) {
    mControllerStarted = started;
    mControllerTerminated = false;
  }
  const QString &controllerDir();
  bool isConfigureDone() const { return !mConfigureRequest; }
  void restartController();
  void setControllerNeedRestart() { mNeedToRestartController = true; }
  bool isWaitingForUserInputEvent() const;
  bool isWaitingForWindow() const { return mWaitingForWindow; }
  void setWaitingForWindow(bool waiting);
  void addNewlyInsertedDevice(OmNode *node);
  void fixMissingResources() const override;

  // path to the project folder containing the proto model
  // returns an empty string if the robot is not a proto node
  QString protoModelProjectPath() const;

  // message dispatching
  virtual void powerOn(bool);
  bool isPowerOn() { return mPowerOn; }
  void dispatchMessage(QDataStream &);
  virtual void handleMessage(QDataStream &);
  virtual void writeAnswer(OmDataStream &);
  virtual bool hasImmediateAnswer() const;
  virtual void writeImmediateAnswer(OmDataStream &);
  void dispatchAnswer(OmDataStream &, bool includeDevices = true);
  void setConfigureRequest(bool b) { mConfigureRequest = b; }

  // device children
  int deviceCount() const { return mDevices.size(); }
  OmDevice *device(int index) const { return mDevices[index]; }
  OmDevice *findDevice(WbDeviceTag tag) const;
  void descendantNodeInserted(OmBaseNode *decendant) override;
  const QList<OmRenderingDevice *> &renderingDevices() { return mRenderingDevices; }

  // update sensors in case of no answer needs to be written at this step
  virtual void updateSensors();

  void renderCameras();

  // field accessors
  const QString &controllerName() const { return mController->value(); }
  const QStringList &controllerArgs() const { return mControllerArgs->value(); }
  const QString &customData() const { return mCustomData->value(); }
  const QString &window() const { return mWindow->value(); }
  bool synchronization() const { return mSynchronization->value(); }
  bool supervisor() const { return mSupervisor->value(); }
  const OmMFDouble &battery() const { return *mBattery; }
  bool selfCollision() const { return mSelfCollision->value(); }

  OmSupervisorUtilities *supervisorUtilities() const { return mSupervisorUtilities; }

  const bool isRobot() const override { return true; };

  // energy accessors and setters
  double currentEnergy() const;
  void setCurrentEnergy(double e);
  double maxEnergy() const;
  double energyUploadSpeed() const;

  // handle key events
  void keyPressed(int key, int modifiers);
  void keyReleased(int key);

  // map qt special key to webots special key, return 0 if not found
  static int mapSpecialKey(int qtKey);
  // return the absolute file name of the robot window file, if it exists
  QString windowFile(const QString &extension = "html") const;
  void showWindow();  // show the Qt-based controller robot window (to be deprecated)
  void updateControllerWindow();

  void processImmediateMessages();

  void setNeedToWriteUserInputEventAnswer() { mNeedToWriteUserInputEventAnswer = true; }

  OmKinematicDifferentialWheels *kinematicDifferentialWheels() { return mKinematicDifferentialWheels; }

  QString encodedName() const;  // name used for controller connections

public slots:
  void receiveFromJavascript(const QByteArray &message);
  void updateControllerDir();

signals:
  void startControllerRequest(OmRobot *robot);
  void immediateMessageAdded();
  void externControllerChanged();
  void controllerChanged();
  void controllerExited();
  void windowChanged();
  void wasReset();
  void toggleRemoteMode(bool enable);
  void sendToJavascript(const QByteArray &);
  void appendMessageToConsole(const QString &message, bool useStdout);
  void userInputEventNeedUpdate();
  void keyboardChanged();
  void windowReady();

protected:
  OmRobot(const QString &modelName, OmTokenizer *tokenizer);

  // reimplemented protected functions
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  virtual void writeConfigure(OmDataStream &);

  // export
  const QString urdfName() const override;

  OmKinematicDifferentialWheels *mKinematicDifferentialWheels;

private:
  // user accessible fields
  OmSFString *mController;
  OmMFString *mControllerArgs;
  OmSFString *mCustomData;
  OmSFBool *mSupervisor;
  OmSFBool *mSynchronization;
  OmMFDouble *mBattery;
  OmSFDouble *mCpuConsumption;
  OmSFBool *mSelfCollision;
  OmSFString *mWindow;
  OmSFString *mRemoteControl;

  bool mNeedToWriteUrdf;
  bool mShowWindowCalled;
  bool mShowWindowMessage;
  bool mUpdateWindowMessage;
  bool mWaitingForWindow;
  QByteArray *mMessageFromWwi;
  bool mDataNeedToWriteAnswer;
  bool mSupervisorNeedToWriteAnswer;
  bool mModelNeedToWriteAnswer;
  bool mPowerOn;
  bool mControllerStarted;
  bool mControllerTerminated;
  bool mNeedToRestartController;
  bool mConfigureRequest;
  bool mSimulationModeRequested;

  QString mControllerDir;

  double mPreviousTime;

  // supervisor
  bool mSupervisorUtilitiesNeedUpdate;
  OmSupervisorUtilities *mSupervisorUtilities;

  // pin
  bool mPin;
  OmVector3 mPinTranslation;
  OmRotation mPinRotation;

  // dynamic libraries
  QString mAbsoluteWindowFilename;
  QString mAbsoluteRemoteControlFilename;

  // sensors
  OmSensor *mBatterySensor;
  OmSensor *mKeyboardSensor;
  OmSensor *mJoystickSensor;
  double mBatteryLastValue;
  QMap<QString, double> mBatteryInitialValues;
  QList<int> mKeyboardLastValue;
  struct JoyStickLastValue {
    int numberOfPressedButtons;
    QList<int> pressedButtonsIndices;
    int numberOfAxes;
    QList<int> axesValues;
    int numberOfPovs;
    QList<int> povsValues;
  };
  JoyStickLastValue *mJoyStickLastValue;
  OmMouse *mMouse;

  // if sensor refresh is needed, update value and return TRUE
  bool refreshBatterySensorIfNeeded();
  bool refreshKeyboardSensorIfNeeded();
  bool refreshJoyStickSensorIfNeeded();

  // joystick interface
  OmJoystickInterface *mJoystickInterface;
  bool mJoystickConfigureRequest;
  QTimer *mJoystickTimer;

  // user input events
  QTimer *mUserInputEventTimer;
  int mMonitoredUserInputEventTypes;
  QDateTime mUserInputEventReferenceTime;
  bool mNeedToWriteUserInputEventAnswer;
  bool mKeyboardHasChanged;

  // other variables
  QList<OmDevice *> mDevices;
  QList<OmRenderingDevice *> mRenderingDevices;
  QList<OmAbstractCamera *> mActiveCameras;
  QList<OmDevice *> mNewlyAddedDevices;
  int mNextTag;

  QList<int> mPressedKeys;

  OmRobot &operator=(const OmRobot &);  // non copyable
  OmNode *clone() const override { return new OmRobot(*this); }
  void init();
  void addDevices(OmNode *node);
  // if reset is TRUE reassign tags to devices (when device config changed)
  // if reset is FALSE, only tag of newly added devices will be assigned
  void assignDeviceTags(bool reset);
  void writeDeviceConfigure(QList<OmDevice *> devices, OmDataStream &stream) const;
  QString searchDynamicLibraryAbsolutePath(const QString &key, const QString &pluginSubdirectory);
  void updateDevicesAfterInsertion();
  void updateControllerStatusInDevices();
  void pinToStaticEnvironment(bool pin);
  double energyConsumption() const;
  void clearDevices();
  int computeSimulationMode();

private slots:
  void updateDevicesAfterDestruction();
  void updateActiveCameras(OmAbstractCamera *camera, bool isActive);
  void updateWindow();
  void updateRemoteControl();
  void updateSimulationMode();
  void updateData();
  void updateSupervisor();
  void updateModel();
  void updateBattery(bool itemInserted);
  void removeRenderingDevice();
  void handleMouseChange();
  void handleJoystickChange();
};

#endif
