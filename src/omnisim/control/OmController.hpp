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

#ifndef OM_CONTROLLER_HPP
#define OM_CONTROLLER_HPP

#include <QtCore/QObject>
#include <QtCore/QProcess>
#include "OmFileUtil.hpp"
#include "OmRobot.hpp"

class QLocalServer;
class QTimer;
class QLocalSocket;
class QTcpSocket;
class QProcessEnvironment;

class OmController : public QObject {
  Q_OBJECT

public:
  // constructor & destructor
  // name: controller name as in Robot.controller, e.g. "<generic>"
  // arguments: controller arguments as in Robot.controllerArgs
  explicit OmController(OmRobot *robot);
  virtual ~OmController();

  // start the controller
  // it never fails: the <generic> controller is started as a fallback
  void start();

  void writeAnswer(bool immediateAnswer = false);
  void writePendingImmediateAnswer() {
    if (mHasPendingImmediateAnswer)
      writeImmediateAnswer();
  }
  void flushBuffers() {
    if (mStderrNeedsFlush)
      flushBuffer(&mStderrBuffer);
    if (mStdoutNeedsFlush)
      flushBuffer(&mStdoutBuffer);
  }
  bool setTcpSocket(QTcpSocket *socket);
  void addRemoteControllerConnection();
  OmRobot *robot() const { return mRobot; }
  int robotId() const;
  const QString &name() const;
  bool synchronization() const { return mRobot->synchronization(); }
  double requestTime() const { return mRequestTime; }
  void resetRequestTime();
  bool isIncompleteRequest() const { return mIncompleteRequest; }
  unsigned int deltaTimeRequested() const { return mDeltaTimeRequested; }
  bool isRequestPending() const { return mRequestPending; }
  bool isRunning() const;
  bool isProcessingRequest() const { return mProcessingRequest; }

signals:
  void hasTerminatedByItself(OmController *);
  void requestReceived();

public slots:
  void readRequest();
  void disconnected();
  void appendMessageToConsole(const QString &message, bool useStdout);
  void writeUserInputEventAnswer();
  void handleControllerExit();

private:
  OmRobot *mRobot;
  OmFileUtil::FileType mType;
  bool mExtern;
  QString mControllerPath;  // path where the controller file is located
  QString mName;            // controller name, e.g. "<generic>"
  QString mCommand;         // command to be executed, e.g. "python"
  QStringList mArguments;   // command arguments
  QString mPythonCommand;
  QString mPythonShortVersion;
  QString mPythonOptions;
  QProcess *mProcess;
  QString mIpcPath;  // path where the socket and memory mapped files are located
  QLocalServer *mServer;
  QLocalSocket *mSocket;
  QTcpSocket *mTcpSocket;
  QByteArray mRequest;
  double mRequestTime;
  bool mHasBeenTerminatedByItself;
  bool mIncompleteRequest;
  unsigned int mDeltaTimeRequested;
  double mDeltaTimeMeasured;
  bool mRequestPending;
  bool mProcessingRequest;
  bool mHasPendingImmediateAnswer;

  QString mStdoutBuffer;
  QString mStderrBuffer;
  bool mStdoutNeedsFlush;
  bool mStderrNeedsFlush;

  template<class T> void sendTerminationPacket(const T &socket, const QByteArray &buffer, const int size);

  void addToPathEnvironmentVariable(QProcessEnvironment &env, const QString &key, const QString &value, bool override,
                                    bool shouldPrepend = false);
  bool removeFromPathEnvironmentVariable(QProcessEnvironment &env, const QString &key, const QString &value);
  void setProcessEnvironment();
  void updateName(const QString &name);

  OmFileUtil::FileType findType(const QString &controllerPath);
  void startExecutable();
  void startGenericExecutable();
  void startPython();
  void startBotstudio();
  void startDocker();
  void copyBinaryAndDependencies(const QString &filename);
  void appendMessageToBuffer(const QString &message, QString *buffer);
  void flushBuffer(QString *buffer);
  QString commandLine() const;

  void prepareTcpStream(OmDataStream &stream);
  int streamSizeManagement(OmDataStream &stream);

  // IPC handshake (core-evolution-plan.md, Phase I1): write the engine hello on the freshly
  // accepted connection, wait for libController's echo, and validate magic/version/nonce.
  // Returns false on any mismatch or timeout -- a build-mismatched pair fails fast and
  // attributed instead of hanging at zero ticks. `device` is the QLocalSocket or QTcpSocket.
  bool performIpcHandshake(QIODevice *device);
  void abortMismatchedConnection();

  // Zero-tick watchdog (Phase I2): armed when an intern controller process is spawned,
  // cleared when it completes the IPC handshake or exits. If it fires, the controller
  // never paired at all -- the exact silent state that used to yield a zero-tick PASS.
  QTimer *mStartWatchdog;
  void armStartWatchdog();
  void clearStartWatchdog();

private slots:
  void addLocalControllerConnection();
  void readStdout();
  void readStderr();
  void info(const QString &message);
  void warn(const QString &message);
  void error(const QString &message);
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void processErrorOccurred(QProcess::ProcessError error);
  void reportControllerNotFound();
  void reportFailedStart();
  void reportMissingCommand(const QString &command);
  void robotDestroyed();
  void writeImmediateAnswer();
};

#endif
