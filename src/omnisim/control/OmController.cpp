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

#include "OmController.hpp"

#include "OmApplicationInfo.hpp"
#include "OmBinaryIncubator.hpp"
#include "OmControlledWorld.hpp"
#include "OmDataStream.hpp"
#include "OmFileUtil.hpp"
#include "OmIniParser.hpp"
#include "OmLanguage.hpp"
#include "OmLanguageTools.hpp"
#include "OmLog.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmRobot.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"
#include "OmTemplateManager.hpp"
#include "OmVersion.hpp"

#include <QtCore/QByteArray>
#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QProcess>
#include <QtCore/QProcessEnvironment>
#include <QtCore/QTimer>
#include <QtCore/QUrl>
#include <QtNetwork/QLocalServer>
#include <QtNetwork/QLocalSocket>
#include <QtNetwork/QTcpSocket>

#include <cassert>
#include <iostream>
#include "../../controller/c/messages.h"

#ifdef _WIN32
#include <windows.h>
#endif

/*
#include <iomanip>
#include <iostream>

static const int maxNumberOfByteDisplayed = 100;

static void printArray(const QByteArray &buffer, const QString &prefix, int id, bool hex, bool ascii) {
  int bufferSize = buffer.size();
  std::cout << prefix.toUtf8().constData() << " at time " << OmSimulationState::instance()->time();
  std::cout << " (id = " << id << ", size = " << bufferSize << ")" << std::endl;
  int numberOfByteDisplayed = bufferSize < maxNumberOfByteDisplayed ? bufferSize : maxNumberOfByteDisplayed;
  if (hex) {
    std::cout << " (Hex) : \"";
    for (int i = 0; i < numberOfByteDisplayed; i++) {
      std::cout << std::uppercase << std::setfill('0') << std::setw(2) << std::hex;
      std::cout << (int)(buffer[i] & 0xFF);
      std::cout << std::nouppercase << std::setfill(' ') << std::setw(0) << std::dec;
      std::cout << ';';
    }
    if (bufferSize > numberOfByteDisplayed)
      std::cout << "...";
    std::cout << "\"" << std::endl;
  }
  if (ascii) {
    std::cout << " (ASCII) : \"";
    for (int i = 0; i < numberOfByteDisplayed; i++) {
      char c = (int)(buffer[i] & 0xFF);
      if (c >= ' ' && c <= '~')
        std::cout << c;
      else
        std::cout << '?';
    }
    if (bufferSize > numberOfByteDisplayed)
      std::cout << "...";
    std::cout << "\"" << std::endl;
  }
}
*/

OmController::OmController(OmRobot *robot) {
  mRobot = robot;
  mControllerPath = mRobot->controllerDir();
  updateName(mRobot->controllerName());

  mType = OmFileUtil::UNKNOWN;
  mExtern = mRobot->controllerName() == "<extern>";
  mServer = NULL;
  mSocket = NULL;
  mTcpSocket = NULL;
  mProcess = NULL;
  mStartWatchdog = NULL;
  mRequestTime = 0.0;
  mDeltaTimeRequested = 0;
  mDeltaTimeMeasured = 0.0;
  mHasBeenTerminatedByItself = false;
  mIncompleteRequest = false;
  mRequestPending = false;
  mProcessingRequest = false;
  mHasPendingImmediateAnswer = false;
  mStdoutNeedsFlush = false;
  mStderrNeedsFlush = false;

  connect(mRobot, &OmRobot::controllerExited, this, &OmController::handleControllerExit);
  connect(mRobot, &OmRobot::immediateMessageAdded, this, &OmController::writeImmediateAnswer);
  connect(mRobot, &OmRobot::userInputEventNeedUpdate, this, &OmController::writeUserInputEventAnswer);
  connect(mRobot, &OmRobot::appendMessageToConsole, this, &OmController::appendMessageToConsole);
  connect(mRobot, &OmRobot::destroyed, this, &OmController::robotDestroyed);
}

OmController::~OmController() {
  // disconnect everything in order to make sure
  // that this function is the last one to be called
  // exception: don't disconnect readyReadStandard*()
  // signals in order to see the latest log messages
  if (mRobot)
    disconnect(mRobot);
  if (mProcess)
    mProcess->disconnect(this);

  QByteArray buffer;
  QDataStream stream(&buffer, QIODevice::WriteOnly);
  stream.setByteOrder(QDataStream::LittleEndian);
  if (mSocket) {
    const int size = 2 * sizeof(int) + sizeof(WbDeviceTag) + sizeof(unsigned char);
    stream << size;               // size, to be overwritten afterwards
    stream << (int)0;             // time stamp, ignored
    stream << (unsigned short)0;  // tag of the root device
    stream << (unsigned char)C_ROBOT_QUIT;
    assert(size == buffer.size());
    sendTerminationPacket(mSocket, buffer, size);

  } else if (mTcpSocket) {
    const int dataSize = sizeof(int) + sizeof(WbDeviceTag) + sizeof(unsigned char);
    const int size = sizeof(unsigned short) + 2 * sizeof(int) + sizeof(char) + dataSize;
    stream << (unsigned short)1;    // number of chunks
    stream << dataSize;             // dataSize, overall
    stream << dataSize;             // dataSize, this chunk
    stream << (char)TCP_DATA_TYPE;  // chunk type
    stream << (int)0;               // time stamp, ignored
    stream << (unsigned short)0;    // tag of the root device
    stream << (unsigned char)C_ROBOT_QUIT;
    assert(size == buffer.size());
    if (mRobot)
      mRobot->removeRemoteExternController();
    if (!mHasBeenTerminatedByItself)
      sendTerminationPacket(mTcpSocket, buffer, size);
  } else if (mProcess && mProcess->state() != QProcess::NotRunning) {
    mProcess->terminate();
    mProcess->deleteLater();
    mProcess = NULL;
  }

  if (mExtern && mRobot) {
    info(tr("disconnected."));
    OmControlledWorld::instance()->externConnection(this, false);
  }

  delete mProcess;
  delete mSocket;
  if (!mHasBeenTerminatedByItself)
    delete mTcpSocket;
  delete mServer;
  if (!mIpcPath.isEmpty())
    QDir(mIpcPath).removeRecursively();
}

template<class T> void OmController::sendTerminationPacket(const T &socket, const QByteArray &buffer, const int size) {
  socket->disconnect();
  // eat the latest messages from the controller
  if (!mRequestPending && socket->isValid()) {
    socket->waitForReadyRead(1000);
    socket->readAll();
  }

  // send the termination packet
  if (socket->isValid()) {
    socket->write(buffer.constData(), size);
    socket->flush();  // otherwise the temination packet is not sent
  }
  // kill the process
  if (mProcess && mProcess->state() != QProcess::NotRunning && !mProcess->waitForFinished(1000)) {
    OmLog::warning(tr("%1: Forced termination (because process didn't terminate itself after 1 second).").arg(name()));
#ifdef _WIN32
    // on Windows, we need to kill the process as it may not handle the WM_CLOSE message sent by terminate()
    mProcess->kill();
#else
    mProcess->terminate();  // on Linux and macOS, we assume the controller will quit on receiving the SIGTERM signal
#endif
  }
}

void OmController::updateName(const QString &name) {
  mName = name;
}

bool OmController::setTcpSocket(QTcpSocket *socket) {
  if (mSocket || mTcpSocket) {  // already connected, refusing
    info(tr("refusing connection attempt from another extern controller."));
    return false;
  }
  const QHostAddress hostAddress(socket->peerAddress().toIPv4Address());
  const int nAllowedIPs = OmPreferences::instance()->value("Network/nAllowedIPs").toInt();
  if (!nAllowedIPs) {  // Empty list
    mTcpSocket = socket;
    return true;
  }
  for (int i = 0; i < nAllowedIPs; i++) {
    const QString ipKey = "Network/allowedIP" + QString::number(i);
    const QString ipString = OmPreferences::instance()->value(ipKey).toString();
    const QStringList ipParts = ipString.split('/');
    const QHostAddress subnet(ipParts[0]);
    const int netmask = ipParts.length() == 2 ? ipParts[1].toInt() : 32;
    if (hostAddress.isInSubnet(subnet, netmask)) {
      mTcpSocket = socket;
      return true;
    }
  }
  return false;
}

void OmController::resetRequestTime() {
  mRequestTime = OmSimulationState::instance()->time();
}

bool OmController::isRunning() const {
  return mRobot->isControllerStarted() && !mHasBeenTerminatedByItself;
}

// the start() method  never fails: if the controller name is invalid, then the <generic> controller starts instead.
void OmController::start() {
  mRobot->setControllerStarted(true);
  if (mExtern) {
    QString message;
    if (mRobot->encodedName() == mRobot->name())
      message = tr("Waiting for local or remote connection on port %1 targeting robot named '%2'.")
                  .arg(QString::number(OmStandardPaths::webotsTmpPathId()))
                  .arg(mRobot->name());
    else
      message = tr("Waiting for local or remote connection on port %1 targeting robot named '%2' (%3).")
                  .arg(QString::number(OmStandardPaths::webotsTmpPathId()))
                  .arg(mRobot->name())
                  .arg(mRobot->encodedName());

    info(message);
    OmControlledWorld::instance()->externConnection(this, false);
    if (OmWorld::printExternUrls()) {
      const QString localUrl = "ipc://" + QString::number(OmStandardPaths::webotsTmpPathId()) + '/' + mRobot->encodedName();
      const QString remoteUrl =
        "tcp://<ip_address>:" + QString::number(OmStandardPaths::webotsTmpPathId()) + '/' + mRobot->encodedName();
      std::cout << localUrl.toUtf8().constData() << std::endl;
      std::cout << remoteUrl.toUtf8().constData() << std::endl;
    }
  } else {
    mProcess = new QProcess();
    connect(mProcess, &QProcess::readyReadStandardOutput, this, &OmController::readStdout);
    connect(mProcess, &QProcess::readyReadStandardError, this, &OmController::readStderr);
    connect(mProcess, &QProcess::finished, this, &OmController::processFinished);
    connect(mProcess, &QProcess::errorOccurred, this, &OmController::processErrorOccurred);
    if (mControllerPath.isEmpty()) {
      warn(tr("Could not find the controller directory.\nStarting the <generic> controller instead."));
      startGenericExecutable();
    }
    mType = findType(mControllerPath);
    setProcessEnvironment();
    switch (mType) {
      case OmFileUtil::EXECUTABLE:
        (name() == "<generic>") ? startGenericExecutable() : startExecutable();
        break;
      case OmFileUtil::PYTHON:
        startPython();
        break;
      case OmFileUtil::BOTSTUDIO:
        startBotstudio();
        break;
      case OmFileUtil::DOCKER:
        startDocker();
        break;
      default:
        reportControllerNotFound();
        startGenericExecutable();
        mType = OmFileUtil::EXECUTABLE;
    }
  }

  mIpcPath = OmStandardPaths::webotsTmpPath() + "ipc/" + mRobot->encodedName();
  QDir().mkpath(mIpcPath);
  const QString fileName = mIpcPath + '/' + (mExtern ? "extern" : "intern");
#ifndef _WIN32
  const QString &serverName = fileName;
#else
  // A per-launch nonce (the simulator PID) uniquifies the pipe name so back-to-back headless
  // launches that reuse the same TCP port (default 1234 -> identical webotsTmpPathId) don't collide on
  // one named pipe: Windows allows multiple server instances of a name, so without the nonce a fresh
  // child could CreateFile(OPEN_EXISTING) onto the PREVIOUS launch's lingering pipe instance and cross
  // the pairing -> "no result". This is the residual launch-flake race (default-flip-plan.md §3.5). An
  // intern child receives the nonce via OMNISIM_IPC_NONCE (setProcessEnvironment); an EXTERN controller
  // -- user-launched, no inherited environment -- reads it from the "ipc-nonce" rendezvous file written
  // below into this instance's port-salted tmp dir (Phase I3, core-evolution-plan.md), so extern pipes
  // are nonce-protected too. An older libController computes the legacy nonce-less name, never finds
  // the pipe, and hits its loud connect diagnostic instead of silently pairing with a stale instance.
  const QString serverName = "webots-" + QString::number(OmStandardPaths::webotsTmpPathId()) + "-" +
                             QString::number(QCoreApplication::applicationPid()) + "-" + mRobot->encodedName();
  // rendezvous file for extern controllers (idempotent: same content for every controller of this launch)
  QFile nonceFile(OmStandardPaths::webotsTmpPath() + "ipc-nonce");
  if (nonceFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    nonceFile.write(QString::number(QCoreApplication::applicationPid()).toUtf8());
    nonceFile.close();
  } else
    OmLog::warning(tr("Cannot write the IPC nonce rendezvous file in '%1': extern controllers built before the "
                      "nonce-protected pipe naming will not find this instance.")
                     .arg(OmStandardPaths::webotsTmpPath()));
  // create an empty file, so that the controllers can see an extern controller is available here
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    OmLog::error(tr("Cannot create empty extern file in '%1'.").arg(fileName));
    return;
  }
  file.write("");
  file.close();
#endif
  // recover from a crash, when the previous server instance has not been cleaned up
  bool success = QLocalServer::removeServer(serverName);
  if (!success) {
    OmLog::error(tr("Cannot cleanup the local server (server name = '%1').").arg(serverName));
    return;
  }
  // create a new socket server to get connected with the controller process
  mServer = new QLocalServer();
  connect(mServer, &QLocalServer::newConnection, this, &OmController::addLocalControllerConnection);
  success = mServer->listen(serverName);
  if (!success) {
    OmLog::error(tr("Cannot listen to the local server (server name = '%1'): %2").arg(serverName).arg(mServer->errorString()));
    return;
  }
  if (mProcess) {
    info(tr("Starting controller: %1").arg(commandLine()));
    mProcess->setWorkingDirectory(mControllerPath);
    mProcess->start(mCommand, mArguments);
    armStartWatchdog();
  }
}

// Zero-tick watchdog (core-evolution-plan.md, Phase I2). An intern controller that is
// spawned but never completes the IPC handshake leaves the engine waiting forever with
// the sim clock at t=0 -- historically an invisible state (no ERROR line, headless PASS).
// The watchdog turns it into an attributed WARNING the headless runner treats as fatal.
// It never kills anything: a slow cold start (heavy Python imports, WSL2 disk) may pair
// late and legitimately, so firing only reports. Tunable via OMNISIM_CONTROLLER_START_TIMEOUT_S.
void OmController::armStartWatchdog() {
  if (mExtern)
    return;  // extern controllers legitimately wait indefinitely for a user-launched process
  int timeoutS = qEnvironmentVariableIntValue("OMNISIM_CONTROLLER_START_TIMEOUT_S");
  if (timeoutS <= 0)
    timeoutS = 60;
  if (!mStartWatchdog) {
    mStartWatchdog = new QTimer(this);
    mStartWatchdog->setSingleShot(true);
    connect(mStartWatchdog, &QTimer::timeout, this, [this, timeoutS]() {
      if (mSocket || mTcpSocket)
        return;  // paired in the meantime
      OmLog::warning(tr("Controller '%1': started %2 seconds ago but never paired with the simulator (no IPC "
                        "connection). The robot has executed ZERO simulation steps -- this run's results for it are "
                        "void. Likely causes: the controller process crashed before wb_robot_init() (check its own "
                        "log), a build-mismatched libController (run 'python -m omnisim doctor'), or a very slow "
                        "cold start (raise OMNISIM_CONTROLLER_START_TIMEOUT_S).")
                       .arg(name())
                       .arg(timeoutS));
    });
  }
  mStartWatchdog->start(timeoutS * 1000);
}

void OmController::clearStartWatchdog() {
  if (mStartWatchdog)
    mStartWatchdog->stop();
}

// The engine's 16-byte IPC hello (frame layout in controller/c/messages.h): magic,
// protocol version, flags, and the launch nonce (our PID -- the same value exported
// to intern controllers as OMNISIM_IPC_NONCE).
static QByteArray ipcHelloFrame() {
  QByteArray hello(OMNISIM_IPC_HELLO_SIZE, '\0');
  memcpy(hello.data(), OMNISIM_IPC_MAGIC, 4);
  hello[4] = (char)(OMNISIM_IPC_VERSION & 0xff);
  hello[5] = (char)((OMNISIM_IPC_VERSION >> 8) & 0xff);
  const quint64 nonce = (quint64)QCoreApplication::applicationPid();
  for (int i = 0; i < 8; ++i)
    hello[8 + i] = (char)((nonce >> (8 * i)) & 0xff);
  return hello;
}

static int ipcHandshakeTimeoutMs() {
  bool ok = false;
  const int configured = qEnvironmentVariableIntValue("OMNISIM_IPC_HANDSHAKE_TIMEOUT_MS", &ok);
  if (ok && configured >= 1000 && configured <= 300000)
    return configured;
  return OMNISIM_IPC_HANDSHAKE_DEFAULT_TIMEOUT_MS;
}

static bool hasWrongIpcMagic(QIODevice *device) {
  return device->bytesAvailable() >= 4 && device->peek(4) != QByteArray(OMNISIM_IPC_MAGIC, 4);
}

bool OmController::performIpcHandshake(QIODevice *device) {
  const QByteArray hello = ipcHelloFrame();
  device->write(hello);
  device->waitForBytesWritten(1000);

  const int timeoutMs = ipcHandshakeTimeoutMs();
  QElapsedTimer timer;
  timer.start();
  while (device->bytesAvailable() < OMNISIM_IPC_HELLO_SIZE && timer.elapsed() < timeoutMs) {
    // A legacy controller starts with a request packet, whose first four bytes cannot be
    // "OMSH". Reject it as soon as the prefix arrives instead of consuming the generous
    // large-fleet startup allowance below.
    if (hasWrongIpcMagic(device)) {
      error(tr("libController sent unrecognized bytes where the OmniSim IPC handshake echo was expected: it predates the "
               "handshake protocol or is a different build than this simulator (a build mismatch -- the former zero-tick "
               "silent hang, now failed fast). Rebuild libController and run 'python -m omnisim doctor'."));
      return false;
    }
    device->waitForReadyRead(100);
  }
  if (device->bytesAvailable() < OMNISIM_IPC_HELLO_SIZE) {
    // A pre-handshake libController sends its first request packet instead of an echo; it is
    // still bytes, so the distinction is made below on the magic. Reaching THIS branch means
    // not even OMNISIM_IPC_HELLO_SIZE bytes arrived -- treat it the same way.
    error(tr("libController did not complete the OmniSim IPC handshake within %1 seconds. The controller process may be "
             "overloaded, may predate the handshake protocol, or may be a different build than this simulator. Run "
             "'python -m omnisim doctor' to verify compatibility; for a known-compatible large launch, "
             "OMNISIM_IPC_HANDSHAKE_TIMEOUT_MS can raise the deadline.")
            .arg(timeoutMs / 1000.0, 0, 'g', 3));
    return false;
  }
  const QByteArray echo = device->read(OMNISIM_IPC_HELLO_SIZE);
  if (memcmp(echo.constData(), OMNISIM_IPC_MAGIC, 4) != 0) {
    error(tr("libController sent unrecognized bytes where the OmniSim IPC handshake echo was expected: it predates the "
             "handshake protocol or is a different build than this simulator (a build mismatch -- the former zero-tick "
             "silent hang, now failed fast). Rebuild libController and run 'python -m omnisim doctor'."));
    return false;
  }
  const quint16 libVersion = (quint8)echo[4] | ((quint16)(quint8)echo[5] << 8);
  if (libVersion != OMNISIM_IPC_VERSION) {
    error(tr("OmniSim IPC protocol version mismatch: this simulator speaks version %1 but libController speaks version "
             "%2 (a build mismatch). Rebuild engine and libController from the same tree; run 'python -m omnisim "
             "doctor' to verify.")
            .arg(OMNISIM_IPC_VERSION)
            .arg(libVersion));
    return false;
  }
  quint64 echoNonce = 0;
  for (int i = 7; i >= 0; --i)
    echoNonce = (echoNonce << 8) | (quint8)echo[8 + i];
  if (echoNonce != (quint64)QCoreApplication::applicationPid()) {
    error(tr("OmniSim IPC handshake echo carried a foreign launch nonce (%1, expected %2): the controller is paired "
             "with a different simulator instance (a stale pipe crossing). Closing the connection.")
            .arg(echoNonce)
            .arg(QCoreApplication::applicationPid()));
    return false;
  }
  return true;
}

void OmController::abortMismatchedConnection() {
  // The handshake failed: close the transport so the peer's blocking read fails loudly, and
  // kill an intern process outright -- it can never pair with this engine, and leaving it
  // alive would recreate the very retry-forever hang the handshake exists to eliminate.
  if (mSocket) {
    mSocket->close();
    mSocket->deleteLater();
    mSocket = NULL;
  }
  if (mTcpSocket) {
    mTcpSocket->close();
    mTcpSocket = NULL;  // owned by OmTcpServer's socket lifecycle, do not delete here
  }
  if (mProcess && !mExtern)
    mProcess->kill();
}

void OmController::addLocalControllerConnection() {
  if (mSocket || mTcpSocket) {  // already connected, refusing
    mServer->nextPendingConnection()->close();
    info(tr("refusing connection attempt from another extern controller."));
    return;
  }
  mSocket = mServer->nextPendingConnection();

  // IPC handshake (I1) -- MUST complete before the connection is trusted or announced.
  if (!performIpcHandshake(mSocket)) {
    abortMismatchedConnection();
    return;
  }
  clearStartWatchdog();  // paired: the zero-tick-silence window is over (I2)

  if (mExtern) {
    info(tr("connected."));
    OmControlledWorld::instance()->externConnection(this, true);
  }
  mRobot->setConfigureRequest(true);

  // wb_robot_init performs a wb_robot_step(0) generating a request which has to be catch.
  // This request is forced because the first packets coming from libController
  // may be splitted (wb_robot_init() sends firstly the robotId and the robot_step(0) package which have to be eaten there)
  while (mSocket->bytesAvailable() == 0)
    mSocket->waitForReadyRead();
  readRequest();
  connect(mSocket, &QLocalSocket::readyRead, this, &OmController::readRequest);
  connect(mSocket, &QLocalSocket::disconnected, this, &OmController::disconnected);
  writeAnswer();  // send configure message and immediate answers if any
}

void OmController::addRemoteControllerConnection() {
  // IPC handshake (I1) -- MUST complete before the connection is trusted or announced.
  if (!performIpcHandshake(mTcpSocket)) {
    abortMismatchedConnection();
    return;
  }

  info(tr("connected."));
  OmControlledWorld::instance()->externConnection(this, true);
  mRobot->newRemoteExternController();
  mRobot->setConfigureRequest(true);

  // wb_robot_init performs a wb_robot_step(0) generating a request which has to be catch.
  // This request is forced because the first packets coming from libController
  // may be splitted (wb_robot_init() sends firstly the robotId and the robot_step(0) package which have to be eaten there)
  while (mTcpSocket->bytesAvailable() == 0)
    mTcpSocket->waitForReadyRead();
  readRequest();
  connect(mTcpSocket, &QTcpSocket::readyRead, this, &OmController::readRequest);
  connect(mTcpSocket, &QTcpSocket::disconnected, this, &OmController::disconnected);
  writeAnswer();  // send configure message and immediate answers if any
}

void OmController::addToPathEnvironmentVariable(QProcessEnvironment &env, const QString &key, const QString &value,
                                                bool override, bool shouldPrepend) {
  const QString nativeValue(QDir::toNativeSeparators(value));
  if (!env.contains(key) || override) {  // key is the name of the environment variable
    env.insert(key, nativeValue);
    return;
  }
  const QString &previousValue = env.value(key);
  if (!previousValue.split(QDir::listSeparator()).contains(nativeValue)) {
    if (shouldPrepend)
      env.insert(key, nativeValue + QDir::listSeparator() + previousValue);
    else
      env.insert(key, previousValue + QDir::listSeparator() + nativeValue);
  }
}

bool OmController::removeFromPathEnvironmentVariable(QProcessEnvironment &env, const QString &key, const QString &value) {
  const QString path = env.value(key);
  QStringList paths = path.split(QDir::listSeparator());
#ifdef _WIN32
  Qt::CaseSensitivity cs = Qt::CaseInsensitive;
#else
  Qt::CaseSensitivity cs = Qt::CaseSensitive;
#endif
  if (!paths.contains(value, cs))
    return false;
  env.remove(key);
  paths.removeAll(QString(""));
  paths.removeDuplicates();
  QMutableStringListIterator i(paths);
  while (i.hasNext()) {
    if (i.next().compare(value, cs) == 0)
      i.remove();
  }
  env.insert(key, paths.join(';'));
  return true;
}

void OmController::setProcessEnvironment() {
#ifdef __linux__
  static const QString ldEnvironmentVariable("LD_LIBRARY_PATH");
#elif defined(__APPLE__)
  static const QString ldEnvironmentVariable("DYLD_LIBRARY_PATH");
#else  // _WIN32
  static const QString ldEnvironmentVariable("PATH");
#endif

  // starts from the parent process environment
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

  // store a unique robot name for the controller
  // OMNISIM_ROBOT_NAME is the ONLY name written: the legacy WEBOTS_ROBOT_NAME twin was retired
  // together with libController's fallback read of it (robot.c), so both halves of the
  // engine<->controller rendezvous moved in lockstep. A libController older than that change
  // reads only the legacy name and will not find its robot -- rebuild it (`python -m omnisim
  // doctor` reports the mismatch).
  env.insert("OMNISIM_ROBOT_NAME", mRobot->name());
  // A stale legacy value inherited from the parent shell must not shadow the canonical one for a
  // controller that still consults it (a third-party binding, say): drop it rather than leave two
  // disagreeing spellings in the child environment.
  env.remove("WEBOTS_ROBOT_NAME");

  // Add the OmniSim lib path to be able to load (at least) libController
  QString ldLibraryPath = OmStandardPaths::controllerLibPath();
  ldLibraryPath.chop(1);
  addToPathEnvironmentVariable(env, ldEnvironmentVariable, ldLibraryPath, false, true);
  // Remove paths needed by OmniSim only
#ifdef _WIN32
  const QString msys64 = QDir::toNativeSeparators(OmStandardPaths::omniSimMsys64Path());
  removeFromPathEnvironmentVariable(env, ldEnvironmentVariable, msys64 + "mingw64\\bin");
  removeFromPathEnvironmentVariable(env, ldEnvironmentVariable, msys64 + "usr\\bin");
#else
  ldLibraryPath = OmStandardPaths::omniSimLibPath();
  ldLibraryPath.chop(1);
  removeFromPathEnvironmentVariable(env, ldEnvironmentVariable, ldLibraryPath);
  // add the controller path in the PATH-like environment variable
  // in order to be able to add easily dynamic libraries there
  // Note: on windows, this is the default behavior
  ldLibraryPath = mControllerPath;
  ldLibraryPath.chop(1);
  addToPathEnvironmentVariable(env, ldEnvironmentVariable, ldLibraryPath, false, true);
#endif

  if (QFile::exists(mControllerPath + "runtime.ini")) {
    OmIniParser iniParser(mControllerPath + "runtime.ini");
    if (!iniParser.isValid())
      warn(tr("Environment variables from runtime.ini could not be loaded: the file contains illegal definitions."));
    else {
      for (int i = 0; i < iniParser.size(); ++i) {
        const QString &value = iniParser.resolvedValueAt(i, env);
        iniParser.setValue(i, value);
        if (iniParser.sectionAt(i) == "environment variables with relative paths")
          warn(
            "[environment variables with relative path] is deprecated, please use [environment variables with path] instead");
        if (iniParser.sectionAt(i) == "environment variables with relative paths" ||
            iniParser.sectionAt(i) == "environment variables with paths")
          addToPathEnvironmentVariable(env, iniParser.keyAt(i), iniParser.valueAt(i), true);
        if (iniParser.sectionAt(i) == "environment variables")
          env.insert(iniParser.keyAt(i), iniParser.valueAt(i));
        if (iniParser.sectionAt(i) == "python") {
          if (iniParser.keyAt(i) == "COMMAND")
            mPythonCommand = OmLanguageTools::pythonCommand(mPythonShortVersion, iniParser.valueAt(i), env);
          else if (iniParser.keyAt(i) == "OPTIONS")
            mPythonOptions = iniParser.valueAt(i);
          else
            OmLog::warning(tr("Unknown key: %1 in python section").arg(iniParser.keyAt(i)));
        }
#ifdef _WIN32
        if (iniParser.sectionAt(i) == "environment variables for windows")
          addToPathEnvironmentVariable(env, iniParser.keyAt(i), iniParser.valueAt(i), true);
#elif defined(__APPLE__)
        if (iniParser.sectionAt(i) == "environment variables for mac os x" ||
            iniParser.sectionAt(i) == "environment variables for macos")
          addToPathEnvironmentVariable(env, iniParser.keyAt(i), iniParser.valueAt(i), true);
#else
        if (iniParser.sectionAt(i) == "environment variables for linux")
          addToPathEnvironmentVariable(env, iniParser.keyAt(i), iniParser.valueAt(i), true);
        if (!OmSysInfo::isPointerSize64bits()) {
          if (iniParser.sectionAt(i) == "environment variables for linux 32")
            addToPathEnvironmentVariable(env, iniParser.keyAt(i), iniParser.valueAt(i), true);
        } else {
          if (iniParser.sectionAt(i) == "environment variables for linux 64")
            addToPathEnvironmentVariable(env, iniParser.keyAt(i), iniParser.valueAt(i), true);
        }
#endif
      }
    }
  }
  // OMNISIM_LIBRARY_PATH is an environment variable that users can edit to prepend paths to the
  // library path. The legacy WEBOTS_LIBRARY_PATH spelling is no longer read; a setup still using it
  // is told so explicitly rather than silently losing its extra library paths.
  if (env.contains("OMNISIM_LIBRARY_PATH"))
    addToPathEnvironmentVariable(env, ldEnvironmentVariable, env.value("OMNISIM_LIBRARY_PATH"), false, true);
  else if (env.contains("WEBOTS_LIBRARY_PATH"))
    warn(tr("WEBOTS_LIBRARY_PATH is set but is no longer read: rename it to OMNISIM_LIBRARY_PATH. "
            "Its paths were NOT added to the controller's library search path."));

  // Add all the libraries subdirectories to the environment
  QStringList librariesSearchPaths;
  if (QFile::exists(OmProject::current()->librariesPath()))
    librariesSearchPaths << OmProject::current()->librariesPath();

  if (mRobot->isProtoInstance()) {
    // search in project folder associated with PROTO instance
    const QString protoLibrariesPath = mRobot->protoModelProjectPath() + "/libraries/";
    if (QDir(protoLibrariesPath).exists())
      librariesSearchPaths << protoLibrariesPath;
  }

  QList<OmNode *> nodes = mRobot->subNodes(true, true, true);
  for (int i = 0; i < nodes.size(); ++i) {
    if (nodes.at(i)->isProtoInstance()) {
      const OmProtoModel *protoModel = nodes.at(i)->proto();
      do {
        if (!protoModel->projectPath().isEmpty()) {
          QDir protoProjectDir(protoModel->projectPath());
          const QString protoLibrariesPath = protoProjectDir.absolutePath() + "/libraries/";
          if (QDir(protoLibrariesPath).exists())
            librariesSearchPaths << protoLibrariesPath;
        }
        protoModel = OmProtoManager::instance()->findModel(protoModel->ancestorProtoName(), "", protoModel->diskPath());
      } while (protoModel);
    }
  }

  foreach (const QString &librariesSearchPath, librariesSearchPaths) {
    const QDir dir(librariesSearchPath);
    const QStringList subDirectories = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    foreach (const QString &subDirectory, subDirectories)
      addToPathEnvironmentVariable(env, ldEnvironmentVariable, librariesSearchPath + subDirectory, false, true);
  }
  if (mType == OmFileUtil::PYTHON) {
    if (mPythonCommand.isEmpty())
      mPythonCommand = OmLanguageTools::pythonCommand(
        mPythonShortVersion, OmPreferences::instance()->value("General/pythonCommand", "python").toString(), env);
    // read the python shebang (first line starting with #!) to possibly override the python command
    QFile pythonSourceFile(mControllerPath + name() + ".py");
    if (pythonSourceFile.open(QIODevice::ReadOnly)) {
      QTextStream in(&pythonSourceFile);
      const QString &line = in.readLine();
      if (line.startsWith("#!")) {
#ifndef _WIN32
        if (line.startsWith("#!/usr/bin/env "))
          mPythonCommand = OmLanguageTools::pythonCommand(mPythonShortVersion, line.mid(15).trimmed(), env);
        else
          mPythonCommand = OmLanguageTools::pythonCommand(mPythonShortVersion, line.mid(2).trimmed(), env);
#else  // Windows: check that the version specified in the shebang corresponds to the version of Python installed
        const QString &expectedVersion = line.mid(line.lastIndexOf("python", -1, Qt::CaseInsensitive) + 6);
        bool mismatch = false;
        int l = expectedVersion.length();
        if (l == 1 && expectedVersion[0] != mPythonShortVersion[0])
          mismatch = true;
        if (l >= 3 && (expectedVersion[0] != mPythonShortVersion[0] || expectedVersion[2] != mPythonShortVersion[1]))
          mismatch = true;
        if (mismatch)
          warn(tr("Python shebang requests python%1, but current path points to Python%2")
                 .arg(expectedVersion, mPythonShortVersion));
#endif
      }
      pythonSourceFile.close();
    }
    addToPathEnvironmentVariable(env, "PYTHONPATH", OmStandardPaths::controllerLibPath() + "python", false, true);
    env.insert("PYTHONIOENCODING", "UTF-8");
  }
  // OMNISIM_INSTANCE_PATH is the ONLY name written; libController reads exactly this name
  // (wbu_system_webots_instance_path in system.c). The legacy WEBOTS_INSTANCE_PATH twin was
  // retired on both sides at once. NOTE: only the VARIABLE name changed -- the folder it points
  // at is still spelled "webots-<tmpId>" on disk (OmStandardPaths::webotsTmpPath), because that
  // string is a rendezvous the controller reconstructs, not a name anybody reads.
  env.insert("OMNISIM_INSTANCE_PATH", OmStandardPaths::webotsTmpPath());
  // Same reasoning as WEBOTS_ROBOT_NAME above: never hand the child two disagreeing spellings.
  env.remove("WEBOTS_INSTANCE_PATH");
  // Per-launch IPC nonce (must match the intern serverName built in start()): lets the child rebuild the
  // same uniquified pipe name webots-<tmpId>-<nonce>-<robot> so rapid sequential launches that reuse the
  // TCP port can't cross onto a stale pipe instance (default-flip-plan.md §3.5). Only set here, on the
  // intern child's environment; extern controllers never see it and keep the legacy unsalted name.
  env.insert("OMNISIM_IPC_NONCE", QString::number(QCoreApplication::applicationPid()));
  // qDebug() << "Environment:";
  // foreach (const QString &element, env)
  //  qDebug() << element;
  mProcess->setProcessEnvironment(env);
}

void OmController::info(const QString &message) {
  if (mExtern)
    OmLog::info(tr("'%1' extern controller: ").arg(mRobot->name()) + message);
  else
    OmLog::info(name() + ": " + message);
}

void OmController::warn(const QString &message) {
  OmLog::warning(name() + ": " + message);
}

void OmController::error(const QString &message) {
  OmLog::error(name() + ": " + message);
}

void OmController::appendMessageToConsole(const QString &message, bool useStdout) {
  appendMessageToBuffer(message, useStdout ? &mStdoutBuffer : &mStderrBuffer);
}

void OmController::readStdout() {
  appendMessageToBuffer(QString::fromUtf8(mProcess->readAllStandardOutput()), &mStdoutBuffer);
}

void OmController::readStderr() {
  appendMessageToBuffer(QString::fromUtf8(mProcess->readAllStandardError()), &mStderrBuffer);
}

void OmController::appendMessageToBuffer(const QString &message, QString *buffer) {
#ifdef _WIN32
  // on windows replace CR+LF by LF
  QString text = message;
  text.replace("\r\n", "\n");
#else
  const QString &text = message;
#endif
  buffer->append(text);
  if (*buffer == mStdoutBuffer)
    mStdoutNeedsFlush = true;
  else
    mStderrNeedsFlush = true;
}

void OmController::flushBuffer(QString *buffer) {
  // Split string into lines by detecting '\n', then send lines one by one to OmLog.
  // When several streams or several controllers are used, this prevents to mix unrelated lines
  int index = buffer->indexOf('\n');
  while (index != -1) {
    const QString line = buffer->mid(0, index + 1);
    if (*buffer == mStdoutBuffer)
      OmLog::appendStdout(line, robot()->name());
    else
      OmLog::appendStderr(line, robot()->name());
    // remove line from buffer
    buffer->remove(0, index + 1);
    index = buffer->indexOf('\n');
  }
  if (*buffer == mStdoutBuffer)
    mStdoutNeedsFlush = false;
  else
    mStderrNeedsFlush = false;
}

void OmController::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  mHasBeenTerminatedByItself = true;
  clearStartWatchdog();  // the exit is reported on its own terms below; a late watchdog line would be noise
  flushBuffers();
  switch (exitStatus) {
    case QProcess::NormalExit:
      if (exitCode == 0)
        OmLog::info(tr("'%1' controller exited successfully.").arg(name()));
      else
        OmLog::warning(tr("'%1' controller exited with status: %2.").arg(name()).arg(exitCode));
      break;
    case QProcess::CrashExit:
      OmLog::warning(tr("'%1' controller crashed.").arg(name()));
      break;
  }
  emit hasTerminatedByItself(this);
}

void OmController::reportControllerNotFound() {
  warn(tr("Could not find controller file:"));
  warn(tr("Expected either: %1, %2 or %3")
         .arg(name() + OmStandardPaths::executableExtension(), name() + ".py", "Dockerfile"));

  // try to give a smart advice
  QDir dir(mControllerPath);
  if (dir.exists(name() + ".c") || dir.exists(name() + ".cpp"))
    info(tr("Try to compile the C/C++ source code, to get a new executable file."));

  warn(tr("Starts the <generic> controller instead."));
}

void OmController::reportMissingCommand(const QString &command) {
  OmLog::warning(tr("Unable to find the '%1' executable in the current PATH. "
                    "Please check your %1 installation. "
                    "It should be possible to launch %1 from a terminal by typing '%1'. "
                    "It may be necessary to add the %1 bin directory to your PATH environment variable. "
                    "More information about the %1 installation is available in OmniSim's documentation.")
                   .arg(command));
}

void OmController::reportFailedStart() {
  warn(tr("failed to start: %1").arg(commandLine()));
  switch (mType) {
    case OmFileUtil::EXECUTABLE: {
      QFileInfo fi(mCommand);
      if (!fi.isFile()) {
        warn(tr("This is not a valid file, maybe a directory."));
        warn(tr("OmniSim expects a binary executable file at this location."));
        return;
      }
      if (!fi.isExecutable()) {
        warn(tr("This is not an executable file, try to change its permissions."));
        return;
      }
      warn(tr("This is not a valid executable file."));
      warn(tr("Maybe it has the wrong binary architecture: try to recompile this controller."));
      return;
    }
    case OmFileUtil::PYTHON:
      reportMissingCommand("python");
      break;
    case OmFileUtil::DOCKER:
      reportMissingCommand("docker");
      break;
    default:
      break;
  }
}

void OmController::processErrorOccurred(QProcess::ProcessError error) {
  switch (error) {
    case QProcess::FailedToStart:
      reportFailedStart();
      break;
    case QProcess::Crashed:
      warn(tr("The process crashed some time after starting successfully."));
      break;
    case QProcess::Timedout:
      warn(tr("The process didn't respond in time."));
      break;
    case QProcess::WriteError:
      warn(tr("An error occurred when attempting to write to the process."));
      break;
    case QProcess::ReadError:
      warn(tr("An error occurred when attempting to read from the process."));
      break;
    default:
      warn(tr("Unknown error."));
      break;
  }
}

OmFileUtil::FileType OmController::findType(const QString &controllerPath) {
  QDir dir(controllerPath);
  const QString &controllerName = (name() == "<generic>") ? "generic" : name();
  if (dir.exists("Dockerfile"))
    return OmFileUtil::DOCKER;
  else if (dir.exists(controllerName + OmStandardPaths::executableExtension()))
    return OmFileUtil::EXECUTABLE;
  else if (dir.exists(QString("build/release/%1%2").arg(controllerName).arg(OmStandardPaths::executableExtension())))
    return OmFileUtil::EXECUTABLE;
  else if (dir.exists(controllerName + ".py"))
    return OmFileUtil::PYTHON;
  else if (dir.exists(controllerName + ".bsg"))
    return OmFileUtil::BOTSTUDIO;

  return OmFileUtil::UNKNOWN;
}

void OmController::startGenericExecutable() {
  updateName("<generic>");
  mControllerPath = OmStandardPaths::resourcesControllersPath() + "generic/";
  mCommand = mControllerPath + "generic" + OmStandardPaths::executableExtension();

  copyBinaryAndDependencies(mCommand);

  mCommand = QDir::toNativeSeparators(mCommand);
  mArguments << mRobot->controllerArgs();
}

void OmController::startExecutable() {
  mCommand = mControllerPath + name() + OmStandardPaths::executableExtension();

  copyBinaryAndDependencies(mCommand);

  mCommand = QDir::toNativeSeparators(mCommand);
  mArguments << mRobot->controllerArgs();
}

void OmController::startPython() {
  if (mPythonCommand == "!")  // wrong python version
    return;
  mCommand = mPythonCommand;
  mArguments = OmLanguageTools::pythonArguments();
  if (!mPythonOptions.isEmpty())
    mArguments << mPythonOptions.split(" ");
  mArguments << name() + ".py";
  mArguments << mRobot->controllerArgs();
}

void OmController::startBotstudio() {
  // display a warning if the robot window is not "botstudio"
  if (mRobot->window() != "botstudio")
    warn(tr("A BotStudio controller was detected, but the 'window' field of the Robot node is not set to \"botstudio\". "
            "The controller probably won't work as expected."));

  // start simply the generic controller, but without modifying the controller path
  QString genericContollerPath = OmStandardPaths::resourcesControllersPath() + "generic/";
  mCommand = genericContollerPath + "generic" + OmStandardPaths::executableExtension();
  copyBinaryAndDependencies(mCommand);
  mCommand = QDir::toNativeSeparators(mCommand);
}

void OmController::startDocker() {
#ifndef __linux__
  warn(tr("Docker controllers are supported only on Linux."));
#else
  mCommand = "docker";
  // execute "docker build -q ." in the controller folder to build the image if needed and retrieve the image id
  QProcess dockerBuild;
  dockerBuild.setWorkingDirectory(mControllerPath);
  dockerBuild.start(mCommand, {"build", "-q", "."});
  if (!dockerBuild.waitForStarted() || !dockerBuild.waitForFinished()) {
    warn(tr("Unable to run docker, is docker installed?"));
    return;
  }
  const QString image(dockerBuild.readAll().trimmed());
  if (image.isEmpty()) {
    warn(tr("Failed to build the docker image in '%1'.").arg(mControllerPath));
    return;
  }
  const QStringList dockerArguments = {"run",  "--network",
                                       "none",  // add "--cpu-shares", "512",
                                       "-v",   OmStandardPaths::webotsTmpPath() + ":" + OmStandardPaths::webotsTmpPath(),
                                       "-e",   "OMNISIM_INSTANCE_PATH=" + OmStandardPaths::webotsTmpPath(),
                                       "-e",   "OMNISIM_ROBOT_NAME=" + mRobot->name(),
                                       image};  // the raw robot name is set, if needed libController will encode it
  mArguments = dockerArguments + mRobot->controllerArgs();
#endif
}

void OmController::copyBinaryAndDependencies(const QString &filename) {
  if (OmBinaryIncubator::copyBinaryAndDependencies(filename) == OmBinaryIncubator::FILE_REMOVE_ERROR) {
    warn(tr("An error occurred during the copy of controller '%1'. An older version will be executed.\n"
            "Please close any running instances of the controller and reload the world.")
           .arg(filename));
    return;
  }

#ifdef __APPLE__
  // silently change RPATH before launching controller, if the controller is not in the installation path.
  if (OmFileUtil::isLocatedInInstallationDirectory(filename, true) || !QFileInfo(filename).isWritable())
    return;

  QProcess process;
  bool success;

  // get current RPATH
  const QString cmd = QString("otool -l %1 | grep LC_RPATH -A 3 | grep path | cut -c15- | cut -d' ' -f1").arg(filename);
  process.start("bash", QStringList() << "-c" << cmd);
  success = process.waitForFinished(500);
  if (!success || !process.readAllStandardError().isEmpty())
    return;
  const QString oldRPath = process.readAllStandardOutput().trimmed();

  // change RPATH
  QStringList args;
  if (oldRPath.isEmpty())
    args << "-add_rpath" << OmStandardPaths::omniSimHomePath() << filename;
  else
    args << "-rpath" << oldRPath << OmStandardPaths::omniSimHomePath() << filename;
  process.start("install_name_tool", args);
  process.waitForFinished(-1);
#endif
}

int OmController::robotId() const {
  return mRobot->uniqueId();
}

const QString &OmController::name() const {
  return mName;
}

QString OmController::commandLine() const {  // returns the command line with double quotes if needed
  QString commandLine = mCommand.contains(' ') ? '"' + mCommand + '"' : mCommand;
  foreach (QString argument, mArguments)
    commandLine +=
      ' ' + (argument.contains(' ') || (argument.contains('"')) ? '\"' + argument.replace('"', "\\\"") + '"' : argument);
  return commandLine;
}

void OmController::handleControllerExit() {
  if (mExtern) {
    processFinished(0, QProcess::NormalExit);
    mRobot->setControllerNeedRestart();
  }
}

void OmController::writeUserInputEventAnswer() {
  // prepare stream
  OmDataStream stream(0);
  if (mTcpSocket)
    prepareTcpStream(stream);

  int delay = 0;
  stream << delay;

  // dispatch the stream to the devices
  mRobot->setNeedToWriteUserInputEventAnswer();
  mRobot->dispatchAnswer(stream, false);

  // size management
  int size = streamSizeManagement(stream);

  // write the request
  if (mTcpSocket) {
    mTcpSocket->write(stream.constData(), size);
    mTcpSocket->flush();  // sometimes packets are simply not sent without flushing
  } else {
    mSocket->write(stream.constData(), size);
    mSocket->flush();  // sometimes packets are simply not sent without flushing
  }
}

void OmController::writeAnswer(bool immediateAnswer) {
  if (mRobot == NULL)
    // controller is being destroyed
    return;

  mHasPendingImmediateAnswer = false;

  // prepare stream
  OmDataStream stream(0);
  if (mTcpSocket)
    prepareTcpStream(stream);

  // delay management
  // the time including the controller process time is the
  // time between two answers
  int delay = 0;
  if (!immediateAnswer && mRequestPending) {
    delay = mDeltaTimeMeasured - mDeltaTimeRequested;
    if (delay < 0)
      delay = 0;
  }
  stream << delay;
  // dispatch the stream to the devices
  mRobot->dispatchAnswer(stream);
  if (mRobot->hasImmediateAnswer())
    mRobot->writeImmediateAnswer(stream);

  // size management
  int size = streamSizeManagement(stream);

  // write the request
  if (mTcpSocket) {
    mTcpSocket->write(stream.constData(), size);
    mTcpSocket->flush();  // sometimes packets are simply not sent without flushing
  } else {
    mSocket->write(stream.constData(), size);
    mSocket->flush();  // sometimes packets are simply not sent without flushing
  }

  // reset request time
  if (!immediateAnswer)
    mRequestPending = false;

  // Debug code to see the content of the packet
  // static int id = 0;
  // printArray(buffer, "Answer", id++, true, true);
  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->startMeasure(OmPerformanceLog::CONTROLLER, mName);
}

void OmController::writeImmediateAnswer() {
  if (!isRequestPending() || isIncompleteRequest() || OmControlledWorld::instance()->isExecutingStep()) {
    // mixing immediate messages sent by OmniSim and the libController could
    // make the simulation hang because of the reception of unexpected messages
    // in order to avoid that the OmniSim immediate messages are postponed
    mHasPendingImmediateAnswer = true;
    return;
  }

  mHasPendingImmediateAnswer = false;
  if (mRobot == NULL)
    // controller is being destroyed
    return;

  if (!mRobot->hasImmediateAnswer())
    return;

  // prepare stream
  OmDataStream stream(0);
  if (mTcpSocket)
    prepareTcpStream(stream);

  // immediate message
  const int delay = -1;
  stream << delay;

  // dispatch answer
  mRobot->writeImmediateAnswer(stream);

  // size management
  int size = streamSizeManagement(stream);
  assert(size > 8);  // the immediate message shouldn't be empty

  // write the request
  if (mTcpSocket) {
    mTcpSocket->write(stream.constData(), size);
    mTcpSocket->flush();  // sometimes packets are simply not sent without flushing
  } else {
    mSocket->write(stream.constData(), size);
    mSocket->flush();  // sometimes packets are simply not sent without flushing
  }
}

void OmController::prepareTcpStream(OmDataStream &stream) {
  unsigned short nbChunks = 0;
  int dataSize = 0;
  int size = 0;
  unsigned char type = TCP_DATA_TYPE;
  stream << (unsigned short)(nbChunks);
  stream << (int)(dataSize);
  stream << (int)(size);
  stream << (unsigned char)(type);
  stream.mSizePtr = sizeof(unsigned short) + sizeof(int);
  stream.mDataSize = 0;
}

int OmController::streamSizeManagement(OmDataStream &stream) {
  unsigned int size = stream.length();
  if (!mTcpSocket) {
    size += sizeof(int);
    QByteArray baSize;
    for (int i = 0; i != sizeof(size); ++i) {
      baSize.append((char)((size & (0xFFu << (i * 8))) >> (i * 8)));
    }
    stream.prepend(baSize);
  } else {
    int chunkSize = stream.length() - stream.mSizePtr;
    int chunkDataSize = chunkSize - sizeof(int) - sizeof(unsigned char);

    if (chunkDataSize) {
      // increase first char by 1
      stream.increaseNbChunks(1);

      // add size and type information for the data chunk
      OmDataStream newDataMeta;
      unsigned char newDataType = TCP_DATA_TYPE;
      newDataMeta << chunkDataSize << newDataType;
      stream.replace(stream.mSizePtr, sizeof(int) + sizeof(unsigned char), newDataMeta);
      stream.mDataSize += chunkDataSize;
    } else
      stream.remove(stream.mSizePtr, 5);

    size = stream.length();
    OmDataStream dataSize;
    dataSize << stream.mDataSize;
    stream.replace(sizeof(unsigned short), (int)sizeof(int), dataSize);
  }
  return size;
}

// this function matches with the reception of a datagram
// Warning: several OmniSim packets can be into a datagram, and
// a OmniSim packet can be splitted into several datagrams
void OmController::readRequest() {
  mProcessingRequest = true;
  if ((mSocket == NULL && mTcpSocket == NULL) || mRobot == NULL)
    return;

  // concat all the data which has not been parsed
  if (mTcpSocket)
    mRequest += mTcpSocket->readAll();
  else
    mRequest += mSocket->readAll();

  const bool needToBlockRegeneration = robot()->supervisor();
  if (needToBlockRegeneration)
    OmTemplateManager::instance()->blockRegeneration(true);

  bool immediateMessagesPending = false;
  while (true) {
    const unsigned int requestSize = mRequest.size();
    unsigned int packetSize = 0;

    // get a webots packet size
    if (requestSize < sizeof(int))
      break;

    QDataStream sizeStream(mRequest);
    sizeStream.setByteOrder(QDataStream::LittleEndian);
    sizeStream >> packetSize;

    if (packetSize == 0)
      break;  // could occur when a controller stops itself

    // check if packet is complete
    mIncompleteRequest = (requestSize < packetSize);
    if (mIncompleteRequest)
      break;

    // create the OmniSim packet
    const QByteArray controllerPacket = mRequest.left(packetSize);

    // Debug code to see the content of the packet
    // static int id = 0;
    // printArray(controllerPacket, "Request", id++, true, true);

    // create the stream on the OmniSim packet
    QDataStream stream(controllerPacket);
    stream.setByteOrder(QDataStream::LittleEndian);

    // read the first int (confirmation of the already readed packetSize
    unsigned int packetSizeConfirmation;
    stream >> packetSizeConfirmation;
    assert(packetSize == packetSizeConfirmation);

    // read ms
    immediateMessagesPending = false;  // e.g. supervisor
    if (packetSize >= 2 * sizeof(int)) {
      unsigned int ms;
      stream >> ms;

      const bool isConfigureMessage = !mRobot->isConfigureDone();

      // dispatch the message
      if (packetSize > 2 * sizeof(int))
        mRobot->dispatchMessage(stream);

      if (mRobot->isWaitingForUserInputEvent())
        // force the controller buffers flush to make sure eventual instructions are shown
        flushBuffers();

      // store stuff
      immediateMessagesPending = (ms == 0 && !isConfigureMessage);
      if (!immediateMessagesPending)
        mRequestPending = (packetSize > 2 * sizeof(int)) || ms > 0;

      if (ms > 0) {
        mDeltaTimeMeasured = OmSimulationState::instance()->time() - mRequestTime;
        mDeltaTimeRequested = ms;
        mRequestTime = OmSimulationState::instance()->time();
      }
    }

    // remove the packet from the request
    mRequest.remove(0, packetSize);

    if (immediateMessagesPending)
      writeAnswer(true);

    OmControlledWorld::instance()->checkIfReadRequestCompleted();
  }

  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log && !immediateMessagesPending)
    log->stopMeasure(OmPerformanceLog::CONTROLLER, mName);

  if (needToBlockRegeneration)
    OmTemplateManager::instance()->blockRegeneration(false);

  emit requestReceived();
  mProcessingRequest = false;
}

void OmController::robotDestroyed() {
  mRobot = NULL;
  OmControlledWorld::instance()->deleteController(this);
}

void OmController::disconnected() {
  if (!mHasBeenTerminatedByItself) {
    if (mSocket) {
      mSocket->deleteLater();
      mSocket = NULL;
    } else if (mTcpSocket) {
      mRobot->removeRemoteExternController();
      mTcpSocket->deleteLater();
      mTcpSocket = NULL;
    }
    mRequestPending = false;
    mProcessingRequest = false;
    mHasPendingImmediateAnswer = false;

    if (mExtern) {
      info(tr("disconnected, waiting for new connection."));
      OmControlledWorld::instance()->externConnection(this, false);
    }
  }
}
