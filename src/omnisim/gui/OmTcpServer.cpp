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

#include "OmTcpServer.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmApplication.hpp"
#include "OmControlledWorld.hpp"
#include "OmController.hpp"
#include "OmField.hpp"
#include "OmHttpReply.hpp"
#include "OmLanguage.hpp"
#include "OmMainWindow.hpp"
#include "OmNodeOperations.hpp"
#include "OmPerspective.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmRobot.hpp"
#include "OmSimulationState.hpp"
#include "OmSimulationWorld.hpp"
#include "OmStandardPaths.hpp"
#include "OmSupervisorUtilities.hpp"
#include "OmTemplateManager.hpp"
#include "OmWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QRegularExpression>
#include <QtNetwork/QTcpServer>
#include <QtWebSockets/QWebSocket>
#include <QtWebSockets/QWebSocketServer>

#include <iostream>

OmMainWindow *OmTcpServer::cMainWindow = NULL;

OmTcpServer::OmTcpServer(bool stream) :
  QObject(),
  mPauseTimeout(-1),
  mWebSocketServer(NULL),
  mClientsReadyToReceiveMessages(false),
  mStream(stream),
  mWorldReady(false) {
  connect(OmApplication::instance(), &OmApplication::postWorldLoaded, this, &OmTcpServer::newWorld);
  connect(OmApplication::instance(), &OmApplication::preWorldLoaded, this, &OmTcpServer::deleteWorld);
  connect(OmApplication::instance(), &OmApplication::worldLoadingHasProgressed, this, &OmTcpServer::setWorldLoadingProgress);
  connect(OmApplication::instance(), &OmApplication::worldLoadingStatusHasChanged, this, &OmTcpServer::setWorldLoadingStatus);
  connect(OmNodeOperations::instance(), &OmNodeOperations::nodeAdded, this, &OmTcpServer::propagateNodeAddition);
  connect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this, &OmTcpServer::propagateNodeAddition);
  connect(OmNodeOperations::instance(), &OmNodeOperations::nodeDeleted, this, &OmTcpServer::propagateNodeDeletion);
  connect(OmTemplateManager::instance(), &OmTemplateManager::preNodeRegeneration, this, &OmTcpServer::propagateNodeDeletion);
}

OmTcpServer::~OmTcpServer() {
  if (isActive())
    destroy();
  OmLog::info(tr("Streaming server closed"));
};

QString OmTcpServer::clientToId(QWebSocket *client) {
  return QString::number((quintptr)client);
}

void OmTcpServer::setMainWindow(OmMainWindow *mainWindow) {
  cMainWindow = mainWindow;
}

//: How many ports beyond the requested one the server will try before giving
//: up. Widened from 10 to 60 on 2026-08-02: the range has to absorb not just
//: concurrent simulators but the ACCUMULATED residue of killed ones, and a
//: batch runner that times a run out leaves a held port behind every time.
//: Eleven slots were exhausted well inside a single benchmark session.
static const int PORT_SCAN_SPAN = 60;

void OmTcpServer::start(int port) {
  static int originalPort = -1;
  QString errorString;
  if (originalPort == -1)
    originalPort = port;
  mPort = port;
  try {
    create(port);
    originalPort = -1;
  } catch (const QString &e) {
    errorString = e;
    if (originalPort + PORT_SCAN_SPAN > port) {
      mPort++;
      start(mPort);
    } else {
      // Every slot in the scan range is taken. The usual cause is NOT other
      // live simulators: it is engines that were KILLED rather than allowed to
      // exit and are still holding their ports, which is exactly what a batch
      // runner does on every timeout. Measured 2026-08-02: a sequence of
      // headless runs started failing wholesale once enough killed engines had
      // accumulated, and the failure was invisible because the message below
      // goes to a console this GUI-subsystem binary does not have. It reaches
      // the log too now (OmGuiApplication::commandLineError).
      OmLog::error(tr("No free TCP port in [%1-%2]: %3. Ports are held by simulator processes that have not exited -- "
                      "including any that were killed rather than closed. Check for leftover omnisim-bin processes, or "
                      "pass --port to pick a range of your own.")
                     .arg(originalPort)
                     .arg(originalPort + PORT_SCAN_SPAN)
                     .arg(errorString),
                   false, OmLog::ODE);
      mPort = -1;  // failed, giving up
    }
  }
  const QString items = (mStream ? "web clients, " : "") + QString("extern controllers and robot windows");
  if (mPort == -1)
    OmLog::error(tr("Could not listen to %1 on port %2. %3. Giving up.").arg(items).arg(port).arg(errorString));
  else if (port != mPort)
    OmLog::warning(tr("Could not listen to %1 on port %2. %3. Using port %4 instead. This "
                      "could be caused by another running instance of OmniSim. You should use the '--port' option to start "
                      "several instances of OmniSim.")
                     .arg(items)
                     .arg(port)
                     .arg(errorString)
                     .arg(mPort));
  else if (mStream)
    OmLog::info(tr("Streaming server listening on port %1.").arg(port));
}

void OmTcpServer::sendToJavascript(const QByteArray &string) {
  const OmRobot *robot = dynamic_cast<OmRobot *>(sender());
  if (robot) {
    QJsonObject jsonObject;
    jsonObject.insert("name", robot->name());
    jsonObject.insert("message", QString::fromUtf8(string));
    const QJsonDocument jsonDocument(jsonObject);
    sendToClients("robot: " + jsonDocument.toJson(QJsonDocument::Compact));
  } else
    OmLog::info("OmTcpServer::sendToJavaScript: Can't send message: " + QString::fromUtf8(string));
}

void OmTcpServer::stop() {
  destroy();
}

void OmTcpServer::create(int port) {
  // Create a simple HTTP server, serving:
  // - a websocket on "/"
  // - texture images on the other urls. e.g. "/textures/dir/image.[jpg|png|hdr]"

  // See if a server is already running on port by trying to connect to it.
  // This is needed because in some environments QTcpServer::listen() uses a socket that is configured to reuse a port [1]
  // and Qt does not provide a way to configure the socket before calling listen() [2].
  // [1] https://doc.qt.io/qt-6/qabstractsocket.html#BindFlag-enum
  // [2] https://stackoverflow.com/questions/47268023/how-to-set-so-reuseaddr-on-the-socket-used-by-qtcpserver
  QTcpSocket socket;
  socket.connectToHost("localhost", port);
  if (socket.waitForConnected(100)) {
    socket.disconnectFromHost();
    throw tr("Port %1 is already in use").arg(port);
  }

  // Reference to let live QTcpSocket and QWebSocketServer on the same port using `QWebSocketServer::handleConnection()`:
  // - https://bugreports.qt.io/browse/QTBUG-54276
  mWebSocketServer = new QWebSocketServer("OmniSim Streaming Server", QWebSocketServer::NonSecureMode, this);
  mTcpServer = new QTcpServer();
  if (!mTcpServer->listen(QHostAddress::Any, port))
    throw tr("Cannot set the server in listen mode: %1").arg(mTcpServer->errorString());
  connect(mWebSocketServer, &QWebSocketServer::newConnection, this, &OmTcpServer::onNewWebSocketConnection);
  connect(mTcpServer, &QTcpServer::newConnection, this, &OmTcpServer::onNewTcpConnection);
  connect(OmSimulationState::instance(), &OmSimulationState::controllerReadRequestsCompleted, this,
          &OmTcpServer::sendUpdatePackageToClients, Qt::UniqueConnection);
  connect(OmLog::instance(), &OmLog::logEmitted, this, &OmTcpServer::propagateOmniSimLogToClients);
}

void OmTcpServer::destroy() {
  disconnect(OmSimulationState::instance(), &OmSimulationState::controllerReadRequestsCompleted, this,
             &OmTcpServer::sendUpdatePackageToClients);
  disconnect(OmLog::instance(), &OmLog::logEmitted, this, &OmTcpServer::propagateOmniSimLogToClients);

  if (mWebSocketServer)
    mWebSocketServer->close();

  foreach (const QWebSocket *c, mWebSocketClients) {
    disconnect(c, &QWebSocket::textMessageReceived, this, &OmTcpServer::processTextMessage);
    disconnect(c, &QWebSocket::disconnected, this, &OmTcpServer::socketDisconnected);
  };
  qDeleteAll(mWebSocketClients);
  mWebSocketClients.clear();

  delete mWebSocketServer;
  mWebSocketServer = NULL;

  delete mTcpServer;
  mTcpServer = NULL;
}

void OmTcpServer::closeClient(const QString &clientID) {
  foreach (QWebSocket *c, mWebSocketClients) {
    if (clientToId(c) == clientID) {
      disconnect(c, &QWebSocket::textMessageReceived, this, &OmTcpServer::processTextMessage);
      disconnect(c, &QWebSocket::disconnected, this, &OmTcpServer::socketDisconnected);
      emit sendRobotWindowClientID(clientToId(c), NULL, "disconnected");
      mWebSocketClients.removeAll(c);
      c->deleteLater();
    }
  }
}

void OmTcpServer::onNewTcpConnection() {
  QTcpSocket *socket = mTcpServer->nextPendingConnection();
  if (socket) {
    mWebSocketServer->handleConnection(socket);
    connect(socket, &QTcpSocket::readyRead, this, &OmTcpServer::onNewTcpData);
  }
}

void OmTcpServer::onNewTcpData() {
  QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());

  const QByteArray request = socket->peek(3);
  if (request == "CTR") {
    addNewTcpController(socket);
    return;
  }

  if (request != "GET")  // probably a WebSocket message
    return;
  const QString &line(socket->peek(8 * 1024));  // Peek the request header to determine the requested url.
  QStringList tokens = QString(line).split(QRegularExpression("[ \r\n][ \r\n]*"));
  if (tokens[0] == "GET" && tokens[1] != "/") {  // "/" is reserved for the websocket.
    const int hostIndex = tokens.indexOf("Host:") + 1;
    const QString host = hostIndex ? tokens[hostIndex] : "";
    const int etagIndex = tokens.indexOf("If-None-Match:") + 1;
    const QString etag = etagIndex ? tokens[etagIndex] : "";
    if (host.isEmpty())
      OmLog::warning(tr("No host specified in HTTP header."));
    sendTcpRequestReply(tokens[1].startsWith("/") ? tokens[1].sliced(1) : tokens[1], etag, host, socket);
  }
}

void OmTcpServer::addNewTcpController(QTcpSocket *socket) {
  const QString &line(socket->read(8 * 1024));
  const QStringList tokens = QString(line).split(QRegularExpression("\\s+"));
  const int robotNameIndex = tokens.indexOf("Robot-Name:") + 1;
  QByteArray reply;
  if (!mWorldReady) {
    reply.append("PROCESSING");
    socket->write(reply);
    return;
  }

  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  const QList<OmController *> &availableControllers = OmControlledWorld::instance()->disconnectedExternControllers();
  if (robotNameIndex) {  // robot name is given
    const QString robotName = tokens[robotNameIndex];
    foreach (const OmRobot *const robot, robots) {
      if (robot->encodedName() == robotName && robot->isControllerExtern()) {
        foreach (OmController *const controller, availableControllers) {
          if (controller->robot() == robot) {
            if (controller->setTcpSocket(socket)) {
              reply.append("CONNECTED");
              socket->write(reply);
              disconnect(socket, &QTcpSocket::readyRead, this, &OmTcpServer::onNewTcpData);
              controller->addRemoteControllerConnection();
              return;
            } else {
              reply.append("FORBIDDEN");
              socket->write(reply);
            }
          }
        }
      }
    }
    reply.append("FAILED");
    socket->write(reply);
  } else {  // no robot name given
    int nbExternRobots = 0;
    const OmRobot *targetRobot = NULL;
    foreach (OmRobot *const robot, robots) {
      if (robot->isControllerExtern()) {
        targetRobot = robot;
        nbExternRobots++;
      }
    }
    if (nbExternRobots == 1) {  // connect only if one robot has an <extern> controller
      foreach (OmController *const controller, availableControllers) {
        if (controller->robot() == targetRobot) {
          if (controller->setTcpSocket(socket)) {
            reply.append("CONNECTED");
            socket->write(reply);
            disconnect(socket, &QTcpSocket::readyRead, this, &OmTcpServer::onNewTcpData);
            controller->addRemoteControllerConnection();
            return;
          } else {
            reply.append("FORBIDDEN");
            socket->write(reply);
          }
        }
      }
    }
    reply.append("FAILED");
    socket->write(reply);
  }
  return;
}

void OmTcpServer::sendTcpRequestReply(const QString &completeUrl, const QString &etag, const QString &host,
                                      QTcpSocket *socket) {
  const QString url = completeUrl.left(completeUrl.lastIndexOf('?'));
  if (OmHttpReply::mimeType(url).isEmpty()) {
    OmLog::warning(tr("Unsupported file type '/%2'").arg(url));
    socket->write(OmHttpReply::forge404Reply("/" + url));
    return;
  }
  QString filePath;
  static const QStringList streamerFiles = {"index.html", "setup_viewer.js", "style.css", "omnisim_icon.png"};
  if (url == "streaming")  // shortcut
    filePath = OmStandardPaths::resourcesWebPath() + "streaming_viewer/index.html";
  else if (streamerFiles.contains(url))
    filePath = OmStandardPaths::resourcesWebPath() + "streaming_viewer/" + url;
  else if (url.startsWith("robot_windows/generic/"))
    filePath = OmStandardPaths::resourcesProjectsPath() + "plugins/" + url;
  else if (url.startsWith("robot_windows/"))
    filePath = OmProject::current()->pluginsPath() + url;
  else if (url.startsWith("~WEBOTS_HOME/"))
    filePath = OmStandardPaths::omniSimHomePath() + url.mid(13);
  else if (url.startsWith("resources/web/wwi/"))
    // Serve the bundled OmnisimView ecosystem (JS modules, WASM, .data files,
    // CSS, fonts, images) so the streaming viewer doesn't phone home to
    // cyberbotics.com for runtime assets. See resources/web/wwi/.
    filePath = OmStandardPaths::omniSimHomePath() + url;
  else if (url.endsWith(".js") || url.endsWith(".css") || url.endsWith(".html"))
    filePath = OmStandardPaths::omniSimHomePath() + url;
  socket->write(filePath.isEmpty() ? OmHttpReply::forge404Reply(url) : OmHttpReply::forgeFileReply(filePath, etag, host, url));
}

void OmTcpServer::onNewWebSocketConnection() {
  QWebSocket *client = mWebSocketServer->nextPendingConnection();
  if (client) {
    connect(client, &QWebSocket::textMessageReceived, this, &OmTcpServer::processTextMessage);
    connect(client, &QWebSocket::disconnected, this, &OmTcpServer::socketDisconnected);
    mWebSocketClients << client;
    if (mStream)
      OmLog::info(tr("Streaming server: New client [%1] (%2 connected client(s)).")
                    .arg(clientToId(client))
                    .arg(mWebSocketClients.size()));
  }
}

void OmTcpServer::sendFileToClient(QWebSocket *client, const QString &type, const QString &folder, const QString &path,
                                   const QString &filename) {
  QFile file(path + "/" + filename);
  if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QString content;
    int numberOfLines = 0;
    while (!file.atEnd()) {
      content += file.readLine();
      numberOfLines++;
    }
    file.close();
    OmLog::info("Sending " + folder + "/" + filename + " " + type + " to web interface (" + QString::number(numberOfLines) +
                " lines).");
    const QString &answer =
      "set " + type + ":" + folder + "/" + filename + ":" + QString::number(numberOfLines) + "\n" + content;
    client->sendTextMessage(answer);
  }
}

void OmTcpServer::processTextMessage(QString message) {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());

  if (message.startsWith("robot:")) {
    QString name;
    QString robotMessage;
    const QString &data = message.mid(6).trimmed();
    const QJsonDocument &jsonDocument = QJsonDocument::fromJson(data.toUtf8());
    if (jsonDocument.isNull() || !jsonDocument.isObject()) {
      // backward compatibility
      const int nameSize = data.indexOf(":");
      name = data.left(nameSize);
      robotMessage = data.mid(nameSize + 1);
    } else {
      name = jsonDocument.object().value("name").toString();
      robotMessage = jsonDocument.object().value("message").toString();
    }
    if (mStream)
      OmLog::info(tr("Streaming server: received robot message for %1: \"%2\".").arg(name).arg(robotMessage));

    if (robotMessage == "init robot window") {
      sendToClients();
      const int nameSize = data.indexOf(":");
      name = data.left(nameSize);
      emit sendRobotWindowClientID(clientToId(client), name, "connected");
    } else {
      const QList<OmRobot *> &robots = OmWorld::instance()->robots();
      const QByteArray &byteRobotMessage = robotMessage.toUtf8();
      foreach (OmRobot *const robot, robots)
        if (robot->name() == name) {
          robot->receiveFromJavascript(byteRobotMessage);
          break;
        }
    }
  } else if (mStream) {
    if (message == "pause") {
      disconnect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
                 &OmTcpServer::propagateSimulationStateChange);
      OmSimulationState::instance()->setMode(OmSimulationState::PAUSE);
      connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
              &OmTcpServer::propagateSimulationStateChange);
      printf("pause\n");
      fflush(stdout);
      client->sendTextMessage("paused by client");
    } else if (message.startsWith("real-time:") or message.startsWith("fast:")) {
      const bool realTime = message.startsWith("real-time:");
      const double timeout = realTime ? message.mid(10).toDouble() : message.mid(5).toDouble();
      if (timeout >= 0)
        mPauseTimeout = OmSimulationState::instance()->time() + timeout;
      else
        mPauseTimeout = -1.0;
      disconnect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
                 &OmTcpServer::propagateSimulationStateChange);
      if (realTime) {
        printf("real-time\n");
        OmSimulationState::instance()->setMode(OmSimulationState::REALTIME);
        client->sendTextMessage("real-time");
      } else {
        printf("fast\n");
        OmSimulationState::instance()->setMode(OmSimulationState::FAST);
        client->sendTextMessage("fast");
      }
      connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
              &OmTcpServer::propagateSimulationStateChange);
      fflush(stdout);
    } else if (message == "step") {
      disconnect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
                 &OmTcpServer::propagateSimulationStateChange);
      OmSimulationState::instance()->setMode(OmSimulationState::STEP);
      printf("step\n");
      fflush(stdout);
      OmSimulationWorld::instance()->step();
      OmSimulationState::instance()->setMode(OmSimulationState::PAUSE);
      connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
              &OmTcpServer::propagateSimulationStateChange);
      printf("pause\n");
      fflush(stdout);
      client->sendTextMessage("paused by client");
    } else if (message.startsWith("timeout:")) {
      const double timeout = message.mid(8).toDouble();
      if (timeout >= 0)
        mPauseTimeout = OmSimulationState::instance()->time() + timeout;
      else
        mPauseTimeout = -1.0;
    } else if (message == "reset") {
      resetSimulation();
      sendToClients("reset finished");
    } else if (message == "reload")
      OmApplication::instance()->worldReload();
    else if (message.startsWith("load:")) {
      const QString &worldsPath = OmProject::current()->worldsPath();
      const QString &fullPath = worldsPath + '/' + message.mid(5).split('/').takeLast();
      if (!QFile::exists(fullPath))
        OmLog::error(tr("Streaming server: world %1 doesn't exist.").arg(fullPath));
      else if (QDir(worldsPath) != QFileInfo(fullPath).absoluteDir())
        OmLog::error(tr("Streaming server: you are not allowed to open a world in another project directory."));
      else if (cMainWindow)
        cMainWindow->loadDifferentWorld(fullPath);
    } else
      OmLog::error(tr("Streaming server: Unsupported message: %1.").arg(message));
  }
}

void OmTcpServer::socketDisconnected() {
  QWebSocket *c = qobject_cast<QWebSocket *>(sender());
  if (c) {
    emit sendRobotWindowClientID(clientToId(c), NULL, "disconnected");
    mWebSocketClients.removeAll(c);
    c->deleteLater();
    if (mStream)
      OmLog::info(tr("Streaming server: Client disconnected [%1] (remains %2 client(s)).")
                    .arg(clientToId(c))
                    .arg(mWebSocketClients.size()));
  }
}

void OmTcpServer::sendUpdatePackageToClients() {
  if (mWebSocketClients.size() > 0) {
    const qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    if (mLastUpdateTime < 0.0 || currentTime - mLastUpdateTime >= 1000.0 / OmWorld::instance()->worldInfo()->fps()) {
      foreach (QWebSocket *c, mWebSocketClients)
        pauseClientIfNeeded(c);
      mLastUpdateTime = currentTime;
    }
  }
}

void OmTcpServer::propagateControllerLogToClients(OmLog::Level level, const QString &message, bool popup) {
  propagateLogToClients(level, message);
}

bool OmTcpServer::isControllerMessageIgnored(const QString &pattern, const QString &message) const {
  if (!QRegularExpression(pattern.arg(".+")).match(message).hasMatch())
    return false;

  return true;
}

void OmTcpServer::propagateOmniSimLogToClients(OmLog::Level level, const QString &message, bool popup) {
  if (message.startsWith("INFO: Streaming server") || level == OmLog::STATUS || level == OmLog::DEBUG)
    // do not propagate streaming server logs, status or debug messages
    return;

  // do not propagate start and exit messages coming from controllers
  if (isControllerMessageIgnored("^INFO: %1: Starting:", message) ||
      isControllerMessageIgnored("^INFO: '%1' controller", message))
    return;

  propagateLogToClients(level == OmLog::INFO ? OmLog::STDOUT : level, message);
}

void OmTcpServer::propagateLogToClients(OmLog::Level level, const QString &message) {
  QString result;

  if (level == OmLog::STDOUT)
    result = "stdout:";
  else
    result = "stderr:";

  result += message;
  sendToClients(result);
}

void OmTcpServer::sendToClients(const QString &message) {
  if (message.isEmpty()) {
    mClientsReadyToReceiveMessages = true;
    if (mMessageToClients.isEmpty())
      return;
  } else if (mMessageToClients.isEmpty())
    mMessageToClients = message;
  else
    mMessageToClients += "\n" + message;
  if (mWebSocketClients.isEmpty() || !mClientsReadyToReceiveMessages) {
    return;
  }
  foreach (QWebSocket *c, mWebSocketClients)
    c->sendTextMessage(mMessageToClients);
  mMessageToClients = "";
}

void OmTcpServer::connectNewRobot(const OmRobot *robot) {
  connect(robot, &OmRobot::sendToJavascript, this, &OmTcpServer::sendToJavascript);
}

bool OmTcpServer::prepareWorld() {
  try {
    foreach (QWebSocket *c, mWebSocketClients)
      sendWorldToClient(c);
  } catch (const QString &e) {
    OmLog::error(tr("Error when reloading world: %1.").arg(e));
    destroy();
    return false;
  }

  return true;
}

void OmTcpServer::newWorld() {
  if (mWebSocketServer == NULL)
    return;

  if (!prepareWorld())
    return;
  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  foreach (const OmRobot *const robot, robots)
    connectNewRobot(robot);

  mWorldReady = true;
}

void OmTcpServer::deleteWorld() {
  mWorldReady = false;
  if (mWebSocketServer == NULL)
    return;
  foreach (QWebSocket *c, mWebSocketClients)
    c->sendTextMessage("delete world");
}

void OmTcpServer::resetSimulation() {
  OmApplication::instance()->simulationReset(true);
  QCoreApplication::processEvents();  // this is required to make sure the simulation reset has been performed before sending
                                      // the update
  printf("reset\n");
  fflush(stdout);
  mLastUpdateTime = -1.0;
  mPauseTimeout = -1.0;
}

void OmTcpServer::setWorldLoadingProgress(const int progress) {
  foreach (QWebSocket *c, mWebSocketClients) {
    c->sendTextMessage("loading:" + mCurrentWorldLoadingStatus + ":" + QString::number(progress));
    c->flush();
  }
}

void OmTcpServer::propagateNodeAddition(OmNode *node) {
  if (mWebSocketServer == NULL || OmWorld::instance() == NULL)
    return;

  if (node->isProtoParameterNode()) {
    // PROTO parameter nodes are not exported to W3D
    foreach (OmNode *nodeInstance, node->protoParameterNodeInstances())
      propagateNodeAddition(nodeInstance);
    return;
  }

  const OmRobot *robot = dynamic_cast<OmRobot *>(node);
  if (robot) {
    connectNewRobot(robot);
    if (mWebSocketClients.isEmpty())
      return;
    foreach (QWebSocket *c, mWebSocketClients)
      sendRobotWindowInformation(c, robot);
  }
}

QString OmTcpServer::simulationStateString(bool pauseTime) {
  switch (OmSimulationState::instance()->mode()) {
    case OmSimulationState::PAUSE:
      return pauseTime ? QString("pause: %1").arg(OmSimulationState::instance()->time()) : "pause";
    case OmSimulationState::REALTIME:
      return "real-time";
    case OmSimulationState::FAST:
      return "fast";
    default:
      return "";
  }
}

void OmTcpServer::propagateSimulationStateChange() const {
  if (mWebSocketServer == NULL || OmWorld::instance() == NULL || mWebSocketClients.isEmpty())
    return;

  QString message = simulationStateString();
  if (message.isEmpty())
    return;
  foreach (QWebSocket *c, mWebSocketClients)
    c->sendTextMessage(message);
}

void OmTcpServer::pauseClientIfNeeded(QWebSocket *client) {
  if (mPauseTimeout < 0 || OmSimulationState::instance()->time() < mPauseTimeout)
    return;

  disconnect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this,
             &OmTcpServer::propagateSimulationStateChange);
  OmSimulationState::instance()->setMode(OmSimulationState::PAUSE);
  connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this, &OmTcpServer::propagateSimulationStateChange);
  client->sendTextMessage(QString("pause: %1").arg(OmSimulationState::instance()->time()));
  printf("pause\n");
  fflush(stdout);
}

void OmTcpServer::sendWorldToClient(QWebSocket *client) {
  const QFileInfoList worldFiles =
    QDir(OmProject::current()->worldsPath()).entryInfoList(OmWorldFileFormat::nameFilters(), QDir::Files);

  QString worlds;
  foreach (const QFileInfo item, worldFiles)
    worlds += QDir(OmProject::current()->dir()).relativeFilePath(item.absoluteFilePath()) + ";";
  worlds.chop(1);  // remove last separator

  const QString &currentWorld = QDir(OmProject::current()->dir()).relativeFilePath(OmWorld::instance()->fileName());
  client->sendTextMessage("world:" + currentWorld + ':' + worlds);
}

void OmTcpServer::sendRobotWindowInformation(QWebSocket *client, const OmRobot *robot, bool remove) {
  if (!robot->window().isEmpty()) {
    QJsonObject windowObject;
    windowObject.insert("robot", robot->name());
    windowObject.insert("window", robot->window());
    // if the robot window should be visible based on the perspective file c.f. OmMainWindow::restorePerspective():
    if (OmWorld::instance()->perspective()->enabledRobotWindowNodeNames().contains(robot->computeUniqueName()))
      windowObject.insert("visible", true);
    if (remove)
      windowObject.insert("remove", true);
    if (OmWorld::instance()->worldInfo()->window() == robot->window())
      windowObject.insert("main", true);
    const QJsonDocument windowDocument(windowObject);
    client->sendTextMessage("robot window: " + windowDocument.toJson(QJsonDocument::Compact));
  }
}

void OmTcpServer::propagateNodeDeletion(OmNode *node) {
  const OmNode *def = static_cast<const OmBaseNode *>(node)->getFirstFinalizedProtoInstance();
  foreach (QWebSocket *c, mWebSocketClients) {
    const OmRobot *robot = dynamic_cast<const OmRobot *>(def);
    if (robot)
      sendRobotWindowInformation(c, robot, true);
  }
}
