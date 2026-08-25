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

#include "OmW3dStreamingServer.hpp"

#include "OmAnimationRecorder.hpp"
#include "OmHttpReply.hpp"
#include "OmNodeOperations.hpp"
#include "OmProject.hpp"
#include "OmSimulationState.hpp"
#include "OmTemplateManager.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"

#include <QtCore/QFileInfo>
#include <QtWebSockets/QWebSocket>

OmW3dStreamingServer::OmW3dStreamingServer() : OmTcpServer(true), mW3dWorldGenerationTime(-1.0) {
}

OmW3dStreamingServer::~OmW3dStreamingServer() {
  if (OmAnimationRecorder::isInstantiated())
    OmAnimationRecorder::instance()->cleanupFromStreamingServer();
}

void OmW3dStreamingServer::start(int port) {
  if (OmWorld::instance()) {
    try {
      OmAnimationRecorder::instance()->initFromStreamingServer();
    } catch (const QString &e) {
      OmLog::error(tr("Error when initializing the animation recorder: %1").arg(e));
      return;
    }
  }
  OmTcpServer::start(port);
}

void OmW3dStreamingServer::stop() {
  // Test that the animation recorder is instantiated.
  // Otherwise, the instance() call can wrongly recreate an instance of the
  // animation recorder in the cleanup routines.
  if (OmAnimationRecorder::isInstantiated())
    OmAnimationRecorder::instance()->cleanupFromStreamingServer();
  OmTcpServer::stop();
}

void OmW3dStreamingServer::create(int port) {
  OmTcpServer::create(port);
  generateW3dWorld();
}

void OmW3dStreamingServer::sendTcpRequestReply(const QString &url, const QString &etag, const QString &host,
                                               QTcpSocket *socket) {
  const QString decodedUrl = QUrl::fromPercentEncoding(url.toUtf8());
  QFileInfo file(OmProject::current()->dir().absolutePath() + "/" + decodedUrl);
  if (file.exists())
    socket->write(OmHttpReply::forgeFileReply(file.absoluteFilePath(), etag, host, decodedUrl));
  else
    OmTcpServer::sendTcpRequestReply(url, etag, host, socket);
}

void OmW3dStreamingServer::processTextMessage(QString message) {
  if (message.startsWith("w3d")) {
    QWebSocket *client = qobject_cast<QWebSocket *>(sender());
    OmLog::info(tr("Streaming server: Client set mode to W3D."));
    mPauseTimeout = message.endsWith(";broadcast") ? -1 : 0;
    if (!OmWorld::instance()->isLoading()) {
      startW3dStreaming(client);
      sendToClients();  // send possible buffered messages
    }
    // else streaming is started once the world loading is completed
    return;
  } else if (message == "reset") {
    // reset nodes visibility
    QWebSocket *client = qobject_cast<QWebSocket *>(sender());
    foreach (const OmBaseNode *node, OmWorld::instance()->viewpoint()->getInvisibleNodes())
      client->sendTextMessage(QString("visibility:%1:1").arg(node->uniqueId()));
    resetSimulation();
    const QString &state = OmAnimationRecorder::instance()->computeUpdateData(true);
    if (!state.isEmpty()) {
      foreach (QWebSocket *c, mWebSocketClients)
        sendWorldStateToClient(c, state);
    }
    sendToClients("reset finished");
    return;
  } else if (message.startsWith("mjpeg: ")) {
    OmLog::error(tr("Streaming server received unsupported MJPEG message: '%1'. You should run OmniSim with the "
                    "'--stream=\"mode=mjpeg\"' command line option.")
                   .arg(message));
    return;
  }
  OmTcpServer::processTextMessage(message);
}

void OmW3dStreamingServer::startW3dStreaming(QWebSocket *client) {
  try {
    if (OmWorld::instance()->isModified() || mW3dWorldGenerationTime != OmSimulationState::instance()->time())
      generateW3dWorld();
    sendWorldToClient(client);
    // send the current simulation state to the newly connected client
    const QString &stateMessage = simulationStateString();
    if (!stateMessage.isEmpty())
      client->sendTextMessage(stateMessage);
  } catch (const QString &e) {
    OmLog::error(tr("Streaming server: Cannot send world date to client [%1] because: %2.").arg(clientToId(client)).arg(e));
  }
}

void OmW3dStreamingServer::sendUpdatePackageToClients() {
  if (mWebSocketClients.size() > 0) {
    const qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    if (mLastUpdateTime < 0.0 || currentTime - mLastUpdateTime >= 1000.0 / OmWorld::instance()->worldInfo()->fps()) {
      const QString &state = OmAnimationRecorder::instance()->computeUpdateData(false);
      if (!state.isEmpty()) {
        foreach (QWebSocket *c, mWebSocketClients) {
          sendWorldStateToClient(c, state);
          pauseClientIfNeeded(c);
        }
      }
      mLastUpdateTime = currentTime;
    }
  }
}

bool OmW3dStreamingServer::prepareWorld() {
  try {
    generateW3dWorld();
    foreach (QWebSocket *c, mWebSocketClients)
      sendWorldToClient(c);
    OmAnimationRecorder::instance()->initFromStreamingServer();
  } catch (const QString &e) {
    OmLog::error(tr("Error when reloading world: %1.").arg(e));
    destroy();
    return false;
  }

  return true;
}

void OmW3dStreamingServer::deleteWorld() {
  if (!isActive())
    return;
  OmAnimationRecorder::instance()->cleanupFromStreamingServer();
  OmTcpServer::deleteWorld();
}

void OmW3dStreamingServer::propagateNodeAddition(OmNode *node) {
  if (!isActive() || OmWorld::instance() == NULL)
    return;

  if (node->isProtoParameterNode()) {
    // PROTO parameter nodes are not exported to W3D or transmitted to webots.min.js
    foreach (OmNode *nodeInstance, node->protoParameterNodeInstances())
      propagateNodeAddition(nodeInstance);
    return;
  }

  OmTcpServer::propagateNodeAddition(node);

  const OmBaseNode *baseNode = static_cast<OmBaseNode *>(node);
  if (baseNode && baseNode->isInBoundingObject())
    return;

  if (!mWebSocketClients.isEmpty()) {
    QString nodeString;
    OmWriter writer(&nodeString, node->modelName() + ".w3d");
    node->write(writer);

    foreach (QWebSocket *c, mWebSocketClients)
      // add root <nodes> element to handle correctly multiple root elements like in case of PBRAppearance node.
      c->sendTextMessage(QString("node:%1:<nodes>%2</nodes>").arg(node->parentNode()->uniqueId()).arg(nodeString));
  }
}

void OmW3dStreamingServer::propagateNodeDeletion(OmNode *node) {
  if (!isActive() || OmWorld::instance() == NULL)
    return;

  OmTcpServer::propagateNodeDeletion(node);
  if (node->isProtoParameterNode())
    node = static_cast<const OmBaseNode *>(node)->getFirstFinalizedProtoInstance();

  foreach (QWebSocket *c, mWebSocketClients)
    c->sendTextMessage(QString("delete:%1").arg(node->uniqueId()));
}

void OmW3dStreamingServer::generateW3dWorld() {
  const OmWorld *world = OmWorld::instance();
  if (!world)
    return;

  QString worldString;
  OmWriter writer(&worldString, QFileInfo(world->fileName()).baseName() + ".w3d");
  world->write(writer);
  mW3dWorld = worldString;
  mW3dWorldGenerationTime = OmSimulationState::instance()->time();
  mLastUpdateTime = -1.0;
}

void OmW3dStreamingServer::sendWorldToClient(QWebSocket *client) {
  // when streaming, the world must be sent first so that asset URL can be computed relative to it
  OmTcpServer::sendWorldToClient(client);

  const qint64 ret = client->sendTextMessage(QString("model:") + mW3dWorld);
  if (ret < mW3dWorld.size())
    throw tr("Cannot send the entire world");

  const QString &state = OmAnimationRecorder::instance()->computeUpdateData(true);
  if (!state.isEmpty())
    sendWorldStateToClient(client, state);

  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  foreach (const OmRobot *robot, robots)
    OmTcpServer::sendRobotWindowInformation(client, robot);

  client->sendTextMessage("scene load completed");
}

void OmW3dStreamingServer::sendWorldStateToClient(QWebSocket *client, const QString &state) const {
  client->sendTextMessage(QString("application/json:") + state);
}
