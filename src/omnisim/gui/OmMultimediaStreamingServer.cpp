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

#include "OmMultimediaStreamingServer.hpp"

#include "OmDragViewpointEvent.hpp"
#include "OmMainWindow.hpp"
#include "OmMultimediaStreamingLimiter.hpp"
#include "OmRobot.hpp"
#include "OmScenePicker.hpp"
#include "OmSimulationState.hpp"
#include "OmSimulationWorld.hpp"
#include "OmView3D.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"

#include <QtCore/QBuffer>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtGui/QMouseEvent>
#include <QtWebSockets/QWebSocket>

static OmView3D *gView3D = NULL;

OmMultimediaStreamingServer::OmMultimediaStreamingServer() :
  OmTcpServer(true),
  mImageWidth(-1),
  mImageHeight(-1),
  mImageUpdateTimeStep(50),
  mLimiter(NULL),
  mAverageBytesToWrite(0),
  mSentImagesCount(0),
  mFullResolutionOnPause(0),
  mBlockedResolutionFactor(-1),
  mTouchEventObjectPicked(false) {
  OmMatter::enableShowMatterCenter(false);
}

OmMultimediaStreamingServer::~OmMultimediaStreamingServer() {
  mTcpClients.clear();
  delete mLimiter;
}

void OmMultimediaStreamingServer::setView3D(OmView3D *view3D) {
  gView3D = view3D;
  gView3D->setVideoStreamingServer(this);
}

void OmMultimediaStreamingServer::start(int port) {
  OmTcpServer::start(port);
  OmLog::info(
    tr("OmniSim multimedia streamer started: resolution %1x%2 on port %3").arg(mImageWidth).arg(mImageHeight).arg(port));
  mWriteTimer.setSingleShot(true);
  connect(&mWriteTimer, &QTimer::timeout, this, &OmMultimediaStreamingServer::sendImageOnTimeout);
  connect(&mLimiterTimer, &QTimer::timeout, this, &OmMultimediaStreamingServer::processLimiterTimeout);
}

void OmMultimediaStreamingServer::sendTcpRequestReply(const QString &requestedUrl, const QString &etag, const QString &host,
                                                      QTcpSocket *socket) {
  if (requestedUrl != "mjpeg") {
    OmTcpServer::sendTcpRequestReply(requestedUrl, etag, host, socket);
    return;
  }
  socket->readAll();

  static const QByteArray &contentType = ("HTTP/1.0 200 OK\r\nServer: OmniSim\r\nConnection: close\r\nMax-Age: 0\r\n"
                                          "Expires: 0\r\nCache-Control: no-cache, private\r\nPragma: no-cache\r\n"
                                          "Content-Type: multipart/x-mixed-replace; boundary=OmniSimFrame\r\n\r\n");
  socket->write(contentType);
  connect(socket, &QTcpSocket::disconnected, this, &OmMultimediaStreamingServer::removeTcpClient);
  mTcpClients.append(socket);
  // if available immediately send the latest image to the client
  if (mUpdateTimer.isValid())
    sendLastImage(socket);
  else if (OmSimulationState::instance()->isPaused())
    // request new image if none has been generated yet
    gView3D->refresh();

  if (!mLimiterTimer.isActive())
    mLimiterTimer.start(1000);
}

int OmMultimediaStreamingServer::bytesToWrite() {
  const QSslSocket *socket = dynamic_cast<QSslSocket *>(mTcpClients[0]);
  if (socket)
    return socket->encryptedBytesToWrite();
  return mTcpClients[0]->bytesToWrite();
}

void OmMultimediaStreamingServer::removeTcpClient() {
  const QTcpSocket *client = qobject_cast<QTcpSocket *>(sender());
  if (client)
    mTcpClients.removeAll(client);
  if (mTcpClients.isEmpty())
    mLimiterTimer.stop();
}

bool OmMultimediaStreamingServer::isNewFrameNeeded() const {
  if (!isActive() || mTcpClients.isEmpty())
    return false;

  if (!mUpdateTimer.isValid() || OmSimulationState::instance()->isPaused())
    return true;

  const qint64 msecs = mUpdateTimer.elapsed();
  return msecs >= mImageUpdateTimeStep;  // maximum update time step
}

void OmMultimediaStreamingServer::sendImage(const QImage &image) {
  const double simulationTime = OmSimulationState::instance()->time();
  sendToClients(QString("time: %1").arg(simulationTime));

  QBuffer bufferJpeg(&mSceneImage);
  bufferJpeg.open(QIODevice::WriteOnly);
  image.save(&bufferJpeg, "JPG");

  const qint64 msecs = mUpdateTimer.isValid() ? mUpdateTimer.elapsed() : mImageUpdateTimeStep + 1;
  if (OmSimulationState::instance()->isPaused() && (msecs < mImageUpdateTimeStep))
    mWriteTimer.start(2 * mImageUpdateTimeStep - msecs);
  else
    sendImageOnTimeout();
}

void OmMultimediaStreamingServer::sendImageOnTimeout() {
  mWriteTimer.stop();

  sendLastImage();
  if (OmSimulationState::instance()->isPaused())
    // force the update on the client side
    // note that on Firefox it is enough to send the boundary line without image
    // but this trick doesn't work on Chrome
    sendLastImage();
  mUpdateTimer.restart();
}

void OmMultimediaStreamingServer::processLimiterTimeout() {
  if (mFullResolutionOnPause == 2)
    return;
  if (mLimiter->isStopped()) {
    if (bytesToWrite() == 0)
      mLimiter->resetStop();
    return;
  }
  if (mSentImagesCount == 0) {
    if (OmSimulationState::instance()->isPaused() && mLimiter->resolutionFactor() > 1) {
      // nothing sent since a while
      // send one image in full resolution
      mFullResolutionOnPause = 2;
      const QSize &fullSize(mLimiter->fullResolution());
      cMainWindow->setView3DSize(fullSize);
    }
    mLimiter->resetStop();
    return;
  }

  const double bytes = (mSentImagesCount > 0) ? ((double)mAverageBytesToWrite) / mSentImagesCount : 0;
  updateStreamingParameters(bytes / mSceneImage.size());
}

void OmMultimediaStreamingServer::updateStreamingParameters(int skippedImagesCount) {
  mLimiter->recomputeStreamingLimits(skippedImagesCount);
  if ((mBlockedResolutionFactor < 0) && (mFullResolutionOnPause > 0 || mLimiter->resolutionChanged())) {
    const QSize &newSize(mLimiter->resolution());
    cMainWindow->setView3DSize(newSize);
    mFullResolutionOnPause = 0;
  }
  mImageUpdateTimeStep = mLimiter->updateTimeStep();
  mAverageBytesToWrite = 0;
  mSentImagesCount = 0;
}

void OmMultimediaStreamingServer::sendLastImage(QTcpSocket *client) {
  if (client && (client->state() != QAbstractSocket::ConnectedState || !client->isValid()))
    return;

  if (mLimiter->isStopped()) {
    if (bytesToWrite() == 0) {
      mLimiter->resetStop();
    } else
      return;
  }

  mAverageBytesToWrite += bytesToWrite();
  mSentImagesCount++;

  const QByteArray &boundaryString =
    QString("--OmniSimFrame\r\nContent-Type: image/jpeg\r\nContent-Length: %1\r\n\r\n").arg(mSceneImage.length()).toUtf8();
  QList<QTcpSocket *> clients;
  if (client)
    clients << client;
  else
    clients = mTcpClients;
  foreach (QTcpSocket *c, clients) {
    c->write(boundaryString);
    c->write(mSceneImage);
    c->write(QByteArray("\r\n"));
    c->flush();
  }
}

void OmMultimediaStreamingServer::sendContextMenuInfo(const OmMatter *node) {
  QJsonObject object;
  object.insert("name", node->name());
  object.insert("docUrl", node->documentationUrl());
  const OmRobot *robot = dynamic_cast<const OmRobot *>(node);
  object.insert("controller", robot ? robot->controllerName() : "");
  const OmSolid *const solid = dynamic_cast<const OmSolid *>(node);
  if (solid) {
    const OmViewpoint *viewpoint = OmWorld::instance()->viewpoint();
    const bool isFollowed = viewpoint->isFollowed(solid);
    object.insert("follow", isFollowed ? viewpoint->followType() : 0);
  } else
    object.insert("follow", -1);
  QJsonDocument jsonDocument(object);
  sendToClients("context menu: " + jsonDocument.toJson(QJsonDocument::Compact));
}

void OmMultimediaStreamingServer::processTextMessage(QString message) {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  if (mFullResolutionOnPause == 2)
    mFullResolutionOnPause = 1;

  if (message.startsWith("mouse")) {
    int action, button, buttons, x, y, modifiers, wheel;
    QString skip;  // will receive "mouse"
    QTextStream(&message) >> skip >> action >> button >> buttons >> x >> y >> modifiers >> wheel;
    if (mBlockedResolutionFactor < 0)
      mBlockedResolutionFactor = mLimiter->resolutionFactor();
    if (mFullResolutionOnPause == 0 && mLimiter && mBlockedResolutionFactor > 1) {
      const double factor = pow(2, mBlockedResolutionFactor - 1);
      x /= factor;
      y /= factor;
    }
    const QPointF point(x, y);
    const Qt::MouseButtons buttonsPressed = ((buttons & 1) ? Qt::LeftButton : Qt::NoButton) |
                                            ((buttons & 2) ? Qt::RightButton : Qt::NoButton) |
                                            ((buttons & 4) ? Qt::MiddleButton : Qt::NoButton);
    const Qt::KeyboardModifiers keyboardModifiers = ((modifiers & 1) ? Qt::ShiftModifier : Qt::NoModifier) |
                                                    ((modifiers & 2) ? Qt::ControlModifier : Qt::NoModifier) |
                                                    ((modifiers & 4) ? Qt::AltModifier : Qt::NoModifier);
    if (action <= 1) {
      QInputEvent::Type type;
      Qt::MouseButton buttonPressed;
      if (action == 0) {
        type = QEvent::MouseMove;
        buttonPressed = Qt::NoButton;
      } else {
        switch (button) {
          case 1:
            buttonPressed = Qt::LeftButton;
            break;
          case 2:
            buttonPressed = Qt::RightButton;
            break;
          case 3:
            buttonPressed = Qt::MiddleButton;
            break;
          default:
            buttonPressed = Qt::NoButton;
            break;
        }
        if (action == -1)
          type = QEvent::MouseButtonPress;
        else if (action == 1) {
          type = QEvent::MouseButtonRelease;
          mBlockedResolutionFactor = -1;
        } else
          type = QEvent::MouseMove;
      }
      QMouseEvent event(type, point, QCursor::pos(), buttonPressed, buttonsPressed, keyboardModifiers);
      if (gView3D) {
        const OmMatter *contextMenuNode = gView3D->remoteMouseEvent(&event);
        if (contextMenuNode)
          sendContextMenuInfo(contextMenuNode);
      }
    } else if (action == 2) {
      wheel = -wheel;  // Wheel delta is inverted in JS and Webots
      QWheelEvent wheelEvent(point, point, QPoint(), QPoint(0, wheel), buttonsPressed, keyboardModifiers, Qt::ScrollUpdate,
                             false);
      if (gView3D)
        gView3D->remoteWheelEvent(&wheelEvent);
    }
  } else if (message.startsWith("touch")) {
    int action, eventType, x, y;
    QString skip;  // will receive "touch"
    QTextStream stream(&message);
    stream >> skip >> action >> eventType;
    if (action == -1) {  // store touch event center
      stream >> x >> y;
      if (mFullResolutionOnPause == 0 && mLimiter && mLimiter->resolutionFactor() > 1) {
        const double factor = pow(2, mLimiter->resolutionFactor() - 1);
        x /= factor;
        y /= factor;
      }
      // D1.4: OmScenePicker computes the world-space hit point itself (worldCoordinates()),
      // so the old screenCoordinates()+toWorld unprojection is gone. On a miss, fall back to
      // the viewpoint position -- the same convention as OmView3D's mouse-press path.
      OmScenePicker picker;
      picker.pick(x, y);
      mTouchEventObjectPicked = picker.selectedId() != -1;
      OmViewpoint *viewpoint = OmWorld::instance()->viewpoint();
      if (mTouchEventObjectPicked)
        mTouchEventRotationCenter = picker.worldCoordinates();
      else
        mTouchEventRotationCenter = viewpoint->position()->value();
      if (eventType == 2) {
        double distanceToPickPosition;
        if (mTouchEventObjectPicked)
          distanceToPickPosition = (viewpoint->position()->value() - viewpoint->rotationCenter()).length();
        else
          distanceToPickPosition = viewpoint->position()->value().length();
        if (distanceToPickPosition < 0.001)
          distanceToPickPosition = 0.001;
        mTouchEventZoomScale =
          distanceToPickPosition * 2 * tan(viewpoint->fieldOfView()->value() / 2) / std::max(mImageWidth, mImageHeight);
      } else
        mTouchEventZoomScale = 1.0;
    } else if (action == 0 && eventType == 1) {  // touch rotate event
      stream >> x >> y;
      if (mFullResolutionOnPause == 0 && mLimiter && mLimiter->resolutionFactor() > 1) {
        const double factor = pow(2, mLimiter->resolutionFactor() - 1);
        x /= factor;
        y /= factor;
      }
      OmRotateViewpointEvent::applyToViewpoint(QPoint(x, y), mTouchEventRotationCenter,
                                               -OmWorld::instance()->worldInfo()->gravityUnitVector(), mTouchEventObjectPicked,
                                               OmWorld::instance()->viewpoint());

      gView3D->refresh();
    } else if (action == 0 && eventType == 2) {  // touch zoom/tilt event
      double tiltAngle, zoom;
      stream >> tiltAngle >> zoom;
      OmZoomAndRotateViewpointEvent::applyToViewpoint(tiltAngle, zoom, mTouchEventZoomScale, OmWorld::instance()->viewpoint());
      gView3D->refresh();
    }
  } else if (message.startsWith("mjpeg: ")) {
    const QStringList &resolution = message.mid(7).split("x");
    const int width = resolution[0].toInt();
    const int height = resolution[1].toInt();
    QString args;
    if ((mImageWidth <= 0 && mImageHeight <= 0) || client == mWebSocketClients.first()) {
      cMainWindow->setView3DSize(QSize(width, height));
      mImageWidth = width;
      mImageHeight = height;
      OmLog::info(tr("Streaming server: Resolution changed to %1x%2.").arg(width).arg(height));
      delete mLimiter;
      mLimiter = new OmMultimediaStreamingLimiter(QSize(mImageWidth, mImageHeight), 50);
    } else {
      // Video streamer already initialized
      OmLog::info(tr("Streaming server: Ignored new client request of resolution: %1x%2.").arg(width).arg(height));
      args = QString("%1 %2").arg(mImageWidth).arg(mImageHeight);
    }
    client->sendTextMessage(QString("multimedia: mjpeg %2 %3").arg(simulationStateString(false)).arg(args));
    const QString &stateMessage = simulationStateString();
    if (!stateMessage.isEmpty())
      client->sendTextMessage(stateMessage);
    sendWorldToClient(client);
    sendToClients();  // send possible bufferized messages
  } else if (message.startsWith("resize: ")) {
    if (client == mWebSocketClients.first()) {
      const QStringList &resolution = message.mid(8).split("x");
      mImageWidth = resolution[0].toInt();
      mImageHeight = resolution[1].toInt();
      OmLog::info(tr("Streaming server: Client resize: new resolution %1x%2.").arg(mImageWidth).arg(mImageHeight));
      cMainWindow->setView3DSize(QSize(mImageWidth, mImageHeight));
      sendToClients(QString("resize: %1 %2").arg(mImageWidth).arg(mImageHeight));
      mLimiter->resetResolution(QSize(mImageWidth, mImageHeight));
    } else
      OmLog::info(tr("Streaming server: Invalid client resize: only the first connected client can resize the simulation."));
  } else if (message.startsWith("follow: ")) {
    const int separatorIndex = message.indexOf(',');
    const QString &mode = message.mid(8, separatorIndex - 8);
    const QString &solidId = message.mid(separatorIndex + 1);
    OmSolid *const solid = OmSolid::findSolidFromUniqueName(solidId);
    OmViewpoint *const viewpoint = OmWorld::instance()->viewpoint();
    if (viewpoint->followedSolid())
      viewpoint->terminateFollowUp();
    if (solid) {
      viewpoint->setFollowType(mode.toInt());
      viewpoint->startFollowUp(solid, true);
    }
  } else if (message.startsWith("w3d")) {
    OmLog::error(tr("Streaming server received unsupported W3D message: '%1'. You should run OmniSim with the "
                    "'--stream=\"mode=w3d\"' command line option.")
                   .arg(message));
    return;
  } else
    OmTcpServer::processTextMessage(message);
}

void OmMultimediaStreamingServer::sendWorldToClient(QWebSocket *client) {
  const OmWorldInfo *currentWorldInfo = OmWorld::instance()->worldInfo();
  QJsonObject infoObject;
  infoObject.insert("window", currentWorldInfo->window());
  infoObject.insert("title", currentWorldInfo->title());
  const QJsonDocument infoDocument(infoObject);
  client->sendTextMessage("world info: " + infoDocument.toJson(QJsonDocument::Compact));
  OmTcpServer::sendWorldToClient(client);

  const QList<OmRobot *> &robots = OmWorld::instance()->robots();
  foreach (const OmRobot *robot, robots)
    OmTcpServer::sendRobotWindowInformation(client, robot);

  client->sendTextMessage("scene load completed");
}
