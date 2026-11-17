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

#ifndef OM_MULTIMEDIA_STREAMING_SERVER_HPP
#define OM_MULTIMEDIA_STREAMING_SERVER_HPP

#include "OmTcpServer.hpp"
#include "OmVector3.hpp"

#include <QtCore/QElapsedTimer>
#include <QtCore/QTimer>

class OmMatter;
class OmMultimediaStreamingLimiter;
class OmView3D;

class OmMultimediaStreamingServer : public OmTcpServer {
  Q_OBJECT

public:
  OmMultimediaStreamingServer();
  ~OmMultimediaStreamingServer() override;
  void sendImage(const QImage &image);

  void setView3D(OmView3D *view3D);
  bool isNewFrameNeeded() const;

signals:
  void imageRequested();

private slots:
  void removeTcpClient();
  void processTextMessage(QString message) override;
  void sendImageOnTimeout();
  void processLimiterTimeout();
  void sendWorldToClient(QWebSocket *client) override;

private:
  void start(int port) override;
  void sendTcpRequestReply(const QString &requestedUrl, const QString &etag, const QString &host, QTcpSocket *socket) override;
  int bytesToWrite();
  void sendContextMenuInfo(const OmMatter *node);
  void sendLastImage(QTcpSocket *client = NULL);
  void updateStreamingParameters(int skippedImagesCount);

  int mImageWidth;
  int mImageHeight;
  int mImageUpdateTimeStep;

  QByteArray mSceneImage;
  QList<QTcpSocket *> mTcpClients;
  QElapsedTimer mUpdateTimer;
  QTimer mWriteTimer;

  OmMultimediaStreamingLimiter *mLimiter;
  QTimer mLimiterTimer;
  int mAverageBytesToWrite;
  int mSentImagesCount;
  // flag keeping track of the current status
  //   0: none
  //   1: scene changed since full resolution image was sent
  //   2: full resolution image just sent
  int mFullResolutionOnPause;
  int mBlockedResolutionFactor;

  double mLastSpeedIndicatorTime;
  OmVector3 mTouchEventRotationCenter;
  bool mTouchEventObjectPicked;
  double mTouchEventZoomScale;
};

#endif
