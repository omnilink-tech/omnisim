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

#include "OmDownloadManager.hpp"

#include "OmDownloader.hpp"
#include "OmSimulationState.hpp"

#include <QtCore/QTimer>
#include <QtCore/QVariant>

OmDownloadManager *OmDownloadManager::cInstance = NULL;

OmDownloadManager *OmDownloadManager::instance() {
  if (!cInstance)
    cInstance = new OmDownloadManager();
  return cInstance;
}

OmDownloadManager::OmDownloadManager() : mCount(0), mComplete(0), mDownloading(false), mTimer(NULL), mDisplayPopUp(false) {
}

int OmDownloadManager::progress() const {
  return mCount == 0 ? 100 : 100 * mComplete / mCount;
}

void OmDownloadManager::reset() {
  mCount = 0;
  mComplete = 0;
  mDownloading = false;
  mDisplayPopUp = false;
}

void OmDownloadManager::abort() {
  QMapIterator<QUrl, OmDownloader *> it(mUrlCache);
  while (it.hasNext()) {
    it.next();
    it.value()->abort();
  }
}

OmDownloader *OmDownloadManager::createDownloader(const QUrl &url, QObject *parent) {
  OmSimulationState::instance()->pauseSimulation();
  const OmDownloader *existingDownload = mUrlCache.value(url, NULL);
  OmDownloader *downloader = new OmDownloader(url, existingDownload, parent);
  connect(downloader, &OmDownloader::destroyed, this, &OmDownloadManager::removeDownloader);
  connect(downloader, &OmDownloader::complete, this, &OmDownloadManager::downloadCompleted);
  mCount++;

  if (!existingDownload) {
    if (!mDownloading) {
      mDownloading = true;
      mTimer = new QTimer(0);
      connect(mTimer, &QTimer::timeout, this, &OmDownloadManager::displayPopUp);
      mTimer->setInterval(1000);
      mTimer->setSingleShot(true);
      mTimer->start();
    }

    mUrlCache.insert(url, downloader);
  }

  return downloader;
}

void OmDownloadManager::removeDownloader(QObject *obj) {
  const QVariant urlProperty = obj->property("url");
  if (urlProperty.isValid() && mUrlCache.contains(urlProperty.toString()))
    mUrlCache.remove(urlProperty.toString());

  if (!obj->property("finished").toBool()) {
    mCount--;
    updateProgress();
  }
}

void OmDownloadManager::downloadCompleted() {
  mComplete++;
  updateProgress();
}

void OmDownloadManager::updateProgress() {
  if (mComplete == mCount) {
    mDownloading = false;
    mDisplayPopUp = false;
    mUrlCache.clear();
    OmSimulationState::instance()->resumeSimulation();
    mProgressUpdateCallback(progress());
  } else if (mDisplayPopUp)
    mProgressUpdateCallback(progress());
}

void OmDownloadManager::displayPopUp() {
  if (mDownloading) {
    mProgressUpdateCallback(progress());
    mDisplayPopUp = true;
  }

  if (mTimer) {
    delete mTimer;
    mTimer = NULL;
  }
}
