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

#include "OmProtoIcon.hpp"

#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmFileUtil.hpp"
#include "OmNetwork.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"

#include <QtCore/QDir>
#include <QtCore/QUrl>

OmProtoIcon::OmProtoIcon(const QString &modelName, const QString &protoPath, QObject *parent) :
  QObject(parent),
  mPath(QString("%1icons/%2.png").arg(QUrl(protoPath).adjusted(QUrl::RemoveFilename).toString()).arg(modelName)),
  mModelName(modelName),
  mDownloader(NULL),
  mReady(true) {
  if (OmUrl::isWeb(mPath)) {
    if (OmNetwork::instance()->isCachedWithMapUpdate(mPath))
      mPath = OmNetwork::instance()->get(mPath);
    else {
      mReady = false;
      mDownloader = OmDownloadManager::instance()->createDownloader(QUrl(mPath), this);
      connect(mDownloader, &OmDownloader::complete, this, &OmProtoIcon::updateIcon);
      mDownloader->download();
    }
  } else if (OmUrl::isLocalUrl(mPath))
    mPath = QDir::cleanPath(mPath.replace("omnisim://", OmStandardPaths::omniSimHomePath()));
}

void OmProtoIcon::updateIcon() {
  assert(mDownloader);
  if (mDownloader->error().isEmpty())
    mPath = OmNetwork::instance()->get(mDownloader->url().toString());
  else
    mPath = QString();
  // else failure downloading or file does not exist (404)

  mReady = true;
  emit iconReady(mPath);
}

void OmProtoIcon::duplicate(QDir destinationDir) {
  assert(mReady);
  if (!QFile::exists(mPath))
    return;

  if (destinationDir.exists("icons") || destinationDir.mkdir("icons"))
    OmFileUtil::forceCopy(mPath, QString("%1/icons/%2.png").arg(destinationDir.absolutePath()).arg(mModelName));
}
