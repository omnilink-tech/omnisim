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

#ifndef OM_URL_HPP
#define OM_URL_HPP

#include <QtCore/QString>

class OmNode;
class OmMFString;
class OmWriter;

namespace OmUrl {
  QString resolveUrl(const QString &rawUrl);
  QString computePath(const OmNode *node, const QString &field, const QString &rawUrl, bool showWarning = false);
  QString computePath(const OmNode *node, const QString &field, const OmMFString *urlField, int index,
                      bool showWarning = false);

  QString combinePaths(const QString &rawUrl, const QString &rawParentUrl);

  QString exportResource(const OmNode *node, const QString &url, const QString &sourcePath, const QString &relativeResourcePath,
                         const OmWriter &writer);

  QString missing(const QString &url);
  const QString &missingTexture();
  const QString &missingProtoIcon();
  bool isWeb(const QString &url);
  bool isLocalUrl(const QString &url);
  QString computeLocalAssetUrl(QString url, bool isW3d);
  QString computePrefix(const QString &rawUrl);

  QString remoteAssetRegex(bool capturing);
  const QString &remoteAssetPrefix();

  const QRegularExpression &vrmlResourceRegex();

  QString expressRelativeToWorld(const QString &url);

  void setWorldFileName(const QString &fileName);

};  // namespace OmUrl

#endif
