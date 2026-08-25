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

#include "OmUrl.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmApplicationInfo.hpp"
#include "OmField.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmMFString.hpp"
#include "OmNetwork.hpp"
#include "OmNode.hpp"
#include "OmProject.hpp"
#include "OmProtoModel.hpp"
#include "OmStandardPaths.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QUrl>

namespace {
  static QString gWorldFileName;
};

void OmUrl::setWorldFileName(const QString &fileName) {
  ::gWorldFileName = fileName;
}

const QString &OmUrl::missingTexture() {
  const static QString missingTexture = OmStandardPaths::resourcesPath() + "images/missing_texture.png";
  return missingTexture;
}

const QString &OmUrl::missingProtoIcon() {
  const static QString missingProtoIcon = OmStandardPaths::resourcesPath() + "images/missing_proto_icon.png";
  return missingProtoIcon;
}

QString OmUrl::missing(const QString &url) {
  const QString suffix = url.mid(url.lastIndexOf('.') + 1).toLower();
  const QStringList textureSuffixes = {"png", "jpg", "jpeg"};
  if (textureSuffixes.contains(suffix, Qt::CaseInsensitive))
    return missingTexture();

  return "";
}

QString OmUrl::computePath(const OmNode *node, const QString &field, const OmMFString *urlField, int index, bool showWarning) {
  // check if mUrl is empty
  if (urlField->size() < 1)
    return "";

  // get the URL at specified index
  const QString &url = urlField->item(index);
  return computePath(node, field, url, showWarning);
}

QString OmUrl::computePath(const OmNode *node, const QString &field, const QString &rawUrl, bool showWarning) {
  QString url = resolveUrl(rawUrl);
  // check if the first URL is empty
  if (url.isEmpty()) {
    if (node)
      node->parsingWarn(QObject::tr("First item of '%1' field is empty.").arg(field));
    else
      OmLog::warning(QObject::tr("Missing '%1' value.").arg(field), false, OmLog::PARSING);
    return missing(url);
  }

  // note: web urls need to be checked first, otherwise it would also pass the isRelativePath() condition
  if (isWeb(url))
    return url;

  if (QDir::isRelativePath(url)) {
    const OmField *f = node->findField(field, true);
    assert(f);
    const OmNode *protoNode = node->containingProto(false);

    protoNode = OmVrmlNodeUtilities::findFieldProtoScope(f, protoNode);

    QString parentUrl;
    if (protoNode) {
      // note: derived PROTO are a special case because instances of the intermediary ancestors from which it is defined don't
      // persist after the build process, hence why we keep track of the scope while building the node itself
      if (protoNode->proto()->isDerived()) {
        if (OmFileUtil::isLocatedInDirectory(f->scope(), OmStandardPaths::cachedAssetsPath()))
          parentUrl = OmNetwork::instance()->getUrlFromEphemeralCache(f->scope());
        else
          parentUrl = f->scope();
      } else {
        const QString &protoUrl = protoNode->proto()->url();
        if (isWeb(protoUrl)) {
          // If the PROTO has a remote URL, resolve it to a local path
          QRegularExpression re(remoteAssetRegex(false));
          QString localPath = protoUrl;
          localPath.replace(re, OmStandardPaths::omniSimHomePath());
          if (QFileInfo(localPath).exists())
            parentUrl = localPath;
          else
            parentUrl = protoUrl;
        } else
          parentUrl = protoUrl;
      }
    } else
      parentUrl = gWorldFileName;

    url = combinePaths(url, parentUrl);
  }

  if (isWeb(url) || QFileInfo(url).exists())
    return url;

  if (showWarning)
    node->warn(QObject::tr("Unable to find resource at '%1'.").arg(url));

  return missing(rawUrl);
}

QString OmUrl::resolveUrl(const QString &rawUrl) {
  if (rawUrl.isEmpty())
    return rawUrl;

  QString url = rawUrl;
  url.replace("\\", "/");

  if (isWeb(url))
    return url;

  if (isLocalUrl(url))
    return QDir::cleanPath(url.replace("omnisim://", OmStandardPaths::omniSimHomePath()));

  return QDir::cleanPath(url);
}

QString OmUrl::exportResource(const OmNode *node, const QString &url, const QString &sourcePath,
                              const QString &relativeResourcePath, const OmWriter &writer) {
  // in addition to writing the node, we want to ensure that the resource file exists
  // at the expected location. If not, we should copy it, possibly creating the expected
  // directory structure.
  const QFileInfo urlFileInfo(url);
  const QString fileName = urlFileInfo.fileName();
  const QString expectedRelativePath = relativeResourcePath + fileName;
  const QString expectedPath = writer.path() + "/" + expectedRelativePath;

  if (expectedPath == sourcePath)  // everything is fine, the file is where we expect it
    return expectedPath;

  // otherwise, we need to copy it
  // but first, we need to check that folders exists and create them if they don't exist
  const QFileInfo fi(expectedPath);
  if (!QDir(fi.path()).exists())
    QDir(writer.path()).mkpath(fi.path());

  if (QFile::exists(expectedPath)) {
    if (OmFileUtil::areIdenticalFiles(sourcePath, expectedPath))
      return expectedRelativePath;
    else {
      const QString baseName = urlFileInfo.completeBaseName();
      const QString extension = urlFileInfo.suffix();

      for (int i = 1; i < 100; ++i) {  // number of trials before failure
        QString newRelativePath = relativeResourcePath + baseName + '.' + QString::number(i) + '.' + extension;

        const QString newAbsolutePath = writer.path() + "/" + newRelativePath;
        if (QFileInfo(newAbsolutePath).exists()) {
          if (OmFileUtil::areIdenticalFiles(sourcePath, newAbsolutePath))
            return newRelativePath;
        } else {
          QFile::copy(sourcePath, newAbsolutePath);
          return newRelativePath;
        }
      }

      node->warn(QObject::tr("Failure exporting resource, too many resources share the same name: %1.").arg(url));
      return "";
    }
  } else {  // simple case
    QFile::copy(sourcePath, expectedPath);
    return expectedRelativePath;
  }
}

bool OmUrl::isWeb(const QString &url) {
  return url.startsWith("https://") || url.startsWith("http://");
}

bool OmUrl::isLocalUrl(const QString &url) {
  return url.startsWith("omnisim://") ||
         OmFileUtil::isLocatedInInstallationDirectory(url, true);
}

QString OmUrl::computeLocalAssetUrl(QString url, bool isW3d) {
  if (!isW3d)
    return url.replace(OmStandardPaths::omniSimHomePath(), "omnisim://");

  if (!OmApplicationInfo::repo().isEmpty() && !OmApplicationInfo::branch().isEmpty()) {
    // when streaming locally, build the URL from branch.txt in order to serve 'omnisim://' assets
    const QString prefix =
      "https://raw.githubusercontent.com/" + OmApplicationInfo::repo() + "/" + OmApplicationInfo::branch() + "/";
    return url.replace("omnisim://", prefix).replace(OmStandardPaths::omniSimHomePath(), prefix);
  }

  // when streaming from a distribution or nightly build, use the actual url
  return url;
}

QString OmUrl::computePrefix(const QString &rawUrl) {
  const QString url = OmFileUtil::isLocatedInDirectory(rawUrl, OmStandardPaths::cachedAssetsPath()) ?
                        OmNetwork::instance()->getUrlFromEphemeralCache(rawUrl) :
                        rawUrl;

  if (isWeb(url)) {
    QRegularExpression re(remoteAssetRegex(true));
    QRegularExpressionMatch match = re.match(url);
    if (match.hasMatch())
      return match.captured(0);
  }

  return QString();
}

QString OmUrl::remoteAssetRegex(bool capturing) {
  // Matches the remote-asset prefix that the packaging scripts bake into a distributed
  // release (see scripts/packaging/update_urls.py), so that such a URL can be mapped back
  // onto the local install instead of being downloaded.
  //
  // OmniSim's own repository comes FIRST: a packaged OmniSim release emits
  // 'omnilink-tech/omnisim' URLs. The legacy 'cyberbotics/webots' alternative is retained
  // (non-capturing, so group numbering is unchanged for all callers) purely so that worlds
  // and PROTOs authored against upstream Webots keep resolving.
  //
  // NOTE the '\.' in the tag character class: upstream's tags ("R2025a") never contained a
  // dot, but OmniSim's are dotted semver ("v5.0.0"), so without it no OmniSim release tag
  // would match and the baked URLs would never resolve back to the local install.
  static QString regex =
    "https://raw.githubusercontent.com/(?:omnilink-tech/omnisim|cyberbotics/webots)/[a-zA-Z0-9\\_\\-\\+\\.]+/";
  return capturing ? "(" + regex + ")" : regex;
}

const QString &OmUrl::remoteAssetPrefix() {
  static QString url;
  if (url.isEmpty()) {
    // Use local WEBOTS_HOME path instead of remote GitHub URL.
    // This ensures all assets resolve locally without network dependency.
    url = OmStandardPaths::omniSimHomePath();
  }
  return url;
}

const QRegularExpression &OmUrl::vrmlResourceRegex() {
  static QRegularExpression resources("\"([^\"]*)\\.(jpe?g|png|hdr|obj|stl|dae|wav|mp3|proto)\"",
                                      QRegularExpression::CaseInsensitiveOption);
  return resources;
}

QString OmUrl::combinePaths(const QString &rawUrl, const QString &rawParentUrl) {
  // use cross-platform forward slashes
  QString url = rawUrl;
  url.replace("\\", "/");
  QString parentUrl = rawParentUrl;
  parentUrl.replace("\\", "/");

  // cases where no URL manipulation is necessary
  if (isWeb(url))
    return url;

  if (QDir::isAbsolutePath(url))
    return QDir::cleanPath(url);

  if (OmUrl::isLocalUrl(url)) {
    // URL fall-back mechanism: only trigger if the parent is a world file, and the file (omnisim://) does not exist
    if (OmWorldFileFormat::isWorldFile(parentUrl) &&
        !QFileInfo(QDir::cleanPath(url.replace("omnisim://", OmStandardPaths::omniSimHomePath()))).exists()) {
      OmLog::error(QObject::tr("URL '%1' changed by fallback mechanism. Ensure you are opening the correct world.").arg(url));
      return url.replace("omnisim://", OmUrl::remoteAssetPrefix());
    }

    // infer URL based on parent's url
    const QString &prefix = OmUrl::computePrefix(parentUrl);
    if (!prefix.isEmpty())
      return url.replace("omnisim://", prefix);

    if (parentUrl.isEmpty() || OmUrl::isLocalUrl(parentUrl) || QDir::isAbsolutePath(parentUrl))
      return QDir::cleanPath(url.replace("omnisim://", OmStandardPaths::omniSimHomePath()));

    return QString();
  }

  if (QDir::isRelativePath(url)) {
    if (OmFileUtil::isLocatedInDirectory(parentUrl, OmStandardPaths::cachedAssetsPath()))
      parentUrl = OmNetwork::instance()->getUrlFromEphemeralCache(parentUrl);
    // if it is not available in those folders, infer the URL based on the parent's url
    if (OmUrl::isWeb(parentUrl) || QDir::isAbsolutePath(parentUrl) || OmUrl::isLocalUrl(parentUrl)) {
      // remove filename from parent url
      parentUrl = parentUrl.sliced(0, parentUrl.lastIndexOf("/") + 1);
      if (OmUrl::isLocalUrl(parentUrl))
        parentUrl.replace("omnisim://", OmStandardPaths::omniSimHomePath());

      if (OmUrl::isWeb(parentUrl))
        return QUrl(parentUrl).resolved(QUrl(url)).toString();
      else
        return QDir::cleanPath(QDir(parentUrl).absoluteFilePath(url));
    }
  }

  OmLog::error(QObject::tr("Impossible to infer URL from '%1' and '%2'").arg(rawUrl).arg(rawParentUrl));
  return QString();
}

QString OmUrl::expressRelativeToWorld(const QString &url) {
  return QDir(QFileInfo(gWorldFileName).absolutePath()).relativeFilePath(url);
}
