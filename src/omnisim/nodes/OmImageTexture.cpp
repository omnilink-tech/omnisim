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

#include "OmImageTexture.hpp"

#include "OmApplication.hpp"
#include "OmApplicationInfo.hpp"
#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmLog.hpp"
#include "OmMFString.hpp"
#include "OmMathsUtilities.hpp"
#include "OmNetwork.hpp"
#include "OmPreferences.hpp"
#include "OmRgb.hpp"
#include "OmSFBool.hpp"
#include "OmSimulationState.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QIODevice>
#include <QtCore/QMap>
#include <QtGui/QImageReader>

#include <assimp/material.h>

#include <utility>

namespace {
  struct DecodedImageCacheEntry {
    QImage image;
    qint64 bytes;
    quint64 lastUse;
    bool transparent;
  };

  QMap<QString, DecodedImageCacheEntry> gDecodedImages;
  qint64 gDecodedImageBytes = 0;
  quint64 gDecodedImageUseSerial = 0;

  qint64 decodedImageCacheLimit() {
    bool ok = false;
    const int configuredMb = qEnvironmentVariableIntValue("OMNISIM_DECODED_TEXTURE_CACHE_MB", &ok);
    const int mb = ok && configuredMb >= 0 ? configuredMb : 256;
    return static_cast<qint64>(mb) * 1024 * 1024;
  }

  QString textureCacheKey(const QString &filePath) {
    const QFileInfo info(filePath);
    const QString identity = info.canonicalFilePath().isEmpty() ? info.absoluteFilePath() : info.canonicalFilePath();
    // Image scaling is preference-dependent, so it is part of the identity in
    // addition to path, size and mtime.  A file edit therefore cannot reuse a
    // stale decoded or GPU texture on the next reload.
    const int quality = OmPreferences::instance()->value("OpenGL/textureQuality", 4).toInt();
    return QString("%1|%2|%3|q%4")
      .arg(QDir::cleanPath(identity))
      .arg(info.size())
      .arg(info.lastModified().toMSecsSinceEpoch())
      .arg(quality);
  }

  void pruneDecodedImageCache() {
    const qint64 limit = decodedImageCacheLimit();
    while (gDecodedImageBytes > limit && !gDecodedImages.isEmpty()) {
      auto oldest = gDecodedImages.begin();
      for (auto it = gDecodedImages.begin(); it != gDecodedImages.end(); ++it) {
        if (it->lastUse < oldest->lastUse)
          oldest = it;
      }
      gDecodedImageBytes -= oldest->bytes;
      gDecodedImages.erase(oldest);
    }
  }

  // Headless texture-decode gate (default ON; OMNISIM_EAGER_TEXTURE_DECODE=1 is the
  // exact-revert hatch). During a WORLD LOAD in a process launched with
  // --no-rendering / --no-window (CLI intent captured at argument-parse time in
  // OmSimulationState::startedWithoutRendering(), so a preferences override cannot
  // fake it), decoding texture pixels is pure waste UNLESS something in the world
  // reads them for its simulation output. Why each conjunct is safe:
  //   - startedWithoutRendering(): nothing will ever draw the main view in this
  //     process; the wgpu renderer draws a material untextured when image() is
  //     NULL, so even a stray render request degrades gracefully.
  //   - isLoading(): the gate covers ONLY the eager load-time decode. updateUrl()
  //     is also connected to the url field (postFinalize), so a runtime URL edit
  //     from the scene tree or a supervisor still decodes normally.
  //   - !needsTextures(): OmSimulationWorld's pre-finalize scan sets this true when
  //     the world contains any Camera (offscreen render reads pixels), infra-red
  //     DistanceSensor (reads the hit surface's red channel) or Pen (paints into
  //     the texture) -- the three simulation consumers of texture pixels. pickColor
  //     is already lazy (decodes on demand), and mIsMainTextureTransparent has zero
  //     external readers, so neither needs the eager decode.
  // 2026-09-02: value-parsed. This read was presence-gated, so `OMNISIM_EAGER_TEXTURE_DECODE=0`
  // ARMED the revert (the OMNISIM_REQUIRE_NEWTON trap); unset/empty/"0"/"false"/"off" now keep the
  // gate and anything else reverts. OmSimulationWorld.cpp's load-time log line parses it the same way.
  bool eagerTextureDecodeForced() {
    static const bool forced = []() {
      const QString v = QString::fromUtf8(qgetenv("OMNISIM_EAGER_TEXTURE_DECODE")).trimmed().toLower();
      return !(v.isEmpty() || v == "0" || v == "false" || v == "off");
    }();
    return forced;
  }

  bool skipHeadlessEagerTextureDecode() {
    const OmWorld *const world = OmWorld::instance();
    return OmSimulationState::instance()->startedWithoutRendering() && world && world->isLoading() &&
           !world->needsTextures() && !eagerTextureDecodeForced();
  }
}  // namespace

void OmImageTexture::init() {
  mExternalTexture = false;
  mExternalTextureRatio.setXy(1.0, 1.0);
  mExternalTextureData = NULL;
  mExternalTextureWidth = 0;
  mExternalTextureHeight = 0;
  mImage = NULL;
  mUsedFiltering = 0;
  mIsMainTextureTransparent = true;
  mRole = "";
  mDownloader = NULL;
}

void OmImageTexture::initFields() {
  mUrl = findMFString("url");
  mRepeatS = findSFBool("repeatS");
  mRepeatT = findSFBool("repeatT");
  mFiltering = findSFInt("filtering");
}

OmImageTexture::OmImageTexture(OmTokenizer *tokenizer) : OmBaseNode("ImageTexture", tokenizer) {
  init();
  initFields();
}

OmImageTexture::OmImageTexture(const OmNode &other) : OmBaseNode(other) {
  init();
  initFields();
}

OmImageTexture::OmImageTexture(const OmImageTexture &other) : OmBaseNode(other) {
  init();
  initFields();
}

OmImageTexture::OmImageTexture(const aiMaterial *material, aiTextureType textureType, const QString &parentPath) :
  OmBaseNode("ImageTexture") {
  init();

  aiString pathString("");
  material->GetTexture(textureType, 0, &pathString);
  // generate URL of the texture from URL of collada/wavefront file
  mUrl = new OmMFString(QStringList(OmUrl::combinePaths(QString(pathString.C_Str()), parentPath)));

  // init remaining variables with default values
  mRepeatS = new OmSFBool(true);
  mRepeatT = new OmSFBool(true);
  mFiltering = new OmSFInt(4);
}

OmImageTexture::~OmImageTexture() {
  destroyWrenTexture();

  if (mIsShallowNode) {
    delete mUrl;
    delete mRepeatS;
    delete mRepeatT;
    delete mFiltering;
  }
}

void OmImageTexture::downloadAssets() {
  if (mUrl->size() == 0)
    return;

  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0));
  if (!OmUrl::isWeb(completeUrl) || OmNetwork::instance()->isCachedWithMapUpdate(completeUrl))
    return;

  delete mDownloader;
  mDownloader = OmDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
  if (!OmWorld::instance()->isLoading() || mIsShallowNode)  // URL changed from the scene tree or supervisor
    connect(mDownloader, &OmDownloader::complete, this, &OmImageTexture::downloadUpdate);
  mDownloader->download();
}

void OmImageTexture::downloadUpdate() {
  updateUrl();
  OmWorld::instance()->viewpoint()->emit refreshRequired();
}

void OmImageTexture::preFinalize() {
  OmBaseNode::preFinalize();

  updateUrl();
  updateRepeatS();
  updateRepeatT();
  updateFiltering();
}

void OmImageTexture::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mUrl, &OmMFString::changed, this, &OmImageTexture::updateUrl);
  connect(mRepeatS, &OmSFBool::changed, this, &OmImageTexture::updateRepeatS);
  connect(mRepeatT, &OmSFBool::changed, this, &OmImageTexture::updateRepeatT);
  connect(mFiltering, &OmSFInt::changed, this, &OmImageTexture::updateFiltering);
  connect(OmPreferences::instance(), &OmPreferences::changedByUser, this, &OmImageTexture::updateFiltering);

  if (!OmWorld::instance()->isLoading())
    emit changed();
}

bool OmImageTexture::loadTexture() {
  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0));
  const bool isWebAsset = OmUrl::isWeb(completeUrl);
  if (isWebAsset && !OmNetwork::instance()->isCachedWithMapUpdate(completeUrl))
    return false;

  const QString filePath = isWebAsset ? OmNetwork::instance()->get(completeUrl) : path();
  const QString cacheKey = textureCacheKey(filePath);
  auto cached = gDecodedImages.find(cacheKey);
  if (cached != gDecodedImages.end()) {
    cached->lastUse = ++gDecodedImageUseSerial;
    mImage = new QImage(cached->image);
    mIsMainTextureTransparent = cached->transparent;
    return true;
  }

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    warn(tr("Texture file could not be read: '%1'").arg(filePath));
    return false;
  }
  const bool r = loadTextureData(&file);
  file.close();
  if (r) {
    DecodedImageCacheEntry entry;
    entry.image = *mImage;
    entry.bytes = static_cast<qint64>(entry.image.sizeInBytes());
    entry.lastUse = ++gDecodedImageUseSerial;
    entry.transparent = mIsMainTextureTransparent;
    gDecodedImageBytes += entry.bytes;
    gDecodedImages.insert(cacheKey, entry);
    pruneDecodedImageCache();
  }
  return r;
}

bool OmImageTexture::loadTextureData(QIODevice *device) {
  QImageReader imageReader(device);
  QSize textureSize = imageReader.size();
  const int imageWidth = textureSize.width();
  const int imageHeight = textureSize.height();
  int w = OmMathsUtilities::nextPowerOf2(imageWidth);
  int h = OmMathsUtilities::nextPowerOf2(imageHeight);
  if (w != imageWidth || h != imageHeight)
    warn(tr("Texture image size of '%1' is not a power of two: rescaling it from %2x%3 to %4x%5.")
           .arg(path())
           .arg(imageWidth)
           .arg(imageHeight)
           .arg(w)
           .arg(h));

  const int quality = OmPreferences::instance()->value("OpenGL/textureQuality", 4).toInt();
  const int multiplier = quality / 2;
  const int divider = 4 * pow(0.5, multiplier);      // 0: 4, 1: 2, 2: 1
  const int maxResolution = pow(2, 9 + multiplier);  // 0: 512, 1: 1024, 2: 2048
  if (divider != 1) {
    if (w >= maxResolution)
      w /= divider;
    if (h >= maxResolution)
      h /= divider;
  }

  mImage = new QImage();

  if (!imageReader.read(mImage)) {
    warn(tr("Cannot load texture '%1': %2.").arg(path()).arg(imageReader.errorString()));
    return false;
  }

  mIsMainTextureTransparent = mImage->pixelFormat().alphaUsage() == QPixelFormat::UsesAlpha;

  if (mImage->format() != QImage::Format_ARGB32) {
    QImage tmp = mImage->convertToFormat(QImage::Format_ARGB32);
    mImage->swap(tmp);
  }

  if (mImage->width() != w || mImage->height() != h) {
    // 0: Qt:FastTransformation
    // 1: Qt:SmoothTransformation
    Qt::TransformationMode mode = (quality % 2) ? Qt::SmoothTransformation : Qt::FastTransformation;
    QImage tmp = mImage->scaled(w, h, Qt::KeepAspectRatio, mode);
    mImage->swap(tmp);
  }

  return true;
}

void OmImageTexture::updateWrenTexture() {
  // D1.4: the WREN GPU texture is gone. This keeps the CPU image lifecycle IDENTICAL
  // (loaded, replaced and cleared at exactly the same points) because the wgpu renderer
  // and pickColor read image(). Deduplicated loads go through the in-memory decoded-image
  // cache in loadTexture() (the old wr_texture_2d cache no longer exists).
  //
  // Before finalization updateUrl() can be called repeatedly while fields are
  // still being assembled.  Do not tear down the just-loaded image in that
  // phase; after finalization, replacement follows the normal lifecycle.
  if (isPostFinalizedCalled())
    destroyWrenTexture();

  if (mUrl->size() == 0)
    return;

  const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0), true);
  if (completeUrl.isEmpty())
    return;

  // Headless texture-decode gate: one guard, placed HERE rather than around the
  // updateWrenTexture() call in updateUrl(), so it covers every caller AND skips
  // the decoded-image cache path too (loadTexture() owns the cache lookup/insert
  // /prune -- no point churning a cache for images we are not decoding). The
  // URL normalization, download scheduling and download-error handling above and
  // in updateUrl() run unchanged; the downloader cleanup below runs unchanged.
  // Full predicate rationale: see skipHeadlessEagerTextureDecode() at the top of
  // this file. Gated on OmWorld::isLoading(), so a runtime url edit (updateUrl is
  // connected to the field in postFinalize) still decodes.
  if (!skipHeadlessEagerTextureDecode()) {
    if (loadTexture()) {
      if (mUrl->size() == 0)
        return;
    }
  }

  delete mDownloader;
  mDownloader = NULL;
}

void OmImageTexture::destroyWrenTexture() {
  // D1.4: the WREN GPU objects are gone; the CPU image teardown keeps the same lifecycle.
  delete mImage;
  mImage = NULL;
}

void OmImageTexture::updateUrl() {
  if (mUrl->size() == 0)
    return;

  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  const int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->blockSignals(true);
    mUrl->setItem(i, item.replace("\\", "/"));
    mUrl->blockSignals(false);
  }

  if (n > 0) {
    const QString &completeUrl = OmUrl::computePath(this, "url", mUrl->item(0));
    if (OmUrl::isWeb(completeUrl)) {
      if (mDownloader && !mDownloader->error().isEmpty()) {
        warn(mDownloader->error());  // failure downloading or file does not exist (404)
        // since the URL is invalid the currently loaded texture should be removed (if any)
        destroyWrenTexture();
        delete mDownloader;
        mDownloader = NULL;
        return;
      }

      if (!OmNetwork::instance()->isCachedWithMapUpdate(completeUrl) && mDownloader == NULL) {
        downloadAssets();  // URL was changed from the scene tree or supervisor
        return;
      }
    }
  }

  updateWrenTexture();

  if (isPostFinalizedCalled())
    emit changed();
}

void OmImageTexture::updateRepeatS() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmImageTexture::updateRepeatT() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmImageTexture::updateFiltering() {
  if (OmFieldChecker::resetIntIfNotInRangeWithIncludedBounds(this, mFiltering, 0, 5, 4))
    return;

  // The filtering level has an upper bound defined by the maximum supported anisotropy level.
  // A warning is not produced here because the maximum anisotropy level is not up to the user
  // and may be repeatedly shown even though a minimum requirement warning was already given.
  const int maxFiltering = OmPreferences::instance()->value("OpenGL/textureFiltering").toInt();
  mUsedFiltering = qMin(mFiltering->value(), maxFiltering);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmImageTexture::setExternalTexture(const unsigned char *image, int width, int height, double ratioX, double ratioY) {
  destroyWrenTexture();

  mExternalTexture = true;
  mExternalTextureRatio.setXy(ratioX, ratioY);
  mExternalTextureData = image;
  mExternalTextureWidth = width;
  mExternalTextureHeight = height;

  emit changed();
}

void OmImageTexture::removeExternalTexture() {
  destroyWrenTexture();

  mExternalTexture = false;
  mExternalTextureRatio.setXy(1.0, 1.0);
  mExternalTextureData = NULL;
  mExternalTextureWidth = 0;
  mExternalTextureHeight = 0;

  updateWrenTexture();
}

int OmImageTexture::width() const {
  if (mExternalTexture)
    return mExternalTextureWidth;
  if (mImage)
    return mImage->width();
  return 0;
}

int OmImageTexture::height() const {
  if (mExternalTexture)
    return mExternalTextureHeight;
  if (mImage)
    return mImage->height();
  return 0;
}

bool OmImageTexture::hasImage() const {
  // D1.4: presence test replacing the old `wrenTexture() != NULL` check — true once CPU
  // pixels are available (a file texture loaded, or an external Display-fed texture).
  return mImage != NULL || (mExternalTexture && mExternalTextureData != NULL);
}

void OmImageTexture::pickColor(const OmVector2 &uv, OmRgb &pickedColor) {
  const unsigned char *data = NULL;
  int w = 0;
  int h = 0;
  if (mExternalTexture) {
    if (!mExternalTextureData)
      return;
    w = mExternalTextureWidth * mExternalTextureRatio.x();
    h = mExternalTextureHeight * mExternalTextureRatio.y();
    data = mExternalTextureData;
  } else {
    if (!mImage) {
      if (mUrl->size() == 0)
        return;  // D1.4: was guarded by the WREN texture handle; nothing to load
      if (!loadTexture() || !mImage) {
        pickedColor.setValue(1.0, 1.0, 1.0);
        return;
      }
    }
    w = mImage->width();
    h = mImage->height();
    data = mImage->bits();
  }

  double u = uv.x();
  double v = uv.y();

  // bound uv into 0.0 and 1.0 according to the repeatX fields
  if (mRepeatS->value()) {
    u = fmod(u, 1.0);
    if (u < 0.0)
      u += 1.0;
  } else
    u = qBound(0.0, u, 1.0);

  if (mRepeatT->value()) {
    v = fmod(v, 1.0);
    if (v < 0.0)
      v += 1.0;
  } else
    v = qBound(0.0, v, 1.0);

  const int index = 4 * (w * qMin((int)(v * h), h - 1) + qMin((int)(u * w), w - 1));
  pickedColor.setByteValue((int)data[index + 2], (int)data[index + 1], (int)data[index]);

  // debug
  // printf("pickedColor (u=%f, v=%f): (r=%f g=%f b=%f)\n", u, v, pickedColor.red(), pickedColor.green(), pickedColor.blue());
}

const QString OmImageTexture::path() const {
  if (mUrl->size() == 0)
    return "";
  if (OmUrl::isWeb(mUrl->item(0)))
    return mUrl->item(0);

  return OmUrl::computePath(this, "url", mUrl, 0);
}

bool OmImageTexture::exportNodeHeader(OmWriter &writer) const {
  if (!writer.isW3d() || !isUseNode() || mRole.isEmpty())
    return OmBaseNode::exportNodeHeader(writer);

  writer << "<" << w3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (defNode())
    writer << " USE=\'" + QString::number(defNode()->uniqueId()) + "\'";
  writer << " role=\'" << mRole << "\' ></" + w3dName() + ">";
  return true;
}

void OmImageTexture::exportNodeFields(OmWriter &writer) const {
  OmBaseNode::exportNodeFields(writer);

  exportMFResourceField("url", mUrl, writer.relativeTexturesPath(), writer);

  if (writer.isW3d()) {
    if (!mRole.isEmpty())
      writer << " role=\'" << mRole << "\'";
  }
}

QStringList OmImageTexture::customExportedFields() const {
  QStringList fields;
  fields << "url";
  return fields;
}

void OmImageTexture::exportShallowNode(const OmWriter &writer) const {
  if (!writer.isW3d() || mUrl->size() == 0)
    return;

  // note: the texture of the shallow nodes needs to be exported only if the URL is locally defined but not of type
  // 'omnisim://' since this case would be converted to a remote one that targets the current branch
  if (!OmUrl::isWeb(mUrl->item(0)) && !OmUrl::isLocalUrl(mUrl->item(0)) && !OmWorld::isW3dStreaming())
    OmUrl::exportResource(this, mUrl->item(0), OmUrl::computePath(this, "url", mUrl, 0), writer.relativeTexturesPath(), writer);
}

QStringList OmImageTexture::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "url"
         << "repeatS"
         << "repeatT"
         << "filtering";
  return fields;
}
