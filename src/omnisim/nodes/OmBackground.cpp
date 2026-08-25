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

#include "OmBackground.hpp"

#include "OmApplication.hpp"
#include "OmApplicationInfo.hpp"
#include "OmDirectionalLight.hpp"
#include "OmDownloadManager.hpp"
#include "OmDownloader.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmGroup.hpp"
#include "OmLight.hpp"
#include "OmLog.hpp"
#include "OmMFColor.hpp"
#include "OmMFString.hpp"
#include "OmMathsUtilities.hpp"
#include "OmNetwork.hpp"
#include "OmNodeOperations.hpp"
#include "OmPreferences.hpp"
#include "OmSFNode.hpp"
#include "OmSFString.hpp"
#include "OmStandardPaths.hpp"
#include "OmUrl.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"


#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtGui/QImage>
#include <QtGui/QImageReader>

// P4a (WREN retirement): stb_image is gone from this file — it existed solely to decode the
// authored *IrradianceUrl HDR faces, which are retired (parsed, warned about, never decoded).

QList<OmBackground *> OmBackground::cBackgroundList;

static const QString gDirections[6] = {"right", "left", "top", "bottom", "front", "back"};

static const QString gUrlNames(int i) {
  return gDirections[i] + "Url";
}

static const QString gIrradianceUrlNames(int i) {
  return gDirections[i] + "IrradianceUrl";
}

static int gCoordinateSystemSwap(int i) {
  static const int enu_swap[] = {5, 4, 0, 1, 3, 2};
  if (OmWorld::instance()->worldInfo()->coordinateSystem() == "ENU")
    return enu_swap[i];
  else  // "NUE" or "EUN"
    return i;
}

static int gCoordinateSystemRotate(int i) {
  static const int enu_rotate[] = {90, -90, 0, 180, -90, -90};
  if (OmWorld::instance()->worldInfo()->coordinateSystem() == "ENU")
    return enu_rotate[i];
  else  // "NUE" or "EUN"
    return 0;
}

void OmBackground::init() {
  mSkyColor = findMFColor("skyColor");
  mLuminosity = findSFDouble("luminosity");
  mAtmosphericSky = findSFString("atmosphericSky");
  for (int i = 0; i < 6; ++i) {
    mUrlFields[i] = findMFString(gUrlNames(i));
    // P4a: retired — parsed so legacy worlds keep loading, never downloaded or decoded.
    mIrradianceUrlFields[i] = findMFString(gIrradianceUrlNames(i));
    mTexture[i] = NULL;
    mDownloader[i] = NULL;
  }
  mIrradianceRetirementWarned = false;
  mTextureHasAlpha = false;
  mTextureSize = 0;
  mUrlCount = 0;
}

OmBackground::OmBackground(OmTokenizer *tokenizer) : OmBaseNode("Background", tokenizer) {
  init();
  if (tokenizer == NULL)
    mSkyColor->setItem(0, OmRgb(0.15, 0.45, 1), false);
}

OmBackground::OmBackground(const OmBackground &other) : OmBaseNode(other) {
  init();
}

OmBackground::OmBackground(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmBackground::~OmBackground() {
  const bool firstInstanceDeleted = isFirstInstance();

  cBackgroundList.removeAll(this);

  if (firstInstanceDeleted) {
    OmBackground *newFirstInstance = firstInstance();
    if (newFirstInstance != NULL)
      // activate next Background node
      newFirstInstance->activate();
  }

  if (!OmWorld::instance()->isCleaning())
    emit OmWrenRenderingContext::instance()->backgroundColorChanged();


  for (int i = 0; i < 6; i++)
    delete mTexture[i];
}

void OmBackground::downloadAsset(const QString &url, int index, bool postpone) {
  // P4a: only the six cubemap *Url faces download; the retired *IrradianceUrl fields never reach
  // this function.
  const QString &completeUrl = OmUrl::computePath(this, gUrlNames(index), url);
  if (!OmUrl::isWeb(completeUrl))
    return;

  delete mTexture[index];
  mTexture[index] = NULL;

  delete mDownloader[index];
  mDownloader[index] = OmDownloadManager::instance()->createDownloader(QUrl(completeUrl), this);
  if (postpone)
    connect(mDownloader[index], &OmDownloader::complete, this, &OmBackground::downloadUpdate);
  mDownloader[index]->download();
}

void OmBackground::downloadAssets() {
  for (int i = 0; i < 6; ++i) {
    if (mUrlFields[i]->size() && !OmNetwork::instance()->isCachedWithMapUpdate(mUrlFields[i]->item(0)))
      downloadAsset(mUrlFields[i]->item(0), i, false);
  }
}

void OmBackground::downloadUpdate() {
  // we need that all downloads are complete before proceeding with the update of the cube map
  for (int i = 0; i < 6; ++i) {
    if (mDownloader[i] && !mDownloader[i]->hasFinished())
      return;
  }

  updateCubemap();
  OmWorld::instance()->viewpoint()->emit refreshRequired();
}

void OmBackground::preFinalize() {
  OmBaseNode::preFinalize();
  cBackgroundList << this;
}

void OmBackground::postFinalize() {
  OmBaseNode::postFinalize();

  // P4a: rendering-independent, so the warning also fires under --no-rendering / OMNISIM_NO_GL.
  warnIrradianceUrlRetired();

  if (isFirstInstance())
    activate();
  else
    parsingWarn(tr("Only one Background node is allowed. The current node won't be taken into account."));
}

void OmBackground::activate() {
  // D1.4: no GL/WREN precondition -- the CPU cubemap faces + field state feed the wgpu sky.
  if (!areWrenObjectsInitialized())
    createWrenObjects();

  connect(mLuminosity, &OmSFDouble::changed, this, &OmBackground::updateLuminosity);
  connect(mSkyColor, &OmMFColor::changed, this, &OmBackground::updateColor);
  connect(OmWorld::instance()->viewpoint(), &OmViewpoint::cameraModeChanged, this, &OmBackground::updateCubemap);
  for (int i = 0; i < 6; ++i) {
    connect(mUrlFields[i], &OmMFString::changed, this, &OmBackground::updateCubemap);
    // P4a: a retired field edited at runtime (scene tree / supervisor) warns instead of silently
    // doing nothing; it no longer triggers a cubemap rebuild because nothing reads it.
    connect(mIrradianceUrlFields[i], &OmMFString::changed, this, &OmBackground::warnIrradianceUrlRetired);
  }
  // Re-evaluate atmospheric vs cubemap mode whenever the preset
  // changes.  Routed through updateCubemap so the same teardown +
  // rebuild path runs in both cases.
  connect(mAtmosphericSky, &OmSFString::changed, this, &OmBackground::updateCubemap);

  updateColor();

  updateCubemap();
}

void OmBackground::createWrenObjects() {
  OmBaseNode::createWrenObjects();
  // D1.4: the WREN skybox + HDR clear quad died with WREN; the wgpu sky reads
  // atmosphericSkyPreset()/skyColor()/cubemapTexture() directly.
}

void OmBackground::updateColor() {
  if (OmFieldChecker::resetMultipleColorIfInvalid(this, mSkyColor))
    return;

  emit OmWrenRenderingContext::instance()->backgroundColorChanged();
}

void OmBackground::updateCubemap() {
  if (areWrenObjectsInitialized()) {
    // if some textures are to be downloaded again (changed from the scene tree or supervisor)
    // we should postpone the applySkyBoxToWren
    bool postpone = false;
    mUrlCount = 0;
    for (int i = 0; i < 6; i++) {
      if (mUrlFields[i]->size())
        mUrlCount++;
    }
    const bool hasCompleteBackground = mUrlCount == 6;
    if (isPostFinalizedCalled()) {
      for (int i = 0; i < 6; i++) {
        if (hasCompleteBackground) {
          const QString &completeUrl = OmUrl::computePath(this, gUrlNames(i), mUrlFields[i]->item(0));
          if (OmUrl::isWeb(completeUrl) && !OmNetwork::instance()->isCachedWithMapUpdate(completeUrl) &&
              mDownloader[i] == NULL) {
            downloadAsset(completeUrl, i, true);
            postpone = true;
          } else {
            delete mTexture[i];
            mTexture[i] = 0;
          }
        }
        // P4a: the retired *IrradianceUrl faces are neither downloaded nor decoded.
      }
    }

    if (!postpone) {
      bool destroy = false;
      if (!hasCompleteBackground) {
        if (mUrlCount > 0) {
          warn(tr("Incomplete background cubemap"));
          destroy = true;
        }
      } else
        for (int i = 0; i < 6; i++)
          if (!loadTexture(i)) {
            destroy = true;
            break;
          }
      if (destroy)
        emit OmWrenRenderingContext::instance()->backgroundColorChanged();
      else if (hasCompleteBackground || mUrlCount == 0)
        emit cubemapChanged();
    }
  }
}

void OmBackground::updateLuminosity() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mLuminosity, 1.0))
    return;

  emit luminosityChanged();
}

bool OmBackground::loadTexture(int i) {
  if (mTexture[i])
    return true;

  const int urlFieldIndex = gCoordinateSystemSwap(i);
  // if a side is not defined, it should not even attempt to load the texture
  assert(mUrlFields[urlFieldIndex]->size() != 0);

  QString url = OmUrl::computePath(this, gUrlNames(i), mUrlFields[urlFieldIndex]->item(0), true);
  if (url == OmUrl::missingTexture() || url.isEmpty())
    return false;

  if (OmUrl::isWeb(url)) {
    if (OmNetwork::instance()->isCachedWithMapUpdate(url))
      url = OmNetwork::instance()->get(url);  // get reference to the corresponding file in the cache
    else {
      if (mDownloader[i] && !mDownloader[i]->error().isEmpty())
        warn(mDownloader[i]->error());
      return false;  // should not move past this point unless the file is available in the cache
    }
  }

  QImageReader imageReader(url);
  if (!imageReader.canRead()) {
    warn(tr("Cannot read texture file: '%1'").arg(url));
    return false;
  }

  const QSize textureSize = imageReader.size();
  if (textureSize.width() != textureSize.height()) {
    warn(tr("The %1Url '%2' is not a square image (its width doesn't equal its height).").arg(gDirections[i], url));
    return false;
  }

  for (int j = 0; j < 6; j++)
    if (mTexture[j]) {
      if (textureSize.width() == mTextureSize)
        break;
      else {
        warn(tr("Texture dimension mismatch between %1Url and %2Url.").arg(gDirections[i], gDirections[j]));
        return false;
      }
    }

  mTextureSize = textureSize.width();
  mTexture[i] = new QImage;
  if (!imageReader.read(mTexture[i])) {
    warn(tr("Cannot load texture '%1': %2.").arg(imageReader.fileName()).arg(imageReader.errorString()));
    return false;
  }

  for (int j = 0; j < 6; j++) {
    if (mTexture[j] && j != i) {
      if (mTexture[i]->hasAlphaChannel() == mTextureHasAlpha)
        break;
      warn(tr("Alpha channel mismatch with %1Url.").arg(gDirections[i]));
      delete mTexture[i];
      mTexture[i] = NULL;
      return false;
    }
  }

  mTextureHasAlpha = mTexture[i]->hasAlphaChannel();
  if (mTexture[i]->format() != QImage::Format_ARGB32) {
    QImage tmp = mTexture[i]->convertToFormat(QImage::Format_ARGB32);
    mTexture[i]->swap(tmp);
  }
  const int rotate = gCoordinateSystemRotate(i);
  // FIXME: this texture rotation should be performed by OpenGL or in the shader to get a better performance
  if (rotate != 0) {
    QPoint center = mTexture[i]->rect().center();
    QTransform matrix;
    matrix.translate(center.x(), center.y());
    matrix.rotate(rotate);
    QImage tmp = mTexture[i]->transformed(matrix);
    mTexture[i]->swap(tmp);
  }

  if (mDownloader[urlFieldIndex]) {
    delete mDownloader[urlFieldIndex];
    mDownloader[urlFieldIndex] = NULL;
  }

  return true;
}

// P4a (WREN retirement): the six *IrradianceUrl fields are retired. They keep PARSING — an
// undeclared field is an ERROR that takes a headless run's exit code to 1 (the
// Solid.immersionProperties precedent), and Background.wrl still declares all six — but the
// authored HDR faces are never downloaded or decoded any more, and the irradiance cube is always
// baked from the sky itself (the atmosphericSky preset's bake, the image cubemap, or skyColor).
// One warning per node, at load and again if a supervisor authors the field at runtime.
void OmBackground::warnIrradianceUrlRetired() {
  if (mIrradianceRetirementWarned)
    return;
  QStringList authored;
  for (int i = 0; i < 6; ++i)
    if (mIrradianceUrlFields[i]->size() > 0)
      authored << gIrradianceUrlNames(i);
  if (authored.isEmpty())
    return;
  mIrradianceRetirementWarned = true;
  warn(tr("The %1 field(s) are retired and ignored: OmniSim bakes the irradiance cube from the sky itself "
          "(the atmosphericSky preset, the cubemap *Url faces, or skyColor). Remove them from the world.")
         .arg(authored.join(", ")));
}

OmRgb OmBackground::skyColor() const {
  return (mSkyColor->size() > 0 ? mSkyColor->item(0) : OmRgb());
}

QString OmBackground::atmosphericSkyPreset() const {
  return mAtmosphericSky ? mAtmosphericSky->value().trimmed() : QString();
}

void OmBackground::exportNodeFields(OmWriter &writer) const {
  OmBaseNode::exportNodeFields(writer);

  if (writer.isW3d()) {
    QString backgroundFileNames[6];
    for (int i = 0; i < 6; ++i) {
      if (mUrlFields[i]->size() == 0)
        continue;

      const QString &resolvedURL = OmUrl::computePath(this, gUrlNames(i), mUrlFields[i], 0);
      backgroundFileNames[i] = exportResource(mUrlFields[i]->item(0), resolvedURL, writer.relativeTexturesPath(), writer);
    }

    QString irradianceFileNames[6];
    for (int i = 0; i < 6; ++i) {
      if (mIrradianceUrlFields[i]->size() == 0)
        continue;

      const QString &resolvedURL = OmUrl::computePath(this, gIrradianceUrlNames(i), mIrradianceUrlFields[i], 0);
      irradianceFileNames[i] =
        exportResource(mIrradianceUrlFields[i]->item(0), resolvedURL, writer.relativeTexturesPath(), writer);
    }

    writer << " ";
    for (int i = 0; i < 6; ++i) {
      if (!backgroundFileNames[i].isEmpty())
        writer << gUrlNames(i) << "='\"" << backgroundFileNames[i] << "\"' ";
      if (!irradianceFileNames[i].isEmpty())
        writer << gIrradianceUrlNames(i) << "='\"" << irradianceFileNames[i] << "\"' ";
    }
  } else {
    for (int i = 0; i < 6; ++i) {
      exportMFResourceField(gUrlNames(i), mUrlFields[i], writer.relativeTexturesPath(), writer);
      exportMFResourceField(gIrradianceUrlNames(i), mIrradianceUrlFields[i], writer.relativeTexturesPath(), writer);
    }
  }
}

QStringList OmBackground::customExportedFields() const {
  QStringList fields;
  for (int i = 0; i < 6; ++i) {
    fields << gUrlNames(i);
    fields << gIrradianceUrlNames(i);
  }
  return fields;
}
