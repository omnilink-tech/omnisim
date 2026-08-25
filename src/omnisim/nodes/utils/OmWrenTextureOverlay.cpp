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

#include "OmWrenTextureOverlay.hpp"

#include "OmWrenRenderingContext.hpp"

#include <QtCore/QFileInfo>
#include <QtCore/QList>
#include <QtCore/QStringList>
#include <QtGui/QImageReader>

#include <cassert>
#include <cstdlib>

static int cResizeIconSize = -1;
static int cCloseIconSize = -1;
static const int cBorderSizeHorizontal = 1;
static const int cBorderSizeVertical = 1;

// registry of live overlays: (overlay, enabled-in-menu)
static QList<std::pair<OmWrenTextureOverlay *, bool> *> cOverlayStatusList;
static std::pair<OmWrenTextureOverlay *, bool> *statusEntry(const OmWrenTextureOverlay *o) {
  for (std::pair<OmWrenTextureOverlay *, bool> *p : cOverlayStatusList)
    if (p->first == o)
      return p;
  return NULL;
}

// z-order: monotonically increasing "put on top" counter (WREN kept the order inside the
// viewport's overlay list; the HUD sorts by this number instead).
static int cTopZOrder = 0;

static int iconSizeFor(const char *fileName) {
  const QString path = QStringLiteral("gl:textures/") + fileName;
  if (!QFileInfo(path).isFile())
    return 16;
  QImageReader r(path);
  const int w = r.size().width();
  return w > 0 ? w : 16;
}

////////////////////////////////////////
//  Constructor  and initializations  //
////////////////////////////////////////

OmWrenTextureOverlay::OmWrenTextureOverlay(void *data, int width, int height, TextureType textureType, OverlayType overlayType,
                                           double maxRange, bool needTransparency) :
  mTextureType(textureType),
  mOverlayType(overlayType),
  mData(data),
  mWidth(width),
  mHeight(height),
  mMaxRange(maxRange),
  mRequestUpdateTexture(true),
  mPixelSize(1.0),
  mDataHasBeenAllocated(false),
  mFracWidth(0.0),
  mFracHeight(0.0),
  mVisible(false),
  mShowDefaultSize(false),
  mZOrder(++cTopZOrder) {
  (void)needTransparency;
  if (!mData)
    allocateBlackImageIntoData();

  if (cCloseIconSize < 0) {
    // The icon PNGs survive (P7's icon work); only their SIZES are needed for the hit regions.
    cCloseIconSize = iconSizeFor("magenta_close_symbol.png");
    cResizeIconSize = iconSizeFor("magenta_resize_symbol.png");
  }

  applyChanges();
  updatePercentagePosition(0.0, 0.0);

  cOverlayStatusList.append(new std::pair<OmWrenTextureOverlay *, bool>(this, true));
  setVisible(false, true);
}

OmWrenTextureOverlay::~OmWrenTextureOverlay() {
  std::pair<OmWrenTextureOverlay *, bool> *e = statusEntry(this);
  if (e) {
    cOverlayStatusList.removeAll(e);
    delete e;
  }

  if (mDataHasBeenAllocated)
    free(mData);
}

bool OmWrenTextureOverlay::resize(double pixelSize, bool showIfNeeded) {
  if (pixelSize <= 0.0)
    return true;

  mPixelSize = pixelSize;
  return applyDimensions(showIfNeeded);
}

void OmWrenTextureOverlay::view3dSize(double &w, double &h) const {
  const OmWrenRenderingContext *const ctx = OmWrenRenderingContext::instance();
  w = ctx && ctx->width() > 0 ? ctx->width() : 1.0;
  h = ctx && ctx->height() > 0 ? ctx->height() : 1.0;
}

void OmWrenTextureOverlay::translateInPixels(int dx, int dy) {
  double vw, vh;
  view3dSize(vw, vh);
  mPercentagePosition += OmVector2(static_cast<double>(dx) / vw, static_cast<double>(dy) / vh);

  applyPosition();
}

void OmWrenTextureOverlay::updatePercentagePosition(double x, double y) {
  mPercentagePosition = OmVector2(x, y);
  applyPosition();
}

bool OmWrenTextureOverlay::applyDimensions(bool showIfNeeded) {
  const float minSize = 32.0f;

  double vw, vh;
  view3dSize(vw, vh);
  const float view3dWidth = static_cast<float>(vw);
  const float view3dHeight = static_cast<float>(vh);

  // Check viewport size is valid
  if (view3dWidth <= 1 || view3dHeight <= 1)
    return true;

  float overlayWidth = mPixelSize * mWidth;
  float overlayHeight = mPixelSize * mHeight;
  const float overlayRatio = overlayWidth / overlayHeight;

  // enforce horizontal constraint
  if (mPercentagePosition.x() * view3dWidth + overlayWidth > view3dWidth) {
    overlayWidth = view3dWidth - mPercentagePosition.x() * view3dWidth - 2.0f * cBorderSizeHorizontal;
    overlayHeight = overlayWidth / overlayRatio;
    mPixelSize = overlayWidth / mWidth;
  }
  // enforce vertical constraint
  if (mPercentagePosition.y() * view3dHeight + overlayHeight > view3dHeight) {
    overlayHeight = view3dHeight - mPercentagePosition.y() * view3dHeight - 2.0f * cBorderSizeVertical;
    overlayWidth = overlayHeight * overlayRatio;
    mPixelSize = overlayHeight / mHeight;
  }
  // enforce minimal size constraint
  if (overlayWidth < minSize || overlayHeight < minSize) {
    if (overlayWidth < overlayHeight) {
      overlayWidth = minSize;
      overlayHeight = overlayWidth / overlayRatio;
      mPixelSize = minSize / mWidth;
    } else {
      overlayHeight = minSize;
      overlayWidth = overlayHeight * overlayRatio;
      mPixelSize = minSize / mHeight;
    }
  }

  if (showIfNeeded && !mVisible)
    mVisible = true;

  mFracWidth = overlayWidth / view3dWidth;
  mFracHeight = overlayHeight / view3dHeight;

  return true;
}

void OmWrenTextureOverlay::applyPosition() {
  double vw, vh;
  view3dSize(vw, vh);
  const float overlayWidth = mPixelSize * mWidth + 2.0f * cBorderSizeHorizontal;
  const float overlayHeight = mPixelSize * mHeight + 2.0f * cBorderSizeVertical;
  mPercentagePosition.setXy(
    qBound(0.0f, static_cast<float>(mPercentagePosition.x()), 1.0f - overlayWidth / static_cast<float>(vw)),
    qBound(0.0f, static_cast<float>(mPercentagePosition.y()), 1.0f - overlayHeight / static_cast<float>(vh)));
}

void OmWrenTextureOverlay::applyChanges() {
  if (!applyDimensions(false))
    return;

  applyPosition();
}

void OmWrenTextureOverlay::updateTexture() {
  if (!mRequestUpdateTexture)
    return;

  // No GPU copy any more: OmHudOverlay samples sourceData() directly each frame. The signal
  // survives for consumers that repaint on it.
  mRequestUpdateTexture = false;
  emit textureUpdated();
}

void OmWrenTextureOverlay::allocateBlackImageIntoData() {
  assert(!mData);

  int imageSize = mWidth * mHeight;
  switch (mTextureType) {
    case TEXTURE_TYPE_BGRA: {
      mData = malloc(4 * imageSize);

      int *dataInt = static_cast<int *>(mData);
      for (int i = 0; i < imageSize; i++)
        dataInt[i] = 0xFF000000;
      break;
    }
    case TEXTURE_TYPE_DEPTH: {
      mData = malloc(sizeof(float) * imageSize);

      float *dataFloat = static_cast<float *>(mData);
      for (int i = 0; i < imageSize; i++)
        dataFloat[i] = 0.0f;
      break;
    }
    default:
      assert(0);
      break;
  }

  mDataHasBeenAllocated = true;
}

/////////////////////////////////////
// Accessors for mElement features //
/////////////////////////////////////

int OmWrenTextureOverlay::left() const {
  double vw, vh;
  view3dSize(vw, vh);
  return mPercentagePosition.x() * vw + cBorderSizeHorizontal;
}

int OmWrenTextureOverlay::top() const {
  double vw, vh;
  view3dSize(vw, vh);
  return mPercentagePosition.y() * vh + cBorderSizeVertical;
}

int OmWrenTextureOverlay::width() const {
  double vw, vh;
  view3dSize(vw, vh);
  return mFracWidth * vw;
}

int OmWrenTextureOverlay::height() const {
  double vw, vh;
  view3dSize(vw, vh);
  return mFracHeight * vh;
}

bool OmWrenTextureOverlay::isVisible() const {
  return mVisible;
}

bool OmWrenTextureOverlay::isEnabled() const {
  const std::pair<OmWrenTextureOverlay *, bool> *e = statusEntry(this);
  return e && e->second;
}

void OmWrenTextureOverlay::putOnTop() const {
  mZOrder = ++cTopZOrder;
}

void OmWrenTextureOverlay::setVisible(bool visible, bool globalOverlaysEnabled) {
  if (visible && globalOverlaysEnabled) {
    if (mPixelSize <= 0)
      resize(1.0, true);

    mVisible = true;
  } else
    mVisible = false;

  std::pair<OmWrenTextureOverlay *, bool> *e = statusEntry(this);
  assert(e);
  if (e)
    e->second = visible;
}

QStringList OmWrenTextureOverlay::perspective() const {
  QStringList perspective;
  const std::pair<OmWrenTextureOverlay *, bool> *e = statusEntry(this);
  perspective << (e && e->second ? "1" : "0");
  perspective << QString::number(mPixelSize);
  perspective << QString::number(mPercentagePosition.x());
  perspective << QString::number(mPercentagePosition.y());
  return perspective;
}

void OmWrenTextureOverlay::restorePerspective(QStringList &perspective, bool globalOverlaysEnabled) {
  assert(perspective.size() >= 4);

  bool visible = perspective.takeFirst() == "1";
  // cppcheck-suppress duplicateAssignExpression
  double pixelsSize = perspective.takeFirst().toDouble();
  // cppcheck-suppress duplicateAssignExpression
  double x = perspective.takeFirst().toDouble();
  // cppcheck-suppress duplicateAssignExpression
  double y = perspective.takeFirst().toDouble();
  resize(pixelsSize);
  updatePercentagePosition(x, y);
  setVisible(visible, globalOverlaysEnabled);
}

/////////////////////////////////////////////////
// Utility function used to display pixel info //
/////////////////////////////////////////////////

void OmWrenTextureOverlay::convertMousePositionToIndex(int x, int y, int &u, int &v, bool &resizeArea) const {
  if (isInside(x, y)) {
    u = (x - left()) * mWidth / width();
    v = (y - top()) * mHeight / height();
    resizeArea = isInsideResizeArea(x, y);
  } else {
    u = -1;
    v = -1;
    resizeArea = false;
  }
}

bool OmWrenTextureOverlay::isInside(int x, int y) const {
  if (!mVisible)
    return false;

  return (x >= left() && x < left() + width() && y >= top() && y < top() + height());
}

bool OmWrenTextureOverlay::isInsideResizeArea(int x, int y) const {
  if (!mVisible)
    return false;

  return ((left() + width() - x) < cResizeIconSize) && ((top() + height() - y) < cResizeIconSize);
}

bool OmWrenTextureOverlay::isInsideCloseButton(int x, int y) const {
  if (!mVisible)
    return false;

  const int right = left() + width();
  return (x >= (right - cCloseIconSize) && x < right && y >= top() && y < (top() + cCloseIconSize));
}

//////////////////////////////////////////////////////
// Static methods for the rendering devices overlay //
//////////////////////////////////////////////////////

void OmWrenTextureOverlay::updateOverlayDimensions() {
  for (const std::pair<OmWrenTextureOverlay *, bool> *p : cOverlayStatusList)
    p->first->applyChanges();
}

void OmWrenTextureOverlay::setElementsVisible(OverlayType type, bool visible) {
  for (std::pair<OmWrenTextureOverlay *, bool> *p : cOverlayStatusList) {
    if (p->first->mOverlayType == type && p->second)  // skip explicitly closed overlays
      p->first->mVisible = visible;
  }
}
