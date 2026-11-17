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

#include "OmRenderingDevice.hpp"

#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmPreferences.hpp"
#include "OmSFInt.hpp"
#include "OmSFVector2.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmWrenTextureOverlay.hpp"


QList<OmRenderingDevice *> OmRenderingDevice::cRenderingDevices;

void OmRenderingDevice::init() {
  // initialized some stuff
  mOverlay = NULL;
  mSetupWidth = -1;
  mSetupHeight = -1;
  mHasBeenSetup = false;
  mIsExternalWindowEnabled = false;

  // Fields initialization
  mWidth = findSFInt("width");
  mHeight = findSFInt("height");
}

OmRenderingDevice::OmRenderingDevice(const QString &modelName, OmTokenizer *tokenizer) : OmSolidDevice(modelName, tokenizer) {
  init();
}

OmRenderingDevice::OmRenderingDevice(const OmRenderingDevice &other) : OmSolidDevice(other) {
  init();
}

OmRenderingDevice::OmRenderingDevice(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmRenderingDevice::~OmRenderingDevice() {
  delete mOverlay;
  cRenderingDevices.removeAll(this);
}

void OmRenderingDevice::preFinalize() {
  OmSolidDevice::preFinalize();

  cRenderingDevices << this;

  updateWidth();
  updateHeight();
}

void OmRenderingDevice::postFinalize() {
  OmSolidDevice::postFinalize();

  if (mWidth)
    connect(mWidth, &OmSFInt::changed, this, &OmRenderingDevice::updateWidth);
  if (mHeight)
    connect(mHeight, &OmSFInt::changed, this, &OmRenderingDevice::updateHeight);
}

int OmRenderingDevice::width() const {
  if (mHasBeenSetup)
    return mSetupWidth;
  else
    return mWidth->value();
}

int OmRenderingDevice::height() const {
  if (mHasBeenSetup)
    return mSetupHeight;
  else
    return mHeight->value();
}

void OmRenderingDevice::setup() {
  if (mHasBeenSetup)
    return;
  if (mWidth)
    mSetupWidth = mWidth->value();
  if (mHeight)
    mSetupHeight = mHeight->value();
  mHasBeenSetup = true;
}

void OmRenderingDevice::updateWidth() {
  if (mWidth && mWidth->value() < 1) {
    parsingWarn(tr("Invalid 'width': changed to 1."));
    mWidth->setValue(1);
    return;
  }

  // warn in case of width modification after the setup
  if (mHasBeenSetup)
    warn(tr("'width' has been modified. This modification will be taken into account after saving and reloading the world."));
}

void OmRenderingDevice::updateHeight() {
  if (mHeight && mHeight->value() < 1) {
    parsingWarn(tr("Invalid 'height': changed to 1."));
    mHeight->setValue(1);
    return;
  }

  // warn in case of height modification after the setup
  if (mHasBeenSetup)
    warn(tr("'height' has been modified. This modification will be taken into account after saving and reloading the world."));
}

void OmRenderingDevice::setPixelSize(double pixelSize) {
  if (mOverlay) {
    bool success = mOverlay->resize(pixelSize);
    if (!success)
      emit overlayVisibilityChanged(false);
  }
}

void OmRenderingDevice::moveWindow(int dx, int dy) {
  if (mOverlay)
    mOverlay->translateInPixels(dx, dy);
}

double OmRenderingDevice::pixelSize() const {
  if (mOverlay)
    return mOverlay->pixelSize();
  return 1.0;
}

OmRenderingDevice *OmRenderingDevice::fromMousePosition(int x, int y) {
  int iMax = -1;
  int maxZorder = -1;
  int size = cRenderingDevices.size();

  for (int i = 0; i < size; i++) {
    const OmWrenTextureOverlay *textureOverlay = cRenderingDevices.at(i)->overlay();
    if (textureOverlay && textureOverlay->isVisible()) {
      int currentZorder = textureOverlay->zOrder();
      if (textureOverlay->isInside(x, y) && currentZorder > maxZorder) {
        iMax = i;
        maxZorder = currentZorder;
      }
    }
  }

  if (iMax < 0 || iMax >= size)
    return NULL;

  return cRenderingDevices.at(iMax);
}

void OmRenderingDevice::toggleOverlayVisibility(bool enabled, bool emitSignal) {
  if (!mOverlay)
    return;

  if (!enabled) {
    // hide overlay
    if (mIsExternalWindowEnabled)
      emit closeWindow();
    else
      mOverlay->setVisible(enabled, areOverlaysEnabled());
  } else if (!mIsExternalWindowEnabled) {
    // show overlay
    mOverlay->setVisible(enabled, areOverlaysEnabled());
  }

  if (emitSignal)
    emit overlayVisibilityChanged(enabled);
}

bool OmRenderingDevice::isOverlayEnabled() const {
  return (mOverlay && mOverlay->isEnabled()) || mIsExternalWindowEnabled;
}

QStringList OmRenderingDevice::perspective() const {
  QStringList perspective;
  if (mOverlay)
    perspective = mOverlay->perspective();
  return perspective;
}

void OmRenderingDevice::restorePerspective(QStringList &perspective) {
  if (mOverlay) {
    mOverlay->restorePerspective(perspective, areOverlaysEnabled());
    if (!perspective.isEmpty())
      emit restoreWindowPerspective(this, perspective);
  }
}

// D1.4 (WREN deletion): the overlay is a state-only model with no GL textures, so there is
// no texture id to hand out any more. The accessors survive for the pop-out-window API and
// always answer 0; the window repaints from the CPU image buffer.
int OmRenderingDevice::textureGLId() const {
  return 0;
}

int OmRenderingDevice::backgroundTextureGLId() const {
  return 0;
}

int OmRenderingDevice::maskTextureGLId() const {
  return 0;
}

int OmRenderingDevice::foregroundTextureGLId() const {
  return 0;
}

void OmRenderingDevice::enableExternalWindow(bool enabled) {
  mIsExternalWindowEnabled = enabled;
  mOverlay->setVisible(!enabled, areOverlaysEnabled());
  emit overlayStatusChanged(!mIsExternalWindowEnabled);
  if (isPostFinalizedCalled() && !isBeingDeleted())
    OmWrenRenderingContext::instance()->requestView3dRefresh();
}

bool OmRenderingDevice::areOverlaysEnabled() const {
  // cppcheck-suppress knownConditionTrueFalse
  if (nodeType() == WB_NODE_CAMERA)
    return !OmPreferences::instance()->value("View3d/hideAllCameraOverlays").toBool();
  // cppcheck-suppress knownConditionTrueFalse
  if (nodeType() == WB_NODE_RANGE_FINDER)
    return !OmPreferences::instance()->value("View3d/hideAllRangeFinderOverlays").toBool();
  return !OmPreferences::instance()->value("View3d/hideAllDisplayOverlays").toBool();
}
