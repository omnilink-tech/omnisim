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

#ifndef OM_WREN_TEXTURE_OVERLAY_HPP
#define OM_WREN_TEXTURE_OVERLAY_HPP

//
// The device-inset overlay MODEL: screen rectangle, z-order, visibility, drag/resize state
// and the device's source pixel buffer.
//
// D1.4 (WREN deletion): this class no longer owns any GPU object. It used to hold the WREN
// overlay + textures and derive its rectangle from them; the rectangle, z-order and
// visibility are now first-class state here, and the DRAWING is OmHudOverlay (P7), which
// reads the SAME left()/top()/width()/height() accessors the isInside*() hit tests use --
// one rectangle by construction. The name is kept to spare its ~15 consumers a rename.
//

#include "OmVector2.hpp"

#include <QtCore/QObject>

class OmWrenTextureOverlay : public QObject {
  Q_OBJECT

public:
  enum TextureType { TEXTURE_TYPE_BGRA, TEXTURE_TYPE_DEPTH };

  enum OverlayType { OVERLAY_TYPE_CAMERA, OVERLAY_TYPE_RANGE_FINDER, OVERLAY_TYPE_DISPLAY, OVERLAY_TYPE_COUNT };

  static void updateOverlayDimensions();
  static void setElementsVisible(OverlayType type, bool visible);

  OmWrenTextureOverlay(void *data, int width, int height, TextureType textureType, OverlayType overlayType,
                       double maxRange = 1.0, bool needTransparency = true);
  virtual ~OmWrenTextureOverlay();

  // get pixel functions
  void convertMousePositionToIndex(int x, int y, int &u, int &v, bool &resizeArea) const;

  // update function
  void requestUpdateTexture() { mRequestUpdateTexture = true; }

  // getters
  // overlay visible in the scene
  bool isVisible() const;
  // isEnabled() corresponds to the check status in the Robot menu:
  // could differ from isVisible() because of global settings
  // or because already open in external window
  bool isEnabled() const;
  bool isInside(int x, int y) const;
  bool isInsideResizeArea(int x, int y) const;
  bool isInsideCloseButton(int x, int y) const;
  int zOrder() const { return mZOrder; }
  // The overlay's SCREEN RECT, in 3D-view logical pixels, origin top-left. OmHudOverlay
  // draws this rectangle and the mouse handlers hit-test it -- same four numbers.
  int left() const;
  int top() const;
  int width() const;
  int height() const;
  // The device's own image buffer -- BGRA8 for a Camera/Display, float metres for a
  // RangeFinder (see textureType()). Never owned here; it is the memory-mapped image the
  // controller reads, so it is live without a copy.
  const void *sourceData() const { return mData; }
  int sourceWidth() const { return mWidth; }
  int sourceHeight() const { return mHeight; }
  TextureType textureType() const { return mTextureType; }
  OverlayType overlayType() const { return mOverlayType; }
  double maxRange() const { return mMaxRange; }
  double pixelSize() const { return mPixelSize; }
  // return current texture size in pixels
  void size(int &width, int &height) const {
    width = mWidth * mPixelSize;
    height = mHeight * mPixelSize;
  }

  // setters
  void setMaxRange(double r) { mMaxRange = r; }
  void putOnTop() const;

  bool resize(double pixelSize, bool showIfNeeded = true);
  void translateInPixels(int dx, int dy);
  void updatePercentagePosition(double x, double y);
  void setVisible(bool visible, bool globalOverlaysEnabled);

  // enable panel overlay showing the effective size of the texture
  void enableDefaultSizeBackground(bool enabled) { mShowDefaultSize = enabled; }

  QStringList perspective() const;
  void restorePerspective(QStringList &perspective, bool globalOverlaysEnabled);

public slots:
  void updateTexture();

signals:
  void textureUpdated();

private:
  void allocateBlackImageIntoData();
  void applyChanges();
  void applyPosition();
  bool applyDimensions(bool showIfNeeded);
  void view3dSize(double &w, double &h) const;

  TextureType mTextureType;
  OverlayType mOverlayType;

  void *mData;
  int mWidth;
  int mHeight;
  double mMaxRange;
  bool mRequestUpdateTexture;
  double mPixelSize;
  bool mDataHasBeenAllocated;
  OmVector2 mPercentagePosition;  // position expressed as a percentage of the 3D view size

  // Derived screen rect (fractions of the 3D view, WREN-overlay convention).
  double mFracWidth;
  double mFracHeight;
  bool mVisible;
  bool mShowDefaultSize;
  mutable int mZOrder;
};

#endif
