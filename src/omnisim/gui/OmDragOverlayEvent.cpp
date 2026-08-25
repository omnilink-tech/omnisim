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

#include "OmDragOverlayEvent.hpp"

#include "OmRenderingDevice.hpp"
#include "OmWrenTextureOverlay.hpp"

OmDragOverlayEvent::OmDragOverlayEvent(const QPoint &initialMousePosition, OmRenderingDevice *renderingDevice) :
  mInitialMousePosition(initialMousePosition),
  mRenderingDevice(renderingDevice) {
}

OmDragTranslateOverlayEvent::OmDragTranslateOverlayEvent(const QPoint &initialMousePosition, const QPoint &windowSize,
                                                         OmRenderingDevice *renderingDevice) :
  OmDragOverlayEvent(initialMousePosition, renderingDevice),
  mWindowSize(windowSize) {
  mHalfWidth = (double)mRenderingDevice->width() * mRenderingDevice->pixelSize() * 0.5f / (double)mWindowSize.x();
  mHalfHeight = (double)mRenderingDevice->height() * mRenderingDevice->pixelSize() * 0.5f / (double)mWindowSize.y();
}

void OmDragTranslateOverlayEvent::apply(const QPoint &currentMousePosition) {
  QPoint difference = currentMousePosition - mInitialMousePosition;
  mInitialMousePosition = currentMousePosition;
  mRenderingDevice->moveWindow(difference.x(), difference.y());
}

OmDragResizeOverlayEvent::OmDragResizeOverlayEvent(const QPoint &initialMousePosition, OmRenderingDevice *renderingDevice) :
  OmDragOverlayEvent(initialMousePosition, renderingDevice) {
  mTextureWidthInv = 1.0 / renderingDevice->width();
  mTextureHeightInv = 1.0 / renderingDevice->height();
  int overlayWidth, overlayHeight;
  mRenderingDevice->overlay()->size(overlayWidth, overlayHeight);
  mInitialOverlaySize = QPoint(overlayWidth, overlayHeight);
  renderingDevice->overlay()->enableDefaultSizeBackground(true);
}

OmDragResizeOverlayEvent::~OmDragResizeOverlayEvent() {
  mRenderingDevice->overlay()->enableDefaultSizeBackground(false);
}

void OmDragResizeOverlayEvent::apply(const QPoint &currentMousePosition) {
  QPoint difference = mInitialOverlaySize + currentMousePosition - mInitialMousePosition;
  double scaleX = difference.x() * mTextureWidthInv;
  double scaleY = difference.y() * mTextureHeightInv;
  mRenderingDevice->setPixelSize(scaleX > scaleY ? scaleX : scaleY);
}
