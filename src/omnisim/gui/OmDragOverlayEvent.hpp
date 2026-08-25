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

#ifndef OM_DRAG_OVERLAY_EVENT_HPP
#define OM_DRAG_OVERLAY_EVENT_HPP

//
// Description: classes allowing to store data related with the device overlay dragging
//

#include "OmAbstractDragEvent.hpp"

class OmRenderingDevice;

// OmDragOverlayEvent abstract class
class OmDragOverlayEvent : public OmDragEvent {
public:
  enum DragOverlayType { TRANSLATE = 0, RESIZE };
  OmDragOverlayEvent(const QPoint &initialMousePosition, OmRenderingDevice *renderingDevice);
  virtual ~OmDragOverlayEvent() override{};
  virtual DragOverlayType type() = 0;
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  QPoint mInitialMousePosition;
  OmRenderingDevice *mRenderingDevice;
};

// OmDragTranslateOverlayEvent class: change the position of an overlay device
//                                    by dragging the mouse
class OmDragTranslateOverlayEvent : public OmDragOverlayEvent {
public:
  OmDragTranslateOverlayEvent(const QPoint &initialMousePosition, const QPoint &windowSize, OmRenderingDevice *renderingDevice);
  virtual ~OmDragTranslateOverlayEvent() override{};
  DragOverlayType type() override { return TRANSLATE; }
  void apply(const QPoint &currentMousePosition) override;

protected:
  QPoint mWindowSize;
  double mHalfWidth;
  double mHalfHeight;
};

// OmDragResizeOverlayEvent class: resize the overlay device
//                                 by dragging the mouse
class OmDragResizeOverlayEvent : public OmDragOverlayEvent {
public:
  OmDragResizeOverlayEvent(const QPoint &initialMousePosition, OmRenderingDevice *renderingDevice);
  virtual ~OmDragResizeOverlayEvent() override;
  DragOverlayType type() override { return RESIZE; }
  void apply(const QPoint &currentMousePosition) override;

protected:
  QPoint mInitialOverlaySize;
  double mTextureWidthInv;
  double mTextureHeightInv;
};

#endif
