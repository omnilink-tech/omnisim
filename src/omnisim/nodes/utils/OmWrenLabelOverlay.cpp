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

#include "OmWrenLabelOverlay.hpp"

QList<OmWrenLabelOverlay *> OmWrenLabelOverlay::cLabelOverlays;
const float OmWrenLabelOverlay::HORIZONTAL_MARGIN = 0.15f;  // in % of height

OmWrenLabelOverlay *OmWrenLabelOverlay::createOrRetrieve(int id, const QString &font) {
  foreach (OmWrenLabelOverlay *overlay, cLabelOverlays) {
    if (overlay->id() == id) {
      // If the font changed, a new label overlay is created
      if (overlay->font() == font)
        return overlay;
      else
        delete overlay;
    }
  }
  return new OmWrenLabelOverlay(id, font);
}

OmWrenLabelOverlay *OmWrenLabelOverlay::retrieveById(int id) {
  foreach (OmWrenLabelOverlay *overlay, cLabelOverlays) {
    if (overlay->id() == id)
      return overlay;
  }
  return NULL;
}

void OmWrenLabelOverlay::updateOverlaysDimensions() {
  // Pure state now: the HUD derives label dimensions per frame from size()/linesCount().
}

void OmWrenLabelOverlay::cleanup() {
  removeAllLabels();
}

void OmWrenLabelOverlay::removeAllLabels() {
  while (!cLabelOverlays.isEmpty())
    delete cLabelOverlays.takeFirst();
}

void OmWrenLabelOverlay::removeLabel(int id) {
  for (int i = 0, size = cLabelOverlays.size(); i < size; ++i) {
    OmWrenLabelOverlay *overlay = cLabelOverlays[i];
    if (overlay->id() == id) {
      cLabelOverlays.removeAll(overlay);
      delete overlay;
      return;
    }
  }
}

void OmWrenLabelOverlay::setText(const QString &text) {
  if (mText == text)
    return;
  mText = text;
  mLinesCount = 1 + mText.count('\n');
  ++mRevision;  // P7: the drawn pixels changed
}

OmWrenLabelOverlay::OmWrenLabelOverlay(int id, const QString &font) :
  mId(id),
  mFontName(font),
  mX(0.0f),
  mY(0.0f),
  mSize(0.0f),
  mLinesCount(1),
  mRevision(1) {
  cLabelOverlays.append(this);

  setColor(0x0);
  setBackgroundColor(0xFF000000);
}

OmWrenLabelOverlay::~OmWrenLabelOverlay() {
  cLabelOverlays.removeAll(this);
}

void OmWrenLabelOverlay::colorToArray(float *dest, int color) {
  dest[0] = (float)(color & 0xFF) / 255.0f;
  dest[1] = (float)((color >> 8) & 0xFF) / 255.0f;
  dest[2] = (float)((color >> 16) & 0xFF) / 255.0f;
  dest[3] = 1.0f - ((float)((color >> 24) & 0xFF) / 255.0f);
}
