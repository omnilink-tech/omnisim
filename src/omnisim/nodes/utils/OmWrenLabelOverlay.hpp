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

#ifndef OM_WREN_LABEL_OVERLAY_HPP
#define OM_WREN_LABEL_OVERLAY_HPP

//
// Description: helper class managing the textual labels (wb_supervisor_set_label, the drag
// captions, the camera recognition caption).
//
// D1.4 (WREN deletion): STATE-ONLY. The WREN overlay + drawable texture + font died with
// src/wren; OmHudOverlay (P7) rasterises every live label with QPainter from exactly this
// state (text, position, size, colours, revision). The name is kept to spare the ~15
// consumers a rename.
//

#include <QtCore/QList>
#include <QtCore/QString>

class OmWrenLabelOverlay {
public:
  static void cleanup();
  static void updateOverlaysDimensions();

  static OmWrenLabelOverlay *createOrRetrieve(int id, const QString &font);
  static OmWrenLabelOverlay *retrieveById(int id);
  static void removeAllLabels();
  static void removeLabel(int id);
  static int movieCaptionOverlayId() { return 65535; }
  static int dragCaptionOverlayId() { return 65534; }
  static int cameraCaptionOverlayId() { return 65533; }
  static void colorToArray(float *dest, int color);
  // P7 (WREN retirement): every live label, so a non-WREN renderer can draw them.
  static const QList<OmWrenLabelOverlay *> &overlays() { return cLabelOverlays; }

  void setText(const QString &text);
  void setPosition(double x, double y) {
    mX = x;
    mY = y;
  }
  void setSize(double size) {
    mSize = size;
    ++mRevision;  // P7: the label is rasterised at its on-screen size, so this changes pixels
  }
  void setColor(int color) {
    colorToArray(mColor, color);
    ++mRevision;
  }
  void setBackgroundColor(int color) {
    colorToArray(mBackgroundColor, color);
    ++mRevision;
  }

  int id() const { return mId; };
  const QString &text() const { return mText; }
  const QString &font() const { return mFontName; }
  double size() const { return mSize; }
  double x() const { return mX; }
  double y() const { return mY; }
  const float *color() const { return mColor; }
  void position(double &x, double &y) const {
    x = mX;
    y = mY;
  }
  void color(int &r, int &g, int &b, float &alpha) const {
    b = mColor[0] * 255;
    g = mColor[1] * 255;
    r = mColor[2] * 255;
    alpha = mColor[3];
  }

  void moveToPosition(float x, float y) { setPosition(x, y); }
  void updateText(const QString &text) { setText(text); }

  // Historical name: commits the pending state (now pure state, so a no-op kept for its
  // ~10 call sites).
  void applyChangesToWren() {}

  // Overlay height as a fraction of the viewport, WREN's own formula. The WIDTH depended on
  // the rasterised text's aspect ratio, which now lives in OmHudOverlay's QPainter metrics --
  // it returns 0 here and no surviving caller reads it for layout.
  float width() const { return 0.0f; }
  float height() const { return mLinesCount * mSize * 0.5f; }
  int linesCount() const { return mLinesCount; }
  const float *backgroundColor() const { return mBackgroundColor; }
  unsigned long long revision() const { return mRevision; }
  QString getFontError() const { return QString(); }

private:
  static QList<OmWrenLabelOverlay *> cLabelOverlays;
  static const float HORIZONTAL_MARGIN;

  OmWrenLabelOverlay(int id, const QString &font);
  ~OmWrenLabelOverlay();

  int mId;
  QString mText;
  QString mFontName;
  double mX;
  double mY;
  double mSize;
  float mColor[4];
  float mBackgroundColor[4];
  int mLinesCount;
  unsigned long long mRevision;  // bumped whenever the drawn pixels would change
};

#endif
