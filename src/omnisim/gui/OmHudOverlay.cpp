// Copyright 2026 OmniLink
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

#include "OmHudOverlay.hpp"

#include "OmRenderingDevice.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWrenLabelOverlay.hpp"
#include "OmWrenTextureOverlay.hpp"

#include <QtCore/QFileInfo>
#include <QtCore/QHash>
#include <QtCore/QString>
#include <QtGui/QColor>
#include <QtGui/QFont>
#include <QtGui/QFontDatabase>
#include <QtGui/QFontMetrics>
#include <QtGui/QImage>
#include <QtGui/QPainter>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

  // WREN's own inset frame: 1 px, and the per-device-type colour taken from the ogre textures
  // (OmWrenTextureOverlay.cpp cBorderSizeHorizontal / cBorderColors). Reproduced rather than
  // invented so a wgpu inset is recognisably the same widget.
  const float kBorderPx = 1.0f;
  const float kBorderColors[3][4] = {
    {1.000f, 0.000f, 1.000f, 1.0f},  // OVERLAY_TYPE_CAMERA
    {1.000f, 0.815f, 0.000f, 1.0f},  // OVERLAY_TYPE_RANGE_FINDER
    {0.000f, 1.000f, 1.000f, 1.0f},  // OVERLAY_TYPE_DISPLAY
  };

  // A device image changes every step, so its GPU texture is re-uploaded every frame. This
  // counter is the revision that forces that; labels use their own revision() instead and
  // therefore upload nothing at all while their text is unchanged.
  unsigned long long gFrameSeq = 1;

  // ---- label rasterisation cache ------------------------------------------------------------
  // Rasterising text with QPainter is cheap but not free, and a label typically does not change
  // for thousands of frames. Keyed by the label pointer; invalidated by its revision() or by a
  // change in the on-screen pixel size (a viewport resize).
  struct LabelRaster {
    unsigned long long revision = ~0ull;
    int w = 0, h = 0;
    std::vector<unsigned char> bgra;
  };
  QHash<const OmWrenLabelOverlay *, LabelRaster> gLabelCache;

  // Font files arrive as absolute .ttf paths (OmSupervisorUtilities resolves them). Load each
  // once into the application font database and remember the family it registered.
  QString familyForFontFile(const QString &path) {
    static QHash<QString, QString> cache;
    QHash<QString, QString>::const_iterator it = cache.constFind(path);
    if (it != cache.constEnd())
      return it.value();
    QString family;
    if (!path.isEmpty() && QFileInfo(path).isFile()) {
      const int id = QFontDatabase::addApplicationFont(path);
      if (id >= 0) {
        const QStringList families = QFontDatabase::applicationFontFamilies(id);
        if (!families.isEmpty())
          family = families.first();
      }
    }
    if (family.isEmpty())
      family = QStringLiteral("Sans Serif");
    cache.insert(path, family);
    return family;
  }

  // OmWrenLabelOverlay::colorToArray packs (b, g, r, 1-transparency) -- see its own
  // color(int&r,int&g,int&b,float&alpha) getter, which reads mColor[2] as red. Follow that
  // rather than assuming rgba, or every label comes out with red and blue swapped.
  QColor wrenColorToQColor(const float *c) {
    const int b = static_cast<int>(std::lround(std::min(1.0f, std::max(0.0f, c[0])) * 255.0f));
    const int g = static_cast<int>(std::lround(std::min(1.0f, std::max(0.0f, c[1])) * 255.0f));
    const int r = static_cast<int>(std::lround(std::min(1.0f, std::max(0.0f, c[2])) * 255.0f));
    const int a = static_cast<int>(std::lround(std::min(1.0f, std::max(0.0f, c[3])) * 255.0f));
    return QColor(r, g, b, a);
  }

}  // namespace

namespace OmHudOverlay {

  bool anyVisible() {
    const QList<OmRenderingDevice *> &devices = OmRenderingDevice::renderingDevices();
    for (int i = 0; i < devices.size(); ++i) {
      const OmWrenTextureOverlay *ov = devices.at(i)->overlay();
      if (ov && ov->isVisible())
        return true;
    }
    return !OmWrenLabelOverlay::overlays().isEmpty();
  }

  void collect(int viewW, int viewH, std::vector<Quad> &out) {
    out.clear();
    if (viewW <= 1 || viewH <= 1)
      return;
    ++gFrameSeq;

    // ---- device output insets (Camera / RangeFinder / Display) --------------------------------
    const QList<OmRenderingDevice *> &devices = OmRenderingDevice::renderingDevices();
    for (int i = 0; i < devices.size(); ++i) {
      OmWrenTextureOverlay *ov = devices.at(i)->overlay();
      if (!ov || !ov->isVisible())
        continue;
      const int x = ov->left(), y = ov->top(), w = ov->width(), h = ov->height();
      if (w <= 0 || h <= 0)
        continue;
      const int sw = ov->sourceWidth(), sh = ov->sourceHeight();
      if (sw <= 0 || sh <= 0)
        continue;
      const void *data = ov->sourceData();
      if (!data)
        continue;

      // Frame first (drawn under the image), in WREN's own per-type colour.
      const int t = static_cast<int>(ov->overlayType());
      if (t >= 0 && t < 3) {
        Quad frame;
        frame.x = static_cast<float>(x) - kBorderPx;
        frame.y = static_cast<float>(y) - kBorderPx;
        frame.w = static_cast<float>(w) + 2.0f * kBorderPx;
        frame.h = static_cast<float>(h) + 2.0f * kBorderPx;
        frame.pixels = nullptr;  // flat fill
        std::memcpy(frame.color, kBorderColors[t], sizeof(frame.color));
        out.push_back(frame);
      }

      Quad q;
      q.x = static_cast<float>(x);
      q.y = static_cast<float>(y);
      q.w = static_cast<float>(w);
      q.h = static_cast<float>(h);
      q.srcW = static_cast<unsigned int>(sw);
      q.srcH = static_cast<unsigned int>(sh);
      q.key = static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(ov));
      q.revision = gFrameSeq;  // a live device image changes every step
      if (ov->textureType() == OmWrenTextureOverlay::TEXTURE_TYPE_DEPTH) {
        // Exactly OmWrenTextureOverlay::copyDataToTexture's depth conversion: a linear
        // metres -> 0..255 grey by 255/maxRange, opaque. Same numbers, so a RangeFinder inset
        // reads identically to the one WREN drew.
        const float *src = static_cast<const float *>(data);
        const double maxRange = ov->maxRange() > 0.0 ? ov->maxRange() : 1.0;
        const float mul = static_cast<float>(255.0 / maxRange);
        q.owned.resize(static_cast<size_t>(sw) * sh * 4u);
        for (int p = 0, n = sw * sh; p < n; ++p) {
          float v = mul * src[p];
          if (!(v >= 0.0f))
            v = 0.0f;
          if (v > 255.0f)
            v = 255.0f;
          const unsigned char g = static_cast<unsigned char>(v);
          unsigned char *d = q.owned.data() + static_cast<size_t>(p) * 4u;
          d[0] = g;  // B
          d[1] = g;  // G
          d[2] = g;  // R
          d[3] = 255;
        }
      } else {
        // BGRA already -- the byte order the GPU texture is created in, so no copy and no
        // swizzle: the inset shows the exact bytes the controller reads.
        q.pixels = static_cast<const unsigned char *>(data);
      }
      out.push_back(std::move(q));
    }

    // ---- supervisor labels --------------------------------------------------------------------
    const QList<OmWrenLabelOverlay *> &labels = OmWrenLabelOverlay::overlays();
    for (int i = 0; i < labels.size(); ++i) {
      OmWrenLabelOverlay *lab = labels.at(i);
      if (!lab || lab->text().isEmpty())
        continue;
      const float fw = lab->width(), fh = lab->height();  // viewport fractions
      if (!(fw > 0.0f) || !(fh > 0.0f))
        continue;
      const int w = static_cast<int>(std::lround(fw * viewW));
      const int h = static_cast<int>(std::lround(fh * viewH));
      if (w < 1 || h < 1 || w > 4 * viewW || h > 4 * viewH)
        continue;

      LabelRaster &cached = gLabelCache[lab];
      if (cached.revision != lab->revision() || cached.w != w || cached.h != h) {
        QImage img(w, h, QImage::Format_ARGB32);
        // Format_ARGB32 is 0xAARRGGBB in a uint32, i.e. B,G,R,A in memory on a little-endian
        // host -- the BGRA8 the HUD quad wants, with no conversion pass.
        img.fill(wrenColorToQColor(lab->backgroundColor()));
        QPainter p(&img);
        p.setRenderHint(QPainter::TextAntialiasing, true);
        const int lines = std::max(1, lab->linesCount());
        QFont font(familyForFontFile(lab->font()));
        // WREN scales one fixed-size glyph texture so that `lines` lines fill the overlay
        // height; the equivalent here is a pixel size of one line's share of it. 0.78 is the
        // usual cap-height/line-height ratio, which keeps descenders inside the box.
        const int px = std::max(1, static_cast<int>(std::lround(0.78 * h / lines)));
        font.setPixelSize(px);
        p.setFont(font);
        p.setPen(wrenColorToQColor(lab->color()));
        // HORIZONTAL_MARGIN (0.15 of one line height) is WREN's own left inset.
        const int margin = static_cast<int>(std::lround(0.15 * h / lines));
        p.drawText(QRect(margin, 0, std::max(1, w - margin), h),
                   Qt::AlignLeft | Qt::AlignVCenter | Qt::TextDontClip, lab->text());
        p.end();
        cached.revision = lab->revision();
        cached.w = w;
        cached.h = h;
        cached.bgra.assign(img.constBits(), img.constBits() + static_cast<size_t>(w) * h * 4u);
      }

      Quad q;
      q.x = static_cast<float>(lab->x() * viewW);
      q.y = static_cast<float>(lab->y() * viewH);
      q.w = static_cast<float>(w);
      q.h = static_cast<float>(h);
      q.srcW = static_cast<unsigned int>(w);
      q.srcH = static_cast<unsigned int>(h);
      // High bit set so a label key can never collide with a device-overlay pointer key.
      q.key = 0x8000000000000000ull ^ static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(lab));
      q.revision = cached.revision;
      q.owned = cached.bgra;
      out.push_back(std::move(q));
    }

    // Resolve the owned-buffer pointers now that nothing else will be appended.
    for (size_t i = 0; i < out.size(); ++i)
      if (!out[i].owned.empty())
        out[i].pixels = out[i].owned.data();
  }

  unsigned int toRenderQuads(const std::vector<Quad> &in, double dpr, OmWgpuHudQuad *out,
                             unsigned int maxOut) {
    if (!out || maxOut == 0)
      return 0;
    unsigned int n = 0;
    for (size_t i = 0; i < in.size() && n < maxOut; ++i) {
      const Quad &q = in[i];
      if (q.w < 1.0f || q.h < 1.0f)
        continue;
      OmWgpuHudQuad r;
      r.x = static_cast<float>(q.x * dpr);
      r.y = static_cast<float>(q.y * dpr);
      r.w = static_cast<float>(q.w * dpr);
      r.h = static_cast<float>(q.h * dpr);
      r.pixels = q.pixels;
      r.srcW = q.srcW;
      r.srcH = q.srcH;
      r.key = q.key;
      r.revision = q.revision;
      r.color[0] = q.color[0];
      r.color[1] = q.color[1];
      r.color[2] = q.color[2];
      r.color[3] = q.color[3];
      r.flipV = q.flipV;
      out[n++] = r;
    }
    return n;
  }

  void clearCaches() {
    gLabelCache.clear();
  }

}  // namespace OmHudOverlay
