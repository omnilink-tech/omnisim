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

#ifndef OM_HUD_OVERLAY_HPP
#define OM_HUD_OVERLAY_HPP

//
// OmHudOverlay -- P7 of the WREN-deletion runbook: the DEVICE HUD INSETS, their frames, and the
// supervisor LABELS, collected as backend-neutral screen-space quads so the wgpu main view can
// draw what OmWrenTextureOverlay / OmWrenLabelOverlay used to draw.
//
// Those two classes are WREN scene objects. When the main view flipped to wgpu on 2026-08-19 the
// Camera / RangeFinder / Display insets and every wb_supervisor_set_label() went dark -- one of
// the five GUI regressions that flip shipped. This module does NOT re-implement them: it reads
// the SAME objects the GUI already hit-tests and hands the renderer the same rectangles.
//
// The rectangle is the whole point. OmWrenTextureOverlay::left()/top()/width()/height() are what
// isInside() -- and therefore OmRenderingDevice::fromMousePosition(), the close button, the
// resize corner and OmDragOverlayEvent -- test against. Drawing from the same accessors makes
// "the inset you see is the inset you can click" structural rather than a coincidence between
// two formulas.
//
// Cost when nothing is shown: anyVisible() walks the rendering-device list (a handful of
// pointers, no scene graph) and the label list, and returns false. No allocation, no
// rasterisation, no GPU work -- the caller skips collect() entirely.
//
// Deliberately not a Q_OBJECT (no signals/slots) -> OTHER_SOURCES, no moc.
//

#include <vector>

class OmWgpuRenderTarget;
struct OmWgpuHudQuad;

namespace OmHudOverlay {

  // One collected quad. `pixels` points either into `owned` (converted / rasterised content) or
  // straight at the device's live image buffer (no copy). Resolved by collect() after the vector
  // has stopped growing, so it is always valid on return.
  struct Quad {
    float x = 0.0f, y = 0.0f, w = 0.0f, h = 0.0f;  // 3D-view LOGICAL pixels, origin top-left
    std::vector<unsigned char> owned;              // BGRA8 when the pixels had to be built
    const unsigned char *pixels = nullptr;         // BGRA8; null -> flat `color` fill
    unsigned int srcW = 0, srcH = 0;
    unsigned long long key = 0, revision = 0;
    float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    bool flipV = false;
  };

  // Is there anything at all to draw? One walk of two small lists; no allocation.
  bool anyVisible();

  // Collect every visible device inset (frame + image) and every live label into `out`.
  // `viewW`/`viewH` are the 3D view's LOGICAL pixel size -- the frame the WREN viewport and the
  // mouse handlers both work in.
  void collect(int viewW, int viewH, std::vector<Quad> &out);

  // Convert the collected quads into the renderer's struct, scaling logical -> target pixels by
  // `dpr` (target width / logical width). Returns the number written, capped at `maxOut`.
  unsigned int toRenderQuads(const std::vector<Quad> &in, double dpr, OmWgpuHudQuad *out,
                             unsigned int maxOut);

  // Drop cached rasterised label images for labels that no longer exist. Called on world unload.
  void clearCaches();

}  // namespace OmHudOverlay

#endif
