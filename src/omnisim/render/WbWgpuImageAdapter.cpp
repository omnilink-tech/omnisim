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

#include "WbWgpuImageAdapter.hpp"

#include "WbWgpuTextureCache.hpp"

#include <QtGui/QImage>

namespace WbWgpuImageAdapter {

  WbWgpuTextureHandle acquireFromQImage(WbWgpuTextureCache &cache, uint64_t textureId,
                                        const QImage &image) {
    if (image.isNull() || image.width() <= 0 || image.height() <= 0)
      return {nullptr, nullptr, 0, 0};

    // R4 material fidelity: cache-hit fast-path. On repeat frames the texture is
    // already uploaded, so skip the expensive convertToFormat + repack entirely.
    // (Without this, a 144-texture scene re-converts every frame and stalls the
    // render loop — the R3.5b hasEntry() split this comment used to defer.)
    WbWgpuTextureHandle cached;
    if (cache.tryGet(textureId, cached))
      return cached;

    // Qt's Format_RGBA8888 = memory-order RGBA on all platforms (no
    // endian flip). Format_ARGB32 is 0xAARRGGBB native-int, which
    // on little-endian (our targets) means memory-order BGRA — wrong
    // for wgpu's RGBA8Unorm.
    QImage rgba = image.convertToFormat(QImage::Format_RGBA8888);
    const uint32_t w = static_cast<uint32_t>(rgba.width());
    const uint32_t h = static_cast<uint32_t>(rgba.height());

    // QImage rows may be padded for alignment (bytesPerLine >= w*4).
    // Repack into a tight buffer the cache + wgpuQueueWriteTexture
    // expect (wgpu handles the 256-byte stride internally; we just
    // need rows that are exactly w*4 bytes apart).
    const int srcStride = rgba.bytesPerLine();
    const int dstStride = static_cast<int>(w) * 4;
    if (srcStride == dstStride) {
      return cache.acquire(textureId, w, h, rgba.constBits(),
                           static_cast<size_t>(dstStride) * h);
    }

    QByteArray packed;
    packed.resize(dstStride * static_cast<int>(h));
    for (uint32_t y = 0; y < h; ++y) {
      memcpy(packed.data() + y * dstStride, rgba.constScanLine(static_cast<int>(y)), dstStride);
    }
    return cache.acquire(textureId, w, h, packed.constData(),
                         static_cast<size_t>(packed.size()));
  }

}  // namespace WbWgpuImageAdapter
