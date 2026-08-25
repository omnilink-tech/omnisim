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

#ifndef OM_WGPU_IMAGE_ADAPTER_HPP
#define OM_WGPU_IMAGE_ADAPTER_HPP

//
// OmWgpuImageAdapter — R3.5b of engine-migration-plan.md §14.3.
//
// Glue layer between Qt's QImage (which is where OmniSim's
// OmImageTexture keeps the original RGBA pixel data — `mImage`
// stays alive after wren::Texture2d destroys its own staging copy
// at GPU upload time) and OmWgpuTextureCache (which wants tightly
// packed RGBA8Unorm bytes).
//
// Source of truth: OmImageTexture loads ARGB32 (Format_ARGB32) at
// `OmImageTexture::createImage` and we hold a `QImage *mImage` for
// the rest of the node's lifetime. The adapter converts to
// Format_RGBA8888 (tight memory-order RGBA on all endians) and
// hands the bytes to OmWgpuTextureCache::acquire.
//
// The conversion is one Qt call + a single byte copy; the cost is
// O(width*height) per upload, paid once per process per texture
// (cache hits the second time).
//
// MeshId convention: the caller picks. Typically reinterpret the
// QImage* or the OmImageTexture* as uint64_t for stability.
//

#include <cstdint>

class QImage;
class OmWgpuTextureCache;
struct OmWgpuTextureHandle;

namespace OmWgpuImageAdapter {

  // Convert + upload the QImage's pixels into the cache.
  // - Returns {nullptr, nullptr, 0, 0} if the image is null, has
  //   zero dimensions, or the cache backend isn't usable.
  // - The QImage may be in any format Qt's convertToFormat
  //   supports; we always go to Format_RGBA8888 internally.
  // - The cache keys on `textureId`, so repeat calls with the
  //   same id skip the conversion + upload entirely.
  OmWgpuTextureHandle acquireFromQImage(OmWgpuTextureCache &cache, uint64_t textureId,
                                        const QImage &image);

}  // namespace OmWgpuImageAdapter

#endif
