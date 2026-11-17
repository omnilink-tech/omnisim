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

#include <cmath>
#include "OmWgpuTextureCache.hpp"

#include "OmLog.hpp"
#include "OmVulkanBackend.hpp"

#include <cstdio>
#include <cstdlib>

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
#    include "webgpu/webgpu.h"
#  endif
#endif

OmWgpuTextureCache::OmWgpuTextureCache(OmVulkanBackend *owner) : mOwner(owner) {
}

OmWgpuTextureCache::~OmWgpuTextureCache() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  for (auto &kv : mEntries) {
    if (kv.second.view)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(kv.second.view));
    if (kv.second.texture)
      wgpuTextureRelease(static_cast<WGPUTexture>(kv.second.texture));
  }
#  endif
#endif
  mEntries.clear();
  mTotalBytes = 0;
}

void OmWgpuTextureCache::evictToBudget() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  // Drop the least-recently-used entries until total VRAM is under budget. Keeps at least one entry so a
  // single huge texture can't loop forever. O(n) scan per eviction; n is bounded by the budget.
  while (mTotalBytes > kBudgetBytes && mEntries.size() > 1) {
    auto lru = mEntries.begin();
    for (auto it = mEntries.begin(); it != mEntries.end(); ++it)
      if (it->second.lastUsed < lru->second.lastUsed)
        lru = it;
    if (lru->second.view)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(lru->second.view));
    if (lru->second.texture)
      wgpuTextureRelease(static_cast<WGPUTexture>(lru->second.texture));
    mTotalBytes -= lru->second.bytes;
    mEntries.erase(lru);
  }
#  endif
#endif
}

bool OmWgpuTextureCache::evictStale() {
  bool evicted = false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  // Only entries NOT touched since the previous rebuild are candidates (see the header).
  while (mTotalBytes > kBudgetBytes && mEntries.size() > 1) {
    auto lru = mEntries.end();
    for (auto it = mEntries.begin(); it != mEntries.end(); ++it)
      if (it->second.lastUsed < mFrameStart && (lru == mEntries.end() || it->second.lastUsed < lru->second.lastUsed))
        lru = it;
    if (lru == mEntries.end())
      break;  // everything left is the working set: exceed the budget rather than re-upload it
    if (lru->second.view)
      wgpuTextureViewRelease(static_cast<WGPUTextureView>(lru->second.view));
    if (lru->second.texture)
      wgpuTextureRelease(static_cast<WGPUTexture>(lru->second.texture));
    mTotalBytes -= lru->second.bytes;
    mEntries.erase(lru);
    evicted = true;
  }
  if (mTotalBytes > kBudgetBytes && !mBudgetWarned) {
    mBudgetWarned = true;
    OmLog::info(QString("[OmWgpuTextureCache] the scene's texture working set (%1 MB in %2 textures) exceeds the "
                        "%3 MB VRAM budget; nothing is evicted while it is in use, so VRAM stays above budget for "
                        "this scene (lower OpenGL/textureQuality to shrink it)")
                  .arg(mTotalBytes / (1024 * 1024))
                  .arg(static_cast<qulonglong>(mEntries.size()))
                  .arg(kBudgetBytes / (1024 * 1024)));
  }
#  endif
#endif
  mFrameStart = mClock + 1;
  return evicted;
}

bool OmWgpuTextureCache::tryGet(uint64_t textureId, OmWgpuTextureHandle &out) {
  auto it = mEntries.find(textureId);
  if (it == mEntries.end())
    return false;
  it->second.lastUsed = ++mClock;  // LRU: mark hot so the budget eviction keeps it
  const Entry &e = it->second;
  out = {e.texture, e.view, e.width, e.height};
  return true;
}

OmWgpuTextureHandle OmWgpuTextureCache::acquire(uint64_t textureId, uint32_t width, uint32_t height,
                                                const void *rgba8, size_t rgba8Len, uint32_t mipLevels) {
  auto it = mEntries.find(textureId);
  if (it != mEntries.end()) {
    it->second.lastUsed = ++mClock;  // LRU: mark hot
    const Entry &e = it->second;
    return {e.texture, e.view, e.width, e.height, {e.meanLin[0], e.meanLin[1], e.meanLin[2]}};
  }

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mOwner || !mOwner->isAvailable() || !mOwner->device() || !mOwner->queue()) {
    Entry sentinel = {};
    mEntries.emplace(textureId, sentinel);
    return {nullptr, nullptr, 0, 0};
  }
  if (!rgba8 || rgba8Len < static_cast<size_t>(width) * height * 4u || width == 0 || height == 0) {
    OmLog::info(QString("[OmWgpuTextureCache] acquire(%1) bad args; refusing").arg(textureId));
    return {nullptr, nullptr, 0, 0};
  }

  WGPUDevice device = static_cast<WGPUDevice>(mOwner->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mOwner->queue());

  // Full mip chain (CPU box-filtered, built once per unique texture at upload): without it the
  // sampler's trilinear request is moot and minified textures — the aerial city's roads/walls —
  // alias into per-pixel shimmer WREN (mipmapped) doesn't have.
  // mipLevels == 1 (a MUTABLE texture, e.g. the Pen paint layer) skips the chain entirely: the
  // loop below is keyed off mipCount, so one level means one upload and updateRgba8 can rewrite
  // level 0 without leaving stale minified levels behind.
  uint32_t mipCount = 1;
  if (mipLevels != 1) {
    uint32_t m = width > height ? width : height;
    while (m > 1) {
      m >>= 1;
      ++mipCount;
    }
  }
  WGPUTextureDescriptor texDesc = {};
  texDesc.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst;
  texDesc.dimension = WGPUTextureDimension_2D;
  texDesc.size = {width, height, 1};
  texDesc.format = WGPUTextureFormat_RGBA8Unorm;
  texDesc.mipLevelCount = mipCount;
  texDesc.sampleCount = 1;
  WGPUTexture tex = wgpuDeviceCreateTexture(device, &texDesc);
  if (!tex) {
    OmLog::info(QString("[OmWgpuTextureCache] CreateTexture failed for id %1").arg(textureId));
    return {nullptr, nullptr, 0, 0};
  }

  // Upload via wgpuQueueWriteTexture — wgpu handles the staging
  // buffer + the 256-byte row stride internally.
  WGPUTexelCopyTextureInfo dst = {};
  dst.texture = tex;
  dst.mipLevel = 0;
  dst.origin = {0, 0, 0};
  dst.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferLayout layout = {};
  layout.offset = 0;
  layout.bytesPerRow = width * 4u;
  layout.rowsPerImage = height;
  WGPUExtent3D extent = {width, height, 1};
  wgpuQueueWriteTexture(queue, &dst, rgba8, rgba8Len, &layout, &extent);

  // Levels 1..N: successive 2×2 box reductions of the previous level (odd edges clamp).
  {
    std::vector<uint8_t> prev(static_cast<const uint8_t *>(rgba8),
                              static_cast<const uint8_t *>(rgba8) +
                                static_cast<size_t>(width) * height * 4u);
    std::vector<uint8_t> cur;
    uint32_t pw = width, ph = height;
    for (uint32_t level = 1; level < mipCount; ++level) {
      const uint32_t cw = pw > 1 ? pw >> 1 : 1, ch = ph > 1 ? ph >> 1 : 1;
      cur.resize(static_cast<size_t>(cw) * ch * 4u);
      for (uint32_t y = 0; y < ch; ++y) {
        const uint32_t y0 = 2 * y, y1 = (2 * y + 1 < ph) ? 2 * y + 1 : ph - 1;
        for (uint32_t x = 0; x < cw; ++x) {
          const uint32_t x0 = 2 * x, x1 = (2 * x + 1 < pw) ? 2 * x + 1 : pw - 1;
          for (int c = 0; c < 4; ++c) {
            const uint32_t s = prev[(static_cast<size_t>(y0) * pw + x0) * 4 + c] +
                               prev[(static_cast<size_t>(y0) * pw + x1) * 4 + c] +
                               prev[(static_cast<size_t>(y1) * pw + x0) * 4 + c] +
                               prev[(static_cast<size_t>(y1) * pw + x1) * 4 + c];
            cur[(static_cast<size_t>(y) * cw + x) * 4 + c] = static_cast<uint8_t>(s / 4u);
          }
        }
      }
      dst.mipLevel = level;
      layout.bytesPerRow = cw * 4u;
      layout.rowsPerImage = ch;
      const WGPUExtent3D mext = {cw, ch, 1};
      wgpuQueueWriteTexture(queue, &dst, cur.data(), cur.size(), &layout, &mext);
      prev.swap(cur);
      pw = cw;
      ph = ch;
    }
  }

  WGPUTextureView view = wgpuTextureCreateView(tex, nullptr);
  if (!view) {
    OmLog::info(QString("[OmWgpuTextureCache] CreateView failed for id %1").arg(textureId));
    wgpuTextureRelease(tex);
    return {nullptr, nullptr, 0, 0};
  }

  Entry e;
  {
    static float sDecodeLut[256];
    static bool sLutInit = false;
    if (!sLutInit) {
      for (int i = 0; i < 256; ++i)
        sDecodeLut[i] = std::pow(i / 255.0f, 2.2f);
      sLutInit = true;
    }
    const uint8_t *px = static_cast<const uint8_t *>(rgba8);
    const size_t n = static_cast<size_t>(width) * height;
    double acc[3] = {0.0, 0.0, 0.0};
    const size_t step = n > 262144 ? n / 262144 : 1;  // sample big textures, exact for small
    size_t cnt = 0;
    for (size_t i = 0; i < n; i += step, ++cnt) {
      acc[0] += sDecodeLut[px[i * 4 + 0]];
      acc[1] += sDecodeLut[px[i * 4 + 1]];
      acc[2] += sDecodeLut[px[i * 4 + 2]];
    }
    if (cnt) {
      e.meanLin[0] = static_cast<float>(acc[0] / cnt);
      e.meanLin[1] = static_cast<float>(acc[1] / cnt);
      e.meanLin[2] = static_cast<float>(acc[2] / cnt);
    }
  }
  e.texture = tex;
  e.view = view;
  e.width = width;
  e.height = height;
  e.bytes = static_cast<size_t>(width) * height * 4u;
  e.lastUsed = ++mClock;
  e.mipLevels = mipCount;
  mEntries.emplace(textureId, e);
  mTotalBytes += e.bytes;
  // Eviction happens at evictStale(), never here: a view released mid-collect dangles in the draws
  // already collected this frame (see beginFrame()).
  // Regression signal for the 3c-B texture-flood bug: log every texture CREATION (cache miss).
  // A correct scene reaches a small, STABLE count (its unique texture files) and then goes quiet
  // because shared files cache-HIT; a key regression (e.g. reverting to pointer keys) makes the
  // count climb without bound (one upload per OmImageTexture instance) → VRAM OOM. The running
  // `entries`/`totalMB` show the LRU budget holding. Gated by OMNISIM_WGPU_TEXLOG=<file>; inert
  // otherwise. See OmWgpuSceneRenderer::stableTexId.
  if (const char *tp = std::getenv("OMNISIM_WGPU_TEXLOG")) {
    static long sCreated = 0;
    ++sCreated;
    if (FILE *fp = std::fopen(tp, "a")) {
      std::fprintf(fp, "createTex #%ld id=%llu %ux%u entries=%llu totalMB=%.1f\n", sCreated,
                   static_cast<unsigned long long>(textureId), width, height,
                   static_cast<unsigned long long>(mEntries.size()), mTotalBytes / 1048576.0);
      std::fclose(fp);
    }
  }
  return {e.texture, e.view, e.width, e.height, {e.meanLin[0], e.meanLin[1], e.meanLin[2]}};
#  else
  (void)rgba8;
  (void)rgba8Len;
  (void)width;
  (void)height;
  (void)mipLevels;
  return {nullptr, nullptr, 0, 0};
#  endif
#else
  (void)mOwner;
  (void)rgba8;
  (void)rgba8Len;
  (void)width;
  (void)height;
  (void)mipLevels;
  return {nullptr, nullptr, 0, 0};
#endif
}

bool OmWgpuTextureCache::updateRgba8(uint64_t textureId, uint32_t width, uint32_t height,
                                     const void *rgba8, size_t rgba8Len, uint64_t revision) {
  auto it = mEntries.find(textureId);
  if (it == mEntries.end())
    return false;
  Entry &e = it->second;
  if (e.lastRevision == revision)
    return false;  // resident bytes are already this revision -- the idle-Pen fast path
  // A sentinel has no texture to write into, and a size change means this key now names DIFFERENT
  // content: in both cases the caller must go through release() + acquire() rather than get a
  // silent no-op. A mip chain would be left stale by a level-0-only write, so refuse that too.
  if (!e.texture || !rgba8 || e.width != width || e.height != height || e.mipLevels != 1 ||
      rgba8Len < static_cast<size_t>(width) * height * 4u || width == 0 || height == 0)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mOwner || !mOwner->isAvailable() || !mOwner->queue())
    return false;
  WGPUQueue queue = static_cast<WGPUQueue>(mOwner->queue());
  WGPUTexelCopyTextureInfo dst = {};
  dst.texture = static_cast<WGPUTexture>(e.texture);
  dst.mipLevel = 0;
  dst.origin = {0, 0, 0};
  dst.aspect = WGPUTextureAspect_All;
  WGPUTexelCopyBufferLayout layout = {};
  layout.offset = 0;
  layout.bytesPerRow = width * 4u;
  layout.rowsPerImage = height;
  WGPUExtent3D extent = {width, height, 1};
  wgpuQueueWriteTexture(queue, &dst, rgba8, rgba8Len, &layout, &extent);
  e.lastRevision = revision;
  e.lastUsed = ++mClock;  // a written texture is by definition hot
  return true;
#  else
  (void)rgba8Len;
  (void)revision;
  return false;
#  endif
#else
  (void)rgba8Len;
  (void)revision;
  return false;
#endif
}

void OmWgpuTextureCache::release(uint64_t textureId) {
  auto it = mEntries.find(textureId);
  if (it == mEntries.end())
    return;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (it->second.view)
    wgpuTextureViewRelease(static_cast<WGPUTextureView>(it->second.view));
  if (it->second.texture)
    wgpuTextureRelease(static_cast<WGPUTexture>(it->second.texture));
#  endif
#endif
  mTotalBytes -= it->second.bytes;
  mEntries.erase(it);
}
