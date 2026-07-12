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

#ifndef WB_WGPU_TEXTURE_CACHE_HPP
#define WB_WGPU_TEXTURE_CACHE_HPP

//
// WbWgpuTextureCache — R3.5 of engine-migration-plan.md §14.3.
//
// Texture analog of WbWgpuMeshCache. Owns GPU-resident
// `WGPUTexture` + `WGPUTextureView` pairs keyed by a stable texture
// ID (typically the WREN-side texture pointer reinterpreted as
// uintptr_t). Each entry is uploaded once per process lifetime.
//
// At R3.5 the cache only supports the most common production
// format: RGBA8Unorm, sample-count 1, no mips. The WREN-image
// adapter (the layer that translates `WrTexture` -> RGBA8 bytes) is
// a separable concern; R3.5 step 1 exposes the upload API +
// runtime-verifies via the probe; the adapter lands as R3.5-step-2
// alongside R3.2b's WrStaticMesh adapter.
//

#include <cstddef>
#include <cstdint>
#include <unordered_map>

class WbVulkanBackend;

struct WbWgpuTextureHandle {
  void *texture = nullptr;      // WGPUTexture
  void *view = nullptr;         // WGPUTextureView
  uint32_t width = 0;
  uint32_t height = 0;
};

class WbWgpuTextureCache {
public:
  explicit WbWgpuTextureCache(WbVulkanBackend *owner);
  ~WbWgpuTextureCache();

  // Upload-on-first-use. `rgba8` is width*height*4 bytes of
  // unpremultiplied RGBA8Unorm pixel data (rows tightly packed —
  // wgpuQueueWriteTexture handles the 256-byte row alignment
  // internally for us).
  WbWgpuTextureHandle acquire(uint64_t textureId, uint32_t width, uint32_t height,
                              const void *rgba8, size_t rgba8Len);

  // R4 material fidelity: cache-hit fast-path. Returns true + fills `out` if
  // `textureId` is already uploaded, WITHOUT the pixel bytes — so WbWgpuImageAdapter
  // can skip the expensive convertToFormat on repeat frames. Non-const: it stamps the
  // entry's LRU clock so a hot texture stays resident under the eviction budget.
  bool tryGet(uint64_t textureId, WbWgpuTextureHandle &out);

  void release(uint64_t textureId);

  size_t entryCount() const { return mEntries.size(); }
  size_t totalBytes() const { return mTotalBytes; }

private:
  struct Entry {
    void *texture = nullptr;
    void *view = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    size_t bytes = 0;
    uint64_t lastUsed = 0;  // LRU clock stamp (updated on every hit/insert)
  };

  // R4 3c-B: evict the least-recently-used entries once total VRAM exceeds this budget. Without a cap the
  // cache grew unbounded when a per-draw texture id churns each frame (the main-view wgpu path's VRAM-OOM
  // crash); the budget bounds it while keeping the hot/stable set resident.
  static constexpr size_t kBudgetBytes = 512u * 1024u * 1024u;  // 512 MB
  void evictToBudget();

  WbVulkanBackend *mOwner;
  std::unordered_map<uint64_t, Entry> mEntries;
  size_t mTotalBytes = 0;
  uint64_t mClock = 0;
};

#endif
