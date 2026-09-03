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

#ifndef OM_WGPU_TEXTURE_CACHE_HPP
#define OM_WGPU_TEXTURE_CACHE_HPP

//
// OmWgpuTextureCache — R3.5 of engine-migration-plan.md §14.3.
//
// Texture analog of OmWgpuMeshCache. Owns GPU-resident
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

class OmVulkanBackend;

struct OmWgpuTextureHandle {
  void *texture = nullptr;      // WGPUTexture
  void *view = nullptr;         // WGPUTextureView
  uint32_t width = 0;
  uint32_t height = 0;
  // LINEAR-light mean colour of the texture (per-channel mean of (v/255)^2.2), computed once at
  // upload. OmniLight's bounce albedo for textured draws — diffuse GI needs the average, not the
  // detail. {1,1,1} on sentinel/failed entries.
  float meanLin[3] = {1.0f, 1.0f, 1.0f};
};

class OmWgpuTextureCache {
public:
  // Rebuild point (2026-09-02). Call right BEFORE re-collecting a draw list against this cache
  // (never per frame, never mid-collect): releases least-recently-used entries down to
  // kBudgetBytes, but only entries NOTHING touched since the previous call -- the previous
  // rebuild's working set stays, so a scene whose textures exceed the budget runs above budget
  // (warned once) instead of re-uploading it every rebuild. Returns true when anything was
  // released; the caller must then drop every bind group keyed by a texture view
  // (OmWgpuRenderTarget::forgetTextureBindGroups), because wgpu recycles handle values and a
  // cached bind group built on the dead view is "invalid" at submit. acquire() no longer evicts:
  // it used to release views mid-collect, and a draw collected earlier in the same frame kept
  // referencing the dead view, so wgpuDeviceCreateBindGroup panicked ("TextureView does not
  // exist") on any scene whose textures exceeded the budget (environments/city.omniworld).
  bool evictStale();
  explicit OmWgpuTextureCache(OmVulkanBackend *owner);
  ~OmWgpuTextureCache();

  // Upload-on-first-use. `rgba8` is width*height*4 bytes of
  // unpremultiplied RGBA8Unorm pixel data (rows tightly packed —
  // wgpuQueueWriteTexture handles the 256-byte row alignment
  // internally for us).
  // `mipLevels`: 0 (the default, and every pre-P3 caller) builds the full CPU box-filtered mip
  // chain. 1 uploads level 0 only -- what a MUTABLE texture wants, since a re-upload would
  // otherwise have to regenerate the whole chain every frame, and it is also what WREN does for
  // the Pen layer (wr_material_set_texture_enable_mip_maps(false)).
  OmWgpuTextureHandle acquire(uint64_t textureId, uint32_t width, uint32_t height,
                              const void *rgba8, size_t rgba8Len, uint32_t mipLevels = 0);

  // W3/P3 mutable-texture path, the texture-side mirror of OmWgpuMeshCache::updateVertices():
  // ONE wgpuQueueWriteTexture into an already-uploaded entry, no release+acquire churn.
  //
  // Refuses (returns false, writing nothing) rather than silently doing the wrong thing when:
  //   * the id is not in the cache, or its entry is a sentinel (backend down at upload time);
  //   * the dimensions or byte count differ from the entry -- this key now names a DIFFERENT
  //     texture, so the caller must go through release() + acquire();
  //   * the entry carries a mip chain -- writing level 0 alone would leave stale minified levels;
  //   * `revision` equals the revision already uploaded into this entry -- the no-op fast path,
  //     which is what makes an idle Pen cost zero GPU traffic.
  // A refusal for the last reason is not an error; callers treat false as "nothing to do".
  bool updateRgba8(uint64_t textureId, uint32_t width, uint32_t height, const void *rgba8,
                   size_t rgba8Len, uint64_t revision);

  // R4 material fidelity: cache-hit fast-path. Returns true + fills `out` if
  // `textureId` is already uploaded, WITHOUT the pixel bytes — so OmWgpuImageAdapter
  // can skip the expensive convertToFormat on repeat frames. Non-const: it stamps the
  // entry's LRU clock so a hot texture stays resident under the eviction budget.
  bool tryGet(uint64_t textureId, OmWgpuTextureHandle &out);

  void release(uint64_t textureId);

  size_t entryCount() const { return mEntries.size(); }
  size_t totalBytes() const { return mTotalBytes; }

private:
  struct Entry {
    float meanLin[3] = {1.0f, 1.0f, 1.0f};
    void *texture = nullptr;
    void *view = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    size_t bytes = 0;
    uint64_t lastUsed = 0;      // LRU clock stamp (updated on every hit/insert)
    uint32_t mipLevels = 1;     // levels actually created (updateRgba8 refuses > 1)
    uint64_t lastRevision = 0;  // caller-supplied revision of the bytes currently resident
  };

  // R4 3c-B: evict the least-recently-used entries once total VRAM exceeds this budget. Without a cap the
  // cache grew unbounded when a per-draw texture id churns each frame (the main-view wgpu path's VRAM-OOM
  // crash); the budget bounds it while keeping the hot/stable set resident.
  static constexpr size_t kBudgetBytes = 512u * 1024u * 1024u;  // 512 MB
  void evictToBudget();

  OmVulkanBackend *mOwner;
  std::unordered_map<uint64_t, Entry> mEntries;
  size_t mTotalBytes = 0;
  uint64_t mClock = 0;
  uint64_t mFrameStart = 0;  // mClock value at the last beginFrame(); entries used since are the working set
  bool mBudgetWarned = false;
};

#endif
