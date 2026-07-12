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

#ifndef WB_WGPU_MESH_CACHE_HPP
#define WB_WGPU_MESH_CACHE_HPP

//
// WbWgpuMeshCache — R3.2 of engine-migration-plan.md §14.3.
//
// Owns the GPU-resident vertex + index buffers for the wgpu render
// path, keyed by a stable mesh ID assigned by the caller (typically
// the WREN-side mesh pointer reinterpreted as uintptr_t). Each entry
// is uploaded once per process lifetime: subsequent acquire() calls
// return the cached handle without re-uploading.
//
// Design notes:
//
//  - The cache holds opaque (void *) WGPUBuffer handles so this
//    header doesn't need to drag in <webgpu/webgpu.h>. The cpp side
//    is the only translation unit that talks to wgpu directly. This
//    keeps OMNISIM_WITH_VULKAN=OFF builds zero-cost — no wgpu
//    symbols leak into the public interface.
//
//  - acquire() returns a {vertex, index, indexCount} triple wrapped
//    in WbWgpuMeshHandle. A handle whose vertex pointer is null
//    means upload failed (out-of-memory, device-lost, build with
//    WB_WGPU_NATIVE_AVAILABLE=OFF, etc.) — callers must check and
//    fall back to WREN-side rasterization.
//
//  - Eviction is deferred to Phase η (post-R6). At R3.2 the cache
//    grows without bound; warehouse_industrial measured at ~80 MiB
//    of mesh data which fits comfortably in any wgpu-capable GPU.
//
//  - The WREN-adapter layer (WrStaticMesh -> byte-stream) is a
//    separable concern that lives in a future file
//    (`WbWgpuMeshAdapter.cpp`, R3.2b). That separation matches the
//    plan's R3.4 ("Path-3 shader port") decoupling: byte streams
//    are engine-agnostic; the adapter is the only place that knows
//    WREN's vertex layout.
//

#include <cstddef>
#include <cstdint>
#include <unordered_map>

class WbVulkanBackend;

struct WbWgpuMeshHandle {
  void *vertexBuffer = nullptr;  // WGPUBuffer (opaque)
  void *indexBuffer = nullptr;   // WGPUBuffer (opaque)
  uint32_t indexCount = 0;
};

class WbWgpuMeshCache {
public:
  explicit WbWgpuMeshCache(WbVulkanBackend *owner);
  ~WbWgpuMeshCache();

  // Upload-on-first-use. `meshId` must be stable across calls for
  // the same logical mesh (e.g. the WREN mesh pointer's bit
  // pattern). `vertexBytes` / `indexBytes` may be null on second+
  // calls — they are read only on the upload miss path.
  //
  // `vertexStride` is the byte-stride of one vertex record. R3.2
  // assumes the WREN layout (pos3 + norm3 + uv2 = 32 bytes); R3.5
  // textures and R3.4 shader port may need a wider stride and a
  // separate format-tag in the key.
  //
  // Returns a handle with null vertexBuffer if the upload failed or
  // the wgpu backend is unavailable. Callers must check before use.
  WbWgpuMeshHandle acquire(uint64_t meshId, const void *vertexBytes, size_t vertexBytesLen,
                           const void *indexBytes, size_t indexBytesLen, uint32_t indexCount,
                           uint32_t vertexStride);

  // Drop a cached entry's GPU buffers explicitly. Used by tests +
  // by future eviction. No-op if the entry isn't cached.
  void release(uint64_t meshId);

  // Diagnostic accessors.
  size_t entryCount() const { return mEntries.size(); }
  size_t totalVertexBytes() const { return mTotalVertexBytes; }
  size_t totalIndexBytes() const { return mTotalIndexBytes; }

private:
  struct Entry {
    void *vertexBuffer = nullptr;
    void *indexBuffer = nullptr;
    uint32_t indexCount = 0;
    size_t vertexBytes = 0;
    size_t indexBytes = 0;
  };

  WbVulkanBackend *mOwner;  // non-owning; backend lifetime > cache
  std::unordered_map<uint64_t, Entry> mEntries;
  size_t mTotalVertexBytes = 0;
  size_t mTotalIndexBytes = 0;
};

#endif
