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

#ifndef OM_WGPU_MESH_CACHE_HPP
#define OM_WGPU_MESH_CACHE_HPP

//
// OmWgpuMeshCache — R3.2 of engine-migration-plan.md §14.3.
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
//    in OmWgpuMeshHandle. A handle whose vertex pointer is null
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
//    (`OmWgpuMeshAdapter.cpp`, R3.2b). That separation matches the
//    plan's R3.4 ("Path-3 shader port") decoupling: byte streams
//    are engine-agnostic; the adapter is the only place that knows
//    WREN's vertex layout.
//

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

class OmVulkanBackend;

struct OmWgpuMeshHandle {
  void *vertexBuffer = nullptr;  // WGPUBuffer (opaque)
  void *indexBuffer = nullptr;   // WGPUBuffer (opaque)
  uint32_t indexCount = 0;
  // OmniLight bake input: CPU copies of the local-space positions (xyz per vertex) and the
  // triangle indices, retained at upload (the cache never evicts, so the pointers are stable
  // for the entry's lifetime). Null on the sentinel/miss paths.
  const std::vector<float> *cpuPositions = nullptr;
  const std::vector<uint32_t> *cpuIndices = nullptr;
  // Local-space bounding sphere (AABB center + half-diagonal), computed once from the vertex
  // positions on the upload-miss path. radius < 0 = unknown -> callers must never cull the draw.
  float localCenter[3] = {0.0f, 0.0f, 0.0f};
  float localRadius = -1.0f;
};

class OmWgpuMeshCache {
public:
  explicit OmWgpuMeshCache(OmVulkanBackend *owner);
  ~OmWgpuMeshCache();

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
  OmWgpuMeshHandle acquire(uint64_t meshId, const void *vertexBytes, size_t vertexBytesLen,
                           const void *indexBytes, size_t indexBytesLen, uint32_t indexCount,
                           uint32_t vertexStride);

  // Read-only lookup: fills `out` and returns true iff `meshId` is
  // already cached. This is acquire()'s hit path with the upload
  // half removed, so a caller can decide whether the bytes are worth
  // PRODUCING before it produces them — W1c needs exactly that, because
  // the two remaining byte sources are expensive in different ways:
  // the codegen primitives rebuild a mesh on the CPU, and the WREN
  // readback needs a current GL context (glGetBufferSubData). A hit
  // here means neither has to happen.
  //
  // A cached MISS (the {null, null, 0} sentinel acquire() stores when the
  // backend is down) also returns true, with null buffers — same value
  // acquire() would have returned, so the "don't retry the upload every
  // frame" contract is unchanged.
  bool tryGet(uint64_t meshId, OmWgpuMeshHandle &out) const;

  // Re-upload the VERTEX stream of an already-cached entry, in place.
  //
  // WHY THIS EXISTS: DEFORMING GEOMETRY. Cloth and SoftBody hand the renderer a
  // brand-new vertex stream every simulation step while their TOPOLOGY (the index
  // buffer) never changes, and the cache's only other write path is
  // acquire()-on-first-use. Without this, a deformable would have to
  // release() + acquire() every frame — two wgpuBufferRelease + two
  // wgpuDeviceCreateBuffer + a full re-derivation of the CPU-side bake copies per
  // node per frame, i.e. allocator churn on the hot path for a mesh whose SIZE
  // never changed. wgpuQueueWriteBuffer into the existing buffer is the operation
  // that was actually wanted.
  //
  // Returns FALSE — and touches nothing — when `meshId` is not cached, when the
  // entry is the {null, null, 0} sentinel, or when `vertexBytesLen` differs from
  // the length the entry was uploaded with. The size check is not defensive
  // padding: it is the contract that makes this safe for a caller keyed on a NODE
  // pointer, where a freed node's address can be reused by a different node with a
  // different vertex count. A false return means "fall back to release+acquire".
  //
  // The index buffer, indexCount and cpuIdx are deliberately NOT touched: this
  // path exists precisely for the case where the topology is fixed. The CPU-side
  // position copy (cpuPos) and the local bounding sphere ARE refreshed, because
  // both are derived from the vertices and both are consumed downstream (the
  // OmniLight bake reads cpuPos; culling reads the sphere) — a stale sphere on a
  // draped sheet would cull it out of frame.
  bool updateVertices(uint64_t meshId, const void *vertexBytes, size_t vertexBytesLen,
                      uint32_t vertexStride);

  // Drop a cached entry's GPU buffers explicitly. Used by tests +
  // by future eviction. No-op if the entry isn't cached.
  void release(uint64_t meshId);

  // ---- PER-CACHE VERTEX EPOCH (P1: deformables on the SENSOR path) ---------
  //
  // THE BUG THIS EXISTS TO MAKE UNREPRESENTABLE. A deforming mesh is re-uploaded
  // once per simulation step, and the "has it moved since my copy?" question was
  // answered by a FUNCTION-LOCAL STATIC in the collector — one variable shared by
  // every caller in the process. That is correct only while there is exactly one
  // caller. The moment a Camera device collects as well, the two disagree: each
  // device owns its OWN OmWgpuMeshCache, so whichever renderer ran first in a step
  // consumed the "the clock advanced" edge and every later one skipped its upload.
  // The first-upload branch still ran, so the second cache got the cloth ONCE and
  // never again — a sheet that animates correctly on screen and is FROZEN at its
  // first pose in the sensor image. It compiles clean and passes any single-frame
  // screenshot test.
  //
  // The fix is to keep the two decisions apart and put each where its state
  // belongs: "should the CPU-side surface be re-read from the solver?" is a
  // process-global, once-per-step decision and stays in the collector; "is MY GPU
  // copy of this mesh stale?" is per (cache, mesh) and lives HERE, next to the
  // buffer it describes. An epoch stored on the entry also dies with the entry, so
  // there is no side table to invalidate and no way for a freed cache to answer for
  // a live one.
  //
  // `epoch` is any monotonic stamp the caller uses to mean "this content version";
  // the deformable path passes OmSimulationState::time(). An entry that does not
  // exist reports FALSE (i.e. "not current"), which is the safe answer: the caller
  // is about to take the upload path anyway. Default -1.0 never equals a real
  // simulation time, so a freshly acquired entry is stale until it is stamped.
  bool vertexEpochIs(uint64_t meshId, double epoch) const;
  void setVertexEpoch(uint64_t meshId, double epoch);

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
    float localCenter[3] = {0.0f, 0.0f, 0.0f};
    float localRadius = -1.0f;
    std::vector<float> cpuPos;      // xyz per vertex (OmniLight bake)
    std::vector<uint32_t> cpuIdx;   // triangle indices, widened to u32
    // Content version of the VERTEX buffer, per cache. See vertexEpochIs() above:
    // -1.0 = never stamped, and never equal to a real simulation time.
    double vertexEpoch = -1.0;
  };

  // Derive the per-entry data that is a pure function of the VERTEX stream: the CPU
  // position copy (OmniLight bake input) and the local bounding sphere (culling).
  // Shared verbatim by acquire()'s upload path and updateVertices()'s re-upload path
  // so the two can never disagree about what a vertex buffer implies.
  static void refreshVertexDerived(Entry &e, const void *vertexBytes, size_t vertexBytesLen,
                                   uint32_t vertexStride);

  OmVulkanBackend *mOwner;  // non-owning; backend lifetime > cache
  std::unordered_map<uint64_t, Entry> mEntries;
  size_t mTotalVertexBytes = 0;
  size_t mTotalIndexBytes = 0;
};

#endif
