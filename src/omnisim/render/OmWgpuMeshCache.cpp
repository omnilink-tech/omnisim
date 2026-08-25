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

#include "OmWgpuMeshCache.hpp"

#include <cmath>
#include <cstring>

#include "OmLog.hpp"
#include "OmVulkanBackend.hpp"

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
#    include "webgpu/webgpu.h"
#  endif
#endif

OmWgpuMeshCache::OmWgpuMeshCache(OmVulkanBackend *owner) : mOwner(owner) {
}

OmWgpuMeshCache::~OmWgpuMeshCache() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  // Release every cached buffer pair. Owning backend's queue/device
  // releases independently; the cache must drop its handles before
  // the device goes away.
  for (auto &kv : mEntries) {
    if (kv.second.vertexBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(kv.second.vertexBuffer));
    if (kv.second.indexBuffer)
      wgpuBufferRelease(static_cast<WGPUBuffer>(kv.second.indexBuffer));
  }
#  endif
#endif
  mEntries.clear();
  mTotalVertexBytes = 0;
  mTotalIndexBytes = 0;
}

bool OmWgpuMeshCache::tryGet(uint64_t meshId, OmWgpuMeshHandle &out) const {
  auto it = mEntries.find(meshId);
  if (it == mEntries.end())
    return false;
  const Entry &e = it->second;
  out.vertexBuffer = e.vertexBuffer;
  out.indexBuffer = e.indexBuffer;
  out.indexCount = e.indexCount;
  out.localCenter[0] = e.localCenter[0];
  out.localCenter[1] = e.localCenter[1];
  out.localCenter[2] = e.localCenter[2];
  out.localRadius = e.localRadius;
  out.cpuPositions = e.cpuPos.empty() ? nullptr : &e.cpuPos;
  out.cpuIndices = e.cpuIdx.empty() ? nullptr : &e.cpuIdx;
  return true;
}

OmWgpuMeshHandle OmWgpuMeshCache::acquire(uint64_t meshId, const void *vertexBytes, size_t vertexBytesLen,
                                          const void *indexBytes, size_t indexBytesLen, uint32_t indexCount,
                                          uint32_t vertexStride) {
  // Hit path, single-sourced with tryGet() so the two can never diverge.
  OmWgpuMeshHandle hit;
  if (tryGet(meshId, hit))
    return hit;

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mOwner || !mOwner->isAvailable() || !mOwner->device() || !mOwner->queue()) {
    // Backend not up — cache the miss as a {null, null, 0} sentinel so
    // we don't retry the upload every frame. Caller falls back to WREN.
    Entry sentinel = {};
    mEntries.emplace(meshId, sentinel);
    return {nullptr, nullptr, 0};
  }
  if (!vertexBytes || vertexBytesLen == 0 || !indexBytes || indexBytesLen == 0 || indexCount == 0) {
    OmLog::info(QString("[OmWgpuMeshCache] acquire(%1) called with empty mesh data; refusing to upload")
                  .arg(meshId));
    return {nullptr, nullptr, 0};
  }

  WGPUDevice device = static_cast<WGPUDevice>(mOwner->device());
  WGPUQueue queue = static_cast<WGPUQueue>(mOwner->queue());

  // Vertex buffer: USAGE_VERTEX so the R3.3+ render-pipeline can bind
  // it as a vertex input, USAGE_COPY_DST so writeBuffer fills it.
  // mappedAtCreation=false: writeBuffer is the simpler upload path
  // and wgpu-native handles the staging dance internally.
  WGPUBufferDescriptor vbDesc = {};
  vbDesc.size = vertexBytesLen;
  vbDesc.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
  vbDesc.mappedAtCreation = false;
  WGPUBuffer vb = wgpuDeviceCreateBuffer(device, &vbDesc);
  if (!vb) {
    OmLog::info(QString("[OmWgpuMeshCache] vertex buffer creation failed for mesh %1 (size %2)")
                  .arg(meshId)
                  .arg(static_cast<qulonglong>(vertexBytesLen)));
    return {nullptr, nullptr, 0};
  }
  wgpuQueueWriteBuffer(queue, vb, 0, vertexBytes, vertexBytesLen);

  WGPUBufferDescriptor ibDesc = {};
  ibDesc.size = indexBytesLen;
  ibDesc.usage = WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst;
  ibDesc.mappedAtCreation = false;
  WGPUBuffer ib = wgpuDeviceCreateBuffer(device, &ibDesc);
  if (!ib) {
    OmLog::info(QString("[OmWgpuMeshCache] index buffer creation failed for mesh %1 (size %2)")
                  .arg(meshId)
                  .arg(static_cast<qulonglong>(indexBytesLen)));
    wgpuBufferRelease(vb);
    return {nullptr, nullptr, 0};
  }
  wgpuQueueWriteBuffer(queue, ib, 0, indexBytes, indexBytesLen);

  Entry e;
  e.vertexBuffer = vb;
  e.indexBuffer = ib;
  e.indexCount = indexCount;
  e.vertexBytes = vertexBytesLen;
  e.indexBytes = indexBytesLen;
  // Local bounding sphere from the vertex positions (pos3 at offset 0 of each stride record):
  // AABB center + half-diagonal. Once per upload; drives shadow/frustum culling downstream.
  // The same loop retains the positions on the CPU for the OmniLight bake (the cache never
  // evicts, so entry vectors are pointer-stable). Shared with updateVertices() so a re-uploaded
  // deformable derives both exactly the way a first upload does.
  refreshVertexDerived(e, vertexBytes, vertexBytesLen, vertexStride);
  // Indices, widened to u32 (the upload accepts u16 or u32 streams: infer from the byte count).
  e.cpuIdx.reserve(indexCount);
  if (indexBytesLen >= static_cast<size_t>(indexCount) * 4) {
    const uint32_t *isrc = static_cast<const uint32_t *>(indexBytes);
    e.cpuIdx.assign(isrc, isrc + indexCount);
  } else if (indexBytesLen >= static_cast<size_t>(indexCount) * 2) {
    const uint16_t *isrc = static_cast<const uint16_t *>(indexBytes);
    for (uint32_t ii = 0; ii < indexCount; ++ii)
      e.cpuIdx.push_back(isrc[ii]);
  }
  auto ins = mEntries.emplace(meshId, std::move(e));
  const Entry &se = ins.first->second;
  mTotalVertexBytes += vertexBytesLen;
  mTotalIndexBytes += indexBytesLen;
  OmWgpuMeshHandle h;
  h.vertexBuffer = se.vertexBuffer;
  h.indexBuffer = se.indexBuffer;
  h.indexCount = se.indexCount;
  h.localCenter[0] = se.localCenter[0];
  h.localCenter[1] = se.localCenter[1];
  h.localCenter[2] = se.localCenter[2];
  h.localRadius = se.localRadius;
  h.cpuPositions = se.cpuPos.empty() ? nullptr : &se.cpuPos;
  h.cpuIndices = se.cpuIdx.empty() ? nullptr : &se.cpuIdx;
  return h;
#  else
  (void)vertexBytes;
  (void)vertexBytesLen;
  (void)indexBytes;
  (void)indexBytesLen;
  (void)indexCount;
  return {nullptr, nullptr, 0};
#  endif
#else
  (void)mOwner;
  (void)vertexBytes;
  (void)vertexBytesLen;
  (void)indexBytes;
  (void)indexBytesLen;
  (void)indexCount;
  return {nullptr, nullptr, 0};
#endif
}

void OmWgpuMeshCache::refreshVertexDerived(Entry &e, const void *vertexBytes, size_t vertexBytesLen,
                                           uint32_t vertexStride) {
  e.cpuPos.clear();
  e.localCenter[0] = e.localCenter[1] = e.localCenter[2] = 0.0f;
  e.localRadius = -1.0f;
  if (vertexStride < 12 || !vertexBytes || vertexBytesLen < vertexStride)
    return;
  const uint8_t *vsrc = static_cast<const uint8_t *>(vertexBytes);
  const size_t n = vertexBytesLen / vertexStride;
  e.cpuPos.reserve(n * 3);
  float mn[3] = {3.4e38f, 3.4e38f, 3.4e38f};
  float mx[3] = {-3.4e38f, -3.4e38f, -3.4e38f};
  for (size_t vi = 0; vi < n; ++vi) {
    float pos[3];
    std::memcpy(pos, vsrc + vi * vertexStride, 12);
    e.cpuPos.push_back(pos[0]);
    e.cpuPos.push_back(pos[1]);
    e.cpuPos.push_back(pos[2]);
    for (int k = 0; k < 3; ++k) {
      if (pos[k] < mn[k])
        mn[k] = pos[k];
      if (pos[k] > mx[k])
        mx[k] = pos[k];
    }
  }
  if (mn[0] <= mx[0]) {
    float halfDiag = 0.0f;
    for (int k = 0; k < 3; ++k) {
      e.localCenter[k] = 0.5f * (mn[k] + mx[k]);
      const float hk = 0.5f * (mx[k] - mn[k]);
      halfDiag += hk * hk;
    }
    e.localRadius = std::sqrt(halfDiag);
  }
}

bool OmWgpuMeshCache::updateVertices(uint64_t meshId, const void *vertexBytes, size_t vertexBytesLen,
                                     uint32_t vertexStride) {
  auto it = mEntries.find(meshId);
  if (it == mEntries.end())
    return false;
  Entry &e = it->second;
  // A sentinel (backend was down at upload time) has no buffer to write into, and a
  // length change means this key now names DIFFERENT geometry -- in both cases the
  // caller must go through release() + acquire() rather than get a silent no-op.
  if (!e.vertexBuffer || !vertexBytes || vertexBytesLen == 0 || vertexBytesLen != e.vertexBytes)
    return false;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (!mOwner || !mOwner->isAvailable() || !mOwner->queue())
    return false;
  WGPUQueue queue = static_cast<WGPUQueue>(mOwner->queue());
  wgpuQueueWriteBuffer(queue, static_cast<WGPUBuffer>(e.vertexBuffer), 0, vertexBytes, vertexBytesLen);
  // cpuPos is re-filled IN PLACE: the handle hands out `&e.cpuPos`, and the Entry lives in
  // an unordered_map, so the vector OBJECT's address is stable no matter what its storage
  // does. Downstream readers (the OmniLight bake) therefore see this frame's geometry.
  refreshVertexDerived(e, vertexBytes, vertexBytesLen, vertexStride);
  return true;
#  else
  (void)vertexStride;
  return false;
#  endif
#else
  (void)vertexStride;
  return false;
#endif
}

bool OmWgpuMeshCache::vertexEpochIs(uint64_t meshId, double epoch) const {
  auto it = mEntries.find(meshId);
  // "Not cached" is deliberately NOT current: the caller is on its way to acquire().
  return it != mEntries.end() && it->second.vertexEpoch == epoch;
}

void OmWgpuMeshCache::setVertexEpoch(uint64_t meshId, double epoch) {
  auto it = mEntries.find(meshId);
  if (it != mEntries.end())
    it->second.vertexEpoch = epoch;
}

void OmWgpuMeshCache::release(uint64_t meshId) {
  auto it = mEntries.find(meshId);
  if (it == mEntries.end())
    return;
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  if (it->second.vertexBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(it->second.vertexBuffer));
  if (it->second.indexBuffer)
    wgpuBufferRelease(static_cast<WGPUBuffer>(it->second.indexBuffer));
#  endif
#endif
  mTotalVertexBytes -= it->second.vertexBytes;
  mTotalIndexBytes -= it->second.indexBytes;
  mEntries.erase(it);
}
