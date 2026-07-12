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

#include "WbWgpuMeshCache.hpp"

#include "WbLog.hpp"
#include "WbVulkanBackend.hpp"

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
#    include "webgpu/webgpu.h"
#  endif
#endif

WbWgpuMeshCache::WbWgpuMeshCache(WbVulkanBackend *owner) : mOwner(owner) {
}

WbWgpuMeshCache::~WbWgpuMeshCache() {
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

WbWgpuMeshHandle WbWgpuMeshCache::acquire(uint64_t meshId, const void *vertexBytes, size_t vertexBytesLen,
                                          const void *indexBytes, size_t indexBytesLen, uint32_t indexCount,
                                          uint32_t /*vertexStride*/) {
  auto it = mEntries.find(meshId);
  if (it != mEntries.end()) {
    const Entry &e = it->second;
    return {e.vertexBuffer, e.indexBuffer, e.indexCount};
  }

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
    WbLog::info(QString("[WbWgpuMeshCache] acquire(%1) called with empty mesh data; refusing to upload")
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
    WbLog::info(QString("[WbWgpuMeshCache] vertex buffer creation failed for mesh %1 (size %2)")
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
    WbLog::info(QString("[WbWgpuMeshCache] index buffer creation failed for mesh %1 (size %2)")
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
  mEntries.emplace(meshId, e);
  mTotalVertexBytes += vertexBytesLen;
  mTotalIndexBytes += indexBytesLen;
  return {e.vertexBuffer, e.indexBuffer, e.indexCount};
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

void WbWgpuMeshCache::release(uint64_t meshId) {
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
