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

#ifndef OM_VULKAN_BACKEND_HPP
#define OM_VULKAN_BACKEND_HPP

//
// OmVulkanBackend — concrete OmRenderBackend that will eventually
// drive a modern GPU renderer (Vulkan via a thin custom layer, or
// bgfx / sokol-gfx — to be decided at R3). At R1 it is a marker only:
// reports kind/name/available, with isAvailable() hardcoded to false
// so worlds that opt into "vulkan" silently fall back to WREN.
//
// Twin of OmNewtonBackend in the physics arm. Same gradual-widening
// discipline: R1 lands the symbol surface and build flag; R3 lands
// the first real call (single-Camera render-to-texture); subsequent
// phases widen.
//
// The .cpp implementation is gated on OMNISIM_WITH_VULKAN: the OFF
// build is byte-equivalent to a WREN-only binary. ON build at R1
// still reports isAvailable()=false because there's no Vulkan SDK
// dep yet; R3 introduces the SDK probe and flips this when a
// Vulkan-capable device is found.
//

#include "OmRenderBackend.hpp"

class OmVulkanBackend : public OmRenderBackend {
public:
  OmVulkanBackend();
  ~OmVulkanBackend() override;
  OmRenderBackendKind kind() const override { return OmRenderBackendKind::Vulkan; }
  const char *name() const override { return "vulkan"; }
  bool isAvailable() const override { return mAvailable; }

  // R3.2 seam: opaque WGPUDevice handle for OmWgpuMeshCache + future
  // R3.3 RTT machinery. Returns nullptr when isAvailable()==false (no
  // wgpu-native dep at build time, init failed at runtime, or
  // OMNISIM_WITH_VULKAN=OFF). Callers must null-check.
  //
  // Kept as `void *` so this header doesn't pull <webgpu/webgpu.h>
  // into every TU that just wants to know whether wgpu is up. The
  // .cpp reinterpret_casts back to WGPUDevice at the one site that
  // talks to wgpu API directly.
  void *device() const { return mDevice; }
  void *queue() const { return mQueue; }

  // R4 seam: opaque WGPUInstance + WGPUAdapter handles. The on-screen
  // surface (OmWgpuSurface) is created via wgpuInstanceCreateSurface
  // (needs the instance) and configured against the formats/present-
  // modes that wgpuSurfaceGetCapabilities reports for this adapter.
  // Same null-check contract + `void *`-to-keep-the-header-clean
  // rationale as device()/queue() above.
  void *instance() const { return mInstance; }
  void *adapter() const { return mAdapter; }

  // Lane E4 (WREN-deletion runbook, GPU-memory readout port): the adapter's identity as
  // wgpuAdapterGetInfo reported it at init ("<device> via <backend>"), empty when
  // isAvailable() is false. Plain C string so this header stays Qt-free (R0 contract).
  const char *adapterSummary() const { return mAdapterSummary; }

  // The wgpu-side answer to wr_gl_state_get_gpu_memory(), and the answer is HONEST:
  // the wgpu C API exposes NO GPU-memory figure at all -- WGPUAdapterInfo is identity
  // strings/IDs, WGPULimits are API limits (maxBufferSize is not VRAM), and wgpu-native's
  // wgpu.h extension adds only object-count registry reports. Verified against the
  // vendored wgpu-native headers (webgpu.h + wgpu.h). So this returns -1, ALWAYS, and the
  // UI must say "unavailable" rather than render it as 0 MB. If a future wgpu-native adds
  // a memory query, this is the one place to plumb it.
  long long gpuMemoryBytes() const { return -1; }

private:
  bool mAvailable;
  void *mDevice = nullptr;   // WGPUDevice (opaque)
  void *mQueue = nullptr;    // WGPUQueue (opaque) — fetched from device
  void *mInstance = nullptr; // WGPUInstance (opaque) — held for wait/poll
  void *mAdapter = nullptr;  // WGPUAdapter (opaque) — kept for diagnostics
  char mAdapterSummary[192] = {0};  // "<device> via <backend>", filled at init
};

#endif
