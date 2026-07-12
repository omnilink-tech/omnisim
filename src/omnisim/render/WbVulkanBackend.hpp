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

#ifndef WB_VULKAN_BACKEND_HPP
#define WB_VULKAN_BACKEND_HPP

//
// WbVulkanBackend — concrete WbRenderBackend that will eventually
// drive a modern GPU renderer (Vulkan via a thin custom layer, or
// bgfx / sokol-gfx — to be decided at R3). At R1 it is a marker only:
// reports kind/name/available, with isAvailable() hardcoded to false
// so worlds that opt into "vulkan" silently fall back to WREN.
//
// Twin of WbNewtonBackend in the physics arm. Same gradual-widening
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

#include "WbRenderBackend.hpp"

class WbVulkanBackend : public WbRenderBackend {
public:
  WbVulkanBackend();
  ~WbVulkanBackend() override;
  WbRenderBackendKind kind() const override { return WbRenderBackendKind::Vulkan; }
  const char *name() const override { return "vulkan"; }
  bool isAvailable() const override { return mAvailable; }

  // R3.2 seam: opaque WGPUDevice handle for WbWgpuMeshCache + future
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
  // surface (WbWgpuSurface) is created via wgpuInstanceCreateSurface
  // (needs the instance) and configured against the formats/present-
  // modes that wgpuSurfaceGetCapabilities reports for this adapter.
  // Same null-check contract + `void *`-to-keep-the-header-clean
  // rationale as device()/queue() above.
  void *instance() const { return mInstance; }
  void *adapter() const { return mAdapter; }

private:
  bool mAvailable;
  void *mDevice = nullptr;   // WGPUDevice (opaque)
  void *mQueue = nullptr;    // WGPUQueue (opaque) — fetched from device
  void *mInstance = nullptr; // WGPUInstance (opaque) — held for wait/poll
  void *mAdapter = nullptr;  // WGPUAdapter (opaque) — kept for diagnostics
};

#endif
