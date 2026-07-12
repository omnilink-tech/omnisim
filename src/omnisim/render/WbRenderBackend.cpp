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

#include "WbRenderBackend.hpp"

#include "WbVulkanBackend.hpp"
#include "WbWrenBackend.hpp"

#include <QtCore/QString>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>

WbRenderBackendKind WbRenderBackendKindFromString(const char *name) {
  if (name == nullptr || *name == '\0')
    return WbRenderBackendKind::Wren;  // default
  if (std::strcmp(name, "wren") == 0)
    return WbRenderBackendKind::Wren;
  // The Vulkan enum value covers both "vulkan" (historical name kept
  // for the dispatcher/class names per Phase ζ deferred rename) and
  // "wgpu" (the name world authors expect post-2026-05-28). The
  // backend is wgpu-native either way; only the .wrl string differs.
  if (std::strcmp(name, "vulkan") == 0 || std::strcmp(name, "wgpu") == 0)
    return WbRenderBackendKind::Vulkan;
  return WbRenderBackendKind::Unknown;
}

const char *WbRenderBackendKindToString(WbRenderBackendKind kind) {
  switch (kind) {
    case WbRenderBackendKind::Wren:
      return "wren";
    case WbRenderBackendKind::Vulkan:
      return "vulkan";
    case WbRenderBackendKind::Unknown:
    default:
      return "unknown";
  }
}

namespace {
  // Two separate once-flags so we can lazy-init Vulkan: pure-WREN
  // worlds shouldn't pay any Vulkan-runtime startup cost just because
  // the dispatcher exists. The Vulkan ctor (R3+) will load the Vulkan
  // loader / pick a physical device / create an instance only when
  // somebody actually asks for the Vulkan backend.
  std::once_flag gWrenFlag;
  std::once_flag gVulkanFlag;
  std::unique_ptr<WbRenderBackend> gWren;
  std::unique_ptr<WbRenderBackend> gVulkan;

  void doInitWren() {
    gWren.reset(new WbWrenBackend());
  }

  void doInitVulkan() {
    gVulkan.reset(new WbVulkanBackend());
  }
}  // namespace

namespace WbRenderBackendRegistry {

  void initialise() {
    // Eager-init WREN only. Vulkan stays cold until somebody asks for it.
    std::call_once(gWrenFlag, doInitWren);
  }

  WbRenderBackend *wrenBackend() {
    std::call_once(gWrenFlag, doInitWren);
    // R3.1 smoke knob: setting OMNISIM_PROBE_WGPU=1 forces the Vulkan
    // backend to initialise the first time anything resolves WREN
    // (i.e. as early as any Viewpoint/Camera goes through render-backend
    // dispatch). Prints `[WbWgpuBackend] wgpu-native init OK` into the
    // log on success, or a fall-back line on failure. Useful for
    // verifying the wgpu-native dep is wired correctly without needing
    // to author a Viewpoint/Camera with `renderBackend "vulkan"` first.
    // Default off so production startup keeps the ~100ms wgpu loader
    // cost off the critical path.
    if (qEnvironmentVariableIntValue("OMNISIM_PROBE_WGPU") != 0)
      std::call_once(gVulkanFlag, doInitVulkan);
    return gWren.get();
  }

  WbRenderBackend *vulkanBackend() {
    std::call_once(gVulkanFlag, doInitVulkan);
    return gVulkan.get();
  }

  // Returns the requested backend if it's available, else falls back
  // to WREN. This is the load-bearing safety net documented in
  // engine-migration-plan.md: a world that asks for Vulkan on a box
  // without a Vulkan-capable GPU silently gets WREN and renders
  // identically. The single warning at world load is emitted by the
  // caller (Camera/Viewpoint renderBackend resolution path), not
  // here, so resolve() stays cheap and silent.
  WbRenderBackend *resolve(WbRenderBackendKind kind) {
    // OMNISIM_FORCE_WREN (or the OMNISIM_LEGACY umbrella) short-circuits every
    // resolution to WREN -- the render-side mirror of physics' OMNISIM_FORCE_ODE
    // (default-flip-plan.md §3.1). It lets a wgpu-ON build reproduce the canonical
    // WREN path for golden-image regression, and gives users a one-switch return to
    // the legacy renderer. Read once via a function-local static (env is immutable
    // mid-process), so resolve() stays cheap and silent. Inert when neither is set.
    static const bool kForceWren = []() {
      const char *const legacy = std::getenv("OMNISIM_LEGACY");
      if (legacy != nullptr && legacy[0] != '\0')
        return true;
      const char *const v = std::getenv("OMNISIM_FORCE_WREN");
      return v != nullptr && v[0] != '\0';
    }();
    if (kForceWren)
      return wrenBackend();
    switch (kind) {
      case WbRenderBackendKind::Vulkan: {
        // Lazy Vulkan init: paying the loader-init cost only here is
        // the difference between "pure-WREN worlds load instantly"
        // and "every omnisim-bin startup spends ~100ms warming up
        // the Vulkan loader".
        WbRenderBackend *vulkan = vulkanBackend();
        if (vulkan != nullptr && vulkan->isAvailable())
          return vulkan;
        return wrenBackend();
      }
      case WbRenderBackendKind::Wren:
      case WbRenderBackendKind::Unknown:
      default:
        return wrenBackend();
    }
  }

}  // namespace WbRenderBackendRegistry
