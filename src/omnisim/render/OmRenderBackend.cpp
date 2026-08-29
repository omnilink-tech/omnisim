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

#include "OmRenderBackend.hpp"

#include "OmLog.hpp"
#include "OmVulkanBackend.hpp"

#include <QtCore/QString>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>

OmRenderBackendKind OmRenderBackendKindFromString(const char *name) {
  if (name == nullptr || *name == '\0')
    return OmRenderBackendKind::Vulkan;  // default: wgpu (F1, wren-deletion-runbook.md -- an empty
                                         // value means "the default", and the default renderer is
                                         // wgpu; it must NOT take the Wren arm or it would fire the
                                         // retired-"wren" warning for a world that never wrote "wren")
  if (std::strcmp(name, "wren") == 0)
    return OmRenderBackendKind::Wren;
  // The Vulkan enum value covers both "vulkan" (historical name kept
  // for the dispatcher/class names per Phase ζ deferred rename) and
  // "wgpu" (the name world authors expect post-2026-05-28). The
  // backend is wgpu-native either way; only the .wrl string differs.
  if (std::strcmp(name, "vulkan") == 0 || std::strcmp(name, "wgpu") == 0)
    return OmRenderBackendKind::Vulkan;
  return OmRenderBackendKind::Unknown;
}

const char *OmRenderBackendKindToString(OmRenderBackendKind kind) {
  switch (kind) {
    case OmRenderBackendKind::Wren:
      return "wren";
    case OmRenderBackendKind::Vulkan:
      return "vulkan";
    case OmRenderBackendKind::Unknown:
    default:
      return "unknown";
  }
}

namespace {
  // Lazy-init: the wgpu ctor loads the loader / picks a device / creates an instance only
  // when somebody actually asks for the backend.
  std::once_flag gVulkanFlag;
  std::unique_ptr<OmRenderBackend> gVulkan;

  void doInitVulkan() {
    gVulkan.reset(new OmVulkanBackend());
  }

  // F1 (wren-deletion-runbook.md, Phase F): OMNISIM_FORCE_WREN and OMNISIM_LEGACY's render
  // arm used to short-circuit EVERY render-backend resolution to WREN. WREN is retired: the
  // only selectable renderer is wgpu, and the WREN code that still ships is the
  // wgpu-native-unavailable last resort (deleted at D1.4), not something a selector may
  // reach. So both variables are IGNORED, but not silently -- an A/B harness that exports
  // OMNISIM_FORCE_WREN and believes it captured a WREN arm would publish a wgpu render as a
  // WREN reference, and a wrong oracle is worse than a lost one. Each set variable produces
  // one warning naming itself, mirroring warnRetiredOdeSelectors() in OmPhysicsBackend.cpp
  // (the physics twins of the same pattern). Read once via a function-local static (env is
  // immutable mid-process), so resolve() stays cheap after the first call.
  void warnRetiredWrenSelectors() {
    static const bool kWarned = []() {
      struct RetiredVar {
        const char *name;
        const char *usedTo;
      };
      static const RetiredVar kRetired[] = {
        {"OMNISIM_FORCE_WREN", "pin every render-backend resolution to WREN for the whole session"},
        {"OMNISIM_LEGACY", "pin rendering onto WREN (this is its RENDER arm; its physics arm is warned about "
                           "separately by OmPhysicsBackend)"}};
      for (const RetiredVar &r : kRetired) {
        const char *const v = std::getenv(r.name);
        if (v != nullptr && v[0] != '\0')
          OmLog::warning(QString("[render] %1 is set but RETIRED and IGNORED: it used to %2. WREN is retired and "
                                 "wgpu is the only selectable renderer, so there is nothing to switch to. Unset the "
                                 "variable -- this run renders wgpu either way (WREN survives only as the last-resort "
                                 "fallback on hosts with no wgpu-native).")
                           .arg(QString::fromLatin1(r.name))
                           .arg(QString::fromLatin1(r.usedTo)));
      }
      return true;
    }();
    (void)kWarned;
  }
}  // namespace

namespace OmRenderBackendRegistry {

  void initialise() {
    // Nothing to eager-init: wgpu initialises on the first resolve that needs it. Kept as a
    // call-site-stable no-op (main.cpp, OmWgpuView).
  }

  OmRenderBackend *vulkanBackend() {
    std::call_once(gVulkanFlag, doInitVulkan);
    return gVulkan.get();
  }

  // F1 (wren-deletion-runbook.md, Phase F): WREN is UNSELECTABLE. Every kind -- Vulkan,
  // Wren, Unknown -- resolves to the wgpu backend when wgpu-native is available. An explicit
  // "wren" request warns once and renders wgpu anyway; the retired OMNISIM_FORCE_WREN /
  // OMNISIM_LEGACY selectors warn once each and are ignored (warnRetiredWrenSelectors).
  //
  // D1.4: the WREN last-resort fallback is DELETED with WREN. A host whose wgpu-native
  // cannot initialise gets the (still non-null) wgpu backend object with isAvailable()
  // false -- every render entry point already branches on that and degrades to "no frame"
  // with its own loud log line. R8 (owner-assumed, 2026-08-22) says every supported
  // platform ships wgpu-native.
  OmRenderBackend *resolve(OmRenderBackendKind kind) {
    warnRetiredWrenSelectors();
    if (kind == OmRenderBackendKind::Wren) {
      // An explicit "wren" (authored field value, or a caller passing the retired kind).
      // Once per process here; the per-node warning naming the offending node is emitted by
      // the field owners (OmViewpoint / OmAbstractCamera / OmCamera renderBackend paths).
      static const bool kWarnedWrenRequest = []() {
        OmLog::warning(QString("[render] a renderBackend resolution asked for \"wren\": WREN is retired; rendering "
                               "through wgpu. (\"wren\" values still parse so legacy worlds load, but they no longer "
                               "select the legacy renderer.)"));
        return true;
      }();
      (void)kWarnedWrenRequest;
    }
    // Lazy wgpu init: paying the loader-init cost only here keeps binary startup paths that
    // never render (e.g. --probe physics-only runs) from warming the wgpu loader.
    OmRenderBackend *vulkan = vulkanBackend();
    if (vulkan != nullptr && vulkan->isAvailable())
      return vulkan;
    // wgpu-native is not usable on this host and WREN is deleted (D1.4): there is no
    // renderer. Warn loudly, once, and hand back the (unavailable) wgpu backend -- callers
    // branch on isAvailable() and degrade to producing no frames rather than crashing.
    static const bool kWarnedNoRenderer = []() {
#ifdef OMNISIM_RENDERERLESS
      // The build named this state: `make release OMNISIM_RENDERERLESS=ON` compiled every
      // wgpu call out (src/omnisim/Makefile refuses a missing wgpu-native otherwise, public
      // issue #7). Say so, instead of sending the reader to debug a wgpu-native install that
      // this binary would never load.
      OmLog::warning(QString("[render] THIS BINARY WAS BUILT WITHOUT A RENDERER (OMNISIM_RENDERERLESS=ON): "
                             "wgpu-native was compiled out and the legacy WREN renderer is deleted (D1.4), so the "
                             "main view, screenshots, the capture service and every Camera/RangeFinder/Lidar device "
                             "produce no frames. Physics and controllers are unaffected. Rebuild with wgpu-native "
                             "(bash scripts/dev/setup_wgpu_native.sh, then make release) for a renderer."));
#else
      OmLog::warning(QString("[render] wgpu-native is UNAVAILABLE on this host and the legacy WREN renderer was "
                             "DELETED (D1.4, wren-deletion-runbook.md): this session has NO renderer -- the main "
                             "view and every wgpu-rendered device will produce no frames. Fix the wgpu-native "
                             "install (see the [OmWgpuBackend] line above for why it failed)."));
#endif
      return true;
    }();
    (void)kWarnedNoRenderer;
    return vulkan;
  }

}  // namespace OmRenderBackendRegistry
