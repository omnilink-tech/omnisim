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

#ifndef OM_RENDER_BACKEND_HPP
#define OM_RENDER_BACKEND_HPP

//
// OmRenderBackend — abstract dispatcher between WREN (default, single-
// threaded OpenGL renderer inherited from Webots) and a future GPU-
// driven Vulkan backend. Lives at the foundation of the engine-
// migration plan's rendering arm; see docs/developer/engine-migration-plan.md.
//
// This header is the rendering-arm twin of OmPhysicsBackend.hpp and
// is intentionally a 1:1 port: same Kind enum, same registry shape,
// same gradual-widening discipline. The physics arm has already
// validated the pattern in production (OmPhysicsBackend), so we
// reuse it verbatim rather than inventing a second style.
//
// Design contract (R0 of the engine-migration plan; non-negotiable):
//
//   - WREN remains the default backend on every Camera and on the
//     main viewport. Every existing world stays byte-equivalent.
//   - This header is included on every build, including
//     OMNISIM_WITH_VULKAN=OFF. The Vulkan-specific impl is gated on
//     the build flag at .cpp level; WREN-only builds carry zero
//     Vulkan code.
//   - At R0, the only concrete implementation is OmWrenBackend (a
//     marker that reports kind/name/available). Real rendering still
//     happens through the existing WREN code path; the backend is
//     only consulted at lookup time to decide whether to short-
//     circuit to a non-WREN path. Adding the abstraction with one
//     impl is a no-op refactor — exactly the point.
//   - Per-Camera (and per-Viewpoint) dispatch keys off a forthcoming
//     `renderBackend` SFString field. Default value "wren" -> WREN.
//     Value "vulkan" -> OmVulkanBackend (or fall-back to WREN when
//     unavailable). Future backends slot in via OmRenderBackendKind.
//
// This header is intentionally pure-virtual: no inline implementations,
// no Qt dependencies, no <wren/...> includes. It can be included from
// any TU. Concrete implementations live in OmWrenBackend.{hpp,cpp} and
// OmVulkanBackend.{hpp,cpp} (the latter introduced at R1).
//

#include <cstddef>

enum class OmRenderBackendKind {
  Wren,     // RETIRED (F1, wren-deletion-runbook.md): parses for legacy worlds, warns, resolves to Vulkan/wgpu
  Vulkan,   // the default renderer ("wgpu" in world files; "vulkan" is the accepted alias)
  Unknown,  // unrecognised value in a world file; resolves to Vulkan/wgpu at runtime
};

// String-to-kind helper, defined in the .cpp.
OmRenderBackendKind OmRenderBackendKindFromString(const char *name);
const char *OmRenderBackendKindToString(OmRenderBackendKind kind);

// This interface is intentionally a THIN SELECTION MARKER: kind/name/
// isAvailable only, no render-op virtuals. The original R0 plan to widen it
// in R1+ (framebuffer creation, draw-list submission, render-to-texture) was
// SUPERSEDED by the shipped design: the render operations live in a shared
// CONCRETE layer (OmWgpuSceneRenderer / OmWgpuSurface / OmWgpuRenderTarget),
// dispatched "select-then-concrete" -- pick the backend via the renderBackend
// field + kind()/isAvailable(), then drive that concrete layer (wgpu reaches
// its device/queue through OmVulkanBackend's concrete accessors). The main
// view AND the sensor cameras share that one pipeline, so no per-op virtual is
// needed. Signed off as final for the architectural baseline in
// docs/developer/dispatcher-surface-signoff.md §2 (2026-06-07).
class OmRenderBackend {
public:
  virtual ~OmRenderBackend() = default;
  virtual OmRenderBackendKind kind() const = 0;
  // Best-effort name for diagnostic logging. Concrete implementations
  // typically return OmRenderBackendKindToString(kind()).
  virtual const char *name() const = 0;
  // Returns true if this backend is currently usable (e.g. OmVulkanBackend
  // returns false when the runtime fell back to WREN because
  // OMNISIM_WITH_VULKAN=OFF or no Vulkan-capable GPU is present).
  virtual bool isAvailable() const = 0;
};

// Process-wide registry. Returns a long-lived backend instance for a
// given kind. Owns the lifetime of the singletons; safe to call from
// any thread (both backends init lazily behind std::call_once).
//
// F1 (wren-deletion-runbook.md): resolve(kind) returns the wgpu backend for
// EVERY kind whenever wgpu-native is available -- WREN is unselectable. An
// explicit Wren request warns once and resolves to wgpu; the only path that
// still returns the WREN backend is the wgpu-native-UNAVAILABLE last resort,
// which warns loudly and is deleted at D1.4.
namespace OmRenderBackendRegistry {
  void initialise();
  OmRenderBackend *vulkanBackend();
  OmRenderBackend *resolve(OmRenderBackendKind kind);
}

#endif
