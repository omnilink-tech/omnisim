# R3 — Rendering Backend Evaluation + Design

Scope: pre-implementation design for the rendering arm's first behavior-change phase. R0–R2 landed the abstraction, build flag, and `renderBackend` field; R3 is the first phase where a Camera or Viewpoint with `renderBackend "vulkan"` actually renders through a non-WREN path.

This doc records the design decisions, the backend choice, the cross-backend mesh-data bridge sketch, and the sub-phasing. **It does not change code.** R3 implementation itself is gated by physics P5 per [engine-migration-plan.md](../engine-migration-plan.md) and stays held until that gate clears.

---

## Decision needed: backend choice

Three candidates considered. The R3 layer goes between `WbVulkanBackend` and the GPU; the dispatcher abstraction means we can swap the choice later without changing call sites, but the cost of swapping grows once R3.2+ work assumes a specific API surface.

### Option A — Custom thin Vulkan layer

Hand-write a minimal Vulkan abstraction (~2–5 kloc) that wraps `vkCreateInstance`, `vkEnumeratePhysicalDevices`, device + queue creation, swapchain, descriptor sets, command buffer recording, and the specific draw / RTT calls we need.

| Pros | Cons |
|---|---|
| Maximum control, zero external runtime dep | 2–5 kloc of layer code to maintain forever |
| No risk of upstream API drift | Cross-platform variance (Vulkan on Windows / Linux / Mac via MoltenVK) becomes our problem |
| Tight integration with our existing GL state machine | Single-developer team can't sustain it — bus factor 1 |
| | Locks us to Vulkan only; no Metal / D3D12 / GL fallback |

### Option B — bgfx

[bgfx](https://github.com/bkaradzic/bgfx) is a mature multi-backend graphics abstraction (~250 kloc, BSD-2). Targets Vulkan / Metal / D3D12 / OpenGL / WebGPU from a single API. Ships its own shader-compiler toolchain (shaderc).

| Pros | Cons |
|---|---|
| Multi-backend "for free" — Vulkan on Linux, Metal on Mac, D3D12 on Windows, GL fallback everywhere | ~2 MB binary size increase |
| BSD-2 license, compatible with our Apache 2.0 + brand-protection posture | Shader pipeline is bgfx-specific (shaderc compiles GLSL/HLSL → SPIRV → backend bytecode) |
| Active maintenance, large user base (used by Minecraft Bedrock, others) | Wraps GL — using GL backend would mean two GL contexts running side-by-side with WREN, fighting for state |
| Existing R3.3 sensor RTT pattern (frame buffer object lifecycle) is well-trodden | Cross-context resource sharing between WREN's GL and bgfx's Vulkan is impossible — they live in parallel |

### Option C — sokol-gfx

[sokol-gfx](https://github.com/floooh/sokol) is a single-header (~10 kloc total) multi-backend renderer. Same idea as bgfx but radically simpler and smaller.

| Pros | Cons |
|---|---|
| Smallest possible footprint | No compute shader path — would block T2.2 (GPU-driven culling) |
| Single-header, trivially embeddable | Single render pass per frame, no multi-pass or deferred |
| Most permissive license (zlib) | Caps the ceiling far below what large-world / 20-husky / Tier 2 needs |

---

## Recommendation: **bgfx**

Reasoning:

1. **The team profile rules out Option A.** OmniSim is a small team; a custom Vulkan layer needs continuous platform-specific maintenance. The existing C2 (Cyberbotics) heritage uses GL via WREN precisely because that was tractable for a small team — Vulkan is meaningfully harder.
2. **Option C caps the ceiling too early.** The eventual Tier 2 work (GPU-driven culling, hierarchical LOD, compute-shader broadphase culling) needs compute shaders, multi-pass rendering, and explicit synchronization. sokol-gfx doesn't grow into those without a forklift upgrade.
3. **bgfx's multi-backend story matches OmniSim's cross-platform posture.** OmniSim ships on Windows, Linux, and Mac. bgfx gives us Vulkan on Linux + Metal on Mac + D3D12 on Windows from one code path. Custom Vulkan would need MoltenVK on Mac, which is its own maintenance burden.
4. **The shader pipeline is a tractable problem.** shaderc compiles GLSL source files to backend-specific bytecode at build time. WREN's existing shaders can be ported to shaderc's input format (GLSL + a few `$input`/`$output` annotations) without rewriting the shader logic.

**This decision is deferred to the user before R3 implementation begins.** If a different choice is preferred (custom Vulkan, sokol-gfx, or another library like Diligent Engine or WebGPU-native), this doc gets updated and R3 proceeds with the new pick.

---

## Cross-backend mesh-data bridge

At R3, WREN owns the scene tree. The Vulkan backend (via bgfx) renders a single opt-in Camera's view. The Camera needs access to the scene's geometry. Three things cross the bridge:

### Meshes

WREN's `wren::Mesh` cache already keys on geometry hash with ref-counting (per [Scene.cpp](../../../src/wren/Scene.cpp) and [StaticMesh.cpp](../../../src/wren/StaticMesh.cpp)). Plan: keep that cache as the source of truth; add a parallel bgfx-side cache keyed on the same `Mesh::sortingId()`.

- On first use of a mesh by a Vulkan-backed Camera: read vertex + index data from WREN's CPU-side mesh storage, upload to a `bgfx::VertexBuffer` + `bgfx::IndexBuffer` via `bgfx::createVertexBuffer`/`createIndexBuffer`, store the handles in a `std::unordered_map<size_t, bgfx::VertexBufferHandle>`.
- On mesh destruction (WREN-side ref-count drops to zero): destroy the corresponding bgfx handles. Hook into the existing StaticMesh destructor.
- No GPU-resource sharing between WREN's GL context and bgfx's Vulkan instance — they're parallel.

### Textures

Same pattern: WREN's texture cache (Texture2d) is the source of truth, bgfx-side cache keyed on the same identity (probably the file path hash that WREN already uses). Upload-on-first-use, destroy-on-WREN-side-drop.

PBR material maps (albedo, metallic, roughness, normal) — each is its own texture, same pattern.

### Materials + shaders

The gnarliest piece. WREN has a shader set under [resources/wren/shaders/](../../../resources/wren/shaders/) — GLSL 330. bgfx wants its own shader format (GLSL + `$input`/`$output` annotations) compiled to backend-specific bytecode by shaderc.

Two paths:

**Path 1 — Hand-port shaders.** Take the WREN shaders and rewrite them in bgfx's shader format. Lossy but tractable; the GLSL math is identical, only the I/O declarations and uniform conventions change. Estimated ~1 week per shader for the dozen-or-so WREN default-program shaders.

**Path 2 — SPIRV-Cross translation.** Compile WREN's GLSL → SPIR-V via `glslang`, then use SPIRV-Cross to emit bgfx-compatible source. Faster initially but the cross-translation output is sometimes ugly and may need hand-tuning. Same end state, less initial work, more debugging risk.

**Recommendation: Path 1.** Reason: the WREN shader set is small (<20 shaders), and hand-porting is a one-time cost. SPIRV-Cross is a runtime layer that we'd need to keep maintaining; hand-ported shaders are static artifacts.

---

## `vkCreateInstance` / `vkEnumeratePhysicalDevices` probe

The `WbVulkanBackend` constructor (currently at [src/omnisim/render/WbVulkanBackend.cpp](../../../src/omnisim/render/WbVulkanBackend.cpp)) is the probe site. With bgfx:

```cpp
WbVulkanBackend::WbVulkanBackend() : mAvailable(false) {
#ifdef OMNISIM_WITH_VULKAN
  bgfx::Init init;
  init.type = bgfx::RendererType::Vulkan;
  init.resolution.width = 64;     // dummy off-screen surface for probe
  init.resolution.height = 64;
  init.platformData.nwh = nullptr;  // headless probe — bgfx supports this
  if (bgfx::init(init)) {
    const bgfx::Caps *caps = bgfx::getCaps();
    if (caps->rendererType == bgfx::RendererType::Vulkan) {
      mAvailable = true;
    } else {
      // bgfx fell back to a non-Vulkan backend; treat as unavailable
      // and let the fall-back layer route to WREN.
      bgfx::shutdown();
    }
  }
#endif
}
```

Failure modes that keep `mAvailable=false`:
- `OMNISIM_WITH_VULKAN=OFF` build (most common)
- bgfx::init returns false (no Vulkan-capable device, driver missing, etc.)
- bgfx falls back to non-Vulkan (e.g. GL — which would conflict with WREN's GL context)

A single `WbLog::warning` per world load when fall-back fires. World still loads, every Camera/Viewpoint still renders through WREN.

---

## Sub-phasing within R3

Each sub-phase ends with a behavior-preserving commit + one-revert rollback.

| Sub-phase | Goal | Files | Duration |
|---|---|---|---|
| R3.1 | bgfx init + isAvailable() probe. Build flag actually does something. No rendering yet. | `WbVulkanBackend.cpp`, Makefile (bgfx dep) | 2–3 days |
| R3.2 | bgfx-side mesh cache. Upload-on-first-use from WREN's mesh data. Single-mesh smoke test (no rendering, just verify the upload). | new `src/omnisim/render/WbVulkanMeshCache.{hpp,cpp}` | 1 week |
| R3.3 | Single-Camera render-to-texture: a Camera with `renderBackend "vulkan"` routes its frame through bgfx. Solid-color shader only; no scene geometry yet. | `WbCamera.cpp`, new `WbVulkanCamera.cpp` | 1–2 weeks |
| R3.4 | One default PBR shader hand-ported to bgfx. R3.3's Camera now renders actual meshes with default material. | `resources/bgfx_shaders/`, build system for shaderc | 1–2 weeks |
| R3.5 | Texture bridge. Cameras with textured materials render correctly. | `WbVulkanTextureCache.{hpp,cpp}` | 1 week |
| R3.6 | Golden-image regression: side-by-side WREN vs bgfx render of the same Camera content. Tune until visual parity. | `tests/rendering/r3_parity/` | 1–2 weeks |

Total: **6–10 weeks** for R3 end-to-end, single-developer. Most of the time is in R3.4 (shader port) and R3.6 (parity tuning).

---

## Risks (with mitigations)

| Risk | What could break | Mitigation |
|---|---|---|
| bgfx + WREN GL contexts fighting for state | Frame corruption on the WREN-rendered viewport whenever bgfx's Camera is also rendering | bgfx must use Vulkan only (caps check above). On systems where bgfx falls back to GL, treat as unavailable. |
| Shader hand-port introduces visual drift | Vulkan-rendered Cameras look subtly different from WREN | Golden-image regression suite in R3.6. Accept ≤2% pixel-diff per tile as the parity bar. |
| Mesh cache lifetime bugs | Crashes on world reload when bgfx handles outlive their WREN counterparts | Hook into existing StaticMesh dtor; bgfx handles cleared synchronously. Smoke test: world load → reload → load 100× without leak or crash. |
| Multi-Camera scenes with mixed backends | Two Cameras in one world, one WREN one Vulkan — render order, frame timing, frame buffer ownership | R3.3 explicitly scopes to single-Camera. Multi-Camera mixed-backend is R4 / R5 territory. |
| bgfx build adds platform fragility | Linux / Mac builds break in CI | bgfx ships as a self-contained submodule; the Makefile addition is mechanical. CI matrix gains an `OMNISIM_WITH_VULKAN=ON` entry once R3.1 lands. |

---

## Decisions to confirm before R3 starts

| Question | Default | Notes |
|---|---|---|
| Backend library | bgfx | This doc's recommendation; reversible until R3.2. |
| Shader pipeline | Hand-port (Path 1) | Reversible until R3.4. |
| Probe library | bgfx::init with type=Vulkan | Bound to backend choice. |
| Cross-platform scope | Vulkan on Linux, Metal on Mac, D3D12 on Windows — all via bgfx | Bound to backend choice; gives all three platforms for free. |
| First demo scene | Single-Camera-renders-spinning-cube smoke world | Authored at R3.3. |

If any of these are vetoed, this doc updates first and the R3 phasing is recomputed.

---

## What this doc deliberately omits

- **R4 (main viewport on Vulkan)** — different problem (no off-screen RTT involved). Designed once R3.6 lands and the bgfx pipeline is proven.
- **R5 (sensor render path on Vulkan for Lidar / RangeFinder)** — depends on the sensor-render code being decoupled from WREN-specific assumptions. Out of scope.
- **R6+ (GPU-driven culling, hierarchical LOD)** — Tier 2 fidelity work. Covered by [engine-migration-plan.md](../engine-migration-plan.md) Tier 2 fidelity section. (Historical note: this archive doc predates the §1.1 decision to augment WREN in place; R3+ Vulkan implementation has since been withdrawn — see engine-migration-plan.md §14.2 (wgpu-first locked decision).)

---

## Status

Document only. No code change. R3 implementation start gated by physics P5 per [engine-migration-plan.md](../engine-migration-plan.md).
