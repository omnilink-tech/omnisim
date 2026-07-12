# R3 — Rendering Backend Evaluation + Design (refreshed)

Design for the rendering arm's first behavior-change phase. R0–R2 landed
the dispatcher abstraction, build flag, and `renderBackend` field; R3 is
the first phase where a Camera or Viewpoint with `renderBackend` set to a
modern-GPU value actually renders through a non-WREN path.

This doc records the **refreshed** design decisions, backend choice,
cross-backend scene bridge, sensor pipeline sketch, and sub-phasing. It
**does not change code**. R3 implementation start is gated by the
[engine-migration-plan.md §16.2](engine-migration-plan.md) re-litigation triggers
— specifically by Newton becoming canonical (trigger #1), which is the
trigger whose firing the rest of this design assumes.

**Supersedes** [archive/r3-rendering-backend-evaluation.md](archive/r3-rendering-backend-evaluation.md),
the original bgfx-leaning evaluation from 2026-04. The archive stays as
historical record; the recommendation here differs.

---

## 0 — What changed since the archived evaluation

The archived doc (April 2026) recommended bgfx, with wgpu-native and
custom Vulkan as alternatives considered and not picked. Four things
have changed since:

1. **The wgpu-native ecosystem stabilized through 2025–2026.** Bevy
   ships on wgpu. Firefox ships WebGPU on Dawn (Chrome's
   reference implementation, also exposed as wgpu-native via Dawn's
   C API). Servo migrated to wgpu. The "young library, will it
   survive" risk that pushed the archive toward bgfx is no longer
   live — wgpu is shipping inside browsers used by billions.

2. **WebGPU's design choices have aged better than bgfx's.** bgfx
   exposes an "old GL with command buckets" feel: per-draw uniforms,
   implicit state machine, single-pass-per-frame. wgpu exposes a
   modern descriptor-set + command-encoder model that maps cleanly to
   Vulkan/Metal/D3D12 native semantics. The translation cost for
   things like bindless materials, indirect draws, and compute
   shaders is much smaller on wgpu.

3. **Newton-on-GPU is the architectural lever.** The archive's cost
   case was sized against a CPU-physics workload where the renderer
   migration buys "future-proofing" but no immediate win. The
   migration's real payoff lives in the Newton-canonical world: GPU-
   resident physics outputs feed directly into a GPU-resident
   renderer with no CPU round-trip. bgfx's per-draw-uniform model
   loses that win; wgpu's storage-buffer / bindless model keeps it.

4. **Apple-GL drop is now a "when not if" timeline.** macOS 15+ has
   begun showing GL deprecation warnings in the system log. wgpu via
   Dawn/Metal handles this transparently. bgfx via its Metal backend
   also handles it, but with more friction (bgfx's Metal path is
   smaller and less battle-tested than wgpu's). Custom Vulkan needs
   MoltenVK, which is its own thing.

The decision in §1 reflects these four.

---

## 1 — Backend choice

Four candidates considered. The R3 layer goes between `WbRenderBackend`
and the GPU. The dispatcher means we can swap the choice later, but
once R3.2+ assumes a specific API surface the swap cost rises sharply.

### Option A — wgpu-native (Recommended) ✅

[wgpu-native](https://github.com/gfx-rs/wgpu-native) is a C/C++ binding
over [wgpu](https://github.com/gfx-rs/wgpu), the Rust implementation of
WebGPU. It targets **Vulkan on Linux, Metal on macOS, D3D12 on Windows**
from one API. Apache 2.0 / MIT dual-licensed.

| Pros | Cons |
|---|---|
| Modern API model (descriptor sets, command encoders, bindless-friendly, indirect draws first-class) | Rust toolchain on contributor machines for the build dependency (one-time `rustup` install) |
| Cross-platform native: Vulkan / Metal / D3D12, plus GL fallback for old hardware | WGSL is the shader format; existing WREN GLSL shaders need conversion (manual or via `naga`) |
| Single shader language (WGSL) — no per-backend bytecode compilation pipeline like bgfx's shaderc | Linking adds a static Rust library (~3–5 MB) |
| Dawn-compatible: when stronger native bindings are wanted, swap to Dawn's C API without changing call sites | WebGPU spec still receiving small refinements through 2026; the underlying API churns at the edges |
| Active ecosystem (Bevy, Firefox, Servo, Deno) — bus factor ≫ 1 | |
| Direct path to compute shaders, storage buffers, bindless materials | |
| **Maps cleanly onto Newton's GPU-resident output** — the architectural lever | |

### Option B — bgfx (archived recommendation)

[bgfx](https://github.com/bkaradzic/bgfx) is the previous recommendation;
see [archive/r3-rendering-backend-evaluation.md](archive/r3-rendering-backend-evaluation.md)
for the full case.

| Pros | Cons |
|---|---|
| Mature, large user base (Minecraft Bedrock, others) | API model is older-feeling (per-draw uniforms, no first-class descriptor sets) |
| Multi-backend out of the box (Vulkan/Metal/D3D12/GL) | Shader pipeline is shaderc — separate toolchain to maintain |
| Self-contained C++ (no Rust dep) | Single-author maintenance — bus factor 1 on the core |
| | Worse fit for the Newton-on-GPU interop case (no first-class bindless / storage buffers) |
| | Compute shader support is good but not as ergonomic as wgpu |

### Option C — Custom thin Vulkan layer

Hand-write ~2–5 kloc of Vulkan wrapping. Rejected for the same reasons
as the archived doc:

- Bus factor 1; maintenance burden grows with every new platform.
- Locks to Vulkan only; macOS needs MoltenVK; Windows-D3D12 path
  closed off.
- The driver-crash class problem from the original 2026-Q2 prototype
  (cost two debugging sessions) reappears with every new vendor.

### Option D — sokol-gfx

Same conclusion as the archived doc: caps the ceiling too early. No
compute shader path → blocks T2.2 (GPU-driven culling) and the
Newton-on-GPU bridge. Cut.

---

## 2 — Recommendation: **wgpu-native**

The case is dominantly about the *architectural payoff* of the migration,
not the rendering-API ergonomics in isolation. wgpu earns this pick for
three reasons that the archived doc didn't weigh heavily enough:

1. **It is the only candidate that natively maps Newton's GPU-resident
   output into the renderer.** Newton's body / particle / contact buffers
   live in GPU memory. wgpu's `wgpu::Buffer` with `STORAGE` usage can
   bind directly to a vertex / compute pipeline reading those buffers.
   bgfx's per-draw-uniform model needs the CPU to upload per-frame data
   it just downloaded from Newton — exactly the round-trip the migration
   is supposed to eliminate.

2. **It is the only candidate whose shader model fits OmniSim's sensor
   workload.** WGSL compute shaders + storage textures + multi-pass
   command encoders are the right primitives for batched sensor render-
   to-texture (lidar / depth / RGB cameras submitting in one
   `wgpu::CommandEncoder::submit`). bgfx can do this but with more
   per-pass setup overhead and worse compile-time validation.

3. **It is the only candidate whose cross-platform story handles every
   one of §16.2's reopen triggers.** Newton-canonical (#1) needs GPU-
   resident scene; wgpu's storage-buffer model has it. Apple-GL drop
   (#3) needs a Metal backend; wgpu's Metal backend is production-tested
   in Firefox. The driver-crash class (referenced in §1.1) is what wgpu-
   core is designed to insulate against.

The runner-up is bgfx. If wgpu's Rust build dependency turns out to be a
deployment blocker that we can't engineer around, bgfx remains a fallback
— but the rest of this design assumes wgpu.

---

## 3 — Architectural payoff

What this migration earns, that staying on WREN forecloses.

### 3.1 — GPU-resident scene (the Newton interop lever)

Today's WREN flow per frame:

```
Newton (GPU)                CPU                       WREN (GPU)
  body positions ---->  copy to host  ---->  upload as uniform per draw
                          (~3000 mat4              (3000 glUniformMatrix4fv,
                           round-trips)             then 3000 glDrawElements)
```

Every model matrix transits GPU → CPU → GPU per frame. T2.2.d-style
shader retrofits batch the *uploads* but don't eliminate the round-trip.

The wgpu-native target:

```
Newton (GPU storage buffer)
  +---> WGSL vertex shader reads from same buffer directly
            (no CPU touch, no upload, indexed by gl_InstanceIndex)
```

A single `wgpu::Buffer` shared between Newton's compute pipeline and the
renderer's vertex pipeline. Newton writes; renderer reads. Zero CPU
intervention per frame for the per-body model matrices.

This is **the** reason to migrate. Every other win is incidental.

### 3.2 — Sensor batched render-to-texture (the multi-robot lever)

OmniSim's positioned target: 16-robot training, 4 cameras each, plus
lidar and depth = ≥ 64 render-to-texture passes per frame at 60 FPS.

WREN today: each Camera does its own `glViewport` + `glBindFramebuffer`
+ `glDrawElements` sequence with full state churn between cameras. The
existing `WbWrenCamera` code is sized for a few cameras, not dozens.

wgpu target:

```rust
// pseudo — actual C bindings
let mut enc = device.create_command_encoder();
for sensor in sensors {
    let pass = enc.begin_render_pass(&sensor.target);
    pass.set_pipeline(&shared_pipeline);
    pass.set_bind_group(0, &scene_bindgroup);     // scene shared across sensors
    pass.set_bind_group(1, &sensor.bindgroup);    // per-sensor camera UBO only
    pass.draw_indexed_indirect(&scene.indirect_buffer, 0);
    pass.end();
}
queue.submit(once(enc.finish()));
```

Single command encoder, N render passes, one submit. Driver batches the
work; the GPU sees one stream of commands instead of N CPU-interleaved
draws.

### 3.3 — GPU-driven culling (the large-world lever)

Tier 2's `T2.2.b/c` (currently deprioritized per §3.0) becomes natural:
WGSL compute shader writes culled-instance indices to an indirect-draw
buffer; the vertex shader consumes that buffer via `draw_indexed_indirect`.
No CPU round-trip for visibility decisions.

This is the marquee modern-GPU-API win. The §3.0 probe showed it's not
the current bottleneck — but the moment scene complexity grows (which
the Newton-canonical world will), it becomes the bottleneck.

### 3.4 — Bindless materials (the asset-pipeline lever)

Today: each material binds its own textures, one program binding at a
time. PBR with five maps × N materials × M renderables = lots of state
changes.

wgpu with descriptor indexing (Vulkan 1.2+ / Metal 3 / D3D12 ResourceHeap):
all textures resident, indexed by material ID per vertex / instance. One
bind group for the entire scene. State changes drop to near-zero across
the frame.

This unlocks Tier 4.2 (deferred decals), Tier 3 SSR + reflection probes,
and any future Lumen-style indirect-light work, all without per-material
bind-group churn.

---

## 4 — Cross-backend scene bridge

At R3, WREN owns the scene tree. wgpu renders **a single opt-in
Camera's** view. The Camera needs access to the scene's geometry,
materials, and (post Newton-canonical) the GPU body buffer.

### 4.1 — Meshes

WREN's `wren::Mesh` cache keys on geometry hash with ref-counting. Plan:
keep that cache as the CPU source of truth; add a parallel wgpu-side
cache keyed on `Mesh::sortingId()`.

- First use of a mesh by a wgpu-backed Camera: read vertex + index data
  from WREN's CPU-side mesh storage, upload to a `wgpu::Buffer` (vertex
  usage + index usage), store handles in
  `std::unordered_map<size_t, WgpuMeshHandles>`.
- On WREN-side ref-count drop: destroy the corresponding wgpu buffers.
  Hook into the existing StaticMesh destructor.
- **No GPU-resource sharing** between WREN's GL context and wgpu's
  Vulkan/Metal/D3D12 instance pre-R6. They are parallel. At R6+ (when
  WREN is no longer the canonical renderer), the wgpu buffers can
  become the source of truth and the GL path goes away.

### 4.2 — Textures

Same pattern: WREN's Texture2d cache is the source of truth, wgpu-side
cache keyed on the same identity. Upload-on-first-use, destroy on
WREN-side drop. PBR maps (albedo/metallic/roughness/normal) are each
their own `wgpu::Texture`.

### 4.3 — Shaders

The hottest piece. WREN has ~20 GLSL 330 shaders under
[resources/wren/shaders/](../../resources/wren/shaders/). wgpu wants
WGSL.

Three paths:

**Path 1 — Hand-port to WGSL.** Take WREN's GLSL, hand-translate to WGSL.
Math is identical; only structure differs (no `uniform` blocks per-se,
all uniforms via `@group(N) @binding(M) var<uniform>`; no `in/out`,
all via `@location`).

Estimated: 1 week per shader for the dozen-or-so core shaders. ~3 months
total, slow path.

**Path 2 — `naga` translation.** `naga` is wgpu's shader compiler — it
ingests GLSL/SPIR-V/HLSL/WGSL and emits any of them. Compile WREN GLSL
→ SPIR-V via glslang, then `naga` SPIR-V → WGSL. Auto-generated WGSL is
sometimes ugly but functionally correct.

Estimated: 1–2 weeks of toolchain setup + per-shader hand-tuning where
naga's output doesn't compile or is suboptimal.

**Path 3 — Selective hand-port.** Hand-port the *core* shaders (PBR,
Phong, default, picking). For the long tail (post-process effects, GTAO,
SMAA, etc.) use `naga` translation. Best of both.

**Recommendation: Path 3.** The core shaders carry most of the
visual-parity risk; hand-porting buys cleanness there. The long-tail
post-process shaders are mechanical and naga handles them fine.

### 4.4 — Newton interop (the marquee R3+ feature)

Post Newton-canonical, the body position buffer is a `wgpu::Buffer` with
`STORAGE | VERTEX` usage, written by Newton's compute pipeline and read
by the renderer's vertex pipeline.

```wgsl
@group(0) @binding(0) var<storage, read> body_transforms: array<mat4x4<f32>>;

@vertex
fn vs_main(
  @location(0) v_coord: vec3<f32>,
  @location(1) v_normal: vec3<f32>,
  @builtin(instance_index) instance_id: u32,
) -> VertexOut {
  let model = body_transforms[instance_id];
  ...
}
```

Single source of truth, zero CPU round-trip. This is what T2.2.d was
trying to fake.

---

## 5 — Sub-phasing within R3

Each sub-phase ends with a behavior-preserving commit + one-revert
rollback. Mirrors the archived doc's structure but updated for wgpu.

| Sub-phase | Goal | Files | Duration |
|---|---|---|---|
| **R3.1** | wgpu-native init + `isAvailable()` probe. Build flag does something. No rendering. | `WbVulkanBackend.cpp` → renamed `WbWgpuBackend.cpp`; Makefile (wgpu-native dep + Rust toolchain detection) | 3–4 days |
| **R3.2** | wgpu-side mesh cache. Upload-on-first-use from WREN's mesh data. Smoke test verifies upload, no rendering. | new `src/omnisim/render/WbWgpuMeshCache.{hpp,cpp}` | 1 week |
| **R3.3** | Single-Camera render-to-texture: a Camera with `renderBackend "wgpu"` routes its frame through wgpu. Solid-color shader, no scene geometry. | `WbCamera.cpp`, new `WbWgpuCamera.cpp` | 1.5 weeks |
| **R3.4** | Path-3 shader port: core PBR + Phong + default hand-ported to WGSL; naga toolchain for long tail. R3.3's Camera renders meshes with default material. | `resources/wgpu_shaders/`, build system for naga | 2 weeks |
| **R3.5** | Texture bridge. Cameras with textured materials render correctly. | `WbWgpuTextureCache.{hpp,cpp}` | 1 week |
| **R3.6** | Golden-image regression: side-by-side WREN vs wgpu render of the same Camera content. Tune until visual parity (≤2% pixel-diff per tile). | `tests/rendering/r3_parity/` | 2 weeks |
| **R3.7 (new)** | Newton interop probe: wire Newton's body buffer to a wgpu storage buffer; verify a single instanced draw reads positions directly. **Gated on Newton P5 + Newton-canonical (§16.2 #1).** | bridge code between `WbNewtonBackend` and `WbWgpuBackend` | 1.5 weeks |

Total **R3.1 through R3.6 single-engineer: 7–10 weeks**, basically the
same as the archived bgfx estimate. R3.7 adds 1.5 weeks but is the win
the whole thing exists for.

---

## 6 — Risks (with mitigations)

| Risk | What could break | Mitigation |
|---|---|---|
| Rust toolchain dependency in OmniSim build | Contributors on machines without rustup hit build failures | One-time `rustup` install documented in build instructions; CI matrix entry pins the Rust version |
| wgpu API churn through 2026 | wgpu update breaks build | Pin to a specific wgpu-native release tag; bump deliberately. WebGPU spec is locked; underlying churn is implementation-detail |
| wgpu + WREN GL contexts fighting for state | Frame corruption | wgpu uses Vulkan/Metal/D3D12 only — never GL. On systems with only GL, treat as unavailable; fall back to WREN |
| WGSL is more verbose than GLSL | Shader port takes longer than estimated | Path-3 (hand-port core + naga long-tail) absorbs most of this risk |
| Mesh cache lifetime bugs | Crashes on world reload | Hook into existing StaticMesh dtor; smoke test (world load → reload × 100) |
| Multi-Camera scenes with mixed backends | Render order, frame timing, frame buffer ownership | R3.3 explicitly single-Camera. Multi-Camera mixed-backend is R4 territory |
| naga's auto-translated WGSL is suboptimal | Performance regression on post-process effects | Path-3 hand-port core; profile naga output for post-process; hand-tune only if observable regression |
| Newton interop assumes physics P5 + canonical | If §16.2 #1 fires before P5, the interop bridge isn't shippable | R3.7 is explicitly gated on P5. If a different §16.2 trigger fires first (Apple-GL drop, GPU-sensor-cost dominance), R3.1–R3.6 still ships and provides value; R3.7 waits |

---

## 7 — Trigger-fired reopening procedure

The §16.2 procedure governs *when* R3 starts. This doc describes *what*
R3 looks like when it does. Summary of the procedure (from
[engine-migration-plan.md §16.2](engine-migration-plan.md)):

1. A trigger fires. Document which one + how it was measured.
2. Re-evaluate the three options from §1.1 against the new evidence.
   This doc's wgpu-native recommendation is the *starting point* for
   that re-evaluation, not a foregone conclusion.
3. If migration is taken: R3 starts at R3.1. The R0–R2 dispatcher seam
   is already in place; no foundation work needed.

Likely first-trigger sequence:

- **If Newton-canonical (#1) fires first** (most likely per
  [engine-migration-plan.md](engine-migration-plan.md)): R3.1–R3.7
  proceed in order, with R3.7 (Newton interop) as the headline.
- **If Apple drops GL (#3) fires first**: R3.1–R3.6 proceed; R3.7
  is deferred until P5 + Newton-canonical. macOS users get the wgpu
  Metal path; Linux/Windows users keep WREN until other triggers fire.
- **If GPU sensor throughput (#2) fires first**: R3.3 + R3.5 +
  multi-Camera (R4) get prioritized over R3.7; the sensor pipeline is
  the headline win.

---

## 8 — Decisions to confirm before R3 starts

| Question | Default | Reversibility |
|---|---|---|
| Backend library | **wgpu-native** | Reversible until R3.2 (mesh cache assumes wgpu's buffer API) |
| Shader pipeline | **Path 3** — hand-port core + naga long-tail | Reversible until R3.4 |
| Probe code site | `WbRenderBackend::isAvailable()` for Kind::WGPU | Bound to backend choice |
| Cross-platform scope | Vulkan on Linux, Metal on macOS, D3D12 on Windows (all via wgpu) | Bound to backend choice |
| First demo scene | Single-Camera-renders-spinning-cube smoke world | Authored at R3.3 |
| Newton interop site | wgpu storage buffer shared with Newton's body buffer | R3.7 scope; depends on Newton API stability at P5+ |

If any default is vetoed when a trigger fires, this doc updates first
and R3 phasing is recomputed.

---

## 9 — What this doc deliberately omits

- **R4 (main viewport on wgpu).** Different problem from R3's off-screen
  RTT (no separate framebuffer; needs swapchain + composition with the
  existing Qt window). Designed once R3.6 lands and the wgpu pipeline
  is proven.
- **R5 (sensor render path on wgpu for Lidar / RangeFinder /
  Recognition).** Depends on the sensor-render code being decoupled
  from WREN-specific assumptions in [src/omnisim/nodes/](../../src/omnisim/nodes/).
  Out of scope here.
- **R6 (canonical-renderer flip).** The point at which `renderBackend`
  default flips from `"wren"` to `"wgpu"` and WREN becomes the legacy
  fallback. Requires §16.2 triggers + complete sensor migration +
  golden-image parity across every shipping demo. Not a short-term
  scope.
- **Tier 2–5 fidelity ports.** Each Tier feature lands first on WREN
  per [engine-migration-plan.md](engine-migration-plan.md); the wgpu port
  happens after R3.6 proves the architecture. The Tier features
  themselves are designed not to be wgpu-specific.

---

## 10 — Status

**Document only.** No code change. R3 implementation start gated by
[engine-migration-plan.md §16.2](engine-migration-plan.md) triggers — none has
fired as of 2026-05-27.

Supersedes [archive/r3-rendering-backend-evaluation.md](archive/r3-rendering-backend-evaluation.md).
The archive doc remains for historical context; this doc is the active
design.
