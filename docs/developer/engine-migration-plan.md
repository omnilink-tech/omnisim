# OmniSim Engine Master Plan

> ## ⚠️ THE PHYSICS ARM IS COMPLETE AND ITS COMPATIBILITY CONTRACT IS RETIRED (2026-08-08)
>
> `bdc02139` deleted the ODE backend (`src/ode/` + `include/ode/`, 106,283 lines).
> **Newton with `SolverMuJoCo` is the only physics backend.** Read every physics section
> of this plan as history: `physicsBackend "ode"`, `OMNISIM_FORCE_ODE`, `OMNISIM_LEGACY`
> and `OMNISIM_ALLOW_ODE_FALLBACK` select nothing, there is no fallback backend, and
> **Phase E's "ODE is never removed — it stays the permanent fallback forever" is
> superseded**. §7's compatibility non-negotiable carries a clause-by-clause amendment.
>
> **The rendering arm is unaffected and still live**: WREN remains canonical, wgpu is
> opt-in and its default flip is still human-gated.
>
> Campaign record, including the six kernel blockers and the defects the deletion left
> open: [ode-retirement-campaign.md](ode-retirement-campaign.md).

> **Current status lives in [§8.1 → "Status refresh — 2026-06-10"](#81--reality-check-vs-the-definition-of-done)
> — code-verified and canonical.** Dated "Status as of …" lines elsewhere in
> this doc are point-in-time snapshots, not the live picture. Short version:
> the architectural-baseline milestone is reached (documented in
> [architectural-baseline.md](architectural-baseline.md) — but **NOT** captured
> as a git tag: `architectural-baseline-v1` was never created; the only baseline
> tag in the repo is `v0.1.0-baseline`); the wgpu main view PASSES the city
> render gate, whose HARD criteria are geometry **coverage** + a deterministic
> **golden-PNG** match — the WREN-vs-wgpu "within-tol" number is **advisory
> only, not a gate** (currently ~37% against a 30% advisory floor; "WREN is not
> a brightness oracle" — the earlier "65% vs 55%" figure was the old
> *non-deterministic* window-grab golden, since replaced), with CSM + TAA + AgX
> in-engine; the full §8 end-state is roughly a third done (~35–40% measured
> Newton corpus-fidelity + a render arm that is gate-passing but not
> parity-complete); both runtime default-flips still gated, so a stock run is
> still ODE+WREN.

**The unified architecture roadmap that ends with OmniSim architecturally
complete:** Newton as the canonical physics solver, wgpu-native as the
canonical renderer, with the ODE+WREN stack remaining as a fallback
forever. (⚠ This sentence used to call that stack "legacy". It is not:
`omnisim-ode` was best or tied-best on 6 of 7 lane-1 analytic scenes, bitwise
reproducible, and the *fastest* single-world backend at every body count
measured. ⚠️ But that is a result about two **integrations**, not two solvers —
bare MuJoCo scored fine on the same scenes and the deficit was in our own
plumbing, since fixed ([correctness-scope.md](../benchmarks/correctness-scope.md)). "Fallback" is its role; "legacy" is not
its fidelity.
Same correction the archive already made to itself in
`archive/newton-default-and-omnisim-rename-plan.md:382`.) Both arms move on the same sequencing because the rendering
migration's architectural payoff is *gated on physics being GPU-resident*.
Doing them separately gets you half the win at full cost; doing them in
sequence gets you the architectural lever that justifies either one.

This doc is the **strategic umbrella *and* the per-arm execution
detail**. Sections §1–§12 own the vision, sequencing, dependency
contract, and end-state definition. Sections §13–§17 own the per-arm
phase status, measurements, decision log, and test matrix that used to
live in `physics-roadmap.md` and `rendering-roadmap.md` before those
were consolidated here on 2026-05-27.

**Supersedes** an earlier narrower "physics + rendering dispatchers"
framing. The framing now is: OmniSim is on a two-decade-old stack
(ODE 2001, WREN-via-Webots 2000s GL forward), and the modern equivalent
of both — Newton + wgpu — was only worth migrating to when they could be
migrated *together* (GPU-resident scene end-to-end). That moment has
arrived because Newton hit production-credible perf in Q1 2026 and wgpu
hit production-credible cross-platform maturity in 2025–2026.

---

## 1 — Vision: architecturally complete OmniSim

**"Architecturally complete"** means: a robotics simulator where physics
and rendering both live on GPU, share the same buffers, and the CPU does
not participate in per-frame body-transform plumbing. Specifically:

| Layer | Today | Architecturally complete |
|---|---|---|
| Physics solver | ODE (CPU, single-threaded, Gauss-Seidel) | Newton (GPU, Featherstone articulation, Apache 2.0) |
| Renderer | WREN (GL forward, per-draw uniforms, CPU-driven submission) | wgpu-native (Vulkan/Metal/D3D12, descriptor sets, compute, indirect draws) |
| Body transform per frame | GPU → CPU → upload-as-uniform → GPU | GPU storage buffer read directly by vertex shader |
| Multi-camera sensor rendering | Per-camera state churn, sized for 1–4 cameras | Tiled rendering, single command encoder, scales to thousands |
| Cross-platform GPU | OpenGL only (Apple deprecating) | Native to each platform via wgpu (Vulkan/Metal/D3D12) |

**The architectural lever:** with both halves migrated, Newton writes
body transforms to a `wgpu::Buffer` and the renderer's vertex shader
reads them via `gl_InstanceIndex` indexing. **The CPU does not touch
per-frame body data.** This eliminates the round-trip that today's
T2.2.d-style shader retrofits can only paper over.

**The non-architectural lever** (just-as-important corollary): legacy
worlds keep working unchanged. ODE and WREN remain as shipping fallbacks
forever. The migration is *off of being the only option*, not off of
existing.

---

## 2 — Today's two-2000s-stack reality

Both ODE and WREN are foundations whose hard ceilings we've already hit
on workloads OmniSim is genuinely targeted at:

| Subsystem | What's hitting the ceiling | Where the ceiling is |
|---|---|---|
| ODE | Single-threaded Gauss-Seidel over coupled joints + contacts | 2-husky head-on: ~98 ms/step. 10-husky: unstable. 20-husky: DNF ([archive/fps-optimization-journey.md](archive/fps-optimization-journey.md)) |
| WREN | Per-draw `glUniformMatrix4fv` + CPU-side frustum culling + single-threaded driver submission | Warehouse_industrial (3062 unit cubes, one mesh): ~22 FPS forward (§3.0 probe in this doc) |
| Sensors (also WREN) | One `glViewport`+`glBindFramebuffer`+draws per camera | Sized for 1–4 cameras. Multi-robot training at ≥ 4 cameras/robot × 16 robots is currently structurally impossible |

The two ceilings are *coupled*. The warehouse perf ceiling is a
per-draw-uniform problem; the multi-husky physics ceiling is a CPU-
solver problem. Fixing physics doesn't help the renderer ceiling, and
vice versa — but fixing them together collapses both ceilings because
the GPU keeps the shared scene buffers and the CPU stops being in the
per-frame loop.

---

## 3 — The successor stack

Each problem has a successor — chosen, in flight, and license-compatible:

### 3.1 — Newton (NVIDIA, Apache 2.0)

GPU-resident rigid-body + articulation solver, built on Warp (NVIDIA's
CUDA-Python toolkit). Powers Isaac Sim 5.0 and Isaac Lab. Featherstone
reduced-coordinate articulation is the right algorithm for vehicle-like
robots (chassis + 4 wheel hinges). Already partially landed as a
dispatcher + opt-in `physicsBackend "newton"` field.

Detail in §13 below. The solver
decision (§1.1) is locked.

### 3.2 — wgpu-native (Rust + C bindings, Apache 2.0 / MIT)

Production C/C++ binding over wgpu, the Rust implementation of WebGPU.
Targets Vulkan on Linux, Metal on macOS, D3D12 on Windows from one API.
Ships inside Firefox, Bevy, Servo, Deno. WGSL shader language. R0–R2
dispatcher seam already landed; R3+ implementation is gated.

Detail in §14.4 below and
[r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md).
The backend pick (refreshed 2026-05-27, was bgfx in the April archive) is
locked **for current direction**, reopens if a §16.2 trigger reframes
the question.

### 3.3 — Why these two specifically

Other candidates were considered for each arm and rejected for
documented reasons (PhysX-5 / custom CUDA on physics, bgfx / raw Vulkan
/ sokol-gfx on rendering). Both choices share three properties that
matter for OmniSim:

1. **Apache 2.0–compatible licensing.** Both can ship in OmniSim without
   creating CLA or sublicense gymnastics with Apache 2.0–derived code.
2. **GPU-resident state with direct interop pathway.** Newton's body
   buffer is a CUDA-resident array; wgpu's storage buffer maps directly
   onto that with Vulkan/Metal CUDA-interop or via Newton's emerging
   wgpu-buffer export hook. Other combinations (PhysX + raw Vulkan,
   Newton + bgfx) don't share this lever as cleanly.
3. **Active multi-million-dollar maintenance.** NVIDIA invests in Newton
   for Isaac Sim; Mozilla + Google invest in wgpu/Dawn for browser
   WebGPU. Neither is a bus-factor-1 dependency.

---

## 4 — Performance targets (research-grounded)

Numbers come from public benchmarks of the same software stacks. Real
OmniSim numbers will land in §13.5 below
and §14.5 below as measurements
arrive on actual OmniSim worlds.

### 4.1 — Physics arm (Newton)

- **Genesis** (Newton-class GPU physics): 43M FPS pure-physics for a
  robot arm + self-collisions on an RTX 4090, ~430,000× real-time. 80×
  faster than Isaac Gym / MuJoCo on equivalent workloads, with
  comparable accuracy. [Stone Tao's benchmark review](https://stoneztao.substack.com/p/the-new-hyped-genesis-simulator-is)
  notes the per-robot rendering cost dominates once visual output is
  required (10–100 FPS with ray-tracing), which is the lever §4.2 below
  addresses.
- **Isaac Lab** (NVIDIA's Newton-precursor PhysX 5 + tiled rendering):
  thousands of parallel environments per GPU. Published architecture
  describes [tiled-camera batching](https://arxiv.org/abs/2511.04831)
  for scaling to 1000+ cameras in one render pass — the exact pattern
  OmniSim's R3.5+ targets.

OmniSim's physics-side measured targets (§13.5 below):

| Workload | ODE today | Newton target |
|---|---|---|
| 2-husky head-on | 98 ms/step | < 5 ms/step |
| 20-husky free-roam | DNF | < 10 ms/step |
| 50-husky free-roam | impossible | 3–5 ms/step (Newton P5 already showed 3.4 ms/step flat at solver layer) |

> **[Note 2026-07-26 — read the "ODE today" column as whole-engine, not solver.]** The
> 98 ms/step is a *full-engine GUI* figure (ODE + WREN + damage system + scene-graph
> traversal), not an ODE solver step time; see the superseded box in §13.5. Same-harness
> ODE step cost on OmniBench's lane-1 scenes is **0.013–0.181 ms/step**. The targets in
> this table are still the right *engine-level* targets — just don't treat the left column
> as an ODE-solver baseline. Likewise the Genesis "43M FPS / 430,000× / 80× faster than
> Isaac Gym / MuJoCo" bullet above is **contested first-party marketing**, not a
> research-grounded target: the methodology critique (substeps=1, one action then ~999
> idle steps, self-collisions off) and the disputed corrected range are recorded in
> [docs/benchmarks/performance-comparison.md §4.8](../benchmarks/performance-comparison.md)
> and [simulator-comparison.md](simulator-comparison.md). Do not size OmniSim's targets
> against it.

### 4.2 — Rendering arm (wgpu)

- **WebGPU vs OpenGL/WebGL** on equivalent forward-render workloads:
  WebGL rendering 15,000 cubes at 30 FPS consumes 95% CPU; WebGPU's
  command-buffer approach drops CPU to "almost zero," per
  [Three.js's WebGPU writeup](https://medium.com/@sudenurcevik/upgrading-performance-moving-from-webgl-to-webgpu-in-three-js-4356e84e4702).
  This is the same CPU-side submission bottleneck T2.2.d's instancing
  trick was working around.
- **wgpu-native vs raw Vulkan** is workload-dependent: 1.65–2× slower
  on M3 Max for a 50K-neuron compute-heavy benchmark, but 20% *faster*
  on NVIDIA A100 for a different large-model workload, per
  [the gfx-rs benchmark discussion](https://github.com/gfx-rs/wgpu/discussions/6688).
  The variance is in CPU validation overhead; the GPU work is
  equivalent. OmniSim's GPU-heavy + bindless-friendly workload is on
  the "wgpu wins" side of this distribution.
- **Bevy** (largest production wgpu consumer) reports
  [10–30% framerate uplift](https://bevy-cheatbook.github.io/setup/perf.html)
  from pipelined rendering (GPU work for frame N runs concurrently with
  CPU work for frame N+1). Free win OmniSim gets once it's on wgpu.
- **Bindless + tiling**, applied to NeRF rendering: 18 → 89 FPS
  (+395%), 420 MB → 180 MB memory, per
  [a 2026 wgpu-py implementation report](https://johal.in/declarative-webgpu-compute-python-wgsl-shader-shaders-2026/).
  The same architectural pattern applies to scatter-placed scenes in
  OmniSim (forest, desert biomes, warehouse).

OmniSim's rendering-side measured targets:

| Workload | WREN today | wgpu target |
|---|---|---|
| warehouse_industrial forward | 35 ms (mostly CPU submission) | < 8 ms |
| 16-robot × 4-cam training scene | structurally impossible | 60 FPS (via tiled-camera R3.5) |
| Sensor-camera render-to-texture cost | dominant on multi-camera worlds | sub-millisecond per camera via tiled batching |

### 4.3 — The combined gain (the architectural lever)

The two arms separately give meaningful but bounded wins. The
*combined* gain — the architectural lever — is on workloads neither arm
can hit alone:

- **64-camera multi-robot training scene at 60 FPS.** ODE makes this
  impossible (physics fails before rendering matters); WREN makes this
  impossible (camera pipeline doesn't scale). Newton + wgpu makes it
  routine: Newton-on-GPU outputs ~5 ms/step physics for 16 robots; wgpu
  tiled-camera renders the 64 viewports in ~5 ms/frame. Combined < 16
  ms/frame = 60+ FPS.
- **GPU-resident agent rollouts.** RL training collects state + observations
  through Python — the state path round-trips today (Newton → host → controller →
  observation). With both arms GPU-resident and a wgpu compute-shader
  observation path, the rollout never touches host RAM for body state.
- **Mobile / Apple-Silicon training.** OpenGL is deprecated on macOS;
  the GL path will fail on a future macOS release. wgpu via Dawn/Metal
  is the only path that keeps OmniSim shipping there.

These workloads are what OmniSim is *positioned* against. Today they're
aspirational. Architecturally complete = they're routine.

### 4.4 — Expected end-state performance (the headline numbers)

Single-table summary of where OmniSim lands when Phase ζ closes. The
left column is what's true today on representative worlds; the right is
the target when both arms finish.

> ⚠ **SUPERSEDED where it quotes physics speedups (2026-08-06).** The
> `98 ms/step → <5 ms/step = ~20×` row is the same retired family as the
> struck-through "17–33×" in §4.1 and §13.5: it compared whole-engine GUI
> figures against a bare solver probe. Same-harness measurement says the
> opposite direction — on the committed `step_cost` bench ODE costs **0.060
> ms/step** on a 5-box scene where Newton costs **1.248 ms/step**. Newton's
> case is **batching** (thousands of parallel GPU envs), not per-step cost.
> The *categorical* rows below (multiplier N/A or ∞ — things ODE cannot do
> at all) are unaffected and are the ones that matter. Never quote a
> Newton-vs-ODE per-step speedup from this table.

| Workload | Today (ODE + WREN) | Architecturally complete (Newton + wgpu) | Multiplier |
|---|---|---|---|
| 2-husky head-on physics | 98 ms/step | < 5 ms/step | ~~**~20×**~~ SUPERSEDED, see above |
| 20-husky free-roam physics | DNF (solver divergence) | < 10 ms/step | **∞** (today impossible) |
| 50-husky free-roam physics | impossible to load | 3–5 ms/step | **∞** (today impossible) |
| warehouse_industrial forward render | 35 ms (CPU submission-bound) | < 8 ms | **~4.4×** |
| 16-robot × 4-cam multi-camera training | structurally impossible | 60 FPS routine | **∞** (today impossible) |
| 64-camera training scene at 60 FPS | impossible (physics + render both fail) | < 16 ms/frame end-to-end | **∞** (today impossible) |
| Sensor render-to-texture per camera | dominant cost on multi-camera worlds | sub-millisecond via tiled batch | **>10×** |
| Newton-on-GPU body transform per frame | CPU round-trip (~3000 mat4 copies for warehouse) | zero CPU touch (storage-buffer read) | **N/A** — architectural |
| macOS shipping risk | breaks when Apple removes GL | works on Metal via wgpu | **N/A** — categorical |
| 50,000-pebble granular sim | impossible | interactive (via CUDA M2 + wgpu indirect draws) | **∞** |
| RL training rollout state path | Newton → host → controller → observation | wgpu compute-shader observation, no host touch | **N/A** — architectural |

The numbers in this table are the **definition of done**. Each row maps
to either a §4.1 physics target, a §4.2 rendering target, or a §4.3
combined-arm target. Real OmniSim measurements as phases land will be
recorded in §13.5 and §14.5; the table here is what they target.

The *categorical* rows (N/A multiplier) are the more important ones in
the long run — they're capabilities the architecture unlocks, not just
faster versions of what works today. Multi-robot multi-camera training,
Apple-Silicon support, and GPU-resident RL rollouts are not "performance
improvements," they're capabilities OmniSim doesn't have today at any
performance level.

---

## 5 — The two arms (where to look for detail)

This umbrella defers the technical detail to the per-arm roadmaps. Both
roadmaps share the dispatcher + compatibility-contract idioms (§7
below).

### 5.1 — Physics arm: ODE → Newton

Doc: this doc.

**Status as of 2026-05-27:**
- P0–P4 + P9 + Phases A/B/C **LANDED**: dispatcher, build flag,
  per-Solid opt-in, sensor read path, world-build path, cross-backend
  contact bridge.
- P5 solver-layer exit criterion **EXCEEDED**: 50 huskies @ 3.4 ms/step
  flat at the solver layer.
- P5 in-OmniSim 20v20 **gated on upstream Newton body-index-30 cliff**.
- P6/P7 **PARTIAL**.
- P8 **NOT STARTED**.
- P1.6 (joint-op widening) **complete** 2026-05-28 across 7 commits
  (`750d84f9` → `27be352a`). 17 virtuals on `OmPhysicsBackend` cover
  per-tick + world-load joint read/write surface across Hinge, Slider,
  Ball, Hinge2, AMotor, LMotor families. Newton overrides where
  meaningful (hinge angle); inherits -1 elsewhere. See §13.3 P1.6 row
  for the carve-out list.
- Phase D (Newton-canonical flip) is the gating event for the rendering
  arm's R3 reopening — see §6.

### 5.2 — Rendering arm: WREN → wgpu

Doc: this doc plus
[r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md)
for the wgpu migration design.

**Status as of 2026-05-28 (wgpu-first locked):**
- R0–R2 **LANDED**: dispatcher, build flag, `renderBackend` SFString
  field on Viewpoint + Camera. All three commits are no-behavior-change
  foundation.
- R3+ **gated on Phase D firing** (Newton-canonical flip, which is in
  turn gated on Newton 1.2 stable releasing). When R3 unblocks, it
  becomes the load-bearing rendering work.
- R3 design refreshed 2026-05-27 — wgpu-native picked over the
  April archive's bgfx recommendation. Seven sub-phases (R3.1–R3.7)
  including the marquee R3.7 (Newton-interop bridge).
- **WREN Tier 1-5 integration is no longer active work.** Per the
  §14.2 wgpu-first decision (2026-05-28), new fidelity features ship
  on wgpu only. Everything already landed on WREN (T1.0 dispatcher,
  T1.3 Mars + Earth sky + sensor-camera determinism, T2.1 timer
  instrumentation) stays and ports forward via Path-3 when R3.4 lands.

---

## 6 — Sequencing: the path to architectural completion

The two arms have an explicit dependency: **the rendering migration's
architectural payoff (R3.7 Newton-interop bridge) is gated on Newton
being canonical**. Until Newton is the default, the GPU-resident scene
buffer that R3.7 reads doesn't exist yet (or, more precisely, exists
only on the rare worlds that opt in to Newton).

This makes the sequencing concrete. **Six phases from today to done:**

```
Phase α  (effectively closed 2026-05-28)
   Physics: P1.6 joint-op widening ✓ (7 slices)
            P5 in-OmniSim 20v20 ✓ (verified end-to-end through harness)
            P6 smoke ✓ (quantitative harness deferred to β)
            P7 reset hook + wire-protocol audit ✓
   Rendering: R0–R2 dispatcher seam ✓
              WREN integration of new Tier 1-5 fidelity work CANCELLED
              (per §14.2 wgpu-first decision 2026-05-28).
              Bug fixes on WREN only from now on.

Phase β  (in progress; partially collapsed into γ/δ on 2026-05-28)
   Physics: P8.1 ✓ statics-on-Newton API + Newton-direct smoke
              (probe re-verified PASS on current build);
              P8.2-P8.4 outstanding (3-5 wk)
            + P6 capture+diff tooling ✓ (capture surfaced a
              RUNTIME-CONFIRMED Newton motor → body translation bug;
              investigation outstanding — see §13.3 P6)
            + Phase D ✓ LANDED 2026-05-28 (Solid.wrl + Robot.wrl
              default "auto"; gating audit at §13.4 D)
   Rendering: nothing on WREN. R3.1+ wgpu work LANDED early —
              see Phase γ.

Phase γ  (scaffolds landed 2026-05-28; ✅ RUNTIME-VERIFIED 2026-05-29
         on a wgpu-ON build — the whole single-Camera-RTT chain now
         proven end-to-end at runtime, not just compile-clean)
   Rendering: R3.1 ✓ wgpu-native init + isAvailable() probe
              R3.2 ✓ mesh cache + R3.2b WREN-mesh adapter
              R3.3 ✓ single-Camera RTT + R3.3b Camera wiring
              R3.4 ✓ shader port — steps 1+2+3+4+5+5b
                    ✅ RUNTIME-VERIFIED 2026-05-29: probes 1-6/10/11 all
                    PASS on a wgpu-ON build (probe 11 = clearAndDrawScene
                    = step-4 scene-walk); camera_wgpu_scene_smoke.omniworld
                    integration reads back 3072/3072 non-clear cyan
                    pixels — the WREN-mesh-readback blocker is GONE via
                    R3.5 primitive codegen.
              R3.5 ✓ texture bridge + R3.5b QImage adapter
              R3.6 ✓ golden-image harness + R3.6b world-mode
   Physics: stabilization — the Newton motor → body translation bug
            (chassis-freeze) is FIXED 2026-05-29 (`d56cbf5`, static
            furniture wrongly got dynamic bodies); Phase-D locomotion
            demos are unblocked (husky drives; head-on stable via
            `OMNISIM_NEWTON_SUBSTEPS`; P6 damage parity within 10×).

Phase δ  (mostly code-landed 2026-05-28; end-to-end demo outstanding)
   Rendering: R3.7 ✓ wgpu storage-buffer instanced draw (host-side
              test data, probe=7 PASS runtime-verified on local
              wgpu-enabled build) + R3.7b ✓ snapshotBodyTranslations
              API in OmNewtonBackend.
   Outstanding: real Newton bodies driving the wgpu storage buffer
              in a real scene. **BOTH gates now CLEARED 2026-05-29:**
              (b) chassis-freeze fixed (`d56cbf5`) → husky drives,
              real position deltas exist; (a) γ scene-walk reproducibly
              runtime-verified on the rebuilt v29 wgpu-ON binary
              (3072/3072 cyan px) after the build-tooling repair
              (`15817ae5`+`d57c0002`). **✅ δ RUNTIME-DEMONSTRATED
              2026-05-29:** the OmCamera δ hook (`OMNISIM_PROBE_DELTA=1`)
              snapshots the live husky bodies and draws them via
              `clearAndDrawInstanced` — `[OmCamera δ] 5 LIVE Newton bodies
              → wgpu instanced storage-buffer draw`, body0 z settling
              0.297→0.285 live. Physics-GPU → render-GPU lever proven
              end-to-end. Opt-in; default scene-walk path byte-unchanged.
              δ polish (MAIN viewport from snapshot, instance culling,
              CUDA/wgpu zero-copy) folds into Phase ε/R4.

Phase ε  (3–6 months)
   Rendering: R4 (main viewport on wgpu)
              R5 (sensor pipeline migration — Lidar / RangeFinder /
                  Recognition / Camera on wgpu, with R3.5 tiled-camera
                  pattern from Isaac Lab as the reference)
              Tier 1-5 fidelity work lands HERE on wgpu — T1.1 AgX,
              T1.2 CSM + PCF, T1.4 TAA, T2.x, T3-T5. Each built ONCE,
              on the right architecture.
              ── Fidelity-prep previews LANDED 2026-05-29 (engine-agnostic,
                 no build): T1.2 CSM (csm-shadows-preview.html), T1.4 TAA
                 (taa-preview.html), T1.1 AgX (agx-tonemap-preview.html).
                 ⚠️ These are standalone WebGL2 HTML A/B pages, NOT engine
                 shaders — the Tier-1 WGSL ports are 0% written (no `.wgsl` in
                 the engine; see §8.1). They are spec-by-example + converged
                 params for the eventual ports, not shipped fidelity.
              ── **R5 RangeFinder (depth) — ✅ IMPLEMENTED + RUNTIME-VERIFIED
                 2026-05-29.** First sensor on the wgpu pipeline. Key insight: a
                 perspective projection's `clip.w` IS the linear view-space
                 distance, so the fragment outputs it directly — no depth-texture
                 readback (sidesteps `Depth24Plus`-not-copyable). Shipped:
                 `kSolidDistance` WGSL (vertex passes `clip.w`, fragment outputs
                 `clamp(viewDepth/far,0,1)` grayscale, `far` in the Scene
                 uniform's pad0.x); `clearAndDrawScene` gained a default-OFF
                 `depthMode`/`farPlane` that swaps to `ensureSceneDepthPipeline()`
                 (reuses the lit bind-group layout + depth view); OmCamera
                 `OMNISIM_CAMERA_DEPTH=1` drives it. **Verified:**
                 `camera_wgpu_scene_smoke` (box front face 0.7 m, far 10) reads
                 back center grayscale **18/255 = 0.0706 ×10 = 0.706 m** — exact.
                 Zero-break: lit path (depthMode off) re-verified cyan
                 3072/3072 on the same binary.
              ── **R5b RangeFinder (REAL-METERS / R32Float) — ✅ IMPLEMENTED +
                 RUNTIME-VERIFIED 2026-05-29.** The precision upgrade the actual
                 RangeFinder *device* node needs: instead of an 8-bit grayscale
                 proof, render `clip.w` straight into an **R32Float** color target
                 and read back metric depth at full float precision (R32Float is a
                 core-WebGPU renderable + *copyable* format, so the readback works
                 where `Depth24Plus` can't). Shipped: `kSolidDistanceF32` WGSL
                 (fragment writes raw `viewDepth` into `.r`, no normalization);
                 `ensureSceneDepthF32Pipeline()` (own R32Float texture; reuses the
                 lit bind-group layout + depth attachment + readback buffer — same
                 4 B/pixel stride); `clearAndDrawSceneDepthF32(farClear, …,
                 float *outMeters)`; OmCamera `OMNISIM_CAMERA_DEPTH=2` drives it
                 (logs precise center metres, normalizes to grayscale for the 8-bit
                 image so existing consumers still work). **Verified** on
                 `camera_wgpu_scene_smoke`: center reads **0.7000 m** (box front
                 face at x=0.7), full float precision vs the grayscale path's
                 quantized 0.706. Zero-break re-confirmed on the SAME binary: lit
                 default = lit-cyan BGRA(0,141,141) 3072/3072; DEPTH=1 grayscale
                 = (18,18,18). The lit + grayscale code paths are byte-untouched
                 (new branch + new methods only).
              ── **R5c real OmRangeFinder DEVICE node — ✅ IMPLEMENTED +
                 RUNTIME-VERIFIED 2026-05-29.** The float target is now wired into
                 an actual sensor device node, not just the Camera proof. The
                 RangeFinder's output buffer is already `float*` metres
                 (`rangeFinderImage()`) — the exact shape
                 `clearAndDrawSceneDepthF32` produces — so the readback lands
                 straight in the device buffer with no conversion. Shipped a
                 shared `OmWgpuSceneRenderer` module (nodes/) holding the scene
                 walk + viewProj + target-management factored out of OmCamera:
                 `ensureTarget()`, `collectWorldDraws()`, `buildViewProj()`.
                 `OmRangeFinder::copyImageToMemoryMappedFile` (opt-in
                 `OMNISIM_RANGEFINDER_WGPU=1`, resolves the Vulkan backend from
                 the registry, falls through to WREN otherwise) walks the world,
                 builds viewProj from the sensor pose + FOV + near + maxRange,
                 renders, clamps to [minRange, maxRange]. **Verified** on
                 `rangefinder_wgpu_smoke` (cyan box front face at 0.7 m fills the
                 frame): `getRangeImage()` centre = **0.7000 m**, min = max =
                 0.7000 (flat wall ⊥ camera axis → uniform view-space depth —
                 correct planar-depth semantics). Camera γ path re-verified
                 byte-identical (lit cyan 3072/3072) on the SAME binary —
                 `OmCamera` is untouched (its object is bit-for-bit the R5b build).
              ── **R5c-follow-up: OmCamera migrated onto OmWgpuSceneRenderer —
                 ✅ DONE + RE-VERIFIED 2026-05-29.** The temporary duplication is
                 gone: OmCamera::ensureWgpuTarget now delegates to
                 `OmWgpuSceneRenderer::ensureTarget`, and its scene walk + viewProj
                 are `collectWorldDraws` + `buildViewProj`. The Camera-specific
                 bits stay in OmCamera (δ probe, DEPTH=1/2 modes, lightDirAmbient,
                 OMNISIM_R34_IDENTITY_VP diagnostic, logging). Camera + RangeFinder
                 are now one render code path. Re-verified all three camera cases
                 byte-identical post-migration: lit cyan BGRA(0,141,141), DEPTH=1
                 grayscale (18,18,18), DEPTH=2 F32 centre 0.7000 m.
              ── **R5d Lidar RADIAL-range foundation — ✅ IMPLEMENTED +
                 RUNTIME-VERIFIED 2026-05-29.** A Lidar ray measures euclidean
                 distance to the hit point, NOT planar depth — so off-axis rays
                 read longer than `clip.w`. Shipped `kSolidRangeF32` (a
                 {viewProj, view, model} uniform — 192 B, reuses the scene
                 bind-group layout — whose fragment outputs `length(viewPos)`),
                 `OmWgpuRenderTarget::{ensureSceneRangeF32Pipeline,
                 clearAndDrawSceneRangeF32}` (renders into the same R32Float
                 target), and `OmWgpuSceneRenderer::buildView` (the view matrix on
                 its own; `buildViewProj` is now `perspective * buildView`).
                 OmCamera `OMNISIM_CAMERA_DEPTH=3` drives it. **Verified** on
                 `camera_wgpu_scene_smoke` (flat box face filling the frame at
                 0.7 m): centre = **0.7000 m** (on-axis range == planar depth),
                 corner = **0.7887 m** (max off-axis — radial > planar, matches
                 √(0.296²+0.222²+0.70²)). DEPTH=2 planar (0.7000) + lit cyan
                 re-verified regression-free on the same binary; additive only
                 (new shader + pipeline + method, the lit/depth/F32 paths
                 untouched). This is the metric a Lidar device node reports — the
                 remaining Lidar work is the wide-FOV cylindrical/spherical
                 PROJECTION (multi-frustum stitch; a single perspective frustum
                 can't span ~180°+), plus the OmLidar device-node wiring.
                 ── OmLidar SCOPING (read 2026-05-29, confirms it's multi-session,
                    NOT a RangeFinder-style drop-in): OmLidar does **not** override
                    `copyImageToMemoryMappedFile`; it fills the mapped file via a
                    private, **non-virtual** `copyAllLayersToMemoryMappedFile()` +
                    `updatePointCloud()`. Output is a **multi-layer** range image
                    (`numberOfLayers × horizontalResolution`) **plus** a point
                    cloud (`WbLidarPoint[]`), not one float buffer. A correct wgpu
                    Lidar therefore needs: (1) a virtual seam on
                    copyAllLayersToMemoryMappedFile (or a render-path hook), (2)
                    multi-frustum render + azimuth/elevation resample into the
                    layer grid (clearAndDrawSceneRangeF32 supplies the per-frustum
                    range), (3) point-cloud derivation from range + ray angles.
                    The R5d radial-range path is the metric foundation; the rest
                    is a dedicated multi-step pass. Next R5: Lidar (per above),
                    then Recognition.
                 ── OmLidar IMPLEMENTATION SPEC (read copyAllLayersToMemoryMapped-
                    File + updatePointCloud 2026-05-29 — turnkey for the next
                    pass): the lidar image is `numberOfLayers × horizontalRes`
                    floats laid out `data[layer*res + col]`, values = radial
                    ranges at **UNIFORM ANGLE**. updatePointCloud rebuilds points
                    as `r·(cosθcosφ, sinθcosφ, sinφ)` with
                    `θ_i = fov/2 + dθ/2 + i·dθ`, `dθ = −fov/w`,
                    `φ_j = vfov/2 + tilt + j·dφ`, `dφ = −vfov/(layers−1)`. A
                    perspective render is uniform-in-tan, so the wgpu path must
                    **angular-resample**: render `clearAndDrawSceneRangeF32` at a
                    frustum covering the FOV (horiz=fov; pick aspect so vert=vfov,
                    i.e. aspect = tan(fov/2)/tan(vfov/2)), then for each (layer j,
                    col i) bilinear-sample the perspective range at
                    `ndc_x = tan(θ_i)/tan(fov/2)`, `ndc_y = tan(φ_j)/tan(vfov/2)`.
                    A single perspective frustum only covers FOV < ~150°, so gate
                    the wgpu path to non-rotating + narrow FOV; fall through to
                    WREN otherwise. Wire as an env-gated (OMNISIM_LIDAR_WGPU=1)
                    branch at the TOP of copyAllLayersToMemoryMappedFile that fills
                    `data` then runs the existing updatePointCloud(0, resolution) —
                    default WREN path untouched (zero regression, like R5c).
                    **Analytic verification**: a flat wall ⊥ axis at planar depth d
                    reads `range[θ] = d/cosθ`, so the box-fills-frame smoke world
                    gives a closed-form expected value per column to assert against.
              ── **R5e OmLidar DEVICE node (single-layer subset) — ✅ IMPLEMENTED
                 + RUNTIME-VERIFIED 2026-05-29.** The spec above, built for the
                 tractable subset: single-layer, non-rotating, narrow FOV (single
                 frustum). `OmLidar::renderRangeViaWgpu` renders
                 `clearAndDrawSceneRangeF32` then angular-resamples (bilinear) the
                 perspective columns into the uniform-angle layout
                 (`θ_i = fov/2 + dθ/2 + i·dθ`), clamps to [minRange,maxRange];
                 wired as an env-gated (`OMNISIM_LIDAR_WGPU=1`) branch at the top
                 of `copyAllLayersToMemoryMappedFile` that then runs the existing
                 `updatePointCloud`. **Verified** on `lidar_wgpu_smoke` (box face
                 0.7 m, fov 0.8): `getRangeImage` centre = **0.7000 m**, edge
                 0.7490 m (monotonic off-axis), and the FULL profile matches the
                 closed form `range[θ]=0.70/cosθ` to **max_err 0.0017 m over 64
                 columns** — not a proof, an analytic match. Default WREN Lidar
                 untouched (gate returns false for any other config → zero
                 regression); the R5-family regression guard (now incl. the Lidar
                 case) re-confirmed ALL PASS on the same binary. REMAINING Lidar:
                 multi-layer (vertical resample) + wide-FOV cylindrical/spherical
                 (multi-frustum stitch) + rotating heads.
                 ── MULTI-LAYER caveat (found while scoping the extension): φ comes
                    from `verticalFieldOfView()` (method = fov·height/width, where
                    height carries a +fov/width half-pixel-alignment term), so the
                    wgpu render must use vertFov = that method value (aspect =
                    tan(fov/2)/tan(vfov/2)) and 2D-bilinear-resample row j at
                    `ndc_y_j = tan(φ_j)/tan(vfov/2)`, `φ_j = vfov/2 − j·vfov/(L−1)`.
                    CRITICAL: the perspective→layer **vertical orientation**
                    (ndc_y→texture-row) cannot be validated by the centred-box
                    smoke world — a layer-flip is symmetric there and would pass
                    silently. The multi-layer pass needs an **asymmetric-elevation**
                    test (e.g. box offset in +z, or two objects at different
                    heights) so a flip is detectable, plus an odd layer count so
                    the centre layer gives the exact φ=0 → 0.70/cosθ anchor. Build
                    it that way, not on the symmetric box.
              ── **R5f OmLidar MULTI-LAYER — ✅ IMPLEMENTED + RUNTIME-VERIFIED
                 2026-05-29.** Done per the caveat above. `renderRangeViaWgpu`
                 gained a `layers > 1` branch: render the radial-range perspective
                 with vertFov == `verticalFieldOfView()` (aspect =
                 tan(fov/2)/tan(vfov/2)), then 2D-bilinear-resample uniform-angle
                 (θ_i, φ_j) into `data[layer*res+col]` with the verified
                 orientation `ndc_y=+1 → row 0` (layer 0 = top = +φ). Gated to
                 non-rotating + tilt 0 + FOV < ~150° (single frustum); single-layer
                 path kept byte-identical. **Verified** on
                 `lidar_wgpu_multilayer_smoke` (7 layers, box offset to z=0.15 for
                 an asymmetric elevation profile): centre layer (j=3, φ=0) centre
                 = **0.7000 m** and its full horizontal profile matches
                 `0.70/cosθ` to **max_err 0.0017 m**; the orientation check reads
                 **above_hits=3 > below_hits=1** (the box-extends-up asymmetry,
                 which a vertical flip would invert) — so the ndc_y→row mapping is
                 proven, not just assumed. Regression guard extended with the
                 multi-layer case; ALL 7 checks PASS (camera ×4 + RangeFinder +
                 Lidar single + Lidar multi) on the same binary. REMAINING Lidar:
                 wide-FOV cylindrical/spherical (multi-frustum stitch) + rotating
                 heads + tilt.
              ── **R5g OmLidar WIDE-FOV (multi-frustum) + azimuth-sign fix — ✅
                 IMPLEMENTED + RUNTIME-VERIFIED 2026-05-29.** Single-layer wide
                 FOV (> 1.4 rad) now splits the azimuth into N = ceil(fov/1.2)
                 sub-frustums, each rendered rotated about the sensor up-axis
                 (`camK = worldMatrix · OmMatrix4(0,0,0, 0,0,1, θ_centre_k)`) and
                 stitched by azimuth. **The wide-FOV seam test caught a latent
                 bug**: the azimuth→ndc_x resample had the wrong SIGN (lidar +θ is
                 LEFT = +y → camera-left → NDC x < 0, per the buildView basis
                 swap), which the symmetric R5e/R5f boxes could not detect (range
                 is mirror-symmetric) but the multi-frustum rotation exposed (box
                 landed on the flank, centre empty). Fixed the sign in ALL three
                 Lidar resamples (single/multi/wide). **Verified**: wide-FOV
                 `lidar_wgpu_widefov_smoke` (fov 2.0, 2 frustums, box straddling
                 the θ=0 seam) centre 0.7002 m, profile matches 0.70/cosθ across
                 the seam to **max_err 0.0009 m**, **seam_gap 0.0000**, flanks =
                 maxRange. And `lidar_wgpu_azimuth_smoke` (box offset +y)
                 DEFINITIVELY confirms +θ = left: all 26 hitting columns at
                 θ∈[+0.33,+0.69], closest +0.474 (matches atan(0.4/0.8)) — no
                 residual mirror. R5e/R5f re-verified ALL PASS (symmetric, the sign
                 flip is invisible to them but now correct). Regression guard now
                 9 checks (camera ×4 + RF + Lidar single/multi/wide/azimuth) ALL
                 PASS. REMAINING Lidar: multi-layer wide-FOV (compose R5f+R5g) +
                 rotating heads + tilt.
              ── **R5h OmLidar MULTI-LAYER × WIDE-FOV — ✅ IMPLEMENTED +
                 RUNTIME-VERIFIED 2026-05-29.** The wide-FOV branch was
                 generalized over layers: each of the N sub-frustums now renders
                 with vertFov == verticalFieldOfView() and is 2D-resampled per
                 (layer, column) — composing the R5g azimuth stitch with the R5f
                 vertical resample + orientation. Single-layer wide-FOV (R5g) flows
                 through the same generalized path (φ=0 centre, re-verified). The
                 narrow paths (R5e/R5f) are byte-untouched. **Verified** on
                 `lidar_wgpu_ml_widefov_smoke` (7 layers, fov 2.0 = 2 frustums, box
                 offset to z=0.15): centre layer (j=3, φ=0) centre = **0.7001 m**,
                 its profile matches 0.70/cosθ ACROSS the θ=0 seam to **max_err
                 0.0009 m**, and orientation **above=3 > below=1** — both the
                 azimuth stitch and the vertical orientation hold simultaneously.
                 Regression guard now **10 checks** (camera ×4 + RF + Lidar
                 single/multi/wide/azimuth/multi-wide) ALL PASS. The non-rotating
                 Lidar projection set (any layers × any FOV) is now complete on
                 wgpu.
              ── **R5i OmLidar TILT — ✅ IMPLEMENTED + RUNTIME-VERIFIED
                 2026-05-29.** tiltAngle is handled by PITCHING the render camera
                 up by the tilt (rotate forward +X toward up +Z = rotation about
                 +Y by −tilt: `cam = cam · OmMatrix4(0,0,0, 0,1,0, −tilt)`), in all
                 three resample branches. The per-layer φ sampling is UNCHANGED —
                 it's relative to the (now tilted) forward, so tilt cancels and
                 lives entirely in the camera orientation; tilt==0 is the identity
                 so R5e/f/g/h stay byte-identical. **Verified** on
                 `lidar_wgpu_tilt_smoke` (tilt 0.3, box offset to z=0.25 so only an
                 up-pitched ray hits it): centre = **0.7313 m** ≈ 0.70/cos(0.3) =
                 0.733 — confirming sign (the up-offset box is hit, not missed) AND
                 magnitude (0.733, not 0.70=ignored, not maxRange=wrong-sign).
                 Regression guard now **11 checks** ALL PASS.
              ── **R5j OmLidar ROTATING HEAD (single-layer) — ✅ IMPLEMENTED +
                 WREN-PARITY VERIFIED 2026-05-30.** The last Lidar feature, and the
                 only stateful/time-dependent one. `renderRotatingWindowViaWgpu`
                 fills the instantaneous fov-window (`mTemporaryImage`) via the
                 radial-range render at the sensor pose yawed by
                 **mPreviousRotatingAngle** (NOT current — that's the basis the
                 shared band-copy's `widthOffset` is computed from, and the heading
                 WREN's one-frame-lagged GL readback also reflects), so the window
                 lands at the SAME global column; the existing band-copy +
                 `widthOffset` + `updatePointCloud` then run byte-unchanged. Rides
                 the same opt-in `OMNISIM_LIDAR_WGPU=1`; single-layer narrow-FOV
                 this increment (multi-layer / wide-FOV rotating + any wgpu miss →
                 WREN fallback, so the default path is byte-untouched).
                 **The crux was the verification, exactly as flagged below.** Built
                 the partial-sweep exact-column harness (`lidar_wgpu_rotating_smoke`,
                 box on the front-RIGHT −y) + a WREN-oracle column diff. It caught a
                 real bug first: rendering at mCurrentRotatingAngle placed the box
                 ~w/2 columns off; switching to mPreviousRotatingAngle fixed it. Now
                 **WREN-parity**: identical closest column (187==187), identical hit
                 band [184..223], clean front-face columns agree to <0.001 m (flanks
                 differ only by the static paths' sub-column resample tolerance);
                 the one-sided box lands all-right-of-centre in the predicted band
                 (anti-mirror + anti-offset). Regression guard now **12 checks** ALL
                 PASS (zero-break re-confirmed on the same binary).
              ── **R5k OmLidar MULTI-LAYER ROTATING — ✅ IMPLEMENTED + VERIFIED
                 2026-05-30.** Composes the R5f vertical resample with the R5j
                 rotating window: `renderRotatingWindowViaWgpu` now 2D-resamples
                 `height()` camera rows by elevation φ(r)=vfov/2−r·vfov/(H−1) into
                 the `height()×width()` window the shared band-copy consumes (layer
                 i reads camera row `(int)(i·skip)`, matching `updatePointCloud`'s
                 phi0+i·dphi); render aspect = tanHalfH/tanHalfV so vertFov==vfov.
                 Single-layer keeps the centre-row path; tilt + yaw stay entirely in
                 the render camera. Same opt-in `OMNISIM_LIDAR_WGPU`. **Verified** on
                 `lidar_wgpu_ml_rotating_smoke` (7 layers, box front-RIGHT + offset
                 UP): centre layer's box lands all-right-of-centre in the predicted
                 column band [179,231] (azimuth, the R5j crux) AND above=3 > below=0
                 layers see it (elevation orientation, the R5f crux) — both hold
                 simultaneously; centre closest 0.7081 m. Guard now **13 checks** ALL
                 PASS.
              ── **R5l OmLidar WIDE-FOV ROTATING — ✅ IMPLEMENTED + VERIFIED
                 2026-05-30. The Lidar family is now COMPLETE on wgpu.** Composes
                 the R5g multi-frustum azimuth stitch with the R5j/R5k rotating
                 window: when a rotating window's fov > 1.4 rad,
                 `renderRotatingWindowViaWgpu` dispatches to
                 `renderRotatingWideFovWindowViaWgpu`, which splits the window
                 azimuth into N = ceil(fov/1.2) sub-frustums, renders each at the
                 sensor pose yawed by **mPreviousRotatingAngle + the per-frustum
                 centre θ_k** (both about +z, so they compose to Rz(prevAngle+θ_k)),
                 +tilt, and stitches by azimuth into the `height()×width()` window
                 (per-row elevation as R5k). The narrow single-frustum path is
                 byte-untouched (separate early-return). Same opt-in
                 `OMNISIM_LIDAR_WGPU`; any wgpu miss → WREN fallback. **Verified** on
                 `lidar_wgpu_wide_rotating_smoke` (fov 2.0 = 2 frustums, 7 layers,
                 box front-RIGHT + offset UP): centre layer's box lands
                 all-right-of-centre in band [179,231] (closest col 188, 0.7091 m)
                 AND above=3 > below=0 (orientation) — multi-frustum stitch +
                 rotating placement + per-layer elevation all hold at once. Guard now
                 **14 checks** ALL PASS (zero-break re-confirmed on the same binary).
                 **No Lidar config remains on WREN fallback** — single/multi-layer ×
                 narrow/wide-FOV × static/rotating × tilt are all on wgpu.

                 Original integration spec (the design the above followed; read the
                 mechanism 2026-05-29 — a genuinely different STATEFUL model):
                 prePhysicsStep yaws the WREN camera by `angle =
                 −(ms·2π·defaultFrequency)/1000` each step (the yaw is applied to
                 mWrenCamera, NOT to matrix()), accumulating mCurrentRotatingAngle;
                 width() = ceil(hres·fov/2π) is the instantaneous window and
                 copyAllLayersToMemoryMappedFile copies only the freshly-swept band
                 [minWidth,maxWidth]+widthOffset into the full-res accumulation
                 buffer with wraparound. wgpu plan: when rotating, render the
                 fov-window at cam = matrix()·Rz(mCurrentRotatingAngle) (+tilt),
                 uniform-angle-resample into mTemporaryImage (width()×height()),
                 SKIP mWrenCamera->copyContentsToMemory, and let the EXISTING
                 band-copy + updatePointCloud run unchanged. Two sign conventions
                 to verify (the Rz yaw direction vs rotateYaw's negative angle, and
                 the widthOffset band placement) + MULTI-STEP verification (step
                 through a sweep, assert the box appears in the band facing it and
                 the accumulation wraps correctly) — not single-shot like the
                 static tests, so it warrants its own careful pass.
                 ── VERIFICATION caveat (the crux, found 2026-05-29): a FULL-sweep
                    test (run ≥2π/Δθ steps, check the box band) is robust but
                    CONVENTION-BLIND — after a full rotation every azimuth is
                    covered, so it cannot detect a reversed yaw direction or a
                    wrong widthOffset. Catching those needs a PARTIAL-sweep test
                    asserting the box lands in the EXACT expected data column,
                    which requires nailing the 360°-buffer mapping (theta0 =
                    minWidth·dtheta − π, dtheta = −2π/resolution, plus the
                    accumulating widthOffset). Design that partial-sweep + exact-
                    column assertion FIRST — it is the hard part, more than the
                    render itself. [DONE 2026-05-30 — see R5j above: this exact-
                    column partial-sweep harness + WREN-oracle diff caught the
                    mCurrentRotatingAngle placement bug the convention-blind
                    full-sweep would have missed.]
              ── **R5 regression guard — ✅ ADDED 2026-05-29.**
                 `scripts/dev/wgpu_sensor_regression.py` runs the camera +
                 RangeFinder smoke worlds headless across every sensor mode and
                 asserts the known-good readback (lit cyan 0,141,141; DEPTH=1
                 18,18,18; DEPTH=2 F32 0.7000; DEPTH=3 radial 0.7000/0.7887;
                 RangeFinder 0.7000), exit 1 on any mismatch. Locks in the whole
                 R5 family (R5/R5b/R5c/migration/R5d) against silent regression —
                 the repeatable form of the manual checks, and a first step toward
                 the Phase ζ golden-image parity harness. Verified ALL PASS.
   Physics: stabilization; Newton-only path becomes the canonical demo set.

Phase ζ  (1–2 months)
   Rendering: R6 — canonical-renderer flip.
              renderBackend default becomes "wgpu".
              WREN demoted to legacy fallback (forever — see §11).
              Visual parity on every shipping demo (golden-image regression).
   = "ARCHITECTURALLY COMPLETE"
```

**Total: 12–18 months single-engineer**, with the dominant phases being
β (physics finishing) and ε (rendering fidelity port). Two-engineer pace
could compress β and ε somewhat; γ and δ are bounded by sequencing more
than by effort.

### 6.1 — Why this sequencing and not parallel

We considered running both migrations concurrently and rejected it:

1. **The architectural payoff requires both halves**, but the rendering
   half *individually* doesn't pay for itself on a CPU-physics workload.
   The §3.0 probe established this — current bottleneck is per-draw
   submission, which T2.2.d-style retrofits address adequately on WREN
   alone. Migrating the renderer first means doing the work for a
   payoff that doesn't materialize until physics also moves.
2. **Single-engineer reality.** "Mid-migration on both engines at once"
   is the failure mode this sequencing eliminates. The physics arm has
   established the dispatcher discipline that lets us migrate one at a
   time without breaking compatibility.
3. **§16.2's trigger #1 is the formal handoff.** Newton-canonical is
   defined as the moment the rendering migration's cost case flips. The
   trigger structure already encodes this sequencing.

### 6.2 — The compatibility cliff and how we avoid it

Each phase has a one-commit rollback at the boundary. Phase boundaries
are designed so that abandoning the migration after any of α–ε leaves
OmniSim in a shipping, supported state. Only Phase ζ (R6, canonical
flip) is irreversible — and even that ships WREN as a permanent
fallback path.

---

## 7 — The compatibility non-negotiable

> ## ⚠️ AMENDMENT 2026-08-08 — THE PHYSICS HALF OF THIS CONTRACT IS RETIRED, DELIBERATELY
>
> **This section is a written promise, and the ODE retirement campaign broke it on
> purpose.** Recording that plainly is the point of this amendment; the clauses below
> are left verbatim so the promise and its retirement are both on the record.
> ⚠ **This retires a written commitment and should be owner-acknowledged rather than
> treated as a routine doc edit.**
>
> `bdc02139` deleted `src/ode/` and `include/ode/` (106,283 lines). Clause by clause,
> for the **physics arm only** — the rendering arm is untouched and every clause below
> still binds it:
>
> | # | clause | status |
> |---|---|---|
> | 1 | *"`OMNISIM_WITH_NEWTON=OFF` builds remain canonical … No file in the existing build is removed or renamed."* | **BROKEN, both halves.** 106,283 lines were removed, and `OMNISIM_WITH_NEWTON=OFF` no longer yields a working simulator — there is no second backend for it to fall back to. The flag survives only as a build-time refusal. |
> | 2 | *"Defaults preserve behavior."* | Already spent by Phase D (2026-05-28) as noted; now unrecoverable for physics. |
> | 3 | *"Per-leaf dispatch — migration is opt-in per-Solid, never world-level."* | **Moot for physics.** There is nothing to opt into or out of; `physicsBackend` has one reachable value. |
> | 4 | *"Runtime fall-through."* | **Fully retired for physics.** There is no fall-through: a missing or broken Newton runtime is a hard failure. `OMNISIM_ALLOW_ODE_FALLBACK` selects nothing. |
> | 5 | *"One-commit rollback at every phase boundary."* | **Not satisfied by the deletion.** Its revert is a multi-thousand-line restore against a moving tree — which is precisely why the campaign's own sequencing bought a soak on the `OMNISIM_WITH_ODE=OFF` *default flip* first, where the revert was one Makefile line. Keep that asymmetry in mind for the rendering arm. |
> | 6 | *"CI matrix expands, never contracts; existing legacy-only entries stay green forever."* | **BROKEN for physics.** The legacy physics entries cannot be green; they cannot run. |
>
> The 2×2 build matrix below is likewise now a 1×2 for physics: only Newton-ON is
> buildable, and "bit-identical to legacy" has no referent on the physics side.
>
> **Phase E in §13.4 says ODE is *"never removed — it stays the permanent fallback
> forever."* That is superseded.** Phase E's aspiration was overtaken by a later owner
> decision to delete. Full record: [ode-retirement-campaign.md](ode-retirement-campaign.md).

The single load-bearing constraint, identical across both arms:

1. **`OMNISIM_WITH_NEWTON=OFF` and `OMNISIM_WITH_VULKAN=OFF` builds
   remain canonical.** Every test that runs today must run identically
   with both flags off. No file in the existing build is removed or
   renamed during the migration. (Note: the build flag is `_VULKAN` for
   historical reasons; the actual backend is wgpu. Renaming is a Phase
   ζ cleanup.)
2. **Defaults preserve behavior.** ⚠ **This clause is HISTORICAL — Phase D
   spent it on 2026-05-28** (see §13.4). `physicsBackend` defaults to
   `"auto"`, not `"ode"`, and `"auto"` resolves to Newton whenever the
   runtime is reachable and the capability gate allows it. `renderBackend`
   still defaults to `"wren"` and the rendering half of this clause still
   holds until Phase ζ.
3. **Per-leaf dispatch.** Migration is opt-in *per-Solid* and *per-
   Camera*, never world-level. Same pattern that's already working for
   Newton.
4. **Runtime fall-through.** ⚠ **The physics half of this clause was
   RETIRED on 2026-08-05 (`85fa6bde`) — "silently" is no longer true and
   was never safe.** The engine now separates a runtime that is *absent*
   (library missing / no CUDA — falls back with one warning per world,
   exactly as written here) from one that is *installed but would not come
   up*, which is refused with a FATAL. Measured on the cold-launch defect:
   5 of 10 launches of a Newton world degraded to ODE under a log saying
   Newton had been requested, and nothing downstream could tell those runs
   apart from Newton runs. `OMNISIM_ALLOW_ODE_FALLBACK=1` restores the old
   behaviour per run. The rendering half is unchanged.
5. **One-commit rollback at every phase boundary.** No phase commits
   anything that requires a follow-up to be working.
6. **CI matrix expands, never contracts.** Every new backend adds a CI
   entry; existing legacy-only entries stay green forever.

The 2×2 build matrix is the load-bearing safety net. Three of the four
entries must reproduce legacy output bit-for-bit:

| Build | Required behavior |
|---|---|
| Newton-OFF, Vulkan-OFF | **Bit-identical to legacy.** |
| Newton-ON, Vulkan-OFF | Identical (no Newton worlds use new field). |
| Newton-OFF, Vulkan-ON | Identical (no Vulkan cameras use new field). |
| Newton-ON, Vulkan-ON | New demos work; legacy unchanged. |

This contract is non-negotiable. If a phase can't satisfy it, the phase
splits or the design changes.

---

## 8 — What "architecturally complete" looks like

When Phase ζ closes:

- `physicsBackend` defaults to `"newton"`; `physicsBackend "ode"` still
  works on every existing world and controller.
- `renderBackend` defaults to `"wgpu"`; `renderBackend "wren"` still
  works on every existing world and viewport.
- The 2×2 build-matrix safety net still holds — `OMNISIM_WITH_NEWTON=OFF`
  + `OMNISIM_WITH_VULKAN=OFF` produces a binary bit-identical to legacy
  OmniSim, useful for hardware-less CI machines and deterministic
  regression tests.
- The Newton body buffer feeds wgpu's vertex pipeline directly. CPU
  does not participate in per-frame body-transform plumbing.
- The 64-camera multi-robot training scene runs at 60 FPS on consumer
  hardware (RTX 4070 / Apple M3 / equivalent).
- macOS users have a working OmniSim irrespective of Apple's GL
  deprecation timeline.
- Every Tier 1–5 fidelity feature from the rendering roadmap has
  shipped on both WREN (interim) and wgpu (canonical).

This is the definition of done. Past Phase ζ, OmniSim's evolution moves
to feature-add (new sensor types, new physics constraints, new fidelity
features) rather than foundational migration. Foundation work for the
two-decade horizon is complete.

### 8.1 — Reality check vs. the definition of done

> **⚠️ Read the 2026-06-10 refresh first.** The original 2026-05-30 audit
> (preserved below as the snapshot) was a *day-5* reading whose headline
> "~1% in" is an **elapsed-calendar-time** ratio (5 days against a 12–18-month
> estimate), **not** a capability measure — and it has been overtaken by ~1300
> commits since (build-default flips, the architectural-baseline milestone
> 2026-06-07 — documented in [architectural-baseline.md](architectural-baseline.md),
> though **never git-tagged** — Newton native collision/contacts W1–W4, the wgpu
> main-view un-gate, the Newton+MuJoCo frictional grasp). This subsection is the canonical,
> code-verified migration status; other status docs point here.

#### Physics-arm update — 2026-06-23 (Newton silent-ODE-fallback init bug FIXED)

> **The biggest reliability gap in the physics arm is closed: Newton now reliably
> ENGAGES wherever the runtime is present.** Through 2026-06-22 a class of
> headless / deploy / CI runs were *silently* running ODE despite resolving to
> `"auto"`→Newton — the root cause is now found and fixed.
>
> - **Silent-ODE-fallback INIT BUG — FIXED (`6a459f84`).** warp's startup banner
>   wrote to a `None`/closed `sys.stdout` under a headless `DEVNULL` launch, so
>   the Newton FFI smoke (`newton.ModelBuilder()`) raised `'NoneType' object has
>   no attribute 'write'`, the `OmNewtonBackend` ctor caught it, and the engine
>   **silently fell back to ODE** — collapsing Newton-tuned worlds for reasons
>   that looked like physics regressions. Fixed by installing a writable-stdio
>   guard *before* the warp import in
>   [OmNewtonBackend.cpp](../../src/omnisim/physics/OmNewtonBackend.cpp). **The
>   real signal that Newton actually drove a world is the
>   `[OmNewtonBackend] world finalised (solver=…)` log line — NOT the earlier
>   `… imports OK` line, which prints even on a fallback.**
> - **`OMNISIM_REQUIRE_NEWTON` (`cfb11d06`) — new deploy/CI assert.** Opt-in
>   (default off); when set and Newton init fails, the engine `OmLog::fatal`s (FATAL
>   log + non-zero exit) instead of silently dropping to ODE. `OMNISIM_FORCE_ODE` /
>   `OMNISIM_LEGACY` still win — an explicit-ODE request short-circuits *before*
>   the Newton ctor, so `REQUIRE_NEWTON` only guards the `"auto"`/`"newton"` paths.
> - **`make release` bundles the Newton runtime BY DEFAULT (`577ff609` +
>   `3de05aa3`).** `BUNDLE_NEWTON ?= 1` (opt-out `=0`), idempotent (skips if
>   `newton-runtime/` is already staged). Build flag stays `OMNISIM_WITH_NEWTON ?=
>   ON`; schema `physicsBackend` default `"auto"`→Newton when the runtime is
>   present. **A from-source clone WITHOUT the runtime still falls back to ODE by
>   design** — the permanent safety net is intact, not removed.
> - **Verified robot reality (Newton-MuJoCo deploys).** With the init bug closed,
>   the earlier "OmniQuad collapses on Newton" reading was largely the *fallback*, not
>   a physics wall: **quadrupeds WALK on Newton-MuJoCo** — OmniQuad +30 m, Go2 +66 m,
>   B2 +95 m, all 0 falls. **Humanoids STAND on Newton-MuJoCo** — G1 / H1 /
>   Valkyrie, as deterministic *pure-pose* statics (classical balance, **not
>   RL**) plus per-robot babysitting knobs; **Atlas
>   never durably deployed**. All legged deploys run `FORCE_MUJOCO` + MJWARP + per-robot
>   knobs, not the default XPBD. **Durable Newton WALKING remains OPEN for every
>   legged robot** — standing ≠ walking, and standing is classical statics, not a
>   learned policy.
> - **Honesty guardrails unchanged.** ODE is **not** deprecated — it stays the
>   permanent fallback. The fidelity tail still stands: ~35–40% world-corpus
>   faithful (5/8 robot worlds), ur_arms ~62 mm drift, mavic diverges,
>   Atlas undeployed. The init-bug fix makes the Newton path *reliable*
>   where present; it does not move the fidelity number.

#### Status refresh — 2026-06-10 (current; code-verified)

Re-audited against the actual tree at HEAD (refreshed 2026-06-10 for the v4.0.0
cut; previous refresh 2026-06-08); every claim cites code. Honest one-liner:
**the architectural-baseline *milestone* is COMPLETE (2026-06-07 — see
[architectural-baseline.md](architectural-baseline.md); ⚠️ documented but **NOT**
captured as a git tag — `architectural-baseline-v1` does not exist in `git tag -l`;
the only baseline tag is `v0.1.0-baseline`); the full §8
"architecturally complete" *end-state* is roughly a third done — a rough
synthesis of the ~35–40% measured Newton corpus-fidelity plus a render arm that
now passes its render gate (the HARD gate is geometry coverage + a deterministic
golden; WREN-vs-wgpu within-tol parity is *advisory, not gating*) but is not
parity-complete, not a single measured
number; and both runtime default-flips are still deliberately gated, so a
*stock* run is still ODE+WREN.** Not "~1%", not "done."

**Build vs. runtime — the load-bearing distinction.**
- Build flags both default **ON**: `OMNISIM_WITH_NEWTON ?= ON`
  ([Makefile:297](../../src/omnisim/Makefile)), `OMNISIM_WITH_VULKAN ?= ON`
  ([Makefile:251](../../src/omnisim/Makefile)). A no-flags `make` compiles both
  successor backends in.
- Runtime defaults **unchanged on purpose**: `renderBackend` defaults to
  `"wren"` ([OmRenderBackend.cpp](../../src/omnisim/render/OmRenderBackend.cpp));
  `physicsBackend` defaults to `"auto"`, which resolves to **Newton when the
  warp/newton runtime is importable, else ODE**
  ([OmPhysicsBackend.cpp](../../src/omnisim/physics/OmPhysicsBackend.cpp)). The
  Newton runtime is **not bundled**, and wgpu needs `WGPU_NATIVE_HOME` at build,
  so a stock binary runs **ODE + WREN** unless the box has warp installed (then
  physics→Newton, render still WREN). Phase ζ's defaults-flip has NOT fired.

**Rendering arm.**
- **Main viewport (R4) is no longer 0% — the audit's "biggest gap" is closed to
  "functional and gate-passing, not parity-complete."** The live `OmView3D`
  frame dispatches `renderMainFrameViaWgpu`
  ([OmView3D.cpp:1509](../../src/omnisim/gui/OmView3D.cpp),
  [:1542](../../src/omnisim/gui/OmView3D.cpp)) — an offscreen wgpu render
  (textured + real-sun-shadowed) → GL blit, **un-gated 2026-06-07** for any
  Viewpoint with `renderBackend "wgpu"`. As of 2026-06-10 the wgpu main view
  **PASSES the city render gate**, whose HARD criteria are geometry **coverage**
  (was anything actually rendered — catches the A1 "floor vanished" bug) + a
  deterministic **golden-PNG** match. ⚠️ **The WREN-vs-wgpu "within-tol" parity
  number is ADVISORY ONLY — printed, not gated** (see
  [render_oracle.py](../../scripts/dev/render_oracle.py) lines 19/22/270: "WREN
  is not a brightness oracle"; the wgpu path gained atmospheric sky +
  hemisphere-IBL ambient + distance fog that legacy WREN lacks, so it is
  intentionally brighter/better-lit and parity dropped *by design*). The current
  **deterministic** city within-tol is **~37%** against a **30%** advisory floor;
  the earlier "65% vs 55%" (commit `83e0599e`) was measured against the *old
  non-deterministic window-grab golden* that swung 16–62% with window layout,
  since replaced by the fixed-resolution offscreen golden. The decisive geometry
  fix was RAW albedo sampling (WREN's texture path is linear, no sRGB decode).
  The wgpu main view also **beats WREN on the city perf benchmark** (async
  pipelined readback, draw-list + bind-group caches). Gate-passing ≠
  parity-complete: real lighting parity (AO/bloom/shadow-softness) and the
  ~100%-parity bar for the ζ default-flip remain open. A separate **true on-screen
  surface/swapchain** path also exists behind `OMNISIM_VIEW3D_WGPU`
  ([OmWgpuSurface.cpp](../../src/omnisim/render/OmWgpuSurface.cpp): create /
  configure / getCurrentTexture / present; picking + selection + overlays in
  [OmWgpuView.cpp](../../src/omnisim/gui/OmWgpuView.cpp)).
- **Sensors (R5)** are deep + headless-verified (Camera depth, RangeFinder, the
  Lidar family) — correct engineering, but off the ζ critical path (ζ is the
  viewport + fidelity, not sensors).
- **Fidelity:** T1.1 AgX is a real in-engine WGSL shader (`kSolidLitAgX`,
  [OmWgpuShaders.cpp](../../src/omnisim/render/OmWgpuShaders.cpp)); T1.2 has
  graduated to **full multi-cascade CSM in-engine** (`kSolidLitTexturedCsm` +
  the 4-cascade `R32Float` shadow array in
  [OmWgpuRenderTarget.cpp](../../src/omnisim/render/OmWgpuRenderTarget.cpp)),
  with soft PCF + contact bias; **T1.4 TAA is in-engine** (Halton sub-pixel
  jitter + ping-pong history + temporal-resolve pass in
  [OmWgpuSceneRenderer.cpp](../../src/omnisim/nodes/OmWgpuSceneRenderer.cpp);
  probes `OMNISIM_PROBE_TAA` / `_TAA_JITTER`). The 2026-06-08→10 fidelity run
  also landed atmospheric sky + day/night, the emissive term (city lights at
  night), distance fog + anisotropic filtering, hemisphere-IBL ambient, and
  MSAA 4× in the textured-shadowed pass. T2–T5 remain not in-engine.

**Physics arm.**
- Newton native **collision shapes** now cover sphere / box / cylinder
  (capsule-substituted to dodge a wheel-lock narrow-phase bug) / capsule / plane
  / triangle-mesh, plus **Hinge2** joints, **static bodies**, and a **MuJoCo
  solver** option for frictional pinch grasps — all in
  [OmNewtonBackend.hpp](../../src/omnisim/physics/OmNewtonBackend.hpp). A static-base
  arm pick-place runs end-to-end on Newton+MuJoCo.
- **Native contact readback is built + wired** (reverses the audit's "real
  contact-impulse API unbuilt"): `getContacts()`
  ([OmNewtonBackend.hpp:175](../../src/omnisim/physics/OmNewtonBackend.hpp))
  snapshots the narrow-phase cache and feeds the damage / contact-points
  supervisor lists ([OmSolid.cpp](../../src/omnisim/nodes/OmSolid.cpp), gated by
  `OMNISIM_NEWTON_NATIVE_CONTACTS`). Caveat: under the default XPBD solver
  `forceMag` is 0 (positional solve) — impulse magnitude only populates under
  MuJoCo — so the damage path can now consume a *native* source, but a full
  contact-impulse API under the default solver is still pending.
- **Measured coverage** (rigorous, from `scripts/dev/newton_coverage.py` +
  `faithful_check.py` — see
  [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md) §2): the robot
  corpus is **100% gate-eligible** (135/135 articulations, after the W1 mesh + W2
  Hinge2 gaps closed), but **eligibility ≠ fidelity** — only **~35–40% of the
  world corpus runs *faithfully* on Newton** (resolves to Newton AND matches ODE
  within the oracle tolerance), with **5 of 8 robot worlds faithful** (incl. rosbot,
  rosbot_xl, panda; ur_arms drifts ~62 mm, mavic is an
  uncontrolled drone, omniquad hits the `omniquad.urdf <rest>` parse collapse). The
  ODE-replacement *workstream* (W0–W7) is ~60% done by task. Real remaining
  gaps: the **PAL write surface is still thin** — Newton functionally
  overrides 7 of ~42 dispatcher virtuals (the body/joint *reads* + `reset`);
  every *write* inherits the base "unsupported" and Newton drives writes through
  its own index API
  ([OmNewtonBackend.hpp:287-330](../../src/omnisim/physics/OmNewtonBackend.hpp)).
  **Legged standing still falls ~t=1.55s** and needs `STATICS=1` / `SUBSTEPS=4`
  babysitting. The 20v20 / body-index-30 multi-body cliff and granular / CUDA
  (§13.7) are unstarted. **[Updated 2026-06-23 — see the physics-arm update
  banner above:** the Newton **silent-ODE-fallback init bug** is now fixed
  (`6a459f84`), so Newton reliably engages wherever the runtime is present, and
  several earlier "Newton collapses" readings were that fallback. Verified
  Newton-MuJoCo deploys: quadrupeds (OmniQuad/Go2/B2) **walk** 0-falls; humanoids
  (G1/H1/Valkyrie) **stand** as deterministic pure-pose statics;
  Atlas undeployed; durable Newton walking still OPEN.**]**

**Testing.** The pre-push gate now runs a **Newton-required** physics gate
(`physics_oracle.py --gate --require-newton`, which asserts Newton was actually
active, not a silent ODE fallback) **plus** a render gate
(`render_oracle.py`), not the audit's "ODE-only gate + trivial magenta-clear
golden" ([.githooks/pre-push](../../.githooks/pre-push)). **[Updated
2026-07-09:** the **panda byte-golden was retired** — it was keyed to the
Franka `panda.wbt` parity world, which was removed when OmniSim narrowed its
arm coverage to a single cobot arm. The render gate now runs the hard
**coverage** check on that arm world (catches a dropped floor / vanished
geometry); the B2 byte-golden is **pending a re-bake** (bake a
golden on a wgpu build with `--update-golden` and re-add the `--golden` flag
to the hook — see the NOTE in `.githooks/pre-push`).**]** Standing GitHub Actions CI
now runs a **targeted RL physics-spec conformance gate**
([.github/workflows/g1-spec-conformance.yml](../../.github/workflows/g1-spec-conformance.yml)
— pytest on every `projects/policies/**` push/PR; there is also a daily
`update_sponsors.yml`). What is still disabled is the **heavy per-OS upstream
build/test matrix** in `.github/workflows.disabled/` — that broad standing CI
is the remaining durability gap, not CI in general.

**What "done" still requires** (see §6 and the per-arm checklists): flip
`renderBackend`→wgpu (needs ~100% lighting parity + cross-platform surfaces —
only Vulkan/Windows is proven), flip `physicsBackend`→newton (needs the runtime
bundled + the PAL write surface + legged stability + contact-impulse under
XPBD), the architectural lever (Newton GPU body buffer read directly by the wgpu
vertex shader) **in production** (demonstrated once in the δ probe, not shipped),
and standing CI. Honest sizing: the hard ~50–60% remains, front-loaded on the
two parity problems that gate the default flips.

#### Original snapshot — audit 2026-05-30 (superseded by the refresh above)

A 5-agent adversarial audit on 2026-05-30 measured the codebase against §8.
Recording the honest delta so this doc stays "the law" and nobody plans off
overstated status. **(Historical snapshot — its "~1% in" headline is
elapsed-calendar-time from day 5, not a capability measure; the bullets below
were true on 2026-05-30 and several are now reversed — see the refresh above.)**

- **Default engines (the load-bearing reality):** on a stock build, *zero*
  pixels render via wgpu and *zero* bodies simulate on Newton. `renderBackend`
  defaults to `"wren"` ([OmRenderBackend.cpp:26-28](../../src/omnisim/render/OmRenderBackend.cpp)); `physicsBackend` schema-defaults to
  `"auto"` but `auto`→Newton only on a `OMNISIM_WITH_NEWTON=ON` binary with a
  live Warp runtime — the shipped build flag is OFF, so `auto`→ODE. wgpu is
  additionally `OMNISIM_WITH_VULKAN=OFF` by default. So Phase ζ's "defaults
  flip" has not happened for either arm.
- **Main viewport (R4) = 0% — the biggest gap.** [`OmView3D.cpp`](../../src/omnisim/gui/OmView3D.cpp) (the actual 3D
  view) contains no wgpu/swapchain/surface/present code; there is no on-screen
  presentation path anywhere in `render/` — `OmWgpuRenderTarget` is offscreen
  RTT + readback only. wgpu cannot put a pixel on screen today even in
  principle. The plan sequences R4 *before* R5; the work did R5 first.
- **Sensor pipeline (R5) is the one deep area** — Camera depth, RangeFinder,
  and the full Lidar family (R5e–R5l: single/multi-layer × narrow/wide-FOV ×
  static/rotating × tilt) are implemented + headless-verified. **Caveat:** all
  opt-in (env-gated), all fall back to WREN, and they produce the *same numbers*
  as the WREN sensors that already ship — i.e. real, correct engineering, but
  not on the Phase ζ critical path (ζ is about the *viewport* + fidelity, not
  sensors). Candidly: R5e–R5l is gold-plating a headless-testable sensor while
  R4 (harder, GUI-gated, higher-visibility) sat untouched.
- **Fidelity (Tier 1-5) ≈ 5% in-engine (was 0% at audit time; T1.1 AgX now
  ported).** UPDATE 2026-05-30: **T1.1 AgX is now a real in-engine WGSL shader,
  wired + runtime-verified** — `kSolidLitAgX` in [`OmWgpuShaders.cpp`](../../src/omnisim/render/OmWgpuShaders.cpp)
  (3×3 inset → log2 → 6th-order contrast → outset, ported from the preview),
  driven by `OmWgpuRenderTarget::ensureSceneAgxPipeline()` + a default-false
  `agxMode` on `clearAndDrawScene`, opt-in via `OMNISIM_CAMERA_AGX=1`
  (commits e50f1b86 shader + 0af498cf wiring). Verified: flag-off golden-gate
  byte-identical (0.00/255), flag-on transforms the drawn geometry
  (box (141,141,0)→(180,180,78), highlight desaturation), zero regression.
  **Still 0% in-engine:** T1.2 CSM + T1.4 TAA (those two remain WebGL2 HTML
  preview pages `docs/developer/*-preview.html`, not shipped passes), plus
  T2–T5. So fidelity is no longer "0% / spec'd-by-preview" wholesale — it is
  T1.1-done, T1.2/T1.4 spec'd-by-preview, T2-T5 unstarted. (At the original
  audit, all of T1.x was preview-only; that snapshot is preserved in the audit
  bullets above but is now superseded for T1.1.)
  - **T1.1 follow-up — exposure DONE (commit 404e3752):** the in-engine AgX
    originally tonemapped the lit colour directly, but that input is already LDR
    (Lambertian × baseColor ∈ [0,1]) with no exposure — so AgX curved an
    already-display-range signal rather than compressing HDR, its actual job (the
    preview multiplies the HDR scene by `uExposure` *before* AgX,
    `agx-tonemap-preview.html:151`). A pre-tonemap exposure/EV scale now lands:
    `kSolidLitAgX` multiplies the lit colour by `u.pad0.x` before `agx()` (with a
    `<= 0 → 1.0` guard). No Scene-struct resize — exposure reuses the existing
    zeroed `pad0.x` slack, which is shared between the two *mutually-exclusive*
    modes that read it (depthMode → farPlane, agxMode → exposure). Driven by
    `OMNISIM_CAMERA_AGX_EV` (integer stops; default 0 → exposure 1.0). Verified on
    a fresh symbol-checked binary: AgX-off → golden gate byte-identical
    (0.00/255); EV=0 → box (180,180,78), byte-identical to the no-exposure AgX
    (exposure 1.0 is a proven no-op); EV=2 (4×) → box (228,228,144), real AgX HDR
    highlight compression (blue lifts 78→144 toward white).
  - **T1.1 follow-up — emissive HDR source DONE (commit b0a2e8d5):** the
    remaining "AgX has no real >1 input" gap is now closed for emissive. Per-
    material emissive (`PBRAppearance emissiveColor × emissiveIntensity`) is
    harvested in `collectShapeDraws` into `OmWgpuSolidDraw.emissiveRGB`, packed
    into the Scene uniform's previously-zeroed `pad1.xyz` (offset 176), and added
    to the lit colour in `kSolidLitAgX` *before* exposure + AgX:
    `lit = (baseColor·intensity + emissive) · exposure`. Only the AgX shader
    reads `pad1`, so every other path is byte-unchanged; emissive defaults to 0
    so worlds without it are untouched. **Verified** (rebuilt VULKAN=ON binary,
    headless from PowerShell): both golden gates exact 0.00/255 (default +
    `_agx`, since those worlds have no emissive); and on `camera_wgpu_emissive_
    smoke.wbt` (black baseColor, emissiveColor 0 0 1 × intensity 4 = effective
    HDR blue 4.0): AgX-off → centre (0,0,0) (plain shader ignores emissive,
    proving non-AgX paths untouched); AgX-on → centre RGB (248,144,144) — the
    4.0 blue compressed to 0.97 and desaturated toward white (R,G lift 0→144),
    AgX doing genuine HDR compression. (Superseded by the specular follow-up
    below; the "still pending: specular" note is now done.)
  - **T1.1 follow-up — specular highlight DONE (commits 717fc735 → 55182c7d):**
    the last named HDR source. A gated Blinn-Phong specular term now lands in
    `kSolidLitAgX`, built in byte-identical sub-steps (each its own verified
    commit): plumb camera world-pos into Scene `pad0.yzw`; vertex shader outputs
    `worldPos`; harvest `1 − roughness` → `pad1.w`; the gated fragment term
    `if (smoothness>0) { H = normalize(−L+V); spec = pow(N·H, shininess)·smooth·4 }`
    (V from camPos−worldPos); OmCamera passes camPos; A/B demo worlds + guard.
    Gating on `smoothness>0` keeps every roughness-1 world (and all non-AgX
    paths) byte-identical. **NB — a mid-stream slip recorded honestly:** the first
    three sub-steps were committed *non-building* — the `.cpp` signature edit
    silently failed while the `.hpp` succeeded, and the failing rebuilds kept
    running the stale binary, masking it; caught + repaired in `235c7ca6`, then
    every step re-verified on a freshly-built binary. **Verified** on
    `camera_wgpu_specular_smoke` (grey 0.5 box, roughness 0.85) vs its rough twin
    (roughness 1.0): AgX-off → (71,71,71) pure diffuse (plain ignores specular);
    AgX-on rough → (146,146,146) diffuse-only; AgX-on smooth → (175,175,175), the
    isolated +29 specular lift; full regression 18/18 PASS. **Geometry note worth
    keeping:** on a *flat* face under the fixed off-axis light, roughness 0 gives
    NO centre highlight (~0.8^256 at the constant half-vector); a mid roughness
    broadens the lobe so a flat face lifts measurably headless. **Net: T1.1 is now
    "AgX curve + exposure + emissive + specular HDR sources, wired +
    runtime-verified."** Remaining for *fuller* HDR: IBL / environment specular
    (a later PBR step).
- **Testing/parity is largely manual + not gated.** Golden-image harness exists
  (`scripts/dev/wgpu_probe_golden.py`) but its only world reference is a 64×48
  *magenta-clear* image — it would not catch a geometry/shading regression. No
  physics-trace test exists. The heavy per-OS build/test CI lives in
  `.github/workflows.disabled/` (inert); the standing GitHub Actions CI that *is*
  live is the targeted RL `g1-spec-conformance.yml` gate (pytest on `projects/policies/**`),
  which does not cover rendering. The pre-push smoke gate forces ODE and skips the
  rendering world. The wgpu sensor regression (14 checks) has tight, genuinely-good
  assertions but is a manual, GPU-gated, one-off. **Default-WREN "zero-break"
  rests on code inspection + per-increment manual re-run, not a standing test —
  and the shared `OmWgpuSceneRenderer` / `OmAbstractCamera` refactor now feeding
  Camera+RangeFinder+Lidar has no tripwire if it breaks the default path.**
- **Newton arm honest readiness ≈ 35-40% as a general ODE replacement** (≈70%
  for the narrow wheeled-vehicle slice): wheeled robots drive correctly
  (chassis-freeze genuinely fixed); but legged standing needs `STATICS=1 +
  SUBSTEPS=4` env-babysitting and still falls at t≈1.55s; damage "parity" is a
  controller-side EMA tuned to a 10× tolerance on a synthetic-impulse proxy
  (real contact-impulse API unbuilt); the PAL is a read-mostly shim (Newton
  overrides ~7 of 42 virtuals; all writes + non-revolute joints + brakes +
  connectors + joint-feedback carved out).
- **Timeline:** active migration is ~5 days old (first commit `4a6ec769`
  2026-05-25 → HEAD 2026-05-30) against a §6 estimate of **12–18 months
  single-engineer**, i.e. ~1% in. Phase labels (α closed, γ/δ verified) make it
  *look* mid-to-late, but the two phases the plan says dominate the budget — β
  (physics finishing) and ε (rendering fidelity) — are the least advanced.
  Commit mix to date skews ~1.5:1 docs/planning over engine code.

**Highest-value next work** (ahead of more sensor variants): R4 main-viewport
on-screen path (needs a wgpu surface/swapchain + `OmView3D` integration — the
real ζ blocker), a non-trivial golden-image reference on an actual scene wired
into the pre-push gate (so the shared-renderer refactor can't silently break the
default WREN path), and then the Tier-1 WGSL ports (AgX/TAA/CSM) the previews
already spec. These are multi-week and partly GUI-gated; they are not
single-session "increments."

### 8.1.1 — Recognition scope correction (verified 2026-05-30)

"Recognition on wgpu" is mostly a non-task — a scope clarification, not pending
work. Verified by reading the code (`OmCamera.cpp`, byte-identical to pre-session
`196547b2`):

- **Object recognition is renderer-agnostic CPU geometry.**
  `computeRecognizedObjects()` ([OmCamera.cpp:1013](../../src/omnisim/nodes/OmCamera.cpp))
  and `setRecognizedObjectProperties()` (1114) use frustum-plane tests,
  bounding-sphere range culling, `isContainedInFrustum`, `projectOnImage` (pure
  trig), object-supplied recognition colors, and `dGeomID` ray occlusion. They
  read **zero rendered pixels**, so they produce identical results on WREN or
  wgpu. There is nothing to "port" — recognition never touched the renderer.
- **Only segmentation (the per-pixel object-ID image) is renderer-coupled**, via
  a second `OmWrenCamera` (1218/1400). That is the genuine wgpu gap, and §7
  already records it as WREN-only with no wgpu equivalent yet. Track the
  remaining rendering work as **"segmentation imaging on wgpu," not "Recognition
  sensor."** (The lone GPU touch in the recognition math, `sphericalFieldOfViewY`
  for spherical/cylindrical FOV, is a shared WREN-camera helper used by every
  backend, not a recognition-render path.)

So the rendering-arm sensor remainder is **segmentation imaging** (small,
WREN-coupled) — object recognition itself is already backend-complete.

---

## 9 — Risks and mitigations

| Risk | What could break | Mitigation |
|---|---|---|
| Newton dependency on NVIDIA / CUDA | Non-NVIDIA users lose the headline benchmark numbers | ODE remains canonical fallback forever; benchmarks document the NVIDIA-only path explicitly |
| wgpu's Rust toolchain dependency | Contributor build complexity | One-time `rustup install` documented in build instructions; CI matrix pins Rust version; wgpu-native ships precompiled binaries for common platforms |
| Newton API churn (young codebase) | Breaking changes between Newton releases | Pin to specific Newton release; bump deliberately; the dispatcher discipline isolates churn to `OmNewtonBackend` |
| wgpu spec / API churn (young) | wgpu update breaks build | Pin to specific wgpu-native release tag; WebGPU spec itself is locked (W3C standard); underlying churn is implementation-detail |
| Schedule slippage on physics blocks rendering | The whole plan stalls | One-arm-at-a-time discipline; rendering Tier 1–5 in-place WREN work proceeds in parallel and ships value regardless of physics |
| Visual parity during R3 | Cameras with `renderBackend "wgpu"` look different from WREN | Golden-image regression suite at R3.6; accept ≤ 2% pixel-diff per tile as parity bar |
| Apple drops GL before Phase γ closes | macOS builds break before wgpu Metal path lands | Phase γ priority bump; meanwhile ship a "GL deprecated, expect breakage on next macOS" warning |
| The four other §16.2 rendering triggers fire before Newton-canonical | Sequencing pressure | Phase γ can start independently; only R3.7 (Newton interop) actually requires Newton-canonical. R3.1–R3.6 ships without it |

---

## 10 — How this doc relates to the per-arm roadmaps

This doc is the umbrella; the per-arm roadmaps are the execution detail.

- **For "what is OmniSim's overall direction?":** read this doc.
- **For "what is the current Newton physics status / next physics phase?":**
  read this doc.
- **For "what's the WREN renderer / Tier N fidelity work / wgpu R3
  design?":** read this doc and
  [r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md).

The per-arm roadmaps own their phase numbering, status tables, perf
measurements, and risk tables. This doc owns the strategic framing,
the cross-arm dependency contract, and the end-state definition.

**Maintenance convention:** when a per-arm phase closes, the per-arm
roadmap updates its status table. This doc updates §5.1 / §5.2 status
lines and the §6 sequencing diagram only at *phase boundaries* (α → β
transition, β → γ transition, etc.). Don't update this doc on every
per-arm sub-phase commit.

---

## 11 — Framing question, answered

Earlier in OmniSim's planning we flagged: *is OmniSim aiming to stay
Webots-compatible?* The answer baked into this plan is **yes — dual-
backend forever**. Both arms migrate *off being the only option*, not
off existing. ODE remains a shipping fallback because deterministic-
regression and hardware-less CI need it — and, as of the 2026-07-24
OmniBench campaign, because it is **the best-scoring integration in the suite**
(not a solver result — [correctness-scope.md](../benchmarks/correctness-scope.md))
on analytic-ground-truth scenes — at dt=4 ms it holds the best or
tied-best value on T1, T2 stick-violation, T4, T5, T6 and T7 (PyBullet
edges it on T3 rolling and T2 slide-accel; MuJoCo 3.8.1 leads none),
with linear momentum exactly zero to double precision and bitwise
determinism. "Fallback" describes its role, not its fidelity. WREN remains a shipping
fallback because the GL path is the lowest-common-denominator on
hardware-poor machines and because Webots-era worlds are easier to
debug-port through a familiar GL stack.

If that constraint is ever relaxed (we decide to break compat for a
faster migration), this plan needs a revision — the dispatcher pattern
can stay, but the gradual-widening discipline becomes optional.

That is **not** the current direction.

---

## 12 — Current focus

- ✅ **Newton silent-ODE-fallback INIT BUG — FIXED 2026-06-23 (`6a459f84`); the
  physics-arm default is now RELIABLE.** This was the load-bearing physics-arm
  reliability gap: under a headless `DEVNULL` launch, warp's import-time startup
  banner wrote to a `None`/closed `sys.stdout`, so the Newton FFI smoke
  (`newton.ModelBuilder()`) raised `'NoneType' object has no attribute 'write'`,
  the `OmNewtonBackend` ctor caught it, and the engine **silently fell back to
  ODE** — so Newton-tuned worlds collapsed for reasons that masqueraded as physics
  regressions (this accounts for much of the earlier "OmniQuad collapses on Newton"
  history). Fixed via a writable-stdio guard installed *before* the warp import in
  `OmNewtonBackend.cpp`. **Newton now reliably ENGAGES wherever the runtime is
  present.** Two corollaries: (a) the real "Newton actually drove this world"
  signal is the `[OmNewtonBackend] world finalised (solver=…)` log line — **NOT**
  the earlier `… imports OK` line, which prints even on a fallback; (b) new knob
  **`OMNISIM_REQUIRE_NEWTON`** (`cfb11d06`, opt-in/default-off) turns an init
  failure into a `OmLog::fatal` (FATAL + non-zero exit) instead of a silent ODE
  drop, as a deploy/CI guard (`FORCE_ODE`/`LEGACY` still win — explicit ODE
  short-circuits before the Newton ctor). And `make release` now bundles the Newton
  runtime **by default** (`577ff609` + `3de05aa3`, `BUNDLE_NEWTON ?= 1`, idempotent —
  skips if `newton-runtime/` is already staged); a from-source clone *without* the
  runtime still falls back to ODE by design. **Verified robot reality on
  Newton-MuJoCo** (`FORCE_MUJOCO` + MJWARP + per-robot knobs, not default XPBD):
  quadrupeds **walk** 0-falls (OmniQuad +30 m, Go2 +66 m, B2 +95 m); humanoids **stand**
  as deterministic *pure-pose* statics, **not RL** (G1/H1/Valkyrie) + babysitting
  knobs; **Atlas never durably deployed**; **durable
  Newton WALKING remains OPEN for every legged robot.** ODE is **not** deprecated —
  it stays the permanent fallback. **ODE-deprecation DIRECTION** (not a flip today):
  reliable-Newton-default (now) → wire `OMNISIM_REQUIRE_NEWTON` into the legged-deploy
  recipes + a GPU-runner CI lane → eventual Phase E "`auto`→Newton-with-no-fallback,
  `ode` an explicit opt-out, ODE kept as a fallback forever." See §13.4 Phase D/E +
  the §8.1 2026-06-23 physics-arm update banner.

- ✅ **Phase-D hybrid-backend-split regression — FIXED 2026-06-01**
  (`OmSolid::effectivePhysicsBackendName`): the third independent fallout
  of Phase D's `"auto"` default. An imported child Solid that kept the
  default `physicsBackend "auto"` was resolved to Newton *even when its
  ancestor `URDFRobot` was explicitly `physicsBackend "ode"`* — the old
  resolver returned the local non-`"ode"` value immediately, so a child's
  *default* `"auto"` overrode the robot's *explicit* `"ode"`. Result: an
  ODE-backed robot was simulated on ODE for its root/chassis but had its
  deeper articulated links (every joint NOT attached to the root —
  OmniQuad's hip_y + knee, but not hip_x) registered as Newton bodies/joints.
  ODE actuated those joints correctly, but `OmHingeJoint::postPhysicsStep`
  read their angle from Newton (which wasn't stepping that robot) → the
  joint **position sensors froze at their seed value** while the joints
  physically moved. This blinded every closed-loop controller: the
  canonical ODE residual walker (`omniquad_residual_deploy.omniworld`,
  +5.55 m straight as of 2026-05-24) collapsed/fell at <1 s because its
  18-dim observation read 8 of 12 joint angles as constants. **Root cause
  pinned empirically:** gravity-off probe showed the knee's *physical*
  relative rotation tracked its command (29°→66°) while the sensor stayed
  pinned at the −0.01 seed; the ODE-world log showed 8 spurious Newton
  hinge registrations (hip_y+knee of all 4 legs, hip_x absent because the
  chassis stayed ODE). **Fix:** an articulation is one coupled multibody
  system and must use a single solver, so an EXPLICIT ancestor backend
  (`"ode"`/`"newton"`) now governs a descendant's default `"auto"`. This
  restores the documented `OmUrdfImporter` inheritance contract
  (set `physicsBackend` once on the Robot, propagate to all links) that
  the default-flip broke. **Verified:** ODE world now registers 0 Newton
  hinges, hip_y/knee sensors track (`0.30→0.30`, `−0.60→−0.60`), the
  residual walker stands (bz≈0.67) and walks again; Newton husky (4
  wheels @2.5 rad/s) and G1 (13 hinges) unaffected — Newton robots
  already resolved to newton, only explicit-`"ode"` robots with `"auto"`
  children change. (Residual yaw/lateral drift remains: the analytic gait
  + residual policy were tuned for 2026-05-24 ODE physics; the model-only
  walker drifts too, so it is a separate gait-transfer matter, not this
  sensor bug — a retrain on current physics restores straightness.)

**Active work** (2026-05-29):

- **Physics arm**: P1.6 ✅, P5 ✅ (20v20 load + finalize verified
  end-to-end), **P6 chassis-freeze ✅ FIXED 2026-05-29** (`d56cbf5`;
  the canonical `newton_husky_smoke_test.omniworld` husky now DRIVES
  -7.0 → +1.09 m at 0.41 m/s and settles onto its wheels; head-on
  damage capture 0 → 57,161 contact events). Root cause was NOT the
  motor/joint wiring — it was Phase D's `"auto"` default making STATIC
  furniture (RectangleArena floor, OmniSimSunMarker) register as
  *dynamic* Newton root bodies; their per-step `changed` pose signal
  fired `resetJointsToDefaults()` on the shared articulation twice per
  step, clobbering every robot back to spawn. Fix = skip no-Physics
  Solids in `OmSolid::flushPendingNewtonRegistrations`
  (`subtreeHasPhysics`). The ke/kd wiring fix (`46f2d9ba`) was correct
  + necessary but not sufficient. See §13.3 P6 row. P7 ✅, **P8.1 ✅
  LANDED 2026-05-28**. Next physics work — see §13.3 ranked priorities.
- ✅ **Phase-D vs test-matrix regression — FIXED 2026-05-29** (`c92d81e4`):
  the `"auto" → newton` default flip silently moved EXISTING ODE
  determinism worlds onto Newton on Newton-ON builds, failing 2 of 4
  pre-push smoke worlds (`accelerometer` "wrong acceleration",
  `contact_points` "cone not rotated as much" — the latter built on
  ODE-specific `ContactProperties` softCFM/softERP). Confirmed
  pre-existing at HEAD (not the freeze fix) via stash+rebuild A/B; it
  violated the §16 "Newton-ON: existing worlds stay identical" row. Fix:
  **`OMNISIM_FORCE_ODE`** env knob — `OmPhysicsBackendRegistry::resolve()`
  short-circuits every kind (Auto + explicit Newton) to ODE when set
  (read once via a function-local static; inert otherwise).
  `tests/smoke/run_smoke.py` exports it for the whole suite, so the
  safety net is now build-config independent (all 3 dynamic smoke worlds
  green; a normal Newton husky run is unaffected). The determinism test
  runner should set it on Newton-ON build machines.
- ⚠️ **Build hygiene 2026-05-29**: the in-tree `omnisim-bin.exe` was
  STALE vs source (built 17:47, before the `46f2d9ba` ke/kd fix at
  22:13) — its jointdiag showed `model_ke=20/kd=3` not the fixed
  `0/500`. Always confirm `nm OmNewtonBackend.o | grep snapshotBody`
  resolves + jointdiag shows the expected ke/kd before trusting a
  Newton runtime result. Build must use **`make OMNISIM_WITH_NEWTON=ON`
  (command-line var, NOT env)** or the Makefile's `?=` default silently
  compiles the NEWTON-OFF stub path.
  Toolchain `make`/`g++` live in
  `C:\msys64`, not the repo's bundled runtime-only `msys64`.
- **Rendering arm**: **WREN gets bug fixes only** (locked 2026-05-28,
  see §14.2 + §15). R3.1–R3.7 + R3.7b all landed 2026-05-28 (same
  session as Phase D); all changed sources compile + link clean on
  default build. The default shipped binary is `WB_WGPU_NATIVE_AVAILABLE=0`
  (wgpu probes log `unavailable`, ODE/Newton-only users unaffected).
  **wgpu build tooling REPAIRED + γ reproducibly RE-VERIFIED 2026-05-29:**
  `setup_wgpu_native.sh` was broken (404 on nonexistent v24.0.5.1 + msvc-vs-gnu
  mismatch) — fixed to v29.0.0.0/gnu (`15817ae5`); `build_with_cd.sh` now
  forwards the wgpu flags (`d57c0002`). On the rebuilt v29 wgpu-ON binary:
  `wgpu-native init OK`, RTT `clearAndRead PASS`, and `camera_wgpu_scene_smoke.omniworld`
  reads back **3072/3072 cyan pixels** (`rendered through wgpu`). So the wgpu
  runtime path (R3.1/R3.3/R3.4 = Phase γ) is now reproducibly green from the
  fixed scripts — the rendering arm's next runtime work (Phase δ) is genuinely
  unblocked, and **Phase δ is now RUNTIME-DEMONSTRATED** (see §14.3 R3.7 +
  §6 Phase δ). **§7 fall-through VERIFIED (no regression):** an earlier draft of
  this note flagged a wgpu-camera-world crash on wgpu-unavailable builds — that
  was a transient **build-inconsistency artifact** (a half-stub binary from a
  failed build sequence), NOT a real bug. `OmCamera::ensureWgpuTarget()` guards
  on `back->isAvailable()` (line 254); the clean default (wgpu-OFF) binary runs
  `camera_wgpu_scene_smoke` to completion via graceful WREN fall-through — no
  crash (verified 2026-05-29). The δ hook (`OMNISIM_PROBE_DELTA`) compiles + is
  inert on the default wgpu-OFF build (verified), so the shipped default render
  path is byte-unchanged. Previews (WebGL2) stay live as engine-agnostic
  spec-by-example — T1.2 CSM preview + **T1.4 TAA preview** (`taa-preview.html`,
  landed 2026-05-29: Halton jitter + history reprojection + 3×3 clamp + feedback,
  converged params jitter ±0.5px / feedback 0.90 / clamp ON) both in repo.

**Highest-leverage item — RESOLVED 2026-05-29** (`d56cbf5`): the
Newton chassis-freeze. Two compounding bugs, both in OmniSim wiring
(none in Newton):

1. **ke/kd wiring** (`46f2d9ba`, 2026-05-28): env defaults 20/3
   unconditionally overrode the per-joint ke=0/kd=500 from
   OmBasicJoint. Correct + necessary, but **not sufficient** — the
   chassis stayed frozen even after it. (The session that landed
   `46f2d9ba` never rebuilt the in-tree binary, so its jointdiag still
   read `model_ke=20/kd=3`; the freeze was diagnosed against a stale
   binary.)
2. **Static furniture as dynamic Newton bodies** (the real freeze):
   Phase D flipped the `Solid`/`Robot` `physicsBackend` default to
   `"auto"`, so every static no-Physics Solid (RectangleArena floor,
   OmniSimSunMarker) stopped being skipped by the
   `effectivePhysicsBackendName()=="ode"` gate and got a *dynamic*
   Newton root body (0.25 kg fallback mass). Each is a top-level (root)
   Solid whose pose field still emits `changed` every tick, so its
   `OmSolid::syncNewtonPoseFromFields` supervisor-write bridge fired
   `resetJointsToDefaults()` on the SHARED `webots_world` articulation
   **twice per step**, snapping every robot's joint_q + body_q back to
   spawn and zeroing velocities → total freeze, solver-independent.
   The prior session missed this because the reset log capped at 20 and
   stamped all calls the same wall-second, so per-step resets looked
   like a one-time startup cascade. An uncapped, step-tagged reset log
   showed `reset_joints#N step=K` advancing in lockstep with the step
   counter — the smoking gun. A `syncNewtonPoseFromFields` trace then
   named the culprits: `solid=sun_marker_driver` + `solid=rectangle
   arena`, NOT the husky.

   **Fix**: `OmSolid::flushPendingNewtonRegistrations` now skips Solids
   with no Physics node anywhere in their fixed-child subtree
   (`subtreeHasPhysics`). Statics stay ODE-side (ground contact via
   `add_ground_plane` + the cross-backend bridge) until P8 statics-on-
   Newton lands — matching ODE's own static-vs-dynamic rule.

**Verification** (rebuilt binary, both fixes live):
`newton_husky_smoke_test.omniworld` chassis drives -7.0 → +1.09 m at
chassis_vx ≈ 0.41 m/s (= 0.165 m wheel × 2.5 rad/s), wheels reach
2.5 rad/s, body settles onto wheels (z 0.297 → 0.131), per-step reset
count 0, body list 7 → 5 (arena + sun marker no longer registered).
End-to-end P6 Newton head-on damage capture: **0 → 57,161** contact
events.

**Immediate follow-ups — both RESOLVED 2026-05-29:**
- ✅ **Head-on XPBD NaN** (`3ba5e079`): the un-frozen huskies' 50/30 rad/s
  head-on crash NaN'd XPBD at step 1. Empirically isolated to SPEED, not
  spawn penetration (re-run at clearance z=0.3 still NaN'd; z=0.1 at 2.5
  rad/s was stable — refuting the workflow's plausible penetration
  hypothesis). Fix = `OMNISIM_NEWTON_SUBSTEPS` (default 1 = byte-identical;
  substeps=4 makes the head-on finite). Single-husky drive unchanged.
- ✅ **P6 damage-debounce parity** (`dadd50ae`): the 57,161-vs-149 gap was a
  Newton-velocity-jitter artifact in the tracker's synthetic
  `impulse_J = mass*|Δv|` proxy (the contact API exposes no real impulse).
  Fix = `OMNISIM_DAMAGE_VEL_SMOOTH` EMA low-pass on per-body velocity
  before differencing (default-off; ODE unchanged). Measured (50/30
  head-on, 30 s, substeps=4): **57,161 → 58** events (vs ODE-raw 39) at
  vs=5 — practical parity (within the `damage_events_diff` 10× tolerance).
  The engine-side contact-depth API (so the proxy isn't needed) is now
  **LANDED** (`94e07156` — `ContactPoint.depth` carries the real ODE
  `dContactGeom.depth`). **BUT depth-level gating was tested + RULED OUT
  2026-05-29** (see p6-captures/README §"depth-LEVEL gating RULED OUT"): a gate
  that skips shallow contacts drops only the approach-phase shallow contacts
  (~965), not the inflation — the 9306-event Newton inflation is `body_qd`
  velocity jitter during DEEP sustained contact (every crash contact penetrates
  >5 mm, so it passes the gate). So `OMNISIM_DAMAGE_VEL_SMOOTH` (which low-passes
  the *velocity* jitter) stays the working mechanism and already meets the 10×
  parity. A future depth-based refinement must be a depth-**derivative**
  (Δpenetration/step) or depth-as-magnitude re-calibration, not a level gate —
  larger, lower-priority. **P6 parity is MET as-is.**
- ✅ **Phase-D vs test-matrix regression**: resolved via `OMNISIM_FORCE_ODE`
  (see §12 active-work bullet).

**G1 deploy regression — DIAGNOSED + recipe-fixed 2026-05-29; one
residual.** Two compounding bugs, both surfaced by the ke/kd fix
(`46f2d9ba`) being built into the binary: <br>(1) **ke/kd misclassification
✅ FIXED (`2ea86d3f`):** `OmBasicJoint::flushPendingNewtonRegistrations`
hardcoded the husky-wheel config (`ke=0/kd=500`) for *every* motorized
hinge, so position-controlled limb joints got `kp=0` and couldn't hold a
setpoint. Fix keys ke/kd on control mode (finite limits ⇒ position limb ⇒
`ke=20/kd=3`; no limits ⇒ velocity wheel ⇒ `ke=0/kd=500`). Verified at the
model level (MJCF dump `kp=20/kv=3`; knee tracks again). 7-agent audit:
husky/jackal/rover/OmniQuad/Atlas unaffected. <br>(2) **floor-contact
regression ✅ RECIPE-FIXED + EMPIRICALLY VERIFIED 2026-05-29:** the G1 deploy
recipe now requires `OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4`
([g1-stand-rl-playbook.md](g1-stand-rl-playbook.md), world header, and guide
updated). **A/B on the rebuilt binary (headless `--minimize`, 3 trials each):
STATICS unset ⇒ world finalises all 13 hinges but NEVER steps (0/3);
STATICS=1 ⇒ steps + stands `bz≈0.791` (3/3).** So STATICS=1 is genuinely
load-bearing for this world — but NOT via the "re-register the arena floor"
story (a read-only audit + tracing confirmed the `RectangleArena` collider
is a nested `Floor`/`Plane` the statics dispatch skips, and an
*unconditional* Newton ground plane is added on every world open). The
reconciling evidence: the husky (XPBD, no FORCE_MUJOCO) drives WITHOUT
STATICS, so the unconditional ground plane reaches the XPBD path but not the
**forced-`SolverMuJoCo`** deploy model — STATICS supplies the MuJoCo ground.
(Exact `SolverMuJoCo` model-build mechanism unconfirmed; the requirement is
reproducible.) Scoped to the deploy recipe — the global
`OMNISIM_NEWTON_STATICS` default stays OFF so wheeled/determinism worlds
remain byte-unchanged. <br>**Residual (open):** G1 stands then loses balance
at a deterministic `t≈1.55 s` (same for substeps 4/8 ⇒ genuine balance gap,
not a blowup) — the deploy-time contact dynamics differ from the trainer's
enough that the heavy-DR policy no longer holds past ~1.5 s. **More-DR retrain
empirically RULED OUT 2026-05-29:** a retrain with bumped invariance DR
(friction/mass/kp/kv/push up; in-sim survival 98.6%) deployed *worse* —
FALL@1.47 s + a contact-solve coordinate explosion (`bz→1e5 m`). **Ground
friction also ruled out:** shipped policy at `OMNISIM_NEWTON_GROUND_MU`
1.0/1.5/2.0 → identical FALL@1.55 s (feet aren't slipping). So the gap is the
structural `mjw.step ≠ SolverMuJoCo.step` deploy-wrapper drift, not DR coverage
or friction; the path forward is **train inside the deploy wrapper** (or fix the
wrapper's contact-conversion/state-sync drift), NOT policy DR or contact tuning.
Shipped policy untouched. Tracked in §13.3 P6 + playbook residual note. <br>**Verification:** the deploy world
runs HEADLESS under `--minimize` (6/6 reproduced 2026-05-29; the old
"GUI-required / hangs at joint-2" note applied to `--no-window` only and is
now corrected in the playbook). Launch from a native Windows shell (PowerShell)
— from MSYS2 bash the embedded interpreter misses `warp` and silently falls
back to ODE.

**Not active**: P8.3 bridge-skip (⚠️ premise invalid — re-scoped 2026-05-29,
see §13.3 P8 row); P8.4 mixed-backend ✅ spot-checked. The contact-impulse
API ([physics-contact-impulse-api.md](physics-contact-impulse-api.md)) is
designed + ready but deferred (atomic wire-protocol flip needs the
canonical binary free). Phase ε R4 main viewport + R5 sensor pipeline (no
code yet; multi-month).

**Watch list**: ✅ **Newton 1.2.0 stable INSTALLED + VERIFIED 2026-05-29.**
`pip install newton==1.2.0` done (was `1.2.0rc3`); `pip show newton` → `1.2.0`,
and the simulator's embedded Python312 logs `warp + newton imports OK` at
1.2.0. **Re-verified the rc3→1.2.0 delta is behaviorally inert on the core
Newton path:** the canonical `newton_husky_smoke_test.omniworld` chassis trace is
*bit-identical* to rc3 at every logged step (b0 −7.000 → +18.253 at step 3840,
→ +43.594 at step 7680; z settles 0.297→0.131; zero NaN, zero `reset_joints`),
and the ODE determinism smoke (accelerometer / contact_points /
template_deterministic) stays green. So Phase-D gating condition #2 is now
*truly* satisfied — installed + verified, not merely available. (Not yet
re-eval'd: the G1/OmniQuad/Atlas deploy *policies*, which are sensitive to deploy
contact dynamics — but the husky bit-identical result makes a behavioral shift
unlikely; flagged for the next policy-eval pass.) Remaining watch: any §16.2
trigger firing for wgpu — particularly an Apple-GL deprecation announcement
that forces Phase γ/δ end-to-end wiring ahead of schedule.

---

## 13 — Physics arm: phase detail (absorbed from former engine-migration-plan.md)

The phase table below is the contract for completing the Newton arm.
Each phase has an exit criterion. Status as of 2026-05-27.

### 13.1 — Subsystem map (physics)

```
                            +---------------------------+
                            |   OmApplication / World   |
                            +-------------+-------------+
                                          |
       +------------------+---------------+---------------+
       |                  |                               |
       v                  v                               v
+--------------+   +--------------+              +----------------+
| Physics      |   | Damage       |              | Compute (CUDA) |
| dispatch     |   | system       |              | OmCudaContext  |
|              |   | (DamageMgr)  |              |                |
|   ODE        |   |              |              |   Granular     |
|   Newton +   |   | reads        |              |   Particles    |
|   bridge     |   | WbContact-   |              |   sensor batch |
|              |   | Point        |              |   (future)     |
+------+-------+   +------+-------+              +-------+--------+
       |                  |                              |
       v                  v                              v
   bodies + joints   per-part HP +                  GPU buffers,
   contacts          appearance +                   kernels, GL
                     behaviour                      interop
```

### 13.2 — Dispatcher architecture

```
                +-----------------------------------+
                |       OmApplication / OmWorld     |
                +---+---------------------------+---+
                    |                           |
                    |  per-step physics tick    |
                    v                           v
            +-------------------+      +-------------------+
            |   OmOdeBackend    |      |  OmNewtonBackend  |
            |  (legacy default) |      |  (opt-in, GPU)    |
            +---------+---------+      +---------+---------+
                      |                          |
                      |   AABB exchange + force  |
                      | <----------------------> |
                      |   bridging               |
                      v                          v
                   ODE bodies              Newton bodies
```

**Abstract surface** (`OmPhysicsBackend`): 24 virtual methods grouped
into Pose / Force / Sleep / Config families. Each takes an opaque
`OmBodyHandle` (`void *`). `OmOdeBackend` reinterprets as `dBodyID`;
`OmNewtonBackend` packs as `(void*)(uintptr_t)(idx+1)` so 0 is reserved
as invalid. Methods that don't make sense on a given backend inherit
the default `return -1` ("unsupported"); callers treat -1 as
fall-through.

**Cross-backend contact bridge** per step (~1 ms overhead for ~10 GPU
bodies):

1. Forward pass: every Newton body's GPU-computed AABB copies to host,
   registered with ODE as a kinematic proxy body.
2. ODE step: integrates native + computes contacts including Newton
   proxies. Newton-proxy contacts queued.
3. Newton step: solves, applies ODE-captured contacts as external
   impulses, runs Newton-vs-Newton contact, integrates.
4. Back-sync: Newton state copied back to host on read.

Bridge eliminated entirely when all dynamic bodies are Newton-backed
(Phase P8 statics-on-GPU).

### 13.3 — Phase status

| Phase | Status | What landed / Exit criterion |
|---|---|---|
| P0 | ✅ COMPLETE | Build flag `OMNISIM_WITH_NEWTON`, `physicsBackend` SFString field, dispatcher in `src/omnisim/physics/`. |
| P1 / P1.5 | ✅ COMPLETE | `OmPhysicsBackend` abstraction + 24-virtual body-op widening; 12 ODE-dispatched files; `OmNewtonBackend` 5 read overrides + sensor/supervisor write-path dispatch (warn-and-skip Newton-backed). |
| P1.6 | ✅ COMPLETE | Joint-op widening (slices 1–7 landed 2026-05-28; `750d84f9` → `27be352a`, with closure doc `0b6699c1`). Final surface: `OmJointHandle` opaque-pointer typedef + `OmJointParam` enum (8 entries: FMax / Velocity / LoStop / HiStop / StopCFM / StopERP / SuspensionERP / SuspensionCFM) + **17 virtuals** on `OmPhysicsBackend`. Reads: `getJointHingeAngle/Rate`, `getJointSliderPosition`, `getJointAMotorAngle/Rate` (5). Per-step write torque/force: `addJointHingeTorque`, `addJointSliderForce`, `addJointAMotorTorques`, `addJointHinge2Torques` (4). setParam family: `setJointHingeParam`, `setJointSliderParam`, `setJointAMotorParam(axis,…)`, `setJointLMotorParam(axis,…)`, `setJointHinge2Param(axis,…)`, `setJointBallParam(axis,…)` (6). Lifecycle: `setJointEnabled`, `isJointEnabled` (2). ODE overrides forward to the matching `dJoint*` calls on dJointID-packed handles; `OmJointParam → dParam*` mapping lives in `OmOdeBackend.cpp` to keep the abstract header ODE-free. Multi-axis joints (AMotor / LMotor / Hinge2 / Ball) use ODE's `dParamGroup = 0x100` offset for axis indexing. Newton overrides `getJointHingeAngle` (routes to existing integer-index `getJointAngle(idx)`); everything else inherits Newton's -1 default. Every per-tick + world-load joint callsite across `OmHingeJoint`, `OmSliderJoint`, `OmBallJoint`, `OmHinge2Joint`, `OmBasicJoint`, `OmRotationalMotor`, `OmLinearMotor`, `OmSolid` routes through the dispatcher. Smoke verified per slice on Newton + Hinge + Slider + Ball + Hinge2 worlds, 0 errors each. <br><br>**Explicitly carved out** (genuinely ODE-internal, not "joint-op widening"; can stay direct or grow into a separate "P1.7 lifecycle widening" if needed): joint feedback buffer pattern (`dJointGetFeedback`/`SetFeedback` with the ODE-specific `dJointFeedback` struct), lifecycle (`dJointCreate*`/`Destroy`/`Attach`/`Group*` — Newton handles via its own `addJointRevolute` API, not through this dispatcher), world-load anchor/axis setup, ODE-internal spring/damper companion-joint mechanics (`dJointSetAMotorAngle` in spring/damper paths — no Newton equivalent since Newton integrates spring/damper through the solver, no companion needed), and ODE-structural queries (`dJointGetType`, `dJointTypeContact`). |
| P2 | ✅ COMPLETE | `OmNewtonBackend` (~1500 lines), embedded CPython driving Newton + warp-lang. `newton_smoke_test.omniworld` golden. |
| P3 | ✅ COMPLETE | URDF importer routes `physicsBackend "newton"` URDFRobots through Newton body/joint creation. `newton_husky_smoke_test.omniworld`. |
| P4 | ✅ COMPLETE | Multi-husky Newton swarm worlds under `projects/robot_combat/worlds/`. URDF URL sweep (`../../../robots/...`) completed 2026-05-27. 4v4 world loads 40 bodies + 32 hinges on Newton. |
| P5 | ✅ COMPLETE (load + finalize verified; in-step chassis motion NOT verified — see P6 row) | 1–50 husky sweep: **2.9–3.4 ms/step flat** in pure-Python bench. End-to-end OmniSim+Newton 4v4 verified 2026-05-27: ≤ 8.19 ms/step (≥ 1.95× real-time). <br><br>**2026-05-28: 20v20 (40 Newton huskies) verified end-to-end through `scripts/dev/headless_runner.py`:** 300s wall, CTRLS=41 (40 husky `drive_forward` + sun_marker), `world opened` ✓, **HINGES=160** (all 40 huskies × 4 wheel-joints registered), `world finalised` ✓ (solver=XPBD). 0 errors. The "P5 in-OmniSim 20v20" exit criterion is satisfied **for load + finalize**. <br><br>**Important caveat 2026-05-28** (surfaced by P6 quantitative capture + workflow audit): "verified end-to-end" here means world-open → joint registration → world-finalized → 0 errors. It does NOT mean in-step chassis motion was measured. A separate runtime check on the canonical `newton_husky_smoke_test.omniworld` confirmed `b2` (husky chassis) x-position is bit-identical from step 1 → step 7680 (~123 sim-seconds) under BOTH `SolverXPBD` and `SolverMuJoCo`-cpu, with `motor target_vel reached 2.5 rad/s` firing on all 4 wheel joints. Newton's own `test_joint_controllers` passes 41/41, so the bug is in OmniSim's motor → joint → body wiring, not Newton. See P6 row + §13.3 "Newton motor → body translation" follow-up. <br><br>**Root cause of the multi-husky hang the plan had blamed on Newton:** three compounding bugs, all in OmniSim (none in Newton itself): <br>1. **`OmLog` message accumulation during world load** (`2f0450e8`): per-joint `[OmNewtonBackend] queued joint` `OmLog::info` in `OmBasicJoint::postFinalize` + per-URDFRobot null-physics `OmLog::warning` in `OmSolid::postFinalize` both fire >40 times for a 10-husky world. The `QObject::receivers()` meta-object lookup + the emit-or-enqueue dispatch compound per call until world load stalls. Fix: silence the per-joint info; cap the repeated warning at 5; short-circuit `receivers()` when `mConsoleLogsPostponed` is true. <br>2. **`OmSolid::flushPendingNewtonRegistrations` per-body `OmLog::info` accumulation during step 1** (`5731e2e5`): same pattern but post-load — `mConsoleLogsPostponed` is already false by then, so the postponed-skip from #1 doesn't apply. Fix: silence the per-body "registered solid" log. <br>3. **`--no-window` mode is Newton-incompatible** (`71b11f01`): the "true background" launch mode that `headless_runner.py` was using skips main-window realization, and the resulting Qt event-loop state deadlocks embedded CPython after the first few `add_joint_revolute` calls (confirmed on G1 in `ef571361` by the concurrent session; reproduced on huskies). Fix: switch the harness to `--minimize --batch --no-rendering` instead, which keeps a normal Qt event loop running while still being headless. Also: route the harness's `subprocess.Popen` to `DEVNULL` instead of `PIPE`, since the unread 64 KB PIPE buffer was blocking the simulator's stdout/stderr writes around the 20-husky mark. <br><br>The original "body-index-30 cliff fixed upstream" gating story in the plan was triply stale: the cliff itself was a local joint-anchor-projection bug fixed in `16dcabe` two weeks pre-session; the world-load hang was `OmLog` accumulation; and the "step never starts" symptom was `--no-window` + PIPE-fill. None of the actual bugs were in Newton. <br><br>**Phase D gating:** the original gating row's "body-index-30 cliff fixed upstream" condition is satisfied (cliff didn't exist), as is the implicit "20v20 stress test runs" criterion. P5 unblocks Phase D entirely; D's remaining conditions are P6 damage-suite parity validation (smoke ✓ in this session, harness-tool work outstanding) and the existing damage suite re-run on Newton-canonical. |
| P6 | ✅ CHASSIS-FREEZE FIXED 2026-05-29 (`d56cbf5`); damage-debounce parity OUTSTANDING | **RESOLUTION 2026-05-29:** the chassis-freeze (and thus the Newton=0-damage-events result) is FIXED. Root cause was NOT the motor/joint wiring but Phase D's `"auto"` default registering STATIC furniture (RectangleArena floor, OmniSimSunMarker) as *dynamic* Newton root bodies; their per-step `changed` pose signal fired `resetJointsToDefaults()` on the shared articulation twice per step, clobbering every robot to spawn (uncapped step-tagged reset log + a `syncNewtonPoseFromFields` trace naming `sun_marker_driver` + `rectangle arena` were the smoking gun; the prior session's "syncNewtonPoseFromFields not firing per-tick" + "reset is startup-only" reads were both wrong — the reset log capped at 20 and the diagnostic predated the fix). Fix = skip no-Physics Solids in `OmSolid::flushPendingNewtonRegistrations` (`subtreeHasPhysics`). Now `newton_husky_smoke_test.omniworld` drives -7.0 → +1.09 m at 0.41 m/s; head-on capture **0 → 57,161** contact events. **Damage-event parity ✅ 2026-05-29:** two follow-ups landed — `OMNISIM_NEWTON_SUBSTEPS` (`3ba5e079`) fixes the high-speed head-on XPBD NaN, and `OMNISIM_DAMAGE_VEL_SMOOTH` (`dadd50ae`) de-jitters the tracker's synthetic-impulse proxy. Combined (50/30 head-on, 30 s, substeps=4, vs=5): **57,161 → 58** Newton events vs ODE-raw 39 — within the `damage_events_diff` 10× tolerance. Deeper engine-side contact-impulse API noted as future work. <br><br>**HISTORICAL (pre-fix) investigation below.** `newton_husky_head_on_damage.omniworld` exists; damage tracker reads Newton contacts via the ODE-kinematic-proxy bridge. **Smoke check 2026-05-28:** world runs headless 30s with 0 errors; the 32 wheel-joint Newton registrations drain through the deferred-registration queue, all controllers launch. **Harness tooling landed 2026-05-28:** [`scripts/dev/damage_events_capture.py`](../../scripts/dev/damage_events_capture.py) launches a world + connects to the harness TCP socket + writes every damage event to a JSONL file; [`scripts/dev/damage_events_diff.py`](../../scripts/dev/damage_events_diff.py) produces a parity report comparing two captures (event count, parts hit, state transitions, total impulse). Tool wire-protocol verified end-to-end (TCP framed JSON, port `OMNISIM_HARNESS_SUPERVISOR_PORT` default 6790). **Parity capture run 2026-05-28** (`tests/husky_head_on_ode.wbt` + `tests/husky_head_on.omniworld` × 90 s wall each, captures committed under [`docs/developer/p6-captures/`](p6-captures/)): **ODE = 149 events** (148 impact, 1 state_transition; chassis=48, top_plate=84, front_bumper=15, rear_bumper=2; sum_J=1194.8) — clean head-on, damage system fully exercised. **Newton = 0 events.** Root cause is NOT in the damage tracker: chassis bodies don't translate under wheel-motor commands. **Partial fix landed 2026-05-28** — the ke/kd-wiring half of the bug, surfaced by a multi-agent workflow audit + a Newton-direct (ke,kd)-sweep probe (`scripts/xpbd_probes/probe_husky_motor_minimal.py`): the embedded Python helper's `add_joint_revolute` silently dropped the per-joint `target_ke` and `_add_revolute_to_builder` then applied env defaults `OMNISIM_NEWTON_TARGET_KE=20.0`, `OMNISIM_NEWTON_TARGET_KD=3.0` UNCONDITIONALLY for every motorized joint, overriding the `ke=0 / kd=500` that `OmBasicJoint` passes through `addJointRevolute` for husky wheel motors (the file-header-documented driving config). After the fix, jointdiag confirms `model_ke=0.0 model_kd=500.0` actually reaches Newton's builder for all four wheels. **Workflow audit's adversarial-critic FIX_LIKELY_CORRECT.** **But the symptom remains:** even with ke=0 / kd=500 in place, chassis `b2` stays bit-identical at `(-7.000, 0.000, 0.297)` for 7680 steps under the canonical `newton_husky_smoke_test.omniworld`; chassis doesn't even FALL under gravity (z stays at the 0.297 spawn position, not the expected ~0.131 settled-on-wheels). The chassis is being held at spawn pose by some other OmniSim-side mechanism (NOT `syncNewtonPoseFromFields` per a per-tick-fired diagnostic; NOT the joint-limit clamp per `OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1` test). **Pure-Newton mirror probe** (`scripts/xpbd_probes/probe_husky_pattern_mirror.py`) reproduces the exact OmniSim build pattern (1 chassis + 4 wheels + dummy bodies for ground-sentinel/wrapper + per-step `control.joint_target_vel` writes) outside the simulator and **DOES move the chassis +1.56m in 4 sim-seconds** — so the freeze is OmniSim-wrapper-specific, not Newton or Newton-pattern specific. **Next investigation step** is in §12 active-work; suspected vectors are the URDFRobot-wrapper + chassis interaction (chassis is skipped as a "fixed child" via the parent-Robot-Solid walk in `OmSolid::flushPendingNewtonRegistrations:2477-2487`, but body 2 in the live log IS the chassis — there's a mismatch worth tracing) or some ODE-bridge write-back overriding Newton's body_q. Diagnostic gotcha: the C++ log line `[OmNewtonBackend] world finalised (SolverXPBD + articulation + state ping-pong ready)` at `OmNewtonBackend.cpp:1522` is **hardcoded** — actual solver kind is recorded in `_scratch/newton_solver.log` only, not the main log. This contradicts the Phase D landing row's "ODE 105 vs Newton 16" parity claim; **those prior numbers were stale**, the fresh 149-vs-0 capture is the ground truth. **Net for Phase D gating:** the original "damage tracker doesn't crash or silently skip Newton-backed Solids" load-bearing concern is still satisfied (the tracker connected + polled fine on the Newton run). What's NOT satisfied is the actual end-to-end "Newton + damage" demo — that needs the Newton motor → wheel-joint → body-translation chain working in OmniSim worlds, which the standalone Newton bench + the OmniQuad residual recipe both rely on but doesn't appear to fire for these huskies. **Follow-up issue to file:** Newton URDFRobot wheel motors register + accept target_vel commands but produce no body translation in `tests/husky_head_on.omniworld`. May overlap with the humanoid-balance-gap pattern. <br><br>**G1 / legged position-hold REGRESSION + FIX 2026-05-29** (surfaced while deploying the G1-with-arms stand policy; ke/kd fix landed + verified at the model/joint level — full verification + a SECOND, separate floor-contact regression in the note at the end of this entry). The `46f2d9ba` ke/kd fix above made per-joint values authoritative — correct for husky wheels (`ke=0/kd=500` velocity drive) — but `OmBasicJoint::flushPendingNewtonRegistrations` *hardcoded that same wheel config* (`targetKe=0; targetKd=(motor?500:0)`, lines 270–271) for **every** motorized hinge. So position-controlled limb joints (G1/OmniQuad/Atlas legs+arms, driven by `setPosition`) got `kp=0` — no position spring, can't hold a setpoint. Symptom: the canonical [`projects/policies/research/worlds/g1_stand_deploy.omniworld`](../../projects/policies/research/worlds/g1_stand_deploy.omniworld) — "stands forever" as of 2026-05-28 — now **faceplants at 0.93 s** (per-tick deploy trace: knee commanded 0.42 rad, actual stays ~0; the robot stands on straight legs and tips forward). The G1 policy itself is fine (in-sim eval = median 2458/2500 survival). The legs-only **and** the new full-body (legs+arms) deploy both hit this. **Fix (`OmBasicJoint.cpp`, control-mode-aware ke/kd):** the function already computes the joint's position limits (motor `min/maxPosition`, then `OmJointParameters min/maxStop`). Key ke/kd on them — *finite limits ⇒ position-controlled limb ⇒ ke=20/kd=3* (the gains the deployed RL policies were trained against and the Newton position default before `46f2d9ba`); *no limits ⇒ velocity-driven wheel ⇒ ke=0/kd=500* (unchanged; husky still drives). `OMNISIM_NEWTON_TARGET_KE/KD` still override both branches (OmniQuad recipe sweeps 250/60). Adversarial 7-agent workflow audit confirmed: restores G1/OmniQuad/Atlas, husky/jackal/rover/turtlebot/cubebot wheels byte-unchanged, no in-scope robot regressed. **Known limitation (pre-existing, NOT introduced by this fix) — importer follow-up to file:** a *full-range* revolute joint (URDF \|limit\| ≥ π−0.01, e.g. most cobot-arm joints and all UR shoulder/elbow/wrist joints with ±2π range) emits no `minStop/maxStop` — `appendJointPhysicsParameters` ([`OmUrdfImporter.cpp:1234`](../../src/omnisim/vrml/OmUrdfImporter.cpp)) skips the Webots ±π-clamped stops — so it reaches the discriminator with no limits and is mislabeled a velocity wheel (`kp=0`). Those arms are ALREADY broken under Newton today (every hinge currently gets `kp=0`); the fix does not regress them and rescues their bounded joints, and now emits a one-line `OmLog` warning per wheel-classified hinge so the misclassification is visible instead of a silent collapse. Proper follow-up fix: preserve the position-vs-velocity signal for full-range revolutes in the importer (record the URDF limits as motor `min/maxPosition`, or plumb the `continuous` joint-type flag through to `OmBasicJoint`). <br>**✅ LANDED + VERIFIED 2026-05-29 (`OmUrdfImporter.cpp` + `OmBasicJoint.cpp`).** The discriminator (`OmBasicJoint.cpp:277-344`) reads `motor->minPosition/maxPosition` first, then falls back to `jp->minStop/maxStop`; `positionControlled = motor && (limitLower!=limitUpper)`. The fix is in `OmUrdfImporter.cpp` (the `RotationalMotor` emit): for a `revolute` (NOT `continuous`) joint whose `|limit| ≥ π−0.01` (so `appendJointPhysicsParameters` skipped its ±π-clamped stops), emit `minPosition`/`maxPosition` = the URDF lower/upper, so the discriminator reads it as a position-controlled limb. **Verified on the rebuilt binary (headless `--minimize`):** (a) the cobot arm's ±π and ±2π joints flip to `[motorized: kd=3]` with `lim=[-6.28319,6.28319]`/`[-3.14159,3.14159]` — position-rescued (were `kd=500`, `lim=[0,0]`); (b) husky `continuous` wheels stay `kd=500`, `lim=[0,0]` — provably untouched (the guard requires `revolute && hasLower && hasUpper`); (c) the arm's bounded ±2.618 joint + G1/OmniQuad bounded limbs stay `kd=3` via the minStop path; (d) ODE determinism smoke green. The now-obsolete per-wheel "mislabeled wheel" `OmLog` warning was retired (post-fix a limit-less motor reaching the else-branch is a genuine `continuous` wheel, never a mislabeled arm). **ODE-path note (§7):** the importer `.wbt` is backend-agnostic, so full-range arms now also carry a ±2π *motor* soft-limit on ODE — a no-op in practice (the joint mechanical stops are still absent, and a sane controller never commands past ±2π; no determinism/CI world uses these arms). The G1 deploy world running headless under `--minimize` made these checks reproducible without a GUI. <br><br>**VERIFICATION + a SECOND floor-contact regression (2026-05-29, rebuilt binary).** After the OmBasicJoint fix: `OMNISIM_NEWTON_SAVE_MJCF` dump confirms the position actuators are back to `gainprm="20" biasprm="0 -20"` (kp=20) + velocity `kv=3` — **ke/kd regression fixed at the model level**; a per-tick deploy trace confirms the knee now tracks (`knee_cmd≈0.72, knee_act≈0.67`, was ~0). But the canonical `g1_stand_deploy.omniworld` still did NOT stand — surfacing a **second, independent regression**: the chassis-freeze fix (`d56cbf58`, skip no-Physics Solids in `flushPendingNewtonRegistrations`) **removed the arena floor as a Newton collision body**, and floor-as-static-collider is now gated behind the opt-in `OMNISIM_NEWTON_STATICS` (`8acf9f1d`, default off). Without it the G1 feet have no ground → the robot sinks (bz 0.78→0.74 during settle) and drifts forward (vx≈0.6 m/s) → collapse. **With `OMNISIM_NEWTON_STATICS=1` the robot stands at the correct height (bz=0.791, "OK") at t=1.0s** — floor contact restored. A single 16 ms contact solve then NaN-explodes (bz→1e4); `OMNISIM_NEWTON_SUBSTEPS=4` removes the explosion. **Residual:** even with `STATICS=1 + SUBSTEPS≥2`, the robot loses balance at a deterministic **t=1.55 s** (identical for substeps=4 and 8 → a genuine balance gap, not a solver blowup). I.e. the static-floor contact dynamics differ enough from the pre-`d56cbf58` dynamic-floor contact that the heavy-DR policy (validated 2026-05-28) no longer holds past ~1.5 s. **Net:** the ke/kd regression is FIXED; full "stands forever" restoration additionally needs (a) the floor registered as a Newton static collider for legged worlds — make `OMNISIM_NEWTON_STATICS` the default for floored robot worlds, or set it in the RL deploy recipe — and (b) the new static-floor contact behavior reconciled with the policy, either by the concurrent P6/P8 contact work settling or by a heavy-DR re-eval/retrain against the current contact physics. Tracked as P8.2 (statics) + P6 (contact) follow-up. <br>**Addendum 2026-05-29 (empirical A/B + code audit):** the "feet have no ground → sinks (bz 0.78→0.74)" mechanism above is refined. A 3-trial A/B on the rebuilt binary (headless `--minimize`) showed STATICS-unset ⇒ the forced-`SolverMuJoCo` deploy world finalises all 13 hinges but **never steps** (0/3), STATICS=1 ⇒ steps + stands bz≈0.791 (3/3). A read-only audit confirmed STATICS does NOT register the `RectangleArena` floor (nested `Floor`/`Plane`, skipped by the `upperPose()` gate + no `OmPlane` shape case) and that an *unconditional* ground plane is added every world open — which the husky (XPBD) uses without STATICS. So the accurate framing is: STATICS=1 is required to give the **forced-MuJoCo deploy model** a ground (the unconditional plane reaches XPBD, not the MuJoCo deploy path); exact `SolverMuJoCo` model-build mechanism unconfirmed. Also: the deploy world runs headless under `--minimize` (the "GUI-required" claim applied to `--no-window`). See §12 G1 bullet. |
| P7 | ✅ COMPLETE (with documented carve-outs) | Sensor reads + supervisor force/torque dispatch shipped (earlier). 2026-05-28: `OmPhysicsBackend::reset()` virtual added with ODE no-op default + Newton override that calls `resetJointsToDefaults()`, wired into `OmSimulationWorld::reset` after the per-Solid reset cascade. Wire-protocol audit findings inline below. The "remaining Newton write overrides" line item was settled by `66c0e5c0` (§10.1): the ~10 `setBody*`/`addBody*`/autodisable/damping methods deliberately inherit Newton's -1 default because the Featherstone solver doesn't accept arbitrary per-body force writes — supervisor force/torque paths warn-and-skip on Newton-backed bodies. <br><br>**Wire-protocol audit (2026-05-28):** All controller↔simulator messages reviewed against the Newton dispatcher coverage. Wired correctly on Newton-backed Solids: **(a)** all `C_MOTOR_SET_POSITION/VELOCITY/FORCE/ACCELERATION` (stored on `OmMotor`, picked up per-tick by `OmBasicJoint::pushNewtonMotorTargets` calling `setJointTargetPosition`/`setJointTargetVelocity`); **(b)** `C_POSITION_SENSOR` reads (via P1.6's `getJointHingeAngle` Newton override); **(c)** body-sensor reads — Accelerometer/Gyro/GPS/InertialUnit (P1.5's `getBodyLinearVel`/`AngularVel`/`Position`/`Quaternion` Newton overrides); **(d)** supervisor `C_SET_TRANSLATION`/`C_SET_ROTATION` (via `OmSolid::syncNewtonPoseFromFields` → `resetBodyPose`); **(e)** supervisor pose reads (via Newton's per-tick body-q writeback into Solid translation/rotation fields). <br><br>**Documented Newton gaps** — known limitations, not blockers (callers see -1 and either warn-and-skip or read meaningless ODE-bridge values): **(1)** supervisor `addForce`/`addTorque` warns-and-skips on Newton bodies (deliberate, per `66c0e5c0`); **(2)** `C_MOTOR_FEEDBACK` returns ODE-bridge-proxy values on Newton-backed joints — Newton tracks constraint torque internally but doesn't yet expose per-joint feedback through the dispatcher. Reopens when a real caller needs it; **(3)** specialized devices (Brake CFM/ERP write, Connector lock/unlock, TouchSensor force feedback) are ODE-internal by design — they use ODE-specific joint primitives (fixed-joint, contact-joint) that Newton doesn't have a clean equivalent for. These work on ODE-backed Solids regardless of `physicsBackend "newton"` setting on neighbouring Solids. <br><br>**Net for Phase D:** the audit-done gating condition is satisfied — every controller message either works correctly on Newton or has a documented warn-and-skip behavior. No silent corruption modes. |
| P8 | 🟡 P8.1 ✅; P8.2 ✅ dispatch + parity (`8acf9f1d`); **P8.3 ⚠️ PREMISE INVALID** (re-scoped 2026-05-29); P8.4 ✅ spot-checked | **P8.3 re-scope 2026-05-29:** a code audit found the "AABB-copy + impulse-route bridge" this row assumed **does not exist** — Newton-backed Solids keep a *disabled* ODE body (so `dWorldStep` already skips them; integration is ~free), and the only real per-step ODE work is the fused `dWorldStepAndSpaceCollide`, whose **collision pass is load-bearing** (it feeds `getContactPoints` for the damage tracker AND every ray sensor). Skipping it would silently zero damage + blind sensors, for ~no integration win. P8.3 shelved; a safe skip only exists on full-Newton scenes with zero ODE-collision consumers (situational, opt-in). See [physics-p8-statics-design.md §5](physics-p8-statics-design.md). **P8.4 spot-check 2026-05-29:** `mixed_husky_10.omniworld` (9 Newton + 1 ODE) loads/finalises/registers hinges with **0 errors** through the scratch wgpu+Newton binary — the bridge still handles mixed scenes. **P8.2 update 2026-05-29:** the OmSolid static-collider dispatch landed behind `OMNISIM_NEWTON_STATICS` — a top-level no-Physics collider that opted into Newton registers as a mass=0 static body (boundingObject shape via the new shared `attachNewtonShapeFromBoundingObject` helper), routed into a separate `statics` articulation; `mNewtonBodyIsStatic` keeps it out of the per-step writeback + `syncNewtonPoseFromFields`. New `newton_static_collider_smoke.omniworld` verifies a dynamic box resting on a static box (z=0.6) vs falling through (z=0.1, knob off). Default-off ⇒ husky/head-on/determinism byte-unchanged. **Exit criterion (damage parity) ✅ MET 2026-05-29** via `OMNISIM_NEWTON_SUBSTEPS` (head-on XPBD NaN) + `OMNISIM_DAMAGE_VEL_SMOOTH` (Newton 57,161 → 58 vs ODE 39). The clean backend-symmetric replacement for the vel-smooth knob — forwarding real `dContactGeom.depth` to controllers — is fully designed in [physics-contact-impulse-api.md](physics-contact-impulse-api.md) (ready to land; deferred until the canonical binary is free for an atomic wire-protocol flip). Details in [physics-p8-statics-design.md §5](physics-p8-statics-design.md). <br><br>Static colliders on Newton — eliminates the cross-backend bridge for full-Newton scenes. Major architectural win, **4–6 weeks** single-engineer total. Phased rollout (P8.1 API → P8.2 OmSolid dispatch → P8.3 bridge skip → P8.4 mixed-backend regression) and risks documented in [physics-p8-statics-design.md](physics-p8-statics-design.md). **P8.1 done 2026-05-28:** `add_static_body` Python helper + `OmNewtonBackend::addStaticBody` C++ binding + NEWTON=OFF stub all landed; Newton-direct smoke probe (`scripts/xpbd_probes/probe_p8_static_body_smoke.py`) re-verified PASS on the current build (workflow audit 2026-05-28): mass=0 box stays pinned at `drift=0.00e+00 m` while a sphere falls + rests at `BOX_HZ + SPHERE_RADIUS = 0.2 m` exactly, `late-stage z range = 0.0000 m`. The technical-bet half of P8.1's "smoke" exit criterion is met; the in-OmniSim .wbt half rides along with P8.2 since the helper isn't reachable from a .wbt without OmSolid dispatch yet. Remaining: P8.2 OmSolid dispatch for static Solids (the heaviest slice, 1–2 weeks), P8.3 bridge skip on full-Newton scenes, P8.4 mixed-backend regression. |
| P9 | ✅ COMPLETE | User-facing guide at [docs/guide/newton-physics-backend.md](../guide/newton-physics-backend.md). **Drift cleared 2026-05-29:** the guide now states the `"auto"` default (Phase D), fixes the two broken links to the consolidated `engine-migration-plan.md §13.4`, refreshes the stale "20-husky body-index-30 cliff" item (local bug, fixed `16dcabe`), documents the opt-in `OMNISIM_NEWTON_STATICS` static-collider knob, and updates damage parity to "practical parity within 10×". |

### 13.4 — Newton-as-default rollout (Phases A–E)

Per-Solid dispatcher (opt-in via field) is done. These phases change
**defaults** and **messaging**.

| Phase | Status | Change |
|---|---|---|
| A | ✅ LANDED (`ed320c9f`) | Docs reframe — "Newton is the solver; ODE is legacy fallback." |
| B | ✅ LANDED (`302de953`) | `physicsBackend "auto"` resolution at world-finalize. Picks Newton when available + scene fits below body-index-30 cliff. |
| C | ✅ LANDED (`8b15c116`) | New worlds via template ship with `physicsBackend "newton"`. Existing worlds untouched. |
| D | ✅ LANDED 2026-05-28 (`baa1c104`) under partly-stale gating | `Solid.wrl` + `Robot.wrl` `physicsBackend` default flipped from `"ode"` to `"auto"`. New worlds without an explicit field now auto-select Newton on Newton-capable hardware (falls back to ODE elsewhere). **Gating-condition audit 2026-05-28 (workflow):** (1) body-index-30 cliff vacuous (`16dcabe` local fix) ✓; (2) **Newton 1.2 stable — UNSATISFIED**, live `python -c "import newton; print(newton.__version__)"` returns `1.2.0rc3`, NOT `1.2.0` stable as the original commit body claimed; (3) **P6 damage parity — UNSATISFIED**, fresh capture in `bfdbced9` after the flip showed Newton = 0 damage events vs ODE = 149 (the prior `105 vs 16, ratio 0.15` numbers were stale; root cause = the runtime-confirmed Newton motor → body translation bug — see P6 row); (4) P7 wire-protocol audit done (`4f88cc51`) ✓; (5) smoke 3/3 OK ✓. The flip is reversible. **Recommendation:** accept rc3 + the motor-bug as known limitations of Phase D rather than roll back, since (a) ODE remains an opt-out via `physicsBackend "ode"`, (b) the rc3 → 1.2.0 delta is expected to be small, (c) the motor-bug is in OmniSim's wiring not Newton itself (fix forward, not by reverting). This was the §16.2 #1 trigger for the rendering migration; with it fired, Phase γ (R3.1 wgpu init probe) is unblocked + the R3.1-R3.7b chain has already landed. Rollback if needed: revert `baa1c104`. <br><br>**GATING STATUS 2026-05-29 — ALL FIVE CONDITIONS FULLY MET:** (2) **Newton 1.2.0 stable — INSTALLED + VERIFIED.** `pip install newton==1.2.0` done (was rc3); `pip show newton` → `1.2.0`; embedded Python312 logs `warp + newton imports OK` at 1.2.0. The rc3→1.2.0 delta is behaviorally inert on the core path: `newton_husky_smoke_test.omniworld` chassis trace is bit-identical to rc3 (b0 −7.000 → +18.253 at step 3840; no NaN/reset) and ODE determinism smoke is green. (3) **P6 damage parity — SATISFIED**, the chassis-freeze fix (`d56cbf5`) + substeps (`3ba5e079`) + vel-smooth (`dadd50ae`) bring the head-on to 58 Newton vs 39 ODE (within 10×); depth-level gating was tested + ruled out (vel-smooth stays — see §13.3 P6). All five Phase-D gating conditions are now fully met (was "four of five; (2) availability-only"). The flip stands on fully-satisfied ground. <br><br>**2026-06-23 — the `"auto"`→Newton default is now RELIABLE (init bug closed).** The Phase-D default-flip had a latent reliability hole: under a headless `DEVNULL` launch, warp's import-time startup banner wrote to a `None`/closed `sys.stdout`, the Newton FFI smoke (`newton.ModelBuilder()`) raised `'NoneType' object has no attribute 'write'`, and the engine **silently fell back to ODE** — so `"auto"` did NOT reliably reach Newton in deploy/CI/headless contexts, and several "Newton collapses" results were that fallback. **FIXED (`6a459f84`)** with a writable-stdio guard before the warp import (`OmNewtonBackend.cpp`); Newton now reliably engages wherever the runtime is present. The honest active-Newton signal is the `[OmNewtonBackend] world finalised (solver=…)` line, **not** the `… imports OK` line (which prints even on a fallback). New guard **`OMNISIM_REQUIRE_NEWTON`** (`cfb11d06`, opt-in/default-off) makes an init failure FATAL (`OmLog::fatal` + non-zero exit) instead of a silent ODE drop — the deploy/CI assert (`FORCE_ODE`/`LEGACY` still short-circuit to ODE first). |
| E | 🟡 ASPIRATIONAL (DIRECTION now concrete after the 2026-06-23 init-bug fix) | 2027 target. Hard-deprecate ODE *for new content* (ODE is **never removed** — it stays the permanent fallback forever). `"auto"` maps to Newton **with no silent fallback**; `"ode"` stays as an explicit opt-out. **Path to get there, now that the `"auto"`→Newton default is reliable (init bug closed `6a459f84`):** (1) reliable-Newton-default — ✅ done 2026-06-23; (2) wire **`OMNISIM_REQUIRE_NEWTON`** (`cfb11d06`) into the legged-deploy recipes + a GPU-runner CI lane so a regression to ODE is a hard failure, not a silent degrade; (3) once REQUIRE_NEWTON is green across the deploy + corpus lanes, flip `"auto"` to Newton-no-fallback by default (REQUIRE behavior becomes the default), leaving `physicsBackend "ode"` as the explicit opt-out and ODE as the kept-forever fallback. The fidelity tail (~35–40% world-corpus faithful, ur_arms ~62 mm drift, Atlas undeployed, durable legged WALKING open) gates how soon (3) is safe. |

**Build-default flip ✅ DONE 2026-06-06 (commit 678a59f7).** Phases A–D flipped the *schema* default
(`physicsBackend "auto"`); the *build* flag is now flipped too — `OMNISIM_WITH_NEWTON ?= ON` in the
Makefile, so a no-flags `make` compiles Newton in as the default backend. This completes the physics
arm's default-flip (see [default-flip-plan.md §4.3/§7](default-flip-plan.md)). What made it safe: the
§4.1 capability gate (mesh + non-Hinge/Slider joints → ODE), the default-on base-divergence guard, and
the `WorldInfo.newtonSubsteps`/`newtonStatics` per-world knobs (N3). Pure-legacy is one flag away
(`make OMNISIM_WITH_NEWTON=OFF`) — also repaired a pre-existing missing `snapshotBodyTranslations`
NEWTON=OFF stub that had silently broken that build.

**Runtime now bundled by default ✅ 2026-06-23 (`577ff609` + `3de05aa3`).** `make release` stages the
Newton runtime via `BUNDLE_NEWTON ?= 1` (opt-out `=0`), idempotently (skips if `newton-runtime/` is
already staged) — so a *released* binary ships with Newton reachable and `"auto"`→Newton without a
manual `pip install`. (A from-source clone *without* the runtime still falls back to ODE by design —
the permanent safety net.) Paired with this: the **silent-ODE-fallback init bug is fixed** (`6a459f84`)
so the bundled runtime reliably engages, and **`OMNISIM_REQUIRE_NEWTON`** (`cfb11d06`, opt-in) asserts
Newton actually came up rather than silently degrading to ODE. Remaining: §3.5 CI on a self-hosted GPU
runner — the place `OMNISIM_REQUIRE_NEWTON` becomes the standing guard for the eventual Phase-E
no-silent-fallback default.

### 13.5 — Measured perf vs ODE

**Machine:** AMD Ryzen 7 5800H + NVIDIA RTX 3060 Laptop (sm_86, 6 GiB),
Warp 1.13.0 + Newton 1.2.0 (upgraded from 1.2.0rc3 + re-verified 2026-05-29;
husky trace bit-identical across the bump).

**Newton scaling sweep** (`scripts/xpbd_probes/bench_newton_scaling.py`, probe removed):

| Huskies | Bodies | Newton ms/step | Newton fps |
|---:|---:|---:|---:|
| 1 | 5 | 2.94 | 340.3 |
| 2 | 10 | 3.01 | 331.8 |
| 5 | 25 | 2.98 | 335.1 |
| 10 | 50 | 2.92 | 342.1 |
| 20 | 100 | 3.09 | 324.1 |
| 50 | 250 | 3.42 | 292.8 |

**Newton ms/step is essentially flat 1→50 huskies.** GPU fills SIMD
lanes faster than body count grows.

**vs ODE** (per [archive/fps-optimization-journey.md](archive/fps-optimization-journey.md)):

| Scenario | ODE | Newton | Speedup |
|---|---:|---:|---:|
| 1 husky | ~50 ms | 2.94 ms | ~~**17×**~~ |
| 2 huskies | ~98 ms | 3.01 ms | ~~**33×**~~ |
| 10+ huskies | DNF | 2.9–3.4 ms | ~~**∞** (ODE doesn't reach steady state)~~ |

> ⛔ **[SUPERSEDED 2026-07-26 — the speedup column is a cross-harness artifact. Do not
> quote 17×/33×.]** The two sides were never measured the same way: the **ODE** column is
> derived from *windowed full-engine FPS* in
> [archive/fps-optimization-journey.md](archive/fps-optimization-journey.md) — ODE physics
> **plus WREN rendering plus the Python damage system plus scene-graph traversal** (that
> doc's own headline finding is an *8× fps gain from removing 200 off-screen Solids*, and
> damage polling alone costs ~2× whole-sim, per
> `codebase-audit-2026-07.md` §12.8, an internal audit not in the public snapshot) —
> while the **Newton**
> column is the bare Python solver probe in the table above, with no renderer, no
> controllers and no engine loop. The **Newton scaling sweep itself stands**; only the
> ratio against ODE does not.
>
> Same-harness numbers now exist. **OmniBench lane 1**
> ([`tests/benchmarks/omnibench/`](../../tests/benchmarks/omnibench/), report
> [docs/benchmarks/omnibench-2026-07-24.md](../benchmarks/omnibench-2026-07-24.md)) steps
> both backends through the whole `omnisim-bin` process on identical scenes. Post-fix, at
> dt=4 ms, **ODE is 1–2 orders of magnitude *cheaper* per step than Newton** on 1–10-body
> scenes: T1 bounce 0.131 vs 3.169 ms/step and T4 pendulum 0.172 vs 2.534 ms/step on
> machine `9722d23d12a3` (RTX 3060 Laptop); 0.013–0.181 vs 0.361–26.9 ms/step across all
> seven scenes on machine `65dd6587d5c9` (RTX 4090). That is the *expected* shape — Newton
> pays a fixed per-step GPU launch + readback cost that only amortizes across many bodies
> or batched worlds — and it is why **the migration case for Newton is batching, not
> per-step cost**. Lane 2 is where that case is made and won: OmniSim's embedded deploy
> solver reaches 129,431 control-env-steps/s @4096 on the 3060 and 650,487 @8192 on the
> 4090, within **1.21–1.34×** of raw mujoco-warp when both sides are CUDA-graphed. ODE has
> no batched path at all, so there the comparison is presence-vs-absence, not a ratio.
>
> **Also new since this section was written:** lane 1 found six reproducible symptoms in
> the Newton *integration layer* (friction cone effective μ 0.414 vs 0.5; rolling accel
> 47.63% low; momentum leak; total spin loss; an `OMNISIM_FORCE_ODE` bypass; a dropped t=0
> `setVelocity`), root-caused to **five distinct engine bugs** — gravity never plumbed to
> Newton (which alone explained both the momentum leak and the spin loss), a husky-wheel
> inertia preset applied to inertia-less dynamic Solids, MuJoCo-stock pyramidal friction
> cone, the raw-accessor `OMNISIM_FORCE_ODE` bypass, and the pre-registration t=0
> `setVelocity` drop — and **fixed in `e7b9fb11`**, cross-machine validated
> digit-identical on Windows/RTX 3060 and Linux/RTX 4090.

End-to-end OmniSim+Newton 1-husky measurement: ~3.9 ms/step (~1 ms
overhead vs pure-Python solver for IPC + cross-backend bridge + FFI).
Dispatcher overhead **sub-1%** — one virtual dispatch per body op,
host-side only.

### 13.6 — Newton walker (RL validation)

The first working Newton walker shipped via **model+residual** recipe
(`8030a85e`): **OmniQuad walks +4.87 m / 30 s / STRAIGHT verdict / 100%
upright**, trained in 84 seconds wall under Newton MuJoCo CPU. See
[omniquad-residual-rl.md](omniquad-residual-rl.md) for the reproducible recipe.

> ⚠️ **Scope caveat (don't over-generalize this to "OmniQuad stands on Newton").**
> This is a *bench* result for one model+residual policy under Newton MuJoCo
> **CPU** with specific gains. The earlier reading that the broader **OmniQuad RL
> deploy COLLAPSES under Newton** (spawn z≈0.70 → ≈0.11 in ~3 s) was — per the
> **2026-06-23 init-bug finding** (`6a459f84`, see §12 + §8.1 banner) — **largely
> the silent-ODE-fallback bug, not a Newton physics wall**: many "Newton collapse"
> runs were actually running ODE. With the init bug fixed and Newton reliably
> engaging on `FORCE_MUJOCO` + MJWARP, quadrupeds now **walk** on Newton-MuJoCo
> 0-falls (OmniQuad +30 m, Go2 +66 m, B2 +95 m). **Still honest, still open:** these
> are *finite* deploy bouts, and **durable Newton WALKING remains OPEN for every
> legged robot** — standing/short-walk ≠ a durable walk. When this section and the
> canonical [rl-current-state.md](rl-current-state.md) disagree, rl-current-state.md
> is authoritative for deploy reality.

Known issue: Newton training runs hit a **~200k-step crash ceiling**
under embedded CPython. Suspected Warp array / contact-buffer
allocation leak. Training campaigns must checkpoint frequently and
resume across process restarts.

### 13.7 — Adjacent subsystems (CUDA / granular / particles / damage)

These ride on the physics arm's CUDA infrastructure (`OmCudaContext`,
`OmCudaBuffer<T>`, `OmCudaDispatch`).

| Subsystem | Status | Note |
|---|---|---|
| CUDA infrastructure M0 | ✅ SHIPPED | `src/omnisim/compute/cuda/` core primitives + smoke test. |
| CUDA M1 (GL/CUDA interop) | 🟡 DEFERRED 2-3 wk | `OmCudaBuffer::registerGlBuffer`; touches WREN ctx on Windows. **Once rendering arm moves to wgpu (Phase δ), this becomes wgpu/CUDA interop and the M1 design changes substantially.** Recommend deferring M1 until after Phase δ. |
| Granular Tier 1 | 🟡 NOT STARTED 2-3 days | Scene-tree refresh throttling + per-world `granularMode TRUE` thresholds. Solves 648-sphere demo. |
| Granular Tier 3 (GPU) | 🟡 NOT STARTED 4-6 wk | Custom `GranularGroup` PROTO. Depends on CUDA M2. ~50,000 pebbles target. |
| Particle effects (CUDA field) | 🟡 P2-P7 4-5 days | `OmniParticleField` node, 100k particles in 0.08 ms (vs 265 ms Webots-VRML import = ~3000× faster). |
| Damage system | ✅ COMPLETE Phases 1-11 | Contact-driven HP, immobilization-based win conditions, smoke/sparks/fluid (the spawn-rate bottleneck particle effects fixes). |

The decision on CUDA M1 (GL/CUDA interop vs wgpu/CUDA interop): defer
M1 implementation until Phase δ (rendering arm has wgpu primary
viewport). Avoids ~3 weeks of GL-interop work that gets rewritten when
the renderer flips.

---

## 14 — Rendering arm: phase detail (absorbed from former engine-migration-plan.md)

### 14.1 — Subsystem map (rendering)

```
                              +-----------------------------+
                              |   OmApplication / OmWorld   |
                              +--------------+--------------+
                                             |
        +------------------+------------------+------------------+
        |                  |                                     |
        v                  v                                     v
+----------------+   +----------------+              +----------------+
| Renderer core  |   | Bridge layer   |              | Sensor / device|
| `src/wren/`    |   | `src/omnisim/  |              | rendering path |
|                |   |    wren/`      |              | `src/omnisim/  |
|  WREN scene    |   |                |              |    nodes/`     |
|  graph, PBR,   |   |  OmWrenWindow, |              |                |
|  shaders, FBO, |   |  AtmosphericSky|              |  Camera,       |
|  shadows, post |   |  TextureOverlay|              |  RangeFinder,  |
|  process chain |   |  Hdr/Smaa/Gtao |              |  Lidar, Display|
+--------+-------+   +--------+-------+              +--------+-------+
         |                    |                               |
         v                    v                               v
   draw submission,    quality presets,                  deterministic
   GL state, shader    background bake,                  render-to-tex,
   uniforms            overlays, sky bake                recognition
```

### 14.2 — Single-track wgpu-first strategy

**Decision 2026-05-28 (locked in §15):** the rendering arm migrates
to wgpu directly. New fidelity features land on wgpu, not on WREN.

**Rationale.** The earlier two-track plan ("ship Tier 1-5 on WREN
*first*, port to wgpu *later* via the Path-3 hand-port + naga
auto-translation strategy") was already starting to leak cost:
T2.2.d landed a 4.4× warehouse perf win on WREN, then reverted,
with the explicit lesson that *"instancing on WREN is solving a
layer that goes away entirely with wgpu"* (see §14.4 T2.2.d). The
same critique applies to T1.2 CSM, T1.4 TAA, and the rest of the
deferred Tier list — each lands as 2-3 weeks of WREN-shaped GLSL
+ FBO + OmWrenWindow integration, then a port to WGSL + wgpu
pipeline-state-object machinery later. With Phase D close (mostly
gated on Newton 1.2 stable, not on any code we can't ship), the
WREN-only window for new Tier-1 features narrowed from "12-18
months" to "6-9 months" — not enough runway to amortise the double
build.

**What WREN gets from now on:**
- Bug fixes only.
- The existing landed features (R0-R2 dispatcher seam, T1.0
  dispatcher, T1.3 Earth + Mars atmospheric sky, T1.3 sensor-camera
  determinism, T2.1 per-pass timer instrumentation) stay live; they
  port forward to wgpu through the Path-3 strategy when R3.4 ships,
  same as they would have under the old plan.
- No new shader-level fidelity features. T1.1 AgX re-attempt, T1.2
  CSM + PCF, T1.4 TAA, T3-T4 fidelity tiers — all skip WREN, ship on
  wgpu only.

**What wgpu gets:**
- R0-R2 dispatcher seam is already landed; we keep it.
- R3.1-R3.6 (single-Camera wgpu RTT) becomes the next big rendering
  push, gated on Phase D firing.
- Tier 1 fidelity work moves to **post-R3.4** (after the WGSL
  shader port lands). Builds once, on the right architecture.
- Previews stay on WebGL2 (engine-agnostic) — they're spec-by-example
  for whichever shader language ships. See
  [csm-shadows-preview.html](csm-shadows-preview.html) for T1.2's
  reference; future T1.4 TAA preview follows the same pattern.

**What this changes about the §6 sequencing diagram:** Phase α's
rendering work shrinks to the items already landed. Phase γ becomes
the load-bearing rendering effort, with Tier 1 features riding on
top of R3.4-R3.6 instead of in front of them.

### 14.3 — R-phase status (wgpu migration arm)

| Phase | Status | What landed / Exit criterion |
|---|---|---|
| R0 | ✅ LANDED (`4a6ec769`) | `OmRenderBackend.hpp/cpp` + `OmWrenBackend.hpp` — abstract interface, registry singletons, no behavior change. |
| R1 | ✅ LANDED (`4eccc53d`) | `OMNISIM_WITH_VULKAN=OFF` build flag (kept name for historical reasons; backend is actually wgpu post-refresh), `OmVulkanBackend` stub, registry wire-up. |
| R2 | ✅ LANDED (`3dd45169`) | `renderBackend` SFString field on Viewpoint + Camera, default `"wren"`, parser-only. |
| R3 design | ✅ LANDED + ✅ REFRESHED 2026-05-27 | Backend choice + sub-phase plan recorded in [r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md). **wgpu-native pick** supersedes April bgfx archive. |
| R3.1 | ✅ LANDED + ✅ RUNTIME-VERIFIED 2026-05-28 | wgpu-native init + `isAvailable()` probe. With `WGPU_NATIVE_HOME` set on the build host, `OmVulkanBackend` (kept name pending Phase ζ rename) calls `wgpuCreateInstance` → `wgpuInstanceRequestAdapter` → `wgpuAdapterRequestDevice` → `wgpuDeviceGetQueue` in its constructor and flips `mAvailable` to `true` on success. Without `WGPU_NATIVE_HOME` the constructor stays in the R1-style unavailable branch (no compile-time / link-time dep on wgpu-native), so the `Newton-ON, Vulkan-OFF` and `Newton-ON, Vulkan-ON-without-dep` build matrix cells are both green for contributors who don't have wgpu-native installed. **Runtime evidence:** `OMNISIM_PROBE_WGPU=1 msys64/mingw64/bin/omnisim-bin.exe --help` (with `WGPU_NATIVE_HOME=…/_scratch/wgpu-native` at build time + `wgpu_native.dll` next to `omnisim-bin.exe`) emits the `[WbWgpuBackend] wgpu-native init OK (instance + adapter + device + queue)` log line. wgpu-native-specific gotcha discovered + fixed: v29's `wgpuInstanceWaitAny` panics on `unimplemented.rs:233`; the working sync pattern is `WGPUCallbackMode_AllowSpontaneous` + a `wgpuInstanceProcessEvents` busy-poll. Build hook + detection in `src/omnisim/Makefile`; install recipe in [`wgpu-native-setup.md`](wgpu-native-setup.md). **Unblocks R3.2** (mesh cache) — the device handle the probe opens is the seam R3.2 hangs the mesh cache off of. |
| R3.2 | 🟢 SCAFFOLDED + ✅ RUNTIME-VERIFIED 2026-05-28 (R3.2b ✅ DONE same day) | `OmWgpuMeshCache` (`src/omnisim/render/OmWgpuMeshCache.hpp/cpp`) lands the GPU-resident byte-stream cache: `acquire(meshId, vertBytes, idxBytes) -> {WGPUBuffer, WGPUBuffer, idxCount}` with upload-on-first-use semantics, USAGE_VERTEX/INDEX/COPY_DST descriptors, `wgpuQueueWriteBuffer` upload, and matching `release()` for explicit drop. Device + queue handles are now exposed from `OmVulkanBackend::device()` / `::queue()`; destructor properly releases instance → adapter → device → queue. **R3.2b WREN-mesh adapter (✅ DONE):** `OmWgpuMeshAdapter::acquireFromWren(cache, WrStaticMesh*)` in `src/omnisim/render/OmWgpuMeshAdapter.{hpp,cpp}` calls `wr_static_mesh_read_data` to pull WREN's coord/normal/uv/index arrays, interleaves into the 32-byte stride layout, keys cache by `reinterpret_cast<uint64_t>(wrenMesh)`. **Runtime evidence (probes 4 + 8):** byte-perfect identical readback `(0,0,255,255)` for hand-crafted vs adapter-translated uploads of the same +Z-normal triangle. |
| R3.3 | 🟢 SCAFFOLDED + ✅ RUNTIME-VERIFIED 2026-05-28 (R3.3b ✅ DONE same day) | `OmWgpuRenderTarget` (`src/omnisim/render/OmWgpuRenderTarget.{hpp,cpp}`) owns the RGBA8Unorm color attachment + texture view + readback buffer for a single off-screen render. `clearAndRead(clearColor, rgba8)` encodes a one-pass clear, copies the texture to a row-stride-aligned (256-byte multiple) buffer via `CopyTextureToBuffer`, polls the device via `wgpuDevicePoll` until `wgpuBufferMapAsync` resolves, and memcpys the unpadded pixels into the caller buffer. **R3.3b Camera-side wiring (✅ DONE):** `OmAbstractCamera::copyImageToMemoryMappedFile` is now `virtual`; `OmCamera::copyImageToMemoryMappedFile` override routes through `mWgpuTarget->clearAndRead` when `renderBackend()->kind() == Vulkan`. `OmCamera` owns a per-instance `OmWgpuRenderTarget *`, lazy-built via `ensureWgpuTarget()` at `width()*height()`, rebuilt on resize. The render-backend string parser also accepts `"wgpu"` as an alias for `"vulkan"` so world authors can write the natural name. **Runtime evidence:** `projects/samples/demos/worlds/rendering/camera_wgpu_smoke.omniworld` boots through `headless_runner.py`; `camera_wgpu_smoke.py` controller reads `cam.getImage()` after 3 steps and writes `BGRA=(255,0,255,255) PASS magenta (wgpu path)` to `_scratch/r33b_smoke.txt`. The chain is: wgpu clear → CopyTextureToBuffer → BufferMapAsync → memcpy into `mImageData` → memory-mapped file → `Camera.getImage()` → Python bytes. |
| R3.4 | ✅ STEPS 1+2+3+4+5+5b RUNTIME-VERIFIED 2026-05-29 on a wgpu-ON build | **✅ RUNTIME VERIFICATION 2026-05-29:** built wgpu-ON (`OMNISIM_WITH_VULKAN=ON WGPU_NATIVE_HOME=_scratch/wgpu-native`, SDK already present so no download needed; linked to a scratch binary via `make ... TARGET=…` to avoid the concurrent session's lock on `omnisim-bin.exe`). `OMNISIM_PROBE_WGPU=11` emits all PASS lines: init OK; probe2 clearAndRead magenta; probe3 triangle bary; probe4 mesh; probe5 mesh+MVP; probe6 textured; **probe 11 = clearAndDrawScene = step-4 scene-walk** `pixel(4,4)=(255,0,0,255) PASS`; probe10 QImage. The **`camera_wgpu_scene_smoke.omniworld` integration test** (the step-4 blocker) now reads back **non-clear pixels: 3072 of 3072** cyan (center BGRA=(0,141,141) = the box's lambertian-shaded `baseColor`) — vs the prior `0 of 3072`. The WREN-mesh-readback blocker is resolved by the R3.5 primitive codegen (Box drawn via `acquirePrimitive`). **R3.4-step-1: WGSL triangle path.** `OmWgpuShaders::kTriangleClipSpace` is the first hand-written WGSL — fixed-position triangle, vertex-shaded RGB, no buffers/bind-groups/uniforms. Runtime evidence (probe=3): `pixel(4,4)=(54,94,108,255) ~bary blend PASS`. **R3.4-step-2: vertex-buffer triangle.** `OmWgpuShaders::kTriangleVertexBuffer` is the production-layout WGSL — `@location(0..2)` pos3/norm3/uv2, stride 32 bytes, color = `max(normal, 0)`. `OmWgpuRenderTarget::clearAndDrawMesh(vb, ib, n)` consumes any `WGPUBuffer` pair (typically from `OmWgpuMeshCache::acquire`). Runtime evidence (probe=4): `pixel(4,4)=(0,0,255,255) PASS`. **R3.4-step-3: MVP-uniform + bind group.** `OmWgpuShaders::kTriangleMVP` adds `@group(0) @binding(0) var<uniform> u : Uniforms { viewProj : mat4x4<f32> }`. `OmWgpuRenderTarget::clearAndDrawMeshMVP(viewProj16, vb, ib, n)` lazy-builds the full stack. Runtime evidence (probe=5): `pixel(4,4)=(0,0,255,255) PASS`. **R3.4-step-4: scene-walk + multi-draw Lambertian shader — SCAFFOLDED but pixel-output BLOCKED.** Three pieces landed: (a) `OmWgpuShaders::kSolidLit` (per-draw uniform with viewProj + model + baseColor + light, single-binding 192-byte block + 64-byte align pad, Lambertian fragment). (b) `OmWgpuRenderTarget::clearAndDrawScene(viewProj, lightDirAmbient, draws[], n, rgba8)` builds one dynamic-offset uniform buffer of `slotCount * 256 B`, depth attachment, one bind-group rebind per draw via dynamic offset. **Runtime-verified standalone (probe=11)**: the same `+Z-normal` triangle from probe=4, fed through `clearAndDrawScene` with `baseColor=(1,0,0)`, reads back `pixel(4,4)=(255,0,0,255) PASS`. (c) `OmCamera::copyImageToMemoryMappedFile` walks `OmWorld::topSolids()` recursively, harvests each Shape's WrStaticMesh through `OmWgpuMeshAdapter::acquireFromWren`, builds a per-Solid model matrix from `OmGeometry::matrix()` + a column-major basis-swap (Webots `+X-forward, +Z-up` → wgpu `-Z-forward, +Y-up`), composes `viewProj = perspective(vertFov, aspect, near, far) * basisSwap * inverse(cameraMatrix())`. With the camera_wgpu_scene_smoke.omniworld test world (a cyan PBRAppearance Box at +X 1m, camera at origin), the scene walk fires correctly — diagnostic log shows `[OmCamera scene] first draw: idx=36 baseColor=(0.00,1.00,1.00) trans=(1.000,0.000,0.000)` and `[OmCamera] 'cam_wgpu' rendered through wgpu (64x48, 1 draw)`. **The blocker:** the readback shows ZERO non-clear pixels — the cube renders to nothing. Root cause diagnosed but not fixed: `wren::StaticMesh::prepareGl` clears `mCoords/mIndices/mNormals/mTexCoords` after uploading them to GL (see `src/wren/StaticMesh.cpp:1916`), and `wren::StaticMesh::readData` then falls back to `glGetBufferSubData` (line 1991+). The OmniSim headless harness runs with `--no-rendering` so there's no GL context, so `readData` returns garbage (no zero-init either) for any mesh built through `wr_static_mesh_unit_*_new` (cache-backed primitives — Box, Sphere, Cylinder, …). `wr_static_mesh_new(...)` from raw arrays (the probe=8 path) keeps `mCoords` populated so it works fine. **Fix paths for next contributor:** (A) — make `prepareGl` keep `mCoords/mIndices` populated alongside the GL buffer (~ +50 MB on a heavy scene, but cleanest); (B) — add a primitive-aware codegen in `OmWgpuMeshAdapter` that generates pos3+norm3+uv2 directly for `OmBox` / `OmSphere` / `OmCylinder` / `OmCapsule` / `OmPlane` from their geometry parameters, bypassing `readData`; (C) — make the wgpu Camera path opt-in via `OmApplication` running with a hidden GL context so `glGetBufferSubData` works. Path B is probably the right call (avoids the memory hit, isolates wgpu-side from WREN-side mesh storage). Diagnostic env knob `OMNISIM_R34_IDENTITY_VP=1` bypasses the viewProj math (passes identity) so contributors can isolate projection-math bugs from mesh-readback bugs. **R3.4-step-5: primitive codegen unblocks step-4 (Fix Path B).** `OmWgpuMeshAdapter::acquirePrimitive(cache, meshId, PrimitiveKind)` (in `src/omnisim/render/OmWgpuMeshAdapter.{hpp,cpp}`) builds the pos3+norm3+uv2+idx stream CPU-side for the four common cache-backed primitives — `Box` (24 verts / 36 idx matching `wren::createUnitBox`), `Plane` (4 verts / 6 idx matching `createUnitRectangle`), `UVSphere` (subdivision-24 lat/long), `Cylinder` (subdivision-24 side + 2 caps). Mesh-id stays the `WrStaticMesh *` bit pattern so caching is shared across the WREN-side and wgpu-side pointers. `OmCamera::collectShapeDraws` switches on `geom->nodeType()` to choose `acquirePrimitive` for those four kinds (falling back to `acquireFromWren` for Capsule / IndexedFaceSet / Mesh / CadShape, which keep `mCoords` populated in WREN — Capsule via `wr_static_mesh_capsule_new` baking radius+height; IFS via raw arrays through `wr_static_mesh_new`). The geometry's local scale (`size` for Box / Plane, `radius` for Sphere, `(radius, radius, height)` for Cylinder — mirroring WREN's `wr_transform_set_scale` choice for each) is composed into the model matrix via `OmMatrix4::scale`. **Build status 2026-05-28 (workflow audit):** all changed sources compile + link clean via `bash scripts/dev/build_with_cd.sh` (`omnisim-bin.exe` rewritten at 7,277,568 bytes; zero errors / zero warnings). The earlier "link blocked by user's running omnisim-bin.exe" caveat no longer applies. **Runtime verification status:** the default-built `omnisim-bin.exe` is `WB_WGPU_NATIVE_AVAILABLE=0` — probe 1 logs `[WbWgpuBackend] wgpu-native unavailable (build flag off, dep missing, or runtime init failed). Probe stops here.` So the `OMNISIM_PROBE_WGPU=N` PASS lines for probes 3-11 + the `camera_wgpu_scene_smoke.omniworld` integration test are **NOT reproducible on the shipped binary by design** (no behavior change for ODE/Newton-only users). To re-verify on this checkout, run `scripts/dev/setup_wgpu_native.sh` then rebuild with the wgpu flag on (recipe in [wgpu-native-setup.md](wgpu-native-setup.md)). On a wgpu-enabled build the smoke world should read back ~half the pixels as the box's cyan `baseColor` (lambertian-shaded against `(0.3, 0.4, -0.85)` + 0.25 ambient); the diagnostic prints `[OmCamera scene] first draw` + `[OmCamera] 'cam_wgpu' rendered through wgpu` remain in place + the integration signal is `non-clear pixels: 0 of 3072` ⇒ `> 0`. **One-shot setup script** [`scripts/dev/setup_wgpu_native.sh`](../../scripts/dev/setup_wgpu_native.sh) lands 2026-05-28. **⚠️ It was BROKEN until 2026-05-29** (so the original "RUNTIME-VERIFIED 2026-05-29" above used a *manually-present* SDK, not this script): it pinned `WGPU_NATIVE_VERSION=v24.0.5.1`, a **non-existent tag (404)**, and downloaded the **msvc** variant while the MinGW build + the script's own `.dll.a` check require the **gnu** variant. **FIXED 2026-05-29** (`15817ae5`): pinned `v29.0.0.0` (the code is "API tracked against wgpu-native v29.0.0+" per `OmVulkanBackend.cpp`) + `wgpu-windows-x86_64-gnu-release.zip`. Separately, [`build_with_cd.sh`](../../scripts/dev/build_with_cd.sh) was fixed (`d57c0002`) to forward `OMNISIM_WITH_VULKAN`/`WGPU_NATIVE_HOME` to make (it silently dropped them, compiling the stub). **Reproducibly RE-VERIFIED 2026-05-29** on a freshly built v29 wgpu-ON binary: `OMNISIM_PROBE_WGPU=2` → `wgpu-native init OK (instance+adapter+device+queue)` + RTT `clearAndRead OK pixel(0,0)=(255,0,255,255) PASS`; `camera_wgpu_scene_smoke.omniworld` → `rendered through wgpu (64x48)` + **non-clear pixels: 3072 of 3072** cyan. The download is still a human-opt-in (auto-mode blocks third-party-binary installs), and — note for in-tool builds — flags must be passed as direct `make` CLI args because this harness's Bash tool strips exported env from child scripts. |
| R3.5 | 🟢 SCAFFOLDED + ✅ STATIC-VERIFIED 2026-05-28 (R3.5b code-landed same day; probe 6 / 10 PASS pixel-values reproducible only on wgpu-enabled rebuild — NOT on shipped default-build binary) | Texture bridge. `OmWgpuTextureCache` (`src/omnisim/render/OmWgpuTextureCache.{hpp,cpp}`) is the texture analog of `OmWgpuMeshCache`: `acquire(textureId, width, height, rgba8bytes)` → `{WGPUTexture, WGPUTextureView}` with `wgpuQueueWriteTexture` upload (handles the 256-byte row alignment internally so callers don't need to). Format pinned at RGBA8Unorm + single mip + sample-count 1. `OmWgpuShaders::kTriangleTextured` is the matching WGSL — uniform at binding 0, `texture_2d<f32>` at binding 1, `sampler` at binding 2. **R3.5b QImage adapter (✅ DONE):** `OmWgpuImageAdapter::acquireFromQImage(cache, textureId, QImage)` in `src/omnisim/render/OmWgpuImageAdapter.{hpp,cpp}`. Approach: pull pixels from `OmImageTexture::mImage` (a `QImage *` that survives wren::Texture2d's mData-destroy-after-GPU-upload because it's owned by the OmniSim node, not WREN). Convert to `Format_RGBA8888` (memory-order RGBA on all endians; ARGB32 is little-endian BGRA which is the wrong byte order for `RGBA8Unorm`). Repack to tight stride if `bytesPerLine != w*4`. **Runtime evidence (probes 6 + 10):** a 2×2 solid-red `QImage(Format_ARGB32)` round-tripped through the adapter renders identical `pixel(4,4)=(255,0,0,255)` to the hand-crafted byte upload of probe=6. |
| R3.6 | 🟢 WGPU-SIDE SCAFFOLDED + ✅ STATIC-VERIFIED 2026-05-28 (R3.6b code-landed same day; 9 reference .ppm files in tree; `[PASS] mean-abs-diff = 0.00` reproducible only on wgpu-enabled rebuild) | Golden-image regression harness for the wgpu render path. Probes 2-10 in `main.cpp` dump their RGBA8 readback to `.ppm` when `OMNISIM_WGPU_PROBE_DIR` is set; [`scripts/dev/wgpu_probe_golden.py`](../../scripts/dev/wgpu_probe_golden.py) is the regression runner: `--update` writes to `docs/developer/wgpu-golden/`, default `--check` mode diffs vs reference at a ≤5/255 (≈2%) mean-absolute-per-channel threshold. **R3.6b world-mode (✅ DONE):** `wgpu_probe_golden.py --world <path.wbt>` boots a .wbt through `headless_runner.py`; the `camera_wgpu_smoke.py` controller dumps the Camera readback to `_scratch/wgpu_world_capture/world_<stem>.ppm` (path passed via `OMNISIM_R33B_PPM_PATH`). World-mode goes through the stable `_scratch` dir not `tempfile` (Windows short-path mangling around the user's 8.3 profile name was breaking the controller's path resolution). **Runtime evidence:** 8 probe goldens + 1 camera_wgpu_smoke world golden, all `[PASS]` at `mean-abs-diff = 0.00`. |
| R3.7 | 🟢 WGPU-SIDE SCAFFOLDED + ✅ STATIC-VERIFIED 2026-05-28 (R3.7b API surface code-landed same day; probe=7 / probe=9 PASS lines reproducible only on wgpu-enabled rebuild) | **Newton-interop bridge** — wgpu storage buffer driving the vertex shader via `@builtin(instance_index)`. `OmWgpuShaders::kTriangleInstanced` declares `@group(0) @binding(1) var<storage, read> bodies : array<vec4<f32>>`; vertex shader reads `bodies[iid].xyz`. `OmWgpuRenderTarget::clearAndDrawInstanced(viewProj, bodyOffsets, bodyCount, vb, ib, n)` grows a `USAGE_Storage` buffer on demand, builds a 2-entry bind group, draws N instances. **Runtime evidence (probe=7):** 2 instances at NDC offsets ±0.5; `pixel(2,4).b=255 pixel(6,4).b=255` PASS. **R3.7b Newton snapshot API (✅ DONE):** `OmNewtonBackend::snapshotBodyTranslations(maxBodies, xyzw)` calls into a Python helper `body_translations_packed(max)` that reuses the existing `_body_q_cache` (one GPU→CPU transfer per step), slices to (x, y, z, 0) per body, returns tight bytes. Writes ≤maxBodies records into the caller buffer; returns count written (≥0) or -1 on no-runtime. **Probe=9 surface test:** without a world loaded the call correctly returns -1; the symbol resolves cleanly. Happy-path runtime test (Newton runs N bodies → snapshot returns N vec4s with the right values → drawn via `clearAndDrawInstanced`) is gated on **TWO** things, not one: (a) R3.4-step-4 scene-walk runtime verification on a wgpu-enabled build, AND (b) the Newton motor → body translation bug (see P6 row) being fixed — without chassis motion there are no body-position deltas to demonstrate. <br><br>**✅ BOTH GATES CLEARED 2026-05-29 — δ happy-path now buildable.** (a) γ scene-walk reproducibly runtime-verified on the rebuilt v29 wgpu-ON binary (`camera_wgpu_scene_smoke` → 3072/3072 cyan px); (b) the chassis-freeze fix (`d56cbf5`) makes the husky drive (−7.0 → +18.25 m), so there ARE per-step body-position deltas to snapshot. The remaining δ work is purely INTEGRATION (no new gate): drive the per-step instance positions from `snapshotBodyTranslations` instead of host test data, in a stepping Newton world. Cleanest path: a per-step δ integration hook (env-gated, e.g. `OMNISIM_PROBE_DELTA`) that, after a Newton world has stepped, calls `snapshotBodyTranslations(N, buf)` → `clearAndDrawInstanced(...)` → reads back and asserts the N instances track the live body positions. (Pre-load `OMNISIM_PROBE_WGPU` probes can't do this — snapshot needs a live, stepped world — so δ lives in the step loop / camera path, not a pre-load probe.) <br><br>**✅ δ RUNTIME-DEMONSTRATED 2026-05-29 (`OmCamera::copyImageToMemoryMappedFile` δ hook + [`camera_wgpu_newton_delta.omniworld`](../../projects/samples/demos/worlds/rendering/camera_wgpu_newton_delta.omniworld)).** With `OMNISIM_PROBE_DELTA=1`, the wgpu Camera calls `OmNewtonBackend::snapshotBodyTranslations` each step and renders the live bodies via `clearAndDrawInstanced` using the camera's own viewProj. On the Newton husky world the log reads: `[OmCamera δ] 5 LIVE Newton bodies → wgpu instanced storage-buffer draw via 'cam_wgpu'; body0=(-7.000,0.000,0.297)` → `0.292` → `0.285` — i.e. the husky's 5 Newton bodies (chassis + 4 wheels) flow from the physics-GPU into the render-GPU vertex shader per-step, with live-changing positions. **The architectural lever (GPU-resident physics feeding GPU-resident rendering) is proven end-to-end.** The hook is opt-in (`OMNISIM_PROBE_DELTA`, default-off) — the normal camera scene-walk path is byte-unchanged (γ re-verified 3072/3072 cyan on the same binary). Remaining δ polish (not blockers): drive the MAIN viewport (not just a Camera RTT) from the snapshot; cull/scale the instance mesh to real body geometry; CUDA/wgpu zero-copy interop (vs the current host round-trip) — these fold into Phase ε/R4. |
| R4 | 🟡 GATED (post R3.6) | Main viewport on wgpu (swapchain + Qt window composition). |
| R5 | 🟡 GATED (post R4) | Sensor pipeline migration — Lidar / RangeFinder / Recognition / Camera on wgpu. Adopts Isaac Lab's tiled-camera pattern. |
| R6 | 🟡 GATED (post R5 + golden parity) | **Canonical-renderer flip.** `renderBackend` default = `"wgpu"`. WREN demoted to legacy fallback. = **Phase ζ "architecturally complete"**. |

### 14.4 — Tier 1-5 fidelity work (WREN-augment track)

These ship on WREN *now* and port to wgpu later. Strategy: each tier
delivers a measurable, named perceptual improvement at a named frame-
time cost. If we cannot write both numbers in advance, the feature
does not ship in that tier.

**Working principle (§13.5 below
absorbed):** Visual-fidelity work (Tier 1) is taste-sensitive and
ships only after verified-better A/B against compatibility with a real
user looking. Performance work (Tier 2) doesn't need this gate —
faster is unambiguously better.

#### Tier 1 — Color and light that's correct (ships on wgpu post-R3.4)

**Track shift 2026-05-28:** Tier 1 originally targeted WREN as the
implementation surface. Per the §14.2 wgpu-first decision, new Tier 1
features now ship on wgpu only — the WREN-side integration on each
sub-item below is **cancelled**. What's already landed on WREN stays.
The WebGL2 previews stay valuable (engine-agnostic spec-by-example).

| Sub-tier | Goal | Status |
|---|---|---|
| T1.0 | Render-backend seam dispatcher | ✅ LANDED (R0-R2) |
| T1.1 | Linear color + AgX tonemap + auto-exposure | ❌ WREN INTEGRATION CANCELLED 2026-05-28 (was ATTEMPTED + REVERTED 2026-04-25 on WREN — user A/B preferred compat). Ships on wgpu post-R3.4 with the WGSL tonemap pipeline. Re-attempt **must** still include A/B reference world set + verified-better A/B *before* shipping, this time vs the wgpu R3.4 baseline. **✅ WebGL2 A/B preview LANDED 2026-05-29** ([agx-tonemap-preview.html](agx-tonemap-preview.html)) — split-screen AgX vs Reinhard vs clip on a synthetic linear-HDR scene (sky gradient + saturated emissive beacon + metallic specular hotspot), EV + auto-exposure(0.18) sliders, minimal-AgX (Sobotka/Wrensch fit) inset→log2→6th-order curve→outset. This is exactly the A/B tool the shipping gate requires; the WGSL port uses the full AgX transform. |
| T1.2 | Cascaded shadow maps + PCF + contact shadows | ⚠️ PREVIEW LANDED 2026-05-28 ([csm-shadows-preview.html](csm-shadows-preview.html)). WREN integration **cancelled**. The preview's GLSL2.0 shaders + cascade-fit JS port to WGSL + wgpu compute-pass for the depth atlases when R3.4 ships. Contact shadows remain a screen-space post-pass added on top of CSM. |
| T1.3 | Hillaire atmospheric sky + volumetric fog | ⚠️ PARTIAL on WREN — Mars + Earth sky LANDED + sensor-camera safety verified (see [csm-shadows-preview.html](csm-shadows-preview.html) sibling sky previews). **Sky portion ports to wgpu via Path-3** when R3.4 ships. **Volumetric fog WREN integration cancelled** — ships on wgpu only. |
| T1.4 | Temporal anti-aliasing | ❌ WREN INTEGRATION CANCELLED 2026-05-28. TAA depends on motion vectors + history-buffer reprojection — both are far more natural on wgpu's pipeline-state-object architecture than on WREN's per-draw uniform model. Ships on wgpu post-R3.4. **✅ WebGL2 preview LANDED 2026-05-29** ([taa-preview.html](taa-preview.html)) — full standard pipeline (Halton(2,3) sub-pixel jitter, ping-pong history, motion-reprojection, 3×3 neighborhood-AABB clamp, feedback blend) with on/off/clamp/feedback/jitter toggles + a hard-aliasing stress grid. Converged starting params for the wgpu port: **jitter ±0.5 px, feedback 0.90, neighborhood clamp ON**. Settles the jitter/clamp choices ahead of the WGSL port, exactly as planned. |

Tier 1 exit: blind A/B panel prefers `modern` over `compatibility` ≥
80% on non-sensor views, measured against the post-R3.4 wgpu baseline.

#### Tier 2 — Scale for big worlds (ships on wgpu)

**Track shift 2026-05-28:** the T2.2.d revert (4.4× warehouse speedup
abandoned because *"instancing on WREN is solving a layer that goes
away entirely with wgpu"*) was the leading indicator for the §14.2
wgpu-first decision. Tier 2 is performance-only — once R3.4 lands,
wgpu's bindless storage buffers + indirect draws + native compute
culling subsume most of Tier 2 by construction.

| Sub-tier | Goal | Status |
|---|---|---|
| T2.1 | Per-pass GPU timers + budget gates | ✅ PARTIAL (WREN timer instrumentation live under `OMNISIM_RENDERER_TIMINGS=1`). Ports to wgpu via the equivalent `wgpu::QuerySet`. Budget-gate auto-disable + CI enforcement deferred to post-R3.4 (will land on wgpu directly, no WREN intermediate). |
| T2.2.a | CPU bounds cache for static renderables | ❌ CUT 2026-05-27. WREN already has a complete dirty-flag AABB cache. |
| T2.2.b/c | GPU compute culling + indirect draws | ❌ CANCELLED on WREN 2026-05-28. Native wgpu indirect draws + storage-buffer-driven culling makes this the right architectural fit; ships as part of R3.4-R3.6. |
| T2.2.d | Instanced draw submission | ❌ CANCELLED on WREN (reverted 2026-05-27, lessons absorbed into §14.2). Bindless storage buffers in wgpu replace per-draw uniforms entirely; the WREN-instancing trick is unnecessary post-R3.4. |
| T2.3 | Hierarchical LOD | ❌ WREN CANCELLED 2026-05-28. Ships on wgpu post-R3.4. |
| T2.4 | Clustered forward+ lighting | ❌ WREN CANCELLED 2026-05-28. Ships on wgpu post-R3.4. |
| T2.5 | Minimal virtual texturing | ❌ WREN CANCELLED 2026-05-28. Ships on wgpu post-R3.4. |

Tier 2 exit (revised): warehouse_industrial 60 FPS mid-tier on the
wgpu R3.4+ baseline. Per the §3.0 probe, warehouse_industrial is the
only biome that doesn't already meet 60 FPS on WREN today, so the
wgpu pipeline's structural-replacement-of-WREN-bottleneck (per-draw
uniforms → bindless storage buffer) is the load-bearing piece.

#### Tier 3-5 (ship on wgpu only)

Per §14.2's wgpu-first track shift, **Tier 3-5 skip WREN entirely**.
They land in Phase ε on the wgpu pipeline, which is the right
architectural fit for compute-heavy fidelity work.

- **Tier 3** (6-8 weeks, wgpu-only): Extended BRDF (clearcoat/sheen/anisotropy),
  SSR, reflection probes, DDGI probes. The reflective-and-detailed-
  surfaces tier.
- **Tier 4** (4-6 weeks, wgpu-only): Virtual shadow maps (multi-light interiors),
  deferred decals (decal-only GBuffer), parallax occlusion mapping,
  triplanar projection (UV-less CAD imports).
- **Tier 5** (9-12 months, multi-engineer, wgpu-only): FSR upscaling (locked over
  DLSS/XeSS for permissive licensing), hardware ray tracing (optional,
  gated), mesh shaders (only if vertex-pipeline ceiling actually hit).

Detail per Tier (file lists, shader paths, exact budgets) lives in
[rendering-and-visual-quality.md](rendering-and-visual-quality.md) and
[asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md),
which remain as authoring-guide references.

### 14.5 — Render-count probe (2026-05-27)

Single load-bearing measurement that informs the rest of Tier 2
sequencing.

Per-frame counts of renderables enqueued / visible (passed visibility-
flag gate) / drawn (survived frustum cull), exposed via
`wr_scene_get_last_render_counts` and logged from `OmWrenWindow`
alongside pass timings. Off by default; gated on
`OMNISIM_RENDERER_TIMINGS=1`.

| Biome | forward ms | post ms | enqueued/visible/drawn | top mesh draws |
|---|---|---|---|---|
| forest | 4.2 (amb 2.5 + perL 1.2 + res 0.5) | 0.4 | 350 / 350 / 150 | 27 max |
| desert_ruins | 2.8 | 0.4 | 271 / 271 / 123 | 66 (id=6, 24v / 12t) |
| **warehouse_industrial** | **24.4** (amb 8.0 + perL 16.4 + res ~0) | 0.7 | 3116 / 3116 / **3114** | **3062 (id=6, 24v / 12t)** |
| mars_big | 3.3 | 0.15 | 399 / 399 / 258 | 35 max |

Three of four biomes already meet T2 60 FPS exit criterion. Only
warehouse_industrial is the outlier (~40 FPS), and its bottleneck is
3062 unit-cube wall/pallet/crate Box-node draws against a single mesh
— a per-draw-submission cost that T2.2.d's instanced submission fixes
on WREN (4.4× measured) and that wgpu eliminates structurally via
indirect-draw + bindless materials.

### 14.6 — §16.2 re-litigation triggers (rendering migration)

**Superseded 2026-05-28** by the §14.2 wgpu-first decision. The
triggers below are kept for historical reference; they're no longer
load-bearing because we're already committed to migrating. The
relevant gating question now is only *when* Phase D fires (= when
Newton 1.2 stable releases), not *whether* to migrate.

The original §1.1 "augment WREN in place for now" decision re-opened
when any of the following fired. **None had fired as of 2026-05-27**;
on 2026-05-28 we re-litigated the broader decision and committed to
wgpu-first independently of any single trigger.

1. **Newton becomes canonical** (Physics Phase D). The trigger most
   likely to fire first per [§13.4](#134--newton-as-default-rollout-phases-ae).
2. **GPU sensor throughput becomes the dominant cost.** Measured
   benchmark: camera/lidar/range-finder render-to-texture passes
   exceed 50% of frame time on a target workload (≥ 4 cameras × ≥ 16
   robots). Adopts Isaac Lab's tiled-camera pattern.
3. **Apple drops OpenGL on macOS.** Deprecated since 10.14; system-log
   deprecation warnings appeared in macOS 15+. When removed (or a
   Metal-only macOS release ships), GL-only path no longer covers a
   primary platform.
4. **A second renderer engineer joins.** The "one engineer can't own
   two backends" assumption carries most of §1.1's cost case.
5. **Tier 3-4 hits an unsolvable GL-driver bug on a primary target.**
   Concrete form of "platform wall GL cannot clear."

**Procedure when a trigger fires:**

1. Document which trigger fired and how it was measured. Add a dated
   row to the decision log (§15).
2. Re-evaluate the three options from §1.1 against the new evidence.
   Re-litigation is the default, not migration.
3. If migration is taken: R3 starts at R3.1 per
   [r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md).

---

## 15 — Decision log (locked decisions across both arms)

| Date | Arm | Decision | Status |
|---|---|---|---|
| Q1 2026 | Physics | Newton + Warp over PhysX-5 and custom CUDA solver | LOCKED |
| Q2 2026 | Rendering | Augment WREN in place through Tiers 1-5 (vs parallel modern-GPU backend) | SUPERSEDED 2026-05-28 by wgpu-first single-track (see row below) |
| Q2 2026 | Rendering | Standalone RHI from day one | NOT DONE (bgfx/wgpu reopen as bundled-backend) |
| 2026-04-25 | Rendering | T1.1 AgX ship vs revert | REVERTED; verified-better A/B gate now mandatory |
| 2026-04-28 | Physics | CUDA-only, NVIDIA-only for compute infrastructure | LOCKED (revisit if Apple Silicon / AMD becomes user base) |
| 2026-05-27 | Rendering | T2.2 first commit | T2.2.d attempted then reverted; T2.2.b/c deprioritized |
| 2026-05-27 | Rendering | Engine-migration R3+ | SUPERSEDED 2026-05-28 — R3.1–R3.7b code-landed under wgpu-first single-track (Phase γ); see §14.3 |
| 2026-05-27 | Rendering | T2.2.a CPU bounds cache | CUT; redirected to render-count instrumentation |
| 2026-05-27 | Rendering | R3 backend pick (refresh) | wgpu-native over bgfx (archived) |
| 2026-05-27 | Both | Roadmap unification | `physics-roadmap.md` + `rendering-roadmap.md` merged into this doc (commit `e1dc72c5`) |
| 2026-05-28 | Physics | P1.6 first step: hinge-angle read through the dispatcher | LANDED; `OmHingeJoint::postPhysicsStep` ad-hoc Newton/ODE branch now routes via `OmPhysicsBackend::getJointHingeAngle/Rate`. Smoke: Newton + ODE husky 5s 0 errors. |
| 2026-05-28 | Physics | P1.6 second step: slider-position + AMotor angle/rate reads | LANDED; `OmSliderJoint::postPhysicsStep` and `OmBallJoint::postPhysicsStep` (3 axes) now route through `OmPhysicsBackend::getJointSliderPosition` and `::getJointAMotorAngle/Rate`. Per-step joint read surface fully on the dispatcher. Smoke: linear_motor + motor3 5s 0 errors each. |
| 2026-05-28 | Physics | P1.6 third step: user-defined torque/force writes | LANDED; `addJointHingeTorque`/`addJointSliderForce`/`addJointAMotorTorques` virtuals + ODE overrides; `OmHingeJoint`/`OmSliderJoint`/`OmBallJoint` prePhysicsStep user-control paths route through the dispatcher. Hot-path joint write-side (add-torque trio) on the dispatcher. Smoke: 4 worlds 5s 0 errors. |
| 2026-05-28 | Physics | P1.6 fourth step: per-step setParam(FMax, Velocity) | LANDED; `OmJointParam` enum + `setJointHingeParam`/`setJointSliderParam`/`setJointAMotorParam(axis,…)` virtuals + ODE overrides mapping the enum to ODE's `dParam*` constants. Per-step hot-path joint surface (read + write) now fully on the dispatcher. Smoke: 4 worlds 5s 0 errors. |
| 2026-05-28 | Physics | P1.6 fifth step: joint enable/disable lifecycle | LANDED; `setJointEnabled(bool)` + `isJointEnabled()` virtuals on `OmPhysicsBackend` + ODE overrides; `OmBasicJoint::setOdeJoint`, `::checkSolids`, `::isEnabled`, and `OmBallJoint::setupAsTaughtJoint` (control-motor enable/disable) now route through the dispatcher. Smoke: 3 worlds 5s 0 errors. |
| 2026-05-28 | Physics | P1.6 sixth step: world-load setParam family | LANDED; `OmJointParam` enum grew to cover `WB_JP_LO_STOP/HI_STOP/STOP_CFM/STOP_ERP/SUSPENSION_ERP/SUSPENSION_CFM`; added `setJointLMotorParam(axis,…)` virtual + ODE override. World-load + spring/damper + suspension + reset callsites in OmHingeJoint, OmBallJoint (3 axes × 4 params + 3 axes × 2 limits), OmSliderJoint, OmRotationalMotor, OmLinearMotor, OmSolid (Hinge/Slider zero-on-disable) all on the dispatcher. Hinge2 + Ball special-joint params deferred to slice 7. Smoke: 4 worlds 5s 0 errors. |
| 2026-05-28 | Physics | P1.6 seventh step: Hinge2 + Ball setParam family | LANDED; added `setJointHinge2Param(axis,…)`, `setJointBallParam(axis,…)`, and `addJointHinge2Torques` virtuals + ODE overrides. Migrations: OmHinge2Joint per-step torque + velocity-control (axes 0 + 1) and world-load joint limits; OmHingeJoint's Hinge2 fall-throughs in `applyToOdeStopErp/Cfm` and `applyToOdeSuspension`; OmRotationalMotor's Hinge2 branch; OmSolid's Hinge2 (2 axes × FMax + Vel) and Ball (3 axes × FMax + Vel) zero-on-disable cases. All ODE-side direct setParam dJoint*Param callsites for Hinge / Hinge2 / Slider / AMotor / LMotor / Ball now route through the dispatcher. Smoke: motor2 (Hinge2), husky_head_on (Hinge), motor3 (Ball), newton_husky_smoke 5s 0 errors each. |
| 2026-05-28 | Physics | **P1.6 COMPLETE** | 17 dispatcher virtuals, 7 sequential commits, every per-tick + world-load joint read/write callsite on the dispatcher. Joint feedback / lifecycle / world-load anchor-axis setup carved out as separate concerns ("P1.7 lifecycle widening" if ever needed) — they're ODE-internal mechanics with no Newton equivalent, since Newton-backed joints route through `OmBasicJoint`'s `setJointTargetPosition/Velocity` and `OmNewtonBackend::addJointRevolute` paths, not the dispatcher. Phase α physics work now narrows to P5 stabilization (gated on upstream Newton fix), P6 (damage-suite parity), P7 (wire-protocol audit + reset hook). |
| 2026-05-28 | Physics | **P7 COMPLETE** | `OmPhysicsBackend::reset()` hook landed (default no-op; Newton override calls `resetJointsToDefaults()`); wired into `OmSimulationWorld::reset` after the per-Solid cascade. Wire-protocol audit found every controller↔simulator message either works correctly on Newton or has documented warn-and-skip behavior. Known limitations: `C_MOTOR_FEEDBACK` (Newton tracks but doesn't expose per-joint constraint torque), specialized-device ODE-internals (Brake/Connector/TouchSensor). The "remaining Newton write overrides" item resolved by referencing the §10.1 / `66c0e5c0` decision (warn-and-skip is the honest answer). Phase D gating: of the five conditions, P6 damage-suite parity and P5 in-OmniSim 20v20 + upstream body-index-30 cliff fix remain. |
| 2026-05-28 | Physics | P6 smoke verified (quantitative parity bumped to Phase β) | `newton_husky_head_on_damage.omniworld` runs headless 30s cleanly. Damage tracker doesn't crash or silently skip Newton-backed Solids (the load-bearing concern). Quantitative event-parity (Newton vs ODE damage-event-stream diff) needs a dedicated harness TCP client subscribing to `damage_events`, which is harness-tooling work not OmniSim-engine work; moved to a Phase β "P6 damage-suite parity harness" line item rather than blocking Phase α. |
| 2026-05-28 | Physics | P5 in-OmniSim ceiling re-characterised: old upstream blocker resolved, new ceiling at 10 huskies | The "body-index-30 cliff" the plan named as P5's upstream gating condition was actually a symptom of the joint-anchor rotation projection bug fixed locally in `16dcabe`. Confirmed by bisect: 9-husky Newton world (`newton_husky_9.omniworld`) loads + finalizes + starts controllers cleanly; 10-husky Newton world (`newton_husky_10.omniworld`) queues all 40 joints then hangs before finalize. Reproduces on both `SolverXPBD` and `SolverMuJoCo` → bug sits in the model-build or Python-finalize bridge, not the solver. Bumped from "blocked on upstream" to "blocked on a localized debug pass next session." Doesn't gate Phase D (per-Solid `"auto"` resolution already falls back to ODE above the ceiling). |
| 2026-05-28 | Physics | P5 hang further localized: Newton-specific, exact-symmetric ODE world works fine | Built `ode_husky_10.omniworld` — same 10-URDFRobot layout but no `physicsBackend "newton"` field. Headless 60s: 11 controllers started (10 drive_forward + sun_marker), world fully loaded, simulator stepping. Newton-equivalent `newton_husky_10.omniworld` headless 180s: queue stops at 39/40 joints (h9's `rear_left_wheel_link`), h9's `rear_right_wheel_link` never enqueues, no log progress past that point. So the hang is in the Newton path of `OmBasicJoint::postFinalize` (between the queue-append for joint 39 and the queue-append for joint 40), NOT in the URDF importer or scene-tree-build. The next session's targeted localization: add a log line BEFORE the queue-append, run 10-husky again, see whether postFinalize is being entered for the 40th joint but not exiting (= internal Newton-path hang), or never entered (= the dispatch up-tree to the 40th joint never happens). <br><br>**Cross-session coordination note (2026-05-28):** the concurrent G1 session committed `36070081` adding `solid-iter` logging to `OmSolid::flushPendingNewtonRegistrations`, and `e80163fe` for joint-queue logging. They flagged the G1 body-registration hang as having regressed between 08:14 and 09:00 builds, but the suspect commits in that window are P7 (`4f88cc51`, only touches `OmSimulationWorld::reset`, not world load) and two doc commits. The 9-husky world still works after all P1.6 + P7 + P6 changes; the 10-husky hang reproduces back through prior sessions per `OmNewtonBackend.cpp:671-679`'s mention of the multi-articulation cliff. Likely the two hangs (G1's 13-leg-joint body-registration hang and husky's 40-joint queueing hang) are distinct Newton bridge bugs at different scales. |
| 2026-05-28 | Physics | **P5 world-load hang FIXED** — root cause was OmLog message accumulation, not Newton | After bisecting (mixed Newton+ODE worlds don't hang → it's Newton-only path) and adding entry/exit diagnostics around `OmBasicJoint::postFinalize`, found that every `OmLog::info`/`warning` call during world load adds compounding per-call cost that stalls the load past ~30-40 messages. The concurrent session reached the same conclusion independently via the G1 path (commit `6388d3fc`). Fix in `2f0450e8` + `5731e2e5`: silence the per-joint queued-joint info; cap the repeated null-physics warning at 5; short-circuit the `receivers()` meta-object lookup when `mConsoleLogsPostponed` is true; silence the per-body "registered solid" info (fires during step, after postponed=false). World load now scales — 10-husky 60s ✓, 20-husky 180s ✓, **40-husky (20v20) 240s ✓ — the P5 north-star world LOADS.** |
| 2026-05-28 | Physics | P5 step-1 body-registration suspected bottleneck (SUPERSEDED later same day — root cause was `--no-window` + PIPE-fill in headless harness, not Newton scaling; see next row) | After world load completes, the simulator must register every Newton-backed Solid (call `addBody` + `addShape` per body via embedded CPython) in step 1's `OmSolid::flushPendingNewtonRegistrations`. Measured per-body cost: 4v4 (40 bodies) reaches 19/32 joints in 60s; 20-husky (100 bodies) reaches 14 hinges in 240s; 30-husky (150 bodies) reaches 9 bodies in 600s; 20v20 (200 bodies) doesn't reach step in 1200s. Scaling looks worse than linear. The bisect worlds in `projects/robot_combat/worlds/tests/newton_husky_{30,20v20_async,20v20_void}.wbt` rule out controller-synchronization-gate as the cause. Next push (next session): batch the C++ → Python `add_body` / `add_joint_revolute` calls so the model-build round-trips Python once instead of N times; OR profile the C++ side per-iter cost (`s->matrix()`, `s->rotationMatrix()`, `rolledUpMass()`) for O(N²) walks. Not a correctness blocker — `physicsBackend "auto"` already falls back to ODE above the Newton perf ceiling. |
| 2026-05-28 | Physics | **P5 COMPLETE — 20v20 north-star verified end-to-end** | The "step-1 body-registration bottleneck" row above turned out to be wrong about the cause. Root cause was actually `headless_runner.py` using `--no-window` (Newton-incompatible event-loop interaction, confirmed by the concurrent G1 session in `ef571361`) PLUS `subprocess.Popen(..., stdout=PIPE, stderr=PIPE)` filling the unread 64 KB OS pipe buffer at scale (writes block once full). Fix in `71b11f01`: switch the harness to `--minimize --batch --no-rendering` + `DEVNULL`. Verified end-to-end through the unmodified harness on the post-fix binary: `newton_husky_20v20.omniworld` 300s, CTRLS=41, OPENED=1, **HINGES=160**, FINALIZE=1, 0 errors. The P5 north-star world fully loads, finalises, and steps. Six P5 myths busted this session: (1) "body-index-30 cliff" was a local joint-anchor bug fixed two weeks ago, (2) the multi-husky hang was `OmLog` accumulation not Newton, (3) "step-1 scaling cliff" was `--no-window` + PIPE fill, (4) Newton handles 40 huskies fine, (5) the harness needed updating not the engine, (6) the entire blocker chain was self-imposed by stale claims in the plan doc. |
| **2026-05-28** | **Rendering** | **wgpu-first; stop new WREN integration work** | Re-litigated the original "augment WREN in place through Tiers 1-5" decision (Q2 2026 row above) and **flipped to a single-track wgpu-first strategy**. Trigger: the T2.2.d revert (a measured 4.4× warehouse perf win abandoned because *"instancing on WREN is solving a layer that goes away entirely with wgpu"*) was the load-bearing example of why double-implementing on WREN-then-wgpu burns engineering for transient value. With Phase D close (mostly gated on Newton 1.2 stable releasing), the WREN-only window for new fidelity features narrowed from "12-18 months" to "6-9 months" — not enough runway to amortise the double build. **What changes:** new Tier 1-5 fidelity features (T1.1 AgX, T1.2 CSM, T1.4 TAA, T2.x, T3-T5) ship on **wgpu only**, post-R3.4. **What stays:** WebGL2 previews (engine-agnostic, spec-by-example for whichever shader language ships); the R0-R2 dispatcher seam; everything already landed on WREN (T1.0 dispatcher, T1.3 Mars + Earth sky + sensor-camera determinism, T2.1 timer instrumentation) — those port forward via Path-3 when R3.4 lands. **What this changes about the §6 sequencing:** Phase α's rendering work shrinks to what's already done; Phase γ (R3.1-R3.6) becomes the load-bearing rendering effort; Phase ε absorbs all Tier 1-5 fidelity work (built once, on the right architecture). |
| **2026-05-28** | **Physics + cross-arm** | **Phase D flip landed under partly-stale gating; needs follow-up** | `Solid.wrl` + `Robot.wrl` `physicsBackend` default flipped to `"auto"` in `baa1c104`. **Gating audit 2026-05-28 (workflow):** (1) body-index-30 cliff vacuous (`16dcabe`) ✓; (2) **Newton 1.2 stable — UNSATISFIED**, live `import newton; newton.__version__` returns `1.2.0rc3`, NOT `1.2.0` as commit body claimed; (3) **P6 damage parity — UNSATISFIED**, fresh capture in `bfdbced9` showed 149 ODE vs 0 Newton (the `105 vs 16, ratio 0.15` numbers were stale; root cause = Newton motor → body translation bug, runtime-confirmed solver-independent on canonical `newton_husky_smoke_test.omniworld`); (4) P7 wire-protocol audit done (`4f88cc51`) ✓; (5) smoke 3/3 OK ✓. The flip is reversible. **Recommendation:** accept rc3 + the motor-bug as known limitations of Phase D rather than roll back — ODE remains an opt-out via `physicsBackend "ode"`, rc3→1.2.0 delta is expected to be small, motor-bug is in OmniSim's wiring (fix it forward). Unblocks Phase γ (R3.1 init probe) per §6 sequencing — Phase γ R3.1-R3.7b have already landed in the same session (`9d103595` … `92c7616c`). |

---

## 16 — Test matrix (required-passing on every PR, forever)

The load-bearing safety net. **Enforcement status 2026-05-28:** this
is a policy contract, enforced locally via the **pre-push smoke
hook** (`tests/smoke/run_smoke.py` over `tests/smoke/smoke_worlds.json`,
4 active worlds: empty / accelerometer / contact_points /
template_deterministic). The **per-OS** CI workflow files live in
`.github/workflows.disabled/` and are not active until the legacy
upstream-Webots CI is replaced. (Standing GitHub Actions CI is *not*
empty, though: a targeted RL `g1-spec-conformance.yml` gate already
runs on every `projects/policies/**` push/PR — it just does not cover this
world/build matrix.) (Naming note: `OMNISIM_WITH_VULKAN`
remains the build flag's literal name for historical reasons; the
runtime backend is wgpu-native — see §14.3 R3.1.)

| Build | Worlds | Required |
|---|---|---|
| Newton-OFF, wgpu-OFF | all existing | bit-identical to legacy |
| Newton-ON,  wgpu-OFF | all existing | identical (no Newton worlds use new field) |
| Newton-OFF, wgpu-ON  | all existing | identical (no wgpu cameras use new field) |
| Newton-ON,  wgpu-ON  | all existing + Newton/wgpu demos | new demos work + legacy unchanged |

Three of four entries must reproduce legacy output bit-for-bit. CI
expands every time a new backend lands; the legacy-only entry stays
green forever.

---

## 17 — Maintenance convention

This doc is the canonical engine roadmap. When closing a sub-item:

- **Sub-phase completion within a phase:** update §13 (physics) or §14
  (rendering) status table. Don't strike-through; just flip the status
  cell.
- **Phase boundary closure (P*N* → P*N+1*, T*N* → T*N+1*, R*N* → R*N+1*):**
  update both the per-arm status table *and* the §6 sequencing diagram.
  Decision-log row added to §15 if a directional choice was made.
- **New feature emerges:** add to the right per-arm section. Update §6
  if it has a cross-arm dependency.
- **Decision re-litigated:** update §15 with a dated row + revise the
  affected per-arm section. Particularly §16.2 triggers when one fires.

Sister docs that survive this consolidation and aren't replaced by
this map:

- [physics-and-determinism.md](physics-and-determinism.md) — physics
  architecture details + determinism contract
- [physics-contact-and-collision-complexity.md](physics-contact-and-collision-complexity.md)
  — contact-pipeline guidance + authoring hazards
- [omniquad-residual-rl.md](omniquad-residual-rl.md) — model+residual RL recipe under Newton
- [rendering-and-visual-quality.md](rendering-and-visual-quality.md) —
  current renderer state, edit map, validation commands
- [asset-pipeline-and-world-quality.md](asset-pipeline-and-world-quality.md)
  — texture, cache, remote-asset, world-authoring rules
- [sensor-and-device-performance.md](sensor-and-device-performance.md)
  — sensor rendering path
- [world-loading-and-template-performance.md](world-loading-and-template-performance.md)
  — parse + load-path behavior
- [r3-rendering-backend-evaluation.md](r3-rendering-backend-evaluation.md)
  — R3 design (wgpu-native, seven sub-phases)
- [observability-and-performance-telemetry.md](observability-and-performance-telemetry.md)
  — existing perf telemetry

This doc replaces engine-migration-plan.md and engine-migration-plan.md. Don't
recreate them — extend this one.
