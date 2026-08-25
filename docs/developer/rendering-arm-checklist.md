# Rendering arm — completion checklist (WREN → wgpu)

Living checklist of everything still required to finish the rendering arm, i.e.
to reach the Phase ζ "architecturally complete" definition-of-done in
[engine-migration-plan.md](engine-migration-plan.md) §8. Verified against §6
(sequencing), §8 (definition-of-done), §8.1 (reality-check audit), and the §ε
R5/Tier detail. Update the boxes as items land; each item should be a verified,
zero-break increment (build clean → headless-verify where possible → commit).

> **Canonical status:** the single code-verified migration-status snapshot is
> [engine-migration-plan.md §8.1 → "Status refresh — 2026-06-08"](engine-migration-plan.md).
> This checklist is the render-arm *task list*; where they disagree, §8.1 wins.

Status snapshot: ~30–35% of the arm toward Phase ζ. **T1.2 CSM is functionally
complete** on the diagnostic camera path — light depth pass + self-shadow (3c) +
cross-object cast shadow (3d, controlled-experiment verified) + 3x3 PCF (soft
edges observed), each a verified zero-break increment. **R4 update (2026-06-07):
the offscreen-wgpu→GL-blit main-view path (`renderMainFrameViaWgpu`,
[OmView3D.cpp:1548](../../src/omnisim/gui/OmView3D.cpp)) is now UN-GATED for any
Viewpoint with `renderBackend "wgpu"`** — functional + leak-free at ~75% lighting
parity (the native-surface-pane track below, behind `OMNISIM_VIEW3D_WGPU`, is a
separate experimental R4 path; both exist in the tree). **Update: the multi-cascade
(CSM) on-GPU path is now wired + verified IN-ENGINE** — `clearAndDrawSceneCsm` +
the `OMNISIM_PROBE_CSM` headless probe reproduce csm_render_prototype.py's GPU-validated
values byte-for-byte (floor under caster darkens, side stays lit, cascade 1 of 3), golden
zero-break. The remaining CSM item is wiring `clearAndDrawSceneCsm` into the live MAIN-VIEW
render (`renderMainFrameViaWgpu`, L3); single-cascade self-shadow still runs only under the
gated `OMNISIM_CAMERA_SHADOW` camera diagnostic. Plan's own estimate for the whole
arm is **12–18 months single-engineer**. HEAD at last update: `2230d3aa`.

---

## ✅ Done + verified
- [x] Sensor pipeline on wgpu — Camera, RangeFinder (R5b/R5c device node), full
      Lidar family R5d–R5j (single/multi-layer × narrow/wide-FOV × tilt ×
      rotating), each golden/regression-guarded.
- [x] OmCamera migrated onto the shared `OmWgpuSceneRenderer`.
- [x] T1.1 AgX tonemap — curve + exposure + emissive + specular HDR sources,
      each runtime-verified + regression-guarded.
- [x] T1.2 CSM **light-space depth pass** (the shadow-map producer half):
      ortho light viewProj builder, `kSolidClipDepthF32` clip.z shader, pipeline,
      pipeline-selection fix — verified center 0.4673, regression-guarded.
- [x] T1.2 CSM **3a** — `kSolidLitShadow` shadow-receiver shader string + header
      decl (UNREFERENCED; WGSL not yet naga-validated — no pipeline builds it).
- [x] T1.2 CSM **multi-cascade fitting math** — `buildCascadeLightViewProjs`:
      generalises the single ortho light frustum (`buildOrthoLightViewProj`) to N.
      PSSM log↔uniform split (`buildCascadeSplits`, blended by `splitLambda`) + a
      per-cascade TIGHT off-center ortho fit: invert each camera sub-frustum's
      viewProj → its 8 world corners → light-space AABB → square + extend-near so
      casters in front still cast. New anon-namespace helpers `orthoOffCenter`
      (general form of `ortho`; symmetric case reproduces it bit-for-bit), `mul4`/
      `mulVec4` (column-major), `invert4` (MESA adjugate). **DONE + VERIFIED (commit
      `cdd8bb2c`):** INERT — no call sites yet (mirrors how `buildOrthoLightViewProj`
      landed) → zero-break. Standalone numeric test 6/6: `orthoOffCenter`==`ortho`
      bit-for-bit, ortho depth near→0/far→1, `invert4` round-trips to I, splits exact
      + strictly increasing, log pulls splits nearer than uniform, and the decisive
      end-to-end check — every cascade's fitted light-frustum CONTAINS its camera
      slice (corners in light NDC x,y∈[-1,1], z∈[0,1]) with an off-axis light. In-tree
      `-fsyntax-only` clean under production flags (C++11, `-DWB_WGPU_NATIVE_AVAILABLE
      -DOMNISIM_WITH_NEWTON -Wall`, no warnings).
- [x] T1.2 CSM **multi-cascade receiver shader** — `kSolidLitCsm` ([OmWgpuShaders.cpp](../../src/omnisim/render/OmWgpuShaders.cpp)):
      N-cascade generalisation of `kSolidLitShadow`. Carries an `array<mat4x4,4>` of per-cascade
      light VPs + a `cascadeSplits` vec4 (far view-depths) in a 448 B `CsmScene` uniform @0, samples
      a `texture_2d_array` shadow map @1 (layer per cascade) via a non-filtering sampler @2, and
      selects the cascade by the fragment's LINEAR view depth (camera clip.w varying — the
      `kSolidDistance` trick), then 3x3-PCFs that layer through the cascade's own light VP.
      `shadowParams.z` = cascade count; strength 0 → byte-identical to `kSolidLit`. **DONE +
      VERIFIED (this commit):** UNREFERENCED — no pipeline builds it yet (mirrors how the 3a
      `kSolidLitShadow` string landed) → zero-break. STRONGER than 3a's deferred validation: naga
      **and** render-pipeline validated standalone on the live RTX 3060 (wgpu-py
      `create_shader_module` + `create_render_pipeline` against the INTENDED bind-group layout —
      uniform@0 448 B + `texture_2d_array` unfilterable-float @1 + non-filtering sampler @2 — and
      the pos3/norm3/uv2 stride-32 vertex layout). So the WGSL semantics + the @group/@binding +
      vertex/fragment I/O are all confirmed buildable before the engine wires the pipeline.
      **NEXT (on-GPU half, needs a wgpu engine relink):** render N depth-map layers (drive
      `buildCascadeLightViewProjs` per cascade into a `texture_2d_array`) + build the `kSolidLitCsm`
      pipeline/bind-group in `OmWgpuRenderTarget` + A/B a cascade-split render.
- [x] T1.2 CSM **end-to-end render proof** — [docs/developer/csm_render_prototype.py](csm_render_prototype.py):
      the whole on-GPU design (the *next* engine step) rendered + pixel-asserted in Python via
      wgpu-py, so it needs NO native build. Drives a Python port of `buildCascadeLightViewProjs`,
      renders N=3 cascade depth layers into a `texture_2d_array`, then the lit+shadow pass using the
      **LIVE `kSolidLitCsm`** read straight from the engine source (doubles as a regression guard).
      **DONE + VERIFIED on the live RTX 3060 (Vulkan), 5/5:** strength-0 deterministic; a floor point
      occluded by a caster DARKENS (lit `(204,204,204)` → shadowed `(82,82,82)`) while a floor point
      to the side stays fully lit (no false shadow); and the shadowed point's linear view-depth
      (clip.w = 22.70) routes it through **cascade 1 of 3** (splits far `[9.61, 23.37, 60]`) — i.e.
      multi-cascade *selection* + a per-cascade light VP + the right array layer are genuinely
      exercised, not a single near cascade covering everything. So the engine wiring below is now a
      faithful translation of a GPU-proven design, not an unproven port.
      **Engine-wiring caveat (coordination):** the single-cascade runtime probe entry lives in
      `OmCamera.cpp` (`OMNISIM_CAMERA_SHADOW(PROBE)`), which is NOT an L2-owned file, so wiring an
      in-engine CSM probe to runtime-verify the `OmWgpuRenderTarget` translation needs an
      owned/coordinated trigger — flag for the integrator (this prototype is the stand-in proof until then).
- [x] T1.2 CSM **on-GPU engine wiring + in-engine verification** — the multi-cascade design is now LIVE
      in the engine (no longer Python-only). `OmWgpuRenderTarget` gains three render-layer methods
      ([render/OmWgpuRenderTarget.{cpp,hpp}](../../src/omnisim/render/OmWgpuRenderTarget.cpp)):
      `ensureShadowMapArray(res,N)` (an N-layer R32Float `texture_2d_array` sampleable shadow map +
      per-layer 2D render views + a shared Depth24Plus occlusion attachment), `ensureSceneCsmPipeline`
      (builds `kSolidLitCsm` against its 3-entry layout — CsmScene uniform @0 448 B dynamic +
      `texture_2d_array` unfilterable-float @1 + non-filtering sampler @2 — naga-validating the shader
      IN-ENGINE), and `clearAndDrawSceneCsm` (N light clip-depth passes into the array layers driven by
      `buildCascadeLightViewProjs`'s per-cascade VPs, then ONE `kSolidLitCsm` lit pass selecting a cascade
      per-fragment by view depth + 3×3 PCF). The render layer stays self-contained (raw float matrices, no
      maths/nodes up-dependency — `WB_RENDER_INCLUDE` omits `-Imaths`/`-Inodes` by design); the OmMatrix4 +
      cascade math lives in `OmWgpuSceneRenderer::csmSelfTest` (nodes/, which owns both). **Resolved the
      engine-wiring caveat** (the old `OMNISIM_CAMERA_SHADOW` trigger was in non-L2 `OmCamera.cpp`): a new
      additive headless probe `OMNISIM_PROBE_CSM` (a 7th render-probe sibling in `gui/main.cpp`, mirroring
      `OMNISIM_PROBE_TEX/INSET`) builds the in-engine prototype scene and renders it. **DONE + VERIFIED on
      the live RTX 3060 (wgpu-ON build):** `OMNISIM_PROBE_CSM` → `built=1 floorLit=(204,204,204)
      shadowed=(82,82,82) litSide=(204,204,204) cascade=1` — the floor under the caster DARKENS, a floor
      point to the side STAYS LIT, and the shadow point routes through **cascade 1 of 3** (multi-cascade
      selection genuinely exercised) — **byte-identical to csm_render_prototype.py's GPU-validated values**
      (the prototype is now a faithful mirror, not a stand-in). **Zero-break:** the panda render golden gate
      (`render_oracle.py`) MATCHES at mean-diff/ch=0.000, within-tol 76% unchanged.
- [x] T1.2 CSM **main-view wiring (L2→L3 seam, gated)** — `clearAndDrawSceneCsm` is now wired into the
      live main-view render (`renderMainFrameViaWgpu` in L3-owned [gui/OmView3D.cpp](../../src/omnisim/gui/OmView3D.cpp))
      behind a **default-off** `OMNISIM_WGPU_MAINVIEW_CSM` gate: it fits N=3 per-camera-frustum cascades
      (`buildCascadeLightViewProjs` over the [0.05, 40] shadow range) and renders the main view through the
      multi-cascade path instead of the single fixed-extent ortho frustum. **VERIFIED:** gate-OFF keeps the
      textured-shadow render + the panda surface golden byte-identical (mean-diff/ch=0.000); gate-ON (forced
      via `OMNISIM_WGPU_MAINVIEW_FORCE`, dumped via `OMNISIM_WGPU_MAINVIEW_DUMP`) renders the panda factory
      scene with correct multi-cascade shadows on the floor. **Caveat:** flat-shaded (clearAndDrawSceneCsm
      uses baseColor, not the material path) — so this is a gated proof-of-wiring, not the default.
- [x] T1.2 CSM **full-material × multi-cascade merge (L2) + main-view upgrade (L3)** — the gated
      `OMNISIM_WGPU_MAINVIEW_CSM` path now renders with BOTH full materials AND multi-cascade shadows.
      New L2 shader `kSolidLitTexturedCsm` ([render/OmWgpuShaders.cpp](../../src/omnisim/render/OmWgpuShaders.cpp))
      = `kSolidLitTexturedShadow` (albedo/roughness/metalness/normal + GGX + sRGB + hemisphere ambient) with
      the `kSolidLitCsm` cascade selection grafted into its shadow term (@6 a `texture_2d_array`; LightU
      carries `array<mat4,4>` light VPs + `cascadeSplits`; per-fragment cascade select by view depth + 3x3
      PCF). New `ensureTexturedCsmPipeline` (9-entry layout, @6 2D-array, @8 336 B) + `clearAndDrawSceneTexturedCsm`
      (N light-depth passes into the cascade array, then one material+multi-cascade lit pass with per-draw
      material bind groups). The gated main-view branch ([gui/OmView3D.cpp](../../src/omnisim/gui/OmView3D.cpp))
      now calls it. **VERIFIED on the RTX 3060:** forced gate-on main-view dump of the panda factory world
      renders with wood-grain table textures, a textured floor + wall paneling, AND multi-cascade shadows
      (1960×1122, std RGB ~[45,40,36] = rich material variation vs the flat render's uniformity); gate-off
      stays the single-cascade textured render byte-identical (additive shader/pipeline/method + the else
      branch is the verbatim original). **NEXT:** flip the main-view DEFAULT to this (human-gated — needs the
      WREN-parity refresh via render_oracle + a golden update), so `renderBackend "wgpu"` worlds get
      materials + multi-cascade by default. That flip is the Phase-ζ decision.

---

## ⬜ T1.2 CSM — finish shadows (near-term, headless-verifiable)
- [x] **3b** — `ensureSceneShadowPipeline`: 3-entry bind-group layout (ShadowScene
      uniform @0 240B + UnfilterableFloat texture @1 + NonFiltering sampler @2)
      building `kSolidLitShadow`. **DONE + VERIFIED (commit 4cd9f6b2):** naga
      compiled the 3a WGSL (headless probe `OMNISIM_CAMERA_SHADOWPROBE=1` → PASS),
      golden 0.00 zero-break. Note: R32Float is non-filterable in core WebGPU, so
      the shader was fixed to `textureSampleLevel` + non-filtering sampler.
- [x] **3c** — two-pass orchestration wired into OmCamera
      (`OMNISIM_CAMERA_SHADOW`, gated/default-off). **DONE + VERIFIED (commit
      98310239):** on camera_wgpu_scene_smoke, self-verified A/B (script echoed
      the env value it set): strength 0.0 → center BGRA (0,141,141); strength 0.6
      → (0,95,95) = real attenuation by the shadow term. Golden genuinely
      byte-identical with the feature off (every probe mean-abs-diff 0.00/255,
      exit 0) → zero-break. NOTE: box-fills-frame cannot distinguish self-shadow
      from a cross-object cast shadow — that distinction is 3d's job, still open.
- [x] **3d** — cross-object cast shadow onto a SEPARATE receiver. **DONE +
      VERIFIED by a controlled experiment (commits `e552f77d` harness,
      `ddf5fa71` control).** The diagnostic grid dump is frame-numbered (`frame N`,
      30-call window) so separate-run A/Bs compare the SAME frame (frames 0/1/2
      byte-identical → deterministic). Two fixtures with IDENTICAL camera + floor +
      light, differing only by the caster: `camera_wgpu_shadow_cast.omniworld` (has an
      elevated caster) and `camera_wgpu_shadow_nocaster.omniworld` (control, caster
      removed). Decisive cell — frame 2, row 5 cols 5-6, which is FLOOR in BOTH
      renders (the caster projects to rows 1-3, not row 5), read from logs:
        cast world, strength 0.0 → 140 140 (lit)
        cast world, strength 0.6 →  77  77 (shadowed)
        control,    strength 0.6 → 140 140 (lit — no caster, no shadow)
      The cell darkens only with shadows on AND the caster present; removing the
      caster restores it to lit. Same framing in both, so it isolates the caster
      as the cause → a true cross-object cast shadow.
      HONEST SCOPE: this proves CORRECTNESS at the resolved cells, not shadow
      extent/quality — the coarse 8×10 grid + straight-down light (shadow mostly
      under the caster) resolve only ~2 floor cells. Self-shadow (3c) also verified
      (box 141→95). Shadow-map pass depth-tested (Depth24Plus/Less). Zero-break.
- [x] **PCF** — 3x3 percentage-closer soft-shadow filtering. **DONE + VERIFIED
      (commit `04cf9eba`).** kSolidLitShadow replaces its single depth-compare
      with a 3x3 kernel: compare the shadow map over a 1-texel neighbourhood
      (texel from `textureDimensions(shadowTex)`, resolution-independent),
      attenuate by `strength * (occluded / 9)`. Invariants: occluded=0 -> fully
      lit, occluded=9 -> fully shadowed (both byte-identical to single-tap),
      0<occluded<9 -> soft penumbra. Verified this turn: naga-validates; SOFT
      EDGES OBSERVED — cast fixture strength-0.6 frame-2 grid shows intermediate
      penumbra cells (floor 98, 119) where single-tap rendered hard 77/140; golden
      8/8 PASS, 0 nonzero diffs (zero-break).

## ⬜ Tier-1 fidelity — remaining
- [~] **T1.3 fog** — the analytic distance-fog FOUNDATION is now in-engine + verified (full volumetric
      froxel scattering is a later tier). `kFogResolve`
      ([render/OmWgpuShaders.cpp](../../src/omnisim/render/OmWgpuShaders.cpp)) — a fullscreen post pass
      sampling a scene-colour + a metric view-distance texture (R32Float, as
      clearAndDrawSceneRangeF32/DepthF32 produce) and blending toward the fog colour by an exponential
      transmittance `1-exp(-density·dist)` — plus `OmWgpuRenderTarget::resolveFog` (4-entry layout:
      FogParams + scene + depth + non-filtering sampler) + a headless probe `OMNISIM_PROBE_FOG`. **DONE +
      VERIFIED on the RTX 3060:** a white scene at 1 m stays clear `(250,251,254)`, at 100 m fogs heavily
      toward the blue fog colour `(57,101,211) = mix(white, fog, 0.865)`, fog-off passes the scene
      through `(255,255,255)`. Golden zero-break (mean-diff/ch=0.000). **Remaining for full T1.3:**
      height/altitude fog (the FogParams `params` slots are reserved), true VOLUMETRIC scattering (froxel
      grid + ray-march), and wiring the pass into the live main view (L3). The reusable analytic
      composite is the foundation.
- [~] **T1.4 TAA** — the temporal-RESOLVE pass is now in-engine + verified (history reproject + 3×3
      neighborhood-AABB clamp + feedback blend + off-screen rejection). `kTaaResolve`
      ([render/OmWgpuShaders.cpp](../../src/omnisim/render/OmWgpuShaders.cpp), the WGSL port of
      taa-preview.html's resolveFS) + `OmWgpuRenderTarget::resolveTaa` (a fullscreen post pass, 4-entry
      layout: TaaParams uniform + cur + hist + filtering sampler) + a headless probe `OMNISIM_PROBE_TAA`
      (8th render-probe sibling). **DONE + VERIFIED on the RTX 3060:** feedback blend
      `mix(white,black,0.9)=(25,25,25)`; the neighborhood clamp suppresses an out-of-range history ghost
      (clamp ON `(0,0,0)` vs OFF `(229,229,229)`); TAA-off passes the current frame through
      `(255,255,255)`; off-screen history rejected `(255,255,255)`. Golden zero-break
      (mean-diff/ch=0.000). **Remaining for full TAA:** (1) the sub-pixel **jitter** half — Halton(2,3)
      8-frame ±0.5px projection offset + per-frame jitter in the scene render; (2) a velocity/
      motion-vector buffer (resolve + accumulator currently take an explicit motion px); (3) the L3
      main-view wiring (`renderMainFrameViaWgpu`). **The ping-pong HISTORY BUFFER half also landed +
      verified** — `OmWgpuRenderTarget::accumulateTaa` (two RGBA8 RenderAttachment|TextureBinding|CopySrc
      buffers swapped each call; the resolve renders into one while sampling the other, building a
      GPU-resident temporal EMA) + `ensureTaaHistory`. `OMNISIM_PROBE_TAA`'s convergence check: seed
      history black, accumulate a white frame repeatedly → the EMA rises **0 → 69 → 250** (converges to
      ~white over ~43 frames, the 8-bit plateau), golden zero-break. **The sub-pixel JITTER math also
      landed + verified** — `OmWgpuSceneRenderer::haltonJitter` (Halton(2,3) 8-frame ±amp px) +
      `jitterViewProj` (a depth-independent, pixel-accurate clip-space shift), verified GPU-free via
      `OMNISIM_PROBE_TAA_JITTER`: the 8-frame sequence spreads 0.813px within ±0.5, and a +4px jitter
      shifts the projected origin exactly +4.000px. **So all three named T1.4 pieces — jitter + history
      buffer + neighborhood clamp — are now in-engine + verified.** What remains is purely WIRING (no new
      algorithms): a velocity/motion-vector buffer (the resolve/accumulator currently take explicit
      motion px) and applying the per-frame jitter + `accumulateTaa` inside the live main-view render
      (`renderMainFrameViaWgpu`), which is L3-owned — an L2→L3 hand-off, not pure L2.

## ⬜ Tier-2 → Tier-5 fidelity — unstarted in-engine
- [ ] **T2.1**
- [ ] **T2.2**
- [ ] **T2.3**
- [ ] **T2.4**
- [ ] **T2.5**
- [ ] **T3**
- [ ] **T4**
- [ ] **T5**
  (Named in §6/§ε as the fidelity ladder; each is its own multi-pass feature,
  "built ONCE, on the right architecture.")

## 🟢 R4 — main viewport on wgpu (the real Phase ζ blocker) — STEPS 1+2+3a DONE
- [x] **R4-step-1: on-screen swapchain/surface bound to a real Win32 window —
      ✅ VISUALLY VERIFIED 2026-06-04 (wgpu-ON build, commit pending).** New
      `OmWgpuSurface` (`src/omnisim/render/OmWgpuSurface.{hpp,cpp}`) creates a
      `WGPUSurface` from an HWND via `wgpuInstanceCreateSurface` +
      `WGPUSurfaceSourceWindowsHWND`, configures it (caps-picked format, Fifo),
      and runs `wgpuSurfaceGetCurrentTexture → clear render pass → wgpuSurfacePresent`
      per frame. Driven by `OmWgpuView` (`src/omnisim/gui/OmWgpuView.{hpp,cpp}`), a
      standalone `QWindow` with `surfaceType=VulkanSurface` (NO GL format → no
      pixel-format clash with WREN's GL on the main view). Opt-in via
      `OMNISIM_VIEW3D_WGPU=1` (spawned from `OmView3D` ctor); default-off ⇒ shipped
      GUI byte-unchanged. Added `instance()`/`adapter()` accessors to
      `OmVulkanBackend`. **Evidence:** with the flag set, a window titled
      "OmniSim — wgpu viewport (R4 step 1)" appears and smoothly cycles RGB; Windows
      also creates wgpu-native's internal "wgpu Device Class" window — i.e. the
      swapchain acquire→clear→present loop runs on the GPU. This answers the
      "can wgpu present a pixel at all" blocker: **yes.** Build needs MSYS `make`
      + a full clean rebuild.
- [x] **R4-step-2: lit, depth-tested 3D rendered into the surface — ✅ VISUALLY +
      NUMERICALLY VERIFIED 2026-06-04 (wgpu-ON build, commit pending).** `OmWgpuSurface`
      gained its own scene pipeline (the `kSolidLit` shader + 256-byte dynamic-offset
      Scene-uniform layout mirrored from `clearAndDrawScene`, but its color target is
      keyed to the SURFACE format — BGRA8UnormSrgb here — since a pipeline's target
      format must match its attachment) + a Depth24Plus attachment, and a `presentScene`
      that renders draws into the acquired backbuffer (no readback) then presents.
      `OmWgpuView` renders a mimosa unit box spinning about +Z, fixed camera at
      (-2.5,0,0) via `OmWgpuSceneRenderer::buildViewProj`. **Numerical self-check
      (`OMNISIM_VIEW3D_WGPU_SELFCHECK=<file>`):** `presentScene(rgbaOut)` copies the
      backbuffer back (CopySrc, when the surface supports it) so two frames at distinct
      rotations are read to CPU and pixel stats written to a file — center px RGB
      (226,220,34)=lit mimosa, corner RGB (63,69,85)=the sRGB-encoded clear, 14% of
      channels differ A↔B (animation). This makes R4 verifiable WITHOUT a human watching.
      **Finding:** the surface is sRGB, so a linear clear `(0.05,0.06,0.09)` displays as
      ~(0.25,0.27,0.33) slate (correct color management; pass a smaller linear value for
      near-black). Remaining: drive the LIVE world geometry + main viewpoint camera (step 3).
- [x] **R4-step-3a: live world from the live Viewpoint camera — ✅ NUMERICALLY VERIFIED
      2026-06-04 (panda.wbt, commit pending).** `OmWgpuView::drawWorld` builds a camera
      matrix from the live main `OmViewpoint` (`buildViewpointCamera`) and renders
      `OmWgpuSceneRenderer::collectWorldDraws` each frame, so the wgpu window mirrors AND
      tracks the WREN main view. The GUI's live WREN GL context makes the WREN-mesh
      readback path usable (unlike the headless camera path). **Self-check (world mode,
      gated on geometry-harvested):** panda → 15 shapes, all draw origins clipw>0 (in
      front), **geometry_px 58.7%**, center pixel a lit arm surface. **Camera-convention
      finding (caught by the clip-projection dump):** a Webots Viewpoint's
      `OmRotation::direction()` is the toward-scene look vector, so `buildViewProj` wants
      `forward = +direction()` (NOT `-direction()`, which put all geometry behind the
      camera, clipw<0). The self-check now also dumps per-draw clip-space projection when
      `OMNISIM_VIEW3D_WGPU_SELFCHECK` is set — reusable R4 instrumentation.
- [x] **R4-step-3b: wgpu view composited INTO the main window — ✅ 2026-06-04 (commit
      pending).** `OmSimulationView` creates `OmWgpuView` and embeds it via
      `QWidget::createWindowContainer` as a splitter pane beside the WREN 3D view (same
      mechanism that hosts WREN). No more floating window — one top-level window, wgpu as a
      live in-app pane rendering the world from the live Viewpoint. Verified: single visible
      top-level window + world-mode self-check (15 draws, present_ok) in the embedded pane.
- [x] **R4-step-3b polish: FOV matches WREN — ✅ 2026-06-04 (commit pending).** Replicated
      `OmViewpoint::updateFieldOfViewY`'s VRML rule (fieldOfView taken on the LARGEST
      dimension) via `horizFovForAspect()`: for a portrait pane the passed horizFov is
      shrunk so `buildViewProj`'s derived vertical fov equals fieldOfView. Verified: a
      160×1122 sliver pane now frames the panda at **60.2%** geometry coverage (was 2.9%).
      Also stabilized the self-check to snapshot after a ~5 s settle (full world load: 222
      shapes), since first-geometry gating fired mid-load on a partial scene.
- [ ] **R4-step-3c (endgame):** make wgpu REPLACE the WREN view as THE main viewport, with
      overlays / picking / manipulator handles / video recorder working on it. The large,
      invasive lift — those all assume WREN's GL context today. **Coupling map + risk-ordered
      plan: [r4-step3c-plan.md](r4-step3c-plan.md).** Headline: main view is 100% WREN with no
      backend seam; only optional-renderings are decoupled. Safe path = build parity ADDITIVELY
      in the wgpu pane (3c-A, breaks nothing), defer the shared-path flip (3c-B) to last, gated
      + reversible, with the determinism/golden net green flag-off. First increment: wgpu
      picking in the pane (3c-A.1).
- [ ] **R4-step-3:** composite the wgpu surface into the main Qt layout / drive the
      real `OmView3D` viewport (vs the current standalone sibling window). This is the
      invasive part — overlays/picking/handles/video-recorder all assume WREN's GL.
- [ ] Route the main 3D view's draw submission through wgpu (replaces `wr_scene_render`).
- [x] ⚠ **Verification:** the GUI binary is windows-subsystem (no console), and GDI
      can't screenshot a GPU swapchain — but R4-step-2's `presentScene` readback
      (`OMNISIM_VIEW3D_WGPU_SELFCHECK`) makes the surface **numerically verifiable**
      (pixel stats → file), so this is no longer human-only. Plan: Phase ε is 3–6 months.

## ⬜ Recognition sensor
- [ ] Per §8.1.1 the plan corrected Recognition as **renderer-agnostic** (not
      gated on wgpu). Confirm/close it as a (small) explicit checklist item.

## ⬜ Phase ζ — "architecturally complete" (plan: 1–2 months after ε)
- [ ] Flip `renderBackend` default `"wren"` → `"wgpu"` (WREN demoted to fallback).
- [ ] Golden-image parity wired into CI as a standing gate (harness exists but is
      not gated; CI workflows currently disabled).
- [ ] Every Tier 1–5 feature shipped on **both** WREN (interim) and wgpu
      (canonical).
- [ ] 64-camera training scene @ 60 FPS on consumer hardware (perf acceptance).
- [ ] 2×2 build-matrix safety net still bit-identical with both flags OFF.

---

**Near-term headless-verifiable runway:** CSM 3b → 3c → 3d → PCF, then T1.4 TAA.
**Multi-month / human-or-GPU-CI-gated tail:** T1.3, T2–T5, R4, Recognition close,
Phase ζ flip + CI + perf.
