# wgpu renderer — current state

*Post-deletion reference, re-verified against the tree on 2026-08-24. This document was a
pre-deletion snapshot until that date and asserted four things that are now false (WREN as the
per-world opt-out, WREN as the Camera-device default, an automatic WREN fallback, and WREN as a
selectable backend at all). Those claims are gone; what still explains something is preserved,
dated, under [History](#history--how-we-got-here).*

**⛔ THERE IS EXACTLY ONE RENDERER, AND WREN IS NOT IT. WREN WAS DELETED ON 2026-08-23**
(`976b9449d`, *"D1.4 — THE ATOMIC WREN DELETION"*: `src/wren` (106 files, 19,847 lines),
`include/wren` (33 files), `src/omnisim/wren` (62 files) and the 86-shader tree — **456 files,
−47,533 lines, one commit**), followed by `1c4f1b413` (D1.5), which retired the hatches and the
comparison tooling's WREN arms. wgpu-native draws the main view, every `Camera` / `RangeFinder` /
`Lidar` device, every screenshot, the capture/cinema pipeline and the `--stream` mjpeg feed.

**⚠️ There is no fallback tier, and that is a deliberate, owner-assumed risk (R8).** A host whose
wgpu-native cannot initialise gets the still-non-null wgpu backend with `isAvailable()` false;
every render entry point branches on that and produces **no frames**, after one loud log line
naming the cause. Physics, controllers and the supervisor are unaffected — the session runs, it
just does not draw ([`OmRenderBackend.cpp:146`](../../src/omnisim/render/OmRenderBackend.cpp)).
Before D1.4 that path resolved to WREN; there is nothing to resolve to now.

The campaign that got here: [wren-retirement-plan.md](wren-retirement-plan.md) (the audit, the
corrections, the what-would-a-user-lose table) and
[wren-deletion-runbook.md](wren-deletion-runbook.md) (the ordered work list, the per-step gates,
the post-deletion endgame pass). **When this document and the runbook disagree, the runbook is the
measurement.**

---

## `"wren"` in a world file: parses, warns, renders wgpu

Per the `Radio` / `Microphone` / `Solid.immersionProperties` precedent, an **undeclared** field is
a hard load ERROR that takes a headless run's exit code to 1 — so the fields keep parsing and
legacy worlds keep loading. What changed at F1 (`35148ad65` + `76a8cb6e2` + `8087caffd`,
2026-08-23) is that none of them selects a renderer any more.

| surface | default | `"wren"` today |
|---|---|---|
| [`Viewpoint.renderBackend`](../../resources/nodes/Viewpoint.wrl) | `"wgpu"` (since 2026-08-19) | parses; warns once per node; the main view renders wgpu |
| [`Camera.renderBackend`](../../resources/nodes/Camera.wrl) | `"wgpu"` (since 2026-08-23) | parses; warns once per node, **by device name**; renders wgpu |
| [`RangeFinder.renderBackend`](../../resources/nodes/RangeFinder.wrl) / [`Lidar.renderBackend`](../../resources/nodes/Lidar.wrl) | `"wgpu"` (fields added in Lane E1, flipped 2026-08-23) | same |
| `WorldInfo.defaultRenderBackend` | `""` (inert) | parses; each consuming node warns once; it can no longer pin a world to anything |
| `OMNISIM_FORCE_WREN`, `OMNISIM_LEGACY` (render arm) | unset | **RETIRED AND IGNORED**, one warning each naming itself ([`OmRenderBackend.cpp:76`](../../src/omnisim/render/OmRenderBackend.cpp)) |

`"vulkan"` remains an accepted alias of `"wgpu"` — the enum member and the class names kept the
historical spelling ([`OmRenderBackend.cpp:39`](../../src/omnisim/render/OmRenderBackend.cpp)).
An empty or `"auto"` value deliberately takes the **wgpu** arm, not the Wren arm, so a world that
never wrote `"wren"` never fires the retirement warning.

⚠️ **The retired selectors warn rather than being silently dropped for one specific reason: a wrong
oracle is worse than a lost one.** An A/B harness that exports `OMNISIM_FORCE_WREN`, believes it
captured a WREN arm, and publishes a wgpu render as a WREN reference is the failure this warning
exists to prevent. Verified at F1: a Camera authoring `renderBackend "wren"` produces an image
**byte-identical** (same 96×72 PPM, checksum 261558207) to its unmodified `"wgpu"` sibling — the
`"wren"` request measurably rendered wgpu.

## Retired **with** the deletion — each degrades loudly, none silently

These are capabilities WREN uniquely implemented and nothing re-implemented. Every one keeps its
field parsing and announces itself.

- **Camera-recognition SEGMENTATION image** — produced nowhere; the buffer stays black and the
  device warns once by name ([`OmCamera.cpp:1947`](../../src/omnisim/nodes/OmCamera.cpp)). ✅ The
  recognition **object list** is unaffected — it rides Newton rays, not a render. The stream
  framing and the memory-mapped file survive, so the controller-side protocol is unbroken.
- **Non-planar (cylindrical / spherical) `Camera` and `RangeFinder` projections** — `buildViewProj`
  is a planar pinhole (`tan(fov/2)`; `tan(π)≈0` degenerates), so the device **DECLINES**, warns
  once by name, and leaves an honestly-empty buffer: `(0,0,0)` for a Camera, `0.0` at every pixel
  for a RangeFinder — *not* a distance and *not* the `+inf` miss contract; nothing was rendered at
  all. ✅ **Lidar's cylindrical pipeline is native wgpu and is fine.** Re-pinned in `eb11249fd`
  with every old WREN-era expectation recorded in the test source; the one authored non-planar
  world outside `tests/` (`spherical_camera.omniworld`) is kept deliberately as the warned
  showcase.
- **Remote-extern camera streaming** — it streamed straight out of the WREN camera and has no wgpu
  equivalent. The resized stream region is left untouched and the device warns
  (`warnWrenlessNoImage`, [`OmAbstractCamera.cpp:429`](../../src/omnisim/nodes/OmAbstractCamera.cpp))
  rather than shipping the caller whatever was in that buffer.
- **Camera post-FX, partially:** `lens`, `lensFlare`, `focus` and `noiseMaskUrl` are retired with
  named per-device warnings (Lane E1). ✅ `noise` and `motionBlur` were **ported** as CPU post-FX
  on the wgpu readback; `OMNISIM_WGPU_SENSOR_POSTFX=0` stands the port down and restores the
  pre-port behaviour (effect absent, warning back).
- **`Background.*IrradianceUrl`** — retired at P4a (`3d1020907`); fields parse, one warning per
  node by name. ✅ `Background.luminosity` and `PBRAppearance.IBLStrength` were **ported** at P4
  (`c3c990b7c`) and scale the wgpu diffuse *and* specular ambient exactly as `pbr.frag` did.
- **Also degraded, each with a code comment:** resize/scale drag handles, the propeller fast/slow
  helix visual switch, and billboard camera-facing.

## What survives of WREN, and why

- **[`resources/wren/`](../../resources/wren/) — 10 files, NOT deletable.** `meshes/arrow.obj` and
  `meshes/circular_arrow.obj` are read by the **wgpu** gizmo
  ([`OmGizmoLines.cpp`](../../src/omnisim/gui/OmGizmoLines.cpp)), which logs at INFO and continues
  when they are missing — so deleting them silently restores invisible-but-draggable gizmos, the
  exact regression P8 fixed. The six `*_close/resize_symbol.png` back the device-HUD chrome (P7),
  `muscle.png` backs the ported `Muscle` surface (P2), and `LICENSE` is the vendored-asset licence
  covering all of them. The `gl:` search paths still resolve here.
- **[`tests/rendering/wren_reference/`](../../tests/rendering/wren_reference/) — the frozen
  reference set (coupling C9).** Two 1280×720 stills captured 2026-08-23 on machine
  `9722d23d12a3` (RTX 3060 laptop) immediately before F1 made WREN unselectable: `beauty_bench`
  (the LOOK benchmark; static, noise floor 0) and `warehouse_industrial` (flagship; conveyor Track
  texture scroll, PBR + legacy materials mixed). ⚠️ **Read them as "what changed relative to
  WREN", never as ground truth** — the wgpu renderer is *deliberately* different (HDR+AgX, GTAO,
  PCSS, SSR, scattered sky). They exist because the comparison tooling lost its reference arm.
- **The `--stream` w3d browser viewer keeps its own WREN.** `resources/web/wwi/` vendors
  `wrenjs.{js,wasm,data}` and a `wren/` JS tree; it is a curl'd artefact that was never built here,
  so the engine deletion does not touch it. **Consequence, accepted deliberately and not a bug to
  fix: the browser-side w3d view permanently diverges in look from the wgpu desktop view.** (The
  `--stream` **mjpeg** feed is a different thing and *is* wgpu — it is fed from the wgpu frame
  tail.)
- **Two rescued non-WREN classes kept their names.** `OmWrenRenderingContext` owns the `VF_*`
  optional-rendering enum that `OmWgpuView` reads every wgpu frame; `OmWrenOpenGlContext` is a
  plain `QOpenGLContext` subclass backing the GL present blit and device pop-outs. Both live in
  [`src/omnisim/render/`](../../src/omnisim/render/) now and contain no WREN.

## Build

A plain `make release` is wgpu-ON on any box that has run
[`scripts/dev/setup_wgpu_native.sh`](../../scripts/dev/setup_wgpu_native.sh) — `OMNISIM_WITH_VULKAN`
defaults to ON and `WGPU_NATIVE_HOME` is auto-discovered from `_scratch/wgpu-native`
([`Makefile:346,363-379`](../../src/omnisim/Makefile)); `wgpu_native.dll` ships automatically next
to the binary via the post-link `EXTRA_CMD`. ⚠️ **An explicit empty `WGPU_NATIVE_HOME=` no longer
selects "the other renderer" — it selects a binary with no renderer at all.** Install recipe:
[wgpu-native-setup.md](wgpu-native-setup.md) (⚠️ its own 2026-05-28 status banner predates both the
flip and the deletion — read it for the install steps, not for the status).

`OMNISIM_GL_OPTIONAL` is retired: since D1.5 an OpenGL failure is **one warning, not a fatal**.
What is lost is named exactly — the GL present/blit fallback, device pop-out windows and GPU pass
timers — while wgpu rendering and compute continue
([`OmGlWindow.cpp:76`](../../src/omnisim/gui/OmGlWindow.cpp)). The old fatals killed sessions the
wgpu renderer could serve fine.

## What the main view renders today

Verified against the source on 2026-08-24. Everything below is **default ON** unless marked.

- **Tone + exposure:** linear-light HDR + AgX tonemap, driven by `Viewpoint.exposure`
  (`2ea060e86` — the "milky AgX" verdict was diagnosed as display-referred shading underneath).
  `OMNISIM_WGPU_AGX=<float>` overrides the exposure; `=0` selects the untouched LDR path
  bit-exactly. `OMNISIM_WGPU_TONEMAP_CURVE=1` is WREN's exact 1-exp curve on the same pipeline.
- **Shadows:** cascaded shadow maps, merged into the one textured-shadow render, with **PCSS**
  contact-hardening (`7132a2b50` — a 16-tap blocker search sets the PCF spread from the
  receiver–blocker gap in metres, so contact edges stay razor-sharp and shadows soften with caster
  distance; no-blocker pixels skip the 25-tap PCF entirely), receiver-plane depth bias and
  normal-offset receivers. `OMNISIM_WGPU_NO_CSM=1` reverts to the legacy single 45 m fitted map;
  `OMNISIM_WGPU_PCSS=0` reverts bit-exactly (`OMNISIM_WGPU_PCSS_SOFT` calibrates, default 0.02).
- **AO:** GTAO, ported from WREN's own `gtao.frag` (`761a5b8e1`) and re-based onto the scene's own
  MSAA depth so the AO geometry prepass is gone (`71252cecc`).
  `Viewpoint.ambientOcclusionRadius` scales the tap spread (radius/2; default 2 unchanged).
  `OMNISIM_WGPU_GTAO=0` reverts to the legacy curvature kernel.
- **Sky:** a Hillaire-class single-scatter atmosphere on the HDR path (Rayleigh + Mie + ozone, a
  sun disc whose spectrum is the transmitted illuminance so it reddens at the horizon by physics,
  a ground-albedo term, and the night kit — stars, Milky Way, moon), marched into a 128×64
  sky-view LUT + 128×1 transmittance strip re-baked only when the sun moves, so the dome pays
  **one texture sample per pixel** (`a3c360d33`). Phase 2 feeds the lighting: a CPU port of the
  same march derives the hemisphere ambient and the analytic-IBL env palette, so shadow fill and
  metal reflections track the actual sky — golden hour and the Mars palette come out of scattering
  parameters, and dragging the sun marker moves the whole light rig. The dome draws **after** the
  opaque span, depth-tested at the far plane, so all sky math costs only visible-sky pixels
  (`75ad8dbf3`). Image-cubemap backgrounds (`Background`'s six `*Url` faces) keep working; keying
  is preset > cubemap > flat clear. Hatches: `OMNISIM_WGPU_SKY_SCATTER=0`,
  `OMNISIM_WGPU_SKY_DERIVED=0`, `OMNISIM_WGPU_SKY_ILLUM`.
- **Clouds:** a 2-scale fBm sheet at ~1.5 km lit by the atmosphere's own transmittance sampled at
  the sun's elevation — amber at sunset, dark at night, no palette (`75ad8dbf3`). Static; wind
  needs a time uniform. `OMNISIM_WGPU_CLOUDS=0`, `OMNISIM_WGPU_CLOUD_COVER` (default 0.4).
- **Reflections:** screen-space reflections on the reversed-Z HDR path — the lit shader's HDR alpha
  carries reflectivity, and one combine pass between resolve and tonemap reconstructs position +
  normal from the scene's own MSAA depth (no G-buffer), marches with jitter + 4-step binary
  refinement, rejects behind-geometry leaks by refined-hit distance, and adds the Fresnel-weighted
  hit colour; a miss keeps the analytic IBL (`7132a2b50`). ⚠️ **Known v1 limits:** curved surfaces
  show chunky noise at close range (depth-derived normals + per-pixel jitter, no roughness blur),
  and translucent glass keeps analytic IBL only (its alpha is real alpha — the blend equation needs
  it). `OMNISIM_WGPU_SSR=0` skips bit-exactly; `OMNISIM_WGPU_SSR_STRENGTH` scales.
- **Global illumination:** OmniLight — a baked path-traced probe volume costing one trilinear
  volume sample per frame. See [omnilight.md](omnilight.md); `OMNILIGHT=0` is the value-parsed
  revert, `OMNILIGHT_RAYS` / `OMNILIGHT_SPACING` / `OMNILIGHT_SCALE` tune it. Volumetrics ride the
  same volume (`OMNISIM_WGPU_VOLUMETRIC=0`). The most recent falloff correction is written up in
  [wgpu-shadow-aura.md](wgpu-shadow-aura.md), and the misattribution there is the more useful half
  of the story.
- **AA:** MSAA 4× plus a **wired** TAA resolve (`OMNISIM_WGPU_TAA=0` skips), ±½-LSB dither.
- **Transparency** (`490a7b5b0`): sorted src-over alpha blending — PBRAppearance and plain-Material
  transparency; translucents neither cast shadows nor darken AO.
- **Lights:** the sun (the only shadow-caster) plus up to 8 extra lights — PointLight / SpotLight /
  additional DirectionalLights as unshadowed fills honouring `on`, `intensity`, colour,
  attenuation, radius and spot cones (`e6a0c09da`). Single-sun worlds re-rendered
  **byte-identical** (sha256) when that landed. The sun's `intensity` / `on` / colour drive real
  energy in the HDR arm.
- **Materials:** the full PBR ladder (albedo/roughness/metalness/normal maps, Cook-Torrance GGX,
  sRGB colour management, exact 2D-affine `TextureTransform`), legacy `Appearance` textures (P11),
  plain-Material `emissiveColor` read in **both** collector branches (the LED fix), and the
  WREN-exact fog curve mixed in linear space.
- **Geometry:** reversed-Z float depth (far-field z-fighting structurally gone), authored near
  plane honoured, `castShadows FALSE` honoured, bounding-sphere shadow + frustum culling
  (`d28d2c349`, `OMNISIM_WGPU_NO_CULL`). Deformables — `Cloth` / `SoftBody` — collect **and
  animate** (`d5897ff7d`); `GranularGroup` particles (P9), `Muscle` and `Track` surfaces (P2), and
  — since `3cbbfd7a5` (W1d) — `ElevationGrid` and `Cone`, whose meshes are generated CPU-side
  mirroring the deleted WREN builders verbatim.
- **Editor + GUI, now native:** the main-view mouse hit test is
  [`OmScenePicker`](../../src/omnisim/gui/OmScenePicker.cpp) — one depth-tested wgpu ID render over
  `collectWorldDraws` with the gizmo handle triangles appended as transient draws, so **drawn ==
  draggable is one triangle set**; world coordinates come from a CPU ray–triangle intersection
  against the picked solid's retained vertices (selection granularity is the collector's per-draw
  Solid — a recorded deviation). Selection tint + AABB outline and the Optional Rendering items
  draw through the W4a overlay-line pass, last, after tonemap, into a private output so TAA can
  neither recolour them nor keep them in history (`OMNISIM_WGPU_OVERLAYS=0`). Device HUD insets,
  supervisor labels and status overlays draw through `OmHudOverlay` on the same rect the hit tests
  use (`OMNISIM_WGPU_HUD=0`); the translate/rotate gizmo computes its handle matrices engine-side
  (`OMNISIM_WGPU_GIZMO=0`). Screenshots re-render one synchronous frame with readback
  (`777ef7836`), and movie recording feeds from that same path (P10, `3c4a55302`).

## What a `Camera` / `RangeFinder` / `Lidar` renders today

⚠️ **The old "the sensor path is untextured / flat-lit / grey-clear by contract" line is now the
`OMNISIM_WGPU_CAMERA_SCENE=0` ARM, not the default.** The default sensor render harvests the
world's own materials (albedo/roughness/metalness/normal, uploaded by the collect through the
device's texture cache), its own `Background`, its own lights, its own `Fog`, and a cast-shadow
pass — from exactly the nodes the main view harvests them from, so a Camera device and the
viewport look at one lighting model rather than two
([`OmCamera.cpp:942`](../../src/omnisim/nodes/OmCamera.cpp)).

**Every camera-family device now sets up WRENLESS.** There is no `OmWrenCamera`, no WREN frustum
and no per-device GL requirement, so camera images work windowed, under `--minimize`, under
`OMNISIM_NO_WINDOW=1`, and with no GL context at all
([`OmAbstractCamera.cpp:272`](../../src/omnisim/nodes/OmAbstractCamera.cpp)). Verified at Lane E5:
the `OMNISIM_NO_WINDOW=1` camera checksum is byte-identical to the windowed run.

⚠️ **The output contract is NOT the main view's, deliberately** (`OM_WGPU_XFER_WREN_SENSOR`). A
Camera device is not a display surface: WREN's colour Camera applied `1-exp(-c·exposure)` then
`pow(., 1/2.2)`, and `tests/api/controllers/camera_checker.c` pins those **encoded** bytes to ±2.
So the sensor render reproduces that curve — it must **not** emit linear light, and must **not**
take the main view's AgX, vignette, grain or auto-exposure (a spatial gain, a time-varying term,
and a term that would make this frame's bytes a function of the previous frame). **Cost:** the
sensor frame is genuinely bigger than the old flat one — a light-depth pass, a 2048² shadow map,
MSAA 4×, an RGBA16F target and a tonemap pass. On a 64×48 probe camera that is noise; on a
1280×720 Camera stepping every basic timestep it is not. `OMNISIM_WGPU_CAMERA_SCENE=0` is the way
back to the cheap flat render.

Two authoring traps fixed in the same pass and worth knowing: `Camera.far` defaults to `0.0`
meaning *infinite*, and the wgpu projection now substitutes WREN's exact 10 km plane (feeding the
raw 0 to `perspective()` gives `m22 = m32 = 0`, so `clip.z ≡ 0` and visibility becomes draw-order
dependent); and `RangeFinder` / `Lidar` return the documented `+inf` on a miss, not a clamp to
`maxRange`.

## Known gaps

| gap | status / impact |
|---|---|
| **`PointSet` / `IndexedLineSet` do not draw** | No wgpu point/line-primitive pipeline exists (topology + a per-vertex colour attribute); the collector's triangle paths cannot represent per-vertex `Color`. `tests/api` `point_set` is one of the two **decided** post-deletion reds. |
| **`camera_revert`** | The other decided red: the wgpu/AgX camera compresses a pure-red box's channel dominance below the test's `>3×` classifier, so the RED blob is never classified. The old reload-loop *hang* is gone — it now fails by name inside a 400-step budget. Owner: whoever owns the sensor look. |
| **⚠️ R8 — wgpu-native on every supported platform is OWNER-ASSUMED, not verified** | With WREN gone there is no third backend, so this is the assumption the whole deletion rests on. Native X11 / XCB / Wayland surface sources exist (`51a1f9f44`, *"so Linux can present without WREN"*) and the GL readback+blit covers a platform with no native surface — but **this box cannot compile the Linux branch** (`QT_FEATURE_xcb -1`), so none of it is verified *on* Linux. If the assumption is withdrawn, R8 goes red regardless of everything else. |
| **NUE (Y-up) worlds render rolled** | Pre-existing from the 2026-08-19 flip: a Y-up authored `Viewpoint` under the Z-up wgpu camera. Measured into the movie path at E5 — two *different* NUE worlds produced **byte-identical** sky-only mp4s. The api movie test only asserts that the file exists, so it stays green through this. |
| **P14 — `city.omniworld` `sun_marker` handshake** | The one open **post-deletion** defect. Deterministic, corpus-only, misclassified by libController as an ABI mismatch and proven by differentials to be a load-timing race (`omnisim doctor` says the pair is compatible; the 8-controller husky swarm passes; a 6.6 KB world with the same marker PROTO passes). |
| **P13 — back-to-back-spawn wgpu abort** | Pre-existing and **A/B-attributed as independent of this campaign**: a wgpu-native panic at `conv.rs:1500` → non-unwinding abort inside `wgpuDeviceCreateBindGroup`, exit `0xC0000409`, triggered by back-to-back engine spawning. ⚠️ `test_suite.py` prints "Test suite complete" even after the abort, so a truncated run looks whole. |
| **`Pen` paint on Box / Cylinder / Cone** | Cross-atlas UV: an accepted deviation with a one-shot warning (the stride fix would have added WREN C-API surface immediately before D1.4 deleted it). Planes and meshes paint correctly. |
| **GUI gates that need a human** | Gizmo drag parity, HUD drag/resize/close across a reload, `BoundingSphere` and the force/torque drag arrows (both selection- or drag-gated, and the tree has no programmatic selection surface), the GPU-memory dialog. Recorded as checklist results, never as claims. |
| **No CI for renderer pixels** | Still true, and it is exactly how the device HUD stayed dark for days after the 2026-08-19 flip. The frozen reference set above is the cheap durable fix; nothing runs it automatically. |
| **GPU-memory readout** | The wgpu C API exposes **no** GPU-memory query (verified against the vendored headers), so the honest substitute prints the adapter identity and says the figure is unavailable. No branch can print a silent `0 MB`. |

## Diagnostic env levers

All off by default and zero-cost when unset. ⚠️ **Mixed gating — check before you trust a `=0`:**
`NO_SWAP`, `NO_REVZ` and `REPORT` are **presence-gated** (setting them at all arms them; unset to
disarm). The feature hatches added during the retirement campaign are **value-parsed** by rule
(`FOO=0` means OFF — never presence-gated; the `OMNISIM_REQUIRE_NEWTON` trap).

| env | effect |
|---|---|
| `OMNISIM_WGPU_REPORT=<file>` | per-100-frame stats: draws, `collectMs`, `renderMs`, `prevBlitMs`, `maxGapMs`, resource counts, `glArms`, and the overlay telemetry `ovCalled` / `ovBatches` / `ovVerts` / `ovOk`. ⚠️ `renderMs` is CPU encode+submit, **not** GPU time |
| `OMNISIM_WGPU_MAINVIEW_DUMP=<png>` (+ `_FRAME=<n>`) | dump a main-view frame (forces the readback path). ⚠️ Blind to overlays **by design** — the readback happens inside the scene render, before the overlay pass |
| `OMNISIM_OPTIONAL_RENDERING=<CSV of View-menu names>` | arm optional renderings with no GUI (the 15 `VF_*` flags only; name-based global items such as `BoundingSphere` are not covered) |
| `OMNISIM_WGPU_AGX=<exposure>` | override the HDR/AgX exposure; `=0` selects the LDR path bit-exactly |
| `OMNISIM_WGPU_NO_CSM=1` / `PCSS=0` / `SSR=0` / `SKY_SCATTER=0` / `SKY_DERIVED=0` / `CLOUDS=0` / `GTAO=0` / `TAA=0` / `VOLUMETRIC=0` / `OMNILIGHT=0` | per-feature exact-revert hatches (value-parsed) |
| `OMNISIM_WGPU_OVERLAYS=0` / `HUD=0` / `GIZMO=0` | restore the pre-W4a / pre-P7 / pre-P8 frame exactly |
| `OMNISIM_WGPU_NO_SWAP` / `NO_REVZ` / `NO_SSAO` / `NO_SHADOW` / `NO_CULL` | kill-switches for bisecting (`NO_SWAP` reverts to readback + GL blit present) |
| `OMNISIM_WGPU_SENSOR_DYNAMIC=0` | sensors collect Solids only — the exact pre-P1 sensor image; main view untouched |
| `OMNISIM_WGPU_DEFORMABLE_EPOCH=0` | restore the global-edge upload rule, i.e. **reproduce the frozen-deformable bug**. For proving the instrument can go red, never for shipping |
| `OMNISIM_WGPU_CAMERA_SCENE=0` / `SENSOR_POSTFX=0` / `CAMERA_NO_GL=0` | sensor-side exact-revert arms (flat render / no CPU post-FX / restore the wrenless-setup fatal) |
| `OMNISIM_WGPU_SHADOWMAP_DUMP=<png>`, `SHADOW_DEBUG=1`, `CSM_DIAG`, `DRAW_DIAG` | shadow and draw-path diagnostics |
| `OMNISIM_WGPU_ERRLOG` / `TEXLOG` / `INITLOG=<file>` | uncaptured-error + draw accounting / texture-cache LRU / backend init stage (adapter + device acquisition) |
| `OMNISIM_PROBE_PICK/READBACK/LINE/TEX/INSET/CSM/TAA/…=<file>` | headless feature probes (run via `--help`) |

**Retired levers — set one and you get a warning, not an effect:** `OMNISIM_FORCE_WREN`,
`OMNISIM_LEGACY` (render arm), `OMNISIM_WGPU_MAINVIEW_FORCE` (there is nothing left to force —
every resolution already lands on wgpu, and when wgpu-native is unavailable forcing it never
worked), `OMNISIM_WGPU_MAINVIEW_CSM` (cascades are the default; `NO_CSM=1` is the revert).
Retired at D1.5 because their `=0` arm selected deleted code: `OMNISIM_WGPU_NATIVE_MESH` /
`_PRIMITIVES` / `_CADSHAPE`, `OMNISIM_LIDAR_WGPU`, `OMNISIM_RANGEFINDER_WGPU`,
`OMNISIM_WREN_POSTFX`, `OMNISIM_WGPU_VIDEO`, `OMNISIM_NEWTON_SKIP_WREN`, `OMNISIM_GL_OPTIONAL`.

## Instruments — and what each one is blind to

- ⛔ **[`render_oracle.py`](../../scripts/dev/render_oracle.py) is no longer a parity oracle.** Its
  mechanism was "render WREN right here and use its framebuffer as the local golden". With WREN
  retired, **both arms render wgpu** — it and [`render_ab.py`](../../scripts/dev/render_ab.py),
  `render_quality.py`, `reversibility_check.py`, `sensor_oracle.py` and
  `wgpu_sensor_regression.py` all carry that retirement notice at the top of the file (D1.5). The
  tools are kept because their wgpu arms and the A/B harness remain useful; a *cross-renderer*
  comparison now means diffing against `tests/rendering/wren_reference/`.
- **`render_ab.py`** is the campaign's verification gate: render one world twice, arms differing
  only by environment, diff pixel-wise. ⚠️ It is **structurally blind to overlays** (same
  readback-before-the-overlay-pass reason as `MAINVIEW_DUMP`), so a `MATCH` verdict there is the
  instrument's contract, not evidence of absence.
- ⭐ **Always measure the noise floor before reading a diff.** On an animated world the floor can
  exceed the signal — measured on `warehouse_husky`, an overlay "signal" of 989 px over threshold
  against a same-arm-twice noise floor of **1060**. On a static sibling (the same world with
  `controller "<none>"`) the floor is exactly **0** (bit-identical frames), which is what makes a
  pixel A/B mean anything at all.
- **`draws=` counts the SCENE draw list only.** An overlays-on vs overlays-off A/B reported
  `draws=1052` in both arms and that identity meant precisely nothing. The overlay pass has its
  own telemetry: `ovCalled=2` distinguishes "reached the draw site with zero geometry" from "never
  reached it", and `ovOk` is `drawOverlayLines`' own return value.
- **`python scripts/dev/wren_deletion_audit.py`** now reads **`DELETION-READY (0 blocking, 0
  retirable)`**, down from 322 at the post-merge pass. ⚠️ It scans **git-tracked files only**, and
  its dead-include detector tests `wr_*` *symbols* — it cannot see `Wr*` struct or `WR_*` enum
  usage, which produced five false positives during the campaign. No include may be deleted on its
  word alone; every one needs a compile.

## Measured performance

⛔ **EVERY `WREN` COLUMN BELOW IS HISTORY AND CANNOT BE RE-RUN.** WREN was deleted on 2026-08-23;
these are what they measured on the date they name, not a backend you can A/B today. The frozen
reference stills are the only surviving cross-renderer artefact. Machine `9722d23d12a3` is an RTX
3060 laptop throughout, except where noted.

**2026-08-19, machine `9722d23d12a3`, city world, GUI, 75 s arms**, after the render-throttle fix
(`51a67446f` — the old throttle charged render time on top of the FPS budget and step-quantized
*both* renderers to half the authored rate):

| metric | WREN (historical) | wgpu |
|---|---|---|
| REALTIME head-to-head: main-view FPS | 20.0 | **23.2** |
| REALTIME head-to-head: speed factor | 0.648× | **0.858×** |
| FAST mode mainFPS (pre-fix both ~9.4) | 21.3 | ~15.6\* |
| Render cost / frame | 35.3 ms | **13–15 ms** |

\* FAST-mode FPS is confounded by sim-speed coupling (cheaper renders → more steps/s → fewer render
opportunities); REALTIME is the demo-experience row. **The June "whole-sim FPS tie" was retracted
as a throttle artifact**: with the fix, wgpu ran the city both smoother and 32% closer to realtime,
because its render was ~2.4× cheaper.

**2026-08-19 evening, machine `9722d23d12a3`, same-session interleaved arms:**

| city_traffic derivative | WREN (historical) | wgpu (full quality stack) |
|---|---|---|
| RENDER-BOUND (controllers idled, basicTimeStep 8, FPS 120): mainFPS | 40.2 | **75.4** (1.88×) |
| RENDER-BOUND: speed factor | 0.325× | **0.625×** |
| Standard city: render cost / frame | 21.5–35 ms (day-night phase) | **12 ms** — with HDR+AgX+GTAO+MSAA+bloom ON |
| Standard city REALTIME (unloaded box): mainFPS | 25.1 @ 0.873× | 25.2 @ 0.879× (both saturate the ~31/s step-cadence ceiling; wgpu does it with ~2× less thread time — city controller wall 21–25 ms → 12–13 ms) |

**2026-08-20, machine `9722d23d12a3`** — the post-flip render-cost campaign, every step gated on a
**pixel-identical** Beauty Bench main-view dump (HDR + 3-cascade CSM + GTAO + bloom active) before
commit. `renderMs` on `city_traffic` (4,587 draws, 1896×1113): **12 → 7–9.**

1. **Per-frame uniform staging killed** (`cf3441550`): the texshadow path rebuilt and uploaded
   ~4.7 MB of uniforms every frame — pass 1 staged (cascades × draws × 256 B) slots each carrying a
   copied VP + model, and pass 2 rebuilt every slot because three per-frame-shared values were
   copied into each one. Those moved into `LightU` (960 → 1056 B, appended); the Scene slots moved
   to a texshadow-owned buffer with a persistent host mirror, memcmp delta detection and coalesced
   ranged uploads (**zero bytes/frame on a static scene**). Measured 12 → 8–9; the
   `OMNISIM_WGPU_STAGING_DELTA=0` control arm is statistically identical, so the win is the pass-1
   kill + the `LightU` hoist.
2. **Encode-call thinning** (`eb110ef8d`): the Scene binding flipped from a dynamic-offset uniform
   to a read-only storage array of the same 256-B slots, indexed in-shader by `instance_index`.
   Pass 1 now binds **once per cascade** (~13.8k `SetBindGroup`+offset calls gone at 3 cascades on
   the city) and pass 2 rebinds only when the 4-texture bind group changes. Measured 7–9 across two
   runs vs 8–9 — the increment is inside run-to-run thermal noise; the call-count reduction is
   structural.

The scattering sky, PCSS + SSR, and the sky-pixels-only pass each landed inside that band: city
`renderMs` **7 steady** with the whole stack on, **7–8 across two runs** with PCSS and SSR both on.

**2026-08-21 retirement-campaign baselines, machine `9722d23d12a3`** — the numbers every later
phase was gated against: `city_traffic` main view **4,585 draws, renderMs 9, collectMs 1**; Beauty
Bench render-oracle within-tol **45%**, coverage **77.1%**; `construction_site_dev` within-tol
**27%**, coverage **100.0%**. ⚠️ **Coverage is the load-bearing number** (it detects geometry
vanishing); within-tol is advisory and legitimately low, because the wgpu path has gained scattered
sky, GI, PCSS, SSR and fog that WREN never had. The standing rule was *no phase may regress city
`renderMs` below the baseline 9*; W1c measured 5–10 against it and W4a 6–10. **No post-deletion
city `renderMs` has been recorded** — the endgame pass verified correctness, not throughput.

⚠️ **The oldest table this document carries, 2026-06-11, does not name its machine.** It is kept
because it is a real measurement, with that defect stated rather than papered over: same world
(`city_traffic` copy), same viewpoint, ~1,400 steps each at `--mode=realtime`.

| metric (2026-06-11, machine not recorded) | WREN (historical) | wgpu |
|---|---|---|
| Render cost / frame @1896×1113 (3,523 draws) | 21.5 ms | **8–9 ms** (~2.4× faster) |
| Whole-sim FPS, full traffic sim | 14.0 | 14.3 (tie — world is sim-bound) |
| Render-bound FPS (controllers idle) | **30.8** | ~25 (app-loop pacing gap, later root-caused to the throttle) |
| Frame pacing | n/a | maxGap 46–79 ms typical after Mailbox present |

**Lighting interpretation diverged deliberately**, and that is why a WREN diff was never a quality
score: the city is authored as full day, wgpu rendered it that way, WREN rendered it markedly dark
("WREN is legacy-dark"). Any `within-tol` figure measured distance from WREN's legacy-dark output.

---

## History — how we got here

*Kept because the reasoning is reusable, not because any of it is current. Every entry below
describes a state that has since been superseded.*

**2026-06-11 — decision: WREN stays the default.** The flip was human-gated on a significant,
meaningful, eye-visible difference in wgpu's favour.

**2026-08-19 — the main-view flip (`adf2aa075`).** `Viewpoint.wrl` went to `"wgpu"` on an explicit
user decision, on the strength of the re-baselined table above. Accepted-with-flip caveats at the
time: metals lacked specular IBL, CSM crispness was unbuilt, and movie recording still captured
WREN frames — all since closed. ⚠️ **It also shipped five undisclosed GUI regressions** that went
unnoticed for days: selection produced no visual feedback, all 21 View→Optional Rendering items
toggled flags nothing read, every device HUD inset and supervisor label was invisible, and gizmos
were invisible **but still draggable**. All are fixed (W4a/W4b, P7, P8, `OmScenePicker`); the
lesson that outlived them is that there was no automated verification path for that entire class of
pixels, which is exactly why `OMNISIM_OPTIONAL_RENDERING` and the `ov*` telemetry now exist.

**2026-08-21 — the retirement campaign opens.** Same shape as the ODE deletion: measured audit →
parity phases with gates → default flips with exact-revert hatches → one deletion commit at the
end. Baselines: **3,136** `wr_*` call sites in `src/omnisim`, **100** files in `src/wren`, **264**
engine files touching WREN.

**2026-08-22 — the Camera flip that was reverted the same day (W3).** `Camera.wrl` went
`"wren"` → `"wgpu"` and straight back. The reasoning for the flip was right — the Camera device's
geometry and sky *are* the scene-graph and materials domains, so nothing there could retire while
it rendered through WREN — but it broke two shipped `tests/api` worlds, and the attribution was
measured rather than assumed, each world run on both arms: `pen` and `camera_image_update` FAILed
on wgpu and were **OK** under `OMNISIM_FORCE_WREN=1`, while `camera_color`, `pen_box` and
`camera_recognition` FAILed on both (pre-existing). ⚠️ **The real error was ORDERING** — the
runbook's own F1 says the flips require P1–P8 parity first, and the flip went in before it. It
re-landed at F1 once P3 (pen texture binding) and P5 (post-FX) closed. Corrected the same day:
*"WREN-vs-wgpu parity cannot be measured on this box"* was wrong as stated — it is world-dependent,
and the degraded readings came from mixing a **wgpu main view** with a **WREN camera**. Force the
whole session when you want a reference arm.

**2026-08-22 — the "deleting WREN deletes Linux rendering" blocker was not right as stated.** It
conflated *WREN the renderer* with *a GL context*, which were already separate in this tree: the
present path had both arms live, the GL blit is 66 lines whose only include is `glad/glad.h`, and
`OmWrenOpenGlContext`'s entire WREN dependency was two `wr_gl_state_set_context_active()` calls.
⚠️ What that did **not** establish, and must not be quoted as if it did: it is a *correctness*
path, not a performance one (the blit is a per-frame readback + upload, unmeasured); it says
nothing about whether **wgpu-native itself** builds and runs on Linux; and none of it was verified
*on* Linux. Native X11 / XCB / Wayland surface sources landed separately in `51a1f9f44`, and the
platform question survives as R8 in [Known gaps](#known-gaps).

**2026-08-23 — F1, F2, D1.4, D1.5 and the endgame pass.** F1 made WREN unselectable (fields parse
and warn, hatches neutralised) with the corpus green at 377 worlds and zero new load errors; F2
re-goldened the sensor tests deliberately — **13 of 15 new reds red→green**, every old value
recorded in the test source *and* in the commit, two honest decided reds surviving; D1.4 deleted
the renderer atomically; D1.5 retired the hatches and the tooling arms. The endgame pass then fixed
W1d and paid F2's owed projection re-pins.

⭐ **W1d is the campaign's sharpest lesson, and it generalises.** `acquireGeometryMesh`'s default
branch *claimed in its comment* to serve "IndexedFaceSet, Mesh, ElevationGrid, Cone, …" but only
ever handled `OmTriangleMeshGeometry` subclasses — and `OmElevationGrid` and `OmCone` derive from
`OmGeometry`, so the `dynamic_cast` returned null. Pre-deletion the null fell through to WREN's
static-mesh readback and nobody noticed. **D1.4 deleted the readback, and from that commit every
terrain and every visual `Cone` was silently skipped from every wgpu render, main view and sensors
alike** — caught by `tests/api` `range_finder` reporting `inf` where its ElevationGrid obstacle
should have been. Fixed in `3cbbfd7a5` by generating both meshes CPU-side from the deleted WREN
builders verbatim: `mars.wbt` terrain went **draws 50 vs 49** with 673,899 of 2,110,248 px changed;
the cone probe drew a correct silhouette, apex up, with a cast shadow; `beauty_bench` (which
contains neither node) was pixel-neutral at mean 0.004 / max 4 against a same-binary noise floor of
exactly 0. **A fallback is what makes a stale comment survivable; deleting it converts every such
comment into a silent hole.**

⭐ **The deformable freeze — live once, and it compiles clean.** Two renderers, one process-global
"the clock advanced" flag, one mesh cache each: the loser of the race uploads once and never again,
drawing the right number of triangles and passing any single-screenshot test. Only a cross-frame
comparison in a session with **both** a main view and a Camera can see it. The upload decision
therefore lives on the cache entry (`OmWgpuMeshCache::vertexEpochIs`), never in a static, and
`OMNISIM_WGPU_DEFORMABLE_EPOCH=0` keeps the failure reproducible so the instrument can be shown to
go red. ⚠️ Which consumer freezes depends on who wins the race — the defect is the shared flag, not
a fixed victim.

Earlier tracker, kept for provenance: [r4-completion-checklist.md](r4-completion-checklist.md), the
pre-flip R4 → wgpu-default task list. Its unchecked boxes are superseded by the campaign documents
above.
