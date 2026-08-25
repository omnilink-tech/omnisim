# WREN retirement plan — wgpu as the only renderer

*Opened 2026-08-21 on the owner's direction. This is the rendering half of the engine
migration (ODE+WREN → Newton+wgpu); the physics half completed with the ODE deletion
(`bdc02139`). It runs the same way: **measured audit → parity phases with gates → default
flips with exact-revert hatches → one deletion commit at the end.***

**Status: audit complete (16 domains); Batch 1, phase W1 and W4a SHIPPED, gated and verified —
W4a's draw path is now measured end-to-end, not assumed (see below). `src/wren` CANNOT be deleted
yet, and the reason is not effort — see §Blockers.**

---

## Baselines (machine `9722d23d12a3`, 2026-08-21, pre-Batch-1)

| Measure | Value |
|---|---|
| `wr_*` call sites in `src/omnisim` (`.cpp`+`.hpp`) | **3,136** |
| Files in `src/wren` | **100** (5.4 MB) |
| Engine files touching WREN | **264** |
| Beauty Bench render-oracle | within-tol **45%**, coverage **77.1%** |
| construction_site_dev render-oracle | within-tol **27%**, coverage **100.0%** |
| city_traffic main view | 4,585 draws, renderMs **9**, collectMs 1 |

The oracle's *coverage* is the load-bearing number (it detects geometry vanishing);
*within-tol* is advisory and legitimately low, because the wgpu path has gained scattered
sky, GI, PCSS, SSR and fog that legacy WREN never had. Verification gate for every batch:
`python scripts/dev/render_ab.py --world <w> --arm-b <HATCH>=0 --perf`.

## Where things stand

The **main view** has been wgpu by default since 2026-08-19 with the full stack at city
renderMs ~9. WREN remains in four roles: the per-world opt-out (`renderBackend "wren"` —
**zero tracked worlds use it**), the automatic fallback where wgpu-native is absent (**all
of Linux**), the **Camera-device RTT path**, and a long tail of GUI/overlay/streaming
consumers.

---

## ⚠ Findings that correct earlier claims — read before acting on anything older

**1. The "linear-output sensor contract" this plan asserted is WRONG.** WREN's colour
Camera is *not* linear: `OmWrenCamera.cpp:841` installs `OmWrenHdr`, whose
`hdr_resolve.frag` applies `1-exp(-c*exposure)` then `pow(., 1/2.2)`, and
`tests/api/controllers/camera_checker.c` pins those *encoded* values to ±2. Implementing
the documented "linear" contract would fail that suite by construction and silently change
what every vision controller sees. **Decide the output contract explicitly before writing
any sensor shader.** Only the segmentation path is genuinely linear/exact.

**2. `wgpu_sensor_regression.py`'s 19/19 PASS is NOT a WREN-parity result, and it enshrines
a live bug.** It is wgpu-vs-hand-computed self-consistency on small synthetic probe worlds.
Every wgpu render target is `RGBA8Unorm` while the controller API reads BGRA
(`wb_camera_image_get_red` is `image[i+2]`, `camera.h:87`) — so **a `renderBackend "wgpu"`
camera returns red and blue swapped**, and the suite's own "known good" for a *cyan* box,
`(0,141,141)`, is that swap frozen as correct (a controller reads red=141, blue=0 from a
box with no red). Verified independently on this tree. Anyone treating that suite as a
parity oracle will ship the swap.

**3. `Camera.far` defaults to `0.0`** (`Camera.wrl:27`, meaning "infinite"; WREN substitutes
10000). The wgpu path passes it raw into `perspective()`, giving m22=m32=0 → `clip.z ≡ 0`
for every fragment → draw-order-dependent depth. Invisible only because all nine probe
worlds declare `far 10.0`.

**4. The agent-facing screenshot and ALL cinematic output are WREN-rendered — but for TWO
DIFFERENT reasons, and an earlier revision of this line got the first one wrong.** The harness
`POST /world/screenshot` is NOT hardwired to WREN: `exportImage` -> `OmApplication::takeScreenshot`
-> `OmSimulationView::writeScreenshot` -> `OmView3D::grabWindowBufferNow()`, which **prefers wgpu**
and re-renders one synchronous wgpu frame with readback. It falls back to the WREN GL grab only
because it gates on `mWgpuPresentedLastFrame` (`OmView3D.cpp:3099`), and the harness runs the
engine in a mode where the wgpu main view never presents. **So phase W2 (headless wgpu repaint)
fixes the agent-screenshot divergence as a side effect — it is not a separate exportImage port.**
The capture/cinema service is the genuinely separate case: it renders
through a `Camera` device whose `renderBackend` defaults to `"wren"`. Measured on
beauty_bench: the WREN capture and the wgpu main view of the same world and camera differ
on **98.6% of pixels** (mean 169/765). So an agent authoring a world through the documented
harness loop sees a materially different image than the user does, and every still and
video the project ships comes out of the legacy renderer.

**5. The obvious fix for (4) does not work yet, and this is W3's real blocker.**
`OMNISIM_CAPTURE_BACKEND=wgpu` on beauty_bench at 1280×720 renders **monochrome**
(R=G=B=66.8, std 29.5), untextured and unlit — identical before and after 120 settle steps.
Cause, from the audit and consistent with the measurement: the wgpu sensor entry point calls
`collectWorldDraws` with **`texCache=nullptr`**, a hardcoded 0.18 grey clear, hardcoded
lights and no shadows. Reproducer is committed in `scripts/capture/omnisim_capture.py`.

**6. The 2026-08-19 wgpu default flip shipped five undisclosed GUI regressions.** On the
default renderer today: selection produces no visual feedback; all 21 View→Optional
Rendering items toggle flags nothing reads; every device HUD inset, supervisor label and
status overlay is invisible; and gizmos are invisible **but still draggable** (the WREN
picker renders into its own FBO). These are pre-existing, not caused by this campaign — but
they are the campaign's to fix, and they explain why W4 matters beyond deletion.

**7. Much of W4 is already written.** `OmWgpuView.cpp` contains ~11 working, offscreen-verified
overlay collectors (bounding edges, COM crosses, joint axes, frustums, normals, lidar rays,
contact crosses, support polygon, translate/rotate gizmos with a verified hit-test) sitting
in an anonymous namespace behind an opt-in env var. Most of W4 is lifting them into a shared
TU and calling them from the production frame, not writing renderers.

---

## Retirement candidates (≈460 `wr_*` sites, ~2,000 lines, zero pixels moved)

| # | Item | Sites | Evidence | Gate |
|---|---|---|---|---|
| R1 | VR headset (OpenVR) | 62 | zero worlds/controllers/tests/docs; four commits ever, all mechanical sweeps; every call site already has a working non-Windows `#else` that IS the retired behaviour. **Enabling VR silently pins the whole main view back to WREN** (`OmView3D.cpp:1557-1576`), so it is already incompatible with the default renderer. | ⚠ owner |
| R2 | Physics debug representations | 202 | **`OmWorld::appendOdeContact` — the only writer of `mOdeContacts` — has ZERO callers** (verified on this tree), so the contact overlay and contact sound are structurally dead. `addForceAtPosition`/`addTorque` are empty bodies: ALT+drag draws "12.3 N" while the solver receives zero. Zero of 1,381 shipped `.wbproj` perspectives enable ContactPoints/CenterOfMass/SupportPolygon. | ⚠ owner (support polygon is live under `FORCE_WREN`) |
| R3 | Skin (FBX skeletal) | 105 | zero `Skin {}` instances in the tree; its worlds were deleted in the v5.0.0 licence hardening; sole consumer of `src/wren/Skeleton*`. Porting needs a per-frame streaming vertex path wgpu does not have. | ⚠ owner — **reverses `5c3ee46d7` (2026-08-16)**, which measured Skin as live on a *load* check, before the renderer flip changed the ground |
| R4 | Four dead shader accessors | 20 | zero references outside their own definitions | safe |
| R5 | Four dead sub-features | ~71 | buoyancy marker hardcoded invisible (Fluid died with ODE); `OmOdeDebugger` every functional line commented out; matter-centre setter has zero callers | safe |
| R6 | emcc/wrenjs build rules | 0 | shipped `wrenjs.wasm` was curl'd from cyberbotics.com, never built here | safe |

⛔ **Do NOT delete `OmSolid::supportPolygon()` / `extractContactPoints()` / `staticBalance()`** —
they back the public C ABI `wb_supervisor_node_get_static_balance`. Deleting them still links
and silently returns garbage.

---

## Phases

- **W1 — cut wgpu's own WREN dependency. ✅ COMPLETE (2026-08-21).** The renderer no longer
  needs WREN to know what the scene looks like, nor a live GL context to collect a frame.
  **W1a** tessellated geometry from the engine's CPU `OmTriangleMesh`.
  **W1b** CadShape retains its assimp tessellation (content-hash keyed, matching WREN's own
  content dedup so shared `.dae` files still share one upload) and takes its world matrix from
  the node's own pose — verified end-to-end: every per-submesh `WrTransform` is an identity
  node under the enclosing pose, and `OmAbstractPose::updateMatrix` computes that same
  composition. Measured **bit-identical** (mean 0.0, max 0) against the hatch-off arm on
  `cadshape_node`. Hatch `OMNISIM_WGPU_NATIVE_CADSHAPE=0`.
  **W1c** primitives get content keys (all Boxes share one buffer), the `wrenMesh()` collection
  gate is gone, and the `makeWrenCurrent()` bracket is removed from `OmView3D` — the collect
  arms the context itself, lazily, RAII-released, behind a cache-hit fast path. Measured:
  beauty_bench `glArms=0` for a whole run; city_traffic `glArms=1` total with
  `glArmsWindow=0` at steady state, where it previously armed on every collect. The arm count
  is exported in `OMNISIM_WGPU_REPORT`, so "the bracket is dead" is a number per world.
  Oracles unchanged from baseline **exactly** (45%/77.1%, 27%/100.0%); city 4,585 draws,
  renderMs 5–10 vs baseline 9; smoke 4/4; 94/94 demo worlds load.
  ⚠ `wr_*` sites went 3,069 → 3,071 — slightly UP, because the hatch-off fallbacks and the
  audit diagnostic still reference WREN. The DEFAULT path is WREN-free; the count drops when
  the hatches are retired, which is a later step.

  *(historical detail)* **W1a** Tessellated geometry sources from the engine's CPU `OmTriangleMesh`
  (byte-equivalent by construction). Verified across IndexedFaceSet, ElevationGrid and a
  dense Cone/mixed scene: max pixel diff ≤17, zero pixels over threshold; city unchanged at
  renderMs 9. Hatch `OMNISIM_WGPU_NATIVE_MESH=0`.
  **W1b** CadShape CPU retention (last `wr_*` in the scene collector). **W1c** re-key
  primitives off the geometry node and drop the `makeWrenCurrent()` bracket — *without this a
  WREN-free build renders zero draws.*
- **W2 — headless wgpu**: repaint driver without a desktop window; harness screenshot and
  synth dump stop needing a session. Unblocks CI for everything after it.
- **W3 — sensor cameras (XL, the pole).** Step zero is building the WREN-vs-wgpu sensor
  comparator, which **does not exist**. Then the three defects above, the contract decision,
  eight sensor post-processes with no wgpu equivalent, and spherical/cylindrical projection
  (zero wgpu code, 10 shipped worlds).
- **W4a ✅ LANDED AND VERIFIED (`e0c179dc3`, verified 2026-08-22).** Selection tint + AABB
  outline and six Optional Rendering items are wired from `OmWgpuView.cpp`'s existing collectors
  into the production frame, drawn last (after tonemap, so AgX/bloom/TAA cannot recolour or smear
  them), into a private output so they never enter TAA history. Hatch `OMNISIM_WGPU_OVERLAYS`.
  **Proven:** idle cost is nil — overlays ON with nothing selected is pixel-identical to OFF on two
  worlds (max 7, zero pixels over threshold), city renderMs 6–10 vs baseline 9, zero wgpu validation
  errors. **The draw path is now proven too**, measured end-to-end on
  `warehouse_husky` with `OMNISIM_OPTIONAL_RENDERING=AllBoundingObjects,JointAxes`:
  `ovCalled=1 ovBatches=2 ovVerts=1664 ovOk=1` with overlays on, `ovCalled=0` with them off. That
  covers collect → draw → publish (`mOverlayActiveView = mOverlayView`, the success path of
  `drawOverlayLines`) → present (`OmView3D.cpp` presents `sceneTextureView()`, which returns
  `mOverlayActiveView` when set, and the draw call site precedes the present in the same frame).
  Gizmo visuals stay deliberately excluded — the pane's gizmo is a second, independent gizmo with
  its own hit test, so drawing it would put visible handles where the live hit regions are not.

  ⚠️ **THE REASON THIS TOOK SIX ATTEMPTS IS WORTH MORE THAN THE RESULT: three separate
  instruments were each BLIND to overlays, and every one of them reported a clean, confident,
  wrong answer.** None of them errored.
  1. **The MAINVIEW/SYNTH dumps and screenshots are blind BY DESIGN.** They read back inside the
     scene render, before the overlay pass — which is exactly why `drawOverlayLines` is documented
     as present-path only. So `render_ab.py`, the campaign's own gate, *structurally cannot* see an
     overlay. Its `MATCH` verdict was not evidence of absence; it was the instrument's own contract.
  2. **`draws=` is blind**, because it is `draws.size()` — the SCENE draw list. The overlay pass
     never touches it. An overlays-on vs overlays-off A/B reported `draws=1052` in BOTH arms, and
     that identity meant precisely nothing.
  3. **Pixel A/B on an ANIMATED world is blind**, because the noise floor exceeds the signal. On
     `warehouse_husky` the "signal" was 989 px over threshold and the same-arm-twice NOISE FLOOR was
     1060 — higher. Re-run on a static sibling (the world with `controller "<none>"`) the noise floor
     is exactly **0** (mean 0.0, max 0, bit-identical frames), which is what makes a pixel A/B mean
     anything at all. **Always measure the noise floor before reading a diff.**

  The permanent fix is the telemetry, not the conclusion: the wgpu report line now carries
  `ovCalled` / `ovBatches` / `ovVerts` / `ovOk` (`ovCalled=2` distinguishes "reached the draw site
  with zero geometry" from "never reached it"; `ovOk` is `drawOverlayLines`' own return). Same
  reasoning as `glArms` — a retirement claim should be a measurement, not an assertion. Enabling an
  optional rendering without a GUI is `OMNISIM_OPTIONAL_RENDERING=<CSV of View menu names>`, which
  is what made any of this testable; before it, the entire class of overlay/gizmo/HUD pixels had no
  automated verification path, and that is precisely how the 2026-08-19 wgpu default flip left
  selection feedback and all 21 Optional Rendering items dark for days without anyone noticing.

- **W4b ✅ RADAR + LIGHT-SENSOR OVERLAYS PORTED (2026-08-22).** `VF_RADAR_FRUSTUMS` and
  `VF_LIGHT_SENSORS_RAYS` now render on the wgpu main view, collected by `gatherRadarFrustums` /
  `gatherLightSensorRays` in `OmWgpuView.cpp` and drawn through the W4a batch path. Chosen because
  neither device renders anything a controller reads — `OmRadar` answers via
  `refreshTargetOcclusionFromNewton()` and `OmLightSensor` via `refreshRayCollisionsFromNewton()`,
  so WREN was doing pure GUI visualisation and there is **no output contract at risk**.
  **Verified numerically, against ground truth rather than against my own expectation:** the radar
  emits **394 vertices, which is exactly WREN's own `10 + 16*steps` at `steps = 24`**
  (`OmRadar.cpp` reserves `3 * (10 + 16 * steps)`), and the light-sensor world emits **4 vertices
  for its 2 `LightSensor` nodes**. Both sample worlds already ship the flag in their perspective
  (`.radar.wbproj` → `globalOptionalRendering: RadarFrustums`), so this is what a user sees, not
  just what an env var forces.
  ⚠️ **One deliberate deviation from WREN, documented in the code:** WREN scales the light-sensor
  ray by `wr_config_get_line_scale()`, a WREN-global view-dependent scale. Reproducing that would
  mean calling into WREN from the wgpu collector — the exact dependency this campaign removes — so
  the ray uses a fixed metric length instead, matching the convention `collectJointAxisLineVerts`
  already uses. The ray POINTS where WREN's points; its LENGTH is metric, not view-scaled.
  This is an ADDITION, not a retirement: the WREN paths are untouched and still render, so nothing
  regresses on the WREN opt-out or on Linux. It does not lower the gate number by itself — it makes
  the corresponding WREN code *safe to delete later*.

- **W3 ⚠ CAMERA DEFAULT FLIPPED AND REVERTED THE SAME DAY (2026-08-22).** `Camera.wrl`
  `renderBackend` went `"wren"` → `"wgpu"` and back. Recording both halves, because the reasoning
  for the flip is still right and the reason for the revert is the part that would otherwise be
  relearned.
  **Why it was flipped:** this is coupling C7, and it gates ~87 other blocking findings —
  `OmGeometry`'s renderables and materials, `OmBackground`'s skybox/irradiance and the whole
  materials domain ARE the Camera device's geometry and sky, so nothing in scene-graph, materials
  or environment can retire while the device renders through WREN.
  **Why it was reverted:** it broke two shipped `tests/api` worlds, and the attribution is measured
  rather than assumed — each was run on both arms:
  | world | wgpu default | `OMNISIM_FORCE_WREN=1` | verdict |
  |---|---|---|---|
  | `pen` | FAIL | **OK** | **broken by the flip** |
  | `camera_image_update` | FAIL | **OK** | **broken by the flip** |
  | `camera_color` | FAIL (103,58,46) | FAIL (59,36,31) | pre-existing |
  | `pen_box` | FAIL | FAIL | pre-existing |
  | `camera_recognition` | FAIL | FAIL | pre-existing |
  The `pen` failure is structural, not a tolerance: **`OmWgpuSolidDraw` has four texture views
  (base/roughness/metalness/normal) and NO pen slot**, so `OmPaintTexture` — which WREN binds at
  texture index 1 (phong) / 8 (PBR) — is simply absent from a wgpu Camera image. `pen.c` asserts
  painted colour through `wb_camera_get_image` at ±2.
  ⚠️ **The real error was ORDERING, and it was mine:** this runbook's own F1 says the flips require
  P1–P8 parity first, and the flip went in before it. It re-lands at F1, after P3 (pen texture
  binding) and P5 (post-FX). It is a one-word change either way.
  ✅ **Not affected, verified:** recognition/segmentation ride the `OmWrenCamera`, which a normal
  GL-context session builds regardless of `renderBackend` (`mWrenlessSetup` is set only under
  `OMNISIM_NO_GL`); and capture/cinema writes the field explicitly and pins `"wren"`.
  ⚠️ **A correction to a claim made earlier the same day:** "WREN-vs-wgpu parity cannot be measured
  on this box" is **wrong as stated**. It is world-dependent — under `OMNISIM_FORCE_WREN=1` the
  whole session is WREN and the GL context is healthy, which is why `pen` passes there. The
  degraded `(59,36,31)` reading came from mixing a wgpu MAIN VIEW with a WREN camera. Force the
  whole session when you want a WREN reference.

- **W4 — the long tail**: overlays/labels, gizmos, selection, streaming — mostly wiring
  existing `OmWgpuView.cpp` collectors; also fixes the five GUI regressions in §6.
- **W5 — Linux wgpu surface. HARD BLOCKER.** See below.
- **W6 — flips + deprecation** (`renderBackend "wren"` warns and resolves to wgpu).
- **W7 — the deletion commit**, only after every gate is green.

## What WREN still uniquely provides — the real deletion checklist

The 3xx "blocking findings" number counts `wr_*` call sites, and most of them disappear *with*
WREN rather than before it — you cannot delete WREN's scene-graph or material code while WREN is
still selectable. So that number is a coupling measure, not a readiness measure. **The question
that actually decides W7 is: what would a user LOSE if `src/wren` vanished tonight?** Measured and
analysed 2026-08-22:

| # | capability | status | evidence |
|---|---|---|---|
| 1 | **Cloth / SoftBody rendering** | ⛔ **MEASURED MISSING, and already regressed in today's default** | `newton_cloth_drape.omniworld` renders **draws=2..3** for the whole scene, the OmniLight BVH snapshot reports **24 tris** (two boxes; a 441-particle cloth is ~800), and `OmCloth`/`OmSoftBody` appear **zero** times in `OmWgpuSceneRenderer.cpp` + `OmWgpuRenderTarget.cpp`. `collectShapeDraws` only ever dynamic_casts to `OmCadShape`, `OmShape`, `OmGroup`, `OmSolid`, `OmBasicJoint`. **15 tracked worlds declare `Cloth {` or `SoftBody {`.** Since wgpu became the default main view on 2026-08-19 those worlds have been rendering without their subject. |
| 2 | **`Muscle` / `Track` surfaces** | ⛔ missing | Same collector gap. `OmMuscle` is `VM_REGULAR`, so a WREN Camera device *does* see it; 7 tracked worlds use it. |
| 3 | **`Pen` paint (`OmPaintTexture`)** | ⛔ missing | Controller-observable *through a Camera*, so this is not GUI-only. |
| 4 | **`Background` irradiance cubemaps + `luminosity`** | ⚠ **`luminosity` + `PBRAppearance.IBLStrength` PORTED 2026-08-22 (P4)**; the irradiance CUBE is not | The two scalars now scale the wgpu diffuse AND specular ambient exactly as `pbr.frag:316-318` does (measured: `lum 2` and `IBLStrength 2` produce the byte-identical pixel — they are one premultiplied float). WREN's phong ambient (`Lights.ambientLight × material.ambient`) is ported too, which is what made `tests/api/worlds/pen.omniworld`'s first assertion pass on a wgpu Camera (207,207,207). ⚠ **Still unported: the irradiance CUBE itself** — and note `atmosphericSky` BAKES one (`OmBackground.cpp:730`), so every atmospheric world takes `pbr.frag`'s cube branch, not the flat `skyColor` fallback. wgpu still substitutes its analytic hemisphere for that base colour. |
| 5 | **Camera post-FX**: `noise`, `noiseMaskUrl`, `lens`, `motionBlur`, `RangeFinder`/`Lidar` `resolution` | ⛔ unported | `OmAbstractCamera::setup()` already declares these unsupported on the wrenless path. Now that the Camera default is wgpu, **authored worlds using them silently lose the effect**. |
| 6 | **Lidar range rendering** | ⚠ partial | `OmLidar` renders its MEASUREMENT; wgpu is opt-in (`OMNISIM_LIDAR_WGPU`) and self-limits — it returns false for rotating heads and `fov > 6.3`, falling back to WREN. |
| 7 | **Device HUD insets, labels, status overlays** | ⛔ missing | `OmWrenTextureOverlay` / `OmWrenLabelOverlay`; one of the five GUI regressions from the 2026-08-19 flip. |
| 8 | **Manipulator gizmos** | ⚠ invisible but draggable | Collectors exist in `OmWgpuView.cpp` with a verified hit-test; deliberately not drawn (the pane's gizmo is a second gizmo with its own hit test). |

✅ **Rows 1–3 are FIXED for the MAIN VIEW** (`d5897ff7d`): cloth/softbody now collect AND
animate there — `newton_cloth_drape` went draws 3→4 / tris 24→536, `newton_softbody_drop`
3→5 / 24→396. It turned out to be **two** defects, not one: the geometry was never collected,
*and* `animateMesh()` is driven by a `wr_scene` frame listener that the wgpu main view never
calls — so fixing only the first would have shipped a cloth frozen at its rest pose that looks
correct in a single screenshot. ⚠️ **Still open: a wgpu-rendered Camera / RangeFinder / Lidar
sees no deformable**, because those collect through `collectWorldDraws` alone. That gap became
reachable the same day, when the Camera default flipped to wgpu — but **measured, ZERO tracked
worlds pair a `Camera {` with a `Cloth {`/`SoftBody {`**, so nothing shipped is affected today.
Author one and it would be. Row 1 is worth fixing **regardless of this campaign** — it is a live
defect in the shipped default, not a retirement prerequisite.

## Blockers (honest)

0. ✅ **THE "DELETING WREN DELETES LINUX RENDERING" BLOCKER IS NOT RIGHT AS STATED (2026-08-22).**
   This document has said from day one that `src/wren` cannot go because Linux has no native wgpu
   surface and would be left with no renderer. That conflates *WREN the renderer* with *a GL
   context*, and the two are already separate in this tree. The present path in
   [`OmView3D.cpp:3229`](../../src/omnisim/gui/OmView3D.cpp#L3229) **already has both arms live**:
   `presentTexture()` on the native surface when one exists, and otherwise
   `OmWgpuGlBlitRgbaToScreen()` — wgpu renders, the pixels are blitted to the window through GL.
   Follow what that fallback actually depends on:
   - [`src/omnisim/render/OmWgpuGlBlit.cpp`](../../src/omnisim/render/OmWgpuGlBlit.cpp) is **66
     lines and its only include is `glad/glad.h`** — zero WREN, and it does not even live in the
     WREN directory.
   - `src/glad` is an independent GL loader with **zero** WREN references.
   - [`OmWrenOpenGlContext`](../../src/omnisim/render/OmWrenOpenGlContext.hpp) (it survived the deletion and now lives in `src/omnisim/render/`) is a **165-line
     `QOpenGLContext` subclass**. Its ENTIRE WREN dependency is two calls to
     `wr_gl_state_set_context_active(true/false)` — telling WREN's GL state tracker the context is
     current. With WREN gone there is nothing to tell, and the class is pure Qt.

   So the work to keep a non-native-surface platform presenting after the deletion is: drop two
   calls, and move one small file out of `src/omnisim/wren/`. That is not a hardware blocker and
   it does not need a native X11/XCB/Wayland surface at all.

   ⚠️ **What this does NOT establish, and must not be quoted as if it did.** (a) It is a
   *correctness* path, not a performance one — the blit is a per-frame readback + upload, so
   shipping Linux on it would be slower than the native surface and possibly slower than WREN;
   that is unmeasured. (b) It says nothing about whether **wgpu-native itself** builds and runs on
   Linux in this tree — that is a separate question from presentation, and it is still open.
   (c) None of it is verified ON Linux, because this box cannot compile the Linux branch
   (`QT_FEATURE_xcb -1`); it is a dependency analysis anyone can re-check by reading the three
   files named above. The honest status is **"the presentation blocker is soluble without new
   surface code"**, not "Linux works".

1. **W5 cannot be compiled or verified on this machine.** It is a Windows box, and this
   checkout's `qtgui-config.h` has `QT_FEATURE_xcb -1` / `QT_FEATURE_wayland -1`, so the X11
   block is preprocessed away — a green Windows build is zero evidence the Linux branch even
   parses. A RunPod pod + Xvfb + lavapipe could prove the chain works *in software* only;
   flipping the Linux default needs a Linux box with a discrete GPU **and** a real display
   server, which **does not exist in this campaign's inventory**. Until it does, deleting
   `src/wren` deletes Linux rendering.
2. **Three owner decisions**, none of them engineering calls: the W3 sensor default flip
   (changes what every vision controller, RL consumer and the cinema pipeline sees), Skin
   retirement, VR retirement. `LensFlare` is a smaller fourth (already silently non-functional
   on wgpu, no warning).
3. **W3 is the schedule, not a phase**, and its instrument does not exist yet.
4. **Verification is serial and wall-clock expensive** — one engine at a time (75 °C thermal
   limit; `TaskStop` does not reap `omnisim-bin` orphans, and a stray one holding TCP 1234
   breaks the pre-push smoke hook). Every render arm is windowed until W2 lands.
5. **Some gates need a human**: gizmo drag parity, HUD drag/resize/close across a reload,
   Linux click-drag orbit. Record them as checklist results, never as claims.
6. **There is no CI for any of the pixels this campaign is about** — which is exactly how the
   device HUD went dark on 2026-08-19 and stayed dark. Committing reference sets early is the
   cheap durable fix.

## The gate: `python scripts/dev/wren_deletion_audit.py`

One re-runnable command answers "what still blocks deleting `src/wren`". It classifies every
`wr_*` site, every `<wren/...>` include and the non-code couplings as **BLOCKING / RETIRABLE /
ALREADY-PORTED**, refuses to count a dependency as resolved just because a hatch exists (the
hatch-off fallbacks are exactly why the `wr_*` count went UP after W1b/W1c), and is self-tested
red-capable. **Current verdict: NOT DELETION-READY — 341 blocking, 18 retirable.** Blocking
findings by domain: W4 overlays/gizmos 9, W3 sensor cameras 4, W6 main-view post-FX 3, core
context/window 2, misc nodes 2, plus scene-graph, joint visuals, environment, W6 flips and W1
geometry-collect. **Re-run it after every batch. The number is the gate; nobody's memory is.**

## Deletion-readiness checklist (measured 2026-08-22)

Things a `rm -rf src/wren` touches that are NOT obvious from the phase list:

1. **Deleting WREN deletes the SAFETY NET, not just a renderer.** `OmView3D.cpp:3072-3080` has two
   present paths: the native wgpu surface, and a fallback that reads pixels back and blits them
   through OpenGL. That fallback runs inside `OmWrenOpenGlContext::makeWrenCurrent()` /
   `swapBuffers()` / `doneWren()`, and `OmWrenOpenGlContext` lives in `src/omnisim/wren/`. So the
   blit cannot outlive WREN — which means **the wgpu surface must work on every supported platform
   with no fallback** before deletion. Today it exists only for Windows HWND.
2. **`src/glad` is shared.** It is not WREN-only: `src/omnisim/render/OmWgpuGlBlit.cpp` includes
   `glad/glad.h`. glad can only go when the blit goes (see 1).
3. **Packaging manifests name WREN assets.** `scripts/packaging/files_core.txt:225-229` ships
   `resources/wren/{meshes,shaders}` by wildcard; those entries need removing in the same change or
   the installer references missing paths.
4. **The web viewer has its OWN WREN.** `resources/web/wwi/` contains `WrenRenderer.js`, a `wren/`
   tree and the prebuilt `wrenjs.{js,data}` — the `--stream` w3d viewer renders in the browser
   through a WREN-derived JS renderer. Deleting `src/wren` does not break it (wrenjs is a vendored
   artifact, never built here), but it does mean **the streamed view will never match the desktop
   view** once the desktop is wgpu-only. That is a divergence to accept deliberately or to schedule,
   not something to discover after the fact.
5. **Public C ABI stays.** `OmSolid::supportPolygon()` / `extractContactPoints()` / `staticBalance()`
   back `wb_supervisor_node_get_static_balance` and must survive any debug-rendering retirement.

## Rules

- Every phase ships an exact-revert hatch, **value-parsed** (`FOO=0` means OFF; never
  presence-gated — the `OMNISIM_REQUIRE_NEWTON` trap).
- No phase may regress city renderMs (baseline 9) or the Beauty Bench A/B.
- Attribute every number to a machine (`env_fingerprint.py`).
- Docs are updated once per batch by the batch owner, with measured numbers — never by the
  implementing tasks (otherwise every batch is a four-way doc conflict).
- The deletion is LAST. A half-deleted renderer is worse than either whole one.
