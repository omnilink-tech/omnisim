# Changelog

All notable changes to OmniSim are recorded here.

The format roughly follows
[Keep a Changelog](https://keepachangelog.com), and OmniSim follows
[Semantic Versioning](https://semver.org).

OmniSim is built on [Webots](https://github.com/cyberbotics/webots) — see
the [Attribution](README.md#attribution) section of the README for the
relationship to upstream. Entries here cover OmniLink's contributions on
top of that foundation.

> ⚠️ **Only the v8.x public-beta releases are published in this repository.**
> Sections for earlier versions are kept below as the development record — they
> say what changed and when — but those releases are **not obtainable here**: no
> tag, no release page, no downloadable archive. Do not read an older section as
> a description of something you can check out. Use the newest v8.x release for
> running code.
>
> The one class of tag that is NOT a release, and is still present, is
> `deps-windows-v1` / `deps-linux-v1` / `deps-mac-v1` — those carry the build's
> dependency archives, which `dependencies/Makefile.*` fetches by name.

---


## [Unreleased]

Nothing yet.

---

## [v8.1.4] — 2026-08-25

### Release engineering

- Fixed distribution assembly for worlds that intentionally have no saved GUI
  perspective. Such worlds now ship on their own and open with OmniSim's
  default view; an existing `.omniperspective` or legacy `.wbproj` remains
  bundled, with the canonical format preferred.
- Added regression coverage for both an absent optional perspective and the
  canonical-over-legacy preference.
- No simulator behaviour changed from v8.1.3. The v8.1.3 packaging run passed
  mixed-encoding URL rewriting and Newton bundle verification, then stopped
  because it manufactured a missing perspective path; use v8.1.4 for the public
  beta package.

---

## [v8.1.3] — 2026-08-25

### Release engineering

- Fixed Windows installer assembly for mixed-encoding world, PROTO, controller,
  and robot-window sources. The packaging URL rewriter now replaces its ASCII
  URLs as raw bytes, preserving every unrelated byte instead of depending on
  the runner's legacy CP-1252 default encoding.
- Added a regression test covering UTF-8 text, a byte that CP-1252 cannot
  decode, and CRLF preservation in the same source file.
- No simulator behaviour changed from v8.1.2. The v8.1.2 engine and Newton
  runtime built and verified successfully, but installer assembly stopped at
  the encoding error; use v8.1.3 for the public beta package.

---

## [v8.1.2] — 2026-08-25

### Release engineering

- Fixed Windows linking against the `actions/setup-python` layout. Its install
  directory ends in `x64`, so deriving the import library from the directory
  name incorrectly produced `-lx64`; the workflow now derives `-lpython312`
  from the running interpreter and verifies the matching `.lib` before build.
- Added bounded retries around MSYS dependency installation so one slow mirror
  does not invalidate an otherwise reproducible release build.
- No simulator behaviour changed from v8.1.1. Use v8.1.2 for the verified
  installer and public beta.

---

## [v8.1.1] — 2026-08-25

### Release engineering

- Fixed the Windows release runner's MSYS environment so the Python installed
  by `actions/setup-python` remains on `PATH` and its `Python.h` is available to
  the Newton backend build. The workflow now fails immediately if that header is
  absent, before spending time compiling the engine.
- No simulator behaviour changed from v8.1.0. The v8.1.0 source tag remains as
  the immutable record of its failed packaging run; use v8.1.1 for the verified
  installer and public beta.

---

## [v8.1.0] — 2026-08-25

### Highlights

- **The first distribution release.** Tagged releases now build and attach a
  Windows 10/11 installer with the complete pinned Newton runtime. The workflow
  refuses to package when that runtime is missing: a clean install cannot
  silently become a renderer-only shell. Linux remains the verified source-build
  path; macOS physics is still unverified and no macOS binary is advertised.
- **A focused public beta instead of a generic launch.** [`BETA.md`](BETA.md)
  gives the first ten external testers a 20-minute install, demo, agent-edit,
  and feedback challenge. A structured Request-a-Sim issue form and
  [`SUPPORT.md`](SUPPORT.md) route failures and capability requests without
  pretending every request belongs in the engine.
- **Current, reproducible launch footage.** The README now shows the wgpu/Newton
  OmniArm 6 real-contact pick-and-place demo rendered from the shipped world.
  The block is ineligible for the bridge's kinematic attachment path and its pose
  is never written by the controller.

### Build / packaging

- Package filenames and installer metadata now follow OmniSim's SemVer tag
  (`v8.1.0`) instead of retaining the inherited `R2025a` Webots label.
- The release script updates the C++ version, Python package version,
  distribution-package version, and training-image pins from the same tag.
- The old dormant Linux/macOS/Windows matrix was replaced by the one platform
  the project can presently certify as a downloadable package. The release
  workflow installs the build toolchain, builds OmniSim, vendors the pinned
  CPython/Newton runtime, verifies imports and a live engine run, and only then
  creates the installer and GitHub Release.
- Packaging diagnostics no longer say a missing Newton bundle falls back to
  ODE. ODE is gone; the correct consequence is no working physics backend.

### Documentation

- Added an AI-tools contribution policy, one support front door, and explicit
  public contact identity (`OmniLink <info@omnilink-agents.com>`).
- Corrected three public setup statements, including the Linux command that
  could not work as written.

---

## [v8.0.0] — 2026-08-24

> ⛔ **READ THIS IF YOU HAVE A WORLD, A ROBOT PACKAGE, OR A SCREENSHOT PIPELINE. This release
> deletes the WREN renderer and replaces every vendor-derived robot package.** Eight things stop
> working or change behaviour, each listed with its fix under **Removed** and **Changed**:
>
> 1. ⛔ **WREN IS DELETED. wgpu-native is the only renderer.** `src/wren` (106 files), `include/wren`
>    (33), `src/omnisim/wren` (62) and 86 shaders are gone — ~31k lines across the campaign.
>    `renderBackend "wren"` **still parses** (an undeclared field is a hard load ERROR — the
>    `Solid.immersionProperties` precedent) but it is a **warned no-op that renders wgpu anyway**.
>    *No fix needed for a world that loads;* delete the field or write `"wgpu"` to silence the warning.
> 2. ⛔ **A host whose wgpu-native cannot initialise now has NO renderer.** There is no fallback tier
>    left. One loud log line names the condition; physics and controllers are unaffected and the
>    session continues. *Fix:* install wgpu-native (`scripts/dev/setup_wgpu_native.sh`).
> 3. ⚠️ **`Camera`, `RangeFinder` and `Lidar` `renderBackend` now default to `"wgpu"`** (`Viewpoint`
>    flipped on 2026-08-19). **Every camera-family device's image now comes out of a different
>    renderer, so its pixel values changed.** *Fix:* re-golden any pinned pixels — we did, and each
>    old value is recorded in-comment beside its replacement.
> 4. ⚠️ **The `Skin` node is deleted.** A world declaring `Skin {}` now logs `Missing declaration for
>    'Skin', unknown node.` + `Skipped unknown 'Skin' node or PROTO.` — **2 errors, headless exit 1** —
>    with that node skipped and the rest of the world still parsing. Same failure class as
>    `Radio`/`Microphone` in v7.0.0. Zero in-tree worlds affected. ✅ The `wb_skin_*` C ABI and
>    `WB_NODE_SKIN` (ordinal 58) are **kept and still exported**.
> 5. ⚠️ **The `Helmet` and `Telephone` PROTOs are deleted** — a licence defect, not a cleanup. A world
>    declaring either now fails to resolve its EXTERNPROTO. *Fix:* the PBR sample is rebuilt around the
>    own-authored `PbrMaterialSpecimen.proto`; there is no drop-in replacement for `Telephone`.
> 6. ⚠️ **Every vendor-derived robot package is gone, replaced by OmniSim's own.** Package paths, world
>    paths, controller directories, `DEF` names, robot `name` fields, the arm bridge's `--robot` id, one
>    agent id and one skill id all changed. **Physics is unchanged** — mapping table under **Changed**.
>    *Fix:* rename in your worlds, launchers and scripts.
> 7. ⚠️ **The arm bridge's vendor hardware backend is removed**, with its operator doc, wiring test and
>    the `--hardware-backend` value that selected it. The generic `HardwareBackend` plug-in mechanism is
>    unchanged — `--hardware-backend <name>` still resolves a sibling `<name>_backend.py` — but **no
>    backend ships**, so this tree has no path to a physical arm. Said plainly rather than implied.
> 8. ⚠️ **`VirtualRealityHeadset` is removed and eight renderer env hatches are retired.** No `.wrl`
>    schema ever existed for the VR node, so **no world breaks**; the View menu entry and the three
>    `wb_supervisor_virtual_reality_headset_*` calls are inert (the C ABI is kept and answers locally
>    with exactly what the pre-existing no-headset path returned). Retired because their `=0` arm
>    selected deleted code: `OMNISIM_WGPU_NATIVE_MESH` / `_PRIMITIVES` / `_CADSHAPE`,
>    `OMNISIM_LIDAR_WGPU`, `OMNISIM_RANGEFINDER_WGPU`, `OMNISIM_WREN_POSTFX`, `OMNISIM_WGPU_VIDEO`,
>    `OMNISIM_NEWTON_SKIP_WREN`. `OMNISIM_FORCE_WREN` and `OMNISIM_LEGACY`'s render arm are warned
>    no-ops, mirroring the retired ODE selectors.
>
> **What does NOT break, and will not: the controller API, in both languages.** Nothing moved this
> release — no namespace change, no header change, no environment-variable change. C and C++
> controllers built against v7.0.0 keep linking and keep running. The `.wbt` / `.omniworld` dual-read
> policy is untouched. And **the physics of every renamed robot is unchanged**: every joint origin,
> axis, limit, effort, mass, CoG, inertia tensor and collision shape is identical, verified by parsing
> the new XML against the originals.

### The renderer — WREN deleted, wgpu is the only renderer

The migration running since v3 finished. wgpu became the default main view on 2026-08-19
(`adf2aa075`, an explicit owner decision), the last WREN selector became unreachable on 2026-08-23
(`35148ad65`), and the tree went the same day (`976b9449d`). What made that order possible was a
**measurement instrument, not a judgement**: `scripts/dev/wren_readiness.py` and
`scripts/dev/wren_deletion_audit.py` reduce "is it safe to delete WREN" to eight re-runnable checks
and a blocking/retirable count, which went **322 → 0** before a line was removed.

Every figure below is machine `9722d23d12a3` (RTX 3060 laptop, Windows 11), on the `city_traffic`
world at 1896×1113 or on the Beauty Bench, as named.

#### Added — the realism campaign that had to land first

- **A physically-scattered sky that FEEDS the lighting, at zero per-frame cost.** Rayleigh + Mie +
  ozone with the engine's own earth/mars constants, a sun disc whose spectrum is the *transmitted*
  illuminance so it reddens at the horizon by physics, a ground-albedo term, and a night kit (stars,
  Milky Way, moon). The march runs in a 128×64 sky-view LUT plus a 128×1 transmittance strip, re-baked
  only when the sun moves, so the dome pays **one texture sample per pixel — cheaper than the palette
  maths it replaced**. Phase 2 is the point: a CPU port of the same march derives the hemisphere
  ambient and the IBL env palette, so shadow fill and metal reflections track the actual sky. City
  renderMs **7 steady**. `OMNISIM_WGPU_SKY_SCATTER=0` reverts pixel-identically.
- **Procedural clouds** — a 2-scale fBm sheet at ~1.5 km lit by the transmitted sun sampled from the
  transmittance strip, so it goes amber at sunset and dark at night by physics. Landed with a pass
  reorder that makes **sky cost scale with visible sky** rather than the whole frame, verified
  pixel-identical before any cloud work.
- **⭐ OmniLight — baked global illumination.** A deterministic multithreaded CPU path tracer over the
  *actual* render triangles bakes an irradiance probe volume (L1 SH per probe) whenever the light rig
  or scene changes; per frame the lit shader replaces the flat hemisphere ambient with one trilinear
  3D-texture sample. It traces sky radiance from the same scattered atmosphere the dome renders — so
  GI and sky cannot disagree — plus transmitted-sun direct light with shadow rays, material albedo,
  **emissive surfaces as lights**, two bounces. Then static point/spot lights bake in with real
  occlusion and their leaky real-time versions crossfade out, so the night scene's visible lie —
  light shining through walls — is gone; and a traced 64×64 specular cubemap with a 3-mip roughness
  ladder, parallax-corrected, so metals and glass mirror the actual scene. ⚠️ **Static-scene GI by
  design** — moving robots deliberately do not retrigger a bake.
- **Volumetric light shafts, PCSS contact-hardening shadows, and screen-space reflections.** Sun
  visibility bakes into the probe volume's spare alpha slab (6 jittered shadow rays per probe, zero
  new textures); at runtime a 16-step jittered march accumulates height-falloff fog × baked visibility
  × a Henyey-Greenstein phase. PCSS estimates blocker distance over a 16-tap search and sets the PCF
  spread from the receiver–blocker gap *in metres*, so shadows are razor-sharp at contact and soften
  with caster distance. SSR reconstructs position and normal from the scene's own MSAA depth (no
  G-buffer). Both on: city renderMs **7–8**, inside the established band.
- **Cascaded shadow maps merged into the one true render** (3 fitted cascades, extent stabilisation
  and texel-grid snapping), **analytic specular IBL** (the "copper looks copper" fix), **HDR bloom**
  with real threshold semantics in linear luma units, **GTAO** ported from WREN's own shaders, **TAA**
  (Halton jitter + reprojection + 3×3 neighbourhood clamp), and a **camera pass** — auto-exposure
  easing toward a mid-grey target, vignette, deterministic photographic grain.
- **The linear-light HDR pipeline is ON BY DEFAULT.** The old verdict "AgX reads milky at 1.0 and
  blown at 2.5" was a diagnosis of the shading *underneath*, which was display-referred: a filmic
  transform was re-compressing an already-final image. `OMNISIM_WGPU_AGX=0` returns to the legacy LDR
  path and was verified **sha256-equal** against the pre-change binary.
- **The Beauty Bench** (`projects/samples/rendering/beauty_bench.omniworld`) — the standing
  render-quality benchmark, agreed with the owner as the replacement for the too-big city when judging
  *look*. The city stays the *performance* baseline.
- **Linux can present without WREN.** `OmWgpuSurface` created only a Windows HWND surface source, so
  on Linux only the GL read-back-and-blit fallback was reachable — and that fallback lived inside
  `src/omnisim/wren/`, which is why the tree could not be deleted. Xlib, XCB and Wayland surface
  sources are added with **zero new includes and zero link flags**; platform detection is at
  **runtime**, because one binary must serve both xcb and wayland.

#### Changed

- **The main-view render throttle was pacing frame *starts* wrong, and fixing it was worth more than
  any shader.** It restarted its timer *after* the render returned and gated on `elapsed > 1000/FPS`,
  so the frame period was `renderTime + budget` — every millisecond of render cost charged on top of
  the budget — and a step quantum just under it (the city's `basicTimeStep 32` against `FPS 30`)
  systematically missed the first eligible boundary and halved the authored rate **for both
  renderers**. Now paced against an absolute due time. Measured (interleaved binary A/B, 75 s arms,
  GUI): **FAST** — WREN 9.4 → 21.3 mainFPS (2.26×), wgpu ~9.4 → ~15.6 (1.66×). **REALTIME
  head-to-head, post-fix** — wgpu **23.2 FPS at 0.858× realtime** against WREN **20.0 FPS at 0.648×**.
- **Two renderer optimisations worth ~30% between them.** Per-frame uniform staging is gone: the
  texshadow path staged and uploaded ~4.7 MB **every frame**. Those values moved into `LightU` and the
  Scene slots into a delta-detected, coalesced-upload buffer, so **a static scene uploads zero bytes
  per frame** — city renderMs **12 → 8–9**. Plus bounding-sphere culling against the shadow box and
  camera frustum, and redundant vertex/index bind dedup. Beauty Bench dumps verified
  **pixel-identical** across the staging change.
- **Screenshots, thumbnails, `--stream` frames and movie recording all come from wgpu now.** The GUI
  screenshot action, thumbnails and the mjpeg feed read the GL buffer via `glReadPixels`, which the
  wgpu present path never fills — so under a wgpu main view every capture was **stale GL content**.
  `grabWindowBufferNow` is now virtual and re-renders one synchronous wgpu frame with readback.
- **The capture / cinema service renders wgpu.** It writes `renderBackend` explicitly, and that pin
  was `"wren"` — so every still and every cinema frame the service had ever produced came out of the
  legacy renderer, *without* the scattered sky, OmniLight GI, PCSS, SSR, TAA or the camera pass the
  main view has had since 2026-08-19. Measured on the Beauty Bench, the WREN capture and the wgpu main
  view of the same world and camera differed on **98.6% of pixels** (mean 169/765) — a different
  image, not a slightly older look. `OMNISIM_CAPTURE_BACKEND` remains for A/B.
- **The main-view mouse hit test is `gui/OmScenePicker`**, replacing WREN's GL picking render. One
  depth-tested wgpu ID render over the same draw collector, with gizmo handle triangles appended as
  transient draws — so **drawn == draggable stays one triangle set** rather than two formulas that
  happen to agree. ⚠️ Recorded deviation: selection granularity is now the collector's per-draw `Solid`.

### Highlights

**A hardware-in-the-loop aircraft lane.** OmniSim had no aerodynamics and no aircraft of any kind.
[`packages/omnisim-hil/`](packages/omnisim-hil/) adds the simulator half of a HIL rig: a 1.4 m
fixed-wing delivery aircraft flown by real MAVLink from flight software in a separate process, so a
PX4 or ArduPilot — SITL or a board on a bench — is a substitution rather than a rewrite.

**Every vendor-derived robot package is replaced by OmniSim's own, and every tracked binary asset now
has a licence trail.** 3,272 assets: 3,186 covered by a licence or provenance file, 86 recorded as
own work, **0 uncovered and 0 baselined debt** — that number was 1,899 uncovered when the audit
started.

**Every external force was silently dropped for 16 days, and now works.** Three call sites gated on an
ODE field that has been NULL since the ODE deletion: `wb_supervisor_node_add_force`,
`..._add_force_with_offset`, and propeller thrust. The failure read as success — worlds loaded,
stepped, logged no ERROR and exited 0 while three helicopters in a shipped sample sat unchanged to
three decimal places for 491.5 s of simulated time.

**The flagship G1 demo had been dead for ten days and nothing caught it.** The newton 1.5 runtime
upgrade in v7.0.0 was migrated version-tolerantly in the *engine* and left `projects/policies/`
behind, so the in-engine hook threw on 100% of ticks and applied not one joint target — while the
engine log read `0 errors`, the sidecar read `finalised: true`, `--require-newton` passed, and the
launcher printed PASS on a robot lying face-up.

**The harness can command robots, not just read them.** `POST /robot/<def>/joints/set` writes joint
targets with settle-and-verify semantics, and `POST /robot/<def>/ik` runs the engine's batched IK
against the exact model the solver steps, as a pure preview.

### Added

- **⭐ `packages/omnisim-hil/` — hardware-in-the-loop for a fixed-wing aircraft.** `aero.py` is a
  component build-up over lifting surfaces (stall blend, rate damping, adverse yaw, and a propeller
  whose thrust falls with advance ratio — the term OmniSim's own `Propeller` node still lacks);
  `mavlink.py` is a stdlib-only MAVLink v2 codec byte-verified against pymavlink and importing nothing
  at runtime; `frames.py` is the **one** place ENU/FLU becomes NED/FRD; `autopilot/` is the flight
  software; `run_hil_demo.py` is one command — engine + autopilot + a measured report. PROTOCOL.md
  gains **section 17** for the MAVLink surface, the one OmniSim surface that is neither HTTP nor JSON.
  - **Measured, machine `9722d23d12a3`, CPU `mj_step`, n=1:** 3 of 3 waypoints, **834.1 m**, still
    airborne at the end; altitude error **0.61 m mean settled, 2.58 m worst**; **13,751 rx / 6,649 tx
    MAVLink messages, 0 bad CRC, 0 resync bytes**; in 4 m/s wind, 3 of 3 waypoints over 845.6 m at
    0.64 m mean error; payload envelope flies at 2.5 kg, degrades at 4.0 kg, cannot fly at 5.5 kg.
  - ⚠️ **Real-time pacing is measured because a HIL rig needs wall clock and nothing had checked it.**
    **1.0005× at `basicTimeStep 20`, but 0.516× at 8** — the wall interval pins to the ~15.6 ms Windows
    timer quantum, and the same scene steps in 0.561 ms under `--mode=fast`, so that is idle wait
    rather than scene cost. ⛔ **A fractional `basicTimeStep` truncates into `QTimer::start(int)` and
    paces permanently fast** (20.5 ms measured **1.0255×**) with no diagnostic and exit code 0.
  - **178 offline tests, no engine and no GPU**, asserting physical properties rather than recorded
    numbers. They caught two model bugs before anything flew and one after: a frame conversion using
    one matrix where two were needed, which put the aircraft's nose onto its right wing. That one
    passes every involution, orthonormality and round-trip property, so **only an assertion in
    physical units could catch it**.
  - ⚠️ **Scope, declared:** the software-in-the-loop tier is complete and this is the simulator half of
    HIL. **No hardware has been attached**, and the autopilot reads attitude from simulator ground
    truth, so a real EKF is not under test. The flight model is a build-up, not CFD.
- **⭐ `POST /robot/<def>/joints/set` — the harness commands joints.** A supervisor
  `Node.setJointPosition` is a **PD setpoint that converges over ticks**, not a teleport, so the verb
  applies targets, advances `settle_steps` (default 16) and then **measures**: per joint `{requested,
  commanded, clamped, position_before, achieved, error, moved, position_controllable, limits, note}`,
  never the argument echoed back. Live-verified: commanded 0.6 → achieved **0.5999995**; a 2.5 command
  on a ±1.0 joint clamps and is flagged; a limit-less wheel commanded 2.0 achieves **5.6e-14** and
  reports `position_controllable: false` with the mechanism named. ⚠️ **Measured and disclosed:** an
  active bridge in hold mode re-asserts its own targets every tick and **wins** (residual 0.42–0.70 rad
  on the UR5e) — command bridge-owned robots through their bridge. PROTOCOL.md §7.33.
- **⭐ `POST /robot/<def>/ik` — the engine's batched IK, wired end-to-end.** `World.solve_ik` landed in
  v7 with zero callers; it now has one, through harness route → supervisor verb → a **new
  controller-ABI function** `wb_supervisor_node_solve_ik` (wire opcode 104, **appended** — an ABI
  addition only, nothing moved) → engine decode → `OmNewtonBackend::solveIk`. **Pure preview: nothing
  moves.** Live closed loop on a passive 2R rig: residuals **3.1e-06 / 1.7e-06 / 1.2e-07 m**, Cartesian
  errors **5.4e-05 / 2.2e-04 / 1.0e-05 m**; an unreachable target reports its true **0.9000 m** residual
  (geometrically exact) and is not applied. ⚠️ First call in a fresh world compiles a warp kernel —
  **2368.7 ms warm disk cache, 8.3 s truly cold, 106–116 ms after** — disclosed in
  `verification.warmup`. `mujoco_warp` is documented **unverified**, not blocked. PROTOCOL.md §7.34.
- **`POST /servo_joint_positions` on the arm bridge — the non-blocking, superseding setpoint lane.**
  The goal verbs treat a joint command as a goal, so a trajectory controller's stream lands **in
  pieces** against their 409 — the measured MoveIt blocker recorded in v7.0.0. The servo verb returns
  at dispatch, is last-write-wins, preempts an in-flight goal and names it in `preempted`, and **never
  echoes the command as an achievement**. Verified live: **24 setpoints at ~18 Hz, 24/24 accepted, zero
  409s, parked max |error| 0.011 rad**. PROTOCOL.md §6.1.
- **⭐ Synthetic-data generation — aligned RGB / depth / instance dumps with seeded domain
  randomization.** `OMNISIM_WGPU_SYNTH_DUMP=<dir>` writes one ground-truth sample per trigger frame:
  rgb, uint16-millimetre depth, per-`Solid` instance ids (flat pick shader, sRGB off for exact byte
  round-trip) and a `meta.json` with intrinsics, extrinsics, light rig and id → node map. The GT passes
  reuse pre-existing render-target paths with a standard view-projection, so they are **pixel-aligned
  with the RGB by construction**. Measured: **12/12 samples, zero retries, ~11 s/sample**, instance map
  pixel-accurate at 46 instances. ⚠️ Needs a desktop session, and it is an **offline** generator.
- **⭐ A smart house, run by an OmniLink agent.** A physics-backed four-room home — thermal model,
  energy metering, hinged front door, six lights, oven/TV/coffee maker — over a 19-tool hub surface,
  plus a benchmark measuring interactive-only against persistent (hourly-wake) operation from
  `/scenario/metrics` only. Verified live 2026-08-19: parity scenario byte-identical between tiers;
  oven-left-on **60 vs 480 house-minutes, 5.47 vs 19.69 kWh, kitchen 31.5 vs 45.9 °C**; door breach
  **50 vs 285 house-minutes**. ⚠️ **Standing orders register but the hosted tick is disabled** —
  documented, not claimed.
- **`scripts/dev/thermal_guard.py`** — enforces the laptop temperature ceiling instead of intending to.
  The full post-deletion 148-world sweep peaked at **66 °C** under it.
- **Three new OmniBench lane 4 probes** — `phenomenon.supervisor_external_force`,
  `device.propeller_thrust`, `device.propeller_inflow` — so the external-force defect cannot regress
  unnoticed.
- **`<omnisim roughness="" metalness="" />`, an optional child of URDF `<material>`.** `OmUrdfImporter`
  **hardcoded roughness 0.5 / metalness 0** for every visual, so no URDF robot could look metallic
  however good its mesh. Negative means unset, so **every URDF that does not opt in emits those same
  constants byte-for-byte.**

### Fixed

- ⛔ **Every external force was silently dropped from 2026-08-08 until now.** `OmSolidMerger::mBody` is
  assigned NULL in the constructor and written nowhere else — the ODE deletion removed its only writer
  — and three call sites still read it as a liveness test. All three now gate on the Newton
  `bodyHandle()`, which is what `C_SUPERVISOR_NODE_ADD_TORQUE` had already been migrated to — **that
  asymmetry is exactly why torque worked and force did not**. Measured: `test_newton_external_wrench.py`
  **RED (v = 0.000000) → 2 passed**; lane 4 `phenomenon.supervisor_external_force` **broken → works**;
  `device.propeller_thrust` **broken → works**, acceleration ratio 1.0000155; the Mavic's wrenches
  **0 → 63,350 over 15,837 ticks**; the BLUE helicopter in `propeller.omniworld` **frozen → 355.3 m
  travelled**. ⚠️ **The documented revert hatch `OMNISIM_NEWTON_NO_EXT_FORCE` was itself inert** — it is
  read inside `applyExternalForceNewton`, which the dead gate prevented reaching, so an A/B against it
  would have measured "no movement" in both arms and concluded the feature worked.
- ⛔ **The flagship G1 walk had been dead for 10 days, and every guard passed on a fallen robot.**
  newton 1.5.0 removed `Control.joint_target_pos` / `joint_target_vel`; v7.0.0 migrated the **engine**
  version-tolerantly and left `projects/policies/` behind. Measured (`mujoco_warp` on `cuda:0`): the
  in-engine hook threw `AttributeError` on **2,210 ticks — 100% of them, from the very first** — and
  applied not one joint target. **Why nothing caught it:** the hook writes pymod exceptions *only* to
  `$OMNISIM_INENGINE_MPC_LOG`; the engine log read `0 errors, 7 warnings`, the sidecar read
  `finalised: true`, and the GUI branch of `run_walk_rl.sh` ran **no** deploy verification at all.
  Fixed across 33 files / 108 sites behind a `_ctl_target_pos(world)` shim mirroring the engine's
  `getattr` resolution; `verify_deploy_hook()` now asserts the hook actually **ran**, proven
  red-capable against a synthetic broken log before being trusted.
  - **After the fix, same world and solver:** x = **+12.90 m** forward, z = 0.734 upright, still walking
    at 115.3 s of sim time. ⚠️ **Still the weight-bearing λ=0.9 balance harness — NOT free-standing** —
    and it curves (−0.208 rad yaw, 1.24 m lateral drift over 12.9 m).
- ⛔ **Every terrain and every visual `Cone` was silently absent from every wgpu render for one commit.**
  The default branch of `acquireGeometryMesh` claimed in its own comment to serve "IndexedFaceSet, Mesh,
  ElevationGrid, Cone, …" and only ever handled `OmTriangleMeshGeometry` subclasses — `OmElevationGrid`
  and `OmCone` are bare `OmGeometry`, so the `dynamic_cast` returned null. Pre-deletion they fell
  through to the WREN static-mesh readback; deleting it **converted the stale comment into a silent
  hole**. Measured: `inf` received where an `ElevationGrid` obstacle stands at 2.0 m. **The mechanism
  generalises: a fallback is what lets stale coverage comments survive.**
- ⛔ **`Cloth`, `SoftBody` and `GranularGroup` were invisible to every wgpu-rendered `Camera`,
  `RangeFinder` and `Lidar`** — a regression against WREN, not a missing nicety. All three hang off
  `OmBaseNode`, not `Solid`, so the collector never walked them. The fix is structural because the
  failure was: `collectDynamicDraws()` is now the one entry point for non-Solid per-step-varying
  content. A second defect, which wiring the sensors alone would have shipped: the "do I need to
  re-upload the vertices?" decision lived in a function-local `static` keyed on the simulation clock,
  correct with one caller and wrong the instant there are two.
- **The dark aura around robots was the GTAO kernel, and the first attempt blamed the wrong subsystem.**
  WREN's `gtao.frag` used its projected radius unclamped; the wgpu port added a 64-pixel clamp as a perf
  guard and left the window derived from the **authored** radius, so every sample landed far inside the
  falloff start and was taken at full weight. Measured (static probe world, so the run-to-run noise floor
  is 0 px over threshold): halo pixels **17,064 → 620, a 27× reduction**, mean barely moving.
  - ⚠️ **The wrong turn is recorded in place, because the method matters more than the fix.** The
    2026-08-22 pass attributed the aura to OmniLight's probe spacing on the strength of **a single
    `OMNILIGHT=0` arm** — which moves *more* pixels (72,701) and is not the aura at all. Three avoidable
    errors produced it: one arm with no alternatives, no noise floor (controllers were live, so poses
    differed between runs), and **a pixel count read as a diagnosis**.
- **`Track` animation had been dead on both renderers since the ODE deletion.** `prePhysicsStep` gated
  the whole update on `mBodyID`, a `dBodyID` NULL since `bdc02139`, so belt elements, wheel spin, texture
  scroll and the `PositionSensor` were all frozen. ⛔ **The animation works again; the PROPULSION does
  not** — see Known limitations.
- A plain `Material`'s `emissiveColor` reaches wgpu sensor images again; **out-of-range now returns
  `+INFINITY`** on the no-noise wgpu range paths per the documented contract; appearance-less `Shape`s
  exist again; Lidar and RangeFinder range noise, resolution and motionBlur are ported to the wgpu
  readback; the `Pen`'s paint reaches a wgpu `Camera` image. Device HUD insets, supervisor labels and
  the manipulator gizmo are drawn again — from the *same* accessors the GUI hit-tests.

### Changed

- **⭐ The vendor-derived robot packages are replaced by OmniSim's own, with kinematics and inertials
  unchanged.** Worlds, controllers, bridges and trained policies that drove the old packages drive the
  new ones:

  | robot | what it replaced | path | robot id | DEF |
  |---|---|---|---|---|
  | **OmniArm 6** | a vendor-derived 6-axis cobot package | `projects/robots/omnisim/omniarm6/` | `omniarm6` | `OMNIARM6` |
  | **OmniArm 7** | a vendor-derived 7-axis cobot package | `projects/robots/omnisim/omniarm7/` | `omniarm7` | `OMNIARM7` |
  | **OmniTug 500** | a vendor-derived 4-wheeled ground tug package | `projects/robots/omnisim/omnitug500/` | `omnitug500` | `OMNITUG500` |
  | **OmniQuad** | the Boston Dynamics Spot package | `projects/robots/omnisim/omniquad/` | `omniquad` | `OMNIQUAD` |

  ⚠️ **The three vendor-derived packages are not named here, and that is deliberate.** Their
  publication permission was withdrawn, so this repository does not carry the vendor's or its
  products' names — and because **v8.0.0 is published as the only version of this repository**,
  there is no public predecessor to migrate a world *from*, which is the only thing a rename map
  would have been for. Spot is named because it is a different vendor and the reasoning for
  removing its geometry is a compliance record worth publishing: see
  [`docs/developer/spot-provenance-research.md`](docs/developer/spot-provenance-research.md).

  66 worlds and 15 demo controller directories were renamed with the packages; the arm bridge's
  `--robot` id is now `omniarm6`, the warehouse courier agent is `omnitug500_warehouse`, and the
  skill `spot_walk` is `omniquad_walk`. **OmniTug 500 remains a visual-only prop** — one link, no
  `<collision>` and no `<inertial>` — so worlds needing it to collide pair it with a companion `Robot`.
  - **Why the geometry had to go, not just the name.** The shipped `.glb` files still carried the
    vendor's internal drawing numbers in their embedded headers; the tug was a raw 347k-triangle CAD
    tessellation with the product's livery transcribed into the URDF as eight material colours; and the
    publication grant existed only as prose in a commit message, which the squashed public snapshot
    never carried anyway. Spot's 13 meshes each named their geometry object
    `02-042137-001-A00 TOP LEVEL DEFEATURED - NOT FOR PRODUCTION.<nnn>` — a manufacturer part number
    with a CAD release annotation carrying Blender's duplicate suffix: **one imported CAD assembly split
    13 ways, not 13 modelled parts**. Both ledgers had asserted the opposite in writing, and a wrong
    statement in a legal notice is worse than an honest uncertainty.
  - **The restyle is provably cosmetic on the arms and the tug**, verified by parsing the new XML against
    the originals: every joint origin, axis, limit, effort, mass, CoG, inertia tensor and collision shape
    identical across all 8 URDFs.
  - ⚠️ **Spot's collision geometry drives physics, so that one was MEASURED, not argued.** Colliders were
    *fitted*, each minimising voxel symmetric difference against the convex hull MuJoCo actually used
    (volume ratios 0.977–1.013). Interleaved A/B on one binary, control arm loading the old CAD colliders
    through the identical world / controller / policy / env: the velocity-conditioned RL walk produced
    **max |Δpos| = 0.00000000 m over ~11,000 steps**, 24.601 m travelled in both arms. *(That is a
    same-config A/B of two collider sets, not a determinism claim.)* ⚠️ **One scenario changed and is
    reported as a finding, not sold as a win:** the get-up policy now recovers where it previously
    flipped. Get-up is the one scenario whose contact set *is* the substituted colliders.
  - **All robot geometry is authored in this repository.** Visual shells come from committed generators
    with `--check` drift gates; collision geometry is primitive cylinders and boxes. **Zero binary
    geometry assets remain** — 46 MB of arm/tug meshes out and 484 KB of text in; Spot 20 MB → 145 KB;
    Robotiq 2F-140 3.0 MB → 13 KB. OBJ is a deliberate choice over a binary mesh: a reviewer can diff it,
    and `--check` proves byte-for-byte that what ships is what the committed generator emits.
- **ROS 2 joint commands select the bridge verb from `/capabilities`**, on both tiers. Verified live on
  Tier 2 (WSL Humble → Windows engine): **60 joint_command messages at 15 Hz, zero 409s, arm settled
  0.0185 rad from target**. ⚠️ **Tier 3's servo path is compile + unit verified only** — no live
  `controller_manager` run — **and MoveIt has still never been brought up.**
- **`/scene/spawn` and `/scene/delete` now disclose `RUNTIME_MUTATION_NOT_IN_SOLVER`.** The
  Newton/MuJoCo model is frozen at `finalizeWorld()`, so a mid-run spawn never reaches the solver (a
  dynamic body will not fall, a static body will not collide) and a delete leaves phantom colliders in.
  Both were measured in v7 and both succeeded **silently**. Every successful response now carries a
  `physics_warning` block naming the consequence and the reload fix, one `world.warning` event fires per
  verb per world-load, and `GET /capabilities` lists `scene.runtime_mutation_physics` under
  `not_supported`. **The engine is unchanged — this is honest reporting of an open gap, not a fix.**

### Licensing, provenance and redistribution

This release is the first that a third party can redistribute with the whole licence trail in the box.

- **Every tracked binary asset now has a licence trail.** 3,272 assets: **3,186 covered** by a licence
  or provenance file at or above them, **86 recorded as own work**, **0 uncovered, 0 baselined debt**.
  It was 1,899 uncovered when the audit started.
  - ⚠️ **That count grew because the gate had been looking at the wrong set, not because assets were
    added.** `ASSET_EXTENSIONS` named no `.npz`, `.svg`, `.gif`, `.ico`, `.exr`, `.mp4` or `.pdf` — and
    an extension the set does not name is not merely unchecked, it is **invisible**: the gate can
    report "0 uncovered" while an unattributable file sits in the tree, because it never looked. Widening
    it surfaced 21 previously-invisible files (20 `.npz` and an `.ico`). None was a real licensing
    problem — but that was luck, and the reason it was luck is that nothing would have said otherwise.
    Five missing `PROVENANCE.md` records were written so the wider set still reads 0 uncovered.
- **Three gates enforce it, and they run in CI** —
  [`.github/workflows/licence-provenance.yml`](.github/workflows/licence-provenance.yml), deliberately
  unfiltered by path: [`test_license.py`](tests/sources/test_license.py) (source headers; its exemption
  baseline is **empty and stays empty** — an entry there is debt, not permission),
  [`test_asset_provenance.py`](tests/sources/test_asset_provenance.py) (every binary needs a trail), and
  [`test_licence_pointers.py`](tests/sources/test_licence_pointers.py) (every path cited by `NOTICE` or
  `THIRD_PARTY_NOTICES.md` resolves — an attribution pointing at a deleted file has silently stopped
  being made). **The existing sources suite had never run in CI at all.**
  - The pointer gate now also classifies a third case: a component **held from the public snapshot**,
    whose licence text is removed *together with* the material it attributes. That is correct rather
    than broken — but the exemption list is checked against the real deny-list by its own test, so it
    cannot become a place to hide a deletion.
- ⛔ **The brand artwork was inside the Apache grant, and now is not.** `TRADEMARKS.md` reserved the
  *marks*; nothing reserved the *copyright in the image files*, and Apache-2.0 §6 does not reach them —
  on the face of the repository, anyone could copy, alter and redistribute the orb and wordmarks so
  long as they stopped short of using them *as* marks. Now reserved explicitly in
  [`resources/branding/LICENSE`](resources/branding/LICENSE), in carve-out item 5 of both attribution
  documents, and in a `TRADEMARKS.md` section that separates the two rights. ✅ **Reproducing the assets
  unmodified to refer factually to OmniSim needs no permission** — that was always the intent and is now
  written down.
- **`NOTICE` now opens with what a redistributor must actually carry** — Apache-2.0 §4's four
  obligations in this project's concrete terms, plus the two things the licence does *not* give you.
- **`THIRD_PARTY_NOTICES.md` was missing its `## 3.` heading entirely**, jumping §2 → §4, which is why
  glad, GLM, stb, SipHash, Qt 6, wgpu-native, OpenAL Soft and the rest had no rows while `NOTICE`
  listed them. The section is restored and populated, including the LGPL components and the linkage
  posture that keeps their obligations satisfiable.
- **Corrections to claims that were simply wrong**, each re-measured rather than re-copied: the PROTO
  census (453/260 → **452/259**), a bundled component that is not in the tree (SVOX Pico), "Spot" still
  listed among marks used nominatively, a carve-out asserting an Assimp obligation was undischarged when
  it had been discharged, and a `template.proto` claim the file itself contradicted.
- **Contribution provenance is enforced, not just documented.** DCO sign-off now has a
  `pull_request`-scoped check, and the PR template carries a provenance checklist whose central question
  is the one that found the Boston Dynamics, Robotiq and Orbbec geometry: **is the licensor the design
  owner?** A vendor's permissively-licensed ROS package does not cover another manufacturer's CAD inside
  it.

### Removed

- ⛔ **The WREN renderer** — see the banner and the renderer section above.
- ⚠️ **The `Skin` node** (1,230 LOC, its `.wrl` schema and its Add-Node icon). A commit four days earlier
  had **refused** this deletion, and that refusal was correct on the question it asked: it measured Skin
  working on Newton in both documented modes and kept it rather than delete a working documented feature
  on "our corpus does not use it". This supersedes it on a **different** question, with owner approval:
  `OmSkin.cpp` carried **108 live `wr_*` call sites**, the largest single-file concentration outside
  `src/omnisim/wren`, and mesh deformation in the render path is exactly what has no home once WREN goes.
  The schema had to go in the *same* change as the factory entry, because `readAllModels()`
  directory-scans `resources/nodes/*.wrl` — a surviving `.wrl` would make the parser accept a node the
  factory then returns NULL for. ✅ **The public ABI is untouched**: `include/controller/` and
  `src/controller/` have a **zero-line diff**, `WB_NODE_SKIN` stays mid-enum at index 58 because it is
  positional public ABI, and all six public `wb_skin_*` functions guard on the NULL that
  `skin_get_struct()` now returns.
- ⚠️ **`Helmet.proto` and `Telephone.proto`.** Both declared MIT citing Khronos `glTF-WebGL-PBR` — MIT is
  that repository's **code** licence, and the DamagedHelmet model is **CC BY-NC**, which cannot ship
  under Apache-2.0 at all.
- ⚠️ **`VirtualRealityHeadset`** — a Windows-only OpenVR/SteamVR node rendered entirely through WREN with
  **zero consumers**: no `.wrl` schema, no PROTO, no world and no controller referenced it. Because there
  was no schema, **no world file can break**.
- ⚠️ **The Valkyrie and Digit robot packages** (161 files, 33 MB). Neither had a valid grant: Digit's
  upstream is one academic's Julia package with no LICENSE on any branch, and Valkyrie's NOSA v1.3 file
  was an **unfilled template** with every blank — including the name of the government agency — left as
  `_____`, so it did not satisfy its own section 3. Both were already publish-denied; they were **deleted
  because a deny-list depends on a file staying correct forever and deletion does not.**
- ⚠️ **The arm bridge's vendor hardware backend**, its operator doc, bring-up checklist, wiring test and
  the `--hardware-backend` value that selected it.
- **~46 MB of third-party mesh data plus a further ~62 MB of unlicensed or orphaned assets**: 190 orphaned
  sky cubemaps, the Orbbec Astra sensor mesh inside the ROBOTIS package loaded by nothing, 4 Spot foot
  collision STLs referenced by no URDF, 6 unlicensed splash renders of other manufacturers' robots that
  nothing displayed, and 9 orphan textures byte-identical to upstream assets under a product-scoped grant.
  - ⚠️ **Where the suspicion was NOT borne out, that is recorded too.** Binary forensics over 1,104 files
    found zero trace of any texture marketplace, so `projects/appearances/` (333 files) is clean and now
    says so.
  - **`compilation_timestamp.h` was assembled from Stack Overflow answers (CC BY-SA) with no licence.**
    Replaced with a clean-room implementation, proven identical over 1,463 date cases.
  - **Blender's Suzanne was replaced** by a committed-generator torus knot. Blender's manual says the GPL
    covers the application, "not the artwork you create with it" — but Suzanne is not artwork a user
    created, it is a coordinate table shipped **inside** GPL source.
  - **The LAFAN1 motion-data cascade.** CC BY-NC-ND 4.0 has two independent blockers, either fatal: §2(a)(1)
    grants the right to reproduce "but not Share, Adapted Material", and NC is incompatible with Apache-2.0.
    43 ghost/checkpoint entries are denied in three labelled classes, the second-order re-recordings
    **precautionary** with the argument set out both ways rather than asserted. ✅ The flagship G1 walk rides
    a Unitree-lineage ghost (BSD-3, licence in-tree) and is untouched. ⚠️ The old deny entry said in its own
    words that the terms "were never verified", then sat unchanged while nobody verified them. **It took
    twenty minutes; the licence was a file in the root of a public repository the whole time.**

### Known limitations — read before quoting any result

By the tree's own executed measurement (OmniBench lane 4, machine `9722d23d12a3`), the matrix reads
**48 probes: 34 works / 5 degraded / 5 broken / 4 absent / 0 inconclusive — 77%**, against 45 probes and
78% at v7.0.0. ⚠️ **That is three probes ADDED, not a regression:** two of the new rows are `works`
(external force and propeller thrust, both previously untestable because the capability was dead) and one
is `broken` by design. No existing verdict moved. Where prose disagrees with
[the generated matrix](docs/benchmarks/lane4-capability-matrix.md), **the matrix is the measurement**.

- ⛔ **`Propeller` thrust does not decay with airspeed.** `OmPropeller.cpp` pins the speed-of-advance term
  to `V = 0.0`, so `thrustConstants[1]` has no effect. Measured: the airframe went from 1.11 to 12.30 m/s
  of axial airspeed and the descent acceleration did not move (4.8100 → 4.8100 m/s², ratio 1.000).
- ⛔ **`Track` PROPULSION is dead under Newton.** The animation was repaired this release; the propulsion
  was not. `contactSurfaceVelocity()` has **zero readers** — it was consumed by ODE's
  contact-surface-velocity mechanism and nothing replaced it. A tracked robot's belt animates and its
  chassis does not move.
- ⛔ **A motorised `BallJoint` still does not actuate**, while its angle readback travels — so a controller
  polling `getPosition()` is told it is moving. Hinge2 works. Unchanged from v7.0.0.
- ⛔ **A node deleted at runtime keeps colliding, and a node spawned at runtime never reaches the solver.**
  Same frozen MuJoCo model, opposite symptoms. Now **disclosed on every response**, but not fixed.
- ⛔ Unchanged from v7.0.0: `ContactProperties.bounce` is declared-but-never-read, every triangle-mesh
  collider is silently convexified, a compound `boundingObject` registers only its first child by default,
  closed kinematic chains build no world at all, `setPosition()` is silently ignored on a motor with no
  declared limits, and no deformable has a readback surface.
- ⚠️ **Bitwise reproducibility remains scoped to the CPU `mj_step` path and is REFUTED on the GPU
  `mujoco_warp` path.** Nothing here changes that, and no cross-machine bitwise claim exists.
  [docs/benchmarks/determinism-scope.md](docs/benchmarks/determinism-scope.md) is the source of truth.
- ⚠️ **Every humanoid result in this tree runs on a weight-bearing balance harness (λ=0.9,
  `HARNESS_KZ=2000`, up to ≈700 N of lift plus ±350 N·m of attitude authority) — a durable free-standing
  humanoid walk remains an OPEN problem.** Quadrupeds carry no harness. The stair climb is the one
  exception (`HARNESS_KZ=0`), and its 3 cm riser is a measured ceiling, not a config choice.
- ⚠️ **No sim-to-real, and this release moved further from it, not closer.** With the vendor hardware
  backend removed, this tree has **no path to a physical arm**. The HIL lane has had **no hardware
  attached**. macOS remains untested and, with no fallback solver and now no fallback renderer, has no
  verified physics *or* rendering path.

---

## [v7.0.0] — 2026-08-17

> ⛔ **READ THIS IF YOU HAVE A CONTROLLER. This release completes the removal of the name "webots"
> from the shipped OmniSim controller API, and one of the four breaks is an ABI break.** In total,
> four things stop working, listed with their fixes under **Removed** below:
>
> 1. **`#include <webots/robot.h>`** — the 91 `webots/` forwarder headers are deleted. *Source break.*
>    Rewrite `webots/` to `omnisim/` in the include line.
> 2. **`from controller import Robot`** — the Python `controller` shim is deleted. *Source break.*
>    Rewrite the import to `from omnisim import Robot`.
> 3. **`WEBOTS_CONTROLLER_URL`, `WEBOTS_HOME`, `WEBOTS_LIBRARY_PATH`** and their siblings — the
>    `WEBOTS_*` runtime environment variables are warned about once and then **ignored**. *Environment
>    break.* Rename them to `OMNISIM_*` wherever they are set. (`WEBOTS_HOME` still works at *build*
>    time.)
> 4. ⛔ **`using namespace webots;`** — the C++ classes are now in **`namespace omnisim`**. **This one
>    is an ABI break, not a source break:** the namespace is part of every mangled C++ symbol, so
>    **every C++ controller must be RECOMPILED**, and an already-built one fails at load rather than
>    misbehaving. There is **no** `namespace webots = omnisim;` alias, deliberately.
>
> **What does NOT break, and will not:** the **C API**. `wb_*` functions (497 exports), the `Wb*`
> types and the `wbu_*` utilities are untouched — none of them contains the string "webots", so none
> of them blocked the goal. **C controllers keep their symbols and need no recompile; only C++ ones
> do.** The `webots-<id>` tmp folder and `\\.\pipe\webots-…` IPC names are also unchanged, and
> Cyberbotics copyright headers, `NOTICE`, `THIRD_PARTY_NOTICES.md` and `TRADEMARKS.md` are untouched
> — Apache-2.0 §4 attribution is independent of this migration.
>
> Separately, and unrelated to the naming work: the **`Radio` and `Microphone` nodes are deleted**
> after being measured inert, so a world declaring one now exits 1. **`Skin` was proposed for the
> same retirement and was KEPT** — it was measured working.

### ROS 2 — a declared non-goal, reversed and shipped in three tiers

#### Added

- **⭐ ROS 2 Tier 3 — a `ros2_control` hardware interface, so stock controllers drive an OmniSim
  robot.** [`packages/omnisim-ros2/src/omnisim_ros2_control/`](packages/omnisim-ros2/src/omnisim_ros2_control/)
  ships `omnisim_ros2_control/OmniSimSystem`, a C++ `hardware_interface::SystemInterface`.
  `controller_manager` sees ordinary hardware: **state** comes from the harness's
  `GET /robot/<def>/joints` (position + velocity interfaces), **commands** go to the robot's own
  bridge — `POST /set_velocity` for a differential-drive base, `POST /set_joint_positions` for an
  arm. Both endpoints already existed, so **no OmniSim-side surface was added and the engine is
  untouched.** A `diff_drive_controller` command is folded back into a body twist by the exact
  algebraic inverse of the controller's own wheel kinematics, which round-trips bit-for-bit when
  `wheel_radius` / `wheel_separation` match the bridge's (verified live at DEBUG:
  `left=0.302847 right=0.302847 r=0.165100 b=0.570800 -> linear=0.050000 angular=0.000000`).
  Worked example in one command: `ros2 launch omnisim_ros2_control husky_diff_drive.launch.py`.
  Dependency-free — a ~200-line JSON reader and a POSIX-socket HTTP client rather than
  `nlohmann/json` and libcurl, so `colcon build` needs nothing that `ros-humble-desktop` +
  `ros2_control` does not already provide. 10 gtest cases (9 pass, 1 skips for want of a
  comma-decimal locale) cover the two things that fail *silently*: the JSON reader, where a
  mis-parse looks like a stationary robot and a `null` velocity read as `0.0` looks like a stopped
  joint, and the diff-drive fold, where a wrong sign looks like a robot that turns instead of
  driving.
- **Tier 3 verified against the simulator, not against the command.** Machine `9722d23d12a3`
  (RTX 3060 laptop, CPU `mj_step`, binary `13906cc6f12451eb`), ROS 2 Humble + `ros2_control` 2.54.0
  in WSL2: `ros2 control list_hardware_interfaces` reports four claimed `velocity` command
  interfaces and eight `position`/`velocity` state interfaces; `joint_state_broadcaster` and
  `diff_drive_controller` both report `active`; and a `cmd_vel_unstamped` publish drove the Husky
  from `x = -0.00000` to `x = +5.59211` — **5.5921 m over 195.22 s of simulated time**, with the
  pose and the clock read out of the simulator. Three alternating A/B repetitions against a control
  arm that posts `/set_velocity` directly with ROS removed entirely agree to within ~4%
  (0.0380/0.0374/0.0440 m/s vs 0.0428/0.0409/0.0402 m/s for a commanded 0.05), so the 12-25%
  shortfall is **the robot and its bridge, not the ROS layer** — it is present with ROS absent.
- **ROS 2 sensor topics — `sensor_msgs/Imu`, `LaserScan` and GPS (Tier 2).** The blocker was never
  the ROS side: `GET /robot/<def>/sensor/<name>` on the harness is a deliberate 501, because a
  supervisor cannot honestly read a device it does not own, and **no shipped bridge implemented the
  optional `/read_sensor` verb** ([PROTOCOL.md §6.6](PROTOCOL.md)). So the source was built first.
  `omnilink_mobile_bridge` now serves `/read_sensor` + `/list_sensors`, marshalling every device
  read onto the sim thread (the controller API is not thread-safe) and enabling devices lazily,
  reporting `warming_up` for the window before the first sample rather than substituting a zero.
  A new `sensor_node` publishes `/imu/data`, `/scan` and `/gps/local`, with sensor frames latched
  onto `/tf_static` from **supervisor-measured** mount poses (`base_link → base_laser` at
  `0.2012, 0, 0.505`), not assumed identities. §6.6 was underspecified — it defined only a scalar
  `value` + `unit` — so it now carries per-type payload shapes, a `layout` block for lidar, the
  mandatory GPS `coordinate_system`, the `null` no-return encoding and the warm-up contract.
  **Verified with values that change under motion**, not merely present: GPS `x` `0.0000 → +5.5918`
  over a drive, lidar finite returns `541 → 190`, and IMU yaw `+0.1300` matching the bridge's own
  `0.1300` after a turn.
- **Partial by measurement, and declared as such.** ⛔ OmniSim's **`Gyro` reads a constant
  `[0,0,0]`** while the robot is demonstrably rotating, and its **`Accelerometer` never produces a
  sample at all** — not even gravity — while the `InertialUnit` emitted into the *same* carrier
  Solid tracks yaw to 4 decimals, so the defect is device-type-specific rather than a URDF-import
  artefact. `sensor_msgs/Imu` therefore ships a real orientation with
  `angular_velocity_covariance[0]` and `linear_acceleration_covariance[0]` set to **`-1`**, the
  ROS convention for "not available", instead of zeros that read as measurements. No robot in the
  tree has a camera, so no `Image`/`CameraInfo` was shipped. A local GPS is published as
  `PointStamped`, not `NavSatFix`, because its values are metres and not degrees.
- **Husky URDF gained a lidar** — a SICK LMS111 on the sensor arch (270°, 541 samples, 0.1–20 m),
  so the `LaserScan` path could be verified against a real device rather than shipped untested.
  ⚠️ It is **inert** unless the world is loaded with `OMNISIM_URDF_USE_SENSORS=1`: the importer
  parses `<gazebo>` sensor blocks always and drops them at emit time when that is unset. Verified
  by a control run — the same world reports `devices: []` without the flag, and 5 devices with it.
  That flag is also the single most likely reason a URDF robot appears to have no sensors; its own
  source comment claiming the emission path crashes device registration is **stale**.

#### Changed

- **ROS 2 HTTP transport now reuses connections — measured 7.9× and zero socket churn.** The bridge
  sets `protocol_version = "HTTP/1.1"` (safe because every response already carried an accurate
  `Content-Length`, with error paths closing the connection since several of them reject *before*
  reading the request body), and the client replaced `urllib.urlopen` — which opens a new socket per
  request and offers no way to reuse one — with a pooled `http.client` connection. Measured A/B,
  300 requests against the live bridge, machine `9722d23d12a3`: **114.8 → 908.8 req/s** and
  **+300 → +0** sockets left in `TIME_WAIT`. That matters more than the speed: each closed socket
  holds an ephemeral port for 120 s against Windows' 16,384, which is how a ~50 Hz bringup
  previously reached 17,487 sockets in `TIME_WAIT` and `WinError 10048`. ⚠️ The **World Harness
  still speaks HTTP/1.0**, so harness-facing nodes still pay a connection per request; the pool
  detects that peer and degrades automatically. `OMNISIM_ROS2_KEEPALIVE=0` reverts the client.

- **⭐ ROS 2 support — reversing a declared non-goal.** OmniSim now implements the ROS 2
  [`simulation_interfaces`](https://github.com/ros-simulation/simulation_interfaces) standard
  (v2.1.0) — the same one Gazebo, Isaac Sim and O3DE implement — plus a live robot surface, as a
  **sidecar package** at [`packages/omnisim-ros2/`](packages/omnisim-ros2/). ROS 2 had been a
  *documented non-goal* since 2026-07-10; the project owner reversed that decision.
  **Tier 1:** 15 services + the `SimulateSteps` action, each mapped onto a harness endpoint that
  already existed. **Tier 2:** `/clock` (with `use_sim_time`), `/tf` + `/tf_static` as a real
  transform tree, `<robot>/joint_states`, `nav_msgs/Odometry` on `/odom`, and `cmd_vel` +
  `joint_command` routed to the robot's own bridge — **never** the harness, because the supervisor
  can teleport a body but cannot drive a motor it does not own (the same reason
  `GET /robot/<def>/sensor/<name>` is a 501). ⛔ **Tier 3 (`ros2_control`) was NOT implemented at
  this point in the log; it landed separately (see the Tier 3 entry above), and MoveIt is still
  out of reach.** ✅ **The engine is unchanged**: no `rclcpp` in
  `src/omnisim/`, no ROS in the engine Makefiles, and a non-ROS user's build is byte-identical.
  90 unit tests need neither ROS nor a simulator. Full story, limitations and the reversed-decision
  history: [docs/developer/ros2-integration.md](docs/developer/ros2-integration.md).
- **`packages/omnisim-ros2/tools/wsl_harness_link.py`** — a Windows↔WSL2 transport needing no
  Administrator rights. OmniSim's engine is Windows-primary and ROS 2 is Linux, but WSL2 cannot dial
  a Windows-host service without a firewall rule; this inverts the direction (Windows dials into
  WSL and parks connections as return paths). A byte pump, not an HTTP proxy, so every present and
  future harness endpoint works unchanged.

#### Known limitations — read before quoting any ROS 2 result

- **⛔ Tier 3 covers velocity-commanded bases only — MoveIt is still out of reach, and the blocker
  is OmniSim's arm bridge, not ROS.** `omnilink_arm_bridge` treats a joint command as a **goal**,
  not a setpoint. Measured on `omnilink_ur5e.omniworld`, three commands 50 ms apart:
  `#1 HTTP 200 accepted=True`, `#2 HTTP 409 accepted=False error='busy'` ("this
  set_joint_positions was NOT applied"), `#3 HTTP 200 accepted=True` once the previous
  interpolation had finished. A trajectory controller writes a setpoint every cycle, so its stream
  would land **in pieces** — worse than failing outright. `command_mode: joint_positions` is
  implemented and logs that exact mechanism on a 409 instead of retrying, but nothing in the tree
  can serve it until a bridge grows a non-blocking, superseding servo verb. That is an OmniSim-side
  change.
- **⛔ Nav2 has never been brought up against OmniSim.** `/odom`, `/cmd_vel`, `/tf`, `use_sim_time`
  and now `ros2_control` all exist, which is everything a Nav2 bring-up consumes — but nobody has
  run one. Treat it as *unblocked*, not as *working*. OmniSim is also still absent from the
  `ros2_control` simulator registry.
- **⚠ `update_rate` is not the actuation rate, and the ceiling is OmniSim's supervisor round trip,
  not TCP.** `read()`/`write()` do no I/O — they swap values with a snapshot under a mutex, so any
  `controller_manager` rate is safe — and one background thread owns the HTTP at `comms_rate_hz`.
  Measured with `omnilink_husky.omniworld` loaded `light`: `GET /robot/HUSKY/joints` costs
  **21.01 ms** (47.5 req/s) against **4.48 ms** (221.8 req/s) for a bare `GET /healthz` on the same
  server — 4.7×, because the joint read is a supervisor RPC serviced at an engine step boundary.
  One read + one write is ~22 ms, so the **hard ceiling is near 45 Hz**; the shipped default is
  25 Hz. The ephemeral-port budget is a *second*, looser ceiling near 136 Hz, so latency binds
  first.
- **⚠ `cmd_vel_timeout` is evaluated on the SIMULATION clock, and the failure is silent.** Under
  `use_sim_time: true` — which this stack needs, because the harness runs worlds at ~13× realtime —
  the stock 0.5 s expires between consecutive messages from a 20 Hz *wall-clock* publisher.
  Measured: the Husky moved **0.0000 m** against a commanded 0.2 m/s while
  `/diff_drive_controller/cmd_vel_out` published all zeros, the controller reported no error and
  the hardware plugin reported 0 failures at its full 19.99 Hz. The only way to see it was to read
  the robot's pose out of the simulator. The shipped config uses 2.0 s and explains how to scale it.
- **⚠ Position commands are silently ignored on an unlimited OmniSim motor, so the plugin refuses
  to activate one.** A motor with no `minPosition`/`maxPosition` is built as a velocity wheel with
  `ke = 0` and `setPosition()` on it does nothing — the Husky's four wheels are exactly that, and
  the engine says so per device. Rather than falling back to velocity control and reporting
  success, the plugin reads the harness's own `lower`/`upper` and fails activation, naming the
  joints. There are no effort interfaces at all: OmniSim exposes no joint effort on any surface, so
  a URDF asking for one is rejected instead of being handed a fabricated zero.
- **⚠ Never read the simulation clock and a pose from two different harness calls.**
  `GET /sim/state` and `GET /robots` are two supervisor RPCs serviced at *different* engine step
  boundaries; under `--mode=fast` the gap between them is seconds of simulated time and it moves
  with HTTP load. The same 10-second drive measured **+50.6%** and **−21.8%** of its commanded speed
  depending only on how busy the harness was — both artefacts. Take the clock and the pose from one
  response, and remember that **a wheel cannot exceed its own velocity setpoint**: an achieved value
  that beats its command is a bad denominator, not a fast robot.
- **`EntityState.twist` and `.acceleration` are returned as ZEROS** because the harness measures no
  body velocity. They are *unmeasured*, not observed-to-be-zero — do not read a zero twist as "this
  object is at rest". Real velocities are on `/odom` and in `JointState.velocity`.
- **There is no pause.** The engine free-runs, so `SIMULATION_STATE_PAUSE` is not advertised and
  `StepSimulation` means "advance at least N basic steps", not "exactly N from a frozen state".
- **⚠ A wall-clock `cmd_vel` duration is not a distance.** Measured on machine `9722d23d12a3`, the
  harness advanced simulation time **39.920 s in 3.015 s of wall time — 13.24× realtime**. The same
  trap bites `get_clock()`: under `use_sim_time` it returns *simulation* time, which is the wrong
  clock for a wall-clock watchdog.
- **⚠ Publish rates cost TCP connections.** Neither the harness nor the per-robot bridges set
  `protocol_version`, so both speak **HTTP/1.0 and close after every response** — no keep-alive, one
  socket per tick. Measured: a 50 Hz bringup drove Windows to **17,487 sockets in `TIME_WAIT`
  against a 16,384-port ephemeral range**, after which `connect()` failed with `WinError 10048` —
  which reads like a bind conflict but is exhaustion, and surfaces as nodes reporting a perfectly
  healthy harness as unreachable. The launch defaults (~45 requests/s) sit inside that budget.

### Highlights

**OmniSim simulates things that are not rigid bodies.** A `Cloth` node on Newton's `SolverVBD`,
coupled to the rigid `SolverMuJoCo` scene over one shared model, and a volumetric tet-FEM
`SoftBody`. A gripper picking up a T-shirt is **measured against a negative control** — tracking
error −1.50 mm and slip 4.17 mm on 616 particles, against −173.06 mm for a control whose jaws never
close, corroborated by a second instrument (commanded-vs-measured jaw gap) that cannot be confounded
by which particles are labelled "gripped". Closing on the hem **misses**, and is reported as a miss.
⚠️ Read the disclosures with the numbers: the garment's shoulders are pinned, so these are *tracking*
figures rather than *load-bearing* ones; self-contact must be off to grasp and on to drape; and the
composed **fold** is **not** demonstrated. See [cloth-simulation.md](docs/developer/cloth-simulation.md).

**Terrain is real.** `ElevationGrid` is a native Newton heightfield collider — OmniBench lane 4
flipped it `broken → works`.

**The engine got 2.1–3.6× faster per step**, and the cause was not where a code reading put it:
warp defaulted to `cuda:0`, so Newton's state arrays lived on the GPU while `mj_step` ran on the CPU,
and every tick paid a PCIe round trip for a simulation that never touched the GPU. Measured on machine
`9722d23d12a3`: 5 boxes 1.582 → 0.446 ms/step, 50 boxes 2.287 → 1.110, an 8-Husky motorised world
2.083 → 0.827 — physics-neutral, with trajectories identical and determinism still bitwise on the CPU
solver.

**The Newton runtime moved to newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0**, migrated
version-tolerantly across four API breaks. Physics identical; the CPU path ~20% faster per step.

**A world that builds no physics now FAILS instead of printing PASS.** A fatal Newton finalize
failure was logged at WARNING, so a loop-closing `SolidReference` or a `Cone` collider produced a
world frozen at its authored pose while `run-headless` reported `0 errors … PASS`, exit 0. It is an
ERROR now, and the first thing it caught was one of our own shipped sample worlds.

**The README states its case in a table** — agent-native surface, hardware floor and footprint,
deformables, Newton maturity and licence, against Webots, Gazebo, Isaac Sim and Isaac Lab. Every
OmniSim cell is measured on one named machine and every competitor cell is their own dated
documentation. Where we lose — ROS 2, photorealism, sim-to-real, free-standing humanoid walk — is in
the same table.

### Added

- **`WorldInfo.newtonNoslipIterations`** — MuJoCo's `mjOption.noslip_iterations`, reachable from a
  world file for the first time. It is a Gauss-Seidel pass over the **friction constraints only**,
  run after the main solve, and it removes the tangential *drift* a soft friction constraint
  accumulates under sustained load — the failure where a gripper holds its commanded normal force
  and the part still creeps out. `SolverMuJoCo` has no kwarg for it and calls `mujoco_warp.put_model`
  unconditionally (which **raises** on a non-zero noslip), so the engine writes `mj_model.opt` after
  solver construction; on `newtonSolver "mujoco_warp"` the request is declined with one WARNING
  rather than silently ignored. Default `0` is MuJoCo's own stock value, so **every existing world is
  byte-identical** — verified: the ladder0 rung-8 scene reproduces all six measurements to the last
  digit on the new binary. Hatch: `OMNISIM_NEWTON_NOSLIP` (value-parsed, `=0` forces it off). The
  finalise line, the `.newton.json` sidecar and `OMNISIM_NEWTON_DUMP_MJMODEL` all now carry it, so a
  run says whether it was on.
  - ⚠ **It is not a general grasp fix, and it was measured before being described as one.** On
    ladder0 rung 8 (0.2 kg part, 3 N per pad at µ = 3 — 9× the Coulomb bound) it moved `carry_rel`
    from 0.4747 m to 0.4796 m and the payload was dropped either way, while `newtonCone "elliptic"`
    + `newtonImpratio 10` took the same scene to 0.0026 m. On bare MuJoCo's stock `solref` the same
    pass fixes the same scene outright. Try it when a grasp creeps; do not assume it.

- **`Cloth` — deformable fabric on Newton's `SolverVBD`.** Two authoring modes: a parametric grid
  patch (`dimX`/`dimY`/`cellX`/`cellY`) and a mesh garment (`url`), the latter with a mandatory mesh
  clean because orphan vertices become mass-0 immovable nails. `SolverMuJoCo` keeps every rigid body
  and joint — so the arm keeps the exact PD servo, armature and friction cone it was tuned for — while
  particles go to `SolverVBD`, exchanged by `SolverCoupledProxy` over one Newton `Model`. A
  `newtonClothSelfContact` world field, because there is no correct default: draping needs it on,
  grasping needs it off, and getting it wrong costs 24× on tracking error. Measured on machine
  `9722d23d12a3`: a 289-particle drape at 1.52–2.82 ms/step (2.7–5.3× real time); a 37,650-particle
  hero garment ships. ⚠️ Cloth is a **GPU** feature — on CPU the same drape runs at 6.7 fps.
- **`SoftBody` — volumetric tet-FEM deformables.** Measured: a pinned face holds 1.500000 exactly
  over 5 s while the free end sags 43.7 mm; a block dropped from z=0.5 rests with its lowest particle
  at z=0.009284, on its own 0.01 particle radius, with no tunnelling; and a 15.6 kg soft block presses
  a 2 kg **dynamic** box from 0.060000 to 0.059385 and holds — genuine two-way soft→rigid coupling.
- **`ElevationGrid` is a native Newton heightfield collider.** Previously scored `broken` by
  OmniBench lane 4; re-measured `works`.
- **Batched inverse kinematics on the live model**, in OmniSim joint slots.
- **Worlds have their own extension, `.omniworld`.** The policy is **dual-read, single-write**: the
  engine, harness, tests and every script accept `.omniworld` and `.wbt` interchangeably and
  indefinitely — external forks exist and a `.wbt` world must keep working — while everything newly
  written gets `.omniworld`. 661 worlds migrated. It is a capability signal, not a rebrand:
  `URDFRobot`, `Cloth`, `SoftBody`, the `omnisim://` scheme and every `newton*` field are unloadable
  in Webots, and `Fluid`/`immersionProperties` now hard-error here.
- **`POST /world/sync`** — the default agent edit loop. It diffs the file against the snapshot that
  produced the running world and auto-selects a live pose batch (325 ms) or a full hot reload
  (2818 ms) against a 6.37 s headless floor, so the agent no longer has to classify its own edit.
- **`run-headless --until-finalized`** — stops the moment Newton finalises and the sidecar exists,
  instead of sleeping out a guessed `--duration`. 15.52 s → 6.37 s, same PASS, same sidecar.
- **`scripts/dev/batch_validate.py`** — one engine process hot-reloading N worlds instead of N fresh
  engines. Measured on 19 worlds: 174 s → 34.6 s at `-j4`. It also dodges the engine startup race —
  the fresh-process baseline scored 18/19 while every reuse configuration scored 19/19.
- **`NEWTON_WORLD_NOT_BUILT`** harness diagnostic code, distinct from `NEWTON_RUNTIME_BROKEN`: the
  runtime is fine, the *world* was refused.

### Fixed

- ⛔ **A total physics failure was a WARNING, so `run-headless` printed PASS and exited 0.** A
  loop-closing `SolidReference` or a `Cone` `boundingObject` makes `SolverMuJoCo` construction raise,
  and the world then gets **no Newton world at all** — every body frozen at its authored pose. Because
  `reportPyError` logged at warning level, the run reported `0 errors, 1049 warnings … PASS`. The
  finalize path now reports fatally: same world, `1 errors, 1 warnings … FAIL`, exit 1. The ~60 other
  `reportPyError` call sites are untouched — each is one feature declining, not a void world — and the
  report is latched so a per-tick retry cannot flood the log. Red-capability was **proven, not
  assumed**: the engine was rebuilt with the old call, 3 of 4 cases went red and the control stayed
  green, then restored. ⚠️ This immediately exposed a shipped sample
  (`projects/samples/devices/worlds/coupled_motors.omniworld`) that has had zero physics since ODE was
  deleted.
- **The `coupled_motors` device sample had no physics at all, and is now a working sample.** Each
  finger was a four-bar parallelogram whose passive rocker closed a loop back onto the fingertip, so
  `SolverMuJoCo` raised `Body 8 has multiple parents in this articulation` and the **entire world** —
  not just the gripper — got no Newton world. It shipped that way since the ODE deletion; the ERROR
  change above did not break it, it stopped it lying. Fingers are open chains now (jaws pivot rather
  than staying parallel — the trade is recorded in both a header comment and `WorldInfo.info`, so it
  survives an engine re-save). Two further defects surfaced only once the world could step, each of
  which would have left a green PASS teaching nothing: **no motor declared `minPosition`/`maxPosition`**,
  so all three were configured as velocity wheels and every `setPosition()` the controller issued was
  ignored; and **`newtonCompoundColliders` defaults FALSE**, so the jaw pad — the second child of a
  `Group` `boundingObject` — was silently never registered and the jaw swept through the block while
  still reporting a full 0.42 rad travel. Now one `setPosition` on the left motor drives both at
  +0.4200 / −0.4200 rad and the block is picked up and set down every cycle.
- **`Accelerometer`, `Gyro` and `GPS` published uninitialised stack memory** as sensor values.
- **Compound colliders dropped cylinder length**; **collider rotations were dropped**; **capsules
  substituted for cylinders lost their length**; **prismatic joints reached the solver with no travel
  limits**; and **`getContactPoints` published body-local points as world coordinates**.
- **49 worlds silently loaded with no robot at all**; 33 Windows demo launchers pointed at worlds that
  no longer exist; 37 policy launchers computed the wrong repo root and none found its world.
- **OmniBench lane 4 scored cloth `absent` on a mis-authored probe.** The probe declared `size 0.5 0.5`
  — never a `Cloth` field — and its `absent_markers` matched the engine's *field* complaint, scoring it
  as a *missing node*. It survived a doc audit because the recorded doc claim ("no cloth solver is
  compiled in") agreed with the wrong measurement: **a probe and a doc can be wrong together**, so a
  `doc_mismatch_count: 0` is not proof of currency. Now `degraded` on 441 registered particles —
  `degraded` rather than `works` because particle state has no supervisor accessor, and a registration
  line is an engine self-report, not physics.
- **`python -m omnisim policy` forwarded fifteen skill-library verbs it never advertised**, so
  `--help` listed six and anyone who checked first concluded the documented commands did not exist.
  Both front doors now build their parser from one shared verb table, pinned by a test.

### Changed

- **[docs/guide/friction-grasp.md](docs/guide/friction-grasp.md) re-derived against measurement.**
  Two of its five recipe fields were contradicted by two independent benchmark arms, and its
  controller section contradicted its own root-cause paragraph 50 lines above it (it told the reader
  to hold a grip with `setForce`, which does **not** put a Newton joint in force mode). The page now
  separates the fields that decide whether a grip holds from the ones that do not, states the scale
  each measurement was taken at, and gives the position-interference recipe with its algebra.

### Removed

- ⛔ **BREAKING (ABI) — the C++ controller API has left `namespace webots`. It is `namespace omnisim`,
  and EVERY C++ CONTROLLER MUST BE RECOMPILED.** This is the final step of the naming migration and
  the only one that is not source-only. `using namespace webots;`, `webots::Robot`, and every
  other `webots::`-qualified name stop compiling; the canonical spellings are `using namespace omnisim;`
  and `omnisim::Robot`.
  - **Why this one breaks the ABI when the others did not.** A C++ namespace is part of the mangled
    name of every symbol declared inside it, so renaming it changes the identity of the whole exported
    C++ surface — nothing is added or removed, but nothing matches either. Measured on the pre-rename
    libraries in this tree: **1009 distinct mangled names carrying `6webots` in
    `lib/controller/CppController.lib`** (2018 archive symbol-table entries once the `__imp_` import
    thunks are counted — that is the origin of the "~2018 exports" figure), plus **44 in
    `CppDriver.lib`** and **19 in `CppCar.lib`**.
  - **What you must do: rebuild, not just re-edit.** Rewrite `webots` to `omnisim` in the `using`
    directive and in any `webots::`-qualified name, then **recompile and relink**. Editing the source
    alone is not enough and skipping the rebuild is not a survivable shortcut: an already-built
    controller imports `_ZN6webots…` from a library that now exports `_ZN7omnisim…`, so it fails at
    **load** with a missing-entry-point / undefined-symbol error. It does not run subtly wrong — it
    does not start. The shipped [`resources/templates/controllers/template.cpp`](resources/templates/controllers/template.cpp)
    now writes `using namespace omnisim;` and says why in a comment.
  - ⛔ **There is deliberately NO compatibility alias.** `namespace webots = omnisim;` was considered
    and refused: it would put the literal string "webots" straight back into a shipped public header,
    which is the entire reason this change exists. The break is clean and announced rather than
    softened, and no future release will reintroduce the alias.
  - ✅ **What does NOT break: the C API, and it is untouched on purpose.** The C ABI is `wb_*`-prefixed
    and **not one of its 497 exported functions contains the string "webots"**, so nothing there
    blocked the goal and nothing there was renamed. `wb_*` functions, the `Wb*` types (`WbDeviceTag`,
    `WbNodeRef`, `WbFieldRef`, …) and the `wbu_*` utilities all keep their names and their symbols.
    **C controllers keep linking and do not need recompiling** — only C++ ones do. Renaming the C
    surface as well would have doubled the blast radius for zero progress on the string, so it was
    ruled out rather than deferred.
  - **Documentation moved with it.** All 32 C++ device pages under `docs/reference/` and the three
    C++ pages under `docs/guide/` (`controller-programming`, `cpp-python`, `supervisor-programming`)
    now show `namespace omnisim` — 162 occurrences. Five stale C++/C include lines the previous
    sweep's pattern could not see (`#include "<webots/Emitter.hpp>"` ×4, `#include "<webots/Gyro.hpp>"`,
    and `"webots/supervisor.h"` in the field-type prose) were repaired in the same pass; they had
    documented a header path deleted in the entry below.

- ⚠️ **BREAKING (source only) — the legacy `webots/` controller-include path and the `controller`
  Python module are gone.** Deleted: all 91 one-line forwarder headers under
  `include/controller/c/webots/` and `include/controller/cpp/webots/`, the 75-line re-export shim at
  `lib/controller/python/controller/__init__.py`, and the generator that produced the headers
  (`scripts/dev/make_omnisim_header_forwarders.py`). `#include <webots/robot.h>` and
  `from controller import Robot` no longer resolve; the canonical `<omnisim/robot.h>` and
  `from omnisim import Robot` are now the only spellings.
  - **This deletion, on its own, is NOT an ABI break** — ⚠️ but do not quote that as a statement about
    the release: the namespace entry above *is* one, and a C++ controller must be rebuilt regardless.
    For this bullet alone: no exported symbol moved. The C API is `wb_*`-prefixed and not one of its
    497 exported functions contains the string "webots"; the `Wb*` types and `wbu_*` utilities are
    untouched. A controller already compiled against the old *include spelling* keeps linking — only
    *recompiling* one against it fails, and the fix is a one-line rewrite of the include or the
    import.
  - ⚠️ **Retained at the time, and no longer:** this entry originally recorded the C++
    `namespace webots` as deliberately kept, because renaming it *would* be a real ABI break. It was
    then renamed anyway, on purpose and with the break accepted — see the namespace entry above.
    Cyberbotics copyright headers, `NOTICE`, `THIRD_PARTY_NOTICES.md` and `TRADEMARKS.md` **are**
    untouched — Apache-2.0 §4 attribution is independent of this migration and was never in scope.
  - **Also fixed here, a latent release-time defect this deletion exposed:**
    `scripts/packaging/files_core.txt` enumerated *only* the `webots/` forwarders and the
    `controller/` Python shim, and never listed the canonical `include/controller/{c,cpp}/omnisim/`
    headers or `lib/controller/python/omnisim/`. Since the packager globs (a missing path expands to
    nothing rather than erroring), a release would have silently shipped forwarders pointing at
    headers that were never packaged, and a Python shim whose `from omnisim import ...` could not
    resolve. All three file lists (Windows/Linux and the macOS `Contents/` variants) now point at the
    canonical paths.

- ⚠️ **BREAKING (environment) — the `WEBOTS_*` runtime environment variables are no longer honoured.**
  `OMNISIM_*` is now both the only spelling this codebase writes and the only one its runtime reads,
  on both sides of the engine↔controller contract at once. `OMNISIM_CONTROLLER_URL`,
  `OMNISIM_ROBOT_NAME`, `OMNISIM_INSTANCE_PATH`, `OMNISIM_HOME`, `OMNISIM_LIBRARY_PATH`,
  `OMNISIM_STDOUT_REDIRECT` / `OMNISIM_STDERR_REDIRECT`, `OMNISIM_LOG_PATH` and `OMNISIM_IPC_NONCE`
  are the names; their `WEBOTS_*` twins are read no more. The engine additionally `remove()`s the
  legacy twins from every child environment, so a stale shell value cannot shadow a canonical one.
  - **What you must do:** rename the variable in whatever sets it — a shell profile, a CI job, an IDE
    run configuration, a launcher script. `WEBOTS_CONTROLLER_URL` (extern controllers) and
    `WEBOTS_HOME` are the two most likely to be set by hand.
  - **It is warned about, not silently dropped, and that is the design — but the two warnings behave
    differently, so know which one you are looking at.** Where a legacy name is set and its canonical
    twin is not, one message naming the new spelling is emitted, then either:
    - **warn and continue as if unset** — `WEBOTS_CONTROLLER_URL` and `WEBOTS_ROBOT_NAME`
      (`robot.c getenv_omnisim()`), `WEBOTS_INSTANCE_PATH` (`system.c`), and `WEBOTS_LIBRARY_PATH`
      (an engine `warn()`; its paths are **not** added to the controller's library search path); or
    - **warn and FAIL** — `WEBOTS_HOME`, because without an install root there is nothing to proceed
      with. The launcher's `get_omnisim_home()` returns false and refuses to start, and the Python
      binding's `wb.py _omnisim_home()` raises `KeyError` with the same message. Do not expect a
      degraded-but-running session here.

    Silence was the real hazard the warnings exist to remove: without the message, a user with only
    `WEBOTS_CONTROLLER_URL` set falls into extern-controller discovery, waits ~50 s and is then told
    their *world file* is wrong.
  - ✅ **`WEBOTS_HOME` still works at BUILD time, and that is not an oversight.** The top-level
    `Makefile` exports it as an alias of `OMNISIM_HOME`, and 16 more Makefiles expand
    `$(WEBOTS_HOME_PATH)` — a cross-Makefile contract variable, not a runtime name. Three engine-side
    reads outside the controller contract also survive untouched: `WEBOTS_EMPTY_PROJECT_PATH`,
    `WEBOTS_TMPDIR` and `WEBOTS_DEBUG`. None of them reaches a controller.
  - ✅ **The on-disk rendezvous names did NOT change** — the `webots-<id>` tmp folder and the
    `\\.\pipe\webots-…` name are a wire contract that the engine and libController each reconstruct
    independently. Renaming one side alone is exactly the documented failure where every controller
    hangs at zero ticks while a headless run still prints `PASS`.

- ⚠️ **BREAKING (world files) — the `Radio` and `Microphone` nodes are deleted.** Both node classes,
  the radio plugin and both `.wrl` schemas are gone (8 files, 817 LOC). A world still declaring
  `Radio {}` or `Microphone {}` now logs `Missing declaration for '<node>', unknown node.` plus
  `Skipped unknown '<node>' node or PROTO.` — **2 errors and a headless exit code of 1** — with that
  node skipped and the rest of the world still parsing. Same failure class, and the same deliberate
  choice, as `Solid.immersionProperties` in v6.0.0. Zero in-tree worlds are affected.
  - **Measured inert before deletion, not assumed from usage.** `wb_microphone_get_sample_data()`
    returned NULL **300 times out of 300** over 2.4 s of sim time, while `get_sample_size()` reported
    a constant 4522061 out of an uninitialised `malloc`. `Radio` loads and can neither transmit nor
    receive on any build of this tree. Both nodes parse, enumerate and survive finalize, which is
    precisely why a bare `run-headless` PASS never noticed either of them.
  - ✅ **`Skin` was proposed for the same retirement and is KEPT — the proposal was wrong about it.**
    It works on Newton in **both** documented modes, verified across five headless runs each with a
    differential control. `OmSkin.cpp` contains no reference to ODE, `dBody`, `dGeom` or physics of
    any kind: it is mesh deformation in the render path, so the ODE deletion never touched it. It is
    also 1,057 of the 1,600 LOC the three nodes were costed at, and it has a reference page, guide
    imagery and test assets. Deleting a working documented feature on "our corpus does not use it"
    was the error the measurement caught.
  - ✅ **The controller ABI is untouched and still exporting.** `Controller.dll` still carries 27
    `wb_radio_*`, 6 `wb_microphone_*` and 7 `wb_skin_*` symbols, and the positional device
    enumerators survive unrenumbered at their original ordinals (`SKIN` 87, `MICROPHONE` 117,
    `RADIO` 118) — they are positional public ABI and must never be removed or reordered. With the
    node type gone, `wb_robot_get_device()` simply returns tag 0 and every `wb_radio_*` /
    `wb_microphone_*` call takes its ordinary "invalid device tag" path, which controllers already
    handle. `include/plugins/radio.h` was also kept deliberately: `src/controller/c/radio.c:99`
    declares `struct WebotsRadioEvent` **by value** and the complete type exists only in that header.

### Known limitations — read before quoting any result

The Newton integration is complete for **rigid-body simulation with declared-geometry contact**, and
shipping for cloth and soft bodies on the VBD path. It is **not** complete. By the tree's own executed
measurement (OmniBench lane 4, 45 probes, one machine), **78% of implemented capabilities work**:
31 works / 5 degraded / 4 broken / 4 absent / 1 no-result. The generated matrix at
[docs/benchmarks/lane4-capability-matrix.md](docs/benchmarks/lane4-capability-matrix.md) is the live
source, and where it disagrees with prose, **the matrix is the measurement**.

✅ **75% → 78% IS a genuine ENGINE improvement — and it is exactly ONE probe, not two.** `2094660ef`
flipped `OMNISIM_NEWTON_BALL_HINGE2` on, and `joint.hinge2_motor` went **`broken` → `works`**: a
motorised `Hinge2Joint` commanded to 0.8 rad in a gravity-free world now tracks its target
**exactly** (`joint_angle_travel_rad` = 0.8000) and carries the arm **0.1951 m**, where it previously
moved 4.00e-04 m. Unlike the friction move below, the capability genuinely did not exist before and
does now: the blocker was newton 1.2.0's d6 → MuJoCo actuator mapping, and the vendored upgrade to
**newton 1.5.0** (`b56be84a0`) fixed it. ⛔ **The commit claimed the same for `BallJoint` and that half
is REFUTED.** `joint.ball_motor` was re-measured on the same binary, same rig, gate on, and moved
**2.67e-07 m** — 267 nanometres. It stays `broken`. The claim survived review because the commit's
evidence, `tests/test_newton_ball_hinge2.py`, has a motorised *hinge2* arm but a **passive** ball arm
(`PositionSensor` only), so nothing in the tree drives a motorised BallJoint. AGENTS.md is corrected
in this release. ⚠️ The ball's **angle readback travels 5.805 rad while the body does not move**, so a
controller polling `getPosition()` is told it is actuating.

⚠️ **Both of those probes were ALSO repaired, and the repair is what made the engine change
visible** — worth separating from the verdict move above. Both authored their motors with no
`minPosition`/`maxPosition` and their joints with no `minStop`/`maxStop`, which
`OmBasicJoint::newtonAxisSpec` builds as a **VELOCITY WHEEL** with `ke = 0`, where `setPosition()` is
ignored *by design* and the engine warns per device. So both probes were measuring the still-open
limit-less-servo defect, never the gate they claimed to test. The hinge2 probe carried a second bug:
it declared `HingeJointParameters` in `jointParameters2`, a field whose declared type is
`JointParameters`, so the engine refused the node, `axis2` fell back to its default `(0,0,1)` —
equal to the authored axis 1 — and reported `Hinge axes are aligned: using x and z axes instead`. No
assertion or threshold changed; only the scenes, so that they can express the capability under test.

⚠️ **72% → 75% is a PROBE REPAIR, NOT AN ENGINE IMPROVEMENT — do not read it as progress.**
`phenomenon.friction_declared_in_world` published `broken` from 2026-08-13 to 2026-08-17 and the
probe was wrong: it dropped a **0.2 m cube** on a **55°** incline, and a block holds only if *both*
µ ≥ tan θ **and** tan θ < b/h. A cube has b/h = 1.0 against tan 55° = 1.428, so it toppled at any
friction and the scene was statically impossible before friction was ever consulted. Re-run in bare
MuJoCo the same cube travels 23.30 m at µ=2, 23.46 m at µ=10 and **22.29 m at µ=100** — a verdict that
does not move when the variable under test is raised fiftyfold was never measuring that variable.
Rebuilt with a low-CoM slab (b/h = 10, topple angle 84.3°) the capability measures **`works`**, and a
new **negative arm** — `phenomenon.friction_slides_below_coulomb_bound`, the same slab at µ=1.3, below
the bound — slid **1.4402 m against the analytic 1.442 m (ratio 0.999)**. Declared friction reproduces
the Coulomb bound to three significant figures. The `broken` claim was retracted from this file and the
README in `1fc3b4b5a`; this release retires the probe that produced it and adds the failing arm that
would have caught it, because a green that cannot go red is not evidence.

⚠️ **Those rows are not all from one engine binary, and the matrix says so — it now names FOUR.**
**39** rows come from `f3f003bf0304bc5e`, 2 — the cloth and soft-body probes — from
`89e978269dc27a23`, **2 — the two friction probes above — from `6aac9ae1b461f567`**, and **2 — the two
joint probes above, re-measured after the `BALL_HINGE2` flip — from `13906cc6f12451eb`**. Two things
follow. The
`89e97826` binary was built from a tree carrying uncommitted engine changes, so those two rows are
**not reproducible from any commit**. And cloth forces MuJoCo onto the **GPU**, so they ran
`mujoco_warp` where the other 43 ran CPU `mj_step` — which matters, because bitwise determinism is
refuted on the GPU path. The campaign is **topped up, not re-swept**, and the summary row says so in
its `deviations`. Read a row's own `machine` block in `results/coverage.jsonl` before quoting it.

What is measurably broken, and would otherwise be found the hard way:

- **A world can declare restitution and get none.** `bounce 0.8` predicts a 0.64 m rebound and
  measures **−0.4 mm**. This is **not a plumbing bug and is not fixable**: MuJoCo has no coefficient
  of restitution — the string appears nowhere in mujoco 3.11.0 — and models contact as a
  `solref`/`solimp` spring-damper. Our defaults map to `solref (0.02, 1.0)`, exactly MuJoCo stock and
  critically damped, which reproduces the −0.4 mm measurement in *bare* MuJoCo. Only e≈0 (today) and
  e≈1 (zero damping) are stable; intermediate values create energy — one measured configuration
  rebounds **661 m** from a 1 m drop. `ContactProperties.bounce` should therefore be treated as
  **declared-but-never-read**. ✅ **The engine now says so**: a world that *authors* a `bounce` (as
  opposed to inheriting the `.wrl` default of 0.5, which all 322 ContactProperties worlds do) gets a
  one-shot warning naming the field and stating that there is no `newton*` field to migrate it to.
  Fixed in the same change: the neighbouring `coulombFriction` warning still advised pinning
  `physicsBackend "ode"`, which since `bdc02139` means the Solid is registered with **no solver at
  all** — no gravity, no contact — so the engine's remedy for "my friction is ignored" was "have no
  physics". A source scan now pins that the advice cannot come back.
- ⚠️ **Retracted from an earlier draft of these notes: "declared friction is unreliable above ~45°."**
  That claim came from lane 4's `phenomenon.friction_declared_in_world` probe, and **the probe is
  invalid**. It drops a **0.2 m cube** on a **55°** incline. A block is in equilibrium only if both
  µ ≥ tan θ *and* tan θ < b/h; a cube has b/h = 1.0 and tan 55° = 1.428, so it **topples at any
  friction whatsoever**. The scene is statically impossible before friction is consulted. Checked in
  bare MuJoCo: a low-CoM slab on the same 55° incline **holds** at µ = 1.4281 (= tan 55° to four
  decimals) and slides at 1.3 — i.e. declared friction reproduces the analytic Coulomb bound to three
  significant figures. ✅ **RE-MEASURED IN-ENGINE 2026-08-17 AND THE MATRIX NO LONGER CARRIES THE
  `broken` VERDICT** — this bullet used to end "the probe's `broken` verdict stands in the shipped
  matrix until it is re-measured in-engine", and that is now done. The probe was rebuilt on a low-CoM
  slab (0.6 × 0.3 × 0.06, b/h = 10, topple angle 84.3° — so the slope can excite sliding and nothing
  else), the 55° incline kept, and it measures **`works`**: slide **0.0008 m** at the declared µ=2.0.
  A **negative arm** was added in the same change — `phenomenon.friction_slides_below_coulomb_bound`,
  the identical slab at µ=1.3, *below* the bound and *above* the engine default of 1.0 — which slid
  **1.4402 m against the analytic 1.442 m, ratio 0.999**. That arm exists because a green which cannot
  go red is not evidence: it fails if the slab holds (friction silently delivered above what the world
  declared) and it fails at the µ=1.0 rate (the declaration ignored), so it rules out both ways the
  positive arm could pass for the wrong reason. The sharpest record of how wrong the old probe was:
  in bare MuJoCo the cube still travels **22.29 m at µ=100**. *(A separate, real friction issue does
  exist — tangential creep under sustained load in a pinch grasp, ~56 mm over a 1.5 s lift, which is
  what `WorldInfo.newtonNoslipIterations` addresses. It is unrelated to inclines.)*
- **`setPosition()` is silently ignored on a motor with no declared limits.** A limit-less
  `RotationalMotor` is deliberately configured as a velocity wheel (`ke=0`). Declare
  `minPosition`/`maxPosition` and it gets `ke = effortLimit * 10`. This is the single most common
  robotics primitive and it fails quietly; our own benchmark author hit it and published a `broken`
  verdict against the engine. ✅ **The warning is now PER DEVICE, not once per process.** Both guards
  were `static bool`, so on any world with a wheeled robot in it the first wheel — a *correctly*
  classified velocity motor — consumed the warning and every genuinely affected servo after it was
  degraded in silence. ⚠️ **The classification itself is UNCHANGED and this is deliberate.** The full
  fix is to promote `ke` when `OmMotor::isPIDPositionControl()` first goes true; the signal exists and
  `pushNewtonMotorTargets` already branches on it every tick. What stops it is blast radius: **1680**
  limit-less motorised joints across **189** files currently get `ke=0` from this branch. The 49
  controllers using the `setPosition(inf)` idiom are safe by construction, but "safe for the idiom we
  grepped" is not "safe", and a wrong default change is worse than a loud warning.
- **A node deleted at runtime keeps colliding, for ever.** There is no remove path from a
  supervisor-deleted `Solid` to the MuJoCo model — `removeBody`/`removeShape`/`unregister` do not exist
  anywhere in the backend or the runtime. A deleted wall still stops a robot and a deleted floor still
  holds a body up. Reload the world after removing collidable nodes.
- **A compound `boundingObject` registers only its FIRST child by default.** A bin authored as a floor
  plus four walls collides as one box; a `Chair.proto` collides as a floating seat slab with no legs.
  **120** compound objects with ≥2 children across **91** files are in this state.
  `WorldInfo.newtonCompoundColliders TRUE` opts in. ✅ **It is no longer silent**: a multi-child `Group`
  boundingObject now warns once per node, naming the OWNING SOLID and how many shapes were dropped.
  ⚠️ **The default is deliberately NOT flipped** — the same flag also selects the inertia source for
  every dynamic multi-collider body, so flipping it would silently change the dynamics of far more than
  the colliders. Decoupling those two is separate, larger work. ✅ Also fixed: the env override was
  **presence-gated**, so `OMNISIM_NEWTON_COMPOUND_COLLIDERS=0` turned the feature **ON** — the same
  trap `OMNISIM_REQUIRE_NEWTON` carries, and inconsistent with every neighbouring knob. It is now
  value-parsed, and the collider and inertia branches read one shared helper so they cannot disagree.
- **Every triangle-mesh collider is silently convexified.** A bowl, cup, tube or room shell is a solid
  lump — measured, a ball rests at rim height, not on the cavity floor. This is MuJoCo's mesh path, not
  ours. Build concave colliders from several convex pieces.
- **Closed kinematic chains are unsupported.** `SolverMuJoCo` is a tree solver, so a loop-closing
  `SolidReference` builds no world at all. As of this release that FAILS loudly instead of passing
  silently. Newton ships `SolverKamino` for exactly this case; OmniSim does not drive it.
- ✅ **RESOLVED — motorised `BallJoint` and `Hinge2Joint` DO actuate, and `OMNISIM_NEWTON_BALL_HINGE2`
  now defaults ON.** This bullet used to say the flag "does **not** fix this". The blocker was real and
  correctly attributed below us, to newton's d6 → MuJoCo actuator mapping for multi-DoF position
  control — but that was measured against the then-vendored **newton 1.2.0**, whose own test suite
  never drove d6 POSITION targets through `SolverMuJoCo`. The `b56be84a0` upgrade to **newton 1.5.0**
  fixed it upstream: `tests/test_newton_ball_hinge2.py`'s hinge2 arm went `xfail` → **XPASS** on the
  current binary (both axes inside 0.05 rad of command, no cross-axis drift, full 0.6 rad travel), so
  the marker is gone and the default is flipped. `=0` remains the exact-revert hatch and the test
  asserts it still reverts. ⚠️ **Remaining caveat:** the ball element is emitted `limited: False`, so a
  `BallJointParameters`' per-axis `min/maxStop` are not enforced by the solver. Hinge2 axes are limited.
- **The GPU solver is a different product.** Choosing `newtonSolver "mujoco_warp"` costs welds,
  `TouchSensor` force, noslip, constraint readback, and the whole ray-sensor family. ✅ **Those ray and
  contact readbacks no longer FREEZE at the build-time pose — they decline.** `raycast_batch` and
  `get_contacts` read `solver.mj_data`, which newton only steps on its `use_mujoco_cpu` branch, and
  neither carried the guard `weld_engage`/`touch_force` already had, so on the GPU they answered
  confidently against the scene as authored at t=0. The decisive differential was run for this release
  (laser `DistanceSensor` on a box in free fall, read at t=0.512 s): CPU control **10.0** = no hit,
  GPU pre-fix **1.0000000447** = a hit at the authored t=0 distance, GPU guarded **10.0**. Rays now
  decline (consumers keep their previous verdict) and contacts fall back to newton's own live narrow
  phase — correct body pairs, support-point positions, depth 0 — both with a one-shot engine-side
  warning. Determinism is **refuted** there: 0 bitwise of 24 same-config cold pairs, 9.152 m apart by
  1000 steps. Bitwise reproducibility holds on the CPU `mj_step` path.
  ([scope](docs/benchmarks/determinism-scope.md))
- **`GranularGroup` particles cannot be read back, and nothing in a scene can push them.** The CUDA
  solver is real and benchmarked (100k particles at 4.50 ms/step) but its robot↔particle coupling is
  currently dead in both directions, and no demo world uses it.
- **No deformable has a readback surface.** No supervisor, controller or harness endpoint can describe
  a `Cloth`, `SoftBody` or `GranularGroup`; every verdict comes from out-of-band engine telemetry.
  `getFromDef(...).getPosition()` returns the authored pose for ever. This caps every deformable row at
  `degraded` regardless of how well the physics runs.
- **Cables/rods, MPM granular, free particles and authored-tet soft bodies exist upstream in Newton and
  have no OmniSim authoring path.** Newton ships 9 solvers; OmniSim drives 2.
- **No sim-to-real.** Zero physical-robot transfer. Every humanoid result runs on a weight-bearing
  balance harness; quadrupeds carry none. macOS is untested and, with no fallback solver, has no
  verified physics path.

## [v6.0.0] — 2026-08-10

> ⚠️ **READ THIS BEFORE ANY OLDER ENTRY IN THIS FILE.** ODE is gone. Every entry below dated
> before 2026-08-08 that describes ODE as a selectable backend, a supported CPU fallback, an
> accuracy reference (a framing that was itself wrong — see
> [docs/benchmarks/correctness-scope.md](docs/benchmarks/correctness-scope.md)), or a rollback
> target — and in particular every entry offering
> `OMNISIM_FORCE_ODE`, `OMNISIM_LEGACY`, `OMNISIM_ALLOW_ODE_FALLBACK` or `physicsBackend "ode"`
> as an escape hatch (v4.0.0 and v5.0.0 both do) — was true when written and is **superseded**.
> Those entries are kept as history, not as instructions. The measurements in them are still
> measurements; the advice in them is not advice any more.

### Removed

- **BREAKING — ODE is DELETED. Newton/MuJoCo is the only physics backend** (commit `bdc02139`,
  2026-08-08). `src/ode` and `include/ode` are gone — 106,283 lines. There is no second solver,
  no CPU fallback tier, and no build flag or environment variable that restores one.
  **What this changes for a user:**
  - **The Newton Python runtime (`newton` / `warp` / `mujoco`) is now a hard requirement**, not a
    capability. A binary that cannot import it through its embedded interpreter has *no physics at
    all* — nothing falls, nothing drives. A stock `make release` bundles it on Windows
    (`BUNDLE_NEWTON ?= 1`); on Linux, pip the wheels into the **system** `python3`.
    `python -m omnisim doctor` is the check.
  - **`Solid.physicsBackend` and `WorldInfo.defaultPhysicsBackend` still accept `"ode"`, but the
    value no longer selects a working solver.** The enum entries are retained because an
    *undeclared* field is a parse ERROR that takes a headless run's exit code to 1, and 21 tracked
    worlds still carry the value. Reading the source: `OmOdeBackend::isAvailable()` is now `false`
    and each of its operations returns `-1`, and `OmSolid::flushPendingNewtonRegistrations` skips
    any Solid whose effective backend resolves to `"ode"` — so such a Solid is registered with no
    solver and is **not simulated**. ✅ **The engine now warns once per such Solid** (any Solid that
    declares a `boundingObject` or a `Physics` node): *"This Solid asks for physicsBackend "ode",
    which no longer selects a physics engine … it is a visual-only body."* An **older log will not
    have that line**, so on a world you did not just run, a frozen body or a prop that never falls is
    still worth checking for `physicsBackend "ode"` first. Never write it into a new world.
  - **`OMNISIM_FORCE_ODE`, `OMNISIM_LEGACY` and `OMNISIM_ALLOW_ODE_FALLBACK` are retired — now
    warned about and ignored.** As of commit `2271e1f8`, `warnRetiredOdeSelectors()` logs one line
    per variable set and backend resolution has no force-ODE branch at all, so the run is Newton
    either way. Unset them regardless: a warning in the log is noise you do not need.
    ⚠️ **Historical, and the reason they were neutralised rather than left alone:** before that
    commit they genuinely did short-circuit resolution *and* make the Newton registration flush
    return early, which was strictly worse than not setting them. Measured 2026-08-08 on
    `tests/physics/worlds/contact_points.omniworld`: under `OMNISIM_FORCE_ODE=1` the run built no Newton
    world, wrote no verdict sidecar, and left the scene **frozen at its authored pose for the whole
    run**. That is no longer reachable.
  - **`OMNISIM_REQUIRE_NEWTON` is NOT retired — it is the one selector that still does something,
    and you should set it for any run whose result you intend to trust.** Newton is required
    unconditionally, but a clone whose runtime is merely *absent* only gets one logged ERROR and a
    motionless scene — which a batch job can miss entirely. This variable turns that into a FATAL
    and a non-zero exit ([`OmNewtonBackend.cpp`](src/omnisim/physics/OmNewtonBackend.cpp), whose own
    comment reads *"Still useful post-ODE"*). Set it for benchmarks, CI lanes and published
    measurements, and assert the `.newton.json` verdict sidecar alongside it.
  - **`OMNISIM_WITH_ODE=ON` is refused with an error.** `OMNISIM_WITH_NEWTON=OFF` still builds but
    no longer produces "a pure-ODE binary" — it produces a binary with no physics implementation.
  - **Nodes removed with the backend:** `Fluid` and `ImmersionProperties`, plus the
    `Solid.immersionProperties` field. Buoyancy, Archimedes' thrust and fluid drag are **not
    simulated**, and there is no replacement node. ⚠️ Unlike the retired-but-parsed fields below,
    `immersionProperties` was removed from the schema outright, so a legacy world declaring it gets
    a `Skipped unknown field` **ERROR** and a non-zero headless exit. The
    `projects/samples/geometries/worlds/floating_geometries.wbt` sample is gone with it.
    `WB_NODE_FLUID` / `WB_NODE_IMMERSION_PROPERTIES` remain in `omnisim/nodes.h` for ABI stability
    but can never be returned.
  - **Fields that still parse (so legacy worlds load) but are no longer read:** `WorldInfo.CFM`,
    `ERP`, `physics`, `optimalThreadCount`, `physicsDisableTime`, `physicsDisableLinearThreshold`,
    `physicsDisableAngularThreshold`, `broadphase`, `contactProperties` (the whole
    `ContactProperties` node, including `coulombFriction`, `bounce`, `softCFM`, `softERP`,
    `maxContactJoints` and the three contact-sound URLs), `defaultDamping`, and `Physics.damping` /
    the `Damping` node. `OmSolidMerger::setOdeDamping()` and `setOdeAutoDisable()` are now empty
    functions. **Use `WorldInfo.newtonGroundMu` / `newtonContactKe` / `newtonContactKd`** for
    contact behaviour. There is no replacement for body sleep (Newton has none), for damping, for
    restitution (`bounce`), for `rollingFriction`, or for contact sound.
  - **Known broken rather than removed, and each fails silently** — do not report these as working:
    motorised **`BallJoint` / `Hinge2Joint` do not actuate** (`OMNISIM_NEWTON_BALL_HINGE2` is
    default OFF because the motorised path does not work either; motors are accepted and ignored
    and position sensors read a frozen 0 — measured: axis 1 reads 0.0000 rad when commanded 0.4);
    **force-type `TouchSensor` reads 0 N** (bumper type works); **contact sound** produces nothing;
    the **contact-points GUI overlay** produces nothing.
  - **No second IN-ENGINE path to cross-check the plumbing.** Note the narrower wording: lane 1's
    oracle is **analytic ground truth** and it is unaffected, and bare `mujoco` and `pybullet` still
    run as independent arms. Through the 2026-07-24 campaign `omnisim-ode` was best or tied-best on
    6 of 7 analytic lane-1 scenes at dt = 4 ms — ⚠️ **a comparison of two integrations, not two
    solvers.** Bare MuJoCo scored fine on those same scenes; the four defects that made our Newton
    integration lose (friction cone, rolling inertia, momentum leak, spin loss) were ours, in the
    layer between the scene graph and the solver, and all four were fixed in `e7b9fb11`. ODE's
    values are **frozen** in `tests/goldens/ode_oracle_goldens.json` as a fixed regression datum.
    What is genuinely gone is the ability to run one `.wbt` two ways and attribute a discrepancy to
    plumbing vs solver — the instrument that found gravity was never plumbed. What should replace
    it is an **open question**: [docs/benchmarks/correctness-scope.md](docs/benchmarks/correctness-scope.md).
    Likewise, re-baselining `tests/physics/worlds/` against Newton is **outstanding work**.
  - **The one comparative number worth carrying forward, as history:** measured same-harness on
    2026-08-06 while ODE still shipped, ODE was *faster* per step at every scene size measured
    (Newton/ODE 40.7× at 1 body → ~1.6–2.2× at 50–200), converging but never crossing. The old
    "Newton is 17–33× faster than ODE" headline was a cross-harness artifact and was already
    retired. Newton's case is **batching**, not per-step cost. Newton's per-step cost is now the
    engine's floor; no post-deletion step-cost campaign has been run.

- **BREAKING — the physics-plugin C ABI is deleted** (ODE-retirement manifest 1). A world's
  `WorldInfo.physics` field used to name a user-compiled shared library implementing
  `webots_physics_init` / `webots_physics_collide` / `webots_physics_step` and calling back
  into the engine through the `dWebots*` host functions. That vocabulary **is** ODE's
  (`include/plugins/physics.h` was a wrapper over `<ode/ode.h>`), so the feature could not be
  carried to the Newton backend and dies with ODE rather than being ported. Deleted: the
  public header `include/plugins/physics.h`; the engine class `WbPhysicsPlugin` and the
  "New Physics Plugin" wizard; `resources/projects/plugins/physics/` (the `*Proc` glue object
  every plugin linked against) and `resources/templates/plugins/physics/`; the channel-0
  Emitter→plugin bridge in `OmReceiver`; and the four physics-plugin regression worlds with
  their plugin and controller sources.
  **Compatibility:** `WorldInfo.physics` stays **declared** in `WorldInfo.wrl` on purpose — an
  *undeclared* WorldInfo field logs an ERROR that takes a headless run's exit code to 1, so
  removing it would make every legacy world read as a crash. The field is parsed and
  **ignored**; a world setting it to anything other than `"<none>"` gets one parse-time
  warning naming the removal and the plugin it asked for. An out-of-tree plugin's own `make`
  now fails immediately with `physics plugins were removed from OmniSim` instead of a
  missing-header cascade. Port the behaviour to a `Supervisor` controller
  (`wb_supervisor_node_add_force` / `add_torque`).

### Changed

- **Newton became the default solver's only shape, and four device paths went native + DEFAULT ON**
  — the work that made deleting ODE survivable. `newtonSolver ""` / `"auto"` now resolves to
  **`SolverMuJoCo`** (`7b431e81`, 2026-08-07); **XPBD was removed the following commit**
  (`94f04222`) — zero of 725 tracked worlds ever selected it, newton's own docs say it does not
  operate on articulations, it measured slower than CPU mujoco at every scale tried, and it drove
  the shipped 10-Husky swarm 0.97 m where mujoco (and, then, ODE) agreed on 2.4 m. A world still
  declaring `"xpbd"` gets the parser's invalid-value warning and the default. The field now selects
  **CPU vs GPU**: `""` / `"auto"` / `"mujoco"` = reference CPU `mj_step` (deterministic, no GPU
  needed); `"mujoco_warp"` = the same solver batched on the GPU, worth it **only** for parallel
  training (at nworld=1 it measured 9.06× slower than CPU) and **not** run-to-run reproducible.
  Four defaults flipped ON in `6eb35675` because with ODE gone there was no fallback left to
  degrade to: the native **raycast service** (`mj_ray`, answering `DistanceSensor` / `Receiver` /
  `LightSensor` / `Radar` / Camera recognition), **welds** (`Connector` / `VacuumGripper`),
  **kinematic (mocap) bodies** for physics-less joint endpoints, and **BUMPER-type `TouchSensor`**
  contact reads. Also on by default since 2026-08-07: **static colliders** (`newtonStatics`) and
  **native contact readback** (`getContactPoints` / `/sim/contacts` / `/sim/grips`) — before those
  flips a static floor collided with nothing (a ball dropped onto a floor topped at z = 0.55 settled
  at **z = 0.0996** on an implicit z = 0 plane that was not in the file; with statics on it rests at
  **0.6496**, exactly box-top + radius) and every Newton world was contact-blind (measured 1008
  contacts on ODE vs **0** on Newton for the same scene). `OMNISIM_NEWTON_STATICS=0` /
  `OMNISIM_NEWTON_NATIVE_CONTACTS=0` are exact-revert hatches and both parse their value.
- **Mass and inertia are computed natively** (`fe35be64`), replacing ODE's `dMass` integrator. The
  inertia tensors produced for primitive `boundingObject`s are **bitwise identical** to the ODE
  values they replaced, which are frozen in `tests/goldens/ode_oracle_goldens.json`.

- **Agent-edge validation programme revision 3** (`docs/developer/agent-edge-validation-plan.md`,
  2026-08-01): the headline Phase W campaign becomes a **product-level comparison driven by
  Claude Code** — pinned CLI version + model id recorded in every row, headless `claude -p`,
  one fresh session per cell in a staged clean workspace that **excludes the benchmark's own
  answer key** (plan §2.7; staging manifest published with the campaign; a contaminated cell
  is INVALID). One condition per simulator, `claude_code`: the product as shipped — OmniSim
  with its `AGENTS.md` counted deliberately as product surface; upstream Webots with its own
  install + docs, the no-`AGENTS.md` asymmetry stated, not hidden. The **API-runner lane
  (revision 2's conditions, Webots bridge, oracle guards) is retained** out of the headline
  path as the mechanism-isolation follow-up — F-surface / conjunct (i) is decided there; the
  product lane evaluates F-comparative only, best-vs-best degenerating to the single
  `claude_code` condition per simulator. Lanes, tasks, graders, n, and the F arithmetic are
  unchanged; the unfreezable-instrument trade (Claude Code cannot be byte-hashed; replication
  means "at this version") is recorded as new threat §5.11. The pre-registration freeze is
  re-executed as **freeze v2** (`tests/benchmarks/agentbench/preregister/`): supersedes v1
  with v1's manifest hash recorded for the audit trail. **Zero scored runs existed at
  amendment time** — a legal pre-run amendment, executed as a version bump rather than a
  silent edit.
- **The agent-edge validation programme is redesigned (revision 2 of
  `docs/developer/agent-edge-validation-plan.md`)** — before any pre-registration freeze, so
  the change is legal exactly once. What the v5.5.0 entry below quotes is revision 1's design
  and is superseded: E1 is restated as a **cost-to-outcome** claim (completion is a gate, not
  the headline — a strong model saturates completion, which is how the first A/B read
  "identical"); the task set splits into three lanes (an authoring **control** lane where a
  tie is the expected result, a closed-loop **decision** lane where the withdrawal rule F is
  evaluated, and a **capability-frontier** lane reported as a capability table with its
  numerics quarantined to that table, never aggregated into a throughput score); F is
  re-specified **per conjunct at the SPEC's own repeat floors** — n = 5, and n = 10 for A1
  (revision 1 cited an "n ≥ 3 floor (SPEC §3.5)" the SPEC does not contain) — replacing
  the 6 tasks × 2 conditions × 3 repeats / 5-of-12-pairs design whose denominator included
  structurally degenerate pairs: upstream Webots has no packaged tool surface, so its
  `shell+tools` condition was undefined and half the pairs could never show the effect,
  stacking the rule toward withdrawal regardless of the truth. The Webots `shell+tools`
  condition is now defined: a bridge wrapping upstream's **entire** published Supervisor +
  Robot function reference (exclusions countersigned), published for the 30-day correction
  window, guarded by three pre-freeze oracle checks — granularity (the bridge may not be
  chattier than the shell on any decision task), distinctness (judged by the non-OmniSim
  reviewer before any scored run), and a stated consequence that never deletes a comparison.
  The rule itself was hardened in-revision after an adversarial design review: survival of a
  conjunct requires **both** evidence channels (completion at Δ ≥ +2, and cost at
  R ≤ 0.85 gated on non-negative completion); a single channel yields only a narrow finding
  with the adverse channel's table printed (two of the four pre-written narrow templates are
  negative); best-condition selection is mechanical with near-ties breaking toward the
  cheaper condition; denominators are pinned (cost ratios defined only at ≥ 3 passing runs
  per side); and the SPEC §6.1 death condition is defined on aggregate totals so it can
  actually fire. The V1–V5 validity table is refreshed against the tree (the standalone
  runner, condition-in-row, and the Webots grading adapter are built and tested since 07-26;
  no real-model row exists anywhere), the decision cost metric is frozen (`tool_calls`;
  tokens reported separately, never summed; `t_agent_s` leads time), and a standing rule is
  adopted: **no row, no result** — a number that exists only in a commit message is not
  quotable, which the Webots FAIL 5/10 grading in `3c995c9c`'s message currently violates
  and is therefore not a result. Scored-run count goes 216 → 420; citation repairs
  throughout (the four-negative packaging sentence lives in `simulator-comparison.md` §8,
  not `agent-native-api.md` §1.3; §9.2 now holds eight corrections, not six; "fairness
  floor" is SPEC §6.2.2; the MCP registry is 18 tools, 22 with the shell set).

### Fixed

- **Five OmniArm 6 grasp worlds declared a contact friction that could not reach the solver.** Each
  declared `ContactProperties.coulombFriction 5` — an ODE-path field Newton does not read — with no
  `newtonGroundMu`, so a bare load ran the pinch at the engine default **1.0**, a fifth of the
  declared value. Now declared where it is read, matching the five sibling worlds already repaired:
  `omniarm6_anypick`, `omniarm6_anypick_line`, `omniarm6_bin_picking`, `omniarm6_declutter_train`,
  `omniarm6_grasp_train` (the last two had no launcher at all, so they always ran at 1.0).
  `lane1/translation_audit.py --sweep` over the demo tree goes 5 findings → 2. ⚠️ The two that
  remain are deliberate, not defects: `omniarm6_physics_pick_place` is hand-tuned to
  `newtonGroundMu 3`, and `granular_sand_demo` declares per-material frictions that a single global
  value structurally cannot represent. ⚠️ Also unreconciled by design: the three flagship pick
  worlds' PowerShell launchers export `OMNISIM_NEWTON_GROUND_MU=1.5`, which still wins over the
  field, so a launcher run and a bare run are self-consistent but not identical to each other.
- **`WorldInfo.coordinateSystem` finally reaches the solver** (`c77cbe98`) — and the blast radius
  was a third of the corpus. The Newton builder was constructed with a hardcoded z-up axis and never
  read the field, so every `NUE` / `EUN` (Y-up) world got two things wrong at once: `WorldInfo.gravity`
  is projected onto the builder's up vector, and the projection of a Y-up world's `(0, -g, 0)` onto
  `(0, 0, 1)` is **exactly zero** — so **nothing fell** (measured: a ball released at y = 3 read
  y = 3.000 at step 15360) — while the implicit ground plane took its normal from the same wrong axis
  and stood up as a **vertical wall** along the world's East axis. **210 of the 719 worlds in this
  tree are `NUE`** and none of them pinned a backend; ODE masked the defect by being the fall-back
  backend, and a headless PASS could not see it (all 140 `tests/api` worlds scored PASS because they
  loaded, stepped and logged nothing). The field is now plumbed to `newton.ModelBuilder.up_axis`
  *before* the implicit ground plane is created. `OMNISIM_NEWTON_COORD_SYSTEM` is value-parsed and
  default on; `=0` pins the historical z-up so the pre-fix physics can be bisected, and re-running a
  `NUE` world under it warns. Pinned by `tests/test_newton_coordinate_system.py`.

---

## [v5.5.1] — 2026-07-27

**Patch: the Linux build could not unpack its own dependencies.**

### Fixed

- **`make release` on Linux died before compiling anything.** Four `tar` calls in
  `dependencies/Makefile.linux` read `tar xfm --no-same-owner FILE -C DIR`. In the bundled `xfm`
  cluster the `f` takes the NEXT argument as the archive name, so tar tried to open a file
  literally called `--no-same-owner` and exited:
  `tar: --no-same-owner: Cannot open: No such file or directory`. That is every dependency the
  Linux build unpacks — Qt, OIS, assimp, openssl — so the dependency stage failed at
  `Makefile:195 webots_dependencies` without reaching a single engine source.
  `--no-same-owner` is correct and stays (it is what makes extraction work on an NFS
  root_squash volume); it simply has to precede the option cluster, so the calls are now
  `tar --no-same-owner -xmf FILE -C DIR`. Verified by reproducing the failure and the fix with
  real tar.
  **Why it went unnoticed:** `Makefile.linux` is gated behind `ifeq ($(OSTYPE),linux)`, so
  Windows never touches it, and the last green Linux build predates the change that introduced
  it. The v5.5.0 release triggered the GHCR training-image build — the first Linux build since —
  which surfaced a break that had already been there.

---

## [v5.5.0] — 2026-07-27

**The measure-your-own-claims release.** The first public release since v5.1.1 — the v5.2.0,
v5.3.0 and v5.4.x milestones were prepared but never published, and all of them fold in here.

The first half is capability. The harness became a mutation surface an agent can discover,
drive and reset; the robot bridges gained an action-result contract that reports what was
MEASURED rather than what was commanded; Shadowing went robot-general and BATON became a
reusable library; the skill library learned to certify what it ships; the engine learned to
run without a window, without a GL context, and — in the physics layer — without Qt at all;
and the OmniLink platform boundary moved into this repository, so key and BYOK setup are
terminal commands now.

The second half is what happened when we pointed those same instruments at our own claims,
and it did not go our way. The first A/B of our agent-facing surface against a bare POSIX
shell was won or tied by the shell. A "GPU physics is bitwise deterministic" grade turned out
to be measuring a stationary sun marker parked at z = 100000. A motion tool was found
delivering 57% of the angle it reported. `run-headless` was certifying a world whose crate had
left the planet. A benchmark assertion had been reading green for weeks while structurally
incapable of failing. Every one of those is below with its number, its machine and its unfixed
remainder — alongside a pre-registered rule that states, in advance, what result would make us
withdraw the claim in public.

### Highlights

- **A certified quadruped turn skill, designed and verified in one day.** `go2_turn` went ghost
  design → validator PASS → in-engine training → a 99.2 % never-fell certification (the two runs
  that missed the 99 % bar are on the record too — a gate that cannot fail is not a gate) →
  composed into the free-standing `go2_walk_turn_walk` BATON sequence: 3/3 headless runs, zero
  falls, ~169° turn-in-place, no harness of any kind. Champion checkpoint ships
  (`gpu_go2_turn_main`, `.pt` + self-contained `.onnx`).
- **Shadowing beats its residual-RL incumbent on the Go2** — +12.6 % speed and 5× straighter on
  the live deploy ruler (an incumbent-upgrade result; the from-scratch corridor-curriculum
  variant reaches 0.370 vs the incumbent's 0.381 m/s). The round-2 champion ships (0.415 m/s,
  99.7 % never-fell).
- **BATON is a library, demonstrated.** One arbiter sequences the craned G1 (walk → carry →
  corner → place, 3/3 zero falls at 91.8 s) and the free-standing Go2 — no world object, no
  torch, no crane, no humanoid anatomy in the arbiter.
- **In-engine training throughput up 5.9×** (132k → 784k env-steps/s at K=16384 on an RTX 4090)
  via slim contact buffers and a kinematics-only reset path, quality-gated at parity. New
  quality-gated reward instruments: peak-torque saturation hinge (`QUAD_W_SATHINGE`) and
  stance-foot slip (`QUAD_W_FOOTSLIP`).
- **A minimal runtime, measured.** A world can now run with no window constructed (−20 % RSS at
  full parity), with no GL context (compute-only "no-GL" mode: a K=4 fleet drops 1593 → 1041 MB,
  −35 %), and with Qt on the `minimal` platform so a headless host never opens a window-system
  connection.
- **The physics layer is Qt-free** — `physics/` reached zero Qt includes, held there by a
  per-directory include ratchet test.
- **OmniBench, a cross-simulator physics benchmark — and the five Newton defects it found.**
  Three lanes: correctness against analytic ground truth, throughput to the post-Genesis
  credibility checklist, and three novel axes (determinism grading, train==deploy structural
  parity, agent-driveability of the HTTP harness). Engines measured: MuJoCo 3.8.1, PyBullet,
  OmniSim/ODE, OmniSim/Newton, over two machines; of 180 comparable OmniSim metric cells, 124
  are numerically identical across Windows and Linux. The suite quotes **no published
  competitor numbers** — every figure is same-harness, and OmniSim's Newton backend embeds
  mujoco-warp, so accuracy deltas against MuJoCo are framed as integration fidelity, never a win.
- **A multi-robot agent demo that reports measured positions, not remembered ones.**
  HuskySwarm — four Clearpath Huskies, one coordinator, 45 tools — is the flagship OmniLink
  demo, with an optional per-Husky unit-agent team beneath it. Motion tools settle and stamp
  the robot's real final pose: without that, the agent narrated its remembered position right
  after physically driving elsewhere.
- **The scaffolding was worth more than the model choice.** On a 6-task hard tier, four
  agent-side changes moved the cheapest routing tier 4/6 → 5/6 and another engine 3/6 → 5/6
  with no model change; halving the prompt halved the cost and lost nothing. The losses are on
  the record too: one task is passed by no model, and a verifier that inaction could beat was
  found and fixed.
- **AgentBench — a sim-vs-sim agent benchmark, and its first A/B says our own surface did NOT
  win.** Same one-sentence task ("build a scene with 10 Huskies and let them move randomly"),
  same model, two tool conditions, artifacts graded blind: the bare `shell` condition — no
  harness, no HTTP surface — reached the same PASS 10/10 on ~36 tool calls against the full
  surface's ~50, and tied on the debug task at ~21 each. The claim that an agent gets more done
  here is therefore **currently unsupported by any measurement we hold**. What the run does not
  establish either way is recorded with it: n=1 per cell; the ablation leaked (both agents ran as
  coding-harness subagents, which auto-inject `AGENTS.md`, so a documentation-free condition was
  structurally unreachable); both conditions ran concurrently on one box sharing one binary and
  one trace directory, so **the wall-clock numbers must not be used at all**; and the graded rows
  carried the wrong condition tag, so the headline survives only as an operator's observation.
- **A pre-registered kill condition for that claim**, written before the data
  ([docs/developer/agent-edge-validation-plan.md](docs/developer/agent-edge-validation-plan.md)):
  6 tasks × 2 conditions × 3 repeats × ≥2 simulators, and the agent-throughput claim is
  **WITHDRAWN** unless a pass-delta of ≥ +1 appears in ≥5 of the 12 (task, simulator) pairs, or a
  tool-call ratio ≤ 0.85 in ≥5 of them. No escape clauses — adding a task, dropping a simulator or
  switching the headline metric after seeing rows voids the campaign — and the withdrawal
  paragraph is pre-written verbatim. Five of five validity requirements are recorded as unmet.
- **The tool contract is a first-order term in agent task success, and we proved it on
  ourselves.** The mobile bridge's `turn` delivered **56.7% of the commanded angle**, reproducible
  to four decimals, while its own source comment documented it as "~−19%" and declared a corrector
  unworkable. Rewriting the control law lands it at **mean |error| 0.44°, max 0.97°, n=8** from
  10° to 270°, measured through the shipped tool on an RTX 3060 laptop (machine `9722d23d12a3`) —
  **with no prompt, model or agent-config change**. The honest trade is 8–55 s per turn against
  ~3 s before. Written up with its limits in
  [docs/developer/tool-design-for-agents.md](docs/developer/tool-design-for-agents.md): it is the
  cheapest term to fix, not a claim that it dominates the model — holding this surface constant,
  model choice moved the same swarm suite 4/4 → 0/4, and that ladder has not been run.
- **Every determinism claim is now scoped to a SOLVER, and two were retracted.**
  [docs/benchmarks/determinism-scope.md](docs/benchmarks/determinism-scope.md) is the source of
  truth and overrides every other doc: **bitwise** on ODE and on `newtonSolver "mujoco"` (CPU
  `mj_step`, verified at 336 contacts / 1344 constraint rows with ten live controllers);
  **refuted** on the GPU `mujoco_warp` path — 0 bitwise of 24 same-config cold pairs, diverging
  from ~4e-5 m at 120 steps to **9.152 m at 1000**, with the mechanism traced to `wp.atomic_add`
  contact-slot claiming and confirmed by saturating the GPU with an unrelated process; **unproven**
  on XPBD, whose one green row is a single light-contact scene; **untested** cross-machine. Struck
  outright: the comparison matrix's "Cross-machine determinism: Bitwise" row (our own census has
  56 of 180 lane-1 cells differing between machines) and "lane 3a grades the underlying simulation
  bitwise-deterministic", which does not hold for the configuration the G1 actually trains in.
- **A stationary sun marker can no longer prove GPU determinism.** The lane had reported "GPU
  physics is bitwise deterministic" at `max_abs_dev = 0.0`. The result was **void**: the recorder
  skipped every node without a DEF name, so the graded CSV held exactly one body — `SUN_MARKER`,
  parked at z = 100000 — while the engine's own step trace in the same directory showed the robots
  0.333 m apart by step 120. Nothing asserted the recording had moved, and a run that died halfway
  was graded on the step intersection as a full result. All three fixed, verified by replaying the
  original failing CSVs, which now grade `no_motion` carrying *"this run proves NOTHING about
  determinism"*. The rule it cost us twice in one week is now written down: **an assertion that has
  never gone red should be assumed broken until you make it go red on purpose.**
- **The harness became a mutation surface, and `/sim/step` stopped costing half a minute.** The
  injected supervisor had honoured `--light` for months, but the harness passed no
  `controllerArgs`, so over HTTP the flag was unreachable and every agent paid for the contact,
  joint-limit and grip trackers whether it read them or not. On a 10-Husky, 298-node world (RTX
  3060 laptop; both runs proven non-degraded Newton/MuJoCo by the sidecar): **`/sim/step` went
  28.5 / 27.0 / 27.3 s to 0.047 / 0.035 / 0.034 s**, and a ten-step advance from 120 s to 0.19 s.
  The kicker is that `/sim/contacts` returned zero in both conditions — on that world the trackers
  cost 27 seconds per step to report nothing. Alongside it: `GET /capabilities`,
  `POST /scene/spawn|delete|set_pose`, `POST /sim/snapshot|restore`, and a `/sim/reset` that
  actually resets.
- **The headless lane no longer certifies a world that is physically impossible.** Measured on
  AgentBench task `C2_fall_through_floor`: a world whose floor Solid has no `boundingObject`, so
  its dynamic body free-falls to **z ≈ −69 km** in an 8 s run, printed `0 errors, 0 warnings …
  PASS` **byte-identically to the fixed world**. New opt-in `--fail-on-runaway` injects a sampler
  into a sibling copy of the world and fails on a pure, unit-tested verdict: |z| past a bound, or
  descent below the lowest detected static collider while still accelerating. The broken world
  exits 1 naming the body and its exit z; both independent fixes exit 0; swept for false positives
  on three unrelated worlds.

### Demos & worlds

- G1 flagship harnessed walk pace nearly doubled; harnessed box delivery cut to 91.8 s, with a
  cleaner arrest for box placement and a physical-suction grasp in the harnessed path.
- Arm demos: bin-picking suction feel + a rigid 6-DoF lock and slab-center placement for the box
  grasp.
- Chat demos gained a **zero-account local-Ollama mode** (`OllamaRelay`) and a hybrid tier that
  keeps local inference behind the OmniLink platform layer.
- **Worlds open looking at their subject.** Measured across all 703 `.wbt` files: 45.4 % of
  robot-bearing worlds opened with the robot entirely out of frame, including the no-args entry
  point. Three framing bugs fixed (an up-vector-free look-at, an aspect-ratio-blind frame
  distance, and a third pasted copy of the broken math serving live agents); `projects/` +
  `distribution/` is now 0/305 out-of-frame, and an authored Viewpoint is never touched.
- **HuskySwarm** (`omnilink_husky_swarm.omniworld`): deterministic geometry tools take the per-robot
  trigonometry away from the model; drive accuracy closed to −0.7 %..−0.1 % over 1–2 m by a
  settle-and-verify loop. **Open-loop turns are explicitly not fixed** (~43 % undershoot at
  90°, documented as such) — the closed-loop tools are the accurate path.
- **Northgate Depot**: a reusable robot-free courier-warehouse environment alongside
  city / desert ruins / forest.
- The mobile bridge gained scene-derived obstacle avoidance, a deadlock-free peer layer with
  bounded holds, a time mutex for contested work columns, and a trailer articulation clamp.
- **The 10-Husky flagship runs on the GPU solver from a self-contained world.** New per-world
  `WorldInfo.newtonNjmax` / `newtonNconmax` (`0` keeps the engine's built-in 256, so every existing
  world is byte-identical). MuJoCo's constraint buffers had been hardcoded at 256 with env-var-only
  override, so a `.wbt` could not use `newtonSolver "mujoco_warp"` past eight robots — it would
  only be correct if launched with extra env vars a world file cannot carry, and two independent
  agents building a ten-Husky scene both hit this and both fell back to ODE. A 4WD Husky rests on
  8 wheel-ground contacts × 4 rows = 32 constraint rows, so ten peak at 320 and overflow the 256
  default from tick 4. ⚠️ **Overflow is silent on Windows** — the runtime's only warning is a
  `wp.printf` from inside a warp kernel, and a GUI-subsystem binary captures none of it.
- **The swarm world's "caps at 8 Huskies" comment now names *which* wall.** Two independent limits
  bite near the same robot count and a commit message briefly conflated them: the documented one is
  an **XPBD actuator** wall (wheel targets settling at ~0.4 rad/s against a commanded 2.5), and
  XPBD has no constraint-buffer concept at all, so `newtonNjmax` neither explains nor helps it.
  That 256/32 = 8 lands on the same number is a coincidence of magnitude; the header says so now.
- **The mobile bridge gained an absolute-coordinate verb.** `drive_to(x, y)` — world frame, always
  blocking, returns `achieved_xy` / `error_m` / `arrived`, and **refuses** an off-site target with
  the bound named rather than clamping. This was the single largest gap in the surface: there was
  no absolute-coordinate verb at all, so the model had to compose a rotation with a translation and
  call `atan2` — the two things LLMs are measurably worst at and a tool is exactly correct at.

### Agents

- **Conversational control of every robot in a chat world**, through one shared intent router —
  grounded natural-language commands, with offline / local-Ollama / keyed routing tiers.
- The arm chat bridge gained a **`learn` capability**: a chat request spawns a skill-learning
  runner, streams stage progress (design/validate/train/certify) into the chat and an SSE feed
  with a live HUD page (`GET /hud`), and registers the certified result as a new runnable verb —
  with honest refusals (one learn at a time; unknown recipes listed; a failed certification
  registers nothing). The runner and its recipes are discovered on disk and are not part of
  this snapshot — without them the verb simply lists nothing to learn.
- Deploy integrity: **no controller may report a bare-baseline run as a policy result** — all 21
  policy-loading controllers audited; load failures are FATAL with an asserted `ONNX loaded:`
  line.
- The arm bridge's `set_tcp_target` now accepts a flat `x/y/z` as well as an `xyz` array.
- **A record of the agent's own actions, and promises that survive the turn.** A 40-turn
  stress run found 26 % of turns contained a fabrication — every one about the robot's *own
  past actions*, while zero tool-sourced claims were wrong. So: an action journal, deferred
  intents that are commitments rather than prose, and durable constraints with a real duration.
- **Grounding made structural, because prompting it had stopped working.** Fabrication across
  three measured builds plateaued (26 % → 13.9 % → 13.5 %); two relay gates now apply instead
  (fetch-and-re-ask on ungrounded state claims; capitulations extracted by a pushy operator are
  dropped from storage, loudly). A later graded re-measurement read 5.7 % at n=35, which is
  **not** a significant improvement (p=0.43) and is reported as such.
- **Three ways the intent mechanism itself lied to the operator**, all fixed — including a
  cancel-by-guessed-id that destroyed a live commitment and replied "Cancelled."
- **Memory stopped destroying itself**: persisting was a whole-blob overwrite that erased
  early-session facts from storage; two writers stopped correcting each other; the agent can
  mark a fact durable.
- **A measured safety model** ([docs/developer/agent-safety-model.md](docs/developer/agent-safety-model.md)):
  the e-stop is deliberately not a tool (not listable, not dispatchable), all motion tools
  refuse while it is latched, the last unguarded motion path is now bounded on worst-case
  travel — and the document names the gaps it does not close; bridge-token enforcement is
  opt-in and labelled as such.
- **Agent identity is no longer hijackable by a test run** (`OMNILINK_AGENT_TAG` keys profile
  and memory everywhere), and tool surfaces that under-report are treated as defects — a
  motion tool can no longer run its loop against a latched e-stop and return success.
- **Supervised long-horizon control, and a benchmark it still fails.** New `omnilink_long_horizon`
  suite: the operator gives one natural-language objective, Mission Captain must form a supervised
  delegation plan, and the specialist must tour all four maze corners, return to start and submit a
  verified claim — scored 0–100 from measured robot pose, the physics-derived visited-cell trail,
  live fault state and the bridge claim log. **No model judge.** New runtime: a shared fail-closed
  outcome vocabulary, a dependent-step ledger, `execute_mission_plan` with fresh-state preflight
  and duplicate-ownership rejection, and a corners bridge that refuses completion unless the robot
  is physically back at the start. The graded baseline **FAILS at 35/100**; a later, better-
  exploring run (settled cells 24 → 55) was **terminated mid-episode by an API connection
  error**, so it is recorded `INVALID` and excluded from every pass rate rather than reported as
  a 40/100 result — infrastructure failure is not a model result. The bridge correctly refused
  the specialist's attempted premature completion claim. Note what the harness does *not*
  capture: the result files record host, platform, python and commit only — no GPU, no backend,
  no model — so these runs are not machine-attributable in the way the rest of this release is.
  **This stack is not yet reliable enough for unsupervised long-horizon robot control**, and the
  benchmark exists to say so.
- **A separate hard orchestration suite**, `omnilink-hard/v1` — seven tasks spanning dependent
  motion, compound concurrency, state restoration, ambiguity handling, guard-triggered recovery,
  live delegation and fleet failure containment, every verdict from measured pose and the recorded
  tool trace. ⚠️ Its 2026-07-26 campaign is **exploratory, not a reproducible measurement**: every
  row is `+dirty` at a commit that predates the suite definition, `results/` is gitignored, and the
  runs are model-unpinned. The best available estimate from the only multi-sample run at that tree
  state is **11/13 graded**, with `hard_square_return` and `hard_conditional_delegation` failing.
- **The agent benchmark refuses to run when another copy of it holds the stack.** Two `matrix.py`
  processes ran concurrently for 36 minutes against the same four bridges, and the collision did
  not merely interleave the runs — it corrupted both: one process cleared the other's e-stop fault
  44 times mid-episode, `/reset_to_home` landed inside the other's episode, and every "the other
  robots must not move" assertion saw the other run's robots moving. A file-based `StackLock` now
  claims the stack **before** preflight, exits 4 naming the holder, pid, start time and argv,
  reclaims a dead pid with a printed notice, and releases in a `finally`. ⚠️ Consequently the
  suite's earlier "first measured run" is **retracted and unquotable** — its rows carry
  `model: null` and neither process wrote a summary.
- **Tool results are verified, and mission state is isolated per agent.** Motion outcomes are
  normalized onto one conservative vocabulary in which *unknown* is deliberately distinct from
  *success*; dependent plan steps execute in order and stop at the first unverified step; parallel
  work claiming the same exclusive resource twice is rejected. The helper module has no OmniSim or
  OmniLink dependency, so every production agent shares it and it is unit-tested without a simulator.
- **Two `husky_maze` tools stopped lying about being blocking.** `drive_forward` and `turn` promised
  in their own descriptions to return when within tolerance or after 30 s; neither actually waited,
  so a model that believed the description issued its next command into a still-moving robot with
  nothing distinguishing that from success. Both now honour `wait` and return
  `{commanded, achieved, error, unit, settled, fault, final_pose}`, with `achieved` measured as
  signed progress along the *original* heading axis rather than Euclidean drift. ⚠️ Making it honest
  immediately exposed a second defect: `drive_forward` delivers **~65% of the commanded distance**,
  outside the tolerance its own description claims. No accuracy claim is made for this bridge; the
  number is recorded so the next person starts from it.

### OmniLink platform integration

- **The platform reference lives here now.** The OmniLink website's documentation site was
  deleted; the parts that are the platform boundary moved to
  [docs/guide/omnilink-key-and-api.md](docs/guide/omnilink-key-and-api.md): the Omni Key,
  `/api/chat`, and the `omnilink` PyPI package. The page leads with the case for **not**
  needing a key at all — the bridges fall back to local Ollama and then to the regex router.
- **`python -m omnisim key`** — where you stand plus the exact line for *this* shell,
  `--open` for the page, and `--check`, which asks the platform rather than eyeballing the key
  prefix and prints the plan the platform reports. Stdlib only.
- **`python -m omnisim byok`** — connect the model provider from the terminal (`--providers`
  works with no key and no network; `--add google` takes hidden input; a provider key is sent
  straight to the platform and never written to disk). An Omni Key is not a model key, and the
  command says so in the words of the `402 BYOK_REQUIRED` you would otherwise hit.
- **Presence, so a dead robot stops looking like a live one**: bridges and agents heartbeat,
  publish their own callback URL, and declare their cadence so a healthy 30 s beat is not
  judged against a 3 s window.
- **Setup is two steps, and the CLI now says so.** An Omni Key identifies the account; a
  model-provider key pays for the tokens, and the platform does not fall back to a system
  key — so a user with only a key hit `402 BYOK_REQUIRED` on their first message after
  being told they were ready. `key` now walks through the provider step as a REQUIRED one,
  and `key --check` reports which providers are actually connected. The 402 itself no longer
  echoes the platform's browser-oriented text (which asks for service-account JSON, though a
  free API key works): the relay answers with `python -m omnisim byok --add google`, and
  keeps serving from its local fallback meanwhile.
- **Honest cost readout**: real USD per run and per hour from a local price table (each request
  counted once — the reported rate had been ~2× high), with a price-drift parity test that
  skips cleanly on a public clone.
- **PROTOCOL.md §5.4.1 — the action-result contract, normative.** *An LLM agent has no independent
  access to the world: every belief it holds about what the robot did came from an action result.*
  The measured consequence here: `POST /drive_forward {"distance": 1.0}` returned
  `{"accepted": true, "distance": 1.0, "eta_s": 1.84}` in **0.01 s**, at which instant the robot had
  travelled **0.019 m**. Six requirements now bind: never report a commanded quantity under a field
  name that reads as a measurement; return `{commanded, achieved, error, settled}` against
  bridge-side ground truth; `achieved` **MUST** be `null` when unmeasured; an action that returns
  before completing must say so in its capabilities description and hand back a monotonic `seq`; a
  mutating action arriving while one is in flight should be `409 busy` rather than silently
  replacing it; and a known inaccuracy goes in the description until it is fixed — *shipping a
  −43% actuator behind a description implying exactness is a protocol defect, not a robot defect.*
- **Conformance is recorded per bridge, not asserted in the aggregate**, with each bridge's
  outstanding gap named in its own row rather than summarised away.
- **PROTOCOL.md now documents what the code does**, derived from route registrations and emit sites
  rather than from either doc — and where they disagreed, the doc was wrong every time. There are
  **ten** event types, not twelve: four documented names have no producer at all, and because
  `?types=` is a literal set-membership test, a documented-but-wrong name returns `200` with an
  empty list, so an agent following our docs would poll a dead stream forever. Also: §10's one
  universal requirement — that every event carry `t` in float sim-seconds — **had never been
  implemented**; `/world/render_stats` returns 0–255 statistics while the example showed 0–1, so a
  client thresholding `< 0.1` for "scene is black" matches nothing; and eight
  implemented-but-undocumented harness endpoints are now written down.

### Authoring & harness

- **IPC hardening:** a fail-fast versioned engine↔controller handshake, a zero-tick watchdog, and
  an extern nonce — a mismatched libController now fails loudly instead of hanging every
  controller silently. Broader OmniLink agent/bridge integration hardened (shared
  `http_security`, bridge-conformance tests, chat-plugin sync, PROTOCOL.md).
- `headless_runner` no longer discards engine stderr; `doctor --strict` gained a Tier-0
  install-coherence gate including the engine↔libController IPC mismatch.
- Cross-machine determinism is a three-tier contract with a measuring instrument per tier
  (same-machine golden compare → same-GPU band → cross-machine statistical equivalence);
  `env_fingerprint` records a stable machine identity next to every result.
- **OmniBench harness discipline**: `run_all.py` writes per-machine, per-date result trees
  with a MANIFEST; `SPEC.md` binds the reports to honesty rules (never compare a batched-GPU
  number to a single-env CPU number; report losses as prominently as wins; carry the machine
  id on every number). The harness refuses to benchmark the wrong engine (`--no-fallback`).
- **The agent-benchmark suite is canonical, with its limits stated**
  ([docs/developer/agent-benchmarks.md](docs/developer/agent-benchmarks.md)): verdicts computed
  from supervisor ground truth, infrastructure failure graded INVALID rather than scored, cost
  attributed once, and its own measurement bugs documented rather than quietly fixed.
- Agent runners gained keep-alive (the simulator dying no longer kills the agent) plus a
  guarded `/relaunch-sim` endpoint that spawns the world only and never kills anything.
- New reference: [docs/developer/viewpoint-convention.md](docs/developer/viewpoint-convention.md),
  with `set_viewpoint.py --auto`, a viewpoint checker and a pre-push hook so the framing fix
  cannot silently regress.
- **`GET /capabilities`** — one call for what you are talking to, what is driving the physics, what
  a step will cost and what it will refuse to do. Backend and solver are read from the engine's own
  `.newton.json` verdict sidecar (with `sidecar_stale` / `sidecar_absent` / `forced_by_env` labelled
  honestly — a short run that never reached finalize proves nothing), plus light mode, the
  authoritative event types, every `not_supported` entry with a `reason` and a `workaround`, and a
  rolling median of the **measured** per-step cost with a `recommended_max_steps_per_request`, so an
  agent sizes its budget instead of discovering the RPC timeout by hanging. Anti-drift is structural
  rather than documentary: the supervisor scans its own emit sites and the harness its own route
  table, and `verified` / `undeclared` / `declared_not_emitted` come back in the response.
- **`POST /scene/spawn` | `/scene/delete` | `/scene/set_pose`**, each returning a verification block
  (node resolved, children delta, pose before/after, settle steps) and typed error codes instead of
  prose 503s. Every verb already existed in the shipped Supervisor binding; none was reachable over
  HTTP. Honest headline: building the 10-Husky scene via verbs is a **wash on wall time** against the
  hand-authored file — the real wins are that per-entity VRML disappears, a bad entity is a fast
  typed rejection carrying the rejected text instead of a whole-file reload, and incremental edits
  stop costing a load. Three rules that cost the most to learn ship with it: a `URDFRobot` **cannot**
  be imported from a string (use `{"clone": "<DEF>"}`); a cloned robot's `name` must be rewritten in
  the node text **before** import, because the engine starts the controller at import and keys the
  IPC channel on the name — measured before the fix, **8 of 9 clones were silently dead behind
  `200 OK`**; and nothing checks interpenetration.
- **`POST /sim/snapshot` | `/sim/restore`, and `/sim/reset` FIXED.** Reset had rewound the clock and
  left the scene where it fell — a silent correctness trap we had documented rather than fixed. It
  now restores the engine's own parse-time state, verified on both backends. Restoring an unsaved
  name is **refused**, because the saved-pose map default-constructs a zero vector on a miss and an
  unguarded restore would teleport the whole scene to the origin.
- **An honest audit of our agent-facing surface against ROS 2 `simulation_interfaces`** — all 21
  services, every cell measured against a live harness on both backends rather than asserted. It
  names the places the thesis does not hold, including that we are not the perception surface for
  anything except the camera.
- **AgentBench's graders are a simulator-neutral physical core plus per-simulator adapters**, so a
  cross-simulator number is possible at all: the originals counted engine-specific nodes and read
  our own sidecar, none of which exists elsewhere. In the evidence bundle, `None` **always** means
  "the adapter could not answer" and never `0`, `False` or `[]`. Zero drift was proven the strong
  way — identical captured evidence re-graded by old and new graders in separate interpreters:
  **0 drifted values, 0 removed, 288 added** (all new basis metadata). Every verdict declares its
  **basis**: core-physical (metres and seconds, identical on any simulator), core-structural, mixed,
  or adapter-attested.
- **A Webots adapter, so one grader drives two simulators.** The graded upstream run scores FAIL
  5/10 — and the five failures are **all absent evidence, every one flagged vacuous**: that is our
  schema refusing to award credit it has no evidence for, **not a finding about Webots**. Every
  deliberate asymmetry makes the *control* arm harder, not easier. Two refusals kept: the adapter
  would not read the other lane's own metrics (a competing grader's verdict is not evidence), and
  faced with several candidate logs it refused to guess which was the witness.
- **The 13 lane-3 determinism probe worlds are committed** — they *are* the evidence base. Every
  headline in the scope doc had rested on worlds that existed on one machine only. ⚠️ The buffer
  sweep found a trap worth its own line: setting `newtonNjmax` to exactly the measured peak puts the
  buffer at capacity, rows are silently truncated, and results move **8.81 m** versus every other
  size. **Size the buffer with headroom, not to the measurement.**

### Build / packaging

- Compute-only mode defaults to the Qt `minimal` platform; the packaging file list follows the
  minimal-runtime path.
- Newton runtime wheels are **pinned** to the runtime-pins manifest (an unpinned install had
  picked up an incompatible Newton release and failed every world load).
- **A GPU training image**, published to GHCR by `.github/workflows/train-image.yml` from
  `docker/Dockerfile.train`: OmniSim built from a release tag on CUDA + Ubuntu 22.04 with the
  Newton/mujoco_warp stack baked in, so a GPU box trains immediately instead of reinstalling
  the same stack each time (`docker run --gpus all -it ghcr.io/omnilink-tech/omnisim-train:v5.3.0`).
  The base is deliberately Ubuntu 22.04, not an ML image: the engine embeds CPython and spawns
  controllers as `python3`, so a base whose PATH python differs from the one the binary links
  produces two interpreters, and the resulting missing-`onnxruntime` failure is SILENT — a
  zero-residual baseline that still exits 0. 22.04's system python is the one the engine links.

### Fixed

- Corrected stale claims across README / AGENTS and the guides.
- The quadruped ghost path no longer hardcodes Go2 joint names — Shadowing LUTs declare their
  robot and the trainer asserts it.
- **Newton physics: gravity, inertia, friction cone and ODE gating** (found by OmniBench's
  diagnosis campaign; six measured symptoms, five distinct engine bugs — all five below).
  `WorldInfo.gravity`
  was never plumbed to the Newton runtime, so **every Newton world ran at the library
  default** — blast radius is narrow (only worlds authoring a non-default gravity), but a spin
  test lost all angular momentum in ~0.6 s and now conserves |ω| = 5.0 over 10 s. Solids
  without an explicit `inertiaMatrix` silently inherited a hard-coded wheel preset and now
  take the geometry tensor (rolling error 47.632 % → 0.058 %; revert with
  `OMNISIM_NEWTON_LEGACY_INERTIA_PRESET=1`). New per-world `newtonCone` / `newtonImpratio`
  knobs (friction-transition error 4.065° → 0.065°; a 10-box stack 1/10 → 10/10) — **the
  global default stays MuJoCo-stock** pending champion re-verification. `OMNISIM_FORCE_ODE`
  now actually suppresses Newton (three gating gaps closed), and a supervisor `setVelocity`
  at t=0 is no longer dropped.
- A world that authors no Viewpoint is framed on its scene at load (auto-inserted viewpoints
  only; authored cameras never touched). Z-aligned rescale fixed in Capsule / Cone / Cylinder.
- `omnilink_husky_swarm.omniworld` finalised on XPBD, which locks lateral wheel pairs on 4-wheel
  rovers (a straight 2 m drive came out as a 65° arc); solver pinned to MuJoCo.
- **Contact pairing was structurally impossible, and it silently defeated our own benchmark.** The
  contact record returned the **queried** solid's own id rather than the other body, so every pair
  was keyed `(id, id)` and the endpoint could only ever report a body touching itself. The
  benchmark's "no robot-robot contact in the first 10 steps" assertion therefore **could never be
  non-zero** — half of it was vacuous while reading green for weeks. Contacts are now paired on the
  shared contact point, and the recorder returns a **witness** alongside the filtered pairs, so an
  empty list can be told apart from a pipeline that structurally cannot name two bodies. On the
  current oracle the witness reports zero, so the grader still marks the clause **vacuous —
  correctly**: ten Huskies on a 12 m ring genuinely never touch, and a scene where nothing collides
  cannot demonstrate that a collision check works.
- **The Newton-vs-ODE speedup directive is revoked.** An archived plan carried a *live* editorial
  instruction telling every doc to lead with "Newton is 30–100× faster than ODE" and to frame ODE as
  the legacy baseline — both errors, propagating forward. The retired 17–33× compared whole-engine
  GUI FPS against a bare solver probe; same-harness, ODE is faster per step on these scenes and
  Newton's case is **batching**. History is kept verbatim under superseded banners with the ratio
  columns struck rather than deleted.
- **Every env-steps/s figure now carries its machine, batch and unit; two were retracted.** The same
  in-engine trainer measures **10,228 env-steps/s at batch 256 on a laptop RTX 3060** and **333,036
  at batch 4096 on an RTX 4090**, and one control step is 8 substeps — so an unqualified figure can
  be off by ~33× in either direction *and* another 8× on units alone. One figure that had no run
  file, batch or date anywhere in the tree was **removed rather than restated**.

### Removed

- **Five older agent-driven OmniLink sample demos** retired (warehouse-logistics,
  warehouse-patrol, mission-control among them); the pieces they genuinely shared were folded
  into the demos that remain.
- **Boston Dynamics Atlas** — model, worlds, controllers and checkpoints. The stand experiment
  was a confirmed negative result, and the upstream geometry's chain of title did not meet the
  bar this repository ships under.
- **The `drone_surveyor` agent demo** (runner, tool surface, knowledge base, docs). Its world
  and the Mavic bridge stay — they also serve the aerial chat demo.
- **NASA Valkyrie (R5) is withheld from the public distribution** — the package, its worlds,
  and its stand/shadow artifacts, all of which shipped in v5.1.1. Its upstream licence is the
  NASA Open Source Agreement 1.3: OSI-approved, but not Apache-2.0-compatible, GPL-incompatible,
  not relicensable, and requiring that a copy of the agreement accompany every redistribution.
  Our own licence audit ruled it must not ship until we hold a real position on that, so it is
  held rather than shipped under a claim we cannot stand behind. It remains in development and
  the Shadowing/skill-library docs still reference it, marked as held. `THIRD_PARTY_NOTICES.md`
  keeps the NOSA text and the carve-out for anyone redistributing a tree that does carry it.

---

## [v5.1.1] — 2026-07-12

**The verification release.** Every claim in the README was checked against the public v5.1.0
artifact three ways — statically against the tree, behaviorally on a Linux public clone, and
behaviorally on Windows. Most claims held, including the load-bearing one: the re-hosted
Unitree G1 policy reproduced its documented free-standing walk exactly (63.06 m at 0.483 m/s
over 130 s, zero falls, zero harness variables, pelvis height matching the docs digit-for-digit).
What did not hold is fixed below.

### README corrections

- **The throughput sentence contradicted its own citation.** It claimed the "GPU-batched RL path
  reaches top-tier throughput"; the linked benchmark says, in bold, that end-to-end RL throughput
  was never measured. It now claims physics-stepping throughput in the class the benchmark
  actually supports.
- **Every Atlas world failed to load the robot** — stale URDF paths from an old directory move
  meant the scene silently contained no robot at all. Paths fixed (the 70-link model loads with
  zero errors), and the brand row now says what is true: model + research worlds, locomotion
  unsolved.
- Smaller: Newton's originators quoted per its own repo ("initiated by Disney Research, Google
  DeepMind, and NVIDIA"), the MuJoCo Playground robot count attributed to its technical report,
  and Valkyrie added to the skill-library sentence (the registry carries a Valkyrie skill).

### Agent surface — the harness and capture services now survive slow platforms

- **The harness no longer loses to slow world loads.** On a virtualized-disk Linux box the
  flagship warehouse world cold-loads in 46–79 s; the supervisor bind window was silently capped
  at 60 s, its connect pings could not succeed against a busy supervisor, and an expired bind
  left the engine wedged (or dead with an X11 error) with every RPC returning 503. Five stacked
  causes, all fixed: `wait_s` honored up to 300 s and now bounding only the POST (a background
  waiter with engine-progress detection — log growth, stdout, CPU time — carries on); escalating
  ping patience; expiry cleanly terminates and reports instead of wedging; a repeat load joins
  the in-flight bind instead of killing it; and supervisor adoption is stability-checked so a
  dying controller from the previous world is never re-adopted. Verified on the failing
  platform: warehouse connects inside a single POST (70–72 s), full loop works, hot reload
  included, 2/2.
- **The capture service delivers the resolution you asked for.** The injected high-resolution
  Camera had been removed in May over an engine segfault that no longer reproduces — since then
  every "1920×1080" screenshot was silently a viewport grab (316×316 under Xvfb, ~1896×1113 on
  Windows). The Camera is restored and enabled by default on both platforms; if it is ever
  unavailable, the response now *says so* (`camera_fallback`, with the delivered vs requested
  size decoded from the PNG itself) instead of silently downgrading. The 8-second supervisor
  bind window gets the same slow-platform treatment as the harness (soft `wait_s`,
  progress-aware, clean expiry), and `/capture/sequence` fails fast with an actionable message
  when ffmpeg is missing.
- **Every HTTP bridge now returns strict JSON, always.** An exception inside any route used to
  produce a zero-byte reply (the error visible only server-side), and sensor floats read before
  the first step emitted bare `NaN` — invalid JSON that non-Python clients reject as nothing.
  All four bridges (arm, mobile, quadruped, drone) and the harness's own endpoints now guard
  every route (errors come back as JSON with a 500) and sanitize non-finite floats. Found along
  the way and fixed: a joint/sensor misalignment in the arm bridge when a motor is missing, and
  two device-name-drift bugs that made the drone bridge exit before its HTTP server ever
  started.
- **The Newton import log line can no longer be misread as a verdict.** It used to end with
  "opt-in via `physicsBackend "newton"`" — which reads as "Newton is off" even when the default
  `"auto"` resolves to Newton at world finalise. It now says exactly that, and points at the
  authoritative `<log>.newton.json` sidecar. Every log matcher was verified against the new
  wording.

### Diagnostics

- **The headless runner no longer discards the engine's stderr.** An intermittent early-exit
  (code 1, a trio of Qt teardown warnings, no cause anywhere) was investigated to its edge: it
  is an orderly exit during embedded-Python/Newton bring-up, and its actual reason is printed
  only to stderr — which the runner routed to DEVNULL. stderr now goes to `<log>.stderr` (a
  file, so the historical pipe-stall bug cannot return) and the runner prints its tail on any
  early exit. The next occurrence will document itself.

### Verified on a real cloud pod

- **The Linux recipe is now RunPod-verified, not just claimed.** The shipped
  `linux_bootstrap.sh` was run from the public repo on a fresh RunPod community pod (RTX A4000,
  Ubuntu 22.04.5): build ~3 minutes on datacenter cores, and the backend-verdict sidecar
  confirmed the real GPU solver — `{"backend":"newton","degraded":false,"finalised":true}`.
  Total verification cost: about four cents.
- **Two cloud-pod traps the trip exposed, both now handled by the bootstrap:** (1) ML pod images
  ship **two Pythons** — `python3` on PATH is the image's (3.11) while the engine links the
  distro's (`libpython3.10`), so wheels installed with plain `python3 -m pip` land where the
  embedded interpreter never looks. The gpu phase now reads `ldd bin/omnisim-bin` and installs
  into the **linked** interpreter. (2) On a pristine machine, warp compiles its CUDA kernels on
  first use — minutes, not seconds — so the smoke phase auto-retries once with a
  kernel-compile-sized window instead of misreporting "no sidecar" as an ODE fallback. Also:
  containers report the host's core count (112 on the pod), so the build's default job count is
  now capped.

### Documentation

- Sidecar guidance corrected: ≥15 s on a fast disk, **≥45 s for cold loads on virtualized disks**
  (WSL2, cloud pods) — and a missing sidecar on a short run proves nothing.
- Linux runtime notes: Pillow for `/world/render_stats` (with the PEP-668 reality), ffmpeg for
  movie encoding, the running-as-root warning, and honest cold-load expectations for
  asset-heavy worlds on slow disks.

## [v5.1.0] — 2026-07-12

**The Linux release.** OmniSim now builds, runs, and drives Newton GPU physics on Linux — from
the public repository, verified end-to-end (Ubuntu, RTX GPU): the engine builds in ~7 minutes,
worlds load and step headless, and the backend-verdict sidecar confirms the real GPU solver
drove the run: `{"backend":"newton","degraded":false,"finalised":true,"solver":"MuJoCo
(mujoco_warp, WorldInfo.newtonSolver)"}`. A flagship locomotion demo (G1 box delivery — on the
disclosed balance harness) ran end-to-end on Linux with its torch LSTM policy.

There is no separate Linux engine: it turned out nothing in the physics code was ever gated to
Windows. What stood between Linux users and a working simulator was a handful of small build and
packaging defects, plus documentation that said — wrongly — that it could not work. Both are
fixed.

### Linux support

- **Build from source works on Linux.** Five defects fixed: the Python-embed flags are now
  discovered via `python3-config` instead of a hardcoded `python3.12` (builds against whatever
  Python the distro ships); the Windows-only Newton runtime-bundle step is OS-guarded so
  `make release` no longer dies after a successful link; the Qt installer survives PEP 668
  (`externally-managed-environment`) on Ubuntu 24.04+; `WbSoundEngine.cpp` gained an include it
  only received transitively on Windows; and every tracked `.sh` script now carries its
  executable bit (57 files — `make` invokes several directly on Linux).
- **One-command setup: [`scripts/install/linux_bootstrap.sh`](scripts/install/linux_bootstrap.sh).**
  apt prerequisites → clone with submodules → `make release` (Qt 6.5.3 arrives automatically via
  `aqtinstall` from Qt's own servers) → GPU wheels → an Xvfb smoke test that **hard-fails**
  unless the Newton sidecar reports a non-degraded, finalised GPU run. This is the supported
  recipe for cloud GPU pods (RunPod and similar Ubuntu + CUDA images).
- **Newton on Linux needs no bundle.** The Windows runtime bundle is a Windows-specific
  DLL-resolution mechanism; on Linux the engine's embedded interpreter resolves the **system**
  `python3`, so `pip install torch warp-lang newton mujoco mujoco-warp` into the system
  interpreter is the entire setup. ⚠️ A **venv is invisible** to the embedded interpreter —
  installing the wheels there produces a silent ODE fallback. NVIDIA/CUDA GPU required for the
  GPU solver.
- **The runtime environment is now supplied automatically.** Invoking `bin/omnisim-bin` directly
  on Linux used to abort with a Qt version clash (`version 'Qt_6.10' not found`) because the
  launcher's `LD_LIBRARY_PATH` was bypassed. The headless runner, the agent harness and the
  capture service now inject the correct Linux runtime env (`LD_LIBRARY_PATH`,
  `QT_QPA_PLATFORM=xcb`, `WEBOTS_TMPDIR`, software-GL fallback) themselves. Xvfb remains
  required for headless boxes — the engine creates a Qt/XCB context even with `--no-rendering`.
- **Python controller spawning:** the engine now falls back to `python3` when a `runtime.ini`
  asks for a bare `python` that does not exist (takes effect with the next engine rebuild; the
  Linux dependency script also installs `python-is-python3` so current binaries work today).
- **Every locomotion deploy demo now has a bash launcher.** The 14 `scripts/dev/run_*deploy*.ps1`
  launchers and the flagship walker gained `.sh` siblings (the `.ps1` remain for Windows). The
  ports are proven faithful — for all 15, the fully-assembled launch environment is identical
  between the two — and live-verified on both platforms: on Windows (Go2 deploy, +25.4 m, no
  fall) and **on Linux from the public repo** (Go2 with its ONNX policy active: **+85.5 m over
  225 s sim, no fall**, Newton sidecar non-degraded). RL-residual demos on Linux additionally
  need `onnxruntime` in the system Python (the bootstrap installs it).
- **The RL trainers' early-stop watchdog works on Linux** (it was silently disarmed off Windows:
  it read an MSYS-only `/proc` field and used `taskkill`; it now falls back to process-group
  signals). Thirteen developer tools that hardcoded the Windows binary path now use the
  cross-platform resolver.

### Documentation

- **The platform-support story is rewritten to match reality — in both directions.** The README
  claimed Linux gets no Newton, no RL and no locomotion demos, and that "the demo launchers are
  PowerShell"; the flagship demo launchers are bash, and Newton on Linux is now verified. In the
  other direction, `system-requirements.md` still carried text inherited from upstream Webots
  claiming OmniSim is "ensured to run" on Ubuntu and macOS — deleted. **macOS remains untested
  and is documented as exactly that.** The quickstart gains a real Linux section.
- Known caveat, stated plainly: **Ubuntu 22.04 / 24.04 are the recommended targets** (broadest
  Python-wheel coverage for the GPU stack). Newer releases work when wheels for their Python
  exist.

## [v5.0.0] — 2026-07-11

**The robot-learning release.** OmniSim gains a complete, robot-agnostic pipeline for
*making a legged robot do a motion*: design a dynamically-feasible reference (a **ghost**),
prove it feasible **before** training, learn to track it (**Shadowing**), package the result
as a versioned **skill**, and compose skills into task sequences (**BATON**). Training moved
**in-engine** — policies now train *through* `omnisim-bin`, so train == deploy bit-exact — and
the cloud training path was **removed**: everything runs on the GPU you already own.

This release supersedes **v4.5.0**, which was version-bumped but never published; all of its
content ships here.

> **A note on honesty, up front.** The humanoid results below are real but **assisted**: the
> G1 walk, box-delivery, and turn demos run on a *visible balance harness* (a "puppet" rig)
> that carries part of the robot's weight and stabilises its attitude. **A durable,
> free-standing humanoid walk remains OPEN.** Every capability claim in this file is
> caveated inline, and the single canonical per-robot answer to "is it actually done in
> deploy?" always lives in
> [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) — if any headline
> here disagrees with that file, **that file is right**. See *Known limitations* at the end.

~370 commits since v4.0.0.

### ⚠️ Breaking changes

- **The `cloud/` Modal-H100 training path is REMOVED.** Training is **in-engine and local by
  policy**. The 16 Modal wrappers were thin subprocess shims over trainers that already ran
  locally. Use [`projects/policies/training/run_walk_rl.sh`](projects/policies/training/run_walk_rl.sh)
  (in-engine, train == deploy) or the standalone-but-still-local trainers in
  `projects/policies/research/training/`. The `OMNISIM_MODAL_GPU` env var is dead.
- **`projects/rl/` → `projects/policies/`** (685 files), with a `control` / `controllers` /
  `worlds` split from `research/`. Update any external paths and imports.
- **Robot removed:** an experimental humanoid model and its demo.
- **Robot asset packages withdrawn pending confirmation of redistribution terms.** We audited the
  provenance of every robot model we redistribute. A few carried no stated licence for their
  source geometry. Rather than keep redistributing CAD whose terms we cannot evidence, those
  packages and the demos that exist only to drive them are withdrawn from distribution until the
  terms are confirmed in writing. The simulator, the arm bridge, the grasping stack and the URDF
  importer are unaffected — point them at any URDF you have the rights to. We would rather ship
  fewer demos than redistribute a manufacturer's CAD we cannot account for.
- **Upstream licences now ship with the robots that need them.** The same audit found geometry we
  redistribute under BSD-3 and NASA-1.3 whose licence text we were not reproducing — a real
  compliance gap, not a formality. Each robot package that carries a third-party licence now ships
  that licence verbatim beside its meshes, and `NOTICE` / `THIRD_PARTY_NOTICES.md` name the actual
  licence and copyright holder for every one.
- **Arms: the openly-licensed lineup is back.** v4.5.0 had narrowed the supported arms down to a
  single vendor's. With that vendor's package now withheld (above), the **Universal Robots
  UR3e / UR5e / UR10e** (BSD-3) are restored, along with their chat demos, the
  `omnilink_multi_arm` world (3× UR5e), and the UR5e loader in `warehouse_logistics`. All three
  are verified driving over the arm bridge (joint tracking to ~0.02–0.03 rad; DLS IK reaches a
  Cartesian target to ~4.6 cm). Every arm OmniSim ships is now one whose redistribution terms we
  can point at. *(The Franka Panda is restored in-tree but **not** shipped in v5.0.0: its URDF
  declares no inertials, so the importer synthesises gram-scale links and the arm will not hold a
  pose. It ships when it works.)* Robotiq / OnRobot / Schunk / vacuum / magnetic
  grippers are unaffected. *(Carried over from the unreleased v4.5.0.)*
- **`book_delivery` → `box_delivery`** across skill manifests, sequences, and demo scripts.
- **Demos retired:** the arm `bin_grab` demo (a one-wall friction grasp is fundamentally marginal)
  and the standalone construction-site demo + benchmark (the *environment* world is kept).
- **The GUI now defaults to the dark Night theme** on all platforms.
- **`run-world` gained a first-run conformance gate** — it can block an interactive launch on
  FAIL. It fails *open*, and `OMNISIM_SKIP_CONFORMANCE=1` bypasses it.
- **The controllers' `warmup_reload` helper is now a no-op** — the cold-first-load articulation
  bug it worked around is fixed, so the startup reload is gone.

### Highlights

- **The three train→deploy gaps are CLOSED — train == deploy is now bit-exact.** The
  discretization mismatch (the live engine steps 4×0.004 s; the trainer was doing 4×0.002 s),
  the settle-and-go handoff, and a post-step joint-limit clamp were the last three. Result: a
  durable straight real-foot G1 walk in the engine. v4.5.0 shipped this as ⚠️ OPEN.
- **In-engine training is the flagship venue.** Policies train *through* `omnisim-bin`
  (Newton / `mujoco_warp`), so there is no second physics stack to keep in sync. GPU-resident
  in-engine PPO runs **~44× faster** than the previous path (~140–200 k env-steps/s sustained on a
  laptop GPU, via zero-copy `wp.to_torch` + CUDA-graph capture).
- **Shadowing** — the robot-agnostic motion method: a *generator* produces a
  dynamically-feasible reference ("the ghost"), a *verifier* numerically certifies it
  **before** any RL, and a *tracker* learns to follow it. The thesis: **reference feasibility
  is the bottleneck** — so verify, then learn.
  ([docs/developer/shadowing.md](docs/developer/shadowing.md))
- **Ghost design formalised as gates.** Feasibility is now four checkable gates — kinematic
  closure, COM support, force-wrench membership, and **PD-realizability** — and `ghost_synth`
  builds ghosts where they hold **by construction**. The accompanying **corridor-vs-torque
  law** (a tracking corridor must exceed τ_ff/kp or the reference is untrackable *by
  construction*) retro-explains a long run of previously mysterious training failures.
  ([docs/developer/ghost-design-rules.md](docs/developer/ghost-design-rules.md))
- **The Skill Library** — the standard packaging of Shadowing + BATON. One versioned manifest
  per skill binds its ghost, validator verdict, deploy env, champion checkpoint, and
  provenance; `skill_lib.py` covers `list` / `preview` / `train` / `run` / `sequence` /
  `verify-demos`. 10 skills across G1 / H1 / Go2 / Spot, 4 BATON sequences.
  ([docs/developer/skill-library.md](docs/developer/skill-library.md))
- **BATON policy switching** — composes specialist policies (walk / turn / carry / stand) into
  task sequences with engineered handovers.
- **First-party MCP server** ([`packages/omnisim-mcp/`](packages/omnisim-mcp/)) — a
  dependency-free stdio JSON-RPC proxy to the `:6789` harness (14 tools), so Claude Desktop
  and Cursor drive OmniSim natively.
- **Newton is now honest about being Newton.** A silent Newton→ODE downgrade is **fatal**
  instead of quiet, and the engine writes a race-free `<log>.newton.json` verdict sidecar at
  world finalisation — its presence is proof Newton drove *that* run.

### Robot learning — Shadowing, skills, BATON

- **Skill Library** (`projects/policies/skills/`): manifests + `skill_lib.py`, cross-cadence
  adapters, per-edge handovers, freeze, and cross-robot reuse. `verify-demos` asserts each
  manifest reproduces its hand-written demo script **key-for-key on the assembled launch env**.
- **BATON** sequences: `box_delivery`, `box_delivery_classic`, `walk_turn_walk`, `turn_solo`.
  ⚠️ BATON's *thesis* — that switching specialists degrades more gracefully over a long
  horizon than one distilled monolith — remains a **well-posed open hypothesis**; the
  success-vs-horizon experiment is unrun. ([docs/developer/policy-switching.md](docs/developer/policy-switching.md))
- **Ghost toolchain**: `ghost_synth`, `ghost_validator`, `ghost_doctor` (a prescriptive
  classifier), `ghost_polish`, `ghost_close`, `ghost_funnel` (the gate-4 PD-realizability
  funnel), `ghost_ff`, `ghost_topp`. The **WBMATCH** similarity metric reached v4 — an honest
  *shape-only* ruler that no longer flatters a policy for tracking a corridor it was handed.
- **`ghost_synth` motion library** — one method (plan the contacts, solve the base + joints),
  one judge, generalising across **walk, squat, kneel, push-up, and 3 cm + 7 cm stairs**. It
  also *measured* the quasi-static walking speed limit (vx ≈ 0.131 m/s), which retroactively
  explained why an earlier "0.45 m/s" reference was only achievable on a crane.
- **In-engine quad RL** — Go2 now trains *through* the engine with no MJCF reparse, so train ==
  deploy bit-exact. On a **trainer-side batched eval** of the deploy-identical model (4096 envs):
  **94.8 % never fell over 48 s, 16.6 m mean, 0.357 m·s⁻¹**; Spot smoke 69.5 %.
  ⚠️ **That is a batched evaluation inside the trainer, not a live single-robot deploy run** — the
  in-engine champions have not yet been given an equivalent live long-run. The live Newton deploy
  walks on record are still the *standalone*-trained policies (Go2 **+86.7 m**, Spot +47.8 m,
  B2 +110.7 m, 0 falls). ⚠️ B2 stiffness is **unreconciled**.
- **Terrain-curriculum quad RL** — blind (observations unchanged), Spot clears 18 cm bumps and
  rubble with 0 falls.
- **Binary-level train↔deploy parity probe**, generalised robot-agnostically — all 6 legged
  robots PASS.
- **Unitree's own official G1 and H1 policies now run inside OmniSim**, which is where the
  recorded reference ghost comes from.

### Demos & worlds

- ⭐ **The Decent Walker** — the flagship humanoid demo: the G1 walks the official Unitree gait
  beside its ghost hologram, with natural thigh-clearing arm swing (LSTM + foresight champion,
  WBMATCH4 **0.868** on the shape-only ruler). ⚠️ Runs on the **visible balance-harness puppet
  rig** — an overhead support that carries part of the robot's weight and stabilises attitude.
  **This is not yet a free-standing walk.** Known-open: live stride runs below the trainer.
- ⭐ **Box delivery** (BATON) — the G1 walks to a cart, lifts a 1.5 kg box, carries it, sets it
  down on a second cart, takes a real ~90° footwork corner, and walks away. **0 falls.**
  ⚠️ Harnessed. ⚠️ **The carry is kinematic, not a grasp.** The box is a real rigid body under
  gravity, but it is an ODE body while the robot runs on Newton — so hand↔box contact is
  structurally impossible. During the carry the box is posed to the hand centroid each tick and
  holds ~1 cm hand clearance by design: *the hands never touch it*. The payload is real to the
  **policy** (it is the trained carry plant, `CARRY_PAYLOAD_KG`), and the locomotion, the corner
  and the fall-free record are real. The grasp is not — see
  [policy-switching.md](docs/developer/policy-switching.md).
- ⭐ **Walk-turn-walk** — a genuine 90° footwork turn in sequence: **90.6–95.6° actual, 3/3,
  0 falls**. ⚠️ Harnessed.
- **G1 stair climb** — a full **5-step live climb**, and the one demo where **the legs do all the
  *vertical* work** (`HARNESS_KZ=0`: no vertical crane assist). A companion demo climbs all five
  treads and then holds a near-motionless stand on the top landing via a position-gated BATON
  handover.
  ⚠️ **The crane is off vertically, but not in attitude — and the champion leans on it.** Our own
  motion-legitimacy verifier (which ships:
  [`verify_motion_legitimacy.py`](projects/policies/training/verify_motion_legitimacy.py)) **FAILS**
  this champion: the attitude springs sustain **|ty| ≈ 77.5 N·m on 77 % of climb ticks** (the crane
  carries the lean), and its **knees contact the treads on 13.6 % of climb ticks**. The stand demo
  passes roughly **1 run in 2** (retry on a fall). It clears the *kinematic* gates; it does not
  clear the *dynamic* ones. Treat this as a promising result under scrutiny, **not a finished
  demo**. Full write-up: [motion-legitimacy.md](docs/developer/motion-legitimacy.md).
  ⚠️ **3 cm risers is the measured ceiling** for the stock-foot G1 — 4 cm degrades to ~2 steps and
  5 cm to none. Taller risers need a foot-morphology change or a vertical assist.
- **Ghost-follow holograms** — the ghost now renders the deploy's *active* reference
  end-to-end through walk → turn → walk, plus dedicated "show" worlds.
- **G1**: manipulation while standing, cube-defense stand, one-leg balance, deterministic squat
  overlay, arm-motion skill.
- ⚠️ **G1 army-crawl** — feasibility is a GO and the reference is designed, but it is **not a
  working motion**: the 25 N·m arms are the wall.

### Agent & bridge surface

- **The arm bridge's real-hardware path is now a pluggable backend, not a hardcoded vendor.**
  Drop a `<name>_backend.py` next to the controller and select it with
  `--hardware-backend <name>` / `--hardware-ip <addr>` (or `OMNILINK_HARDWARE_BACKEND` /
  `OMNILINK_HARDWARE_IP`); the bridge discovers it by module name against a small
  `HardwareBackend` protocol (start/shutdown, status, joint + linear moves, home, grasp/release,
  stop). With no backend installed the bridge is pure simulation and the option is not offered;
  asking for a backend that is not installed fails loudly rather than silently running sim-only.
  **Breaking:** the old vendor-specific spellings are gone — the hardware IP flag is now
  `--hardware-backend`/`--hardware-ip`, the status route is `GET /hardware_status`, and the
  per-vendor key in `/get_robot_state` and the robot-window payload is now `hardware` (carrying
  the backend name). No backend ships in the public snapshot, so no published consumer breaks.
- **The `/sim/events` stream no longer drowns itself.** A Newton registration census intended to
  fire once per world build was re-firing every tick, so **~99 % of the `controller.log` traffic on
  `GET /sim/events` was one repeated line** and real controller output was being dropped
  (`dropped_log` in the hundreds of thousands on a long run). It now fires once per build, still
  re-fires on reload, and stays silent for worlds with no Newton bodies. This was breaking the
  exact HTTP-harness debugging loop [AGENTS.md](AGENTS.md) and [PROTOCOL.md](PROTOCOL.md) tell
  agents to use.
- **The in-app demo launcher no longer offers a button that cannot work.** The policy demos need a
  deploy environment that only their shell script exports; their cards previously had a live
  *Launch* button that loaded a bare world and left the robot lifeless. They now show the exact
  command with a copy button instead.

### Engine & physics

- **Cross-tree contact — characterized, not solved.** `mujoco_warp`'s *island* path (active only
  when a free prop exists — i.e. exactly when you try to grasp something) produces NaNs on the
  first robot↔free-body contact. We now understand it and can reproduce it on demand: a box
  *does* rise under real palm contact, but some runs still go non-finite, so **friction grasping
  is parked**. The working pick-and-place path is a contact-free suction coupling instead.
  This is a diagnosis, not a fix.
- **No silent Newton→ODE fallback** — capability-gate downgrades, orphaned Newton joints, and
  a MuJoCo→XPBD solver downgrade are now **fatal**. Escape hatches: `OMNISIM_ALLOW_ODE_FALLBACK`,
  `OMNISIM_FORCE_ODE`.
- **Newton verdict sidecar** — the engine writes `<log>.newton.json` at world finalisation, so
  "did Newton actually drive this run?" is answerable without scraping logs (the old
  log-scrape was fooled by large logs and could falsely report ODE).
- **Newton compound-body inertia** corrected; degenerate welded-static and dynamic-bin inertias
  de-degenerated (they were causing a contact drop / sink).
- **Launch-flake fixed** — the Windows IPC pipe name was salted only by TCP port, so
  back-to-back launches could cross-connect. Now folds in the PID: **105/105 launches connect
  first-try** (was ~90 %).
- `staticBase` robots were being welded at the origin instead of their spawn pose — fixed.
- **G1 stand deploy observation bug** — the engine's angular velocity is a different frame and
  scale than MuJoCo's `qvel`; using it raw caused a ~1.8 s deploy fall.

### Performance

- **GPU-resident in-engine PPO: ~44×** (~140–200 k env-steps/s sustained on a laptop RTX 5070 Ti;
  ~218 k peak at K=2048).
- **Realtime**: even `newtonSubsteps` re-enables CUDA-graph capture (0.25× → **1.16× realtime**);
  forcing the wgpu/Vulkan main view takes the live GUI from ~0.4× to **~1.2× realtime**.
- **Persistent-sim job server** — keeps one simulator alive across queued jobs: evaluations go
  from 4–6 minutes to **9 seconds** (~30×).
- Harness `render_stats` vectorized (**17×**).

### Packaging, tooling & conformance

- **`omnisim verify-install`** — a per-backend acceptance manifest with hard canaries and soft
  physics bands, plus a fingerprint and an advisor. Wired into `run-world` as a first-run gate.
- **Newton runtime version-parity guard** + CI test — the bundler was `pip install --upgrade`-ing
  while the trainer pinned exact versions, so "bit-exact train == deploy" could silently rot.
- **Reproducible Newton scaling benchmark**, replacing a deleted headline probe.
- **Warp contact-kernel validation harness** — steps physics-only worlds on both `mujoco_warp`
  and CPU `mj_step`. Tiers 1–2 are **clean (10/10 cells)**, which *rules out* the prop contact
  kernels as the grasp defect.
- **~1 GB of tracked training checkpoints purged** — only the referenced champions stay tracked,
  enforced by a pre-push guard.
- Agent runners consolidated onto a shared `OmniLinkAgentRunner` / `_lib` (−4.4 k LOC).

### Documentation

New: [`simulator-comparison.md`](docs/developer/simulator-comparison.md) (10 simulators, every
claim marked verified / unaudited / vendor-claim, adversarially checked),
[`ros2-integration.md`](docs/developer/ros2-integration.md) (**ROS 2 is an explicit non-goal** —
with a working external-bridge recipe, and a pointer to Gazebo for ROS-centric work),
[`skill-library.md`](docs/developer/skill-library.md),
[`ghost-design-rules.md`](docs/developer/ghost-design-rules.md),
[`policy-switching.md`](docs/developer/policy-switching.md),
[`train-deploy-gap.md`](docs/developer/train-deploy-gap.md),
[`closed-loop-chaos-diagnostic.md`](docs/developer/closed-loop-chaos-diagnostic.md) (classifies a
train-vs-deploy divergence as BUG / CHAOS / MATCH before you theorise),
[`install-conformance.md`](docs/developer/install-conformance.md), and the
`projects/policies/training/` + `projects/policies/skills/` READMEs.

The **Shadowing paper** ([`docs/developer/shadowing_paper/`](docs/developer/shadowing_paper/)) —
the method writeup, published here for the first time. Every reported result is a real deploy
rollout traceable to
[rl-current-state.md](docs/developer/rl-current-state.md), and the G1 results state up front that
they run on a partial-support balance harness.

### Legal & licensing

We audited the provenance of every robot model OmniSim redistributes, and fixed what the audit
found. This was a real compliance gap, not paperwork.

- **Upstream licence texts now ship with the geometry they cover**, as BSD-3 clause 1 and NOSA §3
  require: **Unitree** G1 / H1 / Go2 / B2 (BSD-3), **Spot** (BSD-3, Clearpath), **Atlas** (BSD-3,
  Robot Locomotion Group @ CSAIL), **Valkyrie** (NASA-1.3), **Universal Robots** UR3e / UR5e /
  UR10e (BSD-3), **Franka Panda** (Apache-2.0, plus the upstream `NOTICE`). Previously we shipped
  187 Unitree geometry files with no licence text at all.
- **`NOTICE` and `THIRD_PARTY_NOTICES.md` rewritten** to name, per robot, the upstream URL, the
  licence, the copyright holder, and the path to the licence text in this tree. **Spot and Atlas
  leave the "unverified terms" carve-out** — both are cleanly BSD-3 (Spot's meshes were authored
  by Clearpath, not supplied by Boston Dynamics). The carve-out is now **Valkyrie alone**, plus
  the Code2000 fonts.
- **Valkyrie is NASA-1.3 (NOSA)** — OSI-approved but GPL-incompatible and **not relicensable**,
  and it obliges every redistributor to carry the agreement. Labelled accordingly.
- **Universal Robots licenses its UR20-class meshes under separate, restrictive terms.** OmniSim
  ships only the BSD-3 ur3e / ur5e / ur10e families. Do not add a UR20 without re-reading them.
- **Spot renders textured again** — 13 of its meshes referenced a `spot_mat.png` we were not
  shipping. It comes from the same BSD-3 package; it now ships.
- **Trademarks** are used nominatively only. "TurtleBot" is a registered OSRF mark.

### Known limitations (read before quoting any result)

- **No durable free-standing humanoid walk *from a policy we trained*.** Our G1 walk / delivery /
  turn demos run on a weight- and attitude-bearing balance harness; removing it is the open
  problem. For calibration: **Unitree's own official G1/H1 policies, re-hosted unchanged in
  OmniSim, walk free-standing with no harness** (G1: 33.7 m, 0 falls, 0.48 m/s). The engine
  carries an unassisted humanoid walk — our *training* is what doesn't, yet.
- **Stair climbing caps at 3 cm risers** on the stock-foot G1. The 7 cm ghost passes every
  feasibility gate — but **gate-pass is not a climb**: no policy climbs 7 cm live; it plateaus
  at ~2 of 5 steps against a propulsion wall.
- **The stair champion does not pass our own dynamic audit.** The climb is legs-only *vertically*
  (`HARNESS_KZ=0`), but the harness still has attitude authority, and
  [`verify_motion_legitimacy.py`](projects/policies/training/verify_motion_legitimacy.py) — which
  we ship — **FAILS** the champion: the attitude springs sustain ~77.5 N·m on 77 % of climb ticks,
  and its knees contact the treads on 13.6 % of them. It clears the kinematic gates and not the
  dynamic ones. We built the verifier, it caught our own demo, and we are shipping both.
- **We do not have a contact-physics grasp.** Friction grasping goes non-finite on palm contact
  (`mujoco_warp`'s island path) and is **parked**. What works is a *contact-free* suction coupling,
  and the G1 box-delivery carry is **kinematic** — the box is posed to the hand each tick with
  ~1 cm clearance; the hands never touch it. Verified pick-and-place exists on the arm side; a
  humanoid that closes its fingers on an object and holds it by friction does not.
- **The arm demos are pick-and-place, not dexterous manipulation.** The arms that ship (UR3e /
  UR5e / UR10e, Franka Panda) do position control, IK and suction/parallel-jaw picking. There is
  no in-hand manipulation, no force control, and — per the previous point — no friction grasp.
- **BATON's switching advantage is unproven** — an open hypothesis, not a result.
- **B2 quadruped stiffness is unreconciled.**
- **Newton physics, the RL pipeline and every legged demo are Windows + NVIDIA only** today.
  Linux/macOS run the simulator on the ODE fallback. See the platform-support table in the README.
- **Sim-to-real is unproven.** OmniSim demonstrates train == deploy parity *in-engine*. That is
  not the same as zero-shot transfer to physical hardware, and **no policy trained in OmniSim has
  been validated on physical hardware.**

---

## [v4.5.0] — 2026-06-24 *(version-bumped but NEVER PUBLISHED — its content ships in v5.0.0 above; retained for provenance)*

The **Shadowing** release. This cycle is mostly a new, robot-agnostic motion-control
*method* and the demos built on it, plus more Unitree robots. It also **narrows the
supported arm lineup to a single 6-DOF cobot arm** (removing the Universal Robots and
Franka Emika Panda arms — see Removed) and adds a reusable **STEP/CAD → URDF converter**. A note on honesty up front: much of the motion/RL work
below is **sim-validated with an open deploy gap**, flagged inline. The single
canonical, per-robot "is it actually done in deploy?" answer always lives in
[docs/developer/rl-current-state.md](docs/developer/rl-current-state.md); if any
headline here disagrees with that file, that file is right. ~340 commits since v4.0.0.

### Highlights
- **Shadowing** (the headline; formerly "ghost-tracking") — a robot-agnostic motion
  pipeline. A *generator* (trajectory optimization) produces a dynamically-feasible
  reference (the "ghost"); a *verifier* numerically certifies feasibility **before**
  any RL; an RL *tracker* then learns to follow ("shadow") it through to deploy.
  "Planning describes, control solves." Method writeup + research-paper scaffold:
  [docs/developer/shadowing.md](docs/developer/shadowing.md).
- **Arm toss-to-place via Shadowing** — the arm *throws* a cube into a bin **beyond
  its reach** (impossible to carry or solve with IK), landing ~1.5 cm from centre;
  full generate → verify → deploy in OmniSim Newton. The verifier *rejecting* too-far
  bins (the feasibility frontier) is the point of the demo.
- **More Unitree robots** — Unitree **Go2** walks in Newton deploy via the Spot
  residual stack (plus a retarget recipe for other Unitree quadrupeds); Unitree
  **B2** get-up (rise) via Shadowing.

### Demos & worlds
- **Arm toss-to-place** and **arm throw-and-catch** — flagship Shadowing
  manipulation demos.
- **Spot velocity-conditioned walk / stop / walk** — one policy takes a commanded
  speed (including 0 = stop); walks, stops, then resumes. Deploy verdict in
  rl-current-state.md.
- **G1 stand-and-wave**, **G1 sit→stand→sit**, **seated G1 arm-mimic** — Shadowing
  test-beds on the 23-DOF humanoid; sim-validated. On-screen deploy of a *stationary*
  full-body stand remains an open project gap (walking deploys; a static full-body
  stand does not yet).
- **Hill-walk** ghost pipeline for Spot + B2 (gait pitched to the incline, with a
  live preview); the RL tracker is still blocked at the flat→ramp transition.

### Robots & PROTOs
- Unitree **Go2** (walk); Unitree **B2** (walk + get-up scaffolding). Retarget recipe
  for additional Unitree quadrupeds (Go1 / A1 / Aliengo / B1).
- **`scripts/dev/step_to_urdf.py`** — a reusable STEP/STP → URDF converter (tessellate →
  colour-split → URDF; doc: [docs/developer/step-to-urdf.md](docs/developer/step-to-urdf.md)).
  Turns any CAD assembly into a simulatable robot.

### Reinforcement learning
- **Shadowing realized end-to-end** (generator + verifier + tracker) on the arm
  toss, and documented as the canonical recipe.
- **Spot** velocity-conditioned walk/stop/walk; **Go2** walk; **B2** rise — see
  rl-current-state.md for the honest deploy verdict on each.
- ⚠️ A **durable G1 humanoid deploy walk remains OPEN**; the G1 Shadowing demos
  (sit-stand, stand-and-wave) are sim-validated, with the on-screen full-body-stand
  deploy gap noted above.
- **H1 + Valkyrie walking shadows** — feasible walking references (designed +
  ghost-verified) for the Unitree **H1** (5-DOF leg) and NASA **Valkyrie** (130 kg,
  6-DOF leg), plus an **H1 Phase-2 deploy-physics fine-tune trainer** (batched MuJoCo
  solver). ⚠️ sim-validated; the RL tracker that closes the sim-to-deploy gap is
  pending — see rl-current-state.md.

### Removed
- **Universal Robots (UR3e / UR5e / UR10e) and the Franka Emika Panda arms** — OmniSim's
  manipulator lineup is narrowed to a **single 6-DOF cobot arm**. This
  removes the `projects/robots/universal_robots/` and `projects/robots/franka_emika/`
  asset packages; the `omnilink_ur3e` / `omnilink_ur5e` / `omnilink_ur10e` /
  `omnilink_panda` chat demos; the `omnilink_multi_arm` (3× UR5e) world and its
  `OmniSim-Foreman` agent template; the UR5e-specific controllers (`ur5e_omnilink_bridge`,
  `ur5e_ik_slave`, `ur5e_teleop`); and the `panda_hand` gripper preset. The
  `omnilink_arm_bridge` arm registry now contains a single arm. The `warehouse_logistics`
  flagship loses its dock-side UR5e Loader (the Warehouse Foreman now runs the mission
  Picker-only), and the `Axis` control agent is re-pointed to that arm. The Robotiq /
  OnRobot / Schunk / vacuum / magnetic grippers are unaffected.
- **Construction-site demo + benchmark** — the standalone construction-site demo and its
  benchmark were removed; the construction-site *environment* world is kept for reuse.

### Fixed
- **Free-body velocity writes under Newton** — `node.setVelocity()` on a *free*
  (non-articulated) body was a no-op under the Newton/MuJoCo backend (it wrote the
  body twist instead of the free joint's `joint_qd`); fixed so Supervisor velocity
  sets on free bodies take effect. This is what makes the toss release-velocity land.

### Documentation
- [docs/developer/shadowing.md](docs/developer/shadowing.md) — the Shadowing method +
  paper scaffold.
- [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) consolidated
  as the single canonical RL status; the v4.0.0 RL headlines were corrected against it
  (see the **Errata** in the v4.0.0 section below).
- **`CLAUDE.md`** added at the repo root (a one-line `@AGENTS.md` import) so Claude
  Code loads the same project instructions every other agent already reads from
  `AGENTS.md` — no duplicated content, other tools unaffected.

### Build / packaging
- Newton runtime bundle size documented consistently as **~600 MB** (the CHANGELOG and
  the Makefile comment previously said ~450 MB).

---

## [v4.0.0] — 2026-06-12

The engine flips and the walking release. **Newton becomes the default
physics solver** (`physicsBackend "auto"` resolves to Newton; the build
ships `OMNISIM_WITH_NEWTON=ON`); the wgpu render arm reaches its
architectural baseline and is compiled in by default, but **WREN remains
the runtime default main-view renderer** — the Phase ζ flip is deferred
because wgpu is not yet a feature superset of WREN. `OMNISIM_LEGACY=1`
demonstrably reverts the whole stack to ODE + WREN. On top of those
arms: **G1 walks 212 m / 10 min in deploy with zero falls** (mathematical
foot-space planner + IK + residual RL); the first Newton-native Spot
walker walks STRAIGHT with locked heading; the **AnyPick Line** demo
sorts mixed parts with a learned ONNX classifier; **Omni Quest** routes
a Husky across the whole city via an OpenStreetMap-derived sidewalk
graph and a cost-optimal A* router; **The Living City** ships as a
generator-driven 4×4 urban grid; Open Robot Combat (ORC) and arm
bin-picking land as polished demos. 652 commits since v3.0.0.

> **Errata (updated 2026-06-23):** three reinforcement-learning headlines in this
> v4.0.0 entry were later found to overstate the *deploy* result. The honest,
> canonical per-robot status is
> [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) — where it
> disagrees with the bullets below, it is right.
> - **G1 walk** — "G1 walks 212 m / 10 min, zero falls" does **not reproduce in
>   deploy** (that policy topples ~1 s). **No from-scratch G1 policy walks durably
>   free-standing**; the deployed ones topple in ~1.3–1.7 s, and the good-looking G1
>   gait is harness-supported. A durable free-standing deploy walk is **still open**.
>   (Later figures quoting a "finite ~34 s bout" are also not reproducible — that
>   checkpoint is absent from the repo. Quote no G1 walk distance from this section.)
> - **G1 stand** — the deploy stand is real, but it is solved by a **deterministic
>   classical pose (statics), NOT RL**; the heavy-DR RL residual actually
>   *destabilises* it (~2.4 s vs 12 s+ for the pure pose).
> - **Spot "walks straight under Newton"** — under **Newton** the chassis tips (roll
>   instability); at v4.0.0 Spot walked straight only under **forced ODE**, and even
>   there the learned residual was a *passenger* (≈ the same distance with no policy
>   at all). Spot/Newton walking improved in later cycles — see the v4.5 notes and
>   rl-current-state.md for current status.

### Breaking changes

- **`OMNISIM_WITH_NEWTON` defaults to ON.** The build links the Newton
  XPBD solver by default. Distributions that need the old behavior must
  build with `OMNISIM_WITH_NEWTON=OFF` explicitly. **Source builds also
  need the Newton Python runtime** (`newton`, `warp`) on the embedded
  CPython's `sys.path` — run `make -C src/omnisim bundle-newton-runtime`
  to vendor it next to the binary (one-time, ~600 MB). Without it,
  worlds silently fall back to ODE; confirm Newton is live via the
  `[WbNewtonBackend]` line in startup log. Release installers ship the
  runtime pre-bundled. See
  [docs/developer/newton-runtime-bundle.md](docs/developer/newton-runtime-bundle.md).
- **`Solid.physicsBackend "auto"` resolves to Newton.** Worlds that
  depend on ODE-specific contact behavior must pin `physicsBackend
  "ode"` on the Solid (or via the world template). The capability gate
  auto-falls-back to ODE for Newton-unsupported features (e.g. mixed
  hinge+ball articulations) and warns; check `omnisim_log.txt`.
- **`OMNISIM_WITH_VULKAN` defaults to ON** (the build flag that gates
  the wgpu render arm). Builders without a `wgpu-native` toolchain must
  pass `OMNISIM_WITH_VULKAN=OFF` explicitly. **The runtime default
  main-view renderer remains WREN** — wgpu is compiled in but opt-in
  per world via `Viewpoint.renderBackend "wgpu"` (or test-only
  `OMNISIM_WGPU_MAINVIEW_FORCE=1`). The Phase ζ default flip is
  deferred because wgpu is not yet a feature superset of WREN. See
  [docs/developer/wgpu-renderer-status.md](docs/developer/wgpu-renderer-status.md).
  In short, the v4 engine defaults are **Newton physics + WREN
  rendering**.
- **`physicsBackend` ancestor resolution is now strict.** An explicit
  ancestor backend governs descendant `"auto"` — previously articulation
  could split solvers and leave sensors frozen. Worlds that relied on
  the loose behavior may need an explicit per-Solid override.
- **The full-range revolute importer reclassification.** URDF joints
  with a full ±2π range were previously misclassified as velocity wheels
  under Newton; they now register as revolutes. Worlds whose behavior
  depended on the misclassification need to be re-tuned.
- **`OMNISIM_FORCE_ODE=1`** is the new escape hatch for ODE-only
  determinism tests on a Newton-ON build. `OMNISIM_LEGACY=1` reverts
  both physics *and* rendering to the legacy ODE+WREN combo and is the
  documented full-stack rollback.
- **Sponsor button + funding wiring.** A repository-level Sponsorships
  toggle is now expected; `SPONSORS.md` auto-refreshes daily via a
  GitHub Action against the Sponsors API.

### Headlines (new since v3.0.0)

- **G1 humanoid WALKS in deploy** — `+212 m / 10 min, zero falls` _(⚠️ does not
  reproduce in deploy — see Errata above)_ via a mathematical human-gait model (foot-space planner + IK + residual
  RL on top). Earlier in the cycle: G1 STANDS in deploy; then G1 walks
  the platform `+25.9 m / 68.5 s, zero falls`; then full-body 23-DOF
  walk with natural arm swing. The deploy bridge is 56× faster than
  the first cut (`0.08× → 4.5× realtime`); the previous bottleneck was
  launch/sync, not the solver.
- **AnyPick Line** — a perception-driven, learned-classification
  end-of-arm-tool bin-picking line. L-conveyor feeds yawed bins of
  mixed parts; the arm sorts with a 200 KB ONNX classifier (gear / bar /
  T-bracket / tube + a model-free unknown lane). 5-seed reliability
  sweep: 5/5 PASS, 96% emptied / 92% sorted. Cycle time `318 s → 168 s`
  for 18 parts (1.9×). Breakable suction (shear budget), overlapped
  bin staging, contact-honest vacuum (no magnet grabs), strict
  top-down tray placement.
- **Omni Quest full-city navigator** — Husky routes the whole road
  grid via `build_city_graph` (OpenStreetMap → sidewalk graph derived
  from the road grid) and a general cost-optimal A* router with faster
  reroute. A learned free-space segmenter transfers sim→real (ML
  perception loop), and a histogram-match adaptation closes the
  camera sim-to-real gap by 96%. Stereo cameras bumped 128×96 →
  256×192 to close the structural sim-to-real gap. A Locomotion
  actuation seam (twist commands, not wheel velocities) makes the
  algorithm cross-platform (Husky + Jackal, same code).
- **Spot deploy hardened** — six root causes fixed across trainer /
  eval / deploy; Spot now walks with LOCKED heading (gait yaw-steering
  fix + env-port self-collision deadlock fix).
- **city_traffic perf 3×** — `0.34× → 1.0× realtime` end-to-end on
  the showcase city.
- **All RL worlds migrated to Newton** — no ODE stragglers; robot-
  combat demos migrated too; the arm assembly-line demo retired.

### Engine architecture (the migration baseline lands)

- **`architectural-baseline-v1` tag** on commit `bfbe1262` — both arms
  of the engine migration are *structurally* done; the runtime default
  flip for rendering (Phase ζ) is deferred (see
  [wgpu-renderer-status.md](docs/developer/wgpu-renderer-status.md)).
- **`OMNISIM_LEGACY=1` reversibility proven** via `scripts/dev/
  reversibility_check.py` + `OMNISIM_PROBE_BACKENDS` — flipping the
  knob restores ODE+WREN end-to-end.
- **P1.6 joint-op widening COMPLETE** — hinge angle reads, slider +
  AMotor reads, user-defined force/torque writes, per-step
  `setParam(FMax, Velocity)`, joint enable/disable lifecycle,
  world-load `setParam` family, Hinge2 + Ball `setParam`. WbSolid is
  fully migrated off direct ODE joint ops.
- **Unified engine plan** — the five rendering plans and six physics
  plans were consolidated into a single canonical
  [engine-migration-plan.md](docs/developer/engine-migration-plan.md).
- **Default-flip migration framework** — Stage 0 (escape hatches +
  dual-backend oracle + physics CI gate via invariants), Stage 1
  (capability-gate auto→ODE for Newton-unsupported features), Stage 2
  (Newton safe-default base guard + `WorldInfo.substeps`/`statics`),
  Stage 3 (Newton build default flips ON), Phase D FIRE (Solid.wrl +
  Robot.wrl `physicsBackend` default → `"auto"`).
- **`WorldInfo.defaultPhysicsBackend` / `defaultRenderBackend`** —
  pin a whole world to a specific backend pair without touching every
  Solid/Camera.
- **Symmetric legacy escape hatches** — `OMNISIM_FORCE_ODE`,
  `OMNISIM_FORCE_WREN`, `OMNISIM_LEGACY` (sets both).
- **Multi-session migration plan** — parallel-lanes split for engine
  work spread across sessions, with sanctioned isolated git worktrees
  and atomic path-scoped commits.

### Rendering (wgpu — the new render arm, opt-in)

- **wgpu architectural baseline COMPLETE** — A1/A2/B1/B2/B4/C1/C2 all
  signed off. The `WbRenderBackend` C2 surface is final; the
  `WbVulkanBackend` (a thin shim) routes through the wgpu pane.
- **wgpu beats WREN on the city benchmark.** Quality gate at 65%
  within-tolerance (gate threshold: 55%); 2.7× faster main view via
  draw-list and bind-group caches + async pipelined readback.
- **Phase ζ runtime default flip DEFERRED.** WREN remains the default
  main-view renderer; wgpu opt-in via `Viewpoint.renderBackend "wgpu"`
  or `OMNISIM_WGPU_MAINVIEW_FORCE=1`. Decision recorded
  2026-06-11.
- **Native window-swap** — wgpu presentation for the main view via the
  native swapchain; non-sRGB swapchain fix kills the double-gamma
  washout on window swap; Mailbox present for frame pacing.
- **Reversed-Z depth** in the wgpu main view — kills far-field
  z-fighting (city orbit distance).
- **Authored near-plane** — eliminates road z-fighting at orbit
  distance.
- **SSAO** in the wgpu main view — contact darkening with depth weight.
- **Bloom** post-process in the wgpu main view.
- **HDR + AgX tonemap infrastructure** for the main view (opt-in via
  `OMNISIM_WGPU_AGX`).
- **Distance fog + anisotropic filtering** in the wgpu main view;
  linear-space fog composition fixes the zoom-out fade; WREN-exact fog
  curve restores distant street/grass colour.
- **Camera-following fitted shadow frustum** — 4× near-shadow density
  on the same atlas; city-scale shadow frustum widened (12 m → 90 m
  half-extent, light back 130 m).
- **Main view un-gated** — `renderBackend "wgpu"` renders live (3c-B);
  the offscreen → GL blit path landed first, then the surface path
  superseded it (Option B interop ruled infeasible on pinned
  wgpu-native v29).
- **Material fidelity ladder** — albedo, roughness, metalness, normal
  maps; Cook-Torrance GGX specular BRDF; sRGB color management on lit
  shaders; specular highlight and worldPos plumbing landed via T1.1.
- **AgX filmic tonemap** ported to engine WGSL with pre-tonemap
  exposure, emissive HDR source, and a golden-image regression gate.
- **CSM cascaded shadow maps + PCF** (T1.2) — per-cascade
  orthographic light viewProjs, clip-depth shadow shader, shadow-pipeline
  3-binding group, two-pass shadowed render method, 3×3 percentage-closer
  filtering, multi-cascade receiver shader. Soft shadow edges observed;
  cross-object cast shadow verified by controlled A/B.
- **TAA** (T1.4) — sub-pixel Halton jitter + ping-pong history
  accumulator + temporal-resolve pass; `OMNISIM_PROBE_TAA_JITTER` /
  `OMNISIM_PROBE_TAA` probes.
- **Distance fog** (T1.3) — analytic distance-fog resolve;
  `OMNISIM_PROBE_FOG`.
- **Hemisphere-IBL ambient** (T1.x) — world-general hemisphere ambient
  in the wgpu main view + no-reference quality measure.
- **Atmospheric sky + day/night** in the wgpu main view.
- **Emissive term** in the wgpu textured-shadow path — city lights at
  night.
- **Soft shadows** — natural wgpu shadows with contact bias, 5×5 PCF,
  strength 0.8.
- **MSAA 4×** in the textured-shadowed pass.
- **wgpu sensor family** (R5) — RangeFinder (R32Float real-meters
  depth), Camera (sRGB sensor output kept linear), Lidar
  (single-layer → multi-layer → wide-FOV → multi-frustum → tilt →
  rotating-head → wide-FOV rotating). All with regression guards.
- **3c-A interaction kit** — picking, selection highlight, line +
  wireframe pipeline (bounding objects, COM, contact points, joint
  axes, surface normals, lidar rays, camera/sensor view frustums),
  translate / rotate / scale gizmos with hit-test drag, distance-scaled
  handles, depth-independent always-on-top markers, two-batch live-pane
  overlays, full-screen overlay compositing, screenshot pipeline,
  per-pixel roughness map, support polygon overlay.
- **Newton → wgpu interop** demonstrated end-to-end (Phase δ) — bulk
  Newton body-translation snapshot API feeds a wgpu storage-buffer
  instanced draw.
- **wgpu-native v29 integration** — sync-callback fix; LIBS-not-LFLAGS
  link fix; `OMNISIM_PROBE_WGPU` smoke knob; on-screen window present;
  texture cache LRU cap + path-keyed dedup.
- **R3.x runtime-verified path** — vertex/uniform/bind-group/pipeline-
  layout path; mesh cache + vertex buffer pipeline; texture bridge with
  cache + sampler + textured pipeline; QImage → wgpu texture adapter;
  golden-image regression harness; `WrStaticMesh → wgpu` byte-stream
  adapter; `WbCamera` routes through `WbWgpuRenderTarget`.
- **WREN → engine bridge** — `WbCamera` migrated onto the shared
  `WbWgpuSceneRenderer`; collect articulated-robot geometry (joints +
  CadShape) in the main view; WREN mesh-readback validation + plain-
  Appearance fallback in the wgpu collector.

### CUDA / physics (Newton — the new default)

- **Phase D FIRE** — `Solid.wrl` + `Robot.wrl` `physicsBackend`
  default flipped to `"auto"`. Worlds without an explicit backend now
  run on Newton.
- **W-series native primitives** — native plane collision (W1.1),
  native cylinder as oriented capsule (W1.2), native triangle-mesh
  collision (W1), Hinge2 2-DoF support (W2), ball/spherical joint (W2.2),
  external force/torque injection (W3.1 spine), mid-step body velocity
  sets (W3.2), supervisor force/torque API routed to Newton.
- **Native contact readback (W4)** — `get_contacts()` verified
  in-binary; C++ contact accessor + native-vs-ODE comparison harness;
  native contacts feed the supervisor API (verified 0 → 4 transitions);
  native contacts REPLACE the ODE source for multi-body verified.
- **Coverage meter (N-MEASURE)** — Newton-coverage dashboard; current
  reading ~60% of the ODE feature surface.
- **Real per-contact penetration depth** — `ContactPoint.depth`; the
  damage subsystem can opt in via `cp.depth` scoring (`L4`).
- **Arm friction grasp under Newton** via `SolverMuJoCo` — pick-place
  works on Newton (commit `6beae669`).
- **Newton actuates `staticBase` arms** (pin base, fixed-joint root);
  Newton unwraps Shape `boundingObject`s (no more `r=0.12` placeholder);
  prismatic joints + effort-scaled position gains; joint sensors read
  live angle (eval_ik under XPBD).
- **MuJoCo solver-stability engine knobs** — `OMNISIM_NEWTON_SUBSTEPS`
  fixes XPBD NaN at high drive speed; `OMNISIM_DAMAGE_VEL_SMOOTH`
  de-jitters contact velocity (57k → 58 events).
- **`OMNISIM_NEWTON_MJWARP`** — deploy via mujoco_warp (GPU) vs CPU
  mj_step; seed-rebuild uses the deploy engine, not hardcoded CPU.
- **statics-on-Newton dispatch (P8)** — top-level colliders register
  as Newton static bodies (opt-in); fixes Newton chassis-freeze caused
  by static furniture wrongly getting dynamic bodies.
- **Control-mode-aware joint ke/kd** — restores legged position-hold
  (G1 stand regression fix).
- **Articulation single-solver rule** — explicit ancestor backend
  governs descendant `"auto"`; fixes Spot frozen-sensor regression.
- **Rescue full-range revolute arms** from velocity-wheel
  misclassification (the `<rest>` trick generalized).
- **Multi-husky world-load hang fixed** — WbLog message accumulation
  was the culprit.
- **Newton 1.2.0 stable installed and verified** — Phase D gate #2 met.
- **Newton runtime bundling** — `bundle-newton-runtime` make target
  + warn at package time if unbundled.
- **Rolling-friction knob** — `OMNISIM_NEWTON_ROLL_MU` (AnyPick tubes
  stop rolling under it).
- **World-reload singleton teardown** — world reload silently dropped
  robots to ODE because the Newton backend singleton wasn't torn down.
  Fixed.
- **Collide-skip perf restored** — earlier exoneration audit confirmed
  it; restoring it brings back the throughput.
- **G1 walk deploy perf 56×** — `0.08× → 4.5× realtime`. The bridge
  was launch/sync-bound, not the solver.

### Reinforcement learning (humanoid + manipulation)

- **G1 WALKS in deploy — HUMAN GAIT.** _(⚠️ Corrected: does not reproduce in
  deploy — see Errata above.)_ `+212 m / 10 min, zero falls` in deploy. Recipe: mathematical foot-space planner + IK +
  residual RL on top, gait-v2 levers, the four launch fixes, and the
  deploy-gap ledger. Earlier in the cycle: G1 walks the platform
  (`+25.9 m / 68.5 s, zero falls`), G1 walks with arms (full-body
  23-DOF deploy), G1 walks with natural ARM SWING.
- **G1 humanoid (Unitree, 23-DOF) stands in deploy.** _(⚠️ This is a deterministic
  classical pose — statics, not RL; see Errata above.)_ First stable
  humanoid deploy stand on OmniSim. Recipe: heavy domain randomization,
  train on Newton-exact MJCF, deploy with stiff PD + planted feet +
  tuned CoM/ankle PD. The root cause of the original deploy gap was
  forward CoM + destabilising ankle PD, not a sim-to-sim gap.
- **GPU-native G1 trainer** — 5 stacked speedups vs the prior version;
  GPU mujoco_warp residual standing trainer.
- **Active capture-point balance law** for G1 deploy; deterministic
  G1 balancer prototypes (Phase A validation log).
- **Atlas RL pipeline** — port G1 stand recipe to Atlas; baseline-
  equivalent stand policy + DR curriculum.
- **Trainer initial DR** — base-tilt and base-velocity init randomization
  (`--dr-init-tilt-band` / `--dr-init-vel-band`).
- **Canonical two-layer control architecture** — deterministic
  controller + RL residual; documented in [docs/developer/](docs/developer/).
- **Sim-to-deploy gap closing** — three deploy obs-frame fixes for
  Newton; deploy obs feeds body-frame ang-vel to match the trainer.
- **First Newton-native Spot walker** _(⚠️ see Errata above: under Newton the chassis
  tips; straight-line walking was forced-ODE only at v4.0.0)_ — `spot_residual_main` walks
  STRAIGHT under Newton via the model+residual recipe (analytic CPG
  prior + learned residual correction), with heading + steer-to-
  centreline hold for path tracking. Six root causes fixed across
  trainer / eval / deploy; gait yaw-steering fix; env-port
  self-collision deadlock fix; Spot walks with LOCKED heading.
- **All RL worlds migrated to Newton** — no ODE stragglers left.
- **Arm bin-picking journey** — friction grasp under Newton; suction
  end-effector empties dense bin 36/36 via grab+shake singulation;
  tilt-and-pour bin emptying; colour-sort 36 cubes into trays; real
  collider walls + 18-part pile; camera-driven randomly-filled bin
  routine; residual-RL grasp layer.
- **Manipulation push/declutter exploration** — residual lifts
  emptying 15 → 18 cubes; pushing capped by gripper geometry.

### Demos & worlds

- **The Living City** (`showcase/city_traffic.omniworld`) — 4×4 generator-
  driven urban grid: 48 cars routing with right-turns + 2-phase signals,
  36 pedestrians on zebra crossings, wall-to-wall mixed-use blocks,
  shops and restaurants with cafe seating, central park and landmarks,
  city bus that pulls up at stops, day/night cycle with a lit-up night.
  Regenerable via [`scripts/dev/gen_city_traffic.py`](scripts/dev/gen_city_traffic.py)
  + `city_grid.json`.
- **`environments/city.omniworld`** — mixed urban street block as a default
  environment biome; `Car.proto` + parked/queued fleet.
- **Omni Quest** (`projects/omni_quest/`) — outdoor GPS+camera nav:
  M1 GPS waypoints, off-road course (rough terrain + obstacle avoidance),
  real camera-based obstacle avoidance (M3), cross-platform navigation
  (Husky + Jackal, same algorithm), interacting swarm, pedestrian
  sidewalk navigation, stereo-camera + GPS local planner, OpenStreetMap
  → routable walking graph, KITTI stereo + GPS + IMU sensor adapter,
  native EKF state estimator, deploy-with-reroute live in the city.
- **G1 deploy worlds** — `rl/g1_stand_deploy.omniworld`,
  `rl/g1_stand_arms_deploy.omniworld`.
- **Spot Newton deploys** — `rl/spot_residual_deploy_newton.omniworld` (+
  perturb variant); the model+residual recipe live.
- **Open Robot Combat (ORC)** — `robot_combat/orc/orc_open_field.omniworld`
  (2v2), `orc_forest_war.omniworld` (3v3 across a 40 m wooded battlefield),
  `orc_queen_defense.omniworld` (protect-the-queen mode); 20v20 verified
  end-to-end through the harness.
- **BattleBox** — contact-point-gated damage, physical damage +
  immobilization win condition, loud winner banner; tribute matches
  (Hydra vs Gravedigger flipper-vs-spinner, Gravedigger vs BiteForce
  heavyweight, --weapon-mode pulse for flippers).
- **Arm bin-pick worlds** — dense bin emptying, suction + grab/shake,
  tilt + pour, colour sort, robot-tilts-bin demo.
- **AnyPick Line** — perception-driven sorting line. Shape-agnostic
  suction bin picking + shape sort for the arm; L-conveyor feeds yawed
  bins of mixed parts (18/18 sorted); LEARNED part recognition via a
  200 KB ONNX classifier (gear / bar / T-bracket / tube + a model-free
  unknown lane). Reliability sweep: 5 seeds, 5/5 PASS, 96% emptied /
  92% sorted. Cycle time 318 s → 168 s (1.9×). BREAKABLE suction (seal
  has a shear budget). Overlapped flow (next bin stages during
  picking). Contact-honest vacuum (no magnet grabs). Strict top-down
  tray placement (no joint-arc swing). Containment (no part ever
  falls out). Sticky-gum gripper + model-free unknown-part lane. Gears
  + bars route to the BLACK bin.
- **`omni_quest` full-city navigator** — Husky routes the whole road
  grid. `build_city_graph` derives the sidewalk graph from the road
  grid; general cost-optimal A* router + faster reroute; learned
  free-space segmenter transfers sim→real (ML perception loop); camera
  sim-to-real gap closed 96% (histogram-match adaptation); stereo
  cameras bumped 128×96 → 256×192; Locomotion actuation seam (twist
  commands, not wheel velocities) makes the algorithm cross-platform
  (Husky + Jackal, same code).
- **city_traffic perf 3×** — `0.34× → 1.0× realtime` end-to-end on
  the showcase city.
- **`robot_combat` demos migrated to Newton**;
  the arm assembly-line demo retired.

### Procedural worlds (`omniworld`)

- **`city` biome** — joins forest, desert, urban_block, warehouse,
  indoor_apartment, and mars in the recipe set.
- Living-city regeneration via `gen_city_traffic.py` + readable
  `city_grid.json` (resize via XS/YS, re-run).

### Authoring & harness

- **`#include` directive for world files** (VRML extension).
- **`--minimize` + DEVNULL stdout/stderr** in headless runner —
  unblocks Newton 20v20 in batch.
- **Sensor parity oracle** (R5) — wgpu Camera path verified against
  WREN with retry + duration knobs.
- **Capability-gate harness** — Tier B verified (mixed hinge+ball
  articulations auto-fall-back to ODE with a warning); Tier A
  best-effort-safe.
- **Physics dual-backend oracle** — legacy verifies Newton via
  `scripts/dev/render_oracle.py`.
- **Physics CI gate via physical invariants** — `scripts/dev/
  physics_oracle.py` validates both backends agree on energy /
  momentum / quasi-static equilibrium.
- **Render-arm completion checklist** + golden-image regression gate.
- **Newton coverage meter** (`scripts/dev/newton_coverage.py`).
- **wgpu probe knobs** — `OMNISIM_PROBE_WGPU`, `_PROBE_PICK`,
  `_PROBE_INSET`, `_PROBE_CSM`, `_PROBE_TAA`, `_PROBE_TAA_JITTER`,
  `_PROBE_FOG`, `_PROBE_BACKENDS`, `_PROBE_TEX`.
- **MinGW runtime DLLs shipped into `lib/controller/`** so Windows
  controllers load.

### Damage system

- **Contact-point-gated damage** — opt-in `cp.depth` scoring in
  `battlebot_damage_director` (L4).
- **`cp.depth` restricted to non-load-bearing parts** to avoid spurious
  chassis-on-chassis inflation.
- **`OMNISIM_DAMAGE_VEL_SMOOTH`** — de-jitter Newton contact velocity
  (57k → 58 events).
- **DamageTracker reset** clears `vel_ema` per its docstring.
- **damage_director** strips leading DEF from exported part before
  reinserting.

### Build / packaging

- **`OMNISIM_WITH_NEWTON ?= ON`** is the new build default (Stage 3).
- **`OMNISIM_WITH_VULKAN ?= ON`** is the new build default (C1
  architectural baseline).
- **`wgpu-ON` builds auto-ship `wgpu_native.dll`** (supported config).
- **Newton runtime bundling tool** — `bundle-newton-runtime` make
  target; package-time warning if unbundled.
- **`build_with_cd.sh`** forwards `OMNISIM_WITH_VULKAN` and
  `WGPU_NATIVE_HOME` to make.
- **`setup_wgpu_native.sh`** repaired — `wgpu-native v29.0.0.0 (gnu)`
  pinned.
- **Sponsors automation** — `.github/workflows/update_sponsors.yml`
  auto-refreshes `SPONSORS.md` daily from the GitHub Sponsors GraphQL
  API (four tiers via marker pairs).

### Release infrastructure

- **`publish_snapshot.sh` bypasses the local pre-push smoke hook** via
  `OMNISIM_SKIP_PUSH_CHECK=1`. The hook ships that env var as its
  documented escape hatch for binary-less worktrees.
- **`tmp_*.err` scratch files added to `publish_deny.txt`** — agent-run
  stderr captures stay private.
- **Sponsors auto-refresh GitHub Action** runs daily; opt-in tier
  markers in `SPONSORS.md`.

### Documentation

- **Consolidated engine plan** — five rendering plans and six physics
  plans absorbed into a single master plan
  ([docs/developer/engine-migration-plan.md](docs/developer/engine-migration-plan.md)).
- **R3 wgpu-native + Newton-interop design** rewritten and pinned.
- **R4 → wgpu-default completion checklist** as a living tracker.
- **G1 standing playbook** + general sim-to-deploy RL recipe.
- **Canonical RL state doc** — correct stale "stands forever" headlines
  and add base-divergence guard.
- **Humanoid balance gap** — the actual blocker for Atlas + G1 RL,
  characterized.
- **Contact API consumer inventory** + coordination requirement +
  migration entanglement notes.
- **Newton runtime bundling guide** + Newton-as-default release
  procedure.
- **Sponsors automation runbook** (private).
- **Architectural baseline checklist** + final default-flip plan.
- **Parallel-lanes split** for multi-session migration work.
- **Reproducible codebase-stats.md** ("lines we wrote").
- Dropped the unverifiable "first" claim from public positioning.
- **Webots-named icons + web frontend files** renamed to OmniSim.

### Fixed

- **`physicsBackend` ancestor governs descendant `"auto"`** — Spot
  frozen-sensor regression.
- **Articulation must use one solver** — split-solver articulation was
  a regression source.
- **Newton chassis-freeze** — static furniture wrongly got dynamic
  bodies.
- **Newton motor ke/kd wiring** — env defaults silently overrode
  `WbBasicJoint` values.
- **Newton head-on XPBD NaN** at high drive speed — `_SUBSTEPS` knob.
- **G1 floor-contact deploy regression** in the recipe.
- **Spot W6 deploy collapse** — diagnosis pinned to ROLL/lateral
  stability + control-bridge issues.
- **Newton multi-husky world-load hang** — WbLog message accumulation.
- **Newton world-reload silently dropped robots to ODE** — singleton
  world teardown missing on reload.
- **wgpu reversed-Z + authored near plane** — far-field z-fighting and
  road-at-orbit-distance z-fighting on the city.
- **wgpu periodic motion hitch** in the main view eliminated.
- **wgpu non-sRGB swapchain** — window-swap double-gamma washout.
- **wgpu linear-space fog composition** — zoom-out fade fixed.
- **wgpu WREN-exact fog curve** — distant street/grass colour stops
  fading to a wrong tone.
- **wgpu `TextureTransform` applied** — omni_quest grass was one tile
  smeared over 200 m.
- **wgpu `castShadows` honored** — sun marker was shadow-bombing
  spot.omniworld.
- **wgpu corpus-sweep fixes** — dither, normal-offset shadows,
  diagnostic switches.
- **Deterministic offscreen parity golden** — crash fix + gate
  recalibrated.
- **Render-parity gate clarification** — WREN is legacy-dark
  (advisory, not a regression).
- **Spot GPU pipeline sim2sim fidelity** — six root causes fixed
  across trainer / eval / deploy. The pipeline was never sim2sim-
  faithful before this.
- **Spot env-port self-collision deadlock** + **gait yaw-steering
  break** — Spot now walks with locked heading.
- **AnyPick LINE queue** — yawed bins no longer interpenetrate.
- **AnyPick LINE containment** — no part ever falls out.
- **AnyPick LINE strict top-down tray placement** — no joint-arc
  swing.
- **AnyPick LINE true-surface gauge** for model-free unknowns + long-
  part wall corridor + flush model-free picks (no hover).
- **AnyPick LINE tube rolling** — fixed via the Newton rolling-friction
  knob.
- **AnyPick LINE contact-honest vacuum** — no magnet grabs.
- **WbCamera FOV match** — wgpu pane frames at any aspect (R4-3b
  polish).
- **wgpu shadow-render non-determinism** — dangling `modelMatrix16`
  per-draw uniform; pinned via headless RenderDoc capture.
- **Textured-shadowed floor-drop** — pinned to GROUND-plane skip; root
  cause is a timing/sync race, fix landed.
- **Camera sensor sRGB regression** — keep camera SENSOR output linear
  (sRGB is display-only); R5 regression green.
- **wgpu texture-cache key bug** — path-key the cache to dedupe
  shared-file textures; multi-world soak now clean across 6 worlds.
- **wgpu main-view reload crash** + NaN-normal handling + live blit.
- **`build_with_cd.sh` engine-flag forwarding** — exporting before the
  build no longer reaches the child make.
- **`isolated-worktree engine build`** — Qt5+Qt6 coexistence link
  failure documented; junction the gitignored vendored deps.
- **G1 STANDS in deploy** — root cause was forward CoM + destabilising
  ankle PD (not a sim2sim gap).
- **G1 RL deploy obs-frame** — body-frame ang-vel matches the trainer.
- **Spot deploy heading + steer-to-centreline hold** — walk straight
  down the path.
- **Newton actuator position-target writes** were silently no-op'ing.
- **Newton chassis visual** now follows physics (WREN-push fix).
- **MinGW runtime DLLs** ship into `lib/controller/` so Windows
  controllers load.

### Removed

- **Splash images for non-shipped robots.**
- **Stale Cyberbotics path strings** + dead-language pages
  (docs cleanup).
- **`WEBOTS_HOME` self-reference bugs** purged from dev docs ahead of
  public publish.
- **Pre-rebrand `Webots-named` icons + web frontend files** renamed to
  OmniSim equivalents.
- The legacy single-solver articulation policy (now strictly enforced).
- **The arm assembly-line demo retired** — superseded by the AnyPick
  Line and the bin-pick suite.

---

## [v3.0.0] — 2026-05-25

OmniSim stops introducing itself as a Webots fork and starts behaving as
its own engine. Five "dual-accept" compatibility shims that the v2.x
rebrand phases left in place are now strict OmniSim-only (env var, URL
scheme, binary name, project manifest, canonical header tree), the
`src/webots/` source folder is renamed to `src/omnisim/`, and the
physics default flips from ODE to Newton. The new Spot walker — straight
walking via a model+residual recipe — is the first piece of locomotion
that runs end-to-end on the Newton path. A render-backend abstraction
lands as the seam for the upcoming Vulkan migration.

### Breaking changes

- **Source-tree rename: `src/webots/` → `src/omnisim/`.** External code
  that builds against OmniSim sources (private forks, extension modules)
  must update include paths. `omnisim/*.h` and `omnisim/*.hpp` are
  canonical; `webots/*.h` shims remain for one release window only.
- **`WEBOTS_HOME` env var is no longer read.** Set `OMNISIM_HOME` instead.
- **`webots://` URL scheme is no longer accepted.** Use `omnisim://`.
- **`webots-bin.exe` binary alias is gone.** The shipped binary is
  `omnisim-bin.exe` (and the controller equivalent). Anything launching
  `webots-bin.exe` by name will fail.
- **`webots.yaml` project manifest is no longer accepted.** Rename to
  `omnisim.yaml`.
- **Default physics solver flips from ODE to Newton.** `physicsBackend
  "auto"` now resolves to Newton, and new empty worlds nudge Newton for
  freshly imported robots. ODE remains available as the documented
  legacy fallback by setting `physicsBackend "ode"` explicitly on the
  Solid (or via the world template). Worlds that depend on ODE-specific
  contact behaviour may need that flag added.
- **`rename_audit` ceilings lowered** — the policy now enforces
  omnisim-only naming across the surfaces it covers; legacy occurrences
  that previously passed will fail audit.

### Engine architecture (the migration plan reaches a milestone)

- **WbPhysicsBackend dispatcher — P1.5 milestone:** WbSolid is fully
  migrated off direct ODE body ops. Position, quaternion, velocity,
  point-velocity, force, torque, body enable/disable, and
  setGeomAndBodyPositions all route through the backend dispatcher.
- WbGyro, WbAccelerometer, and WbGps dispatch via
  `WbSolid::bodyHandle()` — sensors no longer reach into ODE directly.
- WbNewtonBackend implements the dispatcher's pose-read methods
  (`getBodyPointVel`, position, quaternion) for the body-ops surface.
- `WbConnector::rotateBodies` and `WbSolidMerger::setGeomAndBodyPositions`
  converted; force/torque/velocity application widened to the dispatcher.
- `wb_supervisor_node_add_force[_with_offset]` and
  `wb_supervisor_node_add_torque` are now polymorphic across backends.
- Engine-migration plan tracks P1.5 as COMPLETE; cuda-newton-physics
  plan tracks P7 PARTIAL and P9 COMPLETE.

### Rendering (Vulkan migration seam)

- **R0** — `WbRenderBackend` abstraction lands as the unified seam
- **R1** — `OMNISIM_WITH_VULKAN` build flag + `WbVulkanBackend` extracted
- **R2** — `renderBackend` SFString field on Viewpoint + Camera
- R3 — rendering-backend evaluation + design doc written up
- chassis visual now follows physics (WREN-push fix) + verify-as-shown
  tooling

### Newton physics

- **Real fix for the joint-glitch** — clean-build flag + widened URDF
  deploy + analyzer
- Hard post-step joint-limit clamp + stress test
- `armature`, `limit_ke`, `limit_kd` env vars for joint physics tuning
- `getVelocity()` root-cause fix — was returning 0, broke deploy
- NaN-safe ODE pose writeback in `WbSolid::applyPhysicsTransform`
- `OMNISIM_NEWTON_MJWARP` to deploy via mujoco_warp (vs CPU mj_step)
- broadphase "auto" mode now resolves via top-level Solid AABB
- `worldinfo.broadphase` is a real SFString field; ODE switching plumbed
  end-to-end
- `gatherSleepingStats` — sleeping-island verification telemetry
- WREN instancing-candidate run detector (item 1 of large-world plan)

### Reinforcement learning (the residual recipe)

- **First Spot walker that walks STRAIGHT under Newton.** Method:
  model+residual recipe (analytic CPG prior + learned residual
  correction). Shipped as the canonical `spot_residual_main`.
- Model-based Spot walker (analytic IK + trot gait) — beats v12_200k
  PPO with zero neurons; ships as Phase 4 balance PD + heading-lock
- Custom residual-RL system on the model walker (Phase 5)
- GPU residual deploy controller + docs for the full Spot-walk journey
- GPU mjwarp residual trainer on RTX 5070 with backwards-walking fixes
- Recovery agent: leg-based self-righting on fall (no supervisor
  teleport), model-based righting (orientation-aware geometric), realistic
  motor limits (torque cap + rate-limited targets), reward redesign +
  curriculum, training pipeline scaffold
- Push-recovery perturbation experiment + Newton joint-limit diagnostic
- Action-magnitude penalty (`--act-pen`) for smoother gait
- Live MuJoCo viewer for the GPU residual Spot walker
- Spot URDF: `<rest>` tag extension; widen `hip_x` and `hip_y` to ±1.5
  rad for self-righting
- Heading-lock control + verify harness + reward-shaping knobs
- Extended obs vector to 50 dims (added heading-deviation);
  `SPOT_OBS_DIM` env override
- Deploy fixes: clear finite-diff vel history on loop-reset; faithful
  Spot walker walks 0.45 m/s forward in OmniSim

### Demos & worlds

- **BattleBox** — BattleBots-style combat sport scene
- **robot_combat** — new project for all combat demos; 10 worlds + 2
  controllers moved
- 20-husky top-down arena variant + double-the-huskies marketing scene
- husky_fleet_arena top-down capture variant + shotlists
- husky_maze: cell-level loop detection + min-pivot path planning,
  perception-as-tool (hide `read_camera`, scan_surroundings only),
  shake-free + smarter wedge-escape (basic maze now reliably solves),
  wheels-only navigation (no teleport recovery), agent-written memories
  from 2026-05-24 successful runs, refuses `complete_mission` while a
  bridge fault is live, restored `goto_cell` recovery
- Arm assembly line: real physics pick-and-place + grip-confirm sensors
- Arm bridge: working physics pick-and-place — 6-DOF IK + real grasp
- Spot demos: hide-on-start sun marker + higher initial camera
- walk_demo: restored default dark PBR ground (dropped tile pattern)
- fleet_cam: headless Camera-device recorder for husky fleet arena
- Saved default view perspectives for Spot Newton worlds

### Authoring & harness

- WbProject resolves project root for worlds nested in subdirectories
- Smoke gate auto-builds missing controller binaries before push
- Scripts self-locate the repo root in dev build scripts
- `broadphase_auto.omniworld` smoke world

### Branding & social

- **Brand book**: particle orb, OMNI/SIM wordmark, mimosa palette
- Replaced Webots-branded textures with OmniSim equivalents
- Omnivoice — voiceover tool for the youtube_videos scripts
- Husky combat video pipeline + house video style guide
- Tier 1 video polish — LUFS master, captions, tracking camera
- Tier 3 — Claude critic, smoke gate, b-roll, Discord poster
- Multi-angle combat assembler + 2 new b-roll entries
- Original synthesized soundtrack for the topdown video
- ODE "spot walks straight" journey video + ODE walk demo world
- "Teaching a robot dog to walk" journey video — before/after Spot gait
- husky_maze journey video — 5 top-down shotlists + builder
- Recovery-prompt terminal scene + typing SFX mux
- FUNDING.yml points at the omnilink-tech org; omnilink-agents.com
  custom link added

### Build / packaging

- Full release-build unblockers + URDF path fix
- Rename migration script: `scripts/.../rename_webots_to_omnisim.sh`
- Cleanup: prune empty howto/tutorials samples; rename `protogen_demo`
  → `protogen`
- Move retired/experimental worlds to `samples/_archive/`
- `gitignore`: `.wbproj` globally, `MUJOCO_LOG.TXT`, leftover scratch
- Removed orphan `src/webots/Makefile` left after Phase G

### Documentation

- User-facing Newton physics backend guide
- Migration perf comparison with measured Newton vs ODE numbers
- Newton XPBD scaling sweep (1–50 huskies) — benchmark + writeup
- Engine-migration plan kept in sync as P1.5 closed; item 6 closed
- cuda-newton-physics-plan — phase status block
- Newton-as-default + `src/webots` → `src/omnisim` rename plan doc;
  Phase J documents the rename as historic
- Spot residual-RL writeup: old PPO method vs new model+residual
- Physics pick-and-place + grasp-mode docs
- Scrubbed Java/MATLAB code blocks + language-list mentions
- Dropped dead-language pages + fixed stale Cyberbotics path strings
- Archived `REBRAND_PLAN.md` (all phases shipped); deleted
  `MIGRATION_PLAN.md`; references repointed at AGENTS.md §0

### Fixed

- Newton `getVelocity()` returning 0 (root cause of broken deploy)
- Newton joint-glitch — clean-build flag + URDF deploy widening
- NaN ODE pose writeback in `WbSolid::applyPhysicsTransform`
- WREN visual lagging chassis physics
- husky_maze: shake-free + smarter wedge-escape; reliable basic-maze
  solve
- GPU residual trainer: backwards-walking fix on mjwarp
- GPU residual deploy: cleared finite-diff vel history on loop-reset
  (no spike)
- rendering-normals smoke skip root cause documented

### Removed

- `webots://` URL scheme (use `omnisim://`)
- `webots-bin.exe` binary alias (use `omnisim-bin.exe`)
- `webots.yaml` project manifest alias (use `omnisim.yaml`)
- `WEBOTS_HOME` env-var read (set `OMNISIM_HOME`)
- MATLAB engine support — all docs/code-blocks cleaned up
- `REBRAND_PLAN.md` (archived after all phases shipped)
- `MIGRATION_PLAN.md` (all phases shipped)
- Orphan `src/webots/Makefile` left after Phase G
- Empty howto/tutorials samples

---

## [v2.2.0] — 2026-05-21

The physics-and-learning release. Newton finally holds Spot upright and
walking — a chain of solver fixes (qd-indexed joint targets, inherited-Solid
overwrites, shape-center offsets, collision filtering) turned the "parts
falling off" and "no-stand" bugs into a robot that stands, resets cleanly,
and trains. On top of that lands a GPU-batched MuJoCo-Warp PPO trainer
(~125k env-steps/s that actually learns), first-class pluggable gripper
support on arm robots, and a worlds reorganization into starter / showcase /
environments with three new cinematic environments (forest, desert ruins,
high-rise construction site). No breaking changes; several sample demos
were retired.

### CUDA / physics (Newton)

- **Spot STANDS** — collision filter + shape centers + position spring +
  live feedback. The milestone fix after a long bisect.
- Fixed the no-stand bug: `joint_target_pos` is qd-indexed, not q-indexed
- Fixed "parts falling off": Newton overwrite was skipping inherited
  Solids; scene-tree joint angle now read from Newton
- Fixed actuator position-target writes silently no-op'ing
- Realistic per-body inertia + per-joint effort/velocity/position limits
- Pass Pose translation into shape offsets so feet collide where rendered
- Pass URDF joint limits (minStop/maxStop) into the articulation
- Translate URDF mesh collisions to Newton AABB boxes (wrapper opt-in)
- Seed `joint_q` to the standing pose + rebuild solver; pose-seed reaches
  MuJoCo
- Supervisor-driven resets push pose into Newton's `body_q`; reset clears
  the spawn-pose freeze loop
- Settle Spot actuator on ke=200 / kd=5 (best stability + leg mobility)
- `setValueFromOde` overwrite now invalidates the matrix cache
- `OMNISIM_NEWTON_GROUND_MU` foot-friction knob
- Fixed a GPU-array leak that crashed training at ~200k steps
- XPBD-on-GPU by default for training (3× throughput); MuJoCo CPU fallback
  remains for NaN-prone scenes
- Per-call / per-joint / per-step diagnostics added during the bisect

### Reinforcement learning

- GPU-batched MuJoCo-Warp PPO trainer — ~125k env-steps/s and learns
  (PoC peaked at 813k env-steps/s); env-tunable solver iterations
  (default 10/8) for throughput
- GPU trainer auto-configures from MJCF + Spot export hook + eval
- Real trot gait + standing-regression diagnostics; legacy sin² gait stays
  default, trot is opt-in
- Stand-first training curriculum (from-scratch launcher seeds standing
  pose + stiff leg gain); value-warmup uses a critic-only optimizer for a
  correct actor freeze
- Warm-start launcher for SB3 `continue_training` under Newton physics
- Session isolation so parallel training agents can't kill each other
- Anti-B-mode reward shaping + forward-distance episode logging
- Knobs: `SPOT_PITCH_TRIM`, `SPOT_ACTION_SCALE`, uprightness weight, kd=60
  damping; fixed structural CPG nose-down
- Fixed train/deploy CPG mismatch and a deploy-eval bug that silently
  tested the wrong policy
- Opt-in per-episode body trace (`OMNISIM_AGENT_TRACE`)

### Grippers & arm bridge

- First-class gripper support on arm robots (plan → ship):
  - Phase 1 — decouple grippers into a pluggable effector layer
  - Phase 2 — richer surface: grasp / release / set_width
  - Phases 3 & 5 — kinematic grasp weld + pick-place demo + docs
  - Phase 4 — pluggable gripper drivers
  - Physics-grasp gripper with real 2F-85 fingers (WIP)
- Arm demo: refined gripper into a proper Robotiq 2F-85 shape, mounted
  visibly, anchored to the real flange node (fix clipping), grasp radius
  widened to 0.16 m for reliable picks

### Demos & worlds

- Worlds reorganized into `starter/`, `showcase/`, `environments/`
- New environments:
  - **forest** — real-mesh bushes + floor litter, hand-placed background
    ring, everything grounded (no hovering)
  - **desert_ruins** — ground + dune system, ancient architecture,
    cinematic golden-hour overhaul
  - **construction site** — realistic high-rise, more buildings + plant,
    real procedural construction vehicles, fully-editable dev world
- Unify lighting across all user-facing worlds via a 3-PROTO recipe
- Damage: driving arena sheds parts from real box impacts; husky_damage_arena
  tears parts off under falling weights; detached parts keep the robot's
  real colour
- Mavic 2 Pro: chat-style demo (chat_aerial); chat + Drone Surveyor
  flagship consolidated onto one world
- warehouse_patrol: 2026-05-21 patrol sweep manifests
- ConstructionFrameBuilding replaces Building + BuildingUnderConstruction

### Robots & PROTOs

- Inline pipe/torus boundingObject math; drop `projects/bounding_objects`
- street_furniture PublicToilet shows OmniSim branding, not Webots

### Authoring & harness

- `--no-window` for true background mode (no taskbar entry); suppress the
  world-loading progress dialog in `--no-window`
- Always inject `mingw64/bin` on the Windows controller PATH
- Allow in-place editing of bundled worlds/projects by default
- Smoke: pre-flight abort when port 1234 is held by a running Webots;
  per-world skip flag; mark rendering-normals broken
- Husky bridge: remove teleport-snapping globally — drive on wheels only

### Build / packaging

- CI: local pre-push smoke hook + WORLDS.md drift refresh
- gitignore the NVIDIA Corporation/ driver crash dir

### Documentation

- Worlds index; flip docs/README.md to `OMNISIM_HOME` canonical
- Spot + Newton session-state captures + spot_newton_v4 results writeup
- Plan for first-class gripper support; drop stale control_showcases doc

### Fixed

- Newton no-stand bug (qd-indexed joint targets)
- Newton "parts falling off" (inherited-Solid overwrite + scene-tree joint
  angle)
- Newton actuator position-target writes silently no-op'ing
- GPU-array leak crashing training at ~200k steps
- train/deploy CPG mismatch; deploy-eval testing the wrong policy
- mounted gripper clipping (anchor to real flange node)
- forest objects hovering above the ground

### Removed

- Demos: arm digital-twin, all_urdf_robots, urdf_ur5e, urdf_epuck,
  urdf_tiago, urdf_showcase, mobile_robots_showcase, omnibot_combat,
  two_omnibots, cube_bot, ur5e_omnilink kinematics, object_gallery,
  husky_fleet_outdoor (Axis repointed at warehouse_logistics)
- `projects/bounding_objects` PROTO directory (math inlined)
- Building + BuildingUnderConstruction PROTOs (use ConstructionFrameBuilding)

---

## [v2.1.0] — 2026-05-18

A consolidation release on top of v2.0.0. The headline themes: atmospheric
sky becomes the default backdrop across every world (with a draggable sun
marker), the Webots→OmniSim rebrand reaches into the engine layer via an
alias-not-rename phase plan (binaries, env vars, URL scheme, project
config all dual-accept the new names), a full reinforcement-learning
pipeline ships for Spot (and scaffolding for Atlas), and the OmniLink
integration picks up voice I/O, short-term memory, per-turn telemetry,
and a starter PyPI package. No breaking changes — every rebranded entry
point keeps its prior alias.

### Engine rebrand (Webots → OmniSim, alias-not-rename)

- REBRAND_PLAN.md staged in [Phase 0](docs/) — inventory + safety net,
  no functional change
- Phase A — cosmetic display strings rebranded
- Phase B — `omnisim-bin.exe` / `omnisim-controller.exe` binary aliases
- Phase D — `omnisim/*.h` and `omnisim/*.hpp` C/C++ header forwarders
- Phase E — `omnisim` Python controller package (forwarder)
- Phase F — `OMNISIM_HOME` canonical; `WEBOTS_HOME` dual-read alias
- Phase G — `webots://` URL scheme also accepts `omnisim://`
- Phase H — `projects/*/webots.yaml` also accepts `omnisim.yaml`
- Phase C (`Wb*` → `Om*` class rename) intentionally skipped to preserve
  the "built on Webots" attribution surface
- Doc + in-app sweep so new agent sessions read OmniSim, not Webots
- Two pre-existing breakages flagged during the verification gate, fixed

### Demos & worlds

- MissionControl: 6-Husky fleet on a logistics campus, agent-only
- HuskySwarm: 4-Husky OmniLink coordinator → upgraded to 34-tool
  meta-tool-pattern coordinator
- Construction-site logistics benchmark — scripted vs agentic control
- Multi-arm demo world (3× UR5e on a shared stage)
- Three specialist OmniLink agents shipped as examples — Foreman, Picker,
  Roomba
- Real-robot bridge starter kit example (no Webots, no OmniSim)
- In-sim demo gallery launcher (world + supervisor + Robot Window)
- Worlds subdivided by category; path-refs refreshed
- spot_newton_demo: switched to atmosphericSky + draggable sun marker
- Sweep removed classic-Webots demos that no longer fit OmniSim (twice —
  a merge restored them and they were re-deleted)
- Cinematic atmospheric-sky playground worlds + headless bench

### Atmospheric sky (the new default)

- Atmospheric sky installed on every world with a Background — full sweep
  across 330 atmospheric worlds (last 24 needed surrogate-escape for
  non-UTF-8 .wbt files)
- Legacy photo-cubemap backgrounds ripped out
- Draggable sun marker — Unreal-style glowing sphere; doesn't cast a
  shadow; bound to DirectionalLight position
- DirectionalLight.color → atmospheric sky `sunIlluminance` binding
- Night sky: procedural starfield, moon with maria, marker overhead,
  Milky Way band, varied stars, stars constrained to upper hemisphere,
  realistic moon
- Cinematic tonemap: S-curve contrast + saturation lift
- Per-world IBL auto re-bake on DirectionalLight edits
- Real Lambertian diffuse-IBL bake for atmospheric sky
- IBL white-balance pushed to 0.95 → 1.0 so PBR materials read true albedo
- Damped IBL bake against atmospheric-sky over-tint
- Perlin terrain + fix for IBL axis-swap bug in sky shaders
- Reverted one engine IBL iteration to restore a working baseline

### Robots & PROTOs

- Removed PR2 support entirely (no live demo consumers, blocked sweeps)
- Spot: end-to-end demo runs (articulation + position-bridge wired)
- Spot URDF inertia diagnosis from the RL deploy path

### CUDA / physics

- Newton: multi-parent articulation bug fixed on Spot tree (leaf-first
  joints)
- Newton MuJoCo-CPU env-var override

### Reinforcement learning (new in v2.1.0)

- End-to-end RL pipeline scaffolding for Spot — env wrapper, trainer,
  deploy controller, eval workflow
- Trained Spot policy: 200k-step PPO checkpoint (320 KB), then 100k
  walk-focused checkpoint
- Spot OmniSim walker — verified +9.79 m forward in ODE, never falls
- CPG trot prior + GPU env
- Atlas (Boston Dynamics v5) RL pipeline + MJX deploy-parity fixes
- Pluggable training backends: sb3 / mjx / isaac
- Platform-ify: generic deploy, robot registry, reward recipes
- Headless MuJoCo deploy
- MJX trainer NaN-defensive fixes
- MJX PPO loss fix — sum log_std along act dim, not full batch
- `kp=500` fix unlocks MJX deploy
- SpotEnv connect timeout bumped 30s → 90s
- Deploy controller path resolution + eval workflow
- Body trace from deploy controller + headless eval script
- env-var reward weights for the walk-focused trainer

### Agents (`agents/`)

- All OmniLink agents consolidated under a single `agents/` tree
- Voice I/O in the chat panel (mic in + agent voice out)
- Short-term memory: cross-session chat continuity
- Per-turn usage telemetry (tokens + credits) on every bridge
- Auto-register per-robot profiles + tool callback endpoint
- Demos depend on omnilink-lib explicitly + warn on outdated versions
- OmniLinkClient + sim-to-real docs
- Agent benchmark suite scaffold (3 tasks)

### Authoring & harness

- Agent-driven cinematic capture pipeline
- Marketing slate driver scripts + 4 shotlists
- Cinematic-pipeline 5 fixes (unblock end-to-end)
- Agent gallery page + demo-video capture script
- `headless_runner` honors `OMNISIM_LOG_PATH` for parallel runs
- URDF sensor-segfault bisect narrowed + device-smoke harness
- Untrack harness scratch worlds + gitignore them

### Build / packaging

- omnisim-bridges PyPI package (skeleton, locally installable)
- Prune vehicles target from `projects/Makefile` + relay Makefile stub
- gitignore runtime artifacts (`fps_sweep` summary, `*.egg-info`)
- Drop stray baseline artifacts from rebrand Phase 0

### GUI / UX

- View > Theme submenu — Light / Dark (Night) / Dark (Dusk)

### Release infrastructure

- `publish_snapshot.sh` auto-bumps `omniSimVersionString` in
  `WbApplicationInfo.cpp` to match the release tag before snapshotting
  (idempotent no-op when already current). Was a hand-maintained constant
  that drifted in both v1.0.10 and v2.0.0.

### Licensing

- TRADEMARKS policy added (open code, protected brand model)
- DCO contributor flow (not CLA)

### Documentation

- REBRAND_PLAN.md staging doc — engine refactor phases
- Top-level DEMOS, WORLDS, ARCHITECTURE, MIGRATION_PLAN indexes
- OmniLink integration roadmap — 13 items across 5 phases
- omnilink-roadmap.md annotated with status + commit hashes
- README + AGENTS.md + chat-demos surface every OmniLink artefact
- RL pipeline: README, smoke_deploy, AGENTS.md pointer, eval_policy
  usage + plateau notes
- RL handoff doc — resume-here section with commit SHA, GPU-box handoff

### Fixed

- Newton: 'multi-parent' articulation on Spot tree (leaf-first joints)
- MJX: PPO loss log_std summed along act dim, not full batch
- engine IBL: damp bake against atmospheric-sky over-tint
- sky shaders: IBL axis-swap bug
- two pre-existing breakages flagged during the rebrand verification gate

### Removed

- PR2 robot (no live consumers)
- Classic-Webots demos that no longer fit OmniSim
- Legacy photo-cubemap backgrounds (replaced by atmospheric sky)
- Stray baseline artifacts from rebrand Phase 0

---

## [v2.0.0] — 2026-05-14

OmniSim's first major release. The simulator's centre of gravity moves from
PROTO-authored Webots robots to a URDF-native, agent-driven platform: 14
canonical robots now load from URDF, every shipped demo is wired to the
OmniLink agent platform via the new Wire Protocol, the physics layer gains a
pluggable Newton/XPBD backend with CUDA-accelerated particles and rigid-body
contact, and the rendering path picks up an atmospheric sky stack with
real-world perf gains on the existing forest, mars, and warehouse scenes.

### Breaking changes

- Removed Pico TTS, Java, and MATLAB controller support. Python and C/C++
  remain. Worlds, samples, and build scripts that referenced these are gone.
- Removed the integrated text editor (-3,190 LOC). External editors only.
- Removed the Classic light theme. OmniSim is dark-only.
- Removed i18n machinery. UI strings are English-only for now.
- Removed Cyberbotics menu items and the legacy Webots.cloud upload flow.
- Removed ~30 legacy robots that had no canonical URDF and no live world
  consumers: e-puck, elisa, mir100, hoap2, k-team kheperas/hemisson/koala,
  pioneer2/3, p-rob3, thymio, boebot, fabtino, nao, aibo, qrio, crazyflie,
  shrimp, biorob, robotnik (summit_xl_steel), irb, scara_t6, youbot, ipr,
  ned, tinkerbots, kondo khrs, mantis, scout, tiago family, saeon, sphero
  bb8, surveyor, puma, sojourner, bioloiddog, firebird6, heron USV, atlas,
  jetbot, darwin-op and its dependents (robocup, humanoid_marathon,
  supervisor_set_position_loop test), `projects/humans/` tree.
- Prefs migration: `webots_*.qss` theme keys are read as `omnisim_*.qss`.
  Manually-edited theme overrides need re-saving once.

### Demos & worlds

- 14 per-URDF-robot OmniLink chat demos, each wired end-to-end to the
  OmniLink agent platform (g1-engine default, local fallback)
- OmniLinkStage PROTO adopted across all 14 omnilink_* worlds
- warehouse_foreman: iterations 0-4 — Picker, Loader, vision-driven tag ID,
  orchestrator runs end-to-end, side-detour push fix, ground-truth pallet
  delivery verified
- warehouse_patrol: iterations 0-3 — two-husky patrol squad, bridge port
  multiplexing
- drone_surveyor: iterations 0-3, verified end-to-end
- tour_guide: TIAGo four-room apartment, verified end-to-end
- Arm assembly line: three-arm cobot demo with Robotiq 3F-styled,
  parallel-jaw, and realistic hinged-finger grippers; ARM_BRIDGE_HOST/PORT
  env override; iter-1 vision + NL goals + recovery, end-to-end LLM verified
- Arm digital twin: shadow-mode bridge + reference demo
- husky_maze: drift-gate snap-to-cell smoother corridors, fix corners
  replan loop, unknown-lidar thrash, tokens-per-hour metric
- husky_fleet_arena: 10-husky open-arena variant
- husky rough-terrain hill: 840 HD rocks + heading-PID controller
- mars_max stress benchmark: 8 huskies + 160m world; renderer well within
  budget; regenerate with hills and valleys
- Husky combat: husky_hunt AI, 2-husky head-on demo, 4v4 head-on collision,
  match runner
- two-OmniBot combat arena demo + random walk demo
- CUDA M2: boundsHalfWidth PROTO field, Husky-meets-spheres demo, 10k
  broadphase showcase, two-way robot/particle coupling
- Cinematic capture service: agent-facing render service, deterministic
  stepping, /shutdown endpoint, drone shotlists, 4K static top-down shotlist
  for the husky fleet arena
- Canonical top-down Viewpoint across all 37 worlds

### Robots & PROTOs

- Native URDF support: WbUrdfImporter + URDFRobot world node, inertia +
  sensors gated, fallback colors, composite bounding, smarter joint limits
- PROTO → URDF migrations: Boston Dynamics Spot, Franka Panda, Mavic 2 Pro
  (with custom rotor physics), PR2, Husarion Rosbot + Rosbot XL,
  UR3e/UR5e/UR10e, TurtleBot3 Burger/Waffle/Waffle Pi
- Agent-first PROTO tooling: schemas, validation, authoring, hot-reload,
  tests
- Devices: replace PROTO catalog with xacro macros
- Spot URDF: walk progression — exploding-spawn fix → stable stance →
  supervisor-driven wave gait → IK foot-trajectory walk → closed-loop
  Raibert balance → pure-physics wide-stance walk → statically-stable
  crawl with CoM shift + roll feedback → ported CHAMP control algorithms
  to Python controller
- UR5e: IK keyboard teleop + OmniLink bridge supervisor

### CUDA / physics

- Newton physics backend (additive, ODE remains default): WbPhysicsBackend
  abstraction (P0) → concrete WbOdeBackend + WbNewtonBackend + registry
  (P1) → embed CPython, import warp + newton (P2) → FFI smoke at
  newton.ModelBuilder() (P3.0) → per-world simulation surface (P3.1) →
  smoke sphere registers and steps live (P3.2) → numerical sphere drop
  + land verify (P3.2.e) → rotation readback (P3.4) → sphere radius
  from boundingObject (P3.5) → WbBox bounding + tumbling-box verify
  (P3.6) → WbHingeJoint → Newton revolute, in-binary verified (P3.7.b)
  → motor FFI surface + controller drives wheel (P3.8) → WbCylinder +
  WbCapsule bounding (P3.9) → physicsBackend inheritance from ancestor
  Solid + URDFRobot pass-through, husky on Newton (P3.10–P3.10f) → 10
  huskies sustain 165 fps (P4) → 8-husky watchable demo + scaling-cliff
  investigation → 4v4 head-on collision demo (P5) → damage demo + perf
  instrumentation (P6) → XPBD as primary solver, MuJoCo CPU as fallback
- CUDA GranularGroup: ENU-aware kernel + bouncing scatter demo (M2),
  one-way Husky pushes spheres → uniform-grid broadphase (100x more
  particles, real-time) → two-way coupling (robot pushes balls AND balls
  push robot), bowling-balls + 10k showcase worlds
- CUDA plans: husky head-on (30+ fps target), CUDA rigid-body solver
  (replace ODE for 10-20 husky scenes), CUDA particle effects, CUDA
  compute infrastructure
- CUDA particles: damage_tracker pool client + numba.cuda particle pool
  prototype

### Agents (`omnilink-agents/`)

- omnilink-agents shared SDK + omnisim-runner launcher
- omnisim Python package + CLI + doctor; omnisim.dev and omnisim.damage
  lifted into the package
- Agent runtime observability layer (snapshots + unified event stream)
- omnisim live agent HUD docked alongside the text editor; user-controllable
  font size; bigger default
- Damage system phases 0-21 + repair: contact detection + impulse via
  supervisor (P1) → HP/state model + chassis-impact detection (P2) → HTTP
  query endpoints (P3) → visual damage markers (P4) → behavioral
  consequences via customData gate (P5) → debris bursts on broken
  transitions (P6) → per-part appearance darkening (P7) → cumulative
  impact decals (P8) → wheel detachment on broken transition (P9) →
  per-state mesh swap (P10, Tier A) → particle effects: smoke, sparks,
  fluid stains (P11) → generic damage trait via DamageProfile (P12) →
  agent SDK (P13) → procedural body deformation (P14a-c) → impact-localized
  mesh deformation (P15) → headless full-realism harness (P16a-e) →
  topology fracture: strain detection → island selection → fragment spawn
  + chassis hole → regression test (P17a-d) → repair mechanics: HP regen
  + mesh regen + heal-to-pristine (P18) → generic part detachment + realism
  polish (P19) → slab attribution + car-crash physics (P20+P21); spawn-drop
  suppression; defensive guards against breaking other harness-loaded worlds
- mission_captain + husky_maze: tokens-per-hour metric + COSTS docs
- Tour-guide, foreman, patrol, surveyor agents (see Demos)

### Authoring & harness

- Headless harness self-detects parallel-session collisions, prints
  agent-actionable guidance
- sim-instances: scope kill to our PIDs and add --auto-port
- scripts: scope webots-bin kills to spawned instances (don't touch the
  user's running Webots)
- omnisim CLI pins WEBOTS_HOME to this clone in webots_env()
- Drone surveyor, warehouse foreman, patrol multi-iteration scaffolds
- Per-instance log file + dynamic controller stdout buffer

### Build / packaging

- OmniSim Wire Protocol v1.0 — canonical PROTOCOL.md
- AGENTS.md as the canonical agent entry point at repo root
- Auto-purge orphan robot/asset dirs after pulls; clean_orphans wipes stale
  .o/.d build outputs
- Prune hollow controller dirs + stub Makefiles for Python controllers
- Cleanup: ~90 MB pruned from projects tree (upstream Webots dead weight)
- Cascade-delete darwin-op dependents + trim test source skip-lists
- glm submodule bumps (0af55cce, bf71a834)

### Release infrastructure

- Auto-create GitHub Release pages on publish + backfill helper for
  v1.0.0–v1.0.7
- Switch GitHub Release POST from curl to python urllib (Schannel TLS
  revocation workaround)
- ASCII-only output for backfill_release_pages.py (Windows cp1252)
- Pass `--root` to git diff-tree so orphan first-release dry-runs show file
  list; capture diff-tree output to temp file so the total-files line
  survives pipefail
- Repoint publish target at github.com/omnilink-tech/omnisim
- CHANGELOG.md + auto-generated release-note flow
- omnilink-reports/ rename (was omnilink-bugs/) and add OPERATIONS report
- SECURITY.md responsible-disclosure policy
- youtube_videos/ private marketing-scripts folder (deny-listed)
- ci: disable inherited Webots workflows until validated against OmniSim tree

### Documentation

- AGENTS.md: agent-first positioning, multi-instance support documented
- Beginner guide for the 14 OmniLink chat demos
- OmniSim Wire Protocol v1.0 reference (PROTOCOL.md)
- Long-range plans: Unreal-fidelity rendering, procedural world generation,
  CUDA Newton physics backend, CUDA husky head-on, CUDA rigid-body solver,
  CUDA particle effects, large-world optimization, granular physics
- guide/ + reference/ rebranded to OmniSim; archived inherited Webots
  changelog history (R2020–R2025) under upstream-webots-history/
- Damage system plans (phases 10-18) — mesh deform, particles, generic,
  SDK, procedural deformation, impact-localized deformation, headless
  full-realism dev, topology fracture, repair mechanics
- README: fresh-user onboarding (prereqs, submodule clone, verify, agentic
  setup); README section on capturing videos & screenshots; headless runner
  surfaced in README + quickstart
- URDF: import sensor-gate crash bisect + Python PATH guidance; record
  sensor-gate crash as upstream Webots bug
- benchmarks: outdoor_forest baseline + revised architecture finding;
  peak-RSS sampling per run; --repeats/median + chunky scenario
- OmniLink platform reference docs
- omnilink-agents DEMOS roadmap — 4 next demo agents, ranked + scoped

### Rendering

- Modern renderer backend via OMNISIM_RENDERER env selector
- WbRenderBackend seam, WREN scene iteration + mesh accessor C API
- OmniRender ForwardBackend scaffold; shared WREN viewpoint camera
- Atmospheric sky: Hillaire 2020 multi-scattering LUT, end-to-end procedural
  sky live on mars, procedural PBR irradiance + cubemap-free worlds, HDR
  cubemap for mars, per-pixel sky_apply matching preview HTML byte-for-byte,
  ENU→Y-up axis swap fix
- AgX tonemap, GPU-side auto-exposure, output dither, temporal smoothing
  (all under OMNISIM_RENDERER=modern)
- Mars perf: sun stencil shadows off by default (-51% forward GPU on
  mars_big), rock subdiv 3→2 (-38% mars_big / -46% mars), rock-template
  cache dedup, per-instance displaced mesh geometry
- Cross-world: shadows-off default (-49% on forest), bloom + GTAO defaults
  off (mars_big render 5.0 → 1.3 ms, -74%)
- T2 instrumentation: GPU timestamps for main scene, CPU companion timer
  + FullRenderNow, WREN-internal forward/post-process GPU breakdown,
  per-frame triangle counter, per-geometry draw histogram, forward-pass
  sub-bucket breakdown, aggregate per-viewport scene-render timing

### OmniWorld procedural worlds

- Scaffold + heightmap primitives (T1.1, T1.2)
- Scatter primitives (T1.3); nested scatter + surface manifests (T1.11)
- Layout DSL, solver, JSON schema (T1.4)
- Asset catalog (T1.5)
- Headless-simulator validator (T1.7)
- Biome cookbook (T1.8)
- Per-instance variation + weathering (T1.10)
- Biomes: outdoor_forest (first real biome), outdoor_desert, urban_block,
  warehouse, indoor_apartment, mars (with atmosphere overrides)

### GUI / UX

- WASD free-fly camera + FPS mouselook + numpad view snaps
- Splash screen: orb-centered OmniSim composition, drop Webots robot
  screenshots
- Live agent HUD (WbAgentHud) docked alongside text editor
- Web viewer rebranded to OmniSim, dropped dead Cyberbotics image links
- Streaming viewer vendors wrenjs/enum.js, serves wwi/ tree locally
- Auto-update notifier repointed to GitHub Releases for OmniSim
- mWebotsLogo + webots_icon.png renamed to OmniSim equivalents
- Branding polish: startup-update bug fixed, welcome/updated/About dialogs
  finished
- Docs URLs route Help-menu and node-help to GitHub-hosted OmniSim docs;
  load icons.svg + viewer.js locally instead of from cyberbotics.com
- CLI bug-report URL rewritten to OmniSim issues; --log-performance
  rebranded; metainfo URL typo fixed; Apache modification line; snap-MATLAB
  string cleaned
- Copyright backfill: OmniLink modification line on Cyberbotics-derived
  edits

### Fixed

- URDF importer: silence inertia matrix warning, fix TB3 motion bug,
  inertia gate, sensor emission restructure, TurtleBot3 waffle DAE crash,
  sensor gate progresses past world-load crash via carrier-Solid +
  replace-in-place
- spot.omniworld: world description matches statically-stable wave-gait walk
- husky_random controller: strip broken sensor-based stuck-escape
- Newton: project joint anchors through parent/child rotations
- damage_system: align detached wheel mesh axis with collision cylinder;
  detached wheel uses real Husky wheel.dae mesh; IFS field + env-var
  disable hooks for stability
- Fix rotation pipeline: initialize Wren quats to identity
- prefs migration handles legacy webots_*.qss theme key

### Removed

- Pico TTS, Java, and MATLAB controller support (controllers, build
  scripts, samples, launcher strings)
- Integrated text editor (-3,190 LOC)
- Classic light theme (dark-only)
- i18n machinery
- Cyberbotics menu items + Webots.cloud upload feature
- Lua/Java/MATLAB overclaims in branding
- ~30 legacy robots without canonical URDFs (see Breaking changes above)
- `projects/humans/` tree (no URDF equivalents)
- Sample trees whose target robot was removed
- Mixed-dir dead worlds + scaffolding; residual refs to deleted
  robots/humans/worlds

---

## [v1.0.10] — 2026-05-08

### Demos & worlds

- cuda_particles + multi_robot_damage: P6 closed (fps gap remains)
- Arm assembly line: Robotiq 3F-styled grippers + TCP rotation tracking

### Robots & PROTOs

- damage_system: phase 19 — generic part detachment + realism polish
- damage_system: phases 20+21 -- slab attribution + car-crash physics

### Agents (`omnilink-agents/`)

- damage_system: phase 13 — agent SDK + drop accidental scratch PNGs
- Arm assembly line: parallel-jaw grippers + truststore patch + /reload
- Arm assembly line: realistic hinged-finger grippers, single-Robot host
- docs: rebrand pass — omnilink-agents + video script Webots → OmniSim

### Build / packaging

- omnisim.dev: lift dev CLI into the package, shim the script

### Release infrastructure

- release: record v1.0.9 published private SHA

### Documentation

- damage_system: plan v2 — Phases 10-13 (mesh deform, particles, generic, SDK)
- damage_system: plan — phase 14 (procedural body deformation)
- damage_system: plan — phase 15 (impact-localized mesh deformation)
- large_world: plan — CPU-side instancing, async load, scene streaming, broadphase audit
- damage_system: plan — phase 16 (headless full-realism development)
- damage_system: plan — phase 17 (topology fracture)
- damage_system: plan — phase 18 (repair mechanics)
- observability: agent runtime observability layer (snapshots + unified event stream)
- agents_md: document multi-instance support for fresh agent sessions
- omnilink-agents: shared SDK + omnisim-runner launcher
- cuda_particles: P1 -- bench + plan justifying GPU particle field
- cuda_particles: P1.5 -- python+numba.cuda particle pool prototype
- cuda_particles: P6 -- damage_tracker pool client (followup needed)
- Arm digital twin: shadow-mode bridge + reference demo
- deeper rebrand: GUI strings, CLI --help, AGENTS.md, dev plans
- rebrand pass: OPERATIONS.md + CHANGELOG + world-comments + matlab launcher
- harness: self-detect parallel-session collisions, print agent-actionable guidance
- fps: lite-damage env hook + pool optimisation + journey doc
- plan: cuda husky head-on -- 30+ fps target via binary-level work
- plan: cuda rigid-body solver -- replace ODE for 10-20 husky scenes
- plan: nvidia newton physics backend -- additive, can't break simulator

### Removed

- remove i18n machinery
- remove dead Cyberbotics menu items + Webots.cloud upload feature

---

## [v1.0.9] — 2026-05-04

### Demos & worlds

- damage_system: phase 1 — contact detection + impulse via supervisor

### Agents (`omnilink-agents/`)

- Arm assembly line: iter-1 — vision + NL goals + recovery, end-to-end LLM verified

### Release infrastructure

- release: record v1.0.8 published private SHA

### Documentation

- damage_system: phase 0 — arena world + box-dropper supervisor
- damage_system: phase 2 — HP/state model + chassis-impact detection
- damage_system: phase 3 — HTTP endpoints for damage queries
- damage_system: phase 4 — visual damage markers on state transitions
- damage_system: phase 5 — behavioral consequences via customData gate
- damage_system: phase 6 — debris bursts on broken transitions
- qa cleanup: rename OLink-agents → omnilink-agents in AGENTS.md, fix Linux build, document Linux setup

---

## [v1.0.8] — 2026-05-03

### Release infrastructure

- release: record v1.0.7 published private SHA
- release: switch GitHub Release POST from curl to python urllib

---

## [v1.0.7] — 2026-05-03

### Release infrastructure

- release: record v1.0.6 published private SHA
- release: ascii-only output for backfill_release_pages.py

### Documentation

- release: auto-create GitHub Release pages on publish + backfill helper

---

## [v1.0.6] — 2026-05-03

### Release infrastructure

- release: record v1.0.5 published private SHA

### Documentation

- capture: agent-facing cinematic render service — sister to the validation harness
- capture: playback_speed knob — high-level alternative to settle_steps_per_frame
- perf: per-instance log file + dynamic controller stdout buffer
- bench: cleanup + --repeats/median + chunky scenario; record item-4 second attempt
- Arm assembly line: three-arm cobot demo, iter-0 end-to-end verified
- docs: README section on capturing videos & screenshots
- bench: peak-RSS sampling per run; size item 3 + write session summary

---

## [v1.0.5] — 2026-05-02

### Agents (`omnilink-agents/`)

- warehouse_foreman: hint-first picker prompt — 56% faster, 86% cheaper
- warehouse_foreman docs: Cloud Run, not Vercel
- warehouse_foreman: perception-as-tool — $1.50/hr verified end-to-end
- warehouse_foreman: ship documentation — ARCHITECTURE.md + AGENT_PATTERNS.md
- warehouse_patrol: iteration 0 — world + folder scaffold
- warehouse_patrol: iterations 2 + 3 — Patrol Squad shipped end-to-end
- drone_surveyor: ship iter 0-3 + verify end-to-end
- local_memory: propagate _write_file mkdir fix to 5 sibling agents
- tour_guide: iteration 0 — TIAGo four-room apartment tour, end-to-end verified

### Authoring & harness

- warehouse_patrol: iteration 1 — bridge port multiplexing for two-husky worlds

### Release infrastructure

- release: record v1.0.4 published private SHA

---

## [v1.0.4] — 2026-05-02

### Highlights

- **A 6-DoF cobot arm.** First cobot-class arm in the stock URDF library, complete with per-link visual + collision meshes, a sample world, and a wave demo controller that exercises all six joints.
- **CUDA broadphase.** `GranularGroup`'s O(N²) brute-force collision test is replaced by a uniform-grid linked-list broadphase. Real-time particle ceiling moves from ~2 000 to well past 100 000 — the new `tests/cuda/warehouse_husky_granular_massive.omniworld` showcase runs the larger budget end-to-end on commodity NVIDIA hardware.
- **Warehouse-logistics demo + agents.** A new `warehouse_logistics.wbt` scene driven by two co-located OmniLink agents: `warehouse_foreman` (supervisor scaffolding) and `warehouse_picker` (first end-to-end picking agent — profile, knowledge, memory, picker tool, chat-driven runner).

### Demos & worlds

- `projects/samples/demos/worlds/flagship/warehouse_logistics.wbt` — pallet / forklift logistics world driven by `warehouse_picker`.
- `tests/cuda/warehouse_husky_granular_massive.omniworld` — 10 k-sphere CUDA broadphase showcase world.
- `tests/cuda/launch_warehouse_granular.bat` and `launch_warehouse_granular_massive.bat` — Windows launchers for the granular CUDA demos.

### Robots & PROTOs

- A 6-DOF cobot arm package: URDF, per-link visual (`.glb` / `.stl`) and convex-hull collision (`.obj`) meshes, a wave demo controller, a sample world, and a `webots.yaml` registry entry.
- `URDFRobot.staticBase` — new field on the URDF importer. When `TRUE`, the emitted `Robot` has its root `Physics` block stripped so OmniSim treats the base as a kinematic root. This is the bolted-to-the-floor semantics that arms like the UR5e and Panda need to keep their base from skating around under joint torque.

### CUDA / physics

- `WbGranularGroup` uniform-grid linked-list broadphase. Cell size is `2 · radius` so any contacting pair of centres lands in the same or an adjacent cell; the build pass uses `atomicExchange` chains so each cell ends up pointing at a chain of every particle inside it; the force pass walks the 27 neighbour cells per particle. Grid is reused across substeps.
- Refreshed `tests/cuda/bench_results.md` with the post-broadphase numbers.

### Agents (`omnilink-agents/`)

- `warehouse_foreman/` — supervisor agent scaffolding.
- `warehouse_picker/` — full picking agent: `profile.json`, README, knowledge tool, local-memory + recall tools, `picker` action tool, `scripts/chat_drive.py` runner, agent entry point.
- `DEMOS.md` — roadmap for the next four demo agents, ranked and scoped.

### Authoring & harness

- `husky_omnilink_bridge`: new `drive_to_waypoint` action exposed to the bridge protocol — the foundation the warehouse picker drives the Husky on top of.
- `husky_omnilink_bridge`: drift-gated cell-snap. The husky now snaps to the destination cell only when residual drift exceeds 0.40 m, so clean forward drives no longer pop visually after every cell while post-pivot drift still corrects. The threshold sits well under the 0.5 m wall-clearance buffer.

### Fixed

- `husky_maze`: corners-mode replan loop and unknown-world lidar thrash.

---

## [v1.0.3] — 2026-05-01

### Documentation

- README hero replaces the inline `<video>` embed with an animated GIF preview of the CUDA `GranularGroup` showcase. The full three-minute MP4 stays linked underneath. GitHub's README rewriter strips relative-source `<video>` tags, which made the original embed render as a broken icon on the public landing page.

---

## [v1.0.2] — 2026-05-01

### Highlights

- Three-minute CUDA `GranularGroup` showcase video added to the README hero — `docs/media/videos/cuda_showcase.mp4`.

### Authoring & harness

- `tests/cuda/.harness_granular_group_load.wbt` — minimal hot-reload test world for the granular-group load path through the validation harness.

### Documentation

- README polish around the CUDA hero block.

---

## [v1.0.1] — 2026-05-01

### Highlights

- README rewritten around an OmniSim demo screenshot gallery: warehouse Husky, industrial warehouse, the five Husky maze variants (`husky_maze`, `_unknown`, `_corners`, `_visual`, `_blind`), and the CUDA `GranularGroup` demo — all captured headlessly via the validation harness.
- `GranularGroup` PROTO fully wired into the scene tree: `resources/nodes/GranularGroup.wrl` declaration plus `WbGranularGroup.cpp/.hpp` machinery.

### Build / packaging

- Inherited Webots CI workflows moved to `.github/workflows.disabled/` until validated against the OmniSim tree. The release build job stays active on the public side.

---

## [v1.0.0] — 2026-05-01

First public release of OmniSim. OmniSim is a fork of
[Webots](https://github.com/cyberbotics/webots) repositioned around
agent-driven robotics simulation, distributed by OmniLink under the
Apache License 2.0.

### Highlights

- **Agent-first product surface.** [`AGENTS.md`](AGENTS.md) at the repo root is the canonical entry point for AI coding agents (Claude Code, Codex, Cursor) following the [agents.md open standard](https://agents.md/). It hands a fresh-clone agent everything it needs: build, launch, demo selection, headless-validation contract, HTTP-bridge driving, world iteration via the harness, and validation lanes.
- **Validation harness for agent-driven authoring.** Long-running HTTP service at [`scripts/harness/omnisim_harness.py`](scripts/harness/omnisim_harness.py) wraps a headless simulator subprocess and exposes endpoints for loading `.wbt` files, hot-reloading (~600 ms), screenshots, scene-tree inspection, viewpoint aiming (`/scene/look_at`), exposure stats (`/world/render_stats`), and stepping. Load failures come back as structured diagnostic codes (`PROTO_NOT_FOUND`, `WORLD_PARSE_SYNTAX_ERROR`, `TEXTURE_READ_FAILED`, …) so callers branch on codes rather than regex-matching stderr.
- **`omniworld` procedural world generator.** Recipes: `flat_ground`, `outdoor_forest`, `outdoor_desert`, `warehouse`, `urban_block`, `indoor_apartment`, `mars`. Same `(recipe, seed, params)` always produces a byte-identical `.wbt`. Backed by a Layout DSL + solver + JSON schema, asset catalog, scatter primitives, heightmap primitives, nested scatter + surface manifests, per-instance variation + weathering, and a headless validator.
- **Native URDF support.** New `WbUrdfImporter` plus a `URDFRobot` world-file syntax: drop a URDF + meshes into `projects/robots/<name>/`, reference it from a `.wbt`, and the simulator imports it on world load. Includes mesh loader (STL / Collada / glb), fallback colors, composite bounding, smarter joint limits, motor effort/velocity emission, position sensors, and supervisor flags. Stock URDF library covers UR5e, E-puck, Tiago, Husky, OmniBot.
- **HTTP bridges for runtime agents.** Sample bridges (`ur5e_omnilink_bridge`, `husky_omnilink_bridge`) expose robot control over a local HTTP API so an OmniLink agent can drive a robot model without writing controller code. Reference UR5e bridge listens on `127.0.0.1:6060` with `/state`, `/capabilities`, and `/action` (`set_joint_positions`, `set_tcp_target`, `solve_ik`, `reset_home`, …).
- **`omnilink-agents/` co-located agents.** Agent definitions versioned alongside the worlds and controllers they drive — same productized layout as OmniLink's first-party agents (`profile.json`, `prompts/`, `knowledge/`, `long_term_memory/`, auto-discovered `tools/`, runner). Reference agents at v1.0.0:
  - `husky_maze/` drives the Clearpath Husky across four maze worlds with progressively harder briefs (`husky_maze.omniworld` trivial, `_unknown.wbt` lidar wall-follow, `_corners.wbt` mission-brief, `_visual.wbt` camera-only). Episodic memory of visited / unvisited cells, structured ops view, long-term memory for cross-session compounding.
  - `mission_captain/` provides cross-agent composability via local delegation; live-verified after credentials refresh, with resilience patches.
- **CUDA acceleration layer.** Two-tier additive infrastructure:
  - **CUDA M0** — context, buffer, and dispatch primitives gated behind `OMNISIM_CUDA_SMOKE=1`.
  - **CUDA M2** — `GranularGroup` PROTO with brute-force collision response (~320× CPU speedup at N=400 on the bench), real gravity-integration kernel via NVRTC + CUDA Driver API, Coulomb-capped tangential friction, ENU-aware kernel, WREN host-readback rendering so particles are visible end-to-end, two-way coupling between robots and granular media, `boundsHalfWidth` PROTO field.
- **Modern renderer baseline.** `WbRenderBackend` seam selectable via `OMNISIM_RENDERER`. Modern backend ships pass-through forward + post-process injection, AgX tonemap, GPU-side auto-exposure with temporal smoothing, output dither — all gated to opt-in until A/B-verified-better. Default backend stays WREN to preserve byte-identical visuals.
- **Realism + performance pass on outdoor worlds.** Per-instance displaced rocks, per-instance variation + weathering, log-uniform rock size distribution. Mars biome: fog, craters, ground-fit, multi-layer scatter, Husky fleet option, drivable-crater geometry, drop-height fix, motor-stall escape. `mars_big`: forward GPU 5.0 → 1.3 ms (-74%) via bloom / GTAO defaults off; rock subdivision 3 → 2 (-38%); shadows-off cross-world (-49% on forest, -51% on `mars_big`).
- **Branding.** OmniLink dot-sphere orb established as the canonical OmniSim mark. Splash screen, About box, GUI icons, color palette (black / cream / mimosa) all driven from a single source-of-truth tree under [`resources/branding/omnilink/`](resources/branding/omnilink/). Cyberbotics telemetry pings stripped; share-to-cloud dialog neutered; CLI bug-report URL points at the OmniSim issue tracker; `--log-performance` rebrand.

### Demos & worlds

Ships ~37 demo worlds with a canonical top-down `Viewpoint` across the
set. Featured worlds:

- `warehouse_husky.omniworld` — onboarding demo. Husky random-walks a 26 × 12 m warehouse with reactive, position-based collision recovery.
- `warehouse_industrial.omniworld` — pallet-rack columns, central conveyor, forklift, crates; harder collision scenario. Built end-to-end via the harness.
- `husky_maze.omniworld` + `_unknown.wbt` + `_corners.wbt` + `_visual.wbt` + `_blind.wbt` — progressively harder mazes, including a vision-only world where pixels become tags via a four-cardinal-camera `husky_eye` sidecar.
- `desert_ruins.omniworld` — abandoned-city-in-desert demo built via the harness; real 3D dune terrain, no image backgrounds.
- `husky_fleet_outdoor.wbt`, `mars.wbt` (generated), `swarm_control_showcase.wbt`, `mobile_robot_control_showcase.wbt`, `two_omnibots.wbt`, `ur5e_ik_test.wbt`, `hexapod.wbt`, `stewart_platform.wbt`, `soccer.wbt`.

### Build, launch & developer tooling

- `build_omni.bat` — Windows one-command build. Derives `WEBOTS_HOME` automatically. Wraps the MSYS2 / MinGW64 toolchain.
- `launch.bat` — Windows zero-args launcher. Opens the warehouse Husky demo by default; accepts any `.wbt` path and forwards extra simulator flags.
- `scripts/dev/omnisim_dev.py` — cross-platform helper. Subcommands: `build {core|renderer|gui|controller-libs|all}`, `run-world`, `run-headless` (the headless-validation contract: `--batch --mode=fast --no-rendering --minimize`, monitored log, structured exit codes), `harness`, `test-world`, `test-smoke`, `test-group`, `profile-world`.
- WASD free-fly camera + FPS mouselook, `F`-key follow toggle for the selected object, numpad view snaps in the 3D view.
- Structured runtime log file (`omnisim_log.txt`) at repo root.

### Documentation

- `AGENTS.md` — canonical agent entry point.
- `docs/developer/quickstart.md` — full local build / run walkthrough.
- `docs/developer/agent-map.md` — code-search and subsystem map for agents.
- `docs/developer/simulation-authoring-for-coding-agents.md` — best workflow for building new simulations.
- `docs/developer/omniworld-user-guide.md` and `omniworld-biome-cookbook.md` — procedural world generation.
- Inherited Webots `guide` / `reference` / `automobile` docs rebranded; dead Cyberbotics URLs scrubbed; chapter titles updated.

### Release infrastructure

- `scripts/release/publish_snapshot.sh` — one-commit-per-release publishing script. Operates inside a throwaway worktree under `.build_tmp/release-publish/` and never rewrites history. Defaults to dry-run; `--push` is the only way to actually publish.
- Apache 2.0 file headers brought into line with the OmniLink `copyright-headers.md` convention.
- `SECURITY.md` at repo root.

### Removed

- Cyberbotics-hosted telemetry pings.
- Share-to-cloud dialog wired up to dial out (UI surface kept; outbound network removed).
- Inherited Webots CI workflows moved to `.github/workflows.disabled/` until validated against the OmniSim tree.
- Hardcoded personal paths and machine-specific configuration removed from the tracked tree.
