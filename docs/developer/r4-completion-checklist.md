# R4 → wgpu-default-renderer — completion checklist (living tracker)

The end-to-end task list to make **wgpu the default main-view renderer** (Phase ζ),
from where we are today. Grouped by the risk tiers in [r4-step3c-plan.md](r4-step3c-plan.md);
roughly dependency-ordered. Update boxes as items land — each must be a verified,
flag-gated, zero-break increment (WREN default stays byte-identical until the ζ flip).

Per-arm context: [rendering-arm-checklist.md](rendering-arm-checklist.md),
[engine-migration-plan.md](engine-migration-plan.md) §14.

---

## ✅ Done + verified
- [x] R4-1 — wgpu presents to an on-screen window (swapchain/surface)
- [x] R4-2 — lit, depth-tested 3D into the surface + numerical readback self-check
- [x] R4-3a — live world rendered from the live Viewpoint (tracks the WREN camera)
- [x] R4-3b — composited into the main window as a pane
- [x] R4-3b polish — FOV/aspect matches WREN at any pane size

## ✅ RESOLVED (2026-06-04): the "backend unavailable" wall was a stale stub, not the GPU
Earlier I misdiagnosed the wgpu backend reporting unavailable as a dead GPU
driver. Root cause: `WbVulkanBackend.o` was a stale STUB (built without
WB_WGPU_NATIVE_AVAILABLE in an earlier build), so `isAvailable()` was hardcoded
false even though the rest of the binary linked wgpu_native. **Recompiling
WbVulkanBackend.cpp fixed it** (`dda1da31`); `OMNISIM_WGPU_INITLOG` now traces the
init stage. Lesson: when building
wgpu-ON, ALWAYS touch WbVulkanBackend.cpp; backend-unavailable-at-runtime ≠ driver
fault. The GPU was healthy the whole time.

## 0 — Verification foundation (the unblock) ✅
- [x] **Headless pick-probe** (`08761698`) — `OMNISIM_PROBE_PICK=<file>` renders a
      known triangle via `pickMode`, decodes, writes PASS/FAIL. NO GUI/exposure/timer
      dependency. **VERIFIED PASS 2026-06-04** (center id=1, corner id=0) on the
      rebuilt backend — the GUI-free verification harness the rest of 3c rides on.
- [~] Generalize into a reusable "wgpu-feature → file" probe (extends naturally from
      the pick-probe) so every 3c subsystem is numerically verifiable headlessly. DONE for
      pick (`OMNISIM_PROBE_PICK`), readback (`OMNISIM_PROBE_READBACK`), lines
      (`OMNISIM_PROBE_LINE`), material fidelity (`OMNISIM_PROBE_TEX` → `selfTestTextured`:
      albedo+roughness+metalness+normalMap), and device insets (`OMNISIM_PROBE_INSET` →
      `selfTestInset`). Pattern is established; remaining subsystems reuse it as needed.

## 3c-A — additive parity in the wgpu pane (breaks nothing; flag-gated)
*Each: implement in the wgpu path → probe-verify → commit. WREN untouched.*
- [x] **Picking** — pick pass `7ebb29c9` **RUNTIME-VERIFIED** (probe PASS, `dda1da31`).
      draw→node identity in `collectWorldDraws` + `pickSolidAt` +
      `WbWgpuView::mousePressEvent` → `WbSelection::selectPoseFromView3D` in `cc50902b`
      (compile-verified; click→select still wants a one-time GUI eyeball).
- [x] **Selection highlight** — the selected node's top-solid draws tint toward mimosa
      in the pane (`drawWorld`, re-read every frame so it tracks scene-tree + click
      selection); consistent with the pick path (both resolve to the top solid).
      **VERIFIED PASS 2026-06-04**: self-check renders the world lit twice (highlight
      off/on) and confirms `321 px changed, 0 outside selection` — the tint paints
      exactly the selected solid. (Edge/silhouette *outline* proper — stencil or
      post-process — is a later fidelity refinement; this is the fill highlight.)
- [x] **Manipulator handles** — translate/rotate/scale gizmos in wgpu; hit-test via the
      pick pass; drag-events wired to move/rotate/resize. COMPLETE: render set (translate/
      rotate/scale) + translate & rotate drag (top-level + nested/rotated-parent); scale drag
      is N/A for solids. All hit-tests + the parent-frame transform verified headlessly.
  - [x] **Translate-gizmo rendering** (`f5f77094`) — 3 world-axis lines (R/G/B) at the selected
        solid's origin, always-on-top; live pane (at WbSelection) + verified offscreen on
        camera.wbt (7 gizmos, 971 px). The visual handle.
  - [x] **Translate drag** (`1084a02c`) — `pickHandleAxis` (project gizmo axes → nearest within
        ~10px) + mousePress grab / mouseMove `setTranslation` along the axis / mouseRelease.
        Hit-test VERIFIED headlessly (X-handle midpoint → axis 0); drag-move math sound (full
        press-drag wants a GUI eyeball).
  - [x] **Distance-scaled handles** (`4a52f9a0`) — handle length ~8% of eye→solid distance
        (clamped) so the gizmo keeps a constant on-screen size; live pane + self-check.
  - [x] **Rotate-gizmo rings** (`e08614b8`) — 3 R/G/B rings (about X/Y/Z) per solid, distance-
        scaled, always-on-top. VERIFIED on camera.wbt (rings render, 5066 px). Render-only.
  - [x] **Scale-gizmo boxes** (`a0e20367`) — small R/G/B cubes at the axis ends, distance-scaled.
        VERIFIED on camera.wbt (2057 px). **Gizmo-RENDER set complete (translate/rotate/scale).**
  - [x] **Rotate drag** (`e90748ff`) — `pickRotateRing` (verified) + mousePress grab / mouseMove
        compose-rotation-about-world-axis (`WbMatrix3(axis,Δ)·startRot`) / mouseRelease. Manipulator
        INTERACTION now complete (translate + rotate drag). **Scale drag is N/A for solids** —
        `setScale` is on `WbTransform`, `WbSolid` is not one (no scale field; physics bodies aren't
        scaled); the scale gizmo renders for reference only.
  - [x] **Rotated-parent child-solid drag** (`ebbeb13e`) — translate/rotate drag now applies the
        delta in the PARENT frame (`R_local·unit`), so a solid under a rotated/nested parent tracks
        the on-screen world gizmo axis (was drifting). VERIFIED via a new `parent-frame-drag`
        self-check using `position()` as an oracle on panda's 180°-rotated-parent links:
        `parentRot=2, newErr=3.6e-16, oldErr=0.1`. **Manipulator INTERACTION fully complete**
        (translate + rotate drag, top-level AND nested; scale N/A for solids).
- [x] **Optional renderings** (ADDITIVE-safe) — gated on `WbWrenRenderingContext` `VF_*`
      flags (off by default in the main view → WREN-identical default; toggled by the
      existing View menu). **COMPLETE 8/8**: joint axes, bounding objects, COM, contact points
      (live-verified on the new resting-contact fixture), camera + per-type range-finder
      frustums, surface normals, lidar ray paths, support polygon. All verified headlessly.
  - [x] **Joint axes** (`VF_JOINT_AXES`) — thin cyan bars through each joint anchor along
        its world axis; world transform mirrors `applyToOdeAnchor`/`applyToOdeAxis` via a new
        public `WbJoint::axisRepresentation()` (additive core-API). **VERIFIED 2026-06-04**:
        19 bars on panda, render confirmed (lit-with-axes vs without differ; 2 long-wait runs).
        Depth-occluded by links (axes sit inside the robot) → bar-length / always-on-overlay
        (depth-independent) pass is a follow-up refinement.
  - [x] **Line/wireframe pipeline** (foundation, `24f668c9`) — `WbWgpuRenderTarget::`
        `clearAndDrawLines` + `ensureLinePipeline` (kSolidPick shader + `LineList` topology;
        clears or overlays an existing scene via `loadExisting`, depth-tested). VERIFIED via
        new `OMNISIM_PROBE_LINE`: a red cross renders 231 colored px, corner background.
  - [x] **Bounding objects** (`VF_ALL_BOUNDING_OBJECTS`, `a682e7b2`) — per-primitive wireframe
        edges (Box→12, Sphere→3 rings, Cylinder/Capsule→rings+struts), world-transformed via
        `geom->matrix()`, drawn with `clearAndDrawLines(loadExisting=true)`. VERIFIED + visually
        confirmed: 5521 segments on panda wrap table/shelf/crates correctly. (Offscreen/
        screenshot path; Plane/Mesh skipped this cut.)
  - [x] **Center-of-mass markers** (`adb07ec3`) — magenta cross at each solid's global COM
        (`computedGlobalCenterOfMass`); 165 markers on panda, overlay verified. (No `VF_` flag;
        WREN gates per-solid. Partly depth-occluded inside links — see depth-independent note.)
  - [x] **Contact points** (`VF_CONTACT_POINTS`, `3c87ecde`; on-demand fix + fixture `f0aa9eb2`) —
        cyan crosses at the engine contacts. Switched to `computedContactPoints()` (extracts on
        demand; plain `contactPoints()` was a lazily-cached, usually-empty list) + a resting-contact
        fixture. VERIFIED non-zero: 26 markers / 96 px on the fixture (was "0 in static panda").
  - [x] **Depth-independent (always-on-top) line variant** (`abdd6a94`) — `clearAndDrawLines`
        `depthTest=false` selects a `depthCompare=Always`/no-write pipeline so markers inside
        geometry stay visible. COM markers now change 1563 px (vs 470 depth-tested) + visually
        confirmed (magenta crosses on top across the scene). Markers use it; bounding stays 3D.
  - [x] **Camera/sensor frustums** (`VF_CAMERA_FRUSTUMS`, `d82fa84e`) — near+far rect + 4
        connectors per `WbAbstractCamera`, +X-forward/horizontal-FOV convention, world-
        transformed by `camera->matrix()`, always-on-top. VERIFIED + visually confirmed on
        camera.wbt (1 camera → proper truncated-pyramid frustum). (Per-type RangeFinder gating =
        small refinement.)
  - [x] **Surface normals** (`VF_NORMALS`, `90f66cf2`) — outward lines on primitive geometries
        (Box/Sphere/Cylinder/Capsule), always-on-top. VERIFIED on camera.wbt (80 segments).
        (Per-vertex mesh normals for IndexedFaceSet = CPU-mesh-extraction follow-up.)
  - [x] **Lidar ray paths** (`VF_LIDAR_RAYS_PATHS`, `dc661f13`) — subsampled ray fan per
        `WbLidar` (32×≤6) out to maxRange, +X-forward convention. VERIFIED + visually confirmed
        on lidar.wbt (198 rays). Lidar/range-finder FRUSTUMS already covered (WbLidar is-a
        WbAbstractCamera). **Optional-renderings family now 7/8.**
  - [x] **Per-type RangeFinder frustum gating** (`d6d61889`) — frustum overlay now gates a
        range-finder by VF_RANGE_FINDER_FRUSTUMS and a camera/lidar by VF_CAMERA_FRUSTUMS
        (mirrors WREN's `isRangeFinder()? range : camera`). SELECTIVELY VERIFIED (`53e4f354`) on a
        new dual-sensor fixture (`tests/physics/fixtures/wgpu_dual_sensor.wbt`): camera-only=1,
        range-finder-only=1, both=2, partitions=1 — each flag selects exactly its sensor type.
  - [x] **Support polygon** (`43b71292`) — per top-solid convex hull of ground contacts on the
        horizontal plane (mirrors `WbSolid::supportPolygon()` via `twoStepsConvexHull`, computed
        cold from `computedContactPoints()`). UNBLOCKED by a new resting-contact fixture
        (`tests/physics/fixtures/wgpu_overlay_contacts.wbt`, `f0aa9eb2`) + running `--mode=fast`
        so contacts settle. VERIFIED: 38 hull segments, 164 px [PASS]. **Optional-renderings
        family now 8/8** (joint/bounding/COM/contact/frustums/normals/lidar-rays/support-polygon).
        Same fixture upgraded contact-points from "0 in static" to a real 26-marker / 96 px PASS.
  - [x] **Two-batch live-pane overlays** (`59f5431b`) — `presentScene` now does a depth-tested
        batch (bounding, green) + an always-on-top batch (joint axes as cyan lines, replacing
        the occluded boxes), via a surface no-depth line pipeline + 2-slot uniform. VERIFIED:
        cyan joint axes = 64 px in the LIVE pane (3 runs); bounding via the same path (4956 px
        offscreen / 1684 px live). (Embedded splitter gives the pane a thin strip → can't catch
        both wide at once; constant counts confirm layout, not a bug.)
  - [x] **Contact + COM through the live pane** (`da5f47cb` / `5325e7b7`) — `drawWorld` appends
        VF_CONTACT_POINTS-gated contact crosses AND per-solid-gated COM crosses
        (`globalCenterOfMassRepresentationEnabled()`, mirroring WREN's per-solid toggle — no global
        VF flag) to the always-on-top batch. WREN-identical when off. VERIFIED: com-live-gating
        default=0, after-enable=1 marker [PASS]. (All live overlays share one cyan colour; a
        per-vertex/3rd-batch colour split is a cosmetic refinement.)
  - [x] **WbWgpuSurface line method** (`2fead888`) — `presentScene` line-overlay pass; bounding
        objects now render in the **LIVE on-screen pane**. CONCLUSIVELY VERIFIED 2026-06-04:
        1684 green overlay px + visually confirmed green wireframes wrapping the crate/shelf in
        a wide pane. (A first thin-strip run captured 0 green — pure framing artifact, the
        bounding geometry was out of the narrow column; widening the pane resolved it.)
- [~] **Overlays** — device-output texture overlays (camera/display/range-finder) +
      full-screen overlays (loading/black/status) composited in the pane.
  - [x] **Full-screen overlays** (`e23eed38`) — `drawFullScreenOverlay` + `kFullScreenOverlay`
        (vertex-id full-screen triangle, alpha-blended). VERIFIED: half-alpha black dims the
        scene 65533/65536 px. The loading/black/status overlays.
  - [~] device-output texture insets (camera/display/range-finder image as a corner HUD).
        **Rendering DONE** — `kTexturedQuad` + `drawTexturedInset` primitive (`935101cc`, PROBE_INSET-
        verified) + the full FEED wiring (`b1dd58db`/`d6c0a999`): `runDeviceInsetCheck` finds an
        enabled camera, uploads `image()` (BGRA→QImage→texture), composites it via `drawTexturedInset`,
        asserts inset≠scene + non-uniform, saves `.devinset.png` — an async retry harness (the
        controller enables the camera asynchronously). The wiring PASSES the moment a camera reports
        `isEnabled()+image()`. **Blocked only by the R5 controller-runtime**, confirmed hands-on
        (camera.wbt, fast+realtime, retried to mPhase>40): the controller PROCESS spawns but the
        GUI-side `isEnabled()` never flips in the headless wgpu self-check (enable-IPC/sensor-runtime
        doesn't register). Lighting it up needs the R5 sensor arm or a real controller-stepped run.
- [~] **Screenshots / video**
  - [x] **Screenshot core** — render the world lit to an arbitrary-resolution offscreen
        target → readback → `QImage(RGBA8888)` → PNG. VERIFIED 2026-06-04: 640×480 grab of
        panda.wbt, 303015/307200 non-bg px, PNG saved + **visually confirmed** (correct lit
        3D scene — tables, crates, wall shelf, depth + materials). Built on the proven-
        deterministic readback; foundation for the screenshot action + golden-image CI.
  - [ ] wire into the screenshot menu action + `WbVideoRecorder` when wgpu is the source
        (touches shared GUI paths → 3c-B).
- [x] **Material fidelity** — real `PBRAppearance` so the pane matches WREN, not flat. The full
      map set now samples: baseColorMap + roughnessMap + metalnessMap + normalMap, + per-draw
      specular. Only the Cook-Torrance/sRGB refinements remain (need a WREN golden to match).
  - [x] **Albedo textures** (`fd58d669` fetch + `1632a0c0` render) — `baseColorMap` uploaded
        (`WbImageTexture::image()` → `WbWgpuImageAdapter` → cache, with a `tryGet` fast-path
        fixing a 144-texture per-frame `convertToFormat` stall) and sampled via a new
        `kSolidLitTextured` shader + `ensureSceneTexPipeline` (per-draw texture bind group;
        flat draws stay byte-identical). VERIFIED + visually confirmed on panda: wood-grain
        floor + textured panels/table (PNG 20KB→404KB). Offscreen/screenshot path.
  - [x] **Albedo in the LIVE pane** (`8a85fa3a`) — `presentScene` textured path (surface
        `ensureSceneTexPipeline` keyed to mFormat + per-draw bind group). VERIFIED + visually
        confirmed: live-pane PNG shows the diamond-pattern wall + wood floor (8KB→137KB).
  - [x] **Roughness-aware specular** (`a96d4e2d`) — Blinn-Phong in kSolidLitTextured scaled by
        specularStrength (1-roughness); camera pos → screenshot. Renders, golden-safe (textured
        shader only). Subtle on panda (rough materials); prominent on smooth surfaces.
  - [x] **Live-pane specular** (`667a5b00`) — presentScene now feeds the textured-lit shader
        the per-draw specularStrength + camera world pos (was inert); specular active in the
        live pane, matching offscreen. No regression (pick/screenshot PASS).
  - [x] **Per-pixel roughnessMap** (`20f56c0b`) — `kSolidLitTextured` grows to a 4-entry bind
        group (uniform+albedo+roughness+sampler) in BOTH paths; shader modulates specular by
        `effRough = (1-pad1.w)·map.r` per glTF. Albedo-only draws bind a default-white 1×1
        (byte-identical). VERIFIED end-to-end headlessly via new `OMNISIM_PROBE_TEX` +
        `selfTestTextured`: `albedo=(128,179,230)` matches the hand-computed value exactly,
        black-roughness render saturates brighter (per-pixel modulation proven), pipeline
        builds (4-binding WGSL naga-valid).
  - [x] **metalnessMap + normalMap** (`ea231e23`) — kSolidLitTextured extended 4→6 bindings both
        paths. Metalness: no diffuse + albedo-tinted specular (metallic-roughness). Normal: Schüler
        derivative-tangent perturbation (no mesh tangents). Absent maps → 1×1 defaults
        (white/black/flat) → byte-identical. UNBLOCKED by a new PBR-maps fixture
        (`tests/physics/fixtures/wgpu_pbr_maps.wbt`, `91089acf`) using stock metal PROTOs that ship
        all four maps. VERIFIED via extended `OMNISIM_PROBE_TEX`: metal=(26,77,128) tinted spec,
        tiltNormal collapses the highlight (765→231), albedo/rough byte-identical; real fixture
        renders 4 textured draws end-to-end.
  - [x] **Cook-Torrance GGX BRDF** (`af3736c0`) — specular is now GGX D + Smith G + Schlick F
        (F0=0.04 dielectric / albedo metal), replacing ad-hoc Blinn-Phong. Verified correct via
        the re-tuned PROBE_TEX (matches hand-computed GGX; roughness/metal-Fresnel/normal all pass)
        + panda regression-clean. The standard PBR BRDF WREN uses → moves toward parity.
  - [x] **sRGB color management** (`53fd5f05`) — kSolidLit + kSolidLitTextured linearize the
        sRGB baseColorMap (~pow 2.2), light in linear, sRGB-encode output (~pow 1/2.2); glTF
        convention, wgpu-pane only (WREN/AgX/pick/lines untouched). VERIFIED operation-correct:
        a diffuse-only render round-trips to EXACTLY the input texel (tiltNormal=(26,77,128) ==
        source). Panda regression-clean.
  - [x] **GGX + sRGB material parity vs WREN — VALIDATED** (gate `6a80c0e5`, sharpened `ba48f877`).
        The earlier "needs an external golden" was WRONG: WREN runs here, so `grabWindowBufferNow()`
        IS the local golden. The `wren-parity` self-check renders wgpu from WREN's viewpoint at the
        same res, masks to geometry (pick), saves both frames (`.wren.png`/`.wgpu.png`), and
        DECOMPOSES the gap into brightness (lighting) vs brightness-normalized hue (material). FINDING
        on panda: **alignment GOOD** (frames show identical geometry placement), **hue-diff=0.11 →
        materials/colours MATCH WREN** (GGX/sRGB validated), and the mean-69 gap is **dominated by
        LIGHTING** (wgpu brightness 87 vs WREN 41 — wgpu unshadowed, WREN heavily shadowed). So the
        material BRDFs are parity-correct; the residual is the lighting model, tracked next ↓.
  - [~] **Lighting/shadow convergence to WREN** (residual parity gap; target wgpu brightness 87→WREN 41).
        **CSM rung FUNCTIONAL end-to-end** — increments 1-3 done: `kSolidLitTexturedShadow` shader +
        9-binding pipeline (`39c33e07`, PROBE_TEXSHADOW-valid) + two-pass `clearAndDrawSceneTexturedShadowed`
        (light clip-depth → textured-shadowed lit pass) + parity wiring with a scene-covering light frustum
        (`c54f6869`). MEASURED: the textured-shadowed render works + the gate confirms it moves toward
        WREN (shadowed brightness 84 vs 87 unshadowed). The pipeline/render/measure loop is COMPLETE.
        **Increment 4 (convergence) DEFINITIVELY CHARACTERIZED** (`ffb073c8`): the residual is
        STRUCTURAL, not exposure — scaling wgpu to WREN's brightness (×0.47) makes within-tol WORSE
        (11%→6%), ruling out the ambient-crank shortcut (which would game the metric). **Root cause**
        (panda.wbt): WREN lights via `OmniSimSun` (directional, casts shadows) + `OmniSimSky`
        (procedural-sky IBL/ambient), but the wgpu pane shades with a HARDCODED `lit4=(0.3,0.4,-0.85),
        ambient 0.35` — wrong direction + no IBL → shadows land wrong. **CONVERGENCE NOW ACTIVE +
        IMPROVING** (`0775723c`): harvested the real `OmniSimSun` (a top-level WbDirectionalLight, found
        via the world ROOT — not topSolids; dir=(-0.65,-0.65,-0.39), very different from the hardcoded
        guess) for the lit + shadow frame → **no-shadow within-tol 11%→23%** (mean-diff 69→60), and the
        shadowed render now lands shadows in the right direction (brightness 95→51, target WREN 41).
        Exposure-match still fails (8%<23%) → genuine structural convergence, not gaming. Remaining
        levers (all gate-measured): shadow placement/softness (shadowed within-tol 15% still < no-shadow
        23% — placement off) + `OmniSimSky` IBL/ambient + AO. The multi-session "Tier 1–5 ladder", now
        an ACTIVE measured loop producing real wins (no shortcuts). Frames dumped (.wgpu_shadow.png).
        **HEMISPHERE-IBL ambient implemented + a CORRECTED measurement methodology** (`d15a977f` + the
        intensity sweep): the flat scalar ambient in `kSolidLitTexturedShadow` is replaced by a hemisphere
        term `mix(ground,sky,N·up·0.5+0.5)×intensity` (LightU 80→128B: skyColor/groundColor/upDir; gated on
        shadowParams.z → flat fallback when no sky params → every live/screenshot path byte-identical; the
        parity gate harvests the scene sky colour + `WbWorldInfo::upVector`). PROBE_TEXSHADOW PASS
        (128B/9-binding pipeline naga-validates).
        ⚠️ **METHODOLOGY FIX — the WREN-parity gate is only valid in `--mode=pause` (STATIC scene).** An
        initial `--mode=fast` A/B suggested hemisphere 53%→56%, but that was a **MOTION ARTIFACT and is
        RETRACTED**: under running physics panda's arm sags between the WREN grab, the wgpu renders, and the
        pick pass, so geometry stops aligning with the pick mask → the numbers are garbage (tell: the
        intensity sweep came back NON-MONOTONIC in brightness). Re-run STATIC, the gate reproduces the known
        baseline (no-shadow within-tol 23%, WREN brightness 40) and shows hemisphere **13% vs flat 14% =
        NEUTRAL**; the intensity sweep (1.0→0.4) stays flat ~12% at every step.
        **REAL finding:** the whole textured-SHADOWED rung renders far TOO BRIGHT (145 vs WREN 40 — brighter
        than even the unshadowed 94), so it currently *HURTS* parity (shadowed 13% < plain-lit 23%): "shadows
        don't darken." Ambient directionality is a rounding error on that. **NEXT lever (gate-measured, run
        PAUSED):** make the cast shadow actually darken / fix the shadowed-path exposure (light-frame extent +
        shadow strength + the ambient/direct balance in `kSolidLitTexturedShadow`) to pull 145→~40 — NOT
        ambient colour. The A/B control + intensity sweep stay as the measurement harness for that work.
        🔴 **ROOT CAUSE FOUND by inspecting the dumped frames (`.wren.png` / `.wgpu.png` / `.wgpu_shadow.png`)
        — and it RETRACTS the "CSM rung FUNCTIONAL end-to-end" claim above.** (1) `.wgpu.png` (the unshadowed
        lit render) is FLAT-LIT — every surface evenly illuminated, NO shadows — while `.wren.png` is
        shadow-dominated (deep shade + bright window sun-shafts, brightness 40). The parity gap is therefore
        **SHADOWS**, which is also why AgX (made it brighter, 94→198, within-tol 23%→3%) and a uniform
        exposure scale (23%→8%) both FAIL — you can't turn flat lighting into shafts-in-darkness with a curve.
        (2) The textured-SHADOWED render is PARTLY broken — and a `render-coverage` diagnostic + the
        `.wgpu_shadow_flat.png` dump pin it down precisely (correcting an earlier over-hasty "draws no geometry"
        read of a single frame). Coverage (px ≠ sky clear, full image): plain-lit **2 112 060 / 2 112 192
        (100%)** but textured-shadowed only **~1.0 M (≈50%)**. The flat-shadow frame shows the cause: WALLS, the
        diamond panel, the shelf, the radiator, and every floor OBJECT render correctly — but the **entire
        GROUND/FLOOR plane is the bare sky-blue clear colour**, i.e. the large up-facing ground draw does NOT
        rasterize in pass-2 (`clearAndDrawScene` renders it fine with the same draw list). The missing floor (~half
        the frame) is exactly the 50% coverage gap, and the absent dark-brown floor pixels are why mean brightness
        reads 145 (too bright). An uncaptured-error callback (`OMNISIM_WGPU_ERRLOG`) confirms **NO wgpu validation
        error** fires — so it's a silent state/logic issue (depth precision? large-primitive clip? per-draw
        binding for that specific draw), not a rejected command. **REFINED by pass-2 draw-accounting + 4 repeat
        runs — it is a TIMING/SYNC RACE, not a deterministic skip.** The instrumentation (`[texshadow-pass2]`)
        shows **all 222 draws submit every time** (drawn=222, skipNoBuf=0, skipNoBg=0) — so nothing is skipped on
        the CPU. Yet coverage of the textured-shadowed output is UNSTABLE: a pre-instrumentation run was ~50%;
        after adding a tiny logging delay it became reproducible at **1st call (hemisphere) = 100%, 2nd call
        (flat) = 83%** across 4 back-to-back runs (lit stays 100% always). A reproducible 1st-vs-2nd-call
        difference + sensitivity to a CPU delay ⇒ a **GPU resource/sync hazard** in the two-pass render reusing
        the render target / shadow map / per-draw + LightU buffers across successive `clearAndDrawSceneTexturedShadowed`
        calls (the readback's map-wait completes but the device isn't fully fenced before the next call overwrites
        shared buffers). This is why EVERY brightness/within-tol number on the shadowed path this session was
        unreliable. **FIX ATTEMPT #1 TRIED + FAILED (reverted):** split pass-1 (shadow map) into its OWN submit
        and FENCE to GPU-idle via `wgpuQueueOnSubmittedWorkDone` before pass-2 — the textbook fix for a
        write-then-sample shadow-map hazard. Measured over 4 runs: still UNSTABLE (100/100, 50/31, 100/100,
        100/83) — arguably MORE variable. So the hazard is **NOT the pass-1→pass-2 barrier**; ruled out + reverted
        (kept the tree honest). One run even hit litShadow=2 112 192 (every pixel non-sky — no background at all),
        i.e. the instability isn't only "floor drops," it's broad non-determinism in the textured-shadowed output
        while plain-lit (`clearAndDrawScene`) is rock-stable at 2 112 060 every run. **FIX ATTEMPT #2 TRIED +
        FAILED (reverted):** fence after the render passes, BEFORE the texture→buffer copy (a render→copy barrier
        on `mTexture`, which attempt #1 did not cover). Measured over 6 runs: still UNSTABLE (50/45, 100/100,
        100/50, 98/100, 50/50, 50/50). NOTE the EXACT value 1 071 725 recurs across runs ⇒ not pure random
        garbage but a DETERMINISTIC partial render gated on some timing-quantized state (likely which render
        tick / mPhase the self-check fires on). Two concrete barrier fixes now ruled out → the cause is deeper
        than command-buffer ordering. **FIX ATTEMPT #3 TRIED + FAILED (reverted):** gave pass-2 its OWN
        freshly-created depth texture per call (vs sharing `mScnDepthView` with the stable single-pass path) —
        the stale-depth hypothesis. Measured over 6 runs: still UNSTABLE (100/100, 100/100, 50/45, 50/50,
        100/100, 100/100). So it is **not a shared-depth carryover** either. THREE concrete app-level fixes now
        ruled out by measurement (2 ordering fences + dedicated depth) ⇒ the cause is below the app layer.
        **REMAINING hypotheses for a future focused session (needs RenderDoc / a GPU debugger — not available in
        this env; exact capture recipe in [renderdoc-shadow-capture.md](renderdoc-shadow-capture.md)):**
        ~~(a) dedicated depth texture~~ (RULED OUT, attempt #3); (b) stop sharing `mScnUniformBuffer`/`mShadowUniformBuffer`
        across the 7 successive calls — but a full idle fence between calls already makes shared buffers safe and
        didn't help, so this is unlikely; (c) the 222 per-draw 9-binding bind-groups churned per call (vs lit's
        lighter groups) may expose a wgpu-native allocator/lifetime hazard — cache/pre-allocate (LEADING suspect now);
        (d) the
        recurring-quantized-value clue points at the self-check firing mid-frame — try gating the parity render on
        a higher/settled mPhase or an explicit "scene fully uploaded" signal; (e) a wgpu-native-level bug (plain-lit
        single-pass NEVER exhibits this). Verify any fix by `render-coverage` == lit over ≥6 PAUSED runs AND both
        A/B calls BEFORE trusting any brightness/within-tol number. Only after the render is DETERMINISTIC does
        cast-shadow tuning + OmniSimSky IBL + AO become measurable. (Lesson, the hard way: confirm with a NUMBER over
        MULTIPLE runs — three single-observation reads were over-hasty this session, every one caught by measurement.)
        🟢 **ROOT CAUSE FOUND via RenderDoc (2026-06-06) — and it was done HEADLESSLY/autonomously.** Downloaded the
        RenderDoc portable build, loaded its Vulkan capture layer into wgpu-native via `VK_LAYER_PATH`+
        `VK_INSTANCE_LAYERS` (no admin/registration), added an in-app `StartFrameCapture`/`EndFrameCapture` harness
        to the parity self-check (gated by `OMNISIM_RDC_CAPTURE`) that brackets the shadow render in a 16× loop and
        tags each `.rdc` with its coverage → produced known-GOOD + known-BAD captures, then analysed them with
        `qrenderdoc --python` (PixelHistory + post-VS). **FINDING:** the floor's per-draw transform uniform is read
        as UNINITIALISED GARBAGE on ~50% of runs → its post-VS clip coords are astronomical (z/w out of [0,1]) → the
        primitive is **DEPTH-CLIPPED** → the floor drops. (`EID 1239 passed=False FAILS: depthClipped` on every floor
        pixel.) Garbage in BOTH passes ⇒ the per-draw MODEL slot, not viewProj. **NOT a sync/ordering bug** — 5 fixes
        ruled out (2 pass fences + dedicated depth + 2 write-flushes). The raw-buffer scan then showed the per-draw
        uniform buffer had ~17 garbage slots (3e37) — and the raw buffer being garbage (not a read-offset bug)
        pointed at the CPU.
        🎉 **FIXED (`5c68e406`, 2026-06-06).** The CPU bug: `collectShapeDraws` set each `draw.modelMatrix16 =
        modelStorage.back().data()`, but `modelStorage` is a `std::vector` that grows by `push_back` — every
        reallocation **DANGLED all earlier `modelMatrix16` pointers**, so a draw's transform was read from
        freed/reused memory = intermittent garbage → astronomical clip coords → DEPTH-CLIP → floor drop. The
        lighter single-pass `clearAndDrawScene` mostly survived (freed memory not yet reused), which is exactly
        why it looked GPU-specific + why 5 GPU-sync fixes + mappedAtCreation all failed. **Fix:** after
        `collectWorldDraws` finishes all push_backs (modelStorage final-sized), re-point every draw to its STABLE
        slot (draws+models pushed 1:1). **VERIFIED: render-coverage IDENTICAL across 10/10 PAUSED runs** (litShadow
        == litShadowFlat == lit, zero drops) — was ~50% non-deterministic. **The render is now DETERMINISTIC →
        lighting parity / 3c-B / ζ are UNBLOCKED.** Next: re-measure cast-shadow vs WREN sun-shafts + OmniSimSky IBL
        (now meaningful). Method + scripts: [renderdoc-shadow-capture.md](renderdoc-shadow-capture.md), `_scratch/rdc_*.py`.
        🟡 **LIGHTING PARITY — first reliable tuning win (post-fix).** Reliable baseline: wgpu-mean-brightness 94 vs
        WREN 40, hue-diff 0.11 (materials MATCH — residual is purely lighting); AgX + uniform-exposure both make it
        WORSE → the gap is STRUCTURAL shadow placement. Visual diff confirmed it: WREN renders panda heavily
        shadow-dominated (room in shade + sun-shafts), wgpu was flat-lit. **Lever found + gate-measured:** drop the
        cast-shadow ambient 0.35→0.10 + strength 0.6→1.0 (lowering ambient darkens only indirectly-lit pixels, NOT
        the sunlit ones — unlike a uniform scale). A/B (same WREN frame): shadowed-mean-brightness 85→**53** (target
        40), mean-diff/ch 52→**32**; the dumped frame now shows a real diagonal cast shadow matching WREN's shade
        boundary. **Then the ambient sweep cracked it: WREN's shadows are near-BLACK (no sky fill), so ambient→0
        is the match.** Gate-measured sweep (flat, strength 1.0, same WREN frame): a=0.25→23%, 0.15→21%, 0.10→24%,
        0.05→60%, **0.0→75%** (mean-diff 41→18). Set the primary cast-shadow render to **ambient 0 + full strength**
        → **within-tol 23% → 75%, mean-diff/ch 59→18**, and the dumped frame now VISUALLY matches WREN (deep room
        shadow + the diagonal sunlit boundary + lit right wall + radiator). Hemisphere confirmed net-negative
        (disabled). Residual 25%: wgpu mean-brightness 26 vs WREN 40 (wgpu shadows pure-black, WREN has a hair of
        fill) + exact shaft EDGES — the finer rung (tune sun/direct intensity + shadow-cascade placement). **First
        strong WREN parity on the cast-shadow render.**
        **AMBIENT/FILL LEVER NOW MAXED (75%).** Confirmed both ways: the uniform-ambient sweep (any a>0 worse than 0)
        AND a directional low-intensity hemisphere sky-fill at ambient 0 (tested → **22% << 75%**). WREN's shadows are
        genuinely near-black; ANY fill, uniform or directional, hurts. So the residual 25% is NOT shadow fill — it is
        (a) shaft-EDGE placement (where wgpu's shadow boundary vs WREN's differ) + (b) lit-area brightness (wgpu sunlit
        94 vs WREN's dimmer lit) + AA. AgX is ruled out too (it brightens LDR mid-tones, wrong way: 94→189). These are
        the genuinely-finer multi-rung levers; the high-value autonomous lighting tuning is exhausted at 75%.
- [~] **Robustness**
  - [x] **Degenerate/sentinel draw cull** — VERIFIED 2026-06-04. Verify-first diagnostic
        confirmed 1 of 222 panda draws had `|translation|=1e5` (an off-screen sentinel);
        `collectShapeDraws` now drops non-finite / `|t|>1e4` transforms → 221 draws,
        degenerate scan = 0, `max|t|=11.23`, pick/selection/joint still PASS. (NOTE: the
        Camera path has its own `collectShapeDraws` copy — apply the same cull there when
        convenient for RTT-sensor parity.)
  - [x] resize / DPI correctness — offscreen render targets verified at 64/256/480/640 px
        (pick/self-check/screenshot, all aspect-correct) + DPI applied via `devicePixelRatio`
        in `pixelWidth/Height`; the pane `resizeEvent` reconfigures the surface. The LIVE
        swapchain resize is now ALSO exercised every self-check run: the harness force-resizes the
        window 640×480 → 1400×900 (→ `WbWgpuSurface::resize` swapchain reconfigure), and every
        subsequent render (drawWorld/screenshot/overlays) succeeds at the new size. Only an
        interactive mouse-drag resize is untested (cosmetic; the reconfigure path is proven).
  - [x] **Readback determinism** — VERIFIED deterministic, 2026-06-04, via new headless
        `OMNISIM_PROBE_READBACK` probe (64 renders × 3 runs of one identical scene →
        byte-identical, `framesDiffering=0 maxByteDelta=0`). The earlier "intermittent
        corruption" was a **misdiagnosis, not a readback race**: the selection self-check
        classified changed pixels against `buf` — a pick buffer captured at the *top* of
        `runSelfCheck`, an EARLIER sim instant — which went stale when the robot moved
        between renders (→ spurious "outside" ~1/3 runs). Fixed by classifying against a
        **same-frame** pick buffer (`pidBuf`); selection-highlight now reads `0 outside`
        across 3 runs. The probe gives golden-image CI a determinism gate to lean on.

## 3c-B — the gated flip (the ONLY step that can break the sim — human sign-off)
**HUMAN CHECKPOINT GIVEN 2026-06-06 ("let's do it").** Building it additively, WREN byte-identical.

> ✅ **UN-GATED 2026-06-07 (`d42c5554`).** The experimental `OMNISIM_WGPU_MAINVIEW` flag is removed — a
> Viewpoint that selects `renderBackend "wgpu"` now renders the main view through wgpu directly (the OOM
> that justified the gate was the texture-cache key bug, fixed in `a4fec74b` + 6-world soak). **Verified
> post-un-gate:** default panda.wbt (WREN Viewpoint), no flags → wgpu path never taken (report empty),
> survives, 0 errors = WREN byte-identical; a `renderBackend "wgpu"` Viewpoint, no flags → wgpu main view
> renders 200+ frames (cdOk=1) at 1896×1113, textures plateau at 63, survives 42 s, 0 errors.
> `OMNISIM_WGPU_MAINVIEW_FORCE` kept as a test lever. This is the per-node OPT-IN (Layer B/node); the
> DEFAULT flip (Phase ζ — flipping Viewpoint.wrl's default to "wgpu") stays gated on §5.2 parity (~75%
> now) + §5.3 cross-platform + the determinism/golden gate below.
- [x] **`WbView3D::renderNow` backend-dispatch SEAM (`5ef0e329`, increment 1).** The single main-view
      frame funnel now consults the active Viewpoint's `renderBackend()` via a new `renderMainFrameViaWgpu()`
      helper; returns false → the UNCHANGED `WbWrenWindow::renderNow()` runs (WREN byte-identical; no shipped
      world sets `renderBackend "wgpu"`, so the branch is unreachable for them). Verified: builds clean,
      WREN-default panda.wbt renders + holds a live window, no crash. (The seam lives at WbView3D, not
      WbWrenWindow's default path — the per-backend init/grab dispatch is naturally subsumed here.)
- [x] **Backend-selectable surface — solved by OFFSCREEN render → GL blit (increment 1b).** WbView3D keeps
      its OpenGL surface (so ALL editor interaction stays intact + no HWND pixel-format change = the "HWND
      danger" sidestepped). When wgpu is selected, the live world is rendered offscreen via wgpu → RGBA, then
      blitted into the GL window via a tiny non-Qt TU `render/WbWgpuGlBlit.cpp` (isolates glad from Qt; Y-flip
      via `glBlitFramebuffer`). New WbView3D members: lazy wgpu backend + mesh/texture caches + a cached
      offscreen render target (recreated on resize).
- [x] **Route the main 3D view through wgpu when selected; WREN branch byte-identical** — done (the helper).
      Verified: `renderBackend "wgpu"` WITHOUT the experimental flag → safe WREN fallback (alive, no crash);
      WITH `OMNISIM_WGPU_MAINVIEW=1` → the wgpu render runs + presents.
- [x] **STABILITY HARDENING — ✅ FIXED (`a4fec74b`, 2026-06-06). It was an APP-LEVEL texture-cache key
      bug, NOT a wgpu-native leak (the analysis below this line is RETRACTED).** `collectShapeDraws` keyed
      the wgpu texture cache on the `WbImageTexture*` pointer, but a scene PROTO makes a separate
      `WbImageTexture` per use, so many shapes sharing one texture FILE (one `Plaster.jpg` across the factory
      walls) had distinct pointers → the cache re-uploaded the same file once per instance. panda.wbt's ~63
      unique files → **507 GPU uploads** (mostly 2048²/1024², multi-GB) → VRAM OOM at ~30 s. Fix: key on the
      source file path (`WbWgpuSceneRenderer::stableTexId`). **Verified:** creations 507 → 63, 0 wgpu errors,
      main view renders 200+ frames (`cdOk=1`) at 1960×1122, soaks 75 s+. Found with an env-gated harness
      (`f2dfb949`): `OMNISIM_PROBE_SOAK` (bare submit+readback leak-free, 6000 frames, flat `wgpuGenerateReport`
      → leak is app-side), `OMNISIM_WGPU_MAINVIEW_FORCE` (drive the real main view on a stock world),
      `OMNISIM_WGPU_TEXLOG` + stack capture (pinned the flood to `collectShapeDraws`). **No Rust / wgpu-native
      patch needed.** The un-gate (removing the `OMNISIM_WGPU_MAINVIEW` gate) is the next step — still under
      the 3c-B human checkpoint; needs a longer multi-world soak + the determinism/golden gate below.
      ⚠️ Pre-fix (superseded) analysis follows:
      The per-frame render faults (~0xC0000409, which on Windows is a wgpu-native/Rust PANIC, not a null deref).
      `OMNISIM_WGPU_ERRLOG` over a sustained run captured the actual cause: **`wgpuDeviceCreateTexture … Not
      enough memory left`** (142×) + `wgpuQueueWriteTexture … Texture invalid` (284×) — i.e. a **per-frame GPU
      TEXTURE leak → VRAM OOM → wgpu panic.** A render-only bisect ruled out the GL blit; the render-target
      destructor frees everything + `acquireFromQImage` has a `tryGet` cache fast-path, yet the `writeTexture`
      errors show the texture cache RE-UPLOADING — so the likely leak is the texture cache GROWING because the
      per-draw texId (the `WbImageTexture*`) changes every frame (panda PROTO appearance churn), so the
      unbounded `mEntries` map accumulates a new GPU texture per draw per frame. (The one-shot screenshot/
      self-check never hit it; the live pane uses the SURFACE path, not this offscreen path, and was never run
      long enough to reveal it.) So the wgpu main-view render is gated behind `OMNISIM_WGPU_MAINVIEW` →
      `renderBackend "wgpu"` ALONE degrades safely to WREN. **FOUR targeted fixes TRIED + RULED OUT — the
      34 s fault recurs invariant to all of them:** (1) create the render target ONCE (no per-frame recreate);
      (2) LRU VRAM cap on `WbWgpuTextureCache` (512 MB, evict LRU); (3) confirmed `ensureScenePipeline` is
      guarded (no per-call depth-texture creation); (4) an extra `wgpuDevicePoll` maintain after each readback to
      reclaim retired-frame resources. NONE moved the fault → it is a **wgpu-native-internal accumulation under
      sustained per-frame submit+READBACK**, not an app-level leak. **KEY un-gate insight:** the live wgpu PANE
      (`WbWgpuView`, `presentScene` → SURFACE) renders the same world per-frame and NEVER crashes — because it
      does NOT do the offscreen `copyTextureToBuffer`+`mapAsync` readback. So the readback is the culprit, and the
      un-gate path is to **avoid the per-frame readback**, two options: (A) the WINDOW-SWAP — show `WbWgpuView`
      (its own Vulkan surface, no readback) as the main view when wgpu is selected (needs WbSimulationView
      per-world view switching + WbWgpuView lacks WbView3D's full editor interaction); (B) wgpu↔GL SHARED-TEXTURE
      interop (present the wgpu-rendered texture to the GL window without a CPU round-trip; platform-specific).
      Both are substantial restructures = a design call. The LRU cap is KEPT (sound general hygiene). So
      un-gating remains the open item; the architecture + render path are committed + WREN-default-safe.
      **FEASIBILITY CHECKED — Option B (interop) is INFEASIBLE with the pinned wgpu-native** (`_scratch/wgpu-native`
      headers): the only external-texture API is `WGPUExternalTexture`, whose creation is "extremely
      implementation-dependent and NOT defined in this header" and is for SAMPLING external (YUV) textures INTO
      wgpu — there is NO texture-EXPORT / shared-handle / external-memory / DXGI-shared API to hand a wgpu texture
      OUT to GL. So B needs a wgpu-native upgrade/fork (or a newer wgpu-native that adds interop). ⇒ The un-gate
      narrows to: **A (window-swap to `WbWgpuView`'s surface)** — un-gates but the wgpu main view gets only the
      pane's PARTIAL interaction (pick/select + translate/rotate manipulators I built), NOT WbView3D's full editor,
      so it is not yet a true ζ-grade full replacement; OR a **wgpu-native-level fix** (readback resource reclaim,
      or add an interop export path). Neither is a clean app-level finish; the architecturally-correct
      full-interaction main view (the offscreen-blit seam, committed) needs the wgpu-native readback issue resolved.
- [ ] Gate: determinism smoke (`accelerometer`, `contact_points`, `template_deterministic`)
      + golden images green with the flag OFF — the seam is byte-identical by construction (WREN call
      unchanged + branch unreachable for shipped worlds); a full determinism-smoke run still wanted before ζ.
- [x] Human checkpoint before touching `WbWrenWindow`'s default path — **GIVEN** (and the seam deliberately
      lives at WbView3D, leaving `WbWrenWindow`'s default path untouched).

## Phase ζ — make wgpu the default

> **DECISION 2026-06-11 (user): WREN stays the default; the flip is DEFERRED.** Not a
> stability call — wgpu passes all hard gates and beats WREN ~2.4× on per-frame render
> cost — but wgpu is not yet a feature superset (single-sun lighting, no transparency,
> no image backgrounds, render-bound pacing gap, Windows-only). Full state snapshot +
> measured comparison + pre-flip gap list: [wgpu-renderer-status.md](wgpu-renderer-status.md).
> The flip remains human-gated: do not flip without an explicit user decision.

- [ ] Flip `renderBackend` default `"wren"` → `"wgpu"` on Viewpoint; WREN → documented fallback.
- [ ] Golden-image parity wired into CI as a standing gate.
- [ ] Main-view perf acceptance bar (plan DoD: 64-camera scene @ 60 FPS, consumer HW).
- [ ] 2×2 build-matrix safety net bit-identical with both flags OFF.
- [ ] Cross-platform surfaces: Metal (macOS) + Vulkan (Linux) — today Win32-HWND only.

## Cross-cutting dependencies (gate "truly replaces WREN")
- [ ] Tier 1–5 fidelity on wgpu: AgX ✅, CSM ✅ (diagnostic → wire to main), TAA + T2–T5 pending.
- [x] **R5 sensor pipeline (Lidar/RangeFinder/Camera/Recognition) — parity CONFIRMED, regression GREEN
      (EXIT=0).** Ran `scripts/dev/wgpu_sensor_regression.py` (`OMNISIM_HOME=<checkout>`, PowerShell, `--attempts 6`):
      ALL pass — camera lit/depth1/2/3 (R5/R5b/R5d), light-depth CSM (T1.2), RangeFinder (R5c), lidar R5e–R5l
      (single/multi-layer/wide-FOV/azimuth/tilt/rotating, all match the WREN oracle), camera emissive + specular
      AgX on/off. **FIXED the one regression the run surfaced** (`643b3096`): the `53fd5f05` sRGB change had leaked
      into the camera RTT sensor path (shared `kSolidLit`), shifting camera readback 141→195 and breaking a
      zero-break guard (71→142). Resolved by gating the sRGB encode on `kSolidLit`'s `pad0.x` (0→sRGB, the default
      for every display caller; >0.5→linear) + a `clearAndDrawScene(srgbEncode=true)` param that only `WbCamera`
      sets false → the camera keeps its R5-landed LINEAR space (controllers/ML consumers unchanged), display pane +
      screenshots + WREN-parity gate keep sRGB. Verified: camera/lit back to (0,141,141), specular zero-break back
      to (71,71,71). NOTE: some runs show intermittent "no result after retries" on a case or two — that's the
      separate load-time **non-determinism family** (same root as the shadow-render bug, [needs RenderDoc]); the
      sensor VALUES are correct when a frame is produced, and `--attempts` clears it. Logs: `_scratch/sensor_regression*.log`.
- [x] **Supported wgpu-ON build config + ship `wgpu_native.dll`** — the link step now AUTO-COPIES
      `wgpu_native.dll` next to `omnisim-bin.exe` (Makefile post-link `EXTRA_CMD`, guarded on
      `OSTYPE=windows` + the dll's presence inside the `OMNISIM_WITH_VULKAN=ON` block → non-wgpu /
      non-Windows builds byte-for-byte unaffected). VERIFIED: removed the dll, forced a relink → the
      build re-shipped it. The canonical wgpu-ON recipe (full flag set) + the now-automatic shipping are
      documented in `docs/developer/wgpu-native-setup.md` (§2.6). No more manual PATH/copy step. (CI
      packaging + a build-matrix cell remain part of Phase ζ; this is the local supported config.)

---

**Critical path:** `0 (pick-probe)` → finish picking → manipulators/overlays/optional-
renderings (parallelizable) → material fidelity → `3c-B` flip (human gate) → `ζ` default.
Hardest architectural risk: 3c-B. Largest volume: 3c-A + the fidelity ladder.
