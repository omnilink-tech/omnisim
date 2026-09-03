# WREN deletion runbook — the remaining steps, in order, with exit gates

**Status: DONE 2026-08-23 (`976b9449d`) — `src/wren`, `include/wren`, `src/omnisim/wren` and 86 shaders are deleted; wgpu-native is the only renderer.** Everything below is the runbook as it was executed and is kept as the record; `wren_deletion_audit.py` went with the code it audited. Status as last written: OPEN. `src/wren` (106 files, 19,847 lines) + `src/omnisim/wren` (68 files) are still
present. Gate: `python scripts/dev/wren_deletion_audit.py` → NOT DELETION-READY, 349 blocking,
0 retirable** (347 before P7/P8). ⚠️ The +5 is P7/P8 and it is expected, not a regression: a port
that reads a WREN-side MODEL adds findings until D1 moves that model. **P2 + P11 (`adc7cc0a6`)
leave it at 352, also expected**: they add wgpu code and remove no `wr_*` call site — `OmTrack`'s
and `OmMuscle`'s go with D1.4 like every other node's. It decomposes exactly:
`OmHudOverlay.cpp` includes `OmWrenTextureOverlay.hpp` + `OmWrenLabelOverlay.hpp` (+2 under
"#include of src/omnisim/wren/*"), and `OmGizmoLines.cpp` includes
`OmTranslateRotateManipulator.hpp` (+1 there), `<wren/transform.h>` (+1 under "#include
<wren/...>") and calls `wr_transform_get_matrix` at 4 sites (+1 file under "live wr_* code
sites"). ⚠️ Read this number from a COMMITTED tree: the audit scans git-tracked files only, so
reading it while a new source file is still untracked under-counts (it read 348/349 mid-work and
352 once both commits landed).

This is the execution plan for wren-retirement-plan.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md))'s endgame. That
document is the audit, the history and the corrections; this one is the ordered work list.

## Why the "347 blocking" number is not the plan

Those are `wr_*` call sites, and **most of them disappear WITH WREN rather than before it** — you
cannot delete WREN's scene-graph or material code while WREN is still a selectable renderer. The
number measures coupling, not readiness, and it will stay high until the single deletion commit
takes it to zero. Sequencing off it produces busywork.

**The list that decides deletion is the parity list: what a user would LOSE if `src/wren` vanished
tonight.** Every step below is one item on that list, and every step has a gate that is a
measurement, not an opinion.

## Standing constraints for every step

- ⚠️ **Thermal.** This is the owner's laptop with a **75 °C ceiling**. Wrap every engine run in
  `python scripts/dev/thermal_guard.py run --ceiling 75 -- <cmd>`. **One `omnisim-bin` at a time**,
  never a build concurrent with a render batch, and **no `--mode=fast` throughput benchmarks
  without asking** — that is the hottest thing in the repo and it is what put the GPU at 87 °C on
  2026-08-22. A constraint that is not in a delegated brief does not exist.
- **Shared working tree.** 400+ files belong to other lanes. Commit only with
  `git commit -F <msg> -- <explicit paths>`. Never `git add -A`, a bare `git commit`, `git stash`,
  `git checkout .`, or `git reset --hard`.
- **Every claim is a measurement.** Re-run the audit after each step and quote before/after. Prove
  an instrument can go red before believing it green.
- **Public ABI is frozen.** `wb_*` / `Wb*` symbols and the `nodes.h` enumerator ORDER never change,
  even when the node behind them is deleted (the Radio/Microphone/Skin/VR precedent).

---

## Phase P — parity (must all be green before anything is deleted)

### P1 — deformables for the SENSOR path — ✅ DONE (with P9, `4ac917c0c`)
Cloth/SoftBody render in the main view since `d5897ff7d`, but a wgpu-rendered
Camera/RangeFinder/Lidar still saw nothing: those collect through `collectWorldDraws` alone, and
the main-view fix appends deformables downstream of it.
**Gate:** a world with a Camera pointed at a cloth returns a controller image whose pixels change
when the cloth is removed. **Zero tracked worlds paired a `Camera {` with a `Cloth {`**, so the
gate world was authored as part of this step.

**Landed with P9 in one commit, because they are ONE structural problem.** The sensor sites now
call `OmAbstractCamera::collectWgpuDraws()` — one function on the shared base of Camera /
RangeFinder / Lidar — which calls `collectWorldDraws` and then
`OmWgpuSceneRenderer::collectDynamicDraws()`, the single entry point for every non-`OmSolid`,
per-step-varying node type (Cloth, SoftBody, GranularGroup). The failure was structural — eight
call sites, one of which knew about the dynamic content — so the fix is structural: adding a node
type edits one function, and adding a sensor gets every node type for free.

⚠️ **THE SECOND DEFECT, WHICH IS THE ONE WORTH REMEMBERING.** Wiring the sensors into the existing
`collectDeformableDraws` is not enough, and doing only that ships a *worse* bug than the one it
fixes. That function decided "do I need to re-upload the vertices?" from a **function-local static**
keyed on the simulation clock (`sLastPumpTime`) — correct while there is exactly one caller, wrong
the instant there are two, because **each device owns its own `OmWgpuMeshCache`**. Whichever
renderer reaches a step first consumes the "clock advanced" edge; every later one still takes the
first-upload branch once and then never updates. The loser renders a surface **frozen at its first
uploaded pose**, with the right draw count, the right triangles, a clean build, and a passing
single-screenshot test.

The split: the **pump** (`animateDeformables()`, re-reading the solver into the nodes' CPU arrays)
is genuinely process-global and once per step, and stays a static in the collector; the **upload**
decision moved onto the cache entry itself as `OmWgpuMeshCache::vertexEpochIs()` / `setVertexEpoch()`,
so it is per (cache, mesh) by construction and dies with the entry. Granular needs no such epoch and
the header says why: a particle's per-frame datum is its **model matrix**, written into the caller's
own `modelStorage`, so there is no shared GPU copy whose freshness a global flag could get wrong.

**Gate: MET.** Machine `9722d23d12a3` (RTX 3060 laptop), binary of 2026-08-22 22:46.
New instrument, re-runnable: **`python scripts/dev/wgpu_dynamic_gate.py`** (`--case cloth|granular`),
driving two authored gate worlds + a byte-for-byte control world in
`projects/samples/demos/worlds/rendering/`.

 * ⭐ **RED, and it is a REAL zero.** `OMNISIM_WGPU_SENSOR_DYNAMIC=0` →
   `camera_cloth_wgpu_smoke.omniworld` and `camera_cloth_wgpu_control.omniworld` (the same scene
   with the `Cloth` node and nothing else removed) produce **0 px over threshold, max abs channel
   delta 0** — identical images, i.e. the sensor genuinely cannot see the cloth on that arm.
 * **GREEN.** Default → the same pair differ by **698 px over threshold, max 172** (96×72 readback),
   and a hue count puts **648 red cloth pixels** in the gate image against **0** in the control.
 * ⭐ **ANIMATION, and the headless arm is NOT sufficient for it — say so.** Headless has ONE
   collector, so the per-cache epoch and the global clock edge are indistinguishable there; the
   headless animation number (938 px changed between step 40 and 240) proves the sheet moves, not
   that the fix was needed. The decisive measurement is **WINDOWED**, where the main view and the
   Camera both collect, A/B'd on one binary through `OMNISIM_WGPU_DEFORMABLE_EPOCH` (added
   precisely so this bug stays reproducible):

   | arm | main-view `deformZ` at frames 0 / 100 / 200 / 300 / 400 | camera sample checksums |
   |---|---|---|
   | default | 0.4343/1.5657 → 0.1297/1.2453 → 0.1577/1.2456 → 0.1246/1.2375 | 3 distinct of 3 |
   | `OMNISIM_WGPU_DEFORMABLE_EPOCH=0` | **0.4343/1.5657 four times running**, then 0.1279/1.2372 | 3 distinct of 3 |

   `0.4343/1.5657` is exactly the **authored flat sheet** (centre z = 1.0 ± the 0.8 m square's
   0.5657 half-diagonal). On the reverted arm the main view held that pose for frames 0–300 while
   the simulation ran to **t = 2.88 s** and the Camera's own samples were demonstrably changing —
   and it unfroze at frame 400, immediately after the controller called `cam.disable()` and stopped
   competing for the edge. ⚠️ **Which consumer freezes depends on who wins the race**: here the
   Camera won every step and the MAIN VIEW froze. The defect is the shared flag, not a fixed victim.
 * **Zero cost where neither node type exists:** `beauty_bench` hatches-off vs default
   `mean=0.0179 max=8 px>30=0`, inside its own noise floor `mean=0.0157 max=8 px>30=0`.
 * **No sensor regression:** `scripts/dev/wgpu_sensor_regression.py` **19/19 PASS** on the same
   binary. `run-headless --until-finalized` PASS on the three new worlds and on
   `newton_cloth_drape`, `newton_softbody_drop`, `tests/cuda/granular_group_load`.

**Hatches, all value-parsed, all read once:** `OMNISIM_WGPU_SENSOR_DYNAMIC=0` (sensors collect
Solids only — the exact pre-P1 sensor image, main view untouched), `OMNISIM_WGPU_DEFORMABLE_EPOCH=0`
(restore the global-edge upload rule — for reproducing the freeze, never for shipping),
`OMNISIM_WGPU_GRANULAR=0` (P9, below). `OMNISIM_WGPU_DEFORMABLES=0` is unchanged.

**Audit: unchanged at 352 blocking, and that is expected.** P1/P9 add wgpu code and remove no
`wr_*` call site — `OmGranularGroup`'s 22 sites and the deformables' still go with D1.4, as with
every other node. The number measures coupling, not readiness; see the top of this file.

**Left open:** `hiddenNodes` is threaded through `collectDeformableDraws` / `collectGranularDraws`
with the same default-null semantics `collectWorldDraws` has, and the filter is implemented — but
**every call site passes `nullptr`, exactly as `collectWorldDraws`' own call sites do**. So
`wb_supervisor_node_set_visibility` still cannot hide a Cloth from one viewer on the wgpu path.
That is a pre-existing, tree-wide gap (the whole per-viewer visibility path is unwired), not one
this step introduced; the parameter exists so the two collects cannot diverge when someone wires it.

### P2 — `Muscle` and `Track` surfaces — ✅ DONE (`adc7cc0a6`)
Camera-visible geometry (`OmMuscle` is `VM_REGULAR`) with no wgpu equivalent; 7 tracked worlds
**plus `projects/objects/factory/conveyors/protos/ConveyorBelt.proto`**, which is what makes this
the only parity step with a SHIPPED FLAGSHIP DEMO affected (`warehouse_industrial.omniworld`,
`icon_studio.omniworld`).
**Gate:** `draws` and the OmniLight triangle count rise on a muscle world, and the surface animates
across frames (the same two-defect trap P1's predecessor had: collection without animation is a
half-fix that looks right in one screenshot).

⚠ **THE TWO NODES ARE NOT THE SAME KIND OF THING AND MUST NOT BE PORTED AS ONE.**
 * **`OmTrack` is INSTANCED GEOMETRY, not a deformable.** It derives from `OmSolid`, and
   `animateMesh()` generates **no vertices at all** — it advances `mBeltPositions[i]` and writes
   `wr_transform_set_position` / `_set_orientation` on N `WrTransform`s. So the port is N extra
   MODEL MATRICES reusing a mesh the ordinary Shape collect already knows how to acquire — the
   same shape P9's granular port used. The source geometry is not double-drawn for free reasons
   worth knowing: `animatedGeometry` is an **SFNode field, not a child**, so `collectWorldDraws`'
   children walk never reaches it, which is the wgpu equivalent of WREN's
   `wr_node_set_visible(geom, false)`.
 * **`OmMuscle` is PROCEDURAL and owns no CPU-side geometry at all** — no `mPositions`, no
   `mIndices`; `updateMeshCoordinates()` synthesises the spheroid straight into a `WrDynamicMesh`
   one `wr_dynamic_mesh_add_vertex` at a time. Its INPUTS (`mMatrix`, `mHeight`, `mRadius`) are
   ordinary engine state computed WREN-free in `computeStretchedDimensions()`, so the port is that
   same loop emitting `pos3+norm3+uv2` stride 32 into a byte buffer, uploaded through the
   per-(cache, mesh) epoch. It is also **not appearance-driven**: hardcoded phong colours, the
   `gl:textures/muscle.png` asset, and a status-driven recolour, so `fillAppearanceMaterial` has
   nothing to read and the material is reproduced literally instead.

**What landed.** Both go through `OmWgpuSceneRenderer::collectDynamicDraws()` — the ONE entry point
the main view and `OmAbstractCamera::collectWgpuDraws` share — so the sensors got them for free and
the P1 regression (a node type wired into the main view only) cannot repeat. Three extractions kept
this from being a second implementation of anything:
 * `acquireGeometryMesh()` — the Shape branch's whole geometry→mesh switch, lifted verbatim (same
   cache keys, same hatches, same fallbacks). A belt element and a Shape now get their mesh from
   ONE place, so they cannot tessellate the same node differently.
 * `acquireStreamedMesh()` — the per-frame vertex re-upload, lifted out of `appendDeformableDraw`
   so Cloth, SoftBody and Muscle share one implementation of the decision that has already gone
   wrong once here (`OmWgpuMeshCache::vertexEpochIs`, per-(cache, mesh), never a process-global
   "the clock advanced" bool).
 * `fillDeformableMaterial` → `fillAppearanceMaterial`, now also feeding the Track belt.
Plus `OmTrack::liveTracks()` / `OmMuscle::liveMuscles()` — ctor/dtor-maintained registries, because
⚠ **`anyDeformablesSubscribed()` does NOT cover these two**: the frame listener's track and muscle
lists are PENDING-REBUILD queues that `frameStarted()` CLEARS after running them, so they are empty
most of the time and answer a different question. And
`OmWrenVertexArrayFrameListener::animateTracksAndMuscles()`, the wgpu-side pump for the
`wr_scene_render` frame callback these nodes never receive.

⛔ **A PREREQUISITE FOUND BY THE GATE, AND IT IS BIGGER THAN THE PORT: EVERY TRACK ANIMATION IN THE
TREE WAS DEAD, ON BOTH RENDERERS.** `OmTrack::prePhysicsStep` computed its surface velocity inside
`if (mLinearMotor && mBodyID)`. `mBodyID` comes from `OmSolid::body()` → `OmSolidMerger::body()`,
and `OmSolidMerger::mBody` is a `dBodyID` **set to NULL in the constructor and assigned nowhere else
since ODE was deleted** (`bdc02139`). So the branch was unreachable, `mSurfaceVelocity` was 0.0 on
every step, and with it `travelledDistance`: **the belt never advanced, the `TrackWheel`s never
rotated, the `textureAnimation` scroll never moved, and the LinearMotor's `PositionSensor` read 0.0
for ever.** Measured on `track.omniworld` before the fix — the leading belt element sat at its
authored path position `(-0.15, 0.0535)` for 900 rendered frames. The gate is what exposed it: a
"the surface animates" assertion cannot be satisfied by a renderer when the *engine* is not moving
the surface. Removing the dead gate is the same class of fix as `e4fb9e822` and has **no physics
consumer to disturb** — `contactSurfaceVelocity()` has ZERO readers in the tree, because the ODE
contact-surface-velocity plumbing went with ODE. ⚠ It does **not** make a tracked robot DRIVE:
Track propulsion under Newton is a separate, still-open gap (the belt spins, the wheels turn, the
chassis does not move).

⚠ **AND THE FLAGSHIP'S BELT IS THE *TEXTURE*, NOT THE GEOMETRY.** `ConveyorBelt.proto` declares no
`animatedGeometry` at all — it animates by scrolling its `TextureTransform` every step. That worked
on every wgpu SENSOR (each rebuilds its draw list per frame, and `fillUvTransform` re-reads the
field) and was FROZEN on the main view, whose cached draw list refreshed only the MODEL MATRICES.
`refreshWorldDraws()` now also re-reads each cached draw's UV affine, gated on
`OmTrack::anyTextureAnimation()` so every other world pays one empty-vector test, and with the
appearance re-derived from the refresh record's own (already-`destroyed()`-hooked) node rather than
stored, so nothing new can dangle.

**Gate: MET.** Machine `9722d23d12a3` (RTX 3060 laptop), binary of 2026-08-23 00:33.

*MAIN VIEW*, windowed, `OMNISIM_WGPU_REPORT`, hatch on vs off on ONE binary:

| world | arm | `draws` | `track` / `muscle` | animation signal |
|---|---|---|---|---|
| `projects/samples/devices/worlds/track.omniworld` | default | **187** | track **160** | `trackP` traverses and **wraps** the closed belt path: `(-0.15, 0.0535, rot 0)` → `(0.012, 0.054, 0)` → `(0.175, 0.042, 0.739)` → `(0.094, -0.064, 3.081)` → `(-0.073, -0.064, -3.142)` → `(-0.194, 0.020, -1.303)` → back to the top straight |
| | `OMNISIM_WGPU_TRACK=0` | 27 | 0 | frozen at the authored `(-0.15, 0.0535, 0)` |
| `projects/samples/geometries/worlds/muscle.omniworld` | default | **19** | muscle **5** | `muscleR` (the spheroid's own bounding radius, which tracks its constant-volume stretch) 0.0724 → 0.0687 → 0.0411 → 0.0660 → 0.0719 → 0.0651 → 0.0494 → 0.0504 → 0.0691 → 0.0718 |
| | `OMNISIM_WGPU_MUSCLE=0` | 14 | 0 | `-1` (no muscle draw) |

⚠ **`trackT` — the belt draw's WORLD translation — is deliberately reported NEXT TO `trackP` and is
not sufficient on its own.** On `track.omniworld` it drifts monotonically in Z (`0.1235` →
`-1.0493` over 900 frames) because the *carrier* is descending, and it did exactly that while the
belt was completely frozen. A world-space number cannot separate "the belt advanced" from "the
robot moved"; `trackP` is the same element in the Track's own path coordinates and is bounded by
the closed path.

*CONVEYOR TEXTURE SCROLL*, same world, same instrument (`uvOff` = the largest `|uvB|` over the
CACHED draw prefix):
 * default → **0.0 → 7.05 → 14.74 → 22.44 → 30.17 → 37.87 → 5.57 (wrap) → 13.22**
 * `OMNISIM_WGPU_TRACK=0` → **0.0 seven samples running**, then 5.71 — the one non-zero is the
   600-frame cache-fallback rebuild re-reading the UV once and freezing again, which is precisely
   the pre-P2 behaviour.

*SENSOR PATH*, headless, via the re-runnable `python scripts/dev/wgpu_dynamic_gate.py --case
track|muscle` against two authored gate worlds + a byte-for-byte control each (the control removes
ONLY the `animatedGeometry` / `muscles` field):

| case | ANIMATION (step 40 vs 320) | RED (hatch=0, gate vs control) | GREEN (default, gate vs control) |
|---|---|---|---|
| track | **411 px** over threshold, max 172 | **0 px, max 0** | 260 px, max 175 |
| muscle | **21 px** over threshold, max 145 | **0 px, max 0** | 21 px, max 145 |

(The muscle's pixel counts are small because the muscle is small in frame — 21 px at max delta 145
is a saturated, unambiguous difference, not a marginal one.)

 * **Zero cost where none of these node types exists:** `beauty_bench` default vs
   `OMNISIM_WGPU_TRACK=0 OMNISIM_WGPU_MUSCLE=0 OMNISIM_WGPU_LEGACY_TEXTURE=0`
   `mean=0.0085 max=8 px>30=0`, inside its own noise floor `mean=0.0203 max=7 px>30=0`.
 * **No regression:** the full `wgpu_dynamic_gate.py` (cloth + granular + track + muscle +
   legacy_texture) is **ALL PASS**, and `scripts/dev/wgpu_sensor_regression.py` is **19/19**.
 * `run-headless --until-finalized` **PASS** on the six new gate worlds, on `track.omniworld`,
   `muscle.omniworld` and on the flagship `warehouse_industrial.omniworld`.

⚠ **THE GATE'S "OmniLight triangle count rises" HALF WAS NOT MEASURED — say so rather than implying
it passed.** Both node types DO carry the flag the bake reads (`Muscle.wrl` defaults `castShadows
TRUE`, and a belt element inherits its source Shape's), so unlike P9's particles they are not
excluded on principle and the count plausibly does rise; nobody ran it. The measurement that was
made is `draws` + the per-kind counts + the two animation signals, which is what the defect this
step exists to catch actually shows up in.

**Left open:**
 * ⛔ **Track PROPULSION is dead under Newton.** With the animation revived the belt spins and the
   wheels turn, and the chassis still does not move: `contactSurfaceVelocity()` has no reader, so
   nothing converts belt speed into ground force. `projects/samples/devices/worlds/track.omniworld`
   therefore shows a stationary robot with a running belt. Separate from rendering, and not fixed
   here.
 * **`tests/api/worlds/track.omniworld` and `track_animated_geometry_muscles.omniworld` FAIL, and
   the failure is PRE-EXISTING** — verified by rebuilding with the `mBodyID` hunk temporarily
   reverted and re-running: byte-identical failures on both arms. Their controllers are ODE-era
   assertion suites (`ts_assert_double_in_delta(position[0], 0.031310, 0.00001)`), which Newton
   cannot satisfy. They belong to R5's "record the baseline before D1.4", not to P2.
 * The `wgpuBeltDraws` composition reproduces upstream's `geom->matrix() * matrix().pseudoInversed()`
   **verbatim, including its oddity**: for a rigid track pose it evaluates to a pure translation by
   `R_track · p` rather than the `p` a "geometry in track-local coordinates" reading would give, so
   a sub-pose offset inside `animatedGeometry` is rotated once too often. That is what WREN draws;
   parity means matching it, and correcting it would move the wgpu image away from the WREN one
   with no test able to say which is right.

### P3 — `Pen` paint (`OmPaintTexture`)
Controller-observable **through a Camera**, so this is not GUI-only.
**Gate:** a Pen world's painted texture is readable from a wgpu Camera image.

**Residue — ✅ CLOSED as an ACCEPTED DEVIATION, degrading loudly (E2, 2026-08-23, `64a678126`).**
On **Box / Cylinder / Cone** the Pen paints into the geometry's SECOND texture-coordinate set (the
cross/sub-rect atlas `pickUVCoordinate(uv, ray, 1)` returns — `OmBox::computeTextureCoordinate`'s
`nonRecursive` arm quarters u and halves v per face; Cylinder and Cone carry their own
sub-atlases), while the wgpu vertex stream carries ONE UV (pos3+norm3+uv2, stride 32) — so the ink
layer is sampled with set-0 UVs and paint appears displaced/scaled on those three geometries. The
real fix is a second UV attribute, i.e. a stride change through every builder in
`OmWgpuMeshAdapter`, every pipeline's vertex layout in `OmWgpuShaders`, the
deformable/granular/track/muscle append paths, **and the WREN mesh readback**
(`wr_static_mesh_read_data` returns one uv set — the fix would ADD WREN C-API surface right before
D1.4 deletes it). Deferred until after the deletion, when the second attribute can live wholly in
wgpu-owned builders. Until then `resolvePenTexture` warns **once, by name**, when a Pen has
actually painted onto a Box/Cylinder/Cone under wgpu; zero cost when no Pen exists (`hasAny()`
gates the call). Plane / Sphere / Capsule / ElevationGrid / meshes paint exactly (their two sets
coincide). Verified windowed on the wgpu main view: `pen_box` fires the warning once; `pen`
(Plane board) fires nothing and its controller exits successfully.
⚠️ **NARROWED by the post-merge pass (2026-08-23, see THE POST-MERGE VERIFICATION PASS below):**
"Plane / Sphere / Capsule / ElevationGrid / meshes paint exactly" does not survive the flipped
default — `pen_plane` fails on ink COLOUR ((205,130,10) vs (206,171,58)) and `pen_mesh` floods
paint (3373 px vs <60 expected), both green-on-WREN at the E3 baseline. The residue is wider than
the Box/Cylinder/Cone atlas class this entry documents; re-scoping it belongs to F2's owner.
✅ **RE-SCOPED by F2 (2026-08-23), and the two cases turned out to be DIFFERENT classes:**
`pen_plane` is NOT a placement defect — placement on the Plane is exact (its location/count
asserts pass untouched) and the ink colour (205,130,10) is the same authored 0xFA8A0A ink
composed through the wgpu pipeline, *nearer the authored (250,138,10) than WREN's (206,171,58)
was* — re-goldened with the old values recorded. `pen_mesh` IS the atlas class: a file-loaded
`Mesh` carries its OWN authored uv set-0, which need not coincide with the paint atlas
`computeTextureCoordinate` derives, so ink floods (measured 3373/4096 px) — **`Mesh` is now in
the warned deviation class** (`resolvePenTexture` names it, once), the paragraph above's
"meshes paint exactly" claim is corrected in the code comment, and the test is re-pinned to the
decided behaviour with WREN's placement constants preserved in its comment for the
post-deletion owner. Exact-placement geometries are: Plane, Sphere, Capsule, ElevationGrid.

### P4 — ✅ `luminosity` + `IBLStrength` PORTED (`c3c990b7c`); ✅ `*IrradianceUrl` RETIRED (P4a, `3d1020907`)
Scoped 2026-08-22, ported the same day. **The scoping above got the mechanism wrong in two places,
and both corrections matter more than the port did.**

⚠️ **CORRECTION 1 — WREN does not have ONE ambient model, it has TWO, and it picks by APPEARANCE
TYPE.** Everything above (and the `0.45f` "the wgpu equivalent" line) assumed one.
 * legacy `Appearance` + `Material` → **`phong.frag`**: `ambient = Lights.ambientLight ×
   material.ambient`, where `Lights.ambientLight` is `Σ over ON lights of (ambientIntensity ×
   color)` (`OmLight::computeAmbientLight`) and `material.ambient` is `(ai,ai,ai)` on a textured
   material or `ai × diffuseColor` otherwise (`OmMaterial.cpp:143-153`). **It never reads
   `Background.skyColor`, and `luminosity`/`IBLStrength` never reach it.**
 * `PBRAppearance` → **`pbr.frag:296-321`**: `skyColor(or cube) × (diffuse + specularEnvBRDF) ×
   (luminosity × IBLStrength)`. **It never reads the lights' `ambientIntensity`.**
This is the whole of why `tests/api/worlds/pen.omniworld` failed its FIRST assertion: its white
board is a legacy `Appearance` under a `PointLight { ambientIntensity 1 }`, so WREN shades it from
the LIGHT (neutral 1.0 → the tonemapped **207**) while wgpu shaded it from the SKY
(`pow((0.4,0.7,1.0), 2.2) × 0.7` → **(85,141,187)**). Both numbers reproduce to the byte from the
two formulas, and so does the previous agent's pure-red control **(85,2,3)** — its 2 and 3 are the
analytic specular IBL, which is ~1e-5 in linear and only becomes visible through `x^(1/2.2)`.

⚠️ **CORRECTION 2 — `atmosphericSky` BAKES AN IRRADIANCE CUBE, so the "flat `skyColor` fallback"
is NOT the branch this tree's PBR worlds take.** `OmBackground.cpp:730` calls
`bakeAndReleaseIrradianceCubeMap()` on the atmospheric path, which lands in
`OmPbrAppearance`'s `irradianceCubeTexture()` slot, so `pbr.frag`'s `cubeTextureFlags.x > 0`
branch is live on **every** `atmosphericSky` world — i.e. essentially all of them. (`OmniSimSky.proto`
documents this and measures it: `IBLStrength 1 → 131`, `3 → 198`, Phong control `0`.) The flat
`skyColor` fallback only applies to a world with `atmosphericSky ""` and no cubemap. **The scalar
port is still exact** — `luminosity × IBLStrength` multiplies whichever base the branch produced —
but reproducing WREN's PBR ambient COLOUR would mean porting that baked cube, which is out of scope
by the same reasoning that retires `*IrradianceUrl`.

**What landed.** One per-draw ambient decision computed in the collector `OmWgpuSceneRenderer.cpp`
that the main view AND the Camera device share, so the two call sites cannot drift (they no longer
each own a copy of the model):
 * `PBRAppearance` draws: the existing analytic hemisphere/specular ambient, now multiplied by
   `Background.luminosity × PBRAppearance.IBLStrength` on **both** terms, exactly as
   `pbr.frag:316-318` does. Defaults are 1 × 1, so an unauthored world is byte-identical.
 * legacy `Appearance` draws: WREN's phong ambient, premultiplied CPU-side and applied to the
   TEXTURE-only albedo (WREN forces `diffuseColor` to white on a textured material, and the
   untextured `diffuseColor` is already inside the CPU product), with the analytic specular IBL
   dropped (phong has none).
 * `OmLight::sceneAmbientLight()` is now the ONE definition of `Σ(ambientIntensity × color)`;
   `computeAmbientLight()` (the WREN writer) calls it.
 * Hatch `OMNISIM_WGPU_WREN_AMBIENT=0` (value-parsed) → mode 0 / scale 1 everywhere = the pre-P4
   renderer byte-for-byte. Read once, in one place.

⚠️ **ONE DELIBERATE DEVIATION FROM WREN, and it is the interesting one.** The phong arm engages
only when the world declares a non-zero light `ambientIntensity`. Every light type defaults it to
**0**, so on a canonical-recipe world WREN's phong ambient is EXACTLY ZERO —
`OmniSimSky.proto` says so in its own banner: *"a Phong `Appearance` renders hard (0,0,0) on every
face the sun does not hit directly"*. Reproducing that would put every URDF-imported robot (they
all ship a plain `Appearance`) at pure black on its shadowed side in the main view, which is the
crushed-black regression the hemisphere exists to fix. So: **where WREN has a value, wgpu now
reproduces it exactly; where WREN has nothing, wgpu keeps its own fill.** The affected set is
enumerable — **172 tracked files** declare a light `ambientIntensity > 0`, of which all but five
are under `tests/`; the five are `TexturedBackgroundLight.proto` (59 worlds use it),
`drive_test`, `construction_site_dev`, `site_env_preview` and `icon_studio`.

**Gate: MET.** Machine `9722d23d12a3`, RTX 3060 laptop, binary of 2026-08-22 21:18.
 * ⭐ `pen.omniworld` on a wgpu Camera, RED/GREEN on ONE binary via the hatch:
   `OMNISIM_WGPU_WREN_AMBIENT=0` → `FAILURE ... (r=85,g=141,b=187)`;
   default → the first assertion PASSES at **(207,207,207)** and the test advances **six**
   assertions before failing at (207,186,186) vs (207,189,189) — a 3/255 residual in the cumulative
   **ink** layer (P3), not the ambient. WREN control on the same binary: **`OK: pen`** (the full
   test), so WREN behaviour is unchanged.
 * `luminosity` / `IBLStrength` land, and land IDENTICALLY: on a `camera_color` probe,
   `lum 1 × ibl 1` → **(103,58,46)**, `lum 2` → **(120,67,54)**, `ibl 2` → **(120,67,54)** —
   byte-identical to the luminosity arm, which is the direct proof they are one premultiplied
   float. (Both move the pixel TOWARD the pinned golden (122,75,65).)
 * No main-view regression: `beauty_bench` hatch-on vs hatch-off `mean=0.0089 max=8 px>30=0`,
   INSIDE its own noise floor `mean=0.0236 max=8 px>30=0`.
 * `run-headless projects/samples/devices/worlds/camera.omniworld --until-finalized` → **PASS**.

**✅ P4a `*IrradianceUrl` retirement DONE (E2, 2026-08-23, `3d1020907`).** The six fields keep
PARSING (`Background.wrl` untouched — the `Solid.immersionProperties` precedent) but the authored
HDR faces are never downloaded or decoded: `loadIrradianceTexture` (and this file's whole
`stb_image` dependency) is deleted, the downloader is back to 6 slots, and `applySkyBoxToWren`'s
"2. Load the irradiance map" block kept only its fallback arm — the specular-irradiance bake from
the image cubemap (64) or a 2×2 `skyColor` cube — which is what every tracked world already took
(the one authoring world, `icon_studio`, has `atmosphericSky "earth"` and returns before the
block, so the retirement changes zero pixels under either renderer; the atmospheric bake is
untouched). One warning per node at `postFinalize` (rendering-independent, fires under
`--no-rendering` too) names the authored fields; a runtime supervisor write to a retired field
warns instead of rebuilding a cubemap nothing reads. No hatch — the retired path is deleted code,
and R6 defines retirement-green as field-parses + named warning. **Gate: MET** —
`icon_studio.omniworld` loads 0 errors with the warning naming all six fields (its `run-headless`
FAIL verdict is solely the pre-existing standalone `icon_creator` tool-controller exit-1; an
isolated probe with that controller neutralized is PASS with exactly 1 warning); `beauty_bench`
PASSes under `--fail-on-warning` with 0 warnings; `wgpu_cubemap_sky.omniworld` (six `*Url` faces,
no atmosphericSky — the restructured bake's live path) loads 0 errors.

**Still OPEN in P4 before E2 was:** `*IrradianceUrl` was NOT retired (now done, above), and the
`luminosity` main-view A/B on an AUTHORED world (`forest.omniworld`, 0.65) was **aborted by the
thermal guard at 78 °C** and not retried — the byte-exact camera evidence above stands in for it.

⚠️ Two adjacent gaps this work surfaced and did NOT fix, both pre-existing:
 1. **A legacy `Appearance`'s `texture` is never uploaded to wgpu at all** — the collector's
    material-map block is inside an `if (pbr)` guard (`OmWgpuSceneRenderer.cpp`), so a
    plain-`Appearance` textured surface renders as flat `diffuseColor` under wgpu. `pen`'s board is
    exactly that (`white256.png`), and it only reaches 207 because that texture IS white.
    ✅ **CLOSED by P11** (below): the texture is uploaded and `baseColor` is forced to white the way
    WREN forces a textured `Material`'s `diffuseColor`. The `fillWrenAmbient` predicate this note's
    neighbour describes was corrected in the same change, so one predicate now drives both.
 2. **`omnisim_log.txt.connect_error.txt` produces FALSE handshake failures.** It is append-only,
    never truncated, and keyed by `OMNISIM_IPC_NONCE` == the simulator **PID** — which Windows
    recycles. A `run-headless` at 22:06 was FAILED by six `run=20520` lines written at **15:11**
    by a different process with the same pid; `omnisim doctor` said "engine + libController
    compatible" throughout, and the run PASSed once the stale file was moved aside.

### P5 — Camera + Lidar post-FX: 3 to PORT, 4 to RETIRE, and the "blocker" was never one — ✅ DONE (`545d01e45`)
Scoped 2026-08-22 **by reading the WREN shaders**, which nobody had done. Every formula below is
quoted from source, so this step is now mechanical.

**The alleged blocker does not survive contact.** `range_noise.frag:47-51` is
`depth + intensity * gaussian(uv, seed)` — **additive Gaussian, σ in ABSOLUTE METRES**, seeded once
per frame from `OmSimulationState::time()` and hashed per pixel. **Stateless**: a pure function of
(uv, time, intensity), no RNG state, no history, no GPU dependency. It is a one-line CPU port.
`depth_resolution.frag:16-19` is `floor(r/res + 0.5)*res` — **round-half-up in metres**, also one
line. `color_noise.frag:45-47` is additive Gaussian **per channel** (three independent seeds, alpha
untouched) with σ = `noise` in normalised channel units, then **clamped to [0,1]** — the clamp must
be reproduced or darks and brights will differ. `motionBlur` is
`mix(scene, last, blend)` with **`blend = 0.005 ^ (samplingPeriod_ms / motionBlur_ms)`**
(`OmAbstractCamera.cpp:899-903`, verified numerically: 500 ms at a 32 ms period → 0.712, and
0.712^15.6 = 0.005 ✓).

⚠️ **TWO DOCUMENTATION BUGS FOUND, both of which would mislead anyone authoring noise:**
1. `docs/reference/lidar.md:141` and `docs/reference/rangefinder.md:75` say "a value of 1.0
   corresponds to a gaussian noise having a standard derivation of `maxRange` meters." **The shader
   never multiplies by `maxRange`.** That statement is only true when `maxRange == 1.0` — the Lidar
   `.wrl` default, which is presumably how it was written and never re-checked. The one live user
   (`projects/samples/devices/worlds/lidar.omniworld:368`, `maxRange 8`, `noise 0.1`) gets
   **σ = 0.10 m, not 0.8 m**. Port must match the SHADER (the world was tuned against it) and fix
   the docs.
2. `docs/reference/camera.md:111` says the noise mask is applied BEFORE the gaussian noise. The
   `drawingIndex` order (`OmWrenRenderingContext.hpp:93-106`, sorted ascending at
   `src/wren/Viewport.cpp:95-102`) puts `NOISE_MASK` **after** `COLOR_NOISE`. The code is
   unambiguous; one of the two has been wrong for a long time.

⚠️ **A side effect nobody had costed: the range-noise pass is also a RANGE CLIP.** With
`noise > 0`, anything outside `(minRange, maxRange)` — including the background, which is exactly
`maxRange` — is rewritten to `FLT_MAX` (+inf). The wgpu Lidar arm **clamps** instead
(`OmLidar.cpp:255`). That is a pre-existing WREN/wgpu divergence independent of noise, no test pins
either behaviour, and any noise port inherits the decision.

⚠️ **The authored counts this runbook used were wrong** — corrected by enumeration:
camera `noise` **2** files (not 9; of the nine, one is an `InertialUnit`, five are `noise 0`, one is
a Lidar), Lidar `noise` **1**, `resolution` **0** on any Lidar/RangeFinder, `motionBlur` **3**,
`focus` **2**, `noiseMaskUrl` **1**, `lens` **0**, `lensFlare` **0**.

**PORT (no decision needed — all three are arithmetic on the readback):** Lidar/RF `resolution`
(one line, closes half of P6 for free), Lidar `noise` (**highest urgency of the eight**: it is the
only one failing *silently with no warning at all*), camera `noise` (shares all its code with the
Lidar one). Then `motionBlur` CPU-side, documenting that it blends post-tonemap where WREN blends
pre-tonemap.
**RETIRE, fields kept parsing + warned by name:** `lens` (0 users), `lensFlare` (0),
`focus` (2 — and note the controller-facing focus API is INDEPENDENT of the DoF render and must keep
working; only the blur retires), `noiseMaskUrl` (1, and that user is a demo of the feature itself).
⚠️ `lens` deserves a specific warning wording: it is a **calibration** model, not an effect, so a
retired-`lens` device should say it will produce an UNDISTORTED image, not that "an effect is
missing".

⚠️ **PREREQUISITE, and it gates every Lidar item above:** `resolvesToWgpuBackend()`
(`OmAbstractCamera.cpp:244-268`) returns false for any node with no `renderBackend` field, and
`Lidar.wrl` has none — so a Lidar neither warns nor takes the wgpu envelope. **The same one change
also fixes the `OMNISIM_NO_GL` fatal at `:303-313`.** Land it BEFORE the Lidar ports.

**Gate:** each ported effect measurable in the output; each retired one warned by name with the
field still parsing; both reference-doc bugs fixed in the same change.

**Met (2026-08-23, `545d01e45`, machine `9722d23d12a3` RTX 3060 laptop; every engine run under
`thermal_guard`, peak 64 C against the 75 C ceiling).** All four ports measured, all four
retirements warning by name, and every new regression case proven RED first via the exact-revert
hatch `OMNISIM_WGPU_SENSOR_POSTFX=0` (VALUE-parsed; =0 reproduces the pre-port output, including
restoring the silent-post-FX warning):

  * **Lidar/RF noise** — smoke world (authored 0.1, fov 1.2): residual std vs the analytic
    0.70/cos(θ) profile = **0.0005 m red → 0.1120 m green**, and the 18 miss columns
    **10.0 (clamp) red → +inf green**. On the LIVE `projects/samples/devices/worlds/lidar.omniworld`
    (rotating, 6-layer, wide-FOV — the R5l path, log-verified): median per-column temporal std
    **0.1018 m** on the wgpu arm vs **0.0939 m** on the WREN reference arm and **0.0349 m** under
    the hatch (that floor is rotating-resample jitter, not noise). σ = the authored value in
    ABSOLUTE metres, confirmed end to end.
  * **The +inf-vs-clamp DECISION, stated:** with `noise > 0` the port reproduces WREN's range clip —
    outside the open (`minRange`, `maxRange`) interval, background included, reads +inf. The
    no-noise wgpu clamp is deliberately untouched (pre-existing divergence, still unpinned).
  * **Lidar/RF resolution** — centre **0.7000 red → 0.7500 green** at `resolution 0.25`, 0/64
    columns (and 0/3072 RF pixels) off the grid; quantization runs AFTER noise (RF case shows
    3 distinct levels around the 0.70 face at σ 0.1).
  * **Camera noise** — per-channel std on the flat face **0.46–1.95 bytes red → 11.77/12.19/12.16
    green** vs 12.75 expected (noise 0.05 × 255), alpha std 0 both arms.
  * **motionBlur** — supervisor removes the box: centre red channel decays
    **21→69→101→122→…→160** (bg 161); first-frame ratio 0.657 vs the formula's 0.6546. Red arm
    snaps in one frame. Blends post-tonemap LDR where WREN blends pre-tonemap HDR — a real,
    documented deviation, not parity. Bit-exactness with WREN is not achievable (float32 `sin`
    differs across drivers) and was never the gate; the ports are statistically correct and
    correctly scaled.
  * **Retirements** — a 4-camera evidence world loads with 0 errors and warns each by name:
    `lens` says the device "produces an UNDISTORTED image" (calibration wording as required),
    `focus` names the blur only and says the controller focus API keeps working, `lensFlare` and
    `noiseMaskUrl` by name. The shared warning now names the node type (`Lidar`, not "Camera").
  * `wgpu_sensor_regression.py` **24/24** (19 original cases byte-unchanged — the unaffected-world
    control — + 5 new red-capable cases), new `--engine-mode` flag so the suite runs without
    `--mode=fast` on thermally constrained boxes; `run-headless --until-finalized` PASS on
    `lidar.omniworld` and `camera_wgpu_noise_smoke.omniworld`. Reference docs: `renderBackend`
    documented on lidar.md/rangefinder.md, wgpu port/retire status on camera.md (the two doc bugs
    above were already fixed 2026-08-22).

### P6 — Lidar: NOT the rendering job this runbook first claimed — ✅ DONE (`545d01e45`, with P5)
⚠️ **My original P6 text was WRONG and is corrected here.** It said the wgpu arm "returns false for
rotating heads and `fov > 6.3`" and cited `OmLidar.cpp:158`/`:383`. Measured 2026-08-22:
 * `:158` is a **dispatch**, not a capability limit — `mIsActuallyRotating` there routes control to
   `renderRotatingWindowViaWgpu` (`:445`) / `renderRotatingWideFovWindowViaWgpu` (`:599`).
 * the quoted "multi-layer + wide-FOV deferred → WREN" comment now lives at `:284-285` and is
   **unreachable dead code** — the wide-FOV branch at `:182` returns on every path, so `:283` is
   only reached when `fov <= 1.4`.
 * **Lidar rendering on wgpu has been complete since `a643a464e` (2026-05-30)**, and
   `engine-migration-plan.md` says so. **ZERO of the 26 tracked files declaring a `Lidar {`** fall
   into any remaining fallback set.
This is the campaign's recurring failure mode turned on itself: a plan written from line citations
that had gone stale. Re-read the code before trusting any step in this file.

**What P6 actually is:**
1. ⛔ **`noise` and `resolution` are applied on WREN and DROPPED on wgpu — silently, with no
   warning at all.** They are real post-processing effects between the render and the controller's
   floats (`OmWrenRangeNoise`/`OmWrenRangeQuantization` → `mResultFrameBuffer` →
   `copyContentsToMemory`, the exact call the wgpu arm replaces at `OmLidar.cpp:996`). The
   warn-by-name machinery added today does NOT cover them, because
   `resolvesToWgpuBackend()` returns false for any node with no `renderBackend` field
   (`OmAbstractCamera.cpp:244-250`) and `Lidar.wrl` has none. One live authored user:
   `projects/samples/devices/worlds/lidar.omniworld:368` (`noise 0.1`). **No test pins the
   difference.**
2. Delete the dead early-return at `:284-285` and the six stale comment blocks that still describe
   the single-layer/narrow-FOV envelope.
3. Two real gate holes: the wgpu arms never check `projection` (`Lidar.wrl` defaults
   `"cylindrical"`; a `"planar"` Lidar would silently get cylindrical columns — latent, zero
   tracked users), and the `fov > 6.3` ceiling is asymmetric between the static and rotating entries.
4. ⚠️ **The thing that actually ties Lidar to WREN under F1 is not the renderer — it is a FATAL.**
   `OmAbstractCamera.cpp:310-313` makes a Lidar under `OMNISIM_NO_GL` a hard `OmLog::fatal`,
   precisely because `resolvesToWgpuBackend()` is false for it. Giving Lidar a `renderBackend`
   field (or special-casing it) fixes the fatal AND makes it warn for anything unported.

**Gate:** `noise` + `resolution` measurable in the wgpu output, `wgpu_sensor_regression.py` extended
with a case that can go red for them, and a Lidar loading under `OMNISIM_NO_GL` without a fatal.
⚠️ Add a 3-frustum seam case at `fov 3.1` first: the regression only asserts seam continuity at
`fov 2.0` / 2 frustums, while **all 12 omnitug500 Lidars ship at 3.0–3.1** — untested and shipping.

**Met (2026-08-23, `545d01e45`) — except the fov-3.1 seam case, still open.** The prerequisite fix
is the field, not a special case: `Lidar.wrl` + `RangeFinder.wrl` now declare `renderBackend`
(default `"wren"`, so the 26 tracked Lidar worlds are untouched), and the wgpu arms select on
`OMNISIM_LIDAR_WGPU=1` **or** the resolved field **or** a wrenless setup. What that unblocked,
each verified:
  1. **Item 1 (the silent drop) is closed by the P5 port** — evidence above, including the live
     world at σ 0.1018 m through the rotating wide-FOV path.
  2. **The warnings fire.** A `projection "planar"` Lidar on wgpu now warns by name
     ("cylindrical-only; it would silently produce cylindrical columns") via a Lidar override of
     `wgpuUnsupportedFeature` — the base class's planar-only test is exactly inverted for a Lidar,
     so the override was mandatory, not cosmetic. Item 3's warning half is therefore done; the
     *gating* half (refusing to render planar as cylindrical) is deliberately not, matching the
     warn-don't-fail contract of every other degraded effect.
  3. **The `OMNISIM_NO_GL` fatal is closed.** A `renderBackend "wgpu"` Lidar under `OMNISIM_NO_GL`
     loads and sets up wrenless ("set up on the wgpu render backend with no OpenGL context" in the
     log, engine alive) — `setupWrenless()` latches the `mActual*` values `createWrenCamera()` used
     to latch, and every `mWrenCamera` call site is null-guarded. ⚠️ Honest caveat: under NO_GL the
     sensor render loop does not drive images on this box for ANY device (the Camera control
     produced the same frozen zero buffer; the engine's own NO_GL banner says "will produce NO
     data") — the gate was the fatal, and the fatal is gone. In a NORMAL session the field alone
     (no env var) drives the full wgpu range path: the field-selected smoke run PASSed the analytic
     profile with `OMNISIM_LIDAR_WGPU` unset.
  4. The dead early-return / stale comments at `:284-285` were NOT touched this lane (they are
     dead, not wrong-behaving); they go with the next Lidar edit.
Still open from this entry: the fov-3.1 / 3-frustum seam case, the planar *gating* decision, and
the stale-comment sweep.

### P7 — device HUD insets, labels, status overlays — ✅ DONE (`f81958391`)
`OmWrenTextureOverlay` / `OmWrenLabelOverlay`. One of the five GUI regressions the 2026-08-19
main-view flip shipped.
**Gate:** a camera-device inset is visible in a wgpu main-view frame, proven by a pixel A/B against
its own noise floor (`render_ab.py --noise-floor` FIRST — on an animated world the floor can exceed
the effect).

**Met, and by a stronger instrument than the gate asked for.** A pixel A/B was not used, because
`render_ab.py` is *structurally* blind here: its MAINVIEW dump reads back INSIDE the scene render,
upstream of the overlay passes, so a noise-floor comparison would have measured nothing either way.
`OmWgpuRenderTarget::readbackOverlayOutput()` reads the texture the overlay passes actually wrote,
and `OMNISIM_WGPU_HUD_CHECK=<path>` compares it against the device's own image buffer.
Measured, machine `9722d23d12a3`, windowed 1890x1121, `projects/samples/devices/worlds/camera.omniworld`:

  * `hudCalled=1 hudQuads=2 hudOk=1` sustained;
  * `samples=49 matched=49 maxErr=0 meanAbs=0.000` — every sampled pixel inside the drawn inset is
    **byte-identical** to the Camera's image (not a tolerance: `maxErr` is 0);
  * **`ctlMatched=0`** — the same samples read one pass earlier (i.e. with the HUD off) match on
    ZERO pixels. The instrument is red-capable and the effect is total.
  * Hatch `OMNISIM_WGPU_HUD=0` re-measured: `hudCalled` stays 0 and no verdict file is written.

What landed: `OmWgpuRenderTarget::drawOverlayQuads()` + `OmWgpuHudQuad` + the `kHudQuad` WGSL (a
screen-space textured quad with per-quad tint, sharing the overlay-lines private output so both
compose into one present), and `gui/OmHudOverlay.{hpp,cpp}`, which reads the SAME
`left()/top()/width()/height()` accessors `isInside()` hit-tests against — so the drawn inset and
the clickable inset are one rectangle by construction. Labels are rasterised with QPainter.
Still open: the close/resize ICONS on the inset frame are not drawn (their hit regions still work).

### P8 — manipulator gizmos — ✅ DONE (`152f0f389`)
Collectors with a verified hit-test already exist in `OmWgpuView.cpp` and are deliberately not
drawn, because the pane's gizmo is a second gizmo with its own hit test. Resolve that properly:
one gizmo, drawn where its hit regions actually are.
**Gate:** the drawn handle and the draggable region coincide, measured — not eyeballed.

**The correct answer was "draw the MAIN VIEW's own gizmo geometry", not "reuse the pane's".**
The pane's collectors are wired to the PANE's hit test (fixed `kHandleLen` 0.25 m + a
screen-distance threshold, in `OmWgpuView`'s own mouse handlers). The main view's gizmo is
`OmTranslateRotateManipulator`, whose hit regions are decided by `OmWrenPicker` — a GL picking
render of the real `arrow.obj` / `circular_arrow.obj` meshes through
`resources/wren/shaders/handles.vert`, which still works under a wgpu main view because it renders
into its own framebuffer. `gui/OmGizmoLines.{hpp,cpp}` therefore draws that manipulator, reading
`wr_transform_get_matrix()` on the very handle transforms the picking renderables hang off (== the
shader's `modelTransform`) and `handleScreenScale()` (== its `screenScale`), and reproducing the
shader's scalar term for term.

**Gate met, measured with the LIVE picker** (`OMNISIM_GIZMO_HITTEST_CHECK=<path>` grids each
handle's screen bbox and asks both "inside the drawn triangles?" and "does `OmWrenPicker` report
this handle here?"). Machine `9722d23d12a3`, same world, `DEF GREEN_BOX`, `w = 0.131786 m`:

| axis | kind | tris | both | drawnOnly | pickedOnly | IoU | centroid delta |
|---|---|---|---|---|---|---|---|
| 0 | translate | 192 | 48 | 0 | 0 | 1.0000 | 0.00, 0.00 px |
| 0 | rotate | 864 | 40 | 2 | 0 | 0.9524 | -0.02, -0.57 px |
| 1 | translate | 192 | 39 | 0 | 0 | 1.0000 | 0.00, 0.00 px |
| 1 | rotate | 864 | 41 | 0 | 0 | 1.0000 | 0.00, 0.00 px |
| 2 | translate | 192 | 44 | 14 | 0 | 0.7586 | 1.51, 1.45 px |
| 2 | rotate | 864 | 25 | 16 | 0 | 0.6098 | 5.08, 0.47 px |

**`pickedOnly = 0` on all six** (nothing invisible that grabs) and **`drawnPickedNothing = 0` on
all six** (nothing visible that does not respond). Every one of the 32 `drawnOnly` samples is
`drawnPickedOther` — a pixel where this handle overlaps another handle IN FRONT of it, which the
depth-testing picker correctly gives to the nearer one. The two handles that overlap nothing in
this view score exactly 1.0000 / 0.00 px. Drawing half, with its control arm: gizmo ON
`ovBatches=5 ovVerts=986`, `OMNISIM_WGPU_GIZMO=0` `ovBatches=2 ovVerts=48`.

Still open: the RESIZE manipulator (`OmResizeManipulator`, `HANDLES_RESIZE`) is not drawn, and
neither are the rotation line / double-arrow that appear mid-drag.

---

### P9 - `OmGranularGroup` particles (scoped 2026-08-22)
N `WrRenderable`s sharing ONE `wr_static_mesh_unit_sphere_new`, each with its own transform, fed
per physics step from `mHostPositionBuffer` - a plain CPU `std::vector<float>` of `(x,y,z,radius)`
filled by a CUDA device->host copy (`OmGranularGroup.cpp:1207-1226`). **The data is already CPU-side
in exactly the form a wgpu model matrix wants**, so the carrier is `OmWgpuSolidDraw.modelMatrix16` +
the cached `acquireUnitPrimitive(PrimitiveKind::UVSphere)` the Sphere node already uses
(`OmWgpuSceneRenderer.cpp:634`) - NOT the instanced pass (no lighting/materials/shadows) and NOT the
deformable vertex-stream path (granular vertices never change, only transforms).

⚠️ **THE TRAP, AND IT INDICTS THE EXISTING DEFORMABLE FIX:** `collectDeformableDraws` has exactly
ONE caller in the tree - `OmView3D.cpp:1977`, the main view. `OmCamera.cpp:585` calls only
`collectWorldDraws`. **So Cloth and SoftBody are ALREADY main-view-only under wgpu** (which is
exactly what P1 exists to fix), and a P9 that copies that pattern verbatim would make particles
invisible to every sensor - a REGRESSION AGAINST WREN, not parity, since
`OmGranularGroup.cpp:1171` sets `VM_REGULAR` and every sensor mask intersects it.
⚠️ Second complication: `OmGranularGroup` derives from `OmBaseNode`, **not** `OmSolid`, so
`collectWorldDraws` ("walk the top Solids") can never reach it. It needs a registry or a targeted
walk. Third: 10,000 particles in `warehouse_husky_granular_massive.omniworld` becomes 10,000 draws -
unmeasured, and the instanced path is the fallback if it bites.

**Blast radius is small and worth knowing before prioritising: all 4 declaring worlds are tests or
benchmarks, ZERO shipped demos** (the one showcase world that had particles removed them). And none
of them looks at pixels - the lane-4 probe checks scene-tree presence and says "the particles are
simulated is UNMEASURED"; the CUDA tests parse log text. **So if P9 is skipped, nothing catches it.**

#### ✅ DONE (with P1, `4ac917c0c`) — and it did NOT copy the deformable pattern

Landed in the same commit as P1 for the reason the scoping above predicted: a P9 that reached only
the main view would have been a regression against WREN, and a P9 that copied
`collectDeformableDraws` verbatim — process-global upload flag included — would have inherited the
freeze. Both are avoided structurally: `OmWgpuSceneRenderer::collectDynamicDraws()` is the ONE entry
point every render path calls (main view + `OmAbstractCamera::collectWgpuDraws`, which all three
sensor devices share), and granular carries no per-cache epoch at all *because it needs none* — a
particle's per-frame datum is its model matrix, written into the caller's own `modelStorage`.

**What landed.** A registry on `OmGranularGroup` (ctor/dtor maintained, `anyGranularGroups()` is an
empty-test that constructs nothing, so a world with no particles pays one static bool + one vector
test); one draw per particle carrying the cached unit UV sphere and a TRS model matrix from
`mHostPositionBuffer`; WREN's material term for term (sand-yellow diffuse, specular 0.30,
`castShadows` FALSE, receive-shadows on). `updateRenderingFromHost()` was split so the GPU→host copy
(`refreshHostPositions()`) is **no longer gated on WREN objects existing** — it used to be, harmless
only because `createWrenObjects()` runs on every renderer, and a WREN-less session would have been
handed a buffer of zeros.

⚠️ **ONE DELIBERATE DEVIATION FROM WREN, and it is the right way round.** With no live CUDA state
`mHostPositionBuffer` is all zeros and **WREN draws `count` spheres piled at the world origin** — a
scene nobody authored. `OmGranularGroup::wgpuParticles()` declines to draw instead. A GranularGroup
without CUDA is inert by design and the engine already says so once (`CUDA_NOT_AVAILABLE`);
reproducing the phantom heap would be reproducing a bug.

**Gate: MET.** Machine `9722d23d12a3`, RTX 3060 laptop (CUDA 12.1, `NVIDIA GeForce RTX 3060 Laptop
GPU` — so the CUDA-unavailable branch is *not* exercised here and is reasoned, not measured).
Gate world `projects/samples/demos/worlds/rendering/camera_granular_wgpu_smoke.omniworld`.
 * **SENSOR** (`wgpu_dynamic_gate.py --case granular`): default vs `OMNISIM_WGPU_GRANULAR=0` on the
   same world → **313 px over threshold, max 157**; a hue count reads **252 sand pixels vs 0**.
   ANIMATION: step 20 (the seeded cloud still falling) vs step 150 (settled at one radius) →
   **883 px changed**.
 * **MAIN VIEW**, windowed 1896×1113: `draws` **2 → 252**, the new `granular=` field of
   `OMNISIM_WGPU_REPORT` **0 → 250**.
 * ⚠️ **The gate's "OmniLight triangle count rises" half is NOT met, and that is CORRECT.** The bake
   skips every draw with `castShadows == false` on principle, and WREN sets exactly that on the
   particle renderables — so particles are excluded from the GI field on both renderers and the
   count stays **12 tris (the floor box) in both arms**. Flipping `castShadows` to satisfy the gate
   wording would have broken parity *and* put 250–10,000 spheres into the shadow map.

⚠️ **THE 10,000-PARTICLE MEASUREMENT, AND IT IS NOT FREE.** `warehouse_husky_granular_massive.omniworld`,
same machine, windowed 1896×1113, `OMNILIGHT=0`, interleaved A/B on one binary:

| arm | draws | granular | collectMs (f0 → steady) | renderMs (f0 → steady) | GPU temp in ~20 s |
|---|---|---|---|---|---|
| default | **10016** | 10000 | 105 → **0** | 79 → **8** | **77 °C, run KILLED by the guard** |
| `OMNISIM_WGPU_GRANULAR=0` | 16 | 0 | 68 → 0 | 95 → **0** | **62 °C** |

So the collect is free (the per-particle matrix build is under the 1 ms timer resolution) and the
**render costs ~8 ms/frame for 10,000 lit draws** — a ~125 fps ceiling from the particles alone, not
a stall, but not nothing. The **thermal** cost is the sharper finding: 77 °C against 62 °C for the
identical world with only the hatch changed, on a laptop with a 75 °C ceiling. ⚠️ n=1, the steady
rows are frames 0 and 100 only (the run was killed before it settled), and `renderMs` is integer
milliseconds. **Follow-up, not shipped:** route the particle pass through an instanced draw with a
lit/shadowed pipeline (the existing `clearAndDrawInstanced` has no lighting or materials, so it is
not a drop-in). Until then `OMNISIM_WGPU_GRANULAR=0` is the documented escape for the massive world,
and no shipped demo declares a GranularGroup at all.

### P10 - video recording — ✅ DONE (E2, 2026-08-23, `3c4a55302`)
⚠️ **CORRECTION: this does NOT fail silently today.** `OmView3D.cpp:1767-1778` explicitly bails out
of the wgpu path while recording, with a comment saying movies record through WREN "until a
wgpu-native capture exists". So recording works today by *falling back*. **It goes silent when WREN
is deleted and that fallback has nothing to fall back to.**

**And the replacement already exists and already ships.** `OmView3D::grabWindowBufferNow()`
(`:3484-3518`) is a wgpu-aware grab that forces a synchronous same-frame readback and returns a
deep-copied `QImage` already in `Format_RGB32` - the exact format `FrameWriterThread` wants - and
**`--stream`'s mjpeg feed already uses it** (`OmView3D.cpp:3473-3480`). So P10 is mostly "make the
recorder use the mechanism `--stream` has been using all along", not new capability:
drop `initVideoPBO`/`completeVideoPBOProcessing`, feed `writeSnapshot` from `grabWindowBufferNow()`,
take a `QImage` instead of a raw pointer, **drop the flip** (it is a GL bottom-left-origin artifact
wgpu does not have - corroborated by `OmWgpuGlBlit.cpp:59` and by `grabWindowBufferNow` doing no
flip at all), and delete the `isRecording()` bail-out.
The async PBO ping-pong is **latency-hiding only, not correctness** - a recording frame IS a grab,
and the recorder already re-paces the whole simulation. `OmWgpuRenderTarget` has a ready-made
two-buffer async arm (`asyncReadback`, `mReadbackBuffer`/`mReadbackBufferB`) if a synchronous map
ever proves too slow.
⚠️ Keep the deep copy: WREN's emit hands out a pointer that is unmapped the instant the signal
returns, which is why `FrameWriterThread` copies in its *constructor*. `grabWindowBufferNow` already
returns `img.copy()` - do not "optimise" that away.
⚠️ **Test coverage would not catch a regression**: the one test asserts only that `movie.mp4`
EXISTS. Twenty black frames pass. Only the zero-frames case trips `movie_failed()`. Whether that
test even runs in CI is unverified.

#### ✅ DONE (`3c4a55302`) — what landed, exactly as scoped, plus the DPR answer

1. The `isRecording()` bail-out in `renderMainFrameViaWgpu` is deleted; recording no longer pins
   the main view back to WREN. Hatch **`OMNISIM_WGPU_VIDEO`** (value-parsed, `=0` = OFF) restores
   the bail-out + WREN PBO capture exactly.
2. `initVideoPBO()` / `completeVideoPBOProcessing()` are **virtual**; the `OmView3D` overrides
   decide the mode ONCE per recording (at the recorder's init call) and skip the GL PBO ring on
   the wgpu path. The WREN implementations are byte-untouched and stay until D1.4.
3. `OmVideoRecorder::requestSnapshotIfNeeded` branches per frame: wgpu → a new
   `FrameWriterThread` **QImage arm** fed from `grabWindowBufferNow()` (the `--stream` mechanism);
   WREN → the untouched PBO ring. The QImage arm has **no flip** (the flip undoes GL's
   bottom-left origin; `grabWindowBufferNow` returns top-down rows on both its arms — the WREN
   fall-through flips internally) and **keeps the deep copy in the constructor, on the caller's
   thread** — the WREN fall-through grab wraps a member buffer the next grab overwrites.
4. If wgpu degrades to WREN mid-recording (a sticky failure), `grabWindowBufferNow` falls through
   to the WREN GL buffer by itself — recording continues either way.
5. **DPR != 1 is no longer ambiguous:** the wgpu grab is PHYSICAL pixels (`width()*DPR`); the
   writer thread scales any frame whose size differs from `mVideoResolution` back to it
   (off the GUI thread), so the encoded video is always exactly the requested resolution. DPR=1
   verified on this box; DPR!=1 untested but specified. The WREN arm keeps its historical
   logical-size behaviour untouched. ⚠️ The movie caption overlay (`OmWrenLabelOverlay`) is
   WREN-only and does not appear in wgpu recordings.
   ⚠️ **Post-merge pass (2026-08-23): on a NUE (Y-up) world the wgpu-recorded movie is SKY-ONLY**
   (two different NUE worlds → byte-identical mp4s of the night sky; the shipped
   `supervisor_start_stop_movie` test world is NUE and its existence-only assertion stays green
   through this). The Y-up-viewpoint-under-wgpu class from the 2026-08-19 flip, not a recorder
   defect — the recorder was re-proven on an ENU probe (falling ball visible, 177k px inter-frame
   delta). See THE POST-MERGE VERIFICATION PASS.

**Gate: MET — pixel sample, not existence** (machine `9722d23d12a3`, RTX 3060, windowed, thermal
peak 66 °C). RED first, one binary via the hatch: `OMNISIM_WGPU_VIDEO=0` → the
`movie recording is captured through the WREN renderer` line fires (the old fallback, alive) and
WREN encodes an mp4. GREEN (default): no fallback line; a beauty_bench + falling-ball probe world
recorded **141 frames @ 640×360** — frames non-black (mean 93–95), non-uniform (std 45–56,
287–421 distinct quantized colours), and frame 2 vs frame 100 differ on **79,472 px** (>20/255)
because the ball fell. The cross-arm differential is the proof the green pixels are the wgpu pane:
green vs red frame 100 differ on **93% of pixels**. `wb_supervisor_movie_*` verified end-to-end:
`tests/api/worlds/supervisor_start_stop_movie.omniworld` windowed → controller exited
successfully (is_ready/failed status flow), `movie.mp4` written, 19 frames, non-black, first/last
differ.

### P4b - the ambient model: DONE, and the root cause was not what anyone expected
Landed `c3c990b7c`. **The premise everyone (including this runbook) was working from was wrong:
WREN does not have one ambient model. It has TWO, selected by APPEARANCE TYPE.**

| appearance | WREN ambient | reads |
|---|---|---|
| legacy `Appearance` + `Material` (`phong.frag:82,181,214`) | `Lights.ambientLight x material.ambient`, then x the texture | the LIGHTS' `ambientIntensity`. **Never** `skyColor`, never `luminosity`/`IBLStrength` |
| `PBRAppearance` (`pbr.frag:296-321`) | `skyColor(or cube) x (diffuse + specEnvBRDF) x (luminosity x IBLStrength)` | **Never** the lights' `ambientIntensity` |

wgpu applied the **sky** model to a surface WREN shades from the **lights**. The arithmetic closes
exactly: `pen`'s board is a legacy `Appearance` under `PointLight { ambientIntensity 1 }`, so WREN
gives linear 1.0 -> `(1-e^-1)^(1/2.2)*255` = **207.01 -> 207**, while wgpu gave
`pow((0.4,0.7,1.0),2.2)*0.7` -> **(85.07, 140.9, 186.7) = (85,141,187)** - the reported failure to
the byte, and the earlier pure-red control (85,2,3) falls out of the same model. **The `0.45` scale
and the `pow(...)*0.7` arm were never the cause** and never even entered on that world.

**Proven, all on ONE binary via the hatch `OMNISIM_WGPU_WREN_AMBIENT` (value-parsed):**
`pen`'s first assertion goes (85,141,187) -> **(207,207,207)** and the test now advances **six**
assertions before failing on a **3/255 residual in the cumulative INK layer** (P3's remaining Box-UV
issue), not the ambient. `luminosity` and `IBLStrength` both land and are **byte-identical in
effect** (`lum2` and `ibl2` each give (120,67,54)) - the direct proof they are one premultiplied
float. beauty_bench main-view A/B sits **inside its own noise floor**.

⚠️ **`camera_color` is REFRAMED, and this matters for R5:** it is unchanged by this fix (that world
is `luminosity 1 x IBLStrength 1`), residual (-19,-17,-19) vs the pinned golden - **but the WREN arm
on this box reads (59,36,31), residual (-63,-39,-34). The golden is reproduced by NEITHER renderer
here, and wgpu is the CLOSER of the two.** Stop treating `camera_color` as a wgpu defect.

⚠️ **One deliberate deviation from WREN, and it is the right call but must be known:** the phong
ambient arm engages only when a light declares non-zero `ambientIntensity`, and **every light type
defaults it to 0** - so WREN's phong ambient is *exactly zero* on a canonical-recipe world
(`OmniSimSky.proto` says so: "a Phong `Appearance` renders hard (0,0,0) on every face the sun does
not hit directly"). Reproducing that faithfully would black out the shadow side of every URDF robot.
The port declines to. Affected set is enumerable: 172 tracked files, all but five under `tests/`.

### P11 - legacy `Appearance` textures on wgpu - ✅ DONE (with P2, `adc7cc0a6`)
Every texture-upload path in `OmWgpuSceneRenderer.cpp` is gated on `pbr` (`:857`, `:903`, `:1041`,
`mapSlots` at `:1082`), and a legacy `Appearance` has no `baseColorMap` - so **its texture never
reaches the GPU** and the surface renders flat `Material.diffuseColor`, *lit*, where WREN shows the
image. Concretely today: `elevation_grid_rotation` renders flat PINK instead of tiled grass,
`track` flat brown instead of `track.png`, `camera_noise_mask` flat WHITE instead of a
colour-checker chart.

⚠️ **My 255-file figure was the substring trap and is retracted: `PBRAppearance {` CONTAINS
`Appearance {`.** Brace-matched with a negative lookbehind, the real count is **41 nodes in 33
files, and ALL 33 are under `tests/` - ZERO shipped worlds**. Every non-test legacy `Appearance` in
the tree is untextured (axis markers, a bare cylinder). Of the 41: **18 use `white256.png`, so the
failure is invisible**; 9 are empty `ImageTexture { }` (Display targets); only **13** carry a real
non-white image.

**The port is small, and half of it already shipped.** No new pipeline variant is needed - there is
one scene pipeline, `textureView` is bind-group entry `@1` with a white default, so a legacy
Appearance is *already on* the textured pipeline and merely sampling white. And P4b's shader work
was written for exactly this case: `OmWgpuShaders.cpp:535`'s `albedoTexOnly` carries a comment
naming WREN's phong texture rule verbatim. What remains is CPU-side: populate `draw.textureView`
from `app->texture()->image()` (guaranteed present - `OmImageTexture.cpp:332-334` loads CPU pixels
*because* wgpu needs them), and **force `baseColor` to white when a LOADED texture is present**,
mirroring `OmMaterial.cpp:151-152` (WREN also zeroes specular and shininess). `TextureTransform` is
already carried exactly by the existing `uvA`/`uvB` affine - no change.

⚠️ **A live latent bug to fix in the same change:** `fillWrenAmbient` (`:387`) branches on
`app->texture() != NULL` (*declared*) while WREN branches on `texture()->wrenTexture()` (*loaded*).
One predicate must drive both the white-forcing and the ambient, or the 9 empty-`ImageTexture`
nodes get `(ai,ai,ai)` ambient against a `diffuseColor` albedo - **which is neither renderer**.

⚠️ **NO EXISTING TEST CAN CATCH THIS, and one has to be authored.** `camera_color_spherical` and
`camera_noise_mask` both put a colour-checker chart on a legacy Appearance and assert 12 pixels at
+-2 - but their Cameras take `Camera.wrl`'s `"wren"` default, so they never reach the wgpu collector.
The only tracked world with a wgpu Camera is `pen_wgpu_camera.omniworld`, whose texture is
`white256.png` - **structurally incapable of going red for P11.**

⭐ **P11's deadline is F2, not the deletion - this is the sequencing point.** F2 re-goldens the
`tests/api` cameras against wgpu, and two of those worlds put a colour-checker chart on a legacy
Appearance. Re-goldening them before P11 lands would **bake "flat 255,255,255" into a pinned golden
and permanently destroy the evidence that a texture was ever there.** P11 must land before F2 begins.

#### ✅ DONE (with P2, `adc7cc0a6`)

**What landed, and both halves were required.** No new pipeline: there is one scene pipeline,
`textureView` is bind-group entry `@1` with a white default, so a legacy-Appearance draw was
already ON the textured pipeline and merely sampling white. What was missing was CPU-side.
 1. `applyLegacyAppearanceTexture()` uploads `app->texture()->image()` into `draw.textureView` (and
    its linear mean into `texMeanLin`, so the OmniLight bounce sees the real albedo). Called from
    BOTH material paths — the Shape branch and `fillAppearanceMaterial` — deliberately OUTSIDE the
    `if (texCache && pbr)` blocks, since that guard is exactly what kept these images off the GPU.
 2. It also FORCES `baseColor` to white and `specularStrength` to 0, mirroring
    `OmMaterial.cpp:151-152`. The shader computes `albedo = texture × baseColor`, so without the
    forcing the authored `diffuseColor` tints the whole picture — and twice over, since P4 already
    folded `diffuseColor` into the CPU-side ambient product. `baseColorA` (opacity, from
    `transparency`) is untouched, because WREN keeps it too.

✅ **AND THE LIVE LATENT BUG WAS FIXED IN THE SAME CHANGE, WHICH IS THE POINT.** `fillWrenAmbient`
branched on `app->texture() != NULL` — a texture NODE being **declared** — while WREN branches on
`texture()->wrenTexture()`, a texture actually being **loaded**. The tree carries 9 empty
`ImageTexture { }` nodes (Display targets) that are declared and never load, and for those the old
predicate handed them WREN's TEXTURED ambient `(ai,ai,ai)` against an UNTEXTURED `diffuseColor`
albedo — a combination neither renderer produces. There is now ONE predicate,
`legacyAppearanceImage()`, driving the white-forcing, the upload and the ambient together. It is
deliberately WREN-free (`OmImageTexture::image()` is the CPU pixel buffer loaded *because wgpu needs
it*, set and cleared in lockstep with `mWrenTexture`), so it means the same thing without asking
WREN anything.

**Hatch: `OMNISIM_WGPU_LEGACY_TEXTURE=0`** (value-parsed, read once). It reverts the WHOLE change
including the ambient predicate — under it `fillWrenAmbient` falls back to the pre-P11 "declared"
test — so the red arm is an exact revert rather than half of one.

⚠ **ONE DELIBERATE DEVIATION FROM WREN, for the 3 tracked nodes that carry a `texture` and NO
`Material`.** WREN gives those `default.frag`, which is **unlit**: it outputs the raw texel and
never touches a light. wgpu has no unlit arm, and adding one for three test nodes would be a new
pipeline variant with its own parity surface, so those draws are LIT here. The image is right and
the shading is not — a smaller and far more visible error than the flat grey they render today, and
written down instead of silent.

**Gate: MET, and the gate had to be authored — no existing test could go red.**
`camera_color_spherical` and `camera_noise_mask` do put a colour-checker chart on a legacy
Appearance, but their Cameras take `Camera.wrl`'s default backend and never reach the wgpu
collector; the one tracked world with a wgpu Camera (`pen_wgpu_camera`) uses `white256.png` and is
structurally incapable of failing. New pair, driven by the re-runnable
`python scripts/dev/wgpu_dynamic_gate.py --case legacy_texture`:
`projects/samples/demos/worlds/rendering/camera_legacy_texture_wgpu_{smoke,control}.omniworld` — a
colour-checker chart on `Appearance { material Material { diffuseColor 0.25 0.65 0.30 } texture
ImageTexture { … } }` under a wgpu Camera, and the same world with the `texture` field and nothing
else removed. The **non-white diffuseColor is the point**: it is what makes the white-forcing
half separately falsifiable.

⭐ **RED FIRST, on the PRE-CHANGE binary** (machine `9722d23d12a3`, 2026-08-22 23:07 build): gate vs
control **0 px over threshold, max abs channel delta 0** — byte-identical, i.e. the texture
contributed literally nothing. **GREEN** (2026-08-23 00:33 build):

| assertion | red arm (`OMNISIM_WGPU_LEGACY_TEXTURE=0`) | green arm (default) |
|---|---|---|
| gate vs control | **0 px, max 0** | **3804 px, max 48** |
| distinct 6-bit hues vs the untextured control | — | **320 vs 38** |
| red-dominance margin vs the control | — | **55 vs 1** |

The last row is the white-forcing: the chart's red patches arrive red-DOMINANT, which a picture
multiplied by the board's authored green cannot do.

⚠ **AN INSTRUMENT BUG WORTH KEEPING: the first version of the hue assertion was ABSOLUTE and it
PASSED ON THE BROKEN BUILD.** "≥ 12 distinct quantised colours" reads 38 on a flat green board under
a gradient sky, so it went green while the texture provably contributed 0 pixels. Both chart numbers
are now differences against the same scene with the `texture` field removed. Same lesson the lane-4
campaign keeps re-teaching: a verdict is only as good as the thing it is compared against.

 * **Zero cost elsewhere:** `beauty_bench` default vs the three P2/P11 hatches off,
   `mean=0.0085 max=8 px>30=0`, inside its own noise floor `mean=0.0203 max=7 px>30=0`.
 * **No regression:** full `wgpu_dynamic_gate.py` ALL PASS (5 cases),
   `scripts/dev/wgpu_sensor_regression.py` **19/19**, `run-headless --until-finalized` PASS on both
   new worlds.

**Still true, and it is the sequencing point:** ⭐ **P11's deadline was F2, and F2 may now begin.**
Re-goldening the `tests/api` cameras against wgpu before this landed would have baked
"flat 255,255,255" into a pinned golden and permanently destroyed the evidence that a texture was
ever there.

### Recommended order for the remaining parity steps
✅ **The parity list is COMPLETE (E2, 2026-08-23): P1–P11 are all DONE** (P3's Box-atlas residue
closed as a documented, loudly-degrading deviation; P4a retirement landed). What remains is the
schedule below from E3 on. ⭐ **F2 is UNBLOCKED** — P11 was its
prerequisite, and re-goldening the `tests/api` cameras can now proceed without baking away the
evidence that a legacy-Appearance texture was ever there.

### P13 — the back-to-back-spawn wgpu abort (found 2026-08-23, NOT a deletion blocker)
The api `test-group` runner is killed after ~5 worlds by an engine abort: wgpu-native panics at
`conv.rs:1500` (its C-API conversion layer — typically an invalid descriptor handed in from our
side) and then aborts non-unwinding inside `wgpuDeviceCreateBindGroup`, exit 0xC0000409. Facts:
- **Pre-existing, not the campaign's**: reproduces byte-for-byte with every campaign hatch
  disabled (10-hatch A/B, 2026-08-23). Solo runs of the same worlds do not panic — it needs the
  suite's back-to-back engine spawning, the same pattern as AGENTS.md's documented
  "roughly one launch in three" startup race.
- Both observed runs died at the same place (after `battery`), so it is deterministic-ish under
  the suite's spawn cadence, not pure chance.
- ⚠️ **Compounding suite bug: `test_suite.py` prints "Test suite complete" after the abort**, so a
  killed group run is indistinguishable from a finished one by its output. That is how a truncated
  R5 baseline nearly got recorded as whole, twice.
Defensive fix worth making regardless of attribution: no NULL view should ever reach a bind-group
entry (fall back to the default white — a wrong pixel beats a process abort). Not a deletion
blocker: it happens with WREN present, on the wgpu main-view path, and R5 now routes around it by
running per world.
**Two more instances (post-merge pass, 2026-08-23):** `city.omniworld` aborted 0xC0000409 in a
long-lived batch engine after ~85 consecutive hot-reloads and PASSes solo (rerun-confirmed); and
`camera_revert`'s post-revert wedge (a revert is a reload) — see THE POST-MERGE VERIFICATION PASS.

### P14 - city.omniworld's sun_marker handshake fails post-deletion (found 2026-08-23, OPEN)
Deterministic (3/3): on the post-D1.4 binary, `city.omniworld` alone FAILs `run-headless` with
`the OmniSim IPC handshake failed: engine and libController are different builds` for the
`sun_marker_driver` robot. Measured differentials: `omnisim doctor` says the pair is
nonce-compatible; the 8-controller husky swarm PASSes; a 6.6KB world using the SAME
`OmniSimSunMarker.proto` + `sun_marker` controller PASSes; fresh run-ids each time (not the stale
connect_error trap - it was moved aside twice). So the marker, the controller, and libController
are all individually fine, and the failure needs city's asset-heavy load: a LOAD-TIMING RACE in
the handshake, misclassified by libController as an ABI mismatch. Pre-deletion, solo city passed
(V4, rerun-confirmed) - the deletion shifted load timing enough to flip the race. One world, loud
(run-headless FAILs), physics unaffected. ⚠️ It stains the corpus claim precisely this much: the
post-deletion corpus matches the known-failure set EXCEPT that city's failure MODE changed
(reused-engine abort -> solo handshake fail). Root-cause is open work; the misclassification
("different builds" for what is actually a timeout/partial read) deserves its own fix so the next
race is reported as what it is.

## Phase L — Linux presentation

### L1 — decouple the GL present fallback from WREN
The present path already has both arms (`OmView3D.cpp:3229`): native surface when one exists, else
`OmWgpuGlBlitRgbaToScreen()`. That fallback is **66 lines including only `glad/glad.h`**, and
`OmWrenOpenGlContext` is a `QOpenGLContext` subclass whose entire WREN dependency is two
`wr_gl_state_set_context_active()` calls. Move it out of `src/omnisim/wren/`, drop the two calls
under the deletion, keep `src/glad`.
**Gate:** the engine builds and presents with the WREN scene renderer compiled out but the blit
path retained.

⚠️ **L1 IS TWO PROBLEMS AND THEY ARE NOT THE SAME ONE — do not let the good news on the first hide
the second.** `OmRenderBackendRegistry::resolve()` returns the WREN backend on three paths:
`OMNISIM_FORCE_WREN`/`OMNISIM_LEGACY`, an explicit `Kind::Wren`, and — the load-bearing one —
**`Kind::Vulkan` when `vulkanBackend()->isAvailable()` is false**.
 * **Presentation without a native surface: SOLVED** (the GL blit above). This is what the
   "deleting WREN deletes Linux rendering" claim actually got wrong.
 * **wgpu-native being ABSENT on a platform: OPEN, and it is the real gate.** Once WREN is gone
   there is no third backend to fall back to, so a host where wgpu-native does not build or load
   has NO renderer at all. That is a build/packaging question, not a rendering one, and this
   Windows box cannot answer it for Linux.
So L1's true exit gate is **"wgpu-native builds and initialises on every supported platform"**,
with the GL blit covering only the surface half. Anyone quoting the GL-blit finding as "Linux is
unblocked" is quoting half of it. ⚠️ This box **cannot compile the Linux branch** (`QT_FEATURE_xcb -1`), so the gate is
"Windows build with WREN rendering disabled still presents", plus a documented, unverified Linux
claim. Do not overstate it.

---

## Phase F — flips and deprecation

### F1 — make WREN unselectable
`Viewpoint.renderBackend "wren"`, `Camera.renderBackend "wren"`, `WorldInfo.defaultRenderBackend`,
and `OMNISIM_FORCE_WREN` / `OMNISIM_LEGACY` must stop resolving to a WREN renderer. Per the
Radio/Microphone precedent the FIELDS keep parsing (an undeclared field is a hard ERROR) — they warn
once and resolve to wgpu.
**Gate:** every world in the corpus loads with zero new errors; `batch_validate.py` over the tracked
worlds is green.

### F2 — re-golden the sensor tests, deliberately — ✅ DONE (2026-08-23)
`tests/api` camera goldens were wrong for the default backend and were **knowingly** left
un-regoldened so the evidence of the flip survived. Executed 2026-08-23 against the 15-item R5
new-red list (the post-merge pass below carries the per-item ledger): **13 of 15 red→green**
(4 engine fixes + 3 guard-retirements + 6 measured re-goldens/re-pins, every old value recorded
in the test source AND the commit), and the two that stay red now fail **honestly with a
recorded decision** — `camera_revert` (was a reload-loop hang, now a named FAILURE: the RED blob
never satisfies the >3x channel-dominance test under the wgpu/AgX camera) and `point_set`
(per-vertex `Color` needs a wgpu point-primitive pipeline; port post-D1.4).
**Gate met:** old and new values recorded per test; the four engine fixes are: plain-Material
`emissiveColor` read in both wgpu collector branches (the LED port), the no-noise +inf range
contract on RangeFinder+Lidar (per docs/reference/rangefinder.md:65/:69/:390), the
appearance-less-Shape collector gate, and the discontinuity-aware Lidar angular resample.
Non-planar (cylindrical/spherical) Camera/RangeFinder projections are **RETIRED-unported on the
wgpu arm**: the sensor DECLINES (warned once, by device name) and falls back to WREN while it
exists — post-D1.4 that becomes an honestly-empty image, and the five projection tests will need
their post-deletion re-pin then (today they pass byte-for-WREN via the fallback).

---

## Phase D - the deletion

Inventoried 2026-08-22. **Two of the four files this runbook originally named for deletion were
wrong, and the inventory found three parity gaps that are not in P1-P8.**

### D0 - corrections to what "delete WREN" even means
| file | runbook said | measured |
|---|---|---|
| `nodes/utils/OmWrenMeshBuffers.{hpp,cpp}` | delete | **0 `wr_*` in either file** - misnamed, a pure CPU buffer helper. A *consequence* of the deletion, not part of it. |
| `nodes/utils/OmWrenVertexArrayFrameListener.{hpp,cpp}` | delete | **MUST SURVIVE.** It is **the only registry of live Cloth/SoftBody nodes**, and `OmWgpuSceneRenderer.cpp:52` includes it deliberately. Deleting it re-breaks `d5897ff7d` - cloth freezes at rest pose, compiles clean, looks right in one screenshot. Rename it and excise 3 lines (`:63`, `:160`, `:163`). |
| `gui/OmWrenWindow` | delete | correct, but it is coupling C8 and the only genuinely NEW code in D1 |
| `render/OmWrenBackend.hpp` | delete | correct, trivial (a 6-line marker class) |

### D0b - things inside `src/omnisim/wren/` that are NOT WREN and must be rescued first
- **`OmTesselator`** - pure Qt polygon tessellator, consumed by `nodes/utils/OmTriangleMesh.cpp:25`, which is on the **wgpu** path. Move to `nodes/utils/`.
- **`OmWrenOpenGlContext`** - `QOpenGLContext` subclass; entire WREN content is two `wr_gl_state_set_context_active()` calls. **16 surviving includers.** Move to `render/`.
- **`OmWrenRenderingContext`** - **62 includers, the largest coupling in the tree, and this runbook did not mention it.** Its `.hpp` has zero `wr_*`; it owns the `VF_*` enum that **`OmWgpuView.cpp` reads every wgpu frame**, i.e. it IS the W4a/W4b overlay path. Move and de-WREN (11 excisable sites).

### D0c - `resources/wren/` is NOT deletable wholesale
**`meshes/arrow.obj` and `meshes/circular_arrow.obj` are read by the WGPU gizmo**
(`OmGizmoLines.cpp:129,141`), which logs at INFO and continues when they are missing - so deleting
them **silently restores invisible-but-draggable gizmos**, the exact regression P8 just fixed. The
six `*_close/resize_symbol.png` similarly foreclose P7's remaining icon work. 86 shaders and 10
textures do go.

### D0d - THREE PARITY GAPS THE P1-P8 LIST DOES NOT COVER
- ✅ **`OmGranularGroup` (22 `wr_*` sites) had NO wgpu path at all** - zero hits in every wgpu
  collector file, **4 tracked worlds** declare it. Particle rendering was absent from the capability
  table entirely. **New step P9 — DONE (`4ac917c0c`), main view AND sensors.** The `wr_*` sites
  themselves stay until D1.4, as with every other node.
- **`OmVideoRecorder`'s PBO triple** is implemented in `OmWrenWindow` with **no wgpu equivalent**.
  Losing it means **video recording produces nothing, with no error**. **New step P10 — ✅ DONE
  (`3c4a55302`): the recorder feeds from `grabWindowBufferNow()` under a wgpu main view; the WREN
  PBO triple survives untouched until D1.4.**
- **`OmGuiApplication` builds a SECOND, headless `OmWrenWindow`** (`:528`, `:568`) - the
  `--no-window` path that makes `createWrenObjects()` and camera-sensor setup work with no GUI. Its
  failure mode is "renders nothing", not a crash.

### D1 - the sequence (six commits, only one of which must be atomic)
`D1.0` build-system prep -> `D1.1` rescue the misfiled survivors -> `D1.2` delete already-dead code
-> `D1.3` **C8: extract `OmGlWindow`** (the only new code; of `OmWrenWindow`'s six responsibilities
only three are WREN - window/surface, event plumbing, `renderLater`, the `--stream` feed and
`emit resized()` are pure Qt and must be kept) -> `D1.4` **the atomic deletion** (~150 files; it
cannot be split, because every `wr_*` excision also deletes the WREN twin of a working wgpu path and
the two are wired together at compile time) -> `D1.5` retire the hatches and the 42 tooling
consumers.

### D1 - build-order traps that FAIL LOUDLY (fix in D1.0, first)
1. **`WREN_INCLUDE` (`Makefile:144`) is `-I$(OMNISIM_PATH)/include` - the ONLY `-I` reaching
   `include/glad/glad.h`.** There is no `GLAD_INCLUDE`. Delete it and **the GL present fallback
   stops compiling** - the very thing L1 depends on.
2. **`LIB_WREN` expands to `../wren/libwren.a ../glad/libglad.a`.** Deleting the variable
   **unlinks glad**. Split it first.
3. `WREN_INCLUDE += -Iwren` is the search path for 34 include sites. `OmTesselator` must move
   before the tree goes, or the **wgpu** CPU tessellation breaks. `MOC_FILES` derives from
   `QT_SOURCES`, so a removed `.cpp` without its entry leaves a stale `.moc.o`.

### D1 - changes that FAIL SILENTLY (the dangerous list)
Cloth/softbody freeze (registry deleted) - gizmos invisible-but-draggable (`.obj` deleted) - all 21
Optional Rendering items dark (`OmWrenRenderingContext` mishandled) - video recording produces
nothing - `--no-window` renders nothing - GPU-memory field reads 0 MB.

⚠️ **ADD ONE, FOUND BY P1 AND ALREADY LIVE ONCE: a deforming surface FROZEN AT ITS FIRST UPLOAD.**
Two renderers, one process-global "the clock advanced" flag, one mesh cache each — the loser of the
race uploads once and never again. It compiles clean, draws the right number of triangles, and
passes any single-screenshot test; only a CROSS-FRAME comparison in a session with BOTH a main view
and a Camera can see it. The upload decision therefore lives on the cache entry
(`OmWgpuMeshCache::vertexEpochIs`), never in a static, and `OMNISIM_WGPU_DEFORMABLE_EPOCH=0` keeps
the failure reproducible so the instrument can be shown to go red. R7's cloth item must be checked
with `scripts/dev/wgpu_dynamic_gate.py` **windowed**, not headless — headless has one collector and
cannot tell the two rules apart.

**AN INSTRUMENT BUG: the audit's dead-include detection has FOUR FALSE POSITIVES.** It is a `wr_*`
SYMBOL test and cannot see `Wr*` STRUCT usage, so it calls a header dead while the file declares
`WrMaterial` / `WrTransform` / `WrSceneFogType` members (`OmView3D.hpp:244-245`,
`OmPhysicsVectorRepresentation.hpp:58-66`, `OmFog.hpp:76`). A sweep that trusts it produces four
compile errors. ⚠️ **CORRECTED 2026-08-23 (Lane E4): ZERO includes are really dead.** The one candidate this line used to bless (`gui/OmDragViewpointEvent.cpp:25`) broke the build when deleted: the file uses `WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC` — a **`WR_*` enum constant**, a THIRD symbol class invisible to both the audit's `wr_*` function test and its known `Wr*` struct blind spot. That is the audit's FIFTH dead-include false positive and a new category. No include may be deleted on the audit's word alone; every one needs a compile.

## THE READINESS DEFINITION - what "safe to delete" means, mechanically

"Ready" is not a judgement call and must not be reported as one. It is these eight checks, all
green, each a command someone else can re-run. Anything short of all eight is "not ready", and the
honest report names which line is red.

| # | check | command | why it is the right test |
|---|---|---|---|
| R1 | **No WREN dependency outside the removal set** | `python scripts/dev/wren_deletion_audit.py --fail-on-blocking` (exit 0) | The gate the whole campaign is written against. ⚠️ It scans GIT-TRACKED files only, so run it on a committed tree; and its dead-include detector has known FALSE POSITIVES (it tests `wr_*` symbols and cannot see `Wr*` struct usage), so a green R1 still needs a compile. |
| R2 | **The engine builds with WREN compiled out** | `make release` after D1.4 | The only real proof of R1. |
| R3 | **Smoke suite green** | `make tests-smoke` | 4 worlds with explicit verdicts on the default backend. |
| R4 | **The world corpus still loads** | `python scripts/dev/batch_validate.py --glob 'projects/**/*.omniworld' -j4` | Catches a world that silently stopped parsing. ⚠️ Delete leftover `.harness_*` siblings first or the count inflates. |
| R5 | **No shipped test regresses vs the PRE-CAMPAIGN baseline** | `python scripts/dev/wren_readiness.py` (per-world, serialised) | ⚠️ **The baseline is not "all green"** - `camera_color`, `pen_box`, `camera_recognition` and a `cadshape_node` wgpu panic are red on BOTH arms today. R5 is "no NEW red", which means the baseline must be recorded before D1.4, not remembered. ⛔ **Do NOT record it with `test-group`**: the group runner is killed after ~5 worlds by a PRE-EXISTING engine abort (wgpu-native panic at `conv.rs:1500` → non-unwinding abort in `wgpuDeviceCreateBindGroup`, exit 0xC0000409, triggered by back-to-back engine spawning; A/B-attributed 2026-08-23 — reproduces identically with every campaign hatch disabled), **and `test_suite.py` prints "Test suite complete" even after the abort**, so a truncated run looks whole. The readiness tool now runs per world, serialised, and records a `no_verdict` list alongside `red` — a world that stops producing a verdict counts as a regression at comparison time. The group-runner crash itself is an open defect (`P13` below), independent of the deletion. |
| R6 | **Every capability on the parity list is ported or deliberately retired** | the per-step gates in Phase P | A retirement counts as green ONLY if the field still parses and the engine warns by name (the `Solid.immersionProperties` precedent: an undeclared field is an ERROR that takes headless exit to 1). |
| R7 | **The silent-failure list is individually disproven** | see Phase D "changes that FAIL SILENTLY" | Cloth animates, gizmos draw where they grab, all 21 Optional Rendering items respond, video recording produces a file, `--no-window` renders, the GPU-memory field shows **a real figure or an explicit "unavailable" — never a silent 0 MB** (⚠️ amended by Lane E4: the wgpu C API exposes NO GPU-memory query at all — verified against the vendored headers — so "non-zero" is unsatisfiable post-deletion; the honest substitute prints the adapter identity and says the figure is unavailable). **Each of these compiles clean when broken**, so R2+R3 cannot see them. This is the check most likely to be skipped and the most expensive to skip. |
| R8 | **wgpu-native builds and initialises on every supported platform** | a Linux run | With WREN gone there is no third backend: a host where wgpu-native does not load has NO renderer. This box cannot answer it (`QT_FEATURE_xcb -1`). **Owner-assumed satisfied 2026-08-22**; if that assumption is ever withdrawn, R8 goes red and deletion is blocked again regardless of R1-R7. |

⚠️ **R7 exists because R1-R3 are all compile-time or load-time checks, and every one of the
campaign's worst near-misses compiled clean**: deleting the deformable registry freezes cloth at its
rest pose; deleting two `.obj` files silently restores invisible-but-draggable gizmos; mishandling
`OmWrenRenderingContext` turns every overlay dark. A green build is necessary and nowhere near
sufficient.

## Order and parallelism

P1–P8 are independent of each other and can proceed in any order. L1 is independent of all of them.
F1 requires P1–P8 **and** L1. F2 requires F1. D1 requires everything.

⚠️ **Parallelism is bounded by the thermal ceiling, not by dependencies.** Any agent that loads a
world is a whole engine, and engines must be serialised. So: implementation agents run **one at a
time** for anything that verifies against a running engine; read-only scoping agents can run
alongside freely. This is the constraint that decides throughput here — not the dependency graph.

## THE EXECUTION SCHEDULE (authored 2026-08-23, from the measured state)

Everything above is analysis; this is the committed order of execution. One engine-owning lane at a
time; each lane ends with its evidence in this file and a push. Re-derived state at authoring time:
DONE = P1, P2, P3 (minus the Box-UV residue), P4 (luminosity/IBLStrength; `*IrradianceUrl`
retirement open), P4b, P6-rendering, P7, P8, P9, P11, VR, Skin. Audit 352 blocking / 0 retirable.

| lane | contents | why this position | exit evidence |
|---|---|---|---|
| **E1** ✅ DONE (`545d01e45`) | **P5 + P6**: the `resolvesToWgpuBackend()` predicate fix first (unblocks Lidar warnings AND the `OMNISIM_NO_GL` fatal), then port Lidar/RF `noise`+`resolution` and camera `noise`+`motionBlur`; retire `lens`/`lensFlare`/`focus`/`noiseMaskUrl` with fields parsing + named warnings | fully scoped, formulas measured from the shaders; the only step with a silent wrong-floats defect live today | red-capable regression case in `wgpu_sensor_regression.py`; σ ≈ 0.10 m spread measured on the live lidar world |
| **E2** ✅ DONE (`3c4a55302` + `3d1020907` + `64a678126`) | **P10** video recording (`grabWindowBufferNow()` replaces the PBO triple; `isRecording()` bail-out deleted; hatch `OMNISIM_WGPU_VIDEO`) + **P4a** `*IrradianceUrl` retirement (fields parse, warn once per node by name) + **P3 residue** (Box/Cylinder/Cone cross-atlas UV — documented as accepted deviation + one-shot warning; the stride change would have added WREN C-API surface right before D1.4 deletes it) | all small; P10's replacement already ships (`--stream` uses it) | **MET**: red/green via the hatch on one binary — WREN-fallback line fires at `=0`, gone by default; green mp4 pixel-sampled (141 frames, non-black, non-uniform, frame 2 vs 100 differ on 79k px; 93% of pixels differ vs the WREN arm — the frames are the wgpu pane); `supervisor_start_stop_movie` controller exits successfully; icon_studio loads 0-errors with the named warning; peaks ≤66 °C |
| **E3** ✅ DONE (`5f7d0883a`) | **R5 BASELINE RECORDING** — `wren_readiness.py --record-baseline` on api + rendering groups | ⛔ MUST precede F1 and D1.4: the red set cannot be reconstructed once WREN behaviour changes | `wren-readiness-baseline.json` committed (recorded 2026-08-23T11:32Z, **pre-F1-flip**: api 50 red + 7 no-verdict, rendering 5 + 1) |
| **E4** ✅ MERGED (`6dbb30310`) + VERIFIED (post-merge pass, below) | **D1.0 + D1.1 + D1.2** build prep (split `WREN_INCLUDE`/`LIB_WREN` so glad survives), rescue the misfiled survivors (`OmTesselator`, `OmWrenOpenGlContext`, `OmWrenRenderingContext`, rename the deformable registry), delete dead code — plus the three unowned inventory items: port the **force/torque drag arrows** (`OmPhysicsVectorRepresentation`, via the W4a overlay-line path), a wgpu substitute for the **GPU-memory readout** (`wr_gl_state_get_gpu_memory`), and confirm-or-port **`OmVisualBoundingSphere`** | pure build/moves; every trap is already written down above | build green at every commit; runtime half in **THE POST-MERGE VERIFICATION PASS** below — overlay pass live (`ovBatches` 1→2, `ovVerts` 24→28, `ovOk=1` under `OMNISIM_OPTIONAL_RENDERING`); BoundingSphere + drag-arrow *visuals* are selection/drag-gated and stay on the needs-a-human list |
| **E5** ✅ MERGED (`6dbb30310`) + VERIFIED (post-merge pass, below) | **D1.3 — extract `OmGlWindow` from `OmWrenWindow`** (C8), incl. the headless second window in `OmGuiApplication` | the only genuinely new code; everything after depends on it | build green with `OmWrenWindow` still present; video (both arms, pixel-proven), `--stream` mjpeg (real 400×300 frame pulled), `OMNISIM_NO_WINDOW` (camera checksum byte-identical to windowed), programmatic resize (view tracked 1896×1113 → 1×1 → 400×300 over the ws protocol) — all through the new base; `resized()`'s two GUI consumers stay on the needs-a-human list |
| **E6** ✅ **F1 DONE (`35148ad65`+`76a8cb6e2`+`8087caffd`) + VERIFIED; F2 ✅ DONE (2026-08-23, Lane F2)** | **F1** make WREN unselectable (fields parse + warn, hatches neutralised) + **F2** re-golden the api cameras against wgpu (P11 landed, so the goldens no longer bake in flat textures) | F2 after F1 by definition; both after E3's baseline | F1: per-node + registry warnings fire and a `"wren"` camera's image is **byte-identical** to the wgpu sibling; corpus batch-validate 377 worlds, zero new load errors (4 pre-existing `_archive` stale-URDF fails + one P13 abort that PASSes solo). F2: **13 of the 15 new-reds red→green** (4 engine fixes: LED emissive, +inf range contract, appearance-less Shapes, edge-aware lidar resample; 3 non-planar guard-retirements; 6 measured re-goldens/re-pins, old values recorded per test + per commit); the 2 that remain red are DECIDED honest reds — `camera_revert` (fails naming the wgpu/AgX red-classification gap; the reload-loop hang is gone) and `point_set` (needs a wgpu point-primitive pipeline, post-D1.4). Per-item ledger in THE POST-MERGE VERIFICATION PASS → "F2 — the fix pass" below |
| **E7** ✅ **D1.4 DONE (`976b9449d`) + D1.5 DONE (`1c4f1b413`) + ENDGAME PASS DONE (2026-08-23, below)** | **D1.4 the atomic deletion** (456 files, −47,533 lines, one commit) + **D1.5** hatch/tooling retirement — plus the endgame: the W1d ElevationGrid/Cone fix (`3cbbfd7a5`), the three owed projection re-pins (`eb11249fd`), and the closing sweep | last, by construction | Audit reads **DELETION-READY (0 blocking / 0 retirable)** on the closed tree; final R5 in **THE DELETION ENDGAME PASS** below — the only survivors are the two DECIDED reds (`camera_revert`, `point_set`) plus the pre-existing baseline set; P14 (`city` handshake race) stays the one open post-deletion defect and is corpus-only, not in the api group |

**Standing rule for every lane:** prove the gate red before trusting it green; report the guard's
printed peak; commit with explicit paths; push only after the smoke hook passes.

## THE POST-MERGE VERIFICATION PASS (2026-08-23, main @ `6dbb30310`, binary of 13:43)

The consolidated engine pass every worktree lane deferred (E4/E5 runtime halves, F1 behaviour,
R4 corpus, the R5 comparison). Machine `9722d23d12a3` (RTX 3060 laptop), every engine run under
`thermal_guard --ceiling 75`. Audit on the merged tree: **322 blocking / 0 retirable** (down from
352; F1c taught the audit what F1 changed).

### F1 behaviour — VERIFIED
* A Camera authoring `renderBackend "wren"` warns **by node name** (`DEF CAM_RIG Robot > Camera
  "chart_cam": renderBackend "wren" is RETIRED…`) plus the once-per-process registry warning, and
  its image is **byte-identical** (same 96×72 PPM, checksum 261558207) to the unmodified
  `renderBackend "wgpu"` sibling world — the "wren" request measurably rendered wgpu.
* `OMNISIM_FORCE_WREN=1` and `OMNISIM_LEGACY=1` each fire their `RETIRED and IGNORED` warning
  (LEGACY fires both render and physics arms) and produce the same byte-identical wgpu image.
* E1 retirement warnings still fire on the flipped default (`camera_color_focus` loads 0-errors,
  `focus` warning names the device). ⚠️ **STALE ADVICE in the shared warning suffix**: it ends
  `Set renderBackend "wren" on this Camera to keep it` ([`OmAbstractCamera.cpp:396`](../../src/omnisim/nodes/OmAbstractCamera.cpp))
  — post-F1 that value is a warned no-op, so the advice is dead. One-line wording fix, F2/D1.5.
  ✅ FIXED by F2 (2026-08-23): the suffix now names the retirement ("The WREN renderer that
  implemented it is RETIRED (there is no per-Camera opt-out)") instead of the dead opt-out.
* `pen` via test-world: **FAIL at the documented P3 3/255 ink residual**, byte-identical values to
  the E2-era measurement ((207,186,186) vs (207,189,189)). `camera_image_update`: FAIL against its
  WREN golden ([0 178 127] vs received [6 190 171]) — the F2 class, exactly why F2 exists.

### E5 runtime — VERIFIED (with one pre-existing finding sharpened)
* **Movie recording, default wgpu arm**: on an authored ENU probe (chart + falling ball + the
  `supervisor_start_stop_movie` controller) the mp4 is the live scene — mean ≈ 89–100, std ≈ 61–71,
  **177,434 of 307,200 px** differ between frames 1 and 15 (the ball falls), geometry visible.
* **`OMNISIM_WGPU_VIDEO=0` WREN arm**: fallback line fires once, mp4 shows sky/floor/ball with the
  chart black — which is WREN's documented phong zero-ambient, i.e. the WREN PBO ring is intact.
* ⚠️ **NUE (Y-up) worlds record a SKY-ONLY movie under the wgpu default** — the known Y-up
  viewpoint/Z-up wgpu camera issue measured into the movie path: two *different* NUE worlds
  (`supervisor_start_stop_movie` and a modified copy) produced **byte-identical mp4s** containing
  only the night sky; the WREN-pin arm on the same world records a near-uniform dark pane
  (std ≈ 1). The api movie test asserts only that the file exists, so it stays green through this.
  Pre-existing class (2026-08-19 flip), not the merge's — but now provable in a shipped test world.
* **`--stream` mjpeg**: real 400×300 JPEG pulled off `/mjpeg` (mean ≈ 127–137, std ≈ 55–64) after
  the ws-side `mjpeg: WxH` init. Two traps for the next prober, both client-side, not engine bugs:
  the streamer idles at resolution −1×−1 until a **websocket** client sends `mjpeg: <W>x<H>`
  (a bare HTTP GET /mjpeg gets headers and then nothing), and the init format is `WxH` — a
  malformed one (`400 300`) parses as 0×0 and **resizes the live view to 1×1**.
* **`OMNISIM_NO_WINDOW=1`**: camera images produced, checksum byte-identical to the windowed run.
* **resized()/moc re-binding**: programmatic resize proven end-to-end (`setView3DSize` via the ws
  protocol; view tracked 1896×1113 → 1×1 → 400×300 in both the wgpu report and the delivered frame
  size). The signal's two GUI consumers (device-inset perspective restore, thumbnail) emit no log —
  human check listed below.

### E4 runtime — VERIFIED to its automatable ceiling
* The W4a overlay-line pass (which the drag arrows, BoundingSphere and optional renderings all
  compose into) is live post-merge: `OMNISIM_OPTIONAL_RENDERING=CoordinateSystem,JointAxes,…` →
  `ovBatches` 1→2, `ovVerts` 24→28, `ovOk=1`, with the lever's applied-names log line.
* **The lever does NOT cover name-based global items** — its map is the 15 `VF_*` flags only, so
  `BoundingSphere` cannot be armed from the environment. The `.wbproj`
  `globalOptionalRendering: BoundingSphere` route arms `gEnabled` at load
  ([`OmMainWindow.cpp:1224`](../../src/omnisim/gui/OmMainWindow.cpp)) but the collector additionally
  requires a **selected node** (`overlayEnabled()` = enabled && node), and the tree has no
  programmatic selection surface — GUI-gated, listed below.
* **GPU-memory honesty**: code-verified — the dialog prints real bytes, else
  `unavailable (the wgpu C API exposes no GPU-memory query)`, else `N/A`; no branch can print
  0 MB; `adapterSummary()` is captured at init. The dialog itself is GUI-only (listed below).

### R4 corpus — GREEN (zero new load errors)
`batch_validate` over `projects/**/*.omniworld`: **377 worlds, 372 ok, 5 FAIL**, none new:
4 × `projects/_archive/samples/demos/worlds/showcase/*` (stale relative URDF —
`_archive` has no `robots/` tree; pre-existing, rendering-independent), and
`environments/city.omniworld`, which FAILed **in the reused batch engine** with exit 0xC0000409
(the P13 abort class) and **PASSes solo** (`run-headless --until-finalized`, 0 errors — rerun
confirmed). ⚠️ Thermal note: the corpus could not be swept in one run on this box that day —
`-j2` and `-j1` full sweeps were guard-killed (peaks 77/76/81 °C); the numbers above come from a
resumable ledger driver (6-world rounds, precool, solo-retry fallback), all 377 accounted for.

### R5 — the comparison (RED, and the red is F2's worklist)
`wren_readiness.py` vs the E3 baseline (`recorded 2026-08-23T11:32:13Z`, pre-flip):
**15 NEW red** — api: `camera_color_spherical`, `camera_image_update`, `camera_noise_mask`,
`camera_revert`, `led`, `lidar_rotating`, `lidar_vs_range_finder`, `pen`, `pen_box_scaled`,
`pen_mesh`, `pen_plane`, `range_finder`, `range_finder_spherical`,
`range_finder_spherical_horizontal_vertical`; rendering: `point_set`. **Zero red→green.**
Every one re-run once and confirmed (same failure text twice). Classification:
* **WREN-golden vs wgpu image (pure F2 re-golden work)**: `camera_image_update` (Δ ≈ (6,12,44)),
  `camera_noise_mask` (Δ ≤ 4/255 — plus the retired mask no longer applies), `pen` (the documented
  3/255 ink residual), `pen_box_scaled` (the documented P3 Box-atlas deviation, warned).
* **`range_finder`**: received 3.0 where WREN wrote **+inf** — this is exactly P5's documented
  no-noise clamp-vs-+inf divergence, previously "unpinned", now pinned red by the flip. F2 must
  DECIDE it (reproduce the +inf, or re-golden to the clamp), not just re-golden blindly.
* **Spherical/cylindrical projection family**: `camera_color_spherical` receives (3,3,3) — sky
  only, no geometry — and `range_finder_spherical`/`…_horizontal_vertical`/`lidar_vs_range_finder`
  disagree with expectations by metres. Non-planar *device* projections under the wgpu sensor path
  are not equivalent to WREN's; F2 cannot re-golden these without a decision (port or warn+retire).
* **`led`**: "phong material should be bright red" — a runtime LED colour change does not reach the
  wgpu camera image. Suspected class: `OmLed` writes its on-colour into WREN material state, which
  the wgpu collector never reads. **A behaviour gap, not a golden delta** — needs a port.
* **`pen_mesh` / `pen_plane`**: paint floods (3373 px vs <60) / wrong ink colour
  ((205,130,10) vs (206,171,58)) — ⚠️ **P3's "Plane / meshes paint exactly" claim does not survive
  these two tests**; the accepted-deviation note under P3 covers Box/Cylinder/Cone only. P3's
  residue is wider than documented; correcting the ledger belongs to the owner of F2.
* **`point_set`**: per-vertex `Color` on a `PointSet` wrong under the wgpu camera — a wgpu gap.
* **`camera_revert` — the one genuine post-flip HANG, not a golden.** NO-VERDICT three times
  (readiness run + two reruns, one of them tree-killed at 10 min). After the supervisor revert the
  engine floods `Solid 'robot': the physics collider is NOT the authored geometry -- using sphere
  r=0.001` **once per tick, indefinitely**, and no verdict ever arrives — the post-revert reload
  never settles. Joins the already-red/no-verdict reset/revert class (baseline:
  `supervisor_reset_simulation` red, `supervisor_update_after_reset_simulation` no-verdict), but
  it was green pre-flip. Needs an owner before F2 (a hang cannot be re-goldened).

### F2 — the fix pass (2026-08-23): per-item outcomes for all 15
Executed same-day on the merged tree, every engine run under `thermal_guard --ceiling 75`
(peaks 55–65 °C across the whole pass; no guard kill). **13 red→green, 2 honest decided reds.**

⚠️ **First, a correction to the `camera_revert` bullet above: "once per tick, indefinitely" was
wrong — it is once per WORLD CONSTRUCTION, and the constructions are the bug.** Probed from a
clean cwd with a hard cap: the collider warning count EQUALS the Newton registration count
(67 == 67 in a 7-minute window), i.e. one warning per reload, 67 reloads — a reload LOOP, not a
per-tick engine fault. Mechanism, confirmed by all three probe checks: under the wgpu camera the
RED box (diffuse 1 0.1 0.1) never satisfies the test's `>3x` channel-dominance classifier, so
`red_blob_0.png` is never written, `run_count` stays 0, and every lap ends in
`wb_supervisor_world_reload()` — green+blue blob files present, red absent, forever. The engine
never hung; the TEST looped it.

* **`led` → GREEN (engine fix + measured re-pin).** `OmLed` drives a plain Material's on-colour
  through `Material::setEmissiveColor`; the wgpu collector's two plain-Material branches read
  diffuse+transparency only, so every phong LED was invisible to a wgpu camera. Emissive read
  added at BOTH sites (kept as two edits per the file's own no-shared-refactor rule). Measured:
  phong pixel went dark → **(207,20,20)** — red passes `>0xaa` at the same 207 WREN encoded, and
  ALL THREE LEDs' brightness gates pass (the SpotLight "light" LED too, un-forced). The residual
  green/blue **20** is structural AgX inset-matrix crosstalk on a pure primary (the ±2-exact
  `== 0x00` asserts CANNOT be met under AgX); re-pinned to `< 24` with the old `0x00` recorded.
* **`range_finder` → GREEN (two engine fixes).** (1) The no-noise wgpu clamp-to-maxRange is
  replaced by the contract's +inf — docs/reference/rangefinder.md:65/:69/:390 ("infinity is
  returned" below minRange / above maxRange), matching the E1 noise path's open-interval CLIP —
  on RangeFinder AND all five Lidar resample sites (noise path untouched). (2) The collector's
  Shape gate required `(pbr || app)`, silently dropping every **appearance-less Shape** — legal
  VRML; three of this world's four obstacles were invisible to the sensor (background +inf)
  while WREN reported 2.0 m hits. Gate widened (branch body is null-safe; fallback 0.8 grey).
* **`lidar_rotating`, `lidar_point_cloud` → GREEN** off the same +inf fix (point_cloud re-run as
  a regression check; both WREN and wgpu now report +inf on a miss, `isinf` compares hold).
* **`lidar_vs_range_finder` → GREEN (guard + engine fix).** With the RF's cylindrical arm
  declined to WREN, 125/128 columns agreed (all +inf columns column-for-column); the 3
  disagreeing were silhouette-EDGE columns where the wgpu bilinear angular resample blended a
  hit with the background and INVENTED ranges (worst: 4.6458 m reported where WREN reports a
  miss — a phantom point-cloud point). All five Lidar resample sites now snap to the
  angularly-nearest sample when the contributing samples straddle a depth edge (spread > 5% of
  maxRange); smooth regions keep sub-column bilinear precision.
* **Projection family (`camera_color_spherical`, `range_finder_spherical`,
  `…_horizontal_vertical`) → GREEN (retired with the honest guard).** `buildViewProj` is a
  planar pinhole (`tan(fov/2)`; `tan(pi)≈0` degenerates), so the Camera and RangeFinder wgpu
  arms now DECLINE non-planar projections — warned once per device, by name — and fall back to
  the WREN/base path while it exists (post-D1.4: honestly-empty image + the wrenless warn).
  The three tests pass byte-for-WREN via the fallback, so their re-pin was a **no-op today**;
  the post-deletion re-pin (warned + empty) is owed when D1.4 lands — ✅ **PAID 2026-08-23
  (`eb11249fd`), per-test ledger in THE DELETION ENDGAME PASS below.** Lidar's cylindrical wgpu
  pipeline is native and untouched. `spherical_camera.omniworld` (the ONE authored non-planar
  user outside tests/) is deliberately KEPT as the warned showcase — the fallback still renders
  it correctly today, and its `info` block now names the retirement and the post-D1.4 fate.
* **`camera_image_update` → GREEN (measured re-golden).** The test is about update TIMING; its
  expectation was `255 * diffuseColor` verbatim — true only of WREN's flat ambient render.
  Re-pinned to a measured table (old → new): [0 178 127]→[6 190 171], [179 128 179]→[190 171 190],
  [64 64 179]→[132 132 190]; grey [204]³ and disabled [0]³ unchanged. Lag structure asserted as before.
* **`camera_noise_mask` → GREEN (measured re-golden + retirement re-pin).** Base colours
  (old → new, delta stays 1): (123,71,58)→(127,71,57), (156,205,195)→(162,210,201),
  (207,207,207)→(210,210,210), (44,44,44)→(42,42,42). The opaque-mask arm expected the mask's
  {42,140,32} — `noiseMaskUrl` is E1-RETIRED, so it now asserts the image stays EXACTLY the
  unmasked scene (the supervisor field write must never corrupt the image).
* **`pen` → GREEN (measured re-golden).** Third ink accumulation (207,189,189)→(207,186,186)
  (the documented 3/255 residual, ~1 LSB/stroke); the magenta-overwrite mean compounds the same
  per-stroke residual to (154,178,167)→(99,142,122) — reproduced stable, delta stays 2.
* **`pen_plane` → GREEN / `pen_mesh` → GREEN / `pen_box_scaled` → GREEN** — see the P3
  re-scope note under Phase P above: pen_plane is a colour re-golden (placement exact, wgpu ink
  nearer the authored value than WREN's was), pen_mesh joined the warned atlas deviation class
  (test re-pinned to ink-presence with WREN's placement constants preserved in-comment),
  pen_box_scaled re-pinned to the E2-decided displaced-but-visible behaviour (its ink-presence
  scan uses the (205,130,10) value measured on pen_plane, cross-validating both).
* **`camera_revert` → HONEST RED (decided).** Test rewritten order-independent: reload/compare
  fires only when ALL THREE blobs have been classified, and a 400-step budget (sized under the
  suite's 30 s wall timeout) converts "a colour is never classified" into a named FAILURE:
  `Timed out after 400 steps without classifying every blob (red: NEVER FOUND, green: found,
  blue: found)`. The hang is gone; the red that remains is the real finding — the wgpu/AgX
  camera compresses the red box's channel dominance below the 3× classifier. Same family as the
  AgX-crosstalk deltas above; owner: whoever owns the sensor-pipeline look post-D1.4.
* **`point_set` → RED (decided, unfixed).** Per-vertex `Color` on a `PointSet` needs a wgpu
  point-primitive pipeline (topology + a per-vertex colour attribute) that does not exist; the
  collector's triangle paths cannot represent it. Port owned post-D1.4 with the second-UV/
  vertex-attribute work. Its second (emissive) assertion is masked by the first and could not
  be measured; the F2.2 emissive read plausibly improves it but that is NOT claimed as verified.

Also landed with the pass: the stale advice in the shared degraded-feature warning suffix
(`OmAbstractCamera.cpp` — "Set renderBackend \"wren\" on this Camera to keep it", dead since F1)
now names the retirement instead.

**The post-F2 R5 verdict (full 148-world sweep, `wren_readiness.py --json`, 2026-08-23):**
```
  [  RED  ] R5    no NEW red vs the pre-deletion baseline
            {"api": ["camera_revert"], "rendering": ["point_set"]}
            note: baseline 2026-08-23T11:32:13Z
```
Down from 15 new reds to the two DECIDED ones above — every other world in the sweep matches the
E3 baseline exactly (all remaining REDs/NO-VERDICTs are baseline entries; zero regressions from
the four engine fixes, which was the specific risk of widening the appearance-less-Shape gate).
`camera_revert` now returns a red VERDICT in ~30 s instead of wedging the sweep for 7+ minutes.
R1 (322 blocking, the D1 deletion audit) and R3 (the zombie engine on TCP 1234, environmental)
are unchanged pre-existing reds documented above.

R3 (smoke suite) also reads RED in the tool, and that is **environmental**: the runner hard-aborts
because the documented zombie engine (PID 31680, needs a reboot) holds TCP 1234. All four active
smoke worlds PASS individually — `gravity_rest_height` OK (g=9.8124, rest z=0.599892, hinge
0.6012/0.6020 — the reference numbers to 4 decimals), `wheel_roll_noslip` OK (slip 0.04%),
`empty` + `template_deterministic` load-PASS.

### P13 — two new instances for its ledger
`city.omniworld` aborting (0xC0000409) in a long-lived engine after ~85 consecutive hot-reloads and
PASSing solo; and plausibly `camera_revert`'s post-revert wedge (revert is a reload). Both fit the
"back-to-back load cadence" signature; neither reproduces solo.

### The needs-a-human list (GUI-gated, honestly deferred)
1. **BoundingSphere visual**: View ▸ Optional Rendering ▸ Bounding Sphere with a node selected →
   three great circles at the selection's culling sphere; expect `ovBatches` +1 / `ovVerts` +288.
2. **Drag arrows**: alt-drag a Solid → orange force arrow (torque: dark-yellow helix) exactly at
   the drag; `OMNISIM_WGPU_DRAG_ARROWS=0` hides them; a paused (locked) drag stays visible.
3. **`resized()` consumers**: resize the main window → device-inset overlay layout survives and
   the world thumbnail regenerates (no log output exists for either).
4. **GPU-memory dialog**: Help ▸ OpenGL information → adapter line (`<device> via <backend>`) and
   a real byte figure while WREN lives; the `unavailable (…no GPU-memory query)` wording post-D1.4.
   Never `0 MB`.

Thermal: peaks per phase — V0–V2 ≤ 66 °C; corpus sweep guard-kills at 76–81 °C before the chunked
driver; R5 reruns ≤ 65 °C. The 75 °C ceiling was enforced by the guard throughout; the breaches
above are the guard *working* (kill + resume), not runs that were allowed to cook.

## THE DELETION ENDGAME PASS (2026-08-23, post-D1.4/D1.5, main @ `eb11249fd`)

The closing pass after the atomic deletion (`976b9449d`) and the hatch/tooling retirement
(`1c4f1b413`): one diagnosed rendering fix, the three re-pins F2 left as an IOU, and the final
readiness sweep. Machine `9722d23d12a3`, every engine run under `thermal_guard --ceiling 75`
(peaks 55–59 °C across the whole pass; no guard kill).

### W1d — the default branch's coverage lie: ElevationGrid and Cone were invisible to every wgpu render (FIXED, `3cbbfd7a5`)

`acquireGeometryMesh`'s default branch claimed in its comment to serve "IndexedFaceSet, Mesh,
ElevationGrid, Cone, ..." but only ever handled `OmTriangleMeshGeometry` subclasses — and
**`OmElevationGrid : public OmGeometry` and `OmCone : public OmGeometry`**
(`OmElevationGrid.hpp:28`, `OmCone.hpp:24`), so the `dynamic_cast` returned null. Pre-D1.4 the
null fell through to the WREN static-mesh readback and nobody noticed; **D1.4 deleted the
readback, and from that commit every terrain and every visual Cone was silently SKIPPED from
every wgpu render — main view and sensors alike.** Measured: `tests/api/worlds/range_finder`
FAILed `Received value = inf, Expected value = 2.000000` — its ElevationGrid obstacle invisible
to the sensor. The lesson is the campaign's own W1c lesson again: the WREN readback was a
*fallback that made lies survivable*, and deleting it converts every stale comment about
coverage into a silent hole.

The fix: explicit `WB_NODE_ELEVATION_GRID` / `WB_NODE_CONE` cases generating the mesh CPU-side
in `OmWgpuMeshAdapter`, mirroring the deleted WREN builders **verbatim**
(`976b9449d^:src/wren/StaticMesh.cpp` — `createUnitElevationGrid` line 535, `createUnitCone`
line 252). Axis convention MATCHED to that source, not guessed: the grid lies in the **X-Y
plane at unit spacing with heights along +Z** (post-axis-migration OmElevationGrid is
x/y-named — `xDimension/yDimension/xSpacing/ySpacing`, not the x/z of upstream Webots docs),
WREN's row flip preserved (mesh row `yi` reads height row `dimY-1-yi`), normals the normalised
sum of the 4 neighbouring cells' cross products; `(xSpacing, ySpacing, 1)` and
`(bottomRadius, bottomRadius, height)` ride the model matrix, exactly the two nodes'
`updateScale()` `wr_transform_set_scale` conventions. Content-hashed keys per the capsule's
FNV-1a pattern (the hashed key space gains a 2-bit member id so capsule/cone/grid can never
alias structurally); a supervisor height edit lands a new key and a fresh upload. A
side=FALSE bottom=FALSE Cone joined `primitiveHasNoSurface` — its invisibility used to be an
accident of the default branch's null-mesh read, which it no longer reaches. The default
branch's comment now names its true coverage.

**Gates, all met:**
* `range_finder` test → **OK**, engine line `depth via wgpu (64x64, 4 draws) center=2.0000 m` —
  the owed (32,63)=2.0 expectation returns, no other deltas.
* `beauty_bench` (contains neither node) binary A/B, pre-fix vs post-fix: **mean 0.004, max
  4/255, 0 px > 30**, against a same-binary repeat noise floor of exactly **0** — pixel-neutral
  where the fix has nothing to draw.
* `mars.wbt` terrain: **draws 50 vs 49** against a probe with the grid geometry stripped
  (+1 = the ElevationGrid), **673,899 of 2,110,248 px differ >30** — the terrain plate is
  visibly back.
* Cone probe (r 0.5 / h 1.4 / subdivision 24): **draws=2, 61,842 red-dominant px**, correct
  silhouette, apex up, smooth side shading, shadow cast. Probe worlds deleted after.

### The re-pin ledger — F2's IOU paid (`eb11249fd`)

The decided post-deletion behaviour, now asserted: a non-planar Camera/RangeFinder **DECLINES**
(warned once per device, by name) and its image stays the honestly-empty buffer — (0,0,0)
pixels for a Camera, 0.0 at every pixel for a RangeFinder (not a distance, not the +inf miss
contract; nothing rendered at all). Old expectations recorded in each test source and each
commit message.

| test | old (WREN-era) | new (decided, post-D1.4) |
|---|---|---|
| `camera_color_spherical` | 6 cylindrical + 6 spherical sampled colors (yellow/red/pink/… — full list in the test source) | all 12 samples **(0,0,0)**. ⚠️ Found while re-pinning: the inner `WbDeviceTag spherical_camera` declaration **shadowed** the outer variable, so the spherical sample block was DEAD CODE even in the WREN era — un-shadowed with the re-pin, so both devices' emptiness is now actually asserted |
| `range_finder_spherical` | 2.0 / INFINITY / 2.0 / INFINITY / 2.0 at the five mid-line samples | **0.0 at all five** (the frozen empty buffer) |
| `lidar_vs_range_finder` | RF == Lidar column-for-column within 0.05 m (measured pre-re-pin failure: `0.000000 != inf` at column 0) | per-arm: every RF column **exactly 0.0** (declined), AND the Lidar image carries **both a finite hit and a +inf miss** — the native cylindrical Lidar pipeline is live and can never silently go empty alongside the RF |
| `range_finder_spherical_horizontal_vertical` | — | **NOT flagged** — green on the post-D1.4 binary unchanged, confirmed by a solo run and the closing sweep |

All three re-ran **OK** solo and read `ok` in the closing sweep.

### The closing sweep — R5 final, audit final

`wren_deletion_audit.py --fail-on-blocking` on the closed tree:
**`VERDICT: DELETION-READY (0 blocking, 0 retirable)`, exit 0** — down from 322 at the
post-merge pass; the D1.4/D1.5 pair retired every remaining site and the endgame commits added
none back.

`wren_readiness.py --json` (full api + rendering sweep) vs the E3 baseline:

```
==================================================================================
WREN DELETION READINESS
==================================================================================
  [ GREEN ] R1    no WREN dependency outside the removal set
            VERDICT: DELETION-READY   (0 blocking, 0 retirable)
            note: audit scans GIT-TRACKED files only, and its dead-include detector cannot see Wr* STRUCT usage -- a green R1 still needs R2
  [MANUAL ] R2    engine builds with WREN compiled out
            run `make release` after D1.4
            note: the only real proof of R1
  [  RED  ] R3    smoke suite green
            smoke suite
  [MANUAL ] R4    world corpus still loads
            python scripts/dev/batch_validate.py --glob 'projects/**/*.omniworld' -j4
            note: delete leftover .harness_* siblings first or the count inflates
  [  RED  ] R5    no NEW red vs the pre-deletion baseline
            {"api": ["camera_revert"], "rendering": ["point_set"]}
            note: baseline 2026-08-23T11:32:13Z
  [MANUAL ] R6    every parity item ported or deliberately retired
            see Phase P gates in the runbook
            note: a retirement is green ONLY if the field still parses and warns by name
  [MANUAL ] R7    silent-failure list individually disproven
            cloth/softbody still ANIMATE (not frozen at rest pose) on the wgpu main view; gizmo handles are DRAWN where they are draggable (arrow.obj / circular_arrow.obj p
            note: each of these COMPILES CLEAN when broken -- R2/R3 cannot see them
  [ASSUMED] R8    wgpu-native builds+initialises on every platform
            owner-assumed satisfied 2026-08-22; needs a Linux run
            note: with WREN gone there is no third backend
----------------------------------------------------------------------------------
VERDICT: NOT READY -- R3, R5 red
         1 green, 2 red, 5 unproven
==================================================================================
```

Reading that verdict honestly, line by line:
* **R5 is the expected end-state exactly**: the only survivors above the baseline are the two
  DECIDED reds (`camera_revert` — the wgpu/AgX red-classification gap; `point_set` — needs a
  wgpu point-primitive pipeline). The three re-pinned projection tests read `ok` in this sweep;
  `range_finder` reads `ok` with the W1d fix; `city` is corpus-only and correctly absent from
  the api group (its P14 handshake race remains the one open post-deletion defect).
* **R3 is environmental, not the deletion**: the runner aborts because the documented zombie
  engine (PID 31680, needs a reboot) holds TCP 1234 — the same condition the post-merge pass
  measured, with all four active smoke worlds PASSing individually.
* **R2 is answered**: the deletion tree builds and has been building at every endgame commit
  (`make release` green, W1d verified against the built binary). R4 was answered by the
  post-merge corpus sweep on the F1 tree (377 worlds, zero new load errors) and has not been
  re-run post-deletion in this pass. R6's per-item gates are the Phase P ledger above; R7's
  GUI-gated items stay on the needs-a-human list; R8 stays owner-assumed until a Linux run.

Sweep thermal: whole-pass peak **66 °C** under the guard (ceiling 75, no kill); the W1d and
re-pin verification runs peaked 55–59 °C.
