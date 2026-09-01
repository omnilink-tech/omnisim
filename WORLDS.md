# WORLDS.md — the world dictionary

Every `.wbt` in the repo classified by purpose. Use this when you have a world filename and need to know what it's for; use [DEMOS.md](DEMOS.md) when you want to find a demo by what it does.

> **Canonical lighting recipe.** Every user-facing `.wbt` (demos, samples, RL, top-level robot demos — **332** of the 361 tracked worlds outside `tests/`) uses the same three-PROTO sky+sun recipe defined in [`docs/WORLD_RECIPE.md`](docs/WORLD_RECIPE.md): `OmniSimSky` + `DEF SUN OmniSimSun` + `DEF SUN_MARKER OmniSimSunMarker`. Test worlds under `tests/` are exempt. Exception: the omniworld-**generated** worlds under `distribution/generated_worlds/` currently ship with `TexturedBackground`, pending the emitter's migration to the OmniSimSky recipe. New worlds — human- or agent-authored — MUST follow the recipe; migrate via `python scripts/dev/migrate_world_recipe.py`.

> **Migration status.** All 73 demo worlds are now grouped by category under [`projects/samples/demos/worlds/<category>/`](projects/samples/demos/worlds/) (Phase 3 — done). EXTERNPROTO/texture/mesh paths rewritten to portable `omnisim://` form; URDFRobot `url` paths kept relative (the URDF loader does not honour the URL scheme). Only [`omnilink_launcher.omniworld`](projects/samples/demos/worlds/omnilink_launcher.omniworld) stays at the top.

---

## Categories at a glance

| Category | Root | Worlds | Purpose |
|---|---|---|---|
| [Demo worlds](#1-demo-worlds) | `projects/samples/demos/worlds/{chat,flagship,physics,showcase,environments,rendering,dev,misc,starter,portability}/` + the flat `omnilink_launcher.omniworld` | **115** (114 `.omniworld` + the 1 dual-read-proof `.wbt`) | User-facing showcases, chat demos, plus renderer-smoke & dev worlds. All ship publicly (`publish_deny.txt` holds no entry under this tree) |
| [Generated worlds](#2-generated-worlds) | `distribution/generated_worlds/` | 9 | Procedural scaffolds from the omniworld library (the `mars_small/big/max.wbt` scale variants are gitignored — regenerate them) |
| [Device sample worlds](#3-device-sample-worlds) | `projects/samples/devices/worlds/` | 45 | One world per sensor/actuator — pedagogical tour |
| [Rendering sample worlds](#4-rendering-sample-worlds) | `projects/samples/rendering/worlds/` | 2 | PBR reference + Sponza scene |
| [RL worlds](#5-rl-worlds) | `projects/policies/worlds/` (45, incl. 7 under `repro/`) + `projects/policies/research/worlds/` (99) | 144 | Shipped G1/H1 stand+walk+ghost+stair+turn+crawl+grasp worlds; research archive spans OmniQuad, B2, Go2 + older G1. Counts are for the **public snapshot** — the dev tree carries 47 + 108 = 155, the extra 11 being worlds held with their robot packages (see Removed in the changelog) |
| [API regression worlds](#6-api-regression-worlds) | `tests/api/worlds/` | 139 | Public C/C++/Python API surface |
| [Physics regression worlds](#7-physics-regression-worlds) | `tests/physics/worlds/` | 32 | Contact / friction / joints / determinism (authored against ODE; see the section note) |
| [Rendering regression worlds](#8-rendering-regression-worlds) | `tests/rendering/worlds/` | 8 | Mirror, normals, point sets, CAD-shape |
| [Parser / proto / cache test worlds](#9-parser--proto--cache-test-worlds) | `tests/{parser,protos,cache}/worlds/` | 102 (44 + 49 + 9) | `.wbt`/PROTO grammar + caching |
| [Other-API test worlds](#10-other-api-test-worlds) | `tests/other_api/worlds/` | 2 | C++ controller corners |
| [Manual / GUI-only tests](#11-manual--gui-only-tests) | `tests/manual_tests/worlds/` | 7 | Selection / paste-proto / GUI |
| [Benchmark stress worlds](#12-benchmark-stress-worlds) | `tests/benchmarks/optim/worlds/` | 0 | Directory empty — benchmark set currently runs the 5-world JSON config in [`tests/benchmarks/benchmark_worlds.json`](tests/benchmarks/benchmark_worlds.json), which points at existing physics/rendering/proto worlds |
| [CUDA stress worlds](#13-cuda-stress-worlds) | `tests/cuda/` | 3 | Granular pipeline stress |
| [Default empty world](#14-default-empty-world) | `tests/default/worlds/` | 1 | Fastest-possible boot |

---

## 1. Demo worlds

User-facing showcases. Cross-referenced in [DEMOS.md](DEMOS.md).

### 1a. Chat demos *(one robot, talk to it)*
`projects/samples/demos/worlds/chat/` — one `omnilink_<robot>.omniworld` per URDF robot, incl. the 3-arm `omnilink_multi_arm.omniworld`. *(Count check: `git ls-files projects/samples/demos/worlds/chat/ | grep -c '\.omniworld$'` → **16**. Of the 16, **15** are `omnilink_<robot>.omniworld`; the sixteenth is `omniarm6_talk.omniworld`. All 16 ship publicly — [`scripts/release/publish_deny.txt`](scripts/release/publish_deny.txt) holds no entry under this directory.)* See the [chat demos section in DEMOS.md](DEMOS.md#1-chat-demos--single-robot-natural-language-console) and the in-folder guide [`chat/OMNILINK_CHAT_DEMOS.md`](projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md).

### 1b. Flagship — `worlds/flagship/` (**24** worlds, all public)

*(Count check: `git ls-files projects/samples/demos/worlds/flagship/ | grep -c '\.omniworld$'` → **24**. [`scripts/release/publish_deny.txt`](scripts/release/publish_deny.txt) holds no entry under this directory, so all 24 ship publicly — the old "18 dev / 8 public" split predates the OmniArm replacement of the held robot packages and is gone. The table below lists the highlights; the rest are the `omniarm6_*` manipulation set, `husky_unseen_maze`, and `warehouse_omnilink`.)*

| World | Demo |
|---|---|
| `warehouse_industrial.omniworld` | Industrial warehouse scene |
| `husky_maze.omniworld`, `husky_maze_unknown.omniworld`, `husky_maze_corners.omniworld`, `husky_maze_visual.omniworld`, `husky_maze_blind.omniworld` | Husky Maze (5 difficulty tiers) |
| `omnilink_husky_swarm.omniworld` | Husky swarm coordination |
| `omnilink_smart_house.omniworld` | Smart house — an OmniLink agent runs a physics-backed home (hub bridge on :8766; see DEMOS.md §3) |

### 1c. Physics — `worlds/physics/`

| World | Purpose |
|---|---|
| `newton_smoke_test.omniworld` | Newton engine baseline |
| `newton_husky_smoke_test.omniworld` | Newton + Husky smoke |
| `newton_husky_swarm_drive.omniworld` | Newton swarm |
| `newton_static_collider_smoke.omniworld` | Newton static-collider smoke |
| `cuda_particles_smoke_test.omniworld` | CUDA particle pool |
| `granular_sand_demo.omniworld` | Cohesive granular sand |

Combat-oriented Newton worlds (`newton_husky_head_on*.wbt`, `newton_husky_combat_2.omniworld`) live under [`projects/robot_combat/worlds/`](projects/robot_combat/worlds/).

### 1d. Showcase — `worlds/showcase/`

| World | Purpose |
|---|---|
| `warehouse_husky.omniworld` | Default onboarding demo |
| `husky_fleet_arena.omniworld` | Indoor fleet — render/physics stress |
| `husky_rocks_traverse.omniworld` | Rocky terrain traverse |
| `jackal_drive.omniworld`, `turtlebot3_drive.omniworld` | Drive testbeds |
| `city_traffic.omniworld` | Urban traffic scene |

Combat showcase worlds (head-on, damage arena, brawl, duel) live under [`projects/robot_combat/worlds/`](projects/robot_combat/worlds/). For the BattleBox combat-sport scene — `battlebox_husky_proving.omniworld`, `battlebox_duel.omniworld`, `battlebox_royal_rumble.omniworld` — see the [Robot Combat README](projects/robot_combat/README.md#battlebots-league--battlebots).

### 1e. Environments — `worlds/environments/` (4)

| World | Purpose |
|---|---|
| `city.omniworld` | Urban environment backdrop |
| `desert_ruins.omniworld` | Outdoor rough terrain |
| `forest.omniworld` | Forest environment backdrop |
| `northgate_depot.omniworld` | Northgate Depot — a bare 25.8 × 16.8 m distribution-centre interior: concrete slab, 4 m rendered walls, three double rows of pallet racking, three dock-door panels. **Robot-free and prop-free by design** — drop a ground robot in and drive it. Hand-maintained (no generator) |

### 1f. Misc — `worlds/misc/` (2)

| World | Purpose |
|---|---|
| `cylinder_stack.omniworld` | Stacking physics; referenced from [physics-contact-and-collision-complexity.md](docs/developer/physics-contact-and-collision-complexity.md) |
| `contact_self_check.omniworld` | Newton self-collision contact check |

### 1g. Flat at top — `worlds/`

| World | Purpose |
|---|---|
| `omnilink_launcher.omniworld` | In-sim demo gallery (default entry point) |

### 1h. Rendering smoke — `worlds/rendering/` (19)

wgpu renderer smoke worlds (`*_wgpu_*_smoke.wbt`) — quick visual checks of the wgpu backend. Distinct from the pedagogical [rendering *sample* worlds](#4-rendering-sample-worlds) under `projects/samples/rendering/worlds/`. Not user demos; not in the launcher gallery.

### 1i. Dev — `worlds/dev/` (3)

Developer scratch/preview worlds (`*_preview.wbt`, `construction_site_dev.omniworld`) for in-progress scene iteration. Not user demos.

---

## 2. Generated worlds

Procedurally generated by the `omniworld` library. Most pair with a `.seed.json` capturing the generation parameters (the `*_night` background-validator worlds do not).

| World | Purpose |
|---|---|
| `flat_ground.wbt` | Minimal ground-plane baseline |
| `indoor_apartment.wbt` | Indoor scattered-object scene |
| `outdoor_desert.wbt` | Desert sand-dune outdoor scene |
| `outdoor_forest.wbt` | Forest outdoor scene |
| `urban_block.wbt` | City block |
| `warehouse.wbt` | Indoor warehouse scaffold |
| `mars.wbt` (+ regenerable `mars_small/big/max.wbt` scale variants, gitignored), `mars_night.wbt` | Mars regolith terrain at varying scales |
| `earth_night.wbt` | Earth nightside background validator |

Regenerate: see [`omnisim` Python module](omnisim/) and the omniworld test suite at [`tests/python/omniworld/`](tests/python/omniworld/).

---

## 3. Device sample worlds

`projects/samples/devices/worlds/` — one world per OmniSim device class (the device set is inherited from upstream Webots and unchanged). Pedagogical tour, NOT regression tests (those live under `tests/api/worlds/`).

Devices covered: accelerometer, altimeter, battery, brake, bumper, camera (+ auto-focus / motion-blur / noise-mask / recognition / segmentation / spherical), compass, connector, coupled-motors, display, distance-sensor, emitter-receiver, encoders, force / force3d sensors, GPS (+ lat-long), gyro, IMU, inertial-unit, laser-pointer, LED, lidar, light-sensor, linear-motor, motor / motor2 / motor3, pen, position-sensor, propeller, radar, range-finder, receiver-noise, speaker (+ text-to-speech), supervisor, track, vacuum-gripper.

---

## 4. Rendering sample worlds

`projects/samples/rendering/worlds/`

| World | Purpose |
|---|---|
| `physically_based_rendering.omniworld` | PBR material reference |
| `sponza.omniworld` | Sponza scene — rendering stress |

---

## 5. RL worlds

The policy worlds live under two roots:

- **`projects/policies/worlds/` — 45 tracked worlds** (G1, H1), of which **38 at the top level** + **7 under [`repro/`](projects/policies/worlds/repro/)**: the current stand / walk / ghost deploy, cube-throw robustness, stair-climb, footwork-turn, crawl and arm-grasp campaigns. All run the Newton physics backend. *(The dev tree and the public snapshot now hold the same set — the two worlds that used to differ went with the NASA package on 2026-08-22. Count check: `git ls-files 'projects/policies/worlds/' | grep -c '\.omniworld$'`.)*
- **`projects/policies/research/worlds/` — 99 tracked research/archive worlds** (108 in the dev tree; 9 are held with their robot packages) spanning OmniQuad (34, none held), B2, Go2 and older G1 experiments — a mix of `*_smoke`, `*_deploy`, `*_ghost_preview`, and training templates. De-prioritised research track.

> **`_show` suffix = the hologram variant.** A `*_show.wbt` is the same scene plus the **GHOST hologram** node, and is loaded via `SHOW_WORLD=` by the demo script that owns it (the plain world has no ghost). Demo scripts live in [`projects/policies/demos/`](projects/policies/demos/) — see [DEMOS.md §2](DEMOS.md#skill-demos--baton-sequences).

| Shipped world (`projects/policies/worlds/`) | Purpose |
|---|---|
| `g1_hstand_deploy.omniworld` | G1 whole-body active-balance stand deploy |
| `g1_walk_orig.omniworld`, `g1_walk_orig_void.omniworld`, `g1_walk_bigfoot.omniworld`, `g1_walk_ghost2.omniworld` | G1 in-engine walk deploy variants |
| `g1_ghost_preview.omniworld` | G1 ghost-reference preview |
| `g1_hstand_cubethrow.omniworld`, `g1_hstand_cuberain.omniworld`, `g1_hstand_cubethrow_bigfoot.omniworld` | G1 stand push-recovery / cube-throw robustness |
| `g1_walk_ghost_wide.omniworld`, `g1_walk_puppet.omniworld` | G1 walk on the ORIGINAL foot (not bigfoot) beside a phase-locked ghost hologram — the visible gap between them is the balance work. **`g1_walk_puppet.omniworld` is the stage for the flagship walk + most BATON sequences** (decent walker, box delivery ×2, walk→turn→walk) — ⚠️ they run it on the **weight-bearing puppet harness** (`HARNESS_LAM0=0.9`, `KZ=2000`, up to 700 N ≈ 2× body weight), i.e. **not a free-standing walk**; see [DEMOS.md §2](DEMOS.md#humanoid-rl-deploy) |
| `g1_climb_stairs_demo3.omniworld` | ⭐ **G1 stair-climb demo** (shipped 2026-07-08): full 5-step live climb at 3 cm risers via the walking-ghost + terrain recipe, **legs-only** (`HARNESS_KZ=0`) — run via [`run_climb_stairs.sh`](projects/policies/demos/run_climb_stairs.sh), or [`run_climb_stairs_stand.sh`](projects/policies/demos/run_climb_stairs_stand.sh) to climb **and hold a verified stand on the top landing**. ⚠️ 3 cm is the measured **ceiling** (4 cm ≈ 2 steps, 5 cm ≈ 0) — not "stairs: solved" |
| `g1_climb_stairs_demo3_show.omniworld` | ⭐ **The hologram (`SHOW_WORLD`) target of the shipping stair-climb demo** — `g1_climb_stairs_demo3` + the ghost hologram walking alongside at y = +1.1. Used by `run_climb_stairs_stand.sh` via `SHOW_WORLD=` |
| `g1_climb_stairs.omniworld`, `g1_climb_stairs_walk3.omniworld` | G1 stair-climb TRAIN worlds (shared StairProfile, train == deploy geometry; 7 cm and 3 cm risers) — generated by `make_stair_worlds.py` |
| `g1_climb_stairs_runway.omniworld` | G1 stair-climb TRAIN variant with a long flat **run-up** before the staircase (5 steps @ 3 cm starting at x ≈ 4.13 m instead of ≈ 1.33 m) — lets the walker reach a settled gait before the first riser. Generated by `make_stair_worlds.py` |
| `g1_climb_stairs_synth.omniworld` | G1 stair-climb TRAIN world for the SYNTHESIZED ghost (`ghost_stair_climb_synth_lut.json`) — exactly the 7 cm StairProfile the ghost was solved against |
| `g1_climb_stairs_synth3.omniworld`, `g1_climb_stairs_synth3_show.omniworld` | The 3 cm counterpart: TRAIN world for the synthesized climb ghost (+ its hologram `_show` variant). Part of the **clean-gait successor** route — ⚠️ currently **BLOCKED** on a live-plant contact delta at tread push-off (scores 1.000 in the clean plant, yo-yos live) |
| `g1_stairs3_2step.omniworld`, `g1_stairs3_2step_show.omniworld` | Cut-down 2-tread version of the 3 cm synth-ghost staircase (+ hologram `_show` variant) — the bring-up rung of the synth-climb ladder |
| `g1_climb_stairs_preview.omniworld` | G1 stair-climb ghost preview (hologram climbing the 5 steps) |
| `g1_climb_stairs_puppet.omniworld` | G1 stair-climb deploy on the BATON walk→climb→stand puppet rig |
| `g1_box_grasp.omniworld` | G1 **physics box grasp** world (real Newton box + carts) — the robot picks the box up with its ARMS, zero kinematic writes. ⚠️ **EXPERIMENTAL / BLOCKED** on a warp contact-kernel defect; run via [`run_box_grasp.sh`](projects/policies/demos/run_box_grasp.sh). *(Its in-file header comment is stale — copied from the walk world.)* |
| `g1_step_turn_preview.omniworld` | Step-turn ghost preview — a kinematic ghost replays the constructed 90° footwork turn (footstep plan + IK, COM-over-support) |
| `g1_turn_demo.omniworld` | Footwork-turn stand-alone demo: the real robot does the ~90° turn (wtz=0, pure footwork) beside the turning-reference ghost |
| `g1_turn_ghost_compare.omniworld` | Two 90° step-turn ghosts side by side — the hand-tuned reference vs the COM-solved one, so the difference is visible |
| `g1_crawl_train.omniworld` | G1 commando-crawl training world — crawl-collider URDF (hands/knees/forearms/torso boxes) + prone spawn |
| `g1_crawl_ghost_preview.omniworld` | Commando-crawl ghost-only preview (hologram on forearms + knees; GHOST-FIRST: design → show → agree → train) |
| `h1_stand_deploy.omniworld`, `h1_hstand_cubethrow.omniworld` | H1 stand deploy + robustness |
| [`repro/warp_contact_{A,B,C,D,E,H,I}*.wbt`](projects/policies/worlds/repro/) (7) | **Warp contact-kernel repro worlds** (2026-07-10) — minimal one-pair-per-world scenes for the grasp campaign's NaN hunt: free box on floor / on cart, box stack, G1 standing near a box (far / at cart), G1 alone, G1 hand-touch. Physics-only (no policies) so a CPU-vs-warp A/B is clean. Driven by `run_warp_contact_repro.sh`; these are the blocking evidence behind the ⚠️ box-grasp demo |

Representative research worlds (`projects/policies/research/worlds/`): `omniquad_rl.omniworld` (OmniQuad PPO training template), `omniquad_rl_deploy.omniworld` (loads exported ONNX policy), `rl_deploy.omniworld` (generic deploy harness), `omniquad_newton_demo.omniworld`.

List the full current sets with `ls projects/policies/worlds/*.wbt` and `ls projects/policies/research/worlds/*.wbt`.

Pipeline doc: [`docs/developer/archive/rl-pipeline.md`](docs/developer/archive/rl-pipeline.md) (and the canonical status in [`docs/developer/rl-current-state.md`](docs/developer/rl-current-state.md)). README: [`projects/policies/README.md`](projects/policies/README.md).

---

## 6. API regression worlds

`tests/api/worlds/` — exhaustive coverage of the public C/C++/Python API. Run with [`tests/test_suite.py`](tests/test_suite.py).

Major groups:

| Group | Worlds (representative) |
|---|---|
| Device APIs | `accelerometer`, `altimeter`, `battery`, `brake`, `compass`, `display*`, `gps*`, `gyro`, `imu`, `inertial_unit`, `led`, `lidar*`, `motor*`, `radar*`, `range_finder*`, `track*`, `vacuum_gripper` |
| Camera | `camera_color*`, `camera_image_update`, `camera_noise_mask`, `camera_recognition`, `camera_revert` |
| Distance sensor | `distance_sensor_*` (enable/disable, infra-red, laser, sonar, resolution, vs-mesh, vs-transformed-planes) |
| Emitter/Receiver | `emitter_receiver_*` (determinism, enable/disable, IR, IR-aperture, radio, serial) |
| Pen | `pen.wbt` plus per-primitive pen tests (`pen_box`, `_capsule`, `_cone`, `_cylinder`, `_elevation_grid`, `_indexed_face_set`, `_mesh`, `_plane`, `_sphere`, scaled variants) |
| Motions | `motions_loop`, `motions_regular`, `motions_reverse` |
| Robot lifecycle | `robot_data`, `robot_multiple_step`, `robot_nested`, `robot_node`, `robot_parallel_step`, `robot_synchronous_time`, `robot_time_consecutive_packets`, `robot_wait_for_user_input_event`, `robot_window_html` |
| Supervisor | `supervisor_*` × ~30 (animation, force/torque, get-from-def, import/remove, reset, set-position-orientation, etc.) |
| Touch sensors | `touch_sensor_bumper`, `_force`, `_force3d`, `_kinematic` |

---

## 7. Physics regression worlds

`tests/physics/worlds/` — contact / friction / joints / determinism.

> ⚠️ **This suite was authored against ODE, which was deleted on 2026-08-08 (commit `bdc02139`).**
> Newton/MuJoCo is now the only backend, so some of these worlds exercise behaviour that has
> changed or is currently absent — in particular the `damping*` worlds (the `Damping` node is inert
> on Newton), the `ball_joint*` and `hinge_2*` worlds (motorised `BallJoint` / `Hinge2Joint` do not
> actuate), and anything asserting a force-type `TouchSensor` reading (it reads 0 N). Treat a PASS
> here as "the world loads and steps", not as a physics verdict; re-baselining this suite against
> Newton is **outstanding work**, not something already done.

| World | Tests |
|---|---|
| `ball_joint_reset.omniworld`, `ball_joint_vs_hinge_joints.omniworld` | Ball joint reset and hinge equivalence |
| `collision_multiple_trimesh.omniworld` | Multi-mesh collision |
| `connector_static_autolock.omniworld` | Connector behaviour |
| `contact_points.omniworld` | Contact-point reporting |
| `coupled_motor.omniworld` | Coupled motors |
| `damping.omniworld`, `hinge_*_damping*.wbt` | Damping models |
| `floating_point_precision.omniworld` | Deterministic stepping |
| `dynamic_*_rays.wbt`, `static_*_rays.wbt` | Sensor-ray dynamic/static behaviour |
| `elevation_grid_rotation.omniworld` | Elevation-grid rotation |
| `hidden_parameter_single.omniworld` | Hidden-parameter resume |
| `joint_hard_limits_after_move.omniworld`, `hinge_joint_slot.omniworld` | Joint limits, slots |
| `kinematic_geometry_update.omniworld`, `runtime_geom_update.omniworld` | Live geometry mutation |
| `move_hinge_2_joint_with_suspension.omniworld` | Suspended hinge-2 |
| `rolling_friction.omniworld` | Rolling friction |

---

## 8. Rendering regression worlds

`tests/rendering/worlds/`

| World | Tests |
|---|---|
| `cadshape_node.omniworld` | CadShape node |
| `complex_relative_asset_resolution.omniworld`, `derived_proto_relative_asset_resolution.omniworld`, `relative_asset_resolution.omniworld` | Asset path resolution |
| `mirror.omniworld` | Mirror rendering |
| `normals.omniworld` | Normal-map rendering |
| `point_set.omniworld` | Point-cloud rendering |
| `smooth_shaded_cylinder.omniworld` | Smooth shading |

---

## 9. Parser / proto / cache test worlds

`tests/parser/worlds/`, `tests/protos/worlds/`, `tests/cache/worlds/` — grammar errors, valid forms, PROTO derivation, slot containers, retrieval, caching.

Highlights:
- **Parser** (~45 worlds): missing/extra brackets, malformed templates, recursive proto, JS comments, unknown fields. Expected results in [`tests/parser/expected_results.txt`](tests/parser/expected_results.txt).
- **Protos** (~50 worlds): nested templates, derived protos, slot containers, parameter passing.
- **Cache** (~10 worlds): proto retrieval, texture caching, backwards compatibility.

---

## 10. Other-API test worlds

`tests/other_api/worlds/` — C++ controller specifics.

| World | Tests |
|---|---|
| `cpp_device_with_same_name.omniworld` | Device-name clashes |
| `cpp_import_device.omniworld` | Device import |

---

## 11. Manual / GUI-only tests

`tests/manual_tests/worlds/` — scenarios that need a human in front of the GUI.

Covers: paste-proto-in-def-node, derived-proto-solid-physics, insertion-in-nested-parameter, interaction-with-solid-reference-model, modify-proto-template-field, selection-when-procedural-proto-regeneration, transform-proto-parameter.

---

## 12. Benchmark stress worlds

`tests/benchmarks/optim/worlds/` is currently **empty** — the perf-sweep `.wbt` files have been removed.

The active benchmark set is now the 5-world JSON config in [`tests/benchmarks/benchmark_worlds.json`](tests/benchmarks/benchmark_worlds.json), which re-uses existing worlds (`empty.wbt`, `tests/rendering/worlds/normals.omniworld`, `tests/physics/worlds/contact_points.omniworld`, `projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld`, `tests/protos/worlds/template_deterministic.omniworld`) as smoke benchmarks across the startup / sensor-frame / physics-step / contact-heavy-physics / world-load categories.

Driver: [`tests/benchmarks/run_benchmarks.py`](tests/benchmarks/run_benchmarks.py), [`optim_bench.py`](tests/benchmarks/optim_bench.py), baselines in [`tests/benchmarks/optim-baseline/`](tests/benchmarks/optim-baseline/).

The original parameterised sweep (`chunky_*`, `noisy_*`, `many_cameras_*`, `many_robots_*`) was a Webots-era stress harness; regenerate from scratch under a new strategy if perf-regression coverage is needed again.

---

## 13. CUDA stress worlds

`tests/cuda/`

| World | Purpose |
|---|---|
| `granular_group_load.omniworld` | Granular group load test |
| `warehouse_husky_granular.omniworld`, `warehouse_husky_granular_massive.omniworld` | Granular pipeline stress |

Benchmark numbers: [`tests/cuda/bench_results.md`](tests/cuda/bench_results.md).

---

## 14. Default empty world

`tests/default/worlds/empty.omniworld` — smoke baseline; fastest-possible boot.

---

## How to add a new world

See the recipe in [ARCHITECTURE.md](ARCHITECTURE.md#how-to-add-a-new-world).
