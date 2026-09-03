# AGENTS.md — Running OmniSim as an AI Coding Agent

**This file is for AI coding agents** (Claude Code, Codex, Cursor, custom agent harnesses) working inside a fresh clone of OmniSim. It tells you exactly what to do to get a demo running. The conventions below match the [`AGENTS.md` open standard](https://agents.md/), and tools that respect that standard will load this file automatically.

If you are a human, this file also works as a quick "what does OmniSim do and how do I run a demo" cheat sheet — the [README](README.md) and the [Developer Quickstart](docs/developer/quickstart.md) have the long form.

---

## 0. For agents: read this first (60 seconds)

**Call it OmniSim.** The simulator is OmniSim — its own product, with substantial additions over the upstream Webots engine it forked from (URDF importer, agent-facing HTTP harness, capture / cinema pipeline, OmniLink agent runtime, omniworld procedural world generator, RL pipeline, CUDA granular-physics, multi-instance parallel runs, OmniSim Wire Protocol). When you talk to the user about the running simulator, the binary, the env vars, the URL scheme — say *OmniSim*. Use "Webots" only when explicitly referring to upstream (the GitHub repo, the file-format syntax inherited from VRML, the original PROTO conventions).

A scenario = one `.wbt` world + one or more controllers + (optionally) a long-running harness or bridge service that you, the agent, drive over HTTP. **You** drive the simulator — you do not need to ask the user to launch, reload, or step it.

### Run this first turn

```bash
python -m omnisim doctor
```

Reports the truth about *this clone right now*: OmniSim binary path, **whether there is a physics runtime at all**, engine↔libController ABI compatibility (the IPC-nonce gate, commit `6eea9d76` — a controller lib older than the engine silently hangs *every* controller at zero ticks while a headless run still prints PASS), port status (`6789` harness, `6790` supervisor, `6791` capture), worlds present, recent commits. Don't guess at the state — check it. `--json` for machine-readable.

**Read the VERDICT line and branch on it before anything else in this file.** Two answers mean STOP:

- `binary NOT FOUND` — this clone is not built. `msys64/` is gitignored, so a fresh `git clone` has no engine and every row of the table below assumes one. Build first (§2): `build_omni.bat` on Windows, `bash scripts/install/linux_bootstrap.sh` on Linux, then `make -C src/omnisim bundle-newton-runtime`.
- `physics FAIL` — the Newton runtime is absent. Newton is the ONLY backend, so this is not a degraded mode: nothing falls, nothing collides, no grasp holds, and every run still exits 0. `doctor` prints the fix for your platform.

`doctor` exits non-zero on either, so an agent branching on `$?` gets the right answer. It did not until 2026-08-28 — it reported the fault and exited 0.

### First moves by task type

Each row states the FINAL rule; the full row (measurements, dates, history) is verbatim in [docs/developer/agents-first-moves.md](docs/developer/agents-first-moves.md), one section per row.

| User asks for | Your first move |
|---|---|
| **You are in an MCP client** (Claude Code, Cursor, Claude Desktop) and can see `omnisim` tools | `harness_status` first; unreachable → `python -m omnisim harness` (it reports on the HARNESS, not the engine). Then `load_world {"path": ..., "light": true}` → `get_scene_tree` → `frame` → `screenshot`. [full](docs/developer/agents-first-moves.md#mcp-client) |
| **Run / see a demo** | `python -m omnisim demo` / `demos`; one world: `python -m omnisim run-world <w>.omniworld` or `run-headless <w>.omniworld` (omit `--duration` unless the run must WATCH). §3. [full](docs/developer/agents-first-moves.md#run-a-demo) |
| **Make a legged robot do a motion** (walk, gait styling, expressive motion — any legged robot) | ⭐ **SHADOWING**: [`projects/policies/training/README.md`](projects/policies/training/README.md), [ghost-design-rules.md](docs/developer/ghost-design-rules.md), [rl-current-state.md](docs/developer/rl-current-state.md). ⚠️ The G1 walk is on a WEIGHT-BEARING balance harness — never "free-standing". [full](docs/developer/agents-first-moves.md#legged-motion) |
| **Make a *reusable skill*, or compose skills into a demo** (walk, turn, carry, stand, climb → a BATON sequence) | ⭐ **SKILL LIBRARY**: `python -m omnisim policy list` / `sequence <name>` / `preview` / `train` / `verify-demos` (`policy --help` is the live list). [skill-library.md](docs/developer/skill-library.md). [full](docs/developer/agents-first-moves.md#skill-library) |
| **Author or edit a world (`.omniworld`)** | ⚠️ `.omniworld`, never `.wbt`. `python -m omnisim harness` → **`POST /world/load {"path": ..., "light": true}`** → edit → **`POST /world/sync`** (325 ms pose-only, 2818 ms structural) → `POST /world/screenshot`. §5. [full](docs/developer/agents-first-moves.md#author-a-world) |
| **Make a gripper HOLD something** (pick-and-place, a pinch grasp, carrying a part) | ⭐ Read [docs/guide/friction-grasp.md](docs/guide/friction-grasp.md) FIRST: `newtonGroundMu` (`coulombFriction` is NOT read, bar one bridged case the guide names), `newtonContactKe`, `newtonImpratio`, `basicTimeStep 8`; `setForce` does NOT enter force mode (`OMNISIM_NEWTON_TORQUE_MODE=1` does). Working example: `python -m omnisim run-headless projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld --duration 45`. [full](docs/developer/agents-first-moves.md#gripper-grasp) |
| **Convert a STEP/STP CAD file into a robot** | `python scripts/dev/step_to_urdf.py <in.step> <robot_dir> --name <n> --up z --ground bottom`; verify the up-axis. [step-to-urdf.md](docs/developer/step-to-urdf.md). [full](docs/developer/agents-first-moves.md#step-to-urdf) |
| **Debug a misbehaving controller** | Harness on `:6789` + `GET /sim/events?since=...&log_since=...`; branch on `controller.log`, `joint.limit_hit`, `contact.began`, `damage.*`. [full](docs/developer/agents-first-moves.md#debug-a-controller) |
| **Inspect a running scene** | `GET /robots`, `/robot/<def>/joints`, `/robot/<def>/devices`, `/sim/contacts`, `/sim/grips`. [full](docs/developer/agents-first-moves.md#inspect-a-scene) |
| **Aim a camera at something** (screenshot, framing, "why is my shot empty?") | `POST /scene/frame {"def": "<DEF>"}` (aim + distance + proof), `GET /scene/visible`, `GET /scene/tree?bounds=1` — never iterate on screenshots. [full](docs/developer/agents-first-moves.md#aim-a-camera) |
| **Generate a synthetic dataset** (RGB + depth + instance segmentation, domain-randomized) | ⭐ `python scripts/dev/synth_data.py --world <w> --out <dir> --samples N --seed S`; main-view dump (`OMNISIM_WGPU_SYNTH_DUMP`), desktop session, OFFLINE only. [full](docs/developer/agents-first-moves.md#synthetic-dataset) |
| **Cinematic capture** | `python -m omnisim capture` (port `6791`); `/capture/sequence` → mp4. [scripts/capture/README.md](scripts/capture/README.md). [full](docs/developer/agents-first-moves.md#cinematic-capture) |
| **Cinematic *video*** (storyboard → branded multi-aspect deliverables, with a vision-critique reshoot loop) | `python -m omnisim cinema render <storyboard.json>`. [scripts/cinema/README.md](scripts/cinema/README.md). [full](docs/developer/agents-first-moves.md#cinematic-video) |
| **Agent Build Film** (an AI agent tells the evidence-led story of what it built in OmniSim) | ⭐ [`AGENT_BUILD_FILMS.md`](scripts/cinema/AGENT_BUILD_FILMS.md): `python -m omnisim cinema agent-build-new --title "<title>"` → `agent-build-validate` → `agent-build-voice` → `agent-build-render` → `agent-build-verify`. [full](docs/developer/agents-first-moves.md#agent-build-film) |
| **Damage testing** | `python -m omnisim damage`; `python -m omnisim damage-regression`. [full](docs/developer/agents-first-moves.md#damage-testing) |
| **Test that MANY worlds load** (a sweep, a corpus, CI smoke) | ⭐ `python -m omnisim validate-worlds <worlds...>` ([`batch_validate.py`](scripts/dev/batch_validate.py)): one engine hot-reloads each world (5.0× faster); LOAD check only. [full](docs/developer/agents-first-moves.md#many-worlds-load) |
| **Test that a world loads** | ⭐ `python -m omnisim run-headless <world> --until-finalized --fail-on-warning` (15.52 s → 6.37 s → **5.5 s** as of 2026-09-02; fleet arena 8.0 → 4.8 s: the run now stops `--settle-after-step` 0.5 s after the first physics step instead of sitting out a 2 s grace). PASS ≠ sane physics: `--duration` only to OBSERVE, `--fail-on-runaway` to certify (§3b). [full](docs/developer/agents-first-moves.md#one-world-loads) |
| **Run K worlds in parallel** (batch validation, fleet bench, agent-vs-agent) | N headless `omnisim-bin` processes; ports auto-picked in `[1234, 1294]`; `OMNISIM_LOG_PATH=<unique>` per child. §3e. [full](docs/developer/agents-first-moves.md#parallel-worlds) |
| **Ask what OmniSim can actually SIMULATE** ("can it do cloth / heightfields / a force sensor / N robots?") | ⭐ Read the generated lane-4 matrix [lane4-capability-matrix.md](docs/benchmarks/lane4-capability-matrix.md) (51 probes, 95.7% work, 2026-09-01) — it beats this file; top up with [`merge_coverage.py`](tests/benchmarks/omnibench/lane4/merge_coverage.py), never a text editor. [full](docs/developer/agents-first-moves.md#capability-matrix) |
| **Benchmark physics, throughput, or "how does OmniSim compare?"** | ⭐ OmniBench ([`tests/benchmarks/omnibench/`](tests/benchmarks/omnibench/), [`SPEC.md`](tests/benchmarks/omnibench/SPEC.md), `run_all.py`). ODE rows are history; determinism is bitwise on CPU `mujoco`, refuted on `mujoco_warp`; Newton's case is batching, not per-step cost; ⛔ never benchmark Isaac (NVIDIA SLA §8.9). [full](docs/developer/agents-first-moves.md#omnibench) |
| **Ask whether an AGENT can actually get a job done here** (sim-vs-sim, tool-surface ablations, "is our agent surface worth anything?") | AgentBench ([`tests/benchmarks/agentbench/`](tests/benchmarks/agentbench/), [`run_agentbench.py`](tests/benchmarks/agentbench/run_agentbench.py) `--fake-sim`). ⚠️ The first A/B (bare shell tied 10/10 with fewer calls, n=1) is neither a win nor a refutation. [full](docs/developer/agents-first-moves.md#agentbench) |
| **Anything about VRML / `.wbt` primitives** (`Robot`, `Supervisor`, `.wbt` syntax, the controller library) | Webots syntax + `URDFRobot`, `omnisim://`. ⚠️ Gone with ODE: `Fluid`/`ImmersionProperties` (hard ERROR), ODE tuning fields (parsed, not read); `Radio`/`Microphone` retired. `TouchSensor` WORKS. Check [docs/reference/](docs/reference/). [full](docs/developer/agents-first-moves.md#vrml-primitives) |
| **How to support / sponsor the project** | [github.com/sponsors/omnilink-tech](https://github.com/sponsors/omnilink-tech), [SPONSORS.md](SPONSORS.md). [full](docs/developer/agents-first-moves.md#sponsoring) |

Other CLI verbs (`python -m omnisim --help` is the live list): `validate-worlds`, `verify-install`, `proto`, `agent`, `validate-urdf`, `build`, `compile-commands`, `benchmarks`, `key`, `byok`.

### Hard-won rules (don't relearn these)

FINAL state only, one line each; every measurement, date and self-correction is verbatim in [docs/developer/agents-hard-won-rules.md](docs/developer/agents-hard-won-rules.md), one section per rule.

- ⚠️ **`.omniworld` is the extension; `.wbt` is read forever, never written.** Engine: [`OmWorldFileFormat.hpp`](src/omnisim/core/OmWorldFileFormat.hpp) is the single funnel; Python: accept both. `distribution/`, `tests/benchmarks/` and two proof worlds stay `.wbt` on purpose. [full](docs/developer/agents-hard-won-rules.md#world-extension)
- **You drive the harness** — `POST /world/load` once, `POST /world/sync` after edits, `/sim/reset`, `/sim/step`; never kill the `omnisim-bin` the user is watching. [full](docs/developer/agents-hard-won-rules.md#drive-the-harness)
- ⚠️ **Load the harness with `"light": true`** — 2026-08-29, `husky_fleet_arena`: `/sim/step 1` **6–35 ms light vs 573–606 ms full**. Quiet in light: `/sim/grips`, `contact.*`/`grip.*`/`joint.limit_hit` events; **`/sim/contacts` still answers**. [full](docs/developer/agents-hard-won-rules.md#light-mode)
- ⚠️ **When an agent drives a robot badly, suspect the TOOL before the prompt** — measure the primitive (`turn` delivered 56.7% of its command; the control law was the bug). Return `{commanded, achieved, error, settled}`, never an echo. [tool-design-for-agents.md](docs/developer/tool-design-for-agents.md). [full](docs/developer/agents-hard-won-rules.md#suspect-the-tool)
- ⛔ **Never kill an `omnisim-bin` you did not spawn** — engines coexist; `taskkill /F` reads like a crash in the victim's log. Check its command line and wait, or use your own ports. [full](docs/developer/agents-hard-won-rules.md#never-kill-engines)
- **Kinematic-only props** (conveyors, tables, markers): NO `boundingObject` — hulls snag and lock joints like an IK bug. [full](docs/developer/agents-hard-won-rules.md#kinematic-props)
- **Robots with a base + arm** MUST tuck arm + torso on bridge spawn, or wheel slip mimics a friction bug. [full](docs/developer/agents-hard-won-rules.md#base-plus-arm)
- **`/robot/<def>/sensor/<name>` returns 501 by design** — use `/joints` or the owning controller. [full](docs/developer/agents-hard-won-rules.md#sensor-501)
- **`/sim/state` is session metadata plus the sim clock**, not scene state. [full](docs/developer/agents-hard-won-rules.md#sim-state)
- **An empty `/sim/contacts`:** no body sleep on Newton, `?wake=1` is a no-op, native readback ON since 2026-08-07 (`OMNISIM_NEWTON_NATIVE_CONTACTS=0` blinds it — don't). [full](docs/developer/agents-hard-won-rules.md#empty-contacts)
- ⚠️ **Every triangle-mesh collider is silently convexified** (a bowl is a lump). Concave = several convex pieces in a `Group` **plus `WorldInfo.newtonCompoundColliders TRUE`** — the FALSE default drops every child but the first. [full](docs/developer/agents-hard-won-rules.md#mesh-convexified)
- ⛔ **A `Cone` boundingObject kills the whole world** (`KeyError(9)`); author an `IndexedFaceSet`. [full](docs/developer/agents-hard-won-rules.md#cone-collider)
- **Read the FIELDS, not the pixels:** `GET /scene/node/<def>` reports `boundingObject` and `physics`; a Solid collides only through its `boundingObject`. [full](docs/developer/agents-hard-won-rules.md#read-the-fields)
- **Watch `dropped_sup` / `dropped_log` on `/sim/events`** — non-zero means poll faster or raise `limit`. [full](docs/developer/agents-hard-won-rules.md#dropped-events)
- **Statics are solid under Newton (since 2026-08-07) and contacts see them.** `WorldInfo.newtonStatics TRUE` wins; `OMNISIM_NEWTON_STATICS=0` is the value-parsed revert; the implicit z=0 plane only substitutes for a dropped `Plane` (`OMNISIM_NEWTON_GROUND_PLANE=1` restores it). [full](docs/developer/agents-hard-won-rules.md#static-floors)
- ⚠️ **>~8 wheeled robots on `mujoco_warp`: raise `WorldInfo.newtonNjmax` / `newtonNconmax`** — the pinned 256 overflows SILENTLY (peak `nefc` 336 at N=16). `OMNISIM_NEWTON_CONSTRAINT_STATS=<N>` measures the real peak. [full](docs/developer/agents-hard-won-rules.md#njmax-overflow)
- **`physicsBackend` parses; every value runs on Newton.** `"ode"` is a retired selector: it warns once per world and the Solid runs on Newton (until 2026-09-02 it left the Solid with NO physics — that trap is closed). Omit the field or write `"newton"`; inside `WorldInfo` it is an ERROR. [full](docs/developer/agents-hard-won-rules.md#backend-fields)
- **Engine defaults (v5): Newton/MuJoCo + wgpu are the ONLY physics and renderer.** No runtime = no physics (`doctor`). Ray sensors are DECLINED on `mujoco_warp`. ✅ RESOLVED: `Hinge2Joint` motors (2026-08-17), `BallJoint` motors (2026-09-01; per-axis limits still unenforced), `TouchSensor` (2026-08-13/15; bumper needs `boundingObject`, force needs `Physics` + `+X` aimed + `lookupTable [ ]`); limit-less motors promote to servos on the first `setPosition()` (`OMNISIM_NEWTON_PROMOTE_SERVO=0` reverts). ⛔ Dead: `Track` propulsion; loop-closing `SolidReference` (NO physics, headless FAILs). ODE env vars are ignored; `OMNISIM_REQUIRE_NEWTON` is PRESENCE-gated (unset to disarm). Runtime newton 1.5.0; WREN deleted 2026-08-23. [full](docs/developer/agents-hard-won-rules.md#engine-defaults)
- ⚠️ **The physics runtime runs from the BUNDLE**, not [`omnisim_newton_runtime.py`](src/omnisim/physics/omnisim_newton_runtime.py) — re-stage after every edit (`scripts/packaging/bundle_newton_runtime.py --mode vendor`). [full](docs/developer/agents-hard-won-rules.md#runtime-bundle)
- **Proof Newton drove a run is the sidecar `<OMNISIM_LOG_PATH>.newton.json`**, not `imports OK`; no sidecar = never finalized. wgpu-only rendering; `renderBackend "wren"` is a warned no-op. [full](docs/developer/agents-hard-won-rules.md#newton-sidecar)
- ⚠️ **Humanoid demos run on a WEIGHT-BEARING balance harness** (≈700 N, ±350 N·m) — say so; stair climb is legs-only (`HARNESS_KZ=0`, 3 cm riser ceiling). [full](docs/developer/agents-hard-won-rules.md#balance-harness)
- ⚠️ **Attribute every result to a MACHINE**: `python projects/policies/common/env_fingerprint.py` before quoting a number. [full](docs/developer/agents-hard-won-rules.md#machine-attribution)
- ⚠️ **ROS 2 SHIPS** ([`packages/omnisim-ros2/`](packages/omnisim-ros2/)): Tier 1 services; Tier 2 topics + sensors on the robot's bridge (URDF robots need `OMNISIM_URDF_USE_SENSORS=1`); Tier 3 `ros2_control`; Nav2 up 2026-08-31 (planning only). `--engine-mode realtime` for sensor stacks; the engine stays ROS-free. [ros2-integration.md](docs/developer/ros2-integration.md). [full](docs/developer/agents-hard-won-rules.md#ros-2)
- **Every new world outside `tests/` uses the [canonical lighting recipe](docs/WORLD_RECIPE.md):** `OmniSimSky {}`, `DEF SUN OmniSimSun {}`, `DEF SUN_MARKER OmniSimSunMarker {}`. [full](docs/developer/agents-hard-won-rules.md#lighting-recipe)
- **RL: train IN-ENGINE on the LOCAL GPU; RunPod only when the owner asks.** Humanoids [`run_walk_rl.sh`](projects/policies/training/run_walk_rl.sh) → [`g1_walk_recipe.py`](projects/policies/training/g1_walk_recipe.py); quadrupeds [`run_quad_walk_rl.sh`](projects/policies/training/run_quad_walk_rl.sh) → [`quad_walk_recipe.py`](projects/policies/training/quad_walk_recipe.py). [full](docs/developer/agents-hard-won-rules.md#rl-venue)
- **A headless load check got 21–40% faster on 2026-09-02, and every lever has a value-parsed hatch** (fleet arena `run-headless --until-finalized` 8.03 → 4.81 s, warehouse_industrial 8.83 → 6.35 s, cloth 7.01 → 5.52 s; engine-only start → first step 5.95 → 3.91 s on the fleet arena). The engine cached the per-controller Python probe that ran two `python -c` subprocesses per controller start (`OMNISIM_PYTHON_PROBE_CACHE=0`), stopped drawing a main-view frame — and lazily initialising wgpu-native and baking OmniLight — under `--no-rendering`, made `OmCudaContext` lazy (`OMNISIM_CUDA_EAGER_INIT=1` restores the `CUDA initialized` line at startup; Newton/warp has its own CUDA init), and blocks the Pixar USD stack before `import newton` (`OMNISIM_NEWTON_USD=1`). ⚠️ The Newton preload itself (~3 s in-engine: imports 1.5 s, warp.init 0.3, the ModelBuilder smoke 0.35, `import mujoco_warp` 1.0 — newton's `SolverMuJoCo` imports it even on the CPU path) is inherent and is the critical path of small worlds under `OMNISIM_NO_WINDOW=1`. Refuted the same day, do not re-chase: an event-driven FAST-mode controller wait (engine CPU stayed 92–119% of a core — it is real request-servicing work, not a spin — and the swarm step got slower), and serving body poses from `mj_data` (`OMNISIM_NEWTON_MJ_POSE_CHECK=1` still reads |xpos−body_q| = |v|·dt on the 8-Husky swarm; newton's `_update_newton_state` is 47% of a 0.60 ms step there).
- **`POST /world/load` is LIGHT by default since 2026-09-02** (`light: false` or a `tracking` object opts into the trackers; `OMNISIM_HARNESS_LIGHT=0` restores the old default; the response's `tracking.default_applied` says so). Fleet arena on the current engine: load 5.2 → 4.65 s, `/sim/step 1` 54 → 23 ms — the older "12.1 s vs 4.1 s, 17–47×" figures are the 2026-08-29 engine and are history. `make tests-unit` is the engine-free lane: **71 s**, 1,229 tests (it was 5 min with 7 failures on 2026-09-02 morning). `python scripts/release/bump_version.py X.Y.Z` bumps every version site in one commit.
- **`run-headless --profile` is how you ask where a load went** (stage split, PROTO phases, template-engine split, Newton preload, and every log line stamped `[t+ms]`; `OMNISIM_LOG_TIMESTAMPS=1` alone stamps). **`doctor` now reports CI** (`ci  N/M workflows green on <sha>`; red or in-flight workflows are named with the `gh run view --log-failed` command) — the private repo's CI was red for a day on 2026-09-02 and no local gate showed it; check that row after every push.
- **A Camera/RangeFinder on a big scene no longer re-walks the scene every frame (2026-09-02):** each device keeps a cached draw list like the main view (city + 320×240 camera: 103 → 52 ms/step; `OMNISIM_WGPU_SENSOR_DRAW_CACHE=0` reverts, `OMNISIM_WGPU_REPORT=1` prints per-device collect telemetry). **Load time is self-describing under `OMNISIM_RELOAD_PROFILE=1`** (stages, PROTO phases, template-engine split); on the city the next lever is `QJSEngine::importModule` compiling each filled template (~2.0 s of 6 s after the static-text hoist, `OMNISIM_TEMPLATE_CHUNK_HOIST=0` reverts) — a shared `QJSEngine` was built and measured: no gain on a quiet machine (its apparent 2× was background load), and it needs `wbrandom.js`'s module state reset; what remains is ~1.6 ms per KB of generated JavaScript, a PROTO-side lever.
- **Every `OMNISIM_*` variable is listed in [docs/reference/environment-variables.md](docs/reference/environment-variables.md)** (generated; `python scripts/dev/gen_env_reference.py` regenerates it and a docs test pins it). Read it before inventing a hatch, and write new boolean hatches VALUE-parsed (`=0` means OFF) — `OMNISIM_EAGER_TEXTURE_DECODE`, `OMNISIM_NO_WINDOW`, `OMNISIM_NO_GL` were presence-gated until 2026-09-02.
- **A `.connect_error.txt` entry from an EARLIER run could FAIL a healthy `run-headless` (fixed 2026-09-02).** libController's sidecar is append-only and tagged `run=<engine pid>`; Windows reuses pids, so a healthy 8-Husky swarm run FAILed on three entries an omniquad session had appended 11 hours earlier under the same pid. `OmLog` now deletes the sidecar when it truncates the log, exactly as it does `.newton.json`. If an older binary reports handshake failures for controllers that are not in your world, that is this.

### What is in the tree, and what is not for you

The tree was cut back on 2026-09-02 to what the product, its tests and its documented evidence actually reach (`docs/ARCHIVE.md` lists every removed file with the commit that still holds it: `git show <sha>:<path>`). Read it as four kinds of thing:

| Area | What it is | Agent rule |
|---|---|---|
| `src/`, `include/`, `lib/`, `omnisim/`, `scripts/`, `packages/`, `resources/`, `agents/` | the engine, the controller libraries, the CLI, the harness, the MCP/ROS packages, the PROTO/node catalogue, the co-located agents | engineering — search here |
| `projects/` | worlds, robots, PROTOs, controllers, policies. Every world is catalogued in [`DEMOS.md`](DEMOS.md) / [`WORLDS.md`](WORLDS.md); an uncatalogued world was archived | a world not in the catalogue does not exist |
| `docs/` | reference (`docs/reference`, one page per live node), guides, the four `agents-*.md` reference docs, benchmark evidence (`docs/benchmarks`), the canonical RL status (`rl-current-state.md`) | current by contract; anything historical was moved to `docs/ARCHIVE.md` |
| `tests/` | the unit lane (`make tests-unit`), the engine pins, and the benchmark suites under `tests/benchmarks/` with the evidence the benchmark docs cite | run the lane; benchmark result files are evidence, not fixtures to edit |
| `social/`, `omnilink-docs/oliver_eval/` | the owner's outreach and evaluation workspaces, publish-denied, hidden from ripgrep by `.rgignore` | not engineering; never a source for any claim about the simulator |

Things that are deliberately gone and must not come back: the ODE backend and every stub that dispatched to it (`physicsBackend "ode"` warns once and runs on Newton), the WREN renderer, the mirrored copy of the Newton runtime, per-file version bumps, `.wbt` as a written format, campaign logs and finished plans inside `docs/`, and result dumps nothing cites.

§§1–11 below are the deep reference. The bootstrap above is sufficient for most first-turn tasks; come back to the reference when you need detail (build setup §2, demo catalog §3, harness API §5, controller editing §7, validation §8).

---

## TL;DR — One-paragraph mental model

OmniSim is an open-source robotics simulator built on the [Webots](https://github.com/cyberbotics/webots) engine, with substantial additions: an HTTP harness for agent-driven world authoring, a Camera-based capture / cinema pipeline, the omniworld procedural world generator, the OmniLink agent runtime, an RL training pipeline, CUDA-accelerated granular physics, native URDF import, multi-instance parallel execution, and the OmniSim Wire Protocol for bridges. It is an executable (`omnisim-bin.exe` on Windows, `omnisim` or `omnisim-bin` elsewhere) that loads a world file (`.wbt`), simulates physics/rendering/sensors, and spawns one **controller process** per robot. On Windows the simulator core is `omnisim-bin.exe`, fronted by two thin launchers the build also produces — `omnisim.exe` (console) and `omnisimw.exe` (windowed; this is the shipped entry point the installer's Start Menu / desktop shortcuts point at, see [`scripts/packaging/windows_distro.py`](scripts/packaging/windows_distro.py)). The build also refreshes `webots.exe` / `webotsw.exe` as byte-identical legacy copies of those launchers. There is **no `webots-bin.exe`** — no Makefile produces that name, and nothing should fall back to it. Controllers are scripts under `projects/.../controllers/<name>/<name>.py` (or `.cpp`) that talk to the simulator over an IPC channel via the `omnisim` Python / C / C++ library (⚠️ `omnisim` is the **only** spelling — the legacy `controller` Python module and the `<webots/...>` include path were deleted on 2026-08-16, and the C++ namespace is `omnisim`; see §7). To make a robot do something, you either edit its controller, generate a new world, or — for bridge-style demos like the chat robots — point an HTTP client at a port the running controller exposes.

**Multi-instance, by design.** OmniSim can run as **N parallel `omnisim-bin` processes on the same host**. Each instance auto-allocates its TCP port from the `[1234, 1294]` range and gets a port-isolated tmp / IPC dir, so two simulators don't stomp each other's controller channels. Batch validation, fleet benchmarks, agent-vs-agent matches, and per-PR smoke farms are all "K headless processes against the same or different worlds" workloads — not "one big simulator". See §3e.

---

## 1. Environment check

Before doing anything else, run these read-only checks:

```bash
# Where am I?
pwd

# Is the build present? Look for the simulator binary:
#   Windows: msys64/mingw64/bin/omnisim-bin.exe (the core; webotsw.exe / webots.exe
#            are the windowed + console launchers that exec it. NOT webots-bin.exe --
#            that name is dead, any copy on disk is a stale artefact.)
#   Linux:   bin/omnisim-bin (plus a `webots` launcher shell at the repo root)
#   macOS:   Contents/MacOS/webots (built as `webots`; no `omnisim` alias on macOS yet)
ls msys64/mingw64/bin/omnisim-bin.exe 2>/dev/null \
  || ls bin/omnisim-bin 2>/dev/null \
  || ls Contents/MacOS/omnisim 2>/dev/null || ls Contents/MacOS/webots 2>/dev/null

# Are the CLI verbs usable?
python -m omnisim --help
```

If the binary is missing, build first (§2). `OMNISIM_HOME` (canonical) points at this checkout; `build_omni.bat` and `launch.bat` derive it themselves. **`WEBOTS_HOME` is retired from the engine↔controller RUNTIME contract** (now `OMNISIM_*`-only; a legacy-only export is reported by name) **but NOT from the build** — the top-level [`Makefile`](Makefile) exports it as an alias for the 16 Makefiles that consume it, and `qt_utils` still falls back to it. [full text](docs/developer/agents-reference-sections.md#section-1-environment-check)

### One-time per-clone: enable hooks

```bash
bash scripts/dev/setup_hooks.sh
```

Points `core.hooksPath` at `.githooks/`: post-checkout runs `scripts/dev/clean_orphans.py`, and `pre-push` is local CI — the smoke set (`tests/smoke/smoke_worlds.json`) fails the push on a regression; bypass with `OMNISIM_SKIP_PUSH_CHECK=1 git push`; `make tests-smoke` runs it manually. Ten hosted workflows are active (incl. `linux-build.yml`, `licence-provenance.yml`, `dco.yml`). [full text](docs/developer/agents-reference-sections.md#section-1-environment-check)

---

## 2. Build (only if the binary is missing)

### Windows (preferred — uses MSYS2)

```bat
build_omni.bat
```

Runs `make` with the MinGW64 toolchain on `PATH` and `OMNISIM_HOME` derived from its location; first build 5–15 min. Then vendor the Newton runtime (one-time, ~600 MB, **not optional**):

```bash
make -C src/omnisim bundle-newton-runtime
```

### Linux / macOS / cross-platform

```bash
export OMNISIM_HOME=$(pwd)     # the Makefile exports WEBOTS_HOME as an alias for the sub-makes
python -m omnisim build all
```

### Iterating on the engine (the C++ edit loop)

`make release -j4` from `src/omnisim` (MSYS `make` in a MINGW64 shell on Windows) is the loop: ccache and lld are auto-wired, `WGPU_NATIVE_HOME` is auto-discovered, and since 2026-09-02 the Makefile's ~10 parse-time toolchain probes are cached in `.build_tmp/make-probes.mk` (`build_probes.sh`, keyed on PATH/CXX/OSTYPE and the toolchain binaries' mtimes; `OMNISIM_PROBE_CACHE=0` runs them inline): a no-op `make release` is **1.03 s** (was 1.64) and touch-one-TU + relink **2.29 s** (was 2.69). ⚠️ **Stop every running `omnisim-bin` first** — the link replaces the `.exe` it holds open. ⚠️ **The shipped binary is built WITH CUDA** (it logs `CUDA initialized` on first GranularGroup use). A login shell that drops `nvcc` from `PATH` auto-selects `OMNISIM_WITH_CUDA=OFF` against objects compiled ON and the link dies on `nvrtc*`/`cu*` symbols — pass `OMNISIM_WITH_CUDA=ON` (with the CUDA `bin` on `PATH`) or keep `nvcc` visible. `make linker-info` reports what resolved. [full](docs/developer/agents-reference-sections.md#iterating-on-the-engine-the-c-edit-loop)

Linux (v5.1): `bash scripts/install/linux_bootstrap.sh`, GPU wheels into the **system** `python3` (not a venv), Xvfb for headless — [quickstart → Linux](docs/developer/quickstart.md#linux-quickstart-ubuntu). macOS untested. Subsystems: `python -m omnisim build core|gui|controller-libs` (`build renderer` refuses — wgpu is compiled into the engine). The C++ loop is `make release` in `src/omnisim`, cached by default (ccache; `OMNISIM_LINKER=auto`, pin `bfd`/`lld` for cross-machine binary identity; a build with no wgpu-native is REFUSED — `OMNISIM_RENDERERLESS=ON` by name). [full text](docs/developer/agents-reference-sections.md#section-2-build)

---

## 3. Launch a demo

### 3a. Windowed (visual — for human-in-the-loop debugging or screenshot capture)

No-args opens the **OmniSim demo launcher** (right-click the orb → *Show Robot Window* → *Launch* any demo; index [`DEMOS.md`](DEMOS.md)):

```bat
launch.bat
```

```bash
python -m omnisim run-world projects/samples/demos/worlds/omnilink_launcher.omniworld   # Linux / macOS
```

`launch.bat path\to\world.omniworld` opens a specific world; extra `omnisim-bin.exe` flags follow it.

### 3b. Headless (no window, exits cleanly — preferred for autonomous agent runs)

```bash
python -m omnisim run-headless projects/samples/demos/worlds/showcase/warehouse_husky.omniworld
```

- runs `--batch --mode=fast --no-rendering --minimize --stdout --stderr`, monitoring `omnisim_log.txt`
- **no `--duration`: exits as soon as Newton finalises** (load check; 10 s ceiling); `--duration N` runs N seconds verbatim — only when the run must OBSERVE the sim
- non-zero exit on load failure, or on any warning with `--fail-on-warning`

A bare PASS is a log verdict, not a physics verdict (a hologram floor lets a body reach z = −69 km and still PASS). To certify physics:

```bash
python -m omnisim run-headless <world> --duration 10 --fail-on-runaway
```

It injects the `runaway_watchdog` supervisor into a sibling copy and FAILs, naming the body that left the world. ✅ The cold-first-load trap is RESOLVED (2026-07-05, `eb86f888`; `OMNISIM_FORCE_WARMUP=1` is the safety valve — [write-up](docs/developer/real-grasp-and-the-cold-first-load-trap.md)). [full text](docs/developer/agents-reference-sections.md#section-3-launch-a-demo)

### 3c. Choose a demo

Every URDF robot has a chat demo — `projects/samples/demos/worlds/chat/omnilink_<robot>.omniworld` (×15): right-click → *Show Robot Window*, type `home` / `drive forward 1 m`; offline regex router or OmniLink with `OMNI_KEY` ([guide](docs/guide/omnilink-chat-demos.md), [add your robot](docs/guide/omnilink-add-your-robot.md), [sim-to-real](docs/guide/omnilink-sim-to-real.md)). OmniLink pieces: `python -m omnisim key` ([reference](docs/guide/omnilink-key-and-api.md)); `python -m omnisim byok` (the provider key, hit as `402 BYOK_REQUIRED` — do it for the user; never write a key into this repo). Starters: **Warehouse Husky** `projects/samples/demos/worlds/showcase/warehouse_husky.omniworld` (default); Husky maze `projects/samples/demos/worlds/flagship/husky_maze.omniworld`; swarm `projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld`; Mars `distribution/generated_worlds/mars.wbt`. [full text](docs/developer/agents-reference-sections.md#section-3-launch-a-demo)

## 3d. OmniLink-co-located agents (`agents/production/`)

Agent definitions versioned next to their worlds. Reference: [`agents/production/husky_maze/`](agents/production/husky_maze/) — five maze worlds up to the agent-only `husky_maze_visual.omniworld` / `husky_maze_blind.omniworld`; read its [OVERVIEW.md](agents/production/husky_maze/docs/OVERVIEW.md) first.

```bash
launch.bat projects\samples\demos\worlds\flagship\husky_maze.omniworld
python agents/production/husky_maze/solve.py            # standalone solver
set OMNI_KEY=olink_...
python -m omnisim run-agent --agent husky_maze          # world + agent together
python -m omnisim run-agent --list
```

The registry is discovered from `agents/production/*/omnilink.json` (`discover_agents()` in [`omnisim_run_agent.py`](scripts/dev/omnisim_run_agent.py)). Build new agents on [`agents/production/_lib/`](agents/production/_lib/README.md). [full text](docs/developer/agents-reference-sections.md#section-3d-omnilink-co-located-agents)

## 3e. Running multiple OmniSim instances in parallel

K `omnisim-bin` processes coexist by design — split batch validation, fleet benches and agent-vs-agent across processes. Each picks the next free TCP port in `[1234, 1294]` (`OmTcpServer::start`, `PORT_SCAN_SPAN = 60`; the real ceiling on K is vCPU count) and salts its tmp/IPC dir with it; by default all write the SHARED `<OMNISIM_HOME>/omnisim_log.txt` (`OmLog::initFileLog` in `main.cpp`; `OmLog.cpp` reads `OMNISIM_LOG_PATH` as the override).

### What you must do per child to keep parallel runs clean

- **Set `OMNISIM_LOG_PATH=<unique-per-child path>`** before spawning each `omnisim-bin`, or every child writes `OMNISIM_HOME/omnisim_log.txt` and the last one wins.
- **Don't pin `--port` unless you mean it.** Two children pinned to `1234` fail to bind; leave it default.
- **Give every child a stdout, and know that the engine now KEEPS it (Windows, fixed 2026-08-29).** A non-pipe stdout used to attach the engine to its launcher's console; a second engine could find fd 1 dead → `[Errno 9] Bad file descriptor` in `newton.ModelBuilder()` → FATAL, exit 1 (the "one launch in three" race, public issue #3). The `[main] stdio:` log line says which branch ran.

Canonical pattern: `python tests/benchmarks/optim_bench.py multi-instance --sizes 4 --steps 600` ([`optim_bench.py`](tests/benchmarks/optim_bench.py); [scaling notes](docs/developer/multi-instance-optimization-plan.md)). The harness needs a free `(port, port+1)` pair — `python -m omnisim harness --port 6889 --supervisor-port 6890` or `--auto-port`; the GUI is one window per process. [full text](docs/developer/agents-reference-sections.md#section-3e-multiple-instances-in-parallel)

---

## 4. Driving a robot over HTTP (OmniLink bridges)

Reference bridge: [`omnilink_mobile_bridge.py`](projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py) driving the Husky in [`omnilink_husky.omniworld`](projects/samples/demos/worlds/chat/omnilink_husky.omniworld) (`controllerArgs` `["--robot" "husky" "--port" "8765"]`). Launch the world, then hit `127.0.0.1:8765`:

```
POST /get_robot_state      # current pose, wheel state, fault, last tick
POST /list_robots          # [{id, model, capabilities}]
POST /set_velocity         # {v: <m/s>, w: <rad/s>}
POST /drive_forward        # {distance: <m>}
POST /stop_robot
```

Siblings: arms [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/), quadrupeds `omnilink_quadruped_bridge`, drones `mavic_omnilink_bridge`. Contract: [PROTOCOL.md](PROTOCOL.md). [full text](docs/developer/agents-reference-sections.md#section-4-driving-a-robot-over-http)

---

## 5. Iterating on worlds with the validation harness

An HTTP service wrapping a headless OmniSim subprocess with an injected supervisor — the preferred authoring loop (load → screenshot → inspect → fix → hot-reload) without the GUI.

### Starting it

```bash
python -m omnisim harness --port 6789
```

The module form puts the bundled Qt DLLs on PATH; the underlying script is `scripts/harness/omnisim_harness.py`. Gotchas: **(Windows)** `LAUNCHER_DLL_NOT_FOUND` on the first load = the shell's `PATH` lacks a complete msys2 mingw64 `bin`. **(All)** the harness interpreter needs `Pillow` or `/world/render_stats` is 503. [full text](docs/developer/agents-reference-sections.md#section-5-validation-harness)

### The loop

```bash
# 1. Load a world (cold ~1s for empty.wbt, ~6s for asset-heavy worlds on a
#    fast local disk — on WSL2/virtualized/network disks an asset-heavy cold
#    load has measured 46-79s; slow != hung. See scripts/harness/README.md.)
curl -s -X POST http://127.0.0.1:6789/world/load \
  -H "Content-Type: application/json" \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld"}'

# 2. See what's actually in the scene — confirms positioning before chasing visual bugs
curl -s http://127.0.0.1:6789/scene/tree

# 3. Aim the camera. Computes the axis-angle from camera position + look-at target
#    AND pushes it to the live Viewpoint, so the next screenshot uses it.
curl -s -X POST http://127.0.0.1:6789/scene/look_at \
  -H "Content-Type: application/json" \
  -d '{"position":[-10,-16,10],"target":[0,0,1]}'

# 4. Check exposure as JSON before eyeballing — catches blown-out lighting
#    without an image-eyes round-trip.
curl -s http://127.0.0.1:6789/world/render_stats

# 5. Capture the image
curl -s -X POST http://127.0.0.1:6789/world/screenshot -d '{}' -o shot.png

# 6. Edit the .wbt and POST /world/sync. Pose-only root DEF edits land live;
#    every other edit automatically hot-reloads through the engine parser.
curl -s -X POST http://127.0.0.1:6789/world/sync \
  -H "Content-Type: application/json" \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld"}'
```

### Endpoint cheatsheet

Route + one line; the full row per endpoint is verbatim in [docs/developer/harness-endpoint-reference.md](docs/developer/harness-endpoint-reference.md), the wire contract is [PROTOCOL.md §7](PROTOCOL.md#7-world-harness), and `GET /capabilities` is the live set.

| Endpoint | Purpose |
|---|---|
| `GET /capabilities?probe_step=1` | ⭐ Start here: physics verdict from the sidecar `limits.step_cost`, event types, endpoints, `not_supported`, diagnostic codes. [§7.28](PROTOCOL.md#728-get-capabilities) |
| `POST /world/load {path, light?, tracking?}` | Load; structured `diagnostics[].code` (an OPEN enum). ⭐ `light: true` is the step-cost lever (6–35 vs 573–606 ms/step). [§7.3](PROTOCOL.md#73-post-worldload) · [light](docs/developer/harness-endpoint-reference.md#post-worldload-light) |
| `POST /world/sync {path?}` | ⭐ After any edit; `mode` ∈ `live_pose` / `full_reload` / `no_change` / `rejected` (422) / `busy` (409). [§7.3a](PROTOCOL.md#73a-post-worldsync) |
| `GET /world/diagnostics` · `POST /world/screenshot` · `GET /world/render_stats` | Diagnostics; PNG; exposure stats + `warnings[]`. [§7.4](PROTOCOL.md#74-get-worlddiagnostics)–[§7.6](PROTOCOL.md#76-get-worldrender_stats) |
| `GET /scene/tree` (`?bounds=1`) · `GET /scene/node/<def>` | Node list (+ world-space bounds); field dump incl. `boundingObject` / `physics` presence. [§7.7](PROTOCOL.md#77-get-scenetree), [§7.8](PROTOCOL.md#78-get-scenenodedef) |
| `POST /scene/look_at` · `GET /scene/viewpoint` · `POST /scene/frame` · `POST /scene/orbit` · `GET /scene/visible` | Aim; read the camera; ⭐ aim AND distance with `verification`; nudge; frustum test. [§7.9](PROTOCOL.md#79-post-scenelook_at), [§7.20](PROTOCOL.md#720-get-sceneviewpoint)–[§7.23](PROTOCOL.md#723-get-scenevisible) |
| `POST /scene/spawn` · `POST /scene/delete` · `POST /scene/set_pose` | ⛔ Spawn/delete do NOT reach the solver unless `{"physics": "rebuild"}`; clone a `URDFRobot` with a unique `name`. [§7.29](PROTOCOL.md#729-post-scenespawn)–[§7.31](PROTOCOL.md#731-post-sceneset_pose) |
| `POST /sim/rebuild_physics` | ⭐ Re-register the scene at current poses (97–267 ms; refused on Cloth/SoftBody/GranularBed; welds dropped). [§7.36](PROTOCOL.md#736-post-simrebuild_physics) |
| `POST /sim/step {steps?}` · `POST /sim/reset` | Advance N steps; rewind to t=0 AND restore the authored scene. [§7.10](PROTOCOL.md#710-post-simstep), [§7.11](PROTOCOL.md#711-post-simreset) |
| `POST /sim/snapshot {name}` · `POST /sim/restore {name}` · `GET /sim/snapshots` | Named state; restore keeps the clock; unknown names → `404 SNAPSHOT_NOT_FOUND`. [§7.32](PROTOCOL.md#732-post-simsnapshot-post-simrestore-get-simsnapshots) |
| `GET /sim/state` · `GET /sim/contacts` · `GET /sim/grips` | Session + clock; contact set (works in light mode; `?wake=1` is a no-op); grips. [§7.12](PROTOCOL.md#712-get-simstate), [§7.17](PROTOCOL.md#717-get-simcontacts), [§7.18](PROTOCOL.md#718-get-simgrips) |
| `GET /sim/events` | Unified event stream, two cursors. [§7.19](PROTOCOL.md#719-get-simevents) |
| `GET /robots` · `GET /robot/<def>/joints` · `GET /robot/<def>/devices` | Robots; per-joint snapshot with `hit_limit`; devices. [§7.13](PROTOCOL.md#713-get-robots)–[§7.15](PROTOCOL.md#715-get-robotdefdevices) |
| `POST /robot/<def>/joints/set` · `POST /robot/<def>/ik` | ⭐ Measured joint targets (a bridge in hold mode WINS); ⭐ batched IK preview. [§7.33](PROTOCOL.md#733-post-robotdefjointsset), [§7.34](PROTOCOL.md#734-post-robotdefik) |
| `GET /robot/<def>/sensor/<name>` | 501 by design. [§7.16](PROTOCOL.md#716-get-robotdefsensorname) |
| `GET /robot/damage` · `/robot/damage/events` · `POST /robot/damage/reset` · `/robot/damage/inject` | Damage state, events, heal, fault injection. [§7.24](PROTOCOL.md#724-get-robotdamage)–[§7.27](PROTOCOL.md#727-post-robotdamageinject) |
| `GET /healthz` | Liveness. [§7.2](PROTOCOL.md#72-get-healthz) |

`/sim/events` is the most useful stream for debugging a running scene:

```
sup_cursor = 0; log_cursor = 0
while running:
    batch = GET /sim/events?since={sup_cursor}&log_since={log_cursor}
    sup_cursor = batch["next_since"]
    log_cursor = batch["next_log_since"]
    for evt in batch["events"]:
        handle(evt)  # branch on evt["type"]
```

Filter with `types=contact.began,joint.limit_hit`; `dropped_sup` / `dropped_log` non-zero means poll faster. Reference: [scripts/harness/README.md](scripts/harness/README.md).

### Sister service: capture (port 6791) for cinematic output

Same shape as the harness on `127.0.0.1:6791` (supervisor `:6792`), with a `Camera` sized to the requested resolution; `/capture/sequence` walks a camera path to lossless PNGs and ffmpeg. Shot lists: [`scripts/capture/render.py`](scripts/capture/render.py).

```bash
python -m omnisim capture &
curl -s -X POST http://127.0.0.1:6791/world/load \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld","width":3840,"height":2160}'
curl -s -X POST http://127.0.0.1:6791/capture/camera -d '{"position":[-12,-12,6],"target":[0,0,1]}'
curl -s -X POST http://127.0.0.1:6791/capture/screenshot -d '{}' -o still.png
python scripts/capture/render.py scripts/capture/shotlists/orbit_warehouse.json --ad-hoc
```

**OmniLight** GI is path-traced but BAKED, not per-frame ([omnilight.md](docs/developer/omnilight.md)). Reference: [scripts/capture/README.md](scripts/capture/README.md). [full text](docs/developer/agents-reference-sections.md#section-5-validation-harness)

---

## 6. Generating new worlds (omniworld)

```bash
python scripts/dev/omniworld.py list-recipes
python scripts/dev/omniworld.py describe outdoor_forest
python scripts/dev/omniworld.py generate outdoor_forest --seed 42 --out my_forest.omniworld
python scripts/dev/omniworld.py validate my_forest.omniworld
launch.bat my_forest.omniworld  # or python -m omnisim run-world my_forest.omniworld
```

Recipes `flat_ground`, `outdoor_forest`, `outdoor_desert`, `warehouse`, `urban_block`, `indoor_apartment`, `mars`; same `(recipe, seed, params)` → byte-identical output; `--param key=value`. Guides: [user guide](docs/developer/omniworld-user-guide.md), [biome cookbook](docs/developer/omniworld-biome-cookbook.md). [full text](docs/developer/agents-reference-sections.md#section-6-generating-new-worlds)

---

## 7. Editing a controller

Controllers live in `projects/<area>/controllers/<name>/<name>.{py,cpp,c}`; Python ones need no rebuild.

```python
from omnisim import Robot     # the only import path (the legacy `controller` alias was removed)

robot = Robot()
time_step = int(robot.getBasicTimeStep())

motor = robot.getDevice("left_wheel_joint_motor")
motor.setPosition(float("inf"))
motor.setVelocity(2.0)

while robot.step(time_step) != -1:
    # Per-tick logic. Read sensors, write commands.
    pass
```

- ⚠️ **`omnisim` is the ONLY Python module** ([`lib/controller/python/omnisim/`](lib/controller/python/omnisim/)); the `controller` alias was DELETED 2026-08-16 (source break, one-line port).
- ⚠️ **`#include <omnisim/robot.h>` / `<omnisim/Robot.hpp>` only** — the `<webots/...>` forwarders were DELETED 2026-08-16.
- ⚠️ **The C++ namespace is `omnisim` and that IS an ABI break** — recompile every C++ controller (no alias). The C API (`wb_*`, `Wb*`, `wbu_*`) did NOT move. A standalone `make` reads `OMNISIM_HOME`, falling back to `WEBOTS_HOME` at build time only. [full text](docs/developer/agents-reference-sections.md#section-7-editing-a-controller)

---

## 8. Validating a change

Cheap → expensive:

```bash
# Does it still load? THE default check -- stops the moment Newton finalises
# and the .newton.json sidecar exists, then stops 0.5 s after the first physics step
# (15.52 s -> 6.37 s -> 5.5 s on 2026-09-02, same PASS, same sidecar).
python -m omnisim run-headless path/to/world.omniworld --until-finalized

# Explicit observation window -- only when the run must WATCH the sim.
python -m omnisim run-headless path/to/world.omniworld --duration 15

# ALSO assert no body left the world (a bare PASS and --until-finalized cannot see it).
python -m omnisim run-headless path/to/world.omniworld --duration 10 --fail-on-runaway

# Engine-free pytest lane (-m "not engine")
make tests-unit

# One world through the test suite / fast smoke suite / one group / perf log
python -m omnisim test-world path/to/world.omniworld --nomake
python -m omnisim test-smoke
python -m omnisim test-group api        # api, parser, physics, rendering, cache, protos, other_api
python -m omnisim profile-world path/to/world.omniworld
```

`--fail-on-warning` for a strict log; `--fail-on-runaway` when the claim is about physics (§3b). While authoring, use the harness (§5): `POST /world/load` once, `POST /world/sync` after edits. [full text](docs/developer/agents-reference-sections.md#section-8-validating-a-change)

---

## 9. Where to look when something goes wrong

- **`omnisim_log.txt`** in the repo root — every warning, error and structured runtime message. Read it first.
- **Build problems** — [docs/developer/quickstart.md](docs/developer/quickstart.md) §1–6, [build-and-iteration.md](docs/developer/build-and-iteration.md).
- **URDF import problems** — [urdf-import-debugging.md](docs/developer/urdf-import-debugging.md); `scripts/dev/urdf_import.py --report --strict` for a preflight report.
- **Performance problems** — [profiling-playbook.md](docs/developer/profiling-playbook.md), `OMNISIM_RENDERER_TIMINGS=1`.
- **Subsystem ownership map** — [agent-map.md](docs/developer/agent-map.md).

---

## 10. Conventions to honour

- **Do not edit `src/glm/` or `src/stb/`** (vendored submodules).
- **Do not skip git hooks** (`--no-verify`, `--no-gpg-sign`) unless explicitly told to.
- **Do not commit unless asked.** Prefer specific paths over `git add -A`.
- **Do not invent new CLI flags or scripts.** Use `python -m omnisim` and `scripts/dev/`; propose before working around.
- **Before any email outreach batch, read and obey [`social/launch/EMAIL_OUTREACH_RULES_2026-08-28.md`](social/launch/EMAIL_OUTREACH_RULES_2026-08-28.md)**.
- **Smoke / benchmark worlds are local-asset-only** (no `http(s)://` PROTOs; `omniworld validate` and `asset_locality` enforce it).
- **`OMNISIM_HOME`** is the canonical install-root variable; `WEBOTS_HOME` survives at build time (alias from the top-level [`Makefile`](Makefile)) and in the `qt_utils` fallback ([`StandardPaths.cpp`](resources/projects/libraries/qt_utils/core/StandardPaths.cpp)). [full text](docs/developer/agents-reference-sections.md#section-10-conventions-to-honour)

---

## 11. Further reading

Full annotations: [agents-reference-sections.md → Section 11](docs/developer/agents-reference-sections.md#section-11-further-reading).

- [README.md](README.md) · [ARCHITECTURE.md](ARCHITECTURE.md) · [WORLDS.md](WORLDS.md) · [SUPPORT.md](SUPPORT.md) · [SECURITY.md](SECURITY.md) · [CONTRIBUTING.md](CONTRIBUTING.md)
- [PROTOCOL.md](PROTOCOL.md) — the OmniSim Wire Protocol; the contract for any tool that drives OmniSim
- [docs/developer/README.md](docs/developer/README.md) · [quickstart.md](docs/developer/quickstart.md) · [agent-map.md](docs/developer/agent-map.md) · [simulation-authoring-for-coding-agents.md](docs/developer/simulation-authoring-for-coding-agents.md)
- [rl-current-state.md](docs/developer/rl-current-state.md) — **CANONICAL RL status; append-only, only the top banner is current, and it wins over every other doc.**
- [shadowing.md](docs/developer/shadowing.md) + [ghost-design-rules.md](docs/developer/ghost-design-rules.md) — the Shadowing method; run `ghost_validator.py` BEFORE training
- [skill-library.md](docs/developer/skill-library.md) + [projects/policies/skills/README.md](projects/policies/skills/README.md) — the SKILL LIBRARY
- [omniquad-residual-rl.md](docs/developer/omniquad-residual-rl.md) · [g1-single-source-of-truth.md](docs/developer/g1-single-source-of-truth.md)
- [g1-ghost-fidelity-journey.md](docs/developer/g1-ghost-fidelity-journey.md) — read before claiming a G1 walk · [train-deploy-gap.md](docs/developer/train-deploy-gap.md) · [closed-loop-chaos-diagnostic.md](docs/developer/closed-loop-chaos-diagnostic.md)
- [scripts/harness/README.md](scripts/harness/README.md) · [omniworld-user-guide.md](docs/developer/omniworld-user-guide.md)

If a doc and the code disagree, the code wins — and update the doc in the same change.
