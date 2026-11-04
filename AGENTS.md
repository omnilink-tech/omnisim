# AGENTS.md — Running OmniSim as an AI Coding Agent

**This file is for AI coding agents** (Claude Code, Codex, Cursor, custom agent harnesses) working inside a fresh clone of OmniSim. It tells you exactly what to do to get a demo running. The conventions below match the [`AGENTS.md` open standard](https://agents.md/), and tools that respect that standard will load this file automatically.

If you are a human, this file also works as a quick "what does OmniSim do and how do I run a demo" cheat sheet — the [README](README.md) and the [Developer Quickstart](docs/developer/quickstart.md) have the long form.

---

> **OmniLink access policy, 2026-09-22:** The owner requires an OmniKey for every OmniLink AI experience, including Free. Do not restore keyless chat, automatic local-model selection, or basic-command fallback after connection errors. Ordinary OmniSim typed controls and Stop remain available. Use [the current setup guide](docs/guide/omnilink-chat-demos.md); it supersedes older keyless instructions in this file. Customer-facing pages must not reveal architecture. **Cost comparisons are ALLOWED again (owner, 2026-09-27: cost is a competitive advantage — mention it):** quote a measured benchmark with its conditions (workload, model, what was counted) and link the evidence, but never say or imply the AI layer itself costs nothing — an OmniKey and the user's own model-provider key are still required.

## 0. For agents: read this first (60 seconds)

**Call it OmniSim, and know what it is for.** The simulator is OmniSim — its own product, with substantial additions over the upstream Webots engine it forked from (URDF importer, agent-facing HTTP harness, capture / cinema pipeline, OmniLink agent runtime, omniworld procedural world generator, RL pipeline, CUDA granular-physics, multi-instance parallel runs, OmniSim Wire Protocol). When you talk to the user about the running simulator, the binary, the env vars, the URL scheme — say *OmniSim*. Use "Webots" only when explicitly referring to upstream (the GitHub repo, the file-format syntax inherited from VRML, the original PROTO conventions).

**OmniSim is an open-source robotics workshop for agents.** It is a simulator — say so, it is how people search for it — but it is *more than* one: it is the place where an agent has everything it needs to work on any robotic system. Simulate a complete robot with high-fidelity physics; build the digital twin of a robot the user already owns; connect that twin to the real machine across one control surface; and program, test and debug the whole system for them — all of it drivable by the user simply talking to you. The person the positioning leads with is the one who does **not** have the hardware: someone learning robotics, someone building a robot, someone with an idea and no lab. The goal is to make robotics accessible to anyone with an idea. Describe it that way to the user. ⭐ **The canonical wording — and the honesty gate on what may and may not be claimed — is [docs/developer/positioning.md](docs/developer/positioning.md).** Read it before writing any tagline, blurb, README line or pitch paragraph; it is cited from a capability audit and nothing in it may be widened without a new measurement. In particular: OmniSim gained a **held pause and break-on-event in v9** (§5, PROTOCOL.md §7.38–§7.40) — quote the TWO latency regimes with it, never a single figure — but it still has **no watch conditions, no record/replay/run-diff, and `POST /sim/snapshot` is not a checkpoint** (poses + joint angles only; velocity is never captured) — never imply otherwise; the **sim↔real claim covers the control surface ONLY** (the shipped bridges run against mock drivers, and no policy trained here has been validated on hardware); a **"digital twin" here is a model built from the robot's own URDF/CAD**, not live-synced telemetry; and **Windows has the only prebuilt package** — say so next to the accessibility promise. ⚠️ **"Where robot software gets debugged" is RETIRED** (2026-09-17, owner decision) — it is struck from every surface and from TRADEMARKS.md; never reuse it as a tagline. Debugging is now one pillar of the workshop, and the OmniLink chat demos (§3c) are no longer demoted: talking to it is the interface the positioning leads with.

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
| **You are in an MCP client** (Claude Code, Cursor, Claude Desktop) and can see `omnisim` tools | **46 tools** (was 37). `harness_status` first; unreachable → `python -m omnisim harness` (it reports on the HARNESS, not the engine). Then `load_world {"path": ..., "light": true}` → `get_scene_tree` → `frame` → `screenshot`. ⭐ New in v9: `sim_pause` / `sim_resume` / `sim_break` / `sim_breaks` / `sim_break_clear` are the debugger, and `robot_prompt` / `robot_tool` / `robot_state` / `robot_events` reach a robot's OmniLink bridge (gated `/prompt` + `/tool`, never the ungated REST verbs) so you no longer have to curl it. ⚠️ `robot_events` is the BRIDGE's ring, not the harness's `get_events` — two rings, two cursors — and it dies with the bridge process. [full](docs/developer/agents-first-moves.md#mcp-client) |
| **Run / see a demo** | `python -m omnisim demo` / `demos`; one world: `python -m omnisim run-world <w>.omniworld` or `run-headless <w>.omniworld` (omit `--duration` unless the run must WATCH). §3. [full](docs/developer/agents-first-moves.md#run-a-demo) |
| **Make a legged robot do a motion** (walk, gait styling, expressive motion — any legged robot) | ⭐ **SHADOWING**: [`projects/policies/training/README.md`](projects/policies/training/README.md), [ghost-design-rules.md](docs/developer/ghost-design-rules.md), [rl-current-state.md](docs/developer/rl-current-state.md). ⚠️ The G1 walk is on a WEIGHT-BEARING balance harness — never "free-standing". [full](docs/developer/agents-first-moves.md#legged-motion) |
| **Make a *reusable skill*, or compose skills into a demo** (walk, turn, carry, stand, climb → a BATON sequence) | ⭐ **SKILL LIBRARY**: `python -m omnisim policy list` / `sequence <name>` / `preview` / `train` / `verify-demos` (`policy --help` is the live list). [skill-library.md](docs/developer/skill-library.md). [full](docs/developer/agents-first-moves.md#skill-library) |
| **Author or edit a world (`.omniworld`)** | ⚠️ `.omniworld`, never `.wbt`. `python -m omnisim harness` → **`POST /world/load {"path": ..., "light": true}`** → edit → **`POST /world/sync`** (325 ms pose-only, 2818 ms structural) → `POST /world/screenshot`. §5. [full](docs/developer/agents-first-moves.md#author-a-world) |
| **Make a gripper HOLD something** (pick-and-place, a pinch grasp, carrying a part) | ⭐ Read [docs/guide/friction-grasp.md](docs/guide/friction-grasp.md) FIRST: `newtonGroundMu` (`coulombFriction` is NOT read, bar one bridged case the guide names), `newtonContactKe`, `newtonImpratio`, `basicTimeStep 8`; `setForce` does NOT enter force mode (`OMNISIM_NEWTON_TORQUE_MODE=1` does). Working example: `python -m omnisim run-headless projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld --duration 45`. [full](docs/developer/agents-first-moves.md#gripper-grasp) |
| **Convert a STEP/STP CAD file into a robot** | `python scripts/dev/step_to_urdf.py <in.step> <robot_dir> --name <n> --up z --ground bottom`; verify the up-axis. [step-to-urdf.md](docs/developer/step-to-urdf.md). [full](docs/developer/agents-first-moves.md#step-to-urdf) |
| **Drive a robot with LANGUAGE** (an operator sentence, not a tool call) | ⭐ `POST /prompt` on the robot's bridge (`:8765`, the Mavic on `:6090`). Parser first (free), model only on what it declines, then the **gate** vets. `POST /tool` for a typed call. ⚠️ The direct REST verbs (`/drive_forward`, `/turn`, `/set_velocity`) are UNGATED — [GATE_COVERAGE.md](packages/omnisim-bridges/GATE_COVERAGE.md). §4. |
| **Debug a misbehaving controller** | ⭐ **Arm a break, do not poll.** `POST /world/load {"light": false}` (light silences 5 of the 11 types and a break on one is REFUSED) → `POST /sim/pause` → `POST /sim/break {"types": [...], "filter": {...}}` → `POST /sim/step {"steps": 400}` runs to the breakpoint and stops early → read a STILL scene → single-step. Then `GET /sim/events?since=...&log_since=...`; branch on `break.hit`, `controller.log`, `joint.limit_hit`, `contact.began`, `damage.*`. ⚠️ Detection is never sub-step and has TWO latency regimes. [full](docs/developer/agents-first-moves.md#debug-a-controller) |
| **Inspect a running scene** | `GET /robots`, `/robot/<def>/joints`, `/robot/<def>/devices`, `/sim/contacts`, `/sim/grips`. [full](docs/developer/agents-first-moves.md#inspect-a-scene) |
| **Aim a camera at something** (screenshot, framing, "why is my shot empty?") | `POST /scene/frame {"def": "<DEF>"}` (aim + distance + proof), `GET /scene/visible`, `GET /scene/tree?bounds=1` — never iterate on screenshots. [full](docs/developer/agents-first-moves.md#aim-a-camera) |
| **Generate a synthetic dataset** (RGB + depth + instance segmentation, domain-randomized) | ⭐ `python scripts/dev/synth_data.py --world <w> --out <dir> --samples N --seed S`; main-view dump (`OMNISIM_WGPU_SYNTH_DUMP`), desktop session, OFFLINE only. [full](docs/developer/agents-first-moves.md#synthetic-dataset) |
| **Cinematic capture** | ⭐ [Cinematic replay](scripts/cinema/CINEMATIC_REPLAY.md): record OmniSim poses → authored Blender geometry → Cycles proxy → review → final. Native image capture is explicit (`python -m omnisim capture`, port `6791`) for debugging/live-renderer evidence. [full](docs/developer/agents-first-moves.md#cinematic-capture) |
| **OmniLink demo video** | ⭐ Owner-approved [minimal native style](scripts/cinema/OMNILINK_DEMOS.md): one robot, one arena, fixed camera, silence, prompt → action. Native OmniSim footage throughout; no Blender. Save real prompt/state evidence separately from the clean screen. |
| **Cinematic *video*** (advertisements, demos, hero footage) | ⭐ `python -m omnisim cinema new` defaults to Blender replay. Follow [CINEMATIC_REPLAY.md](scripts/cinema/CINEMATIC_REPLAY.md): meaningful scenario → measured motion → proxy review → final render → edit. Never silently substitute native footage. [full](docs/developer/agents-first-moves.md#cinematic-video) |
| **Agent Build Film** (an AI agent tells the evidence-led story of what it built in OmniSim) | ⭐ [`AGENT_BUILD_FILMS.md`](scripts/cinema/AGENT_BUILD_FILMS.md): `python -m omnisim cinema agent-build-new --title "<title>"` → `agent-build-validate` → `agent-build-voice` → `agent-build-render` → `agent-build-verify`. [full](docs/developer/agents-first-moves.md#agent-build-film) |
| **Make a part on a machine twin** (CNC milling, laser cutting/engraving, 3D printing, a product from design to assembly) | ⭐ OmniSim Manufacturing: `python -m omnisim manufacturing setup` / `doctor` FIRST (the stages' Python packages, pinned, CPython 3.12; a missing one stops a stage with its name, exit 3), then `python -m omnisim manufacturing` (machines, worlds, then `<process> <tool>`), guide [projects/manufacturing/README.md](projects/manufacturing/README.md). Products (clothes rack, desk robot, SparkRover Mini): `python -m omnisim manufacturing products`, then `product <name> <stage>`; the worked story is [manufacturing-concept-to-robot.md](docs/guide/manufacturing-concept-to-robot.md), the recipe [manufacturing-make-your-own-product.md](docs/guide/manufacturing-make-your-own-product.md). Each product's `release/` is regenerated by its `release.py` (no engine). Machines in `machines/<name>/` (config + CAD pack under the vendor's licence), shared drives/CAD/world kit in `core/`. ⚠️ Simulation only: no twin is validated against its real machine, so a number is the twin's, with its conditions. ⚠️ A print is **twin** (engine) or **ideal** (`printing/ideal_print.py`, no engine, no axis dynamics): name the mode with every print number. |
| **Damage testing** | `python -m omnisim damage`; `python -m omnisim damage-regression`. [full](docs/developer/agents-first-moves.md#damage-testing) |
| **Test that MANY worlds load** (a sweep, a corpus, CI smoke) | ⭐ `python -m omnisim validate-worlds <worlds...>` ([`batch_validate.py`](scripts/dev/batch_validate.py)): one engine hot-reloads each world (5.0× faster); LOAD check only. [full](docs/developer/agents-first-moves.md#many-worlds-load) |
| **Test that a world loads** | ⭐ `python -m omnisim run-headless <world> --until-finalized --fail-on-warning` (15.52 s → 6.37 s → **5.5 s** as of 2026-09-02; fleet arena 8.0 → 4.8 s: the run now stops `--settle-after-step` 0.5 s after the first physics step instead of sitting out a 2 s grace). PASS ≠ sane physics: `--duration` only to OBSERVE, `--fail-on-runaway` to certify (§3b). [full](docs/developer/agents-first-moves.md#one-world-loads) |
| **Run K worlds in parallel** (batch validation, fleet bench, agent-vs-agent) | N headless `omnisim-bin` processes; ports auto-picked in `[1234, 1294]`; `OMNISIM_LOG_PATH=<unique>` per child. §3e. [full](docs/developer/agents-first-moves.md#parallel-worlds) |
| **Ask what OmniSim can actually SIMULATE** ("can it do cloth / heightfields / a force sensor / N robots?") | ⭐ Read the generated lane-4 matrix [lane4-capability-matrix.md](docs/benchmarks/lane4-capability-matrix.md) (51 probes, 95.7% work, 2026-09-01) — it beats this file; top up with [`merge_coverage.py`](tests/benchmarks/omnibench/lane4/merge_coverage.py), never a text editor. [full](docs/developer/agents-first-moves.md#capability-matrix) |
| **Benchmark physics, throughput, or "how does OmniSim compare?"** | ⭐ OmniBench ([`tests/benchmarks/omnibench/`](tests/benchmarks/omnibench/), [`SPEC.md`](tests/benchmarks/omnibench/SPEC.md), `run_all.py`). ODE rows are history; determinism is bitwise on CPU `mujoco`, refuted on `mujoco_warp`; Newton's case is batching, not per-step cost; ⛔ never benchmark Isaac (NVIDIA SLA §8.9). [full](docs/developer/agents-first-moves.md#omnibench) |
| **Ask whether an AGENT can actually get a job done here** (sim-vs-sim, tool-surface ablations, "is our agent surface worth anything?") | AgentBench ([`tests/benchmarks/agentbench/`](tests/benchmarks/agentbench/), [`run_agentbench.py`](tests/benchmarks/agentbench/run_agentbench.py) `--fake-sim`). ⚠️ The first A/B (bare shell tied 10/10 with fewer calls, n=1) is neither a win nor a refutation. [full](docs/developer/agents-first-moves.md#agentbench) |
| **Anything about VRML / `.wbt` primitives** (`Robot`, `Supervisor`, `.wbt` syntax, the controller library) | Webots syntax + `URDFRobot`, `omnisim://`. ⚠️ Gone with ODE: `Fluid`/`ImmersionProperties` (hard ERROR), ODE tuning fields (parsed, not read); `Radio`/`Microphone` retired. `TouchSensor` WORKS. Check [docs/reference/](docs/reference/). [full](docs/developer/agents-first-moves.md#vrml-primitives) |
| **How to support / sponsor the project** | [github.com/sponsors/omnilink-tech](https://github.com/sponsors/omnilink-tech), [SPONSORS.md](SPONSORS.md). [full](docs/developer/agents-first-moves.md#sponsoring) |

Other CLI verbs (`python -m omnisim --help` is the live list): `manufacturing`, `validate-worlds`, `verify-install`, `proto`, `agent`, `validate-urdf`, `build`, `compile-commands`, `benchmarks`, `key`, `byok`.

### Hard-won rules (don't relearn these)

FINAL state only, one line each; every measurement, date and self-correction is verbatim in [docs/developer/agents-hard-won-rules.md](docs/developer/agents-hard-won-rules.md), one section per rule.

- ⚠️ **`.omniworld` is the extension; `.wbt` is read forever, never written.** Engine: [`OmWorldFileFormat.hpp`](src/omnisim/core/OmWorldFileFormat.hpp) is the single funnel; Python: accept both. `distribution/`, `tests/benchmarks/` and two proof worlds stay `.wbt` on purpose. [full](docs/developer/agents-hard-won-rules.md#world-extension)
- **You drive the harness** — `POST /world/load` once, `POST /world/sync` after edits, `/sim/reset`, `/sim/step`; never kill the `omnisim-bin` the user is watching. [full](docs/developer/agents-hard-won-rules.md#drive-the-harness)
- ⚠️ **Load the harness with `"light": true`** — 2026-08-29, `husky_fleet_arena`: `/sim/step 1` **6–35 ms light vs 573–606 ms full**. Quiet in light: `/sim/grips`, `contact.*`/`grip.*`/`joint.limit_hit` events; **`/sim/contacts` still answers**. [full](docs/developer/agents-hard-won-rules.md#light-mode)
- ⚠️ **A BREAK has two latency regimes and neither is sub-step; the two clocks are DIFFERENT RULERS.** Held plus `/sim/step`: 0 supervisor-ms, `hold_latency_steps` 0, at most one basic step of engine time. Free-running: one SUPERVISOR TICK — `engine_tick_ms` measured 8 / 16 / 48 / 128 ms, ~600 ms on a loaded full-tracking world — and on a 3-body fixture a whole 1 s drop fitted inside one tick, so `contact.began` never fired (`fired: false`). PAUSE, THEN STEP. `sim_time_ms` is the supervisor's per-iteration counter (it stamps every other harness response); `engine_time_ms` is the engine's clock (~700 engine-ms inside one supervisor tick on a 3-body world) and does NOT rewind on `/sim/reset`; `paused` is `null`, not `false`, when nobody could be asked. ⚠️ `/sim/step` is exact on the SUPERVISOR clock only — 0–4 basic steps of engine overshoot — so read `engine_advanced_ms`, never assert it. [full](docs/developer/agents-hard-won-rules.md#break-latency)
- ⚠️ **"The pause exists" and "the pause worked" are different claims.** `0aafab096` (2026-09-15) shipped `PauseLease` with no test and SEVEN defects (frozen clock over a moving engine; a lease expiry that hung the supervisor forever; a lease and a `break_hit` that survived a `/world/load`; detection lagging 17 basic steps). All fixed 2026-09-22, covered by 43 engine-free tests + 4 live cases. Do not quote pause behaviour from a build between those dates. [full](docs/developer/agents-hard-won-rules.md#pause-history)
- ⚠️ **When an agent drives a robot badly, suspect the TOOL before the prompt** — measure the primitive (`turn` delivered 56.7% of its command; the control law was the bug). Return `{commanded, achieved, error, settled}`, never an echo. [tool-design-for-agents.md](docs/developer/tool-design-for-agents.md). [full](docs/developer/agents-hard-won-rules.md#suspect-the-tool)
- ⛔ **Never kill an `omnisim-bin` you did not spawn** — engines coexist; `taskkill /F` reads like a crash in the victim's log. Check its command line and wait, or use your own ports. When you kill your OWN, reap its controllers too — an orphaned bridge keeps its port and answers for the next world (2026-09-07). [full](docs/developer/agents-hard-won-rules.md#never-kill-engines)
- **Kinematic-only props** (conveyors, tables, markers): NO `boundingObject` — hulls snag and lock joints like an IK bug. [full](docs/developer/agents-hard-won-rules.md#kinematic-props)
- ⚠️ **A towed trolley is a PHYSICAL body by default (2026-10-20).** `omnilink_mobile_bridge --pallets` now velocity-servos the trolley (free body, acceleration-limited) instead of teleporting it and pinning its cargo; parts ride on contact and friction. Give the cart a low `newtonFriction` (it rolls on casters) and its cargo a higher one, or the floor drag ratchets the cargo off. `--tow kinematic` restores the old path; the warehouse flagship pins it until its park-row collection is ported. Kinematic bases (OmniTug) now ramp linear speed (`kin_accel_mps2`).
- **Robots with a base + arm** MUST tuck arm + torso on bridge spawn, or wheel slip mimics a friction bug. [full](docs/developer/agents-hard-won-rules.md#base-plus-arm)
- **`/robot/<def>/sensor/<name>` returns 501 by design** — use `/joints` or the owning controller. [full](docs/developer/agents-hard-won-rules.md#sensor-501)
- **`/sim/state` is session metadata plus BOTH clocks**, not scene state — and since v9 also `paused`, `lease_remaining_ms`, `break_hit`, `breaks_armed`. [full](docs/developer/agents-hard-won-rules.md#sim-state)
- **An empty `/sim/contacts`:** no body sleep on Newton, `?wake=1` is a no-op, native readback ON since 2026-08-07 (`OMNISIM_NEWTON_NATIVE_CONTACTS=0` blinds it — don't). [full](docs/developer/agents-hard-won-rules.md#empty-contacts)
- ⚠️ **Every triangle-mesh collider is silently convexified** (a bowl is a lump). Concave = several convex pieces in a `Group`; every piece collides by default since 2026-10-09 (before, only the first did unless `WorldInfo.newtonCompoundColliders TRUE`). [full](docs/developer/agents-hard-won-rules.md#mesh-convexified)
- ⛔ **A `Cone` boundingObject kills the whole world** (`KeyError(9)`); author an `IndexedFaceSet`. [full](docs/developer/agents-hard-won-rules.md#cone-collider)
- **Read the FIELDS, not the pixels:** `GET /scene/node/<def>` reports `boundingObject` and `physics`; a Solid collides only through its `boundingObject`. [full](docs/developer/agents-hard-won-rules.md#read-the-fields)
- **Watch `dropped_sup` / `dropped_log` on `/sim/events`** — non-zero means poll faster or raise `limit`. [full](docs/developer/agents-hard-won-rules.md#dropped-events)
- **Statics are solid under Newton (since 2026-08-07) and contacts see them.** `WorldInfo.newtonStatics TRUE` wins; `OMNISIM_NEWTON_STATICS=0` is the value-parsed revert; the implicit z=0 plane only substitutes for a dropped `Plane` (`OMNISIM_NEWTON_GROUND_PLANE=1` restores it). [full](docs/developer/agents-hard-won-rules.md#static-floors)
- ⚠️ **Plain static colliders share Newton's WORLD body (2026-09-07): count BODIES, not blocks.** A static Solid with no joint in its subtree, no TouchSensor / Connector / VacuumGripper and no cloth coupling is no longer a MuJoCo body of its own. Measured on N one-metre static boxes plus one ball: 2000 used to die in `mj_broadphase` (`mj_stackAlloc: out of memory`, its pair buffer grows with the square of the body count) and 5000 never finalised; now 2000 finalise 9 s after launch and 5000 in 16 s, the ball rests at the right height, contacts and raycasts still name the Solid, and the census line reads `N of the statics on the world body`. A static that carries a device, a joint or cloth coupling keeps its body. Two more walls fell with it: newton's `add_shape` walked every earlier world shape per add (5000 statics registered in 6.9 s, now 0.6 s) and its pair enumeration was a Python double loop (12.5 M iterations at 5000 shapes; now vectorised). The next wall is vendored: newton's own MuJoCo conversion (`_convert_to_mjc`, `add_geoms`, mujoco_warp `put_model`) is quadratic and finalises 20000 statics in ~130 s. Hatches, value-parsed: `OMNISIM_NEWTON_STATICS_ON_WORLD=0`, `OMNISIM_NEWTON_FAST_PAIRS=0`. `run-headless --until-finalized` now FAILS a world whose census registered bodies but never finalised (it printed PASS on the 5000 world). `OMNISIM_BINARY` is honoured by the harness and the lane-4 runner too, so a scratch build can be validated without touching the live engine. [full](docs/developer/agents-hard-won-rules.md#world-body-statics)
- ⚠️ **A `staticBase` robot's root COLLIDES (2026-09-30).** Until then the staticBase branch welded the root to the world with NO collision shape: grippers, props and other robots passed silently through every fixed robot base (78 tracked worlds). Now the root's `boundingObject` and its joint-free fixed children attach to the welded body; `OMNISIM_NEWTON_STATIC_BASE_COLLIDERS=0` reverts (value-parsed). Harness mutation verbs (`/robot/<def>/joints/set` and siblings) now emit `contact.*`, `grip.*` and `joint.limit_hit` during their settle steps; before, only `/sim/step` did. Pin: `tests/test_newton_static_base_colliders.py`.
- ⚠️ **A light position servo used to let every constraint through (fixed 2026-10-17).** Servos are built `kd = effort*0.5` with no armature; MuJoCo's implicit step applies `M + dt*kd` but the constraint solve uses the bare `M`, so on a light joint contacts, limits and `<mimic>` couplings acted at `M/(M+dt*kd)` strength (Robonine SO-ARM 102 jaws 13.6, wrist 857: jaws closed 21 mm into the cube and threw it). Fixed-base servos over the bound now get `dt*kd - M` of armature; free-floating robots are untouched unless `OMNISIM_NEWTON_SERVO_ARMATURE_SCOPE=all`. Hatch `OMNISIM_NEWTON_SERVO_ARMATURE_BOUND=0`. It also retired the friction-grasp guide's "newtonContactKe trap". Pin: `tests/test_newton_servo_armature_bound.py`.
- ⚠️ **A Robot's own root collider registers by default (2026-10-17)** unless it reaches >2 mm below the robot's own WHEELS at spawn (then it is quieted and the load log says by how much). Before, `newtonRobotColliders FALSE` silently dropped a URDF root link's `<collision>` (Kachaka tipped onto its nose instead of dragging). Since 2026-10-22 only branches that ROLL (a limit-less revolute / Hinge2 spin axis off the up axis) count: until then two mirror gripper fingers counted as feet, and the Xense follower standing on its base lost its base collider and fell through the floor. A robot with no wheels (gripper, arm, legs) keeps its root collider at any height. Hatch `OMNISIM_NEWTON_ROBOT_ROOT_COLLIDER=0`. Pin: `tests/test_newton_robot_root_collider.py`.
- ⚠️ **A slider position servo is at least `effort / travel` stiff (2026-10-22).** `ke = effort*10` is a hinge gain (saturates at 0.1 rad); on a slider it saturated at a 100 mm error, so a 12 mm docking pin (Kachaka) at 10 N pushed 1.2 N, under its own weight, and never moved. Motorised sliders with finite limits now get `ke = max(10, 1/travel) * effort`, kd scaled alike; ≥0.1 m travel, limit-less sliders and explicit `_TARGET_KE` / `_FINGER_KE` are untouched. Short-travel grippers (2F-85, ALOHA, Piper) squeeze harder than before. Hatch `OMNISIM_NEWTON_SLIDER_GAIN_FLOOR=0`. Pin: `tests/test_newton_slider_gain_floor.py`.
- ⚠️ **A force at an offset acts where you apply it, whatever the centre of mass (fixed 2026-11-01).** The engine sends `addForceWithOffset` (and every propeller thrust) as a torque about the Solid's ORIGIN; newton's `body_f` wants it about the COM. On a Solid whose COM is off its origin (any that declares `inertiaMatrix` + `centerOfMass`) the push acted displaced by that offset: a clothes rack with its COM 1.02 m up tipped at 62 % of the statics answer, now within 4 %. The runtime re-references it; hatch `OMNISIM_NEWTON_EXT_WRENCH_AT_COM=0`. Pin: `tests/test_newton_ext_wrench_at_com.py`.
- ⚠️ **A Solid with no `inertiaMatrix` keeps its centre of mass where it is (fixed 2026-11-03).** The geometry-derived tensor is composed about the Solid ORIGIN and was passed with the COM pinned AT the origin, so `centerOfMass` (and the boundingObject's own centroid) never reached the solver (MuJoCo `ipos` 0); only an explicit `inertiaMatrix` carried its COM. A part whose origin is off its material rested with its weight at the origin: a 40×40×20 mm 0.1 kg box with its shape 30 mm off its origin rolled 86° off a flat floor in 1 s (now 0.05 mm, 0°, like the centred box; the desk-robot assembly's caster, gearmotors, PCB and standoffs). The tensor is shifted to the COM and passed with it; the load log names the moved bodies (`geometry COM: N body(ies) ... largest D mm`). 19 of the 147 demo worlds have such bodies and all 147 still load (AnyPick objects up to 26 mm, the CNC/laser/printer twins' slider carriages up to 301 mm, the Forklift PROTO's declared COM 578 mm, the tower gantry mast 2 m); SV08 axis tracking, universal pick (14/15) and the desk-robot drive test are unchanged. Hatch `OMNISIM_NEWTON_GEOM_INERTIA_COM=0`. Pin: `tests/test_newton_geom_inertia_com.py`.
- ⚠️ **A still scene "vibrating" in the window was TAA, not physics (fixed 2026-11-01).** The resolve reprojected history from the previous frame's jitter, validated it with a depth the jitter flips on silhouettes, and cut it whenever one jittered sample differed from the average, so every edge of a still scene shimmered (idle CNC twin: 10k–59k pixels changed by >8 levels per frame, now 1.1k–3.2k; the joints moved 5e-12 m). Measure joints before chasing a physics jitter; `OMNISIM_WGPU_TAA=0` is the A/B. Hatch `OMNISIM_WGPU_TAA_STILL_EDGES=0`. Pin: `tests/test_taa_still_scene.py`.
- ⚠️ **OmniLight no longer rebakes while something deforms (fixed 2026-11-02).** Its scene key counted the dynamic tail (MillingStock tiles, cloth, soft bodies, granular, tracks, muscles), so a CNC twin cutting aluminium rebaked GI 96 times in 90 s (57 s of BVH + trace on all cores but one) and the lighting visibly swapped about once a second; now 1. Deforming geometry is baked as it stands when something else triggers a bake. Hatch `OMNISIM_OMNILIGHT_KEY_DYNAMIC=1`. Pin: `tests/test_omnilight_milling_rebake.py`.
- ⚠️ **An idle window that keeps the GPU busy: suspect float noise before the renderer (fixed 2026-11-02).** Render-on-change hashed model matrices bit for bit, and a body a servo holds away from zero moves by float noise every step (the LongMill twin parked where a job ends: up to 4e-9 m a frame), so the view never held: 23–33 % of an RTX 3060 Laptop's 3D engine for a still picture, now 6.5–10.6 %. Matrices hash at 1e-6. Windows' `\GPU Engine(*)\Utilization Percentage` counters attribute load per process; `nvidia-smi pmon` misattributed it to another app. Hatch `OMNISIM_WGPU_RENDER_ON_CHANGE_EXACT=1`. Pin: `tests/test_render_hold_servo_noise.py`.
- ⚠️ **A light position-servo axis moves like `dt × kd`, not like its mass: feed THAT forward (2026-11-02).** The servo-armature bound raises any joint lighter than `dt × kd` to it, so at 1 ms substeps and kd = 10 kN·s/m (maxForce 20 kN) a 0.65 kg printer toolhead moves like 10 kg. Fed the bare masses, the SV08 twin's axes overshot every acceleration by up to 0.3 mm; fed `max(m, dt × kd) / ke`, its walls track at 9 µm median (`projects/manufacturing/printing/world.py _controller_args`). Pin: `tests/test_printer_twin.py`.
- ⚠️ **A Display on a Shape drew WHITE under wgpu (fixed 2026-11-04).** The renderer never uploaded a Display's texture; a Display-fed `baseColorMap` is now an updatable texture re-uploaded when the Display redraws (PBRAppearance only). A screen that shows nothing on a binary built before that date is this, not your controller. Pin: `tests/test_display_texture_wgpu.py`.
- ⚠️ **One trimesh/FCL `CollisionManager` over every solid keeps at most 100,000 contacts and SILENTLY drops the pairs past it.** At 259 SparkRover solids it found 110,997 and lost two real interferences (2026-11-03). Check pair by pair over the box-overlap pairs (`spark_rover/integrate.py interference()`); never trust one manager's "no collision" on a big assembly.
- ⚠️ **On the laser twin 6 mm plywood needs 6 passes, one more than Sienci's table (2026-11-04).** The kerf model fitted to the 3 mm row predicts 5, but at 5 every SparkRover 6 mm part stayed joined by a 0.16–0.20 mm web on its outline; `spark_rover/chassis.py LASER[6.0]` is 6 passes (+3.7 min a plate). Check a part cut FREE in the height map, not the pass count.
- ⚠️ **`MillingStock` holds at most 1984 cells a side** (`OmMillingStock.cpp kMaxCellsPerAxis`): a laser sheet longer than 99 mm at 0.05 mm is cut in windows and stitched (`spark_rover/make.py`); the seams leave a sub-0.2 mm mark a real part does not have.
- ⚠️ **A Camera on a world with a `MillingStock` froze every moving body (fixed 2026-11-01)**: the milling collect re-pointed only its own draws after a reallocation, so a cached draw list read stale matrices. If camera frames freeze while the cache is on and look right with `OMNISIM_WGPU_SENSOR_DRAW_CACHE=0`, suspect a dynamic collect. Pin: `tests/test_camera_draw_cache_motion.py`.
- ⚠️ **>~8 wheeled robots on `mujoco_warp`: raise `WorldInfo.newtonNjmax` / `newtonNconmax`** — the pinned 256 overflows SILENTLY (peak `nefc` 336 at N=16). `OMNISIM_NEWTON_CONSTRAINT_STATS=<N>` measures the real peak. [full](docs/developer/agents-hard-won-rules.md#njmax-overflow)
- ⚠️ **`steps_executed` counts supervisor steps; a physics step that RAISED is skipped while the clock advances (fixed 2026-11-04).** Read `physics_ok` / `physics_steps` on `/sim/step` and `/sim/state` (from the sidecar's `step_errors`); the first raise is an engine ERROR, so `run-headless` FAILs. `mj_stackAlloc: out of memory` is MuJoCo's ARENA, fixed at compile, which `newtonNjmax`/`newtonNconmax` do NOT size on the CPU solver: raise `WorldInfo.newtonArenaMB` / `OMNISIM_NEWTON_ARENA_MB` (6,000 statics + a 2,700-piece moving body: MuJoCo's 15 MiB raised every step from the landing, 64 MiB ran clean at ~244 ms/step). A slightly short arena may instead DROP contacts with no raise (one `MUJOCO ARENA FULL` warning; a slab fell through its floor). Pins: `tests/test_physics_step_errors.py`, `tests/test_physics_step_errors_engine.py`.
- **`physicsBackend` parses; every value runs on Newton.** `"ode"` is a retired selector: it warns once per world and the Solid runs on Newton (until 2026-09-02 it left the Solid with NO physics — that trap is closed). Omit the field or write `"newton"`; inside `WorldInfo` it is an ERROR. [full](docs/developer/agents-hard-won-rules.md#backend-fields)
- **Engine defaults (v5): Newton/MuJoCo + wgpu are the ONLY physics and renderer.** No runtime = no physics (`doctor`). Ray sensors are DECLINED on `mujoco_warp`. ✅ RESOLVED: `Hinge2Joint` motors (2026-08-17), `BallJoint` motors (2026-09-01; per-axis limits still unenforced), `TouchSensor` (2026-08-13/15; bumper needs `boundingObject`, force needs `Physics` + `+X` aimed + `lookupTable [ ]`); limit-less motors promote to servos on the first `setPosition()` (`OMNISIM_NEWTON_PROMOTE_SERVO=0` reverts). ⛔ Dead: `Track` propulsion; loop-closing `SolidReference` (NO physics, headless FAILs). ODE env vars are ignored; `OMNISIM_REQUIRE_NEWTON` is PRESENCE-gated (unset to disarm). Runtime newton 1.5.0; WREN deleted 2026-08-23. [full](docs/developer/agents-hard-won-rules.md#engine-defaults)
- ⚠️ **A velocity servo's STALL TORQUE was bounded by the gain clamp (`kv <= M_ii/dt`), not by the declared effort** — fixed 2026-09-11 `69b4b024b` with rotor armature (`OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO`, default 100, `=1`/`=0` reverts); Husky yaw 0.0069 → 0.532, TB3 0.2383 → 0.958, XL 0.0069 → 0.525, straight-line unchanged. Signature: the LOADED motion fails while the unloaded one is fine, **and** the response doubles per halving of `sub_dt`. `getMaxTorque`, `setControlPID` P and the URDF effort all prove NOTHING here. [full](docs/developer/agents-hard-won-rules.md#wheel-stall-torque)
- ⚠️ **`setAvailableTorque(0)` is a FREE wheel, so a DC-motor model must BRAKE above no-load speed (2026-11-03).** The desk robot's drive controller clamped `stall × (1 − |ω|/ω₀)` at zero; in a spin the first wheel past no-load went passive (the runtime also strips its wheel armature and servo gains at effort 0), the other wheel's traction swung the robot round it, and the ground back-drove the free wheel to 58 rad/s against a 34.56 maxVelocity (mean 45.8, yaw 17.6 vs 13.87 predicted). Not an engine fault: `OMNISIM_DEBUG_LAUNCH` showed 0 actuator torque on the free wheel, the yaw acceleration matched the driven wheel's traction (21.6 predicted, 22 rad/s² measured), and with both wheels passive the yaw DECAYED. Use the line's magnitude, `stall × |1 − |ω|/ω₀|`, with the velocity target at ω₀: the servo then drives below no-load and brakes above it (spin 33.95 rad/s, yaw 13.65, straight unchanged). Before calling an over-speed joint an engine fault, check whether its torque limit was 0. [full](docs/developer/agents-hard-won-rules.md#passive-above-no-load)
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
- **Every `OMNISIM_*` variable is listed in [docs/reference/environment-variables.md](docs/reference/environment-variables.md)** (generated; `python scripts/dev/gen_env_reference.py` regenerates it — ⚠️ never hand-edit the page. Two tests pin it: [`docs/tests/test_env_reference.py`](docs/tests/test_env_reference.py) is the thorough one, but it runs under `make tests-docs`, **not** under the `make tests-unit` you actually run — it sat RED for an unknown period on 2026-09-11 with the page drifted, so [`tests/test_env_reference_current.py`](tests/test_env_reference_current.py) now repeats the drift check in the default lane). Read it before inventing a hatch, and write new boolean hatches VALUE-parsed (`=0` means OFF) — `OMNISIM_EAGER_TEXTURE_DECODE`, `OMNISIM_NO_WINDOW`, `OMNISIM_NO_GL` were presence-gated until 2026-09-02.
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

OmniSim is an **open-source robotics workshop for agents** — more than a simulator: the place where an agent has everything it needs to work on any robotic system, for a person who may have no hardware at all. Simulate a complete system on high-fidelity physics; build a digital twin from a real robot's URDF or CAD; address the simulated robot and the physical one across one control surface (the software above the driver is what is portable — §"hard-won rules"); and program, test and debug the result. Its deepest bench is the debugging one, and that is the part below: robots fail in ways you cannot see — a gripper that drops the box two times in five, a joint quietly pinned against its limit, a contact that never happened — so the surface is built to answer *what actually happened*: eleven code-verified event types on one cursor-paged stream (`GET /sim/events`, contacts, grips, `joint.limit_hit`, damage, and the controllers' own stdout), plus contact / joint / device / bounds inspection that internally holds the engine for the duration of its own walk, so each response is one consistent instant (that guard is per-read — you cannot hold it yourself; see the limits below). On the CPU solver (`newtonSolver "mujoco"`, the default) the same scene runs bitwise identically, so a failure you can reproduce is a failure you can fix. And the instruments state their own blind spots instead of guessing: `/sim/contacts` returns `completeness` and `empty_set_reasons[]` rather than an empty list, `/capabilities` publishes what the simulator refuses to do with a reason per gap, and `--fail-on-runaway` FAILs rather than passes when the evidence is too thin to certify. ⭐ **v9 added the debugger half of time control**: `POST /sim/pause` holds the engine ACROSS calls (leased — 30 s default, 300 s cap, so a dead client cannot freeze the sim), `POST /sim/break` arms a breakpoint on an event and freezes the scene when it fires, and `POST /sim/step` then single-steps — it stops early on a break and reports `steps_executed` + `stopped_on_break`, which makes it continue-to-breakpoint. ⚠️ **Detection is NEVER sub-step, and there are two regimes.** Held plus `/sim/step`: 0 supervisor-ms, at most ONE basic step of engine time. FREE-RUNNING: one SUPERVISOR TICK, and a tick is not a basic step — measured 8 ms to ~600 ms of engine time depending on load, and on a 3-body fixture a whole one-second drop fitted inside one tick and no `contact.began` fired at all. Pause, THEN step. ⚠️ **Light mode is the default** and silences 5 of the 11 event types (`contact.*`, `grip.*`, `joint.limit_hit`); a break armed on a silenced type is REFUSED (`BREAK_EVENT_TYPE_UNAVAILABLE`) rather than accepted-and-never-fired — load `{"light": false}` for a debugging session. Still open: no watch conditions, no record, no replay, no run-diff, and `POST /sim/snapshot` is not a checkpoint. It is built on the [Webots](https://github.com/cyberbotics/webots) engine, with substantial additions: the agent-facing HTTP harness above, a Camera-based capture / cinema pipeline, the omniworld procedural world generator, the OmniLink agent runtime, an RL training pipeline, CUDA-accelerated granular physics, native URDF import, multi-instance parallel execution, and the OmniSim Wire Protocol for bridges. It is an executable (`omnisim-bin.exe` on Windows, `omnisim` or `omnisim-bin` elsewhere) that loads a world file (`.wbt`), simulates physics/rendering/sensors, and spawns one **controller process** per robot. On Windows the simulator core is `omnisim-bin.exe`, fronted by two thin launchers the build also produces — `omnisim.exe` (console) and `omnisimw.exe` (windowed; this is the shipped entry point the installer's Start Menu / desktop shortcuts point at, see [`scripts/packaging/windows_distro.py`](scripts/packaging/windows_distro.py)). The build also refreshes `webots.exe` / `webotsw.exe` as byte-identical legacy copies of those launchers. There is **no `webots-bin.exe`** — no Makefile produces that name, and nothing should fall back to it. Controllers are scripts under `projects/.../controllers/<name>/<name>.py` (or `.cpp`) that talk to the simulator over an IPC channel via the `omnisim` Python / C / C++ library (⚠️ `omnisim` is the **only** spelling — the legacy `controller` Python module and the `<webots/...>` include path were deleted on 2026-08-16, and the C++ namespace is `omnisim`; see §7). To make a robot do something, you either edit its controller, generate a new world, or — for bridge-style demos like the chat robots — point an HTTP client at a port the running controller exposes.

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

Every URDF robot has a chat demo — `projects/samples/demos/worlds/chat/` (**21 chat demos: 20 `omnilink_<robot>.omniworld` plus `omniarm6_talk.omniworld`**, split 7 arms / 7 mobile bases / 6 quadrupeds / 1 drone across the four bridges; `omnilink_husky_langsoak.omniworld` (port 8775) and `husky_nav2_static_obstacle.omniworld` (the manual Nav2 benchmark) are benchmark fixtures, not demos, and 21 of the 23 pin port 8765 so two launched at once collide. All five surfaces — the launcher's `demos.json`, `DEMOS.md`, `WORLDS.md`, the chat index and the sweep — are pinned together by `tests/test_demo_catalogue.py`. The count reached 21 on 2026-09-21, when the Mavic gained a gated `/prompt` and stopped being the one robot excluded from the sweep; incl. the Deep Robotics Lite3 / X30 / M20 / M20S / M20 Piper added 2026-09-08 — [`projects/robots/deep_robotics/`](projects/robots/deep_robotics/PROVENANCE.md) also holds the DR02 humanoids and a stand world per robot): right-click → *Show Robot Window*, type `home` / `drive forward 1 m`. ⚠️ **Not a regex router** — that ladder is retired. A deterministic parser interprets first, a model is called only for what it declines (`OMNI_KEY`), and `gate.check()` vets whatever produced the frames ([guide](docs/guide/omnilink-chat-demos.md), [add your robot](docs/guide/omnilink-add-your-robot.md), [sim-to-real](docs/guide/omnilink-sim-to-real.md)). OmniLink pieces: `python -m omnisim key` ([reference](docs/guide/omnilink-key-and-api.md)); `python -m omnisim byok` (the provider key, hit as `402 BYOK_REQUIRED` — do it for the user; never write a key into this repo). Starters: **Warehouse Husky** `projects/samples/demos/worlds/showcase/warehouse_husky.omniworld` (default); Husky maze `projects/samples/demos/worlds/flagship/husky_maze.omniworld`; swarm `projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld`; Mars `distribution/generated_worlds/mars.wbt`. [full text](docs/developer/agents-reference-sections.md#section-3-launch-a-demo)

## 3d. OmniLink-co-located agents (`agents/production/`)

Agent definitions versioned next to their worlds. Reference: [`agents/production/husky_maze/`](agents/production/husky_maze/) — five maze worlds up to the agent-only `husky_maze_visual.omniworld` / `husky_maze_blind.omniworld`; read its [OVERVIEW.md](agents/production/husky_maze/docs/OVERVIEW.md) first.

```bash
launch.bat projects\samples\demos\worlds\flagship\husky_maze.omniworld
python agents/production/husky_maze/solve.py            # standalone solver
set OMNI_KEY=olink_...
python -m omnisim run-agent --agent husky_maze          # world + agent together
python -m omnisim run-agent --list
```

Showcase demos of OmniLink driving a robot — README, world generator, film scripts, evidence — live in [`agents/omnilink_demos/`](agents/omnilink_demos/README.md). The registry is discovered from `agents/production/*/omnilink.json` (`discover_agents()` in [`omnisim_run_agent.py`](scripts/dev/omnisim_run_agent.py)). Build new agents on [`agents/production/_lib/`](agents/production/_lib/README.md). [full text](docs/developer/agents-reference-sections.md#section-3d-omnilink-co-located-agents)

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
POST /prompt               # ⭐ an operator SENTENCE -> parser, then model, then the GATE
POST /tool                 # ⭐ a typed tool call {tool, ...args} -> the GATE -> the robot
POST /get_robot_state      # current pose, wheel state, fault, last tick
POST /list_robots          # [{id, model, capabilities}]
POST /set_velocity         # ⚠️ UNGATED  {v: <m/s>, w: <rad/s>}
POST /drive_forward        # ⚠️ UNGATED  {distance: <m>}
POST /stop_robot           # ⚠️ UNGATED
```

⚠️ **The five direct REST verbs bypass the safety gate. `/prompt` and `/tool` are the vetted paths.** The gate (`packages/omnisim-bridges/src/omnisim_bridges/gate.py`) is a deterministic veto on `(utterance, frames)` that does not care who produced the frames, and it sits on `/prompt` and `/tool` only — [`GATE_COVERAGE.md`](packages/omnisim-bridges/GATE_COVERAGE.md) is the authoritative list of what it does not reach. Prefer `/tool` over the bare actuator unless you specifically want to test the ungated path. A tool must be REGISTERED (`gate.register_tools`, done at relay start-up from each `Tool.parameters`) to be checked at all: `unknown_tool` is deliberately dropped at every call site, so a tool the gate has never heard of is not merely unknown, it is unchecked.

⚠️ **Rails are per-surface since 2026-09-21.** `check(utterance, frames, surface=...)` — a drone's `move_body{vertical}` is a climb railed at 120 m, a quadruped's is a body shift railed at 1.0 m. Pass your bridge's surface: with none, the rail falls back to the surface the spec recorded **only when exactly one robot class registered that tool**, and a tool two classes registered takes the ground rail instead. ⚠️ "No surface means the strictest rail" is FALSE and was a live false-refusal bug — [GATE_COVERAGE.md §1](packages/omnisim-bridges/GATE_COVERAGE.md).

⭐ **Robots can talk to each other (2026-10-20).** Set `OMNILINK_CREW="Picker=8765,Tug=8766,Packer=8767"` (and optionally `OMNILINK_CREW_OPERATOR=Sam`, `OMNILINK_CREW_LOG=<jsonl>`) for every bridge in a world and each gets a `message_robot` tool ([`crew.py`](packages/omnisim-bridges/src/omnisim_bridges/crew.py), all four bridge kinds). A message lands on the crewmate's own `/prompt` as `"Picker: …"`, always reaches its MODEL (never the parser), and the crewmate's gate vets whatever it does. Fire-and-forget, rate-capped. Worked example: [`agents/omnilink_demos/crew_order/`](agents/omnilink_demos/crew_order/README.md).

Siblings: arms [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/), quadrupeds `omnilink_quadruped_bridge`, drones `mavic_omnilink_bridge` (which gained `/prompt` + `/tool` on 2026-09-21; its `POST /action` is gated too). Contract: [PROTOCOL.md](PROTOCOL.md). [full text](docs/developer/agents-reference-sections.md#section-4-driving-a-robot-over-http)

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
| `POST /sim/pause {lease_ms?}` · `POST /sim/resume` | ⭐ Hold the engine ACROSS calls. Leased: default 30 s, clamped [1 s, 300 s], so a dead client cannot freeze the sim. Resume is idempotent; a second pause EXTENDS. [§7.38](PROTOCOL.md#738-post-simpause-post-simresume) |
| `POST /sim/break {types, filter?, lease_ms?, once?}` · `GET /sim/breaks` · `DELETE /sim/break/<id>` | ⭐ BREAKPOINT. Freezes the scene on a matching event and emits `break.hit`. ⚠️ A type this session silences is REFUSED `400 BREAK_EVENT_TYPE_UNAVAILABLE` — scoped, so `damage.*` still arms in light mode. `POST /sim/break/delete {"break_id"}` is the twin. [§7.40](PROTOCOL.md#740-post-simbreak-get-simbreaks-delete-simbreakbreak_id) |
| `POST /sim/step {steps?}` · `POST /sim/reset` | Advance N steps; rewind to t=0 AND restore the authored scene. ⭐ A step STOPS EARLY on a break: read `steps_executed` + `stopped_on_break`, never `steps_requested`. [§7.10](PROTOCOL.md#710-post-simstep), [§7.11](PROTOCOL.md#711-post-simreset) |
| `POST /sim/snapshot {name}` · `POST /sim/restore {name}` · `GET /sim/snapshots` | Named state; restore keeps the clock; unknown names → `404 SNAPSHOT_NOT_FOUND`. [§7.32](PROTOCOL.md#732-post-simsnapshot-post-simrestore-get-simsnapshots) |
| `GET /sim/state` · `GET /sim/contacts` · `GET /sim/grips` | Session + BOTH clocks + `paused` / `lease_remaining_ms` / `break_hit` / `breaks_armed`; contact set (works in light mode; `?wake=1` is a no-op); grips. ⚠️ `sim_time_ms` and `engine_time_ms` are DIFFERENT RULERS. [§7.12](PROTOCOL.md#712-get-simstate), [§7.39](PROTOCOL.md#739-get-simstate--the-pause-and-break-fields), [§7.17](PROTOCOL.md#717-get-simcontacts), [§7.18](PROTOCOL.md#718-get-simgrips) |
| `GET /sim/events` | Unified event stream, two cursors, **11 types** (`break.hit` is the eleventh). [§7.19](PROTOCOL.md#719-get-simevents) |
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
- **Performance problems** — [profiling-playbook.md](docs/developer/profiling-playbook.md); `run-headless --profile` for a load, `OMNISIM_WGPU_REPORT=1` for the renderer (`renderMs` is CPU encode+submit, not GPU time). ⛔ `OMNISIM_RENDERER_TIMINGS` is DEAD — WREN-era, absent from `src/` and from the generated [environment-variables.md](docs/reference/environment-variables.md).
- **Subsystem ownership map** — [agent-map.md](docs/developer/agent-map.md).

---

## 10. Conventions to honour

- **Do not edit `src/glm/` or `src/stb/`** (vendored submodules).
- **Do not skip git hooks** (`--no-verify`, `--no-gpg-sign`) unless explicitly told to.
- **Do not commit unless asked.** Prefer specific paths over `git add -A`.
- **Do not invent new CLI flags or scripts.** Use `python -m omnisim` and `scripts/dev/`; propose before working around.
- **Before any email outreach batch, read and obey `social/launch/EMAIL_OUTREACH_RULES_2026-08-28.md` (private; `social/` is held back from the public snapshot)**.
- **Smoke / benchmark worlds are local-asset-only** (no `http(s)://` PROTOs; `omniworld validate` and `asset_locality` enforce it).
- **`OMNISIM_HOME`** is the canonical install-root variable; `WEBOTS_HOME` survives at build time (alias from the top-level [`Makefile`](Makefile)) and in the `qt_utils` fallback ([`StandardPaths.cpp`](resources/projects/libraries/qt_utils/core/StandardPaths.cpp)). [full text](docs/developer/agents-reference-sections.md#section-10-conventions-to-honour)

---

## 11. Further reading

Full annotations: [agents-reference-sections.md → Section 11](docs/developer/agents-reference-sections.md#section-11-further-reading).

- [docs/developer/positioning.md](docs/developer/positioning.md) — **CANONICAL: what OmniSim says it is.** Every tagline, package description, doc front door and pitch paragraph must be traceable to it, and its "what is actually true" table is the honesty gate on what may be claimed. Read it before describing OmniSim to anyone.
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
