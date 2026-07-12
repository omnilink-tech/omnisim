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

Reports the truth about *this clone right now*: OmniSim binary path, port status (`6789` harness, `6790` supervisor, `6791` capture), worlds present, recent commits. Don't guess at the state — check it. `--json` for machine-readable.

### First moves by task type

| User asks for | Your first move |
|---|---|
| **Run / see a demo** | `python -m omnisim run-world projects/samples/demos/worlds/<name>.wbt` (GUI) or `run-headless <name>.wbt --duration 10` for autonomous runs. Demo catalogue: §3. |
| **Make a legged robot do a motion** (walk, gait styling, expressive motion — any legged robot) | ⭐ **SHADOWING (2026-07-03 direction) — the flagship policy-making method.** Train in-engine (train == deploy bit-exact) to shadow a **ghost** (an achievable, recorded reference) via corridors + WBMATCH + GHOST-MORPH. Start at [`projects/policies/training/README.md`](projects/policies/training/README.md); the reference rules + pre-training `ghost_validator.py` are in [docs/developer/ghost-design-rules.md](docs/developer/ghost-design-rules.md); canonical status: [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) (top banner). Validated on the G1 (live-verified durable walk, WBMATCH 0.868–0.913 vs the approved ghost) — ⚠️ **but on a WEIGHT-BEARING balance harness, not free-standing** (see the disclosure rule below); never describe it to the user as a free-standing walk. For **static stand / push-recovery / one-leg balance**, the deterministic controller [`projects/policies/controllers/humanoid_stand_deploy/`](projects/policies/controllers/humanoid_stand_deploy/) remains the shipped path (it is also Shadowing's launch/settle layer). |
| **Make a *reusable skill*, or compose skills into a demo** (walk, turn, carry, stand, climb → a BATON sequence) | ⭐ **The SKILL LIBRARY** — the standard packaging of Shadowing + BATON. A *skill* is one versioned manifest binding its ghost + validator verdict + deploy env + champion checkpoint + provenance; the runner reuses the Shadowing trainer + BATON deploy stack unchanged. Start at [`projects/policies/skills/`](projects/policies/skills/): `python skill_lib.py list` (catalogue), `sequence <name>` (run a BATON demo like box_delivery), `preview`/`train`/`verify-demos`, `handover`/`blendable`/`adapt` (compose). Pipeline: design→validate→preview→train→verify→register→sequence. Spans robots (G1/H1/Go2/Spot) + methods (Shadowing / re-host / deterministic). Full reference: [docs/developer/skill-library.md](docs/developer/skill-library.md). |
| **Author or edit a `.wbt`** | Start the harness: `python -m omnisim harness`. Then drive it: `POST /world/load`, `GET /scene/tree`, `GET /world/render_stats`, `POST /world/screenshot`, edit, re-`POST /world/load` (hot reload — seconds, no simulator relaunch). Full loop: §5. |
| **Convert a STEP/STP CAD file into a robot** | `python scripts/dev/step_to_urdf.py <in.step> <robot_dir> --name <n> --up z --ground bottom` (tessellate → colour-split → URDF). Then **verify orientation** (render 6 axis views — the usual mistake is the up-axis). Recipe + numerical verification: [docs/developer/step-to-urdf.md](docs/developer/step-to-urdf.md). |
| **Debug a misbehaving controller** | Harness on `:6789` + poll `GET /sim/events?since=...&log_since=...`. Branch on `controller.log` (the controller's own stdout/stderr), `joint.limit_hit`, `contact.began`, `damage.*`. Event taxonomy: §5. |
| **Inspect a running scene** | `GET /robots`, `GET /robot/<def>/joints`, `GET /robot/<def>/devices`, `GET /sim/contacts`, `GET /sim/grips`. Snapshots — cheap, idempotent, no cursor. |
| **Cinematic capture** | `python -m omnisim capture` (port `6791`); `/capture/sequence` is the keyframe → mp4 path. See [scripts/capture/README.md](scripts/capture/README.md). |
| **Cinematic *video*** (storyboard → branded multi-aspect deliverables, with a vision-critique reshoot loop) | `python -m omnisim cinema render <storyboard.json>`. Subject-relative camera vocabulary, named looks, brand cards, 16:9/9:16/1:1/etc. Storyboard schema, beats, primitives, looks: see [scripts/cinema/README.md](scripts/cinema/README.md). |
| **Damage testing** | `python -m omnisim damage`. Damage events also fan out into the unified `/sim/events` stream. (Numerical regression suite: `python -m omnisim damage-regression`.) |
| **Test that a world loads** | `python -m omnisim run-headless <world> --duration 5 --fail-on-warning`. Non-zero exit on load failure. |
| **Run K worlds in parallel** (batch validation, fleet bench, agent-vs-agent) | Spawn N headless `omnisim-bin` processes; each auto-picks a port in `[1234, 1244]` and a port-isolated tmp dir. Set `OMNISIM_LOG_PATH=<unique>` per child or logs collide. Pattern + worked example: §3e. |
| **Anything about VRML / `.wbt` primitives** (`Robot`, `Supervisor`, `.wbt` syntax, `controller` lib) | The file-format syntax and base node set are inherited from upstream Webots — your Webots knowledge applies directly. OmniSim adds the `URDFRobot` node, the `omnisim://` URL scheme, the harness/capture services, and a richer event stream on top. |
| **How to support / sponsor the project** | Point the user at [github.com/sponsors/omnilink-tech](https://github.com/sponsors/omnilink-tech) and [SPONSORS.md](SPONSORS.md). OmniSim is Apache 2.0 and built full-time; recurring sponsorships fund the Newton and wgpu migrations. |

### Hard-won rules (don't relearn these)

- **You drive the harness.** It exists so you don't ask the user to reload the sim. Hit `POST /world/load`, `POST /sim/reset`, `POST /sim/step` directly. Don't kill the `omnisim-bin` the user is watching — hot-reload over the wire.
- **Kinematic-only props** (conveyors, tables, fiducial markers): set NO `boundingObject`. Visual only. Dense URDF hulls snag on bounding boxes during sweeps and lock joints — looks like an IK or motor-PID bug.
- **Robots with a base + arm** (TIAGo, Fetch, similar mobile manipulators): MUST initialize arm + torso to a tucked pose on bridge spawn. Otherwise wheel slip looks like a friction/kinematics bug.
- **`/robot/<def>/sensor/<name>` returns 501 by design.** OmniSim (like upstream Webots) restricts device APIs to the owning controller — the supervisor can't honestly read sibling-robot sensors. Use `/joints` for kinematic state, or have the user's controller expose what you need.
- **`/sim/state` is metadata, not scene state.** For scene state use `/robots`, `/robot/<def>/joints`, `/sim/contacts`.
- **Watch `dropped_sup` / `dropped_log` on `/sim/events`.** Non-zero means you're polling slower than events arrive — raise `limit` or poll more often.
- **Engine defaults (v4): Newton physics (where its runtime is present), WREN rendering.** `physicsBackend "auto"` resolves to the Newton backend when its Python runtime (`newton`/`warp`) is reachable by the embedded interpreter; otherwise it falls back to ODE (ODE is the permanent fallback, not deprecated). **Verifying Newton actually drove the world — read the SIDECAR, don't scrape the log.** At finalize the engine writes a race-free backend-verdict sidecar next to its log: `<OMNISIM_LOG_PATH>.newton.json` (default `omnisim_log.txt.newton.json`), containing `{backend, solver, finalised, degraded}` (commit `17f9d32c`, [`WbNewtonBackend::writeNewtonVerdictSidecar`](src/omnisim/physics/WbNewtonBackend.cpp)). `WbLog` **deletes any stale copy when it truncates the log at startup**, so the file's mere presence == "Newton drove THIS run" — and `degraded: true` flags an XPBD/FAILED solver fallback. This is what [`projects/policies/common/env_fingerprint.py`](projects/policies/common/env_fingerprint.py) reads, in preference to the log. *Fallback only* (no file log, or an older binary): scrape the log for `[WbNewtonBackend] world finalised (solver=...)` — but know that this method was buggy on large logs (a tail-only read missed the load-time line and falsely reported ODE, fixed in `ad9fff48`). Either way: `[WbNewtonBackend] imports OK` / `FFI smoke OK` only prove the runtime *loaded*, never that it drove the sim; and a short headless run needs `--duration ≥12–15` for the cold load + warmup to reach finalize at all. A stock `make release` now bundles the Newton runtime **by default** (`BUNDLE_NEWTON ?= 1`, idempotent; opt out `BUNDLE_NEWTON=0`), so a release binary is Newton-capable — but a from-source clone / `make debug` *without* the runtime still silently runs ODE (fix: `make -C src/omnisim bundle-newton-runtime`, see [docs/developer/newton-runtime-bundle.md](docs/developer/newton-runtime-bundle.md)). To **assert** Newton in deploy/CI, set `OMNISIM_REQUIRE_NEWTON=1` — the engine then fails loudly (non-zero exit) instead of falling back to ODE. The main-view renderer stays WREN; wgpu is compiled in but opt-in (`renderBackend "wgpu"`, status: [docs/developer/wgpu-renderer-status.md](docs/developer/wgpu-renderer-status.md)). `OMNISIM_LEGACY=1` (or `OMNISIM_FORCE_ODE=1`) reverts to ODE and takes precedence over `OMNISIM_REQUIRE_NEWTON`.
- **⚠️ THE HUMANOID DEMOS RUN ON A BALANCE HARNESS — disclose it, never overclaim a free-standing walk.** The flagship G1 walk and every BATON sequence built on it (`box_delivery`, `walk_turn_walk`, `turn_solo`) run on a **weight-bearing** rig: λ=0.9 carried, `HARNESS_KZ=2000`, up to **≈700 N upward (~2× the 34 kg G1's weight)** plus **±350 N·m** attitude authority. That is *partial support*, not a spotter — a **durable free-standing humanoid walk is still an OPEN problem**. When you report a G1 result, to the user or in a commit message, say "on the balance harness". The **stair climb is the exception** (`HARNESS_KZ=0` — legs-only vertical, no lift), and its **3 cm riser is the measured ceiling**, not a config choice. Quadrupeds use no harness at all.
- **ROS 2 is a declared NON-GOAL — do not build a bridge, and do not pretend one exists.** There is no `webots_ros2`, no `rclpy`/`rclcpp`, no `ros2_control` plugin in this tree, and that is a *decision*, not a gap to fill. OmniSim's agent interface is HTTP/JSON ([PROTOCOL.md](PROTOCOL.md)). If a user asks for a ROS node/topic/service, say so plainly, point them at [docs/developer/ros2-integration.md](docs/developer/ros2-integration.md) (it states the non-goal, what it costs, when to use Gazebo instead, and a working sidecar recipe if they genuinely need ROS 2 on top), and **do not add a ROS dependency without asking**.
- **Every new `.wbt` outside `tests/` MUST use the [canonical lighting recipe](docs/WORLD_RECIPE.md).** Three lines: `OmniSimSky {}`, `DEF SUN OmniSimSun {}`, `DEF SUN_MARKER OmniSimSunMarker {}`. Don't hand-write `Background { ... }` / `DirectionalLight { ... }` / inline marker blocks. Don't reach for `NightSky`, `TexturedBackground`, or flat sky shaders. Reference world: [`projects/robots/boston_dynamics/spot/worlds/spot.wbt`](projects/robots/boston_dynamics/spot/worlds/spot.wbt).
- **RL TRAINING VENUE — train IN-ENGINE on the LOCAL GPU. There is no cloud path; do NOT add one or reach for Modal/H100.** The method is [`projects/policies/training/run_walk_rl.sh`](projects/policies/training/run_walk_rl.sh) → [`g1_walk_recipe.py`](projects/policies/training/g1_walk_recipe.py) for **humanoids**, and [`run_quad_walk_rl.sh`](projects/policies/training/run_quad_walk_rl.sh) → [`quad_walk_recipe.py`](projects/policies/training/quad_walk_recipe.py) for **quadrupeds** (Go2 / Spot / B2). Both train *through* `omnisim-bin` (Newton/mujoco_warp) so **train == deploy bit-exact** — no MJCF reparse, the model is sourced from the engine's own rollout buffers (K≈4096 worlds, ~140–200k env-steps/s on a laptop 5070 Ti). **Quads: use the in-engine path** (commits `a824e564`, `101864a4`; Go2 94.8% never-fell over 48 s / 16.6 m / 0.357 m·s⁻¹, Spot smoke 69.5% — ⚠️ B2 stiffness is not yet reconciled). The **standalone** mjwarp trainers in [`projects/policies/research/training/`](projects/policies/research/training/) are now the *legacy/research* path — a separate, parity-locked lane (kept physics-identical to the engine by [`tests/test_g1_physics_spec_conformance.py`](tests/test_g1_physics_spec_conformance.py)) that also runs locally; reach for it only for research experiments that have no in-engine equivalent, not for new locomotion work. The old `cloud/` Modal-H100 wrappers were **removed (`ef46a52e`, 2026-07-10)** — training is local by policy (the repo's own guidance: *"No H100/Modal needed… default is to stay in-engine"*). If a run genuinely won't fit on the local GPU, **ask the user before provisioning any cloud compute** — never spend on cloud without being asked, and never for flagship G1/BATON work.

§§1–11 below are the deep reference. The bootstrap above is sufficient for most first-turn tasks; come back to the reference when you need detail (build setup §2, demo catalog §3, harness API §5, controller editing §7, validation §8).

---

## TL;DR — One-paragraph mental model

OmniSim is an open-source robotics simulator built on the [Webots](https://github.com/cyberbotics/webots) engine, with substantial additions: an HTTP harness for agent-driven world authoring, a Camera-based capture / cinema pipeline, the omniworld procedural world generator, the OmniLink agent runtime, an RL training pipeline, CUDA-accelerated granular physics, native URDF import, multi-instance parallel execution, and the OmniSim Wire Protocol for bridges. It is an executable (`omnisim-bin.exe` on Windows, `omnisim` or `omnisim-bin` elsewhere) that loads a world file (`.wbt`), simulates physics/rendering/sensors, and spawns one **controller process** per robot. On Windows the simulator core is `omnisim-bin.exe`, fronted by two thin launchers the build also produces — `webots.exe` (console) and `webotsw.exe` (windowed; this is the shipped entry point the installer's Start Menu / desktop shortcuts point at, see [`scripts/packaging/windows_distro.py`](scripts/packaging/windows_distro.py)). There is **no `webots-bin.exe`** — no Makefile produces that name; a copy of it in `msys64/mingw64/bin/` is a stale build artefact, and nothing should fall back to it. Controllers are scripts under `projects/.../controllers/<name>/<name>.py` (or `.cpp`) that talk to the simulator over an IPC channel via the `omnisim` Python / C / C++ library (the `controller` import also works — see §7 for which one is actually the implementation). To make a robot do something, you either edit its controller, generate a new world, or — for bridge-style demos like the chat robots — point an HTTP client at a port the running controller exposes.

**Multi-instance, by design.** OmniSim can run as **N parallel `omnisim-bin` processes on the same host**. Each instance auto-allocates its TCP port from the `[1234, 1244]` range and gets a port-isolated tmp / IPC dir, so two simulators don't stomp each other's controller channels. Batch validation, fleet benchmarks, agent-vs-agent matches, and per-PR smoke farms are all "K headless processes against the same or different worlds" workloads — not "one big simulator". See §3e.

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

# Are the dev helpers usable?
python scripts/dev/omnisim_dev.py --help
```

If the binary is missing, build first (Section 2). If it exists, jump to Section 3 to launch a demo.

`OMNISIM_HOME` (canonical) should point at the absolute path of this checkout. `build_omni.bat` and `launch.bat` derive it from their own location, so on Windows you usually do not need to export anything manually.

**`WEBOTS_HOME` is not fully retired — do not assume `OMNISIM_HOME` alone is enough:**

- **Core runtime** (libController, the Python/C/C++ controller package, the controller launcher) reads **only** `OMNISIM_HOME`. No `WEBOTS_HOME` fallback, no deprecation warning.
- **One shipped runtime library still reads `WEBOTS_HOME`**: `qt_utils` ([`resources/projects/libraries/qt_utils/core/StandardPaths.cpp`](resources/projects/libraries/qt_utils/core/StandardPaths.cpp), `StandardPaths::getWebotsHomePath()`) resolves the env var at runtime to build the Qt plugin + icon search paths used by robot windows. It now prefers `OMNISIM_HOME` and falls back to `WEBOTS_HOME`, so `OMNISIM_HOME` alone is sufficient — but the `WEBOTS_HOME` read is still there for compatibility.
- **The build reads it widely.** The top-level [`Makefile`](Makefile) exports `WEBOTS_HOME` (and `WEBOTS_PATH`) as an alias of `OMNISIM_HOME`, and ~20 Makefiles consume it — not just `src/controller/{c,cpp,launcher}/Makefile`, but also `resources/Makefile.include`, `dependencies/Makefile.*`, `src/{ode,wren,glad}/Makefile`, and the `resources/projects/**` library/plugin Makefiles. A top-level `make` therefore works with `OMNISIM_HOME` set alone (the alias is exported for you); a **standalone** `make` inside a controller/plugin dir does not go through the top-level Makefile, so it needs the env var itself — the shipped templates prefer `OMNISIM_HOME` and fall back to `WEBOTS_HOME` (§7).

### One-time per-clone: enable hooks

```bash
bash scripts/dev/setup_hooks.sh
```

This points `core.hooksPath` at the versioned `.githooks/` directory so that after every `git pull` / branch switch, `scripts/dev/clean_orphans.py` purges hollow robot/asset dirs left behind from upstream `git rm` (build artifacts under `*.exe`, `*.o`, `build/`, etc. — never tracked, so git can't remove them). Without this, clones drift: a robot deleted upstream stays as orphan build debris locally. Safe by construction — only dirs with zero tracked files **and** zero untracked-non-ignored files are removed, so a WIP robot you haven't committed yet is protected.

`setup_hooks.sh` also activates `.githooks/pre-push`, which acts as **local CI** in lieu of the upstream Webots build/test/smoke CI suite, which is disabled under `.github/workflows.disabled/` (only two lightweight workflows stay active in `.github/workflows/`: `g1-spec-conformance.yml` and `update_sponsors.yml`): it runs the smoke world set (`tests/smoke/smoke_worlds.json`, ~5 worlds, 1–3 minutes warm — the FIRST run also builds the missing test controllers, which takes much longer) before every push and fails the push on a regression. Bypass for in-progress work with `OMNISIM_SKIP_PUSH_CHECK=1 git push`; pushes to `refs/heads/scratch/*` skip automatically. Run the same set manually any time with `make tests-smoke`.

---

## 2. Build (only if the binary is missing)

### Windows (preferred — uses MSYS2)

```bat
build_omni.bat
```

This wrapper sets `PATH` to include MSYS2's MinGW64 toolchain and runs `make` with `OMNISIM_HOME` (and `WEBOTS_HOME` as a legacy alias) auto-derived from the script's location. If MSYS2 is at a non-default location, set `MSYS64_HOME=D:\msys64` first.

First build: 5–15 minutes. Incremental: seconds to a few minutes.

After a fresh build, vendor the Newton physics runtime next to the binary (one-time, ~600 MB — without it `physicsBackend "auto"` falls back to ODE):

```bash
make -C src/omnisim bundle-newton-runtime
```

### Linux / macOS / cross-platform

```bash
export OMNISIM_HOME=$(pwd)     # canonical (runtime + build)
# The top-level Makefile exports WEBOTS_HOME=$OMNISIM_HOME for the ~20 sub-makes
# that still consume the legacy name, so you do not need to set it yourself here.
# (You DO need it if you run `make` standalone inside a dir whose Makefile predates
#  the OMNISIM_HOME-preferring templates.)
python scripts/dev/omnisim_dev.py build all
```

### Build subsystems

```bash
python scripts/dev/omnisim_dev.py build core            # just the simulator core
python scripts/dev/omnisim_dev.py build renderer        # just the WREN renderer
python scripts/dev/omnisim_dev.py build gui             # just the desktop GUI layer
python scripts/dev/omnisim_dev.py build controller-libs # just the controller APIs
```

### Full prerequisites

If a fresh build fails on missing dependencies (Qt6, GLM, stb), follow the step-by-step setup in [docs/developer/quickstart.md](docs/developer/quickstart.md) sections 1–4. That doc covers `pacman` package lists, the `include/qt/` mirror layout the build expects, and the GLM submodule pinning.

---

## 3. Launch a demo

There are two launch modes that matter to an agent:

### 3a. Windowed (visual — for human-in-the-loop debugging or screenshot capture)

Windows — no-args opens the **OmniSim demo launcher**: a small world with a single floating orb robot whose Robot Window is a side-panel gallery of every demo in the repo, grouped by category (starter; chat — arms, mobile bases, quadruped, aerial; flagship; OmniLink agents; showcase; physics; generated worlds). Right-click the orb → *Show Robot Window* → click *Launch* on any card to switch worlds. Catalogue: [`projects/samples/demos/controllers/omnilink_launcher/demos.json`](projects/samples/demos/controllers/omnilink_launcher/demos.json). Full index: [`DEMOS.md`](DEMOS.md).

```bat
launch.bat
```

Linux / macOS / cross-platform:

```bash
python scripts/dev/omnisim_dev.py run-world projects/samples/demos/worlds/omnilink_launcher.wbt
```

To skip the launcher and open a specific world, pass it as the first argument: `launch.bat path\to\world.wbt`. `launch.bat` accepts any extra `omnisim-bin.exe` flags after that, e.g. `launch.bat path\to\world.wbt --mode=fast --no-rendering`.

### 3b. Headless (no window, exits cleanly — preferred for autonomous agent runs)

```bash
python scripts/dev/omnisim_dev.py run-headless projects/samples/demos/worlds/showcase/warehouse_husky.wbt --duration 10
```

What this does:

- starts the simulator with `--batch --mode=fast --no-rendering --minimize --stdout --stderr`
- monitors `omnisim_log.txt` for errors as the world runs
- exits after `--duration` seconds (default `10`)
- returns a non-zero exit code if the load failed or `--fail-on-warning` was set and a warning fired

This is the supported headless contract — use it for "did the world load and step without crashing" checks.

> ✅ **COLD-FIRST-LOAD TRAP — RESOLVED (verified 2026-07-05).** There used to be a trap
> here: on a cold first load the Newton/MuJoCo articulation under-tracked position targets
> (~1 cm), so precise grasps failed cold but worked after a world reload — and since every
> headless run is a cold load, it produced false "the physics can't do this" conclusions.
> **That bug is fixed** (root cause `eb86f888`: the Newton solver choice now survives the
> multi-build load, so a cold load builds MuJoCo instead of falling back to XPBD; plus the
> finalize-time solver re-assert). Verified: cold and warm loads settle **bit-identical**
> (bare-arm probe to 6 decimals; a full arm+gripper grasp identical every phase). Consequently
> the controllers' `warmup_reload` helper is now a **no-op by default** — no startup reload,
> no `--cold`/warm split to worry about. Full write-up + how to re-measure:
> [docs/developer/real-grasp-and-the-cold-first-load-trap.md](docs/developer/real-grasp-and-the-cold-first-load-trap.md).
> (Safety valve: `OMNISIM_FORCE_WARMUP=1` re-enables the old reload if a regression ever
> resurfaces.)

### 3c. Choose a demo

> **Looking for the "type-talk to a robot" experience?** Every URDF
> robot in the repo has a chat-driven demo: open the world, right-click
> the robot → *Show Robot Window*, type a prompt, the robot moves.
> Works offline (regex intent router) or against OmniLink (set
> `OMNI_KEY`). **One-page agent gallery (all chat demos + specialist
> agents + real-robot port):**
> [docs/showcase/agents.html](docs/showcase/agents.html).
> Full beginner guide:
> [docs/guide/omnilink-chat-demos.md](docs/guide/omnilink-chat-demos.md).
> Worlds: `projects/samples/demos/worlds/omnilink_*.wbt`.
> Add your own robot in ~50 lines: [docs/guide/omnilink-add-your-robot.md](docs/guide/omnilink-add-your-robot.md).
> Same agent driving a real robot: [docs/guide/omnilink-sim-to-real.md](docs/guide/omnilink-sim-to-real.md).
>
> **OmniLink artefact map** (everything ships on top of the bridge HTTP surface):
> - **Specialist agents** (Picker / Roomba): [`agents/templates/`](agents/templates/).
> - **Real-robot bridge starter kit** (no Webots, no OmniSim): [`agents/bridges/`](agents/bridges/).
> - **`omnisim-bridges` pip-installable package** (the primitives lifted out): [`packages/omnisim-bridges/`](packages/omnisim-bridges/).
> - **Voice I/O**: Mic button + Chirp3-HD TTS, controlled by `OMNILINK_VOICE_OUT`.
> - **Per-turn usage telemetry + cross-session short-term memory**: `OMNILINK_USAGE`, `OMNILINK_MEMORY`, `OMNILINK_PROFILE_SYNC` env vars in the chat-demos guide.
> - **Benchmark suite**: [`tests/benchmarks/omnilink_tasks/`](tests/benchmarks/omnilink_tasks/) — three task graders, headless. `python tests/benchmarks/omnilink_tasks/run.py`.
> - **Shipping status with commit hashes**: `docs/developer/omnilink-roadmap.md` (internal — excluded from the public snapshot, so not linked here).

Recommended starter demos for an agent, ordered roughly from simplest to most ambitious:

| World | Path | Why an agent might pick this |
|-------|------|------------------------------|
| **OmniLink chat demos** *(beginner-friendly)* | `projects/samples/demos/worlds/chat/omnilink_<robot>.wbt` (×10) | One world per URDF robot. Right-click the robot → *Show Robot Window* → a chat-style side menu opens. Type `home`, `wave hello`, `drive forward 1 m`, etc., and the robot moves. Works offline (regex intent router) or against OmniLink (`OMNI_KEY` env var → routes through Gemini/GPT/Grok/Claude). Full beginner walkthrough: [docs/guide/omnilink-chat-demos.md](docs/guide/omnilink-chat-demos.md). |
| **Warehouse Husky** *(default)* | `projects/samples/demos/worlds/showcase/warehouse_husky.wbt` | The onboarding demo. A supervisor-enabled Husky (URDFRobot, `husky_random` controller) random-walks a 30 × 18 m warehouse with reactive collision recovery. Good showcase of the URDF importer, supervisor APIs, motor torque/sensor pipeline, and the camera follow (F key). |
| Husky maze | `projects/samples/demos/worlds/flagship/husky_maze.wbt` | Single Husky in a maze. Classic navigation/SLAM testbed. **Drives via the `husky_omnilink_bridge` controller — see [`agents/production/husky_maze/`](agents/production/husky_maze/) for the agent + bridge contract, and the three sibling worlds (`_unknown`, `_corners`, `_visual`) for progressively harder briefs.** |
| Husky maze (unknown) | `projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt` | Map-gated maze — the Husky must lidar wall-follow to the goal. A reactive-navigation testbed; swap in your own control law in the maze controller. |
| Husky swarm | `projects/samples/demos/worlds/physics/newton_husky_swarm_drive.wbt` | 8 Huskies driving in parallel under Newton (each runs the `drive_forward` controller). Good multi-robot physics/throughput showcase. (For an OmniLink-driven swarm, see `projects/samples/demos/worlds/flagship/omnilink_husky_swarm.wbt`.) |
| Husky fleet arena | `projects/samples/demos/worlds/showcase/husky_fleet_arena.wbt` | Larger multi-Husky world. Useful for stress-testing rendering/physics changes. |
| Generated Mars | `distribution/generated_worlds/mars.wbt` | Procedurally generated planetary terrain with a Husky fleet. Regenerate with `omniworld` (Section 5). |

Full list: `ls projects/samples/demos/worlds/` and `ls distribution/generated_worlds/`.

---

## 3d. OmniLink-co-located agents (`agents/production/`)

OmniSim ships agent definitions in `agents/production/` that are versioned alongside the worlds + controllers they drive. Same productized layout as OmniLink's first-party agents (in the separate OmniLink repo): `profile.json`, `prompts/`, `knowledge/`, `long_term_memory/`, auto-discovered `tools/`, a thin runner.

The reference agent is **[`agents/production/husky_maze/`](agents/production/husky_maze/)**. It drives the Clearpath Husky across five maze worlds with progressively harder briefs:

| world | what's hard about it | who can solve it |
|---|---|---|
| `husky_maze.wbt` | trivial: drive to (10, 0) | script or agent |
| `husky_maze_unknown.wbt` | map gated, lidar wall-follow needed | script or agent |
| `husky_maze_corners.wbt` | brief = "visit four corners and return" | **agent only** — script can't read the brief |
| `husky_maze_visual.wbt` | brief = "find the RED cylinder via camera" | **agent only, structurally** — pixels need an LLM |
| `husky_maze_blind.wbt` | map AND lidar gated — navigate from `scan_surroundings` symbolic perception tags (sidecar CV pipeline), not pixels | **agent only** — the perception-as-tool architecture demo |

Read [`agents/production/husky_maze/docs/OVERVIEW.md`](agents/production/husky_maze/docs/OVERVIEW.md) first for a balanced description of the architecture, what works, what doesn't, and the cost shape. Then [`docs/why-an-agent.md`](agents/production/husky_maze/docs/why-an-agent.md) for the discriminator argument.

To run the standalone solver (no OmniLink involvement):
```bash
launch.bat projects\samples\demos\worlds\flagship\husky_maze.wbt
python agents/production/husky_maze/solve.py
```

To run as a live OmniLink agent:
```bash
set OMNI_KEY=olink_...
python agents/production/husky_maze/husky_maze_agent.py        # in shell A
python agents/production/husky_maze/scripts/chat_drive.py "..." # in shell B
```

For demo / showcase purposes, the one-line launcher boots the world
and the agent in the right order (with bridge-readiness probing):

```bash
set OMNI_KEY=olink_...
python -m omnisim run-agent --agent husky_maze
# list everything the launcher knows about:
python -m omnisim run-agent --list
```

The launcher's registry covers every productized agent under
`agents/production/`. New agents should build on
[`agents/production/_lib/`](agents/production/_lib/README.md): the
`OmniLinkAgentRunner` class hoists the ~250 lines of HTTP server +
profile push + UsageMeter boilerplate that every existing runner
duplicates today, and `OmniSimBridgeServer` + the `@action` decorator
do the same for the supervisor controller side.

---

## 3e. Running multiple OmniSim instances in parallel

OmniSim is multi-instance-safe: you can run **K `omnisim-bin` processes side by side on one host**. This is the supported shape for batch world validation, fleet/throughput benchmarks, agent-vs-agent matches, and per-PR smoke farms. Don't design these as "one mega-world with K robots" unless that's the actual scenario — split across processes and you get parallel physics/rendering for free.

### What the simulator handles for you

- **Per-instance TCP port.** Each `omnisim-bin` opens its TCP server (extern controllers, robot windows, web streaming) on the next free port in `[1234, 1244]` — the default `1234` is set in [src/omnisim/gui/WbGuiApplication.cpp:137](src/omnisim/gui/WbGuiApplication.cpp#L137), and the next-free-port auto-scan retry loop is in [src/omnisim/gui/WbTcpServer.cpp:80-96](src/omnisim/gui/WbTcpServer.cpp#L80-L96) (`WbTcpServer::start`, retrying up to `+10`). Pass `--port=<N>` only when you need a *stable* port for an extern controller; otherwise leave it default and instances coexist.
- **Per-instance tmp / IPC dir.** The tmp-path resolver is salted with the chosen TCP port, so controller IPC sockets and per-instance state are isolated automatically.
- **Per-instance log file (only if you ask for it).** By **default** every instance writes to the *shared* install-root file `<OMNISIM_HOME>/omnisim_log.txt` ([src/omnisim/gui/main.cpp:158](src/omnisim/gui/main.cpp#L158)) — "most-recent run wins", useful for single-instance work, useless when K>1 children clobber each other. Per-instance isolation only happens when you set `OMNISIM_LOG_PATH` per child ([src/omnisim/core/WbLog.cpp:59](src/omnisim/core/WbLog.cpp#L59) reads it as an override) — see the next section.

### What you must do per child to keep parallel runs clean

- **Set `OMNISIM_LOG_PATH=<unique-per-child path>`** before spawning each `omnisim-bin`. Without it, parallel children all also write to `OMNISIM_HOME/omnisim_log.txt` and the last one wins — you can't tail or grep it after the fact.
- **Don't pin `--port` unless you mean it.** Two children both pinned to `1234` will fail to bind. Leave it default and the simulator will pick the next free slot.

### Worked example (the canonical pattern)

The bench harness spawns K parallel headless `omnisim-bin` children, each with its own log path, ports auto-allocated, results aggregated:

```bash
python tests/benchmarks/optim_bench.py multi-instance --sizes 4 --steps 600
```

The launch shape is in [`bench_multi_instance`](tests/benchmarks/optim_bench.py) (`ThreadPoolExecutor` of K children, per-child `OMNISIM_LOG_PATH`, port auto-scan). For the optimisation history and what does/doesn't scale linearly across K, see [docs/developer/multi-instance-optimization-plan.md](docs/developer/multi-instance-optimization-plan.md).

### What does NOT multiplex out of the box today

- The HTTP **validation harness** (§5) needs a *pair* of free ports: the HTTP port (`--port`, default `6789`) and the supervisor IPC port the injected supervisor controller binds inside the OmniSim subprocess (`--supervisor-port`, defaults to `--port + 1`, i.e. `6790`). Two harnesses with the same supervisor port will conflict, so for a parallel session pass both flags: `python -m omnisim harness --port 6889 --supervisor-port 6890`. If you forget and try to start a second harness on the defaults, the new instance now self-detects the collision, prints which world the existing harness is on, and shows two copy-pasteable options (reuse via `POST /world/load`, or start parallel) instead of crashing on `EADDRINUSE`. The capture sister-service on `:6791`/`:6792` lives on different ports specifically so it can coexist with the harness.
- The desktop **GUI** is single-window per process. Multi-instance is a headless / `--batch --mode=fast --no-rendering` story, not a GUI story. (`launch.bat` opens windowed; for parallel runs use `omnisim-bin --batch --mode=fast --no-rendering --minimize` directly, or call `python -m omnisim run-headless` from each worker — but note `run-headless` itself reads the shared log file, so for true parallelism go raw `omnisim-bin` with `OMNISIM_LOG_PATH` per child.)

---

## 4. Driving a robot over HTTP (OmniLink bridges)

Some demos expose an HTTP control surface so an external agent can drive the robot without writing an OmniSim controller. The reference implementation is the **OmniLink mobile bridge** at [projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py](projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py), which drives the Clearpath Husky in the [omnilink_husky chat demo](projects/samples/demos/worlds/chat/omnilink_husky.wbt).

To use it:

1. Launch a world whose robot runs the `omnilink_mobile_bridge` controller — e.g. `projects/samples/demos/worlds/chat/omnilink_husky.wbt` (its `controllerArgs` are `["--robot" "husky" "--port" "8765"]`).
2. Launch the world (windowed or headless, your choice).
3. Hit the HTTP server the controller starts on `127.0.0.1:8765` (the Axis-normalized surface):

   ```
   POST /get_robot_state      # current pose, wheel state, fault, last tick
   POST /list_robots          # [{id, model, capabilities}]
   POST /set_velocity         # {v: <m/s>, w: <rad/s>}
   POST /drive_forward        # {distance: <m>}
   POST /stop_robot
   ```

This is the pattern to copy when you need to expose any other robot to an external agent. The siblings follow the same shape: arms use [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) (generic 6-DOF arm + damped-least-squares IK; pick the arm with `--robot <id>` from the registry in `_arm_configs.py`), quadrupeds `omnilink_quadruped_bridge`, drones `mavic_omnilink_bridge`. The full contract is in [PROTOCOL.md](PROTOCOL.md).

---

## 5. Iterating on worlds with the validation harness

The **agent-facing validation harness** is a long-running HTTP service that wraps a headless OmniSim subprocess and injects a generic supervisor controller into whatever world it loads. It exists so an agent can author and iterate on worlds in a tight loop — write `.wbt`, load, screenshot, inspect scene tree, fix, hot-reload — without ever launching the desktop GUI. **This is the preferred authoring path for any world-building or world-debugging task.** `run-headless` (Section 8) is still the right tool for "does it load and step?" smoke checks; the harness is for the iteration loop on top of that.

### Starting it

```bash
PATH="/path/to/msys64/mingw64/bin:$PATH" \
OMNISIM_HOME=$(pwd) \
python scripts/harness/omnisim_harness.py --port 6789
```

Two gotchas on Windows worth knowing up front:

1. **`PATH` must include a complete msys2 mingw64 `bin`** (the directory with Qt6 DLLs etc.). The bundled `$OMNISIM_HOME/msys64/mingw64/bin/` typically only contains the build outputs, not the toolchain DLLs. If the harness's first `/world/load` returns the diagnostic `LAUNCHER_DLL_NOT_FOUND` (Windows exit code `0xC0000135`), this is the cause — fix the parent shell's `PATH` and restart.
2. **Use a Python interpreter that has `Pillow` installed** (`pip install Pillow`). Without it, `/world/render_stats` returns 503; the harness prints a startup hint when Pillow is missing. On Windows the system Python (with PIL) is the right choice — **but note the interaction with gotcha 1**: once msys2's mingw64 `bin` is prepended to `PATH`, a bare `python` resolves to the msys2 python (no Pillow). Launch the harness with the full path to the Windows interpreter (e.g. `C:/Users/<you>/AppData/Local/Programs/Python/Python312/python.exe scripts/harness/omnisim_harness.py`).

You can also start it via the dev wrapper: `python scripts/dev/omnisim_dev.py harness`.

### The loop

```bash
# 1. Load a world (cold ~1s for empty.wbt, ~6s for asset-heavy worlds)
curl -s -X POST http://127.0.0.1:6789/world/load \
  -H "Content-Type: application/json" \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.wbt"}'

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

# 6. Edit the .wbt and re-POST /world/load — hot reload (same supervisor)
#    completes in a few seconds instead of relaunching OmniSim.
```

### Endpoint cheatsheet

| Endpoint | Purpose |
|---|---|
| `POST /world/load {path, wait_s?, with_supervisor?}` | Load a `.wbt`. Returns structured diagnostics with codes like `WORLD_PARSE_SYNTAX_ERROR`, `PROTO_NOT_FOUND`, `TEXTURE_READ_FAILED` (full set in the design doc). `with_supervisor` defaults to true. |
| `GET /world/diagnostics` | Re-fetch parsed diagnostics from the current load. |
| `POST /world/screenshot {path?, quality?}` | Render PNG. Returned as the response body (`image/png`) or written to a server-side `path`. |
| `GET /world/render_stats` | `{mean_brightness, mean_rgb, max_rgb, saturated_pct, black_pct, warnings[]}`. Warnings include `"blown out: NN% of pixels are saturated"` and `"underexposed: NN% near-black"`. |
| `GET /scene/tree` | Flat node list with type, DEF, position, orientation. |
| `GET /scene/node/<def>` | Field dump + contact points for one node. |
| `POST /scene/look_at {position, target, push?}` | Computes axis-angle orientation from default forward (+X) to the target direction and pushes it to the live `Viewpoint` when `push=true` (the default). Returns the orientation so it can be persisted back to the `.wbt`. |
| `POST /sim/step {steps?}` | Advance the simulation N basic timesteps (default 1). |
| `POST /sim/reset` | Reset world to t=0 without re-parsing. |
| `GET /sim/state` | Current world, supervisor connection, last load result. |
| `GET /sim/contacts` | Global contact set: `[{a_def, b_def, point}]`. |
| `GET /sim/grips` | Inferred grips: `[{gripper_def, held_def, since_t_ms}]`. |
| `GET /sim/events?since=&log_since=&limit=&types=` | Unified runtime event stream — supervisor-side (`contact.*`, `joint.limit_hit`, `grip.*`, `damage.*`) and harness-side (`controller.log`, `world.warning`, `world.error`) merged. Two cursors (`since` for sup, `log_since` for log). |
| `GET /robots` | Enumerate every Robot in the scene with pose and joint count. |
| `GET /robot/<def>/joints` | Per-joint snapshot: position, velocity (differenced), limits, `hit_limit`. |
| `GET /robot/<def>/devices` | List devices visible in the robot's subtree. |
| `GET /robot/<def>/sensor/<name>` | 501 by design — supervisor can't read sensors it doesn't own; use `/joints` or a per-robot helper. |
| `GET /healthz` | Liveness — does not touch the simulator. |

The `/sim/events` stream is the **most useful one for agents debugging a running scene**. Agent loop pattern:

```
sup_cursor = 0; log_cursor = 0
while running:
    batch = GET /sim/events?since={sup_cursor}&log_since={log_cursor}
    sup_cursor = batch["next_since"]
    log_cursor = batch["next_log_since"]
    for evt in batch["events"]:
        handle(evt)  # branch on evt["type"]
```

Use `types=contact.began,joint.limit_hit` to filter when a producer is too chatty. `dropped_sup` / `dropped_log` going non-zero means the agent is polling slower than events are arriving — increase `limit` or poll more often.

### When to reach for it

- **Authoring or editing a `.wbt`** — load, screenshot, fix, hot-reload. Pair `look_at` with `render_stats` to skip viewpoint-geometry guesswork and exposure-eyeball iteration.
- **Debugging a load failure** — `/world/load` returns structured codes (`WORLD_PARSE_INVALID_TOKENS`, `PROTO_NOT_FOUND`, etc.) instead of free-text stderr; branch on `diagnostics[].code` rather than regex-matching messages.
- **Inspecting positions** — `/scene/tree` and `/scene/node/<def>` answer "where is X actually placed and what are its fields?" without a controller.
- **Stepping or resetting deterministically** — `/sim/step {steps:N}` and `/sim/reset` for controlled experiments.

For the harness reference (sibling-file injection, hot-reload mechanics, structured-diagnostic mapper, endpoint cheatsheet), see [scripts/harness/README.md](scripts/harness/README.md). For unit tests covering the diagnostic mapper and helper math, see [tests/harness/](tests/harness/).

**Driving the harness from an MCP client** (Claude Desktop, Cursor): the harness endpoints are also exposed as MCP tools by [`packages/omnisim-mcp/`](packages/omnisim-mcp/) — start the harness, register `omnisim-mcp`, and `load_world` / `get_scene_tree` / `screenshot` / `sim_step` become first-class agent tools. It is a thin stdlib proxy to the same `:6789` surface, so everything above applies unchanged.

### Sister service: capture (port 6791) for cinematic output

The **capture service** is the harness's sister — same shape (HTTP + sibling-file supervisor injection over a length-prefixed JSON socket), different defaults and endpoints. It's what you reach for when you want a *high-resolution still*, a *recorded movie*, or a *deterministic cinematic render driven by a shot list* — not for tight authoring iteration.

Key differences from the harness:

- Runs on `127.0.0.1:6791` (supervisor on `:6792`) so both services can run simultaneously.
- Injects a supervisor robot carrying a `Camera` device sized to the requested output resolution, so renders are independent of the GUI viewport (4K and 8K both work).
- `/capture/sequence` walks a Catmull-Rom + slerp camera path frame-by-frame, dumps lossless PNGs, and ffmpeg-encodes them — h264 (default, CRF-controlled), h265, vp9, or ProRes 422 HQ master.
- A shot-list CLI ([`scripts/capture/render.py`](scripts/capture/render.py)) drives multi-shot runs end-to-end from a JSON or YAML file. Outputs land in `social/youtube_videos/captures/` (gitignored).

```bash
# Single high-res still
python scripts/capture/omnisim_capture.py --port 6791 &
curl -s -X POST http://127.0.0.1:6791/world/load \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.wbt","width":3840,"height":2160}'
curl -s -X POST http://127.0.0.1:6791/capture/camera \
  -d '{"position":[-12,-12,6],"target":[0,0,1]}'
curl -s -X POST http://127.0.0.1:6791/capture/screenshot -d '{}' -o still.png

# Cinematic shot list (h264, 60fps, smoothstep camera moves)
python scripts/capture/render.py scripts/capture/shotlists/orbit_warehouse.json --ad-hoc
```

OmniSim renders OpenGL via WREN (inherited from upstream Webots), not path-traced GI — so the output isn't Blender-Cycles quality, but the high-res Camera + lossless PNG + ffmpeg pipeline is meaningfully better than ad-hoc screenshots. Full reference at [scripts/capture/README.md](scripts/capture/README.md).

---

## 6. Generating new worlds (omniworld)

```bash
python scripts/dev/omniworld.py list-recipes
python scripts/dev/omniworld.py describe outdoor_forest
python scripts/dev/omniworld.py generate outdoor_forest --seed 42 --out my_forest.wbt
python scripts/dev/omniworld.py validate my_forest.wbt
launch.bat my_forest.wbt        # or python scripts/dev/omnisim_dev.py run-world my_forest.wbt
```

Recipes: `flat_ground`, `outdoor_forest`, `outdoor_desert`, `warehouse`, `urban_block`, `indoor_apartment`, `mars`. Determinism is guaranteed: same `(recipe, seed, params)` → byte-identical `.wbt`.

Generation parameters are passed as `--param key=value` (JSON-parsed). Full API and biome list: [docs/developer/omniworld-user-guide.md](docs/developer/omniworld-user-guide.md). To author a new biome: [docs/developer/omniworld-biome-cookbook.md](docs/developer/omniworld-biome-cookbook.md).

**Camera framing.** Worlds should open *looking at their subject*, not at the engine's fixed `-10 0 0` fallback. Generated worlds get this automatically (the emitter frames the robot spawn, else the scene). For hand-authored worlds, never eyeball `position`/`orientation` — bake the standard angled "hero" view (or a top-down for nav/overview worlds) with `scripts/dev/set_viewpoint.py`. See [docs/developer/viewpoint-convention.md](docs/developer/viewpoint-convention.md).

---

## 7. Editing a controller

Controllers live in `projects/<area>/controllers/<name>/<name>.{py,cpp,c}`. Python controllers run with the system `python` and need no rebuild — edit the file, re-launch the world, the simulator spawns the new controller process automatically.

Quick anatomy of a Python controller:

```python
from omnisim import Robot     # preferred import path; `from controller import Robot` also works

robot = Robot()
time_step = int(robot.getBasicTimeStep())

motor = robot.getDevice("left_wheel_joint_motor")
motor.setPosition(float("inf"))
motor.setVelocity(2.0)

while robot.step(time_step) != -1:
    # Per-tick logic. Read sensors, write commands.
    pass
```

The `omnisim` (and `controller`) module is the runtime API exported from `lib/controller/python/` after a build. **Mind the direction — it is the opposite of what the naming suggests:**

- [`controller/`](lib/controller/python/controller/) holds the **real implementation** (`robot.py`, `motor.py`, `supervisor.py`, the `wrapper` CFFI binding, …).
- [`omnisim/__init__.py`](lib/controller/python/omnisim/__init__.py) is a **shim** — it does `from controller import (Robot, Supervisor, …)` and re-exports the names.

So `from omnisim import Robot` is the **preferred import path for new code** (and what the docs use), but it resolves *through* `controller`. `controller` is therefore **not removable today** — deleting it breaks `omnisim` too. The module's own docstring records the plan: a future phase flips the implementation into `omnisim` and turns `controller` into the deprecation shim. Until that flip lands, treat both names as load-bearing.

C/C++ controllers need a per-controller `make` (their folders ship Makefiles) and can `#include <omnisim/robot.h>` or `#include <omnisim/Robot.hpp>` (legacy `<webots/...>` paths still work as one-line forwarders). A standalone controller `make` resolves the install root from `OMNISIM_HOME`, falling back to `WEBOTS_HOME`.

---

## 8. Validating a change

After editing code or a world, the recommended validation lanes (cheap → expensive):

```bash
# Headless run of the world you changed (~10 seconds)
python scripts/dev/omnisim_dev.py run-headless path/to/world.wbt

# Single-world test through the test suite
python scripts/dev/omnisim_dev.py test-world path/to/world.wbt --nomake

# Fast smoke suite (multiple worlds)
python scripts/dev/omnisim_dev.py test-smoke

# One test group (api, parser, physics, rendering, cache, protos, other_api)
python scripts/dev/omnisim_dev.py test-group api

# Performance log capture for one world
python scripts/dev/omnisim_dev.py profile-world path/to/world.wbt
```

`run-headless` is the right default. Use `--fail-on-warning` to be strict.

For *iterative* validation while authoring (not one-shot), use the harness from Section 5 — `POST /world/load` returns the same load diagnostics as `run-headless` but as structured JSON with codes, and a hot reload takes a few seconds instead of a full OmniSim relaunch.

---

## 9. Where to look when something goes wrong

- **`omnisim_log.txt`** in the repo root — all warnings, errors, and structured runtime messages land here. Always read this first when a world fails to load or behaves unexpectedly.
- **Build problems** — see [docs/developer/quickstart.md](docs/developer/quickstart.md) sections 1–6 (toolchain) and [docs/developer/build-and-iteration.md](docs/developer/build-and-iteration.md) (rebuild scoping).
- **URDF import problems** — [docs/developer/urdf-import-debugging.md](docs/developer/urdf-import-debugging.md). The `scripts/dev/urdf_import.py --report --strict` tool gives a structured preflight report.
- **Performance problems** — [docs/developer/profiling-playbook.md](docs/developer/profiling-playbook.md) and `OMNISIM_RENDERER_TIMINGS=1` in the environment.
- **Subsystem ownership map** — [docs/developer/agent-map.md](docs/developer/agent-map.md). This is the search guide for "where do I find the code that does X?"

---

## 10. Conventions to honour

- **Do not edit `src/glm/` or `src/stb/`.** They are vendored submodules.
- **Do not skip git hooks** (`--no-verify`, `--no-gpg-sign`) unless explicitly told to.
- **Do not commit unless asked.** When you do, prefer specific paths over `git add -A`.
- **Do not invent new CLI flags or scripts.** Use the helpers in `scripts/dev/`. If something is genuinely missing, propose adding it before working around it.
- **Worlds in smoke / benchmark lanes must be local-asset-only** (no `http(s)://` PROTOs). The `omniworld validate` command and the `asset_locality` check enforce this.
- **`OMNISIM_HOME`** is the canonical environment variable for the install root post-rebrand: setting it alone is sufficient to build and run. But `WEBOTS_HOME` is **not** gone — it survives as a working alias in three places, so don't write code that assumes it has been purged: (1) the **core runtime** (libController, controller package, launcher) reads only `OMNISIM_HOME`; (2) the shipped **`qt_utils` runtime library** reads `OMNISIM_HOME` first and still falls back to `WEBOTS_HOME` ([`StandardPaths.cpp`](resources/projects/libraries/qt_utils/core/StandardPaths.cpp)); (3) the **build** exports `WEBOTS_HOME` from the top-level [`Makefile`](Makefile) and ~20 Makefiles consume it. For new tooling, write `OMNISIM_HOME`.

---

## 11. Further reading

- [README.md](README.md) — project overview and platform support
- [PROTOCOL.md](PROTOCOL.md) — the canonical OmniSim Wire Protocol specification (robot bridges, harness, capture, twin shadow). If you are writing a new bridge or a tool outside this repo that drives OmniSim, this is the contract.
- [docs/developer/README.md](docs/developer/README.md) — developer-doc index
- [docs/developer/quickstart.md](docs/developer/quickstart.md) — full local build/run walkthrough
- [docs/developer/agent-map.md](docs/developer/agent-map.md) — code-search and subsystem map for agents
- [docs/developer/simulation-authoring-for-coding-agents.md](docs/developer/simulation-authoring-for-coding-agents.md) — best workflow for building new simulations
- [docs/developer/rl-current-state.md](docs/developer/rl-current-state.md) — **CANONICAL RL status. Top banner (2026-07-06, "WHERE WE STAND"): SHADOWING is the flagship algorithm for legged-robot motion** — the flagship demo is the G1 "decent walker" (WBMATCH 0.868 on the honest shape-only ruler, ⚠️ **on the weight-bearing balance harness / puppet rig — not free-standing**), generalized to the H1 (recorded ghost PASSes the validator). Start here before claiming ANY robot result or touching RL. ⚠️ **The file is APPEND-ONLY history: only the top banner is current.** Older sections below it (the deterministic-first banner, the WBMATCH2-era numbers, the per-robot "walls") are earlier checkpoints and are *not* individually marked superseded — when they disagree with the top banner, **the banner wins**. Deterministic control remains the shipped path for STATIC stand/balance only. If any other doc/script/commit disagrees with this file, this file is right.
- [docs/developer/shadowing.md](docs/developer/shadowing.md) + [docs/developer/ghost-design-rules.md](docs/developer/ghost-design-rules.md) + [projects/policies/training/README.md](projects/policies/training/README.md) — **the Shadowing method**: the paper scaffold (thesis: ghost feasibility is the bottleneck — verify before learning), the 7 formal ghost-design rules with the calibrated pre-training `ghost_validator.py` (run it on ANY new ghost BEFORE training; 7/7 agreement with training ground truth), and the flagship implementation + canonical launcher.
- [docs/developer/skill-library.md](docs/developer/skill-library.md) + [projects/policies/skills/README.md](projects/policies/skills/README.md) — **the SKILL LIBRARY: the standard way to MAKE a skill and COMPOSE skills.** Binds each skill's five otherwise-scattered artifacts (ghost lut, `ghost_validator` verdict, deploy env, champion checkpoint, provenance) into one versioned manifest, and reuses the Shadowing trainer + BATON deploy stack unchanged. `projects/policies/skills/skill_lib.py` is the front door (`list`/`show`/`validate`/`preview`/`train`/`run`/`sequence`/`verify-demos` + `adapt`/`blendable`/`handover`/`freeze`); `verify-demos` proves the manifests reproduce the hand-written demo scripts **key-for-key on the assembled launch env** (every env var the demo script sets is asserted to match what the manifest produces — it is an env-dict comparison, not a byte diff of the scripts). Skills span robots (G1/H1/Go2/Spot) and methods (Shadowing / Unitree re-host / deterministic overlay); 10 skills + 4 BATON sequences ship today. Read this to add a new skill or build a new BATON demo.
- [docs/developer/spot-residual-rl.md](docs/developer/spot-residual-rl.md) — OmniSim-native RL pipeline (residual PPO on a model-based gait), Spot is the first robot
- [docs/developer/g1-single-source-of-truth.md](docs/developer/g1-single-source-of-truth.md) — **VERIFIED: the G1 RL trainer and the OmniSim Newton deploy run the SAME physics.** Both derive their physical model from one source — [`projects/policies/research/backends/g1_physics.json`](projects/policies/research/backends/g1_physics.json) + [`g1_physics_spec.py`](projects/policies/research/backends/g1_physics_spec.py) + the prim URDF (geometry/limits read live). Proven three ways: a compiled-`MjModel` field diff (trainer vs the *literal* extracted deploy runtime → 0 real-physics gaps), a GPU golden-trajectory test, and a **live H100 training run whose persisted `physics_config.json` byte-matches `SPEC.newton_env()`** on every parameter (solver, substeps, ke/kd, friction, joint clamp, inertia, seed pose, prim-URDF sha). Enforced in CI ([`tests/test_g1_physics_spec_conformance.py`](tests/test_g1_physics_spec_conformance.py)); golden harness [`projects/policies/research/training/g1_golden_parity.py`](projects/policies/research/training/g1_golden_parity.py). The rule: never re-declare a physics constant on either side — import the spec.
- [docs/developer/g1-ghost-fidelity-journey.md](docs/developer/g1-ghost-fidelity-journey.md) — **"make G1 walk ≥80% like the ghost" — the full honest journal.** A numerical per-joint similarity metric (`--eval-ghost-similarity`); the human ghost is a ~67% *physical* wall (a balancing biped must deviate from a kinematic reference); a **feasible** ghost (`--build-achieved` + `--gait-style achieved`) lifts the **shape** match to **84–88% over a 3 s window** — but the policy **topples ~7 s** and the deploy falls sooner (durability is the open **trainer↔deploy gap**, not similarity). Read before claiming a working G1 walk.
- [docs/developer/train-deploy-gap.md](docs/developer/train-deploy-gap.md) — **the train→deploy gap as TWO gaps** (pipeline-parity vs durability) + the enumerated, re-verified divergence table (COM default-off, finite-diff qd, launch-IC, contact) + **Unitree's proven obs/action/reward deploy recipe** as the durability answer. Read when working on making any RL policy deploy durably.
- [docs/developer/closed-loop-chaos-diagnostic.md](docs/developer/closed-loop-chaos-diagnostic.md) — **RUN THIS when a policy behaves differently in the trainer than in deploy and you don't know why.** A hardened BUG-vs-CHAOS classifier (`projects/policies/research/training/closed_loop_parity_compare.py`): run the same policy both sides, and the *shape* of the divergence says whether it's a real pipeline/obs `[BUG]`, intrinsic physical `[CHAOS]` (a free biped is unstable — divergence is unavoidable, fix with robustness not parity), or a clean `[MATCH]`. Don't theorise about the physics before classifying the divergence.
- [scripts/harness/README.md](scripts/harness/README.md) — reference for the HTTP harness covered in Section 5
- [docs/developer/omniworld-user-guide.md](docs/developer/omniworld-user-guide.md) — procedural world generation
- [CONTRIBUTING.md](CONTRIBUTING.md) — contribution process

If a doc and the code disagree, the code wins — and update the doc in the same change.
