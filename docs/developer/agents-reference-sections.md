# AGENTS.md sections 0–11 — full reference

> **Verbatim reference moved out of `AGENTS.md` on 2026-09-02.** Every passage below is the
> original text, word for word (markers, dates, commit hashes and self-correction history included);
> `AGENTS.md` now carries a short summary of each item and links here. Nothing was paraphrased.


This is the complete text of every §0–§11 passage that `AGENTS.md` now summarises, in the file's original order and with its original sub-headings. (§5's endpoint cheatsheet table is in [harness-endpoint-reference.md](../../docs/developer/harness-endpoint-reference.md); §0's first-moves table and hard-won rules are in [agents-first-moves.md](../../docs/developer/agents-first-moves.md) and [agents-hard-won-rules.md](../../docs/developer/agents-hard-won-rules.md).)

## Section 0 VERDICT branch

Reports the truth about *this clone right now*: OmniSim binary path, **whether there is a physics runtime at all**, engine↔libController ABI compatibility (the IPC-nonce gate, commit `6eea9d76` — a controller lib older than the engine silently hangs *every* controller at zero ticks while a headless run still prints PASS), port status (`6789` harness, `6790` supervisor, `6791` capture), worlds present, recent commits. Don't guess at the state — check it. `--json` for machine-readable.

**Read the VERDICT line and branch on it before anything else in this file.** Two answers mean STOP:

- `binary NOT FOUND` — this clone is not built. `msys64/` is gitignored, so a fresh `git clone` has no engine and every row of the table below assumes one. Build first (§2): `build_omni.bat` on Windows, `bash scripts/install/linux_bootstrap.sh` on Linux, then `make -C src/omnisim bundle-newton-runtime`.
- `physics FAIL` — the Newton runtime is absent. Newton is the ONLY backend, so this is not a degraded mode: nothing falls, nothing collides, no grasp holds, and every run still exits 0. `doctor` prints the fix for your platform.

`doctor` exits non-zero on either, so an agent branching on `$?` gets the right answer. It did not until 2026-08-28 — it reported the fault and exited 0.

## Section 1 Environment check

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
python -m omnisim --help
```

If the binary is missing, build first (Section 2). If it exists, jump to Section 3 to launch a demo.

`OMNISIM_HOME` (canonical) should point at the absolute path of this checkout. `build_omni.bat` and `launch.bat` derive it from their own location, so on Windows you usually do not need to export anything manually.

**`WEBOTS_HOME` is retired from the engine↔controller RUNTIME contract, but NOT from the build, and not from every last runtime read — the distinction is exact and the bullets below are the whole of it:**

- ⚠️ **The `WEBOTS_*` variables of the engine↔controller contract are GONE, and this bullet used to say the opposite.** The engine↔controller contract is now written and read under the `OMNISIM_*` name only, on both sides at once: the engine writes `OMNISIM_ROBOT_NAME` / `OMNISIM_INSTANCE_PATH` (and actively `remove()`s the legacy twins from the child environment so a stale shell value cannot shadow them), the launcher exports `OMNISIM_CONTROLLER_URL` / `OMNISIM_STD*_REDIRECT`, libController writes `OMNISIM_PIPE_IN`, and **libController no longer reads a single `WEBOTS_*` fallback** — `OMNISIM_HOME`, `OMNISIM_CONTROLLER_URL`, `OMNISIM_ROBOT_NAME`, `OMNISIM_INSTANCE_PATH`, `OMNISIM_STDOUT_REDIRECT`, `OMNISIM_STDERR_REDIRECT`, `OMNISIM_LOG_PATH`, `OMNISIM_IPC_NONCE`. The engine likewise reads `OMNISIM_LIBRARY_PATH` only. **This is a real break for anyone whose shell exports the old names** — most plausibly `WEBOTS_CONTROLLER_URL` for an extern controller, or `WEBOTS_HOME`. It is deliberately **not silent**: legacy-only is detected and reported once, naming the new variable (`robot.c getenv_omnisim()`, `system.c`, `omnisim_controller.c get_omnisim_home()`, `wb.py _omnisim_home()`, and an engine `warn()` for `WEBOTS_LIBRARY_PATH`), so the symptom is a named message rather than a 50-second wait ending in "Giving up".
- **Note the two things that did NOT change and must not be "finished":** the on-disk rendezvous names (`webots-<tmpId>` tmp folder, `\\.\pipe\webots-…`) are a wire contract the engine and libController each reconstruct, and the exported `wb_*` / `wbu_*` / `Wb*` ABI (including `wbu_system_webots_instance_path`, declared in the public header `include/controller/c/omnisim/utils/system.h`) is frozen.
- ⚠️ **Do NOT generalise the bullet above into "no `WEBOTS_*` name is read anywhere at runtime" — that is measurably false.** Three engine-side reads survive outside the controller contract and are unaffected by the migration: `WEBOTS_EMPTY_PROJECT_PATH` ([`OmStandardPaths.cpp:231`](../../src/omnisim/core/OmStandardPaths.cpp#L231), used only when `OMNISIM_EMPTY_PROJECT_PATH` is unset), `WEBOTS_TMPDIR` ([`OmStandardPaths.cpp:280`](../../src/omnisim/core/OmStandardPaths.cpp#L280)), and `WEBOTS_DEBUG` (an `OMNISIM_DEBUG` alias in `OmMainWindow.cpp` / `OmView3D.cpp`). None of them reaches a controller.
- **One shipped runtime library still reads `WEBOTS_HOME`**: `qt_utils` ([`resources/projects/libraries/qt_utils/core/StandardPaths.cpp`](../../resources/projects/libraries/qt_utils/core/StandardPaths.cpp), `StandardPaths::getWebotsHomePath()`) resolves the env var at runtime to build the Qt plugin + icon search paths used by robot windows. It now prefers `OMNISIM_HOME` and falls back to `WEBOTS_HOME`, so `OMNISIM_HOME` alone is sufficient — but the `WEBOTS_HOME` read is still there for compatibility.
- **The build reads it widely.** The top-level [`Makefile`](../../Makefile) exports `WEBOTS_HOME` (and `WEBOTS_PATH`) as an alias of `OMNISIM_HOME`, and **16** Makefiles consume it (17 mention it; the top-level one exports rather than expands it) — not just `src/controller/{c,cpp,launcher}/Makefile`, but also `resources/Makefile.include`, `dependencies/Makefile.*`, `src/{wren,glad}/Makefile`, and the `resources/projects/**` library/plugin Makefiles. A top-level `make` therefore works with `OMNISIM_HOME` set alone (the alias is exported for you); a **standalone** `make` inside a controller/plugin dir does not go through the top-level Makefile, so it needs the env var itself — the shipped templates prefer `OMNISIM_HOME` and fall back to `WEBOTS_HOME` (§7).

### One-time per-clone: enable hooks

```bash
bash scripts/dev/setup_hooks.sh
```

This points `core.hooksPath` at the versioned `.githooks/` directory so that after every `git pull` / branch switch, `scripts/dev/clean_orphans.py` purges hollow robot/asset dirs left behind from upstream `git rm` (build artifacts under `*.exe`, `*.o`, `build/`, etc. — never tracked, so git can't remove them). Without this, clones drift: a robot deleted upstream stays as orphan build debris locally. Safe by construction — only dirs with zero tracked files **and** zero untracked-non-ignored files are removed, so a WIP robot you haven't committed yet is protected.

`setup_hooks.sh` also activates `.githooks/pre-push`, which acts as **local CI** for the Windows dev loop. ⚠️ The "in lieu of any hosted CI" framing this line used to carry is dead (corrected 2026-09-01): the upstream Webots suite is still disabled under `.github/workflows.disabled/`, but **ten** workflows are active in `.github/workflows/` — and one of them IS a build CI: **`linux-build.yml`** (since 2026-08-27) compiles the tree on Linux by driving `scripts/install/linux_bootstrap.sh` itself, on PRs and on engine-path pushes to `main`, so a green run is evidence about the documented user path. The other nine: `g1-spec-conformance.yml`, `update_sponsors.yml`, `licence-provenance.yml` (the ~10 s licensing/provenance gate, deliberately **unfiltered by path** because the failure mode is a new file in a directory nobody thought to list, and it runs on every push and PR), `dco.yml` (a Developer-Certificate-of-Origin sign-off check scoped to `pull_request` ONLY, never `push` — the DCO is about INBOUND contributions, so running it on push would just paint `main` red), `physics-runtime-check.yml` (guards the Newton runtime pins + `linux_bootstrap.sh`), `release.yml` (tag-triggered release/installer builds), `train-image.yml` (**not** lightweight: builds the CUDA GPU training image from `docker/Dockerfile.train` and pushes it to GHCR, on every change to that Dockerfile or the workflow, and on `workflow_dispatch`), `runtime-image.yml` (the runtime Docker image, same shape), and `publish-omnisim-mcp.yml` (`workflow_dispatch`-only package publish). The pre-push hook runs the smoke world set (`tests/smoke/smoke_worlds.json` — **7 declared, 3 carrying `skip: true` with a measured `skip_reason`, so 4 actually run**; 1–3 minutes warm — the FIRST run also builds the missing test controllers, which takes much longer) before every push and fails the push on a regression. Bypass for in-progress work with `OMNISIM_SKIP_PUSH_CHECK=1 git push`; pushes to `refs/heads/scratch/*` skip automatically. Run the same set manually any time with `make tests-smoke`.

---

## Section 2 Build

### Windows (preferred — uses MSYS2)

```bat
build_omni.bat
```

This wrapper sets `PATH` to include MSYS2's MinGW64 toolchain and runs `make` with `OMNISIM_HOME` (and `WEBOTS_HOME` as a legacy alias) auto-derived from the script's location. If MSYS2 is at a non-default location, set `MSYS64_HOME=D:\msys64` first.

First build: 5–15 minutes. Incremental: seconds to a few minutes.

After a fresh build, vendor the Newton physics runtime next to the binary (one-time, ~600 MB — **not optional**: Newton is the only physics backend, so without this the binary has no dynamics at all):

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
python -m omnisim build all
```

On **Linux** (supported as of v5.1) the scripted path is `bash scripts/install/linux_bootstrap.sh`; for Newton, pip the GPU wheels into the **system** `python3` (not a venv), and run headless under Xvfb — details + runtime env vars in [docs/developer/quickstart.md → Linux quickstart](../../docs/developer/quickstart.md#linux-quickstart-ubuntu). macOS remains untested.

### Build subsystems

```bash
python -m omnisim build core            # just the simulator core
python -m omnisim build renderer        # ⚠️ DEAD since the WREN deletion -- see below
python -m omnisim build gui             # just the desktop GUI layer
python -m omnisim build controller-libs # just the controller APIs
```

⚠️ **`build renderer` builds nothing, and as of 2026-08-24 it says so instead of pretending.** Its recipe was `make -C src/wren` — deleted with WREN on 2026-08-23 (`976b9449d`). The target name survives in the top-level [`Makefile`](../../Makefile)'s `.PHONY` and goal-filter lists with **no recipe behind it**, so `make renderer` prints `Nothing to be done for 'renderer'` and exits **0** — a build command reporting success while building nothing, which is exactly the failure class this file keeps warning about. The subcommand now refuses with an explanation and a non-zero exit rather than routing to that target. There is no separate renderer subsystem any more — the wgpu backend is compiled *into* the engine from `src/omnisim/render/` (plus `src/omnisim/nodes/OmWgpuSceneRenderer.cpp` for camera-family devices), so `build core` / `build gui` is what rebuilds the renderer.

### Iterating on the engine (the C++ edit loop)

`make release` from `src/omnisim` is the loop. It is **automatically cached and fast — you do not need to pass anything** (commit `c858d0d9b`, measured on machine `9722d23d12a3`: touch one TU → working binary, **5672 ms → 2601 ms**):

- **ccache is wired into the standalone sub-make**, not just the top level. Before this it was not, and the repo's cache had three cacheable calls in its entire lifetime — a hit is 2330 → 135 ms, so branch switches, rebases and A/Bs are near-free. `USE_CCACHE=0` opts out. It self-probes, so a missing or broken ccache silently falls back to a plain compiler rather than failing the build.
- **`OMNISIM_LINKER` defaults to `auto`** — lld when installed (`pacman -S mingw-w64-x86_64-lld`), GNU ld otherwise. Once the compile is cached, the link *is* the loop: 3738 → 2830 ms. ⚠️ Two boxes can now link with different linkers and produce different binary sha256, which `env_fingerprint.py` records — **pin `OMNISIM_LINKER=bfd` (or `=lld`) for anything whose binary identity must match across machines.** `make linker-info` reports what resolved, plus the ccache state.
- ⚠️ **HISTORICAL TRAP, NOW FIXED — do not re-learn it.** A plain `make release` used to **delete `omnisim-bin.exe`** on any box where wgpu-native had ever been set up: the objects on disk carry `-DWB_WGPU_NATIVE_AVAILABLE`, nothing recorded that, and a build without `WGPU_NATIVE_HOME` re-linked them without `-lwgpu_native` → undefined references → make removes its own target. `WGPU_NATIVE_HOME` is now auto-discovered from the one place `scripts/dev/setup_wgpu_native.sh` installs to, so **you no longer need to pass `OMNISIM_WITH_VULKAN=ON WGPU_NATIVE_HOME=...` by hand** (older notes and recipes still say to; they are stale). ✅ **And since 2026-08-29 a build with NO wgpu-native is REFUSED, not silently produced** (public issue #7): an explicit empty `WGPU_NATIVE_HOME=`, `OMNISIM_WITH_VULKAN=OFF`, or a clone where `setup_wgpu_native.sh` was never run all used to build *green* and ship a binary with **no renderer at all** (the `WB_WGPU_NATIVE_AVAILABLE=0` branch) — physics and controllers ran, nothing drew, and the hole surfaced later as an empty screenshot. `src/omnisim/Makefile` now `$(error)`s at parse time with the setup command in the message; the compute-only binary is still available **by name** — `make release OMNISIM_RENDERERLESS=ON` — which also defines `-DOMNISIM_RENDERERLESS` so the binary logs `THIS BINARY WAS BUILT WITHOUT A RENDERER` on its first render request instead of sending you to debug a wgpu-native install it would never load. `clean` / `linker-info` / `bundle-newton-runtime` are exempt. Verified by dry run on all five configurations (opted-out → refused, exit 2; opted-out + flag → warns and proceeds; auto-discovered → links `-lwgpu_native`; `clean` while opted-out → no refusal; `OMNISIM_WITH_VULKAN=OFF` → refused, naming the flag).
- A stable precompiled header exists (`OMNISIM_USE_PCH=1`, QtCore + stdlib; 2578 → 1264 ms on a small TU) but is **off by default on purpose**: once ccache is on it is a wash, winning only on genuine cache misses. Turn it on when writing a lot of new source.

### Full prerequisites

If a fresh build fails on missing dependencies (Qt6, GLM, stb), follow the step-by-step setup in [docs/developer/quickstart.md](../../docs/developer/quickstart.md) sections 1–4. That doc covers `pacman` package lists, the `include/qt/` mirror layout the build expects, and the GLM submodule pinning.

---

## Section 3 Launch a demo

There are two launch modes that matter to an agent:

### 3a. Windowed (visual — for human-in-the-loop debugging or screenshot capture)

Windows — no-args opens the **OmniSim demo launcher**: the "hello world" house scene (the Beauty Bench lot, wgpu-rendered (the engine default since the 2026-08-19 flip)) with a floating orb robot whose Robot Window is a side-panel gallery of every demo in the repo, grouped by category (starter; chat — arms, mobile bases, quadruped, aerial; flagship; OmniLink agents; showcase; physics; generated worlds). Right-click the orb → *Show Robot Window* → click *Launch* on any card to switch worlds. Catalogue: [`projects/samples/demos/controllers/omnilink_launcher/demos.json`](../../projects/samples/demos/controllers/omnilink_launcher/demos.json). Full index: [`DEMOS.md`](../../DEMOS.md).

```bat
launch.bat
```

Linux / macOS / cross-platform:

```bash
python -m omnisim run-world projects/samples/demos/worlds/omnilink_launcher.omniworld
```

To skip the launcher and open a specific world, pass it as the first argument: `launch.bat path\to\world.omniworld`. `launch.bat` accepts any extra `omnisim-bin.exe` flags after that, e.g. `launch.bat path\to\world.omniworld --mode=fast --no-rendering`.

### 3b. Headless (no window, exits cleanly — preferred for autonomous agent runs)

```bash
python -m omnisim run-headless projects/samples/demos/worlds/showcase/warehouse_husky.omniworld
```

What this does:

- starts the simulator with `--batch --mode=fast --no-rendering --minimize --stdout --stderr`
- monitors `omnisim_log.txt` for errors as the world runs
- **with no `--duration`: exits as soon as Newton finalises the world** (a load check; 10 s ceiling), announcing that on stdout so it is never silent
- with an explicit `--duration N`: runs N seconds, honoured verbatim -- use this whenever the run must OBSERVE the simulation
- returns a non-zero exit code if the load failed or `--fail-on-warning` was set and a warning fired

This is the supported headless contract — use it for "did the world load and step without crashing" checks. **That is ALL a bare PASS means.** It is a log verdict, not a physics verdict: the run is judged on ERROR/WARNING lines, controller-start failures and the engine's own step counter, and nothing in the engine log records a body leaving the world. Measured on AgentBench task `C2_fall_through_floor`: a world whose floor Solid has **no `boundingObject`** — so its dynamic body free-falls for ever through a hologram floor, reaching **z = −69 km** — printed `0 errors, 0 warnings … PASS`, *byte-identically to the fixed world*. A bare PASS cannot certify a physics fix.

**When the point of the run is to certify physical behaviour, add `--fail-on-runaway`:**

```bash
python -m omnisim run-headless <world> --duration 10 --fail-on-runaway
```

It injects the `runaway_watchdog` supervisor into a **sibling copy** of the world (`.omnisim_runaway_<stem>.wbt`, deleted at exit; the world itself is never touched), samples every top-level dynamic body's pose, and FAILS — naming the body, its exit z and its exit vertical speed — when one has left the world: `|z|` past `--runaway-z-limit` (default 1000 m), or below the lowest static collision surface *while still accelerating downward*. A body that fell and **landed**, one descending at a steady rate, and one still legally mid-air at exit all pass; on the same C2 pair it FAILs the broken world and PASSes both independent fixes. It is opt-in because it adds a Robot to a world copy and needs a run long enough to collect samples (≥8 s; the summary line prints the sample count and the roster it tracked, so partial coverage is visible rather than assumed).

> ✅ **COLD-FIRST-LOAD TRAP — RESOLVED (verified 2026-07-05).** There used to be a trap
> here: on a cold first load the Newton/MuJoCo articulation under-tracked position targets
> (~1 cm), so precise grasps failed cold but worked after a world reload — and since every
> headless run is a cold load, it produced false "the physics can't do this" conclusions.
> **That bug is fixed** (root cause `eb86f888`: the Newton solver choice now survives the
> multi-build load, so a cold load builds MuJoCo instead of falling back to what was then
> the XPBD default — XPBD was removed outright on 2026-08-07; plus the
> finalize-time solver re-assert). Verified: cold and warm loads settle **bit-identical**
> (bare-arm probe to 6 decimals; a full arm+gripper grasp identical every phase). Consequently
> the controllers' `warmup_reload` helper is now a **no-op by default** — no startup reload,
> no `--cold`/warm split to worry about. Full write-up + how to re-measure:
> [docs/developer/real-grasp-and-the-cold-first-load-trap.md](../../docs/developer/real-grasp-and-the-cold-first-load-trap.md).
> (Safety valve: `OMNISIM_FORCE_WARMUP=1` re-enables the old reload if a regression ever
> resurfaces.)

### 3c. Choose a demo

> **Looking for the "type-talk to a robot" experience?** Every URDF
> robot in the repo has a chat-driven demo: open the world, right-click
> the robot → *Show Robot Window*, type a prompt, the robot moves.
> Works offline (regex intent router) or against OmniLink (set
> `OMNI_KEY`). **One-page agent gallery (all chat demos + specialist
> agents + real-robot port):**
> [docs/showcase/agents.html](../../docs/showcase/agents.html).
> Full beginner guide:
> [docs/guide/omnilink-chat-demos.md](../../docs/guide/omnilink-chat-demos.md).
> Worlds: `projects/samples/demos/worlds/omnilink_*.wbt`.
> Add your own robot in ~50 lines: [docs/guide/omnilink-add-your-robot.md](../../docs/guide/omnilink-add-your-robot.md).
> Same agent driving a real robot: [docs/guide/omnilink-sim-to-real.md](../../docs/guide/omnilink-sim-to-real.md).
>
> **OmniLink artefact map** (everything ships on top of the bridge HTTP surface):
> - **The Omni Key** (the one thing this repo can't produce for itself): `python -m omnisim key` — prints where you stand plus the exact shell line, `--open` opens the page, `--check` asks the platform whether the key works. Full reference — what it unlocks, BYOK, and calling `/api/chat` from your own code — is [docs/guide/omnilink-key-and-api.md](../../docs/guide/omnilink-key-and-api.md). **The OmniLink website no longer has a documentation site**; that page is where its platform reference went.
> - **The provider key is a SECOND step, and the user will hit it as a `402 BYOK_REQUIRED`.** The Omni Key identifies the account; a model-provider key pays for the tokens (OmniLink takes 0% markup and never resells them). `python -m omnisim byok` shows what is connected and what is missing, `--providers` explains the options, and `--add google` connects one with hidden input. **If a user asks you to set up OmniLink, do this for them** — it is an ordinary authenticated API call, not a browser-only step, and google is the only provider with a free tier and no card. Never write a provider key into a file in this repo; the command sends it straight to the platform, which stores it encrypted.
> - **Specialist agents** (Roomba): [`agents/templates/`](../../agents/templates/).
> - **Real-robot bridge starter kit** (no Webots, no OmniSim): [`agents/bridges/`](../../agents/bridges/).
> - **`omnisim-bridges` pip-installable package** (the primitives lifted out): [`packages/omnisim-bridges/`](../../packages/omnisim-bridges/).
> - **Voice I/O**: Mic button + Chirp3-HD TTS, controlled by `OMNILINK_VOICE_OUT`.
> - **Per-turn usage telemetry + cross-session short-term memory**: `OMNILINK_USAGE`, `OMNILINK_MEMORY`, `OMNILINK_PROFILE_SYNC` env vars in the chat-demos guide.
> - **Benchmark suite**: [`tests/benchmarks/omnilink_tasks/`](../../tests/benchmarks/omnilink_tasks/) — **23 graded tasks**: **16** in the core suite ([`ol_suite.py`](../../tests/benchmarks/omnilink_tasks/ol_suite.py) — capability / tool-selection / delegation / honesty / safety, against four Huskies) plus **7** in the hard suite ([`ol_hard_suite.py`](../../tests/benchmarks/omnilink_tasks/ol_hard_suite.py)), and the legacy single-robot lane (`tasks.py`, `mobile_drive_1m`: Husky displacement ≥ 0.9 m). Headless; every verdict comes from measured robot pose + the recorded tool-call trace, never from what the agent says about itself. ⚠️ **Still a scaffold, not a published agent score** — `results/` is gitignored, so **no result row ships in this tree**; the numbers exist only on whoever ran it. `python tests/benchmarks/omnilink_tasks/matrix.py --list` to see the contract without running anything, `matrix.py --dry-run --dry-run-script mixed` for the offline harness check, `run.py` for the legacy lane.
> - **Shipping status with commit hashes**: `docs/developer/omnilink-roadmap.md` (internal — excluded from the public snapshot, so not linked here).

Recommended starter demos for an agent, ordered roughly from simplest to most ambitious:

| World | Path | Why an agent might pick this |
|-------|------|------------------------------|
| **OmniLink chat demos** *(beginner-friendly)* | `projects/samples/demos/worlds/chat/omnilink_<robot>.omniworld` (×15, all `.omniworld` — this row wrote `.wbt` until 2026-09-01, against the dual-read/single-write rule above — and all 15 ship publicly: [`scripts/release/publish_deny.txt`](../../scripts/release/publish_deny.txt) holds no chat entry at all since `f29cdae88` removed the stale `omnilink_panda` rule, verified by grep 2026-09-01) | One world per URDF robot. Right-click the robot → *Show Robot Window* → a chat-style side menu opens. Type `home`, `wave hello`, `drive forward 1 m`, etc., and the robot moves. Works offline (regex intent router) or against OmniLink (`OMNI_KEY` env var → routes through Gemini/GPT/Grok/Claude). Full beginner walkthrough: [docs/guide/omnilink-chat-demos.md](../../docs/guide/omnilink-chat-demos.md). |
| **Warehouse Husky** *(default)* | `projects/samples/demos/worlds/showcase/warehouse_husky.omniworld` | The onboarding demo. A supervisor-enabled Husky (URDFRobot, `husky_random` controller) random-walks a 30 × 18 m warehouse with reactive collision recovery. Good showcase of the URDF importer, supervisor APIs, motor torque/sensor pipeline, and the camera follow (F key). |
| Husky maze | `projects/samples/demos/worlds/flagship/husky_maze.omniworld` | Single Husky in a maze. Classic navigation/SLAM testbed. **Drives via the `husky_omnilink_bridge` controller — see [`agents/production/husky_maze/`](../../agents/production/husky_maze/) for the agent + bridge contract, and the three sibling worlds (`_unknown`, `_corners`, `_visual`) for progressively harder briefs.** |
| Husky maze (unknown) | `projects/samples/demos/worlds/flagship/husky_maze_unknown.omniworld` | Map-gated maze — the Husky must lidar wall-follow to the goal. A reactive-navigation testbed; swap in your own control law in the maze controller. |
| Husky swarm | `projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld` | 8 Huskies driving in parallel under Newton (each runs the `drive_forward` controller). Good multi-robot physics/throughput showcase. (For an OmniLink-driven swarm, see `projects/samples/demos/worlds/flagship/omnilink_husky_swarm.omniworld`.) |
| Husky fleet arena | `projects/samples/demos/worlds/showcase/husky_fleet_arena.omniworld` | Larger multi-Husky world. Useful for stress-testing rendering/physics changes. |
| Generated Mars | `distribution/generated_worlds/mars.wbt` | Procedurally generated planetary terrain with a Husky fleet. Regenerate with `omniworld` (Section 5). |

Full list: `ls projects/samples/demos/worlds/` and `ls distribution/generated_worlds/`.

---

## Section 3d OmniLink co-located agents

OmniSim ships agent definitions in `agents/production/` that are versioned alongside the worlds + controllers they drive. Same productized layout as OmniLink's first-party agents (in the separate OmniLink repo): `profile.json`, `prompts/`, `knowledge/`, `long_term_memory/`, auto-discovered `tools/`, a thin runner.

The reference agent is **[`agents/production/husky_maze/`](../../agents/production/husky_maze/)**. It drives the Clearpath Husky across five maze worlds with progressively harder briefs:

| world | what's hard about it | who can solve it |
|---|---|---|
| `husky_maze.omniworld` | trivial: drive to (10, 0) | script or agent |
| `husky_maze_unknown.omniworld` | map gated, lidar wall-follow needed | script or agent |
| `husky_maze_corners.omniworld` | brief = "visit four corners and return" | **agent only** — script can't read the brief |
| `husky_maze_visual.omniworld` | brief = "find the RED cylinder via camera" | **agent only, structurally** — pixels need an LLM |
| `husky_maze_blind.omniworld` | map AND lidar gated — navigate from `scan_surroundings` symbolic perception tags (sidecar CV pipeline), not pixels | **agent only** — the perception-as-tool architecture demo |

Read [`agents/production/husky_maze/docs/OVERVIEW.md`](../../agents/production/husky_maze/docs/OVERVIEW.md) first for a balanced description of the architecture, what works, what doesn't, and the cost shape. Then [`docs/why-an-agent.md`](../../agents/production/husky_maze/docs/why-an-agent.md) for the discriminator argument.

To run the standalone solver (no OmniLink involvement):
```bash
launch.bat projects\samples\demos\worlds\flagship\husky_maze.omniworld
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

The launcher's registry is auto-discovered from
`agents/production/*/omnilink.json`
([`omnisim_run_agent.py:131`](../../scripts/dev/omnisim_run_agent.py#L131)), so ⚠️ **it
covers only the agents that ship a manifest — 6 of the 8 today** (recounted
2026-09-01): `axis`, `husky_maze`, `husky_swarm`, `mission_captain`,
`omnitug500_warehouse`, `smart_house`.
`skill_factory` ships a runner (`skill_factory_agent.py`) and no manifest, so
`--list` cannot see it; `omniarm6_persistent` has no runner. (The
`omniarm6_digital_twin` empty directory this line used to name is gone
entirely.) Registering an agent means adding
its `omnilink.json`. New agents should build on
[`agents/production/_lib/`](../../agents/production/_lib/README.md): the
`OmniLinkAgentRunner` class hoists the ~250 lines of HTTP server +
profile push + UsageMeter boilerplate that every existing runner
duplicates today, and `OmniSimBridgeServer` + the `@action` decorator
do the same for the supervisor controller side.

---

## Section 3e Multiple instances in parallel

OmniSim is multi-instance-safe: you can run **K `omnisim-bin` processes side by side on one host**. This is the supported shape for batch world validation, fleet/throughput benchmarks, agent-vs-agent matches, and per-PR smoke farms. Don't design these as "one mega-world with K robots" unless that's the actual scenario — split across processes and you get parallel physics/rendering for free.

### What the simulator handles for you

- **Per-instance TCP port.** Each `omnisim-bin` opens its TCP server (extern controllers, robot windows, web streaming) on the next free port in `[1234, 1294]` — the default `1234` is set in the `int port = 1234;  // default value` line in [src/omnisim/gui/OmGuiApplication.cpp](../../src/omnisim/gui/OmGuiApplication.cpp), and the next-free-port auto-scan retry loop is in [src/omnisim/gui/OmTcpServer.cpp](../../src/omnisim/gui/OmTcpServer.cpp) (`OmTcpServer::start`, retrying up to `PORT_SCAN_SPAN = 60`). Pass `--port=<N>` only when you need a *stable* port for an extern controller; otherwise leave it default and instances coexist. ⚠️ **The span is 60, not the 10 it was, and the difference is not headroom you can spend on concurrency.** It was widened in `66f0c378` because eleven slots were exhausted *inside a single benchmark session*: a batch runner that KILLS engines on timeout leaves them holding their ports, so the range has to absorb concurrent simulators **plus the accumulated residue of killed ones**. Exhausting it now logs the range, the socket error and the likely cause instead of dying mute — but the fix is to let engines exit, not to raise K. In practice the real ceiling on K is vCPU count, well below 60.
- **Per-instance tmp / IPC dir.** The tmp-path resolver is salted with the chosen TCP port, so controller IPC sockets and per-instance state are isolated automatically.
- **Per-instance log file (only if you ask for it).** By **default** every instance writes to the *shared* install-root file `<OMNISIM_HOME>/omnisim_log.txt` (the `OmLog::initFileLog(omnisimDirPath + "/omnisim_log.txt")` call in [src/omnisim/gui/main.cpp](../../src/omnisim/gui/main.cpp)) — "most-recent run wins", useful for single-instance work, useless when K>1 children clobber each other. Per-instance isolation only happens when you set `OMNISIM_LOG_PATH` per child ([src/omnisim/core/OmLog.cpp](../../src/omnisim/core/OmLog.cpp) reads it as an override (`qEnvironmentVariable("OMNISIM_LOG_PATH")`)) — see the next section.

### What you must do per child to keep parallel runs clean

- **Set `OMNISIM_LOG_PATH=<unique-per-child path>`** before spawning each `omnisim-bin`. Without it, parallel children all also write to `OMNISIM_HOME/omnisim_log.txt` and the last one wins — you can't tail or grep it after the fact.
- **Don't pin `--port` unless you mean it.** Two children both pinned to `1234` will fail to bind. Leave it default and the simulator will pick the next free slot.
- **Give every child a stdout, and know that the engine now KEEPS it (Windows, fixed 2026-08-29).** `omnisim-bin.exe` is a GUI-subsystem binary; until this fix, any stdout that was not a pipe (the null device, a file) made it `AttachConsole()` to its launcher's console — a console it did not own, shared with every other engine and controller that launcher's console tree had spawned — and the file you handed it received **nothing**. Worse, an engine started *against one already running* could find its fd 1 dead by the time the embedded interpreter first wrote to it: warp's greeting raised `[Errno 9] Bad file descriptor` out of `newton.ModelBuilder()`, the FFI smoke read that as a broken runtime, **FATAL, exit 1** — measured 3 of 7 rounds with `python scripts/dev/launch_race_stress.py --concurrent 2 --stagger 12` (machine `9722d23d12a3`, pre-fix binary), 0 of 8 with a pipe. That is the mechanism behind the "roughly one launch in three" startup race (public issue #3); the Qt teardown marks in its signature are just what any `exit(1)` during startup prints. The engine now attaches only when it was given **no** stdout at all (launched from a console without redirection, the case the attach was written for), and the interpreter's stdio probe is a real `os.fstat()` rather than a zero-length write that could not fail. The `[main] stdio:` INFO line at the top of every log says which branch ran.

### Worked example (the canonical pattern)

The bench harness spawns K parallel headless `omnisim-bin` children, each with its own log path, ports auto-allocated, results aggregated:

```bash
python tests/benchmarks/optim_bench.py multi-instance --sizes 4 --steps 600
```

The launch shape is in [`bench_multi_instance`](../../tests/benchmarks/optim_bench.py) (`ThreadPoolExecutor` of K children, per-child `OMNISIM_LOG_PATH`, port auto-scan). For the optimisation history and what does/doesn't scale linearly across K, see [docs/developer/multi-instance-optimization-plan.md](../../docs/developer/multi-instance-optimization-plan.md).

### What does NOT multiplex out of the box today

- The HTTP **validation harness** (§5) needs a *pair* of free ports: the HTTP port (`--port`, default `6789`) and the supervisor IPC port the injected supervisor controller binds inside the OmniSim subprocess (`--supervisor-port`, defaults to `--port + 1`, i.e. `6790`). Two harnesses with the same supervisor port will conflict, so for a parallel session pass both flags: `python -m omnisim harness --port 6889 --supervisor-port 6890`. **Or stop picking numbers and pass `--auto-port`**: it scans upward for a free `(port, port+1)` pair, binds there, and prints the pair it chose to **stderr** so the caller can discover the real listening address. The `python -m omnisim harness` wrapper **does** forward it (since `12dd24eb9`), so `python -m omnisim harness --auto-port` works; earlier revisions of this line said it did not and sent you to invoke the script directly, which is no longer necessary. If you forget and try to start a second harness on the defaults, the new instance now self-detects the collision, prints which world the existing harness is on, and shows two copy-pasteable options (reuse via `POST /world/load`, or start parallel) instead of crashing on `EADDRINUSE`. The capture sister-service on `:6791`/`:6792` lives on different ports specifically so it can coexist with the harness.
- The desktop **GUI** is single-window per process. Multi-instance is a headless / `--batch --mode=fast --no-rendering` story, not a GUI story. (`launch.bat` opens windowed; for parallel runs use `omnisim-bin --batch --mode=fast --no-rendering --minimize` directly, or call `python -m omnisim run-headless` from each worker — but note `run-headless` itself reads the shared log file, so for true parallelism go raw `omnisim-bin` with `OMNISIM_LOG_PATH` per child.)

---

## Section 4 Driving a robot over HTTP

Some demos expose an HTTP control surface so an external agent can drive the robot without writing an OmniSim controller. The reference implementation is the **OmniLink mobile bridge** at [projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py](../../projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py), which drives the Clearpath Husky in the [omnilink_husky chat demo](../../projects/samples/demos/worlds/chat/omnilink_husky.omniworld).

To use it:

1. Launch a world whose robot runs the `omnilink_mobile_bridge` controller — e.g. `projects/samples/demos/worlds/chat/omnilink_husky.omniworld` (its `controllerArgs` are `["--robot" "husky" "--port" "8765"]`).
2. Launch the world (windowed or headless, your choice).
3. Hit the HTTP server the controller starts on `127.0.0.1:8765` (the Axis-normalized surface):

   ```
   POST /get_robot_state      # current pose, wheel state, fault, last tick
   POST /list_robots          # [{id, model, capabilities}]
   POST /set_velocity         # {v: <m/s>, w: <rad/s>}
   POST /drive_forward        # {distance: <m>}
   POST /stop_robot
   ```

This is the pattern to copy when you need to expose any other robot to an external agent. The siblings follow the same shape: arms use [`omnilink_arm_bridge`](../../projects/samples/demos/controllers/omnilink_arm_bridge/) (generic 6-DOF arm + damped-least-squares IK; pick the arm with `--robot <id>` from the registry in `_arm_configs.py`), quadrupeds `omnilink_quadruped_bridge`, drones `mavic_omnilink_bridge`. The full contract is in [PROTOCOL.md](../../PROTOCOL.md).

---

## Section 5 Validation harness

The **agent-facing validation harness** is a long-running HTTP service that wraps a headless OmniSim subprocess and injects a generic supervisor controller into whatever world it loads. It exists so an agent can author and iterate on worlds in a tight loop — write `.wbt`, load, screenshot, inspect scene tree, fix, hot-reload — without ever launching the desktop GUI. **This is the preferred authoring path for any world-building or world-debugging task.** `run-headless` (Section 8) is still the right tool for "does it load and step?" smoke checks; the harness is for the iteration loop on top of that.

### Starting it

```bash
# The PATH prefix is Windows-only (Qt6 DLLs for the OmniSim subprocess);
# on Linux just set OMNISIM_HOME and run with your python3.
PATH="/path/to/msys64/mingw64/bin:$PATH" \
OMNISIM_HOME=$(pwd) \
python scripts/harness/omnisim_harness.py --port 6789
```

Two gotchas worth knowing up front:

1. **(Windows) `PATH` must include a complete msys2 mingw64 `bin`** (the directory with Qt6 DLLs etc.). The bundled `$OMNISIM_HOME/msys64/mingw64/bin/` typically only contains the build outputs, not the toolchain DLLs. If the harness's first `/world/load` returns the diagnostic `LAUNCHER_DLL_NOT_FOUND` (Windows exit code `0xC0000135`), this is the cause — fix the parent shell's `PATH` and restart.
2. **(All platforms) The Python interpreter that runs the harness needs `Pillow`** (`pip install Pillow`). Without it, `/world/render_stats` returns 503; the harness prints a startup hint when Pillow is missing.
   - **Windows**: the system Python (with PIL) is the right choice — **but note the interaction with gotcha 1**: once msys2's mingw64 `bin` is prepended to `PATH`, a bare `python` resolves to the msys2 python (no Pillow). Launch the harness with the full path to the Windows interpreter (e.g. `C:/Users/<you>/AppData/Local/Programs/Python/Python312/python.exe scripts/harness/omnisim_harness.py`).
   - **Linux**: `pip install Pillow` into whichever `python3` launches the harness. On Ubuntu 24.04+ (PEP 668) that means `pip install --break-system-packages Pillow` or running the harness from a venv — **the harness itself runs fine in a venv**. Don't conflate this with the Newton runtime rule above: it's the *engine's embedded interpreter* (the system `python3`) that must not be a venv; the harness is an ordinary Python process and can live wherever Pillow is installed.

You can also start it via the dev wrapper: `python -m omnisim harness`.

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

| Endpoint | Purpose |
|---|---|
| `GET /capabilities?probe_step=1` | ⭐ **Start every session here.** One call answers what you are talking to and what it will refuse: `physics` **read from the engine's own `.newton.json` verdict sidecar**, with `source` naming the provenance — `sidecar` / `engine_log` / `sidecar_stale` / `sidecar_unreadable` / `sidecar_absent` / `retired_selector_ignored`. ⚠️ **`backend` never says `"ode"` any more** (fixed 2026-08-08; it used to, which contradicted its own `detail` text on the field an agent actually branches on). The negative case is `backend: "unverified"` with `source: "sidecar_absent"`, and it means **"Newton did not finalize this world"** — most often a run too short to reach finalize (budget ≥15 s, ≥45 s on virtualised disks), otherwise a runtime that would not come up. It does **not** mean another engine drove the world; there is no other engine. A set `OMNISIM_FORCE_ODE` / `OMNISIM_LEGACY` / `OMNISIM_ALLOW_ODE_FALLBACK` reports `backend: "newton"` with `source: "retired_selector_ignored"` — the variable is warned about and ignored, so the run is Newton regardless. `limits.step_cost` + `recommended_max_steps_per_request` (a rolling median of the **measured** per-step cost on *this* world, so you size a step budget instead of discovering the 120 s RPC timeout by hitting it — `?probe_step=1` advances one step to measure it); `event_types` served **from the code** (the supervisor scans its own `emit()` call sites, so doc drift is impossible) with `suppressed` naming what a `--light` session will not produce; `endpoints` cross-checked against the request handler's own source; `not_supported`, every gap with a `reason` and a `workaround`; and the `diagnostic_codes` / `request_error_codes` enums. Needs no supervisor and no loaded world. |
| `POST /world/load {path, wait_s?, with_supervisor?, light?, tracking?}` | Load a `.wbt`. Returns structured diagnostics with codes like `WORLD_PARSE_SYNTAX_ERROR`, `PROTO_NAME_MISMATCH`, `TEXTURE_READ_FAILED`. **The set is OPEN — read it from `GET /capabilities` → `diagnostic_codes`, never from a count written here** (a hard-coded count has now been stale twice, 33 then 54; PROTOCOL.md §7.3 makes open-enum the contract, so this line stopped carrying a numeral on 2026-09-01; it was 56 that day): **40** distinct codes across the classifier's 48-entry rule table ([`scripts/harness/diagnostic_codes.py`](../../scripts/harness/diagnostic_codes.py)), + **9** `CUDA_CODES`, + `NEWTON_ZERO_DYNAMIC_BODIES` (synthesized from a matched rule rather than owning one), + `UNKNOWN`, + **5** the harness synthesizes when the engine never got far enough to log (`LAUNCHER_DLL_NOT_FOUND`, `SIMULATOR_EXITED_NONZERO`, `SUPERVISOR_BIND_STALLED`, `SUPERVISOR_BIND_CEILING`, `WORLD_DIR_NOT_WRITABLE`). `with_supervisor` defaults to true. |
| `POST /world/sync {path?, settle_steps?, reset_physics?, wait_s?, light?}` | ⭐ **Default after any authored edit.** Compares the file with the exact source snapshot that produced the running world. If and only if all semantic changes are numeric `translation`/`rotation` values on existing root-level DEF nodes, validates the whole batch, applies it live, resets moved bodies, settles once, and returns measured positions (`mode: "live_pose"`). Comments/format-only edits return `mode: "no_change"`. Geometry, collision, mass, material, controller, nested-node, add/remove, malformed, light-mode, or ambiguous edits automatically use the ordinary engine reload (`mode: "full_reload"`). Do not pre-classify the edit yourself. ⚠️ **Two more `mode` values exist and this row used to omit them — branch on all five:** `rejected` (HTTP **422** — no path and no loaded world, world not found, unreadable file, or a bad `settle_steps`) and `busy` (HTTP **409** — another load or sync already in flight; retry). Status is 200 when `ok`, else 409 for `busy`, else 422 (the `409 if result.get("mode") == "busy" else 422` branch in [`omnisim_harness.py`](../../scripts/harness/omnisim_harness.py)). |
| `POST /world/load {"light": true}` | ⭐ **The step-cost lever — and it is NOT just a multi-robot lever, which is how this row used to read.** `light=true` injects the supervisor with `--light`, dropping the per-step contact / joint-limit / grip trackers that walk the whole scene graph every basic step. Measured on a 298-node 10-Husky world under Newton (machine `9722d23d12a3`): `/sim/step` **27.0 s → 0.034 s** (~790×), a 10-step advance **120.0 s → 0.19 s** (~630×). ⚠️ **Those are PRE-`3b952b61d` figures and public issue #4 quoted them back at us as current. Re-measured 2026-08-29 on the same world (`husky_fleet_arena`, 309 nodes, CPU `mj_step`, same machine): full `/sim/step 1` **573–606 ms** vs light **6–35 ms** (~17×); 10 steps **2855–3187 ms** vs **48–67 ms** (~47×); the load itself 12.1 s vs 4.1 s. Smaller, still an order of magnitude — the advice stands, and every supervised `/world/load` response now carries a `tracking` block naming the mode and this cost, so an agent that never read this row still learns it from the response.** ⚠️ **It matters just as much on a TINY scene if that scene steps slowly** — measured 2026-08-14 on `newton_cloth_drape.omniworld`, two static bodies and one 289-particle sheet: `world/load` reload **13414 → 3131 ms**, `sim/step 1` **4298 → 210 ms**, `sim/step 30` **66671 → 1496 ms** (44.6×), `sim/reset` **7660 → 1870 ms**. Those heavy-mode figures predate `3b952b61d`, which cached the per-step scene walk and took heavy-mode cloth stepping to 191 ms/step; light is still ~4× better again. See the harness rule above for the mechanism — the cost is round-trips × step time, not node count, so "small world, skip the flag" is exactly wrong. The trade: `/sim/grips` returns empty and the `contact.*` / `grip.*` / `joint.limit_hit` event types go quiet — `?types=` is an exact-match allowlist, so filtering on a suppressed type returns an empty stream, not an error (`GET /capabilities` → `event_types_detail.suppressed` names them). ⚠️ **`/sim/contacts` is NOT suppressed and this row used to say it was** — it is served by `observe.collect_contacts`, which walks the scene per call and never reads the `ContactTracker`. On the Husky world the tracker-fed surfaces returned empty anyway, so the trackers were pure cost. **Light is the DEFAULT since 2026-09-02** — a `POST /world/load` naming neither `light` nor `tracking` runs light and says so in the response's `tracking.default_applied`. Ask for the trackers with `{"light": false}` (all three) or a `tracking` object (per-tracker); `OMNISIM_HARNESS_LIGHT=0` makes full tracking the process-wide default again. On the current engine the fleet arena loads 4.65 s light vs 5.2 s full and steps 23 ms vs 54 ms — the 12.1 s / 17–47× figures above are the 2026-08-29 engine and are history. ✅ **Per-tracker toggles ship as of 2026-09-01 (public issue #4):** `{"tracking": {"contacts": false, "joint_limits": false, "grips": false}}` on `POST /world/load` drops exactly the named trackers instead of all three — e.g. keep `joint.limit_hit` while paying no contact walk (measured: partial mode steps at light-mode cost, ~10 ms vs ~600 ms full on the fleet arena). `GET /capabilities` reports the per-mode suppression honestly, and the load response's `tracking.mode` reads `light`/`partial`/`full`. |
| `GET /world/diagnostics` | Re-fetch parsed diagnostics from the current load. |
| `POST /world/screenshot {path?, quality?}` | Render PNG. Returned as the response body (`image/png`) or written to a server-side `path`. |
| `GET /world/render_stats` | `{mean_brightness, mean_rgb, max_rgb, saturated_pct, black_pct, warnings[]}`. Warnings include `"blown out: NN% of pixels are saturated"` and `"underexposed: NN% near-black"`. |
| `GET /scene/tree` | Flat node list with type, DEF, position, orientation. |
| `GET /scene/node/<def>` | Field dump + contact points for one node. The dump includes **`boundingObject`** and **`physics`** as `{field_exists, present, summary}` — the two fields that decide whether a node collides and whether it moves (a floor with no `boundingObject` is a hologram). |
| `POST /scene/look_at {position, target, push?}` | Computes axis-angle orientation from default forward (+X) to the target direction and pushes it to the live `Viewpoint` when `push=true` (the default). Returns the orientation so it can be persisted back to the `.wbt`. |
| `GET /scene/tree?bounds=1` | Same tree, plus each node's **world-space** `{center, radius, bbox_min, bbox_max, size, exact}`. This is the number every camera decision needs; opt-in because it walks all geometry. |
| `GET /scene/viewpoint` | **Read** the live camera: position, orientation, `fieldOfView`, near/far, follow, plus derived `forward`/`up`/`right` and the resolved horizontal + vertical FOV for the real viewport aspect. |
| `POST /scene/frame {def\|defs\|target+radius, mode?, margin?, push?}` | ⭐ **The camera verb to reach for first.** Computes BOTH aim and distance so the subject fills the frame, pushes it, and returns a `verification` block (angular offset vs half-FOV, subject screen bbox in pixels, `fits`). `mode`: `hero` (default) / `top_down` / subject-relative `front`/`back`/`left`/`right`/`top`. |
| `POST /scene/orbit {azimuth_deg?, elevation_deg?, dolly?, pan?, center?\|def?}` | Incremental nudge **relative to the current view** — every other camera API is absolute. |
| `GET /scene/visible?defs=A,B` | What is on screen right now: frustum test, screen-space bbox + centroid in pixels, distance, angular offset, and a hint like `"off-screen: 34 deg to the left, 12 deg up"`. The closed-loop feedback signal for aiming. |
| `POST /scene/spawn {vrml\|type+fields\|clone, def, name?, translation?, rotation?, parent?, index?, settle_steps?, reset_physics?}` | ⛔ **BY DEFAULT A SPAWNED NODE HAS NO PHYSICS — IT RENDERS AND IT IS IN `/scene/tree`, BUT THE SOLVER NEVER SEES IT (measured 2026-08-17). ✅ SINCE 2026-09-01 THERE IS AN OPT-IN FIX: pass `{"physics": "rebuild"}` (or call `POST /sim/rebuild_physics` after the spawn) and the node IS simulated — W1.7 shipped in `88487d988`; details and caveats below.** The default is deliberately unchanged (a rebuild costs 97–267 ms and drops engaged welds, so it is never applied silently), and in that default spawn is only a working *scene-graph* primitive. Both directions fail: a spawned **dynamic** body never falls, and a spawned **static** body never collides. Measured on the CPU `mj_step` path against an in-session control (machine `9722d23d12a3`), floor topped at z=0.50 so the implicit ground plane cannot substitute: the *authored* control box settled at **z=0.599892** (floor top + half box = 0.600) while a spawned twin of that exact box, released at z=1.5, read **z=1.5 unchanged after 2200 explicit steps and ~87 s of simulated time** — not one float ULP; and a spawned static platform topped at z=1.00 was **fallen straight through**, the control landing back on the authored floor at 0.599892. The engine log is the mechanism: **exactly one** `registered 1 dynamic + 1 static Newton bodies` pass, emitted at load, the two spawns adding **zero**, and every `[OmNewtonBackend] step` line to step 61440 listing only `b0` (floor) and `b1` (control). **The failure is silent engine-side — 0 errors, 0 warnings, and the response returns `verification.node_resolved: true`** — but the HARNESS now tells you (2026-08-19 honest interim): every successful `/scene/spawn` and `/scene/delete` response carries a `physics_warning` block (`code: RUNTIME_MUTATION_NOT_IN_SOLVER`), the first use per verb per world-load emits one `world.warning` into `/sim/events`, and `GET /capabilities` lists the gap under `not_supported` (`scene.runtime_mutation_physics`). Cause: `finalizeWorld()` sets `openForBuild=false` ([`OmNewtonBackend.cpp`](../../src/omnisim/physics/OmNewtonBackend.cpp), `OmNewtonBackend::finalizeWorld`), every `addBody`/`addShape*` verb guards on it, and `OmNewtonBackend::ensureWorldOpen()` refuses to reopen mid-run. ⚠️ **This is the exact MIRROR of the runtime-delete defect above — same frozen MuJoCo model, opposite symptom: delete leaves phantoms IN, spawn leaves real nodes OUT.** So in DEFAULT mode use spawn for cameras/markers/visual props and for staging a scene you will then reload; do **not** use it for anything that must fall, collide, or be picked up, and never treat a spawned floor or wall as a collision surface — unless you opt in. ✅ **THE OPT-IN FIX SHIPPED 2026-09-01 (`88487d988`, W1.7 — runtime scene mutation, both directions at once): a mid-run PHYSICS REBUILD.** A new engine verb (opcode 105, `wb_supervisor_simulation_rebuild_physics`) tears down the live Newton world and re-registers the WHOLE scene at its **current** poses — live velocities are replayed and motor targets re-pushed automatically, so a running robot keeps driving. Harness surface: `POST /sim/rebuild_physics {settle_steps?}`, or `{"physics": "rebuild"}` directly on `/scene/spawn` / `/scene/delete`. Measured (machine `9722d23d12a3`, CPU `mj_step`): rebuild costs **97–267 ms** (in-process SolverMuJoCo reconstruction, skipping the module-load 98%); the spawned box from this row's own reproducer, frozen at z=1.5 for 120 steps, lands at **0.599892258644104** after rebuild — **bit-identical** to the authored control's rest height, with the control body unmoved; a deleted floor genuinely stops colliding (both bodies fell through); and an 8-Husky motorised world drove THROUGH a mid-run rebuild at unchanged speed (+1.749 vs +1.689 m per 2 s window). ⚠️ Three caveats before you lean on it: it is **REFUSED with `409 REBUILD_REFUSED` on Cloth / SoftBody / GranularBed worlds** (those re-register from *authored* state, so a rebuild would teleport them — reload instead); **engaged `Connector`/`VacuumGripper` welds are DROPPED**, with a loud warning naming the count — do not rebuild mid-grasp; and **bitwise step-for-step continuation across a rebuild is NOT claimed** (a fresh world is a fresh solver state). The DEFAULT spawn/delete behaviour is unchanged — `physics_warning` still attached unless the caller opts into the rebuild. Three input forms: a raw VRML node string, a `type` + `fields` spec (VRML composed for you), or `{"clone": "<DEF>"}`. Measured 0.27–0.44 s per spawn (Newton, light); the 0.03–0.32 s figure alongside it was the ODE path and is history — Newton's is the only cost you can hit now. ⚠️ **A `URDFRobot` CANNOT be spawned from a string — clone one instead.** `URDFRobot { url ... }` is a *source* expansion done by `OmTokenizer::tokenizeFile`; a supervisor import goes through `tokenizeString`, never expands it, and `OmParser::protoNodeList()` then classifies it as a PROTO that `importNode` refuses. The same refusal hits **every** PROTO the loaded world does not declare `IMPORTABLE EXTERNPROTO`. So the container world needs **one** authored robot, not zero. ⚠️ **A clone's `name` must be unique and must be right AT import time** — the engine starts the controller immediately and keys its IPC channel by the robot's name; a clone carrying the source's name collides (`refusing connection attempt from another extern controller`), that controller exits 1, and the robot simply never moves. Measured before the fix: **8 of 9 clones silently dead, no error anywhere in the HTTP responses.** Pass `name` and the harness rewrites it depth-aware in the node text. Failures are a `422` `SPAWN_REJECTED` carrying the rejected VRML + the engine's own parse error. |
| `POST /scene/delete {def\|defs, settle_steps?}` | Remove nodes by DEF. Unknown DEFs come back named rather than failing the batch. |
| `POST /scene/set_pose {def, translation?, rotation?, reset_physics?, settle_steps?}` | Move an existing node. A supervisor field write lands on the engine's **next** step, so this defaults to `settle_steps: 1` and **`reset_physics: true`** (a teleported body otherwise keeps its velocity and drifts, which reads as "the pose did not stick"). ⚠️ **Nothing checks interpenetration** — it will happily place a dynamic body inside static geometry and let the solver resolve it: on lane3, `BALL` (rest z ≈ 0.149) placed at z = 0.1 tunnelled through the floor and read **z = −2251** moments later. `GET /scene/node/<def>?bounds=1` before placing is the check. |
| `POST /sim/rebuild_physics {settle_steps?}` | ⭐ **Make runtime spawns/deletes reach the solver (2026-09-01, W1.7).** Tears down the live Newton world and re-registers the WHOLE scene at its current poses — velocities replayed, motor targets re-pushed, 97–267 ms measured. `409 REBUILD_REFUSED` on Cloth/SoftBody/GranularBed worlds (reload those); engaged welds are dropped with a loud warning; bitwise continuation across a rebuild is not claimed. Same effect inline via `{"physics": "rebuild"}` on `/scene/spawn`/`/scene/delete` — see that row. |
| `POST /sim/step {steps?}` | Advance the simulation N basic timesteps (default 1). |
| `POST /sim/reset {restore?, verify?, settle_steps?}` | Rewind the clock to t=0 **and restore the scene**, without re-parsing. It used to only rewind the clock and leave a fallen body where it fell; it now also loads the engine's own parse-time state `"__init__"` (`OmNode`'s constructor sets it, `OmPose`'s saves the authored pose under it — nothing has to be snapshotted first). `{"restore": null}` gets the old clock-only behaviour back. |
| `POST /sim/snapshot {name}` / `POST /sim/restore {name, settle_steps?}` / `GET /sim/snapshots` | Named engine-side state snapshots over `Node.saveState()` / `loadState()` from the scene root — recursive over the whole scene. `restore` puts the bodies back **without** rewinding the clock, and reports `verification.vs_snapshot.max_pose_delta_m` (top-level poses only; a body still falling legitimately reports non-zero). ⚠️ **Restoring an unknown name is refused on purpose** — the engine's saved-pose `QMap` default-constructs a **zero vector** on a miss, so an unguarded restore would teleport the whole scene to the origin; you get `404 SNAPSHOT_NOT_FOUND`. Names die with the world (the registry lives in the supervisor, which every load restarts); `__`-prefixed names are reserved. A supervisor-taken "initial" snapshot is **not** the authored state — the engine free-runs before the controller's first step (lane3's `BALL` is authored at z=1.0 and first reads z=0.1); use `/sim/reset` for authored. |
| `GET /sim/state` | Current world, supervisor connection, last load result. |
| `GET /sim/contacts` | Global contact set: `[{a_def, b_def, point, paired}]` **plus a `tracking` block** (what was walked, which bodies are idle). ⚠️ **`?wake=1` is a TOTAL no-op — it advances nothing and costs nothing; drop it anyway.** It existed because ODE auto-disabled a body idle for `WorldInfo.physicsDisableTime` and a disabled body generated no contacts, so a crate demonstrably resting on a floor returned `[]`. Newton has no body sleep, and native contact readback has been on by default since 2026-08-07, so a resting body reports its contacts without help. The two settle steps it used to take were **deleted**, not merely re-worded, so the read is idempotent again and can rejoin the harness's transparent-retry set — measured, `tracking.woken` reports `applied: false, steps_advanced: 0`. ⚠️ Do not infer stepping from the clock: the wrapped engine **free-runs between HTTP calls** (measured 88–112 ms of sim time per idle `/sim/state` poll with no other call), so `sim_time_ms` moves across any pair of requests and a non-zero delta around a read proves nothing about that read. |
| `GET /sim/grips` | Inferred grips: `[{gripper_def, held_def, since_t_ms}]`. |
| `GET /sim/events?since=&log_since=&limit=&types=` | Unified runtime event stream — supervisor-side (`contact.*`, `joint.limit_hit`, `grip.*`, `damage.*`) and harness-side (`controller.log`, `world.warning`, `world.error`) merged. Two cursors (`since` for sup, `log_since` for log). |
| `GET /robots` | Enumerate every Robot in the scene with pose and joint count. |
| `GET /robot/<def>/joints` | Per-joint snapshot: position, velocity (differenced), limits, `hit_limit`. |
| `GET /robot/<def>/devices` | List devices visible in the robot's subtree. |
| `POST /robot/<def>/joints/set` | ⭐ **The first robot-commanding endpoint (2026-08-19).** `{"joints": {"<name>": <rad>}, "settle_steps"?}` — supervisor joint targets with settle-and-verify: the write is a PD setpoint that converges over ticks (default settle 16 steps), so each joint returns `{commanded, achieved, error, moved, clamped, position_controllable, limits}` measured, never echoed. ⚠️ A limit-less motor (no `minPosition`/`maxPosition`) is a ke=0 velocity wheel whose `setPosition` is ignored — the verb pre-classifies and reports `position_controllable: false` instead of lying. ⚠️ An active bridge in hold mode re-asserts its own targets every tick and WINS (measured 0.42–0.70 rad residual on the UR5e) — command bridge-owned robots through their bridge; this verb owns passive/supervisor-only robots. PROTOCOL.md §7.33. |
| `POST /robot/<def>/ik` | ⭐ **Batched IK against the exact model the solver steps (2026-08-19)** — `{"effector": "<DEF>", "targets": [[x,y,z],...], "tool_offset"?, "iterations"?}`, pure PREVIEW (nothing moves; apply the returned angles via `/joints/set`, whose joint names the response maps to). Per-target `residual` in metres from the solver's own FK — an unreachable target reports its true residual, never "reached". Closed-loop verified: Cartesian error 1.0e-05–2.2e-04 m on a passive rig; ⚠️ first call in a fresh world compiles a warp kernel (~2.4 s warm disk cache, 8.3 s truly cold; ~110 ms after — `verification.warmup` discloses it). Hinge/Slider joints only (Ball/Hinge2 excluded, multi-coordinate); `mujoco_warp` unverified. PROTOCOL.md §7.34. |
| `GET /robot/<def>/sensor/<name>` | 501 by design — supervisor can't read sensors it doesn't own; use `/joints` or a per-robot helper. |
| `GET /robot/damage` | Damage state of the tracked robot (per-part HP / state). |
| `GET /robot/damage/events?since=&limit=` | A filtered view of the `damage.*` events — same records the unified `/sim/events` stream carries, with their own cursor. |
| `POST /robot/damage/reset` | Heal every part **without** resetting the simulation. |
| `POST /robot/damage/inject {part, state?, hp_delta?}` | Set a part's damage state directly — the fault-injection verb, so a damage-response path can be tested without staging a collision. |
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
- **Debugging a load failure** — `/world/load` returns structured codes (`WORLD_PARSE_INVALID_TOKENS`, `PROTO_NAME_MISMATCH`, `EXTERNPROTO_DOWNLOAD_FAILED`, etc.) instead of free-text stderr; branch on `diagnostics[].code` rather than regex-matching messages. The codes (an OPEN enum -- read the live set from `GET /capabilities`, never a written count) are enumerated in [`scripts/harness/diagnostic_codes.py`](../../scripts/harness/diagnostic_codes.py) and served live by `GET /capabilities` — trust the live enumeration over any written count.
- **Inspecting positions** — `/scene/tree` and `/scene/node/<def>` answer "where is X actually placed and what are its fields?" without a controller.
- **Stepping or resetting deterministically** — `/sim/step {steps:N}` and `/sim/reset` (which now restores the authored scene, not just the clock) for controlled experiments; `/sim/snapshot` + `/sim/restore` for a rollback point that is not t=0.

For the harness reference (sibling-file injection, hot-reload mechanics, structured-diagnostic mapper, endpoint cheatsheet), see [scripts/harness/README.md](../../scripts/harness/README.md). For unit tests covering the diagnostic mapper and helper math, see [tests/harness/](../../tests/harness/).

**Driving the harness from an MCP client** (Claude Desktop, Cursor): the harness endpoints are also exposed as MCP tools by [`packages/omnisim-mcp/`](../../packages/omnisim-mcp/) — start the harness, register `omnisim-mcp`, and `load_world` / `get_scene_tree` / `screenshot` / `sim_step` become first-class agent tools. It is a thin stdlib proxy to the same `:6789` surface, so everything above applies unchanged.

### Sister service: capture (port 6791) for cinematic output

The **capture service** is the harness's sister — same shape (HTTP + sibling-file supervisor injection over a length-prefixed JSON socket), different defaults and endpoints. It's what you reach for when you want a *high-resolution still*, a *recorded movie*, or a *deterministic cinematic render driven by a shot list* — not for tight authoring iteration.

Key differences from the harness:

- Runs on `127.0.0.1:6791` (supervisor on `:6792`) so both services can run simultaneously.
- Injects a supervisor robot carrying a `Camera` device sized to the requested output resolution, so renders are independent of the GUI viewport (4K and 8K both work).
- `/capture/sequence` walks a Catmull-Rom + slerp camera path frame-by-frame, dumps lossless PNGs, and ffmpeg-encodes them — h264 (default, CRF-controlled), h265, vp9, or ProRes 422 HQ master.
- A shot-list CLI ([`scripts/capture/render.py`](../../scripts/capture/render.py)) drives multi-shot runs end-to-end from a JSON or YAML file. Outputs land in `social/youtube_videos/captures/` (gitignored).

```bash
# Single high-res still
python scripts/capture/omnisim_capture.py --port 6791 &
curl -s -X POST http://127.0.0.1:6791/world/load \
  -d '{"path":"projects/samples/demos/worlds/flagship/warehouse_industrial.omniworld","width":3840,"height":2160}'
curl -s -X POST http://127.0.0.1:6791/capture/camera \
  -d '{"position":[-12,-12,6],"target":[0,0,1]}'
curl -s -X POST http://127.0.0.1:6791/capture/screenshot -d '{}' -o still.png

# Cinematic shot list (h264, 60fps, smoothstep camera moves)
python scripts/capture/render.py scripts/capture/shotlists/orbit_warehouse.json --ad-hoc
```

OmniSim renders through **wgpu-native** (Vulkan / D3D12 / Metal) — a real-time raster stack (scattered-sky dome, PCSS shadow maps, SSR, TAA, volumetrics) plus **OmniLight**, which *is* path-traced GI but **baked, not per-frame**: a CPU path tracer bakes an L1-SH irradiance probe volume off-frame and the lit shader pays one trilinear 3D-texture sample per pixel ([docs/developer/omnilight.md](../../docs/developer/omnilight.md)). So the output is not a per-frame path trace and is not Blender-Cycles quality — but the high-res Camera + lossless PNG + ffmpeg pipeline is meaningfully better than ad-hoc screenshots. Full reference at [scripts/capture/README.md](../../scripts/capture/README.md).

---

## Section 6 Generating new worlds

```bash
python scripts/dev/omniworld.py list-recipes
python scripts/dev/omniworld.py describe outdoor_forest
python scripts/dev/omniworld.py generate outdoor_forest --seed 42 --out my_forest.omniworld
python scripts/dev/omniworld.py validate my_forest.omniworld
launch.bat my_forest.omniworld  # or python -m omnisim run-world my_forest.omniworld
```

Recipes: `flat_ground`, `outdoor_forest`, `outdoor_desert`, `warehouse`, `urban_block`, `indoor_apartment`, `mars`. Determinism is guaranteed: same `(recipe, seed, params)` → byte-identical `.wbt`.

Generation parameters are passed as `--param key=value` (JSON-parsed). Full API and biome list: [docs/developer/omniworld-user-guide.md](../../docs/developer/omniworld-user-guide.md). To author a new biome: [docs/developer/omniworld-biome-cookbook.md](../../docs/developer/omniworld-biome-cookbook.md).

**Camera framing.** Worlds should open *looking at their subject*, not at the engine's fixed `-10 0 0` fallback. Generated worlds get this automatically (the emitter frames the robot spawn, else the scene). For hand-authored worlds, never eyeball `position`/`orientation` — bake the standard angled "hero" view (or a top-down for nav/overview worlds) with `scripts/dev/set_viewpoint.py`. See [docs/developer/viewpoint-convention.md](../../docs/developer/viewpoint-convention.md).

---

## Section 7 Editing a controller

Controllers live in `projects/<area>/controllers/<name>/<name>.{py,cpp,c}`. Python controllers run with the system `python` and need no rebuild — edit the file, re-launch the world, the simulator spawns the new controller process automatically.

Quick anatomy of a Python controller:

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

⚠️ **`omnisim` is now the ONLY Python module name — the `controller` alias was DELETED (2026-08-16), and `from controller import Robot` raises `ModuleNotFoundError`.** [`lib/controller/python/omnisim/`](../../lib/controller/python/omnisim/) holds the real implementation (`robot.py`, `motor.py`, `supervisor.py`, the `wb` ctypes binding, …) and is the runtime API exported from `lib/controller/python/` after a build. The 75-line `controller/__init__.py` re-export shim that used to sit beside it — aliasing every submodule via `sys.modules['controller.motor'] = omnisim.motor` — is gone. **This is a SOURCE break, not an ABI break:** no exported symbol moved, so already-compiled controllers are unaffected, and porting a Python controller is one line (`controller` → `omnisim` in its import). Repo-wide the only files still importing `controller` are under `tests/benchmarks/`, which is out of scope by policy (see below).

C/C++ controllers need a per-controller `make` (their folders ship Makefiles) and **must** `#include <omnisim/robot.h>` or `#include <omnisim/Robot.hpp>`. ⚠️ **The legacy `<webots/...>` include path was DELETED (2026-08-16)** — all 91 one-line forwarders under `include/controller/{c,cpp}/webots/` are gone, so `#include <webots/robot.h>` no longer compiles. Port by rewriting `webots/` to `omnisim/` in the include line.

⚠️ **THE C++ NAMESPACE IS NOW `omnisim`, AND THIS ONE *IS* AN ABI BREAK — this bullet said the opposite until 2026-08-16, so do not trust an older session's memory of it.** `namespace webots` became `namespace omnisim`, and `using namespace webots;` no longer compiles. The C++ namespace is part of every mangled symbol, so this is not a source-only rename: measured on the pre-rename `lib/controller/CppController.lib`, **1009 distinct mangled names carry `6webots`** (2018 archive symbol-table entries once the `__imp_` import thunks are counted — that is where the "~2018 exports" figure comes from), plus **44 in `CppDriver.lib` and 19 in `CppCar.lib`**. **Every C++ controller must be RECOMPILED**, not merely re-edited: an already-built `.exe` imports `_ZN6webots…` from a DLL that now exports `_ZN7omnisim…` and fails at load with a missing-entry-point error. There is deliberately **no compatibility alias** — `namespace webots = omnisim;` would put the string straight back into the shipped header, which is the whole reason the rename happened. The shipped [`template.cpp`](../../resources/templates/controllers/template.cpp) writes `using namespace omnisim;`.

⚠️ **The C API did NOT move, and the asymmetry is the point.** The C ABI is `wb_*`-prefixed and not one of its 497 exported functions contains the string "webots"; the `Wb*` types (`WbDeviceTag`, `WbNodeRef`, …) and the `wbu_*` utilities are untouched. **C controllers keep their symbols and do not need recompiling** — only the C++ ones do. If you find yourself renaming a `wb_` or `Wb` identifier, that is out of scope and doubles the blast radius for nothing.

A standalone controller `make` resolves the install root from `OMNISIM_HOME`, falling back to `WEBOTS_HOME` — that fallback is **build-time only** and survives because `$(WEBOTS_HOME_PATH)` is a contract variable shared with `resources/Makefile.include` and `dependencies/Makefile.*`. At **runtime** there is no such fallback any more: the launcher and libController read `OMNISIM_HOME` only (§1).

---

## Section 8 Validating a change

After editing code or a world, the recommended validation lanes (cheap → expensive):

```bash
# Does it still load? THE default check -- stops the moment Newton finalises
# and the .newton.json sidecar exists, instead of sleeping out a guessed
# --duration. Measured 15.52 s -> 6.37 s (and 45.34 s -> 6.37 s against the
# cold-disk advice), same PASS, same sidecar.
python -m omnisim run-headless path/to/world.omniworld --until-finalized

# Headless run with an explicit observation window. Use --duration only when
# the run must actually WATCH the simulation for that long; for a pure load
# check it is pure sleep.
python -m omnisim run-headless path/to/world.omniworld

# ALSO assert the end state is physically possible (no body has left the world
# through a missing collision surface). Use this whenever the change you are
# validating is a PHYSICS fix -- a bare PASS above cannot see it, and neither
# can --until-finalized, which only proves load + finalize.
python -m omnisim run-headless path/to/world.omniworld --duration 10 --fail-on-runaway

# Single-world test through the test suite
python -m omnisim test-world path/to/world.omniworld --nomake

# Fast smoke suite (multiple worlds)
python -m omnisim test-smoke

# One test group (api, parser, physics, rendering, cache, protos, other_api)
python -m omnisim test-group api

# Performance log capture for one world
python -m omnisim profile-world path/to/world.omniworld
```

`run-headless` is the right default. Use `--fail-on-warning` to be strict about the log, and `--fail-on-runaway` when the claim you are about to make is about the *physics* rather than the load (see §3b: both variants of the C2 fall-through task passed the log-only lane identically).

For *iterative* validation while authoring (not one-shot), use the harness from Section 5: initial `POST /world/load`, then `POST /world/sync` after edits. Sync returns the same structured diagnostics whenever it falls back to a hot reload, while safe pose-only changes avoid the parse/physics rebuild entirely.

---

## Section 9 Where to look when something goes wrong

- **`omnisim_log.txt`** in the repo root — all warnings, errors, and structured runtime messages land here. Always read this first when a world fails to load or behaves unexpectedly.
- **Build problems** — see [docs/developer/quickstart.md](../../docs/developer/quickstart.md) sections 1–6 (toolchain) and [docs/developer/build-and-iteration.md](../../docs/developer/build-and-iteration.md) (rebuild scoping).
- **URDF import problems** — [docs/developer/urdf-import-debugging.md](../../docs/developer/urdf-import-debugging.md). The `scripts/dev/urdf_import.py --report --strict` tool gives a structured preflight report.
- **Performance problems** — [docs/developer/profiling-playbook.md](../../docs/developer/profiling-playbook.md) and `OMNISIM_RENDERER_TIMINGS=1` in the environment.
- **Subsystem ownership map** — [docs/developer/agent-map.md](../../docs/developer/agent-map.md). This is the search guide for "where do I find the code that does X?"

---

## Section 10 Conventions to honour

- **Do not edit `src/glm/` or `src/stb/`.** They are vendored submodules.
- **Do not skip git hooks** (`--no-verify`, `--no-gpg-sign`) unless explicitly told to.
- **Do not commit unless asked.** When you do, prefer specific paths over `git add -A`.
- **Do not invent new CLI flags or scripts.** Use the helpers in `scripts/dev/`. If something is genuinely missing, propose adding it before working around it.
- **Before drafting or sending any OmniSim email outreach batch, read and obey [`social/launch/EMAIL_OUTREACH_RULES_2026-08-28.md`](../../social/launch/EMAIL_OUTREACH_RULES_2026-08-28.md).** Every target needs a verified project fact, one bounded experiment, one measurable result, exactly one call to action, and no first-touch star request. Respect the outreach-control-room freeze and mark mismatched or weakly verified targets `do_not_send`.
- **Worlds in smoke / benchmark lanes must be local-asset-only** (no `http(s)://` PROTOs). The `omniworld validate` command and the `asset_locality` check enforce this.
- **`OMNISIM_HOME`** is the canonical environment variable for the install root post-rebrand: setting it alone is sufficient to build and run. **The engine↔controller RUNTIME contract is now `OMNISIM_*`-only** — libController, the controller launcher and the Python package read no `WEBOTS_*` fallback at all, and the engine writes no `WEBOTS_*` twin (it removes any it inherits). Where `WEBOTS_HOME` does survive is **build-time and one GUI library**, so don't write code that assumes it has been purged everywhere: (1) the shipped **`qt_utils` runtime library** reads `OMNISIM_HOME` first and still falls back to `WEBOTS_HOME` ([`StandardPaths.cpp`](../../resources/projects/libraries/qt_utils/core/StandardPaths.cpp)); (2) the **build** exports `WEBOTS_HOME` from the top-level [`Makefile`](../../Makefile) and **16** more Makefiles expand `$(WEBOTS_HOME)` (`$(WEBOTS_HOME_PATH)` / `$(WEBOTS_CONTROLLER_LIB_PATH)` are cross-Makefile contract variables, not runtime names). For new tooling, write `OMNISIM_HOME`.

---

## Section 11 Further reading

- [README.md](../../README.md) — project overview and platform support
- [PROTOCOL.md](../../PROTOCOL.md) — the canonical OmniSim Wire Protocol specification (robot bridges, harness, capture, twin shadow). If you are writing a new bridge or a tool outside this repo that drives OmniSim, this is the contract.
- [docs/developer/README.md](../../docs/developer/README.md) — developer-doc index
- [docs/developer/quickstart.md](../../docs/developer/quickstart.md) — full local build/run walkthrough
- [docs/developer/agent-map.md](../../docs/developer/agent-map.md) — code-search and subsystem map for agents
- [docs/developer/simulation-authoring-for-coding-agents.md](../../docs/developer/simulation-authoring-for-coding-agents.md) — best workflow for building new simulations
- [docs/developer/rl-current-state.md](../../docs/developer/rl-current-state.md) — **CANONICAL RL status. Read the top banners first (the newest is dated in its heading; "WHERE WE STAND" 2026-07-06 remains the humanoid checkpoint): SHADOWING is the flagship algorithm for legged-robot motion** — the flagship demo is the G1 "decent walker" (WBMATCH 0.868 on the honest shape-only ruler, ⚠️ **on the weight-bearing balance harness / puppet rig — not free-standing**), generalized to the H1 (recorded ghost PASSes the validator). Start here before claiming ANY robot result or touching RL. ⚠️ **The file is APPEND-ONLY history: only the top banner is current.** Older sections below it (the deterministic-first banner, the WBMATCH2-era numbers, the per-robot "walls") are earlier checkpoints and are *not* individually marked superseded — when they disagree with the top banner, **the banner wins**. Deterministic control remains the shipped path for STATIC stand/balance only. If any other doc/script/commit disagrees with this file, this file is right.
- [docs/developer/shadowing.md](../../docs/developer/shadowing.md) + [docs/developer/ghost-design-rules.md](../../docs/developer/ghost-design-rules.md) + [projects/policies/training/README.md](../../projects/policies/training/README.md) — **the Shadowing method**: the paper scaffold (thesis: ghost feasibility is the bottleneck — verify before learning), the 7 formal ghost-design rules with the calibrated pre-training `ghost_validator.py` (run it on ANY new ghost BEFORE training; 7/7 agreement with training ground truth), and the flagship implementation + canonical launcher.
- [docs/developer/skill-library.md](../../docs/developer/skill-library.md) + [projects/policies/skills/README.md](../../projects/policies/skills/README.md) — **the SKILL LIBRARY: the standard way to MAKE a skill and COMPOSE skills.** Binds each skill's five otherwise-scattered artifacts (ghost lut, `ghost_validator` verdict, deploy env, champion checkpoint, provenance) into one versioned manifest, and reuses the Shadowing trainer + BATON deploy stack unchanged. `python -m omnisim policy` is the front door (`list`/`show`/`validate`/`ghost`/`preview`/`train`/`run`/`sequence`/`verify-demos`/`benchmark`/`index` + `adapt`/`blendable`/`handover`/`freeze`, plus the natively-implemented `graph`/`ir`/`matrix`/`env-hash`/`promote`/`audit`); `projects/policies/skills/skill_lib.py` implements the pipeline half and remains directly runnable. `verify-demos` proves the manifests reproduce the hand-written demo scripts **key-for-key on the assembled launch env** (every env var the demo script sets is asserted to match what the manifest produces — it is an env-dict comparison, not a byte diff of the scripts). Skills span robots (G1/H1/Go2/OmniQuad) and methods (Shadowing / Unitree re-host / deterministic overlay); 15 skills + 6 BATON sequences ship today. Read this to add a new skill or build a new BATON demo.
- [docs/developer/omniquad-residual-rl.md](../../docs/developer/omniquad-residual-rl.md) — OmniSim-native RL pipeline (residual PPO on a model-based gait), OmniQuad is the first robot
- [docs/developer/g1-single-source-of-truth.md](../../docs/developer/g1-single-source-of-truth.md) — **VERIFIED: the G1 RL trainer and the OmniSim Newton deploy run the SAME physics.** Both derive their physical model from one source — [`projects/policies/research/backends/g1_physics.json`](../../projects/policies/research/backends/g1_physics.json) + [`g1_physics_spec.py`](../../projects/policies/research/backends/g1_physics_spec.py) + the prim URDF (geometry/limits read live). Proven three ways: a compiled-`MjModel` field diff (trainer vs the *literal* extracted deploy runtime → 0 real-physics gaps), a GPU golden-trajectory test, and a **live H100 training run whose persisted `physics_config.json` byte-matches `SPEC.newton_env()`** on every parameter (solver, substeps, ke/kd, friction, joint clamp, inertia, seed pose, prim-URDF sha). Enforced in CI ([`tests/test_g1_physics_spec_conformance.py`](../../tests/test_g1_physics_spec_conformance.py)); golden harness [`projects/policies/research/training/g1_golden_parity.py`](../../projects/policies/research/training/g1_golden_parity.py). The rule: never re-declare a physics constant on either side — import the spec.
- [docs/developer/g1-ghost-fidelity-journey.md](../../docs/developer/g1-ghost-fidelity-journey.md) — **"make G1 walk ≥80% like the ghost" — the full honest journal.** A numerical per-joint similarity metric (`--eval-ghost-similarity`); the human ghost is a ~67% *physical* wall (a balancing biped must deviate from a kinematic reference); a **feasible** ghost (`--build-achieved` + `--gait-style achieved`) lifts the **shape** match to **84–88% over a 3 s window** — but the policy **topples ~7 s** and the deploy falls sooner (durability is the open **trainer↔deploy gap**, not similarity). Read before claiming a working G1 walk.
- [docs/developer/train-deploy-gap.md](../../docs/developer/train-deploy-gap.md) — **the train→deploy gap as TWO gaps** (pipeline-parity vs durability) + the enumerated, re-verified divergence table (COM default-off, finite-diff qd, launch-IC, contact) + **Unitree's proven obs/action/reward deploy recipe** as the durability answer. Read when working on making any RL policy deploy durably.
- [docs/developer/closed-loop-chaos-diagnostic.md](../../docs/developer/closed-loop-chaos-diagnostic.md) — **RUN THIS when a policy behaves differently in the trainer than in deploy and you don't know why.** A hardened BUG-vs-CHAOS classifier (`projects/policies/research/training/closed_loop_parity_compare.py`): run the same policy both sides, and the *shape* of the divergence says whether it's a real pipeline/obs `[BUG]`, intrinsic physical `[CHAOS]` (a free biped is unstable — divergence is unavoidable, fix with robustness not parity), or a clean `[MATCH]`. Don't theorise about the physics before classifying the divergence.
- [scripts/harness/README.md](../../scripts/harness/README.md) — reference for the HTTP harness covered in Section 5
- [docs/developer/omniworld-user-guide.md](../../docs/developer/omniworld-user-guide.md) — procedural world generation
- [CONTRIBUTING.md](../../CONTRIBUTING.md) — contribution process

If a doc and the code disagree, the code wins — and update the doc in the same change.
