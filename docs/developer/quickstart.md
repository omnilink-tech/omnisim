# OmniSim Developer Quickstart (Windows)

This guide covers everything needed to build and run OmniSim from source on Windows. It assumes a clean machine with no prior MSYS2 or Webots installation. Building on Linux? Jump to the [Linux quickstart](#linux-quickstart-ubuntu) at the end.

See also:

- [../../AGENTS.md](../../AGENTS.md) — agent-facing TL;DR (build, launch, headless run, HTTP bridge, validation), in copy-paste form
- [README.md](README.md) for the developer-doc index
- [build-and-iteration.md](build-and-iteration.md) for the narrowest rebuild paths
- [header-hygiene-and-rebuild-reduction.md](header-hygiene-and-rebuild-reduction.md) for keeping rebuild scope under control
- [validation-playbook.md](validation-playbook.md) for change-to-test mapping
- [profiling-playbook.md](profiling-playbook.md) for performance logging and benchmarks

## Prerequisites

- Windows 10/11
- ~10 GB free disk space (MSYS2 + dependencies + build artifacts), plus ~1 GB for the Newton runtime bundle (no longer optional on Windows — it is the only physics backend)
- Git for Windows (for cloning the repo)
- A Windows **CPython 3.12** from [python.org](https://www.python.org/downloads/), installed to the default per-user location. The v4 default build (`OMNISIM_WITH_NEWTON=ON`) embeds CPython and needs its build headers — the Makefile globs `…/Programs/Python/Python3NN/include/Python.h` and takes the **highest** version it finds (override with `PYTHON_HOME=`/`PYTHON_LIB=` make arguments). ⚠ **3.10 and 3.11 are not usable**: `newton` 1.5.0 raises `TypeError: Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool].` at `ModelBuilder()` on 3.10, so the engine links, loads worlds, and nothing moves. Version policy for every platform lives in one place — [system-requirements.md](../guide/system-requirements.md). ⚠ The embedded interpreter is **mandatory**, not a choice: `bdc02139` deleted `src/ode`, so Newton with `SolverMuJoCo` is the only physics backend and there is no pure-ODE legacy stack to fall back to — `OMNISIM_WITH_NEWTON=OFF` leaves the engine with no physics at all. The same Python also runs the dev tooling (`python -m omnisim`, the harness, `omniworld`).

> **A note on paths.** This guide assumes you have set **`OMNISIM_HOME`** to the absolute path of your local checkout. **Setting it alone is sufficient** — the top-level `Makefile` exports `WEBOTS_HOME` for you, for the build Makefiles that still expand the old name. The scripts and the build system never assume a fixed install location — set this once per shell and the rest of the commands work unchanged. The bundled `build_omni.bat` derives it from its own location, so on Windows you usually do not need to export anything manually.
>
> ⚠️ **`WEBOTS_HOME` is NOT an "alias accepted everywhere" — this note said so until 2026-08-16 and contradicted the environment table further down this same page.** It is a **build-only** alias now; the runtime reads `OMNISIM_HOME` and nothing else, and a shell exporting only the legacy name gets a warning and a refusal rather than a fallback. See the [environment variables](#environment-variables) table.
>
> ```bash
> # MSYS2 MINGW64 terminal — run from the root of your clone:
> export OMNISIM_HOME=$(pwd)
> ```
>
> ```bat
> :: cmd.exe — adjust to wherever you cloned the repo:
> set OMNISIM_HOME=C:\path\to\omnisim
> ```

## 1. Install MSYS2

Download and install from https://www.msys2.org/ to `C:\msys64`.

After installation, open an MSYS2 MINGW64 terminal and update:

```bash
pacman -Syu --noconfirm
pacman -Su --noconfirm
```

## 2. Install build dependencies

From the MSYS2 MINGW64 terminal:

```bash
pacman -S --noconfirm \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-make \
  make \
  mingw-w64-x86_64-qt6-base \
  mingw-w64-x86_64-qt6-websockets \
  mingw-w64-x86_64-qt6-tools \
  mingw-w64-x86_64-qt6-multimedia \
  mingw-w64-x86_64-qt6-declarative \
  mingw-w64-x86_64-wget \
  mingw-w64-x86_64-freetype \
  mingw-w64-x86_64-openal \
  mingw-w64-x86_64-minizip \
  git \
  curl \
  unzip \
  zip
```

`curl` and `unzip` are what `scripts/dev/setup_wgpu_native.sh` (step 5) uses to fetch and unpack the wgpu-native release, so install them even if you have a Windows `curl.exe` on `PATH`.

## 3. Set up Qt6 headers

The build system expects Qt6 headers at `include/qt/` inside the project with a nested layout. From the MSYS2 MINGW64 terminal:

```bash
cd "$OMNISIM_HOME"  # or wherever the repo lives

mkdir -p include/qt
for d in /mingw64/include/qt6/Qt*; do
  name=$(basename $d)
  mkdir -p include/qt/$name/$name
  cp -r $d/* include/qt/$name/$name/
done
```

## 4. Set up Git submodules

The project depends on two submodules: `glm` (math library) and `stb` (image loading).

```bash
cd "$OMNISIM_HOME"

# If submodule directories are empty (common for non-git-cloned copies):
git clone https://github.com/g-truc/glm.git src/glm
cd src/glm && git checkout 1.0.1 && cd ../..

git clone -b patch-1 https://github.com/omichel/stb.git src/stb
```

GLM 1.0.1 is used for compatibility with GCC 15. Later versions may have `operator=` issues with `packed_highp` types.

## 5. Build

### First: fetch wgpu-native (or the build has no renderer)

```bash
cd "$OMNISIM_HOME"
bash scripts/dev/setup_wgpu_native.sh
```

Run this once per clone, **before** the first build. wgpu-native is the only renderer — WREN was deleted on 2026-08-23 (`976b9449d`), along with `src/wren` and `src/omnisim/wren` — and the link against it is conditional: `src/omnisim/Makefile` defines `WB_WGPU_NATIVE_AVAILABLE` only when `WGPU_NATIVE_HOME` resolves, and every `wgpu*` call lives behind that macro. Skip this step and — until 2026-08-29 — the build was *green* and nothing drew: no main view, no screenshots, no capture service, no Camera device (public issue #7). **The Makefile now refuses that build at parse time**, before one TU compiles, with the setup command in the message; a deliberately compute-only binary is still available by name, `make release OMNISIM_RENDERERLESS=ON`, and it logs at runtime that it has no renderer. `setup_wgpu_native.sh` installs to the one path the Makefile auto-discovers (`$OMNISIM_HOME/_scratch/wgpu-native`), so you do not need to export `WGPU_NATIVE_HOME` yourself. Details: [wgpu-native-setup.md](wgpu-native-setup.md).

⚠ An **explicit empty** `WGPU_NATIVE_HOME=` on the make command line (or `OMNISIM_WITH_VULKAN=OFF`) still opts out — but since the WREN deletion that no longer selects "the other renderer", it selects a binary with no renderer at all, so the Makefile **refuses it** unless you also pass `OMNISIM_RENDERERLESS=ON`. `clean`, `linker-info` and `bundle-newton-runtime` are exempt from the check.

### The build itself

From the MSYS2 MINGW64 terminal (with `OMNISIM_HOME` exported as shown in the prerequisites):

```bash
make -C "$OMNISIM_HOME" -j$(nproc) release 2>&1 | tail -20
```

Or, from `cmd.exe` at the repo root, use the bundled wrapper which derives `OMNISIM_HOME` from its own location and invokes MSYS2's `make` for you:

```bat
build_omni.bat
```

If your MSYS2 install is not at `C:\msys64`, set `MSYS64_HOME` first: `set MSYS64_HOME=D:\msys64`.

`build_omni.bat` runs make with half your logical cores at below-normal process priority, so the machine stays responsive during the build. Set `MAKE_JOBS=N` first to override (more jobs = faster but RAM-hungrier; the big C++ translation units can thrash laptops at full parallelism).

First build takes 5-25 minutes depending on hardware. Incremental builds can range from seconds to several minutes depending on which headers and simulator layers changed. If a small edit triggers too much rebuild work, use `build-and-iteration.md` and `header-hygiene-and-rebuild-reduction.md` before widening the change.

### What gets built

| Target | Output | Description |
|--------|--------|-------------|
| GLAD | `src/glad/glad.a` | OpenGL function loader (static lib). Still linked: the GL present/blit fallback path uses it. |
| Renderer | *(no separate target)* | ⚠️ There is **no** `src/wren/wren.a` any more — this table listed one until 2026-08-28. WREN was deleted on 2026-08-23 (`976b9449d`); the wgpu backend compiles into the engine from `src/omnisim/render/` (plus `src/omnisim/nodes/OmWgpuSceneRenderer.cpp` for Camera-family devices), so a core build **is** a renderer build. `wgpu_native.dll` is downloaded by `setup_wgpu_native.sh` and copied next to `omnisim-bin.exe` by the link recipe. |
| omnisim-bin | `msys64/mingw64/bin/omnisim-bin.exe` | Main simulator binary. ⚠️ **There is no `webots-bin.exe`** — this row claimed that alias until 2026-08-16; no Makefile produces the name and nothing should fall back to it. The legacy *launchers* `webots.exe` / `webotsw.exe` do exist, as byte-identical copies of `omnisim.exe` / `omnisimw.exe`. |
| Controller (C) | `lib/controller/Controller.dll` | C controller API |
| Controller (C++) | `lib/controller/CppController.dll` | C++ controller API |
| Projects | Various `.exe` in `projects/` | Sample robot controllers |

### Build just the core simulator

```bash
make -C "$OMNISIM_HOME"/src/omnisim -j$(nproc) release
```

This skips controller libs and sample projects. Useful when iterating on the simulator itself.

### Build just the controller libraries

```bash
make -C "$OMNISIM_HOME"/src/controller -j$(nproc) release OMNISIM_HOME="$OMNISIM_HOME"
```

## 6. Set up runtime dependencies

The simulator needs Qt6 plugins and MinGW DLLs at runtime. Run these once after a fresh build:

```bash
# Copy all MinGW runtime DLLs
cp /mingw64/bin/*.dll "$OMNISIM_HOME"/msys64/mingw64/bin/

# Qt6 platform plugin (required for window creation)
mkdir -p "$OMNISIM_HOME"/msys64/mingw64/bin/platforms
cp /mingw64/share/qt6/plugins/platforms/qwindows.dll "$OMNISIM_HOME"/msys64/mingw64/bin/platforms/

# Qt6 image format plugins (required for JPEG/PNG texture loading)
mkdir -p "$OMNISIM_HOME"/msys64/mingw64/bin/imageformats
cp /mingw64/share/qt6/plugins/imageformats/*.dll "$OMNISIM_HOME"/msys64/mingw64/bin/imageformats/

# Qt6 TLS plugins (required for HTTPS, avoids "No TLS backend" warnings)
mkdir -p "$OMNISIM_HOME"/msys64/mingw64/bin/tls
cp /mingw64/share/qt6/plugins/tls/*.dll "$OMNISIM_HOME"/msys64/mingw64/bin/tls/
```

## 7. Run the simulator

### From a batch file (recommended)

Use the included `launch.bat` at the repo root. It derives `OMNISIM_HOME` from its own location, so it works from any drive or directory without editing. Calling it with no arguments opens the **demo launcher** (right-click the orb robot → *Show Robot Window* → pick a demo from the gallery); pass a world path to open a specific world directly:

```bat
launch.bat                                                              REM demo launcher (default)
launch.bat projects\samples\demos\worlds\showcase\warehouse_husky.omniworld   REM specific world
```

### From the MSYS2 terminal

```bash
export PATH="$OMNISIM_HOME"/msys64/mingw64/bin:/mingw64/bin:$PATH
"$OMNISIM_HOME"/msys64/mingw64/bin/omnisim-bin.exe
```

### Open a specific world

```bash
"$OMNISIM_HOME"/msys64/mingw64/bin/omnisim-bin.exe "$OMNISIM_HOME"/projects/samples/demos/worlds/showcase/warehouse_husky.omniworld
```

### Headless run (no window)

For agent loops, CI, or any time you don't want an OmniSim window to open, use the headless runner. It launches `omnisim-bin.exe` with `--minimize --batch --no-rendering --mode=fast --stdout --stderr` (`--minimize` is the long-standing default; `--no-window` — `OMNISIM_NO_WINDOW=1`, zero widget construction, camera devices still render offscreen through wgpu — was recorded here as deadlocking Newton's embedded CPython on multi-articulation worlds. **That claim was stale (public issue #5): it dates from a 2026-05-28 XPBD-era measurement, the Linux CI smokes have run under `OMNISIM_NO_WINDOW=1` since the 22.04 work, and on 2026-08-29 the 8-Husky swarm (40 dynamic bodies, 9 controllers) and the G1 humanoid both finalised and stepped under `--no-window` on Windows, byte-for-byte the same finalize/step lines as the `--minimize` control.** Pass `--no-window` for containers and GPU-less hosts, where a main view has nothing to draw on):

```bash
# Load check -- stops the moment Newton finalises and writes its sidecar.
python scripts/dev/headless_runner.py projects/samples/demos/worlds/showcase/turtlebot3_drive.omniworld --until-finalized

# Observation run -- --duration is a wall-clock SLEEP, so pass it only when the
# run must actually watch the simulation for that long.
python scripts/dev/headless_runner.py projects/samples/demos/worlds/showcase/turtlebot3_drive.omniworld --duration 10
```

Omitting `--duration` selects `--until-finalized` for you (announced on stdout) with a 30 s ceiling — `DEFAULT_DURATION_S` in `headless_runner.py`. The ceiling is only ever a ceiling: the run returns as soon as the world finalises *and* takes its first physics step, so a larger number costs a healthy world nothing.

It writes the engine log to `omnisim_log.txt`, tails it for errors and warnings, and returns a structured exit code (0 PASS / non-zero FAIL). See [AGENTS.md §3b](../../AGENTS.md) (headless runs) and [§8](../../AGENTS.md) (validating a change) for the full argument list — `--until-finalized`, `--fail-on-warning`, `--fail-on-runaway` — and the rationale.

### Demo worlds to try

| World | Path | Description |
|-------|------|-------------|
| **Warehouse Husky** *(default)* | `projects/samples/demos/worlds/showcase/warehouse_husky.omniworld` | Onboarding demo. Husky random-walks a warehouse with reactive collision recovery. Try `click + F` to follow it, `LMB + WASD` to fly. |
| Husky maze | `projects/samples/demos/worlds/flagship/husky_maze.omniworld` | Single Husky in a maze — classic navigation testbed |
| Husky fleet arena | `projects/samples/demos/worlds/showcase/husky_fleet_arena.omniworld` | 10 Huskies random-walking a walled arena; tests multi-robot collision recovery |
| Generated Mars | `distribution/generated_worlds/mars.wbt` | Procedurally generated planetary terrain with Husky fleet (regenerable via `omniworld`) |

For an agent-driven workflow (headless, structured exit, supported run contract):

```bash
python -m omnisim run-headless projects/samples/demos/worlds/showcase/warehouse_husky.omniworld --until-finalized
```

To regenerate a procedural world:

```bash
python scripts/dev/omniworld.py list-recipes
python scripts/dev/omniworld.py generate mars --seed 42 --out my_mars.omniworld
launch.bat my_mars.omniworld
```

The world extension is `.omniworld`. The policy is **dual-read, single-write**: the engine, harness and every script read `.omniworld` and `.wbt` interchangeably and indefinitely (`distribution/generated_worlds/mars.wbt` above is a frozen artifact and still loads), but everything you *generate* or author gets `.omniworld`.

## 8. Debug and log output

All warnings and errors are logged to `omnisim_log.txt` in the project root. Check this file after launching to diagnose issues:

```bash
cat "$OMNISIM_HOME"/omnisim_log.txt
```

To get a console-attached build (shows output in the terminal, useful for development), use the supported target:

```bash
make -C "$OMNISIM_HOME"/src/omnisim -j$(nproc) debug
```

The mechanism is one line in `src/omnisim/Makefile`: `LD_FLAGS += -Wl,-subsystem,windows` is applied **only** under `BUILD_GOAL=release`, so a `debug` build of the same sources produces a console-subsystem `omnisim-bin.exe` and stdout/stderr land in your terminal. Objects go to `build/debug/` (`OBJDIR=build/$(BUILD_GOAL)`), so this is a full compile the first time and does not disturb your release objects; it does write the same `TARGET` path, so a later `make release` puts the GUI binary back.

If you want to keep the release objects and just relink them into a *separate* console binary, mirror the Makefile's own `LIBS` line rather than inventing one:

```bash
cd "$OMNISIM_HOME"/src/omnisim
g++ -o "$OMNISIM_HOME"/msys64/mingw64/bin/omnisim-debug.exe \
  build/release/*.o -Wl,--enable-auto-import \
  -L"$OMNISIM_HOME"/msys64/mingw64/bin -L/mingw64/bin -L/mingw64/lib \
  ../glad/glad.a \
  -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets \
  -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml \
  -L"$OMNISIM_HOME"/_scratch/wgpu-native/lib -lwgpu_native \
  -lopenal -lopengl32 -liphlpapi -ld3d9 -lgdi32 -lglu32 \
  -lOIS -ldinput8 -ldxguid -lole32 -lsapi -loleaut32 -luuid \
  -lfreetype-6 -lopenvr_api -lassimp-5
```

⚠️ This block used to link `../wren/wren.a` and `-lode` (and `-lpico`, which no Makefile has referenced for years). All three are gone — `src/wren` was deleted on 2026-08-23 (`976b9449d`) and `src/ode` in `bdc02139`, and `ODE_LINK` is now defined-and-empty — so the command could not link at all. The list above tracks `src/omnisim/Makefile`'s Windows `LIBS`; if a relink fails on an undefined symbol, read that line rather than guessing, because it is the only thing that is kept current.

The difference: `omnisim-bin.exe` from `make release` is a Windows GUI app (no console). A `debug` build, or `omnisim-debug.exe` above, keeps the console attached so you can see stderr/stdout directly.

## 9. Subsystem map

```
src/
  glad/         OpenGL function loader. Rarely needs changes.
  glm/          Math library (submodule). Do not edit directly.
  stb/          Image loading (submodule). Do not edit directly.
  controller/   Controller APIs: c/, cpp/, launcher/. Edit here for robot
                programming interface changes. (The Python API is not built
                here -- it is a ctypes binding shipped as source at
                lib/controller/python/omnisim/.)
  python/       The omniworld procedural world generator package.
  omnisim/      Main simulator. This is where most work happens.
    app/        Application lifecycle, selection, perspectives
    control/    Controller process management and IPC
    core/       Logging, preferences, paths, network, project management
    editor/     Text editor, project relocation dialogs
    engine/     Simulation stepping, world lifecycle
    gui/        Main window, 3D view, menus, toolbars, dialogs, main.cpp entrypoint
    maths/      Math utilities (vectors, matrices, polygons)
    nodes/      World node types (robots, sensors, actuators, shapes, etc.)
    nodes/utils/ World management, node factories, template engine
    physics/    Physics backend layer (Newton / `SolverMuJoCo`)
    render/     The renderer. wgpu-native backend, render targets, shaders,
                mesh/image adapters. Edit here for graphics work — there is no
                separate renderer library to build, it compiles into the engine.
    scene_tree/ Scene tree widget and property editors
    sound/      Audio system
    user_commands/ Undo/redo, action manager
    vrml/       VRML/PROTO parser, tokenizer, node model, URL resolution
    widgets/    Reusable UI widgets

projects/       Sample worlds, robot models, object models, appearances
resources/      Runtime assets (icons, fonts, shaders, translations, node defs)
tests/          Test suites (API, parser, physics, rendering, cache)
docs/           Documentation (guide, reference, developer)
```

## 10. Common tasks

### Fast developer CLI

The repo now exposes a thin developer command layer for targeted build and validation:

```bash
python -m omnisim --help
python -m omnisim build core
python -m omnisim build gui
python -m omnisim test-smoke
python -m omnisim test-world tests/api/worlds/accelerometer.omniworld
python -m omnisim run-headless tests/api/worlds/accelerometer.omniworld
python -m omnisim profile-world tests/rendering/worlds/normals.omniworld
```

`python -m omnisim` is the only spelling: the `scripts/dev/omnisim_dev.py` shim that used to forward to it was deleted on 2026-09-02.

⚠️ **There is no `build renderer`.** It now exits non-zero with an explanation (`omnisim/dev/commands.py`), because the `renderer` make target it used to call survives only in the top-level `Makefile`'s `.PHONY` and goal-filter lists with **no recipe behind it** — `make renderer` prints `Nothing to be done for 'renderer'` and exits **0**, a build command reporting success while building nothing. Use `build core` (or `build gui`); the wgpu renderer is part of the engine.

⚠️ **`--nomake` means "do not re-compile the controllers"** (`tests/test_suite.py`). On a fresh clone the test controllers do not exist yet, so passing it on the first run tests a world whose controller binary is missing. Omit it the first time, then add it once the controllers are built.

Equivalent make aliases are also available:

```bash
make sim-core
make sim-gui
make controller-libs
make tests-smoke
make benchmarks
make compile-commands
```

If `ccache` is installed, the root makefile now uses it by default. Disable it with `USE_CCACHE=0`.

### "I changed a node type" (e.g., edited a file in `src/omnisim/nodes/`)

```bash
make -C "$OMNISIM_HOME"/src/omnisim -j$(nproc) release
# Then relaunch the simulator
```

### "I changed the renderer" (e.g., edited a file in `src/omnisim/render/`)

```bash
make -C "$OMNISIM_HOME"/src/omnisim -j$(nproc) release
```

Same command as a node-type change, and that is the point: there is no separate renderer library to build first. This section used to name `src/wren/` and run `make -C src/wren` before relinking; WREN was deleted on 2026-08-23 (`976b9449d`) and the directory does not exist. The wgpu backend is compiled into the engine from `src/omnisim/render/` (plus `src/omnisim/nodes/OmWgpuSceneRenderer.cpp`, which drives Camera-family device rendering), so a core build **is** a renderer build.

### "I changed a controller API" (e.g., edited `src/controller/c/`)

```bash
make -C "$OMNISIM_HOME"/src/controller -j$(nproc) release OMNISIM_HOME="$OMNISIM_HOME"
```

### "I want a completely clean rebuild"

```bash
make -C "$OMNISIM_HOME"/src/omnisim clean
make -C "$OMNISIM_HOME" -j$(nproc) release
```

## Environment variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `OMNISIM_HOME` | Root of the OmniSim installation (canonical) | Auto-detected from binary location |
| `WEBOTS_HOME` | **Build-only legacy alias.** The runtime no longer reads it: libController, the Python controller package and the extern-controller launcher read `OMNISIM_HOME` only, and warn once if they find just the legacy name. Still consumed by the controller **build** Makefiles. | Same as `OMNISIM_HOME` |
| `OMNISIM_DEPENDENCY_PATH` / `WEBOTS_DEPENDENCY_PATH` | Where build dependencies are downloaded | `$OMNISIM_HOME/dependencies` |
| `OMNISIM_CONTROLLER_URL` | Target simulator instance for an **extern** controller, e.g. `ipc://1234/robot_name` or `tcp://<ip>:<port>/robot_name`. WARNING: the `WEBOTS_CONTROLLER_URL` alias was **retired** -- if only the legacy name is set, libController warns once and ignores it. | Most recent local instance |

Note: `WEBOTS_HOME` is the upstream-Webots name. It is still accepted by the **build** Makefiles, but the runtime no longer reads it. New code should write `OMNISIM_HOME`.

## Known issues

- GLM versions newer than 1.0.1 may fail to compile with GCC 15 due to `noexcept` specification mismatches — hence the `git checkout 1.0.1` in step 4.

⚠️ Two entries were removed from this list on 2026-08-28 because they named things that no longer exist. "Java and SWIG controllers are skipped if `JAVA_HOME` is not set": `src/controller/` contains `c/`, `cpp/` and `launcher/` only — there is no Java or SWIG step in the build, and the Python API is a hand-written ctypes binding shipped as source. "The `blimp` sample controller fails to link (missing `-lwinmm`)": no `blimp` controller is tracked in the repo, and no Makefile in the tree references `-lwinmm`.

## Linux quickstart (Ubuntu)

This guide is Windows/MSYS2-first, but **Linux is supported as of v5.1** — verified end-to-end on Ubuntu (WSL2, RTX 5070 Ti): Newton GPU physics confirmed via the backend-verdict sidecar, and a flagship locomotion demo (G1 box delivery) run to completion.

**Budget 25–45 minutes** for the whole documented path on a fresh box: apt 2–6 min, clone 1–3 min, `make release` ~14 min at 4 cores and ~7 min at 24, physics wheels 2–4 min (plus ~2.5 GB and several more minutes if torch is installed — see below). This line used to quote only the "~7-minute build on 24 cores" figure, which is the `make` step alone and reads as the whole install.

**Supported targets: Ubuntu 24.04 and 22.04** (the engine embeds Python **3.12** on both). On 24.04 that is the system interpreter and nothing extra happens. On 22.04 the system python3 is 3.10, where `newton` 1.5.0 raises `TypeError: Union[arg, ...]: each arg must be a type. Got wp.array[wp.bool].` at `ModelBuilder()` — identically on 1.5.1, so no bump fixes it — and an engine that embedded it would build, load worlds, and have *nothing move*, which is far harder to diagnose than a failed install. So the bootstrap's `python` phase installs 3.12 from deadsnakes there, `build` embeds it (and asserts the link on the produced binary with `ldd`), and `gpu` installs the wheels into the interpreter the binary links. The system `python3` keeps running controllers and the CLI — fine at 3.10, controllers never import newton. Ubuntu 26.04 / Python 3.14 passes the version guard but is wheel-fragile. All of it measured by [`physics-runtime-check.yml`](../../.github/workflows/physics-runtime-check.yml) and both legs of `linux-build.yml`; full policy in [system-requirements.md](../guide/system-requirements.md).

Newton's batched-GPU profile (`newtonSolver "mujoco_warp"`) needs an NVIDIA/CUDA GPU; **without one you still get physics** — the default `SolverMuJoCo` runs on the CPU (`mj_step`). ⚠ 2026-08-08: this used to read "without one, worlds run on the ODE CPU fallback" — there is no ODE fallback any more (`bdc02139` deleted `src/ode`); the CPU path *is* MuJoCo.

### One command

```bash
bash scripts/install/linux_bootstrap.sh
```

### Phase by phase (what that command runs)

The script is the recipe — it is the thing CI runs and the thing that gets fixed when a step breaks, so **drive it phase by phase rather than retyping its commands by hand**. Every phase is independently runnable and re-runnable:

```bash
bash scripts/install/linux_bootstrap.sh deps    # apt prerequisites (~35 packages)
bash scripts/install/linux_bootstrap.sh fetch   # clone + glm 1.0.1 / stb submodules
bash scripts/install/linux_bootstrap.sh wgpu    # wgpu-native: the ONLY renderer
bash scripts/install/linux_bootstrap.sh build   # make release (fetches its own Qt 6.5.3)
bash scripts/install/linux_bootstrap.sh gpu     # the pinned physics stack -> the LINKED python
bash scripts/install/linux_bootstrap.sh smoke   # Xvfb headless run + sidecar check
bash scripts/install/linux_bootstrap.sh all     # everything, in order (the default)
```

What each phase is *for*, and the trap it exists to avoid:

- **`deps`** — about 35 apt packages, not the half-dozen this section used to list as "what the script does". Several are non-obvious and each was added after a measured failure: `libdbus-1-dev` (the vendored `libQt6DBus` needs it **at link time**; without it the build dies after ~6 minutes of compiling), `libvulkan1` + `mesa-vulkan-drivers` (a software Vulkan adapter for wgpu-native — a wgpu-native failure is a non-unwinding Rust panic across the C FFI boundary, so it aborts the process rather than degrading), the `libxcb-*` set the Qt xcb plugin dlopens, and `python3-dev` (the engine embeds CPython). Do **not** apt-install Qt6 dev packages: the build vendors its own Qt 6.5.3 and mixing system headers with the vendored libs causes a version clash.
- **`python`** — a >=3.11 interpreter for the engine to embed, where the distro lacks one. On 24.04 it is a no-op; on 22.04 it installs Python 3.12 from the deadsnakes PPA (plus pip via ensurepip), because `newton` raises at `ModelBuilder()` on the system 3.10 and the `build` phase would refuse to link it.
- **`fetch`** — `git clone` plus glm pinned to 1.0.1 and stb from the `omichel` `patch-1` branch, and it re-applies exec bits on `scripts/**/*.sh`.
- **`wgpu`** — runs `scripts/dev/setup_wgpu_native.sh`. **This is the step a hand-written recipe omits and the one that costs you the renderer.** wgpu-native is the only renderer since the WREN deletion (`976b9449d`), the link against it is conditional on `WGPU_NATIVE_HOME` resolving, and without it the build is green and nothing draws. `phase_build` now hard-fails if `lib/webots/libwgpu_native.so` is missing afterwards, so this cannot fail silently any more.
- **`build`** — `make -j$JOBS release`. Qt 6.5.3 arrives automatically via `aqtinstall` from Qt's own servers, not from apt. `JOBS` defaults to `min(nproc, 32)`, because containers report the **host's** core count (a RunPod pod can claim 112) and a bare `-j$(nproc)` massively over-parallelises.
- **`gpu`** — the physics wheels. Two things here are not guessable and are the reason to use the phase rather than a `pip install` line of your own: (1) the wheels must land in **the interpreter the binary links**, which the phase reads out of `ldd bin/omnisim-bin` — not a venv (the embedded interpreter ignores venvs) and not necessarily `python3` on `PATH` (ML cloud images repoint `/usr/bin/python3` at their own build while apt's `python3-dev` still belongs to the distro's); and (2) the **controllers** run in a *different* process from a *different* interpreter, so the phase installs `onnxruntime` there too and then **asserts** the import. Skip that assert and every ONNX deploy controller silently runs with **zero residual**, prints one warning, and exits 0 — a passing demo in which the policy under test never ran.
- **`smoke`** — the acceptance test: a demo world under `xvfb-run` with `OMNISIM_REQUIRE_NEWTON=1`, then the backend-verdict sidecar is checked for `degraded: false` and `finalised: true`. It retries once with a 300 s window if the runtime clearly loaded but finalize was not reached, because warp compiles its CUDA kernels on first use (measured: minutes on a pristine pod).

If you do want the literal commands — for a container image, say — take them from the script. These are the ones most often written wrongly:

```bash
# The pinned physics stack. `pip install torch warp-lang newton mujoco mujoco-warp`
# fails outright on Ubuntu 24.04 with "error: externally-managed-environment", and
# even with the flag it under-installs. Every element below is load-bearing:
#   --break-system-packages : PEP 668 (24.04's pip refuses to touch the system env)
#   --ignore-installed      : apt's own python3-typing-extensions has no RECORD
#                             file, so pip cannot uninstall it to upgrade and the
#                             whole install aborts
#   the == pins             : scripts/packaging/newton_runtime_pins.py is the single
#                             source of truth; an unpinned stack silently desyncs
#                             train==deploy (a 2026-07-17 pod died this way)
#   onnxruntime             : the CONTROLLERS' hard inference dependency
sudo -H python3 -m pip install --break-system-packages --ignore-installed \
  warp-lang==1.16.0 mujoco-warp==3.11.0 mujoco==3.11.0 newton==1.5.0 \
  usd-core==26.5 newton-usd-schemas==0.5.0 \
  numpy onnx onnxscript onnxruntime

# torch is TRAINING-ONLY -- `import torch` appears nowhere under src/ or
# lib/controller/ -- and it is ~2.5 GB from the CUDA index, by far the largest
# item in the install. The gpu phase installs it only when nvidia-smi sees a
# device; skip it entirely if you are not training.
sudo -H python3 -m pip install --break-system-packages --ignore-installed \
  torch --index-url https://download.pytorch.org/whl/cu128

# Load check under Xvfb (mandatory: a Qt/XCB context is created even with
# --no-rendering). --until-finalized stops the moment Newton finalises and the
# sidecar exists, so it neither sleeps out a guessed --duration nor ends before
# the evidence exists. Its ceiling defaults to 30 s -- and when the engine has
# named the mujoco_warp (GPU) path and not finalised yet, the runner announces
# an extension of up to a further 180 s, because warp compiles its CUDA kernels
# on first use. On a PRISTINE GPU box that first compile can still outrun the
# extension (minutes, measured on a fresh RunPod A4000); the bootstrap's smoke
# phase handles it by retrying once with a 300 s window.
xvfb-run -a python3 -m omnisim run-headless \
  projects/samples/demos/worlds/physics/newton_smoke_test.omniworld --until-finalized

# Verify Newton actually drove the run -- read the sidecar, not the log
cat omnisim_log.txt.newton.json
# expect: {"backend":"newton","degraded":false,"finalised":true,"solver":"MuJoCo (...)"}
```

### Runtime environment when invoking `bin/omnisim-bin` directly

Every `python -m omnisim` verb that spawns the engine — `run-world`, `run-headless`, `run-agent`, `test-world`, the harness and the capture service — now sets these for you: `omnisim_env()` in `omnisim/dev/runner.py` applies `linux_runtime_env()` once, for all callers, and it is a no-op off Linux. ⚠️ That was **not** true until 2026-08-28: `run-headless`, the harness and capture each called `linux_runtime_env` for themselves, while `run-world`, `run-agent` and `test-world` — the GUI verbs README and BETA lead with — did not, so they aborted with `version 'Qt_6.10' not found` when a system Qt was installed. Trust the spawners now; you still need these by hand if you exec `bin/omnisim-bin` (or the `webots`/`omnisim` launcher shell is bypassed) yourself:

```bash
export LD_LIBRARY_PATH=$OMNISIM_HOME/lib/webots   # real shipped path; the launcher exports this exact directory
export QT_QPA_PLATFORM=xcb
export OMNISIM_TMPDIR=/tmp                        # WEBOTS_TMPDIR is still read as a legacy alias
export LIBGL_ALWAYS_SOFTWARE=1
```

### Gotchas

- **Qt XCB platform plugin fails to load** (`Could not load the Qt platform plugin "xcb"`): the plugin needs transitive XCB libraries that `qt6-base` doesn't pull in on Ubuntu. The `deps` phase already installs these, so you should only hit this if you assembled the dependencies by hand:

  ```bash
  sudo apt install libxcb-cursor0 libxcb-cursor-dev libxcb-icccm4 \
    libxcb-image0 libxcb-keysyms1 libxcb-render-util0
  ```

- **Shell scripts from a non-git copy** (tarball/zip) may lose their exec bits: `find "$OMNISIM_HOME"/scripts -name "*.sh" -exec chmod +x {} \;`. (Git clones are fine as of v5.1 — the exec bits are tracked.)
- **Demo launchers**: the `bash projects/policies/demos/*.sh` launchers run on Linux as-is; the `scripts/dev/run_*deploy*.ps1` launchers get `.sh` siblings as of v5.1.
- The RL trainers (`projects/policies/training/run_walk_rl.sh` etc.) run on Linux; their early-stop watchdog was MSYS-only and is being fixed — trainings themselves work.
- **`ffmpeg` is needed for movie encoding** (`sudo apt install ffmpeg`): the capture service's `/capture/sequence` renders PNG frames and then shells out to ffmpeg for the mp4/webm/ProRes encode. Stills and screenshots work without it.
- **Running as root prints a warning on every run.** Normal on cloud pods and default-root WSL setups — it's harmless; add a non-root user if you want it gone.
