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
- A Windows CPython 3.10+ from [python.org](https://www.python.org/downloads/), installed to the default per-user location. The v4 default build (`OMNISIM_WITH_NEWTON=ON`) embeds CPython and needs its build headers — the Makefile auto-detects the install (override with `PYTHON_HOME=`/`PYTHON_LIB=` make arguments). ⚠ The embedded interpreter is **mandatory**, not a choice: `bdc02139` deleted `src/ode`, so Newton with `SolverMuJoCo` is the only physics backend and there is no pure-ODE legacy stack to fall back to — `OMNISIM_WITH_NEWTON=OFF` leaves the engine with no physics at all. The same Python also runs the dev tooling (`python -m omnisim`, the harness, `omniworld`).

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
  unzip \
  zip
```

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
| GLAD | `src/glad/glad.a` | OpenGL loader (static lib) |
| Wren | `src/wren/wren.a` | Graphics renderer (static lib) |
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

For agent loops, CI, or any time you don't want an OmniSim window to open, use the headless runner. It launches `omnisim-bin.exe` with `--minimize --batch --no-rendering --mode=fast --stdout --stderr` and stops the sim after the duration (`--no-window` is deliberately *not* used: it skips main-window realization but deadlocks Newton's embedded CPython FFI on multi-articulation worlds, so `--minimize` is the safe headless default):

```bash
python scripts/dev/headless_runner.py projects/samples/demos/worlds/showcase/turtlebot3_drive.omniworld --duration 10
```

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
python scripts/dev/omnisim_dev.py run-headless projects/samples/demos/worlds/showcase/warehouse_husky.omniworld --duration 10
```

To regenerate a procedural world:

```bash
python scripts/dev/omniworld.py list-recipes
python scripts/dev/omniworld.py generate mars --seed 42 --out my_mars.wbt
launch.bat my_mars.wbt
```

## 8. Debug and log output

All warnings and errors are logged to `omnisim_log.txt` in the project root. Check this file after launching to diagnose issues:

```bash
cat "$OMNISIM_HOME"/omnisim_log.txt
```

To build a console-attached debug version (shows output in terminal, useful for development):

```bash
cd "$OMNISIM_HOME"/src/omnisim
g++ -o "$OMNISIM_HOME"/msys64/mingw64/bin/omnisim-debug.exe \
  build/release/*.o -Wl,--enable-auto-import \
  -L"$OMNISIM_HOME"/msys64/mingw64/bin -L/mingw64/bin -L/mingw64/lib \
  ../wren/wren.a ../glad/glad.a \
  -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets \
  -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml \
  -lode -lopenal -lopengl32 -liphlpapi -ld3d9 -lgdi32 -lglu32 \
  -lOIS -ldinput8 -ldxguid -lole32 -lsapi -loleaut32 -luuid \
  -lpico -lfreetype-6 -lopenvr_api -lassimp-5
```

The difference: `omnisim-bin.exe` is a Windows GUI app (no console). `omnisim-debug.exe` keeps the console attached so you can see stderr/stdout directly.

## 9. Subsystem map

```
src/
  glad/         OpenGL function loader. Rarely needs changes.
  wren/         3D rendering engine. Edit here for graphics/shader work.
  glm/          Math library (submodule). Do not edit directly.
  stb/          Image loading (submodule). Do not edit directly.
  controller/   Controller APIs (C, C++, Java, Python).
                Edit here for robot programming interface changes.
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
    plugins/    Plugin loading system
    scene_tree/ Scene tree widget and property editors
    sound/      Audio system
    user_commands/ Undo/redo, action manager
    vrml/       VRML/PROTO parser, tokenizer, node model, URL resolution
    wren/       Rendering integration (camera effects, overlays, context)
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
python scripts/dev/omnisim_dev.py --help
python scripts/dev/omnisim_dev.py build core
python scripts/dev/omnisim_dev.py build renderer
python scripts/dev/omnisim_dev.py test-smoke
python scripts/dev/omnisim_dev.py test-world tests/api/worlds/accelerometer.omniworld --nomake
python scripts/dev/omnisim_dev.py run-headless tests/api/worlds/accelerometer.omniworld
python scripts/dev/omnisim_dev.py profile-world tests/rendering/worlds/normals.omniworld
```

Equivalent make aliases are also available:

```bash
make sim-core
make sim-gui
make renderer
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

### "I changed the renderer" (e.g., edited a file in `src/wren/`)

```bash
make -C "$OMNISIM_HOME"/src/wren release
make -C "$OMNISIM_HOME"/src/omnisim -j$(nproc) release  # relink
```

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

- Java and SWIG controllers are skipped if `JAVA_HOME` is not set or SWIG is not installed. This is fine for most development.
- The `blimp` sample controller fails to link (missing `-lwinmm`). This is a minor sample issue, not a core simulator problem.
- GLM versions newer than 1.0.1 may fail to compile with GCC 15 due to `noexcept` specification mismatches.

## Linux quickstart (Ubuntu)

This guide is Windows/MSYS2-first, but **Linux is supported as of v5.1** — verified end-to-end on Ubuntu (WSL2, RTX 5070 Ti): ~7-minute build on 24 cores, Newton GPU physics confirmed via the backend-verdict sidecar, and a flagship locomotion demo (G1 box delivery) run to completion.

**Recommended targets: Ubuntu 22.04 / 24.04** (Python 3.10 / 3.12 — the safest targets for the Newton GPU wheels; RunPod-style cloud pods on these images are the smoothest path). Ubuntu 26.04 / Python 3.14 works today but is wheel-fragile. Newton's batched-GPU profile (`newtonSolver "mujoco_warp"`) needs an NVIDIA/CUDA GPU; **without one you still get physics** — the default `SolverMuJoCo` runs on the CPU (`mj_step`). ⚠ 2026-08-08: this used to read "without one, worlds run on the ODE CPU fallback" — there is no ODE fallback any more (`bdc02139` deleted `src/ode`); the CPU path *is* MuJoCo.

### One command

```bash
bash scripts/install/linux_bootstrap.sh
```

The bootstrap script (v5.1) does the whole recipe: apt dependencies → `git clone --recurse-submodules` → `make release` (Qt 6.5.3 arrives automatically via `aqtinstall` from Qt's own servers — not from apt) → the GPU wheels pip-installed into the system `python3` → an Xvfb headless smoke run with the Newton sidecar check.

### Manual steps (what the script does)

```bash
# 1. apt dependencies (python3-dev is required — the engine embeds CPython)
sudo apt install build-essential git python3-dev python3-pip libopenal-dev xvfb

# 2. Clone with submodules
git clone --recurse-submodules https://github.com/omnilink-tech/omnisim.git
cd omnisim && export OMNISIM_HOME=$(pwd)

# 3. Build (Qt 6.5.3 is fetched automatically via aqtinstall during the build)
make -j$(nproc) release

# 4. Newton GPU wheels — into the SYSTEM python3, NOT a venv.
#    The engine's embedded interpreter (bare Py_InitializeEx) resolves the system
#    python3's sys.path and ignores virtualenvs; wheels in a venv are invisible
#    to it, and since bdc02139 deleted src/ode there is nothing left to fall
#    back to -- the engine has NO physics backend and fails hard instead of
#    quietly downgrading. The Windows runtime bundle is not involved on Linux.
pip install torch warp-lang newton mujoco mujoco-warp

# 5. Run headless under Xvfb (mandatory: a Qt/XCB context is created even with
#    --no-rendering)
xvfb-run -a python scripts/dev/omnisim_dev.py run-headless \
  projects/samples/demos/worlds/physics/newton_smoke_test.omniworld --duration 15

# 6. Verify Newton actually drove the run — read the sidecar, not the log
cat omnisim_log.txt.newton.json
# expect: {"backend":"newton","degraded":false,"finalised":true,"solver":"MuJoCo (...)"}
```

### Runtime environment when invoking `bin/omnisim-bin` directly

The spawners (`omnisim_dev.py`, the demo launchers) are absorbing these, but if you exec the binary yourself you need:

```bash
export LD_LIBRARY_PATH=$OMNISIM_HOME/lib/webots   # real shipped path; the launcher exports this exact directory
export QT_QPA_PLATFORM=xcb
export OMNISIM_TMPDIR=/tmp                        # WEBOTS_TMPDIR is still read as a legacy alias
export LIBGL_ALWAYS_SOFTWARE=1
```

### Gotchas

- **Qt XCB platform plugin fails to load** (`Could not load the Qt platform plugin "xcb"`): the plugin needs transitive XCB libraries that `qt6-base` doesn't pull in on Ubuntu:

  ```bash
  sudo apt install libxcb-cursor0 libxcb-cursor-dev libxcb-icccm4 \
    libxcb-image0 libxcb-keysyms1 libxcb-render-util0
  ```

- **Shell scripts from a non-git copy** (tarball/zip) may lose their exec bits: `find "$OMNISIM_HOME"/scripts -name "*.sh" -exec chmod +x {} \;`. (Git clones are fine as of v5.1 — the exec bits are tracked.)
- **Demo launchers**: the `bash projects/policies/demos/*.sh` launchers run on Linux as-is; the `scripts/dev/run_*deploy*.ps1` launchers get `.sh` siblings as of v5.1.
- The RL trainers (`projects/policies/training/run_walk_rl.sh` etc.) run on Linux; their early-stop watchdog was MSYS-only and is being fixed — trainings themselves work.
- **`ffmpeg` is needed for movie encoding** (`sudo apt install ffmpeg`): the capture service's `/capture/sequence` renders PNG frames and then shells out to ffmpeg for the mp4/webm/ProRes encode. Stills and screenshots work without it.
- **Running as root prints a warning on every run.** Normal on cloud pods and default-root WSL setups — it's harmless; add a non-root user if you want it gone.
