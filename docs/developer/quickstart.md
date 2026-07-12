# OmniSim Developer Quickstart (Windows)

This guide covers everything needed to build and run OmniSim from source on Windows. It assumes a clean machine with no prior MSYS2 or Webots installation.

See also:

- [../../AGENTS.md](../../AGENTS.md) — agent-facing TL;DR (build, launch, headless run, HTTP bridge, validation), in copy-paste form
- [README.md](README.md) for the developer-doc index
- [build-and-iteration.md](build-and-iteration.md) for the narrowest rebuild paths
- [header-hygiene-and-rebuild-reduction.md](header-hygiene-and-rebuild-reduction.md) for keeping rebuild scope under control
- [validation-playbook.md](validation-playbook.md) for change-to-test mapping
- [profiling-playbook.md](profiling-playbook.md) for performance logging and benchmarks

## Prerequisites

- Windows 10/11
- ~10 GB free disk space (MSYS2 + dependencies + build artifacts), plus ~1 GB for the optional Newton runtime bundle
- Git for Windows (for cloning the repo)
- A Windows CPython 3.10+ from [python.org](https://www.python.org/downloads/), installed to the default per-user location. The v4 default build (`OMNISIM_WITH_NEWTON=ON`) embeds CPython and needs its build headers — the Makefile auto-detects the install (override with `PYTHON_HOME=`/`PYTHON_LIB=` make arguments, or build the pure-ODE legacy stack with `OMNISIM_WITH_NEWTON=OFF`). The same Python also runs the dev tooling (`python -m omnisim`, the harness, `omniworld`).

> **A note on paths.** This guide assumes you have set `OMNISIM_HOME` (canonical) — or, equivalently, `WEBOTS_HOME` (legacy alias accepted everywhere) — to the absolute path of your local checkout. The scripts and the build system never assume a fixed install location — set this once per shell and the rest of the commands work unchanged. The bundled `build_omni.bat` derives both names from its own location, so on Windows you usually do not need to export anything manually.
>
> ```bash
> # MSYS2 MINGW64 terminal — run from the root of your clone:
> export OMNISIM_HOME=$(pwd)
> export WEBOTS_HOME=$OMNISIM_HOME    # legacy alias; both names are accepted
> ```
>
> ```bat
> :: cmd.exe — adjust to wherever you cloned the repo:
> set OMNISIM_HOME=C:\path\to\omnisim
> set WEBOTS_HOME=%OMNISIM_HOME%
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
| ODE | `src/ode/ode.a` | Physics engine (static lib) |
| GLAD | `src/glad/glad.a` | OpenGL loader (static lib) |
| Wren | `src/wren/wren.a` | Graphics renderer (static lib) |
| omnisim-bin | `msys64/mingw64/bin/omnisim-bin.exe` (alias: `webots-bin.exe`) | Main simulator binary |
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
launch.bat projects\samples\demos\worlds\showcase\warehouse_husky.wbt   REM specific world
```

### From the MSYS2 terminal

```bash
export PATH="$OMNISIM_HOME"/msys64/mingw64/bin:/mingw64/bin:$PATH
"$OMNISIM_HOME"/msys64/mingw64/bin/omnisim-bin.exe
```

### Open a specific world

```bash
"$OMNISIM_HOME"/msys64/mingw64/bin/omnisim-bin.exe "$OMNISIM_HOME"/projects/samples/demos/worlds/showcase/warehouse_husky.wbt
```

### Headless run (no window)

For agent loops, CI, or any time you don't want an OmniSim window to open, use the headless runner. It launches `omnisim-bin.exe` with `--minimize --batch --no-rendering --mode=fast --stdout --stderr` and stops the sim after the duration (`--no-window` is deliberately *not* used: it skips main-window realization but deadlocks Newton's embedded CPython FFI on multi-articulation worlds, so `--minimize` is the safe headless default):

```bash
python scripts/dev/headless_runner.py projects/samples/demos/worlds/showcase/turtlebot3_drive.wbt --duration 10
```

It writes the engine log to `omnisim_log.txt`, tails it for errors and warnings, and returns a structured exit code (0 PASS / non-zero FAIL). See the [README](../../README.md#headless-run-no-window--ideal-for-ai-agents) for the full argument list and the rationale.

### Demo worlds to try

| World | Path | Description |
|-------|------|-------------|
| **Warehouse Husky** *(default)* | `projects/samples/demos/worlds/showcase/warehouse_husky.wbt` | Onboarding demo. Husky random-walks a warehouse with reactive collision recovery. Try `click + F` to follow it, `LMB + WASD` to fly. |
| Husky maze | `projects/samples/demos/worlds/flagship/husky_maze.wbt` | Single Husky in a maze — classic navigation testbed |
| Husky fleet arena | `projects/samples/demos/worlds/showcase/husky_fleet_arena.wbt` | 10 Huskies random-walking a walled arena; tests multi-robot collision recovery |
| Generated Mars | `distribution/generated_worlds/mars.wbt` | Procedurally generated planetary terrain with Husky fleet (regenerable via `omniworld`) |

For an agent-driven workflow (headless, structured exit, supported run contract):

```bash
python scripts/dev/omnisim_dev.py run-headless projects/samples/demos/worlds/showcase/warehouse_husky.wbt --duration 10
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
g++ -o "$OMNISIM_HOME"/msys64/mingw64/bin/webots-debug.exe \
  build/release/*.o -Wl,--enable-auto-import \
  -L"$OMNISIM_HOME"/msys64/mingw64/bin -L/mingw64/bin -L/mingw64/lib \
  ../wren/wren.a ../glad/glad.a \
  -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets \
  -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml \
  -lode -lopenal -lopengl32 -liphlpapi -ld3d9 -lgdi32 -lglu32 \
  -lOIS -ldinput8 -ldxguid -lole32 -lsapi -loleaut32 -luuid \
  -lpico -lfreetype-6 -lopenvr_api -lassimp-5
```

The difference: `omnisim-bin.exe` (and its legacy alias `webots-bin.exe`) is a Windows GUI app (no console). `webots-debug.exe` keeps the console attached so you can see stderr/stdout directly.

## 9. Subsystem map

```
src/
  ode/          Physics engine (Open Dynamics Engine). Rarely needs changes.
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
    ode/        ODE integration layer
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
python scripts/dev/omnisim_dev.py test-world tests/api/worlds/accelerometer.wbt --nomake
python scripts/dev/omnisim_dev.py run-headless tests/api/worlds/accelerometer.wbt
python scripts/dev/omnisim_dev.py profile-world tests/rendering/worlds/normals.wbt
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
| `WEBOTS_HOME` | Legacy name. Read only by the headless runner / dev tooling as a fallback when `OMNISIM_HOME` is unset (with a console note), and by the controller **build** Makefiles. Not read by the runtime binary or the Python controller package. | Same as `OMNISIM_HOME` |
| `OMNISIM_DEPENDENCY_PATH` / `WEBOTS_DEPENDENCY_PATH` | Where build dependencies are downloaded | `$OMNISIM_HOME/dependencies` |

Note: `WEBOTS_HOME` is the upstream-Webots name and remains accepted as a legacy alias for compatibility with any third-party tooling that hasn't migrated yet. New code should write `OMNISIM_HOME`.

## Known issues

- Java and SWIG controllers are skipped if `JAVA_HOME` is not set or SWIG is not installed. This is fine for most development.
- The `blimp` sample controller fails to link (missing `-lwinmm`). This is a minor sample issue, not a core simulator problem.
- GLM versions newer than 1.0.1 may fail to compile with GCC 15 due to `noexcept` specification mismatches.

## Linux notes (Ubuntu)

This guide is Windows/MSYS2-first. A full Linux quickstart isn't yet written, but the gotchas a Linux build typically hits — and their fixes — are below. These are reproducible on a clean Ubuntu install.

### Build prerequisites

```bash
# Shell scripts pulled from a non-git copy may not be executable
find "$OMNISIM_HOME"/scripts -name "*.sh" -exec chmod +x {} \;

# OpenAL headers are not installed by default on Ubuntu
sudo apt install libopenal-dev
```

### Runtime (Qt XCB platform plugin)

If launching the simulator fails with:

```
Could not load the Qt platform plugin "xcb"
Fatal: application failed to start
```

…the Qt XCB plugin is missing transitive XCB libraries that aren't pulled in by `qt6-base` on Ubuntu. Install:

```bash
sudo apt install libxcb-cursor0 libxcb-cursor-dev libxcb-icccm4 \
  libxcb-image0 libxcb-keysyms1 libxcb-render-util0
```

After this, the warehouse_husky world launches normally.
