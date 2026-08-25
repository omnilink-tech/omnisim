# OmniSim Rollback and Recovery Guide

This document explains how to return to a known-good state if something breaks during development.

## Tagged Baselines

| Tag | Description | Date |
|-----|-------------|------|
| `v0.1.0-baseline` | Working build with file logging, local asset resolution, developer docs, and all textures loading correctly. | 2026-04-11 |

## How to roll back

### Option 1: Check out the tag (non-destructive)

This puts you in a detached HEAD state. Your current branch is untouched.

```bash
git checkout v0.1.0-baseline
```

To go back to your branch afterwards:

```bash
git checkout main
```

### Option 2: Reset the branch to the tag (destructive to commits after the tag)

This moves `main` back to the tagged commit. All commits after the tag on this branch are discarded.

```bash
git reset --hard v0.1.0-baseline
```

If you already pushed newer commits to GitHub and need to force the remote back:

```bash
git push --force origin main
```

### Option 3: Create a new branch from the tag

This preserves all current work on `main` while giving you a clean branch from the baseline.

```bash
git checkout -b fresh-start v0.1.0-baseline
```

## After rolling back: rebuild

After any rollback, you need to rebuild the simulator since the source files changed but the compiled binaries in `msys64/mingw64/bin/` are still from the old build.

```bash
# From MSYS2 MINGW64 terminal, cd to your OmniSim checkout root first:
export OMNISIM_HOME="$(cygpath -w "$PWD")"  # Linux/macOS: just $PWD
make -C src/omnisim clean
make -j$(nproc) release
```

Then relink the debug version if you use it (run from the repo root):

```bash
cd src/omnisim
g++ -o ../../msys64/mingw64/bin/omnisim-bin.exe \
  build/release/*.o -Wl,--enable-auto-import \
  -L../../msys64/mingw64/bin -L/mingw64/bin -L/mingw64/lib \
  ../wren/wren.a ../glad/glad.a \
  -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets \
  -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml \
  -lode -lopenal -lopengl32 -liphlpapi -ld3d9 -lgdi32 -lglu32 \
  -lOIS -ldinput8 -ldxguid -lole32 -lsapi -loleaut32 -luuid \
  -lpico -lfreetype-6 -lopenvr_api -lassimp-5
```

## Creating new baselines

Before starting a major change, create a new tagged baseline:

```bash
git tag -a v0.X.0-description -m "Description of what works at this point."
git push origin --tags
```

## Verifying a baseline works

After checking out any baseline, confirm it works:

1. Build: from the repo root, `make -j$(nproc) release`
2. Launch: `launch.bat` (from the repo root)
3. Check log: `cat omnisim_log.txt` (should have zero warnings)
4. Confirm textures render in the 3D view
