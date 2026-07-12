# OmniSim Build and Iteration Guide

This guide explains how to keep the edit-build-run loop as short as possible without lying about what the current make-based build really does.

## Current Build Topology

The top-level build still behaves like a product build:

1. resolve platform dependencies
2. build `src/ode`
3. build `src/glad`
4. build `src/wren`
5. build `src/omnisim`
6. build controller libraries
7. build resources and projects

The important consequence is that OmniSim is not yet split into small independently linked runtime modules. The public fast-path targets are wrappers over this structure.

## Public Build Targets

### `python scripts/dev/omnisim_dev.py build all`

Runs the full product-style build.

Use this when:

- you need a fully rebuilt developer tree
- you changed multiple subsystems
- you are preparing a broad integration change

### `python scripts/dev/omnisim_dev.py build core`

Runs the top-level `webots_target` path. Today that means:

- dependencies
- `src/ode`
- `src/glad`
- `src/wren`
- `src/omnisim`

Use this when:

- you changed runtime code and want a safe rebuild
- you changed code in `src/ode`, `src/wren`, or `src/omnisim`
- you do not want to rely on a stale static library or stale dependency artifact

### `python scripts/dev/omnisim_dev.py build gui`

Builds only `src/omnisim`.

Use this when:

- you changed files under `src/omnisim`
- you did not change `src/ode`, `src/glad`, or `src/wren`
- you want the narrowest practical rebuild of the main simulator binary

### `python scripts/dev/omnisim_dev.py build renderer`

Builds only `src/wren`.

Use this when:

- you are iterating on the renderer library itself
- you want to catch compile errors quickly in `src/wren`

Important:

- `build renderer` updates the static renderer library
- it does not by itself relink the desktop simulator binary
- after renderer changes that affect the running simulator, follow with `build gui` or `build core`

### `python scripts/dev/omnisim_dev.py build controller-libs`

Builds only the controller libraries and launcher.

Use this when:

- you changed `src/controller`
- you changed `include/controller`
- you are iterating on controller APIs or launcher behavior

### `python scripts/dev/omnisim_dev.py build package`

Runs the packaging path. This is not part of the normal fast inner loop.

## Equivalent Make Targets

These wrapper targets exist at the repository root:

- `make sim-core`
- `make sim-gui`
- `make renderer`
- `make controller-libs`
- `make tests-smoke`
- `make benchmarks`
- `make compile-commands`

They are convenience names, not proof that the architecture is already modular.

## Rebuild Matrix

### If you changed `src/omnisim/gui`, `src/omnisim/editor`, `src/omnisim/scene_tree`, or `src/omnisim/widgets`

Narrowest rebuild:

```bash
python scripts/dev/omnisim_dev.py build gui
```

### If you changed `src/omnisim/nodes`, `src/omnisim/engine`, `src/omnisim/control`, or `src/omnisim/vrml`

Narrowest rebuild:

```bash
python scripts/dev/omnisim_dev.py build gui
```

Safer rebuild:

```bash
python scripts/dev/omnisim_dev.py build core
```

Use the safer rebuild if the change touches runtime behavior that depends on `src/wren`, `src/ode`, generated resource state, or static-link integration details.

### If you changed `src/wren`

Fast compile-only check:

```bash
python scripts/dev/omnisim_dev.py build renderer
```

Runnable simulator rebuild:

```bash
python scripts/dev/omnisim_dev.py build renderer
python scripts/dev/omnisim_dev.py build gui
```

Safer rebuild:

```bash
python scripts/dev/omnisim_dev.py build core
```

### If you changed `src/ode`

Use:

```bash
python scripts/dev/omnisim_dev.py build core
```

There is no narrower public target yet.

### If you changed `src/controller` or `include/controller`

Use:

```bash
python scripts/dev/omnisim_dev.py build controller-libs
```

### If you changed test harnesses or world files only

Usually no rebuild is required. Run validation directly with `--nomake` when appropriate.

## Practical Inner Loops

### Renderer loop

```bash
python scripts/dev/omnisim_dev.py build renderer
python scripts/dev/omnisim_dev.py build gui
python scripts/dev/omnisim_dev.py test-world tests/rendering/worlds/normals.wbt --nomake
```

### Runtime loop

```bash
python scripts/dev/omnisim_dev.py build gui
python scripts/dev/omnisim_dev.py test-smoke --nomake
```

If the change is broad or touches physics or controller synchronization, use `build core` instead of `build gui`.

### Controller API loop

```bash
python scripts/dev/omnisim_dev.py build controller-libs
python scripts/dev/omnisim_dev.py test-group api --nomake
```

### One-world investigation loop

```bash
python scripts/dev/omnisim_dev.py build gui
python scripts/dev/omnisim_dev.py run-world tests/api/worlds/accelerometer.wbt
```

### Headless automation loop

```bash
python scripts/dev/omnisim_dev.py build gui
python scripts/dev/omnisim_dev.py run-headless tests/api/worlds/accelerometer.wbt
```

## Compile Database and Tooling

### `compile_commands.json`

Generate it with:

```bash
python scripts/dev/omnisim_dev.py compile-commands
```

Current behavior:

- this relies on `bear`
- it wraps the `webots_target` path
- it is the best current way to give editors and coding agents a compile database without replacing the build system

### `ccache`

If `ccache` is installed, the top-level makefile uses it by default.

Disable it with:

```bash
USE_CCACHE=0 make -j8 release
```

## What Still Makes Builds Slow

- `src/omnisim` is still a very large target with a broad include surface
- the main simulator links statically against `src/wren`
- many headers pull in more of the product than they need
- runtime, desktop shell, parser, and renderer code still share a large build boundary

## Developer Rules for Faster Rebuilds

- prefer forward declarations over new header includes where possible
- avoid moving GUI dependencies into runtime-facing headers
- keep changes out of vendored trees unless the task is explicitly about vendored code
- use `build gui` or `build controller-libs` before reaching for a full build
- use `--nomake` on validation commands when the current tree is already built

## What We Want Later

The current target names are the public interface we want to preserve. Over time they should stop being wrappers over the legacy make graph and become true narrow targets.
