# OmniSim Build and Iteration Guide

This guide explains how to keep the edit-build-run loop as short as possible without lying about what the current make-based build really does.

> ✅ **WREN was DELETED on 2026-08-23** (commit `976b9449d`: `src/wren` + `include/wren` + `src/omnisim/wren`). There is no separate renderer library any more — the wgpu backend compiles **into the engine** from `src/omnisim/render/`, so renderer changes rebuild through `build core` / `build gui`, and `build renderer` refuses with an explanation instead of building nothing. (Doc updated 2026-09-01.)

## Current Build Topology

The top-level build still behaves like a product build:

1. resolve platform dependencies
2. build `src/glad`
3. build `src/omnisim` (which includes the wgpu renderer in `src/omnisim/render`)
4. build controller libraries
5. build resources and projects

The important consequence is that OmniSim is not yet split into small independently linked runtime modules. The public fast-path targets are wrappers over this structure.

## Public Build Targets

### `python -m omnisim build all`

Runs the full product-style build.

Use this when:

- you need a fully rebuilt developer tree
- you changed multiple subsystems
- you are preparing a broad integration change

### `python -m omnisim build core`

Runs the top-level `omnisim_target` path. Today that means:

- dependencies
- `src/glad`
- `src/omnisim`

Use this when:

- you changed runtime code and want a safe rebuild
- you changed code in `src/omnisim` (renderer code in `src/omnisim/render` included)
- you do not want to rely on a stale static library or stale dependency artifact

### `python -m omnisim build gui`

Builds only `src/omnisim`.

Use this when:

- you changed files under `src/omnisim` (including renderer code under `src/omnisim/render`)
- you did not change `src/glad`
- you want the narrowest practical rebuild of the main simulator binary

### `python -m omnisim build renderer` — RETIRED (refuses)

⚠️ Its recipe was `make -C src/wren`, deleted with WREN on 2026-08-23. As of 2026-08-24 the subcommand **refuses with an explanation and a non-zero exit** rather than routing to a target with no recipe behind it (which printed `Nothing to be done` and exited 0 — a build command reporting success while building nothing). There is no separate renderer subsystem: rebuild renderer code with `build core` or `build gui`.

### `python -m omnisim build controller-libs`

Builds only the controller libraries and launcher.

Use this when:

- you changed `src/controller`
- you changed `include/controller`
- you are iterating on controller APIs or launcher behavior

### `python -m omnisim build package`

Runs the packaging path. This is not part of the normal fast inner loop.

## Equivalent Make Targets

These wrapper targets exist at the repository root:

- `make sim-core`
- `make sim-gui`
- `make renderer` — ⚠️ a dead name: it survives in the top-level Makefile's `.PHONY` list with no recipe behind it, prints `Nothing to be done for 'renderer'` and exits 0. Do not use it; renderer code rebuilds through `make sim-core` / `make sim-gui`.
- `make controller-libs`
- `make tests-smoke`
- `make benchmarks`
- `make compile-commands`

They are convenience names, not proof that the architecture is already modular.

## Rebuild Matrix

### If you changed `src/omnisim/gui`, `src/omnisim/editor`, `src/omnisim/scene_tree`, or `src/omnisim/widgets`

Narrowest rebuild:

```bash
python -m omnisim build gui
```

### If you changed `src/omnisim/nodes`, `src/omnisim/engine`, `src/omnisim/control`, or `src/omnisim/vrml`

Narrowest rebuild:

```bash
python -m omnisim build gui
```

Safer rebuild:

```bash
python -m omnisim build core
```

Use the safer rebuild if the change touches runtime behavior that depends on generated resource state or static-link integration details.

### If you changed `src/omnisim/render` (the wgpu renderer)

Renderer code is part of the engine target, so this is the ordinary engine loop:

```bash
python -m omnisim build gui
```

Safer rebuild:

```bash
python -m omnisim build core
```

### If you changed `src/controller` or `include/controller`

Use:

```bash
python -m omnisim build controller-libs
```

### If you changed test harnesses or world files only

Usually no rebuild is required. Run validation directly with `--nomake` when appropriate.

## Practical Inner Loops

### Renderer loop

```bash
python -m omnisim build gui
python -m omnisim test-world tests/rendering/worlds/normals.omniworld --nomake
```

### Runtime loop

```bash
python -m omnisim build gui
python -m omnisim test-smoke --nomake
```

If the change is broad or touches physics or controller synchronization, use `build core` instead of `build gui`.

### Controller API loop

```bash
python -m omnisim build controller-libs
python -m omnisim test-group api --nomake
```

### One-world investigation loop

```bash
python -m omnisim build gui
python -m omnisim run-world tests/api/worlds/accelerometer.omniworld
```

### Headless automation loop

```bash
python -m omnisim build gui
python -m omnisim run-headless tests/api/worlds/accelerometer.omniworld
```

## Compile Database and Tooling

### `compile_commands.json`

Generate it with:

```bash
python -m omnisim compile-commands
```

Current behavior:

- this relies on `bear`
- it wraps the `omnisim_target` path
- it is the best current way to give editors and coding agents a compile database without replacing the build system

### `ccache`

If `ccache` is installed, the top-level makefile uses it by default.
`CCACHE_BASEDIR` defaults to the OmniSim checkout root so release objects can be
reused across equivalent checkout paths.  Override it in the environment if a
different cache boundary is required.

`CCACHE_DIR` defaults to `.build_tmp/ccache` inside the checkout. This keeps the
cache writable for coding agents and sandboxed IDEs that can modify the
workspace but cannot write the user's global AppData cache. Export a different
`CCACHE_DIR` before make if a shared global cache is preferred.

Inspect the active configuration, hit/miss counts, and uncacheable reasons with:

```bash
make ccache-stats
```

Dependency files are emitted during normal compilation with `-MMD -MP`.  Before
this change, make ran a separate `-MM` compiler process for every C++ source.
Those preprocessing-only processes did no compilation, doubled front-end work,
and appeared in ccache statistics as uncacheable/unsupported-option calls (the
exact label depends on ccache version).  On the current Windows simulator target,
the build graph has 617 compiled object sources (including generated MOC sources):
compiler processes for a clean engine compile therefore fall from 1,233 to 617,
a 50.0% reduction.  The generated `.d` files use `-MP` phony header targets, so
header deletion and incremental rebuild behavior remain safe.

The simulator's hand-written makefile deliberately emits explicit `-MF` and `-MT`
arguments.  Its logical object targets are basenames resolved into `build/<mode>`
through `vpath`; therefore a dependency target must be `Foo.o`, not
`build/release/Foo.o`.  Generated MOC rules are also restricted to the declared
MOC file set so `-MP` phony headers cannot create speculative nested MOC targets.
Simulator depfiles live under `build/<mode>/.deps`.  On the first incremental
build after this format change, one shell migrates legacy depfiles in a batch and
writes a stamp; this measured 5.0 seconds for 610 files on Windows, versus 34--66
seconds when each file launched its own MSYS recipe.  Later dependency evaluation
measured 0.8 seconds and does not rerun the migration.

Recompute those structural counts from the current source lists without cleaning
or compiling anything:

```bash
make -C src/omnisim dependency-stats
```

ccache statistics are cumulative.  Existing unsupported-option counts do not
disappear after updating the makefiles.  To measure a clean before/after window:

```bash
ccache --zero-stats
make -j8 sim-gui BUNDLE_NEWTON=0
make ccache-stats
```

Disable it with:

```bash
USE_CCACHE=0 make -j8 release
```

### Agent inner loop and staged linking

```bash
python -m omnisim build changed                 # affected objects only
python -m omnisim build changed --link          # compile, then normal link
python -m omnisim build changed --link --staged # link omnisim-bin.next.exe
python -m omnisim build activate-staged         # explicit handoff when GUI is closed
```

`build changed` compares the working tree to `HEAD` by default (`--base` changes
that) and maps edited headers/sources through `build/release/.deps/*.d`. It does
not relink unless asked. Build-system edits conservatively select a full GUI
build because flags and source membership may have changed.

All public logical targets pass through the top-level Makefile, so `build gui`
and `controller-libs` do not bypass ccache. On Windows the CLI adds
the compiler and MSYS make directories to `PATH`, and carries a checkout-local
`_scratch/wgpu-native` SDK into incremental relinks when present.

Default parallelism is capped at 12 non-oversubscribed jobs. Override it with
`--jobs N` or `OMNISIM_BUILD_JOBS=N`.

Optional toolchain accelerators:

```bash
OMNISIM_USE_PCH=1 make sim-gui       # stable standard-library PCH
OMNISIM_LINKER=lld make sim-gui      # LLVM lld, when ld.lld is installed
```

The PCH is opt-in because a warm ccache hit is cheaper. An explicit lld request
fails clearly when lld is absent; GNU bfd remains the portable default.

Staged linking writes `msys64/mingw64/bin/omnisim-bin.next.exe`, allowing a build
while the primary GUI is running. Activation retains the old executable as
`omnisim-bin.previous.exe`. The wgpu DLL post-link step hashes source and
destination and skips an identical locked DLL.

### Runtime-cycle overlap

Newton preload begins immediately after file logging initializes and overlaps
CPython/Warp/Newton import with Qt setup, PROTO retrieval, tokenization, and
parsing. The simulator thread waits for the future and adopts the Python GIL
before backend construction. Use `OMNISIM_NEWTON_ASYNC_PRELOAD=0` for the
synchronous control, `OMNISIM_NEWTON_PRELOAD_PROFILE=1` for preload/wait timing,
and `OMNISIM_RELOAD_PROFILE=1` for per-phase world-load timing.

On the reference Windows machine with `gravity_rest_height.omniworld`, synchronous
load phases measured 3,577 ms and early async preload measured 1,751 ms;
construction fell from 3,488 to 1,662 ms (about 1.83 s, 51%). Both staged-binary
runs finalized CPU `mj_step` and passed. This is one-machine evidence, not a
cross-machine guarantee.

## What Still Makes Builds Slow

- `src/omnisim` is still a very large target with a broad include surface
- the renderer compiles inside that same target, so renderer edits pay the engine link
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
