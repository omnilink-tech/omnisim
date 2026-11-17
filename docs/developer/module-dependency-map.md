# OmniSim Module Dependency Map

This document explains where code should live today, where the current build graph leaks across subsystem boundaries, and how to keep future changes from making rebuild scope and validation cost worse.

> ✅ **WREN was DELETED on 2026-08-23** (commit `976b9449d`: `src/wren` + `include/wren` + `src/omnisim/wren`). The wgpu backend is compiled into the engine from `src/omnisim/render/` (plus `src/omnisim/nodes/OmWgpuSceneRenderer.*`); there is no separate renderer library or `build renderer` target any more. WREN mentions below that describe the historical leak surface are kept where the lesson still applies. (Banner added 2026-09-01.)

Use it when:

- deciding which directory should own a new feature
- reviewing an include or dependency addition
- splitting code so `build core` can stay narrow
- planning the longer move toward a true simulation-core boundary

## Why This Matters

OmniSim still builds through a large make-based graph. That means directory structure and header dependencies directly control:

- how much code recompiles after a change
- whether a headless or renderer-only change can be validated without the full desktop product
- whether coding agents can reason about the repo as layers instead of as one giant binary

The fastest path is not just "compile faster." It is "compile less."

## Current Dependency Picture

`src/omnisim/Makefile` already contains a useful warning: the module include lists are meant to stay ordered from fewer dependencies to more dependencies. That file is the best compact description of the current dependency graph.

In practice the current layers look like this:

- `maths`: low-level math support
- `core`: common Qt/core services, process state, networking utilities, application-wide helpers
- `util`: mixed support code used by runtime and content handling
- `vrml`: tokenizer, parser, and low-level world description handling
- `render`: the wgpu-native rendering backend (`src/omnisim/render`; the `wren` module was deleted 2026-08-23)
- `physics`: physics backend integration (Newton / `SolverMuJoCo`)
- `plugins`: plugin-side integration hooks
- `engine`: simulation runtime orchestration
- `control`: controller and synchronization plumbing
- `nodes`: scene graph node types, devices, templates, assets, and a large amount of cross-cutting logic
- `app`: top-level application/bootstrap code
- `scene_tree`, `editor`, `widgets`, `gui`: desktop-shell and editing surfaces

That looks reasonable at a distance, but the details matter more than the labels.

## Where The Boundaries Leak Today

### `nodes` is not a clean runtime-core layer

`WB_NODES_INCLUDE` currently pulls in Qt Core, Qt Network, Qt GUI, controller headers, OIS, FreeType, stb, Assimp, CUDA, and local includes from `app`, `core`, `engine`, `render`, `sound`, `util`, and `vrml` (re-read from `src/omnisim/Makefile` 2026-09-01; the WREN include left the list with the 2026-08-23 deletion).

✅ 2026-08-08 — one entry left this list for real: **ODE headers are no longer in the `nodes` include closure.** `bdc02139` deleted `src/ode` + `include/ode` (106,283 lines), so node-level code can no longer drag in a physics-engine header at all. That is a genuine reduction in the leak surface described below, not a relabelling. Campaign record: [ode-retirement-campaign.md](ode-retirement-campaign.md).

That means a change in node-level code can easily drag in:

- GUI-facing dependencies
- rendering-facing dependencies
- asset pipeline dependencies
- controller-facing dependencies

This is the single biggest reason "simulation core" is still a logical idea rather than a buildable boundary.

### `app` is bootstrap, not simulation core

`WB_APP_INCLUDE` depends on `control`, `editor`, `engine`, `nodes`, `plugins`, `scene_tree`, and `vrml`.

That makes `app` a top-of-stack integration layer. It should not be treated as a dependency target for lower-level runtime code.

### `gui` is correctly broad, but must stay top-only

`WB_GUI_INCLUDE` reaches nearly every subsystem. That is expected for the desktop shell, but it means GUI dependencies must not leak downward into parser, world, or step-loop code.

### `scene_tree` is part of the desktop shell, not world ownership

The scene tree consumes runtime state and dictionary behavior, but runtime code should not be shaped around scene-tree refresh or selection management.

If a runtime change exists mainly to keep the editor tree happy, the dependency direction is probably backwards.

## Practical Layer Model For Contributors

Until the build is fully modularized, treat the repo as three operational layers and one packaging layer.

### 1. Simulation core

Owns:

- world loading
- parser and node instantiation
- node graph state
- physics stepping
- controller IPC and synchronization
- deterministic simulation behavior

Primary directories today:

- `src/omnisim/core`
- `src/omnisim/vrml`
- `src/omnisim/engine`
- large parts of `src/omnisim/nodes`
- parts of `src/controller`

### 2. Rendering

Owns:

- wgpu-native integration (surfaces, render targets, shaders, mesh/texture caches)
- render-target and overlay management
- viewport and sensor rendering mechanics

Primary directories today:

- `src/omnisim/render`
- render-facing parts of `src/omnisim/nodes` (notably `OmWgpuSceneRenderer.*`)

### 3. Desktop shell

Owns:

- Qt widgets and dialogs
- editor tools
- scene tree
- project tools
- viewport controls and desktop-only workflows

Primary directories today:

- `src/omnisim/gui`
- `src/omnisim/editor`
- `src/omnisim/widgets`
- `src/omnisim/scene_tree`

### 4. Packaging and integration

Owns:

- platform packaging
- top-level build orchestration
- bundled resources and projects
- smoke and regression entrypoints

Primary directories today:

- repo root `Makefile`
- `resources`
- `projects`
- `.github/workflows`

## Review Rules For New Dependencies

Use these rules during review.

1. Lower layers may depend on lower or same-level code, never on higher-level desktop code.
2. Parser, world, stepping, and controller code should not gain `QtWidgets`, scene-tree, or editor dependencies.
3. Rendering code may consume simulation state, but simulation state should not require renderer headers to exist.
4. Desktop code may observe runtime state, but runtime code should not perform work only to satisfy desktop refresh behavior.
5. If one helper is forcing a heavy include, prefer a narrower interface or callback rather than pulling the full dependency through a header.

## File-Move Heuristics

When you are unsure where a class belongs, use this test:

- if it can run in a headless benchmark, it probably belongs in simulation core
- if it allocates textures, shaders, framebuffers, or render targets, it belongs in rendering
- if it owns dialogs, models, selections, widgets, or editor-specific state, it belongs in the desktop shell
- if it mainly coordinates legacy build, packaging, or release outputs, it belongs in the integration layer

Good moves:

- move image decoding policy out of a broad node header into a narrow implementation unit
- move desktop-only observers out of runtime classes and into adapter code
- move renderer policy behind a small runtime-facing service boundary

Bad moves:

- adding GUI includes to parser or world headers because one utility is convenient there
- putting scene-tree refresh logic inside runtime mutation code
- letting a controller or node helper depend directly on desktop-only preferences or widgets

## Immediate Boundary Improvements Worth Pursuing

These are good phase-two refactor candidates because they improve both architecture and iteration speed.

### Narrow `OmImageTexture` dependencies

`OmImageTexture` currently reaches into image loading, cache behavior, and renderer (wgpu) texture creation. The file is a boundary hotspot because a node-level change can affect GUI, asset, and renderer behavior together.

Near-term direction:

- keep format sniffing and metadata extraction in a narrow utility
- isolate texture upload and renderer object creation behind a renderer-facing implementation file
- keep node-facing state and URL semantics independent from desktop concerns

### Separate load orchestration from desktop bootstrap

`OmApplication` currently sits in the path that tokenizes, syntax-checks, and initializes the world. That is workable, but it reinforces the idea that the desktop app owns the simulation lifecycle.

Near-term direction:

- move load orchestration into a runtime-facing service
- keep desktop bootstrap responsible only for app startup, windowing, and tool wiring

### Keep scene-tree coupling out of runtime mutation paths

Dictionary updates and node regeneration currently ripple into scene-tree layout and selection logic. That is a major source of mental overhead.

Near-term direction:

- treat dictionary changes as runtime events
- let desktop adapters translate them into model updates
- avoid runtime code making broad desktop refresh decisions

## Safe Validation After Dependency Changes

After changing subsystem boundaries, always validate the narrowest path first:

1. `make sim-core`
2. `make renderer` if render-facing code moved
3. `make sim-gui` if desktop integration changed
4. the smallest smoke or headless world that exercises the new boundary

If a refactor only claims to improve simulation-core boundaries but still requires a full desktop rebuild to validate, the change probably did not go far enough.

## What Success Looks Like

A healthy dependency map has these properties:

- a change in `src/omnisim/render` does not force unrelated desktop code to rebuild
- a parser or node-graph change can be validated headlessly
- GUI code observes simulation state instead of shaping it
- coding agents can infer "edit here when changing X" without reading half the repo first

That is the standard to optimize toward even before the build system is fully replaced.
