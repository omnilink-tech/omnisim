# OmniSim Phase Two Execution Program

This document turns the broader phase-two architecture plan into a more concrete implementation program.

It is intentionally practical:

- which workstreams should happen first
- what each workstream must prove
- which files and subsystems are the likely starting points
- how to keep validation narrow while the architecture evolves

> **Freshness:** WS1's measurement foundation has partial deliverables already in tree — `CpuPassTimer` / `GpuPassTimer` and the `OMNISIM_RENDERER_TIMINGS` env var give per-pass renderer cost; the validation harness (`scripts/harness/`) covers part of WS7. The named "what to do first" items inside each workstream may be partly stale — confirm against current code before treating them as the next step. When a workstream is fully retired, delete its section.

Read this together with:

- `phase-two-architecture-plan.md`
- `improvement-backlog.md`
- `performance-handbook.md`

## Purpose

The repo already has a clear direction:

- establish a true simulation-core surface
- make runtime, rendering, and desktop boundaries clearer
- shorten the build and validation loop

What phase two still needed was a better execution order.

The most important rule is this:

Do not start with a renderer rewrite or a build-system replacement. Start by making the current system measurable and decomposable.

## Program Principles

### 1. Each milestone must shrink uncertainty

Architecture work only counts if it makes later changes easier to scope, build, and validate.

### 2. Every workstream needs a narrow validation path

If a workstream can only be validated through the full desktop product, it is still too broad.

### 3. Measurement comes before migration

Before changing a boundary, make its current cost visible.

### 4. Headless runtime support is a forcing function

If a subsystem claim is "this is part of simulation core," it should eventually be testable without depending on the full desktop shell.

## Workstreams

## Workstream 1: Telemetry Foundation

Goal:

- make startup, controller, and render costs measurable enough to guide the rest of phase two

Primary code starting points:

- `src/omnisim/util/WbPerformanceLog.*`
- `src/omnisim/gui/WbView3D.cpp`
- `src/omnisim/engine/WbSimulationWorld.cpp`

What to do first:

- restore trustworthy main FPS logging
- split coarse `loading` timing into smaller load-phase buckets
- add first-frame and asset-wait metrics
- add controller wait and retry metrics

Exit criteria:

- benchmark and profile logs distinguish load, controller, render, and asset wait cost
- at least one automated benchmark run can compare those fields over time

## Workstream 2: Controller Boundary Cleanup

Goal:

- make controller transport, protocol, and step scheduling separable

Primary code starting points:

- `src/controller/c/robot.c`
- `src/omnisim/control/WbController.cpp`
- `src/omnisim/control/WbControlledWorld.cpp`

What to do first:

- replace fixed-size redirected stream reads
- document and then formalize controller lifecycle states
- log controller wait and packet pressure
- isolate scheduler policy from transport code

Exit criteria:

- controller-heavy behavior can be profiled and validated headlessly
- controller code has a smaller documented runtime contract

## Workstream 3: Startup, Reset, And Asset Lifecycle Cleanup

Goal:

- separate world structure loading from remote-asset readiness and reset waiting

Primary code starting points:

- `src/omnisim/app/WbApplication.cpp`
- `src/omnisim/nodes/utils/WbWorld.cpp`
- `src/omnisim/engine/WbSimulationWorld.cpp`
- `src/omnisim/core/WbDownloadManager.cpp`
- `src/omnisim/core/WbDownloader.cpp`

What to do first:

- measure parse, instantiate, regenerate, finalize, and asset wait time separately
- make benchmark and smoke worlds local-asset only
- define explicit startup and reset mode expectations
- reduce hidden dependence on synchronous asset waits

Exit criteria:

- startup logs can explain where time went
- deterministic headless runs are not silently shaped by remote-asset behavior

## Workstream 4: Runtime Data-Structure And Query Cleanup

Goal:

- reduce repeated tree scans and other expensive correctness-oriented queries in hot or repeated paths

Primary code starting points:

- `src/omnisim/nodes/utils/WbNodeUtilities.cpp`
- `src/omnisim/nodes/utils/WbTemplateManager.*`
- `src/omnisim/nodes/utils/WbDictionary.*`

What to do first:

- instrument repeated descendant queries
- add cheaper query or cache boundaries where the same checks recur
- reduce mutation-triggered full scans

Exit criteria:

- common transform, validation, and mutation operations stop relying on repeated broad descendant walks
- runtime mutation work becomes easier to reason about and benchmark

## Workstream 5: Rendering And Sensor Boundary Cleanup

Goal:

- separate steady-state simulation work from render-specific upload and sensor costs

Primary code starting points:

- `src/omnisim/gui/WbView3D.cpp`
- `src/omnisim/nodes/WbImageTexture.cpp`
- `src/omnisim/nodes/WbBackground.cpp`
- `src/omnisim/nodes/WbCamera.cpp`
- `src/omnisim/wren/*`
- `src/wren/*`

What to do first:

- move CPU-side texture and environment transforms toward GPU-side handling
- add first-frame and GPU-transfer observability
- reduce repeated recognition and overlay cost in sensor-heavy paths
- keep desktop quality heuristics visible and measurable

Exit criteria:

- sensor-heavy and rendering-heavy worlds can be profiled separately
- renderer work no longer hides behind one coarse frame-time story

## Workstream 6: Desktop Shell Decoupling

Goal:

- keep scene-tree and editor behavior responsive without pushing desktop concerns into runtime ownership

Primary code starting points:

- `src/omnisim/core/WbGuiRefreshOracle.cpp`
- `src/omnisim/scene_tree/WbSceneTreeModel.cpp`
- `src/omnisim/scene_tree/WbSceneTree.cpp`
- `src/omnisim/gui/*`

What to do first:

- add desktop-shell transaction metrics
- reduce unnecessary layout invalidation
- batch runtime-originating GUI updates
- preserve selection by identity rather than broad reset behavior

Exit criteria:

- runtime mutation storms do not automatically become GUI storms
- scene-tree behavior is cheaper and easier to predict during live simulation

## Workstream 7: Test Harness And Validation Surface Cleanup

Goal:

- make the fast developer loop reflect true subsystem boundaries

Primary code starting points:

- `scripts/dev/omnisim_dev.py`
- `tests/test_suite.py`
- `tests/smoke/run_smoke.py`
- `tests/benchmarks/run_benchmarks.py`
- `.github/workflows/*`

What to do first:

- enrich scenario manifests
- split test discovery from execution
- add smaller reusable runners
- align CI lanes with subsystem questions instead of legacy group shape

Exit criteria:

- one-world and one-subsystem validation become easier to explain and run than the broad legacy path
- CI defaults are fast enough to shape local iteration

## Milestone Ladder

### Milestone A: Rebuild Trust In Measurement

Deliver:

- main FPS logging restored
- smaller load buckets
- benchmark logs stable enough to compare

Why first:

- later architecture work needs trustworthy before-and-after data

### Milestone B: Make Headless Runtime Validation Real

Deliver:

- clear controller/runtime validation path
- deterministic local-asset startup expectations
- narrower one-world validation commands

Why next:

- this is the minimum useful simulation-core contract

### Milestone C: Remove The Most Expensive Hidden Couplings

Deliver:

- controller lifecycle made explicit
- startup asset wait cost separated from world construction
- repeated descendant scans reduced in hot mutation paths

Why here:

- these are the biggest cross-cutting blockers to both performance and maintainability

### Milestone D: Separate Render Cost From Everything Else

Deliver:

- first-frame metrics
- clearer sensor and GPU-transfer accounting
- better boundaries around texture and background processing

Why here:

- rendering changes are easier once the runtime and startup surfaces are clearer

### Milestone E: Turn Logical Boundaries Into Real Build And Test Boundaries

Deliver:

- smaller test runners
- more targetable CI lanes
- simulation-core and renderer surfaces that are materially easier to build separately

Why last:

- build and CI boundaries become durable only after runtime ownership is clearer

## Near-Term Execution Order

If phase two started now, the first practical sequence should be:

1. restore and extend performance telemetry
2. fix controller stream buffering and add controller-pressure metrics
3. split startup timing and remove remote assets from benchmark-critical scenarios
4. reduce repeated descendant scans and mutation-side scans
5. add first-frame and desktop-shell metrics
6. refactor the test harness around reusable scenario runners

That is a better order than trying to attack every boundary at once.

## What To Avoid

Avoid:

- replacing the whole build system before the subsystem seams are clearer
- large multi-directory refactors without new telemetry first
- broad architectural moves that still require the full desktop binary for validation
- adding more heuristics without logging and measurement

These are the kinds of changes that feel ambitious but slow the program down.

## Definition Of Success

Phase two is working if:

- contributors can explain where time goes during startup, stepping, controllers, and rendering
- targeted runtime changes have a headless validation story
- render-heavy changes can be tested without rebuilding everything else
- desktop-shell work stops shaping runtime ownership
- the build and test surface starts matching the logical subsystem map

That is the implementation standard for the next wave of work.
