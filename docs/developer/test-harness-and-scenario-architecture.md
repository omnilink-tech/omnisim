# Test Harness And Scenario Architecture

This document explains how the current validation surface is put together, where it is still too coarse or stateful, and how it should evolve to support a faster development loop.

It is grounded in:

- `tests/test_suite.py`
- `tests/smoke/run_smoke.py`
- `tests/benchmarks/run_benchmarks.py`
- `.github/workflows.disabled/developer_fast_path.yml` (currently disabled)
- `.github/workflows.disabled/smoke_linux_fast.yml` (currently disabled)

## Why This Matters

Build speed is only half of iteration speed. The other half is how cheaply a change can be validated.

Right now OmniSim has the beginnings of a good layered workflow:

- a developer CLI
- smoke manifests
- benchmark manifests
- a fast Linux CI lane

But the underlying test harness is still shaped around an older, broader execution model.

## Current Structure

### Developer-facing wrappers

The newer surface is simple:

- `tests/smoke/run_smoke.py` loads `smoke_worlds.json` and forwards those worlds to `tests/test_suite.py`
- `tests/benchmarks/run_benchmarks.py` loads `benchmark_worlds.json`, adds `--performance-log`, and forwards worlds to `tests/test_suite.py`
- `scripts/dev/omnisim_dev.py` exposes `test-smoke`, `test-group`, `test-world`, `run-headless`, and `profile-world`

That is the right public direction.

### Legacy core harness

The execution engine is still `tests/test_suite.py`.

It:

- groups tests as `api`, `cache`, `other_api`, `physics`, `protos`, `parser`, and `rendering`
- writes coordination files such as `worlds.txt`, `world_counter.txt`, `output.txt`, `omnisim_stdout.txt`, and `omnisim_stderr.txt`
- launches OmniSim in fast batch no-rendering mode
- uses file-based bookkeeping to track how many worlds ran and what happened
- handles the `cache` group specially by restarting per world

This works, but it is still too stateful and too monolithic for the fastest possible contributor loop.

## Code-Backed Pressure Points

### 1. Scenario wrappers are thin, but the core runner is still broad

Smoke and benchmark entrypoints already exist, but they still inherit the behavior of the larger legacy harness.

That means:

- one world run still pulls in substantial harness behavior
- group-level assumptions still shape execution
- new targeted workflows are constrained by older infrastructure

### 2. File-based coordination makes the harness harder to reason about

`tests/test_suite.py` writes several mutable files in `tests/` to coordinate execution and result tracking.

That creates friction for:

- parallelization
- reuse by other tools
- narrower programmatic runners
- clearer failure reporting

It is also harder for coding agents to understand than a smaller in-memory or structured-result runner would be.

### 3. Headless fast mode is good, but it does not cover every performance question

The harness launches OmniSim with:

- `--mode=fast`
- `--stdout`
- `--stderr`
- `--batch`
- `--minimize`
- `--no-rendering` (only when no supervisor is attached; omitted when a supervisor is in play so screenshots work)

That is excellent for many API and runtime checks.

But it also means:

- main-view first-frame performance is not represented
- desktop-shell regressions are not represented
- some rendering costs are intentionally absent from the default harness

The fix is not to make every test graphical. The fix is to be explicit about which questions each lane can answer.

### 4. Benchmark coverage is still narrow

The benchmark manifest currently covers:

- startup on empty world
- one rendering-oriented normals world
- one steady-state physics world (contact points)
- one contact-heavy physics world (`newton-4v4-husky-head-on`)
- one PROTO/world-load case

That is a useful start, but it still lacks at least:

- a large-world memory case
- a clear first-frame render case
- a controller-heavy chatter case
- a sensor-heavy but deterministic camera or overlay case

### 5. CI lanes exist, but they are still light wrappers over current structure

The repo already has (currently parked under `.github/workflows.disabled/`):

- `developer_fast_path.yml`
- `smoke_linux_fast.yml`

Those are useful, but they do not yet mean the whole test surface is fully decomposed.

They are proof that the interface can be layered, not proof that the underlying runner is already optimal.

## Recommended Direction

### 1. Split discovery, execution, and reporting

`test_suite.py` currently mixes:

- scenario discovery
- world list generation
- simulator process execution
- result collection
- output formatting

Those should become separable layers.

That would make it easier to build:

- one-world runners
- one-group runners
- benchmark-only runners
- CI adapters

without duplicating harness logic.

### 2. Move toward structured scenario manifests

The JSON manifests are a good start. They should grow into richer scenario descriptions with fields such as:

- name
- world path
- category
- required mode: headless, sensor, desktop, cache-sensitive
- benchmark expectations
- allowed platforms

That gives both humans and tools a stable way to choose the right validation slice.

### 3. Make headless and graphical contracts explicit

Each scenario should clearly declare whether it needs:

- pure headless no-rendering mode
- sensor rendering only
- full main-view rendering
- desktop-shell behavior

This avoids using the wrong lane for the wrong question.

### 4. Reduce mutable shared state in the harness

Prefer structured temporary directories or explicit result objects over shared top-level coordination files whenever the harness is refactored.

That will improve:

- debuggability
- parallel safety
- CI composability
- contributor understanding

### 5. Expand the benchmark set by performance question

Keep one stable world per question:

- startup
- world load
- first-frame render
- steady-state physics
- sensor-heavy frame
- controller-heavy interaction
- large-world memory footprint

That structure maps directly to the performance roadmap.

## Low-Risk Changes To Do First

These are good immediate improvements:

- keep smoke and benchmark manifests small and local-asset only
- add missing benchmark categories incrementally
- document which questions `--no-rendering` lanes can and cannot answer
- make scenario categories visible in CI logs

These improve contributor understanding before the deeper harness cleanup begins.

## Phase-Two Refactor Sequence

1. add richer scenario metadata to smoke and benchmark manifests
2. split test-suite discovery from process execution
3. add a narrow reusable one-world runner
4. separate headless, sensor, and desktop scenario capabilities
5. reduce shared mutable coordination files
6. let CI lanes compose the same small runner building blocks

That order preserves the current validation surface while making it more targetable.

## Validation Guidance

After harness work, validate with:

- `python scripts/dev/omnisim_dev.py test-smoke`
- `python scripts/dev/omnisim_dev.py test-group parser`
- `python scripts/dev/omnisim_dev.py test-world tests/api/worlds/accelerometer.omniworld`
- `python scripts/dev/omnisim_dev.py benchmarks`

The important thing to check is not only that the worlds still run, but that the execution path is easier to target and explain than before.
