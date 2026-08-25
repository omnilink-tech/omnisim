# OmniSim Profiling Playbook

This guide covers the performance instrumentation that already exists in the tree and how to use it consistently.

See also:

- [benchmark-authoring.md](benchmark-authoring.md)
- [performance-handbook.md](performance-handbook.md)
- [sensor-and-device-performance.md](sensor-and-device-performance.md)

## What Exists Today

OmniSim already exposes a performance log path through:

- `--log-performance=<file>[,<steps>]`
- `python scripts/dev/omnisim_dev.py profile-world <world>`
- `python scripts/dev/omnisim_dev.py benchmarks`

The current log includes measurements for:

- `loading`
- `prePhysics`
- `physics`
- `postPhysics`
- `mainRendering`
- `virtualRealityHeadsetRendering`
- `gpuMemoryTransfer`
- `trianglesCount`
- per-device rendering buckets
- per-controller buckets
- average FPS

## FPS Logging Notes

Average FPS is part of the log format and is populated by the WREN-side hook in `src/omnisim/gui/OmView3D.cpp`: `logWrenStatistics()` accumulates frames during the main render path (`++mRenderedFrameCount`) and calls `OmPerformanceLog::setAvgFPS(...)`. It is flushed from `OmMainWindow.cpp` on world unload and on shutdown.

Interpretation:

- `mainFPS` is present in the output format (the log column is `<mainFPS>`)
- it reflects the desktop GUI main viewport, since frames only accumulate when the WREN render path runs
- in headless / `--no-rendering` runs that path is skipped, so FPS is not a meaningful metric there — rely on `mainRendering`, `gpuMemoryTransfer`, and world-specific before/after comparisons instead

## Useful Commands

### Profile one world

```bash
python scripts/dev/omnisim_dev.py profile-world tests/rendering/worlds/normals.omniworld
```

This writes to:

```text
tests/benchmarks/last-performance.log
```

Unless `--log` is provided.

### Profile one world to a dedicated log

```bash
python scripts/dev/omnisim_dev.py profile-world tests/physics/worlds/contact_points.omniworld --log tests/benchmarks/logs/contact_points-local.log
```

### Run the benchmark set

```bash
python scripts/dev/omnisim_dev.py benchmarks --nomake
```

This writes one log per benchmark under:

```text
tests/benchmarks/logs/
```

## Current Benchmark Set

The current benchmark runner uses:

- `resources/projects/worlds/empty.omniworld` for startup
- `tests/rendering/worlds/normals.omniworld` for a sensor-heavy-frame proxy
- `tests/physics/worlds/contact_points.omniworld` for steady-state physics
- `projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld` for a contact-heavy physics step
- `tests/protos/worlds/template_deterministic.omniworld` for world-load behavior

This set is intentionally small. It is a fast comparison set, not a full performance certification suite.

## How To Read The Log

### `loading`

Treat this as a broad startup bucket. It currently mixes:

- world parsing
- node instantiation
- PROTO/template work
- synchronous asset download and cache activity
- world finalization

It is useful for regression detection but not yet precise enough for root-cause attribution.

### `prePhysics`

Work done before the physics step. This includes setup and some controller/runtime bookkeeping.

### `physics`

The Newton `SolverMuJoCo` step (`mj_step` on CPU, `mujoco_warp` on GPU) and related physics work. This is the most important bucket for throughput regressions in physics-heavy worlds.

### `postPhysics`

Work done after physics and before viewport rendering finishes the step.

### `mainRendering`

Main viewport rendering time. Use this when investigating rendering regressions in the desktop simulator.

### `gpuMemoryTransfer`

A useful proxy for:

- sensor texture uploads
- overlay uploads
- camera recognition overlays
- depth-to-texture conversion costs

If this grows, inspect camera, display, and overlay paths first.

### `trianglesCount`

A scene-complexity signal that helps explain why a rendering or loading number moved.

### Per-device rendering buckets

These are dynamic entries emitted for rendering devices. They help separate:

- main viewport cost
- sensor camera cost
- rendering-window cost

### Per-controller buckets

These are dynamic controller timing entries. They help identify whether a slowdown is in simulator stepping or controller-side processing.

## Profiling Discipline

To keep comparisons meaningful:

- compare the same world against the same world
- rebuild once, then use `--nomake` on repeated runs
- keep cache state consistent across before/after runs
- use the same simulation mode and startup flags
- record whether the run was headless or desktop

## Fast Comparison Workflow

1. Build once.
2. Run the same benchmark world before the change.
3. Save the log.
4. Apply the change.
5. Rebuild the affected target.
6. Run the same benchmark again with `--nomake`.
7. Compare `loading`, `physics`, `mainRendering`, and `gpuMemoryTransfer`.

## When To Use Headless Profiling

Use headless profiling when:

- the change is in parser, runtime, or controller synchronization
- you want to exclude desktop rendering from the measurement
- you are measuring startup or world-load behavior

Use desktop profiling when:

- the change is visual
- the change affects WREN, post-processing, shadows, or sensor rendering
- you need to measure the main viewport path

## Gaps To Close Later

The next instrumentation upgrades should be:

- extend FPS logging to headless / `--no-rendering` runs (today it only accumulates on the desktop WREN render path)
- split `loading` into parse, asset download, template regeneration, and finalization
- add a first-frame metric
- add a dedicated sensor-heavy frame benchmark
- add a large-world memory footprint benchmark
- add simple machine-readable comparison tooling for CI
