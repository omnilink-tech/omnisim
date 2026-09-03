# OmniSim Benchmark Authoring Guide

This guide explains how to add and maintain useful performance scenarios for OmniSim.

It complements [profiling-playbook.md](profiling-playbook.md) by focusing on benchmark design rather than only benchmark execution.

## Purpose

Good benchmarks should help answer:

- did startup get slower?
- did world loading get slower?
- did physics throughput change?
- did sensor or render cost move?
- is the result deterministic enough to compare over time?

Bad benchmarks create noise instead of signal.

## Current Benchmark Entry Points

The repo currently supports:

- `python -m omnisim profile-world <world>`
- `python -m omnisim benchmarks`
- `tests/benchmarks/benchmark_worlds.json`

## Benchmark Categories

Every benchmark world should belong to one primary category.

### Startup

Use a minimal world that isolates startup overhead and basic load/finalization work.

### World load

Use a world that stresses parsing, PROTO/template behavior, or load-time scene assembly.

### Steady-state physics

Use a world that spends meaningful time in collision and Newton `SolverMuJoCo` stepping.

### Sensor-heavy frame

Use a world that stresses:

- cameras
- displays
- overlays
- depth handling
- GPU transfer

### Asset-heavy or large-world

Use a world that stresses:

- scene size
- textures
- mesh count
- memory footprint

This category is not fully represented in the current set and should be expanded later.

## Benchmark Design Rules

### Rule 1: One benchmark, one main reason

A benchmark should have one dominant cost center. Mixed worlds are harder to interpret.

### Rule 2: Prefer local deterministic assets

Remote assets introduce cache and network variance. Avoid them in benchmark worlds.

### Rule 3: Keep world intent obvious

From the world name alone, a contributor should be able to understand why it exists.

Examples:

- startup-empty
- physics-contact-points
- rendering-normals
- proto-deterministic

### Rule 4: Avoid warnings unless the warning is the subject

Warnings create noise in logs and can mask regressions.

### Rule 5: Keep benchmarks small enough to run often

Benchmarks are useful only if contributors are willing to run them before and after a change.

## What Makes A Benchmark Unusable

- remote assets
- unstable controller behavior
- nondeterministic load sequence without documentation
- too many unrelated cost centers
- warnings that appear or disappear based on environment

## How To Add A Benchmark

1. Choose the primary category.
2. Make sure the world is local-asset and deterministic enough for comparison.
3. Add the entry to `tests/benchmarks/benchmark_worlds.json`.
4. Run the benchmark locally at least twice.
5. Check that the log is understandable and stable enough to compare.
6. Document why the world exists if the intent is not obvious from the name.

## Suggested Benchmark Expansion

### Add a true sensor-heavy benchmark

The repo needs a world that clearly stresses sensor rendering and GPU transfer beyond the current proxy case.

### Add a large-asset benchmark

The repo needs a world that clearly stresses:

- texture decode
- asset load
- scene assembly

### Add an explicit contact-heavy benchmark

The repo needs a world that stresses contact count and collision handling more than generic physics stepping.

### Add a large-world memory benchmark later

This should become a later-stage benchmark once measurement support is ready.

## Comparison Rules

When comparing benchmarks:

- compare the same world against itself
- rebuild once, then use `--nomake` for repeated runs
- keep cache state consistent
- record thread configuration and whether rendering was enabled
- note whether the run was intended to be deterministic or throughput-oriented

## Metrics To Watch

Depending on category, pay attention to:

- `loading`
- `prePhysics`
- `physics`
- `postPhysics`
- `mainRendering`
- `gpuMemoryTransfer`
- `trianglesCount`
- per-device buckets
- per-controller buckets

Remember that `avgFPS` is not currently the strongest metric under WREN until its update hook is restored.

## Benchmark Review Checklist

- does the benchmark isolate one primary cost center?
- are all critical assets local?
- does it run without warnings?
- does it produce readable performance logs?
- is it cheap enough to use in normal contributor workflow?

## CI Guidance

Benchmarks should be used in two ways:

- quick local or PR-level before/after comparisons for a small representative set
- broader, slower trend tracking for merge queues or scheduled runs

Do not make every benchmark world part of the default fast path unless it is truly cheap.

## Maintenance Rules

When a benchmark world stops being representative:

- update it
- replace it
- or explicitly retire it

Do not leave stale benchmark worlds in place just because they already exist. Performance suites must evolve with the simulator.
