# Observability And Performance Telemetry

This document focuses on the current measurement surface in OmniSim, where it is already useful, and where it still fails to explain important slowdowns.

It is grounded in:

- `src/omnisim/util/OmPerformanceLog.hpp`
- `src/omnisim/util/OmPerformanceLog.cpp`
- `src/omnisim/gui/OmView3D.cpp`

## Why This Matters

A simulator cannot be optimized reliably if contributors cannot tell:

- which phase is slow
- whether the slowdown is CPU, controller, render, or asset related
- whether the performance log reflects the behavior users actually saw

OmniSim already has useful telemetry. The next step is making it complete enough to drive the roadmap.

## What Exists Today

`OmPerformanceLog` covers the main step buckets (`loading` and its sub-buckets `loadingParse`, `loadingInstantiate`, `loadingDownload`, `loadingFinalize`, plus `prePhysics`, `physics`, `postPhysics`, `mainRendering`, `gpuMemoryTransfer`, `trianglesCount`) plus per-device rendering aggregates, per-controller aggregates, average speed factor, average FPS, and world-close reporting. See `profiling-playbook.md` for the full field reference and collection commands.

That is a strong base. The problem is not "there is no telemetry." The problem is that the current telemetry is still missing several decisive slices.

## Code-Backed Gaps

### 1. Main FPS reports an accumulated average, not per-frame jitter

`OmPerformanceLog` has:

- `setAvgFPS(double value)`
- a `<mainFPS>` output column in the world-close report

`OmView3D::logWrenStatistics()` feeds it from live renderer statistics: it divides `mRenderedFrameCount` (incremented once per rendered frame) by the elapsed wall time since the last call, and `OmMainWindow` invokes it during the run.

That means the contributor-visible `<mainFPS>` number is a single accumulated average over the measurement window. It does not surface per-frame jitter or worst-case frame time, so a render path that is mostly fast but occasionally stalls can still report a healthy mean FPS.

### 2. `loading` is sub-bucketed, but template/dictionary and asset-wait work are not isolated

The log already splits startup into `loading`, `loadingParse`, `loadingInstantiate`, `loadingDownload`, and `loadingFinalize` columns. Those four sub-buckets are instrumented with real `startMeasure`/`stopMeasure` calls (parse and instantiate in `OmWorld.cpp`, download and finalize in `OmSimulationWorld.cpp`), so contributors can already see parse vs instantiate vs download vs finalize costs.

What is still folded into the coarser buckets:

- template and dictionary settlement
- asset wait time as a phase distinct from `loadingDownload`

Splitting those out is the remaining startup-telemetry gap.

### 3. First-frame behavior is not separated from steady-state rendering

The current categories tell us about accumulated rendering work, but not clearly about:

- time to first visible frame
- cold texture upload cost
- first sensor frame cost

Those are often the numbers users actually feel during startup and reset.

### 4. Controller time is visible, but controller pressure is not

Per-controller aggregates exist, but the log still does not tell us:

- how many retries happened
- how much time was spent waiting for controller responses
- packet volume
- console redirect pressure

So the log can say "controllers are expensive" without revealing which mechanism caused it.

### 5. Desktop-shell cost is mostly outside the current telemetry

The performance log is strong on simulation and render timing, but weak on:

- scene-tree update cost
- layout invalidation frequency
- selection restore work
- desktop refresh backlog

That leaves an entire class of "the simulator feels slow" reports under-instrumented.

## Recommended Telemetry Roadmap

### 1. Add per-frame jitter alongside average main-FPS

Average FPS is already plumbed (`logWrenStatistics()` → `setAvgFPS()`). The high-value addition is worst-case detail:

- report worst-case / 99th-percentile frame time, not just the mean
- ensure the numbers reflect the main viewport users actually see
- document any platform-specific limitations

Without per-frame detail, an occasionally-stalling render path can still show a healthy mean and hide regressions from benchmark logs.

### 2. Finish splitting world-load telemetry into real phases

Parse, instantiate, download, and finalize already have dedicated buckets. Add distinct fields for the phases still folded into them:

- template or dictionary settlement
- asset wait time as a phase distinct from `loadingDownload`

This rounds out one of the highest-leverage telemetry slices in the repo.

### 3. Add first-frame and cold-start metrics

Track:

- first-frame render time
- first sensor frame time
- first-frame GPU transfer
- startup-to-ready time

That gives contributors a way to optimize what users perceive first.

### 4. Add controller-pressure metrics

Track:

- controller wait time
- retry counts
- packet bytes
- redirected stdout/stderr bytes
- controller startup time

These should sit beside the existing per-controller timing aggregates.

### 5. Add desktop-shell transaction metrics

Track at least:

- scene-tree update count
- layout change count
- longest UI transaction time
- selection restore count

That will make GUI responsiveness work measurable instead of anecdotal.

### 6. Add benchmark-comparison tooling, not just raw logs

The repo now has a benchmark runner, but the real value comes when logs are compared consistently over time.

Later work should standardize:

- baseline storage
- tolerance policy
- category-based summaries
- CI-friendly regressions versus noise filtering

## Measurement Principles

### Measure boundaries, not just totals

A single total time is useful only when the system is already well understood.

OmniSim still needs boundary timing because many costs cross subsystem seams:

- controller and runtime
- parser and template manager
- asset fetch and startup
- sensor rendering and GPU transfer
- runtime mutation and desktop shell projection

### Keep telemetry cheap enough to leave on in benchmark paths

If collecting the numbers changes the numbers too much, contributors will stop trusting them.

Prefer:

- aggregated counters
- per-step timers
- phase-level event markers

Over:

- verbose tracing by default
- large per-object logs in normal fast-path runs

### Make logged fields stable

A telemetry format becomes useful to CI and coding agents only when fields are stable enough to parse automatically.

That matters for:

- benchmark comparison tooling
- machine-readable summaries
- future repository-local agent workflows

## Suggested Implementation Order

1. add per-frame main-FPS jitter alongside the existing average
2. finish splitting `loading` (template/dictionary and asset-wait phases)
3. add first-frame timing
4. add controller wait and retry telemetry
5. add desktop-shell transaction counters
6. add log comparison tooling

This order pays down the biggest blind spots first.

## Validation Guidance

After telemetry changes, validate with:

- `python scripts/dev/omnisim_dev.py profile-world resources/projects/worlds/empty.omniworld`
- `python scripts/dev/omnisim_dev.py benchmarks`
- one sensor-heavy scenario
- one controller-heavy scenario

Check two things:

- the new numbers are present and nonzero when expected
- the added logging does not distort the fast-path run significantly

Telemetry is only good if it is both truthful and cheap enough to keep.
