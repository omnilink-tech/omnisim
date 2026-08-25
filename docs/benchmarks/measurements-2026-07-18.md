# OmniSim headless performance baseline — 2026-07-18

This baseline records the measurements taken before the 50-robot headless
throughput optimization campaign. Results are attributed to one machine and one
engine build; they are not portable hardware claims.

## Environment

- OmniSim: 5.1.1, commit `8c1bf7b1`
- machine id: `9722d23d12a3`
- CPU: AMD Family 25 Model 80, 8 cores / 16 threads
- GPU: NVIDIA GeForce RTX 3060 Laptop GPU, driver 596.36
- runtime: Newton 1.2.0, Warp 1.13.0, MuJoCo 3.8.1,
  MuJoCo-Warp 3.8.0.3
- benchmark mode: Windows, headless, `--batch --mode=fast --no-rendering`
- generated many-robot worlds use `basicTimeStep 16` and Newton XPBD
  (`XPBD(iters=10)` in the backend-verdict sidecar)

The full machine-attributed raw matrix is
`tests/benchmarks/optim-baseline/current-8c1bf7b1-machine-9722d23d12a3.json`.

## Whole-simulator many-robot baseline

`effective tick` is `(process wall time - logged loading time) / steps`. It
therefore includes controller synchronization, physics, post-physics state
projection, and other runtime work not represented by an individual performance
bucket.

| robots | physics ms/step | effective ms/tick | effective steps/s | real-time factor (16 ms) |
|---:|---:|---:|---:|---:|
| 5 | 10.602 | 54.923 | 18.21 | 0.291 |
| 25 | 10.427 | 89.046 | 11.23 | 0.180 |
| 50 | 12.708 | 120.552 | 8.30 | 0.133 |

The campaign acceptance target is at least **16.59 effective steps/s** for the
same 50-robot world and 300-step measurement, with unchanged final physics state
and trajectories. This is a 2x improvement over the 8.30 steps/s baseline.

World finalization dominates logged startup: 9.95–14.95 seconds across the
many-robot sweep, or roughly 98–99% of logged loading time.

## Multi-instance scaling

Including process startup and world finalization:

| instances | aggregate steps/s | speedup vs 1 | scaling efficiency |
|---:|---:|---:|---:|
| 1 | 9.51 | 1.00x | 100% |
| 2 | 18.34 | 1.93x | 96.5% |
| 4 | 27.23 | 2.86x | 71.6% |

## Raw GPU solver reference

`tests/benchmarks/newton_scaling_bench.py --worlds 1 256 1024 4096 --steps 200`
measured the MuJoCo-Warp solver separately from the full simulator:

| parallel worlds | ms/batched step | physics FPS | physics env-steps/s |
|---:|---:|---:|---:|
| 1 | 5.862 | 170.6 | 171 |
| 256 | 6.007 | 166.5 | 42,617 |
| 1,024 | 6.098 | 164.0 | 167,910 |
| 4,096 | 6.545 | 152.8 | 625,778 |

This uses MuJoCo-Warp, whereas the whole-simulator many-robot baseline uses
XPBD. It demonstrates GPU throughput headroom but is not a direct before/after
comparison for the engine benchmark.

## Diagnostics

The existing `OMNISIM_NEWTON_XPBD_GRAPH=1` path successfully captured a CUDA
graph after tick 10, but did not materially reduce the normal full-engine
physics bucket. A diagnostic-only run with both graph replay enabled and the
base-divergence guard disabled reduced the logged physics bucket:

| robots | baseline physics ms | diagnostic physics ms | bucket speedup |
|---:|---:|---:|---:|
| 5 | 10.602 | 6.741 | 1.57x |
| 50 | 12.708 | 9.222 | 1.38x |

The guard-off run is not a proposed configuration and did not pass an
end-to-end performance acceptance test. It isolates the cost of mandatory
GPU-to-CPU safety/readback synchronization and motivates moving the validity
check onto the GPU while keeping the guard enabled.

Raw diagnostic files:

- `tests/benchmarks/optim-baseline/current-xpbd-graph-8c1bf7b1-machine-9722d23d12a3.json`
- `tests/benchmarks/optim-baseline/current-xpbd-graph-no-guard-8c1bf7b1-machine-9722d23d12a3.json`

## Known measurement gaps

- controller wait, retry, and packet-pressure time are not separately logged
- the benchmark's existing `scaling_efficiency` field measures child spread,
  not aggregate scaling efficiency
- headless `mainFPS` is not meaningful
- the many-camera benchmark mixes sensor work with one controller process per
  camera, so it cannot yet attribute the wall-time slope to rendering alone
- the full benchmark runner writes its aggregate JSON only at the end and has
  poor progress/partial-result behavior

These gaps must be closed alongside optimization so performance claims remain
reproducible and behavior-preserving.

## Optimization trial handoff

The first optimization pass was stopped after measurement and hypothesis
testing. No simulator or controller source change was retained. A compact
machine-attributed trial record is committed at
`tests/benchmarks/optim-baseline/optimization-trials-2026-07-18-machine-9722d23d12a3.json`.

The pass added an opt-in final-state capture to `optim_bench.py`. At the exact
last simulation step it records every benchmark robot's root position,
orientation matrix, and six-axis velocity. The rejected engine scheduling
trials and the empty-controller batching trial produced byte-identical state
JSON to their paired originals (300-step SHA-256
`96ffaad92aa45e11442342d90fc1e1f46b32b9f8403521327db6011f808b9bb7`).

Three hypotheses were rejected:

- coalescing controller retry signals made the isolated run slower;
- event-driven FAST-mode wakeups, with and without the existing thread yield,
  were performance-neutral in paired 50-robot runs;
- batching the built-in empty controller's no-op step requests produced only a
  noisy 1.05x paired improvement, far short of the 2x target.

An important benchmark correction was discovered: `many-robots` requests the
`omnibot_random` controller, but that controller is outside the benchmark
project's controller search path. All robot controllers in the committed
baseline therefore fell back to the native empty `<generic>` controller. The
50-robot numbers remain valid for the committed world, physics, and IPC load,
but they are not a measurement of 50 active Python random-walk controllers.

Later trials were strongly contaminated by unrelated concurrent OmniSim work on
the same machine: paired original rates fell from the clean 8.30 steps/s
baseline to 6.05 and eventually 1.33 steps/s, while physics buckets varied from
10.9 to 55.6 ms. These rates are recorded for auditability but must not be used
as acceptance results. Resume from the committed clean baseline and rerun on an
idle machine. The most promising unimplemented direction remains the default-on
Newton base-divergence guard's per-tick GPU-to-CPU readback; its guard-off
diagnostic reduced the 50-robot physics bucket from 12.708 to 9.222 ms, and the
guard must remain enabled with identical behavior in any real fix.
