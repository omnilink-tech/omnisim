# GranularGroup GPU vs CPU Sweep — Results

Auto-generated companion to [bench_granular_group.py](bench_granular_group.py)
and [§13.7 of docs/developer/engine-migration-plan.md](../../docs/developer/engine-migration-plan.md#137--adjacent-subsystems-cuda--granular--particles--damage)
(M2 — the old `physics-roadmap.md` was merged into `engine-migration-plan.md`).
The CPU baseline is the pre-CUDA Solid+Sphere physics path measured at
[tests/granular-spike/results.md](../granular-spike/results.md) on
2026-04-28.

## Run conditions

- Hardware: NVIDIA GeForce RTX 3060 Laptop GPU (sm_86, 6 GiB), CUDA 12.1
- Simulator: OmniSim built with `OMNISIM_WITH_CUDA=ON`, MSYS2 mingw64 g++ 15.2
- Per-particle params: r=0.02 m, m=5 g, k_spring=300 N/m, k_damp=0.5 N·s/m,
  k_friction=0.5 N·s/m, μ=0.5 (Coulomb cap)
- Outer step: basicTimeStep=16 ms; 8 internal substeps (dt_inner=2 ms)
- Walls: 0.6 m × 0.6 m floor (particles pile, no spreading)
- Settle window: kernel-ms samples taken from step ≥ 96 (~1.5 s in, after
  free-fall, contacts dense)
- Headless: `--mode=fast --no-rendering --minimize`, 5 s wall per N

## Numbers (with friction, brute-force O(N²) — pre-broadphase)

| N | GPU kernel ms/step (mean) | min | max | samples | CPU ms/step (baseline) | speedup |
|---|---|---|---|---|---|---|
| 100 | 0.222 | 0.163 | 0.379 | 37 | 0.545 | 2.5x |
| 200 | 0.376 | 0.283 | 0.657 | 37 | 7.966 | 21.2x |
| 400 | 0.613 | 0.566 | 0.702 | 37 | 180.327 | 294.0x |
| 600 | 0.798 | 0.698 | 1.067 | 37 | DNF (>120 s) | ∞ |
| 1000 | 0.997 | 0.933 | 1.188 | 36 | n/a | — |
| 2000 | 2.290 | 1.930 | 3.163 | 19 | n/a | — |
| 4000 | DNF (sim ran slower than wall, hit kernel-timing settle window late) | — | — | — | n/a | — |

Brute force breaks past N≈2 000 — N² scaling makes contacts dominate.

## Numbers (uniform-grid broadphase — current state)

cellSize = 2·radius, ~9 neighbour-cell linked-list walks per particle
instead of N pair tests. Built fresh once per outer step.

| N | GPU kernel ms/step (mean) | min | max | samples |
|---|---|---|---|---|
| 1000 | 0.380 | 0.278 | 0.655 | 29 |
| 2000 | 0.391 | 0.251 | 0.756 | 29 |
| 5000 | 0.623 | 0.463 | 1.071 | 29 |
| 10000 | 0.669 | 0.547 | 1.249 | 29 |
| 20000 | 1.023 | 0.760 | 1.920 | 29 |
| 50000 | 2.484 | 2.033 | 3.493 | 28 |
| 100000 | 4.502 | 3.954 | 5.628 | 18 |

Real-time budget at basicTimeStep=16 ms is 16 ms/step. The broadphase
puts the ceiling well past 100 000 particles on this hardware — at
N=100k we're still **3.5× real-time** with substantial headroom.

Comparison: brute force handled 1 000 at 1.12 ms; broadphase handles
100 000 at 4.5 ms — 100× more particles for 4× the cost. Effective
per-particle work has dropped about 25× thanks to the cell-list walk.

## Friction-vs-no-friction (perf cost of the new feature)

| N | No-friction ms/step | With-friction ms/step | Δ |
|---|---|---|---|
| 100 | 0.179 | 0.222 | +24% |
| 200 | 0.252 | 0.376 | +49% |
| 400 | 0.564 | 0.613 | +9% |
| 600 | 0.743 | 0.798 | +7% |
| 1000 | 0.965 | 0.997 | +3% |

Friction adds a few extra FLOPs per pair (decompose tangential velocity,
viscous force, Coulomb cap). At low N the relative overhead is largest
because there's idle time to absorb; at high N the kernel is bandwidth-
bound and friction's cost is hidden.

## Pile-shape effect of friction

At N=1000 in the 0.6 × 0.6 m box, the steady-state pile at step 656 (~10 s
of sim) reads back as `min=0.020 mean=0.076 max=0.158` — a stable ~16 cm
deep pile, ~4 sphere-layers thick. `max y` stays within 0.001 m across the
last 5 readbacks, confirming the pile has settled. Without friction the
particles continue to slowly redistribute under gravity and never quite
stop sliding (the Coulomb cap is what creates an angle of repose).

## Reading this

- **GPU kernel ms/step** is the per-outer-step kernel cost — `cuEventRecord`
  around the 8 substeps × 2 kernels (computeForces + integrate) = 16 launches.
  This is GPU-side time only, not including the periodic host-readback (which
  happens once per 16 outer steps for telemetry).
- **CPU ms/step** is `physics ms/step` from the granular spike: the average
  wall-clock time per simulated step inside ODE on the same machine, with
  basicTimeStep=32 ms (CPU configuration; the GPU path uses 16 ms).
- **Speedup** is a fair within-an-order-of-magnitude comparison even with the
  basicTimeStep difference, because both numbers measure cost per step. The
  GPU at 16 ms must run twice as many steps for the same sim time, so a
  "fair" speedup divides by 2 → e.g. N=400 is still **160×** faster after
  that adjustment.

## What this proves

- The whole motivating constraint of the CUDA plan (CPU physics cliff at
  N≈400) is dissolved: the GPU stays under 1 ms/step at N=1000.
- Real-time headroom on GPU at N=1000 is `16 ms / 0.965 ms ≈ 16×`. Means
  N=2000–4000 should still be real-time pending a measurement.
- The kernel is brute-force O(N²), so doubling N quadruples the work. At
  large N the uniform-grid broadphase from the plan becomes useful — but the
  data here says "not yet": N=1000 is already an order of magnitude past the
  CPU cliff and still has 16× headroom.

## What this does NOT prove

- This is **gravity + sphere-sphere collisions only**. No coupling to ODE
  rigid bodies (a robot wheel pushing through the pile is M3 work — see plan).
- No rendering — particles exist purely as positions in a CUDA buffer. M1
  (GL/CUDA interop) is the path to render them at zero PCIe cost; until then
  the host-readback path serves visualization.
- Contact friction is not modelled (only normal penalty + damping along the
  normal). Particles slip past each other freely tangentially. Adding a
  tangential-spring friction term is one of the next M2 increments.

## How to reproduce

```bash
# Build with CUDA on (default if nvcc is on PATH):
make -C src/omnisim OMNISIM_WITH_CUDA=ON release

# Run the sweep:
python tests/cuda/bench_granular_group.py
# or for a custom range:
python tests/cuda/bench_granular_group.py --counts 100 500 1000 2000 --duration 8
```
