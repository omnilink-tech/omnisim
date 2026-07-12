# OmniSim first-hand performance measurements — 2026-06-14

Raw data backing `docs/benchmarks/performance-comparison.md`. All numbers below were
measured first-hand on this checkout/machine on 2026-06-14, not copied from docs.

## Test machine

- CPU: AMD Ryzen 7 5800H (AMD64 Family 25 Model 80, 8C/16T)
- GPU: NVIDIA GeForce RTX 3060 **Laptop** GPU, 6 GiB, sm_86, mempool enabled
- Stack: Warp 1.13.0, Newton 1.2.0, MuJoCo 3.8.1, CUDA Toolkit 12.9 / Driver 13.2
- Class: a ~2021 gaming laptop GPU — deliberately modest, NOT a datacenter card.
  Competitor headline numbers are usually on A100/H100/RTX 4090/5090; keep that in
  mind when comparing absolute throughput.

## Benchmark 1 — Newton XPBD physics scaling (single-env, CPU-readback each step)

Script: `scripts/xpbd_probes/bench_newton_scaling.py`. **[Note 2026-07-09: that probe
was later removed (removal recorded in
`docs/developer/engine-migration-plan.md`); these numbers stand as a recorded
2026-06-14 measurement. The surviving benchmark harness is
`tests/benchmarks/optim_bench.py`.]** Husky model = chassis + 4
wheels + 4 revolute actuators per robot; SolverXPBD, 10 iters, dt=1/60, 500 timed
steps including per-step host readback of chassis pose (mirrors the engine's
WbNewtonBackend readback path). "realtime" = 16.67 ms tick budget / ms-per-step.

| huskies | bodies | dof | ms/step | fps   | realtime |
|--------:|-------:|----:|--------:|------:|---------:|
| 1       | 5      | 10  | 3.683   | 271.5 | 4.53x    |
| 2       | 10     | 20  | 3.651   | 273.9 | 4.57x    |
| 5       | 25     | 50  | 3.653   | 273.7 | 4.56x    |
| 10      | 50     | 100 | 3.718   | 269.0 | 4.48x    |

Key result: **near-perfect weak scaling** — step time is essentially flat (3.68 →
3.72 ms) as the body count grows 10x (5 → 50 bodies). The GPU solver is nowhere near
saturated at this scale; cost is dominated by fixed per-step overhead, not body count.
~270 physics fps / ~4.5x real-time on a laptop GPU.

(Project's recorded §13.5 sweep in engine-migration-plan.md shows 2.94–3.42 ms/step /
~290–342 fps for the same sweep without the per-step numpy readback in the hot loop;
the ~0.7 ms delta here is the readback. Both tell the same story.)

## Benchmark 2 — GPU-batched MuJoCo-Warp RL throughput

Script: `projects/policies/research/tools/mjwarp_throughput_poc.py`. Builds the OmniSim quadruped in
Newton, exports the EXACT MuJoCo model Newton simulates to MJCF (zero sim-to-sim gap),
loads it into mujoco_warp, batches to N parallel worlds on cuda:0, times 200 steps.
Model: nq=19 nv=18 nu=24 nbody=14 ngeom=5.

| parallel envs (N) | 200 steps wall | env-steps/s |
|------------------:|---------------:|------------:|
| 256               | 1.27 s         | 40,258      |
| 1024              | 1.26 s         | 162,604     |
| 4096              | 1.24 s         | 661,636     |

Key result: **near-linear scaling and GPU not saturated even at 4096 envs** — wall
time per 200 steps is flat (~1.25 s) while env count grows 16x, so throughput grows
~16x (40k → 662k env-steps/s). Contrast the legacy CPU Webots pipeline: ~30–150
env-steps/s for 1–2 envs. The GPU-batched path is ~3–4 orders of magnitude faster
on the same machine.

## Documented OmniSim numbers reused in the paper (clearly attributed there)

- ODE baseline (archive/fps-optimization-journey.md): 2-husky head-on 10.2 fps
  (0.16x realtime); +200-particle pool 1.3 fps (0.02x); 1-husky cube drop 20.1 fps.
- Newton vs ODE per-robot (engine-migration-plan.md §13.5): 1 husky 17x, 2 huskies
  33x, 10+ huskies ODE DNF -> Newton 2.9–3.4 ms (effectively unbounded speedup).
- Render wgpu vs WREN (wgpu-renderer-status.md, city baseline, 1896x1113, 3,523
  draws): render cost/frame 21.5 ms (WREN) vs 8–9 ms (wgpu, ~2.4x); whole-sim FPS
  14.0 vs 14.3 (sim-bound tie); CHANGELOG headline "2.7x faster main view".
- RL on this 3060 (rl-accelerated-training.md / rl-current-state.md): SB3 single-env
  ~161 steps/s; MJX 1024 envs ~20k–50k; G1 stand verified ~27k–62k env-steps/s on the
  3060 (the "132k / RTX 5070" headline is unverified on documented hardware).
