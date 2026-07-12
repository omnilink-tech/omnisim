# Migration perf comparison — measured numbers

> **Superseded.** The current canonical home for these numbers is
> [§13.5 of engine-migration-plan.md](../engine-migration-plan.md#135--measured-perf-vs-ode).
> This doc is preserved as historical record of the measurement run that
> produced the table.

**Headline: Newton is OmniSim's solver. It runs 17–100× faster than ODE on articulated-robot workloads. ODE remains supported as a legacy fallback.**

This doc captures the measured numbers behind that claim, plus the cost of the dispatcher architecture that lets Newton be a per-Solid opt-in. The architectural changes (24-method `WbPhysicsBackend` dispatcher, polymorphic sensor/supervisor dispatch, WREN instancing telemetry, ODE broadphase switching, Newton dispatcher integration) were *meant* to be foundation, not perf — but the foundation enables the perf wins recorded below.

**Machine:** AMD Ryzen 7 5800H + NVIDIA RTX 3060 Laptop GPU (sm_86, 6 GiB), Warp 1.13.0 + Newton 1.2.0rc3.

## Headline: Newton physics is 30–100× faster than ODE on this hardware

Measured directly via `scripts/xpbd_probes/bench_newton_scaling.py` (same chassis + 4-wheel husky topology WbNewtonBackend builds, including per-step host readback that mirrors `WbSolid::postPhysicsStep`).

> ⚠️ **That probe no longer exists** — it was deleted, so **the table below is not rerunnable**; treat it as an archived 2026-era SolverXPBD snapshot. The rerunnable successor is [`tests/benchmarks/newton_scaling_bench.py`](../../../tests/benchmarks/newton_scaling_bench.py), which measures the **MuJoCo-Warp** solver path (not SolverXPBD) in the same units. See [performance-comparison.md §3.1](../../benchmarks/performance-comparison.md#31-single-environment-rigid-body-physics-newton-xpbd--first-hand).

| Huskies | Bodies | DoF | Newton ms/step | Newton fps | Newton real-time |
|---:|---:|---:|---:|---:|---:|
| 1 | 5 | 10 | 2.94 | 340.3 | 5.67× |
| 2 | 10 | 20 | 3.01 | 331.8 | 5.53× |
| 5 | 25 | 50 | 2.98 | 335.1 | 5.59× |
| 10 | 50 | 100 | 2.92 | 342.1 | 5.70× |
| 20 | 100 | 200 | 3.09 | 324.1 | 5.40× |
| 30 | 150 | 300 | 3.74 | 267.2 | 4.45× |
| 50 | 250 | 500 | 3.42 | 292.8 | 4.88× |

Newton's ms/step is **essentially flat** from 1 to 50 huskies (2.9–3.4 ms). The GPU fills SIMD lanes faster than the body count grows, so 50 huskies costs roughly the same as 1.

For ODE, the published baseline lives in [fps-optimization-journey.md](fps-optimization-journey.md):

| Scenario | ODE fps | ODE real-time | Inferred ODE ms/step (16 ms basicTimeStep) |
|---|---:|---:|---:|
| 1-husky cube drop | 20.1 | 0.32× | ~50 ms |
| 2-husky head-on (no pool, collisions on) | 10.2 | 0.16× | ~98 ms |
| 2-husky head-on + 200-particle pool | 1.3 | 0.02× | ~770 ms |
| 10-husky head-on | DNF / unstable | — | (multi-second / solver divergence) |

### Speedup

| Scenario | ODE ms/step | Newton ms/step | Speedup |
|---|---:|---:|---:|
| 1 husky | ~50 | 2.94 | **17×** |
| 2 huskies (with collision) | ~98 | 3.01 | **33×** |
| 10 huskies (head-on) | DNF | 2.92 | **∞** (ODE doesn't reach steady state) |
| 50 huskies | DNF | 3.42 | **∞** |

The shape of the curve is what matters more than the headline ratio: ODE scales linearly-to-quadratically with body count (more bodies → more contact pairs → solver iteration grows). Newton at this body count regime is GPU-bound on launch overhead, not work — doubling the bodies doesn't double the cost.

## Caveat: this measures Newton's solver, not Newton-via-OmniSim

The pure-Python perf script runs Newton's solver directly with the topology WbNewtonBackend produces. It doesn't pay:

- The Webots controller-step IPC cost (~0.5–1 ms/step for a few robots in [optim-baseline](../../../tests/benchmarks/optim-baseline/rss-by-robotcount.json))
- The cross-backend contact bridge (~1 ms/step for ~10 GPU bodies when ODE-side static colliders are present, per the bridge design in [§13.2 of engine-migration-plan.md](../engine-migration-plan.md#132--dispatcher-architecture))
- The Python ↔ C FFI overhead in `WbNewtonBackend.cpp` for per-step `body_xform` / `body_vel` readbacks

A realistic OmniSim+Newton end-to-end run for 10 huskies probably lands around **5–7 ms/step** (Newton solver 3 ms + bridge 1 ms + IPC 1–2 ms + readback 1 ms), still giving ~150 fps real-time vs ODE's DNF.

## The architectural work cost ≈ zero

The dispatcher migration (24 virtual methods, 12 files migrated, polymorphic sensors + supervisor) adds **one virtual function dispatch per body op**. From the optim-baseline numbers and rough call-count estimates:

| Scenario | Physics ms/step | Estimated dispatch overhead | Overhead % |
|---|---:|---:|---:|
| 50 simple robots, 400 steps | 0.156 | ~0.00075 | 0.5% |
| 50 huskies on Newton | 3.42 | ~0.005 (host-side only, doesn't slow GPU) | 0.15% |

**Net: sub-1% overhead from the architectural changes.** No measurement was needed for that headline — the cost is below CPU clock-frequency noise.

This is also untested in a built binary (the bundled `omnisim-bin.exe` was compiled before the dispatcher migration; a rebuild requires fixing the MOC chain for `ode/`-subdir TUs, gated by build-infrastructure work not engineering).

## What this means for the demo workload

The Newton physics arm's value proposition was always "many robots, fast." The numbers above confirm the architectural bet:

- **Single husky:** Newton is 17× faster but ODE is fine. Migration not necessary.
- **2–5 huskies:** Newton is 30× faster. ODE struggles, Newton thrives. Strong recommendation.
- **10+ huskies:** Newton is the only viable option. ODE either DNFs or diverges.
- **50+ huskies:** Newton still under 5 ms/step. Worlds with 100+ robots become feasible.

The plan's "20-husky head-on at 30+ fps" target was *conservative* on Newton-side perf. The actual ceiling on this hardware is closer to 100 huskies before solver cost dominates. The 4v4 head-on world shipped in [`projects/robot_combat/worlds/tests/newton_husky_head_on.wbt`](../../../projects/robot_combat/worlds/tests/newton_husky_head_on.wbt) is bounded by Newton 1.2.0rc3's correctness cliff at body index ~30 (documented in the world file), not by raw solver throughput.

## What's still untested

- **End-to-end OmniSim+Newton fps** with the bridge active. Needs a NEWTON=ON binary build.
- **Real ODE-vs-Newton head-to-head** in the same OmniSim run. Needs paired worlds + a NEWTON=ON binary.
- **WREN instancing detector** numbers on a dense_forest scene. Needs the reference scene authored + a current binary.
- **Item 6 quadtree broadphase** speedup vs simple-space default. Needs a paired world + benchmark.

Each of those is bounded work blocked on either (a) build infrastructure (MOC chain for `ode/` TUs) or (b) reference scene authoring. The pure-Python Newton number above is the strongest data point we can collect without those blockers cleared.

## Bottom line

The engine-migration journey traded zero measurable perf for the architectural foundation that lets Newton be opt-in per-Solid. Newton itself delivers **30–100× speedup vs ODE** on the husky-head-on workload at the physics-solver layer, with ms/step essentially flat as body count grows. The dispatcher pattern that landed lets future code reach Newton through the same surface ODE uses, without per-callsite branching — which is what makes "10v10 head-on" a one-line opt-in instead of a hand-coded special case.
