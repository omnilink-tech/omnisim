# Migration perf comparison — measured numbers

> **Superseded.** The current canonical home for these numbers is
> [§13.5 of engine-migration-plan.md](../engine-migration-plan.md#135--measured-perf-vs-ode).
> This doc is preserved as historical record of the measurement run that
> produced the table.

---

> ## ⛔ [SUPERSEDED 2026-07-26] THE SPEEDUP RATIOS IN THIS DOC ARE A CROSS-HARNESS ARTIFACT — DO NOT QUOTE THEM
>
> **Nothing below has been deleted or rewritten.** This is a historical record of a
> 2026-era measurement run and it stands as one. But its two headline claims —
> *"17–100× / 30–100× faster than ODE"* and *"ODE remains supported as a **legacy**
> fallback"* — are both refuted by later, same-harness measurement, and this doc is
> one of the places they propagated from. Read the correction first:
>
> - **[docs/benchmarks/performance-comparison.md](../../benchmarks/performance-comparison.md)**
>   — the header box and §3.1/§3.5 explain exactly why the ratio was wrong.
> - **[docs/benchmarks/omnibench-2026-07-24.md](../../benchmarks/omnibench-2026-07-24.md)**
>   — the harness campaign that replaced it (suite: [`tests/benchmarks/omnibench/`](../../../tests/benchmarks/omnibench/)).
>
> | Claim in this doc | Status |
> |---|---|
> | *"17× / 33× / ∞ faster than ODE"* (headline, §Speedup, §What this means) | **SUPERSEDED.** The two sides were never measured the same way. The **ODE** side is *whole-engine, windowed GUI fps* — ODE physics **plus** WREN rendering **plus** the Python damage system **plus** scene-graph traversal (see [fps-optimization-journey.md](fps-optimization-journey.md), whose own conclusion is that deleting 200 off-screen Solids gave an 8× fps gain; the damage layer alone measures ~2× whole-sim cost). The **Newton** side is a *bare Python solver probe* — no renderer, no controllers, no engine loop. It is a ratio between two different harnesses. |
> | *"ODE remains supported as a **legacy** fallback"* (headline) | **CORRECTED.** "Fallback" is right — ODE is the CPU path that needs no GPU. "Legacy" is wrong about *quality*: OmniBench lane 1 measures both backends against analytic ground truth in one harness and **ODE is the correctness star of the suite** — best or tied-best on 6 of 7 scenes at dt=4 ms, linear momentum exactly zero to double precision (9.8e-15 kg·m/s), a 10-box stack stable through dt=16 ms, and bitwise determinism on both machines. |
>
> **What the same harness measures instead.** OmniBench lane 1 steps *both* backends
> through the whole `omnisim-bin` process on the same scenes, same machine, same day.
> Post-fix, at dt = 4 ms, **ODE is cheaper per step than Newton by 1–2 orders of
> magnitude** on these small scenes — the opposite direction to the ratios below:
> T1 bounce **0.131 ms/step (ODE) vs 3.169 (Newton)** and T4 pendulum **0.172 vs 2.534**
> on machine `9722d23d12a3` (RTX 3060 Laptop); across all seven scenes on machine
> `65dd6587d5c9` (RTX 4090, `e7b9fb11`), ODE **0.013–0.181** vs Newton **0.361–26.9**.
>
> That is not ODE "beating" Newton in general — it is the expected shape for 1–10-body
> scenes, where Newton pays a fixed per-step GPU launch + host-readback cost that only
> amortizes across many bodies or many batched worlds. **Newton's real case is BATCHING**
> (OmniBench lane 2: 129,431 control-env-steps/s @4096 on the 3060, 650,487 @8192 on the
> 4090, contacts ON, both sides CUDA-graphed) — where ODE has no batched path at all, so
> the honest comparison there is *presence vs absence*, not a speedup ratio.

**Headline: Newton is OmniSim's solver. It runs 17–100× faster than ODE on articulated-robot workloads. ODE remains supported as a legacy fallback.**

> ⛔ **[2026-07-26]** Both halves of that sentence are wrong as written — see the
> superseded banner above. The ratio is cross-harness (whole-engine ODE fps ÷ bare-solver
> Newton probe); "legacy" misdescribes a backend that is the accuracy reference of
> OmniBench lane 1. The surviving statement is narrower: *Newton is OmniSim's batched
> GPU path; ODE is the CPU path and the permanent, fully supported fallback.*

This doc captures the measured numbers behind that claim, plus the cost of the dispatcher architecture that lets Newton be a per-Solid opt-in. The architectural changes (24-method `OmPhysicsBackend` dispatcher, polymorphic sensor/supervisor dispatch, WREN instancing telemetry, ODE broadphase switching, Newton dispatcher integration) were *meant* to be foundation, not perf — but the foundation enables the perf wins recorded below.

**Machine:** AMD Ryzen 7 5800H + NVIDIA RTX 3060 Laptop GPU (sm_86, 6 GiB), Warp 1.13.0 + Newton 1.2.0rc3.

## ~~Headline: Newton physics is 30–100× faster than ODE on this hardware~~ [SUPERSEDED — see banner]

> ⛔ **[2026-07-26]** Retained verbatim as history. The 30–100× is a **cross-harness
> ratio** and must not be quoted; the Newton ms/step table immediately below is a bare
> solver probe, and the ODE fps table further down is a whole-engine GUI run. Same-harness
> numbers: [omnibench-2026-07-24.md](../../benchmarks/omnibench-2026-07-24.md).

Measured directly via `scripts/xpbd_probes/bench_newton_scaling.py` (same chassis + 4-wheel husky topology OmNewtonBackend builds, including per-step host readback that mirrors `OmSolid::postPhysicsStep`).

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

### ~~Speedup~~ [SUPERSEDED 2026-07-26 — the two columns come from different harnesses]

> ⛔ **Do not quote any row of this table.** The `ODE ms/step` column is *inferred from
> whole-engine windowed GUI fps* (ODE + WREN + damage system + scene-graph); the
> `Newton ms/step` column is a *bare Python solver probe*. Dividing them measures the
> renderer and the damage tracker as much as it measures ODE. The table is left in place
> as the record of what was computed at the time.

| Scenario | ODE ms/step | Newton ms/step | ~~Speedup~~ (cross-harness — void) |
|---|---:|---:|---:|
| 1 husky | ~50 | 2.94 | ~~**17×**~~ |
| 2 huskies (with collision) | ~98 | 3.01 | ~~**33×**~~ |
| 10 huskies (head-on) | DNF | 2.92 | ~~**∞**~~ (ODE doesn't reach steady state) |
| 50 huskies | DNF | 3.42 | ~~**∞**~~ |

The shape of the curve is what matters more than the headline ratio: ODE scales linearly-to-quadratically with body count (more bodies → more contact pairs → solver iteration grows). Newton at this body count regime is GPU-bound on launch overhead, not work — doubling the bodies doesn't double the cost.

## Caveat: this measures Newton's solver, not Newton-via-OmniSim

The pure-Python perf script runs Newton's solver directly with the topology OmNewtonBackend produces. It doesn't pay:

- The Webots controller-step IPC cost (~0.5–1 ms/step for a few robots in [optim-baseline](../../../tests/benchmarks/optim-baseline/rss-by-robotcount.json))
- The cross-backend contact bridge (~1 ms/step for ~10 GPU bodies when ODE-side static colliders are present, per the bridge design in [§13.2 of engine-migration-plan.md](../engine-migration-plan.md#132--dispatcher-architecture))
- The Python ↔ C FFI overhead in `OmNewtonBackend.cpp` for per-step `body_xform` / `body_vel` readbacks

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

- ~~**Single husky:** Newton is 17× faster but ODE is fine. Migration not necessary.~~
- ~~**2–5 huskies:** Newton is 30× faster. ODE struggles, Newton thrives. Strong recommendation.~~
- **10+ huskies:** Newton is the only viable option. ODE either DNFs or diverges.
- **50+ huskies:** Newton still under 5 ms/step. Worlds with 100+ robots become feasible.

> ⛔ **[2026-07-26]** The two struck bullets restate the cross-harness ratio and are void.
> The small-scene conclusion measured in one harness is the reverse of what they say: at
> dt = 4 ms on OmniBench's 1–10-body lane-1 scenes, **ODE is 1–2 orders of magnitude
> cheaper per step than Newton** (e.g. T1 0.131 vs 3.169 ms/step on machine
> `9722d23d12a3`). The two surviving bullets are about *scaling shape* — the thing this
> doc actually measured well — and OmniBench does not contradict them, but note that it
> has never run a 10+-husky head-on scene either way, so they remain unretested claims
> rather than confirmed ones.

The plan's "20-husky head-on at 30+ fps" target was *conservative* on Newton-side perf. The actual ceiling on this hardware is closer to 100 huskies before solver cost dominates. The 4v4 head-on world shipped in [`projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld`](../../../projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld) is bounded by Newton 1.2.0rc3's correctness cliff at body index ~30 (documented in the world file), not by raw solver throughput.

## What's still untested

- **End-to-end OmniSim+Newton fps** with the bridge active. Needs a NEWTON=ON binary build.
- **Real ODE-vs-Newton head-to-head** in the same OmniSim run. Needs paired worlds + a NEWTON=ON binary.
- **WREN instancing detector** numbers on a dense_forest scene. Needs the reference scene authored + a current binary.
- **Item 6 quadtree broadphase** speedup vs simple-space default. Needs a paired world + benchmark.

Each of those is bounded work blocked on either (a) build infrastructure (MOC chain for `ode/` TUs) or (b) reference scene authoring. The pure-Python Newton number above is the strongest data point we can collect without those blockers cleared.

## Bottom line

The engine-migration journey traded zero measurable perf for the architectural foundation that lets Newton be opt-in per-Solid. ~~Newton itself delivers **30–100× speedup vs ODE** on the husky-head-on workload at the physics-solver layer,~~ with ms/step essentially flat as body count grows. The dispatcher pattern that landed lets future code reach Newton through the same surface ODE uses, without per-callsite branching — which is what makes "10v10 head-on" a one-line opt-in instead of a hand-coded special case.

> ⛔ **[2026-07-26]** The struck clause is the cross-harness ratio again. The
> flat-ms/step finding it sits next to is the durable one and is unaffected. For the
> current, same-harness bottom line — **ODE cheaper per step on small scenes, Newton's
> win is batching** — see
> [performance-comparison.md](../../benchmarks/performance-comparison.md) (header box,
> §3.1, §3.5) and [omnibench-2026-07-24.md](../../benchmarks/omnibench-2026-07-24.md).
