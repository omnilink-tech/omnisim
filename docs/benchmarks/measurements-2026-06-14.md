# OmniSim first-hand performance measurements — 2026-06-14

Raw data backing `docs/benchmarks/performance-comparison.md`. All numbers below were
measured first-hand on this checkout/machine on 2026-06-14, not copied from docs.

> ⚠️ **[Added 2026-08-08.] EVERY ODE NUMBER BELOW IS NOW PERMANENTLY UNREPRODUCIBLE.**
> `bdc02139` deleted the ODE backend (`src/ode/` + `include/ode/`, 106,283 lines). Newton
> with `SolverMuJoCo` is the only physics backend. The measurements stand as a dated
> record; the *comparisons* they support can never be re-derived, in either direction.
>
> Read that carefully in both directions, because this file's history is a lesson in
> exactly this failure mode: the "17× / 33× / unbounded Newton-over-ODE" claim was
> **retired** as an invalid cross-harness ratio, and the same-harness measurement that
> retired it found **ODE faster** (T1 bounce 0.131 vs 3.169 ms/step; T4 pendulum 0.172 vs
> 2.534). Neither the discredited claim nor its refutation can be re-run. The refutation
> was never overturned, so it remains the better-founded of the two — but **do not revive
> the 17×/33× framing on the grounds that its counter-evidence is also frozen.**
>
> Also still true and now unfixable: there is **no measured ODE batched row anywhere**, so
> "Newton is faster because it batches" has no in-tree denominator and no longer can have
> one. See [step-cost-2026-08-06.md](step-cost-2026-08-06.md) (frozen) and
> [../developer/ode-retirement-campaign.md](../developer/ode-retirement-campaign.md).

> **[Added 2026-07-26.] The raw numbers below stand; three of the *derived* claims do
> not.** A same-harness, machine-attributed successor now exists — **OmniBench**
> ([`tests/benchmarks/omnibench/`](../../tests/benchmarks/omnibench/), campaign report
> [omnibench-2026-07-24.md](omnibench-2026-07-24.md)) — and where the two disagree,
> OmniBench wins: it measures every engine in one harness, on one machine, with the
> machine id on every row. The reasoning is in the header box and §3.5 of
> [performance-comparison.md](performance-comparison.md). Nothing here is deleted; the
> affected lines carry inline caveats:
>
> | Line | Caveat |
> |---|---|
> | Benchmark 2's 40,258 / 162,604 / **661,636** env-steps/s | Real measurements of an **idle, ungraphed, contact-light** model, counting **raw physics steps**. Not comparable to OmniBench's lane-2 rows (see the caveat under that table). |
> | *"~3–4 orders of magnitude faster [than the CPU pipeline]"* | **Retired.** Divides a batched-GPU number by a single-env CPU number. |
> | *"Newton vs ODE per-robot … 17x / 33x / unbounded"* | **Retired.** Cross-harness ratio; the ODE side is whole-engine GUI fps. |

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
OmNewtonBackend readback path). "realtime" = 16.67 ms tick budget / ms-per-step.

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

## Benchmark 2 — GPU-batched MuJoCo-Warp ~~RL~~ **physics** throughput

Script: `projects/policies/research/tools/mjwarp_throughput_poc.py`. Builds the OmniSim quadruped in
Newton, exports the EXACT MuJoCo model Newton simulates to MJCF (zero sim-to-sim gap),
loads it into mujoco_warp, batches to N parallel worlds on cuda:0, times 200 steps.
Model: nq=19 nv=18 nu=24 nbody=14 ngeom=5.

| parallel envs (N) | 200 steps wall | env-steps/s |
|------------------:|---------------:|------------:|
| 256               | 1.27 s         | 40,258      |
| 1024              | 1.26 s         | 162,604     |
| 4096              | 1.24 s         | 661,636     |

> ⚠️ **[2026-07-26] Four things about this table, all verifiable from the script and the
> row above — they do not invalidate the numbers, they define what the numbers are.**
> 1. **It is not an RL number** (hence the struck word in the heading). The timed loop is
>    `mjw.step(mw_m, mw_d)` and nothing else — no observations, no reward, no policy
>    forward pass, no learning. It is an *upper bound* on RL throughput.
> 2. **The loop never writes `ctrl` — this is an IDLE model.** That is the same
>    methodology defect `performance-comparison.md` §4.8 criticises in Genesis ("one action
>    then ~999 no-op idle steps"). OmniBench's lane-2 rows redraw random position targets
>    every control step and carry an explicit `idle_guard_ok: true`.
> 3. **No CUDA graph.** OmniBench measured graph capture alone to be worth **1.9×–5.1×**
>    on the OmniSim side (graphed ÷ ungraphed at batch 8192 → 256, machine
>    `65dd6587d5c9`), so an ungraphed figure is not comparable to a graphed one in either
>    direction.
> 4. **`ngeom=5` is for THAT 2026-06-14 export, and this table counts RAW PHYSICS STEPS.**
>    Five collision geoms on a 14-body quadruped is a nearly contact-free model, so the
>    scene is *cheap* and the throughput correspondingly optimistic. OmniBench lane 2's Go2
>    export is `nbody=14`/**`ngeom=14`** with contacts ON (~24.9 contact candidates per
>    world), and one lane-2 env-step is a **control step = 8 physics substeps**. So
>    OmniBench's lane-2 rows are **different units on a different export** — a replacement,
>    not a re-measurement of this row. Multiply a lane-2 control-step rate by 8 before
>    setting it beside anything here, and even then the models differ.
>
> The live successor rows (contacts ON, actions never idle, graphed both sides) are in
> [omnibench-2026-07-24.md](omnibench-2026-07-24.md): OmniSim's own embedded solver at
> **129,431 control-env-steps/s @4096** on this same 3060 (`9722d23d12a3`) and **650,487
> @8192** on an RTX 4090 (`65dd6587d5c9`).

Key result: **near-linear scaling and GPU not saturated even at 4096 envs** — wall
time per 200 steps is flat (~1.25 s) while env count grows 16x, so throughput grows
~16x (40k → 662k env-steps/s). ~~Contrast the legacy CPU Webots pipeline: ~30–150
env-steps/s for 1–2 envs. The GPU-batched path is ~3–4 orders of magnitude faster
on the same machine.~~

> ⛔ **[SUPERSEDED 2026-07-26]** The struck sentences divide a **batched-GPU** number by a
> **single-env CPU** number — the comparison OmniBench honesty rule 1
> ([`SPEC.md`](../../tests/benchmarks/omnibench/SPEC.md)) forbids outright, and which
> `performance-comparison.md` §1 also forbids ("units never to mix in one column"). There
> is still **no measured ODE batched row anywhere** — OmniBench lane 2 has no ODE variant
> — so the ratio has no same-harness denominator and cannot be repaired, only retired.
> What is defensible: **ODE has no batched path; Newton does, and it scales with batch
> size.** The scaling observation in the unstruck sentence is unaffected.

## Documented OmniSim numbers reused in the paper (clearly attributed there)

- ODE baseline (archive/fps-optimization-journey.md): 2-husky head-on 10.2 fps
  (0.16x realtime); +200-particle pool 1.3 fps (0.02x); 1-husky cube drop 20.1 fps.
  ⚠️ **[2026-07-26] These are WHOLE-ENGINE windowed GUI fps** — ODE physics *plus* WREN
  rendering *plus* the Python damage system *plus* scene-graph traversal — not an ODE
  step rate. That doc's own conclusion is that removing 200 off-screen Solids gave an 8x
  fps gain, and the damage layer alone measures ~2x whole-sim cost. Cite them as
  whole-engine fps or not at all.
- ~~Newton vs ODE per-robot (engine-migration-plan.md §13.5): 1 husky 17x, 2 huskies
  33x, 10+ huskies ODE DNF -> Newton 2.9–3.4 ms (effectively unbounded speedup).~~
  > ⛔ **[SUPERSEDED 2026-07-26 — cross-harness; do not quote.]** Those ratios divide the
  > whole-engine ODE fps directly above by a **bare Python Newton solver probe**
  > (`scripts/xpbd_probes/bench_newton_scaling.py`, since removed) — no renderer, no
  > controllers, no engine loop. Measured in **one** harness (OmniBench lane 1, dt=4 ms,
  > post-fix, whole `omnisim-bin` process both sides), the direction reverses: **ODE is
  > cheaper per step than Newton by 1–2 orders of magnitude on these 1–10-body scenes** —
  > T1 bounce **0.131 (ODE) vs 3.169 (Newton)** ms/step and T4 pendulum **0.172 vs 2.534**
  > on machine `9722d23d12a3`; **0.013–0.181 vs 0.361–26.9** across all seven scenes on
  > `65dd6587d5c9`. That is the expected shape at this body count, where Newton pays a
  > fixed per-step GPU launch + host-readback cost. **Newton's real case is batching**, not
  > per-step cost. Rows: `tests/benchmarks/omnibench/results/9722d23d12a3/2026-07-24/lane1/`
  > (suite `omnibench/v1-postfix`) and
  > `tests/benchmarks/omnibench/results_4090_validate/results_validate/lane1/`.
- Render wgpu vs WREN (wgpu-renderer-status.md, city baseline, 1896x1113, 3,523
  draws): render cost/frame 21.5 ms (WREN) vs 8–9 ms (wgpu, ~2.4x); whole-sim FPS
  14.0 vs 14.3 (sim-bound tie); CHANGELOG headline "2.7x faster main view".
- RL on this 3060 (rl-accelerated-training.md / rl-current-state.md): SB3 single-env
  ~161 steps/s; MJX 1024 envs ~20k–50k; G1 stand verified ~27k–62k env-steps/s on the
  3060 (the "132k / RTX 5070" headline is unverified on documented hardware).
