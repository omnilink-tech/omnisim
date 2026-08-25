# OmniBench 2026-07-24 — first cross-machine campaign (RTX 3060 Laptop + RunPod RTX 4090)

**2026-07-24.** First full run of OmniBench (`tests/benchmarks/omnibench/`, suite
`omnibench/v0`), the repo's cross-simulator physics benchmark: SimBenchmark /
Erez-Tassa-Todorov-lineage correctness scenes with analytic ground truth (lane 1),
throughput per the post-Genesis credibility checklist — contacts ON, realistic model,
actions never idle, stated GPU, machine fingerprint on every number (lane 2), and
three novel axes: determinism grading, train==deploy structural parity, and
agent-driveability of the HTTP harness (lane 3). Engines: MuJoCo 3.8.1, PyBullet,
OmniSim/ODE *(backend deleted 2026-08-08 — see the banner below)*, OmniSim/Newton
(embedded mujoco-warp, `newtonSolver "mujoco"`).

This report contains only same-harness, same-machine measurements. No published
competitor numbers are quoted (deliberate for v0 — see Open gaps).

> ⚠️ **2026-08-08 — TWO OF THIS CAMPAIGN'S FOUR ENGINES NO LONGER EXIST, AND LANE 1
> HAS LOST ITS ORACLE.** `bdc02139` deleted the ODE backend; `94f04222` removed
> XPBD. Every `omnisim-ode` row below, and the XPBD half of lane 3a, is a
> **historical measurement that can never be re-run** — nothing is retracted, and
> the numbers were honestly obtained on the dates and machines stated, but no
> future run can confirm or refute them.
>
> **The structural consequence, stated precisely: there is no second IN-ENGINE path
> to cross-check the plumbing.** ⚠️ Not "no oracle" — lane 1's oracle is **analytic
> ground truth** and it is untouched. What is gone is the ability to run one `.wbt`
> through two integrations and diff, which is how every Newton integration defect
> here was found. Lane 1 used ODE as that second path and its world generator
> emitted an **`ode_pin`** variant for every
> scene (see the "Bonus engine bug" headline below — that pin was mandatory, not
> incidental). With ODE gone the lane cannot be executed as designed. The frozen
> reference values survive in
> [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json),
> which is now the only ODE artefact in the tree — and it is a **golden file, not
> an oracle**: it can tell you Newton's answer *changed*, never which answer was
> right. The remaining cross-*simulator* arms (MuJoCo 3.8.1, PyBullet) are still
> live and are now the only external check on lane-1 scenes.
>
> **What this campaign's ODE result should still be allowed to say.** "`omnisim-ode`
> was the best-scoring integration in this suite" was true and is the reason the
> retirement was argued on capability and maintenance grounds rather than fidelity
> ones. ⚠️ **Do not compress it to "ODE was more accurate than MuJoCo" — that was
> never measured and the headlines below say the opposite:** bare mujoco scored fine
> on the same scenes, so the deficit was in our integration layer. Do **not** invert
> it into "the fidelity problem was solved" either — the four defects were fixed, but
> the second in-engine path that found them is gone. Scope: [correctness-scope.md](correctness-scope.md). And do not read the deletion as a Newton fidelity
> improvement: the four Newton integration defects this campaign pinned were fixed
> separately in `e7b9fb11`, long before the deletion.
>
> ⚠️ Also note that a validity audit one day before the deletion
> ([`lane1-validity-2026-08-07.md`](lane1-validity-2026-08-07.md)) found that much
> of lane 1's Newton-vs-ODE gap was measuring **modelling difference, not error** —
> T1 scores a restitution coefficient MuJoCo does not have, T1/T6 give Newton a
> duplicate floor, two T2 metrics are broken as instruments, and the substeps
> section has no data behind it. Read that audit before quoting any "Newton loses
> N of 11" figure from this report.
>
> Campaign record: [`../developer/ode-retirement-campaign.md`](../developer/ode-retirement-campaign.md).

## Machines

**Four machine ids appear in the rows — three campaigns plus one fingerprint
artifact.** Do not conflate the two 4090s: `6fa66da0cde0` is the **pre-fix** pod
and `65dd6587d5c9` the **post-fix** validation pod, on different drivers,
kernels and binaries.

| | 9722d23d12a3 (local) | 6fa66da0cde0 (pod, pre-fix) | 65dd6587d5c9 (pod, post-fix) |
|---|---|---|---|
| venue | laptop, Windows 11 | RunPod RTX 4090, EU-RO-1 (terminated after one campaign) | RunPod RTX 4090, pod `mpg4buqmwt9asg` (terminated) |
| GPU | RTX 3060 Laptop 6 GB (driver 596.36) | RTX 4090 24 GB (driver 570.195.03) | RTX 4090 24 GB (driver 580.126.20) |
| CPU / OS | AMD Family 25 Model 80, 16 threads | x86_64, 32 cores; Linux 6.8.0-90 | x86_64, 32 cores; Linux 6.8.0-107 |
| build / binary sha | v0 `a74b7699` / `5087587d3e4b5940`; post-fix `b5c089d4` / `f95073f976323787` | `95f5a2c` / `47c57f8bcec1c9ec` | rows say `95f5a2c` *or* null / **`36732b708231390e`** |
| python / numpy | 3.12.9 / 2.4.4 | 3.11.10 / 2.4.6 | 3.11.10 / 2.4.6 |
| raw results | `tests/benchmarks/omnibench/results/9722d23d12a3/2026-07-24/` | `tests/benchmarks/omnibench/results_4090/results/` | `tests/benchmarks/omnibench/results_4090_validate/results_validate/` |

Two id caveats that bite anyone filtering the jsonl:

- **A fourth id, `6849f1be4974`, appears on exactly one row** — the post-fix
  pod's T7 `spin` newton row (21:08:52Z). Same recorded `host` fingerprint
  (`80715bbfc6a0`), same binary sha, same libController as `65dd6587d5c9`; that
  run's `nvidia-smi` query timed out (`gpu: "nvidia-smi present but query TIMED
  OUT (dGPU asleep?)"`), and since the id is
  `sha256(gpu_model|cpu|os|host)[:12]`
  ([`env_fingerprint.py`](../../projects/policies/common/env_fingerprint.py)) a
  degraded GPU string mints a new id. Same machine, not a fourth box — but it
  is the **headline T7 cell**, so cite it as such.
- **Suite-tag warning:** the post-fix pod's rows are tagged
  `suite: "omnibench/v0"` but were **scored by the v1 scorer** (they carry
  `energy_blowup_max`, `angular_momentum_drift_rel_v0`,
  `angular_momentum_drift_abs`). A tag-based filter mis-buckets them: v1
  semantics under a v0 label. The local post-fix rows are correctly tagged
  `omnibench/v1-postfix`.

Reproduce: `python tests/benchmarks/omnibench/run_all.py` (writes
`results/<machine_id>/<utc-date>/` with per-lane jsonl, logs, MANIFEST.json,
summary.md). The pod campaigns ran from the private ops tree (not in the public
snapshot); the shipped suite reproduces every local-machine number with `run_all.py`.

**Honesty rules (from SPEC.md, binding on this report):** never compare a
batched-GPU number to a single-env CPU number; OmniSim's Newton backend embeds
mujoco-warp, so accuracy deltas vs MuJoCo are solver-family-internal and are framed
as *integration fidelity*, never "beating MuJoCo"; losses are reported as
prominently as wins — `deviations` and dropped configs are part of the result; and
every number quoted outside `results/` carries its machine id. Every deviation the
implementers had to take (restitution calibration, friction-cone overrides, the
ODE world-pinning workaround) is recorded in the result rows and summarized below.

## Lane 1 — physics correctness (dt = 4 ms unless noted)

Lane-1 metric values reproduced across the two v0 machines (9722d23d12a3 vs the
pre-fix pod 6fa66da0cde0) to a striking degree: of 180 comparable OmniSim metric
cells, 124 are **numerically identical**; of the 56 that differ, 29 are at
rel < 1e-9 and only 13 at rel ≥ 1e-3 — 9 of those in T5, 3 in T6 creep, 1 the
T4 dt=32 blow-up artifact. mujoco and pybullet are **not** identical at full
precision: only 97 of their 180 cells are bit-identical, and at dt=4 mujoco's
T5 `angular_momentum_drift_rel` diverges in the **4th significant figure**
(2.0898410169824966 vs 2.08856949244862; pybullet's at the 6th). Read the
cross-machine claim as *identical to the 3–4 s.f. these tables print*, with T5
and T6 creep the places where it breaks. Tables below show one
metric column (identical on both machines unless footnoted) and per-machine
wall-clock. Wall ms/step includes each engine's full step path; the OmniSim rows
step the whole engine process, the mujoco/pybullet rows are bare library calls —
compare shapes, not absolute cross-engine wall numbers.

### T1 `bounce` — restitution (rel RMSE over 5 peaks)

| engine | rmse_rel | wall ms/step 3060 / 4090 |
|---|---|---|
| mujoco | 0.1478 | 0.0028 / 0.0013 |
| pybullet | 0.0734 | 0.0042 / 0.0012 |
| omnisim-ode | **0.0179** | 2.85 / 2.03 |
| omnisim-newton | 0.0381 | 2.99 / 2.21 |

Sweep story: mujoco's fixed dt=1 ms solref mapping goes unstable at dt≥8 ms
(RMSE 35.97 / 33.52 / 396.5 at 8/16/32 — energy *gain*, the honest sweep result of
freezing the calibrated mapping). omnisim-newton is the best small-dt performer
(0.0103 at dt=1) after its own recorded calibration (`OMNISIM_NEWTON_CONTACT_KD=7`,
~0.08 m soft-contact penetration at impact — recorded, not hidden).

### T2 `incline` — friction-cone stick/slip (μ=0.5, θc=26.57°)

| engine | stick_violation_max (m) | slide_accel_rel_err | transition_angle_err (deg) |
|---|---|---|---|
| mujoco | 0.000442 | 0.0062 | 0.065 |
| pybullet | 0.00195 | 6.6e-05 | 0.065 |
| omnisim-ode | **2.9e-05** | **8.8e-05** | 0.065 |
| omnisim-newton | 0.181 | 0.191 | **4.065** |

**omnisim-newton defect #1 (friction-cone offset):** the stick→slip transition
sits 4.07° below the analytic 26.57° at every dt ≤ 16 — i.e. effective μ ≈
tan(22.5°) = **0.414 vs the configured 0.5** — and sub-critical boxes slide
0.18–0.27 m instead of sticking. dt-independent and bit-identical on both machines.

### T3 `roll` — rolling without slip (a = 5/7·g·sinθ)

| engine | roll_accel_rel_err | slip_ratio |
|---|---|---|
| mujoco | 0.0010 | 0.0017 |
| pybullet | **2e-15** | 4e-15 |
| omnisim-ode | 0.0011 | 0.0018 |
| omnisim-newton | **0.4763** | 0.0017 |

**omnisim-newton defect #2 (rolling inertia):** acceleration is **47.63% low**,
identical to 6+ digits at dt ∈ {1,2,4,8} on both machines, while slip_ratio stays
≈0 — the sphere rolls cleanly but as if its rolling inertia were wrong. A
dt-independent, deterministic constant-factor error points at the integration
layer's inertia/contact plumbing, not solver dynamics.

### T4 `pendulum_energy` — 3-link chain, 10 s (drift normalized by peak KE)

| engine | energy_drift_rel | drift slope (/s) |
|---|---|---|
| mujoco | 0.387 | −0.048 |
| pybullet | 0.386 | −0.048 |
| omnisim-ode | **0.130** | −0.013 |
| omnisim-newton | 0.497 | −0.061 |

At dt=1 ms omnisim-ode is 0.009 — an order of magnitude better than every other
engine at any dt tested. The dt=32 rows are artifacts (see Metric caveats).

### T5 `momentum` — floating chain, gravity off (linear must be 0)

| engine | linear_momentum_max (kg·m/s) | angular_momentum_drift_rel |
|---|---|---|
| mujoco | 5.85 | 2.09 |
| pybullet | 6.03 | 2.20 |
| omnisim-ode | **9.8e-15** | 0.059¹ |
| omnisim-newton | **18.7** | 0.9999 |

¹ 0.396 on the 4090 — the only lane-1 metric that materially differs across
machines; it is noisy by construction (Metric caveats). The linear-momentum result
(≈1e-14, exact zero to double precision) reproduces on both machines.

> ⚠ **CORRECTED 2026-08-06 — defect #3 is NOT an integration-layer bug.** It was
> filed below as one, on the reasoning that raw mujoco scores fine on the same
> scenes so the fault must be ours. Measured directly on a minimal free-floating
> two-link chain driven by an INTERNAL couple (net external force and torque both
> zero, so the centre of mass must not move at all):
>
> | | COM drift |
> |---|---|
> | dt = 4 ms | 0.0770 m |
> | dt = 2 ms | 0.0160 m |
> | dt = 1 ms | 0.0031 m |
>
> It converges with the timestep, so it is **truncation error**, and raising the
> solver iterations from the default to 50 and then 200 changes it by **not one
> bit** — so it is the integrator, not the constraint solve. Two further facts
> place it: a pure torque on a SINGLE free body injects exactly zero linear
> momentum on Newton (so the external-wrench plumbing is clean), and ODE reads
> exactly 0.000000 on the same couple because its impulse-based solver conserves
> momentum by construction rather than approximately.
>
> So the honest statement is that **ODE conserves momentum exactly and MuJoCo
> conserves it to the order of its integrator** — a characterisable property of
> the solver, not a bug we can fix in the plumbing. The T5 number below stands as
> measured; only its attribution changes. Reducing dt, or an integrator with
> better conservation, is the lever.
>
> ⚠ The scorer's figure additionally depends on WHICH readback it uses: on the
> lane-1 chain, momentum summed from the velocity readback peaks at 3.910 while
> the same run's position-derived momentum peaks at 1.531, and per body the two
> disagree by up to 0.9 m/s. That gap is unexplained and is its own open question.

**omnisim-newton defect #3 (momentum leak):** 17.2–24.3 kg·m/s of linear momentum
injected across the dt sweep (both machines), and angular momentum is fully lost
(drift ratio ≈1.0 at every dt). The run deviations also record the trigger the
implementers had to work around: the Newton backend gives motorized hinges a
hardcoded position servo, so the torque phase uses a supervisor addTorque couple —
and momentum is still injected.

### T6 `stack` — 10-box tower, 10 s (survivors / creep m·s⁻¹ / penetration m)

| engine | dt=4 | largest dt with 10 survivors |
|---|---|---|
| mujoco | 10 / 2.4e-06 / 0.00062 | 16 ms |
| pybullet | 10 / 1.9e-04 / 0.00036 | 4 ms |
| omnisim-ode | **10 / 5.2e-08 / 0.00025** | 16 ms |
| omnisim-newton | 10 / 1.1e-03 / 0.00062 | 4 ms (collapses to 1 box at 8 ms) |

### T7 `spin` — Dzhanibekov box, gravity off (|L| and rot KE conserved)

| engine | angmom_drift_rel | rot_ke_drift_rel |
|---|---|---|
| mujoco | 0.0063 | 0.0045 |
| pybullet | 2.0e-05 | 3.6e-08 |
| omnisim-ode | **1.9e-05** | **3.5e-08** |
| omnisim-newton | **1.0** | **1.0** |

**omnisim-newton defect #4 (total spin loss):** drift ratio exactly 1.0 at every
dt on both machines — the free body loses all angular momentum and rotational KE.
Related integration finding from the same runs: supervisor `setVelocity` is not
plumbed to Newton bodies at all (measured ω stays ~0; the recorder had to fall
back to a torque-impulse spin-up). PyBullet's number carries its own recorded
caveat: it damps the off-axis seed so the intermediate-axis tumble never erupts in
the 10 s window — its conservation describes a near-steady spin.

### ⚡ 2026-08-06 — `newtonSubsteps 4` flips three lane-1 rows to Newton

Measured on the current binary, dt=4 ms, same machine, the ONLY change being
`OMNISIM_NEWTON_SUBSTEPS=4` (default is 1):

| metric | ODE | Newton, substeps=1 | Newton, substeps=4 | winner |
|---|---|---|---|---|
| T4 `energy_drift_rel` | 0.130 | 0.381 | **0.1165** | **Newton** |
| T4 drift slope (/s) | −0.013 | −0.0473 | **−0.0116** | **Newton** |
| T7 `angmom_drift_rel` | 1.935e-05 | 2.548e-05 | **1.610e-06** | **Newton, 12×** |
| T7 `rot_ke_drift_rel` | 3.466e-08 | 7.954e-08 | 5.923e-08 | ODE (gap 2.3× → 1.7×) |
| T5 `linear_momentum_max` | 9.78e-15 | 3.910 | 1.753 | ODE (gap halved) |

This is the same root cause the T5 correction identified: **integrator
truncation**. Substepping shrinks the effective integration step without
changing the control rate, so every metric limited by truncation improves, and
nothing that is limited by the constraint solve moves (T6's creep is
bit-identical either way).

**Newton now wins 3 of 11 lane-1 metrics (T3 roll, T4 energy, T7 angular
momentum), ties 2, and loses 6** — against 1 win before this run.

⚠ **AND IT IS NOT A FREE WIN — substepping TRADES friction for integration.**
Re-measured across the sweep at dt=4 ms:

| metric | ODE | sub=1 | sub=4 | sub=8 |
|---|---|---|---|---|
| T1 `bounce_height_rmse_rel` | 0.01788 | 0.03811 | **0.01033** | **0.00969** |
| T7 `angmom_drift_rel` | 1.935e-05 | 2.548e-05 | 1.610e-06 | **3.986e-07** |
| T7 `rot_ke_drift_rel` | **3.466e-08** | 7.954e-08 | 5.923e-08 | 6.250e-08 |
| T2 `slide_accel_rel_err` | **8.778e-05** | 0.03811 | **0.467** | — |

T1 flips to Newton (1.8×) and T7's angular momentum reaches 48× better than
ODE, but **T2's sliding accuracy degrades 12×**, and T7's rotational KE
plateaus around 6e-08 and never reaches ODE. So there is no single global
setting that wins: metrics limited by *integrator truncation* want substeps,
metrics limited by *contact/friction resolution* are hurt by them. The likely
mechanism is that MuJoCo's `solref`/`solimp` contact parameters are expressed
relative to the timestep, so shrinking the substep without rescaling them
changes the effective contact stiffness — untested, and the obvious next
experiment.

**Standing as measured: Newton wins 4 of 11 (T1 bounce, T3 roll, T4 energy,
T7 angular momentum), ties 2, loses 5** — against 1 win before this work. It
does NOT win overall, and no configuration found so far makes it do so.

⚠ **The default is NOT flipped.** Substepping costs step time (T4 measured 1.646
ms/step at substeps=4 against ~0.9 at substeps=1), and changing it alters the
physics of every existing Newton world, including the RL champions. It is
recorded here as a per-world lever with a measured effect, not as a new default.
The cost is far lower than it used to be — `899eb425` batches the solver
conversions so N substeps pay them once rather than N times — which is what
makes this trade worth having at all.

⚠ **T6's friction is not comparable between backends.** The scene declares
`coulombFriction [ 0.8 ]`, which is an ODE-path field Newton ignores; Newton
runs it at the `newtonGroundMu` default of 1.0. Forcing
`OMNISIM_NEWTON_GROUND_MU=0.8` changed the creep by not one bit, which suggests
that knob does not reach box-box contacts at all. The 109× creep gap is
therefore not yet a like-for-like comparison and should not be quoted as one.

### Lane-1 headlines

- **omnisim-ode is the best-scoring INTEGRATION in the suite** (not a solver result —
  see the second bullet, and [correctness-scope.md](correctness-scope.md)) — near-analytic T1/T2/T3,
  linear momentum exactly zero to double precision in T5, the cleanest T7
  conservation alongside PyBullet, best T4 at small dt, and a 10-box stack stable
  through dt=16 ms. Reproduced bit-identically on both machines.
  > ⚠ **HISTORICAL as of `bdc02139` — and this is the row people will misread.**
  > It was true on 2026-07-24 and it is unrepeatable now. The correct present-tense
  > sentence is: *"OmniSim's best-scoring lane-1 INTEGRATION was ODE, which has since
  > been deleted; the suite no longer has a second in-engine path to cross-check the
  > plumbing. Bare MuJoCo scored fine on the same scenes, and the four defects behind
  > Newton's deficit were in our integration layer and were fixed in `e7b9fb11`."*
  > It is **not** *"ODE is the fallback tier"* (there is no fallback), nor *"ODE was
  > a legacy tier"* (it outscored Newton), nor *"fidelity is fine now"*.
- **omnisim-newton, as integrated, has four real, reproducible defects**: the T2
  friction-cone offset (effective μ 0.41 vs 0.5), the T3 rolling-accel 47.63%
  deficit, the T5 momentum leak, and the T7 total spin loss. Framing per honesty
  rule 2: raw mujoco(-warp) scores fine on the same scenes with the same solver
  family, so these live in **our Newton integration layer** (contact / inertia /
  joint plumbing between the scene graph and the solver), not in the solver.
  They are fixable engine bugs that the suite now pins down with dt-independent,
  cross-machine-reproducible numbers.
- **Bonus engine bug (lane-1 ODE plumbing):** `OMNISIM_FORCE_ODE=1` alone does not
  stop the staticBase-Robot Newton registration; the Newton-backed ancestor then
  freezes supervisor pose readback of chain links (`OmSolid::postPhysicsStep`
  P3.10f). Workaround used for every ODE row: an `_odepin` world variant with
  `WorldInfo defaultPhysicsBackend "ode"`. Recorded in every ODE row's deviations.
  > ⚠ **This bullet is why lane 1 cannot simply be "re-pointed" at Newton.** The
  > `ode_pin` was not a convenience — the lane's reference arm was *generated* as a
  > pinned world variant, so the generator, the fixtures and the scoring all assume
  > a second backend. `OMNISIM_FORCE_ODE`, `OMNISIM_LEGACY` and
  > `defaultPhysicsBackend "ode"` all name nothing after `bdc02139`. Rebuilding a
  > correctness lane means choosing a *new* reference (analytic ground truth alone,
  > or the external MuJoCo/PyBullet arms), not editing a pin.

## Lane 2 — throughput (Go2, contacts ON, actions never idle, env-steps/s)

4090 columns here are the **pre-fix** pod `6fa66da0cde0`; the post-fix pod's
lane-2 numbers are in the cross-machine section at the end.

| tier | batch | 3060 raw mjwarp | 3060 omnisim-newton | 4090 raw mjwarp | 4090 omnisim-newton |
|---|---|---|---|---|---|
| A sim_only | 256 | 45,685 | 2,631 | 71,564 | 12,170 |
| A sim_only | 1024 | 120,577 | 10,117 | 246,437 | 47,259 |
| A sim_only | 4096 | 165,616 | 36,979 | 629,680 | 181,853 |
| A sim_only | 8192 | — | — | 839,542 | 349,364 |
| B sim_infer | 256 | 39,684 | | 40,223 | |
| B sim_infer | 1024 | 104,845 | | 132,755 | |
| B sim_infer | 4096 | 145,423 | | 292,617 | |
| B sim_infer | 8192 | — | | 341,269 | |
| C sim_train | 256 | | 10,228 | | not_run |
| C sim_train | 4096 | | — | | not_run |

- Tier A raw = the engine's own exported MjModel stepped by mujoco-warp with
  CUDA-graph capture (zero translation confound); tier A omnisim = the embedded
  deploy solver (Newton SolverMuJoCo) stepped in-process, **no CUDA graph**
  (matches the historical probe methodology; recorded as a conservative lower
  bound). Batch 8192 was run only on the 24 GB pod.
- **Methodology asymmetry, stated plainly:** the headline OmniSim/raw ratios —
  3060: 17.4× / 11.9× / 4.5× at 256/1024/4096; 4090: 5.9× / 5.2× / 3.5× / 2.4× —
  compare a *graphed* baseline to an *ungraphed* OmniSim probe. Dev shakeout rows
  on the same 3060 (`tests/benchmarks/omnibench/lane2/results/throughput.jsonl`)
  measured the raw baseline with the graph off: 2,043 vs OmniSim's 1,997 @256 and
  29,318 vs 28,972 @4096 — **~parity when both are ungraphed**. Most of the tabled
  gap is CUDA-graph capture, not per-step solver overhead; a graphed OmniSim
  variant is the v1 fix.
- Tier B is raw mjwarp + the Go2 champion ONNX in the loop, inference on CPU
  (onnxruntime had no CUDA EP in either environment; obs GPU→CPU + infer + action
  CPU→GPU all inside the timed loop, as deploy does it). That is why the 4090 gains
  little at small batch (40,223 vs the 3060's 39,684 @256) — CPU-bound.
- Tier C (in-engine PPO, 15 iterations, steady-state = median windowed
  env-steps/s excluding warmup, `QUAD_FAST_RESET=1`): **10,228 @256 on the 3060**
  (after one not_run first attempt with the same signature as the pod failures).
  On the **pre-fix** pod it is **not_run at both attempted batches** (256, 4096;
  cleared later on the post-fix pod — see the cross-machine section): the
  trainer produced no env-steps/s window samples, `started=False rc=0`; console
  logs preserved under `results_4090/_scratch/`. Recorded as an open gap, not
  diagnosed here.

## Lane 3 — novel axes (local numbers authoritative)

### 3a determinism — **bitwise on ODE and on Newton/XPBD, in the light-contact sphere-drop world, on three machines**

> ⚠️ **BOTH CONFIGURATIONS IN THIS HEADING ARE GONE** (ODE `bdc02139`, XPBD
> `94f04222`), so this row is historical in its entirety. The **live** determinism
> claim is the CPU `newtonSolver "mujoco"` result, and the current scope — with
> what is and is not verifiable — is [`determinism-scope.md`](determinism-scope.md),
> which is the source of truth for every external determinism statement. Note in
> particular that the "10 bitwise rows" figure quoted from this campaign is the two
> 5-sphere rows across three machines, and **one of those two rows was the ODE
> arm**.

> ⚠️ **Scope this row before quoting it.** The result below is genuine and
> reproduces. It is **not** a statement about OmniSim's GPU physics path, which a
> later adversarial re-test **refuted**. The per-configuration scope — what
> reproduces, what does not, and the mechanism — is
> [determinism-scope.md](determinism-scope.md), and that file is the source of
> truth for any external determinism claim.

**The world**, because the grade is only as broad as its scene:
[`lane3_determinism.wbt`](../../tests/benchmarks/omnibench/lane3/worlds/lane3_determinism.wbt)
— five `DEF`'d spheres dropped in a tight cluster onto a box pedestal
(`randomSeed 42`, `basicTimeStep 4`, no joints, no motors, no actuated robot;
only the `lane3_recorder` supervisor). The spheres collide with the pedestal and
with each other, so it does exercise multi-contact resolution — but it is a
**light-contact** scene, not fleet scale.

Identical world + seed, 400 compared steps: `cold_cold` and `cold_warm`
(worldReload in the same process) are **bitwise** (max abs dev 0, no divergence
step) for omnisim-ode and omnisim-newton, on the 3060 *and* the pre-fix 4090 pod
— the pod determinism results are valid and agree. Newton runs verified via the
backend sidecar; ODE runs verified by sidecar absence. The **third** machine (the
post-fix 4090 pod `65dd6587d5c9`) is in the addendum below; across all three
campaigns that is **10 bitwise rows out of 10** — which is the "10/10 rows"
figure other docs cite, and all ten are this one world on ODE or XPBD.

**Solver caveat — load-bearing, do not drop it:** this world's sidecar reports
`XPBD(iters=10)`, whereas every lane-1 world pins `newtonSolver "mujoco"` (and
lane-1 Newton runs the CPU `mj_step` path). So this row's "both backends" means
**ODE and Newton-under-XPBD**, and:

- it does **not** transfer to the solver lane 1 measures; and
- it does **not** cover the GPU `mujoco_warp` path. On that path an adversarial
  re-test on machine `9722d23d12a3` scored **0 bitwise out of 24 same-config
  cold pairs** across six scenes (80 → 336 concurrent contacts), deviating
  ~5e-5 m by 120 steps and **9.152 m by 1000 steps**. The same worlds graded
  **bitwise, 5 pairs of 5**, on the CPU paths (`newtonSolver "mujoco"` /
  ODE) — including a 336-contact, ten-robot scene with ten live controllers.
  Numbers, scenes and the confirmed `wp.atomic_add` mechanism:
  [determinism-scope.md](determinism-scope.md) §1–2.

XPBD also runs warp kernels, so treat the XPBD half of this row as **measured on
this light scene only** — it has not been re-measured at contact density, and
the GPU result above is a standing reason not to assume it generalises.

### 3b train==deploy structural parity (G1, `g1_golden_parity.py --structural`)

| config | real physics gaps | repr diffs | pass |
|---|---|---|---|
| deploy-default (legacy COM-at-link-origin) | 1 (`body_ipos`) | 3 | no |
| `OMNISIM_NEWTON_USE_LINK_COM=1` | **0** | 3 | **yes** |

Machine 9722d23d12a3, both rows `p2_trustworthy: true`. The one real gap in the
legacy default is the known COM-at-link-origin placement; the link-COM flag
closes it (the 3 remaining diffs are representational: body_mass/body_pos/nbody
bookkeeping, not physics). Pre-fix pod: both configs `ran=false` —
`ModuleNotFoundError: No module named 'warp'` in that pod's parity interpreter
(rc=1, recorded gap; local numbers are the result).

### 3c agent driveability — **10/10 on the live harness** (3060, omnisim-newton)

| probe | latency (ms) | probe | latency (ms) |
|---|---|---|---|
| load_valid_world | 8,795 | events_cursor_stream | 818 |
| hot_reload_edited_world | 3,987 | robot_joints_state | 968 |
| scene_tree_poses | 1,421 | scene_frame_verified | 3,853 |
| scene_tree_bounds | 2,495 | screenshot_png | 1,016 |
| sim_step_deterministic | 76,165 | broken_world_structured_diagnostic | 242,674 |

Score 10/10 = 1.0. Probe list: `tests/benchmarks/omnibench/lane3/DRIVEABILITY.md`.
Two measured findings recorded in the rows: at the time of the campaign `/sim/reset`
rewound time but did **not** restore node state, so state reset required a world
reload — **since fixed** (`25fbc755`): reset now rewinds the clock *and* restores the
engine's parse-time state on both backends, though the probe deliberately keeps the
reload path so the 10/10 score stays comparable across the fix boundary. And the sim
free-runs between harness RPCs, so the determinism probe compares rest states and
pays a large per-step RPC cost (hence the 76 s latency; strict trajectory
determinism is lane 3a's job). The broken-world probe passes — structured codes
`PARSE_ERROR`/`WORLD_PARSE_SYNTAX_ERROR`, never falsely healthy — but takes 243 s.
Pre-fix pod driveability: 2/10, rc=1 — that harness's `/world/load` returned 422
(`SIMULATOR_EXITED_NONZERO`) and every downstream probe failed; only the two
probes that don't need a loaded world passed. **The two scores are not the same
configuration**: the pod's rows are tagged `engine: omnisim-ode`, the local
rows `engine: omnisim-newton`, so 2/10 vs 10/10 is a backend difference as well
as a machine one. Recorded gap; the 3060 Newton run is the authoritative
driveability score.

## Metric caveats (v0)

- **T5 `angular_momentum_drift_rel` is noisy on ODE** and is the one lane-1 metric
  that disagrees across machines (0.059 vs 0.396 at dt=4; non-monotonic 0.06–8.0
  across the sweep) while ODE's linear momentum sits at 1e-14. The reference |L|
  after the actuation window is small, so the normalization amplifies noise —
  metric v0 issue, not an ODE physics finding.
- **T4 drift is normalized by peak KE, not |E(0)|** (E(0)=0 exactly for the
  horizontal release), recorded in every row. The dt=32 rows where drift ≈ 1e-8
  (newton 7.3e-09, mujoco 1.5e-08) are **artifacts, not conservation**: those runs
  blow up mid-window (recorded peak-KE normalizers 2.2e9 J and 5.9e8 J) and the
  enormous normalizer collapses the ratio.
- T7 pybullet conservation describes a near-steady spin, not a tumble (eruption
  outside the window) — recorded in its deviations.
- **Friction-cone config is not symmetric across engines**, so cone-sensitive
  cells are not strictly like-for-like. The elliptic-cone + impratio-10 override
  is applied to **mujoco on T2 and T3**, and (post-fix) to **omnisim-newton on
  T2 and T6** — not to mujoco T6, not to newton T3. Each override is recorded in
  its own row's deviations; quote T2/T3/T6 with that in mind.

## Open gaps

- Pre-fix pod lane-3 parity (`warp` missing in that pod's parity interpreter)
  and driveability (world load failed, 2/10, on omnisim-ode) both rc=1 — local
  results authoritative. The parity *execution* gap is closed on the post-fix
  pod, but its verdict is `p2_trustworthy=false`; driveability has never been
  rerun on a pod at all.
- Pre-fix pod lane-2 tier C `not_run` at both attempted batches (256, 4096); one
  local attempt failed identically before succeeding. Diagnostic consoles in
  `results_4090/_scratch/`. Cleared @4096 on the post-fix pod; @256 never rerun
  on a pod.
- B2, Isaac Lab, Genesis: not measured (not installed). Published-numbers
  comparisons are deliberately excluded from v0 so that every number in this
  report is same-harness, same-machine.

## Next steps

1. File the four Newton-integration bugs (T2 friction-cone offset, T3 rolling
   inertia, T5 momentum leak, T7 spin loss) plus the `OMNISIM_FORCE_ODE`
   staticBase-registration bug, each with its lane-1 repro command.
   > **Status:** the four Newton defects were fixed in `e7b9fb11` and
   > cross-machine validated (see the sections below). The `OMNISIM_FORCE_ODE`
   > bug is **moot** — the env var, and the backend it selected, are deleted
   > (`bdc02139`). ⚠ Note the repro commands in this report are **not runnable
   > as written**: every one that names an ODE arm or an `_odepin` world variant
   > targets a backend that is gone.
2. ~~A CUDA-graphed OmniSim lane-2 variant~~ — done, see the v1 addendum below.
3. ~~Metric v1: T5 normalization fix, T4 blow-up guard~~ — done (v1 addendum);
   the tier-C startup gate is partially addressed (xvfb wrap + console-tail
   capture in the `not_run` row); a live pod re-run is still needed to confirm.

## v1 addendum (2026-07-25, machine 9722d23d12a3)

### Lane 2 — the graphed-vs-ungraphed asymmetry is retired

`run_throughput.py` tier `Ag` now CUDA-graph-captures the omnisim-newton
embedded-solver path (one 8-substep period, `model.collide` +
`SolverMuJoCo.step` ×8, via warp `capture_begin`/`capture_end` — the raw
tier's machinery). Capture works with no structural blocker. Measured on the
3060 (rows appended to the local lane-2 jsonl, `cuda_graph: true`):

| batch | raw graphed | omnisim ungraphed (v0) | **omnisim graphed (v1)** | raw/omnisim, both graphed |
|---|---|---|---|---|
| 256 | 45,685 | 2,631 | **37,732** | **1.21×** (was 17.4×) |
| 4096 | 165,616 | 36,979 | **129,431** | **1.28×** (was 4.5×) |

As the v0 shakeout rows predicted, the headline gap was CUDA-graph capture,
not solver overhead: with both sides graphed the embedded deploy solver is
within ~21–28% of raw mujoco-warp. Physics sanity is unchanged (per-run
`base_z_mean` matches the ungraphed rows to 3 decimals; idle guard OK). The
4090 `Ag` rows await the next pod campaign — they landed on the post-fix
pod; see the cross-machine section.

### Lane 1 — metric v1 (T4/T5), local recordings re-scored

Re-scored the existing local T4/T5 recordings with the v1 scorer; 48 new
`suite: omnibench/v1` rows appended to the local lane-1 `results.jsonl`
(v0 rows untouched). **The pre-fix 4090's lane-1 T4/T5 rows remain
v0-metric** — that pod's recordings were not fetched, only its jsonl, so
there is no v1-metric pre-fix 4090 number. (The later post-fix pod's rows
*are* v1-scored despite their `v0` tag — see Machines.)

- **T5** `angular_momentum_drift_rel` now normalizes by the **peak |L| during
  the actuation window** (the injected momentum scale) instead of the small
  post-actuation residual |L|; the v0 value is kept per-row as
  `angular_momentum_drift_rel_v0`, and `angular_momentum_drift_abs`
  (kg·m²/s) is emitted alongside (absolute drift is also the metric's
  fallback when peak |L| < 1e-6). At dt=4: ODE 0.059→0.022, mujoco
  2.09→1.45, pybullet 2.20→1.38, newton 0.9999→0.125. Two honest caveats:
  the drive is a symmetric couple, so a *good* engine's peak |L| is itself a
  small residual and the ODE sweep stays non-monotonic (the absolute drift,
  1e-3–0.25 kg·m²/s, is the cleaner ODE story); and newton's smaller v1 ratio
  reflects its large in-window |L| excursion (peak 1.76 vs 0.22 retained) —
  the T5 momentum-leak defect stands (`linear_momentum_max` 18.7 unchanged).
- **T4** now emits `energy_blowup_max` = max|E(t)| / the *analytic* swing
  energy (22.07 J = mgΣxᵢ) and `energy_blew_up` (blowup > 10). The analytic
  normalizer is the point: an exploding run inflates its own recorded peak
  KE, so max|E|/peakKE is exactly 1.0 on the blow-ups (measured) and cannot
  discriminate. v1 flags **three** blow-ups — newton dt=32 (9.8e7), mujoco
  dt=32 (2.7e7), and **pybullet dt=32 (494.5)**, whose plausible-looking
  v0 drift of 0.999 had hidden a mid-window explosion. All healthy runs
  score ≤ 0.72.

### Lane 2 tier C on the pre-fix pod — diagnosed

Root cause of both `not_run` attempts: `run_quad_walk_rl.sh` →
`headless_runner.py` launches `omnisim-bin` **without** the `xvfb-run -a`
wrapper that `common/engine_launch.py` gives every lane-1/3a engine run —
on the display-less pod the engine aborted at Qt platform-plugin init
("could not connect to display", SIGABRT rc=-6) before the trainer started
(`started=False`), while xvfb-wrapped lane-1 runs on the same pod passed.
Not the STARTUP_S/STALL_S watchdog, not `OMNISIM_TORCH_SITE`, not missing
QUAD assets (the world was found and loading when Qt aborted). Fixes:
tier C now self-wraps in `xvfb-run -a` on display-less Linux and captures
the launcher-console tail into the `not_run` row; a directory `--out` no
longer crashes row emission (the pod lost a tier-C row to
`IsADirectoryError`); `launch_omnibench_pod.sh` now installs the runner-
python GPU wheels (warp-lang/mujoco-warp/newton pinned from the engine
interpreter's freeze, + torch if missing, + Pillow) with hard import
asserts — the 2026-07-24 runner python had no `warp`, which silently
skipped the lane-2 GPU tiers in an rc=0 run and killed lane-3 parity —
and preflights `xvfb-run`/`libxcb-cursor0`. **Confirmation needs a live
pod**; no pod was launched for this addendum — confirmed the next day on the
post-fix pod (`xvfb_wrapped: true`, tier C @4096 ran; cross-machine section).

## Post-fix rerun (2026-07-25, machine 9722d23d12a3 — RTX 3060 Laptop)

The v0 campaign's "four Newton-integration defects" **collapse to three root
causes**, now fixed in-engine (binary relinked 2026-07-25 09:37, incremental
rebuild of the edited TUs — build `b5c089d4`, binary sha `f95073f976323787`,
distinct from this machine's v0 binary `5087587d3e4b5940`; rows below appended
to the local lane-1 `results.jsonl` as `suite: "omnibench/v1-postfix"`, old
rows untouched):

1. **Gravity was never plumbed to Newton** (`WorldInfo.gravity` ignored; the
   builder always ran at -9.81, and gravity-0 scenes fell onto the implicit
   ground plane) — this one root cause explains BOTH the T5 "momentum leak"
   and the T7 "spin brake". Fixed: `OmNewtonBackend::setWorldGravity`
   plumbed from the WorldInfo flush.
2. **Husky-wheel inertia preset fallback** (a dynamic Solid with no explicit
   `inertiaMatrix` silently inherited `diag(0.0094m, 0.0167m, 0.0094m)`) —
   the T3 rolling defect. Fixed: the ODE-integrated geometry tensor is fed
   to Newton (`OMNISIM_NEWTON_LEGACY_INERTIA_PRESET=1` reverts; OmRobot
   wrapper bodies excluded, URDF links unaffected — they ship explicit
   tensors).
3. **MuJoCo-stock pyramidal friction cone + impratio 1** — the T2
   static-friction transition offset (inscribed-pyramid creep). Fixed via
   NEW per-world `WorldInfo.newtonCone` / `newtonImpratio` fields (T2/T6
   worlds pin `"elliptic"` + `10`); the global default deliberately stays
   MuJoCo stock pending champion re-verification.

Plus two non-scene defects: the **`OMNISIM_FORCE_ODE` bypass** (raw
`newtonBackend()` accessors constructed and drove Newton regardless; now
gated by `OmPhysicsBackendRegistry::odeForced()`) and the **t=0 supervisor
`setVelocity` drop** (pre-registration immediate message hit the dead ODE
body; now cached C++-side and queued runtime-side, drained after finalize).

| test (newton, dt=4) | v0 | post-fix | note |
|---|---|---|---|
| T2 stick violation (m) | 0.181 | **0.00044** | predicted 0.00044 |
| T2 transition err (deg) | 4.07 | **0.065** | = ODE's 0.065 (bracket resolution) |
| T2 slide accel rel err | 0.191 | **0.038** | predicted ~0.038 |
| T3 roll accel rel err | 0.476 | **0.00058** | predicted 0.00058; revert lever reproduces 0.4763 |
| T5 linear momentum max (kg·m/s) | 18.7 | **3.91** | see caveat below |
| T5 ang-mom drift rel (v1 / v0) | 0.125 / 0.9999 | 0.977 / 1.181 | **worse on both rulers** — normalizer artifact, see caveat |
| T6 stack survivors dt=8 | 1/10 | **10/10** | dt=4 creep 1.1e-3 → 5.7e-6 m/s (202×) |
| T6 max penetration dt=4 (m) | 0.000617 | 0.000902 | **regressed +46%** — the one metric that got worse; which fix caused it is not isolated |
| T7 ang-mom drift rel | 1.0 (total loss) | **2.5e-5** | ‖ω‖ = 5.000 through 10 s, z pinned at 2 |
| T1 bounce rmse (regression) | 0.03811169943784742 | 0.03811108985171102 | **not** unchanged: moved in the 8th s.f. (equal to 4 s.f.) |
| T4 energy drift (regression) | 0.497 | 0.381 | changed — explained: the capsule links had no explicit inertia, so v0 ran them on the wheel preset; fix 2 gives the true tensor |
| ode T1 / T4 (regression) | 0.017878 / 0.129986 | identical | ODE lane byte-identical |

**T5 honest caveat:** the prediction was |P| < 0.5 throughout. Measured:
the plateau IS < 0.5 (and |P|=0.23 at t=10), but a transient spike to 3.91
remains at t≈0.5 s when the freely-spinning chain whips through its folded
configuration at ~74 rad/s. It is dt-convergent (max 1.56 at dt=1) —
discretization of the joint constraint under extreme rates, not a steady
leak — and the links no longer collapse to the phantom floor.

**T5's angular-drift RATIO got worse, and that is a normalizer artifact —
state it, don't bury it.** v1 0.1245 → 0.9771 and v0 0.9999 → 1.1806, because
the recorded peak-|L| normalizer collapsed from **1.757 to 0.3112 kg·m²/s** —
the same drift divided by a 5.6× smaller reference. The rows record the
normalizer, not the mechanism; the plausible reading is that the fixes removed
the phantom-floor interaction that used to pump a large in-window |L|
excursion, but that is inference. The **absolute** drift is the honest ruler
here and it
rose only 0.219 → **0.304 kg·m²/s**, still well under MuJoCo's 0.765 and
PyBullet's 0.797 on the same scene. Cite `linear_momentum_max` (18.7 → 3.91)
and `angular_momentum_drift_abs`; do not cite the T5 ratio in either
direction as a physics result. Follow-up: none planned; the scene is an
intentionally violent stress test.

**Unexplained, flagged not-measured: the ODE wall-clock collapse.** ODE lane-1
metrics are byte-identical pre/post, but ODE wall ms/step fell 13–22× on this
machine across the fix (T1 2.854 → 0.1306 = 22×, T4 2.227 → 0.1716 = 13×; only
those two ODE tests were rerun) and the post-fix pod's ODE times are 8–349×
below the pre-fix pod's (T3 4.557 → 0.01307 = 349×, T1 2.032 → 0.01350 = 150×,
T6 only 7.9×) — with T7 the lone exception, essentially unchanged
(0.01073 → 0.01261, i.e. slightly slower). The FORCE_ODE-bypass fix no longer
constructing and stepping Newton alongside ODE is the obvious hypothesis, but
**nothing in the rows establishes it** and the two pods differ in kernel,
driver and concurrency. Treat lane-1 wall as machine-and-campaign-scoped, and
this speedup as an open observation, not a claimed result of the fix.

T7's meta additionally flipped from `method: "torque_impulse"` to
`method: "setVelocity"`, `spinup_steps: 1` — the t=0 setVelocity fix
verified by the recorder's own probe. `OMNISIM_FORCE_ODE=1` on a non-pinned
Newton world now produces no `.newton.json` sidecar with a fully-moving
chain (LINK3 z-range 1.29 m; pre-fix the links froze), and the same world
without the env var still writes a non-degraded Newton sidecar.

Suite gates after the rebuild: `test_g1_deploy_runtime_sync` +
`test_g1_physics_spec_conformance` tier 1 — 19 passed (runtime mirror
regenerated in the same change). Also fixed while here: the mirror
generator `_gen_deploy_runtime.py` still wrote to the pre-rename
`projects/rl/backends/` path.

## Cross-machine validation of the Newton fixes (2026-07-25, RTX 4090)

Same-day cross-machine check of the post-fix rerun above, on machine
**65dd6587d5c9** — a *different* 4090 from the pre-fix pod `6fa66da0cde0`:
RunPod, pod `mpg4buqmwt9asg`, driver 580.126.20, Linux 6.8.0-107 / glibc 2.35,
python 3.11.10 / numpy 2.4.6, engine **rebuilt on the pod 2026-07-25 20:55 UTC
from commit `e7b9fb11`'s tree** (coherent HEAD archive, top-level rebuild;
libController `94875d0ecb7f659e`, ipc-nonce gate OK). One ~25-minute session
(21:04–21:29 UTC). Raw results:
`tests/benchmarks/omnibench/results_4090_validate/results_validate/`.
Different OS, compiler, CPU, GPU, and driver from the 3060 rerun.

Three provenance warnings for anyone citing these rows:

- **Binary sha256 `36732b708231390e` is the only unambiguous build
  discriminator.** The rows disagree with themselves and with this report:
  10 of 14 lane-1 rows (and all lane-2/3 rows) record `build: "95f5a2c"`,
  4 record `null`, and this section attributes the tree to `e7b9fb11`.
  `95f5a2c` is also the **pre-fix** pod's build string, against a different
  binary (`47c57f8bcec1c9ec`) — so the string is stale and cannot separate
  pre- from post-fix. The sha256 can.
- **The T7 row is stamped a different machine id** (`6849f1be4974`, see
  Machines) — same host, same binary, degraded fingerprint. It is the
  headline T7 cell, so any "all these numbers are 65dd6587d5c9" claim is
  wrong for exactly that row.
- **The campaign MANIFEST certifies only part of the campaign.** It covers a
  final ODE-only invocation (`21:15:15 → 21:19:57`, `lanes=1 engines=omnisim
  backends=ode dt_ms=4`, 7 runs, `parallel: 6`); the newton lane-1 rows
  (21:04–21:14), lane 2 (21:21–21:25) and lane 3 (21:27–21:28) came from
  earlier invocations whose manifests were overwritten — its `args` even list
  `lane2_batches: [256, 1024, 4096]` while lane-2 rows exist at 8192. So its
  `gaps: []` does **not** certify the newton, lane-2 or lane-3 rows.

### Lane 1 — the fixes reproduce (newton, dt=4)

Pre-fix column = pod `6fa66da0cde0`; post-fix 4090 = pod `65dd6587d5c9`
(T7 rows: `6849f1be4974`); post-fix 3060 = machine `9722d23d12a3`,
`suite: omnibench/v1-postfix`.

| test | pre-fix 4090 | post-fix 4090 | post-fix 3060 |
|---|---|---|---|
| T2 stick violation (m) | 0.1806 | **0.00068** | 0.00044 (differs) |
| T2 transition err (deg) | 4.065 | **0.06505117707799002** | identical |
| T2 slide accel rel err | 0.1908 | **0.03811175830671643** | identical |
| T3 roll accel rel err | 0.4763 | **0.0005821351711044256** | identical |
| T3 slip ratio | 0.00174 | **0.00036208197940195433** | identical |
| T5 linear momentum max (kg·m/s) | 18.69 | **3.910038157066208** | identical |
| T5 ang-mom drift rel | 0.9998911228451609 (v0 ruler only — this pod was never v1-rescored) | 0.9771421122170494 | 0.9771431331900141 — *ratio worse, normalizer artifact* |
| T6 settle creep dt=4 (m/s) | 1.15e-3 | **3.94e-06** | 5.69e-06 (differs) |
| T6 max penetration (m) | 0.000617 | 0.0009017646312713679 | identical to 4090 — but **up 46% vs pre-fix** |
| T7 ang-mom drift rel | 1.0 (total loss) | **2.54772084353779e-05** | identical |
| T7 rot-KE drift rel | 1.0 | **7.953878228986613e-08** | identical |
| T1 bounce rmse (regression) | 0.03811169943784742 | 0.03811108985171102 | identical to 4090; **moved in the 8th s.f.** vs pre-fix |
| T4 energy drift (regression) | 0.4974 | 0.38096878866089273 | 0.38096889349873825 (6 s.f.) |

"identical" = **equal to every recorded digit** (15–16 significant figures)
across Windows/MinGW/RTX 3060 and Linux/gcc/RTX 4090 — including T3's headline
0.000582, T5's transient peak 3.910038157066208, and T7's 2.5e-05.
Digit-identity across different compilers, OSes, CPUs, and GPU generations says
that **on the path lane 1 pins — `newtonSolver "mujoco"`, i.e. CPU `mj_step` —**
the post-fix Newton step has no machine-dependent term in the integration path
for these scenes. Read it no wider than that: it is a statement about the CPU
MuJoCo solver on seven analytic scenes, **not** about `mujoco_warp`, whose
run-to-run reproducibility is refuted on this machine
([determinism-scope.md](determinism-scope.md)), and not a claim of bitwise
trajectory identity across machines — which remains untested and which the two
exceptions below already qualify. Cross-machine, the metrics that
differ are exactly the contact-settle-sensitive scalars — T2 stick displacement
(0.44 vs 0.68 mm) and T6 creep (5.69e-06 vs 3.94e-06 m/s), both still 2–3
orders under pre-fix — plus T4's last digits (agrees to 6 s.f.).

Two things the fixes did **not** improve, both reproduced on the pod and both
already flagged above: T6 `max_penetration_m` **rose** 0.000617 → 0.000902 m
(+46%) while creep fell ~200–290×, and the T5 angular-drift **ratio** is worse
on both rulers (v0 0.9999 → 1.181, v1 0.125 → 0.977) purely because its
peak-|L| normalizer collapsed 1.757 → 0.3112 — the absolute drift moved only
0.219 → 0.304 kg·m²/s. Neither is a cross-machine discrepancy; both are
pre-vs-post facts that the 4090 confirms.

ODE regression lane on the pod: T1 0.017877779728827616 and T4
0.12998577671535375, both digit-identical to the 3060 and to both pre-fix
campaigns — the ODE lane's *metrics* are untouched by the fixes, as required.
Its **wall-clock** is a different story and an open one (see the post-fix
addendum's not-measured note: this pod's ODE steps 8–349× faster than the
pre-fix pod's, cause unestablished).

### Lane 3 on the new binary

- **Determinism: bitwise** in the light-contact sphere-drop world
  (`lane3_determinism.wbt`), `cold_cold` *and* `cold_warm` (worldReload in the
  same process), 400 compared steps, max abs dev 0, on the freshly built
  binary (`36732b708231390e`); Newton verified via the backend sidecar, which
  reports **`XPBD(iters=10)`** for this world — so, as in v0, the grade covers
  the **XPBD** path on that one scene, not the MuJoCo solver lane 1 pins, and
  not the GPU `mujoco_warp` path (refuted — §3a and
  [determinism-scope.md](determinism-scope.md)). ODE determinism was not
  rerun this campaign (code path untouched; the 2026-07-24 bitwise result
  stands).
- **Parity now *runs* on a pod, but its verdict is not usable** — the
  2026-07-24 `warp`-missing gap is closed (the wheel-install fix from the v1
  addendum, confirmed live), and that is the whole result. Both configs report
  `pass=true` with 0 real physics gaps, but both also record
  `p2_trustworthy=false` ("P2 deploy driver INCONCLUSIVE — gaps count
  untrustworthy") where the 3060's rows are `p2_trustworthy=true`. The pod
  additionally reports `representational_diffs=0` where the 3060 reports 3
  (`body_mass`/`body_pos`/`nbody`) — a second symptom of the inconclusive P2
  driver, not a discrepancy that got fixed. **The pod's zeros neither confirm
  nor contradict the 3060 result**; the 3060 parity table above stands as the
  authoritative one.

### Lane 2 — first successful pod tier C, and the 4090 graphed rows land

**Tier C (in-engine PPO through `omnisim-bin`) ran on a pod for the first
time: 333,036 env-steps/s @4096** (steady-state median of windowed samples,
first window excluded; 30 iterations, legacy-trot, `QUAD_FAST_RESET=1`,
`trainer_done=true`, rc=0, and `xvfb_wrapped: true` in the row — the
v1-addendum xvfb diagnosis confirmed by a live pod). With full PPO rollout +
update in the loop that is **0.67× the graphed *OmniSim* tier-A rate** at the
same batch (500,105) and **0.51× the graphed *raw* rate** (654,034) — quote
which baseline you mean, the two differ by a third.

The 4090 `Ag` rows the v1 addendum promised (both sides CUDA-graphed):

| batch | raw graphed | omnisim ungraphed | **omnisim graphed** | ratio, both graphed | (was, v0 ungraphed) |
|---|---|---|---|---|---|
| 256 | 73,609 | 11,840 | **60,619** | **1.21×** | 5.9× |
| 1024 | 254,945 | 45,575 | **210,852** | **1.21×** | 5.2× |
| 4096 | 654,034 | 177,753 | **500,105** | **1.31×** | 3.5× |
| 8192 | 870,459 | 343,075 | **650,487** | **1.34×** | 2.4× |

Same conclusion as the 3060: the v0 headline gap was CUDA-graph capture, not
solver overhead. Ungraphed/raw rates match the 2026-07-24 pod campaign within
~±4% (raw @8192: 870,459 vs 839,542) — same GPU model, different pod and
driver. What ran, honestly: tier A (raw graphed; omnisim graphed + ungraphed)
at all four batches, tier B raw-only at all four batches (tier B has no
omnisim variant in v0/v1), tier C @4096 only (no @256 pod rerun); lane 2 has
no ODE variant.

### Caveats (this campaign)

- **Driveability intentionally not rerun** — the pod was terminated early by
  operator choice; the 3060's 10/10 (on omnisim-**newton**) remains the only
  authoritative score. The pre-fix pod's 2/10 was omnisim-**ode** and is not a
  like-for-like comparison in either direction.
- **T5's fold-whip transient reproduces to the digit** (3.910038157066208):
  deterministic discretization, not noise — the metric caveat in the post-fix
  rerun stands unchanged, including its ratio-vs-absolute warning.
- Validation scope is dt=4 only — no dt sweep, no mujoco/pybullet reruns (those
  engines are unaffected by the fixes), no ODE determinism rerun, no tier-C
  @256, no driveability.
- The cone/solver asymmetries in Metric caveats apply to these cells too: the
  elliptic-cone override is on newton T2 and T6 but not T3, and the lane-3a
  determinism world runs XPBD while lane 1 pins the MuJoCo solver.
- **Lane 3a never tested the GPU solver, and the GPU solver does not pass.**
  Every determinism row in this campaign is ODE or Newton/XPBD on one
  light-contact scene. A later re-test of `newtonSolver "mujoco_warp"` scored
  0 bitwise of 24 same-config cold pairs (up to 9.152 m deviation at 1000
  steps), while the CPU `mj_step` path graded bitwise 5/5 on the same worlds.
  Adding a GPU-path row and a contact-dense XPBD row to lane 3a is the open v1
  item; scope and evidence: [determinism-scope.md](determinism-scope.md).
- The validation consumed five failed pod rounds before this one succeeded;
  the lessons are codified in the ops runbook (private tree — not part of the
  public snapshot), not retold here.
