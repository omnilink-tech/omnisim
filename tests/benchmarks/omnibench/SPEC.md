# OmniBench — cross-simulator physics benchmark suite

**Status:** v0 contract. Every lane implements against THIS file; do not invent
divergent scene parameters. If a parameter here is physically impossible in one
engine, implement the closest honest mapping and record the deviation in the
result JSON (`deviations` field) — the deviation IS a finding, not a bug to hide.

Design lineage: SimBenchmark (ETH RSL) + Erez/Tassa/Todorov ICRA'15 timestep-sweep
methodology + the post-Genesis credibility checklist (contacts ON, realistic
models, policy-driven actions, stated GPU, open runnable scripts, machine
fingerprint on every number).

## Layout

```
tests/benchmarks/omnibench/
  SPEC.md               <- this file (the contract)
  common/               <- shared: scene constants, results schema, scoring, fingerprint
  lane1/                <- physics correctness (analytic ground truth)
    run_mujoco.py         raw MuJoCo implementation of the scenes
    run_pybullet.py       raw PyBullet implementation of the scenes
    run_omnisim.py        OmniSim runner (backend = newton; `ode` refused), uses worlds/ + controllers/
    worlds/  controllers/
    score.py              error metrics vs analytic reference, Pareto data
    translation_audit.py  the .wbt -> mjModel contract audit (see below)
  lane2/                <- throughput per the credibility checklist (3-tier)
  lane3/                <- determinism grading, train==deploy parity, agent-driveability
  lane4/                <- capability coverage, resource envelope, CPU-only
    capabilities.py       the probe registry (claims + worlds + assertions)
    gen_worlds.py         registry -> committed worlds/ (--check for drift)
    run_coverage.py       4a; run_envelope.py 4b; cpu_only.py 4c
    report.py             rows -> the MEASURED capability matrix
  run_all.py            <- one entry point; writes results/<machine_id>/<utc-date>/
  results/              <- gitignored except committed campaign summaries
```

## Global conventions

- Gravity **9.81 m/s²** (−z). SI units everywhere. Seed **42** where applicable.
- Timestep sweep (lane 1): **dt ∈ {1, 2, 4, 8, 16, 32} ms**. Analytic reference
  where closed form exists; otherwise self-convergence vs the engine's own
  finest-dt run (Erez methodology). Reference engines may additionally run
  dt=0.25 ms as a numeric ground-truth check of the analytic formulas.
- Every result row records wall-clock **ms/step** alongside the error metric →
  speed-vs-accuracy Pareto, never accuracy alone or speed alone.
- Machine attribution is mandatory: every results file embeds the output of
  `projects/policies/common/env_fingerprint.py` (machine id, GPU, CPU, build sha,
  binary sha, stack versions).

## Results schema (JSON Lines, one row per (test, engine, dt) run)

```json
{"suite": "omnibench/v0", "test": "bounce", "engine": "omnisim-newton",
 "dt_ms": 4, "metrics": {"...": 0.0}, "wall_ms_per_step": 0.0,
 "steps": 0, "sim_seconds": 0.0, "deviations": ["..."],
 "machine": {"...": "..."}, "utc": "2026-07-24T00:00:00Z"}
```

`engine` ∈ `mujoco`, `pybullet`, `omnisim-newton` (lane 2/3 may add
`mujoco-warp-raw`). Writer helper lives in `common/results.py` — use it.
`omnisim-ode` remains a **readable** value because recorded `results*/` rows
carry it, but it is no longer **writable**: src/ode was DELETED (commit
bdc02139).

`omnisim-unverified` is the label for a row produced through `omnisim-bin` whose
**backend could not be proven** from the `.newton.json` verdict sidecar (the run
never reached world-finalize, the log path did not match, or Newton came up
degraded). It replaced `omnisim-ode` as lane 3c's fallback on 2026-08-08: with
ODE deleted, "no sidecar" can no longer mean "ODE drove it", so the old default
stamped every sidecar-less row with the name of an engine that does not exist and
a consumer could not tell those rows from real ODE measurements. A row carrying
this label MUST also carry its reason in `deviations` (prefix
`engine=omnisim-unverified:`), and `run_all.py` raises it as an `[attribution]`
finding. It is honest, not publishable: re-run before quoting it.

## Lane 1 scenes (fixed parameters)

### T1 `bounce` — restitution fidelity
Sphere r=0.1 m, m=1 kg, dropped from h₀=1.0 m (center start z = 1.1), e=0.8,
friction 0. Analytic: peak heights hₙ = h₀·e²ⁿ, n=1..5. Metric
`bounce_height_rmse_rel`: RMS of (measured−analytic)/analytic over the first 5
peaks. Engines without a direct restitution coefficient (MuJoCo/Newton solref
mapping) tune to e=0.8 on a calibration drop at dt=1 ms, record the mapping in
`deviations`, then run the sweep with that fixed mapping.

### T2 `incline` — friction-cone stick/slip vs analytic
Box 0.2³ m, m=1 kg, μ=0.5 (tan⁻¹0.5 ≈ 26.57°), on inclines
θ ∈ {15°, 20°, 25°, 26°, 27°, 30°, 35°}. Below critical angle: must stick
(|displacement| < 1 mm over 3 s). Above: analytic slide a = g(sinθ − μcosθ).
Metrics: `stick_violation_max_m` (θ < θc), `slide_accel_rel_err` (θ > θc, fit
over 1 s after release), `transition_angle_err_deg` (bisection of observed
stick→slip vs 26.57°).

### T3 `roll` — rolling without slip
Solid sphere r=0.1 m, m=1 kg, μ=0.8, released on the 20° incline from T2.
Analytic (rolling, solid sphere): a = (5/7)·g·sinθ. Metric
`roll_accel_rel_err`, plus `slip_ratio` (|v − ωr|/v at t=1 s; should be ≈0).

### T4 `pendulum_energy` — articulated energy conservation
3-link chain: capsule links l=0.5 m, r=0.02 m, m=1 kg each, hinge joints, zero
damping/friction, no contacts. Released from horizontal, simulated 10 s.
Metric `energy_drift_rel` = |E(10 s) − E(0)|/|E(0)| and
`energy_drift_slope` (per-second linear fit) — secular drift is the failure,
bounded oscillation is fine.

### T5 `momentum` — floating-base momentum conservation
Same 3-link chain, free-floating, **gravity off**, no contacts. Middle joint
driven by a 2 s sinusoidal torque (τ = 5·sin(2πt) N·m), then free for 8 s.
Linear momentum must stay 0; angular momentum constant after the actuation
window. Metrics: `linear_momentum_max` (kg·m/s), `angular_momentum_drift_rel`
over the free window.

### T6 `stack` — multi-contact stability
Tower of N=10 boxes 0.1³ m, m=0.5 kg, μ=0.8, e=0, stacked with 1 mm gaps,
simulated 10 s. Metrics: `stack_survivors` (boxes within 2 cm of start XY and
still ordered in z), `settle_creep_m_s` (top-box drift velocity over the last
5 s), `max_penetration_m` at steady state.

### T7 `spin` — single-body angular momentum / gyroscopic integration
Box 0.4×0.2×0.1 m, m=1 kg, gravity off, no contacts, initial spin about the
unstable intermediate axis ω=(0.01, 5.0, 0.01) rad/s. 10 s. The Dzhanibekov
tumble has no simple closed form, but |L| and rotational KE are conserved.
Metrics: `angmom_drift_rel`, `rot_ke_drift_rel`.

Notes for implementers:
- Record trajectories at every physics step (positions + velocities of the
  bodies of interest); compute metrics offline in `score.py` from the recorded
  arrays, identically for all engines. Recording format: one .npz or .csv per
  run under a temp/results dir — scorer takes the file path.
- PyBullet: `setPhysicsEngineParameter(fixedTimeStep=dt, numSolverIterations=…)`
  — use engine defaults unless a scene requires otherwise; record any override.
- OmniSim: worlds under `lane1/worlds/` are headless-only test worlds (exempt
  from the canonical-lighting rule since they live under tests/). Backend:
  Newton only, asserted with `OMNISIM_REQUIRE_NEWTON=1` and verified via the
  `.newton.json` sidecar next to `OMNISIM_LOG_PATH`. Duration budget ≥15 s so
  finalize is reached and the sidecar exists.

> ⚠️ **LANE 1 HAS NO SECOND IN-ENGINE ARM ANY MORE (2026-08-08) — it still has its
> ORACLE.** The oracle is the **analytic ground truth** each scene encodes, and that
> is unaffected; bare MuJoCo and PyBullet also still run. `ODE =
> OMNISIM_FORCE_ODE=1` used to be the second *integration* here, and it was best or
> tied-best on 6 of 7 scenes at dt=4 — ⚠️ an integration result, not a solver one:
> bare mujoco scored fine on the same scenes and the four defects were in our own
> plumbing, since fixed in `e7b9fb11`
> ([correctness-scope.md](../../../docs/benchmarks/correctness-scope.md)).
> **src/ode was
> DELETED (commit bdc02139)**, so the arm is gone and `--backend ode` is
> refused rather than silently served by Newton. Two consequences to state
> plainly rather than paper over: (1) what lane 1 still measures is Newton
> against the **analytic ground truth** each scene encodes, which is the
> harder and more useful half, but it can no longer answer "is Newton as good
> as ODE here"; (2) the retired env knob cannot be repurposed into a second arm,
> and both of its failure modes are silent. The current engine **ignores**
> `OMNISIM_FORCE_ODE` (verified 2026-08-08: the run comes up on Newton and writes
> a normal `.newton.json` sidecar), so an "ode" arm would be a second **Newton**
> arm and the lane would report agreement it never measured; an earlier build the
> same day still honoured it, and then the scene stayed **frozen at its authored
> pose for the whole run** (measured on `tests/physics/worlds/contact_points.omniworld`).
> Either way the row is a measurement of something other than what its
> `omnisim-ode` label claims.
>
> **What lane 1's reference actually is:** the **per-scene analytic ground
> truth** encoded in [`common/scenes.py`](common/scenes.py) and applied by
> [`lane1/score.py`](lane1/score.py) — the scorer reads **no engine's numbers**
> as a reference. `mujoco` and `pybullet` remain as **external cross-checks**
> (independent implementations that corroborate a scene, not the truth for it).
>
> **There is no frozen file of lane-1 ODE values, and one used to be cited
> here.** `tests/goldens/ode_oracle_goldens.json` is a *different* body of
> evidence: 29 measurements across **8 DEVICE-parity families** (`raycast`,
> `native_inertia`, `weld`, `touch_force`, `receiver_occlusion`,
> `lightsensor_occlusion`, `radar_occlusion`, `kinematic_native`) — **zero
> T1–T7 scenes** — and its `weld` and `kinematic_native` families carry an
> `oracle_status_note` saying ODE was never a numerical oracle there either.
> The ODE arm's own **105 lane-1 rows** survive only as history under
> `results*/`; they are **permanently unrepeatable**, and lane 1 has lost its
> only in-engine corroborating implementation. Do not read that as "lane 1 is
> unaffected".

### The translation audit (`lane1/translation_audit.py`)

What the deleted arm was actually load-bearing for was **attribution**: running
one `.wbt` through two integrations to tell "the solver got this wrong" from "we
handed the solver the wrong model". Analytic ground truth cannot separate those,
and neither can `mujoco`/`pybullet` — they validate the SOLVER and do not read
`.wbt`. That is the layer with this project's entire track record of real bugs
(gravity never plumbed; `coordinateSystem` never reaching the solver, which
zeroed gravity in 210 NUE worlds; a husky-wheel inertia preset on every
`Solid`).

`translation_audit.py` replaces it by comparing the authored `.wbt` contract
against the **exact mjModel the solver stepped** (`OMNISIM_NEWTON_DUMP_MJMODEL`,
written at finalize): gravity magnitude *and* up-axis, timestep vs substeps,
per-geom friction and `condim`, body masses, collidability, and every
declared-but-unread field. It refuses to audit a model it cannot attribute to a
verified Newton run, and exits non-zero on any ERROR so it can gate a build.

**Binding rules for this tool:**

1. **A check that cannot be evaluated is a FINDING, never silence.** It shipped
   with the opposite behaviour — the dump's numpy reprs broke the number parse,
   the gravity check skipped itself, and the report read green.
2. **Never quote a green audit without the negative arms.** `--self-test` runs
   live probes for each coordinate system and a non-Earth gravity, then requires
   deliberately wrong models to raise ERROR. A green that cannot go red is not
   evidence.
3. **The audit measures the world AS AUTHORED** — it strips the
   `OMNISIM_NEWTON_*` scene knobs from the child environment. A lane that
   compensates through env vars (as `run_omnisim.py` legitimately does) will
   still show the world's own declaration failing to reach the model. That is
   the intended reading: it is a statement about the FILE's reproducibility, not
   about the lane's numbers.
4. It reads the model at finalize, so it is **blind to runtime drift** — notably
   the measured, unfixed defect where a supervisor-deleted node is never removed
   from the MuJoCo model.

First corpus run (2026-08-09) returned 10 ERROR / 35 WARN over 13 lane-1 scenes,
all one defect: the worlds declared `ContactProperties.coulombFriction` (an
ODE-path field Newton does not read) while the model ran the default μ=1.0. Now
0/0 after `gen_worlds.py` was changed to declare `newtonGroundMu` /
`newtonContactKd`; metrics verified bit-identical.
See [../../../docs/benchmarks/lane1-postdeletion-2026-08-09.md](../../../docs/benchmarks/lane1-postdeletion-2026-08-09.md).

**It runs as a campaign stage.** `run_all.py` invokes it after the lane-1 matrix
and raises each contract violation as a `[translation]` finding, so a campaign
cannot publish lane-1 numbers without also saying whether the worlds behind them
describe themselves. `--skip-translation-audit` opts out and is worth naming in
any report that uses it.

**Corpus scale (`--sweep DIR`, static, no engine).** The systemic defect is
statically decidable, so the whole tree costs milliseconds. Two rules:

5. **`results/` is evidence, not corpus.** Worlds under a results directory,
   worktree copies, and harness scratch files (`.omnisim_*`, `.harness_*`,
   `.capture_*`) are excluded by default. They record what an agent produced on
   a date; counting them inflates the defect number and "fixing" one falsifies
   the record. `--include-artifacts` opts in, for inspection only.
6. **`--fix` is a self-description change, never a retune.** It declares
   `newtonGroundMu = the world's own coulombFriction`. Because env > field >
   default, it is numerically inert for any launcher that already exports the
   variable, and moves only the previously-broken bare load. It refuses three
   cases on purpose — μ=0 (the unset sentinel makes it inexpressible),
   per-material frictions (one global value cannot represent them), and worlds
   already declaring the field — because a migration that silently mis-states
   what it achieved is worse than one that stops.

## Lane 2 — throughput (three-tier, credibility checklist)

Model: Go2 quadruped (contacts + self-collision ON, production solver settings,
the same MjModel the engine itself exports — zero translation confound).
Tiers, each reported separately, never summed or conflated:
  A `sim_only`   — batched physics stepping, random-policy actions (never idle)
  B `sim_infer`  — A + ONNX champion policy inference in the loop
  C `sim_train`  — short in-engine PPO run (existing recipe), steady-state env-steps/s
Batch sizes {256, 1024, 4096} (drop what OOMs on 6 GB — record the drop).
Baselines on the same machine: raw `mujoco_warp` stepping the exported MjModel
(same batch sizes) → OmniSim overhead = OmniSim / raw ratio, the headline.
Reuse, don't reinvent: `tests/benchmarks/newton_scaling_bench.py`,
`optim_bench.py multi-instance`, `projects/policies/research/training/probe_newton_batch_throughput.py`.

## Lane 3 — novel axes

3a `determinism` — grade per backend: run the identical world+seed twice from
cold, compare full trajectories → `bitwise` | `tolerance(<1e-9)` | `divergent`;
plus cold-vs-warm-load parity. Reuse the oracle_dumper recorder pattern.
3b `parity` — train==deploy: score `g1_golden_parity.py --structural` field
diff into {real_physics_gaps: N} and assert 0; record as a benchmark row so
competitors' (structural) trainer≠deployer gap has a number to sit next to.
3c `driveability` — machine-scored probe of the agent-facing surface: N
capability probes against a live harness (load world, scene tree, structured
diagnostics on a broken world, hot reload, event stream, camera framing with
numeric verification). Score = probes passed / N, plus latency per probe.
Document the probe list so other sims can be scored by hand honestly.

## Lane 4 — capability coverage and resource envelope

The prior question the other three lanes are conditional on: **what can this
simulator simulate, and how much of it, on this machine?** Lanes 1/1R ask
whether the physics is right, lane 2 how fast, lane 3 whether it is
deterministic and driveable — none of them answers "can it simulate the thing
I have at all", which until this lane existed was answered only in prose.

4a `coverage` — a registry of executed capability probes
([`lane4/capabilities.py`](lane4/capabilities.py)), one committed `.wbt` each,
each judged by an assertion in physical units whose docstring is published as
the claim under test. Verdicts:

| verdict | meaning |
|---|---|
| `works` | present, and the measurement lands where physics says it must |
| `degraded` | present and doing something, missing the physical target by a stated amount |
| `broken` | present in the schema — world loads, device accepted — and measurably does nothing |
| `absent` | not in the schema; established by the engine **refusing** the declaration, never by "we did not try" |
| `inconclusive` | **not a capability verdict** — the probe's own instrument failed. Excluded from every score. |

`broken` vs `absent` is the distinction the lane exists for, and neither a
static nor a dynamic test settles it alone (a static test sees `BallJoint`'s
motor accepted and calls it present; a dynamic test sees the joint never move
and calls it broken). Both run. **A capability that parses and then does
nothing is what a load-only smoke run reports as PASS.**

Headline is `works / (works + degraded + broken)` — of the capabilities that
EXIST, how many work. `absent` is deliberately excluded: no cloth solver is a
scope statement, not a defect, and folding it into a percentage would make the
engine look broken for something it never claimed.

4b `envelope` — per-machine scaling. The realtime ceiling is reported as a
**bracket between two measured sizes**, never an interpolation. Plus the
silent-cliff detector: on `mujoco_warp` the fixed `njmax` constraint buffer
truncates in silence (its only diagnostic is a `wp.printf` inside a warp
kernel, discarded by a GUI-subsystem binary), so a scene degrades with a clean
log and exit 0. The cap does not exist on CPU `mj_step`, whose `efc` arrays
grow dynamically — the summary row states that rather than letting "no
overflow" read as "tested and cleared". Timing reuses `step_cost`'s
two-nested-window differencing.

4c `cpu_only` — the engine re-run with every CUDA device hidden
(`CUDA_VISIBLE_DEVICES=-1`). It is **not** a GPU-less-hardware result and the
row says so; the honest external phrasing is "runs with no CUDA device visible
to the process".

Scope: rigid-body simulation. Rendering quality is out of scope by design —
the Camera and Lidar probes assert only that an image exists and is
non-degenerate, and record dropping `--no-rendering` as a deviation.

Reference: [`lane4/README.md`](lane4/README.md), including the rules for
adding a probe and the seven calibration errors this lane made about itself
before it produced a correct finding about the engine.

## Honesty rules (non-negotiable)

1. Never compare a batched-GPU number to a single-env CPU number.
2. OmniSim's Newton backend embeds mujoco-warp — accuracy deltas vs MuJoCo are
   solver-family-internal; frame them as integration fidelity, not "beating MuJoCo".
3. Report losses as prominently as wins. `deviations` and dropped configs are
   part of the result.
4. Any number quoted outside `results/` must carry its machine id.
5. Wall-clock comparisons must state what each row includes: OmniSim rows step
   a whole engine process (IPC included); bare-library rows (mujoco/pybullet) do
   not. Never present the two as like-for-like.
6. **Never aggregate rows of different epistemic status into a win/loss count.**
   Lane 1 is a **defect detector, not a leaderboard**. Its metrics answer three
   different questions and the validity audit
   ([../../../docs/benchmarks/lane1-validity-2026-08-07.md](../../../docs/benchmarks/lane1-validity-2026-08-07.md))
   found them being reported as one score: (a) *is this engine modelling
   different physics?* — often legitimately, e.g. momentum metrics that largely
   detect a coordinate representation; (b) *is the implementation buggy?* — the
   lane's genuine value, five real defects found this way; (c) *does the engine
   honour the `.wbt` contract?* — which is the translation audit's question, not
   a trajectory metric's. "Newton loses N of 11" mixes all three and means
   nothing. Quote a named metric on a named scene at a named dt, or quote
   nothing.
7. **A metric at the integrator's own truncation floor is not a deficit.** T4's
   `energy_drift_rel` 0.381 sits beside raw MuJoCo's 0.387 on the same scene:
   that is the solver family's error, i.e. the ceiling our integration can reach,
   and reporting it as OmniSim scoring badly is a category error. Before calling
   a lane-1 gap ours, check the external arm on the same row.
8. ⛔ **DO NOT ADD AN ISAAC SIM / ISAAC LAB ARM TO THIS SUITE, AND DO NOT PUBLISH
   ANY NUMBER WE MEASURED FROM NVIDIA SOFTWARE.** This is a licence constraint,
   not a methodological preference. **NVIDIA Software License Agreement §8.9**
   (last modified 2026-05-07) states:

   > *"Customer may not distribute or disclose to third parties the results of
   > benchmarking, competitive analysis, regression or performance data relating
   > to the Software without the prior written permission from NVIDIA."*
   > — <https://www.nvidia.com/en-us/agreements/enterprise-software/nvidia-software-license-agreement/>

   The same clause class appears in the Isaac Sim Additional Software and
   Materials License and the Isaac ROS Software License, and it plausibly reaches
   Omniverse Kit, `ovrtx`, `omni.replicator.core` and the Isaac ROS `.so` GEMs.

   **The sanctioned posture, which costs us nothing:** cite NVIDIA's *own
   published* figures, attributed and dated, and never generate our own. That is
   already what [`docs/developer/simulator-comparison.md`](../../../docs/developer/simulator-comparison.md)
   and the README do — every Isaac cell there is their documentation, marked as
   such. Note this suite already declines to run Isaac (§Layout), so today the
   rule is descriptive; it is written down here so it stays that way when someone
   reasonably proposes adding the arm.

   ⚠️ **This is a documented constraint, not legal advice, and nobody in this
   repo is counsel.** If a measured Isaac comparison ever becomes commercially
   necessary, the question for a lawyer is narrow and worth asking precisely:
   *does §8.9 bind a party who never accepted the SLA — e.g. results obtained
   from a third-party publication, or from a cloud instance whose operator
   accepted it — and does it survive the May 2026 Omniverse licensing change?*
   Until that has an answer in writing, treat the ban as absolute.

   ✅ **What is unaffected**, because none of it is NVIDIA software: bare MuJoCo,
   MuJoCo-Warp, PyBullet, upstream Webots and Gazebo remain measurable and are
   measured here. **Newton** is Apache-2.0 under the Linux Foundation, so
   measuring it — including as the engine Isaac Lab also uses — carries no such
   restriction. The constraint is narrow; it removes one arm, not the suite.
