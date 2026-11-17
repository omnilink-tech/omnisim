# The physics, side by side

**What this page is.** The physics half of "how does OmniSim compare?", split into the two things
that are constantly conflated in simulator comparisons: **what we measured** and **what everyone
else claims**. It was moved out of the [README](../../README.md) to keep the front page short; nothing
was softened on the way.

Two tables, deliberately kept apart. The first is **measured** — three engines, one harness, one
machine, one afternoon. The second is **cited** — nobody else's physics was measured by us, and a
row we could not source says so instead of guessing.

Evidence markers, as everywhere else in this repo: 📊 measured by OmniBench · ✅ verified against a
primary source · ◐ primary-source extraction, not audited · ⚠️ vendor claim or contested · — not
established.

**Related pages.** Capability comparison with sourcing: [simulator-comparison.md](../developer/simulator-comparison.md) ·
throughput: [performance-comparison.md](performance-comparison.md) · what "correct" is scoped to:
[correctness-scope.md](correctness-scope.md) · what "deterministic" is scoped to:
[determinism-scope.md](determinism-scope.md) · the suite itself:
[tests/benchmarks/omnibench/](../../tests/benchmarks/omnibench/).

---

## 1. Measured: correctness against analytic ground truth 📊

OmniBench lane 1 drops seven scenes with closed-form answers into each engine and scores the error
against the analytic solution — **no engine's output is used as the reference**. Rows below are
`dt = 4 ms`, machine `9722d23d12a3` (RTX 3060 laptop, Ryzen 16-core, Windows 11), engine build
`78bf841d0` (binary sha256 `68d40223f5d0e9c1`, Newton-attributed via the `.newton.json` verdict
sidecar on every run), OmniSim on the default CPU `mj_step` solver, MuJoCo 3.8.1 and PyBullet as
bare library calls. Reproduce with `python tests/benchmarks/omnibench/run_all.py --lanes 1 --dt-ms 4`.

| scene | what it tests | metric (lower is better) | OmniSim / Newton | MuJoCo 3.8.1 | PyBullet |
|---|---|---|---|---|---|
| T1 `bounce` | restitution, e = 0.8 | peak-height RMSE, rel | **0.0381** | 0.1478 | 0.0734 |
| T2 `incline` | friction-cone stick/slip | stick violation, m | 4.42e‑4 | 4.42e‑4 | 1.95e‑3 |
| T3 `roll` | rolling without slipping | accel error vs (5/7)·g·sinθ, rel | 5.82e‑4 | 1.00e‑3 | **2.04e‑15** |
| T4 `pendulum_energy` | articulated energy conservation | energy drift, rel | **0.3810** | 0.3871 | 0.3859 |
| T5 `momentum` | floating-base momentum | angular-momentum drift, kg·m²/s | **0.510** | 0.765 | 0.797 |
| T6 `stack` | 10-box multi-contact stability | survivors ↑ / max penetration, m | 10 / 9.02e‑4 | 10 / 6.18e‑4 | 10 / **3.56e‑4** |
| T7 `spin` | Dzhanibekov / gyroscopic integration | angular-momentum drift, rel | 2.55e‑5 | 6.25e‑3 | **1.95e‑5** |

Bold is best-of-row, and left off T2 because the two leading values differ by 6e‑9 m — a distinction
the metric cannot support. **Do not add the bolds up into a score** either: these are seven different
questions, and the suite's own honesty rules forbid a win/loss tally. Four things travel with this
table or it misleads:

- **n = 1 per cell, one machine, one timestep.** The full six-point dt sweep, every metric, and the
  per-row deviation notes are in [lane1-postdeletion-2026-08-09.md](lane1-postdeletion-2026-08-09.md).
- **Three of these metrics score a feature an engine does not have.** MuJoCo has no restitution
  coefficient — T1 bounce is emergent from an under-damped constraint spring, calibrated at
  dt = 1 ms — and its soft contact model treats T6's penetration as *deliberate*. PyBullet's
  Dzhanibekov tumble never erupts inside T7's 10 s window, so its conservation figure describes a
  near-steady spin, not a tumble. Every row carries these in its `deviations` field; read them
  before quoting a cell.
- **This is integration fidelity, not a solver contest.** OmniSim's backend *is* the MuJoCo solver
  family, so a delta against bare MuJoCo measures our translation layer between the scene graph and
  the solver — never "beating MuJoCo".
- **Not wall-clock.** Lane 1's timings include a whole engine process and IPC on our side and a bare
  library call on the others. They are marked `INDICATIVE_ONLY` in the data for that reason.

The suite found five real bugs in our own Newton integration on its first run, including gravity
never being plumbed — every Newton world silently ran at −9.81 regardless of `WorldInfo.gravity`.
T3's rolling-acceleration deficit was **47.63%** before that fix and is **0.058%** above. Building
this table also turned up a same-day regression in the suite itself: a floor lift had reached the
OmniSim world generator and the scorer but not the MuJoCo and PyBullet arms, which were dropping
bodies through half a metre of air onto a floor the scorer no longer believed in (fixed in
`f732f028e`; all four affected cells returned to their pre-lift values bit-for-bit).

### 1.1 Reading the retired ODE columns

Older campaigns in this directory carry an `omnisim-ode` arm. Read those columns as **history**:
they were measured while ODE still shipped, and the arm was removed with the engine on 2026-08-08
(commit `bdc02139`). Two results are worth carrying forward.

First, through 2026-07-24 `omnisim-ode` was best or tied-best on 6 of 7 scenes at dt = 4 ms.
⚠️ **That is a comparison of two integrations, not two solvers, and it is routinely misread.** Bare
MuJoCo scored fine on the same scenes with the same solver family; the four defects that made *our
Newton integration* lose — friction-cone offset, rolling inertia, momentum leak, spin loss — were in
our own plumbing between the scene graph and the solver, and all four were fixed in `e7b9fb11`.
ODE's values are now **frozen** in [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json)
as a fixed regression datum. So the deletion did not cost accuracy: it cost the **second in-engine
path** that let a discrepancy be attributed to plumbing rather than to the solver — the instrument
that caught gravity never being plumbed at all. Lane 1's actual oracle is analytic ground truth and
it is untouched. Scope, and the four sentences not to say:
[correctness-scope.md](correctness-scope.md).

Second, the suite's first run **found five real bugs in our own Newton integration**, all since
fixed and re-validated on a second machine. A benchmark that only flatters its author isn't a
benchmark.

Bitwise reproducibility still holds on the CPU MuJoCo solver and still **does not** hold on the GPU
`mujoco_warp` path — per-configuration scope in [determinism-scope.md](determinism-scope.md).

---

## 2. Cited: how the field is built, and what each project claims for itself

Nothing in this table was measured by us. Every cell is the project's own documentation or an
independent study, and the sourcing is in
[simulator-comparison.md](../developer/simulator-comparison.md).

| | solver family | friction cone | GPU-batched physics | determinism, **as the project scopes it** | publishes accuracy *results*? |
|---|---|---|---|---|---|
| **OmniSim** | Newton → MuJoCo solver; CPU `mj_step` default, `mujoco_warp` opt-in ⊘ | elliptic or pyramidal (`newtonCone`) ⊘ | Yes, `mujoco_warp` ⊘ | **Bitwise on CPU `mj_step`** (5/5 contact-rich pairs, incl. 336 contacts + ten live controllers); **refuted on `mujoco_warp`** (0 bitwise of 24 pairs, 9.152 m apart by 1000 steps); cross-machine **untested** 📊 | **Yes** — dated, dt-swept, machine-attributed 📊 |
| **MuJoCo** (+MJX/MJWarp) | soft-constraint convex optimisation, *explicitly not an LCP*; Newton / CG / PGS ✅ | elliptic **and** pyramidal ✅ | Yes — MJX (JAX, GPU+TPU), MJWarp ✅ | *"Bit-wise equality"* achievable **within one version, on the same architecture**; explicitly not across versions or OSes. MJWarp's own FAQ answers **"No"** for GPU ✅ | No — ships invariant tests (energy *ordering*, momentum drift), publishes no results table ✅ |
| **Isaac Sim 6.0 / Isaac Lab** | PhysX 5, TGS default (PGS available); Newton backend **experimental**, MJWarp-only ✅ | PhysX: 2-axis linearised per contact patch (no cone vocabulary in its docs); Newton path exposes both ✅ | Yes ✅ | *"Given the same hardware and Isaac Sim version"* — rigid bodies + articulations **only**, explicitly **not** deformables, explicitly **not** across hardware; plus a GPU work-scheduling caveat ✅ | **None found** — its 13 published benchmark KPIs are all speed, size and load time ◐ |
| **Gazebo Sim** (Jetty) | DART default (Featherstone ABA, Dantzig LCP); Bullet + Bullet-Featherstone preliminary; TPE kinematic-only ✅ | linearised / pyramidal (approximated Coulomb cone) ✅ | **No** — CPU physics; GPU is rendering and sensors only ✅ | **No guarantee published anywhere** ✅ | **None found** — but it ships a substantial analytic test suite (damped free fall against the closed form, torsional friction to under 1%) ✅ |
| **Genesis** 1.3.x | MuJoCo-derived quadratic-penalty formulation; CG / Newton; plus FEM, MPM, PBD, SAP ✅ | **pyramidal only** — docs concede the anisotropy ✅ | Yes — *"tens of thousands of environments on a single GPU"* ✅ | ⚠️ **Its docs and its marketing contradict each other**: docs say seeding *"does not guarantee bit-for-bit determinism on a GPU"* without a slow debug mode; the 1.0 blog claims *"bit-exact result consistency across runs"* ✅ | No. Its published numbers were speed, and the **43 M FPS / 430,000× real-time** headline was independently re-measured at **0.29 M FPS** (~150× lower) once substeps, self-collision and idle actions were corrected. Genesis then published a creditably self-critical methodology report — but the headline was deleted from the README rather than retracted ⚠️ |
| **CoppeliaSim** | ships five — Bullet, ODE, Vortex, Newton Dynamics, MuJoCo. **Pin one or the row is meaningless**, and note the shipped default is **Bullet 2.78**, a 2011-era build ✅ | backend-dependent ✅ | No ✅ | Vendor **disclaims** it: *"you should never expect to always get the exact same results"* ✅ | **None** — and its own manual calls physics engines *"relatively imprecise and slow"* ✅ |
| **Webots** (upstream, R2025a) | ODE, hardcoded to `dWorldStep` — the direct big-matrix solver, ODE's most accurate and most expensive; `dWorldQuickStep` is never called ✅ | ODE pyramid ✅ | No ✅ | Ships a **bitwise** determinism test; reproducible under six stated conditions — same machine, same version, single-threaded (*"using more than 1 thread can result in non replicable simulations"*). Cross-machine not claimed ✅ | No — its physics tests assert **golden values captured from the simulator itself**, plus orderings ✅ |
| **PyBullet / Bullet** | `btMultiBodyConstraintSolver` — Featherstone multibody + sequential impulse; 50 iterations, dt 1/240 s ✅ | **implicit cone by default**; the pyramid is the opt-*out* (`enableConeFriction=0`) ✅ | No — PyBullet wraps *"Bullet 2.x API on the CPU"*; the OpenCL GPU pipeline is experimental and unreachable from `pybullet` ✅ | Author claims *"fully 100% deterministic after calling resetSimulation"* on one platform/compiler — but broadphase pair ordering (`m_deterministicOverlappingPairs`) **defaults to false** ✅ | No — one genuine analytic test (a pendulum against a `scipy.integrate.odeint` reference), one scene, one dt ✅ |

---

## 3. Two conclusions we think are defensible, and one we are careful *not* to draw

- **Shipping analytic-ground-truth tests is normal, and we are not claiming otherwise.** Drake ships
  a whole closed-form solutions library, Chrono checks commercial MSC ADAMS reference trajectories
  into git, Gazebo asserts damped free fall against its closed form — and **Newton, the engine
  OmniSim runs on, ships 13 closed-form verification scenes across six solvers**. Lane 1 is not a
  new idea.
- **What we could not find is anyone who *publishes the numbers*.** Vendor benchmark pages are
  speed, memory and load time, essentially without exception; Isaac Sim publishes 13 KPIs and not
  one is a correctness metric. The only cross-engine accuracy suites are dormant (SimBenchmark's
  last commit is 2021-09-05, and it was written by RaiSim's own developers, who disclose it) or are
  paper artifacts. A dated, machine-attributed, dt-swept correctness table with a runnable command
  under it appears to be rare — we say *rare*, not *first*.
- **We are not claiming OmniSim is the most accurate simulator, and no one should.** The published
  record does not support a single ranking: Drake beat Bullet beat MuJoCo on real-world rigid
  impacts ([Acosta et al., RA-L 2022](https://arxiv.org/abs/2110.00541)), and the ordering
  *inverts* on cloth. The one independent study that measures engines against motion-capture ground
  truth concludes plainly that *"no engine dominates across rigid, textile, and volumetric
  regimes"* ([GAUGE](https://arxiv.org/abs/2608.05948), 2026-08-06 — a preprint days old, and
  unreplicated; it covers Isaac Sim, Genesis and Newton, and finds every one of them 3–4× the real
  noise floor on a bouncing ball). Fidelity is task-dependent. Any table implying otherwise,
  including this one, would be wrong.
