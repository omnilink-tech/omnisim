# OmniSim in context: an evidence-tiered comparison with today's robotics simulators

> ## ⚠️ 2026-08-08 — ONE OF THIS DOCUMENT'S FOUR MEASURED ENGINES NO LONGER EXISTS
>
> `bdc02139` deleted the ODE backend (`src/ode/` + `include/ode/`, 106,283 lines).
> **Newton with `SolverMuJoCo` is OmniSim's only physics backend.** Because this is the
> **positioning** document — the one whose sentences end up in external material — three
> corrections are binding on anything quoted from it:
>
> 1. **Never present OmniSim/ODE as a current tier.** Every 📊 ODE row below was honestly
>    measured and is now **historical and unrepeatable**. The 📊 tier's own definition
>    ("this tier exists for exactly four engines … because those are the four we can
>    actually run") is now three engines: OmniSim/Newton, MuJoCo 3.8.1, PyBullet — and
>    **four again since 2026-08-13**, when upstream Webots R2025a/ODE joined as a live
>    control arm (§1.6). It was never five: OmniSim/ODE and upstream ODE are not the same arm.
> 2. **⚠ THE NO-GPU / CPU-ONLY CLAIM MUST BE RE-WORDED, NOT DROPPED.** Several rows say
>    OmniSim runs on a CPU-only box **"via ODE"**. The claim is still *true* — CPU
>    `SolverMuJoCo` (`mj_step`) is a genuine CPU path — but the **mechanism named is
>    gone**, and this wording also appears in a **work-package deliverable of an external
>    funding commitment** to a non-RTX/CPU fallback. That makes it a
>    third-party commitment whose stated mechanism changed. It also has **not been
>    verified on a genuinely GPU-less machine since the change** (it was campaign item
>    A1, which never ran). Say "CPU `mj_step`", and do not claim it is measured until it is.
> 3. **ODE was removed while its INTEGRATION still scored better in our own suite, and
>    while it was the cheaper one per step.** ⚠️ "Integration", not "solver" — the
>    comparison was `omnisim-ode` vs `omnisim-newton`; bare MuJoCo scored fine on the same
>    scenes, and the four defects behind Newton's deficit were in our own plumbing and were
>    fixed in `e7b9fb11`. So do not let the deletion be read as a fidelity or speed
>    improvement, do not let "ODE = the legacy tier" — corrected twice already, see §9 —
>    creep back in a third time, and do **not** say "ODE was more accurate than MuJoCo",
>    which was never measured. Full scope: [correctness-scope.md](../benchmarks/correctness-scope.md).
>
> Consequence for the evidence base: **there is no second IN-ENGINE path to cross-check the
> plumbing** (the oracle itself, analytic ground truth, is untouched, and bare MuJoCo and
> PyBullet still run). OmniBench lane 1 used ODE as that second path and its generator
> emitted an `ode_pin` per scene; frozen values survive in
> [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json)
> (a golden file, not an oracle). The surviving cross-*simulator* arms (MuJoCo, PyBullet)
> are now the only external check. Record:
> [ode-retirement-campaign.md](ode-retirement-campaign.md).

> ## 🆕 2026-08-16 — FIVE OF THIS DOCUMENT'S CLAIMS WERE STALE
>
> Every correction below is a measurement **that already existed in-tree**, committed, with
> a machine id on it. Nothing was newly run for this edit; the document had simply not
> caught up, and it is what README.md points readers to.
>
> 1. **"No real-hardware validation" was true on 2026-07-26 and is FALSE since 2026-08-13.**
>    ladder0 rung 18 replays **50 recorded real cube tosses** and scores OmniSim on the same
>    metric the published baselines use (§1.6, §2.3, §7). ⚠️ **This is not an accuracy
>    win** — we are 24.8 % of a cube width from reality and the best engine on that data
>    manages 13.5 %. What it buys is that we now *have a number on the board*.
> 2. **"We do not run upstream Webots" was FALSE.** It is a live control arm on two
>    independent instruments — ladder0 and AgentBench (§1.6). ⚠️ The AgentBench arm **beats
>    us 4/5 to 0/10** on one task; that is printed in §1.6 with both of the caveats that cut
>    each way, not buried and not softened away.
> 3. **The lane-4 capability headline moved**: 26/4/9/5 (67 %) → **28 `works` / 4 `degraded`
>    / 6 `broken` / 5 `absent` / 1 no-result = 44 probes, 74 %** (§0).
> 4. **The `njmax` constraint cliff is now confirmed**, after OmniBench lane 4b honestly
>    reported `cliff_detector_validated: false` (§1.6).
> 5. Two evidence-hygiene defects elsewhere in the tree are **flagged, not fixed** (§9.4).

**Rewritten 2026-07-26.** (Supersedes the 2026-07-10 edition; nothing verified there was
deleted, several things were corrected — see §9.)

This is the *capability, evidence, and positioning* reference. Its companion,
[performance-comparison.md](../benchmarks/performance-comparison.md), is a **2026-06-14
snapshot whose four headline ratios are superseded** by its own 2026-07-26 header box —
read that box before quoting it. The measured numbers that are current live in
**[omnibench-2026-07-24.md](../benchmarks/omnibench-2026-07-24.md)** and are summarised
here in §1.

**Three things changed in this edition and they matter:**

1. A **defined subset of OmniSim's cells is now measured**, not self-attested — by
   OmniBench, in one harness, with a machine id on every row (§1). The rest of the OmniSim
   column is still self-attested and still marked as such.
2. Our own **"agent-facing API" claim was too strong and is corrected** (§4.1). Competitors
   *do* have programmatic scene-control surfaces — a ROS 2 standard, natively implemented
   by three of them (OmniSim became a fourth implementation on 2026-08-17, as a **sidecar**
   rather than natively). Our honest differentiator is narrower.
3. The **accuracy column is empty across the whole field**, and the canonical suite that
   used to fill it is dead (§2). This is the most consequential finding in the document.

---

## Contents

- [§0 How to read this — the evidence tiers](#0-how-to-read-this--the-evidence-tiers)
- [§1 What we measured (OmniBench)](#1-what-we-measured-omnibench)
- [§2 Why the accuracy column is empty](#2-why-the-accuracy-column-is-empty)
- [§3 Identity, maintenance, licence, determinism](#3-identity-maintenance-licence-determinism)
- [§4 The capability matrix](#4-the-capability-matrix)
- [§5 Where OmniSim is actually differentiated](#5-where-omnisim-is-actually-differentiated)
- [§6 Per-simulator notes](#6-per-simulator-notes)
- [§7 Where OmniSim loses](#7-where-omnisim-loses)
- [§8 The one-paragraph positioning](#8-the-one-paragraph-positioning)
- [§9 What we could not verify](#9-what-we-could-not-verify)
- [§10 Method and references](#10-method-and-references)
- [§11 Sources and caveats behind the README table](#11-sources-and-caveats-behind-the-readme-table)

---

## 0. How to read this — the evidence tiers

Cross-simulator comparison is adversarial territory: vendors publish best-case numbers,
listicles copy each other, and licences change between releases. So every cell below
carries a marker saying *how we know*.

| Marker | Meaning |
|---|---|
| 📊 | **Measured by us, in one of our own harnesses** — OmniBench (`python tests/benchmarks/omnibench/run_all.py`), ladder0 (`tests/benchmarks/ladder0/run_ladder.py`), or AgentBench. One machine per number, machine id + engine-binary sha256 on every row. **This tier exists for exactly five engines** — OmniSim/ODE (historical), OmniSim/Newton, MuJoCo 3.8.1, PyBullet, and **upstream Webots R2025a/ODE** (added 2026-08-13 — ladder0 and AgentBench control arms, §1.6) — because those are the ones we can actually run. It is **never** applied to Isaac, Gazebo, Genesis, Drake, SAPIEN, Brax, CoppeliaSim, or Unity. |
| ✅ | **Verified against a primary source** by direct fetch (repo, GitHub API, official docs, licence file), with the fetch date given. |
| ◐ | **Primary-source extraction, not audited.** Pulled from an official doc/repo but not re-fetched in this pass. *Probably true, not audited.* |
| ⚠️ | **Vendor claim or contested.** First-party marketing, unstated baseline, or an actively disputed figure. |
| ⊘ | **Self-attested against this checkout.** Checked by us, by us, with no external source and no independent audit. The **weakest** tier in the document. |
| — | **Not established.** We looked and found no citable answer. Written as `[NOT FOUND]` where the absence is itself the finding. |

> ### ⚠️ Read this asymmetry before you read anything else
>
> The 2026-07-10 edition said *every* OmniSim cell was self-attested. **That is no longer
> true, and the change is narrow enough to state exactly:**
>
> - **📊 Measured (OmniBench):** OmniSim/ODE and OmniSim/Newton physics correctness on
>   seven analytic scenes; Go2 batched throughput; determinism grade; G1 train↔deploy
>   structural parity; HTTP-harness driveability. Numbers in §1.
> - **📊 Measured (OmniBench lane 4, added 2026-08-10):** the **capability** cells
>   themselves — which object primitives, joints, actuation modes, devices and
>   world-level contracts actually work — plus the per-machine scaling envelope and the
>   CPU-only (no-CUDA-visible) claim. 45 executed probes, one committed `.wbt` each,
>   each judged by an assertion in physical units:
>   [lane4-capability-matrix.md](../benchmarks/lane4-capability-matrix.md),
>   [`tests/benchmarks/omnibench/lane4/`](../../tests/benchmarks/omnibench/lane4/).
>   **This is the first time this document's capability claims are anything other than
>   self-attested** — so read the **generated matrix, never a figure copied into prose
>   here**, when the two differ. Headline, **2026-08-17**: **32 `works` / 5 `degraded` /
>   4 `broken` / 4 `absent` / 0 no-result**, i.e. **78% of the capabilities that exist
>   actually work**. ⚠️ This box carried the first campaign's **26/4/9/5 = 67%** until
>   2026-08-16, which is exactly the failure mode the previous sentence warns about.
>   ⭐ **AND IT IS NO LONGER ONE MACHINE.** The full 45-probe sweep ran on **M6** (RunPod
>   RTX A4500 / EPYC 7352, Linux) the same day: **43 of 45 verdicts agree**, and neither
>   disagreement is the hardware — `joint.hinge2_motor` is the `OMNISIM_NEWTON_BALL_HINGE2`
>   default flip that the pod's older binary predates (confirmed by re-running the probe on
>   M1 under `--env OMNISIM_NEWTON_BALL_HINGE2=0`, which reproduces the pod's arm
>   displacement to all 17 printed digits), and `object.deformable_cloth` timed out on the
>   pod's cold warp JIT, which is an instrument failure and is scored as such. ⚠️ **The
>   identical 78% is three offsetting changes, not a replication** — say so when quoting it.
>   ⚠️ **The second machine's real payoff was a STALE ROW**: `device.lidar` had been
>   published as `no result` since 2026-08-15 while working. The pod measured it working
>   (min range 2.90079665 m vs an expected 2.9, 32/32 finite returns); M1 then re-measured
>   it working in 3.7 s. That is the 31 → 32 above.
>   [Campaign](../benchmarks/lane4-multimachine-2026-08-17.md).
>   ✅ **THE 75% → 78% MOVE IS A GENUINE ENGINE IMPROVEMENT, and it is the one movement
>   in this box that may be quoted as progress.** `2094660ef` flipped
>   `OMNISIM_NEWTON_BALL_HINGE2` on — the blocker was newton 1.2.0's d6 → MuJoCo actuator
>   mapping, fixed by the vendored upgrade to newton 1.5.0 — and `joint.hinge2_motor` went
>   `broken → works`: a motorised `Hinge2Joint` commanded 0.8 rad now tracks it **exactly**
>   and carries the arm **0.1951 m**, against 4.00e-04 m before. ⛔ **The same commit
>   claimed `BallJoint` too and that half is refuted**: `joint.ball_motor` moved
>   **2.67e-07 m** on the same binary and stays `broken`, because the commit's evidence
>   test has a motorised hinge2 arm but a *passive* ball arm. ⚠️ Both probes were also
>   repaired (they authored limit-less motors, which the engine builds as velocity wheels
>   that ignore `setPosition()` by design) — the repair is what made the engine change
>   measurable, and no assertion or threshold moved.
>   ⚠️ **THE EARLIER 72% → 75% MOVE ON 2026-08-17 IS A PROBE REPAIR, NOT AN ENGINE
>   IMPROVEMENT — do not quote that one as progress.** `phenomenon.friction_declared_in_world`
>   published `broken` for four days on an invalid scene: it dropped a **0.2 m cube** on a
>   **55°** incline, and a block holds only if *both* µ ≥ tan θ **and** tan θ < b/h. A cube
>   has b/h = 1.0 against tan 55° = 1.428, so it toppled at any friction whatsoever. The
>   giveaway, in bare MuJoCo: the same cube travels 23.30 m at µ=2, 23.46 m at µ=10 and
>   **22.29 m at µ=100** — a verdict that will not move when the variable under test is
>   raised fiftyfold was never measuring that variable. Rebuilt on a low-CoM slab
>   (b/h = 10, topple angle 84.3°) it measures `works`, and its new **negative arm**
>   (`phenomenon.friction_slides_below_coulomb_bound`, same slab at µ=1.3, *below* the
>   bound) slid **1.4402 m against the analytic 1.442 m — ratio 0.999**. So declared
>   friction reproduces the Coulomb bound to three significant figures, and the pair now
>   brackets tan 55° from both sides, which the single passing arm never could. The
>   `broken` claim was retracted from the README and CHANGELOG in `1fc3b4b5a`; the probe
>   itself was retired on 2026-08-17. Counts move `broken` 6→5, `works` 28→30 (the
>   negative arm is a new probe, 44→45).
>   Three further notes travel with the number. **(1)** The `object.deformable_cloth` repair this
>   box used to ask for **has landed** — it was a mis-authored probe, never an engine
>   limitation, and the giveaway was in its own row: a *field*-level diagnostic
>   (`Skipped unknown 'size' field in Cloth node`, for a field `Cloth` has never had),
>   which proves the node parsed, while the same row's machine block recorded the VBD
>   cloth solver initialising. Its `absent_markers=("Cloth",)` matched that field
>   complaint and published it as a missing node. Re-measured, the row is `degraded`,
>   evidenced by the engine registering the authored **441 particles**; the counts above
>   move with it (`absent` 5→4, `degraded` 4→5, 74%→72%). ⚠️ **The "one run, one binary"
>   provenance is therefore GONE and the matrix now carries its mixed-binary banner** —
>   and as of 2026-08-17 it names **three** builds, not two: **41** rows on
>   `f3f003bf0304bc5e` (2026-08-15), **2** on `89e978269dc27a23` (2026-08-16, cloth +
>   soft-body), **2** on `6aac9ae1b461f567` (2026-08-17, the two friction probes). The
>   campaign is **topped up, not re-swept**. Read a row's own machine block, not this box.
>   **(2)** The `device.touch_bumper` doc-vs-measurement mismatch this box used to name is
>   **resolved** — engine fixed, re-measured working. **(3)** The one scaling threshold the
>   lane could not reproduce (the `njmax` cliff — `cliff_detector_validated: false`,
>   explicitly *not* a pass) has since been **confirmed by a second instrument** (§1.6).
>   ⚠️ One blemish on the "one binary" provenance: `joint.hinge2_motor` is the only row
>   logged `engine: omnisim-unverified` with `finalise UNCONFIRMED (SUSPECT)`, and it is
>   one of the 6 `broken`.
> - **⊘ Still self-attested:** the rest of the OmniSim column — licence, URDF import,
>   rendering class, ROS posture, agent-surface capability, sim-to-real status. No
>   external audit exists for any of it.
>
> Measured is *not* the same as independently verified. OmniBench is our harness, run by
> us. What it adds over self-attestation is that it is **reproducible, machine-attributed,
> and it reports our losses as loudly as our wins** — it is how we found five of our own
> engine bugs (§1.5). Treat it as auditable, not as audited. Nothing here has been
> replicated by a third party.

Three structural warnings:

1. **"A step" is not a portable unit** across engines, and neither is "FPS." OmniBench's
   lane-2 env-step is one **16 ms control step = 8 × 2 ms physics substeps**; MJX/MJWarp
   headline numbers are **single physics steps**. An 8× unit factor sits between them
   before you even reach the scene difference.
2. **The scene defines everything.** A free-floating ant is not eight colliding quadrupeds.
   Contact count dominates: MJWarp's own nightly spread is ~3,400× across scenes (§2.4).
3. **Licence and maintenance facts rot fast.** Isaac Sim went from proprietary EULA to
   Apache-2.0 within this document's memory. Every claim is date-stamped. Re-check before
   quoting externally.

---

## 1. What we measured (OmniBench)

Suite: [`tests/benchmarks/omnibench/`](../../tests/benchmarks/omnibench/), contract in
[`SPEC.md`](../../tests/benchmarks/omnibench/SPEC.md). Campaign report:
[omnibench-2026-07-24.md](../benchmarks/omnibench-2026-07-24.md). Raw rows under
`tests/benchmarks/omnibench/results*/`.

**Machines** (a result that does not name its box is not a result):

| key | machine id | hardware | OS / toolchain |
|---|---|---|---|
| **M1** | `9722d23d12a3` | RTX 3060 Laptop 6 GB (driver 596.36), AMD Fam25 Mod80, 16 threads | Windows 11, MinGW, py 3.12.9 |
| **M2** | `6fa66da0cde0` | RunPod RTX 4090 24 GB (driver 570.195.03), 32 cores | Linux 6.8.0-90, gcc, py 3.11.10 — **pre-fix** |
| **M3** | `65dd6587d5c9` | RunPod RTX 4090 24 GB (driver 580.126.20), 32 cores | Linux 6.8.0-107, gcc, py 3.11.10 — **post-fix** |
| **M4** | `c72ce5632c81` | RunPod RTX 4000 Ada 20 GB (driver 550.127.05), AMD EPYC 7352, 48 threads | Ubuntu 22.04, gcc, py 3.11 — throughput campaign, 2026-08-17 |
| **M5** | `b5dadd645b1f` | RunPod RTX 4090 24 GB (driver 570.195.03), 32 threads | Ubuntu 22.04, gcc, py 3.11 — throughput campaign, 2026-08-17 |
| **M6** | `8ab788c4c833` | RunPod RTX A4500 20 GB (driver 570.195.03), AMD EPYC 7352 24-Core @ 2.3 GHz, 48 threads | Ubuntu 22.04, gcc, py 3.11 — **lane 4 + cloth campaign, 2026-08-17** |

⚠️ **M2/M3 ran different builds on different OSes from M1**, and so do M4–M6: no two of these
machines have ever run the same engine binary, because one is a MinGW/Windows build and the rest are
gcc/Linux builds off a network volume. Every cross-machine comparison in this document therefore
confounds machine with compiler and OS unless it says otherwise. The one place that confound is
absent is the cloth step-cost matrix, which never launches the engine
([lane4-multimachine-2026-08-17.md §4](../benchmarks/lane4-multimachine-2026-08-17.md)).

Engines under test: **OmniSim/ODE**, **OmniSim/Newton** (embedded mujoco-warp,
`newtonSolver "mujoco"`), **MuJoCo 3.8.1**, **PyBullet** (version string not recorded by
the rows — a suite gap).

### 1.1 Lane 1 — physics correctness against analytic ground truth 📊

> ⚠️ **This table is PRE-DELETION** — it still carries an `omnisim-ode` column, and that arm was
> removed with the engine on 2026-08-08 (`bdc02139`). For the current three-engine table, plus the
> cited what-each-project-claims comparison, see
> [physics-comparison.md](../benchmarks/physics-comparison.md); the full post-deletion sweep is
> [lane1-postdeletion-2026-08-09.md](../benchmarks/lane1-postdeletion-2026-08-09.md). The ODE values
> below are retained as a frozen regression datum, not as a live measurement.

All values `dt = 4 ms`, machine **M1**, OmniSim/Newton **post-fix** (2026-07-25). Lower is
better everywhere except T6 survivors. **Bold = best of the four.**

| test | metric | OmniSim/ODE | OmniSim/Newton | MuJoCo 3.8.1 | PyBullet |
|---|---|---|---|---|---|
| T1 `bounce` | restitution RMSE (rel, 5 peaks) | **0.01788** | 0.03811 | 0.1478 | 0.07337 |
| T2 `incline` | stick violation (m) | **2.90e-05** | 4.42e-04 | 4.42e-04 | 1.95e-03 |
| T2 | slide accel rel err | 8.78e-05 | 3.81e-02 | 6.18e-03 | **6.56e-05** |
| T2 | transition angle err (deg) | 0.0651 | 0.0651 | 0.0651 | 0.0651 |
| T3 `roll` | rolling accel rel err | 1.06e-03 | 5.82e-04 | 1.00e-03 | **2.04e-15** |
| T4 `pendulum` | energy drift rel (10 s) | **0.1300** | 0.3810 | 0.3871 | 0.3859 |
| T5 `momentum` | linear momentum max (kg·m/s, must be 0) | **9.78e-15** | 3.910 | 5.847 | 6.033 |
| T5 | angular momentum drift **abs** (kg·m²/s) | **1.03e-03** | 0.3041 | 0.7646 | 0.7968 |
| T6 `stack` | survivors / creep (m·s⁻¹) / penetration (m) | 10 / **5.21e-08** / **2.45e-04** | 10 / 5.69e-06 / 9.02e-04 | 10 / 2.35e-06 / 6.18e-04 | 10 / 1.87e-04 / 3.56e-04 |
| T6 | largest dt with 10/10 survivors | **16 ms** | ≥8 ms¹ | **16 ms** | 4 ms |
| T7 `spin` | angular momentum drift rel | **1.93e-05** | 2.55e-05 | 6.25e-03 | 1.95e-05² |
| T7 | rotational KE drift rel | **3.47e-08** | 7.95e-08 | 4.54e-03 | 3.55e-08² |

¹ Post-fix Newton was rerun only at dt=4 and dt=8 (both 10/10); the sweep above 8 ms was
not repeated. ² PyBullet's T7 conservation describes a **near-steady spin, not a
Dzhanibekov tumble** — its own recorded deviation notes it damps the off-axis seed ~3× in
the first second, so the intermediate-axis eruption never happens in the 10 s window. Not
comparable in kind to MuJoCo's 6.2e-03.

**Cross-machine reproduction (M1 Windows/MinGW/3060 vs M3 Linux/gcc/4090):** T2 transition
angle, T2 slide accel, T3 roll accel + slip, T5 linear momentum, T6 penetration, T7 both
metrics, T1 and T4 regression cells are **digit-identical to 15–16 significant figures**.
The two that differ are the contact-settle-sensitive scalars: T2 stick violation 0.44 mm
(M1) vs 0.68 mm (M3), and T6 creep 5.69e-06 vs 3.94e-06 m·s⁻¹ — both still 2–3 orders
below pre-fix.

**Cell-level cross-machine census** (M1 vs M2, 180 comparable OmniSim v0 cells, recomputed
from the rows): **124 bit-identical**; of the 56 that differ, **29 at rel < 1e-9**, 14 in
1e-9…1e-3, **13 at rel ≥ 1e-3** (9 in T5, 3 in T6 creep, 1 a T4 dt=32 blow-up artifact).
MuJoCo and PyBullet are *not* bit-identical across machines: only 97 of their 180 cells
are, and at dt=4 MuJoCo's T5 ratio diverges in the **4th** significant figure.

**Do not read wall-clock across engines from these rows.** OmniSim lane-1 timings are
measured inside the supervisor controller and include the whole engine-process step plus
IPC; MuJoCo/PyBullet rows are bare in-process library calls. Different quantities.

### 1.2 Lane 2 — batched throughput 📊

Unitree **Go2**, contacts ON, self-collision structurally off in the engine's own exported
model, actions `uniform_random_joint_range` every control step (**never idle**;
`idle_guard_ok: true` on every row). **Unit: control env-steps/s, where one env-step = one
16 ms control step = 8 × 2 ms physics substeps.**

| batch | 3060 raw mjwarp (graphed) | 3060 OmniSim/Newton (graphed) | ratio | 4090 raw mjwarp (graphed) | 4090 OmniSim/Newton (graphed) | ratio |
|---|---|---|---|---|---|---|
| 256 | 45,685 | 37,732 | 1.21× | 73,609 | 60,619 | 1.21× |
| 1024 | 120,577 | n/m | — | 254,945 | 210,852 | 1.21× |
| 4096 | 165,616 | 129,431 | 1.28× | 654,034 | 500,105 | 1.31× |
| 8192 | n/m (6 GB OOM) | n/m | — | 870,459 | 650,487 | 1.34× |

3060 = M1, 4090 = M3. **The v0 headline ratios (17.4× / 5.9× / 3.5× / 2.4×) are retired**:
they compared a CUDA-graphed baseline against an ungraphed OmniSim probe. Graphed against
graphed, the embedded deploy solver is within **21–34%** of raw mujoco-warp. Independently
corroborated by dev-shakeout rows with *both* sides ungraphed: 1.02× @256 and 1.01× @4096.

**Full closed-loop RL (tier C — in-engine PPO through `omnisim-bin`, rollout + update +
engine round-trip in the loop):** **10,228** env-steps/s @256 on M1; **333,036** @4096 on
M3 — 0.67× the graphed OmniSim stepping rate at the same batch, 0.51× the graphed raw
rate. Quote which baseline you mean.

**⚠️ Tier A OmniSim rows step the embedded deploy solver in-process, not the full
`omnisim-bin` engine loop. Only tier C does that.** Never conflate them.

### 1.3 Lane 3 — determinism, parity, driveability 📊

| axis | result | scope |
|---|---|---|
| **Determinism** | **Bitwise** — `max_abs_dev = 0.0`, `first_div_step = -1`, 400 compared steps, **10/10 rows** — on **ODE** and on **Newton/XPBD in one light-contact sphere-drop world**. ⚠️ On the GPU `mujoco_warp` solver it is **refuted, not unmeasured**: 0 bitwise of 24 same-config cold pairs | `cold_cold` **and** `cold_warm` (worldReload in the same process), on M1, M2 and M3. Per-configuration scope, the `mujoco_warp` refutation and the CPU `mj_step` bitwise-5/5 counterpart: [determinism-scope.md](../benchmarks/determinism-scope.md) |
| **Train↔deploy structural parity** (G1, `g1_golden_parity.py --structural`) | deploy-default: **1** real physics gap (`body_ipos`, the legacy COM-at-link-origin); with `OMNISIM_NEWTON_USE_LINK_COM=1`: **0** real gaps, 3 representational diffs, **pass** | M1 only — the sole rows with `p2_trustworthy: true` |
| **Agent driveability of the HTTP harness** | **10/10, score 1.0** across 10 probes (load, hot-reload, scene tree, bounds, deterministic step, event cursor, joint state, verified framing, screenshot, structured diagnostic on a broken world) | M1, `omnisim-newton`. Probe list: [`lane3/DRIVEABILITY.md`](../../tests/benchmarks/omnibench/lane3/DRIVEABILITY.md) |

**Determinism caveat that must travel with the claim** — the single most misquotable number
in this document:

1. **"Both backends" means ODE and Newton-under-XPBD, not "all of Newton."** The lane-3a
   world's backend sidecar reports **`XPBD(iters=10)`**, whereas every lane-1 world pins
   `newtonSolver "mujoco"` (and lane-1 Newton ran the **CPU `mj_step`** path on M3). The
   grade does **not** automatically transfer to the solver lane 1 measures.
2. **One light scene.** `lane3_determinism.wbt` is five spheres dropped on a pedestal with
   no joints, no motors and no actuated robot — real multi-contact, but nothing near fleet
   contact density. XPBD also dispatches warp kernels, so the XPBD half of this grade is
   **not** evidence that XPBD reproduces at scale; that has not been measured.
3. **The GPU `mujoco_warp` path is refuted.** On machine `9722d23d12a3`: **0 bitwise of 24**
   same-config cold pairs across six scenes (80 → 336 concurrent contacts), ~5e-5 m
   deviation at 120 steps growing to **9.152 m at 1000 steps**, mechanism traced to
   `wp.atomic_add` contact-slot claiming in mujoco_warp and confirmed by a GPU-contention
   A/B (3× wider spread, no range overlap). The **same worlds** graded **bitwise 5/5** under
   `newtonSolver "mujoco"` (CPU `mj_step`) — including a 336-contact, ten-robot scene with
   ten live controllers — and under ODE.
4. ✅ **Cross-machine bitwise identity was untested until 2026-08-17. It now HOLDS, scoped
   to one binary.** Two RunPod hosts running **the same** Linux binary (sha256
   `6f7e2217426a2088`) — an RTX 4000 Ada / EPYC 7352 / 48 threads and an RTX 4090 /
   32 threads — produced **byte-identical** recordings on both the 5-sphere scene and the
   contact-rich one (10 robots + 64-box tower, 336 contacts, ten live controllers). Lane 1
   agrees independently: **107 of 114** comparable metric cells bit-identical across those
   machines, the other 7 at rel ≤ 2.7e-16 (one ULP, in the offline scorer).
   ⚠️ **The scope word is "binary", never "version".** The Windows laptop's own build differs
   from both pods on the same scenes — that is the negative half of the result, and it is why
   this may never be stated as "OmniSim is reproducible across machines". Same binary, yes;
   same *source* built by a different toolchain, no.
   ⚠️ This also **retires the old "56 of 180 cells differ" census** in §4 as a cross-machine
   claim: that comparison ran two *different builds on different OSes*, so the spread it
   measured was the build, not the machine.

Full scope table, numbers and reproduction commands:
[determinism-scope.md](../benchmarks/determinism-scope.md). That file is the source of truth
— if this document disagrees with it, it is right and this is stale.

**Parity caveat:** M3's parity rows report 0 gaps *and* `p2_trustworthy=false` ("P2 deploy
driver INCONCLUSIVE"), plus `representational_diffs=0` where M1 reports 3. **M3's zeros
neither confirm nor contradict M1.** Driveability was never rerun on a pod; M2's 2/10 was
on `omnisim-ode` with rc=1 and is not like-for-like against M1's 10/10 on
`omnisim-newton`.

### 1.4 What is *not* measured — stated, not left blank

| simulator | measured by us? |
|---|---|
| OmniSim/ODE, OmniSim/Newton, MuJoCo 3.8.1, PyBullet | **Yes** — OmniBench §1.1–1.3 |
| **Upstream Webots R2025a / ODE** ⬅ *corrected row* | **Yes — we run this engine.** Not by OmniBench, but as a live control arm on two independent instruments: **ladder0** (rungs 9 and 11, cross-arm numbers in §1.6) and **AgentBench** (Phase W). ⚠️ This document said "we do not run this engine" until 2026-08-16. ⚠️ Cross-platform as installed — WSL2 Linux Webots vs native Windows OmniSim ([webots-control-baseline.md](webots-control-baseline.md) §8) |
| Isaac Sim / Isaac Lab | **Not measured — we do not run this engine.** Not installed. ⚠ "No contact with it" would be too strong: [`isaac_remote.py`](../../projects/policies/research/backends/isaac_remote.py) is a **written but never-executed** Isaac Lab training backend (its own docstring: *"Tested locally: no"*), and `docs/benchmarks/data/isaaclab-via-maniskill-benchmark-rtx4090.csv` is a committed **third-party** measurement of Isaac Lab — ManiSkill's, not ours, and cited nowhere |
| Gazebo (gz-sim) | **Not measured — we do not run this engine.** |
| Genesis | **Not measured — we do not run this engine.** |
| Drake | **Not measured — we do not run this engine.** |
| SAPIEN / ManiSkill 3 | **Not measured — we do not run this engine.** |
| Brax | **Not measured — we do not run this engine.** |
| CoppeliaSim | **Not measured — we do not run this engine.** |
| Unity / O3DE / Colosseum | **Not measured — we do not run this engine.** |

**The rest of this table was re-audited 2026-08-16 and every remaining row holds.** No
adapter, runner, install step or import exists anywhere in the tree for Gazebo, Genesis,
Drake, SAPIEN/ManiSkill, Brax, CoppeliaSim, Unity, O3DE or Colosseum. AgentBench's `sims.py`
*declares* gazebo / isaac / coppeliasim / genesis / sapien with `status="declared"` and a
`blocked_by` field, and `adapters/__init__.py` registers `gazebo` and `isaac` so a named
simulator resolves to "not implemented yet" rather than "unknown sim" — but
[`preregister/v1/arm_gates.json`](../../tests/benchmarks/agentbench/preregister/v1/arm_gates.json)
is the machine-readable truth: `{gazebo: false, genesis: false, isaac: false, omnisim: true,
webots: true}`. ⚠️ **A declared arm is not an arm.** One nuance worth keeping: **Drake's
published numbers are load-bearing here** — they are the external acceptance bound rung 18
is judged against (`ladder0/rungs.py`, `RUNG18_BASELINES`, taken from Acosta et al. Table
II). Cited, never run.

**No published competitor number is quoted anywhere in the OmniBench campaign.** That was
deliberate for v0: every number in that report is same-harness, same-machine. The
competitor throughput figures collected in §2.4 of *this* document are their published
numbers under their conditions, kept in a separate table for exactly that reason.

Also not measured, at all: **rendering-on throughput** (every lane-2 row is headless
batched stepping); **an ODE batched row** (lane 2 has no ODE variant, so the old
"batched-GPU ÷ single-env-CPU" ratio still has no same-harness denominator); **an OmniSim
variant of tier B**. ⚠️ **"Any real-hardware validation" used to be the last item on this
list and no longer belongs on it** — OmniBench still has none, but ladder0 rung 18 does
(§1.6), so the gap is now specific to this suite rather than to the project.

### 1.5 The suite found five of our own engine bugs

OmniBench v0 recorded four reproducible Newton-integration defects plus two non-scene
ones. They root-caused to **five distinct engine bugs**, all fixed 2026-07-25 (`e7b9fb11`)
and cross-machine validated:

| defect | pre-fix 📊 | post-fix 📊 |
|---|---|---|
| **`WorldInfo.gravity` never plumbed to Newton** — explains both T5 and T7 | T5 linear momentum **18.69** kg·m/s; T7 drift **exactly 1.0** (total loss) at every dt | T5 **3.910**; T7 **2.55e-05** / rot-KE **7.95e-08** |
| **Husky-wheel inertia preset fallback** on Solids with no explicit `inertiaMatrix` | T3 roll accel err **0.4763** (47.6% low), dt-independent | T3 **5.82e-04** |
| **MuJoCo-stock pyramidal cone + impratio 1** | T2 stick **0.1806** m, transition err **4.065°** (effective μ 0.414 vs 0.5) | T2 **4.42e-04** m, **0.0651°** |
| **`OMNISIM_FORCE_ODE` bypass** (raw accessors drove Newton regardless) | required an `_odepin` world workaround on every ODE row | gated by `OmPhysicsBackendRegistry::odeForced()` |
| **t=0 supervisor `setVelocity` dropped on Newton** | recorder forced to a torque-impulse spin-up | `method: "setVelocity"`, `spinup_steps: 1`, ‖ω‖ = 5.000 through 10 s |

Two things the fixes did **not** improve, reported as loudly as the wins: T6
`max_penetration_m` **rose** 6.17e-04 → 9.02e-04 m (**+46%**) while creep fell ~200–290×,
and the T5 angular-drift **ratio** got worse on both rulers (v0 0.9999 → 1.181; v1 0.125 →
0.977) because its peak-|L| normalizer collapsed 1.757 → 0.3112 — the **absolute** drift
moved only 0.219 → 0.304 kg·m²/s. Cite `linear_momentum_max` and
`angular_momentum_drift_abs`; do not cite the T5 ratio in either direction.

One open observation, explicitly flagged **not-measured**: ODE wall-clock fell 13–22× on
M1 and 8–349× between the pods across the fix, with byte-identical metrics. The
FORCE_ODE-bypass fix is the obvious hypothesis; **nothing in the rows establishes it.**

### 1.6 Outside OmniBench — real-hardware validation, and upstream Webots as a control arm 📊

**Added 2026-08-16; the measurements are from 2026-08-01 and 2026-08-13.** OmniBench is not
the only suite in this tree that produces 📊 rows, and this document had caught up with
neither of the other two. [`tests/benchmarks/ladder0/`](../../tests/benchmarks/ladder0/) is
a physics ladder whose expected values are derived from Newtonian mechanics and the geometry
the scene declares — with exactly one exception, rung 18, whose ground truth is **a recorded
measurement of physical reality**. [`tests/benchmarks/agentbench/`](../../tests/benchmarks/agentbench/)
scores whether an LLM can get a job done, with a per-simulator adapter.

ladder0 campaign: **2026-08-13**, machine **M1** (`9722d23d12a3`), engine binary sha256
**`cbd80861c9f0d314`**, MuJoCo 3.8.1 CPU `mj_step`, upstream **Webots R2025a / ODE under
WSL2** at `optimalThreadCount 1`.

#### Real-hardware validation exists now — and we are not good at it

Rung 18 replays **50 recorded real cube tosses** onto a table, from row 0 of each recording,
scored against the recording **by the same metric the published baselines use**:

| | position err (% of cube width) | rotation err (deg) |
|---|---|---|
| **OmniSim / Newton → SolverMuJoCo, CPU** 📊 | **24.845** | **23.104** |
| **bare MuJoCo 3.8.1, CPU** 📊 (our own arm) | **24.845** | 23.104 |
| Drake — published (Acosta et al., §2.3) ◐ | **13.5 ± 8.2** | 16.5 ± 20.0 |
| Bullet — published ◐ | 14.9 ± 8.9 | 16.5 ± 20.2 |
| MuJoCo — published ◐ | 25.1 ± 10.8 | 21.7 ± 21.4 |
| upstream Webots / ODE | *not run on this rung* — the arm declared it `UNIMPLEMENTED` | |

⚠️ **Do not present this as an accuracy win, and do not let anyone quoting this document do
it either.** ladder0's own README carries the honest sentence and it is the one to reuse:
*"OmniSim is where the field is, and the field is not good at tossed cubes."* We are
**24.8% of a cube width** away from what the cube actually did; the best engine measured on
this data manages **13.5%**. The passing bound (35.9%) is the published MuJoCo row plus one
published standard deviation — clearing it means *our translation layer did not lose what
the solver gave us*, not that OmniSim is accurate.

What it does establish — and it is exactly what §7 used to say we had nothing of:

- **We publish our own score on a real-world impact dataset.** Before 2026-08-13, every
  number in this document was analytic ground truth or self-consistency.
- **`embed_gap` = 0.000127 percentage points of cube width (≈0.13 µm)** 📊 between our
  Newton→SolverMuJoCo path and the bare MuJoCo it embeds, on the same 50 tosses reduced by
  the same code. **Our number is indistinguishable from the solver we embed.** That is what
  a faithful translation layer looks like and it says nothing about whether either engine is
  right — it is the one check in the ladder we can lose and cannot win.
- **`tunnel_depth` = 0 on both arms**, so neither figure is a cube that fell through the
  table.

#### Upstream Webots is a control arm — on two independent instruments

⚠️ **Mandatory caveat, and it travels with every number below**
([webots-control-baseline.md](webots-control-baseline.md) §8): the comparison **as installed
is cross-platform** — WSL2 Linux Webots against native Windows OmniSim, same CPU and GPU.
WSL2 CPU performance is near-native, but this **is not same-platform parity**. A
same-platform run needs either a Windows-native Webots install (refused — its installer
writes machine-scope `WEBOTS_HOME` and would damage this checkout) or OmniSim under Linux.

**ladder0 — three arms side by side** 📊. All three PASS rungs 9 and 11:

| quantity | OmniSim | upstream Webots (ODE) | bare MuJoCo |
|---|---|---|---|
| rung 9 `repeat_delta` (judged, tol 0) | **0.0** | **0.0** | **0.0** |
| rung 9 `fall_interval` (analytic 0.147821 s, tol 8 ms) | 0.147685 | 0.147685 | 0.147685 |
| rung 11 `distance_worst` (judged, tol 0.05) | 8.55e-04 | **2.82e-14** | 2.48e-03 |

`fall_interval` agreeing to six decimals across three independently authored scenes and
three different solvers is the *correct* outcome, and it is why the lower rungs differentiate
nobody. On rung 11, **upstream ODE is ten orders better than we are, and we are ~3× better
than bare MuJoCo** — that ordering is published as it fell.

**AgentBench — and this arm does not flatter us** 📊. Campaigns `phasew_cc_v1` (OmniSim) vs
`phasew_cc_v1_B` (upstream Webots), **2026-08-01**, same model:

| task | OmniSim | upstream Webots |
|---|---|---|
| `B1_overlap_audit` | 3/5 | **4/5** |
| `B2_subject_in_frame` | **0/10** | **4/5** |
| `B3_measure_and_report` | 2/10 | 1/5 *(same rate, 20%)* |
| `C1_parse_error_fix` | 5/5 | 5/5 |
| `C2_fall_through_floor` | 5/5 | 5/5 |

⚠️ **B2 is 0/10 for us against 4/5 for the engine we forked**, and B1 loses too. This is the
same programme whose first surface-ablation A/B also failed to show our surface winning
(§5.1), and it is printed here for the same reason: a comparison document that only shows
the arms it wins is not a comparison document.

**Two things must travel with those rows, in both directions.** ⚠️ **Against reading the
loss as bigger than it is:** 8 of our 10 B2 cells and 7 of our 10 B3 cells are graded
**`INVALID`, not `FAIL`** — each carries a `grading_recovery` block reading
`kind: "contaminated_environment"`, i.e. the session provably worked against a foreign
harness world and **the verdict measured cross-cell contamination rather than the agent**.
`0/10` is a true PASS count, but eight of those ten are the harness disqualifying itself.
⚠️ **Against reading it as smaller than it is:** the two genuinely graded B2 cells are both
FAIL, the B1 loss has no INVALIDs in it at all, and the contamination is *our* harness's
defect. And ⚠️ **the Webots arm did not sweep**: on `A1_husky_swarm_10` it scored **0/10,
all FAIL**, with no OmniSim counterpart published — so that task settles nothing either way.

#### The `njmax` constraint cliff, confirmed by a second instrument 📊

OmniBench lane 4b swept driven rovers under `mujoco_warp` and found peak `nefc` tracking
**32·N exactly** — but could not make anything overflow, so it reported
**`cliff_detector_validated: false`** and refused to call a green it could not turn red a
pass. ladder0's GPU study (`omnisim/variants.py r11warp`) now has it:

- **`nefc` = 32·N exactly** — 256 at N=8, **512 at N=16** — so the 256 default runs out
  between N=8 and N=9. **The documented ~9-robot threshold is confirmed**, on a different
  scene from the 10-Husky world it was first measured on. A second instrument agreeing, not
  the same measurement repeated.
- At N=16 against the default cap it **overflows**, and the engine's own latched
  `CONSTRAINT BUFFER OVERFLOW` warning fires. `distance_worst` degrades from 0.000877 into
  the 0.0017–0.0032 range.
- ⚠️ **Never quote a single degradation number.** Two runs of the *identical* configuration
  differ by 3–7×, which is what the engine's own warning predicts — a nondeterministic GPU
  atomic race decides which constraint rows get dropped. **Quote the sign and the order of
  magnitude, nothing finer.**
- On CPU `mj_step` the cliff **does not exist** — `mj_step` sizes its own arena. So this is
  a `mujoco_warp` property, with the CPU run as a control rather than as an assumption.

---

## 2. Why the accuracy column is empty

Everything in §1 is *our* measurement of *five* engines (four until §1.6 added upstream
Webots). If you want to know how any of the
other simulators score on the same physics, there is nowhere to look. This section is why.
It is a field-wide gap, not a scoreboard.

### 2.1 The canonical cross-simulator accuracy suite is dead

**SimBenchmark** ([leggedrobotics/SimBenchmark](https://github.com/leggedrobotics/SimBenchmark),
[site](https://leggedrobotics.github.io/SimBenchmark/)) is the suite everyone still cites
for cross-engine physics accuracy. Verified by GitHub API, 2026-07-26 ✅:

- **Last push `2021-09-05`.** Five years stale, 227 stars, not archived — so it still looks
  alive from a link.
- It tested **MuJoCo 1.5**; MuJoCo is now **3.10.0** ◐ (two major versions and a complete
  solver-ecosystem turnover later).
- Its own site states it was **run by RaiSim's developers** ◐ — who win most of its rows.
- It covers **none** of: PhysX/Isaac, Newton, Genesis, Brax, SAPIEN, MJX, MJWarp, Webots,
  or Gazebo's DART.

It is not citable as current evidence about any simulator a person would choose in 2026.

### 2.2 What the vendors publish: [NOT FOUND], with one important correction

| engine | published conservation / analytic-error **data**? |
|---|---|
| Isaac Sim / PhysX 5 | **[NOT FOUND]** ◐ |
| Newton | **[NOT FOUND]** as published data ◐ — **but see the correction below** ✅ |
| Genesis | **[NOT FOUND]** ◐; a maintainer disclaims physics-matching guarantees ◐ |
| SAPIEN | **[NOT FOUND]** ◐ |
| Brax | **[NOT FOUND]** ◐ |
| MuJoCo | **[NOT FOUND]** as a cross-engine accuracy dataset ◐ |

> **Correction to the 2026-07-26 research pass, found while re-verifying ✅.** The research
> reported that Newton's test suite "contains NO energy/momentum conservation test." **That
> is false.** Newton ships
> [`newton/tests/test_physics_verification.py`](https://github.com/newton-physics/newton/blob/main/newton/tests/test_physics_verification.py)
> (© 2026, Apache-2.0, fetched 2026-07-26) — an explicitly V&V-framed suite with free fall,
> pendulum period, **Test 3: Energy Conservation**, projectile, Coulomb friction threshold,
> restitution/rebound height, **Test 6: Momentum Conservation**, conical pendulum and
> four-bar, each compared against a closed-form reference with tolerances "tied to the
> integrator order and step size." There is also a separate
> `newton/tests/determinism/test_solver_determinism.py`.
>
> The **accurate** statement is narrower and still holds: those tests **assert pass/fail
> tolerances, they do not publish measured error values**, so they produce no number you
> can put in a comparison table. And Newton's own docstring draws exactly the line this
> section is about:
>
> > *"They are not a measure of physical plausibility, real-world fidelity, or agreement
> > with another simulator; those belong in separate validation or cross-code suites."*
>
> The leading open GPU physics engine says cross-code validation belongs somewhere else —
> and there is no current somewhere else. That is the gap, stated by a vendor.

### 2.3 What real accuracy evidence exists — four studies, and they disagree

These are peer-reviewed and worth reading. Note that they **do not agree with each other**,
which is the point.

| study | what it measured | result |
|---|---|---|
| **Acosta, Yang & Posa**, *Validating Robotics Simulators on Real-World Impacts*, IROS / RA-L 2022 — [arXiv 2110.00541](https://arxiv.org/abs/2110.00541) ✅ (title/authors/venue re-fetched 2026-07-26) | Real-world cube tosses and a Cassie biped landing, against **Drake, MuJoCo, Bullet** with system-identified contact parameters | Abstract ✅: simulators "capture inelastic impacts well while failing to capture elastic impacts." Per-simulator position errors of **Drake 13.5% < Bullet 14.9% < MuJoCo 25.1%** are reported from the body ◐ — **not re-verified here**; the abstract carries no numbers. |
| **Blanco-Mulero et al.**, RA-L 2024 — [arXiv 2310.09543](https://arxiv.org/abs/2310.09543) ◐ | **Cloth** manipulation against real data | **Inverts the Acosta ranking — MuJoCo best.** ◐ |
| **Le Lidec et al.**, T-RO 2024 — [arXiv 2304.06372](https://arxiv.org/abs/2304.06372) ◐ | Solver-level analysis of contact models | Explains *why* rankings move with the contact regime ◐ |
| **Farley, Nagpal et al.**, *Simulation Modelling Practice and Theory* 2022 — [DOI 10.1016/j.simpat.2022.102629](https://doi.org/10.1016/j.simpat.2022.102629) ◐ | Real-hardware comparison — **the only one of the four covering Webots** ◐ | — |

**The inversion is the finding.** Drake wins rigid-body impacts and MuJoCo wins cloth, in
two peer-reviewed studies four years apart. **Fidelity is task-dependent and no simulator
wins everywhere.** Any document — including this one — that implies a single accuracy
ranking is wrong.

**Since 2026-08-13 OmniSim has its own row on the Acosta cube-toss data** 📊 — **24.845%**
position error, against Drake's published 13.5% (§1.6). It does not disturb the inversion
finding and it is not a good score; what it changes is that this document's accuracy column
is no longer *empty for OmniSim*. It is still empty for everyone else in §2.2.

### 2.4 Competitor published throughput — their numbers, their conditions

> ### 🚧 SEPARATOR — DO NOT COMPARE THIS TABLE WITH §1.2
>
> Everything below is **the vendor's own number on the vendor's own hardware, scene, and
> unit**. None of it was measured by us, none of it is in OmniBench's harness, and the
> units differ (**MJX/MJWarp count physics steps; §1.2 counts control steps of 8 substeps
> each**). Different robot, different GPU, different contact count. We state this table so
> readers know the landscape, not so anyone can divide one by the other.

| source | conditions | figures |
|---|---|---|
| **Isaac Lab** published table ◐ | RTX 4090, RL Games, headless. Three separate columns (step / +inference / +train). **The physics backend is never named on that page** ◐ | Cartpole-Direct 4096 envs: **1.1M / 910k / 510k**. **Velocity-Rough-G1 humanoid 4096 envs: 94k / 88k / 82k.** Repose-Cube-Shadow 8192: 200k / 190k / 170k |
| **MJWarp nightly** ◐ — *the gold standard for how to publish this*: dated, per-commit, full hardware | RTX 6000 Ada, commit `a7748dfd`, 2026-07-23 | franka **22.6M** steps/s @32768 but **0.0 mean contacts** (DeepMind's own gloss: it "measures how quickly you can simulate nothing useful happening"); humanoid **5.37M** @8192 (22.3 contacts); unitree_g1_flat **2.45M** @8192 (13.1); aloha_clutter **207k** @2048; cloth **7.4k** @32 |
| **MJX docs** ◐ | single humanoid | **650k** (M3 Max CPU) · **1.8M** (64-core CPU) · **950k** (A100, MJX-JAX @8192) · **2.7M** (TPU v5 @16384). Note a 64-core CPU beats an A100 here |
| **MuJoCo Playground** ◐ | A100, end-to-end PPO | G1 Joystick **106k** · Go1 **417k** |
| **ManiSkill 3** ◐ | RTX 4090, state-only | FrankaMove **330k** @4096 — the raw CSV row is kept at [../benchmarks/data/](../benchmarks/data/README.md) |
| **Genesis** ⚠️ | — | **43M FPS** added to the README 2024-12-18 (`73f9d2d2`) and **removed 2026-05-27** (`ca921457`) **with no explicit retraction** ◐. An independent re-run gave **0.29M** (~150× lower) ◐; Yuval Tassa called the comparison "disingenuous" ◐. The **"10–80× faster than Isaac/MJX"** claim is **still live with no stated methodology** ⚠️ |

Only one cell in this table has its raw source data kept in-tree: ManiSkill's own published
benchmark CSVs live in [`docs/benchmarks/data/`](../benchmarks/data/README.md), so the 330k
can be traced to the row it came from. Everything else here is a figure read off a vendor
page — re-fetch before requoting, and record the URL this time.

**The ~3,400× spread inside MJWarp's own nightly** — 22.6M with zero contacts down to 7.4k
on cloth — is the single best argument that a cross-simulator FPS leaderboard is
meaningless without the contact count printed next to it.

---

## 3. Identity, maintenance, licence, determinism

### 3.1 Maintenance — GitHub API, fetched 2026-07-26 ✅

Every release tag, date, and 12-month commit count in this table was fetched directly from
the GitHub API on 2026-07-26 and matched the research pass exactly.

| project | latest release | released | commits, last 12 mo | licence | note |
|---|---|---|---|---|---|
| **MuJoCo** | 3.10.0 | 2026-06-22 | **1948** | Apache-2.0 | the reference engine |
| **Newton** | v1.4.0 | 2026-07-16 | **1586** | Apache-2.0 | Linux Foundation |
| **Drake** | v1.55.0 | 2026-07-15 | **1259** | BSD-3 ◐ | |
| **Isaac Lab** | v3.0.0-beta2.patch1 | 2026-07-02 | **1097** | BSD-3-Clause ✅ | **still self-described beta** |
| **Genesis** | v1.2.3 | 2026-07-18 | **952** | Apache-2.0 | 29,639 stars ✅ |
| **mujoco_warp** | 3.10.0.3 | 2026-07-22 | **922** | Apache-2.0 | |
| **O3DE** | 2605.0 | 2026-05-27 | **434** | Apache-2.0 ◐ | |
| **Gazebo (gz-sim)** | 10.0.0 (Jetty) | 2025-10-14 | **260** | Apache-2.0 | LTS to May 2031 ✅ |
| **Brax** | v0.14.2 | 2026-03-15 | **72** | Apache-2.0 ◐ | |
| **ManiSkill** | v3.0.1 | 2026-04-21 | **67** | Apache-2.0 code, **CC BY-NC assets** ✅ | |
| **Upstream Webots** | **R2025a** | **2025-02-04** | **48** | Apache-2.0 | ⚠️ see decay curve below |
| **SAPIEN** | 3.0.3 | 2026-03-10 | **15** | **inconsistent** ✅ — see §3.2 | |
| **Isaac Sim** | v6.0.1 | 2026-06-22 | **12** | NOASSERTION ✅ — see §3.2 | release-drop only; **contributions not accepted** ✅ |
| **PyBullet (bullet3)** | 3.25 | **2022-04-24** | **1** | Zlib | **dead** |
| **Unity-Robotics-Hub** | v0.7.0 (2022-02-03) | — | **0** | — | **dormant** |
| **AirSim** | v1.8.1 (2022-07-18) | — | **2** | — | **dead** (shut down July 2022; Colosseum is the fork) |
| **OmniSim** | — | — | — | Apache-2.0 ⊘ | this checkout |

> ### ⚠️ Upstream Webots is decaying, and OmniSim inherits that
>
> Commits per calendar year, `cyberbotics/webots`, GitHub API 2026-07-26 ✅:
>
> **2022: 1027 → 2023: 608 → 2024: 221 → 2025: 53 → 2026 YTD: 36**
>
> That is a **~28× decline in four years**, and the last tagged release is **R2025a,
> 2025-02-04** — approaching eighteen months old. The 2026-07-10 edition of this document
> called Webots "Active ✅" on the strength of that release tag. The commit curve says
> something more specific: *maintained, not developing.* This cuts both ways for OmniSim —
> the upstream we merge from is quiet, so divergence is cheap, but so is the bug-fix flow
> we get for free.

### 3.2 Licence nuance that actually matters ✅

**Isaac Sim's "Apache-2.0" is incomplete.** Verified by direct fetch of the repo LICENSE
and the GitHub API, 2026-07-26 ✅:

- GitHub classifies the repo **`NOASSERTION` / "Other"**, not Apache-2.0 ✅.
- The LICENSE file's own preamble, verbatim ✅: *"Building or using the software requires
  additional components licenced under other terms. These additional components include
  dependencies such as the Omniverse Kit SDK, as well as 3D models and textures."*
- Those components sit under the
  [NVIDIA Isaac Sim Additional Software and Materials License](https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/);
  the research reports that **redistribution to third parties requires an NVIDIA AI
  Enterprise licence** ◐ — *the linked agreement was not re-fetched in this pass*.
- `CONTRIBUTING.md`, verbatim ✅: *"We currently do not acccept any contributions to this
  repository."* (typo in the original).

**Isaac Lab is genuinely BSD-3-Clause** ✅ (GitHub API). Do not carry Isaac Sim's caveat
across to it.

**SAPIEN's licence is inconsistent across three sources** ✅: the in-repo `LICENSE` file is
**Apache-2.0** (Hillbot Inc. / UCSD SU Lab) ✅, the GitHub API classifies the repo
**NOASSERTION** ✅, and PyPI reportedly says **MIT** ◐ *(PyPI not re-fetched)*. If you are
shipping a product on SAPIEN, get this in writing.

**CoppeliaSim remains the outlier**: free **Edu** edition restricted to students/teachers/
professors of schools and universities — excluding companies, research institutions,
non-profits and foundations alike — and not usable for any commercial purpose; Pro/Lite
pricing is not published ✅ (2026-07-10 fetch, not re-verified this pass ◐).

### 3.3 Determinism — a row worth having, but it must be compared like with like

> **⚠️ Correction to our own earlier framing (2026-07-26).** The previous edition put a
> single "OmniSim: **Bitwise**" cell next to MJWarp's "No" and Isaac's "same hardware only,"
> and read that as a win. **That comparison was invalid**: our bitwise grade came from ODE
> and from Newton-under-XPBD on a light scene, and we were setting it against competitors'
> **GPU** answers. Measured on our own GPU `mujoco_warp` path we land **exactly where MJWarp
> says it lands** — not reproducible — and for the same mechanism (atomic contact-slot
> ordering). The row is now split by configuration so each cell has a like-for-like
> counterpart. What survives as a differentiator is at the bottom of this section, and it is
> not the grade.

| project / configuration | published determinism claim | evidence |
|---|---|---|
| **OmniSim / ODE** (CPU) | **Bitwise** — `cold_cold` *and* `cold_warm`, 400 steps, `max_abs_dev = 0.0` on the light five-sphere world (the only world covered on three machines); separately bitwise on a **80/320**-contact ten-robot ring, one machine. ⚠️ The 336-contact scene is the `newtonSolver "mujoco"` row below, **not** ODE | 📊 §1.3 + [scope](../benchmarks/determinism-scope.md) §1 |
| **OmniSim / Newton `newtonSolver "mujoco"`** (CPU `mj_step`) | **Bitwise, 5/5** — including a 336-contact / 1344-constraint-row scene with ten live controllers, and a 64-box collapsing pile | 📊 [scope](../benchmarks/determinism-scope.md) §1 |
| **OmniSim / Newton XPBD** (warp kernels) | **Bitwise** on the one light-contact sphere-drop world — 10/10 rows across 3 machines / 2 OSes / 2 compilers / 2 GPU models. ⚠️ **Untested at contact density**, and XPBD dispatches warp kernels, so do not read it as a GPU guarantee | 📊 §1.3 |
| **OmniSim / Newton `newtonSolver "mujoco_warp"`** (GPU) | **Not reproducible — 0 bitwise of 24** same-config cold pairs across six scenes; 9.152 m deviation at 1000 steps. **Same position as MJWarp below, same cause** (`wp.atomic_add` contact-slot claiming) | 📊 [scope](../benchmarks/determinism-scope.md) §2 |
| **OmniSim, cross-machine** | **Untested.** Every row above is one machine reproducing *itself*; no run compares trajectories *between* machines. What we do have cross-machine is lane-1 *metric* digit-identity to 15–16 s.f. on the CPU `mj_step` path (§1.1) — a weaker statement than trajectory bitwise identity | 📊 §1.1 |
| **Isaac Lab** | Identical results **only on the same hardware + same version**; explicitly **no determinism for non-rigid**; `enable_enhanced_determinism` defaults **False** | ◐ |
| **Newton** | Contact **order** is nondeterministic unless `deterministic=True` (**default off**); **no cross-hardware claim**. Ships `tests/determinism/test_solver_determinism.py` ✅ | ◐ / ✅ |
| **MuJoCo (C)** | Deterministic within one version + architecture | ◐ |
| **MJWarp (GPU)** | Its FAQ answers **"No"** outright and recommends CPU for deterministic results | ◐ |
| **MJX-JAX** | **[NOT FOUND]** | — |
| **Drake** | Architecturally principled — all randomness routed through a random input port — but **scoped to same binary + same inputs**; no cross-machine claim | ◐ |
| **PyBullet** | Maintainer: *"only deterministic on 1 platform/compiler"* | ◐ |
| **Upstream Webots** | Reproducible under stated conditions, and **cross-machine if no OpenGL-dependent sensors** ◐. **📊 Measured here, not merely claimed:** `repeat_delta` = **0.0** on ladder0 rung 9 (R2025a/ODE under WSL2) — the same grade our CPU `mj_step` row gets, on the same scene | ◐ / 📊 §1.6 |
| **Genesis / Brax** | **[NOT FOUND]** | — |
| **SAPIEN** | Explicitly **disclaims** trajectory determinism; warns that changing `num_envs` can change every env | ◐ |

**What actually differentiates us on this axis, stated precisely.** Not the grade. On the
GPU path we are level with MJWarp, and MuJoCo's CPU guarantee ("same version + same
architecture") is the same shape as ours — arguably better-established, since it is a
maintained guarantee rather than one campaign. What is genuinely unusual is that **we
publish the per-configuration scope**: which solver, which scene, which contact count, how
far it diverges when it diverges, and the mechanism
([determinism-scope.md](../benchmarks/determinism-scope.md)). Every project in the table
above states a *policy* — yes / no / same-hardware-only; none of them publishes the measured
divergence magnitude per configuration, and none publishes the configuration where their own
answer flips. We also published the **false positive** that briefly inflated our own claim
and the three harness defects behind it (§4 of the scope doc). That is a documentation
practice, not a physics result — and it is the honest version of this row.

Two things not to say: *"OmniSim is deterministic and Isaac/MJWarp aren't"* (false — compare
the same class of solver), and *"our determinism is cross-machine"* (untested).

---

## 4. The capability matrix

Competitor cells carry markers and are sourced in §6. **OmniSim cells are ⊘ self-attested**
unless marked 📊.

| | **OmniSim** | Isaac Sim / Isaac Lab | Gazebo (gz-sim) | MuJoCo (+MJX/MJWarp) | Upstream Webots | Genesis | ManiSkill 3 / SAPIEN | PyBullet | CoppeliaSim | Unity / O3DE |
|---|---|---|---|---|---|---|---|---|---|---|
| **Licence** | Apache-2.0 ⊘ | **NOASSERTION** ✅ (Sim — see §3.2) · BSD-3 ✅ (Lab) | Apache-2.0 | Apache-2.0 | Apache-2.0 | Apache-2.0 ✅ | Apache-2.0 code, **CC BY-NC assets** ✅; SAPIEN inconsistent ✅ | Zlib | **Paid commercial** ✅ | Proprietary / Apache-2.0 (O3DE) ◐ |
| **Cost to ship a product** | Free ⊘ | Free code; **3rd-party redistribution needs AI Enterprise** ◐ | Free | Free | Free | Free | Free code; assets non-commercial ✅ | Free | **Contact vendor** ✅ | Unity licence / free |
| **Default physics** | Newton, **exclusively** — CPU `mj_step` default, `mujoco_warp` opt-in. **No fallback**: ODE deleted `bdc02139`, XPBD removed `94f04222` (2026-08-08) ⊘ | PhysX 5; Newton **experimental** ✅ | DART; Bullet pluggable ◐ | MuJoCo | ODE | multi-solver (rigid/FEM/MPM/PBD) ◐ | SAPIEN/PhysX | Bullet | Bullet/ODE/Vortex/Newton | PhysX / Chaos |
| **GPU physics** | Yes (Warp) ⊘ | Yes | No | Yes (MJX/MJWarp) | No | Yes ◐ | Yes ◐ | No | No | No |
| **Batched RL in-engine** | Yes 📊 (§1.2) | Yes (Isaac Lab) | No | Yes | No | Yes ◐ | Yes ◐ | No | No | CPU-parallel only |
| **Hardware floor** | Laptop GPU; **CPU-only via CPU `mj_step`** 📊 — measured with every CUDA device hidden from the process (OmniBench lane 4c). ⚠ The old wording said "via ODE", a mechanism deleted in `bdc02139`; and this is **not** a GPU-less-*hardware* result — the box still has a driver, a CUDA runtime and the GPU wheels. Say "runs with no CUDA device visible to the process". Verdict + numbers: [lane4-capability-matrix.md](../benchmarks/lane4-capability-matrix.md) | **RTX 4080 + 16 GB VRAM** ✅; A100/H100 **unsupported** ✅ | CPU | Laptop GPU (CPU MuJoCo fine) | CPU | CUDA / ROCm / Metal ✅ | NVIDIA GPU | CPU | CPU | Mid GPU |
| **Photoreal rendering** | No (WREN; wgpu opt-in) ⊘ | **Yes — RTX path tracing** | Limited | None built-in | Limited | Ray tracer (Luisa) ◐ | Rasterized, GPU-parallel ◐ | Minimal | Limited | **Yes** |
| **Native URDF import** | Yes (`URDFRobot`) ⊘ | Yes | Yes (via SDF) | Yes | Yes | Yes | Yes | Yes | Yes | Yes ◐ |
| **First-party ROS 2** | **Yes — sidecar** (`packages/omnisim-ros2/`); **no `ros2_control`** ⊘ | Bridge ✅; `ros2_control` *community* ✅ | **Yes — ros-controls hosted** ✅ | `ros2_control` **hosted** ✅ | `webots_ros2` (community) ✅ | No ✅ | No ✅ | No ✅ | No ✅ | Community-grade ◐ |
| **ROS 2 `simulation_interfaces`** ⬅ *corrected row* | **Yes — sidecar, not native** (15 svc + 1 action, v2.1.0) ⊘ | **Yes — native** (`isaacsim.ros2.sim_control`, 19 services + 1 action) ◐ | **Yes — native** (`ros_gz/src/gz_simulation_interfaces/`) ◐ | No ◐ | No ◐ | No ◐ | No ◐ | No ◐ | No ◐ | **Yes — O3DE** (`Gems/SimulationInterfaces`) ◐ |
| **Other remote control surface** ⬅ *corrected row* | HTTP/JSON harness + capture + bridges ⊘ 📊 (10/10 driveability) | Omniverse Kit `omni.services.transport.server.http` ◐ | — | MuJoCo MPC **gRPC agent server** ◐ | `--stream` + extern controllers on **TCP:1234** ◐ | — | — | `connect()` over **SHARED_MEMORY / UDP / TCP / gRPC** ◐ | vendor remote API ◐ | — |
| **Transport / dependency of that surface** ⬅ *the actual differentiator* | **plain HTTP + JSON in the ENGINE. No ROS, no DDS, no in-process Python, no editor plugin** — ROS 2 is an optional sidecar over that same HTTP ⊘ | ROS 2 + DDS, **or** an in-process Kit extension | **ROS 2 + DDS** | in-process C/Python, or MPC's gRPC | own TCP protocol | — | — | in-process Python or Bullet's own protocol | vendor protocol | ROS 2 + DDS |
| **First-party MCP server** | **Yes** ([`packages/omnisim-mcp/`](../../packages/omnisim-mcp/)) ⊘ | third-party ◐ | third-party ◐ | No ◐ | No ◐ | No ◐ | No ◐ | No ◐ | No ◐ | No ◐ |
| **Windows** | **Primary** ⊘ | Supported ✅ (Win 11) | Weak | Yes | Yes ✅ | Yes ✅ | — | Yes | Yes | Yes |
| **Maintenance** | Active ⊘ | Sim **12 commits/12 mo** ✅ (release drops, contributions closed); Lab 1097 ✅ | 260 ✅ | 1948 ✅ | **48 ✅ — decaying** | 952 ✅ | ManiSkill 67 ✅ / SAPIEN 15 ✅ | **1 ✅ — dead** | Active ◐ | Unity **0 ✅ — dormant**; O3DE 434 ✅ |
| **Run-to-run determinism** ⬅ *corrected row — was "Cross-machine determinism: Bitwise"* | **Bitwise on CPU `mj_step`** (the default; verified at 336 contacts with ten live controllers); **GPU `mujoco_warp` not reproducible** (0 bitwise of 24 same-config cold pairs); **cross-machine untested** 📊 (§3.3, [scope](../benchmarks/determinism-scope.md)). ⚠ This row previously read "bitwise on ODE and CPU `mj_step`", with XPBD bitwise on one light scene — **both of those configurations were deleted** (`bdc02139`, `94f04222`) and are now unverifiable, which also means the CPU result no longer has an independent second backend corroborating it | same HW + version only ◐ | — | CPU only; MJWarp GPU says "No" ◐ | if no GL sensors ◐ | [NOT FOUND] | disclaimed ◐ | 1 platform only ◐ | — | — |
| **Physics accuracy vs analytic ground truth** | 📊 §1.1 (ODE + Newton) + §1.6 (ladder0) | **not measured**¹ | **not measured**¹ | 📊 §1.1 (MuJoCo 3.8.1) + §1.6 | 📊 **§1.6 — ladder0 rungs 9 and 11, both PASS** ⬅ *corrected cell* | **not measured**¹ | **not measured**¹ | 📊 §1.1 | **not measured**¹ | **not measured**¹ |
| **Accuracy vs recorded real-world impacts** ⬅ *new row* | 📊 **24.845%** pos / **23.104°** rot on 50 real cube tosses (§1.6) — **mid-field, and worse than Drake's published 13.5%** | [NOT FOUND] ◐ | [NOT FOUND] ◐ | 📊 24.845% / 23.104° (our arm); **25.1 ± 10.8%** published ◐ | *arm declared `UNIMPLEMENTED` on this rung* | [NOT FOUND] ◐ | [NOT FOUND] ◐ | **14.9 ± 8.9%** published ◐ | [NOT FOUND] ◐ | [NOT FOUND] ◐ |

¹ *"Not measured" here means **we do not run this engine** (§1.4) — not that it scored
badly, and not that anyone else has measured it either (§2).* ⚠️ **Upstream Webots no
longer belongs under this footnote and did until 2026-08-16** — we do run it, on two
instruments (§1.4, §1.6). Drake has no column here; its published impact-accuracy numbers
are in §1.6 and §2.3, and it is the engine to beat on that axis.

---

## 5. Where OmniSim is actually differentiated

Three claims survive an adversarial reading. **One of them is narrower than the previous
edition said.**

### 5.1 The agent surface — corrected: it is the transport, not the existence

> **⚠️ Correction to our own 2026-07-10 claim.** That edition said *"No established
> simulator ships a first-party HTTP surface built for the coding-agent iteration loop"*
> and framed competitors as having *"third-party MCP only."* **The first half was too
> strong and the second half was the wrong axis.** Competitors have real, first-party,
> programmatic scene-control surfaces — including a **ROS 2 standard** for exactly this.

**[`ros-simulation/simulation_interfaces`](https://github.com/ros-simulation/simulation_interfaces)**
(Apache-2.0 ✅, re-fetched 2026-07-26) standardizes the scene-control verbs an agent needs:
`LoadWorld` ✅, `UnloadWorld` ✅, `GetCurrentWorld` ✅, `GetAvailableWorlds` ✅,
`GetSimulatorFeatures` ✅, `GetSpawnables` ✅, `GetEntityBounds` ✅, `GetNamedPoses` ✅,
`SetEntityInfo` ✅, plus `Get`/`SetSimulationState`, `StepSimulation`, `ResetSimulation`,
`SpawnEntity`, `DeleteEntity`, `GetEntities`, `Get`/`SetEntityState` ◐ *(the last group was
not visible in the re-fetched page listing; taken from the research pass)*.

Implemented **natively** by ◐: **Gazebo** (`ros_gz/src/gz_simulation_interfaces/`),
**Isaac Sim** (`isaacsim.ros2.sim_control`, 19 services + 1 action), **O3DE**
(`Gems/SimulationInterfaces`). **Not** implemented by Webots, MuJoCo, Drake, PyBullet,
Genesis, Brax, or SAPIEN ◐.

Other genuine remote surfaces ◐: PyBullet's `connect()` over SHARED_MEMORY / UDP / TCP /
gRPC; Omniverse Kit's `omni.services.transport.server.http`; Webots' `--stream` plus extern
controllers on TCP:1234; MuJoCo MPC's gRPC agent server. (Drake's Meshcat is http+ws but
serves **human widgets** — visualization, not machine control ◐.)

**So what is left, stated precisely:**

- **The transport is plain HTTP + JSON**, and **the dependency is nothing.** Driving
  Gazebo's or Isaac's implementation of the standard means standing up ROS 2 and a DDS
  participant, or loading an in-process Kit extension. Driving OmniSim means `curl`. For a
  coding agent in a sandbox with no ROS install, that difference is the whole game.
- **The surface is specified and versioned as a contract**, not as a bag of services:
  [PROTOCOL.md](../../PROTOCOL.md) defines four independently-versioned surfaces (Robot
  Bridge `:8765`, World Harness `:6789`, Capture `:6791`, Twin Shadow) with **structured
  load diagnostics** (`PROTO_NAME_MISMATCH`, `WORLD_PARSE_SYNTAX_ERROR`) an agent branches on
  instead of regex-matching stderr, and a unified `/sim/events` cursor stream.
- **It is measured, not asserted** 📊: 10/10 on OmniBench's driveability lane (§1.3) —
  including hot-reload with the edit visibly applied, verified camera framing with a
  numeric in-frame proof, and a broken world returning structured codes and **never
  falsely healthy**. Two honest findings from those same rows: `/sim/reset` rewinds time
  but does **not** restore node state, and the broken-world probe takes **243 s**.
- **A first-party MCP server** ([`packages/omnisim-mcp/`](../../packages/omnisim-mcp/))
  exposes it to Claude Desktop, Cursor, and any MCP client. Third-party MCP servers exist
  for Isaac and Gazebo ◐ — that is not the point, and we should stop making it the point.

### 5.2 Newton as the shipped default, not an experiment

Verified ✅: Newton is Apache-2.0, co-developed by **NVIDIA, Google DeepMind and Disney
Research**, contributed to the **Linux Foundation** for vendor-neutral governance; built on
**NVIDIA Warp + OpenUSD**, differentiable, multi-solver with **MuJoCo Warp as its key
solver**; **v1.4.0 shipped 2026-07-16 with 1586 commits in the last 12 months** ✅.

Verified ✅: **Isaac Lab's Newton integration is explicitly experimental** — develop branch,
Isaac Lab **3.0 still self-described beta** ✅, with breaking-change and no-production-support
warnings. Isaac Lab's *production* physics remains PhysX 5.

The precise claim: **OmniSim runs Newton as its default in-engine backend today, while the
flagship commercial stack still carries Newton as an experimental option.** That is a
statement about *default posture*, not maturity. PhysX 5 is battle-tested; Newton is
eighteen months old; and OmniBench found **five bugs in our own Newton integration layer**
(§1.5). Do not upgrade this into "OmniSim's physics is more mature than Isaac's."

### 5.3 Train == deploy, bit-exact, and now measured

OmniSim's trainer and its Newton deploy runtime derive their physical model from one source
of truth — [`g1_physics.json`](../../projects/policies/research/backends/g1_physics.json)
+ `g1_physics_spec.py` + the prim URDF — enforced in CI by
[`tests/test_g1_physics_spec_conformance.py`](../../tests/test_g1_physics_spec_conformance.py).
See [g1-single-source-of-truth.md](g1-single-source-of-truth.md).

**What is new since 2026-07-10 is that the claim now has a number** 📊: OmniBench lane 3b
scores the structural parity at **1 real physics gap** on the deploy default and **0** with
`OMNISIM_NEWTON_USE_LINK_COM=1` (§1.3).

**Do not add "and the simulation itself is bitwise-deterministic" to this claim** — an
earlier edition did, and it does not hold for the configuration the G1 actually trains and
deploys in. Lane 3a's bitwise grade is ODE and Newton/XPBD on a light-contact world; the
GPU `mujoco_warp` solver the batched trainer runs is **not** run-to-run reproducible
(§3.3, [determinism-scope.md](../benchmarks/determinism-scope.md)). Train == deploy here
means **one physical model and one solver**, which is what the parity score measures — it
has never meant identical trajectories, and the repo's own
[closed-loop-chaos-diagnostic.md](closed-loop-chaos-diagnostic.md) exists precisely because
they diverge.

**That is still not sim-to-real.** The field's bar is **MuJoCo Playground** ✅: quadruped
joystick locomotion in ~5 min, Unitree G1 walking in under 30 min on 2× RTX 4090, and
zero-shot transfer to **six** physical platforms — Go1, G1, Berkeley Humanoid, Booster T1,
LEAP Hand, Franka. **OmniSim has not cleared it.** Our sim-to-real story is
sim-to-*deploy*, in-engine. The canonical, unflattering status is
[rl-current-state.md](rl-current-state.md).

---

## 6. Per-simulator notes

### Isaac Sim / Isaac Lab — the photoreal, high-floor incumbent
The **RTX renderer** is genuinely photoreal and the reason Isaac wins perception and
synthetic-data work outright. The cost is a steep, verified hardware floor: minimum
**RTX 4080**, **16 GB VRAM** (48 GB "ideal") ✅, and — counterintuitively — **GPUs without
RT cores (A100, H100) are not supported** ✅. A datacenter card you would train on cannot
run the simulator.

Two things to get right: the **licence is not plain Apache-2.0** (§3.2) ✅, and the *Sim*
repo took **12 commits in 12 months** ✅ with **contributions not accepted** ✅ — it is a
release-drop, not a collaborative repo. **Isaac Lab is the live project** (1097 commits ✅)
and is genuinely BSD-3 ✅, still tagged **beta** ✅. It implements ROS 2
`simulation_interfaces` natively ◐ (§5.1).

### Gazebo — the ROS default
Apache-2.0, CPU physics, DART default with Bullet pluggable ◐. **Jetty 10.0.0 (2025-10-14)
is LTS to May 2031** ✅; Ionic is *not* LTS and EOLs Dec 2026 ✅. 260 commits/12 mo ✅.

Its moat is ROS, and it just got wider: `ros2_control` is **hosted first-party by the
ros-controls organization** ✅, and Gazebo implements **ROS 2 `simulation_interfaces`
natively** ◐ — the standardized scene-control surface (§5.1). For a ROS-centric lab this is
the correct choice and we should say so.

### Upstream Webots — OmniSim's parent
Apache-2.0. **R2025a, 2025-02-04** ✅ is the last tagged release, and the commit curve is
**1027 → 608 → 221 → 53 → 36 YTD** ✅ (§3.1). It has `--stream` and extern controllers on
TCP:1234 ◐ but does **not** implement `simulation_interfaces` ◐. It is the honest baseline
for what OmniSim inherited — broad robot/sensor/world library, CPU ODE physics,
competent-but-not-photoreal rendering — and it is the only project in §3.1 whose activity is
in structural decline.

### MuJoCo / MJX / MJWarp — the RL physics reference
Apache-2.0, **3.10.0** ✅, **1948 commits/12 mo** ✅ — the most active project in the table.
**MJWarp requires an NVIDIA GPU** for fast simulation ✅; Warp targets CUDA, so there is no
AMD path. MJWarp's **nightly benchmark page is the best publishing practice in this
document** ◐ — dated, per-commit, full hardware, and it prints the **mean contact count**
next to every figure, which is what makes the numbers interpretable (§2.4).

MuJoCo is not a *robot simulator* in the Webots/Gazebo sense — no world editor, no sensor
zoo, no GUI scenario authoring. It is a physics engine plus an ecosystem. In OmniBench's
lane 1 it scored mid-field: T3 rolling accel within **0.10%** of analytic and T2 slide
accel within **0.62%**, but worst-of-four on T1 restitution (0.1478) and T7 conservation
(6.25e-03, two orders above ODE and PyBullet) 📊 — and its **fixed
dt=1 ms solref calibration goes unstable at dt ≥ 8 ms** (T1 RMSE 35.97 / 33.52 / 396.5 at
8/16/32 ms) 📊, which is the honest consequence of freezing a calibration, not a MuJoCo
defect.

### Drake — the accuracy-literate outlier this table used to omit
BSD-3 ◐, **v1.55.0 (2026-07-15), 1259 commits/12 mo** ✅. Not an agent-authoring tool and
not a batched-RL engine, but it is the simulator with the strongest *published* real-world
impact-accuracy result (Acosta et al., §2.3) and the most architecturally principled
determinism story (all randomness through a random input port ◐). Its Meshcat surface is
http+ws but serves human widgets, not machine control ◐. If your question is "which
simulator has been checked against physical impacts," Drake is the answer, and it is not us.

### Genesis — the cautionary tale, now a real project
Apache-2.0 ✅, **29,639 stars** ✅, genuinely active (**v1.2.3, 2026-07-18; 952 commits**) ✅,
unifying rigid/FEM/MPM/PBD with three render paths ◐, running on CUDA, ROCm **and Apple
Metal** ✅ — the broadest GPU-vendor support here.

Its Dec-2024 launch claimed **43M FPS / 430,000× real-time** ⚠️. The claim was added to the
README 2024-12-18 and **removed 2026-05-27 with no explicit retraction** ◐; an independent
re-run measured ~**0.29M** (~150× lower) ◐; Yuval Tassa called the comparison
"disingenuous" ◐. **The "10–80× faster than Isaac/MJX" claim is still live with no stated
methodology** ⚠️. A maintainer separately disclaims physics-matching guarantees ◐.

The lesson OmniSim takes is procedural, not competitive: publish measured numbers with the
hardware and the contact count attached, or do not publish them.

### ManiSkill 3 / SAPIEN — the manipulation specialist
ManiSkill **v3.0.1** ✅ (67 commits ✅), SAPIEN **3.0.3** ✅ (15 commits ✅). GPU-parallelizes
**both simulation and rendering**; published **FrankaMove 330k FPS @4096 state-only on an
RTX 4090** ◐. Heterogeneous parallel scenes (each env a different scene) ◐ is a capability
OmniSim does not have. **Licence traps, plural:** assets are **CC BY-NC 4.0 —
non-commercial** ✅, and SAPIEN's own licence is inconsistent across three sources ✅
(§3.2). SAPIEN explicitly **disclaims** trajectory determinism ◐.

### PyBullet — dead, and it should be labelled that way
Zlib. Last release **3.25, 2022-04-24** ✅ — over four years — and **1 commit in the last 12
months** ✅. The 2026-07-10 edition called it "maintenance mode"; the API says dead. It is
nonetheless **an excellent CPU rigid-body engine and it scored best-in-suite on two
OmniBench metrics** 📊 (T3 rolling accel at 2e-15, and T7 conservation — with the
near-steady-spin caveat). Widely cited, still correct, not where new work goes. Its
`connect()` supports SHARED_MEMORY / UDP / TCP / gRPC ◐ — a real remote surface, older than
most of the ones marketed today.

### CoppeliaSim — the licence outlier
Three editions — **Edu / Pro / Lite** ✅. Free Edu is restricted to students/teachers/
professors of schools and universities, **excludes companies, research institutions,
non-profits and foundations alike** ✅, and **cannot be used for any commercial purpose** ✅.
Pro/Lite pricing is not published ✅. Everything else here is free to ship with (modulo
ManiSkill's assets). *(Fetched 2026-07-10; not re-verified this pass.)*

### Unity Robotics / O3DE / Colosseum — the dormant lane, with one exception
**Unity-Robotics-Hub: 0 commits in 12 months** ✅, last tag v0.7.0 (2022-02-03) ✅. Community
members regarded it as abandoned as early as 2024 ◐; a self-identified former Unity
robotics team member stated in Jan 2025 that the team was laid off years earlier ◐.
**AirSim: 2 commits** ✅, shut down July 2022; **Colosseum** is the MIT community fork ◐,
newest tag v2.1.0 (June 2023) ◐, so current use means building from main.

**The exception is O3DE** — **2605.0 (2026-05-27), 434 commits/12 mo** ✅ — which implements
ROS 2 `simulation_interfaces` natively ◐. It is the live open game-engine robotics lane;
Unity's is not.

---

## 7. Where OmniSim loses

State these before anyone else does. **Three of these are new or sharpened by the 2026-07
research and OmniBench.**

- **On the one real-hardware dataset we have run, we are mid-field and worse than Drake.**
  ⚠️ **This bullet read "No real-hardware validation study … OmniSim appears in none … not
  one number has been checked against a physical robot" until 2026-08-16. That was true on
  2026-07-26 and has been false since 2026-08-13** — ladder0 rung 18 replays 50 recorded
  real cube tosses and scores **24.845%** of a cube width (§1.6). The loss is what the
  number *says*: **Drake manages 13.5% and Bullet 14.9% on the same data** (Acosta et al.,
  RA-L 2022 ✅ / ◐), so we are behind both, and it is **one dataset, one object class,
  tossed rigid cubes**, replayed by us in our own harness on one machine. We still appear
  in **none** of the four published studies (§2.3), no third party has scored OmniSim on
  anything, and **OmniBench itself still has no real-hardware lane**. What changed is that
  the gap is now quantified instead of total.
- **Our GPU path shipped five integration bugs and we only found them by building the
  bench. 🆕** Until 2026-07-25 the Newton backend ignored `WorldInfo.gravity` entirely (so
  **every Newton world ran at −9.81** regardless of configuration), silently applied a
  Husky-wheel inertia preset to any Solid without an explicit `inertiaMatrix`, ran a
  pyramidal friction cone at impratio 1, let `OMNISIM_FORCE_ODE` be bypassed, and dropped
  t=0 `setVelocity`. All fixed and cross-machine validated (§1.5) — but they were live in
  shipped binaries for the whole period this document previously described Newton as our
  advantage. Assume there are more.
- **Our throughput is measured on two machines; vendors measure on fleets. 🆕** §1.2 is one
  laptop 3060 and one RunPod 4090. Isaac Lab, MJWarp and MJX publish across CPU/A100/TPU/
  RTX-6000-Ada matrices. ⚠️ **This bullet said "we do not run any competitor's engine" until
  2026-08-16 and that is wrong** — we run **upstream Webots** (§1.6), on ladder0 and on
  AgentBench. It is a weak correction though: upstream is our own *parent*, not a rival
  stack; it is measured on physics correctness and agent tasks, **never on throughput**; and
  the comparison is cross-platform (WSL2 vs native Windows). The other three arms in §1.1
  are engines we embed or import. **No head-to-head against Isaac, Gazebo, Genesis, Drake or
  SAPIEN exists in any form**, and where Webots *is* head-to-head it beats us on one
  AgentBench task 4/5 to 0/10.
- **ROS 2 support is new, and its arm half is missing.** ⚠️ This bullet used to read "No ROS 2
  bridge — and now also no `simulation_interfaces`", then "and `ros2_control` is missing". Both were
  overtaken: OmniSim implements the `simulation_interfaces` standard ◐ (15 services + the
  `SimulateSteps` action) plus `/clock`, `/tf`, `JointState`, `/odom`, `cmd_vel`, sensor topics
  **and** a `hardware_interface::SystemInterface` — all as a **sidecar** over the existing HTTP
  surface, with the engine itself still ROS-free. What remains a genuine loss: `ros2_control` is
  verified only for a **velocity-commanded base** (`diff_drive_controller` on the Husky), and
  **MoveIt is still out of reach** ✅ — measured, the arm bridge answers `409 busy` to a joint
  setpoint arriving while the previous one is still interpolating, so a trajectory would land in
  pieces; **no Nav2 stack has ever been brought up against OmniSim** ✅; OmniSim still does not
  appear in the `ros2_control` simulator registry ✅; and the implementation is a sidecar, not
  native, so it is one process hop and one HTTP round trip away from the engine — measured, that
  round trip caps a control loop near **45 Hz**, because a joint-state read is a supervisor RPC
  serviced at an engine step boundary (21.01 ms, against 4.48 ms for a bare HTTP GET on the same
  server). **For a lab whose stack is ROS 2 end to end, Gazebo remains better integrated.**
  Detail and limitations: [ros2-integration.md](ros2-integration.md).
- **Not photoreal.** Isaac's RTX renderer and UE5's Nanite/Lumen are a different visual
  class. Default main view is still WREN (OpenGL); wgpu is opt-in per world
  ([wgpu-renderer-status.md](wgpu-renderer-status.md)).
- **Sim-to-real is unproven.** Sim-to-*deploy* parity in-engine 📊 is not zero-shot transfer
  to six physical robots ✅ (§5.3).
- **Young fork, narrow shoulders.** MuJoCo took 1948 commits last year, Newton 1586, Drake
  1259, Isaac Lab 1097 ✅. The Newton migration is incomplete
  ([engine-migration-plan.md](engine-migration-plan.md)) and per-robot locomotion maturity
  ranges from solid to open research ([rl-current-state.md](rl-current-state.md)).
- **NVIDIA-shaped GPU path.** Warp targets CUDA. Genesis's ROCm/Metal support ✅ is broader.

**The counterweight, and it is real:** **Isaac Sim will not start without an RT-core GPU**
✅ — no A100, no H100, RTX 4080 floor, 16 GB VRAM ✅. OmniSim runs on a CPU-only box —
**⚠ since `bdc02139` via CPU `SolverMuJoCo` (`mj_step`), not via ODE, and that path has
NOT been verified on a genuinely GPU-less machine** (it was campaign item A1, which never
ran) — and reaches the GPU-batched tier on a **laptop RTX 3060 at 129,431 control
env-steps/s @4096** 📊 (§1.2). Accessibility is a measured axis and it is where OmniSim's
advantage is least contestable, but **the CPU half of it is currently an argument, not a
measurement.** Verify before quoting it externally; it is also the mechanism named in an
external funding deliverable.

**And a second counterweight worth stating plainly:** OmniBench's best-scoring *integration*
is **OmniSim/ODE** 📊 (⚠️ an integration result, not a solver result — bare MuJoCo scored fine
on the same scenes and the deficit was ours, [correctness-scope.md](../benchmarks/correctness-scope.md)) — best-in-suite on T1, T2, T4, T6
and T7, linear momentum exactly zero to double precision on T5, and **bitwise reproducible
run-to-run** — on the light five-sphere
world across three machines, and on a 10-robot ring at 80/320 contacts on one machine.
(The **336**-contact ten-robot scene is the CPU `newtonSolver "mujoco"` row, not ODE;
[scope](../benchmarks/determinism-scope.md) §1.) The CPU fallback we
describe as an accessibility concession is also the most accurate *and* the most reproducible
backend we ship — which is the same trade MuJoCo's own docs describe, and a real reason to
pick ODE (or `newtonSolver "mujoco"`) over the GPU path when a run has to be replayable.

---

## 8. The one-paragraph positioning

> OmniSim is an Apache-2.0 robotics simulator that inherits Webots' breadth of robots,
> sensors and worlds, and adds three things the incumbents do not combine: it **defaults to
> Newton** (the Linux-Foundation, NVIDIA/DeepMind/Disney GPU physics engine that Isaac Lab
> still carries as experimental); it exposes the whole simulator over **plain HTTP and JSON
> with no ROS, no DDS, no in-process Python and no editor plugin** — where the competing
> scene-control surfaces are real but reach you through a ROS 2 stack or an in-process
> extension, and a ROS 2 sidecar is available on top for those who want one — with a
> first-party MCP server and a measured 10/10 agent-driveability
> score; and it runs the **GPU-batched RL path on a laptop GPU**, with a CPU `mj_step` path
> that is also the only configuration in which it is bitwise reproducible, where Isaac Sim
> requires an RT-core card and refuses to run on an A100. It is not photoreal, it has no
> `ros2_control` plugin (so no Nav2 or MoveIt out of the box), its GPU integration layer had five
> bugs in it as recently as 2026-07, and on
> the one **real-world impact dataset it has ever been scored against it sits mid-field —
> 24.8% of a cube width, behind Drake's 13.5%** (§1.6). For ROS-centric integration work, use
> Gazebo. For photoreal perception and synthetic data, use Isaac Sim. For accuracy checked
> against real-world impacts, read Drake's literature. For an agent-driven simulator you can
> `curl`, on the hardware you already own, use OmniSim.

---

## 9. What we could not verify

### 9.1 Now measured (was open in the 2026-07-10 edition)

- **OmniSim's physics accuracy** — was entirely unmeasured; now seven analytic scenes ×
  4 engines × 6 timesteps, cross-machine (§1.1) 📊.
- **The OmniSim-vs-raw-mujoco-warp throughput gap** — the old 17.4× figure was a
  graphed-vs-ungraphed artifact; now **1.21–1.34×** graphed-against-graphed (§1.2) 📊.
- **Determinism** — was an unstated assumption; now measured **per configuration** (§3.3) 📊:
  bitwise on CPU `mj_step` (at 336 contacts with ten live controllers), **refuted on the
  GPU `mujoco_warp` path** (0 of 24 pairs), and **untested cross-machine**. ⚠ Two further
  bitwise configurations were measured and are now **unverifiable** — ODE, and Newton/XPBD
  in one light-contact world (10/10 rows on three machines) — because both were deleted
  (`bdc02139`, `94f04222`). Only the CPU `mj_step` row is a live claim.
  Scope doc: [determinism-scope.md](../benchmarks/determinism-scope.md).
- **Agent-driveability of the harness** — was a design claim; now a 10-probe score (§1.3) 📊.
- **Maintenance and release facts for 16 projects** — re-fetched from the GitHub API
  2026-07-26; **every figure in the research pass matched** ✅ (§3.1).
- **Isaac Sim's licence and contribution posture** — re-fetched from the LICENSE and
  CONTRIBUTING files ✅ (§3.2).

**Added 2026-08-16** — four things this document called unmeasured that were already
measured and committed in-tree:

- **Real-hardware validation** — the largest open item of the 2026-07-26 edition. Now one
  dataset: 50 recorded real cube tosses, **24.845%** / **23.104°**, `embed_gap` 0.000127 pp,
  `tunnel_depth` 0 (§1.6) 📊. Mid-field, behind Drake.
- **Upstream Webots as a running engine** — was "we do not run this engine"; it is a control
  arm on ladder0 (rungs 9 and 11) and on AgentBench (§1.6) 📊, ⚠ cross-platform as installed.
- **The `njmax` constraint cliff** — OmniBench lane 4b could not reproduce it and honestly
  reported `cliff_detector_validated: false`; ladder0's GPU study confirms `nefc = 32·N` and
  the N=16 overflow with the engine's own warning firing (§1.6) 📊.
- **The lane-4 capability headline** — this document carried 26/4/9/5 (67%) after the
  generated matrix had moved to 28/4/6/5/1 (74%) (§0) 📊.

### 9.2 Corrected — things this document previously got wrong

1. **"No established simulator ships a first-party agent-facing scene-control API."** Wrong.
   ROS 2 `simulation_interfaces` is an Apache-2.0 standard ✅ implemented natively by Gazebo,
   Isaac Sim and O3DE ◐ — and, since 2026-08-17, by OmniSim as a sidecar. Corrected in §5.1; the
   differentiator is the transport and the dependency, not the existence.
2. **"Third-party MCP only" as the competitive frame.** Dropped. Whether a wrapper is
   first- or third-party is not what distinguishes the surfaces.
3. **"PyBullet: maintenance mode."** It is **dead** — 1 commit in 12 months ✅, last release
   2022-04-24 ✅. (It is also still one of the two most accurate engines in our own lane 1 📊.)
4. **"Webots: Active ✅."** True of the release tag, misleading about the project: **48
   commits/12 mo and a 28× four-year decline** ✅ (§3.1).
5. **"Isaac Sim: Apache-2.0."** GitHub classifies it **NOASSERTION** and its own LICENSE says
   building or using it requires components under other terms ✅ (§3.2).
6. **Newton "has no conservation tests."** This was in the 2026-07-26 research pass and it is
   **false** — `newton/tests/test_physics_verification.py` has explicit energy and momentum
   conservation tests against closed-form references ✅ (§2.2). The surviving claim is that it
   publishes no error *values*.
7. **"ODE = the legacy tier we are leaving."** Corrected in the performance paper's header
   box and again here: `omnisim-ode` is OmniBench's **best-scoring integration** 📊 (§7).
   ⚠️ And the over-correction is now itself on this list — see item 9.
8. **"Cross-machine determinism: Bitwise"** (the §4 matrix row) and **"OmniSim: Bitwise"** as
   a single cell against competitors' GPU answers (§3.3). Both **struck 2026-07-26**. Two
   separate errors. (a) *Cross-machine* was never tested — the 10/10 rows are each machine
   reproducing itself, and this document's own §1.1 census records 56 of 180 cells differing
   between M1 and M2, 13 of them at rel ≥ 1e-3, which contradicts the row directly. (b) The
   grade came from ODE and Newton/**XPBD on a light-contact scene**, so setting it against
   MJWarp's "No" compared unlike things; measured on our own GPU `mujoco_warp` path we score
   **0 bitwise of 24** pairs — MJWarp's position, MJWarp's mechanism. Both rows are now split
   by configuration, and the surviving differentiator is that we publish the scope, not that
   we score better. Evidence: [determinism-scope.md](../benchmarks/determinism-scope.md).
9. **"No real-hardware validation" and "we do not run upstream Webots."** Both **struck
   2026-08-16**; both had been false since 2026-08-13 (§1.6). Also struck: a stale lane-4
   capability headline, a `njmax` cliff described as unreproducible, and an ODE clause left
   standing in the §8 positioning paragraph after ODE was deleted. The lesson is the one §0
   already states about the capability matrix — **a figure copied into prose goes stale
   silently** — and this document is what README.md points readers to, so it goes stale
   *loudly*.

10. **"No ROS 2 bridge — and now also no `simulation_interfaces`."** **Struck 2026-08-17.** ROS 2
    was a documented *non-goal*, not an unfinished feature, and this document said so in §4, §7
    and §8. The project owner reversed the decision, and OmniSim now implements the standard as a
    sidecar over the existing HTTP surface ([`packages/omnisim-ros2/`](../../packages/omnisim-ros2/)).
    Two things worth carrying forward. First, **the old reasoning was not wrong so much as falsely
    binary** — "HTTP is our agent interface" and "we should have a ROS bridge" were treated as
    alternatives, and they were never that; the HTTP surface is unchanged and the engine is still
    ROS-free. Second, the cost estimate was wrong in our own favour's *opposite* direction: the
    stated path was porting `webots_ros2` as a multi-week workstream, when in fact the harness
    already served every verb the standard asks for. **The remaining loss is real and narrower**:
    no `ros2_control`, so no Nav2 and no MoveIt.

### 9.3 Still open

- **Real-hardware validation is ONE dataset, which is not zero and is not enough.** 50 rigid
  cube tosses, replayed by us, one machine, one object class (§1.6). No cloth, no legged
  landing, no manipulation, **no third-party replication**, and **no OmniBench lane**. ⚠ This
  bullet read "No real-hardware validation of any OmniSim number" until 2026-08-16.
- **No independent replication of anything in §1.** OmniBench is our harness, run by us on
  our machines. Auditable ≠ audited.
- **Acosta et al.'s per-simulator error percentages** (Drake 13.5% / Bullet 14.9% / MuJoCo
  25.1%) come from the paper body ◐; only title, authors, venue and abstract were re-fetched
  ✅ — and **the abstract carries no numbers**. Treat the ordering as reported, not verified.
- **The three `simulation_interfaces` native implementations** ◐ were not individually
  fetched, nor were the remaining service names in §5.1. OmniSim's own implementation is a
  sidecar and is **not** counted among the native three.
- **Isaac Sim's "redistribution requires AI Enterprise"** ◐ — the linked NVIDIA agreement was
  not re-fetched.
- **SAPIEN's PyPI-says-MIT** ◐ — not re-fetched. Two of the three sources *were* verified and
  they disagree with each other ✅.
- **The MJWarp determinism "No"**, the **Isaac Lab / Newton / Drake / Genesis / Brax
  determinism claims**, and the **Isaac Lab and MJX throughput tables** are research-pass
  extractions ◐, not re-fetched here.
- **Genesis's true throughput remains contested** ⚠️ — the withdrawn 43M and an independent
  0.29M differ by ~150×, and the live "10–80×" claim has no methodology. Cite no Genesis
  number as settled.
- **Our own suite's known holes**: no rendering-on throughput, no ODE batched row, no tier-B
  OmniSim variant, PyBullet's version string not recorded, the lane-3a XPBD-vs-lane-1-MuJoCo
  solver split, and the non-symmetric friction-cone overrides across engines on T2/T3/T6.
  All enumerated in [omnibench-2026-07-24.md](../benchmarks/omnibench-2026-07-24.md).

### 9.4 Flagged for v7 — found while correcting this document, deliberately NOT fixed here

Two evidence-hygiene defects elsewhere in the tree. Both are documentation/plumbing
problems rather than measurement problems, and neither is this file's to fix — recorded so
they are visible rather than rediscovered.

1. **A committed Pareto frontier that no document reports.**
   [`lane1-validity-2026-08-07.md`](../benchmarks/lane1-validity-2026-08-07.md)**:55** states
   *"No Pareto exists anywhere in the tree"* — while `SPEC.md` mandates one. It was built and
   committed two days later in `e797ca240`, at
   `tests/benchmarks/omnibench/results/9722d23d12a3/2026-08-09-pareto/pareto.json`: **7
   scenes × 3 engines × 6 timesteps, 113 accepted points, with an explicit `rejected` list**
   of 13 (diverged `bounce` runs, `energy_blew_up` pendulums at dt=32, collapsed stacks).
   `grep -rn pareto.json docs/` returns **zero hits**, and
   [ode-retirement-campaign.md](ode-retirement-campaign.md)**:612** still repeats *"speed-accuracy
   Pareto that does not exist"*. **Two stale sentences and one unreported artefact.** Fix the
   sentences and report the frontier, or delete it — an unreported result is the same failure
   mode as an unreported loss.
2. **Lane 3's results directory is gitignored.**
   `tests/benchmarks/omnibench/lane3/results/.gitignore` is `*` plus `!.gitignore`, and
   `git ls-files` on that directory returns **only the `.gitignore` itself** — so the
   determinism, train↔deploy-parity and driveability rows this document cites in §1.3, §3.3
   and §5.1 exist **only on one machine's disk**. ⚠️ **AgentBench's own rule is "no row, no
   result."** ⚠️ And **lane 4 sits next to it doing the opposite, with its reason written
   down** in its own `.gitignore`: *"the .jsonl rows ARE the evidence and are committed (they
   carry a machine id and an engine-binary sha256, so a row is meaningful outside the machine
   that produced it)."* Same suite, same argument, opposite policy. By AgentBench's rule
   three of this document's 📊 claims are currently unreproducible by anyone but us on that
   box — a weaker link than anything the five corrections above touched.

---

## 10. Method and references

**OmniSim numbers (📊)** come from `tests/benchmarks/omnibench/`, run 2026-07-24 (M1, M2)
and 2026-07-25 (M1 post-fix, M3). Every row carries a machine id, engine-binary sha256,
libController sha256, stack versions and a `deviations` list. Reproduce with
`python tests/benchmarks/omnibench/run_all.py`. Binding honesty rules are in
[`SPEC.md`](../../tests/benchmarks/omnibench/SPEC.md): never compare a batched-GPU number to
a single-env CPU number; OmniSim's Newton backend embeds mujoco-warp, so accuracy deltas vs
MuJoCo are solver-family-internal and are framed as *integration fidelity*, never "beating
MuJoCo"; losses are reported as prominently as wins; every number quoted outside `results/`
carries its machine id.

**Since 2026-08-16 the 📊 tier also covers two other in-tree suites** (§1.6), under the same
rule that every row names its machine and its binary: **ladder0**
([`tests/benchmarks/ladder0/`](../../tests/benchmarks/ladder0/), campaign 2026-08-13, M1,
engine sha256 `cbd80861c9f0d314`, arms omnisim / upstream-webots / mujoco) and **AgentBench**
([`tests/benchmarks/agentbench/`](../../tests/benchmarks/agentbench/), campaigns
`phasew_cc_v1` and `phasew_cc_v1_B`, 2026-08-01). The upstream-Webots install recipe, its
traps, and the cross-platform caveat that must accompany every Webots number are in
[webots-control-baseline.md](webots-control-baseline.md).

**Competitor facts** come from two `deep-research` passes (2026-07-10, 209 subagents) and a
targeted pass on 2026-07-26. In **this** editing pass the following were re-fetched
first-hand and are marked ✅: SimBenchmark's push date and star count (GitHub search API);
release tag + date for 16 projects and 12-month commit counts for 8 (GitHub API); Webots'
per-year commit curve; Isaac Sim's LICENSE text, GitHub licence classification and
CONTRIBUTING policy; Isaac Lab's BSD-3 classification; SAPIEN's LICENSE file vs API
classification; `simulation_interfaces`' licence and part of its service list; Newton's test
directory listing and `test_physics_verification.py` contents; arXiv 2110.00541's title,
authors, venue and abstract. Everything else carries ◐ or ⚠️.

**OmniSim-side capability facts (⊘)** were checked against this checkout
(`src/omnisim/nodes/`, `src/controller/launcher/`, `PROTOCOL.md`, `scripts/harness/`,
`packages/omnisim-mcp/`), not against its own docs. They are self-attested.

**References.**
OmniBench — [campaign report](../benchmarks/omnibench-2026-07-24.md), [`SPEC.md`](../../tests/benchmarks/omnibench/SPEC.md), [`lane3/DRIVEABILITY.md`](../../tests/benchmarks/omnibench/lane3/DRIVEABILITY.md), [lane-4 capability matrix](../benchmarks/lane4-capability-matrix.md) ·
ladder0 — [`tests/benchmarks/ladder0/README.md`](../../tests/benchmarks/ladder0/README.md), [`CONTRACT.md`](../../tests/benchmarks/ladder0/CONTRACT.md) ·
AgentBench — [`tests/benchmarks/agentbench/SPEC.md`](../../tests/benchmarks/agentbench/SPEC.md), [webots-control-baseline.md](webots-control-baseline.md), [agent-edge-validation-plan.md](agent-edge-validation-plan.md) ·
SimBenchmark — [leggedrobotics/SimBenchmark](https://github.com/leggedrobotics/SimBenchmark), [site](https://leggedrobotics.github.io/SimBenchmark/) ·
Accuracy studies — [arXiv 2110.00541](https://arxiv.org/abs/2110.00541) (Acosta/Yang/Posa, IROS+RA-L 2022), [arXiv 2310.09543](https://arxiv.org/abs/2310.09543) (Blanco-Mulero, RA-L 2024), [arXiv 2304.06372](https://arxiv.org/abs/2304.06372) (Le Lidec, T-RO 2024), [DOI 10.1016/j.simpat.2022.102629](https://doi.org/10.1016/j.simpat.2022.102629) (Farley 2022) ·
ROS 2 scene control — [ros-simulation/simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces), [ros_gz](https://github.com/gazebosim/ros_gz), [`ros2_control` simulators](https://control.ros.org/jazzy/doc/simulators/simulators.html) ·
Newton — [newton-physics/newton](https://github.com/newton-physics/newton), [`test_physics_verification.py`](https://github.com/newton-physics/newton/blob/main/newton/tests/test_physics_verification.py), [developer.nvidia.com/newton-physics](https://developer.nvidia.com/newton-physics), [Linux Foundation announcement](https://www.linuxfoundation.org/press/linux-foundation-announces-contribution-of-newton-by-disney-research-google-deepmind-and-nvidia-to-accelerate-open-robot-learning) ·
MuJoCo — [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco), [mujoco_warp](https://github.com/google-deepmind/mujoco_warp), [MuJoCo Playground](https://arxiv.org/abs/2502.08844) ·
Isaac — [isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim), [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab), [Isaac Sim additional-materials licence](https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/), [Newton integration status](https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/index.html) ·
Gazebo — [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim), [releases](https://gazebosim.org/docs/latest/releases/) ·
Drake — [RobotLocomotion/drake](https://github.com/RobotLocomotion/drake) ·
Webots — [cyberbotics/webots](https://github.com/cyberbotics/webots), [R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a) ·
Genesis — [Genesis-Embodied-AI/Genesis](https://github.com/Genesis-Embodied-AI/Genesis), [issue #181](https://github.com/Genesis-Embodied-AI/Genesis/issues/181) ·
ManiSkill / SAPIEN — [haosulab/ManiSkill](https://github.com/haosulab/ManiSkill), [haosulab/SAPIEN](https://github.com/haosulab/SAPIEN), [arXiv 2410.00425](https://arxiv.org/abs/2410.00425) ·
PyBullet — [bulletphysics/bullet3](https://github.com/bulletphysics/bullet3) ·
CoppeliaSim — [licensing](https://manual.coppeliarobotics.com/en/licensing.htm) ·
Unity / O3DE / Colosseum — [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub), [o3de/o3de](https://github.com/o3de/o3de), [CodexLabsLLC/Colosseum](https://github.com/CodexLabsLLC/Colosseum).

When this doc and the code disagree, the code wins — and update this doc in the same change.

---

## 11. Sources and caveats behind the README table

The README's `## How OmniSim compares` section is **tables only**, by design — the prose that used to
sit between those tables is here. Every qualifier below is load-bearing: none of it is decoration, and
a claim quoted without its qualifier is a claim we have not made.

### 11.1 Agent-native — we are not alone, and the difference is narrower than it looks

**NVIDIA shipped a real agent surface in June 2026.** Isaac Sim's repo carries its own `AGENTS.md`, a
`CLAUDE.md`, and **25 `SKILL.md` skills** with Claude Code and Cursor integration. Any claim that
OmniSim is the only agent-driveable simulator is false and should not be made.

**Gazebo's surface is genuine too.** ROS 2
[`simulation_interfaces`](https://gazebosim.org/docs/latest/ros2_sim_interfaces/) is a typed,
standardised spawn / delete / step / reset / query API with reference implementations in Gazebo,
Isaac Sim and O3DE. Webots is the outlier here, not the norm — it has no network authoring API at
all, only an in-process Supervisor library and an undocumented IPC/TCP controller channel.

**The difference we do claim is transport and typing, not existence.** Ours is typed verbs with
structured diagnostics over plain HTTP that `curl` can drive — no ROS/DDS stack, no in-process
Python. Isaac Sim's scene control is
[`isaacsim.code_editor.python_server`](https://docs.isaacsim.omniverse.nvidia.com/latest/development_tools/python_server.html):
you send Python source over a raw socket and read back stdout. It is powerful and genuinely
headless-capable; it is not a schema'd API, and NVIDIA's own docs warn that binding it publicly
*"allows any machine on the network to execute arbitrary Python code in your Isaac Sim session."*
Their official MCP server is **documentation semantic search — 5 tools, none of which touch a running
simulator**.

⚠️ **We publish no agent success rate, and the table must never imply one.** Both `agentbench/` and
`omnilink_tasks/` gitignore their `results/`, so no score ships in this tree, and no recorded
`omnilink_tasks` run has had an LLM in the loop — every one is the regex router. The one tool-surface
ablation we ran declined to show a win: the bare shell tied the full HTTP surface on outcome with
*fewer* calls. We claim surface size, never performance.

### 11.2 The hardware claim — exact wording is mandatory

The sanctioned phrasing is **"runs with no CUDA device visible to the process."** The CPU-only probe's
own `scope` field forbids the stronger form: *"CUDA devices hidden from the process; the machine still
has a driver, a CUDA runtime and the GPU wheels installed. This is NOT a GPU-less-hardware result and
must never be quoted as one."* OmniSim has **never** been verified on genuinely GPU-less hardware.

⚠️ **The no-CUDA-visible claim is now measured on TWO machines** (M1 and M6, 2026-08-17): both
finalise Newton, both land the analytic drop at z = 0.6499 against an expected 0.65, and both report
the trajectory identical to their own GPU-visible run (max deviation 0.0 m).

Two further limits: the claim does **not** extend to cloth, soft bodies or granular media, which are
GPU features; and we publish **no RAM or VRAM footprint**, because none has ever been measured —
there is not one `ram|vram|rss|peak_mem` field anywhere in the benchmark corpus. ⚠️ **The cloth
figure in that parenthesis used to read "6.7 fps on CPU" and it was stale by 2.9×.** Re-measured
2026-08-17 on M1 with the device forced (`cloth_bench.py --device cpu`, `solver_obj`
`SolverCoupledProxy(mjc cpu=True)`, no graph): **51.87 ms/step = 19.3 frames/s = 0.154× real time**.
The conclusion is unchanged — 0.154× is not a simulation anyone can drive, against 2.81–3.39× on the
GPU — but the number is corrected, and it is still **one machine**. MPM granular remains
0.15–0.25× real time, also one machine.

### 11.3 Why we refuse the Isaac Lab throughput ratio

Isaac Lab publishes **94,000 env-step FPS** for `Isaac-Velocity-Rough-G1-v0` at 4096 envs on an RTX
4090 — [PhysX backend](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/performance_benchmarks.html),
which the page states explicitly. **No Newton-backend throughput is published for Isaac Lab at all.**

Our comparable-looking figure is 500,105 on a 4090. Normalising the two would let us claim roughly
10×, and it would be dishonest: different robot (G1 humanoid vs Go2 quadruped), different terrain
(rough vs flat), and a different substep count per env-step (their `decimation = 4` against our 8).
Present both figures attributed, or present neither.

### 11.4 The Webots / Gazebo deformable claim is the literature's, not ours

This is the strongest negative claim in the README table, so it is sourced rather than asserted.

**Primary:** J. Collins, S. Chand, A. Vanderkop, D. Howard, *A Review of Physics Simulators for
Robotic Applications*, **IEEE Access 9:51416–51431 (2021)**,
[DOI 10.1109/ACCESS.2021.3068769](https://doi.org/10.1109/ACCESS.2021.3068769) — open access, CSIRO /
QUT, 200+ citations. Its Table 1 (*Soft-Body Contacts*) marks **✗ for both Gazebo and Webots**; its
Table 3 (*Deformable Objects*) marks **✗ for Gazebo**.

**Corroboration, three ways:**
1. Zafra-Navarro et al., *Survey of Simulators for Deformable Objects in Robotics*, Jornadas de
   Automática 46 (2025), [DOI 10.17979/ja-cea.2025.46.12171](https://doi.org/10.17979/ja-cea.2025.46.12171)
   — a survey whose entire subject is deformable-object simulators **does not mention Gazebo or Webots
   at all**.
2. [`gazebosim/gz-physics#222`](https://github.com/gazebosim/gz-physics/issues/222), *"Support DART
   soft body simulation"* — opened **2021-03-09**, still open and unassigned.
3. Webots runs **only its own fork of ODE** (maintainer statement,
   [webots#4869](https://github.com/cyberbotics/webots/discussions/4869)), and ODE has no deformable
   support of any kind. The absence is architectural, not a missing feature flag.

⚠️ **Cite Collins for Gazebo and Webots only.** It is from 2021 and predates Isaac Sim 5.x/6.x,
MuJoCo 3's `flex`, Genesis and Newton entirely.

⚠️ **One dissent exists and we do not hide it.** Wong et al. 2025 ([arXiv 2505.01458](https://arxiv.org/abs/2505.01458))
Table 2 marks Gazebo as supporting soft bodies *and* cloth. Three sources contradict it; our reading is
that it describes Bullet-the-library's nominal capability rather than what Gazebo's `gz-physics` plugin
layer actually exposes. We judge it wrong, and say so rather than omitting it.

### 11.5 The cloth-grasp numbers — what they are and are not

The grasp table's tracking error is (rise of the gripped region) − (rise of the measured jaw midpoint).
The corroborating **jaw gap** is read from jaw *body poses*, not particles, which is what makes it
independent: it cannot be confounded by which particles are labelled "gripped".

⚠️ **The fabric is pinned**, so these are *tracking* numbers, not *load-bearing* ones — the garment is
not hanging from the jaws by its own weight. ⚠️ **The negative control is not contact-free**: its open
jaws plough through the garment and drag it 77–135 mm, so it bounds "did the jaws do anything", not
"did the jaws touch anything". ⚠️ **Self-contact has no correct default** — draping needs it on,
grasping needs it off, and leaving it on costs **24×** on tracking error (−0.92 mm → −22.11 mm).
⚠️ The composed **fold** is **not** demonstrated.

Full derivation and the worlds involved: [cloth-simulation.md](cloth-simulation.md).

### 11.6 Why VBD, and why not XPBD

[*Vertex Block Descent*](https://arxiv.org/abs/2403.06321) (Chen, Liu, Yang & Yuksel, ACM TOG 43(4),
SIGGRAPH 2024) solves the variational form of implicit Euler by vertex-level Gauss–Seidel block
coordinate descent. The property that matters for robotics is **unconditional stability**: cloth can be
integrated at a timestep chosen by the *robot's* control loop rather than by the fabric's stiffness.
[*Augmented VBD*](https://dl.acm.org/doi/10.1145/3731195) (SIGGRAPH 2025) extends it to hard
constraints; Newton's `SolverVBD` uses VBD for particles and AVBD for rigid bodies.

XPBD, the usual alternative, projects constraints rather than converging to the implicit-Euler
solution, so its effective stiffness depends on iteration count and timestep — which is why it
**structurally cannot hold a static pinch**. That is not a preference; it is the wall this project hit,
and it is why XPBD was removed outright on 2026-08-07.

### 11.7 GAUGE — an independent measurement, quoted in both directions

[GAUGE](https://arxiv.org/abs/2608.05948) (arXiv 2608.05948, 2026-08-06, Shanghai AI Laboratory — no
NVIDIA or Google authorship) benchmarks Isaac Sim 6.0.0, Genesis 1.12.0 and **Newton 1.3.0** against
real motion-capture trajectories (16 cameras @ 180 Hz, sub-mm, RTX 4090). Newton runs `SolverMuJoCo`
for rigid and **VBD for both textile and volumetric** tasks there.

It is the only independent measurement of Newton we know of, and it does **not** uniformly favour it.
The rows, lower being better — this table moved here from the README on 2026-08-17, because GAUGE
measures **Newton**, not OmniSim, and it was being read as a claim about our simulator:

| task | metric | **Newton 1.3.0** | Isaac Sim 6.0.0 | Genesis 1.12.0 |
|---|---|---|---|---|
| Pendulum | period duration | **1.09** | 1.10 | 2.47 |
| Textile stretching | RMSE | **0.73** | 0.73 | 1.51 |
| Textile flinging | RMSE | 9.25 | **128.26** | 8.54 |
| Textile bending | RMSE | **19.90** ✗ | 7.94 | 11.73 |
| Turntable | DTW | **66.72** ✗ | 0.61 | 2.01 |

So Newton leads on pendulum period, ties Isaac Sim on textile stretching, and **loses badly on the
turntable (66.72 against 0.61, >100×) and on textile bending**. Isaac Sim's textile *flinging* error
of 128.26 against Newton's 9.25 is the single sharpest independent datum we hold about its cloth
path — and it sits in the same table as our own two losses, which is the only honest way to quote it.
The paper's own conclusion is the headline: *"no uniformly faithful physics engine."*

⚠️ **Two caveats.** It is a **preprint**, not peer-reviewed, and appears to be n=1 per cell. And it
tested **Newton 1.3.0**; OmniSim ships 1.5.0, so its rows are not a measurement of what we ship.
⚠️ Our own transcription of its result table came from a page-fetch summariser rather than from the
rendered PDF — unlike the Collins tables, which were read directly. Re-read the paper before quoting
any individual GAUGE figure externally.
