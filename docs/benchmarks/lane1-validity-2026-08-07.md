# Is lane 1 measuring the right thing? Mostly not — a validity audit

**Question asked:** *"are we sure we are not measuring the wrong thing, because maybe
Newton is right in its own way?"*

**Answer: substantially yes, we are measuring the wrong thing** — though not in a way
that exonerates Newton, because the defects that actually block progress are not the
ones lane 1 scores.

Written 2026-08-07 against `8e5f022f`. Sources: the published engine-benchmarking
literature, MuJoCo's primary documentation, and the suite's own code.

> ⚠️ **2026-08-08 — LANE 1 LOST ITS SECOND IN-ENGINE ARM (not its oracle).** `bdc02139`
> deleted the ODE backend. **Lane 1 used ODE as its second integration and its world
> generator emitted an `ode_pin`**, so the lane as designed cannot be run. Note the
> precise wording: lane 1's *oracle* is **analytic ground truth** and is unaffected, and
> bare MuJoCo and PyBullet still run as external arms — what is gone is the ability to
> diff two integrations of the same `.wbt`. See [correctness-scope.md](correctness-scope.md). The frozen reference values survive in
> [`tests/goldens/ode_oracle_goldens.json`](../../tests/goldens/ode_oracle_goldens.json) —
> now the only ODE artefact in the tree, and a **golden file, not an oracle**: it can detect
> that Newton *changed*, never adjudicate which engine was right.
>
> This audit is therefore **more** load-bearing than when it was written, not less. It is the
> record of what that lane did and did not establish, and every "Newton loses N of 11"
> statement anywhere in this tree now traces back to numbers that can never be re-derived.
> Read §9 for what survives on its own merits — the contract violations, which never depended
> on the ODE comparison.
>
> The ODE analysis below (Erez et al., `dWorldStep`/Dantzig, `softCFM`, the maximal- vs
> generalized-coordinate argument) is preserved intact: it is the reasoning that explains
> *why* the comparison was shaky, and it stays valid as an account of a system that used to
> ship. Campaign record: [`../developer/ode-retirement-campaign.md`](../developer/ode-retirement-campaign.md).

---

## 1. The suite cites a paper whose method it inverts

`SPEC.md:8` names **"Erez/Tassa/Todorov ICRA'15 timestep-sweep methodology"** as design
lineage. That paper argues *against* the thing we built
([PDF](https://roboti.us/lab/papers/ErezICRA15.pdf), §III):

> "We seek to develop universal tests and performance metrics that can be applied to any
> model system. **This is in contrast with benchmarks such as single rigid-body motion or
> energy and momentum conservation which rely on analytical solutions.** … typical robotic
> systems do not preserve energy, momentum, symplectic forms or any other known quantity."

Their primary metric is **self-consistency**: integrate at h = 1/64 ms to obtain an
*engine-specific* reference trajectory, then measure deviation at larger h. That measures
numerical integration error and deliberately excludes model error. And on ranking:

> "in order to evaluate an engine we need to characterize its **entire** speed-accuracy
> curve."

**`SPEC.md:38-39` mandates exactly that — a speed-accuracy Pareto, "never accuracy alone".
No Pareto exists anywhere in the tree** (`grep -i pareto` hits only SPEC.md). Every
headline is read at fixed dt = 4.

The same requirement appears in both other cited ancestors. SimBenchmark (ETH RSL): *"to
evaluate performance fairly, the whole speed-accuracy curve must be taken into account."*
Peters & Hsu (OSRF, ECCOMAS 2015) compare cross-engine cost with the caption **"Timestep
chosen to give equal accuracy"** — i.e. the published control variable is *accuracy*, not dt.

## 2. Our momentum results are the literature reproducing itself

Erez et al. measured ODE beating joint-coordinate engines on **both** momenta, and
explained the mechanism:

> "Constant linear momentum means that the system state remains on a manifold which is a
> **linear subspace in Cartesian coordinates** … The same manifold is **curved in joint
> coordinates** … even [RK] cannot compete with Cartesian engines on this test."

They call it *"a genuine advantage of simulation in Cartesian coordinates."* On angular
momentum: *"In the astronaut test ODE performs very well despite using Euler integration.
This is probably because it implements semi-implicit integration of Coriolis forces."*

ODE is maximal-coordinate; MuJoCo is generalized-coordinate. **T5 (`linear_momentum_max`
3.910 vs 9.78e-15) and T7 are a published, explained representational consequence.**
Reporting them as "Newton loses" without that context inverts their meaning.

⚠ A correction recorded here because it was made in the other direction during this audit:
it is tempting to argue that semi-implicit Euler conserves linear momentum exactly when the
net external force is zero (T5 is gravity-free and driven by an equal-and-opposite couple),
and therefore that 3.910 must be a plumbing bug. **That reasoning holds in maximal
coordinates and not in generalized ones**, which is precisely Erez's point. The residual
concern about T5 is not the sign but the magnitude: the same result doc records the
velocity-derived momentum peaking at 3.910 while the position-derived peaks at **1.531** on
the same run — a 2.5× disagreement between two rulers that makes the *value* untrustworthy
independent of the physics.

## 3. Three metrics score features MuJoCo does not have

| metric | what MuJoCo's own docs say |
|---|---|
| **T1 `bounce_height_rmse_rel`** | **There is no restitution coefficient in MuJoCo.** Bounce is emergent from an under-damped constraint spring: *"Smaller [dampratio] values result in under-damped or bouncy constraints."* `mjContact.solreffriction` was added later specifically because separate normal/friction `solref` *"is required for elastic frictional collisions"* — elasticity was retrofitted through solver parameters, never exposed as a coefficient. |
| **T6 `max_penetration_m`** | Penetration is defended as *increased* realism: *"For soft contacts complementarity has to be violated… **the deviations from LCP actually increase physical realism in the presence of soft contacts**."* The payoff is a uniquely-invertible, convex contact model. |
| **T2 / T6 friction cone** | `pyramidal` + `impratio 1` is the documented **speed/robustness** default: *"Elliptic cones are a better model of the physical reality, but pyramidal cones sometimes make the solver faster and more robust."* And: *"When contact slip is a problem, the best way to suppress it is to use elliptic cones, large impratio, and the Newton algorithm with very small tolerance."* We score the default against an accuracy metric the docs tell you to change a knob for. |

The suite already knows about T1 — `lane1/run_omnisim.py:35-36`: *"restitution: no mapping
exists (contact compliance ke/kd only) — T1's e=0.8 is not expressible; the measured bounce
IS the finding."* Both sides are then hand-fitted anyway: ODE gets `softCFM 1e-6` (against a
0.001 default) and Newton gets `OMNISIM_NEWTON_CONTACT_KD=7` (against 100).

The right frame is Le Lidec et al., *Contact Models in Robotics: a Comparative Analysis*,
IEEE T-RO 2024 ([arXiv:2304.06372](https://arxiv.org/abs/2304.06372)): **every engine
violates different physical laws and none satisfies all of them.** LCP engines keep
Signorini and lose friction isotropy to pyramid linearisation; convex engines keep Coulomb
and maximum-dissipation and lose Signorini. MuJoCo is in the compliant/relaxed family by
design. **The honest output is a per-law violation profile, not a win/loss tally** — a
tally systematically penalises whichever engine relaxes the laws your scenes happen to test.

## 4. Two metrics are broken as instruments

- **T2 `slide_accel_rel_err` is near-singular.** a(27°) = g(sin27° − 0.5·cos27°) =
  0.0833 m/s². A 0.3% error in *effective* μ produces a **16% relative error** at that
  angle. Averaging a relative error whose denominator → 0 at θc makes this a μ-offset
  amplifier, not an acceleration test. Report per-angle, or drop θ = 27 from the mean.
- **T2 `transition_angle_err_deg` is floored.** It is a 7-point bracket, not the bisection
  `SPEC.md:71` describes: resolution 0.5°, floor 0.065°. Three engines report exactly
  0.065 — the floor, not a measurement. (This also explains identical decimal tails across
  campaigns: `atan(0.5) = 26.56505117707799°`, so any half-integer bracket midpoint yields
  the same fractional part. `|22.5 − 26.565…| = 4.065…`; `|26.5 − 26.565…| = 0.065…`.)
- **T5 `angular_momentum_drift_rel` is a normalizer artefact** — the result doc says so
  itself: the normalizer collapsed 1.757 → 0.311 across a fix, so the ratio worsened while
  the absolute drift moved 0.219 → 0.304. Cite `_abs` only.

## 5. Solver effort is not matched, and the control arm is a different integrator

- **ODE runs `dWorldStep`** — the direct big-matrix Dantzig LCP, exact per island, **no
  iteration budget** ([`OmSimulationCluster.cpp:117`](../../src/omnisim/engine/OmSimulationCluster.cpp#L117)).
  Not quickstep; there is no `dWorldSetQuickStepNumIterations` anywhere in `src/omnisim/`.
- **Newton runs MuJoCo's defaults** — no lane-1 world sets `newtonIterations`, so
  `SolverMuJoCo` falls through to solver=newton, **iterations 100, ls_iterations 50**,
  integrator `implicitfast`, cone pyramidal, impratio 1, tolerance 1e-8.
- **The raw-MuJoCo control arm uses `Euler`**, not `implicitfast`.

Consequence: the inference in `omnibench-2026-07-24.md:283-285` — *"raw mujoco scores fine
on the same scenes with the same solver family, so these live in our Newton integration
layer"* — **does not hold for integrator-limited metrics** (T4, T5, T7), because the two
arms do not share an integrator.

## 6. Two scenes are not backend-neutral, and two have a duplicate floor

- `t2_incline_*.wbt` and `t6_stack.wbt` carry **`newtonCone "elliptic"` + `newtonImpratio
  10` in the same file both lanes load** (only `defaultPhysicsBackend` differs) — Newton-only
  tuning ODE cannot receive. `t1_bounce_*.wbt` carries `softCFM 1e-6`, ODE-only. **Neither
  world is a complete, backend-neutral description of its own physics.**
- **T1 and T6 floors sit with their top face at exactly z = 0** (`translation 0 0 -0.1`,
  `size … 0.2`), and `OmNewtonBackend::openWorld()` calls `addGroundPlane()`
  **unconditionally**. So under Newton the body resolves against two coincident manifolds
  where ODE has one — and `CONTACT_KD=7` was hand-fitted against that doubled stiffness.
  Fix: move the floor off z = 0.
- **T6's stated caveat is itself wrong.** `omnibench-2026-07-24.md:267-272` says Newton
  *"runs it at the newtonGroundMu default of 1.0"*; `lane1/run_omnisim.py:66` sets T6 → 0.8.
  The real asymmetries are the elliptic cone (Newton but not raw MuJoCo) and contact
  compliance never being equalised (ODE `soft_cfm 0.001` vs Newton `ke 2500 / kd 100`).

## 7. The headline claim has no data behind it

"Newton wins 4 of 11 at substeps=4" originates in commits `0e3bb59d` and `85bf57da`. **Both
touch only `docs/benchmarks/omnibench-2026-07-24.md`.** No jsonl row in any results tree
carries a `substeps` key; lane 1 has no substeps knob (`newtonSubsteps` appears in zero
lane-1 worlds, `OMNISIM_NEWTON_SUBSTEPS` nowhere under `lane1/`). The experiment was ad-hoc
env-var driving and is **not reproducible from the shipped suite**. Same for the T6
`GROUND_MU=0.8` "changed not one bit" claim. `SPEC.md` honesty rule 4 (every number carries
a machine id) is also violated by that section.

## 8. The metric MuJoCo would win does not exist

Erez's grasp-stability test — the largest timestep at which a grasped object stays in the
hand — measured **MuJoCo 16 ms vs ODE 0.25 ms, a ~64× advantage**, and the paper notes it is
*invisible at small fixed dt* because "all engines manage to complete the test at small
timesteps." That is a directly robotics-relevant property, it is the axis MuJoCo is
optimised on, and lane 1 has no equivalent.

## 9. What survives as a genuine Newton deficiency

| finding | status |
|---|---|
| T2 `stick_violation_max_m` 4.4e-04 vs 2.9e-05 (15×) | **real** |
| T6 `settle_creep_m_s` (~100×) | **real**, but see §6 — the published reason for it being non-comparable is wrong |
| T4 `energy_drift_rel` 0.381 vs 0.130 at matched dt | **real but mislabelled** — integrator truncation, and Newton 0.381 ≈ raw MuJoCo 0.387 |
| per-step cost | **real**, from [`step_cost`](step-cost-2026-08-06.md), not from lane 1 |

And separately — **the contract violations, which are the ones that actually matter**:
`WorldInfo.contactProperties` / `coulombFriction` ignored in ~202 worlds, statics intangible
by default, contacts invisible by default, `control.joint_f` inert under XPBD, a limit-less
motor silently ignoring `setPosition`. Those are **not** modelling differences; they are
product defects. **And they make the fidelity metrics unmeasurable**: you cannot compare
friction fidelity when the declared μ never reaches the contact.

## 10. Three questions were being conflated

- **(A) Is Newton modelling different physics?** Largely yes, and legitimately. §2, §3.
- **(B) Is Newton's implementation buggy?** Lane 1 found five real integration defects. That
  is its genuine value and it should keep doing it.
- **(C) Does Newton honour the `.wbt` contract?** Demonstrably not — and this is what blocks
  retirement, not (A).

Lane 1 currently reports a mixture of all three as one score.

## 11. Recommended changes

1. **Re-frame lane 1 in `SPEC.md` as a defect detector, not a leaderboard.** Add an honesty
   rule: *do not aggregate rows of different epistemic status into a win/loss count.*
2. **Build the Pareto `SPEC.md` already mandates.** The dt sweep and `wall_ms_per_step`
   already exist. ⚠ The cost axis must be rebuilt first: lane-1 wall numbers are taken
   through the controller IPC, fold each run's world-finalize into step 0, and were recorded
   at `--parallel 6` with n=1 per cell — ODE's `bounce` sweep reads 0.749 / 1.597 / **0.131**
   / 6.840 / 9.989 / 21.724 ms/step across dt 1→32, which is not physics. **The 0.131 cell is
   the single lowest value in that sweep and is the number quoted in the 24× headline.** Use
   [`step_cost`](step-cost-2026-08-06.md)'s differenced method instead.
3. **Re-scope or drop:** T2 `slide_accel` (per-angle), T2 `transition_angle` (real bisection
   or drop), T5 `angular_momentum_drift_rel` (drop, cite `_abs`), T6 `max_penetration_m`
   (drop, or re-scope to matched contact stiffness).
4. **Relabel T5/T7 momentum** as a coordinate-representation property with the Erez citation.
5. **Fix the coincident floor** in T1/T6 and equalise contact compliance before quoting T6.
6. **Add a max-stable-timestep-for-task metric** (Erez's grasp test) — the axis we currently
   cannot see.
7. **Report law-violation profiles** (Signorini / Coulomb / MDP / penetration) per Le Lidec,
   instead of a tally.
8. **Retract or reproduce the substeps section.** It has no data.

## 12. The larger objective finding

No published study shows analytic-scene accuracy predicts sim-to-real transfer. Acosta et
al. (RA-L 2022) fitted contact parameters for Drake, MuJoCo and Bullet against real impact
data and **no simulator won**; accuracy was limited by *"numerous model differences between
the real robot and the simulators."* The 2026 Annual Review synthesis, *The Reality Gap in
Robotics*, proposes a **Sim-to-real Correlation Coefficient** — does sim performance
*predict* real performance — and states outright that *"it is more important in practice to
reduce the sim-to-real performance gap than to reduce the reality gap."*

OmniSim has **zero** sim-to-real evidence
([`omnilink-sim-to-real.md:20`](../guide/omnilink-sim-to-real.md): *"No policy trained in
OmniSim has been validated on physical hardware"*). One real-hardware session would be
worth more than every remaining analytic scene.

Also unmeasured: **Newton's batching advantage has never been compared against ODE.** Lane 1
excludes it by construction (batch 1) and lane 2 has no ODE arm — 0 of 532 result rows pair
them. "Newton's case is batching" is currently prose.

> ⚠ **And it will stay prose in that form.** `bdc02139` deleted ODE, so the comparison this
> paragraph asks for is now **permanently unbuildable**. The claim did not become true by
> losing its counterfactual. What can still be measured is Newton-batched-GPU against
> Newton-CPU-sequential — which quantifies *the batching*, not *the engine choice*. State it
> that way, with K and the hardware named.
