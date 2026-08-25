# Ghost Construction — State of the Art Survey & Improvement Roadmap

> **Scope:** how Shadowing builds the dynamically-feasible reference ("the ghost") today, a
> verified survey of better methods, and a ranked, low-risk roadmap. Companion to
> [shadowing.md](shadowing.md) (the method) and [ghost-tracking-pipeline.md](ghost-tracking-pipeline.md).
> Produced from a multi-agent code+literature audit (2026-06-23); external claims web-verified.

## 1. How the ghost is built today (verified against code)

Four distinct construction methods exist; **MPPI is the workhorse**:

| Method | What | Used by | Feasibility |
|---|---|---|---|
| **Receding-horizon MPPI** ([ghost_generator.py](../../projects/policies/research/shadowing/ghost_generator.py)) | Predictive-sampling MPC over MuJoCo, control = position-target actuators | G1 sit-stand/stool/standwave, B2 get-up, Go2 walk, hill `--mppi` | by construction (soft) |
| Analytic foot-space gait + 2-link IK ([g1_human_gait.py](../../projects/policies/control/gait/g1_human_gait.py)) | LIPM/Winter/achieved/human sub-styles | G1 walk, quad trots | kinematic only |
| Kinematic FK replay on terrain ([hill_terrain.py](../../projects/policies/research/shadowing/hill_terrain.py)) | Analytic gait evaluated on an incline | hill-walk (B2/Go2/OmniQuad) | FK check only |
| Visual-only ghost URDF ([make_ghost_urdf.py](../../projects/robots/unitree/g1/urdf/make_ghost_urdf.py)) | Strips inertia/collision → physics-free hologram | demos | n/a (display) |

Notes confirmed in the audit: there is **no human-mocap retargeting** (the "mocap" is digitized
Winter normative curves baked as constants); `g1_sitstand_trajopt.py` is **named** trajopt but is
gradient-free MPPI; true gradient TO is a stub aspiration.

### The MPPI generator's structural weaknesses (code-grounded)

1. **All constraints are soft penalties** — torque/friction/joint-limit/balance/fell-over are
   additive costs or a `np.clip` to joint *range*; nothing is ever hard-rejected. "Feasible by
   construction" means only "the sim didn't object on this seed." The word *certified* is not yet earned.
2. **`effort` and `smooth` cost weights were dead code** — declared in the default weight dict but
   never read by either cost function. There was **no** smoothness or effort regularization at all.
   (Fixed — see §4.)
3. **Per-timestep white noise + AR filter** — control sampled independently per step (search dim =
   horizon × n_dof); chatters, sample-inefficient.
4. **Deploy runs a MuJoCo solver — gated by config, not XPBD (CORRECTED 2026-06-23).** Generator
   (MuJoCo CPU C lib), trainer (`mujoco_warp` GPU), *and legged deploy* all run the MuJoCo contact
   model: deploy selects `newton.solvers.SolverMuJoCo` whenever `WorldInfo.newtonSolver
   "mujoco"/"mujoco_warp"` or `OMNISIM_NEWTON_FORCE_MUJOCO` is set (the RL deploy path,
   [g1_deploy_runtime.py:1305-1380](../../projects/policies/research/backends/g1_deploy_runtime.py#L1305-L1380));
   only the *default* Newton path is XPBD. So the solver-*family* gap is NOT real for correctly-
   configured legged worlds — and for G1, trainer↔deploy physics is already golden-verified
   ([g1-single-source-of-truth.md](g1-single-source-of-truth.md): a compiled-MjModel field diff →
   0 real-physics gaps). The REAL residual gaps are: (a) a **config gap** — an RL world that forgets
   the flag/env silently falls to XPBD → a `mujoco_warp`-trained policy collapses (the documented
   "long-running G1 deploy gap", [OmNewtonBackend.cpp:356-363](../../src/omnisim/physics/OmNewtonBackend.cpp#L356-L363));
   (b) a small **implementation gap** CPU `mj_step` vs `mujoco_warp` (mitigate with
   `OMNISIM_NEWTON_MJWARP=1` to match the trainer exactly); (c) the generator's hand-built MJCF vs
   the deploy URDF→Newton-converted model params (solref from ke/kd, friction, substeps).
5. **Verifier rigor (not engine-faithfulness) is the gap** — the certificate
   ([feasibility_certificate.py](../../projects/policies/research/shadowing/feasibility_certificate.py)) is a
   per-step contact-wrench LP in MuJoCo CPU. Since deploy is *also* a MuJoCo solver (§4), this is the
   *same contact family* as deploy — closer to deploy-faithful than first claimed; the residual is
   CPU-vs-`mujoco_warp` + model-source, not solver family. The real weakness is rigor: ZMP/
   capturability are reported but **not gated**; friction is a pyramid not a cone; the scalar score is
   weakly informative.

## 2. What the literature says (web-verified)

- **Differentiable-sim TO — ruled out for this stack.** MuJoCo-Warp (Newton's backend) is *not*
  differentiable today (MJWarp docs / mujoco_warp#500); and stiff-contact gradients are biased/
  high-variance (Suh et al., ICML 2022). MJX-JAX is differentiable but is not our deploy stack. Stay sampling-based.
- **Learned/diffusion priors (AMP/ASE/CALM, MDM, PhysDiff, PARC) — complementary, not a replacement.**
  Every mature method derives feasibility from a *downstream physics stage* (exactly our MPPI+verifier);
  AMP-style priors *delete* the explicit reference and the certificate. Data-hungry, training-unstable.
  Use only as a diversity front-end if motion *variety* ever becomes the binding constraint.
- **Spline/knot-point control parameterization — the consensus low-risk upgrade.** Shared foundation of
  MuJoCo MPC (arXiv:2212.00541) and DIAL-MPC (arXiv:2409.15610). Cuts search dim from horizon→knots,
  intrinsically smooth. Caveat: knot density must preserve sharp contact impulses.
- **DIAL-MPC** (ICRA 2025) reports **13.4× lower tracking error than MPPI** via diffusion-style noise
  *annealing* + horizon-scaled noise. Caveat (de-hyped): that number is for **online** torque control,
  not offline ghost generation; off-label as a generator but its annealing ideas port cheaply.
- **Gradient-based phased TO (Crocoddyl/iLQR-over-MuJoCo + ALTRO augmented-Lagrangian)** beats MPPI on
  smoothness/sample-efficiency/hard-constraint satisfaction *for known gaits*; full contact-*discovery*
  TO (Posa/CIO/Drake) imports convergence fragility we don't need (our gaits are known). Best fit:
  iLQR over our exact MuJoCo dynamics, warm-started by MPPI, AL outer loop for hard limits.
- **Reduced-order centroidal/SRBD first stage** (Di Carlo 2018; Dai-Valenzuela-Tedrake; BiConMP) gives a
  fast, momentum- & contact-feasible reference *by construction*, then refine whole-body.
- **Rigorous feasibility certificate:** add **contact-wrench-cone** feasibility + **ZMP/CoP-in-support** +
  **DCM/capture-point N-step capturability** (Caron; Koolen; Englsberger) — all cheap from a MuJoCo rollout.
- **Human-mocap retargeting front-end** is the dominant humanoid-skill approach (ExBody2, H2O/OmniH2O,
  HumanPlus, PHC, KungfuBot, OmniRetarget). The field has converged on *retarget → project-to-feasibility →
  RL-track* — i.e. our MPPI+verifier **is** the projection stage everyone bolts on; we lack only the
  retarget front-end. GMR (github.com/YanjieZe/GMR) + LocoMuJoCo AMASS/LAFAN1 G1 retargets are off-the-shelf.

## 3. Ranked roadmap (cheap → ambitious)

1. **[DONE] Spline-parameterized MPPI + activate the dead smooth/effort costs** (§4). Opt-in, ~50 LOC.
2. **[DONE] Deploy-solver config guard + world pins (REFRAMED 2026-06-23).** *Not* a foreign-engine
   replay — deploy is already a MuJoCo solver (§4). Shipped: (i) a **runtime require-guard**
   `OMNISIM_REQUIRE_MUJOCO_SOLVER=1` ([g1_deploy_runtime.py](../../projects/policies/research/backends/g1_deploy_runtime.py))
   that fails loudly if a deploy world resolves to XPBD — analogous to `OMNISIM_REQUIRE_NEWTON`, one level
   deeper; (ii) a **lint** [check_deploy_solver.py](../../scripts/dev/check_deploy_solver.py) (`--fix`,
   CI-able) that found **66 legged Newton worlds with no `newtonSolver "mujoco"` pin** (the "baked in the
   world" migration was never done) and pinned them all. Verified: `g1_smoke.omniworld` now logs
   `solver_kind=MuJoCo (… WorldInfo.newtonSolver)` headless with no FORCE_MUJOCO env. **Still open:**
   generalize the G1-only golden trainer↔deploy parity ([g1_golden_parity.py](../../projects/policies/research/training/g1_golden_parity.py))
   to every robot/ghost (covers the model-source gap (c)).
3. **Rigorous certificate metrics** — add CWC + CoP-in-support + DCM N-step capturability to
   `feasibility_certificate.py` (gate, don't just report). *Medium.*
4. **Hard sample-feasibility** — clamp torque/joint samples to box limits + reject friction-cone violators
   inside the sampler, so "certified" is earned. *Cheap.*
5. **Human-mocap retargeting front-end** (GMR/LocoMuJoCo → KungfuBot-style filter → our MPPI/verifier →
   RL). The lever for the maintainer's "any motion, any robot" north star. *Larger.*
6. **iLQR-over-MuJoCo + ALTRO** gradient polish, MPPI warm-started — hard-constraint-satisfying references.
   *Larger; revisit if #1 isn't enough.*

Explicitly **not** recommended now: differentiable-sim TO (stack + contact-gradient blockers); learned
generative priors as a *replacement* (abandons the feasibility certificate).

## 4. What was prototyped (opt-in, behavior-preserving)

[ghost_generator.py](../../projects/policies/research/shadowing/ghost_generator.py):
- `generate(..., n_knots=N)` — knot-spline control parameterization (`_spline_basis`); default `None` =
  unchanged per-timestep path.
- `effort` / `smooth` weights wired into `_rollout_cost` (effort = target excursion from rest; smooth =
  control rate). Defaults set to **0.0** so prior behavior is byte-identical (verified: the `_demo` still
  ends at `base_z=0.756`). Opt in via `weights={"smooth": ...}`.

A/B on the G1 hold-squat ghost ([bench_spline_ghost.py](../../projects/policies/research/shadowing/bench_spline_ghost.py)):

| config | K | ctrl_rate | jerk | tilt_final | tilt_max |
|---|---|---|---|---|---|
| A vanilla | 48 | 0.570 | 0.906 | 9.34° | 14.52° |
| **B spline+smooth (knots=5)** | 48 | 0.313 | **0.354** | **5.44°** | **9.13°** |
| C spline+smooth (knots=5) | **24** | 0.506 | 0.604 | 4.43° | 13.10° |

At equal compute the spline+smooth ghost has **61% less jerk and tighter balance**; at *half* the sample
budget it is still smoother than vanilla. Next: wire `n_knots` into the production `generate_*.py` scripts
and tune knot count per motion (more knots for contact-rich initiations).

### Deploy-solver guard + world pins (roadmap #2 — shipped)
- **Runtime guard** `OMNISIM_REQUIRE_MUJOCO_SOLVER=1` — raises at world load if the Newton solver resolves
  to XPBD (default unset → no behavior change). Added to the **source-of-truth** `kNewtonRuntimeSource` in
  [OmNewtonBackend.cpp](../../src/omnisim/physics/OmNewtonBackend.cpp) and re-extracted to
  [g1_deploy_runtime.py](../../projects/policies/research/backends/g1_deploy_runtime.py) via `_gen_deploy_runtime.py`
  (drift-guard pytest green). *Takes effect after a C++ rebuild; the world pins below already work on the
  current binary.* Wired into the **23 legged deploy run-scripts** (`scripts/dev/run_*_deploy*.ps1`) next to
  the existing `OMNISIM_REQUIRE_NEWTON` assertion.
- [check_deploy_solver.py](../../scripts/dev/check_deploy_solver.py): lints/repairs the `newtonSolver "mujoco"`
  pin on RL/legged Newton worlds. **66 worlds pinned** (`66 files changed, 66 insertions(+)`); lint now 71/71.
- Root cause: deploy run-scripts forced MuJoCo via `OMNISIM_NEWTON_FORCE_MUJOCO`, but the worlds weren't
  self-describing — loading one outside its script (GUI/harness/headless) silently fell to XPBD → a
  `mujoco_warp`-trained policy collapses (the documented "long-running G1 deploy gap").
