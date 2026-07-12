# G1 Deterministic Brain — research log & coordination

> 📍 **Canonical G1 status:** [rl-current-state.md](rl-current-state.md) (the *G1 — detail*
> section) is the single source of truth. If a status claim here disagrees with that file,
> that file is right — fix this one.

**Goal.** Make the *physics* G1 walk as stably as the kinematic **ghost**, using a
**fully deterministic controller** — no RL, no learned weights. A single hand-written
"brain" that takes robot state in and produces joint targets out: a deterministic
analogue of a neural-net policy.

**This file is the coordination point.** Multiple Claude sessions / engineers work on
this in parallel. Before you start, read the *Module ownership* table and the *Research
log* so you don't redo something or collide. After a meaningful result, append a dated
entry to the *Research log*.

---

## 0. Why open-loop (the ghost) is not enough

`projects/policies/control/gait/g1_human_gait.py` is already a deterministic, foot-space gait model +
2-link IK. The **ghost** (`projects/policies/controllers/g1_ghost/g1_ghost.py`) plays it with
physics turned **off** (`staticBase TRUE`, kinematic `setPosition`). That is the only
reason it looks perfect — nothing can disturb it.

A real biped is **open-loop unstable**: it tips over (mostly *sideways*) every step.
Feed-forward playback has no disturbance rejection, so small deviations compound into a
fall. RL was supplying the missing feedback. The deterministic brain replaces that RL
with **hand-written, tuned feedback laws** that read the state every tick and correct the
reference. This is classical model-based legged control and it is known to work
(Raibert; Pratt capture point; Englsberger DCM).

**Key identity:** with all feedback gains = 0, the brain output **equals the ghost
reference**. Feedback is strictly *added on top*. So we can always diff "brain vs ghost"
and attribute the difference to a specific feedback term.

---

## 1. The plant (what we control — do not re-derive, it's measured)

| Property | Value | Source |
|---|---|---|
| Actuation | per-joint position PD, **kp=100, kd=5** | `g1_full_kp100.mjcf.xml` actuators |
| Control dt | **16 ms** (62.5 Hz) | trainer `DT=0.016` |
| Physics | 4 substeps × 4 ms | trainer `SUBSTEPS=4`, `PHYS_DT=0.004` |
| Spawn height | pelvis z = **0.78 m**, quat = upright | trainer `SPAWN_Z=0.78` |
| Actuated DOF | 13 legs+waist (brain) + 10 arms (held to a swing ref) | `LEGS_JOINTS`, `ARM_JOINTS` |
| Deploy-matched model | `projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml` | calibrated, deploy-faithful |

The brain outputs **position targets**; the kp=100/kd=5 PD turns them into torque. The
brain must keep the robot upright while tracking the gait reference, using only what a
real robot could measure (base IMU = orientation + ang-vel + lin-vel, joint encoders,
foot contact). It must **not** use absolute world (x, y) for control — only for logging.

---

## 2. The fast offline loop (the research instrument)

Iterate in **plain MuJoCo**, not OmniSim. Same MJCF, seconds per run, headless,
scriptable. OmniSim is only for milestone validation.

```
python projects/policies/control/g1_brain_sim.py            # default: canonical brain, 20 s, headless
python projects/policies/control/g1_brain_sim.py --controller openloop   # the ghost-in-physics baseline
python projects/policies/control/g1_brain_sim.py --render --seconds 30    # watch it
python projects/policies/control/g1_brain_sim.py --set k_cp_sag=0.6 k_ank_r=2.0   # override gains
python projects/policies/control/g1_brain_sim.py --sweep k_cp_lat=0.6,0.9,1.2,1.6  # 1-D sweep
```
(`g1_brain_sim.py` is THE harness; it targets the committed canonical brain
`projects/policies/control/g1_brain.py`. The old parallel `projects/policies/control/brain/` harness
+ scratch brain were consolidated away — there is one brain and one harness now.)

Reported metrics: distance walked (base x), survival time / fall time, mean |tilt|,
feet-clearance, and per-second status. **Fall = base z < 0.45 or |roll|>0.8 or
|pitch|>0.8.**

---

## 3. The brain interface (the contract — keep stable)

`projects/policies/control/g1_brain.py`, single file, internally sectioned by control module.
**Shipped by the controller session — this is the real, authoritative signature.** The
offline harness and (later) the OmniSim deploy controller both build `BrainState` from
what a real robot measures and call `step`:

```python
s = BrainState(
    roll, pitch, yaw,            # base orientation (rad)              — IMU
    roll_rate, pitch_rate, yaw_rate,  # body-frame angular velocity   — IMU
    vx, vy, vz,                  # pelvis world-frame linear velocity  — IMU/estimator
    bz,                          # pelvis height (m)
    joint_q, joint_qd,           # (13,) leg+waist, LEGS_JOINTS order
    t,                           # walk time since launch (drives the stride ramp)
)
leg_targets(13) = brain.step(s, dt)     # arms available as brain.arms (10,)
```

Gains live in `BrainConfig` (one dataclass, env-overridable via `G1_BRAIN_*`). The brain
depends only on `numpy` + `g1_human_gait`. Note `bz` (pelvis height) IS used for control
(height hold) — it is observable on the real robot from the base estimator, unlike
absolute x/y which must never enter a control law.

---

## 4. Module ownership (claim a module before editing it)

The brain is one file but cleanly sectioned. Each section is a feedback law with its own
gains. Claim one here (edit this table) so two sessions don't fight over the same law.

The brain `step()` applies its layers in order (see the docstring in `g1_brain.py`).
Claim a layer here (edit this table) before retuning/extending it.

| Layer | Where in `step()` | Responsibility | Gains | Owner |
|---|---|---|---|---|
| 0 feedforward | `ghg.targets_np(...)` | foot-space plan + IK → 13 targets (the ghost) | `vx,freq,style,ramp_s,hip_scale,a_lat,a_arm` | controller-session |
| 1 foot placement | `# 1.` | capture-point/DCM, swing-leg hip offset (sagittal+lateral) | `kfp_x,kdcm_x,kfp_y,kdcm_y,kroll_fp,fp_clamp` | controller-session |
| 2 ankle strategy | `# 2.` | stance ankle pitch/roll on tilt+rate+CoM-vel | `kp_ap,kd_ap,kv_ap,kp_ar,kd_ar,kv_ar,ankle_clamp` | _unclaimed_ |
| 3 posture/trunk | `# 3.` | stance-hip offsets to keep pelvis level | `kp_post_p,kd_post_p,kp_post_r,kd_post_r,hip_clamp` | _unclaimed_ |
| 4 height | `# 4.` | knee extension vs sag (`kk=0` off by default) | `kk,z_ref,knee_clamp` | _unclaimed_ |
| 5 phase adaptation | `# 5.` | slow the clock when off-balance | `phase_gate,phase_floor` | _unclaimed_ |

**Tooling ownership (non-controller):**
| Piece | File | Owner |
|---|---|---|
| Offline MuJoCo harness + metrics + sweeps | `projects/policies/control/g1_brain_sim.py` | harness-session (this doc's author) |
| Coordination doc + research log | this file | harness-session |
| OmniSim deploy integration (no-RL mode) | `projects/policies/research/controllers/g1_walk_deploy/` | **DONE** (controller-session) |
| OmniSim run/verify + launcher `run_g1_brain_deploy.ps1` | `projects/policies/research/runners/run_g1_brain_deploy.ps1` | omnisim-deploy-session |

**Coordination rules:**
- All tunable gains live in `BrainConfig`, env-overridable as `G1_BRAIN_<FIELD>`. Never
  hard-code a gain inside a law.
- The harness is the source of truth for "does it walk". Report distance/survival/tilt in
  the research log when you change a layer, so the next session sees the empirical effect.
- With all feedback gains 0 the brain MUST reduce to the ghost reference (the smoke test
  in `g1_brain.py __main__` checks this) — keep that invariant when editing layers.

---

## 5. Research log (append-only, newest at top)

### 2026-06-15 — MERGE reconciliation of the two parallel 2026-06-14 sessions
The CONSOLIDATION + STAND-sweep entries further below were written in parallel with
commit `7c3c538f` and merged here. One nuance to flag:
the CONSOLIDATION entry says the scratch `projects.policies.control.brain.*` effort was "never
committed" — that was true in its branch, but the *other* branch DID land those files
(commit `7c3c538f`, "land the stopped sibling deterministic-brain session's work").
So post-merge both exist: the canonical `projects/policies/control/g1_brain.py` AND the
scratch `projects/policies/control/brain/{g1_brain,g1_brain_mjsim,g1_brain_sim}.py` +
`projects/policies/research/runners/run_g1_brain.ps1` + `run_g1_openloop_baseline.ps1`. Per both entries the
scratch `brain/` files should be **retired** now that consolidation is done — left in
place pending maintainer sign-off (see the STAND-sweep entry's "Consolidation note").

### 2026-06-14 — REAL-TIME deploy via distillation (MPC → fast policy → OmniSim)
The MPC walks but is ~20x too slow for real-time: a plan rolls physics forward
H*4≈220 SEQUENTIAL steps (the 0.88 s horizon), ~346 ms even on GPU with graph
capture (`mujoco_warp`, K parallel worlds are free, but the time-chain can't be
parallelised — that's the recurrence). So real-time full-physics MPC is out.

**The real-time path is DISTILLATION** — the MPC as a *teacher*:
- `g1_distill.py` — shared obs (37-d) + tiny MLP + pure-numpy forward pass.
- `_scratch/distill_collect.py` — run the MPC, log (obs, residual) pairs.
- `_scratch/distill_train.py` — behavior-clone obs→residual.
- `g1_distilled_brain.py` — G1_BRAIN deploy module; numpy policy (~us) → REAL TIME.
- `g1_mpc_brain.py` — the (slow) MPC-in-deploy adapter (MuJoCo sidecar), for ref.

**What's PROVEN:** the distilled policy runs **real-time in OmniSim/Newton** —
step counter climbs 30→480 at speed (MPC was frozen at step 30 for 4 min), and
the controller is only **7 % of the control loop** (`ctrl%=7`); the sim is bottle-
necked by Newton+rendering, not the policy. The full pipeline (MPC teacher → net →
real-time Newton deploy) runs end-to-end; the brain load is confirmed via diag.

**What's NOT solved:** the walk is short (~1.2 s). Two causes: (1) the policy is
fragile — thin data (2 teacher trajectories, ~6k samples) → distribution-shift
drift (BC holds ~1.2-1.7 s even offline; a first DAgger round with exploration
noise BACKFIRED — chaotic near-fall labels diluted the clean walk); (2) a
MuJoCo→Newton transfer gap — it walks FORWARD in MuJoCo but BACKWARD in Newton
(residuals tuned to MuJoCo dynamics). Path to a robust real-time walker: more
diverse teacher data + careful DAgger (no over-exploration, several rounds), and
close the transfer gap (collect teacher data IN Newton, or domain-randomise the
teacher). Real-time ARCHITECTURE: done. Robust real-time WALK: open ML grind.

**Newton-gap diagnosis (followup):**
- The deploy obs is NOT the cause. Reproduced the deploy's joint-velocity estimate
  (finite-diff + low-pass, `_scratch/test_deployobs.py`) inside the MuJoCo harness:
  the policy still walks FORWARD (+0.59 m vs +0.42 m with true qvel), same ~1.25 s
  fall. So the forward→backward flip in Newton is a real **dynamics difference**
  between the engines (or a launch/phase-timing mismatch), not an observation bug.
- Deploy-infra obstacle: the OmniSim controller loads the distilled brain (and
  writes its diag) INCONSISTENTLY across runs (~50% over 4 tries; a stale `.pyc`
  also masked per-step logging) — so reliable deploy-side obs capture is blocked.
- **Fix plan (for next session):** (1) make the controller brain-load reliable
  (debug the G1_BRAIN env propagation / warm-up-reload diag); (2) **domain-randomise
  the teacher** (friction / mass / kp / gravity / contact solref per episode in the
  collector) + inject obs noise, so the policy is robust to the MuJoCo→Newton gap;
  (3) re-collect + retrain + redeploy. Collecting teacher data directly IN Newton is
  the gold standard but ~27 h at the deploy's ~8 s/tick (infeasible). Scratch rigs:
  `_scratch/distill_collect.py`, `dagger_collect.py`, `distill_train.py`,
  `test_distilled.py`, `test_deployobs.py` (all gitignored).

### 2026-06-14 — MODEL-PREDICTIVE BRAIN: the G1 walks FORWARD and far (breaks the ceiling)
The reflex/gain brain caps at ~2.3 s because it only **reacts** to the current
tilt. A deterministic **model-predictive** controller — `projects/policies/control/g1_mpc.py`
— breaks that wall by **predicting**. It is sampling-based MPC (MPPI) that uses
**MuJoCo itself as the predictor**: each 16 ms tick it samples K balance residuals,
rolls each ~0.9 s forward in a CLONE of the plant (all K in one batched
`mujoco.rollout` C call), scores uprightness + ghost-speed + no-drift, applies the
MPPI-weighted best, and re-plans (receding horizon). No learned weights; seeded ⇒
fully reproducible. It only commits to corrections a TRUE rollout shows keep it up —
exactly what analytic capture-point placement couldn't do on the compliant kp=100 legs.

**Results (offline harness, deploy-matched plant):**
- **balances 30 s+** ending perfectly upright (reflex brain: falls 2.3 s);
- **SUSTAINED FORWARD walking ~0.18 m/s**: +6.2 m in 35 s, **recovering from
  disturbances** (caught a 0.38 rad roll wobble at 26 s and kept walking). The clean
  module reproduces it (22 s OK, +3.7 m).

**Three things mattered (each measured):**
1. **Horizon H must exceed the step period** (~0.77 s, H≥45): a one-step lookahead
   balances in place but can't plan a stable forward gait (H=45 walks ~5 s, H=55
   sustains). The divergence timescale √(z/g)≈0.28 s is why a short horizon is blind
   to the multi-step runaway.
2. **iters≥2 (iterative MPPI refinement)** cuts sampling variance — low K/iters is
   seed-dependent (some seeds fall ~3 s, some sustain 15 s).
3. **ANTI-DRIFT cost (penalise lateral vy + heading yaw)** was THE fix for a "falls
   at ~18 s regardless of speed" wall: with only forward-speed in the cost, sideways/
   heading drift is unpenalised, accumulates, and tips it. Penalising it turned the
   18 s topple into a self-correcting limit cycle.

**Speed–stability tradeoff:** ~0.18 m/s (iters=2) is the sustained limit; pushing
speed (vx_target 0.45, vx_cap 0.7, or dropping to iters=1) walks faster (0.3–0.35 m/s)
but destabilises at ~14–23 s. Toward **+50 m**: a long run at 0.18 m/s (~280 s sim;
the MPC runs ~18× real-time, so ~90 min wall). The MPC is an offline/harness controller
(it rolls the model out at runtime); an OmniSim-deploy port would carry a MuJoCo model
alongside Newton purely for planning. Scratch search rigs: `_scratch/brain_mpc*.py`.

### 2026-06-14 — the DETERMINISTIC CEILING: exhaustive push for long walking (goal: +50 m)
Goal was to push the brain to +50 m. **Result: a deterministic linear-feedback
controller caps at ~2.3 s / +0.7 m FORWARD on this plant** — the committed
x0=-0.03 is already essentially the robust optimum. Everything below was tried in
the fast offline harness; every candidate was **robustness-checked across settle
0.30–0.50 s** to reject knife-edge flukes:

- Speed governor (dynamic x0 on velocity error): WORSE (gait discontinuity).
- Higher cadence (freq ↑): rolls over — lateral fails first.
- Sagittal capture authority (k_cp_sag, cp_sag_max ↑): **no measurable effect**.
- Hip-pitch PD posture (added rate damping kd_torso_p): maxed; cranking flips PITCH→ROLL.
- Lateral gains (sway, k_cp_lat, k_ank_r, k_wt): already at their knife-edge optimum.
- **TRUE capture-point foot placement** (leg-FK, land the swing foot AT the DCM
  relative to the stance foot): WORSE. On compliant kp=100 legs, aggressive
  absolute foot repositioning over-brakes (drifts backward) and destabilises
  (rolls). Only the gentle *stock additive* correction works — three independent
  attempts (touchdown-weight, absolute, sagittal-only) all regressed.
- Reactive stepping (speed the gait clock when diving forward): WORSE.
- Gait structure (duty ↑, taller CoM, shuffle): all WORSE; the default gait is optimal.
- **Automated joint-gain search** (600 evals, full ~19-D space): finds 3.6 s
  configs, but they are FRAGILE — collapse to 1.7–2.8 s (and drift backward) under
  a 0.05 s settle change. Rejected as near-fabrication.
- **Robustness-optimised search** (maximise *worst-case* survival over settle):
  best worst-case ~2.2 s — i.e. the committed x0=-0.03 IS the robust optimum.

WHY: the plant sits at the edge of static stability — no passive static stand exists
(finding #3), compliant ankles can't hold the toppling torque, and the divergence
timescale √(z/g)≈0.28 s ≪ the 0.77 s step period, so a single step can't catch the
fall and the aggressive foot placement that might isn't realisable on soft legs. This
is the **humanoid balance gap**: RL itself held only ~1.55 s in this deploy, and the
long-distance RL walk milestone of that era is ⛔ **RETRACTED** — it does NOT
reproduce (topples ~0.99 s). In this exact physics
*no* controller — deterministic or learned — currently walks far; the deterministic
brain (2.3 s forward) actually outlasts the RL deploy.

**+50 m (~125 s) is ~55× the robust deterministic ceiling — not reachable by a
hand-tuned controller on this plant.** Paths that could reach it: (a) a stiffer-ankle
plant (raise the toppling margin); (b) a learned residual TRAINED in this exact solver
(the "train-in-deploy-solver" milestone-1 fix in the humanoid-balance-gap notes), not
the old training-env policy; (c) event-driven whole-body model-based control. The
committed x0=-0.03 stands as the robust best. (Search rigs: `_scratch/brain_exp.py`,
`_scratch/brain_opt.py` — gitignored.)

### 2026-06-14 — CONSOLIDATION + the propulsion fix (finding #7): forward at last
**The several brain attempts are now ONE.** `projects/policies/control/g1_brain.py` is
the single canonical brain and the **default** the deploy harness loads — the
parallel `projects.policies.control.brain.g1_brain` scratch effort (never committed) is folded
in. Changes: deploy `G1_BRAIN_MODULE` default repointed to `...control.g1_brain`
(it pointed at the missing scratch module); the brain's `BrainState` rates are
now fed body-frame ω on all three axes in the deploy (was finite-diff Euler for
roll/pitch — the correctness note from the entry below, now fixed).

**The brain WALKS FORWARD now (it used to drift backward and tip).** The whole
effort was stuck because the IK feedforward has **no forward propulsion** — the
body drifts backward and topples (`ik` openloop/brain both fall BACKWARD here,
not forward as the scratch-brain finding #2 claimed). The fix was not push-off
or swing reach (both dead ends) but the **stance-foot stride center `x0`**:

| config | dist | survival | mean&#124;tilt&#124; | dir |
|---|---|---|---|---|
| old default (x0=-0.02) | **-0.53 m** | 1.79 s | 0.248 | backward |
| **new default (x0=-0.03, k_wt 0.6→0.4)** | **+0.72 m** | **2.32 s** | **0.161** | **forward** |

Shifting the stride center back by 1 cm puts the CoM ahead of the ankles, so the
stance slide propels the body forward. Robust across settle 0.30–0.50 s. (A
*decoupled* stand=-0.02/walk=-0.04 scores +0.83/2.64 but is a settle-timing
fluke — collapses at any other settle, so we ship the robust coupled x0=-0.03.)
The new wall is a forward **pitch** dive ~2.3 s — the inherent biped-instability
ceiling (RL deploy held only ~1.55 s), gated by the divergence timescale
√(z/g)≈0.28 s ≪ the 0.77 s step period. Next lever: faster corrective steps
(higher cadence, small stride) to beat 0.28 s; then a true leg-FK lateral DCM.

### 2026-06-14 — OmniSim STAND sweep confirms "PD-off" in the REAL engine + 2 gotchas (scratch-brain session)
Independently re-derived findings #3/#4/#5 directly in OmniSim/Newton (not offline),
which strengthens them — the destabilisation is real-engine, not a MuJoCo artifact:
- **Reactive joint-tilt PD destabilises the OmniSim stand at every sign/gain tried**
  (`stand_only`, deep-squat -0.30/0.52, KE=400). Feedback-OFF (pure pose) is strictly
  the most stable; adding PD always falls *sooner*:
  feedback-off **~2.0 s** (best) · +hip posture +1.0 → 1.60 s · +hip −2.5 → 1.41 s
  (overshoots *backward*) · +gentle ankle+hip sagittal → 1.10 s · +lateral roll PD → 0.75 s
  (roll kick = exactly the journey's "finite-diff roll_rate kicks ankle_roll" note).
  ⇒ confirms layers 2-3 should default OFF; balance must come from pose + foot placement.
- The feedback-off deep-squat tips **forward/sagittal** (roll stays 0.000 in OmniSim),
  matching faithful-harness finding #2 and contradicting my *scratch* `brain/g1_brain_mjsim.py`
  which showed a LATERAL topple. Cause: that scratch harness used MuJoCo defaults
  (`implicitfast`, default solref/iters) — **NOT faithful**. ⇒ heed §2: use the canonical
  faithful `control/g1_brain_sim.py`; an unfaithful harness invents a lateral instability.
- **GOTCHA for everyone:** `headless_runner.py --duration` is **WALL-CLOCK** seconds, not
  sim seconds. World+Newton+mjwarp init eats ~10-20 s, so a short `--duration` expires
  before the controller runs (looks like "survived, no telemetry"). Use ≥35 s for a stand.
- **Consolidation note:** the deploy default `G1_BRAIN_MODULE=projects.policies.control.brain.g1_brain`
  points at the *scratch* brain (weaker hip-angle foot placement); the effective one is the
  Cartesian `control/g1_brain.py`. Suggest flipping the deploy default to `control.g1_brain`
  and retiring the scratch `brain/` files (`g1_brain.py`, `g1_brain_mjsim.py`) +
  `projects/policies/research/runners/run_g1_brain.ps1` + `run_g1_openloop_baseline.ps1` once owners agree.

### 2026-06-14 — OmniSim/Newton cross-validation + the deploy path (omnisim-deploy-session)
- **The brain now runs in the REAL engine** (not just offline MuJoCo): the `G1_BRAIN=1`
  branch in `g1_walk_deploy.py` (controller-session) + launcher
  `projects/policies/research/runners/run_g1_brain_deploy.ps1` (full-body arms world by default; `-Legs`, `-Gui`,
  `-CpuEngine`). Tune via `G1_BRAIN_*` env, no code edits. rtf ≈ 5–6×, so Newton runs are
  cheap too — not just the offline harness.
- **Newton agrees with the offline harness** (so the fast loop is representative — good):
  default-gain brain falls **@1.38 s** in Newton/mjwarp (full-body) vs 0.78 s plain-MuJoCo,
  same forward-sag→topple class (Newton roll 1.68 only blows up after bz collapses, exactly
  as harness finding #2). ⇒ tune in the offline harness, validate milestones in OmniSim.
- **Height-hold (Layer 4 `kk`, was off) is a small sagittal trim, not the cure:** vx=0
  sweep → kk=4 best (stand 1.17→**1.46 s**, tilt 0.16→**0.10**), kk=8 over-corrects (drifts
  backward). Confirms finding #5 — planted-foot offsets only *delay* the topple. Leaving
  Layer 4 unclaimed; flagging kk≈4 as useful trim once Layer-1 foot placement (finding #6)
  is doing the real work.
- **Correctness note for controller-session:** the deploy builds `BrainState` with the
  finite-diff Euler-angle rates for roll/pitch but body-frame ω for yaw (inconsistent). The
  brain is tuned offline against body-frame ω (`d.qvel[3:6]`), so all three rates should be
  `ang_vel_body[0/1/2]`. One-liner in the `G1_BRAIN` block — left to you rather than edited
  live to avoid clobbering the hot controller file.

### 2026-06-14 — harness online + launch-fall diagnosis (harness-session)
Built `projects/policies/control/g1_brain_sim.py` (faithful: same MJCF, Newton solver 100-iter,
contacts solref[0.02,1], dt 0.004×4, friction — verified identical to the trainer). All
numbers plain-MuJoCo, settle 0.3–0.4 s, fall = bz<0.45 ∨ |roll|>0.8 ∨ |pitch|>0.8.

> **Provenance:** findings 1–5 are about the *plant physics* and are version-independent
> (re-confirmed on the canonical `control/g1_brain.py`: openloop **1.12 s**, default-gain
> **1.86 s** — its feedback nearly doubles survival). Finding 6 was measured on the earlier
> *scratch* `projects/policies/control/brain/g1_brain.py` and is **superseded** for the canonical brain —
> see the correction in #6.

**Six findings, in priority order:**
1. **Baseline confirms the thesis.** Open-loop (zero-gain brain = the ghost, but with
   physics) falls at **0.85 s / 0.49 m**. Default-gain brain: **0.78 s / 0.55 m** (its
   lateral gains currently make it slightly *worse*). Zero-gain invariant holds.
2. **The launch fall is SAGITTAL (forward face-plant), not lateral.** Pitch climbs
   monotonically `+0.02→+0.13→+0.48→+1.07` while vx runs away `0.13→1.77`; roll only
   explodes *after* bz has already collapsed. Divergence timescale ≈ 0.28 s = the
   inverted-pendulum constant √(z/g). **Fix sagittal before lateral.**
3. **No static stand pose exists.** Held passively (flat-foot), walk_nom falls fwd @1.1 s,
   deep-squat(-0.30/0.52) @1.6–1.8 s, deeper(-0.40/0.74) @3.4 s, very-deep tips *backward*.
   Reason: ankle stiffness 2·kp=200 N·m/rad is right at the toppling margin m·g·h≈172 with
   the CoM ~0.5 m up; compliant legs (kp=100) let the body fall through. The earlier note's
   "deploy pose stands 15 s in plain mujoco" is **NOT reproducible** in faithful physics —
   RL was always actively balancing; there is no passive static stand.
4. **Ankle-pitch PD makes it WORSE.** Sweeping kp_ap −1.5→−12 *flips* the fall fwd→back
   and stronger gain falls *faster* (compliant-leg coupling; matches the earlier finding that
   ankle PD destabilised the RL deploy). Ankle is not the balance lever here.
5. **Hip/posture PD barely helps** — only pushes the fall 1.66 s→1.9 s; pitch still
   diverges. With planted feet, the only horizontal CoM force is CoP shift, capped by the
   0.17 m foot. **Planted-foot joint offsets fundamentally cannot balance this G1.**
6. **THE BUG TO FIX: the current foot-placement law has no measurable effect.** A sweep of
   freq{1.3,1.8,2.4} × fp_clamp{0.3,0.6,0.8} × kdcm_x{0.2,0.4,0.6} × ramp{0.5,2.0} gives
   the **identical** result every cell: 1.39 s / +0.70 m. The robot topples the same way
   regardless of how it's told to step → Layer-1 (`# 1.`) is not actually repositioning
   the foothold under the falling CoM. Likely causes: offset applied as a swing-leg
   *hip-angle* gated by swing weight engages mid/late-swing (too late) and is too small
   (÷L_LEG=0.6); the IK foot target should be shifted *before* IK, and the placement must
   target the **measured-velocity capture point** (ξ = x_com + v/ω ≈ +0.43 m at v=1.5),
   well beyond the nominal 0.31 m stride and the 0.18 m the clamp allows.
   **CORRECTION (canonical brain):** the committed `control/g1_brain.py` already does much
   of this — its capture-point layer (`k_cp_lat`/`k_cp_sag`, Cartesian) is *effective*
   (openloop 1.12 s → default 1.86 s). So finding 6's "no-op" applies only to the scratch
   `brain/` version. The remaining gap on the canonical brain is the **launch**: it still
   tops out ~1.9 s. Next lever there = capture authority + reach at launch (still the right
   direction), not a rewrite.

**Recommended direction (for the controller session — your call):** make Layer-1 the
*dominant* balance term and physically correct — shift the swing-foot **Cartesian target**
toward the capture point and re-run IK, with enough authority (≥0.4 m reach) and engaged
from early swing; treat ankle/hip as fine trim only. Then the launch can't be a static
hold — it must step in place immediately and let foot placement catch the CoM. Harness is
ready to sweep whatever you wire up: `--set`, `--sweep`, `--render`.

### 2026-06-14 — kickoff
- Established the framing above. Built `g1_brain_sim.py` (offline harness) and
  `g1_brain.py` v1 (feed-forward reference + capture-point foot placement + ankle CoP +
  lateral hip-roll + height hold). Baselines + first walk results recorded below as they
  land.

<!-- Append new dated entries here. Format:
### YYYY-MM-DD — <one-line headline>
- what changed, the numbers (distance/survival/tilt), what it means, next lever.
-->
