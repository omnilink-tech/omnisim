# The Ghost Method — OmniSim's standard RL recipe

**Status: CANONICAL (research writeup + standard spec). Supersedes the
2026-05-29 "two-layer control architecture" proposal; this is the realized
form, named.** Updated 2026-06-16.

> **One sentence.** Every OmniSim robot is taught to move by building a
> physics-free **kinematic reference** of the ideal motion — the **ghost**
> (a.k.a. the **shadow**) — and then training a tightly-bounded controller to
> **mimic it under real physics**, with the learned part trained *inside the
> deploy solver* so it transfers.

The Ghost Method is OmniSim's answer to the sim-to-deploy gap. It is the same
idea as **Residual Policy Learning** (Silver et al. 2018; Johannink et al.
2018) — `π(s) = π_base(s) + f_θ(s)` — but organized around three named pieces
that generalize across the whole zoo (G1, OmniQuad, Atlas, and any robot we
add next):

1. **The Shadow** — a hand-built, physics-free *kinematic* model of the ideal
   gait/motion (`projects/policies/control/gait/*`). It never falls because it ignores
   physics; it defines *what good looks like*.
2. **The Ghost** — that shadow, *rendered* as a translucent second robot beside
   the real one (`projects/policies/controllers/*_ghost/`), phase-locked to the real
   robot's forward progress. The visible gap between ghost and robot **is** the
   tracking error + sim-to-deploy error, made watchable.
3. **The Mimic** — the deployed controller that drives the *physical* robot to
   follow the shadow: a deterministic baseline (gait + IK + balance) plus a
   **bounded learned residual** trained in the deploy-faithful solver.

This document is both the **research narrative** (everything we tried, in
order, including the dead ends and the MPC line) and the **standard contract**
(how to apply the method to any robot). It is the doc all per-robot RL work
should converge to. Live per-robot status: [rl-current-state.md](rl-current-state.md)
and [rl-journey.md](rl-journey.md). The MPC deep-dive lives in
[g1-mpc-deterministic-brain-research.md](g1-mpc-deterministic-brain-research.md).

---

## 1. The premise — why "build the ideal, then mimic it"

We arrived at the Ghost Method by exhausting the alternatives. The premise rests
on three findings that are now settled and non-negotiable:

**(a) Pure end-to-end RL does not transfer.** Heavy domain-randomized PPO trains
a G1 to stand at ≈98 % in-sim but the Newton deploy topples at ~1.55 s — and
*more* randomization deployed *worse*. Atlas PPO converged to μ≈0 (a
near-saturating analytic baseline starved the gradient). From-scratch PPO on
OmniQuad found the stand-still local optimum (zero reward, zero risk) after 500 k
steps. End-to-end learning either does not transfer or does not learn.

**(b) A good hand-model gets you most of the way — and the learned part then
has little left to do.** OmniQuad's hand-coded gait+IK+balance walks **+5.03 m**
straight with no policy; the learned residual on top walks **+4.87 m** — a
near-no-op "passenger." This is exactly what the residual-RL literature predicts:
*"if the initial policy is perfect, the residual should have no influence."*

**(c) The unifying law:** **learned value scales inversely with how complete the
deterministic baseline is on the training distribution.** So a residual cannot
be what makes a biped stand — a deterministic layer must — and the residual only
earns its keep where the baseline is *provably* incomplete: the unmodeled
regime (engine dynamics drift, contact error, external pushes), never the
nominal task.

> **Sharpened 2026-06-23 — "passenger" is the *benign* end; the malign end is
> "saboteur."** Law (c) says a residual on a complete baseline earns *nothing*.
> The OmniQuad-walk result (§7.1) is the gentle version of that: the residual is a
> harmless **passenger** (+5.03 m → +4.87 m). But on a **marginal-stability
> static task** — a 2-contact inverted pendulum held at its stability edge — a
> complete baseline makes the correction layer **net-negative, not net-zero.** It
> doesn't just fail to help; it actively topples a robot that was standing. The
> full mechanism + the stand-and-hold-cubes evidence is **§3.8**.

The Ghost Method operationalizes (a)–(c):

- Put almost all of the competence in a **deterministic, inspectable** layer
  (the shadow + a feedback law that mimics it). This is a *controls* problem,
  not a learning problem, and it ships even with zero RL.
- Keep the learned layer **small, bounded, and honest** — its only job is the
  unmodeled regime, and we hold it to a *measured* delta over the baseline.
- Train that learned layer **in the deploy solver itself**, because the one gap
  nothing else closes is the engine-dynamics gap between the trainer and Newton.

What we are "making our own" is **not** a learning algorithm. It is (i) the
ghost/shadow/mimic interface, (ii) the discipline of training the residual
inside the deploy solver, and (iii) the honesty bar: the residual is justified
only by a measured improvement over the bare baseline.

---

## 2. Terminology (use these words consistently)

| Term | What it is | Where it lives |
|---|---|---|
| **Shadow** | The physics-free *kinematic gait model* — the ideal trajectory the robot should produce. Foot-space planning + closed-form IK; never falls. | `projects/policies/control/gait/g1_human_gait.py`, `omniquad_trot_gait.py` |
| **Ghost** | The shadow *displayed* as a translucent hologram robot (staticBase, no physics) beside the real one, phase-locked to its forward progress. A debugging/demo lens: the gap = the error the mimic must close. | `projects/policies/ghosts/g1/`, `omniquad_ghost/` |
| **Mimic** | The controller on the *physical* robot that tracks the shadow: deterministic baseline + bounded learned residual. | `projects/policies/research/controllers/g1_walk_deploy/`, `omniquad_residual_deploy/` |
| **Baseline (Layer 1)** | The deterministic part of the mimic: shadow feedforward + IK + a *feedback* balance law. Ships on its own. | `projects/policies/control/g1_brain.py`, `omniquad_gait.py`+`omniquad_balance.py`+`omniquad_kinematics.py` |
| **Residual (Layer 2)** | The learned part: a tightly-bounded additive trim on top of the baseline, zero-initialized, trained in the deploy solver. | ONNX policy loaded by the deploy controller |
| **Imitation / "follow-the-shadow" reward** | The RL reward term that pays the policy to keep the robot's actual pose on the shadow. | `gpu_mjwarp_g1_walk_trainer.py` |

"Ghost" and "shadow" are used interchangeably in older notes; going forward,
**shadow = the model, ghost = its on-screen rendering.**

---

## 3. The research narrative — everything we tried

This is the honest chronology. The dead ends are kept on purpose: they are why
the Ghost Method is shaped the way it is.

### 3.0 From-scratch PPO (the null result, 2026-05-24)

Vanilla 500 k-step PPO on OmniQuad converged to *stand still* — the zero-reward,
zero-risk local optimum. Lesson: **we already know how these robots should
move; encode it.** This produced the first residual recipe (gait+IK+balance +
±3 cm learned foot trim): 20 k steps in **52 s** vs failure at 500 k.

### 3.1 Heavy-DR pure PPO for standing (2026-05-28) — *the canonical sim-to-deploy gap*

Added the full DR cocktail (±30 % per-body mass, ±50 % friction, ±50 % damping,
**±40 % actuator kp/kv**, **0–3 tick action latency**, gravity jitter, pushes,
obs noise). G1 stands indefinitely in the mjwarp trainer (99.6 %) but **1.55 s
in OmniSim**, and *more* DR deployed *worse*. Matching the deploy wrapper
(solver match, MJCF match, obs-source match — five iterations) did not close it.

The real cause was found **outside RL**, by reproducing the passive NOMINAL pose
(no policy) in plain MuJoCo: it *also* tipped at ~1.5 s. The URDF importer placed
the foot ~35 mm back, putting the CoM ~5 mm *ahead* of the foot front — a
divergent inverted pendulum, and the ankle PD destabilized roll. **Fix
(f48f00b7):** deeper squat (hip −0.30 / knee 0.52) + ankle PD **off** by
default → G1 stands indefinitely in pure pose (verified in plain MuJoCo and
mujoco_warp). The "deterministic-first" layer earned its place here.

**Settled truth #1:** *deterministic-first ≠ a passive pose hold.* A static
squat under the exact deploy solver is an unstable pendulum. Layer 1 must be an
**active closed-loop balance law**, not a setpoint chased through a stiff PID.

### 3.2 Atlas — the honest negative result (2026-05-28)

Ported the G1 recipe to 30-DOF Atlas. PPO **never learned anything**: trained vs
zero-action median survival was identical (41 vs 41 steps). Two findings: (i)
mass DR in mujoco_warp is per-*run*, not per-*env* (a seed lottery, dropped);
(ii) a near-saturating analytic baseline (88 % reward at iter 1) starves the
gradient. Shipped as a documented negative result — it is the sharpest evidence
for law (c).

### 3.3 G1 walking — the long thread (2026-05-28 → 2026-06-14)

- **The 20× stiffness mismatch.** Early walk policies fell at ~1.7 s; the cause
  was *not* "6 cm feet are the wall" but a **joint-stiffness sim-to-deploy
  mismatch** (trainer kp=20 vs deploy KE=400). Stiffness-matched MJCF (kp=100) +
  ankle counter-rotation + foot-contact rewards → **gpu_g1_walk15: +25.9 m, zero
  falls** (10f7d989). *Rule learned: always diff the deploy-dump actuator gains
  against the trainer MJCF first.*
- **Foot-space shadow.** The joint-space sine CPG was replaced by the foot-space
  **human gait model** ([`g1_human_gait.py`](../../projects/policies/control/gait/g1_human_gait.py)):
  Cartesian planning + closed-form IK, calibrated to 0.00 mm. This is the modern
  shadow the residual rides on.
- **Four stacked deploy-gap bugs.** The "trainer-walks, deploy-dies" mystery
  was, by instrumentation (not guessing), *four* bugs at once: (1) launch at a
  mid-swing phase (start at double-support instead); (2) gravity-sag rest-starts
  out of distribution; (3) a **tanh-vs-clamp ONNX export** bug weakening
  mid-range actions up to 24 % (latent since the stand trainer); (4) unplumbed
  njmax/nconmax → contact overflow → explosion. Fixing all four reached the long
  walks. *Rule learned: instrument the trainer-vs-deploy obs diff; select
  policies by deploy samples, not trainer reward.*
- **Style (gpu_g1_walk25/26/27).** Reward shaping (measured, not eyeballed) for
  foot clearance, knee double-bend, swing height, contact timing, ankle push-off
  — a natural, human-looking gait (see §5).
- **Stop-in-the-middle (gpu_g1_walk29_vc).** A velocity-conditioned policy that
  launches from stand, accelerates, *and decelerates back to a stand* on command
  — no second policy.

> ⛔ **RETRACTION (2026-06-14, hardened 2026-06-19):** the headline long-distance,
> zero-fall G1 walk figures of this era are **RETRACTED as unreproducible** — they
> were old-deploy-path numbers and are **not** a result. The shape-c8 milestone they
> belonged to **topples sideways at ~0.99 s** on the current Newton deploy.
> **A different, later policy DID close the deploy gap, but only to a
> FINITE bout:** `gpu_newton_g1_walk_ft_pdoff_clamp` walks **+5.9 m / 33.8 s** in the
> real Newton deploy (`9b6df709`, after the silent-XPBD-fallback fix `cbe5e6f0`) —
> then falls; a **durable ≥80 %-ghost walk is OPEN.** True-headless deploy can crash
> (code 1); use the GUI and set `G1_DEPLOY_LOG` for telemetry. The fragility traces
> to the same engine-dynamics gap discussed in §3.5/§8, not to the gait model.
> Canonical status: [rl-current-state.md](rl-current-state.md); see also
> [g1-walk-deploy notes](../../projects/policies/research/controllers/g1_walk_deploy/).

### 3.4 The deterministic brain — reflex → capture-point → the 2.3 s ceiling

In parallel we built a *fully hand-written* mimic
([`g1_brain.py`](../../projects/policies/control/g1_brain.py)): LIPM + capture point.
The shadow is the feedforward (ideal foot trajectories); the brain adds the
feedback the balance RL had been approximating — ankle CoP, lateral weight
transfer, posture, capture-point foot placement, adaptive phase.

It hit a **~2.3 s ceiling.** A purely *reactive* law (proportional to the
current tilt) is structurally blind to a multi-step runaway: the
inverted-pendulum divergence timescale √(z/g) ≈ 0.28 s is far shorter than the
~0.77 s gait step period. Aggressive analytic capture-point placement
over-braked and rolled the soft-ankle robot (three independent attempts
regressed).

**The one transferable win (Finding #7, 2026-06-14):** *forward drive is a
**stance-foot placement** problem, not push-off.* Shifting the stride center
*back* (`x0` −0.02 → −0.03) puts the CoM ahead of the ankles so the stance leg
propels the body forward: −0.53 m (backward!) → **+0.72 m forward**, survival
1.79 → 2.32 s. The missing ingredient was still **lookahead** — which led to
the MPC.

### 3.5 The MPC / MPPI trials — "predict, don't react" (2026-06-14)

To break the reactive ceiling we built a **deterministic, sampling-based MPC**
([`g1_mpc.py`](../../projects/policies/control/g1_mpc.py)) that uses **MuJoCo itself as
the predictor**. Every 16 ms tick (full method + cost table in
[g1-mpc-deterministic-brain-research.md](g1-mpc-deterministic-brain-research.md)):

1. snapshot the true plant state;
2. sample `K=100` balance-residual vectors around a warm-started nominal —
   acting on **8 balance joints** (hip pitch/roll + ankle pitch/roll, both legs)
   **on top of the same shadow** the brain and the RL policy use;
3. roll each candidate `H=55` steps forward in a cloned plant (one batched,
   threaded `mujoco.rollout` call);
4. score (uprightness + forward-speed tracking + **anti-drift** + height +
   effort; early falls penalized by how early);
5. MPPI-average by `exp(−cost/λ)`, apply the first action, advance, re-plan.

**Results (offline harness, deploy-matched plant):**

| controller | distance | survival |
|---|---|---|
| reflex/gain brain | −0.53 … +0.72 m | ~2.3 s |
| **MPC (MPPI, H=55, K=100, iters=2)** | **+6.2 m in 35 s** | sustained; recovers from a ~0.38 rad roll wobble |

Three measured insights, in priority of transfer value:

1. **Lookahead works.** Predicting (rolling true physics forward) beats reactive
   PD on this plant. `H` must exceed the gait step period (H≥55 ≈ 0.88 s).
2. **The anti-drift cost (`w_vy`, `w_yaw`) is the single most transferable
   insight.** Penalizing lateral + heading drift turned a "falls at ~18 s
   regardless of speed" wall into a **self-correcting limit cycle**. This maps
   directly onto an RL reward term.
3. **`iters ≥ 2`** (iterative MPPI refinement) makes survival seed-independent.

### 3.6 Distillation — the real-time path that hit a wall (2026-06-14)

Full-physics MPC plans by rolling H·4 ≈ 220 **sequential** steps per tick
(~346 ms): the K worlds parallelize for free on mujoco_warp, but the time-chain
is a recurrence that cannot — so it is **~20× too slow** for real-time deploy.

The intended fix was **distillation** — the MPC as a *teacher*. We built a shared
37-d observation, a tiny (256,256) MLP, and a **pure-numpy** forward pass
(microseconds/tick), then behavior-cloned `(obs → residual)`
([`g1_distill.py`](../../projects/policies/control/g1_distill.py),
[`g1_distilled_brain.py`](../../projects/policies/control/g1_distilled_brain.py)).

**What worked:** the distilled net runs **real-time in OmniSim/Newton** (~7 % of
the control loop; the sim is bottlenecked by Newton + rendering, not the
policy). The whole pipeline runs end-to-end. *The real-time architecture is
sound.*

**What stalled — the negative result that decided the program (be6f46c8):** a
**MuJoCo→Newton transfer gap.** The distilled residual, tuned to MuJoCo
dynamics, **walks forward in MuJoCo but backward in Newton.** The observation
pipeline was *ruled out* — reproducing the deploy's finite-diff/low-pass qvel
estimate in the harness still walks forward — so the flip is a **real engine
dynamics difference** (MuJoCo soft contacts vs Newton's `SolverMuJoCo` LCP
contacts + wrapper latency), not an obs bug. Collecting teacher data *in Newton*
is ~27 h (infeasible). The practical fix — domain-randomize a *learned* residual
to span the engine gap, in the deploy solver — is **exactly what the RL path
already does.**

### 3.7 The decision — RL "follow the shadow," trained in the deploy solver

So the program converged on the Ghost Method's learned half: an **imitation-
residual PPO walker** ([`gpu_mjwarp_g1_walk_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_walk_trainer.py))
— a bounded residual on the same shadow, with a reward that **pays the policy to
keep its pose on the gait model** (follow the shadow), trained in
**`mujoco_warp` = the deploy Newton solver** under heavy DR. It closes, by
construction, the very gap that killed the MPC distillation.

| | MPC (research) | RL "follow the shadow" (standard) |
|---|---|---|
| residual source | online MPPI rollouts, deterministic | PPO policy, learned offline |
| predictor / solver | MuJoCo (offline / sidecar) | **mujoco_warp = the deploy solver** |
| sim-to-deploy gap | **open** (the distillation flip) | **closed by construction** (trains in deploy solver + heavy DR) |
| real-time | no (~20× too slow); only the distilled net is | **yes** |
| reproducible | yes (seeded) | no (learned weights) |
| anti-drift | explicit cost terms | reward shaping + imitation pull |

The MPC is **kept as research**, not shipped: it is a clean deterministic
baseline, a potential teacher/warm-start, and the source of the anti-drift
insight.

### 3.8 Stand-and-hold-cubes — "passenger vs. saboteur," and why a residual on a static stand goes net-NEGATIVE (2026-06-23)

A demo task — a humanoid stands, cubes are thrown at it one-by-one from the
sides, it must **hold its stand while it gets hit** — turned into the cleanest
worked example of law (c)'s *malign* end. Generic controller +
worlds: [`humanoid_stand_deploy`](../../projects/policies/controllers/humanoid_stand_deploy/),
[humanoid-deterministic-stand.md](humanoid-deterministic-stand.md).

**The result that demanded an explanation:** the **pure-deterministic** stand
(stiff position hold of a CoM-centred squat) + a tiny **hand-coded reactive
ankle lean** holds **all 8 cubes indefinitely** — yet a **bounded RL residual**
trained on top of the *same* stand **self-topples in ~1.3 s with NO push at
all.** A learned layer made a *stationary* robot fall. That is the smoking gun:
there was nothing left to improve, so all the residual could do was add motion
where stillness was optimal.

**Why a residual hurts a static stand — three mechanisms beyond "no headroom":**

1. **Authority *is* the instability.** The stiff hold works precisely *because*
   it clamps every joint hard and leaves the CoM almost no freedom to wander.
   Any correction layer needs authority (joint freedom, `ACT_SCALE` rad) to act
   — and on a static hold that authority is *exactly* the freedom that lets one
   misjudgment diverge into a fall. The optimal control authority for "stand
   still" is ≈ 0; anything above it is net-negative, because the downside
   (reopen the pendulum instability the stiff hold just closed) dwarfs the
   upside (≈ 0). Observed directly: a bigger-authority stepping residual (0.6
   rad) toppled *sooner*, and clamping the deploy authority did not rescue it.
2. **The sim-to-deploy gap mis-times an *active feedback loop*; a pose is
   immune.** A *pose* is robust to the engine-dynamics gap (§8) — a pose is a
   pose regardless of the exact solver. A *feedback policy* is not: its
   corrections, tuned to the trainer dynamics, arrive mistimed/miscalibrated
   against the deploy dynamics, and because it is a closed loop on a divergent
   plant, those errors **compound** rather than wash out.
3. **No redundancy to absorb the error → passenger becomes saboteur.** This is
   the variable that decides *which* end of law (c) you land on. **OmniQuad walking**
   (4 contacts, statically stable) has kinematic redundancy that *absorbs* a
   mistimed residual → the residual is a harmless **passenger** (§7.1). **A
   humanoid static stand** (2 contacts, inverted-pendulum margin) has *no*
   redundancy to absorb it → the same class of error **topples** the robot →
   **saboteur.** Same law, opposite sign, set by the task's stability margin.

**It is not an RL artefact — *any* active correction layer does it.** The
hand-coded reactive lean (no learning at all) is **load-bearing for the marginal
G1** (without it G1 face-plants ~2.7 s on a back-hit) but it makes the
already-stiff **H1 fall at ~6 s** and **Valkyrie at ~18 s** — both hold *all*
cubes indefinitely with the lean *off*. So the effect is structural to
"correction layer on a sufficiently-complete static baseline," learned or not.

**The decision rule (add to the SOP, metric #3/#4 in §9):** on a static-balance
task, gate the correction layer — learned *or* hand-coded — on a **measured,
per-robot** improvement over the bare hold. If the bare hold already performs
the task (H1/Valkyrie), **ship it and add nothing**, and *expect* a correction
layer to be net-negative, not neutral. Reach for a residual only where the
deterministic stand is genuinely *marginal* (G1) — and even then prefer the
cheapest sufficient feedback (a 1-DOF ankle lean) over a high-dimensional
policy. The general principle stands: **standing is "hold a solution," so the
honest move is to hold it and let nothing — learned or not — perturb it;**
walking/get-ups/throws are "keep finding the solution," and that is where a
learned layer earns its keep.

---

## 4. The standard contract (apply this to any robot)

This is the part that generalizes. It is the actual "OmniSim RL architecture."

### 4.1 One shared shadow + baseline module per robot

The shadow and the deterministic baseline live in **one** module, imported by
**both** the trainer env and the deploy controller, written as pure
array/tensor math (no Python control flow) so the *identical source* runs as
scalar NumPy at deploy and batched GPU-torch in training. This is the discipline
that kills our #1 historical silent bug: hand-copied baseline math that lets the
residual learn against baseline-A and deploy onto baseline-B.

`g1_human_gait.py` / `omniquad_trot_gait.py` already do this: each ships
`targets_np` (deploy) and `targets_torch` (trainer) from one file, self-tested
against each other and against MuJoCo forward-kinematics on the deploy-matched
MJCF.

The same discipline now covers the **physics** itself, not just the shadow. For
G1 the trainer↔deploy "byte-match the physics" requirement (same solver, gains,
friction, clamp, model) is **no longer hand-matched** — both sides import one
source (`g1_physics.json` + `g1_physics_spec` + the prim URDF; never re-declare a
physics constant), with a conformance CI gate against drift and a golden-parity
harness ([`g1_golden_parity.py`](../../projects/policies/research/training/g1_golden_parity.py))
that compiles the trainer and the *literal* deploy runtime to MuJoCo models and
diffs every dynamic field. As of 2026-06-18 it is **verified** to be the same
physics — 0 real-physics gaps structurally and 8.5 mm GPU trajectory drift
(modulo the opt-in `OMNISIM_NEWTON_USE_LINK_COM` flag + documented residual diffs).
Full writeup: [g1-single-source-of-truth.md](g1-single-source-of-truth.md). Extend
this same pattern (one physics source + a golden diff) to each new robot.

### 4.2 Composition (additive residual)

```
targets = shadow(state) + balance_feedback(state) + ACT_SCALE * residual(obs)
motor.setPosition(targets[i])    # position mode — there is no Newton torque sink (see §6)
```

OmniQuad composes at the **foot-target** level (`foot = shadow + balance_dz +
RES_SCALE·residual → IK → motors`, `RES_SCALE = 0.03 m`); G1 composes at the
**joint** level (`ACT_SCALE` rad). The contract is uniform: *shadow + feedback
is a function of state; the residual is a tightly-bounded additive trim in a safe
space; everything shares one source.*

### 4.3 Bounds + the three anchors (keep the residual a residual)

- **Tight bound.** Keep `ACT_SCALE` / `RES_SCALE` small (OmniQuad ±3 cm; for G1 the
  current shipped scale is 0.3 rad, flagged as large — treat shrinking it toward
  ~0.1 rad as a goal, and treat "the residual needs more authority" as evidence
  **Layer 1 is too weak**, not that the bound is wrong).
- **Anchor 1 — zero-init the residual MLP's final layer** (step-0 output ≡ the
  baseline).
- **Anchor 2 — small `log_std_init` (−1.5)** (stays near baseline early).
- **Anchor 3 — a KL-to-zero / L2-on-residual penalty** so the policy provably
  returns to baseline wherever it cannot help. (Flagged as still-missing in
  [omniquad-residual-rl.md](omniquad-residual-rl.md); add it.)

### 4.4 Observation contract

Proprioceptive only (so the actor is deployable). **Obs order is fatal** and is
a matched contract edited in lockstep across env + deploy in the *same commit*.
Include the disturbance state *and its rate* (a residual blind to rate-of-change
cannot learn disturbance rejection). The critic may take privileged signals
(CoM, DCM, applied push) under asymmetric actor-critic.

### 4.5 Train in the deploy solver; bare baseline is the fallback

- **Train the residual in the deploy-faithful solver** (`SolverMuJoCo` /
  mujoco_warp), **not** blind heavy DR alone — DR deployed G1 *worse* when the
  base wasn't right. This is the load-bearing rule: it is the only thing that
  closes the engine-dynamics gap (§3.6, §8).
- **Fallback = the bare baseline.** Zeroing the residual (or any ONNX load
  error) runs Layer 1 alone, which — unlike a NOMINAL-PD fallback — actually
  performs the task. This is both the zero-policy gate harness and the
  graceful-degrade path.

### 4.6 The ghost is a first-class debugging/demo tool

Every robot gets a `*_ghost` controller that renders the shadow as a translucent
hologram beside the real robot, **phase-locked to the real robot's forward
progress** so they stay in step. The visible gap *is* the tracking + sim-to-
deploy error — the cheapest possible diagnostic, and the centerpiece of the
"watch it learn to match the ghost" demo. Standard env knobs (shared naming):
`<ROBOT>_GHOST_ALPHA` / `_TINT` (hologram look), `_Y` / `_Z` (placement),
`_SELF_WALK` (walk on its own clock across the floor), `_LOG` (telemetry).

---

## 5. Making the shadow realistic ("how good the ideal looks")

The shadow is not just a metronome — its **realism directly sets the quality of
the learned gait**, for two reasons: (i) it is the imitation target, so a
biomechanically faithful shadow yields a natural-looking policy; (ii) a
dynamically *plausible* reference is far easier to track stably than an
arbitrary one. So we invest in the model.

The G1 shadow ([`g1_human_gait.py`](../../projects/policies/control/gait/g1_human_gait.py))
plans in foot space and realizes joints through closed-form IK, with realism
knobs (all env-overridable as `G1_GAIT_*`):

- **Winter hip scaling** (`winter_hip_scale ≈ 0.75`) — hip ranges scaled toward
  human gait-analysis (Winter) norms rather than the URDF mechanical max.
- **Frontal/transverse-plane modes** (`lateral ∈ {sway, lipm, achieved,
  human-3D}`, `yaw`) — the "improved shadow": lateral weight shift and torso/arm
  counter-rotation, not just sagittal stepping.
- **Lateral hip amplitude / step width** (`lat_hip_amp`, `step_width`) — the
  pelvic list and stance width that read as "walking," not "shuffling."
- **Counter-phase arm swing** (`arm_swing`), **quintic swing arc** (zero
  velocity + acceleration at lift-off and touchdown), **foot clearance**, and a
  **stride ramp** (`ramp_s`) that grows step length from zero so the gait starts
  as stepping-in-place from the standing pose (no out-of-distribution launch
  snap).

On top of the model, the **style rewards** (gpu_g1_walk25/26/27) shape foot
clearance, knee double-bend, swing height, contact timing, and ankle push-off —
each measured against biomechanics literature, not eyeballed. The result is a
gait that both *looks* human and *is* easier to keep upright.

> Rule of thumb: **before adding residual authority, improve the shadow.** Most
> "the policy can't do X" problems are really "the ideal doesn't include X."

---

## 6. Verified platform constraints (Newton deploy)

Checked against the code; design with these, not against them:

- **Position-mode is the default deploy path; a torque sink now EXISTS but is
  opt-in (updated 2026-06-19).** By default `OmNewtonBackend` writes only
  `joint_target_pos` / `joint_target_vel` (POSITION_VELOCITY) ⇒ the mimic is a
  **position-mode** controller through the native PD. A Newton joint-torque sink
  (`set_joint_force` → `control.joint_f`) was added 2026-06-16 (`d3946eed`), gated by
  `OMNISIM_NEWTON_TORQUE_MODE`, so a torque-mode law (capture-point/WBC) no longer
  needs a fresh backend change — but it is **not** the default path and is unproven
  for G1. (The earlier flat "`motor.setTorque()` has no Newton sink" is now outdated.)
- **Contacts are not wired through Newton at deploy.** `getContactPoints()` is
  unpopulated under the Newton backend. ⇒ build any support polygon
  *geometrically* from foot-link `getPose()`; do not depend on a measured CoP.
  (`getCenterOfMass()`, `getVelocity()`, `getOrientation()`, `addForce()` *are*
  available and engine-agnostic.)
- **Deploy environment (required).**
  `OMNISIM_NEWTON_FORCE_MUJOCO=1` (⚠ **now redundant**: `SolverMuJoCo` is the default
  since `7b431e81` and XPBD was removed in `94f04222`, so there is nothing to force it
  away from — harmless to keep, but it no longer buys anything),
  `OMNISIM_NEWTON_MJWARP=1` (GPU mjwarp, matches the trainer),
  `OMNISIM_NEWTON_STATICS=1` (⚠ also now the default, since 2026-08-07),
  `OMNISIM_NEWTON_SUBSTEPS=4` (a single 16 ms contact
  solve NaN-explodes), `OMNISIM_URDF_USE_INERTIA=1`. Enable
  `OMNISIM_NEWTON_BASE_GUARD=1` during bring-up so a transient tip can't NaN-mask
  whether the law was converging.
- **Observation frame.** MuJoCo free-joint `qvel[3:6]` is body-frame but
  `getVelocity()` is world-frame — rotate world ang-vel into the body frame
  (`Rᵀ·ω`) at deploy (fixed 4398d3e9). A correctness win for *any* policy.

---

## 7. Per-robot application

| Robot | Shadow | Baseline (Layer 1) | Residual (Layer 2) | Status / gate |
|---|---|---|---|---|
| **G1** (13-DOF biped, ~34 kg) | `g1_human_gait` (realistic, §5) | `g1_brain` (LIPM + capture-point + ankle CoP + weight transfer + posture) **or** the trainer baseline | joint-delta residual, trained in mjwarp with imitation + anti-drift + style rewards | Stand: **solved** (stands indefinitely, pure pose). Walk: long walks achieved; **deploy reproduction fragile** (§3.3) — open work is the engine gap (§8) |
| **OmniQuad** (12-DOF quadruped) | `omniquad_trot_gait` (the g1_human_gait port) | `omniquad_gait` + `omniquad_balance` + `omniquad_kinematics` + heading-hold + recovery FSM | ±3 cm foot-offset residual | **Walks under Newton** (+4.87 m / 30 s with the residual, +5.03 m model-only) — the residual is a **passenger** (§7.1). *An earlier "walks under ODE" result (+5.55 m) is historical and unrepeatable: ODE was deleted 2026-08-08 (`bdc02139`).* Reference implementation of the contract |
| **Atlas** (30-DOF, ~175 kg, tiny feet) | (port `g1_human_gait`) | seed from atlas stand law | **do not** train a residual until the bare law stands | Documented negative result (μ≈0); a residual on a non-standing baseline is the known failure |

### 7.1 OmniQuad in detail — the reference, and its honest gap

OmniQuad is the cleanest, oldest instance of the contract and is worth studying
first ([`omniquad_residual_deploy.py`](../../projects/policies/research/controllers/omniquad_residual_deploy/omniquad_residual_deploy.py)):

- **Shadow + baseline:** trot foot trajectories (`foot_targets`) + per-leg
  balance `dz` (`balance_offsets`) → closed-form IK → motors, with a
  deterministic **heading-hold PD** (yaw + lateral steer-to-centreline, fed only
  to the gait, never to the policy obs) and a scripted **recovery FSM**.
- **Residual:** an 18→12 ONNX policy, ±1 each, scaled to **±3 cm** foot offsets,
  zero residual on ONNX error.
- **Disturbance demo:** `addForce`/velocity impulses + a cosmetic flying cube
  whose `mass·speed` is back-solved from the trained `dv` range — the
  "throw-boxes-at-the-robot" showpiece, reusable for every robot.
- **Results:** model-only walker **+5.03 m**; with the learned residual
  **+4.87 m** under Newton — the residual is a **passenger**. That is *correct*
  per law (c): the analytic layer is already at the limit, so the residual earns
  nothing on the nominal task. The recipe's contribution is knowing *when to
  stop asking the policy to learn*.

**The conformance gap (the same as G1's):** there are two OmniQuad gait paths. The
**canonical** one — `omniquad_trot_gait` (a direct port of `g1_human_gait`, single
numpy+torch source) — is used by the **ghost** and the **GPU trainer**. The
**shipped Newton deploy** still rides the older `control/omniquad_gait` +
heading-hold — a baseline whose constants were **originally tuned against ODE**
(tuning provenance only: ODE was deleted 2026-08-08 and the deploy has no ODE
dependency; it runs entirely on Newton) — because the GPU-trained residual **does
not transfer to Newton** (`omniquad_gpu_residual_deploy`) — the same MuJoCo→Newton
engine gap that flipped G1's distilled walker. Fully converging OmniQuad's deploy onto
`omniquad_trot_gait` + an in-solver-trained residual is **gated on closing that gap**
(§8); until then the deploy keeps that working baseline (ODE-tuned, Newton-run),
and the ghost shows the canonical shadow. This is documented, not hidden.

---

## 8. The one open problem — the MuJoCo→Newton engine gap

Everything else has a known answer; this is the live frontier.

The trainer's `mjw.step()` (mujoco_warp, soft contacts) and the deploy's
`SolverMuJoCo.step()` (LCP rigid contacts + ~1-tick control latency +
gain-scaling drift) **differ enough to flip a marginal biped.**

> ⚠ **Terminology correction (2026-08-08):** this sentence used to describe
> `SolverMuJoCo` as *"MuJoCo wrapped in Newton's XPBD articulation"*. That was wrong when
> written and is doubly wrong now. `SolverMuJoCo` is MuJoCo's own
> **generalized-coordinate** solver operating on Newton's articulation; XPBD was a
> *separate, maximal-coordinate* solver that never touched this path — and it was removed
> entirely in `94f04222`. The trainer/deploy divergence described here is real and
> unaffected; only the label was wrong. OmniQuad (4 contacts, statically stable) absorbs the drift
via kinematic redundancy — hence the passenger result. G1 (2 contacts,
inverted-pendulum margin) sits at the stability edge, so 1 tick of latency + 5 %
torque scaling moves it from "barely standing" to "tipping in ~1 s." This is why
the distilled MPC flipped direction (§3.6) and why the G1 walk deploy is fragile
(§3.3).

**The Ghost Method's answer is structural:** train the residual *inside*
`SolverMuJoCo` itself, under heavy DR (actuator kp/kv ±40 %, action latency 0–3
ticks, mass/friction jitter) that spans the engine gap by construction. The
remaining engineering is (i) trainer throughput on `SolverMuJoCo` (the faithful
CPU trainer is slow; mjwarp is fast but is the *soft-contact* twin, and even
bit-identical MJCFs aren't enough because the `.step()` *wrappers* differ —
playbook dead end #8), and (ii) selecting policies by **deploy** rollouts, not
trainer reward. This is multi-session work and is the principal RL task on the
roadmap.

> **Update 2026-06-24 (H1 walk) — "train in the deploy solver" was tested
> head-on, and it did NOT close the gap; it regressed the deploy.** A Phase-2
> trainer (`gpu_newton_h1_walk_trainer.py`, `cf200cdc`) fine-tunes the H1 champion
> *through the exact deploy solver* (`newton.solvers.SolverMuJoCo`). Versus the
> mjwarp-trained champion (run 3: deploy **2.03 s / +1.45 m forward**), the
> in-solver fine-tune deployed **worse**: 1.58 s / backward on a fresh-URDF model,
> and even 0.66 s / backward on the *matched* dumped-MJCF model. Two lessons sharpen
> §4.5 and the claim above: (1) **"same solver ≠ matched physics"** — a fresh
> `add_urdf`+`add_ground_plane` build silently uses newton's *default* friction, not
> the deploy's `mu=2.0`; load the dumped deploy MJCF via `newton add_mjcf` instead
> (`da8b171a`). (2) Even matched model **+** matched solver is **not** enough: run 3
> and the fine-tune are byte-identical in the batched trainer yet diverge in deploy
> (the batched trainer even walks *backward* where the deploy walks forward). The
> binding gap is the **launch initial condition** (the deploy's ~0.3 s settle lean +
> residual velocity, absent in the batched reset) and the **observation pipeline**
> (world-frame `getVelocity` + finite-diff `qd` vs the trainer's exact MuJoCo-frame
> `qvel`) — not the solver. So "train in the deploy solver" is necessary but not, by
> itself, sufficient. Full writeup: [h1-walk-rl-journey.md](h1-walk-rl-journey.md);
> canonical status: [rl-current-state.md](rl-current-state.md).

---

## 9. Success metrics & the honesty bar

1. **Bare baseline performs the task** under the exact deploy solver, **measured**
   (logged pose), not asserted — the gate prior work skipped.
2. **The robot ships with zero RL** if the baseline clears the gate. RL is
   additive insurance, not the floor.
3. **Residual learned-value delta** (Layer 2's whole justification): the policy
   strictly beats the bare baseline at matched seeds on the *unmodeled* regime
   (e.g. ≥+30 % max recoverable push impulse). Produce this number; it has rarely
   been produced for any robot.
4. **Residual is a correct no-op on the nominal task** (`|residual| ≈ 0`,
   survival == baseline with no perturbation). Report the passenger-on-nominal
   result as the *designed, correct* outcome.
5. **Deploy robustness:** seeded episodes with no NaN/explosion; ONNX-error
   auto-fallback keeps it performing.
6. **Reproduction is part of the result.** A distance achieved once is a *lead*,
   not a milestone, until it reproduces from a cold deploy. (See the G1 walk
   caveat, §3.3 — and write down what *doesn't* reproduce.)

---

## 10. SOP — applying the Ghost Method to a new robot

1. **Build the shadow** in `projects/policies/control/gait/<robot>_*.py`: foot-space plan +
   closed-form IK, one file shipping `targets_np` (deploy) and `targets_torch`
   (trainer), self-tested numpy-vs-torch and IK-vs-FK against the deploy MJCF.
   Invest in realism (§5).
2. **Render the ghost** in `controllers/<robot>_ghost/`: mirror
   [`g1_ghost.py`](../../projects/policies/controllers/ghost_hologram/ghost_hologram.py) — hologram,
   phase-locked to the real robot, `SELF_WALK` + `LOG` knobs.
3. **Write the deterministic baseline** (shadow feedforward + a *feedback*
   balance law) and prove the **bare baseline performs the task** under the
   deploy solver (metric #1). Ship here if it clears.
4. **Add the bounded residual:** zero-init final layer, small `log_std`, KL/L2
   anchor, tight `ACT_SCALE`; obs = proprioception + disturbance state & rate.
5. **Train in the deploy solver** (`SolverMuJoCo`/mjwarp) with imitation
   ("follow-the-shadow") + anti-drift (`w_vy`,`w_yaw`) + style rewards + a
   survival-gated push curriculum for the disturbance regime.
6. **Select by deploy rollouts**, not trainer reward. Measure the learned-value
   delta (metric #3). Report passengers honestly.

---

## 11. References

**Literature:** Silver et al., *Residual Policy Learning* (arXiv 1812.06298);
Johannink et al., *Residual RL for Robot Control* (arXiv 1812.03201);
capture-point / Divergent Component of Motion (Pratt, Englsberger); Winter,
*Biomechanics and Motor Control of Human Movement*; MPPI (Williams et al.).

**OmniSim:**
[rl-current-state.md](rl-current-state.md) (live status) ·
[rl-journey.md](rl-journey.md) (master narrative) ·
[g1-mpc-deterministic-brain-research.md](g1-mpc-deterministic-brain-research.md) (MPC deep-dive) ·
[g1-stand-rl-playbook.md](g1-stand-rl-playbook.md) ·
[g1-walk-rl-journey.md](g1-walk-rl-journey.md) ·
[omniquad-residual-rl.md](omniquad-residual-rl.md) ·
[sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) ·
[atlas-stand-rl-journey.md](atlas-stand-rl-journey.md).

**Code:**
shadows — [`g1_human_gait.py`](../../projects/policies/control/gait/g1_human_gait.py),
[`omniquad_trot_gait.py`](../../projects/policies/control/gait/omniquad_trot_gait.py) ·
ghosts — [`g1_ghost.py`](../../projects/policies/controllers/ghost_hologram/ghost_hologram.py),
[`omniquad_ghost.py`](../../projects/policies/research/controllers/omniquad_ghost/omniquad_ghost.py) ·
baselines — [`g1_brain.py`](../../projects/policies/control/g1_brain.py),
[`omniquad_gait.py`](../../projects/policies/control/omniquad_gait.py),
[`omniquad_balance.py`](../../projects/policies/control/omniquad_balance.py) ·
MPC — [`g1_mpc.py`](../../projects/policies/control/g1_mpc.py),
[`g1_distill.py`](../../projects/policies/control/g1_distill.py) ·
mimics — [`g1_walk_deploy.py`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py),
[`omniquad_residual_deploy.py`](../../projects/policies/research/controllers/omniquad_residual_deploy/omniquad_residual_deploy.py) ·
trainer — [`gpu_mjwarp_g1_walk_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_walk_trainer.py).
