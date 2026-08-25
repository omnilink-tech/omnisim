# G1 universal motion tracker — the north star

> 🏗️ **The realized architecture for this objective is now
> [ghost-tracking-pipeline.md](ghost-tracking-pipeline.md)** — feasible reference
> (a *planner* / trajectory optimization produces a dynamically-achievable ghost) +
> robust RL tracking, made robot- and motion-agnostic. That doc is the canonical
> *how*; this doc is the *why/objective* + the G1 walking test-bed ledger. The key
> insight it adds: the "reference-as-input" ingredient below must be a *dynamically
> feasible* trajectory from a planner, not a hand-drawn kinematic one — "planning
> describes, control solves."

> 📍 **Canonical G1 status:** [rl-current-state.md](rl-current-state.md) (the *G1 — detail*
> section) is the single source of truth. If a status claim here disagrees with that file,
> that file is right — fix this one.

**The objective (stated by the project maintainer, 2026-06-15):**
> Develop a *recipe* where the robot can always follow the ghost almost perfectly,
> for *any* motion the ghost does.

This is a **universal physics-based motion tracker**: one policy that takes an
arbitrary reference motion (the "ghost") as input and reproduces it on the physical
robot, closely, in deploy. It is the DeepMimic → AMP → PHC / MaskedMimic line of work.
Everything in the G1 walking effort ([`g1-walk-rl-journey.md`](g1-walk-rl-journey.md),
[`g1-improved-shadow.md`](g1-improved-shadow.md)) is a step toward this — walking is the
*test-bed* for the recipe, not the end goal.

**Key truth:** in *simulation*, universal tracking is largely solved (PHC/MaskedMimic
reproduce whole mocap datasets). The frontier is **deploy** — crossing the sim→engine→real
gap. So the hard, motion-agnostic blocker is deploy robustness + fidelity, and that is
exactly what we grind on with walking.

---

## The recipe (the reusable method)

> **⭐ Ingredient 0 — THE GHOST MUST BE ACHIEVABLE (maintainer-marked extremely important, 2026-06-19).**
> The reference fed to the tracker must be **physically reproducible by the robot**. Never train
> to mimic a ghost the robot can't achieve — infeasible references come from **morphology**
> (missing DOFs, e.g. the G1's lack of a torso-pitch joint → it can't sit bolt-upright),
> **actuation** (too little torque to cancel a reaction), or **balance/contact** limits. An
> unachievable ghost caps the imitation reward and *no amount of training/scale closes it*
> (seated-G1: bolt-upright ghost stuck ~62–68% even after a 164 M-step A100 run). The fix is a
> **better ghost, not a better policy**: derive it from what the robot can do — record-and-replay
> the achieved motion, or retarget/distill onto the robot's kinematics + balance ([improved
> shadow](g1-improved-shadow.md)). This is the same lesson as #1 and the "Design principle" below,
> stated as a hard rule: a universal tracker is only as good as the *feasibility* of its ghosts.

| # | ingredient | what it is | status |
|---|---|---|---|
| 1 | **reference-as-INPUT** | ghost pose now + a short horizon ahead is fed to the policy as a *command*, not baked in. `policy(proprio, target_traj) → action`. | **partial** — lookahead obs exists, but the gait is still hardcoded in the action baseline. For *any* motion, flip this: drop the hardcoded gait, make the full target pose the command. |
| 2 | **broad motion corpus** | "any motion" → train over many motions (walk, turn, squat, reach, recover). | **missing** — needs a motion source (AMASS/mocap retargeted to G1, or procedural). The "many ghosts / variation" idea, extended across motion *types*. |
| 3 | **tracking reward** | pay for matching pose + joint velocity + keypoints. | **have** — imitation / shape / swing-track / track-vel / frontal-track. |
| 4 | **deploy robustness** | the tracked motion survives engine/real gaps. | **in progress** — this is THE blocker. Contact-softness DR (2026-06-15) was the breakthrough: it crosses the warp→Newton *stability* gap. Fidelity-in-deploy still open. |
| 5 | **scale** | universal tracking is data/compute hungry. | partial — fast trainer (~100k env-steps/s); corpus + iterations would grow. |

**Design principle for the reference (learned the hard way):** match the *gait / shape /
distribution*, NOT the instant pose tick-for-tick (tick-tracking fights balance timing and
HURTS — walk t1/t2). And **condition on GOALS** (speed, heading — command them), but
**randomize-BLIND on NUISANCES** (contact, style jitter, asymmetry — for robustness).

**Architectural move it eventually forces:** today the controller is
`gait_baseline + small_residual` (walk-specific). The universal form is a
**goal-conditioned tracker**: the ghost trajectory is a runtime input, the policy outputs
full actions, no hardcoded gait. We are incrementally earning the pieces (lookahead = the
seed; velocity-conditioning = conditioning on one command; improved shadow = feasible
targets).

---

## Honest ceilings

- **Sim: "almost perfect" is reachable** for arbitrary motion. The only floor is actuator
  PD compliance: ~8° per loaded joint at kp=100, ~6° at kp=200 (measured). Zero is off the
  table (and undesirable — the ghost is kinematic; perfect tracking = not balancing).
- **Deploy: the open research frontier** (sim2real whole-body tracking — OmniH2O, ExBody,
  HumanPlus). Our warp↔Newton gap is the in-house version. This is a grind, not a weekend.
- **Realistic target:** ~6° *uniform* per joint, no splay outliers = indistinguishably
  human. Already there sagittally (6.4°); the frontal plane is the active battle.

---

## Sequencing (do NOT reorder)

1. **Prove tracking + deploy on ONE motion (walking), including the sim gap.** ← we are here.
   The sim gap is motion-agnostic, so cracking it on walking cracks the hard part for
   everything.
2. **Flip to goal-conditioned** (reference as input, drop the baked gait).
3. **Feed a motion corpus** (mocap-retargeted "many ghosts of all kinds").
4. **Scale.** → any ghost motion, tracked closely, in deploy.

---

## Progress ledger (walking test-bed)

| policy | trainer splay | deploy | takeaway |
|---|---|---|---|
| walk26_shape_c8 (champion) | 28.8° | ⛔ its long-distance zero-fall figure is **RETRACTED** (trainer/old-path only); deploy TOPPLES ~1 s (`FALL@1.06s`) | trainer walks far but splayed (sagittal solved); does NOT reproduce in the Newton deploy |
| walk30_shadowC | 21.7° | **fell @ 6.6 s** | improved shadow cuts trainer splay, but COLLAPSES in Newton (sim gap) |
| walk31_shadowCdr (+contact DR) | ~21° | **+131 m / 337 s**, deploy splay 26.4° | **contact DR crosses the STABILITY gap** — the recipe-4 breakthrough |
| walk32_shadowCfid (+tight residual 0.3) | ~slim | fell @ 10 s, deploy splay 18.6° | residual-cap MECHANISM works (splay 26.4→18.6) but past the stability cliff — Newton needs the splay to balance |
| walk33_shadowCwide (wider DR 0.65) | ~slim | fell @ 29 s, deploy splay 24.8° | wider DR = "too wide → mush": −1.6° splay for −308 s survival. **DR ceiling reached.** |

**Lateral-fidelity-in-deploy is now FULLY characterized — every cheap lever exhausted:**
reward (helps trainer not deploy), residual-cap (falls), contact-param-match (already
identical), wider DR (worse). The ~26° deploy splay is **solver-gated load-bearing
balance**. The ONLY remaining fidelity lever is **Newton fine-tune** (train in the deploy
engine — expensive). **Decision: walk31 is the best deployable shadow-C (131 m, lateral-
stable); bank it and advance to goal-conditioning (lever 3) — higher leverage toward the
universal objective than the last few degrees of lateral polish.**

**Recipe lessons banked:**
1. DR must randomize **contact `solref`/`solimp`**, not just friction — contact stiffness
   is the dominant warp↔Newton difference (`--dr-solref-scale`). This crossed the
   *stability* gap.
2. **In deploy, the lateral splay is largely LOAD-BEARING balance** — the trainer's slim
   gait overfits warp's forgiving contact; Newton genuinely needs the splay. So deploy
   *fidelity* on the frontal plane is NOT a trainer-reward / residual-cap problem — it
   needs the engine gap actually closed (**Newton fine-tune**), so the slim gait is
   Newton-stable rather than forced. This is why lever 2 (below) is the real next step.

---

## Open levers (in priority order)

1. **Deploy fidelity** (current): `--frontal-res-scale` (cap the splay residual) +
   stronger frontal-track. If reward+residual tug-of-war stalls, escalate.
2. **Newton fine-tune** — a few chunks in the deploy engine; the gold-standard gap-closer.
3. **Goal-conditioning** — make the reference a runtime input (ingredient 1).
4. **Motion corpus + AMP** — mocap-retargeted "match the distribution of realistic motion"
   (ingredient 2); the documented "beyond local optimality" frontier.
5. **kp=200 actuators** — only for the last ~2° of fidelity; hardware/stability cost.
