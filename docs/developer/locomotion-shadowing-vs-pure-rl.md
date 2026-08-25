# Walking RL — why shadowing stalls, and the full-authority path

> **⚠️ SUPERSEDED (2026-07-03).** This doc's conclusion ("Shadowing is the wrong architecture
> for continuous-balance locomotion") judged the PRE-PARITY era: standalone trainers with the
> unsolved dt/handoff/clamp deploy gaps and HAND-DESIGNED kinematic ghosts. Post-parity,
> in-engine Shadowing with a RECORDED ghost produced the durable live-verified G1 walk and is
> now the FLAGSHIP method ([projects/policies/training/](../../projects/policies/training/README.md),
> [ghost-design-rules.md](ghost-design-rules.md), [rl-current-state.md](rl-current-state.md) top
> banner). The analysis below stands as history of WHY hand-designed ghosts fail — which the
> ghost validator now checks mechanically.


> **Status (2026-06-25): a course-correction, evidence-backed.** For the H1 walk
> (and dynamic-balance locomotion generally) the **Shadowing** architecture
> (hard-track a hand-designed kinematic ghost + a small bounded RL residual) is the
> *wrong tool* — it produces the natural gait we want but **cannot stabilise**. The
> field's answer for "natural **and** stable" is the inverse: **full-authority RL,
> stability as the primary objective, naturalness as a *soft* reward.** This doc
> records the reasoning + the supporting evidence so we don't relearn it. It does
> NOT retire Shadowing — see *When Shadowing is still right* at the bottom.

Companion to [shadowing.md](shadowing.md) (the method), [rl-current-state.md](rl-current-state.md)
(canonical status), and [h1-walk-rl-journey.md](h1-walk-rl-journey.md) (the H1 arc that exposed this).

---

## 1. The empirical observation (the maintainer's, and it's correct)

Two regimes, from our own history:

- **Pure RL** (early OmniQuad, and the classic from-scratch legged-RL): the robot
  **walks and does NOT fall** — but the gait looks strange / unnatural (asymmetric,
  flailing, over-bent).
- **Shadowing** (track a designed "ghost" gait + bounded residual): the gait looks
  **natural and is exactly what we want** — but it **falls in ~2 s** and never
  achieves a durable walk, no matter how much we tune.

The tension is real and well known in the field: **style vs. stability.** Our
architecture lands hard on *style*, and pays with *stability*.

## 2. Why Shadowing cannot stabilise a walk (the mechanism)

Our walk = **"play a fixed kinematic recording (the ghost), let a tiny bounded
residual (±0.3 rad) nudge it."** That has two fatal properties for balance:

1. **The ghost is a *kinematic* reference, not a *balance* solution.** It is a
   sequence of joint angles that *looks* like walking. A real walk's stability lives
   in **reactive foot placement and push-off timing** ("the CoM is drifting → take a
   bigger/quicker step *there*"). That information is simply **not present** in a
   hand-drawn joint-angle curve. Tracking it perfectly still falls.

2. **The residual has almost no authority, and spends it fighting the ghost.** At
   res_scale 0.3 the policy can only tweak each joint ±0.3 rad. Worse: the ghost
   keeps commanding "step *here*" while the policy needs "step *there* to catch the
   fall" — feedforward and balancer work against each other.

**The deepest framing: we made *style* the primary objective (track the ghost) and
*stability* an afterthought (a small correction). That is backwards.** Stability
must be the policy's own primary job; style is a flavour you add on top.

This also explains our documented *"residual is a passenger / saboteur"* finding —
a symptom of the same root cause: the residual has too little authority to matter,
or it destabilises because it is correcting the wrong thing.

## 3. How the field actually gets natural AND stable

The whole field converged on **inverting our setup**: full control authority +
stability primary + naturalness *soft*. Three points on a spectrum, same principle:

1. **Pure RL + reward shaping (the workhorse — Unitree, ANYmal, legged_gym,
   Cassie sim-to-real).** Pure RL, so **stable by construction**; the "ugly gait" is
   removed not by imitation but by **regularising rewards**: torque/energy penalty
   (→ smooth, efficient), **left-right symmetry** loss (→ no limp), action-rate
   penalty (→ no jitter), a **foot-clearance + contact-schedule** reward (→ a clean
   stepping rhythm), upright-posture reward, velocity-command tracking, and **early
   fall termination**. Naturalness comes from constraints on *how it moves*, not
   from copying a trajectory. *(Rudin et al. 2021 "legged_gym"; Siekmann/Xie et al.
   Cassie.)*

2. **DeepMimic-style soft imitation** *(Peng et al. SIGGRAPH 2018)*: keep a
   reference — ideally **real mocap retargeted to the robot**, not a hand-drawn ghost
   — but as a **soft reward**, while the policy outputs the **full** action and gets a
   strong task + fall reward. The policy is free to deviate wherever it needs to stay
   up, paying only a small style cost. Full authority + soft style — the opposite of
   our bounded residual + hard playback.

3. **Adversarial Motion Priors / AMP** *(Peng et al. SIGGRAPH 2021)* — current SOTA
   for natural+robust. Replace explicit tracking with a learned **style
   discriminator** over a *dataset* of natural motions; reward = task + "looks like
   the dataset." The policy never tracks a phase-locked trajectory — it keeps the
   *distribution* of its motion natural while doing whatever stabilises it.

**Common thread:** the policy owns balance (full authority + fall termination);
naturalness is a soft objective layered on a stable backbone — never the reverse.

## 4. The decision for OmniSim

For the **H1 walk specifically, switch to full-authority RL + reward shaping** (the
legged_gym / DeepMimic recipe): the policy outputs the joint PD targets directly
(or a *large* residual on a *balance* controller, not on a kinematic recording),
stability is the dominant reward, and the ghost — if used at all — is demoted to a
**soft style term** alongside symmetry / energy / smoothness / foot-schedule
regularisers. This unifies the maintainer's two observations: **pure RL (so it walks and
doesn't fall) shaped to look natural (so it walks the way we want)** — instead of
the reverse.

### The concrete reward set (the pure-RL recipe)
- **Task**: track a commanded forward velocity (and 0 lateral / 0 yaw for straight
  walking); upright torso; **early fall termination** (the load-bearing stability
  signal).
- **Gait shaping**: a periodic phase clock as a policy input + a **foot
  contact-schedule** reward (left/right alternate, ~duty 0.6) + **foot-clearance**
  (lift the swing foot) — this is what turns "don't fall" into "step rhythmically."
- **Naturalness regularisers**: **left-right symmetry** (mirror loss), **torque /
  energy** penalty, **action-rate / smoothness** penalty, joint-limit / posture
  penalty, nominal-pose bias (small).
- **Robustness**: domain randomisation (push, mass/friction, latency, obs noise) so
  it survives the sim-to-deploy gap.

## 5. Supporting evidence from this session (2026-06-24/25)

The H1 push is what made this conclusion unavoidable — and it produced real
infrastructure worth keeping:

- **The faithful deploy-in-the-loop trainer (the session's win, commit 440f2987).**
  The batched Newton `SolverMuJoCo` trainer walked *backward* vs the deploy's forward
  on a byte-faithful model — because `add_mjcf` had dropped `joint_target_ke`, so the
  control **never reached the joints** (the RL residual had *literally zero effect*;
  proven by res_scale 0/0.3/1.0 giving identical evals). Re-adding it made a
  faithful, ~100k-steps/s deploy-in-the-loop trainer where the sweet-spot policy
  **transfers** (2.08 s fwd, on par with run 3) — breaking the session-long "every
  retraining regresses" wall.
- **But durability never came (7 training runs).** Even with control + contacts +
  model + settle IC + finite-diff qd + a capture-point foot-placement law all
  matched, **extended training overfits a residual trainer↔deploy gap** and the
  deploy regresses below the ~400-it sweet spot. The *trainer* gets more durable +
  forward; the *deploy* goes backward. That's the shadowing-residual architecture
  hitting its ceiling, not a bug.
- **The capture-point (DCM) foot-placement law** (commits 726815ae, 1cc72a69) was
  the right *idea* (give the controller authority over where the foot lands) but
  implemented as a **sagittal** correction on the ghost; H1's fall became strongly
  **lateral** — exposing the real structural weakness: **H1 has 5-DOF legs with NO
  ankle roll**, so it has almost no sideways balance. A lateral (step-width)
  capture-point would be needed — another reason the policy needs full authority,
  not a bounded nudge.

**Verdict that stands:** ~2 m forward (run 3 / the 400-it faithful policy) is the
best Shadowing-based H1 walk; a durable 10–20 m walk is **not reachable by tuning
the bounded-residual-on-a-ghost architecture.**

## 6. When Shadowing IS still right (don't overcorrect)

Shadowing is not wrong in general — it is wrong *for continuous dynamic balance*.
It shines when the reference is **dynamically feasible** and the task is **not a
continuous balance problem**: get-up / rise (B2, G1), reaching, sit-to-stand,
toss-to-place, replaying a recorded motion. There the ghost is a genuine plan the
policy can track, and the bounded residual is enough. Keep Shadowing for those; use
**full-authority RL + reward shaping** for *walking* and other continuous-balance
locomotion.

> One-line rule to remember: **a kinematic reference describes a gait; it does not
> solve balance. Stability has to be the policy's job — make style the soft term,
> not the hard constraint.**

## 7. What pure RL actually delivered (the honest result, 2026-06-25)

The course-correction was executed in full (Modal H100); the campaign is documented
blow-by-blow in [h1-walk-rl-journey.md §7](h1-walk-rl-journey.md). The honest outcome:

- **Pure RL + reward shaping does train a walk** — and it took two non-obvious fixes:
  (1) **anti-reward-hacking** — the first run *marched in place* at vx 0.12;
  velocity-tracking had to be made *primary* (sharp σ + a non-saturating linear pull)
  and survival *cheap* before it would translate forward; (2) **closed-loop** — the
  naive policy was nearly **observation-independent** (open-loop, phase-clocked; a
  1 rad/s qd perturbation moved the action < 5 %), so it ignored feedback and could not
  recover. The fix was **obs frame-stacking** + a **speed-regulating reward**, which
  *does* make the policy feedback-driven and speed-regulating (committed `f7a6ac0d`).

- **But durability was NOT solved.** Running the *honest* evaluator (survival +
  distance-before-first-fall, not the auto-reset-inflated per-step reward) on the best
  closed-loop policy: it walks **~0.31 m/s** and covers **~0.5 m (max ~2.1 m) before
  falling, every ~1.7 s — in the trainer itself.** Deploy is consistent (~0.3–0.7 m).
  So pure RL **matched** Shadowing's ~2 m wall; it did not break it.

**Refined verdict:** the §1–§5 claim "*Shadowing is the wrong tool, pure RL will make it
walk*" is **half right**. Pure RL is the right *architecture* (full authority + feedback
+ stability-primary), and Shadowing genuinely *can't* stabilise a walk. But **neither has
produced a durable H1 walk** — the ~1–2 m wall is the *same* in both, it is a
**durability** problem (not an architecture problem) and — the session's biggest
correction — **not** primarily a sim-to-deploy problem: the trainer policy is itself only
a ~0.5 m walker. Open levers: durability-weighted long-horizon training with an honest
in-loop survival metric, and H1's structural lateral weakness (no ankle-roll). **Do not
report an H1 walk from reward/value curves — they hid the fall frequency once; run the
survival + distance eval.**
