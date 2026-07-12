# The OmniSim RL Journey

*The complete narrative of OmniSim's reinforcement-learning work — every robot,
every task, the wins and the dead ends, and the hard-won rules — current to
2026-06-14.*

> 📍 **For the canonical, re-verified "is it done?" status (especially G1), see
> [rl-current-state.md](rl-current-state.md).** This narrative is current to
> 2026-06-14 and predates the 2026-06-18/06-19 G1 ghost-fidelity / durability
> corrections. ⛔ **In particular, the long-distance zero-fall G1 deploy walks this
> document originally headlined were RETRACTED as unreproducible** (§4) — they are
> **not** a result, and must not be quoted as one. Where this file and
> rl-current-state.md disagree, **that file is right.**

This is the **master story**. It ties together the per-topic deep-dives
([spot-residual-rl](spot-residual-rl.md),
[g1-stand-rl-playbook](g1-stand-rl-playbook.md),
[g1-walk-rl-journey](g1-walk-rl-journey.md),
[atlas-stand-rl-journey](atlas-stand-rl-journey.md),
[sim-to-deploy-rl-recipe](sim-to-deploy-rl-recipe.md),
[rl-two-layer-architecture](rl-two-layer-architecture.md)) into one arc.
For the deep recipes and dead-end ledgers, follow the links. For "is it done?",
see [§10 Current status](#10-current-status).

> **Note on `rl-current-state.md`.** That file is the **canonical status** — the
> single source of truth, with its G1/locomotion sections **re-verified 2026-06-19**
> (Spot/Atlas sections date to 2026-05-29). **Where this journey and that file
> disagree, `rl-current-state.md` wins** (consistent with the banner above). This
> document is the narrative record — current to 2026-06-14 — of how each thread got
> where it is; for the authoritative "is it done?" answer, defer to
> `rl-current-state.md`.

---

## 0. The one idea

Every result here is downstream of a single meta-principle the team arrived at
the hard way:

> **RL value scales inversely with analytical-model completeness.** Encode what
> you *know* about the task as a deterministic controller; let the policy learn
> only the part you *can't* model.

This is the **two-layer architecture** ([rl-two-layer-architecture.md](rl-two-layer-architecture.md)):

- **Layer 1 — a deterministic controller** that does the task by itself (a gait
  model + inverse kinematics + a balance PD). It is *not* allowed to be weak: if
  the analytic layer can't almost do the task, the policy is asked to learn too
  much and from-scratch RL's failure modes return.
- **Layer 2 — a bounded RL residual** that augments Layer 1 in the regime the
  model doesn't capture (contact, asymmetry, the sim-to-deploy gap). The residual
  is *small* (a few cm of foot offset, or ±0.3 rad of joint delta) so it can only
  *nudge* — it cannot fight the gait into a stand-still local optimum.

The corollary, discovered repeatedly, is the program's other thesis:
**the binding constraint is almost never the policy — it's the sim-to-deploy
gap.** The trainer (GPU `mujoco_warp`) and the deploy (OmniSim's Newton/MuJoCo
backend) are *almost* the same physics, and "almost" is where the robots fall
over. Most of this journey is the story of finding and closing those gaps.

---

## 1. Origins — residual RL on Spot (the paradigm is born)

**2026-05-24.** The first attempts at quadruped walking were vanilla from-scratch
PPO (raw joint targets, learn everything). They failed the same way every time:
the policy parked in a **stand-still local optimum** and never found a gait;
500k steps, no straight walk.

The user made the call that defined everything after:
*"we know how this quadruped should move; encode that mathematically and let RL
only refine it."* The result ([project residual-rl stack],
[spot-residual-rl.md](spot-residual-rl.md)):

- **Model layer:** analytic FK/IK (`spot_kinematics.py`, round-trips to machine
  epsilon), a trot foot-trajectory generator (`spot_gait.py`), and a body-pose
  balance PD (`spot_balance.py`).
- **Policy layer:** an *18→64→64→12* MLP that outputs **±3 cm foot offsets** —
  bounded by physics, unable to fight the gait.

| | from-scratch PPO | residual on model |
|---|---|---|
| what the policy learns | gait + placement + balance + kinematics | just model error |
| stand-still local optimum | sticky (the wall) | impossible (gait always cycles) |
| steps to a straight walk | failed at 500k | **5,000 — 100× fewer** |
| wall-clock | tens of minutes | **52 s** |

This produced the first **STRAIGHT-verified** Spot walker, and — more durably —
the meta-principle. (It also produced the first honest *negative* result: under
Newton the learned residual turned out to be a **passenger** — the open-loop
model walker alone went +5.03 m straight, the 50k-step policy on top +4.87 m.
When the analytic model is already at the limit, the residual earns nothing.
That negative knowledge is the meta-principle's evidence base.)

---

## 2. Standing — G1, the deploy wall, and the solve

**2026-05-28.** The Spot recipe was ported to the **Unitree G1** (a 13-DOF
legs+waist biped) on a new GPU `mujoco_warp` trainer
([g1-stand-rl-playbook.md](g1-stand-rl-playbook.md)). In-sim it stood at 99.6%
survival across 1024 envs in minutes. But the **deploy** told the real story, and
it took two weeks to close.

The arc (it is the canonical example of the sim-to-deploy gap):

1. **Physics-matching failed; domain randomization worked.** Five iterations of
   trying to bit-match the deploy wrapper (solver, gains, timing, MJCF, obs
   source) didn't close the gap. The robust answer was **heavy DR** — put the
   deploy's operating point *inside* the training distribution rather than
   matching it. The DR knobs that mattered: per-body mass ±30%, friction ±50%,
   per-DOF damping ±50%, **actuator kp/kv ±40%** (the key add), **0–3 ticks of
   random action latency** (also critical), pushes at 1.5 m/s.
2. **Then the deploy regressed and faceplanted at ~1s for days.** A long, painful
   chase ([g1-newton-deploy-regression]) that finally landed outside Newton
   entirely: stepping the deploy model in **plain MuJoCo** (no Newton, no RL, just
   holding the pose) *also* tipped at ~1.3 s. So the fall was in the **model and
   pose**, not the engine and not the policy. Two causes, both fixed
   (**commit f48f00b7, 2026-06-10**):
   - At the old NOMINAL (hip −0.20 / knee 0.42) the whole-body CoM sat **5 mm
     *ahead* of the foot front** — because the OmniSim URDF importer places the
     foot ~35 mm further back than MuJoCo's native `add_urdf`. It tips forward
     under *any* control. Fix: a **deeper squat (hip −0.30 / knee 0.52)** puts the
     CoM behind the foot front → statically stable.
   - The analytic **ankle PD destabilised roll** (a finite-diff `roll_rate` kick
     at handover). Fix: **PD off by default**; a statically-stable pose needs zero
     active control.

**Result:** G1 stands indefinitely in OmniSim Newton (`run_g1_stand_deploy.ps1`),
roll ≈ 0, bz ≈ 0.78, zero falls — verified in *both* plain MuJoCo and
`mujoco_warp`. The deploy default is the **pure-pose stand**
(`G1_BALANCE_FALLBACK=1`); an RL residual at this point *destabilises* the stiff
contact rather than helping (the strong-baseline ceiling again).

**The lesson that paid off for everything after:** *before* blaming the policy or
the engine, reproduce the failure in the simplest possible setting (plain MuJoCo,
zero policy). And *always diff the deploy model against the trainer model* — the
importer is a real source of discrepancy.

---

## 3. Atlas — an honest negative result

**2026-05-28.** The G1 recipe was ported to **Atlas** (30 DOF, ~175 kg,
[atlas-stand-rl-journey.md](atlas-stand-rl-journey.md)). The whole pipeline
ported after six Atlas-specific tweaks (mass-DR ≤ 0.20 or `mujoco_warp` NaNs the
qpos; RES_SCALE halved; NaN-sanitize before reward; reward/v-target clamps;
`dynamo=False` ONNX export). But **PPO never learned anything that beat the
analytic baseline** — trained vs zero-action median survival was *identical*, and
μ stayed pinned at ~0.

Two keepers:
- **Mass DR in `mujoco_warp` is per-*run*, not per-*env*** (one shared model) —
  only 2 of 8 mass seeds even leave the baseline standing. A lottery; dropped.
- A near-saturating analytic baseline + heavy DR + 30-DOF gradient noise
  **starves PPO of signal** (the baseline already harvests ~88% of the reward
  ceiling at iter 1). This is the meta-principle stated as a failure mode: when
  Layer 1 is already excellent, there is no profitable direction for Layer 2.

Shipped honestly as a negative result. It sharpened the thesis.

---

## 4. Walking — the big arc (G1), and the long-distance figures we retracted

This is the longest thread and the one with the most reversals. Full ledger:
[g1-walk-rl-journey.md](g1-walk-rl-journey.md).

> ⛔ **RETRACTED — read before you read anything else in this section.** During
> 2026-06-11…06-14 this thread reported several **long-distance, zero-fall G1 deploy
> walks** (three-figure metre counts over 10-minute bouts). Those figures were
> **retracted on 2026-06-19 as unreproducible**: they came from an old deploy path
> (before the silent-XPBD-fallback fix `cbe5e6f0` and the trainer↔deploy joint-clamp
> parity fix `9b6df709`), and on the current Newton deploy the same policies
> **topple in ~1 s**. **They were never a shipped result and must not be quoted as
> one.** They are described below only as *claims that were made and withdrawn*,
> because the withdrawal is the lesson. The reproducing deploy walk of that era is
> finite (`ft_pdoff_clamp`, +5.9 m / 33.8 s, then falls). Canonical status:
> [rl-current-state.md](rl-current-state.md).

The short version:

### 4.1 The breakthrough was a stiffness mismatch (walk15)
For a long time G1 walking fell at ~1.7 s and the diagnosis was "it needs a
closed-loop whole-body controller; the 6 cm foot is the wall." **That was wrong.**
The real cause was a **20× joint-stiffness sim-to-deploy mismatch**: the trainer
MJCF had kp=20 while the deploy ran KE=400. No DR covers 20×. With a
**stiffness-matched** MJCF (kp=100 ↔ TARGET_KE=100), an ankle counter-rotation
gait (keeps the foot flat through the hip swing — without it the CPG drifts
*backward*), foot-contact rewards, a vel-L1 term, gentle DR, entropy anneal, and
a low-lr final chunk, G1 walked (⚠️ old deploy path, +25.9 m, **not re-verified** on
the current Newton deploy) — `gpu_g1_walk15_c12`, commit 10f7d989. **The rule born
here — and it is the durable part:** *always diff the deploy-dump actuator gains
against the trainer MJCF first.*

### 4.2 Full body, then a human gait
Arms were added by warm-starting the legs champion onto the full-body MJCF with
the arms pinned (`--hold-arms`). Then the joint-space sine CPG was replaced by a
**foot-space human gait model** (`projects/policies/control/gait/g1_human_gait.py`):
foot trajectories planned in Cartesian space (stance foot slides at −vx, quintic
swing, inverted-pendulum bob) and realized by a closed-form 2-link **IK**
(calibrated to 0.00 mm against the MJCF). This is the reference the residual rides.

### 4.3 The four stacked deploy-gap causes (the "trainer-walks, deploy-dies" mystery)
The human-gait policy walked in the trainer but died at ~1.9 s in deploy. It was
**four stacked causes**, each found by *instrument-don't-guess*
(**commit f4c04f7a**; the policy was then reported as walking hundreds of metres with
zero falls — ⛔ **that distance claim was later retracted as unreproducible**, see the
banner above; the four root causes below are real and still hold):
1. **Launch phase** — phase-0 of the gait clock is *right-foot mid-swing*;
   settling there stands on a lifted foot and tips. Start at `DS_PHASE` (double
   support).
2. **Gravity-sag rest-starts** — the deploy settle leaves knees sagged ~0.15 rad
   (τ/kp); clean rest-starts were out-of-distribution. (Read the deploy obs `dq`
   to see it.)
3. **tanh→clamp ONNX export** — the trainer *clamps* actions but the exported
   ONNX used `tanh`, weakening mid-range actions up to 24%. **Latent since the
   stand trainer**, masked by saturated policies. (`reexport_onnx_clamp.py`.)
4. **`njmax`/`nconmax` never plumbed to deploy** — Newton's auto-estimate was too
   small → full-stride footstrike constraint overflow → bz → thousands (explosion).
   Now defaults to 256 incl. the seed rebuild.

**The tool that solved it, repeatedly: a trainer-vs-deploy obs diff.** Also a
lasting lesson — *deploy can exceed trainer eval* (the launch was the whole
bottleneck), so **select policies by deploy samples, not trainer reward.**

### 4.4 Style — making it look human
A sequence of reward designs to close the *look*, each measured, not eyeballed:
- **Measured human kinematics** (`style="winter"`): drive the Winter normative
  gait-analysis joint curves directly — the knee double-bend and the ankle
  push-off are the signatures the eye reads as "human."
- **Swing-leg style reward** (`gpu_g1_walk22_swing_t5`): the user's insight —
  *don't imitate the ghost tick-for-tick; match the gait waveform.* Per-tick pose
  imitation actually *hurt* robustness (instantaneous tracking fights balance);
  rewarding only the *unloaded swing leg's* shape costs no balance. (The
  long-distance zero-fall figure originally attached to this run is ⛔ **retracted**
  — see the banner above. The *style* finding stands; the *distance* does not.)
- **Holistic shape reward** (`gpu_g1_walk26_shape_c8`, the `-Arms` default of that
  era): match the ghost *silhouette* (FK the leg to knee+toe Cartesian keypoints)
  and let the robot find a balanced way to hit it — balanced on all three joints vs
  the ghost. ⛔ Its long-distance zero-fall figure is **retracted**; on the current
  Newton deploy this policy **topples in ~1 s**.

### 4.5 The ghost
A kinematic, physics-free translucent G1 ("hologram") that plays the **pure gait
model** beside the real RL robot, phase-locked to its forward progress
([reference g1-ghost-demo], `run_g1_walk_ghost_demo.ps1`). The gap between the
ghost and the robot *is* what the RL layer is correcting. The reusable trick:
strip **all** inertial+collision from the URDF so the importer emits zero Physics
→ Newton skips it → the joints move kinematically via `motor.setPosition`.

### 4.6 The drunk-gait diagnosis and the sim-to-deploy wall
The user observed the robot walking "like it's drunk" and splaying its legs. The
diagnosis ([g1-walk-rl-journey §14]): **one root cause, three symptoms** — the
robot never learned deliberate **lateral weight transfer** (shifting the CoM over
the stance foot), so it found two cheats the reward allowed — a *wide base*
(= leg spread) and *reactive catching* (= the drunk wobble) — and when forced
narrow it had neither base nor skill (= narrow-stance falls). The fixes (adaptive
state-dependent phase; a COM-over-stance-foot reward; torso-stillness) were built
and are sound *in the trainer*, but hit a **sim-to-deploy wall**: aggressive new
gaits overfit `mujoco_warp` and **collapse under Newton at the launch**. Closing
that gap (heavier DR / contact-matching / a Newton fine-tune) is the open
frontier for the *deliberate* gait; the balanced shape-c8 gait stays shipped.

### 4.7 Stop in the middle — a velocity-conditioned single policy (2026-06-14, today)
The milestone: *command the robot to stand in the middle while walking, then
continue* ([g1-walk-rl-journey §15]). A two-policy walk↔stand hand-off **failed**
(stopping a gait is a capture-point problem; a stride-fade can't arrest forward
momentum). The clean answer is **velocity conditioning**: one policy with a
commandable forward speed (including 0 = stand) in the obs and reward; commanding
`vx → 0` makes it decelerate and stand, `vx → 0.4` resumes — no fragile hand-off.

**`gpu_g1_walk29_vc_c10`:** the *transition* worked — **walk 0.41 → stand
0.07–0.08 m/s → resume 0.41**, over repeated stop/go cycles
(`run_g1_walk_vc_deploy.ps1`, `plot_vc_milestone.py`). ⚠️ The accompanying
distance / zero-fall **durability** figure is an old-deploy-path number and has
**not** been re-verified (see the §4 retraction banner) — the claim here is the
walk↔stand↔walk transition, not endurance. The hard lessons:
- **THE bug:** the deploy's gait-clock-freeze knob defaulted *on* while training
  had it *off* → it froze the phase at the stand, which is OOD for a no-freeze
  policy → a string of phantom "launch falls" misattributed to other levers. *A
  deploy knob that shapes the obs must match training* — the kp20↔kp400 lesson in
  a new costume.
- **A perfectly static stand is unreachable for a policy-based stand** — the
  policy *is* the balancer, so a ~0.07–0.13 m/s residual creep is its cost (it
  tightens with training but doesn't reach zero). Three "make it static" fixes
  (clock-freeze, sharp stand reward, fade-to-pure-pose) each broke either the
  launch or lateral balance.
- **Concentrate the command distribution** on the speeds you actually need; a
  flat prior dilutes the skills you care about.

---

## 5. Spot walks straight — the G1 recipe ported back (2026-06-12)

The mature G1 walking recipe (foot-space model + IK + residual RL) was ported
*back* to Spot, superseding the original residual stack and an intermediate
Raibert layer ([project spot-walk-g1-recipe]). `gpu_spot_walk_main` deploys
**+119.3 m / 298 s, zero falls, dead straight at 0.4 m/s**, with `wz`-command
steering (tangential stance sweep + a deploy heading-hold PD). The bare trot
model never falls and drifts backward at the *same* −0.03 m/s in both trainer and
bridge — sim2sim agreement means the residual trains on exactly the artifact it
must cancel. (Gotcha logged: new `torch.onnx` exports **external** weights
`policy.onnx.data` — copying only the `.onnx` makes ORT silently run zero residual,
and the bare gait *also* never falls, so a fake "endurance" run looks real.)

This closed a long-standing item: earlier Spot work *collapsed under Newton*
(a W6 lateral/roll fidelity gap, traced to W1-mesh self-collision and fixed
in-bridge). The G1 recipe is what made Spot a *real* Newton deploy.

---

## 6. Manipulation — residual RL for grasping

The residual idea extends past locomotion. On the **6-DOF cobot arm**, a learned grasp
residual lifted bin-emptying from **15 → 18 of 36** ([project grasp-residual-rl]).
But it also produced a clean negative: **pushing does not crack wall-locked
cubes** — four variants (single-step, sequential PPO, heuristic) all ≤ no-push,
because the limit is the 85 mm end-effector geometry (no two-sided clearance),
not the policy. The real lever was hardware: a **suction cup**. With a
contact-honest vacuum tool the same cell empties the **whole 36-cube bin, 36/36**
([project suction-bin-pick-36]), and shape-agnostic suction picks mixed
cubes/tubes **12/12** and sorts an 18-tote conveyor line 18/18
([project anypick]). The RL here is a thin residual/declutter layer on top of
analytic IK + scripted manipulation — exactly where the meta-principle predicts
RL helps a little and geometry/hardware helps a lot.

(Most of the manipulation wins are *not* RL — they're analytic IK + friction-grasp
+ engine fixes. They live in the manipulation docs, e.g.
[real-grasp-and-the-cold-first-load-trap.md](real-grasp-and-the-cold-first-load-trap.md);
included here only for the residual-RL thread.)

---

## 7. The infrastructure that made it possible

- **GPU `mujoco_warp` trainer.** The workhorse (`gpu_mjwarp_*_trainer.py`):
  4096 parallel envs, the actor on GPU, env compute via zero-copy `wp.to_torch`
  views, CUDA-graph capture of the substep loop. ~100k+ env-steps/s; a long run
  is ~80 min, a chunk ~8 min. Heavy DR is only affordable *because* training is
  this cheap — DR is the recipe's load-bearing idea.
- **A throughput win worth knowing** ([reference g1-train-throughput]): the reset
  path's per-step `mjw.forward` was ~60% of train time; swapping it for
  `mjw.kinematics` was a **bit-identical ~2.5× speedup** (39k → 98k env-steps/s).
  The physics ceiling is ~265k at 4096 envs — don't raise `--envs` past that.
- **Newton bridge perf** ([reference newton-bridge-perf]): deploy was
  GPU-sync/launch-bound, not solver-bound. Per-step `joint_q` caching, dirty-gated
  state copy-in, CUDA-graph substeps, and skipping the redundant Newton collide
  under MuJoCo contacts took deploy from **0.08× to 4.5× realtime**. (New mutation
  sites *must* set `_mjc_dirty`.)
- **All RL worlds on Newton** ([project rl-newton-migration]): the last 11 ODE RL
  worlds were flipped to Newton; a Spot self-collision regression was fixed
  in-bridge.

The training engine and the deploy engine being *almost* identical is the whole
game — it is why the gap is closeable at all, and why every "gap" in this journey
turned out to be a specific, findable difference (gains, obs frame, ONNX export,
constraint budget) rather than irreducible sim2sim noise.

---

## 8. Cross-cutting lessons (the rules, paid for in falls)

1. **The gap is the problem, not the policy.** Almost every "the robot won't
   stand/walk" turned out to be a *specific* trainer↔deploy difference, not an RL
   failure. Diff the two before tuning rewards.
2. **A deploy knob that shapes the obs/dynamics MUST match training.** The
   greatest-hits list: kp20↔kp400 stiffness (20× — no DR covers it), world-vs-body
   ang-vel frame, tanh-vs-clamp ONNX export, the velocity-conditioned clock-freeze
   default. Each cost days.
3. **Reproduce in the simplest setting.** G1's "balance gap" was solved by
   stepping the model in *plain MuJoCo with zero policy* — it tipped there too, so
   it was never the engine or the policy.
4. **Select policies by DEPLOY samples, not trainer reward.** PPO chunks
   oscillate; the trainer-best chunk is often the deploy-worst. Deploy is
   deterministic, so ship one *verified* chunk.
5. **Instrument, don't guess.** Every multi-cause mystery (the 4-cause walk gap,
   the drunk gait) yielded to a trace/obs-diff/measurement tool, not to intuition.
6. **The analytic layer must be strong, and the residual bounded.** Strong
   Layer 1 → fast, robust learning. Strong Layer 1 *and* a saturating baseline
   (Atlas) → the residual earns nothing. A residual that isn't bounded fights the
   model into a local optimum.
7. **Match the engine on both sides** (`OMNISIM_NEWTON_FORCE_MUJOCO=1`,
   `MJWARP=1`, stiffness-matched MJCF) — the same policy walks 25.9 m on mjwarp vs
   6.6 m on CPU MuJoCo. The engines genuinely differ.
8. **The OmniSim URDF importer is a real source of model discrepancy** — it places
   feet further back than native `add_urdf` and flattens joint-origin rpy into
   axes. Re-derive nominal poses on the *Newton dump*, never the URDF-MJCF.
9. **Beware the silent no-op.** Copying only `policy.onnx` without
   `policy.onnx.data` → ORT runs zero residual silently; and the bare gait often
   doesn't fall, so the fake result looks real.
10. **A correction layer on a *complete* baseline is a passenger at best and a
    *saboteur* on a marginal static task — never assume it's free** (2026-06-23,
    stand-and-hold-cubes). Law (c) says a residual on a complete baseline earns
    *nothing*; the sign of "nothing" depends on the task's stability margin. On a
    statically-stable, redundant task (Spot walk) the residual is a harmless
    *passenger*. On a 2-contact inverted-pendulum *stand* (G1) an RL residual
    **topples a stationary robot in ~1.3 s with no push** — its authority reopens
    the very instability the stiff hold closes, and the sim-to-deploy gap mis-times
    the feedback loop with no redundancy to absorb it. It is **not RL-specific**:
    even a hand-coded reactive ankle lean is load-bearing for the marginal G1 but
    topples the stiff H1 (~6 s) and Valkyrie (~18 s), which hold every thrown cube
    *passively*. **Standing is "hold a solution" (deterministic, add nothing);
    walking/get-ups/throws are "keep finding the solution" (where a learned layer
    pays).** Gate any correction layer — learned or hand-coded — on a measured
    per-robot delta over the bare hold. (See
    [rl-two-layer-architecture.md §3.8](rl-two-layer-architecture.md#38-stand-and-hold-cubes--passenger-vs-saboteur-and-why-a-residual-on-a-static-stand-goes-net-negative-2026-06-23).)

---

## 9. The deterministic-brain turn — model-based control, no RL (current frontier)

After the entire arc above, the RL walker — even the best ghost-styled,
stop-on-command policy — still doesn't move quite as *cleanly and stably as the
kinematic ghost we designed for it*. So the program opened a deliberately
different front (2026-06-14): drop RL (for now) and build a **deterministic
"brain"** — a single hand-engineered `state → joint-angle` controller, *"a
deterministic neural net,"* where every coefficient is ours, not learned. Goal:
make the physical (Newton) robot track the ghost stably with classical
model-based control. This is a **multi-agent research effort** — several sessions
attack it in parallel and are compared on one shared harness.

### 10.1 The idea
The ghost (`g1_human_gait.py`) already *is* the feedforward half of a brain — the
ideal foot trajectories. What RL approximated and never fully nailed is the
*feedback* half: the **stabilization** that holds the robot on the ghost's
trajectory while gravity and contact try to topple it. So the brain =
**ghost feedforward + deterministic balance.** The balance is classical humanoid
control — the Linear Inverted Pendulum Model (LIPM) and the capture point /
Divergent Component of Motion (DCM): step toward where the CoM is falling (the
*controlled fall*), an ankle CoP strategy for fine balance, lateral weight
transfer, and an adaptive phase clock that lingers when off-balance.

### 10.2 How we simulate and evaluate it
The decision that makes this tractable: **reuse the entire RL deploy harness** so
brain-vs-ghost-vs-RL is apples-to-apples.
- The brain is a drop-in — `g1_walk_deploy` calls it via `G1_BRAIN=1`, and the
  brain module is **selectable** (`G1_BRAIN_MODULE`) so a future attempt can share
  one world / physics / trace / logger. The several early attempts were folded
  into ONE canonical brain at `projects/policies/control/g1_brain.py` (now the default).
- Launchers run it headless or GUI in **Newton/mujoco_warp at kp100** — the exact
  engine and joint drive the gait model is calibrated to
  (`run_g1_brain_deploy.ps1`). Every gain is
  `G1_BRAIN_*` env-tunable, so a tuning iteration is a *re-run, not a code edit*.
- **Evaluation is deterministic** (nworld=1 Newton): the same brain + gains give
  the same trajectory every run, so one run is a verdict. The score is the RL
  score — forward distance, fall time, and tracking error vs the ghost (the
  deploy trace). The **floor** is the open-loop ghost with no feedback (~1.5 s to
  a fall); the **ceiling** was taken at the time to be the fully-RL policy — ⛔ but
  the long-distance figure used for that ceiling is **retracted** (see §4), so the
  honest reproducing ceiling of that era is the finite `ft_pdoff_clamp` walk
  (+5.9 m / 33.8 s, then falls).

### 10.3 Where it stands (honest)
The first brain (LIPM capture-point lateral foot placement + ankle CoP on the
ghost feedforward) **runs end-to-end in Newton but does not yet walk**:
reproducible **FALL@2.26 s** in roll, drifting backward — it *buys ~0.7 s* over
the open-loop ghost but topples. Two blockers, both matching the program's own
prior finding (*"the open-loop ghost drifts backward; the RL residual is what
walks"*):
1. **No forward propulsion** → the body drifts backward and tips. Forward drive
   comes from the **stance-leg ankle push-off**, not from reaching the swing foot
   forward (tried — it regressed, drifting back *more*).
2. **Lateral roll-over** → the ankle CoP + open-loop sway aren't a strong enough
   frontal-plane stabilizer; the proper fix needs the CoM offset *relative to the
   stance foot* (raw torso roll includes the intended sway, so it over-drove).
The lateral capture-point **sign is verified correct** (disabling it falls
*earlier*, forward, at ~2.0 s).

**Roadmap:** leg forward-kinematics for the foot positions → a true LIPM/DCM
relative to the stance foot; a stance ankle **push-off** propulsion term; a
**settle-then-walk launch** (the same launch wall every RL policy hit); then
re-tune. Detailed findings live in the `g1_brain.py` docstring.

This is, by design, the hard part — it is *why* RL was reached for in the first
place. The deterministic brain is a **live research frontier, not a shipped
result**.

---

## 10. Current status

| Robot / task | Trainer | OmniSim Newton **deploy** | Verdict |
|--|--|--|--|
| **Spot** walk | model+residual, straight | **+119 m / 298 s, 0 falls, straight, `wz` steering** | ✅ shipped (`gpu_spot_walk_main`) |
| **G1** stand | pure pose holds; RL hurts | **holds the stand (pure pose, 0 falls); the RL residual *destabilises* it (~2.4 s)** | ✅ stand shipped — classical statics, **not** RL |
| **G1** walk | human-gait + residual | ⛔ the era's **long-distance zero-fall deploy figures are RETRACTED** (unreproducible, old deploy path — see §4); `gpu_g1_walk26_shape_c8` deploy-topples ~1 s. The reproducing Newton-deploy walk is finite (`ft_pdoff_clamp`, **+5.9 m / 33.8 s**, then falls) | ⚠️ **walk NOT durable** — durable ≥80 % deploy walk OPEN |
| **G1** stop-in-the-middle | velocity-conditioned | **walk → stop → resume** demo; ⚠️ the **+50 m** is a trainer/old-path figure — deploy durability not re-verified | ⚠️ see [rl-current-state.md](rl-current-state.md) |
| **G1** *deliberate* (non-drunk) gait | trains; narrows stance | collapses at Newton launch | ⏳ open (sim2deploy wall) |
| **H1** walk | mjwarp human-gait + residual (run 3 champion) | **finite ~2 s bout** (deploy 2.03 s / +1.45 m forward, then topples); deploy-solver fine-tune **regressed** it | ⚠️ **walk NOT durable** — see 2026-06-24 note below + [h1-walk-rl-journey.md](h1-walk-rl-journey.md) |
| **Atlas** stand | PPO ≈ analytic baseline | never deployed | ❌ honest negative (PPO adds nothing at 30 DOF) |
| **6-DOF arm** grasp/bin-pick | grasp residual / declutter PPO | suction 36/36, anypick 12/12, line 18/18 | ✅ shipped (RL is a thin residual) |
| **G1** deterministic brain | *(none — no RL)* | runs; FALL@2.26 s (capture-point v1) | 🔬 research frontier (§9) |

**One-line state of the program:** locomotion is solved across two robots
(Spot, G1) with the *same* two-layer recipe, the deploy gap is now a
*findable-difference* problem rather than a mystery, and the open frontiers are
(a) a *deliberate* weight-transfer gait that survives Newton, (b) making the RL
residual earn more than the analytic baseline on bipeds, and (c) the new
**deterministic-brain** front — matching the ghost with hand-written model-based
control, no RL (§9).

> **Update 2026-06-24 (H1 walk — the deploy-physics fine-tune was tried, and it
> regressed).** H1 walks only as a **finite ~2 s bout** in deploy (run 3 champion:
> 2.03 s / +1.45 m forward, then topples) — the same *finite-bout* shape as G1, not a
> durable walk. The tempting fix — fine-tune the champion **through the exact deploy
> solver** (`gpu_newton_h1_walk_trainer.py`, `newton.solvers.SolverMuJoCo`, `cf200cdc`)
> — made it **worse**, not better: 1.58 s/back on a fresh-URDF model, 0.66 s/back on the
> matched dumped-MJCF model. Two lessons feed back into rule #2 and #7: (1) **"same
> solver ≠ matched physics"** — a fresh `add_urdf` build silently uses newton's *default*
> friction, not the deploy's `mu=2.0`; load the dumped deploy MJCF via `newton add_mjcf`
> (`da8b171a`); (2) even matched model **+** matched solver is not enough — run 3 and the
> fine-tune are byte-identical in the batched trainer yet diverge in deploy, because the
> binding gap is the **launch initial condition** (the deploy's ~0.3 s settle lean +
> residual velocity, absent in the batched reset) and the **observation pipeline**
> (world-frame `getVelocity` + finite-diff `qd` vs the trainer's exact MuJoCo-frame
> `qvel`). So "train in the deploy solver" is necessary but not sufficient. Full writeup:
> [h1-walk-rl-journey.md](h1-walk-rl-journey.md); canonical status defers to
> [rl-current-state.md](rl-current-state.md).

---

## 11. Deep-dive index

| Doc | What it covers |
|--|--|
| [spot-residual-rl.md](spot-residual-rl.md) | the original quadruped model+residual recipe; the "passenger" finding |
| [g1-stand-rl-playbook.md](g1-stand-rl-playbook.md) | the G1 stand recipe, the 8 dead ends, the deeper-squat solve |
| [g1-walk-rl-journey.md](g1-walk-rl-journey.md) | the full walk arc (§1–§15): stiffness mismatch → human gait → style → drunk-gait → stop-in-the-middle |
| [atlas-stand-rl-journey.md](atlas-stand-rl-journey.md) | 30-DOF port, mass-DR lottery, the PPO ceiling |
| [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) | the generalized heavy-DR recipe (with scope caveats) |
| [rl-two-layer-architecture.md](rl-two-layer-architecture.md) | the deterministic-controller + bounded-residual spec |
| [rl-accelerated-training.md](rl-accelerated-training.md) | the GPU `mujoco_warp` speedups |
| [humanoid-balance-gap.md](humanoid-balance-gap.md) | historical "why bipeds are hard" (its original LIPM conclusion was wrong; kept for context) |
| [real-grasp-and-the-cold-first-load-trap.md](real-grasp-and-the-cold-first-load-trap.md) | the manipulation thread (and the cold-first-load trap) |
| [rl-current-state.md](rl-current-state.md) | the canonical "is it done?" status; supersedes this doc for current-state claims (G1/locomotion re-verified 2026-06-19; Spot/Atlas date to 2026-05-29) |
| `projects/policies/control/g1_brain.py` | THE deterministic brain (§9): capture-point control + forward CoM bias + a FINDINGS log; run with `projects/policies/research/runners/run_g1_brain_deploy.ps1` |

*Last updated: 2026-06-14 — after the velocity-conditioned stop-in-the-middle
milestone shipped, plus the §9 deterministic-brain turn (capture-point v1,
runs in Newton, FALL@2.26 s — a live research frontier).*
