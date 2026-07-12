# The Ghost-Tracking Motion Pipeline

> **STATUS (2026-07-03): superseded as the how-to.** The Shadowing pipeline is now the FLAGSHIP
> method with a maintained implementation + launcher at
> [projects/policies/training/](../../projects/policies/training/README.md) and formal reference
> rules + pre-training validator in [ghost-design-rules.md](ghost-design-rules.md). Start there;
> this doc remains as the original pipeline description.


> **⚠️ RENAMED → "Shadowing" (2026-06-20).** This architecture is now called **Shadowing**
> (reference = "the shadow"; the robot "shadows" it). The current, paper-oriented write-up
> with the generator → verifier → tracker breakdown lives in
> **[shadowing.md](shadowing.md)** — read that. This doc is kept for history.

**Feasible reference (planner) + robust RL tracking — robot- and motion-agnostic.**

> This is OmniSim's **canonical way to make any robot perform any motion in deploy.**
> It is the realized architecture behind the "universal motion tracker" north star
> ([g1-universal-tracker.md](g1-universal-tracker.md)). Read this before starting any
> new motion-control / imitation / "make the robot do X" effort, on any robot.

---

## The thesis (one paragraph)

To make a robot perform a motion robustly in the real deploy engine, **split the
problem in two.** A **planner** produces a *dynamically-feasible* reference
trajectory — the "**ghost**" — that the robot can physically execute (it obeys the
robot's dynamics, contacts, torque limits, and balance). An **RL policy** then learns
to **track that reference robustly** across disturbances and the sim→deploy gap.
**The planner describes the problem accurately; RL solves the robust control.**
Equivalently: **planning says *what*, control learns *how*.**

---

## Why this architecture (the reasoning that led here)

A purely **kinematic ghost** (hand-drawn joint angles + base pose per frame) is a
*puppet*: it has no physics, cannot fall, ignores momentum and contact, and can
describe motions the robot physically cannot do. "Just track the ghost" then turns
out to be a **full dynamic balance-control problem in disguise** — the puppet hides
the 90% that is hard (staying upright every millisecond, handing weight between
contacts, respecting torque limits). That is *why* mimicking a kinematic ghost is so
hard, and why our hand-drawn sit-to-stand ghost left the robot on a contact
knife-edge (the seated start launched it forward).

There are really **two distinct sub-problems**, each suited to a different tool:

| sub-problem | "the question" | best tool | weakness alone |
|---|---|---|---|
| **WHAT** — a feasible motion exists; produce it | planning / trajectory optimization | finds dynamically-valid trajectories (handles contact handoffs, CoM-in-support, torque limits as *constraints*) | open-loop, brittle to disturbance/model error |
| **HOW** — reproduce it under uncertainty | reinforcement learning | robust closed-loop feedback; crosses the sim→deploy gap (domain randomization) | bad at *discovering* a complex motion from scratch |

The pipeline combines their strengths: the **planner makes the reference feasible by
construction**, so the **RL policy never has to discover the motion** — it only has to
*robustify* it, which is exactly what RL is good at. This is the modern humanoid
recipe (trajectory-optimization + learned tracking / model-based reference + RL
residual).

---

## The three ghost rules (design discipline)

Every reference must satisfy these (saved as memories; they are now *enforced by the
planner* rather than checked by hand):

1. **GHOST-FIRST** — design the reference, *show it* to the maintainer, *agree*, then build
   the mimic. Never train against a ghost nobody approved.
2. **ACHIEVABLE** — the robot must be physically able to reproduce it. **Upgraded
   meaning:** *dynamically* feasible (from the planner), not merely *quasi-static*
   (hand-checked CoM-over-feet). Quasi-static feasibility is necessary but **not
   sufficient** — it ignores momentum and contact transitions, which is the gap that
   a trajectory optimizer closes.
3. **RESPECT PHYSICS / SURROUNDINGS** — no floor penetration, no clipping into
   furniture, feet planted where they bear weight, friction cones respected.

A trajectory-optimized reference satisfies all three **by construction**, because they
are constraints in the optimization.

---

## The pipeline (four stages)

```
  (A) INTENT            (B) PLAN                     (C) TRACK              (D) DEPLOY
  human goal/      trajectory optimization      RL policy learns to     run in Newton/
  keyframes   -->  over the ROBOT'S dynamics --> TRACK the reference --> real; robust
  + constraints    => FEASIBLE reference table   under domain random.    via the DR in C
       |                    (the "ghost")              |                        |
   ghost-first          achievable + physics-       reward = match the      sim->deploy
   (show+agree)         respecting BY CONSTRUCTION   reference + alive       gap crossed
```

**Stage A — INTENT (human).** Specify the motion goal: keyframes / end-effector
targets / gait + velocity / "stay standing while reaching." Show a preview, agree
(ghost-first). This is the only hand step, and it specifies *intent*, not the
trajectory.

**Stage B — PLAN (the planner = the "feasible ghost").** A **trajectory optimizer**
solves for a reference trajectory over the robot's dynamics:
- **decision variables:** joint trajectory `q[t]`, base pose `base[t]`, (optionally
  torques / contact forces).
- **constraints:** equations of motion; contact (feet/hands on the right surfaces, no
  penetration, friction cones); actuation limits (torque, velocity, joint range);
  **balance** (CoM/ZMP inside the support polygon, or angular-momentum bounds);
  boundary + keyframe conditions from the intent.
- **objective:** minimize effort / jerk / deviation-from-intent.
- **output:** a **feasible reference** — the ghost the robot *can* do. The optimizer
  *naturally* solves the hard parts (e.g. the chair→feet contact handoff: it shifts
  the CoM over the feet *before* the butt leaves the chair, with feasible torques),
  so the reference has built-in stability margin instead of a knife-edge.

**Stage C — TRACK (RL).** A policy learns a feedback controller that reproduces the
reference under **domain randomization** (mass, friction, contact `solref`/`solimp`,
pushes, observation noise). Reward = **match the reference** (joint tracking + base
pose + contact schedule) + stay-alive − small smoothness penalties. Because the
reference is feasible and complete, the reward is simple ("track the ghost") and the
problem is well-posed — no hand-crafted balance terms.

**Stage D — DEPLOY.** Run the policy in OmniSim Newton (and ultimately real). The DR
in Stage C is what crosses the warp→Newton / sim→real gap (see
[sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md)).

---

## The interface that makes it general

The whole thing is held together by **one data contract: the reference table** — a
time-indexed trajectory, robot-agnostic in shape:

```
  reference[t] = { q[t]          : all joint angles
                   base_pose[t]  : pelvis/root position + orientation
                   contacts[t]   : which end-effectors bear load (schedule)  }
```

- **Any planner** (trajectory opt, model-based controller, recorded demo, retargeted
  mocap) outputs this table.
- **One generic tracker** consumes it (our `gpu_mjwarp_*` trainer already tracks a
  `REF_LEGS/ARMS/X/Z/PITCH/...` lookup table indexed by `ep_step`).
- **New motion** = new intent → planner → new reference table → *same tracker*.
- **New robot** = its model (URDF/MJCF) → planner uses it → *same machinery*.

So the pipeline is a function: **`(robot model, motion intent) → deployable policy`**.
That is the universal motion engine.

### Generality, concretely
- **Stand-from-chair (G1)** — intent: sit→stand keyframes. Planner finds the feasible
  weight-shift + chair→feet handoff. RL tracks. *(current test-bed)*
- **Walk** — intent: gait + commanded velocity. Planner produces a feasible gait
  (or a gait library); RL tracks. (The walk effort approximated this with a hand-built
  foot-space model — the planner *generalizes* that step.)
- **Manipulate while standing** — intent: end-effector trajectory + "stay balanced."
  Planner produces a whole-body feasible motion (reach + counter-balance); RL tracks.
- **Spot / other robots** — identical pipeline with Spot's model; quadruped balance
  constraints make Stage C even easier.

---

## What we already have vs. the missing piece

| piece | status |
|---|---|
| Generic **tracker** (ghost-tracking trainer, reference-table interface, RSI, DR) | **have** — validated; tracks rise/stand/sit cleanly when the reference is given |
| **Deploy** recipe (DR to cross the gap) | **have** — [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) |
| The three **ghost rules** | **have** — memories + above |
| **Planner** (trajectory optimization → *dynamically*-feasible reference) | **MISSING — the focus now** |

Today the reference is **hand-drawn** (kinematic, only quasi-statically checked),
which is exactly what leaves the seated start on a contact knife-edge. Stage B (the
trajectory optimizer) replaces the hand-drawing and removes the knife-edge.

---

## The planner: trajectory optimization (the build target)

**Goal:** given the robot model + the motion intent, output a feasible reference table.

**Options (pick by rigor vs. effort):**
- **MuJoCo MPC (mjpc)** — predictive sampling / iLQG over the MuJoCo model; closest to
  our stack, handles contact, relatively quick to stand up a first solution.
- **Direct contact-implicit trajectory optimization** over the MuJoCo dynamics (with
  analytic/finite-diff derivatives + an NLP solver) — most control over the
  formulation.
- **Legged-TO libraries** (Crocoddyl / OCS2 / TOWR) — mature, but a heavier
  integration into the OmniSim/MuJoCo stack.

**Formulation (generic):** `min effort/jerk` s.t. `dynamics + contacts + limits +
balance + keyframe/boundary` constraints → feasible `q[t], base[t], contacts[t]`.

**First target — G1 sit-to-stand:** the optimizer must keep the CoM inside the support
through the chair→feet handoff with feasible torques. The result is a reference that
the existing tracker can follow *from the dead-seated start* (no knife-edge), which is
the exact thing the hand-drawn ghost could not give us.

---

## Is this RL or "just ML"?

Both, split by job:
- **The reference is a planning / optimal-control problem** — solving physics +
  optimization, *not* learning.
- **The robust tracking is an RL problem** — a feedback policy under uncertainty.
- (The reference *could* also be **learned** from a motion corpus — supervised /
  imitation — which is how you scale to *many* motions later. For a single motion,
  trajectory optimization gives a guaranteed-feasible reference directly.)

So "ghost describes, RL solves" = "**planning describes, control solves**."

---

## Testing achievability EARLY (don't trust a static CoM check)

**The single most expensive mistake on this project: trusting a static feasibility
check and discovering dynamic infeasibility only after hours of training.** The G1
sit-to-stand burned ~9 training runs tuning references/rewards/geometry/DR to fix a
persistent ~40° forward "bow", on the belief that a *no-step* upright stand was
achievable — because the hand-drawn ghost passed a **static CoM-over-feet check**. It
was never dynamically achievable: the chair forces the feet forward and the G1 has **no
torso-pitch joint**, so the rigid pelvis cannot get its CoM over the planted feet
without a **step**. A static check cannot see this. The fix was to change the ghost to
an achievable motion (rise + **step forward** so the feet end *under* the body — the
stable `g1_stand` configuration).

**Run these cheap tests BEFORE committing to a long training campaign on a new ghost:**

1. **RENDER the target/achieved pose and LOOK at it.** Offscreen
   `mujoco.Renderer` → PNG (see `_scratch/render_ref.py` / `render_pose.py`), or the
   live viewer (`SITSTAND_VIEW=1` in the trainer). Numbers (`tilt=38°`) under-convey;
   one image of the forward-collapse made it obvious instantly. We were "flying blind"
   on numbers for many runs.
2. **HOLD-TEST the target pose.** Train a quick policy that ONLY holds the standing
   pose (`--rsi 1.0`, strong upright reward, ~1000 iters). If it can't even *hold* the
   pose (G1 settled to ~18° tilt from a standing start), the pose is **not a stable
   attractor for the controller** → the full task won't work. Stop and rethink the
   ghost; do not keep tuning the transition.
3. **A recurring failure mode is a STRUCTURAL signal.** If the same failure (here: the
   bow) recurs across every variation and matches the task's whole history, treat it as
   a morphology/geometry limit, not a reward bug.

Budget ~10 minutes on a render + hold-test before spending hours tuning toward a ghost
that may be physically impossible. This is the operational form of the **achievable
ghost** rule.

---

## Roadmap

1. **Build Stage B for the G1 sit-to-stand** (trajectory optimization → feasible
   reference). ← current work
2. **RL-track it** with the existing trainer; verify the seated-start knife-edge is
   gone; deploy in Newton.
3. **Generalize the planner interface** so a new motion is just a new intent +
   constraints (walk, reach-while-standing).
4. **Second robot (Spot)** through the same pipeline.
5. **Scale via a learned reference** (motion corpus) for many motions — the full
   universal tracker.

## Related docs
- [g1-universal-tracker.md](g1-universal-tracker.md) — the north-star objective this realizes.
- [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) — Stage C/D (crossing the deploy gap).
- [g1-sitstand-journey.md](g1-sitstand-journey.md) — the first test-bed + why the hand-drawn ghost failed.
- [g1-improved-shadow.md](g1-improved-shadow.md) — earlier "achievable shadow" work (record/retarget references).
- Ghost rules (memories): ghost-first, achievable, respect-physics.
