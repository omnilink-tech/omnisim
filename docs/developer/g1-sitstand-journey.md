# G1 sit → stand → sit mimic — journey + state

> Note: file paths in this doc were repointed after the 2026-06 policies consolidation (rl → policies; robot artifacts now under `projects/policies/research/{controllers,worlds,runners}/`).

## UPDATE 2026-06-20 — pivoted to a GHOST-TRACKING recipe (upright stand)

The maintainer rejected the v5 "stand" below as a **bow** (~28–40° forward lean, not a real
stand). Root cause: the feet plant ~0.38 m forward and can't reposition without a step,
so the CoM stays behind the feet → the robot bows. Straight-knee / bent-knee / strong-
upright / foot-under-reward all bowed ~28° — **not fixable by reward shaping.**

**New architecture (the right one): design the FULL achievable motion in the ghost,
reward the robot to just TRACK it.** Three ghost rules now govern this (saved as
memories): (1) **ghost-first** — design+show+agree before mimicking; (2) **achievable**
— the robot must be able to do it (CoM over feet verified); (3) **respect physics** —
no floor penetration / chair clipping (use IK to keep feet planted).

- `projects/policies/control/gait/g1_sitstand.py` REBUILT on closed-form leg IK (reuses
  `g1_human_gait._leg_ik`/`_ankle_for_foot_pitch`): feet **planted at x=0.28** (clear
  of the 0.22 chair) on the floor; body **leans forward + shifts over the feet**, rises
  to **upright** (pelvis 0.80) in front of the chair, sits back. Achievability proven by
  a **CoM-over-feet quasi-static check** — the vertical (no-lean) version TIPPED BACK
  during the rise; the lean+forward-shift fixed it.
- `gpu_mjwarp_g1_sitstand_trainer.py` REWORKED to **ghost-tracking**: the whole
  reference is precomputed as lookup tables (REF_LEGS/ARMS/X/Z/PITCH/B by ep_step);
  reward = exp-track the 13 legs + base x + z + lean − tiny penalties (NO hand-crafted
  balance terms); OBS_DIM 53; RSI poses from the table. **Validated** — learns cleanly
  (reward ~2.9, value climbing), policy tracks rise→upright→sit when started mid-motion.
- Tools: `run_g1_sitstand_ghost_preview.ps1` (design preview), `run_g1_sitstand_compare.ps1`
  (A/B), trainer `--eval` phase trace + `SITSTAND_DUMP=<csv>` (record achieved ghost).

**OPEN BLOCKER (next):** the dead-**seated start** (t=0) is unstable — the robot flies
forward off the chair (x→1.0) within 0.5 s. Seated CoM is behind the feet (must lean on
the chair) but the chair contact ejects it; RSI hides this in training. Policy
`gpu_g1_sitstand_track1` tracks mid-motion but fails from seated. The sweep showed it's
a **contact knife-edge** (a 2 cm spawn change flips rest↔launch) — a *quasi-static*
hand-drawn ghost is fundamentally not enough here.

**THE FIX (2026-06-20, agreed direction): a real PLANNER.** Per
[ghost-tracking-pipeline.md](ghost-tracking-pipeline.md), replace the hand-drawn ghost
with a **trajectory-optimized, dynamically-feasible** reference: the optimizer keeps the
CoM in the support through the chair→feet handoff with feasible torques, so the seated
start has built-in margin (no knife-edge). Then RL-tracks it (the tracker already
works). ~~Also the deploy controller `g1_sitstand_mimic.py` is STALE (51-dim obs) —
rewrite for the 53-dim track policy before deploying.~~ **RESOLVED (c2c25f51):** the
deploy controller now mirrors the 53-dim trainer obs exactly (Stage D wired). Sit-stand
is now the **first test-bed for the general pipeline**, not a one-off.

---

## (2026-06-19, superseded by the pivot above)

> **Status:** SOLVED in sim (clean rise → 5 s stand → sit back, mean pelvis-height
> error 0.063 m). In the OmniSim **Newton deploy** the robot reliably does **one full
> cycle** but stands **hunched ~40°** and falls if the motion loops — the warp→Newton
> **model gap**. Cheap gap-closers are exhausted; the clean-upright deploy is the open
> frontier (model-match retrain / Newton fine-tune). This extends the seated mimic
> ([g1-sit-mimic-plan.md](g1-sit-mimic-plan.md)) to a balance task with a vertical
> excursion, and is a test-bed for the universal tracker
> ([g1-universal-tracker.md](g1-universal-tracker.md)).

## Objective
The ghost stands from the chair, stands in front of it for 5 s, then sits back — and the
robot must mimic this motion, dynamically balancing (RL), not pinned.
Per the **achievable-ghost rule** ([g1-improved-shadow.md]) the
reference must be physically reproducible; the displayed ghost replays the robot's
**achieved** motion, not an idealized one.

## The pieces (all new this session)
| file | role |
|---|---|
| `projects/policies/control/gait/g1_sitstand.py` | the shared reference: blend SEATED↔STANDING over a timeline (sit 1 s → rise 2.5 s → stand 5 s → sit 2.5 s → seated 1.5 s); `full_targets(t)`, `ref_pelvis_z(t)`, `ref_pelvis_x(t)`, `blend(t)`. |
| `projects/policies/research/training/gpu_mjwarp_g1_sitstand_trainer.py` | GPU mujoco_warp PPO trainer. 51-dim obs `[vlin, vang, proj_g, q-ref_legs, qd, last_action, phase(b, b_ahead, z_err)]`, 13 leg+waist actions = residual on the time-varying leg reference; arms feed-forward. |
| `projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml` | `g1_full_kp100` (kp=100 position actuators) + chair seat/back colliders. **Force-added** (`*.mjcf.xml` is gitignored). |
| `projects/policies/research/controllers/g1_sitstand_mimic/` | deploy controller — free base (never pinned), 51-dim obs mirroring the trainer byte-for-byte, leg = ref + 0.15·policy, arms feed-forward, telemetry + `-Record`. |
| `projects/policies/research/controllers/g1_sitstand_ghost/` | translucent ghost — replays the achieved CSV (base x,z,rot + 23 joints) or the idealized reference. |
| `projects/policies/research/worlds/g1_sitstand_mimic.wbt` | real (free-base, Newton) + ghost (staticBase) G1s, each with a chair. |
| `projects/policies/research/runners/run_g1_sitstand_mimic.ps1` | launcher: `[-Headless] [-Record] [-Policy <run>] [-Duration N]`. `-Record` captures the achieved cycle; default replays it as the achievable ghost. |
| `projects/policies/research/training/` (g1_sitstand trainer) | added the `g1_sitstand` trainer, run locally (cloud/ Modal wrapper since removed). |

Run it: `powershell -File projects/policies/research/runners/run_g1_sitstand_mimic.ps1 -Policy gpu_g1_sitstand_v5`
(GUI, real-time; the first ~30 s is the warmup reload, so use `-Duration 90`).

## The two root-cause bugs (both non-obvious, cost most of the session)
The policy/reward were NEVER the problem. A **passive (zero-action) playback** test —
"does the bare reference even produce the motion?" — exposed both:

1. **Dead-kp MJCF.** `g1_sit.mjcf.xml` (the seated trainer's model) was built from
   `g1_full.mjcf.xml` (`gainprm=0` on the position actuators → **kp = 0**, damping
   only). The legs had *no position-control torque*, so commanding the standing
   angles could never lift the body. The seated task only "worked" because it's
   nearly passive (the chair holds the robot up; the policy outputs were no-ops).
   **Fix:** `g1_sit_kp100.mjcf.xml` (from `g1_full_kp100`, kp=100). Open-loop
   reference then rises to standing and sits back. *Lesson: a sit-to-stand needs
   active leg torque; ALWAYS verify the trainer MJCF actuators have non-zero kp.*

2. **Seated pose's feet through the floor → spawn explosion.** At the old
   `Z_SEATED=0.44 / SPAWN_Z=0.47`, FK put the feet **4–7 cm below the floor**; the
   contact solver launched the robot up and **1.67 m forward** at t=0 — and *every*
   policy inherited the chaos (v2/v3/v4 all gave an identical 0.14 m error, the tell
   that it was physics, not control). The G1's short legs make the true free-base
   seated rest **pelvis ≈ 0.55** (feet flat). **Fix:** `Z_SEATED=0.55 / SPAWN_Z=0.57`.
   *Lesson: FK-check feet-vs-floor at the spawn pose for any seated/crouched start.*

Note: the G1 cannot put butt-on-seat AND feet-on-floor (legs too short for a 0.33 m
seat) — the seated pose is a feet-on-floor deep perch at the chair edge.

## The recipe that worked (in sim)
- **Achievable reference** (kinematic blend SEATED↔STANDING + a reference pelvis
  height the trainer rewards — that height reward is what turns "extend legs" into
  "actually rise").
- **RSI** (`--rsi 0.7`): 70 % of resets start at a random phase (some already
  standing, posed chair-consistent via `ref_pelvis_x = b·0.30`) so the policy
  experiences standing-balance directly instead of discovering it by luck.
- **Tight residual** (`--res-scale 0.15`): forces the legs to FOLLOW the
  seated↔stand trajectory; with the loose ±0.3 the policy overrode the phase and
  parked at a safe compromise height (the v3 failure).
- **x-anchor reward**: kills the destabilising forward lurch (v3 ran to x=0.43 and
  toppled, couldn't sit back).
- **NaN guards**: contact-softness DR blows up a few envs; treat non-finite state as
  a fall (reset) + sanitize reward, else the NaN poisons the gradient (the v7 crash).

Result (eval, no-DR): holds seated → rises to ~0.71 → holds 5 s → sits back.
mean |pelvis_z − ref| = 0.063 m (v5).

## Policies
- **`gpu_g1_sitstand_v5`** — the demo/deploy pick (no contact DR; least deploy hunch
  37–42°). Committed (policy + `achieved.csv`).
- v6 (upright 1.2 — no tilt improvement), v7 (contact DR 0.4), v8 (heavy DR) — all
  hold the motion in sim but none closed the deploy gap; not committed.

## Deploy gap (the open problem)
Same physics engine both sides (mujoco_warp via `FORCE_MUJOCO`), so the gap is the
**model**: trainer = `g1_sit_kp100.mjcf.xml`, deploy = `g1_23dof_omnisim_prim.urdf`
imported to Newton (primitive collision). Standing balance is far more sensitive to
this than walking, so the walk's contact-DR cure isn't enough here. Deploy behaviour
(matched kp=100): **cycle 1 rises to 0.61, hunched ~40°, joints track 85–90 %, sits
back; cycle 2 accumulates lean and falls.** Demo plays **once** then holds seated.

**Levers tried, none closed it:** contact-softness DR (`solref`/`solimp`), body-local
angular-velocity obs fix (negligible — ω≈0 in slow motion), higher deploy stiffness
(KE=250 → *worse*, the 2.5× gain mismatch oscillates), heavy across-the-board DR.

**Remaining (the real investment, for tomorrow):**
1. **Model-match retrain** — train on an MJCF generated from the *deploy* prim URDF
   (same inertia/collision) → no model gap. Most principled.
2. **Newton fine-tune** — a few chunks in the deploy engine (the documented
   gold-standard gap-closer; expensive).
3. **Deploy balance-assist PD** (G1_BAL-style upright PD) — the walk's hack.

## GPU strategy (settled with data)
The `mjw.kinematics` reset patch (this trainer was unpatched) gave **41k → ~150–194k
env-steps/s** on the laptop (3.6×). Probes (same trainer, env-steps/s): laptop@2048
41k → 150k patched; A100@4096 55k, **A100@8192 25k (bandwidth collapse)**;
H100@8192 64k, **H100@16384 116k (scales clean)**. **Verdict: train local** for this
size (patched laptop beats the unpatched H100); the A100 is the worst choice; use
H100/B200 only when a task genuinely needs 16k+ envs (won't fit the laptop's 12 GB).

## Next session
Pick a deploy gap-closer (model-match retrain is the cleanest first try), or bank the
sim-proven recipe and advance the universal tracker. The sim recipe is reusable for
any seated/crouched balance motion.
