# OPEN PROBLEM: the legitimate G1 stair climb

**Status: OPEN (2026-07-11, end of the two-day stair campaign).** The goal — a G1 that climbs
a 5-step staircase *the way a person would* (chest forward, feet stepping tread to tread,
fully self-supporting) and stands at the top, verified end-to-end by numbers with no human
judgment in the loop — is **not achieved**. This file is the definitive statement of what IS
achieved, everything the campaign learned, the best assets, and the precise next experiments.
The ship rule is frozen: nothing is called a climb until
`projects/policies/demos/grade_stair_champion.sh` prints a **double-pass** (kinematic +
motion-legitimacy rulers) on a **live** 5-step run.

## What exists and works today

- **The shipped demo** (`projects/policies/demos/run_climb_stairs_stand.sh`): climbs all five
  3 cm risers and stands motionless on the landing, ~1-in-2 pass rate, per-run FOOT_LOG
  self-verification. **Kinematics only** — its header states plainly that it fails the
  legitimacy ruler (crane-assisted lean, knee contact). It is a demo of the pipeline, not of
  a solved skill.
- **The verification system** (the campaign's most durable product; owner-directed after
  catching a champion cheating that every kinematic gate had passed):
  - `projects/policies/training/verify_motion_legitimacy.py` — L0 upright, L1 crane support
    (wrench statistics + sustained/at-cap fractions), L2 contact purity (all-body census vs a
    per-motion allowed set), L3 feet-under-load. Motion-agnostic.
  - `projects/policies/training/certify_stair_human.py` — chest-forward window, tread
    placement vs the collider sole, lateral lane, summit stand.
  - `projects/policies/demos/grade_stair_champion.sh` — the one-command double-ruler live exam.
  - Instrumentation (env-gated, in `g1_walk_recipe.py`): `CRANE_LOG`, `CONTACT_LOG_ALL` +
    `CONTACTMAP`, `EVALYAW`, `COLLECT_ONSTAIR`/`SPAWN_STATES`.
  - Doctrine: [motion-legitimacy.md](motion-legitimacy.md) (the two-rulers rule, the
    crane-never-weans trap, verify-dynamics-not-trajectory).

## Everything the campaign learned (the ladder of discoveries, in order)

1. **The corridor-vs-torque law binds at every riser.** A 3 cm step-up demands ~0.192 rad of
   knee deviation from a flat-walk reference (7 cm: 0.233). Every "chest-forward" failure and
   every historical climb-ghost failure traced to corridors ≤ the demand: the straight climb
   was infeasible *by construction*, so policies found whatever the constraints left open —
   the yaw-twist, the knee-lever, the crane-lean. **Audit the constraints before blaming the
   learner.** Corridor 0.22–0.24 unlocked the first straight climbs within ~100 iterations.
2. **Kinematic rulers pass cheaters.** The corridor-0.22 champion passed heading, placement,
   steps, and stand while the crane carried 77.5 N·m of sustained pitch torque (robot weight:
   334 N) and its knees pressed the treads 13.6% of climb ticks. Hence the two-ruler doctrine.
3. **The crane that never weans creates crane-dependent champions.** `HARNESS_GRAD_SURV=2.0`
   (unreachable) held λ=0.9 forever; PPO exploited the springs. With graduation enabled the
   crane weaned λ 0.9→0.30 on the flat-ghost route (measured capability ceiling ~surv 0.5-0.6
   at 0.30-0.35) and λ 0.9→0.35 in a *single 400-iteration run* on the ghost route.
4. **Two phantom "engine bugs" were unit mismatches.** The "live-only yaw veer" and "batched
   climbs straight" both dissolved when the same scalar was logged on both sides (`ydrift` is
   meters, not yaw; distance is not heading). **Log the SAME quantity on BOTH sides before
   declaring a train↔deploy divergence.** Open-loop parity is verified: pure-PD propulsion
   0.99 m batched ≡ 1.0 m live; LATPROBE knee tracking ~2 mrad; solver/contact params equal.
5. **The sway-precession is real physics, both sides**: the crane's ghost-sway attitude
   spring precesses a lagging gait (pure-PD veers −0.76 rad by x≈1.0, bit-identical, in BOTH
   plants). Sway is also propulsion — a level spring kills the veer *and* the drive.
   Deploy-side re-alignment knob: `HARNESS_ATT_PHASE` (−6 bins ≈ the null).
6. **The climb-ghost route had TWO locks** (3-agent autopsy with receipts,
   `_scratch/foot_redesign/stairsynth*_rl.txt`): (a) the corridor law — all eleven 7 cm runs
   at corridor 0.12 vs demand 0.233; (b) the **leash loophole** — with `SEQ_LEASH_LEAD` the
   reference waits, so standing scores surv 1.0 / gmatch 0.97 (re-proven at corridor 0.24:
   distance collapsed 0.20→0.02). **The pick: `SEQ_EVENT_CLOCK` × gentle LR (5e-5)** — the
   reference physically holds at each touchdown gate; standing earns nothing.
7. **The resurrected ghost route** (stairsynth17, 2-step curriculum ghost, corridor 0.24,
   GHOST_FF on a FRESH policy — warm-starts double-compensate, proven): sustained batched
   climbing (dist 0.37–0.47 over six evals, surv 0.96–0.99) and **11 crane graduations in one
   run**. Live at λ0.35: **the campaign's only L1 CRANE PASS** (fy 2 N, ty 11 N·m, sustained
   0.7% — genuinely self-supporting) + L3 pass. It attempts the step (foot plants on tread 1)
   but does not complete the course live.
8. **Continuations are non-monotonic**: the follow-up run recovered 0.33→0.81 then froze at
   exactly 0.610–0.613 for 275 iterations (collapsed into the wait-at-gate attractor). Keep
   the most robust checkpoint, not the latest.
9. **Entry-state parity is NOT the live blocker** (falsified 2/2 + code check): the batched
   judge starts from the standard stand pose, same as live; teleporting the live robot into
   the ghost's bin-0 deep crouch made it retreat and fall. (`STAND_POSE_LEGS` env remains for
   explicit 12-angle seeds; the `WALK_MJW_RESET=1` handoff teleport is the state-parity tool —
   note the demo composition disables it for BATON continuity.)
10. **Deploy env parity for seq ghosts**: the deploy needs `GHOST_SEQ/SEQ_TERRAIN/
    SEQ_EVENT_CLOCK/SEQ_LEASH_LEAD` explicitly — the lut's `seq:true` alone does not arm the
    clock scheme the policy trained with.
11. **Speed levers** (owner: "use the whole GPU"): `PPO_NENV=4096` → 110–138k steps/s (4× the
    1024 default) at 8.4 GB/83% on the 5070 Ti; the 2-step curriculum ghost halves routine
    cost; an early-verdict kill line (best dist < 0.30 by it≈150) convicts a shuffle in 25
    minutes instead of 2.5 hours. The largest wall-clock tax is GPU sharing between sessions.
12. Supporting facts: foot "sinking" is 22 mm of visual mesh overhanging the collider
    (physics penetration <1 mm); `maxContactJoints` is per material pair (10 default —
    raised to 40 in the stair-world generator; demo3 kept at 10 because champions version
    WITH their plant); `VX_MAX=0.7` deploy default is OOD vs trained caps ~0.4;
    the trainer CONFIG dump omits corridor/harness envs (recover true launches from session
    transcripts); W_TRACK_ANG=6 is suicide-by-penalty (~2 is right); sharpening the lateral
    gaussian regressed climbing (penalty dominance — climbing needs wobble).

## Best assets (on disk, not git-tracked until demo-referenced)

- `runs/wr_stairsynth17.pt` (+`_it350`) — the self-supporting ghost-route champion (obs 153).
- `runs/wr_stairsynth18_it200.pt` — its best continuation point (surv 0.81/dist 0.45).
- `runs/wr_stairhuman14.pt` — the corridor-0.22 kinematic champion (ships in the demo).
- `runs/wr_stairhuman19_it100.pt` — flat-ghost route's best (surv 0.92 @ λ0.35).
- `_scratch/foot_redesign/onstair_bank.npz` — 7,296 harvested on-stair states.
- Ghosts: `ghost_stair_climb3_synth_lut.json` (3 cm, gates 1–3 pass, funnel 0.942, ffdq) and
  `ghost_stair3_2step_lut.json` (its 2-step curriculum subset).

## The open problem, precisely, and the next experiments

**stairsynth17 climbs batched and hesitates live at the first touchdown gate, self-supported.**
Open-loop plant parity is proven; the divergence is closed-loop and specific to the seq
machinery. Next, in order:
1. **Trace the seq clock on both sides** (the method that killed every phantom): log phase,
   event-gate index, and leashed bin per tick, live and batched world-0, and diff. The live
   scalar mirror of the batched tensor clock is the prime suspect.
2. If the clock matches: diff the obs ref-block values at the hold (live vs a batched world
   held at the same gate).
3. When live completes the 2-step course: warm-transfer to the 5-step ghost (same obs-153
   contract), continue crane graduation toward λ≈0, clean the L2 knee-brush (knees touch the
   riser while waiting crouched), then `grade_stair_champion.sh` on the 5-step world.
4. The flat-ghost weaning ladder remains a valid parallel track (frontier: surv ~0.6 at
   λ0.30; slope positive but slow).

History, run logs, and per-run numbers: `docs/developer/train-deploy-gap.md` §2b–2c,
`docs/developer/motion-legitimacy.md`, memory file `project_live_yaw_veer_contact_parity.md`,
and `_scratch/foot_redesign/stairhuman*_rl.txt` / `stairsynth*_rl.txt`.
