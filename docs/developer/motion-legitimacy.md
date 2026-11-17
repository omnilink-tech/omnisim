# Motion legitimacy: verify the DYNAMICS, not just the trajectory

**Status: doctrine.** Written 2026-07-11 after the owner caught the chest-forward stair
champion cheating — it passed every kinematic gate (heading window, tread placement, step
sequence, motionless summit stand) while *dynamically* the crane's attitude springs carried
its lean up the staircase (77.5 N·m sustained pitch torque on 77% of climb ticks; the whole
robot weighs 334 N) and its knees pressed on the treads (13.6% of climb ticks). No kinematic
ruler can see that. **Every demo claim must now pass TWO rulers:**

1. **The kinematic ruler** (per-motion: e.g. `certify_stair_human.py` for stairs) — does the
   trajectory look right? Heading, placement, sequence, end state.
2. **The legitimacy ruler** (`projects/policies/training/verify_motion_legitimacy.py`,
   motion-agnostic) — is the motion achieved HONESTLY? Three gates:
   - **L1 CRANE SUPPORT** — the harness must not do the work. Reports |fy| |fz| |tx| |ty|
     statistics over the motion window plus the *sustained-torque fraction* (>40 N·m) and the
     *at-cap fraction* (>340 of the 350 N·m clamp). A policy trained under a crane WILL learn
     to lean on it if the crane never leaves (see the trap below).
   - **L2 CONTACT PURITY** — a census of every robot-body contact against the motion's
     allowed-contact set (walk/climb: feet only; crawl: feet+knees+hands; push-up: feet+hands).
     Knees on stairs get named and counted.
   - **L3 FEET SUPPORT** — the allowed support bodies must be under load continuously
     (airborne fraction, longest unloaded gap).

## How to run it (any deploy or batched eval)

```bash
# instrument the run (env-gated, zero cost when off):
...  CRANE_LOG=1 CONTACT_LOG=1 CONTACT_LOG_ALL=1  ...
# grade it (window = the motion phase in base-x; foot geoms per robot):
python projects/policies/training/verify_motion_legitimacy.py <tag>_mpc.txt 1.0 2.6 6,7,12,13
```

Gotchas measured while building it:
- **Newton renames bodies to `body_N` / `static_body_N`** — classify by GEOM ids, not names.
  The `CONTACTMAP g=<id> body=<name>` lines (auto-dumped) give the map. G1: foot geoms =
  **6,7,12,13** (ankle-pitch + ankle-roll, both sides); knees = `body_9` / `body_15`.
- **Window the motion phase** (by base-x or time). Including the stand segment — or a
  post-fall lying period — floods the contact census with whole-body contacts.
- The **live** `mjw_data.contact` snapshot is PARTIAL (known issue) — stair/terrain contacts
  are trustworthy (cross-checked vs FOOT_LOG), full-census claims should use a batched eval.

## The trainer traps this exists to catch (generalize to EVERY trainer)

1. **⛔ THE CRANE THAT NEVER WEANS.** `HARNESS_GRAD_SURV=2.0` (used by the whole stair
   campaign) means the graduation bar is unreachable: the crane holds λ=0.9 for the entire
   training run, so PPO — an exploit-finding machine — learns to *use* the springs as
   support. The champion is then crane-dependent by construction and the batched evals
   can't tell (they run the same crane). **Rule: every locomotion trainer graduates the
   crane (default `HARNESS_GRAD_SURV=0.9` steps λ down 0.1 per passing eval) unless the
   run is explicitly a puppet-stage experiment — and a champion trained at fixed λ>0 must
   never be presented as a finished skill.** The recipe now logs a loud warning banner when
   the graduation bar is set unreachable.
2. **⛔ KINEMATIC GATES ALONE PASS CHEATERS.** Heading/placement/end-state rulers cannot
   distinguish "climbs" from "is carried while making climbing shapes". Gate on both rulers.
3. **⛔ UNMEASURED SUPPORT PATHS.** Foot-filtered contact logging was blind to knee contacts
   for the entire campaign. When auditing *how* a motion is achieved, log ALL contacts
   (`CONTACT_LOG_ALL=1`) and classify — never assume the only contacts are the intended ones.
4. **Reward-side counterpart (`W_KNEE_LOW`)**: a cheap, CUDA-graph-safe proxy penalty for
   knee-support postures — penalizes a knee dropping within `KNEE_CLEAR` (default 0.15 m) of
   its own-side foot height. Turn it on for any legged motion where knees are not an allowed
   support (it is a *shaping* term; the verifier remains the gate).

## The wider meta-lessons of this campaign (three, all measurement)

1. **Log the SAME scalar on BOTH sides before declaring a train↔deploy divergence.** Two
   phantom "engine bugs" were manufactured from unit mismatches (`ydrift` meters vs yaw
   radians; distance vs heading). `EVALYAW` now logs batched yaw; keep it that way.
2. **Verify the dynamics, not the trajectory** (this doc).
3. **The corridor-torque law binds at every terrain feature.** A 3 cm riser needs
   0.192 rad of knee deviation; a 0.15 corridor makes the straight climb *infeasible by
   construction* — the
   policy will then find whatever cheat the constraints leave open: the yaw-twist, the
   knee-lever, the crane-lean. **A "weird" learned strategy is usually the only strategy
   the constraints allowed. Audit the constraints before blaming the learner.**

Case study data (wr_stairhuman14, the kinematically-passing cheater), reproducers, and the
full campaign ladder: `docs/developer/train-deploy-gap.md` §2b–2c and the demo header of
`projects/policies/demos/run_climb_stairs_stand.sh`.
