# G1 — deterministic table manipulation WHILE standing

**Status (2026-06-27): working, single-cube, deterministic (no RL).** The G1 holds
its deterministic squat stand and, with one arm, **reaches to a table in front,
picks a cube off it, lifts it, moves it, and sets it back down** — while the lower
body and the *other* arm keep balance. This pushes the cube-defense stand
(survive-thrown-cubes) one step further: the robot is no longer just *resisting*
disturbances, it is *doing a task* with its arms without falling.

This is an extension of the generic deterministic stand
([humanoid_stand_deploy](../../projects/policies/controllers/humanoid_stand_deploy/),
[humanoid-deterministic-stand.md](humanoid-deterministic-stand.md)), consistent
with the deterministic-control-first direction
([rl-current-state.md](rl-current-state.md)). No policy is trained or run.

## Run it

```powershell
# Stand + pick/move/place a cube on a table (headless)
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Manip -Duration 40
# Same, windowed
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Manip -Gui
# Manipulate AND get hit by thrown cubes at the same time (composes)
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Manip -Throw -Duration 60
# Fast iteration (cold, no warm reload — grasp is less precise)
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Manip -Cold -Duration 40
```

World: [`projects/policies/research/worlds/g1_hstand_manip.omniworld`](../../projects/policies/research/worlds/g1_hstand_manip.omniworld).
Spec: [`specs/g1_manip.json`](../../projects/policies/controllers/humanoid_stand_deploy/specs/g1_manip.json).

## How it works

The whole thing is a **deterministic overlay** inside the existing stand controller
([humanoid_stand_deploy.py](../../projects/policies/controllers/humanoid_stand_deploy/humanoid_stand_deploy.py)).
Per tick, after the lower-body balancers run and before the motors are commanded, a
small state machine drives ONE arm through a pick-and-place trajectory:

```
INIT → (READY) → ABOVE → DESCEND → GRASP → LIFT → MOVE → PLACE → RELEASE → RETRACT → … → DONE
```

The five design ideas that make it stand:

1. **Reach with one arm, balance with the other.** In the bare stand the arms are
   the *fast balancer* (`arm_balance`, the proven kp=6 shoulder swing — removing it
   drops the marginal arms-down stand). So we free only the **right** arm for the
   task (full IK override of its 5 joints) and keep `arm_balance` running on the
   **left** arm. The legs/ankles/hips are untouched. When the task finishes (`DONE`)
   the right arm returns to nominal and *both* arms balance again, so the stand stays
   durable long after the cube is placed.

2. **World-space trajectory + per-tick IK against the live base.** Each waypoint is a
   world TCP target; the arm joints are solved every few ticks by the proven
   damped-least-squares IK ([arm_ik.py](../../projects/policies/controllers/humanoid_stand_deploy/arm_ik.py),
   copied from `omnilink_arm_bridge`) using the **live** pelvis pose. So the hand
   tracks the world cube even as the body leans to balance — reach and balance are
   decoupled.

3. **Feed-forward CoM-back compensation.** An extended arm pulls the CoM toward the
   toes (the same disturbance the reactive lean handles for a cube-from-behind).
   While reaching we pre-lean the body back **open-loop**, proportional to how far
   forward the hand is, on three levers (strongest first): **ankle** (whole-body
   lever), **hip** (torso), and a **counterweight swing of the free arm** (the human
   reach posture). Open-loop, so no destabilising feedback; the slow `auto_trim`
   integral mops up the rest.

4. **Posture-biased (null-space) IK — natural, body-clear arm.** The 5-DOF arm reaching a
   3-DOF position has 2 redundant DOF; a plain position-only IK resolves them arbitrarily and
   routes the elbow/wrist *through the torso* (and contorts the wrist). The IK
   ([arm_ik.py](../../projects/policies/controllers/humanoid_stand_deploy/arm_ik.py) `dls_ik`,
   `q_rest`/`posture_gain`) adds a secondary task in the null space of the reach that pulls the
   redundant DOF toward a natural posture (elbow abducted **out**, wrist + shoulder-yaw ≈0). Verified
   numerically: at every waypoint the elbow and wrist stay **outboard of the torso** (y ≤ −0.17 m) so
   the arm never enters the body, with the reach target still hit to ≤~10 mm. The arms also **rest in a
   natural hands-down pose** (not the old `shoulder_pitch=1.5708`, which FK shows points the arms
   *backward-and-up*).

5. **Kinematic weld grasp + pinned set-down.** The G1 hand is a *fixed rubber hand with no fingers*
   (`left/right_wrist_roll_rubber_hand`, 5-DOF arm). So "grasp" is a **kinematic
   weld** (magnet-style attach), not a physical fingered grasp — the proven pattern
   for a fingerless effector. On approach, when the real hand link is within
   `grasp_radius` of the cube it attaches; each tick the cube rides the live hand
   pose with `setVelocity([0]*6)` (the warm-solver bounce fix). On release the cube is
   **pinned** at the place target (zero velocity) until the hand retracts clear
   (`pin_release_dist`), then it rests — otherwise the still-overlapping hand's contact
   flings it off the table. Verified: a placed cube stays put on the table for the rest
   of the run.

6. **Approach geometry that doesn't knock the cube.** Cubes sit in a ~11×10 cm
   "sweet spot" reach window; the arm always moves between cubes via a **high
   ready/home pose above the table** and descends vertically, so it never sweeps
   *through* a cube. The grasp welds on the way down (before the hand presses the
   cube).

## Honest limitations

This is genuinely at the edge of what a marginal deterministic humanoid stand + a
5-DOF fingerless arm can do. Specifically:

- **Weld, not a real grasp.** No fingers exist on the G1 hand, so the pick is a
  kinematic attach + a clean kinematic set-down. It is contact-honest for a
  magnet-like rigid hand; it is **not** a force-closure grasp and would not do
  orientation-critical insertion.
- **One cube in the sweet spot at a time.** The reachable-and-stable table region is
  only ~11×10 cm. Two independently-graspable 7 cm cubes need ~14 cm separation to
  avoid the ~10 cm hand knocking the neighbour, which doesn't fit — so the demo
  reliably manipulates **one** cube. The controller/spec are N-cube-general (a
  `cubes` list), so more cubes work wherever the envelope allows. A static second
  cube sits on the table as a backdrop, out of the reach path.
- **Marginal stand.** The bare G1 squat stand is itself marginal (see
  [rl-current-state.md](rl-current-state.md)); adding a one-arm reach consumes most
  of the remaining margin. Reaching *across the body* or *far outboard* topples it;
  the cubes are kept near the reaching shoulder's sagittal line. The stand holds
  cleanly through the manipulation and well beyond.
- **Warm reload for grasp precision.** `-Manip` sets `HSTAND_WARMUP_RELOAD=1`: a
  cold first load under-tracks the arm by ~1 cm and misses the cube. `-Cold` runs
  faster but the grasp is less reliable (the weld radius absorbs some of it).
- **Real-time.** The G1 (30-body articulation) under Newton/`mujoco_warp` runs
  well below real-time headless on this box (and slower still when another GPU job
  is running), so a full pick-place takes a while in wall-clock. The physics is the
  certified Newton path (fingerprint `verdict: OK`).

## Tuning knobs (spec `manip` block / env)

| Field | Env | Meaning |
|---|---|---|
| `reach_arm` | `HSTAND_MANIP_REACH_ARM` | which arm does the task (`right`); the other balances |
| `grasp_radius` | `HSTAND_MANIP_GRASP_RADIUS` | weld attach distance (hand link → cube), m |
| `start_s` | `HSTAND_MANIP_START_S` | settle time before the task begins |
| `c_reach`/`c_ank`/`c_cw` | `HSTAND_MANIP_C_REACH`/`_C_ANK`/`_C_CW` | feed-forward gains: hip / ankle / counterweight-arm lean-back per m of forward reach |
| `ff_clamp`/`ank_clamp`/`cw_clamp` | `HSTAND_MANIP_FF_CLAMP`/… | clamps on those |
| `offsets.{z_above,grasp_dz,lift_dz,place_dz}` | — | approach/grasp/lift/set-down heights |
| `timing.{move_s,dwell_s,grasp_dwell_s}` | — | trajectory segment durations |
| `chain_{left,right}` / `tcp_{left,right}` / `limits_{left,right}` | — | per-arm URDF IK chain (extracted from `g1_23dof_omnisim.urdf`) |
| `cubes[]` | — | list of `{def, place}` to manipulate in order |

## Verification

Staged, parsed from the deploy log (`_scratch/stand/g1_hstand.log`):

- `settle done` — stand established.
- `MANIP_GRASP attached:GRASP_CUBE0 dist=…` — the weld caught the cube.
- per-second `held=Y` + the cube's `z` rising — a *real* lift (cube follows the hand).
- `MANIP_PLACE … OK` — cube set down within tolerance of the target.
- `MANIP_SUCCESS: N/N cubes placed` and **0 `FALL@` lines** — task done, never fell.
