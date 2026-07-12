# Skill: arm_motion — balance on two legs while moving the arms in 3-D

**Class:** humanoid · **Robot:** G1 · **Kind:** deterministic · **Status:** ✅ verified (2026-06-28)

The G1 holds the deterministic two-leg stand and runs a looping routine of arm
**exercises**, so the arms move freely in 3-D while the legs keep balancing — and it
keeps standing through external cube hits (`-Throw`). This is the "aware of its hands,
moving them around doing exercises, while balancing" skill.

## Run

```powershell
# via the skills launcher
python projects/policies/skills/run_skill.py arm_motion --duration 60
python projects/policies/skills/run_skill.py arm_motion --throw --gui

# or directly
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -ArmMotion -Duration 60
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -ArmMotion -Throw -Gui
```

## The routine (6 exercises, looping, with a rest dwell between each)

| exercise | what it does | why it's balance-safe |
|---|---|---|
| `lateral_raise` | arms out to ~50° and back | symmetric lateral → ~zero fore/aft CoM |
| `arm_circles` | hands circle in a mostly-lateral plane | roll-dominant, tiny pitch |
| `elbow_curls` | forearms curl in/out | distal → low CoM impact |
| `arm_swing` | arms swing alternately fore/aft (a march) | 180° out of phase → net fore/aft CoM cancels |
| `wave` | right arm up-and-out, hand waves | one arm up-and-out (lateral); left arm balances |
| `flex` | arms out + forearms curl up (double-biceps) | lateral + distal, two safe primitives |

## How it balances (and the one hard rule)

G1's arms-down stand is **marginal** and uses the arms as its *fast balancer*. So this
skill does **not** take the arms away from the balancer (that topples it within ~3 s —
verified). Instead the exercise sinusoid is **superimposed** on the live `arm_balance`,
amplitudes are kept moderate so the balancer keeps its authority, a forward-kinematics
**CoM-back feedforward** pre-compensates the arms' forward shift, and a **rest dwell**
between exercises lets the stand re-centre.

**The hard rule, learned by toppling repeatedly:** *bilateral forward/up arm raises
shift the CoM forward faster than the marginal stand can recover.* So forward motion is
done as an **alternating march** (`arm_swing`, fore/aft cancels) and reaches are angled
**out** (lateral), never straight forward. Lateral and distal motions are always safe.
Full journal: [docs/developer/g1-arm-motion-skill.md](../../../../../docs/developer/g1-arm-motion-skill.md).

## Verified

- **arm_motion alone:** 0 falls over **206 s** sim (2+ full loops of all 6 exercises),
  ends upright, pitch within ±0.06 rad throughout.
- **arm_motion + `-Throw`:** 0 falls; 5/8 cube throws scored solid torso HITs (≤0.18 m
  closest approach) and were absorbed while exercising.

## Files

- spec: [`specs/g1_arm_motion.json`](../../../controllers/humanoid_stand_deploy/specs/g1_arm_motion.json) (`arm_motion` block + exercise library + FK chains)
- overlay: the `arm_motion` block in [`humanoid_stand_deploy.py`](../../../controllers/humanoid_stand_deploy/humanoid_stand_deploy.py)
- world: [`research/worlds/g1_arm_motion.wbt`](../../../research/worlds/g1_arm_motion.wbt)
- launch flag: `-ArmMotion` in [`run_humanoid_stand_deploy.ps1`](../../../../../scripts/dev/run_humanoid_stand_deploy.ps1)

## Tuning knobs (spec `arm_motion` block / env)

| key | env | meaning |
|---|---|---|
| `start_s` | `HSTAND_ARM_MOTION_START` | when the routine begins (after settle) |
| `rest_s` | `HSTAND_ARM_MOTION_REST` | rest dwell between exercises (re-centre) |
| `ff.c_hip/c_ank` | `HSTAND_ARM_MOTION_C_HIP/_C_ANK` | CoM-back feedforward gains |
| exercise `terms[]` | — | per-joint `bias + amp*sin(2π·freq·t + phase)` |
