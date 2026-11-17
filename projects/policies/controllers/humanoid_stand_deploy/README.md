# humanoid_stand_deploy — generic deterministic humanoid stand

Robot-agnostic **pure-pose** stand controller (no RL). Generalises the proven
G1 deterministic stand to any humanoid via a JSON spec. Full write-up:
[docs/developer/humanoid-deterministic-stand.md](../../../../docs/developer/humanoid-deterministic-stand.md).

## Run

```powershell
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1       -Duration 30
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot valkyrie -Duration 40
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot <robot> -Gui
```

## Status

- **H1** ✅ stands (KE 800, `ank_bias −0.06`)
- **Valkyrie** ✅ stands (KE 2000, `ank_bias −0.02`)

## Add a new humanoid

1. Write `specs/<robot>.json`:
   - `joint_order`: every actuated joint (held at nominal; missing motors warn, don't fail).
   - `nominal`: per-joint radians. Legs: a flat-foot squat `hip_pitch<0`, `knee>0`,
     `ankle_pitch=-(hip_pitch+knee)`.
   - `ankle_pitch_joints` / `hip_pitch_joints`: targets for the `ank_bias`/`hip_bias` trim.
   - `deploy_ke`/`deploy_kd`: Newton position-hold stiffness (heavier ⇒ stiffer).
   - `ank_bias`: backward ankle trim to centre the CoM (start 0, sweep if it tips).
   - `upright`: `{bz_min, roll_max, pitch_max}` fall thresholds.
2. Write `projects/policies/worlds/<robot>_stand_deploy.wbt` (copy an existing one; set
   `url`, `translation z = spawn_z`, `controller "humanoid_stand_deploy"`).
3. Add `<robot>` to the `-Robot` `ValidateSet` in `run_humanoid_stand_deploy.ps1`.

## Table manipulation (G1) — pick/move/place a cube while standing

`-Manip` runs a deterministic manipulation overlay: the G1 holds the stand and, with
one arm, picks a cube off a table in front, lifts/moves/places it, while the other
arm + legs keep balance (fingerless hand → kinematic weld grasp). Composes with
`-Throw`. Full write-up + honest limitations:
[docs/developer/g1-manipulation-while-standing.md](../../../../docs/developer/g1-manipulation-while-standing.md).

```powershell
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Manip -Gui
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Manip -Throw -Duration 60
```

Spec `specs/g1_manip.json` (`manip` block) + sibling `arm_ik.py` (FK/DLS-IK) +
world `research/worlds/g1_hstand_manip.omniworld`.

## Squat (G1) — quasi-static deep knee-bend and return

`-Squat` runs a deterministic **squat** overlay on the held stand: the legs blend
from the standing nominal DEEPER into a knee-bend (smoothstep ramp), HOLD at the
bottom, then RISE back, repeated `reps` times — all while the stand's reactive
balance keeps the CoM centred. It is pure feedforward leg targets (no RL); the
foot stays flat (`ankle delta = -(hip+knee)`) and an open-loop depth-proportional
ankle lean-back (`ff_ank`) pre-centres the CoM through the transient.

```powershell
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Squat -Duration 40
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Squat -Gui
```

**Verified (warm):** G1 squats **~10 cm** (pelvis `bz` 0.763 → 0.662), HOLDs, and
RISEs back, **2 reps, 0 falls, ends upright** — peak pitch only ±0.05 rad (14× under
the 0.7 limit), genuinely quasi-static (peak speed ≤0.04 m/s). The **RISE is the
touchy phase** (standing up is the unstable half of the motion): too little `ff_ank`
topples forward, too much topples backward, so deepen by raising `ff_ank` *and*
slowing `rise_s` together. Spec `specs/g1_squat.json` (`squat` block).

`squat` block fields: `start_s`, `descend_s`, `hold_s`, `rise_s`, `between_s`,
`reps`; `delta` (per-joint bottom-of-squat offsets from nominal); `ff_ank`/`ff_hip`
(open-loop depth-proportional lean-back); `bz_drop` (expected pelvis sink — the
fall check relaxes `bz_min` by `frac*bz_drop` so a deliberate crouch is not flagged
as a fall). Each knob is overridable via `HSTAND_SQUAT_*` env vars.

To add the squat to another humanoid, drop a `squat` block into its `specs/<robot>.json`
(or make `specs/<robot>_squat.json`) — the controller and the `-Squat` flag are
robot-agnostic; each robot needs its own short `delta`/`ff_ank`/`rise_s` tune.

## Tuning knobs (env / launch params)

| Param | Env | Meaning |
|---|---|---|
| `-Ke` / `-Kd` | `OMNISIM_NEWTON_TARGET_KE/KD` | global position-hold stiffness (spec `deploy_ke/kd`) |
| `-AnkBias` | `HSTAND_ANK_BIAS` | backward ankle trim, rad (neg = lean back) — spec `ank_bias` |
| `-HipBias` | `HSTAND_HIP_BIAS` | hip-pitch trim — spec `hip_bias` |
| `-AnkleKp/-AnkleKd` | `HSTAND_ANKLE_KP/KD` | optional analytic ankle balance PD (default 0 = pure pose) |
| `-BalClamp` | `HSTAND_BAL_CLAMP` | clamp on the balance correction |
| — | `HSTAND_DEBUG=1` | verbose startup checkpoints |
