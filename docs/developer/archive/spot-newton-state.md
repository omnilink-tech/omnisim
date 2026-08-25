# Spot under Newton — current state

Snapshot of the Spot + Newton work after the May 2026 sessions. The
journey was: "Spot's articulation crashes the builder" → "Spot stands
intact under Newton with realistic URDF physics, but does not yet walk"
→ **"Spot walks straight under Newton, 4.87m / 30s, STRAIGHT verdict,
trained in 84 seconds via the model+residual recipe."** The
from-scratch attempts (v3, v4) below are preserved as the cautionary
chapter that motivated the recipe change. The canonical Newton walker
now lives in [spot-residual-rl.md](../omniquad-residual-rl.md#newton-port--same-recipe-different-physics-walks-straight-in-84-seconds).

## What's committed and working

All of the following is in the binary today (`OMNISIM_WITH_NEWTON=ON`
build, MuJoCo CPU solver path):

| component | location | state |
|---|---|---|
| Articulation builder (leaf-first joint feed safe) | `OmNewtonBackend.cpp` `kNewtonRuntimeSource` | 13 Spot bodies + 12 hinges finalise cleanly |
| Position-target bridge from `motor->setPosition` | `OmBasicJoint::pushNewtonMotorTargets` | wired through `control.joint_target_pos` |
| Live joint-angle readback | `OmNewtonBackend::getJointAngle` | reads `state.joint_q` |
| Newton overwrite of Solid pose | `OmSolid::postPhysicsStep` | uses `setValueFromOde` + manual `setMatrixNeedUpdate` to break the reset-hook feedback loop |
| Per-shape transforms (Pose `<origin>`) | `OmSolid::flushPendingNewtonRegistrations` | foot sphere registers `at (0,0,-0.32)` matching URDF |
| URDF mesh → AABB box collision | `computeBoundingObjectMeshAabb` | chassis 89×26×21 cm, upper leg with -13.5 cm centre offset |
| URDF effort / velocity / position limits | passed through `add_joint_revolute` kwargs | `effort=80 N·m`, `vel=20 rad/s`, joint stops per URDF |
| URDF inertia tensors (`OMNISIM_URDF_USE_INERTIA=1`) | `OmSolid::flushPendingNewtonRegistrations` | chassis I=(0.20, 1.14, 1.17), legs match URDF |
| Newton-aware Supervisor reset | `OmSolid::syncNewtonPoseFromFields` | warps `body_q` + reset `joint_q` on field writes |
| Parent-child collision filter | `_add_revolute_to_builder` | `collision_filter_parent=True` |
| SpotEnv training under Newton | `SpotEnv` + `spot_rl_newton.omniworld` + `train_spot_newton.py` | 570 steps/s smoke under MuJoCo CPU |
| End-to-end PPO loop | `continue_training_newton.py`, ONNX export | 200k-step run produces converged `spot_newton_v3` checkpoint |

## What did not work — from-scratch PPO under Newton (v3, v4)

The from-scratch attempts here predate the model+residual port and are
kept as the cautionary chapter. Both failed; v3 found B-mode, v4
couldn't even stand. See the postmortem below, then jump to the
[Newton residual walker section in spot-residual-rl.md](../omniquad-residual-rl.md#newton-port--same-recipe-different-physics-walks-straight-in-84-seconds)
for the recipe that actually walked.

**Walking under deploy.** Two trained policies tested:

1. `spot_omnisim_main` (ODE-trained `spot_walk_v12`, 140k steps under ODE):
   - **Under MuJoCo CPU + URDF effort limits:** frozen at spawn z=0.7. The actuator force the policy commands (gentle `ke=20 (target - q)` against ~zero error) is below static friction / gravity threshold. Spot stands but never moves.
   - **Under XPBD GPU:** NaN's within ~120 sim steps. XPBD's constraint solver amplifies small force imbalances under tight effort limits.

2. `spot_newton_v3` (from-scratch 200k under Newton MuJoCo CPU,
   `alive_bonus=1.0, lin_vel_wt=1.0, vx=0.1`):
   - Monitor.csv shows clean convergence: last 10 episodes all 1023 steps, reward ~2400.
   - But deploy ALSO NaN's around step 60 under MuJoCo CPU. Training never evaluated mid-run, so PPO settled on a policy that produces a "stand still and ride the alive bonus" *training* trajectory but commands action sequences that destabilise Spot when CPG-summed in the deploy controller.

**The diagnosis worth flagging for the next round.** Training reward
~2400/episode = `1023 alive_bonus_steps × 1.0 + ~1400 from
velocity/term/stability terms`. A policy that earns this can do so
two ways:

- A. Stand upright AND move slowly forward (the walker we want).
- B. Output extreme-but-balanced commands that hold the joints at
  their position limits — the constraint forces cancel and Spot
  doesn't fall, while the velocity reward picks up small drift.

v3 converged on B. We can tell because deploy crashes immediately on
the same commands the training reported as +2.3/step reward. A B-mode
policy doesn't generalise — the moment any small disturbance differs
between training and deploy, the joint-limit ride pattern breaks and
the body kicks itself over.

## How to verify (the test that's missing)

The training pipeline does not currently evaluate intermediate
checkpoints under the deploy world. Add an SB3 callback that, every
N checkpoints:

1. Loads the latest `spot_<N>_steps.zip`.
2. Runs `projects/policies/research/tools/_eval_newton.py` against `spot_newton_demo.omniworld`.
3. Logs forward distance + time-to-fall to monitor.csv as a side
   channel.

Without this, the trainer cannot tell A-mode from B-mode.

## Recommended next session

1. **Add the deploy-eval callback.** Picks up B-mode collapse before
   wasting hours on a useless training run.
2. **Reward-shape against joint-limit commands.** Penalise actions
   whose absolute value is above ~0.8 of the URDF effort limit. PPO
   then can't ride the limits.
3. **From-scratch training under MuJoCo CPU with the callback active**, 
   500k steps. With B-mode blocked, the only path to high reward is
   actual locomotion.
4. **Stop touching the XPBD-GPU path** until the policy is sound.
   The constraint-solver-amplifies-instability failure mode is
   independent of the policy quality and burns time chasing red
   herrings.
5. **Reuse the curriculum**: `SPOT_R_ALIVE_BONUS=1.0`,
   `SPOT_R_LIN_VEL_WT=1.0`, `SPOT_FIXED_VX=0.1`. The from-scratch
   100k variant with `ALIVE_BONUS=0` collapsed to "freeze" within
   100 episodes — the alive bonus is load-bearing.

## Update: anti-B-mode attempt (spot_newton_v4) — made it worse

Implemented the documented plan (joint-limit penalty + forward-distance
logging + vx bonus) and ran 500k from-scratch. Two hard findings:

1. **Webots+Newton has a ~200k-step crash ceiling.** Every long run
   (v2, v4) dies with `ConnectionResetError` around 200k steps / ~22
   min — the embedded-CPython/Newton process accumulates state across
   thousands of supervisor resets and eventually the controller
   subprocess dies mid-`env.reset()`. The Newton-aware reset
   (936cf85a) raised throughput but didn't fix the leak. **Any
   training campaign must checkpoint frequently and resume across
   process restarts**, or the leak must be found first (suspect: Warp
   array / contact-buffer allocation in the helper module's per-step
   path, or model/state objects not being released on reset).

2. **The reward shaping backfired.** v4 config
   (`SPOT_R_JOINT_LIMIT_WT=-0.3`, `SPOT_R_VX_BONUS_WT=1.5`) produced a
   policy that *never learned to stand*: ep_len_mean stayed flat at
   ~45 steps the entire run (v3, without these terms, grew ep_len to
   1023). Deploy eval of the v4 200k checkpoint: falls at 0.69 s, 0.4%
   upright, min_bz 0.10. The `vx_bonus=1.5` is the prime suspect — it
   rewards forward lunging before the policy can balance, so PPO
   chases velocity into the ground. The joint-limit penalty also
   removed v3's stable (if degenerate) standing point without leaving
   a reachable upright gradient.

   **Lesson: shape in stages, not all at once.** The right sequence is
   probably: (a) train to *stand* first (alive bonus only, no velocity
   incentive, no joint penalty) until ep_len saturates at 1023; (b)
   THEN add a small vx bonus to coax forward motion; (c) only add the
   joint-limit penalty if B-mode actually reappears, and at a much
   smaller weight (-0.05, not -0.3).

### Scoreboard

| policy | training | deploy (Newton MuJoCo CPU) |
|---|---|---|
| spot_omnisim_main (ODE v12) | n/a (ODE-trained) | frozen at spawn, 0 motion |
| spot_newton_v3 (alive=1, no penalty) | ep_len -> 1023, rew ~2400 | frozen / NaN under XPBD |
| spot_newton_v4 (+joint pen +vx bonus) | ep_len flat ~45, rew ~-42 | falls at 0.69 s, 0.4% upright |
| **spot_residual_newton_v1 (model+residual, 50k)** | 84s wall, ep_len ~60 | **+4.87m / +0.13m lat / +3.2° yaw / 100% upright, STRAIGHT** |

The model+residual recipe in `spot_residual_newton_v1` succeeded where
from-scratch PPO failed for the same reason it succeeded under ODE: the
analytical model layer (gait + IK + balance) already produces a
competent walker before training begins, and under Newton's stiffer
physics it produces one that's better than the ODE baseline. PPO is
then refining, not discovering, so the sticky local optima (stand
still, B-mode joint-limit riding) are unreachable. See
[spot-residual-rl.md](../omniquad-residual-rl.md#newton-port--same-recipe-different-physics-walks-straight-in-84-seconds)
for the full recipe and reproduction commands.

## Commands

```powershell
# Smoke (verify env stable before training)
python projects/policies/research/tools/_smoke_newton_env.py

# From-scratch training (200k, ~14 min wall under MuJoCo CPU)
python projects/policies/research/training/train_spot_newton.py `
  --envs 1 --single --steps 200000 `
  --run-name spot_newton_<n> --ckpt-every 25000

# Eval a trained policy
python projects/policies/research/tools/_eval_newton.py `
  --policy projects/policies/research/inference/policies/spot_newton_<n>/policy.onnx `
  --duration 30 --vx 0.1

# Visual (GUI) playback
python projects/policies/research/tools/_show_newton_policy.py `
  --policy projects/policies/research/inference/policies/spot_newton_<n>/policy.onnx
```

## Session commit chain (May 2026)

`587fb334` multi-parent articulation fix → `0bfd83c3` position bridge
+ scene-tree pose sync → `e674da71` mesh-AABB collision shapes →
`1da574da` per-shape Pose translation → `f3422eaf` actuator joint-
target wiring (`joint_target_pos` not `joint_target`) → `58073fcc`
URDF effort/velocity/inertia plumbing → `936cf85a` Newton-aware
Supervisor reset → `94d2d7f8` SpotEnv + training launcher →
`5d5a52d3` warm-start launcher → `db285403` XPBD-GPU as default
(later regressed back) → `52e49547` freeze-loop fix (setValueFromOde)
→ `a6578d01` from-scratch curriculum + MuJoCo default →
`db2f68ab` matrix-cache invalidation after setValueFromOde →
`859770d8` show-script MuJoCo default.

`spot_newton_v3` checkpoints live at
`projects/policies/research/training/runs/spot_newton_v3/checkpoints/` and the ONNX
at `projects/policies/research/inference/policies/spot_newton_v3/`. **Don't delete
them** — useful as a "this is what B-mode looks like" reference for
the next training round.
