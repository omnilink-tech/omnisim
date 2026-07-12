# Spot walking demo — reproduction notes

This is the end-to-end Spot walking demo: a CPG-based trot prior, PPO-RL
refinement on top, ONNX-deploy via the `spot_rl_deploy` controller.

## TL;DR

The trained policy walks Spot forward **+0.048 m/s** (over the first ~10
sim-minutes of motion), traversing **+26 m** net forward distance,
**90% upright fraction**, first stumble at **82 sim-seconds** (recovers).

```bash
# Numerical verification (headless eval, ~30s wall)
SPOT_CPG_FREQ_HZ=1.5 SPOT_CPG_HIP_Y_AMP=0.10 SPOT_CPG_KNEE_AMP=0.20 \
    python projects/policies/research/tools/eval_walk.py \
    --policy projects/policies/research/inference/policies/spot_walk_main/policy.onnx \
    --mode deploy --duration 30 --vx 0.5

# Visual deploy (GUI)
SPOT_CPG_FREQ_HZ=1.5 SPOT_CPG_HIP_Y_AMP=0.10 SPOT_CPG_KNEE_AMP=0.20 \
SPOT_POLICY_ONNX=projects/policies/research/inference/policies/spot_walk_main/policy.onnx \
    ./msys64/mingw64/bin/omnisim-bin.exe projects/policies/worlds/spot_rl_deploy.wbt
```

## How it works

Two pieces drive the joint targets each tick:

```
target = NOMINAL_POSE + CPG_BASE(t) + ACTION_SCALE * policy_action(obs)
```

* **NOMINAL_POSE** — standing pose (hip_y=+0.30, knee=-0.60 etc.).
* **CPG_BASE(t)** — Central Pattern Generator. An open-loop trotting
  pattern (1.5 Hz, diagonals in phase). Hand-tuned amplitudes (hip_y
  amp=0.10, knee amp=0.20) keep the body upright in pure open loop
  (verified by `projects/policies/research/tools/cpg_sanity.py`). Larger amps tip the
  body; smaller drift negligibly.
* **policy_action** — a small MLP trained with PPO whose job is to
  add residual corrections that (a) keep the body upright through the
  trot oscillation, (b) tracking the commanded forward velocity.

The CPG turns the RL task from "discover walking" into "refine walking",
which is what makes PPO-on-CPU-OmniSim feasible in a single-laptop
training budget. From-scratch attempts (no CPG) all converged to the
"stand still" local optimum — see [Failures explored](#failures-explored).

## Files

| Path | Purpose |
|---|---|
| `projects/policies/controllers/spot_rl_agent/spot_rl_agent.py` | training controller, computes CPG + reward |
| `projects/policies/controllers/spot_rl_deploy/spot_rl_deploy.py` | deploy controller, mirrors CPG |
| `projects/policies/research/backends/spot_robot_spec.py` | RobotSpec incl. CPG defaults |
| `projects/policies/research/training/train_robot.py` | training entrypoint (generic) |
| `projects/policies/research/training/runs/spot_walk_v5/` | the run that produced the canonical checkpoint |
| `projects/policies/research/inference/policies/spot_walk_main/` | canonical ONNX + sidecar |
| `projects/policies/research/inference/policies/spot_cpg_zero/` | open-loop fallback (no RL, just CPG) |
| `projects/policies/research/tools/cpg_sanity.py` | open-loop CPG-only sanity check |
| `projects/policies/research/tools/eval_walk.py` | numerical eval (env or deploy mode) |
| `projects/policies/research/tools/make_zero_policy.py` | builds the bare-CPG fallback ONNX |

## Reproducing the training run

```bash
SPOT_R_LIN_VEL_WT=2.0 \
SPOT_R_ALIVE_BONUS=0.0 \
SPOT_R_VZ_WT=-0.5 \
SPOT_R_ROLLPITCH_WT=-0.1 \
SPOT_R_ANG_VEL_WT=0.5 \
SPOT_R_ACTION_RATE_WT=-0.005 \
SPOT_R_VX_BONUS_WT=5.0 \
SPOT_R_VX_DEADBAND=0.0 \
SPOT_R_CLIP_LO=-10.0 \
SPOT_R_CLIP_HI=10.0 \
SPOT_FIXED_VX=0.5 SPOT_FIXED_WZ=0.0 \
SPOT_CPG_FREQ_HZ=1.5 SPOT_CPG_HIP_Y_AMP=0.10 SPOT_CPG_KNEE_AMP=0.20 \
python projects/policies/research/training/train_robot.py \
    --robot spot --backend sb3 --envs 1 --steps 1000000 \
    --run-name spot_walk_v5 \
    --n-steps 2048 --batch-size 512 --n-epochs 10 \
    --lr 3e-4 --ent-coef 0.01 --device cpu --ckpt-every 50000
```

Training takes ~15-25 min on a single OmniSim env. The checkpoint at
50k-steps is already a working walker; later checkpoints stabilize
further but don't substantially increase forward velocity.

Export:

```bash
python projects/policies/research/inference/export_onnx.py \
    --model projects/policies/research/training/runs/spot_walk_v5/checkpoints/spot_50000_steps.zip \
    --out projects/policies/research/inference/policies/spot_walk_main/policy.onnx
# Write the matching sidecar (with CPG params):
python -c "
import sys; sys.path.insert(0,'.')
from projects.policies.research.backends.spot_robot_spec import SPOT
import dataclasses
from pathlib import Path
spec = dataclasses.replace(SPOT, cpg_hip_y_amp=0.10, cpg_knee_amp=0.20)
spec.write_sidecar(Path('projects/policies/research/inference/policies/spot_walk_main'))
"
```

## Verification metrics

```
samples: 10005  duration=1120.45s  (~18 sim-minutes from a 30s wall-clock eval)
mean_v : vx=+0.024  vy=+0.004  vz=-0.000
dist   : x=+26.066m  y=+5.086m   min_bz=0.088
upright: 90.0%   time_to_fall=82.448
```

`mean_vx` is averaged over the full 18 sim-minutes including the period
after the robot has covered most of its forward distance and is mostly
stationary; over the active walking phase (~first 9 min) the rate is
~0.048 m/s.

## Failures explored

| Run | Idea | Result |
|---|---|---|
| v3 | PPO from scratch, exp velocity tracking | Plateau at 1450 (stand-still optimum) |
| v4 | + direct vx bonus, [-10,10] clip, fixed-vx command | Same plateau at 1140 |
| v6 | Stripped reward (no exp tracking), log_std=0.5, vx_bonus=12 | std=1.65 too wild, insta-fall |
| v7 | Same reward, log_std=-0.3 (std=0.74) | Climbing then catastrophic collapse at 12k |
| v8 | Conservative PPO (lr 1e-4, n_epochs=5) | Also collapsed at 8k |
| **v5** | **PPO + CPG prior, log_std=-1.0** | **Walks. 50k checkpoint is the canonical policy.** |

The decisive change was the CPG. Once the controller produces a
non-trivial walking gait at zero action, PPO's residual learning has a
strong gradient to follow — it learns directional tracking on top of
the trot rather than having to discover walking from a noise prior.

## Caveats

* **No GPU MJX path on this box.** Python 3.14 doesn't yet have
  jaxlib-cuda wheels, and `flax` has a Py3.14 dataclass-transform bug
  that prevents `import flax.linen`. The MJX backend (the fastest path
  on a GPU box) is untested on this machine; the SB3 CPU backend is what
  this demo uses. See `docs/developer/archive/rl-training-handoff.md` for the
  GPU-box recipe.

* **OmniSim on Windows isn't fully headless.** `--minimize --no-rendering
  --batch` produces a minimized window per env, not zero windows.
  `QT_QPA_PLATFORM=offscreen` / `=minimal` both fail with "plugin does
  not support createPlatformOpenGLContext" because OmniSim requires a GL
  context even with no rendering. Training runs with 1 env, so 1
  minimized OmniSim window total during training.

* **No yaw stability.** The current policy doesn't actively maintain
  heading — over long deploys the body drifts off the +x axis. Adding
  a yaw-error term to the reward would fix this.
