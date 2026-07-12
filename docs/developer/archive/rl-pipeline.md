# OmniSim RL pipeline — Spot quadruped

End-to-end reinforcement-learning pipeline for training a Spot policy inside
OmniSim. Spot is the first robot wired up; the layout is reusable for other
URDF-imported robots.

## At a glance

```
projects/policies/
├── envs/
│   ├── spot_env.py            # gymnasium.Env wrapping one OmniSim subprocess
│   └── validate_env.py        # random-action rollout smoke test
├── controllers/
│   ├── spot_rl_agent/         # OmniSim controller for training (TCP server)
│   │   └── spot_rl_agent.py
│   └── spot_rl_deploy/        # OmniSim controller for inference (loads ONNX)
│       └── spot_rl_deploy.py
├── worlds/
│   ├── spot_rl.wbt            # training world (per-env via SPOT_RL_PORT env)
│   └── spot_rl_deploy.wbt     # deploy world — loads ONNX policy
├── training/
│   ├── train_spot.py          # SB3 PPO + SubprocVecEnv
│   └── runs/                  # one subdir per training run
├── inference/
│   ├── export_onnx.py         # SB3 zip -> ONNX
│   └── policies/<run>/policy.onnx
└── tools/
    └── patch_spot_urdf_for_rl.py    # foot sphere + joint damping URDF edits
```

## Quick start

```bash
# 1) Install deps (one-time)
python -m pip install "stable-baselines3[extra]" gymnasium onnx onnxruntime tensorboard

# 2) Sanity-check the env wiring
python projects/policies/research/envs/validate_env.py

# 3) Train (default: 4 envs, 1M timesteps, ~60 min wall on a modest box)
python projects/policies/research/training/train_spot.py --envs 4 --steps 1000000 \
       --run-name spot_ppo_main

# 4) Export the trained model to ONNX
python projects/policies/research/inference/export_onnx.py \
       --model projects/policies/research/training/runs/spot_ppo_main/spot_final.zip \
       --out   projects/policies/research/inference/policies/spot_ppo_main/policy.onnx

# 5) Run the deploy world (GUI)
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/worlds/spot_rl_deploy.wbt
```

## How it works

### Per-env subprocess architecture

Each parallel env is its own headless OmniSim subprocess. SpotEnv writes a
per-env copy of the world (`.spot_rl_envN.wbt`) with a unique TCP port baked
into `controllerArgs`, then launches `omnisim-bin.exe`. The OmniSim process
spawns the `spot_rl_agent` Python controller, which binds a TCP server on
that port. The SpotEnv (running in the trainer process) connects.

The wire protocol is fixed-size numpy float32 packets:

```
obs_packet  (controller -> trainer, each tick):
    float32[OBS_DIM=50]   observation
    float32               reward
    uint8                 done flag

cmd_packet  (trainer -> controller, each tick):
    uint8                 tag: 0=ACTION 1=RESET 2=QUIT
    float32[ACT_DIM=12]   residual joint deltas (ignored for RESET/QUIT)
```

### Observation (50-D)

| Slice | Field | Notes |
|---|---|---|
| 0:3 | base linear velocity | world frame (m/s) |
| 3:6 | base angular velocity | world frame (rad/s) |
| 6:9 | projected gravity | body frame; tells policy which way is up |
| 9:21 | 12 joint positions | order FL/FR/RL/RR × hip_x/hip_y/knee |
| 21:33 | 12 joint velocities | finite difference |
| 33:45 | last action | the previous tick's residual command |
| 45:48 | velocity command | (vx, vy, wz) the user / curriculum wants |
| 48 | gait clock | (sin/cos pair could be added here later) |
| 49 | heading deviation | wrapped yaw error from spawn yaw (rad); lets the policy see drift |

`OBS_DIM` defaults to `50` (set via the `SPOT_OBS_DIM` env var). `obs[49]`
was added later so the policy can SEE its yaw drift; the legacy 49-dim layout
(no heading term) is still selectable with `SPOT_OBS_DIM=49`.

All observations are clipped to `[-10, 10]` and NaN-sanitized inside the
controller. Without this, a single physics divergence crashes PPO when it
builds the Normal action distribution.

### Action (12-D, residual policy)

Each output is a delta in `[-1, 1]` that's scaled by `ACTION_SCALE = 0.15 rad`
and added to a nominal standing pose (hip_x=±0.30, hip_y=+0.30, knee=-0.60).
Final motor targets are clamped to the URDF joint limits. The training-time
`ACTION_SCALE` MUST match the value baked into the deploy controller
(both files declare it at the top — keep them in sync).

Residual on top of nominal is the easiest formulation for legged RL —
the policy has to learn corrections, not the full pose. The wave-gait
controller in `spot_simple_pose.py` could be used as a stronger nominal
to bootstrap learning further.

### Reward (legged_gym style)

```
+1.0 · exp(-‖v_xy − v_xy_target‖² / 0.25)     forward velocity tracking
+0.5 · exp(-(wz − wz_target)² / 0.25)         yaw rate tracking
−1.0 · vz²                                    don't bob
−0.1 · (roll² + pitch²)                       stay level
−0.02 · (wx² + wy²)                           don't roll/pitch
−0.005 · ‖action − prev_action‖²              action smoothness
−2.5e-7 · ‖joint_acc‖²                        joint smoothness
+0.05                                         alive bonus
```

Termination penalty `−1.0` on `bz < 0.30` or `|roll|, |pitch| > 1.0 rad`.

Reward clamped to `[−5, 5]` and NaN-checked. Max episode length 1024 ticks.

### URDF fixes for RL stability

The URDF was modified in three ways to make physics tractable for RL:
* **Foot ball collider** — each lower_leg's collision is a 3.5 cm sphere at
  the foot tip instead of the original long-bar mesh. The bar mesh caused
  side-of-leg friction artifacts that destabilized any open-loop gait.
* **Realistic motor limits** — `effort=80 Nm` (was 1000), `velocity=20 rad/s`
  (was 1000). The policy must learn around real actuator limits.
* **Joint damping** — `<dynamics damping="0.05"/>` on all 12 leg joints.
  Without damping RL discovers self-oscillation exploits.

URDF joint range tightening (`hip_x ±[0.001, 0.60]`, `hip_y [0.001, 0.60]`,
`knee [-1.20, -0.01]`) also ensures the URDF importer seeds the joints at
the standing pose midpoint, so the robot spawns standing instead of
in a crumpled knee-bent default.

### Training hyperparameters

PPO defaults from SB3 except:
* network: `[256, 256]` MLP, tanh
* `n_steps=512` per env, `batch_size=1024`, `n_epochs=4`
* `gamma=0.99`, `gae_lambda=0.95`, `clip_range=0.2`
* `ent_coef=0.005`, `vf_coef=0.5`, `max_grad_norm=1.0`
* `lr=3e-4`

### Speed

On a single laptop (RTX 3060):
* 1 env: ~155 steps/s
* 4 envs: ~300 steps/s (SubprocVecEnv overhead is real on Windows)
* 1 M timesteps ≈ 55 min wall

For production-quality walking, plan for 10-50 M timesteps. Either run the
trainer overnight or migrate to Isaac Gym / Brax for GPU-parallel rollouts
(those have 1000+ envs and 100-1000× wall speedup).

## Deploying a trained policy

The `spot_rl_deploy` controller loads an ONNX file and runs it as the Spot
URDFRobot's controller. Two ways to point it at a specific policy:

```bash
# A) env var override
SPOT_POLICY_ONNX=path/to/policy.onnx omnisim-bin projects/policies/worlds/spot_rl_deploy.wbt

# B) drop the file at the default location
mkdir -p projects/policies/research/inference/policies/spot_ppo_main
cp my_policy.onnx projects/policies/research/inference/policies/spot_ppo_main/policy.onnx
omnisim-bin projects/policies/worlds/spot_rl_deploy.wbt
```

Walk command (`vx`, `vy`, `wz`) is controlled by env vars `SPOT_VX`, `SPOT_VY`,
`SPOT_WZ`. The deploy controller does no episode reset; if the policy crashes
the body, you'll see it.

## End-to-end evaluation

```bash
python projects/policies/research/inference/eval_policy.py \
    --policy projects/policies/research/inference/policies/main/policy.onnx \
    --duration 20.0 --vx 0.4
```

Spawns the deploy world headless, runs the policy for 20 s of sim,
reports forward dx, lateral dy, mean vx, upright fraction, and fall
detection. Trace CSV is at `C:\tmp\husky_trace\spot_deploy.csv`.

## Plateau at "standing only" — what to do next

The vanilla 500 k-step run typically plateaus around `ep_rew_mean ≈
650-700`. At this stage the policy reliably keeps Spot upright (full
1024-step episodes, no falls) but doesn't actively track the velocity
command — the reward gets enough from the alive bonus + orientation
penalty + partial tracking that it doesn't NEED to learn to walk yet.

Improvements that unlock actual locomotion:

1. **Curriculum on velocity command.** Start with `vx ∈ [0, 0.05]`, ramp
   to `[0.2, 0.8]` over 1-2 M steps. Forces the policy to learn standing
   first, then walking.
2. **Stronger velocity tracking weight.** Bump `R_LIN_VEL_WT` from 1.0
   to 2.0+, lower the alive bonus to 0.01.
3. **Foot air time reward.** Reward the policy for cycling feet (each
   foot's swing phase ≥ 200 ms is rewarded, otherwise penalised). The
   reference legged_gym recipe includes this; we omitted it for v1.
4. **Random pushes** during episodes (sim-to-sim robustness).
5. **Friction & mass domain randomization.**
6. **Terrain randomization** — heightfields, slopes.
7. **Longer training** — 5-10 M+ steps with the above.

## Wall-clock budgets we've observed

| Steps | Wall (CPU, 4 envs) | Typical `ep_rew_mean` |
|---|---|---|
| 25 k | 90 s | ~500 (smoke) |
| 100 k | 4 min | ~640 |
| 500 k | 30-40 min | ~700 |
| 5 M | 4-6 hr | usually walks (with curriculum) |

For production-quality walking, plan for 10-50 M timesteps. Either run
the trainer overnight or migrate to Isaac Gym / Brax for GPU-parallel
rollouts (those have 1000+ envs and 100-1000× wall speedup over CPU
SubprocVecEnv).
