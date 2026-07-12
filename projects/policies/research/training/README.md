# projects/policies/research/training/

PPO trainers for OmniSim RL. Many entry points live here for historical reasons; this README orients new readers (and future-you) to which one to actually run.

## TL;DR — which trainer should I use?

### 🟢 Default recipe (use this for any new robot)

| Goal | Trainer | Notes |
|---|---|---|
| **Biped / manipulator / any robot deploying to OmniSim Newton** | [`gpu_mjwarp_g1_stand_trainer.py`](gpu_mjwarp_g1_stand_trainer.py) | Heavy-DR pure PPO on GPU mujoco_warp. **132 k env-steps/s** on RTX 5070; 30 M-step run in ~3 min 43 s. Closes the sim-to-deploy gap by training to be invariant to wrapper drift. Copy this file as the template for any new robot/task — see [`sim-to-deploy-rl-recipe.md`](../../../../docs/developer/sim-to-deploy-rl-recipe.md). Produces `gpu_g1_stand_robust` for G1. |

### Quadruped / static-stability recipes (kept; preferred when applicable)

| Goal | Trainer | Notes |
|---|---|---|
| Train the **canonical Spot walker** | [`train_residual.py`](train_residual.py) | Residual RL on the model walker (gait + IK + balance PD). 12-dim foot-offset action. **20× sample-efficient** vs from-scratch PPO; ~52 s wall on CPU produced `spot_residual_main`. |
| Same recipe, Newton physics | [`train_residual_newton.py`](train_residual_newton.py) | Sets the Newton-side opt-ins (URDF inertia, MuJoCo CPU solver, wrapper shape, seed-pose, KE/KD/μ) automatically. Produces `spot_residual_newton_main`. |
| Push-recovery residual | [`train_residual_perturb_newton.py`](train_residual_perturb_newton.py) | Same as above + Δv impulse injection during training. Justifies the RL part of the residual stack. |
| Recovery (get-up) residual | [`train_recovery.py`](train_recovery.py) | Scripted-pose classifier + residual; uses `SpotRecoveryEnv` (16-dim obs, episodic). |
| GPU-batched residual (Spot) | [`gpu_mjwarp_residual_trainer.py`](gpu_mjwarp_residual_trainer.py) | ~60 k env-steps/s on RTX 5070; same residual recipe. **Heads up:** policies trained on raw mujoco_warp diverge from OmniSim Newton — apply the heavy-DR recipe in `gpu_mjwarp_g1_stand_trainer.py` if you hit transfer issues. |
| Newton-faithful CPU training | [`newton_solver_trainer.py`](newton_solver_trainer.py) | Multiprocess PPO on the *exact* deploy engine (Newton SolverMuJoCo CPU). ~1.5 k steps/s/env but the only fully transfer-faithful CPU path. Useful as a baseline; the heavy-DR GPU recipe is generally faster end-to-end. |

> **Standard shipped policies:**
> - Spot walker: **`spot_residual_main`** ([`spot-residual-rl.md`](../../../../docs/developer/spot-residual-rl.md))
> - G1 stand:    **`gpu_g1_stand_robust`** ([`g1-stand-rl-playbook.md`](../../../../docs/developer/g1-stand-rl-playbook.md))

### Pipeline template helpers (new-robot bring-up)

| File | Purpose |
|---|---|
| [`import_newton_mjcf.py`](import_newton_mjcf.py) | Convert an `OMNISIM_NEWTON_SAVE_MJCF` dump (anonymous `body_N` / `joint_N` names) to a trainer-friendly MJCF with readable joint/body names. **Run this once per robot to capture the model Newton actually builds** — closes most sim-to-deploy MJCF gaps before any training. |
| [`build_g1_mjcf.py`](build_g1_mjcf.py) | Legacy: build MJCF from URDF with hand-set kp/kv. **Superseded** by `import_newton_mjcf.py` for any robot OmniSim deploys (Newton's dump is bit-identical to deploy; hand-built MJCFs always drift). |

## Continue / warm-start

| Trainer | Use when |
|---|---|
| [`continue_training.py`](continue_training.py) | Pushing a stable-but-slow walker faster. Loads a `.zip`, swaps in current env (reward weights from env vars), keeps training. Does NOT set Newton opt-ins. |
| [`continue_training_newton.py`](continue_training_newton.py) | Warm-continue under Newton with bit-identical dynamics to the source run (physics opt-ins + CPG prior forced; reward shaping left to caller). |
| [`continue_spot_newton.py`](continue_spot_newton.py) | Newton warm-start from an ODE-trained checkpoint (e.g. `spot_walk_v12`). Tuned for a "keep walking" curriculum. |

## Legacy / historical

These predate the residual stack and are kept so existing scripts/demos still work. Prefer `train_residual.py` for new work.

| Trainer | Status |
|---|---|
| [`train_spot.py`](train_spot.py) | Back-compat shim → forwards to `train_robot.py --robot spot`. |
| [`train_robot.py`](train_robot.py) | Generic from-scratch PPO over any registered robot + backend (sb3 / mjx). Still the path for new robots; superseded for Spot. |
| [`train_spot_walk.py`](train_spot_walk.py) | From-scratch PPO, reward-shape variant pushing velocity tracking. |
| [`train_spot_newton.py`](train_spot_newton.py) | From-scratch PPO under Newton. Use `train_residual_newton.py` instead. |
| [`gpu_mjwarp_trainer.py`](gpu_mjwarp_trainer.py) | CPG-residual GPU trainer (predecessor of `gpu_mjwarp_residual_trainer.py`). |
| [`spot_native.py`](spot_native.py) | Helper: builds the deploy-faithful Spot as a NATIVE Newton articulation. Imported by the Newton trainers; not a CLI entry point. |
| [`view_gpu_spot_residual.py`](view_gpu_spot_residual.py) | Live MuJoCo viewer for a trained GPU residual policy. Eval-only, not a trainer. |

## Outputs

- Runs land in [`runs/<run-name>/`](runs/) (gitignored). Checkpoints, the final `.zip`, and `.msgpack` model artifacts.
- Convert to ONNX with [`../inference/export_onnx.py`](../inference/export_onnx.py). Drop the result in [`../inference/policies/<run-name>/policy.onnx`](../inference/policies/) — the canonical policies (allowlisted in `.gitignore`) are checked in; everything else is local.
- Per-env scratch worlds (`projects/policies/worlds/.spot_rl*_env*.wbt`) are written next to the source world (controllers resolve relative to the world file) and are gitignored. They accumulate on disk across runs; safe to `rm` between runs.
