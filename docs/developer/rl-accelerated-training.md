# OmniSim accelerated training

> **For new-robot bring-up, start with the sim-to-deploy recipe** in
> [`sim-to-deploy-rl-recipe.md`](sim-to-deploy-rl-recipe.md) — it pins
> down the OmniSim-Newton-faithful path. The G1 standing case study is
> in [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md). The mujoco_warp
> backend below is the engine those recipes are built on.

> **Running MJX training on a new GPU box?** Read
> [`archive/rl-training-handoff.md`](archive/rl-training-handoff.md) first — it has the
> exact commands, dep list, and a smoke test/bench to run before the
> full training, plus a status snapshot of what's verified vs in-flight.

OmniSim's RL pipeline supports three training backends. They all
produce a `policy.onnx` with the same input/output schema, so the
OmniSim deploy controller doesn't care where the policy was trained.

```
                                         ┌──────────────────────────┐
  RobotSpec + TrainingConfig             │ OmniSim deploy world     │
       │                                 │ + spot_rl_deploy.py      │
       ▼                                 │   (or any robot's deploy)│
 ┌──────────────────────┐                │                          │
 │ Backend (interchangeable)             │  policy.onnx loaded via  │
 │                      │   policy.onnx  │  onnxruntime each tick   │
 │  sb3   — CPU subprocesses ───────────►│                          │
 │  mjx   — GPU-batched JAX/MuJoCo XLA  ─┤                          │
 │  isaac — NVIDIA Isaac Lab (remote)   ─┤                          │
 └──────────────────────┘                └──────────────────────────┘
```

## Choosing a backend

| Backend | Install | Speed | When to use |
|---|---|---|---|
| `sb3` | `pip install stable-baselines3 gymnasium` | ~150-500 steps/s | Always available; CPU; small experiments and debugging |
| `mjx` | `pip install jax jaxlib mujoco-mjx flax optax` | ~500 steps/s CPU, ~20k-100k steps/s on GPU | Production-scale training; GPU recommended; matches Isaac Gym's architecture |
| `isaac` | NVIDIA Isaac Sim + Isaac Lab (~20 GB; Linux preferred) | ~100k+ steps/s | If your team already runs Isaac Lab and wants to keep using it |

```bash
# See which are available on this machine
python projects/policies/research/training/train_spot.py --backend list
```

## sb3 backend — the default

Uses our existing OmniSim subprocess env (`SpotEnv` in
`projects/policies/research/envs/spot_env.py`). One OmniSim process per training env,
TCP/IP between them. Works everywhere SB3 works (i.e. any machine
with Python). Slow but always-correct baseline.

```bash
python projects/policies/research/training/train_spot.py --backend sb3 \
    --envs 4 --steps 500000
```

## mjx backend — fast, OmniSim-native

The MJX backend is the OmniSim-native equivalent of Isaac Gym. It uses
MuJoCo MJX as the physics engine: a JAX-compiled version of MuJoCo that
runs **batched physics steps on GPU**. Same architecture as Isaac —
1000+ parallel envs in one process, tensor-resident observations and
actions, custom JAX PPO. No Isaac Sim install required.

```bash
# CPU mode (JAX falls back to CPU when no GPU is present)
python projects/policies/research/training/train_spot.py --backend mjx \
    --envs 256 --steps 10000000

# GPU mode — install JAX with CUDA support, see below
```

### Getting GPU acceleration

JAX with CUDA on **Linux** or **WSL2** (Windows Subsystem for Linux):

```bash
pip install --upgrade "jax[cuda12]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
```

JAX on **native Windows** is CPU-only (this is JAX's limitation, not
OmniSim's). If you want GPU on Windows: install WSL2, run training
inside WSL, the `policy.onnx` exits at the end and deploys in your
existing Windows OmniSim install.

Verify GPU is detected:
```bash
python -c "import jax; print(jax.devices())"
# Expected: [CudaDevice(id=0)]  or similar
```

### What MJX does under the hood

1. **URDF -> MuJoCo XML**: `projects/policies/research/backends/_urdf_to_mjcf.py`
   rewrites ROS `package://` mesh URIs to absolute paths, adds
   position actuators for the joints listed in `RobotSpec.joint_names`,
   compiles a `mujoco.MjModel`.
2. **Batched env**: vmap over `mjx.step` for `--envs N` parallel sims.
3. **Custom JAX PPO**: ~250 LOC, mirrors SB3's hyperparameters for
   apples-to-apples comparison.
4. **ONNX export**: extracts the actor MLP weights from the flax
   train state, builds a PyTorch `nn.Sequential` with identical
   architecture, exports with `torch.onnx.export`. Same schema as
   the SB3 export. Bit-identical inference output (verified within
   `1e-4` in the export sanity check).

### Tuning knobs for MJX

```bash
# More envs (use as many as your GPU memory permits)
--envs 1024

# Bigger rollout buffer per update (recommended for stable PPO)
--n-steps 24 --batch-size 6144   # i.e. 24 * 1024 / 4 = batch=6144

# Domain randomization (planned; for now reward weights are env vars)
SPOT_R_LIN_VEL_WT=2.0 \
SPOT_R_ALIVE_BONUS=0.02 \
python projects/policies/research/training/train_spot.py --backend mjx --envs 1024 ...
```

## isaac backend — NVIDIA Isaac Lab adapter

For teams who already have Isaac Lab installed and want the same
"hit one button" UX. **Not auto-tested in this repo** — the adapter
follows NVIDIA's public docs but verification needs an Isaac Lab
install to point at.

```bash
# One-time
# 1. Install Isaac Sim + Isaac Lab per
#    https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/
# 2. Tell the adapter where it lives:
export ISAAC_LAB_PATH=/path/to/IsaacLab

# Then train
python projects/policies/research/training/train_spot.py --backend isaac --steps 10000000
```

What the adapter does:
1. Discovers Isaac Lab via `$ISAAC_LAB_PATH`.
2. Generates a task config file mirroring our `RobotSpec` (obs, action,
   reward, termination).
3. Subprocess-launches `isaaclab.sh -p -m
   isaaclab.scripts.reinforcement_learning.rsl_rl.train`.
4. Finds the resulting rsl_rl checkpoint in Isaac Lab's `logs/`.
5. Converts the torch state_dict to OmniSim's ONNX schema by
   reconstructing the actor MLP layer by layer.
6. Drops the ONNX in `projects/policies/research/inference/policies/<run_name>/`.

The exact Isaac Lab task version (legged_gym vs legged_robot vs
ManagerBased) is documented in
`projects/policies/research/backends/isaac_remote.py` and may need adjustment if
NVIDIA renames their reference tasks.

## Adding a new robot to OmniSim's RL platform

This is a real "OmniSim feature" — anyone with a URDF can train a
policy and deploy it without editing the trainer or the deploy
controller. Concretely, four files:

### 1. The URDF — must already be in OmniSim

Live in `projects/robots/<vendor>/<robot>/urdf/<robot>.urdf`. The MJX
backend handles two URDF gotchas automatically:

* **`package://` mesh URIs** — rewritten to absolute paths assuming
  the URDF lives in `<package>/urdf/` and meshes in `<package>/meshes/`.
* **No actuators on revolute joints** — MuJoCo doesn't auto-promote
  URDF revolutes, so the loader adds position actuators for the joints
  listed in your `RobotSpec.joint_names`.

For sim stability under RL (cycles of swing/stance, full body-weight
loads), the URDF should have:

* foot ball colliders (or proper toe geometry) on lower legs / end
  effectors — long-bar collision meshes generate side friction that
  destabilises any open-loop or learned policy;
* `<dynamics damping="..."/>` on each joint (~0.05 N·m·s/rad for
  Spot-class motors) — RL discovers self-oscillation exploits without
  it;
* realistic `effort` and `velocity` limits on each joint — the policy
  learns around real actuator constraints.

The Spot URDF demonstrates all three; see commit `6333f075` and
`projects/policies/research/tools/patch_spot_urdf_for_rl.py` for the exact edits.

### 2. The RobotSpec — `projects/policies/research/backends/<robot>_robot_spec.py`

```python
from pathlib import Path
from .base import RobotSpec

REPO_ROOT = Path(__file__).resolve().parents[3]

JOINT_NAMES = (
    # ordered list of motor joints. MUST match the URDF's joint names
    # exactly (the deploy controller calls robot.getDevice(f"{name}_motor")).
    "hip_x_left", "hip_y_left", "knee_left",
    "hip_x_right", "hip_y_right", "knee_right",
)

NOMINAL_POSE = (0.0, 0.3, -0.6,  0.0, 0.3, -0.6)   # standing pose
JOINT_LIMITS_LO = (-0.5, 0.0, -1.2,  -0.5, 0.0, -1.2)
JOINT_LIMITS_HI = (+0.5, 0.6, -0.05, +0.5, 0.6, -0.05)

MY_BIPED = RobotSpec(
    name="my_biped",
    urdf_path=REPO_ROOT / "projects" / "robots" / "mine" / "biped" / "urdf" / "biped.urdf",
    joint_names=JOINT_NAMES,
    nominal_pose=NOMINAL_POSE,
    joint_limits_lo=JOINT_LIMITS_LO,
    joint_limits_hi=JOINT_LIMITS_HI,
    action_scale=0.15,
    obs_dim=9 + 3 * len(JOINT_NAMES) + 4,      # legged_locomotion default
    spawn_translation=(0, 0, 0.55),
    max_episode_steps=1024,
    obs_recipe="legged_locomotion",            # or "manipulation" / "balance"
    motor_pid=(20.0, 0.0, 0.3),
)
```

### 3. Register it — one line in `robot_registry.py`

```python
# projects/policies/research/backends/robot_registry.py
ROBOTS = {
    "spot":     ("projects.policies.research.backends.spot_robot_spec", "SPOT"),
    "my_biped": ("projects.policies.research.backends.my_biped_robot_spec", "MY_BIPED"),
}
```

### 4. Train + deploy — no other code edits

```bash
# Verify registry
python projects/policies/research/training/train_robot.py --robot list
# Expected: spot, my_biped

# Train (any backend)
python projects/policies/research/training/train_robot.py --robot my_biped --backend mjx \
       --envs 1024 --steps 10000000 --run-name my_biped_v1

# Output:
#   training/runs/my_biped_v1/biped_final.msgpack       (checkpoint)
#   inference/policies/my_biped_v1/policy.onnx           (deploy artifact)
#   inference/policies/my_biped_v1/robot_spec.json       (sidecar)

# Deploy in OmniSim
OMNISIM_POLICY_ONNX=projects/policies/research/inference/policies/my_biped_v1/policy.onnx \
   omnisim-bin.exe projects/policies/research/worlds/rl_deploy.wbt
```

The generic `rl_deploy` world has Spot as its default URDF; for your
robot, copy `rl_deploy.wbt` to `my_biped_rl_deploy.wbt` and swap the
`URDFRobot` `url` field to your URDF path. That's the only edit
needed — the controller reads everything else from the sidecar.

## Reward recipes (for non-locomotion robots)

The MJX backend ships three pluggable reward recipes selected via
`RobotSpec.obs_recipe`:

| Recipe | What it rewards | Use for |
|---|---|---|
| `legged_locomotion` | xy velocity tracking, yaw rate tracking, low body bob, level orientation, smoothness, alive bonus | quadrupeds, bipeds, anything that walks |
| `manipulation` | end-effector distance to target, joint smoothness | reach arms (vel_cmd interpreted as xyz target) |
| `balance` | level orientation + low body angular velocity (no command tracking) | pendulum, standing-only demos |

Per-recipe weights come from env vars at training time:

```bash
SPOT_R_LIN_VEL_WT=2.0 SPOT_R_ALIVE_BONUS=0.02 \
  python projects/policies/research/training/train_robot.py --robot spot --backend mjx ...
```

To add a custom recipe: define a function in
`projects/policies/research/backends/reward_recipes.py` and call `register("my_recipe", fn)`.

## Cross-backend compatibility

| Backend | Robot-agnostic? |
|---|---|
| `sb3`   | Currently Spot-only (gym env is Spot-specific). Generalising means copying `spot_env.py` per robot — `~half-day` per robot. |
| `mjx`   | Fully robot-agnostic via `RobotSpec` + `obs_recipe`. No code edits needed. |
| `isaac` | Currently scaffolded for legged_gym-style Spot. Other robots need an Isaac task definition tweak in `isaac_remote.py`. |

For new robots, **`mjx` is the path that "just works"**.

## ONNX schema

Every backend writes the same ONNX:

| Tensor | Shape | dtype |
|---|---|---|
| input `obs` | `[B, obs_dim]` | float32 |
| output `action` | `[B, len(joint_names)]` | float32, range ~[-1, 1] |

The deploy controller clips actions to `[-1, 1]`, multiplies by
`ACTION_SCALE`, adds to `NOMINAL_POSE`, clamps to joint limits, sends
to motors.

## Wall-clock benchmarks

Approximate, on a laptop with RTX 3060 + i7-12700H:

| Config | Steps/sec |
|---|---|
| `sb3 --envs 1` (single OmniSim subprocess) | 161 |
| `sb3 --envs 4 --single` | 300-500 |
| `mjx --envs 32` CPU | ~500-1000 |
| `mjx --envs 1024` CPU | ~500-2000 (CPU plateau) |
| `mjx --envs 1024` on 3060 (WSL2 + jaxcuda) | ~20 000-50 000 |
| `mjx --envs 4096` on 3060 (WSL2 + jaxcuda) | ~30 000-100 000 |
| `isaac --num_envs 4096` Isaac Lab on 3060 | ~30 000-100 000 |

10 M timesteps:
- sb3 single env: ~17 hours
- sb3 4 envs: ~5 hours
- mjx CPU 1024 envs: ~1-3 hours
- mjx GPU 1024 envs: ~5-15 minutes

## Troubleshooting

**mjx: "MuJoCo could not load URDF: package://..."**
Mesh URI rewriting requires the URDF live in `<package>/urdf/` with
meshes under `<package>/meshes/`. If your layout differs, edit
`_rewrite_package_uris` in `projects/policies/research/backends/_urdf_to_mjcf.py`.

**mjx: zero actuators (`nu=0`)**
URDF revolute joints don't auto-become MuJoCo actuators. The
`load_or_convert` helper adds position actuators for the joints listed
in `RobotSpec.joint_names`. Check the names match the URDF.

**isaac: "Isaac Lab not found"**
Set `ISAAC_LAB_PATH` to the directory containing `isaaclab.sh` (or
`isaaclab.bat` on Windows).

**ONNX deploy: action signs flipped or joints go to wrong limits**
The deploy controller's `JOINT_LIMITS_LO/HI` and `NOMINAL_POSE` must
match the `RobotSpec` used at training time. They're meant to be
imported from `spot_robot_spec.py` or your robot's equivalent.
