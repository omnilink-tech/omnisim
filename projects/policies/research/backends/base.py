# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Backend protocol — minimal contract for an OmniSim RL trainer.

A backend must:
  * declare whether it `is_available()` on this machine
  * `train()` for a target step count given a RobotSpec + TrainingConfig
  * write a policy.onnx that matches the OmniSim deploy controller's
    expected input/output schema for the robot

That's it. Each backend internally manages its own physics engine,
parallelism, and PPO implementation. The OmniSim deploy layer is
backend-agnostic — it loads the ONNX + the matching robot_spec.json
sidecar and runs it.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, Optional, Protocol, Sequence


# ──────────────────────────────────────────────────────────────────────
# Robot spec
# ──────────────────────────────────────────────────────────────────────

@dataclass
class RobotSpec:
    """Everything a backend needs to instantiate one robot for RL.

    Designed to be reusable: any URDF-imported OmniSim robot can be
    trained by populating this. Spot's spec is in `spot_robot_spec.py`.
    """

    name: str
    """Robot identifier, used for output paths and ONNX naming."""

    urdf_path: Path
    """Absolute path to the URDF that OmniSim deploys. The MJX backend
    converts this to MuJoCo XML (caching at urdf_path.with_suffix('.xml'))
    via the conversion helper in `backends/_urdf_to_mjcf.py`."""

    joint_names: Sequence[str]
    """Ordered list of motor joint names. Same order is used everywhere:
    in observations, actions, and the deploy controller. Length =
    ACTION_DIM."""

    nominal_pose: Sequence[float]
    """Standing pose joint angles, ordered to match joint_names. Action
    = nominal_pose + ACTION_SCALE * policy_output."""

    joint_limits_lo: Sequence[float]
    joint_limits_hi: Sequence[float]
    """Per-joint lower/upper bounds (rad). Action clamped to these
    after adding nominal_pose."""

    action_scale: float = 0.15
    """Residual policy: how much delta the network can express around
    the nominal pose, in radians."""

    obs_dim: int = 49
    """Observation vector length. Must match the deploy controller."""

    spawn_translation: Sequence[float] = (0.0, 0.0, 0.70)
    spawn_rotation: Sequence[float] = (0.0, 0.0, 1.0, 0.0)
    """Initial body pose at episode reset (axis-angle for rotation)."""

    max_episode_steps: int = 1024

    obs_recipe: str = "legged_locomotion"
    """Which observation packing the policy expects. The generic deploy
    controller looks this up to build the obs vector consistently. Built-in
    recipes (see `obs_recipes.py` / generic deploy controller):
      'legged_locomotion' : 9 + 3*nu + 4 D (Spot/ANYmal/quadrupeds)
      'manipulation'      : 6 + 3*nu     D (arm reach tasks)
      'balance'           : 6 + 2*nu     D (standing/bipedal balance)
    Custom recipes can be registered by the controller side."""

    motor_pid: Sequence[float] = (20.0, 0.0, 0.3)
    """(kp, ki, kd) for setControlPID at deploy time. None of OmniSim's
    URDFRobot exposes torque control without explicit PID tuning."""

    motor_pid_per_joint: Sequence[Sequence[float]] = field(default_factory=lambda: ())
    """Optional per-joint (kp, ki, kd), one entry per `joint_names`. When
    non-empty, the MJX backend uses these per-actuator gains; when empty,
    it falls back to the scalar `motor_pid` for every joint. Heavy robots
    (e.g. Atlas at 175 kg) need stiff hold on non-task joints (arms,
    back) and moderate gains on task joints (legs) so the policy's
    residual still has authority where it matters."""

    # ── CPG trotting prior (legged robots) ──
    cpg_freq_hz: float = 0.0
    """Gait frequency in Hz; 0.0 disables the CPG (legacy "residual on
    nominal pose" behavior). For trot, ~1.5-2.0 Hz works well for Spot."""

    cpg_hip_y_amp: float = 0.0
    """Hip-y oscillation amplitude (rad). Drives the fore/aft leg swing."""

    cpg_knee_amp: float = 0.0
    """Knee oscillation amplitude (rad). Drives the foot-lift during swing."""

    cpg_leg_phase: Sequence[float] = field(default_factory=lambda: ())
    """Per-joint phase offset in [0, 1). Length = nu (one per joint).
    Default empty → deploy controller picks trot phases by joint name."""

    # ── Serialization for the sidecar file the deploy controller reads ──

    def to_json(self) -> str:
        """Serialize to a JSON string the generic deploy controller can read.
        Path is stringified relative to the repo root if possible, else
        absolute."""
        d = asdict(self)
        d["urdf_path"] = str(self.urdf_path)
        # Tuples become lists when JSON-encoded; explicit listification is
        # for stable round-trip.
        for k in ("joint_names", "nominal_pose", "joint_limits_lo",
                  "joint_limits_hi", "spawn_translation", "spawn_rotation",
                  "motor_pid"):
            d[k] = list(d[k])
        if d.get("motor_pid_per_joint"):
            d["motor_pid_per_joint"] = [list(p) for p in d["motor_pid_per_joint"]]
        return json.dumps(d, indent=2)

    def write_sidecar(self, dest_dir: Path) -> Path:
        """Write `<dest_dir>/robot_spec.json` next to a policy.onnx."""
        dest_dir = Path(dest_dir)
        dest_dir.mkdir(parents=True, exist_ok=True)
        out = dest_dir / "robot_spec.json"
        out.write_text(self.to_json())
        return out

    @classmethod
    def from_json(cls, text: str) -> "RobotSpec":
        d = json.loads(text)
        d["urdf_path"] = Path(d["urdf_path"])
        for k in ("joint_names", "nominal_pose", "joint_limits_lo",
                  "joint_limits_hi", "spawn_translation", "spawn_rotation",
                  "motor_pid"):
            if k in d:
                d[k] = tuple(d[k])
        return cls(**d)

    @classmethod
    def from_file(cls, path: Path) -> "RobotSpec":
        return cls.from_json(Path(path).read_text())


# ──────────────────────────────────────────────────────────────────────
# Training config
# ──────────────────────────────────────────────────────────────────────

@dataclass
class TrainingConfig:
    total_steps: int = 1_000_000
    parallel_envs: int = 4
    n_steps_per_env: int = 512
    batch_size: int = 1024
    n_epochs: int = 4
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    ent_coef: float = 0.005
    vf_coef: float = 0.5
    max_grad_norm: float = 1.0
    target_kl: Optional[float] = None
    """If set, SB3 PPO aborts the current update epoch when the policy's
    KL divergence from the rollout policy exceeds this. Acts as a safety
    rail: prevents a single bad batch from catastrophically shifting the
    policy mean (the failure mode we hit repeatedly during walk training)."""

    # Reward weights (read by env at runtime; backends pass through).
    r_lin_vel_wt: float = 1.0
    r_ang_vel_wt: float = 0.5
    r_vz_wt: float = -1.0
    r_rollpitch_wt: float = -0.1
    r_ang_xy_wt: float = -0.02
    r_action_rate_wt: float = -0.005
    r_joint_acc_wt: float = -2.5e-7
    r_alive_bonus: float = 0.05
    # Direct linear-forward-velocity bonus. Used by both the Webots
    # controller (via env var) and the MJX legged_locomotion recipe.
    r_vx_bonus_wt: float = 0.0
    r_vx_deadband: float = 0.05

    # Termination thresholds.
    bz_fail: float = 0.30
    roll_fail: float = 1.0
    pitch_fail: float = 1.0

    # Velocity command range (curriculum-ready).
    vx_min: float = 0.20
    vx_max: float = 0.80
    wz_min: float = -0.30
    wz_max: float = 0.30

    # Network.
    net_arch: Sequence[int] = field(default_factory=lambda: [256, 256])
    log_std_init: float = -1.0

    # Run output naming.
    run_name: Optional[str] = None
    checkpoint_every_steps: int = 100_000

    # Device hint ("auto" | "cpu" | "cuda" | "tpu").
    device: str = "auto"

    # Free-form passthrough for backend-specific knobs.
    extra: Dict = field(default_factory=dict)


# ──────────────────────────────────────────────────────────────────────
# Backend protocol
# ──────────────────────────────────────────────────────────────────────

class TrainingBackend(Protocol):
    """Protocol for a training backend."""

    name: str

    def is_available(self) -> bool:
        """Return True if this backend can run on the current machine.
        Backends check for required libraries (Isaac Sim, MJX, etc.)
        and platform/GPU constraints here."""
        ...

    def train(
        self,
        robot: RobotSpec,
        config: TrainingConfig,
        out_dir: Path,
    ) -> Path:
        """Train a policy and return the path to the final checkpoint
        (backend-native format). Should also write tensorboard logs
        under out_dir/tb and intermediate checkpoints under
        out_dir/checkpoints/."""
        ...

    def export_onnx(self, ckpt: Path, out: Path, robot: RobotSpec) -> None:
        """Convert backend-specific checkpoint to OmniSim's canonical
        ONNX schema (input='obs' shape [B, obs_dim], output='action'
        shape [B, len(joint_names)]). Must round-trip with onnxruntime
        such that `sess.run(None, {'obs': obs}) == backend.predict(obs)`
        to within ~1e-4."""
        ...


# ──────────────────────────────────────────────────────────────────────
# Backend registry
# ──────────────────────────────────────────────────────────────────────

_REGISTRY: Dict[str, Callable[[], TrainingBackend]] = {}


def register(name: str, factory: Callable[[], TrainingBackend]) -> None:
    """Register a backend factory. Lazy: factory not called until
    get_backend(name) so backends with optional deps don't crash on
    import."""
    _REGISTRY[name] = factory


def list_backends() -> Dict[str, bool]:
    """Map backend name -> availability. Lets the CLI print
    'sb3: available, mjx: available, isaac: missing'."""
    out = {}
    for name, factory in _REGISTRY.items():
        try:
            b = factory()
            out[name] = b.is_available()
        except Exception:
            out[name] = False
    return out


def get_backend(name: str) -> TrainingBackend:
    if name not in _REGISTRY:
        raise ValueError(
            f"Unknown backend {name!r}. Registered: {sorted(_REGISTRY)}"
        )
    return _REGISTRY[name]()


# Auto-register the shipped backends. Each module is imported lazily so
# missing optional deps don't break others.
def _autoregister() -> None:
    # SB3 (always present — it's how we got here).
    try:
        from . import sb3_cpu
        register("sb3", sb3_cpu.factory)
    except Exception as e:
        print(f"[backends] skipping sb3: {e}")

    # MJX (requires jax + mujoco_mjx + flax + optax).
    try:
        from . import mjx_gpu
        register("mjx", mjx_gpu.factory)
    except Exception as e:
        print(f"[backends] skipping mjx: {e}")

    # Isaac (write-only adapter; no auto-test).
    try:
        from . import isaac_remote
        register("isaac", isaac_remote.factory)
    except Exception as e:
        print(f"[backends] skipping isaac: {e}")


_autoregister()
