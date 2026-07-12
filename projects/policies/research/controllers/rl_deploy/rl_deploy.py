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

"""Generic OmniSim RL deploy controller — loads any trained policy.

Reads a RobotSpec from a JSON sidecar next to the ONNX policy and uses
it to drive the robot's motors. No robot-specific code: same controller
runs Spot, ANYmal, a manipulator, anything with a `robot_spec.json` +
`policy.onnx` pair.

How it finds its policy + spec:

  1. Env var `OMNISIM_POLICY_ONNX` -> path to policy.onnx
  2. Fallback: world's controllerArgs `--policy <path>`
  3. Fallback: `projects/policies/research/inference/policies/spot_ppo_main/policy.onnx`
     (the canonical shipped Spot policy)

The matching `robot_spec.json` is assumed to live in the same
directory as the policy. Backends write it during ONNX export.

How it builds the observation:

  Looks at `RobotSpec.obs_recipe` and calls the matching packer below.
  Built-in recipes:
    * legged_locomotion  : 9 + 3*nu + 4 D
    * manipulation       : 6 + 3*nu D
    * balance            : 6 + 2*nu D
"""
from __future__ import annotations

# Diagnostic boot log. The controller writes major-step messages here so
# any startup failure inside the Webots controller subprocess (where
# stderr can be eaten in --batch mode) is still visible. Only enabled
# when OMNISIM_BOOT_LOG=1 is set; off by default to keep the controller
# silent in production.
import os as _os
_BOOT_LOG = r"C:\tmp\rl_deploy_boot.log" if _os.environ.get("OMNISIM_BOOT_LOG") else None
def _boot(msg):
    if _BOOT_LOG is None:
        return
    try:
        with open(_BOOT_LOG, "a") as _f:
            _f.write(f"rl_deploy: {msg}\n")
    except Exception:
        pass

if _BOOT_LOG is not None:
    try:
        _os.makedirs(r"C:\tmp", exist_ok=True)
        open(_BOOT_LOG, "w").close()
    except Exception:
        pass
_boot("imports starting")


import argparse
import json
import math
import os
import sys
import traceback
from pathlib import Path

import numpy as np

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(REPO_ROOT))

_boot(f"REPO_ROOT={REPO_ROOT}")


# Inline RobotSpec parsing — we deliberately do NOT import the backend
# package because that triggers _autoregister, which tries to import
# jax + torch + mujoco. Those work fine in the trainer process but
# can hang or take 30+ s inside Webots's controller subprocess, which
# breaks the controller startup handshake.
class _RobotSpecLite:
    """Minimal RobotSpec — JSON-loaded read-only view, no dataclass."""
    def __init__(self, d):
        self.name = d["name"]
        self.urdf_path = Path(d["urdf_path"])
        self.joint_names = tuple(d["joint_names"])
        self.nominal_pose = tuple(d["nominal_pose"])
        self.joint_limits_lo = tuple(d["joint_limits_lo"])
        self.joint_limits_hi = tuple(d["joint_limits_hi"])
        self.action_scale = float(d.get("action_scale", 0.15))
        self.obs_dim = int(d["obs_dim"])
        self.spawn_translation = tuple(d.get("spawn_translation", (0, 0, 0.7)))
        self.spawn_rotation = tuple(d.get("spawn_rotation", (0, 0, 1, 0)))
        self.max_episode_steps = int(d.get("max_episode_steps", 1024))
        self.obs_recipe = d.get("obs_recipe", "legged_locomotion")
        self.motor_pid = tuple(d.get("motor_pid", (20.0, 0.0, 0.3)))
        # Optional per-joint (kp, ki, kd) — when present, applied per
        # actuator instead of the scalar motor_pid. Atlas uses this to
        # clamp arms+back+neck at stiff hold so deploy matches training.
        self.motor_pid_per_joint = tuple(
            tuple(p) for p in d.get("motor_pid_per_joint", ()) or ()
        )
        # CPG params — match the training controller's trot prior. If
        # absent in the sidecar, defaults to no CPG (legacy behavior).
        self.cpg_freq_hz = float(d.get("cpg_freq_hz", 0.0))
        self.cpg_hip_y_amp = float(d.get("cpg_hip_y_amp", 0.0))
        self.cpg_knee_amp = float(d.get("cpg_knee_amp", 0.0))
        # Per-leg phase offsets for trot, ordered by joint_names. Default
        # trot: FL+RR in phase, FR+RL half-cycle offset.
        self.cpg_leg_phase = tuple(d.get(
            "cpg_leg_phase", (0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0)))

    @classmethod
    def from_file(cls, path):
        return cls(json.loads(Path(path).read_text()))

RobotSpec = _RobotSpecLite


# ── Observation recipes ──

def _proj_gravity_from_orientation_matrix(ori9):
    return np.array([-ori9[2], -ori9[5], -ori9[8]], dtype=np.float32)


def _cpg_offsets(t, n_act, joint_names, freq_hz, hip_y_amp, knee_amp, leg_phase):
    """Mirror of the training controller's CPG. Returns per-joint
    deviation to add on top of nominal_pose. The same formula as
    spot_rl_agent.cpg_base_offsets so the policy sees the same
    target-tracking distribution at deploy as during training."""
    base = np.zeros(n_act, dtype=np.float32)
    if freq_hz <= 0.0 or (hip_y_amp == 0.0 and knee_amp == 0.0):
        return base
    omega = 2.0 * math.pi * freq_hz
    for i, jname in enumerate(joint_names):
        phase = float(leg_phase[i]) if i < len(leg_phase) else 0.0
        theta = omega * t + 2.0 * math.pi * phase
        if jname.endswith("_hip_y"):
            base[i] = hip_y_amp * math.cos(theta)
        elif jname.endswith("_knee"):
            s = max(0.0, math.sin(theta))
            base[i] = -knee_amp * (s * s)
        # hip_x stays at nominal.
    return base


def pack_legged_locomotion(obs_dim, n_act, v_lin, v_ang, proj_g, q, qd,
                           last_action, vel_command, clock):
    obs = np.zeros(obs_dim, dtype=np.float32)
    s = 0
    obs[s:s+3] = v_lin; s += 3
    obs[s:s+3] = v_ang; s += 3
    obs[s:s+3] = proj_g; s += 3
    obs[s:s+n_act] = q; s += n_act
    obs[s:s+n_act] = qd; s += n_act
    obs[s:s+n_act] = last_action; s += n_act
    obs[s:s+3] = vel_command; s += 3
    obs[s] = clock
    return obs


def pack_manipulation(obs_dim, n_act, v_lin, v_ang, proj_g, q, qd,
                      last_action, vel_command, clock):
    obs = np.zeros(obs_dim, dtype=np.float32)
    s = 0
    obs[s:s+n_act] = q; s += n_act
    obs[s:s+n_act] = qd; s += n_act
    obs[s:s+n_act] = last_action; s += n_act
    obs[s:s+3] = vel_command; s += 3   # interpreted as xyz target
    obs[s:s+3] = proj_g
    return obs


def pack_balance(obs_dim, n_act, v_lin, v_ang, proj_g, q, qd,
                 last_action, vel_command, clock):
    obs = np.zeros(obs_dim, dtype=np.float32)
    s = 0
    obs[s:s+3] = v_ang; s += 3
    obs[s:s+3] = proj_g; s += 3
    obs[s:s+n_act] = q; s += n_act
    obs[s:s+n_act] = qd; s += n_act
    return obs


_RECIPES = {
    "legged_locomotion": pack_legged_locomotion,
    "manipulation": pack_manipulation,
    "balance": pack_balance,
}


def _parse_args():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--policy", default=None)
    p.add_argument("--spec", default=None)
    args, _ = p.parse_known_args()
    return args


def find_policy(cli_args) -> Path:
    candidates = []
    if cli_args.policy:
        candidates.append(Path(cli_args.policy))
    env = os.environ.get("OMNISIM_POLICY_ONNX") or os.environ.get("SPOT_POLICY_ONNX")
    if env:
        candidates.append(Path(env))
    candidates.append(REPO_ROOT / "projects" / "rl" / "inference" / "policies"
                      / "spot_ppo_main" / "policy.onnx")
    for c in candidates:
        if c.exists():
            return c
    return candidates[-1]


def find_spec(cli_args, policy_path: Path) -> Path:
    candidates = []
    if cli_args.spec:
        candidates.append(Path(cli_args.spec))
    env = os.environ.get("OMNISIM_ROBOT_SPEC")
    if env:
        candidates.append(Path(env))
    candidates.append(policy_path.parent / "robot_spec.json")
    candidates.append(REPO_ROOT / "projects" / "rl" / "inference" / "policies"
                      / "spot_ppo_main" / "robot_spec.json")
    for c in candidates:
        if c.exists():
            return c
    return candidates[-1]


def _read_velocity_command():
    vx = float(os.environ.get("OMNISIM_VX") or os.environ.get("SPOT_VX", 0.5))
    vy = float(os.environ.get("OMNISIM_VY") or os.environ.get("SPOT_VY", 0.0))
    wz = float(os.environ.get("OMNISIM_WZ") or os.environ.get("SPOT_WZ", 0.0))
    try:
        cmd_file = REPO_ROOT / "projects" / "rl" / "inference" / "current_command.txt"
        if cmd_file.exists():
            lines = cmd_file.read_text().strip().splitlines()
            if len(lines) >= 3:
                vx, vy, wz = float(lines[0]), float(lines[1]), float(lines[2])
    except Exception:
        pass
    return vx, vy, wz


def main() -> int:
    _boot("main starting")
    _boot(f"argv={sys.argv} cwd={os.getcwd()}")
    cli = _parse_args()
    policy_path = find_policy(cli)
    if not policy_path.exists():
        msg = f"[rl_deploy] ERROR: policy not found at {policy_path}\n"
        sys.stderr.write(msg)
        _boot(msg.rstrip("\n"))
        return 1

    spec_path = find_spec(cli, policy_path)
    if not spec_path.exists():
        msg = f"[rl_deploy] ERROR: robot_spec.json not found at {spec_path}\n"
        sys.stderr.write(msg)
        _boot(msg.rstrip("\n"))
        return 1

    spec = RobotSpec.from_file(spec_path)
    _boot(f"spec={spec.name} joints={len(spec.joint_names)} "
          f"obs_dim={spec.obs_dim} recipe={spec.obs_recipe}")

    import onnxruntime as ort
    sess = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
    _boot(f"loaded policy {policy_path.name}, instantiating Supervisor...")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0
    _boot(f"Supervisor ready, step_ms={step_ms}; resolving {len(spec.joint_names)} motors")

    motors = []
    sensors = []
    missing = []
    for i, jname in enumerate(spec.joint_names):
        mname = f"{jname}_motor"
        m = robot.getDevice(mname)
        if m is None:
            missing.append(mname)
            continue
        motors.append(m)
        try:
            if hasattr(m, "setControlPID"):
                if (spec.motor_pid_per_joint
                        and i < len(spec.motor_pid_per_joint)):
                    kp, ki, kd = spec.motor_pid_per_joint[i]
                else:
                    kp, ki, kd = spec.motor_pid
                m.setControlPID(float(kp), float(ki), float(kd))
        except Exception:
            pass
        s = m.getPositionSensor()
        if s is not None:
            s.enable(step_ms)
        sensors.append(s)

    if missing:
        _boot(f"missing motors: {missing}")
        return 1
    _boot(f"{len(motors)} motors resolved; getting supervisor handle")
    self_node = robot.getSelf()
    if self_node is None:
        _boot("no supervisor handle")
        return 1

    n_act = len(spec.joint_names)
    nominal = np.array(spec.nominal_pose, dtype=np.float32)
    lo = np.array(spec.joint_limits_lo, dtype=np.float32)
    hi = np.array(spec.joint_limits_hi, dtype=np.float32)

    for i, m in enumerate(motors):
        m.setPosition(float(nominal[i]))

    # Settle phase: step the sim ~20 ticks holding the nominal pose before
    # the policy starts driving. This mirrors what the training controller
    # does on every episode reset, so the policy sees the same initial
    # state distribution at deploy as during training. Without this, the
    # body is still in mid-drop from the spawn height when the first
    # policy action is applied, the obs is garbage, and the policy fails.
    for _ in range(20):
        if robot.step(step_ms) == -1:
            break

    prev_q = nominal.copy()
    # Re-read sensors after the settle so prev_q reflects the actual
    # settled joint positions, not the (target) nominal pose.
    for i, s in enumerate(sensors):
        if s is not None:
            try:
                prev_q[i] = float(s.getValue())
            except Exception:
                pass
    last_action = np.zeros(n_act, dtype=np.float32)

    vx, vy, wz = _read_velocity_command()
    vel_command = np.array([vx, vy, wz], dtype=np.float32)

    recipe = _RECIPES.get(spec.obs_recipe, pack_legged_locomotion)

    _boot(f"init complete, vel_cmd=({vx},{vy},{wz}), entering main loop")

    sim_time = 0.0
    sim_ms = 0

    trace = None
    trace_env = (os.environ.get("OMNISIM_DEPLOY_TRACE")
                 or os.environ.get("SPOT_DEPLOY_TRACE"))
    _boot(f"trace env={trace_env!r}")
    if trace_env:
        try:
            os.makedirs(r"C:\tmp\husky_trace", exist_ok=True)
            trace = open(r"C:\tmp\husky_trace\spot_deploy.csv", "w", buffering=1)
            trace.write("t_ms,bx,by,bz,roll,pitch,yaw,vx,vy,vz\n")
            _boot(f"trace opened")
        except Exception as _te:
            _boot(f"trace open FAILED: {_te}")
            trace = None
    next_trace_ms = 0

    while robot.step(step_ms) != -1:
        sim_time += step_dt
        sim_ms += step_ms
        try:
            pos = self_node.getPosition() or [0, 0, 0]
            ori = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
            vel = self_node.getVelocity() or [0]*6
        except Exception:
            pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6

        if trace is not None and sim_ms >= next_trace_ms:
            roll = math.atan2(ori[7], ori[8])
            pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
            yaw = math.atan2(ori[3], ori[0])
            trace.write(f"{sim_ms},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                        f"{roll:.4f},{pitch:.4f},{yaw:.4f},"
                        f"{vel[0]:.4f},{vel[1]:.4f},{vel[2]:.4f}\n")
            next_trace_ms = sim_ms + 100

        v_lin = np.array(vel[:3], dtype=np.float32)
        v_ang = np.array(vel[3:6], dtype=np.float32)
        proj_g = _proj_gravity_from_orientation_matrix(ori)
        q = np.zeros(n_act, dtype=np.float32)
        for i, s in enumerate(sensors):
            if s is not None:
                try:
                    q[i] = float(s.getValue())
                except Exception:
                    q[i] = 0.0
        qd = (q - prev_q) / max(step_dt, 1e-4)
        prev_q = q

        # Clock phase-locked to the CPG frequency (matches training).
        _clock_freq = spec.cpg_freq_hz if spec.cpg_freq_hz > 0.0 else 0.5
        clock = (sim_time * _clock_freq) % 1.0
        obs = recipe(spec.obs_dim, n_act, v_lin, v_ang, proj_g, q, qd,
                     last_action, vel_command, clock)
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0).astype(np.float32)

        action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        last_action = action.copy()
        cpg = _cpg_offsets(sim_time, n_act, spec.joint_names,
                           spec.cpg_freq_hz, spec.cpg_hip_y_amp,
                           spec.cpg_knee_amp, spec.cpg_leg_phase)
        target = nominal + cpg + spec.action_scale * action
        target = np.clip(target, lo, hi)
        for i, m in enumerate(motors):
            m.setPosition(float(target[i]))

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as _e:
        _boot(f"CRASH {_e}\n{traceback.format_exc()}")
        raise
