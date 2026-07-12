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

"""OmniSim deploy controller for the CANONICAL G1 walk policy.

Runs the policy trained by ``projects/policies/research/training/gpu_humanoid_walk_trainer.py``
(``HumanoidWalkEnv``, robot=g1). That trainer is PURE RL (no ghost / no residual /
no frame-stack / no lookahead): a 47-d observation, a 12-d action clamped in-graph
to [-1, 1], lower-body only (the 12 leg joints; the waist is PD-held at 0).

OBSERVATION PARITY IS THE #1 REQUIREMENT. This controller assembles the obs in the
EXACT order ``HumanoidWalkEnv._build_obs`` uses, with the EXACT scales, so the
deployed policy sees what it was trained on. The 47-d vector, in order::

    [ base_ang_vel_BODY * 0.25            (3)
      projected_gravity                   (3)
      command * [2.0, 2.0, 0.25]          (3)   (vx, vy, yaw_rate)
      (joint_pos - default_pose)          (12)
      joint_vel * 0.05                     (12)
      last_action                          (12)
      sin(2*pi*phase), cos(2*pi*phase)     (2) ]

The shared primitives (projected gravity, the world->body ang-vel R^T rotation, the
finite-diff joint-velocity estimator) come from ``projects.policies.research.backends.g1_env_core``
-- the SAME module the trainer-parity tooling uses -- so parity is structural.

ACTION -> MOTOR (every tick)::

    action       = onnx(obs)                          # already in [-1, 1]
    target[i]    = default_pose[i] + 0.25 * action[i] # 12 leg joints
    target[i]    = clamp(target[i], LIM_LO[i], LIM_HI[i])
    motors.setPosition(target[i])                     # 12 leg motors
    waist_yaw    -> 0.0                                # held
    last_action  = action

Env vars:
    G1_CANON_ONNX   path to policy.onnx (default: the v3 run)
    G1_DEPLOY_LOG   optional side log (mirrors the per-second telemetry)
    G1_CMD_VX/VY/WZ optional command overrides (default vx=0.3, vy=0, wz=0)

This is a CLEAN sibling of g1_walk_deploy.py: no gait-model, no lookahead, no
frame-stack, no two-policy machinery, no balance PD. Just obs -> onnx -> motors.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

# SINGLE SOURCE for the shared obs primitives (projected gravity from the 9-float
# orientation matrix, the world->body ang-vel R^T rotation, and the finite-diff
# joint-velocity estimator). Using these is what makes obs parity STRUCTURAL.
from projects.policies.research.backends import g1_env_core as core  # noqa: E402
# Joint ORDER + per-joint URDF limits come from the spec (the trainer clamps to the
# SAME limits -- the MJCF jnt_range is byte-identical to SPEC.leg_limits()[:12]).
from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:
    try:
        from omnisim import Supervisor as _Robot
    except Exception:  # last resort: plain Robot (no getSelf -> telemetry degraded)
        from omnisim import Robot as _Robot


# ── Policy joints: the 12 G1 leg joints, EXACTLY the trainer's g1_cfg() order. ──
# (SPEC.LEGS_JOINTS[:12] is byte-identical to that tuple; the 13th is waist_yaw.)
POLICY_JOINTS = list(SPEC.LEGS_JOINTS[:12])
WAIST_JOINT = SPEC.LEGS_JOINTS[12]   # "waist_yaw_joint" -- held at 0.0
assert POLICY_JOINTS == [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
], POLICY_JOINTS

NJ = 12
OBS_DIM = 47

# default_pose -- the trainer's g1_cfg() deep-squat seed (len 12). This is the
# nominal the obs subtracts (q - default) AND the base the action offsets from.
DEFAULT_POSE = np.array([
    -0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
    -0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
], dtype=np.float32)

# Per-joint URDF limits, in POLICY_JOINTS order. The trainer clamps its position
# targets to the MJCF jnt_range; that range is byte-identical to the URDF prim
# limits SPEC reads, so this is exact train<->deploy clamp parity.
_lo, _hi, _vel, _eff = SPEC.leg_limits()
LIM_LO = np.asarray(_lo[:12], dtype=np.float32)
LIM_HI = np.asarray(_hi[:12], dtype=np.float32)

# Control / gait constants -- MUST match the trainer.
ACT_SCALE = 0.25     # target = default + ACT_SCALE * action  (HumanoidWalkEnv.ACT_SCALE)
GAIT_PERIOD = 0.8    # seconds per gait cycle (HumanoidWalkEnv GAIT_PERIOD)
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)   # command obs scale

# Fixed forward command (vx, vy, yaw_rate) -- always moving so the phase advances.
COMMAND = np.array([
    float(os.environ.get("G1_CMD_VX", "0.3")),
    float(os.environ.get("G1_CMD_VY", "0.0")),
    float(os.environ.get("G1_CMD_WZ", "0.0")),
], dtype=np.float32)

# Fall thresholds (telemetry only).
FALL_Z = 0.45
FALL_TILT = 0.8


def _find_onnx() -> Path:
    p = os.environ.get("G1_CANON_ONNX")
    if p:
        return Path(p)
    return _REPO / "projects/policies/research/training/runs/g1_walk_v3/policy.onnx"


def _ori_to_rpy(o):
    """roll, pitch, yaw from the 9-float row-major body->world matrix (matches
    g1_walk_deploy._ori_matrix_to_rpy)."""
    return (math.atan2(o[7], o[8]),
            math.asin(max(-1.0, min(1.0, -o[6]))),
            math.atan2(o[3], o[0]))


def main() -> int:
    side_log_path = os.environ.get("G1_DEPLOY_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg: str) -> None:
        sys.stderr.write(msg)
        sys.stderr.flush()
        if side_log is not None:
            side_log.write(msg)
            side_log.flush()

    # ── Load the ONNX policy. ──
    policy_path = _find_onnx()
    sess = None
    if policy_path.exists():
        try:
            import onnxruntime as ort  # type: ignore
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say(f"[g1_walk_canon] loaded ONNX {policy_path}\n")
        except Exception as e:
            say(f"[g1_walk_canon] ONNX load failed ({e}); holding default pose\n")
    else:
        say(f"[g1_walk_canon] no ONNX at {policy_path}; holding default pose\n")
    in_name = sess.get_inputs()[0].name if sess is not None else "obs"

    # ── Robot + devices. ──
    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0      # control dt; 16 ms -> 0.016 s, matches trainer DT
    say(f"[g1_walk_canon] step_ms={step_ms} dt={step_dt:.4f} cmd={COMMAND.tolist()}\n")

    motors = {}
    sensors = {}
    for jn in POLICY_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[g1_walk_canon] missing motor {jn}_motor\n")
            return 1
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors[jn] = s
        except Exception:
            sensors[jn] = None
    # Waist motor (held at 0). Optional -- legs-only URDFs may not expose it.
    waist_motor = robot.getDevice(f"{WAIST_JOINT}_motor")

    # Supervisor self node for base pose / velocity (the obs needs world-frame
    # angular velocity, rotated into the body frame; and base height/pitch for
    # telemetry). Obtained exactly as g1_walk_deploy does.
    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass
    if self_node is None:
        say("[g1_walk_canon] WARNING: no supervisor self node; obs ang-vel/gravity "
            "will be degraded\n")

    # ── Settle: hold the default pose briefly so the cold articulation reaches the
    #    squat before the first policy tick (parity with the trainer's seed pose). ──
    for jn, q in zip(POLICY_JOINTS, DEFAULT_POSE):
        motors[jn].setPosition(float(q))
    if waist_motor is not None:
        waist_motor.setPosition(0.0)
    for _ in range(max(1, int(0.5 / step_dt))):
        if robot.step(step_ms) == -1:
            return 0

    # ── Per-episode obs state. ──
    last_action = np.zeros(NJ, dtype=np.float32)
    phase = 0.0                              # gait phase in [0, 1), starts at 0
    # Finite-diff joint velocity (deploy has no velocity sensor). dt=0.016, raw
    # finite-diff (tau=0) -- the SHARED estimator the trainer-parity path defines.
    qd_est = core.JointVelEstimator(step_dt, 0.0)
    # Seed prev_q from the settled pose so the first qd isn't a fake spike.
    seed_q = DEFAULT_POSE.copy()
    for i, jn in enumerate(POLICY_JOINTS):
        s = sensors.get(jn)
        if s is not None:
            try:
                seed_q[i] = float(s.getValue())
            except Exception:
                pass
    qd_est.reset(seed_q.copy())
    prev_q = seed_q.copy()

    cmd_obs = (COMMAND * CMD_SCALE).astype(np.float32)   # command * [2,2,0.25]

    sim_ms = 0
    last_log_ms = 0
    start_x = None
    fell_at = None

    while True:
        if robot.step(step_ms) == -1:
            break
        sim_ms += step_ms

        # ── Base pose / velocity (world frame). ──
        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0.0, 0.0, 0.0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                vel6 = self_node.getVelocity() or [0.0] * 6
            except Exception:
                pos, ori, vel6 = [0.0, 0.0, 0.0], [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.0] * 6
        else:
            pos, ori, vel6 = [0.0, 0.0, 0.0], [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.0] * 6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        if start_x is None:
            start_x = bx
        ang_world = vel6[3:6]

        # ── Joint positions (from sensors) + finite-diff velocity. ──
        joint_q = np.zeros(NJ, dtype=np.float32)
        for i, jn in enumerate(POLICY_JOINTS):
            s = sensors.get(jn)
            if s is not None:
                try:
                    joint_q[i] = float(s.getValue())
                except Exception:
                    joint_q[i] = prev_q[i]
            else:
                joint_q[i] = prev_q[i]
        joint_qd = qd_est.update(joint_q).astype(np.float32)
        prev_q = joint_q.copy()

        # ── OBS ASSEMBLY -- EXACT order/scale of HumanoidWalkEnv._build_obs. ──
        # 1) base angular velocity, BODY frame, * 0.25.
        #    Trainer reads MuJoCo free-joint qvel[3:6] (already body-frame); the
        #    deploy's getVelocity() ang-vel is WORLD-frame, so rotate R^T first.
        ang_body = np.array(core.world_ang_to_body_matrix(ori, ang_world),
                            dtype=np.float32)
        o_ang = ang_body * 0.25
        # 2) projected gravity = (-R[6], -R[7], -R[8]).
        o_pg = np.array(core.proj_gravity_from_matrix(ori), dtype=np.float32)
        # 3) command * [2.0, 2.0, 0.25].
        o_cmd = cmd_obs
        # 4) (joint_pos - default_pose).
        o_q = joint_q - DEFAULT_POSE
        # 5) joint_vel * 0.05.
        o_qd = joint_qd * 0.05
        # 6) last_action (previous tick's clamped action; zeros on tick 0).
        o_la = last_action
        # 7) sin(2*pi*phase), cos(2*pi*phase).
        ph = 2.0 * math.pi * phase
        o_gait = np.array([math.sin(ph), math.cos(ph)], dtype=np.float32)

        obs = np.concatenate([o_ang, o_pg, o_cmd, o_q, o_qd, o_la, o_gait]).astype(np.float32)
        # Trainer sanitises: nan_to_num then clamp to [-10, 10].
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0)
        assert obs.shape[0] == OBS_DIM, f"obs shape {obs.shape} != {OBS_DIM}"

        # ── Policy -> action (already clamped to [-1,1] in-graph; clamp again for safety). ──
        if sess is not None:
            try:
                out = sess.run(None, {in_name: obs[None, :]})
                action = np.clip(out[0][0], -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[g1_walk_canon] inference failed: {e}; holding\n")
                action = np.zeros(NJ, dtype=np.float32)
        else:
            action = np.zeros(NJ, dtype=np.float32)

        # ── Action -> motor targets. ──
        targets = DEFAULT_POSE + ACT_SCALE * action
        targets = np.clip(targets, LIM_LO, LIM_HI)
        for i, jn in enumerate(POLICY_JOINTS):
            motors[jn].setPosition(float(targets[i]))
        if waist_motor is not None:
            waist_motor.setPosition(0.0)

        # ── Update obs state for the NEXT tick. ──
        last_action = action
        # Always-moving command -> phase always advances (trainer: phase advances
        # only when |cmd|>0.05, which is always true here for vx=0.3).
        phase = (phase + step_dt / GAIT_PERIOD) % 1.0

        # ── Telemetry (per second). ──
        roll, pitch, _yaw = _ori_to_rpy(ori)
        fell = (bz < FALL_Z) or (abs(roll) > FALL_TILT) or (abs(pitch) > FALL_TILT)
        if fell and fell_at is None:
            fell_at = sim_ms / 1000.0
        if sim_ms - last_log_ms >= 1000:
            fwd = bx - start_x
            tag = f"FALL@{fell_at:.2f}s" if fell_at is not None else "OK"
            say(f"[g1_walk_canon] t={sim_ms/1000:5.1f}s tick={sim_ms//step_ms} "
                f"bz={bz:.3f} fwd={fwd:+.3f}m roll={roll:+.3f} pitch={pitch:+.3f} "
                f"phase={phase:.2f} |act|max={np.abs(action).max():.2f} {tag}\n")
            last_log_ms = sim_ms

    return 0


if __name__ == "__main__":
    sys.exit(main())
