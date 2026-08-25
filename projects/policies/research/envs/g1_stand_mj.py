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

"""G1 standing-baseline in pure MuJoCo (no OmniSim, no mujoco_warp).

Loads the G1 legs URDF (`g1_legs_mjcf.urdf` — the legs-only variant with
the `<mujoco>` compiler block stripped so MuJoCo's meshdir doesn't
double-prefix), spawns the robot at the spec's NOMINAL_POSE, and runs
the same NOMINAL + ankle-balance PD that `g1_model_walk.py` uses on the
OmniSim side. Reports per-second pelvis roll/pitch/bz so you can see
whether the analytic layer holds the robot upright.

This is the prerequisite check for any residual RL: if the analytic
baseline can't stand, residual RL on top won't either.

Usage:
    python projects/policies/research/envs/g1_stand_mj.py [--seconds 10] [--no-balance]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from projects.policies.research.backends.g1_robot_spec import (  # noqa: E402
    JOINT_NAMES, NOMINAL_POSE,
)


# Joints actually present in the legs-only URDF (13 of the spec's 23).
LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
NOMINAL_BY_JOINT = dict(zip(JOINT_NAMES, NOMINAL_POSE))
NOMINAL_LEGS = np.array([NOMINAL_BY_JOINT[j] for j in LEGS_JOINTS], dtype=np.float32)


def _quat_to_rpy(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    """ZYX intrinsic -> roll (x), pitch (y), yaw (z)."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="Sim duration in seconds (default 10).")
    parser.add_argument("--no-balance", action="store_true",
                        help="Disable active ankle balance PD.")
    parser.add_argument("--spawn-z", type=float, default=0.78,
                        help="Initial pelvis z (default 0.78, matches OmniSim).")
    # Whole-body balance: ankle catches small disturbances; hip catches
    # larger ones. OmniQuad's balance PD does the equivalent through leg-z
    # offsets driving IK; G1 hits the same effect directly via hip joints.
    parser.add_argument("--kp-ankle-pitch", type=float, default=4.0)
    parser.add_argument("--kd-ankle-pitch", type=float, default=0.4)
    parser.add_argument("--kp-ankle-roll", type=float, default=4.0)
    parser.add_argument("--kd-ankle-roll", type=float, default=0.4)
    parser.add_argument("--kp-hip-pitch", type=float, default=6.0)
    parser.add_argument("--kd-hip-pitch", type=float, default=0.6)
    parser.add_argument("--kp-hip-roll", type=float, default=4.0)
    parser.add_argument("--kd-hip-roll", type=float, default=0.4)
    parser.add_argument("--bal-clamp", type=float, default=0.5)
    parser.add_argument("--actuator-kp", type=float, default=400.0)
    parser.add_argument("--actuator-kv", type=float, default=20.0)
    args = parser.parse_args()

    from projects.policies.research.backends._urdf_to_mjcf import load_or_convert
    urdf = REPO / "projects/robots/unitree/g1/urdf/g1_legs_mjcf.urdf"
    if not urdf.exists():
        print(f"ERROR: {urdf} not found. Run patch_g1_urdf_for_rl.py + strip the "
              f"<mujoco> block first.", file=sys.stderr)
        return 1

    # Stiff legs (the spec uses 200-300 kp for legs); arm/waist joints absent here.
    model = load_or_convert(urdf, actuator_joints=list(LEGS_JOINTS),
                            kp=args.actuator_kp, kv=args.actuator_kv)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    # Spawn at NOMINAL_POSE with pelvis at spawn_z.
    data.qpos[0:3] = (0.0, 0.0, args.spawn_z)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)  # unit quaternion, identity rotation
    # qpos[7..] are the 13 actuated joints in the order MuJoCo discovered them.
    # Build the map from joint name -> qpos index.
    qpos_idx = {}
    for jname in LEGS_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            print(f"ERROR: joint {jname} not in MJCF", file=sys.stderr)
            return 1
        qpos_idx[jname] = model.jnt_qposadr[jid]
    for jname, target in zip(LEGS_JOINTS, NOMINAL_LEGS):
        data.qpos[qpos_idx[jname]] = float(target)
    mujoco.mj_forward(model, data)

    # Actuator targets default to nominal pose.
    ctrl_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                  for i in range(model.nu)]
    print(f"[g1_stand_mj] {model.nu} actuators: {ctrl_names}")
    # MuJoCo's load_or_convert names actuators "<joint>_act" or similar.
    # Map our joint order to ctrl indices via name match.
    ctrl_map = {}
    for i, n in enumerate(ctrl_names):
        for jname in LEGS_JOINTS:
            if jname in n:
                ctrl_map[jname] = i
                break
    missing = [j for j in LEGS_JOINTS if j not in ctrl_map]
    if missing:
        print(f"WARNING: actuators not found for {missing}", file=sys.stderr)

    ankle_pitch_l = ctrl_map.get("left_ankle_pitch_joint")
    ankle_pitch_r = ctrl_map.get("right_ankle_pitch_joint")
    ankle_roll_l = ctrl_map.get("left_ankle_roll_joint")
    ankle_roll_r = ctrl_map.get("right_ankle_roll_joint")
    hip_pitch_l = ctrl_map.get("left_hip_pitch_joint")
    hip_pitch_r = ctrl_map.get("right_hip_pitch_joint")
    hip_roll_l = ctrl_map.get("left_hip_roll_joint")
    hip_roll_r = ctrl_map.get("right_hip_roll_joint")

    sim_dt = float(model.opt.timestep)
    total_steps = int(args.seconds / sim_dt)
    print(f"[g1_stand_mj] sim_dt={sim_dt}s, running {total_steps} steps "
          f"({args.seconds}s) balance={'OFF' if args.no_balance else 'ON'}")

    survived = 0
    fell_at = None
    last_log_step = 0
    log_every = max(1, int(0.5 / sim_dt))  # ~2 Hz log

    prev_roll = 0.0
    prev_pitch = 0.0

    for step in range(total_steps):
        # Read pelvis pose from qpos.
        bx, by, bz = data.qpos[0], data.qpos[1], data.qpos[2]
        qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
        roll, pitch, yaw = _quat_to_rpy(qw, qx, qy, qz)
        roll_rate = (roll - prev_roll) / sim_dt
        pitch_rate = (pitch - prev_pitch) / sim_dt
        prev_roll, prev_pitch = roll, pitch

        # Set ctrl to nominal pose.
        for jname in LEGS_JOINTS:
            ci = ctrl_map.get(jname)
            if ci is not None:
                data.ctrl[ci] = NOMINAL_BY_JOINT[jname]

        # Whole-body balance: pitch error → ankle pitch + hip pitch
        # corrections; roll error → ankle roll + hip roll corrections.
        # Hip and ankle corrections are SAME SIGN so the whole leg leans
        # back toward upright (OmniQuad does this through IK; G1 has direct
        # hip access so we apply the leg-tilt directly).
        if not args.no_balance:
            # Pitch (forward/back lean).
            ap_corr = args.kp_ankle_pitch * pitch + args.kd_ankle_pitch * pitch_rate
            ap_corr = max(-args.bal_clamp, min(args.bal_clamp, ap_corr))
            hp_corr = args.kp_hip_pitch * pitch + args.kd_hip_pitch * pitch_rate
            hp_corr = max(-args.bal_clamp, min(args.bal_clamp, hp_corr))
            # Roll (sideways lean).
            ar_corr = args.kp_ankle_roll * roll + args.kd_ankle_roll * roll_rate
            ar_corr = max(-args.bal_clamp, min(args.bal_clamp, ar_corr))
            hr_corr = args.kp_hip_roll * roll + args.kd_hip_roll * roll_rate
            hr_corr = max(-args.bal_clamp, min(args.bal_clamp, hr_corr))

            if ankle_pitch_l is not None:
                data.ctrl[ankle_pitch_l] += ap_corr
            if ankle_pitch_r is not None:
                data.ctrl[ankle_pitch_r] += ap_corr
            if ankle_roll_l is not None:
                data.ctrl[ankle_roll_l] += ar_corr
            if ankle_roll_r is not None:
                data.ctrl[ankle_roll_r] += ar_corr
            # Hip pitch: positive pelvis pitch (nose-up) → BOTH thighs
            # rotate forward (positive hip_pitch in URDF convention) to
            # pull the torso forward toward upright.
            if hip_pitch_l is not None:
                data.ctrl[hip_pitch_l] += hp_corr
            if hip_pitch_r is not None:
                data.ctrl[hip_pitch_r] += hp_corr
            # Hip roll: positive roll (right-side down) → tilt both
            # thighs to push pelvis back to level. Left + right have
            # opposite URDF sign conventions so we apply opposite signs.
            if hip_roll_l is not None:
                data.ctrl[hip_roll_l] += hr_corr
            if hip_roll_r is not None:
                data.ctrl[hip_roll_r] -= hr_corr

        mujoco.mj_step(model, data)

        upright = (bz > 0.45 and abs(roll) < 0.6 and abs(pitch) < 0.6)
        if upright:
            survived += 1
        elif fell_at is None:
            fell_at = step * sim_dt

        if step - last_log_step >= log_every:
            tag = "OK" if upright else f"FALL@{fell_at:.2f}s"
            print(f"[g1_stand_mj] t={step*sim_dt:5.2f}s "
                  f"bx={bx:+.2f} bz={bz:.3f} "
                  f"roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f}  {tag}")
            last_log_step = step

    survival_frac = survived / total_steps
    print(f"\n[g1_stand_mj] DONE: survived {survived}/{total_steps} steps "
          f"({survival_frac*100:.1f}%); first fall at "
          f"{fell_at if fell_at else 'never'} s")
    return 0 if survival_frac > 0.95 else 1


if __name__ == "__main__":
    sys.exit(main())
