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

"""Per-robot kinematic + motion-preset configs for omnilink_arm_bridge.

Adding a new arm: drop a new entry into ARM_CONFIGS. The bridge's
generic dispatch + intent router pick it up with no code changes.

Each config field:

    joint_names      ordered list of revolute joints driven by the bridge.
                     Names must match the URDF importer's motor names
                     (joint_name -> joint_name_motor for revolute joints).
    joint_limits     [(low, high)] in radians, one pair per joint_names.
    home_pose        rad per joint. The bridge sends this on init and on
                     "reset_to_home". Picked to keep the arm clear of the
                     base and floor.
    ready_pose       optional named pose -- "ready" / "tucked" alias.
    wave_amplitudes  optional rad-amplitudes per joint for the "wave"
                     preset. Bridge oscillates each joint at 0.8 Hz with
                     these amplitudes around home_pose for ~6 s.
    gripper_motors   optional list of gripper motor names. Bridge exposes
                     open_gripper / close_gripper if set.
    gripper_open_q   joint values applied on open_gripper.
    gripper_close_q  joint values applied on close_gripper.
    ik               optional. None means the bridge advertises position
                     control + presets only. A dict with keys
                     {chain, max_iters, tol, damping, max_dq} enables
                     damped-least-squares IK to a TCP target.

The DLS IK chain format (per-joint tuple): (offset_xyz, offset_rpy, axis_xyz)
in the URDF parent-frame convention. UR5e ships a worked chain;
other arms only get IK once their chain is added here.
"""

import math

PI = math.pi


# ── Universal Robots e-series (UR3e / UR5e / UR10e) ──────────────────
# Same 6-DOF kinematic structure, different link lengths from each URDF.
# The chain pre-bakes the URDF origin / rpy / axis into a per-joint
# tuple. UR5e chain is the verified one from ur5e_omnilink_bridge.

_UR5E_CHAIN = [
    ((0.0,  0.000, 0.163), (0.0, 0.0,      0.0), (0.0, 0.0, 1.0)),
    ((0.0,  0.138, 0.000), (0.0, 1.570796, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, -0.131, 0.425), (0.0, 0.0,      0.0), (0.0, 1.0, 0.0)),
    ((0.0,  0.000, 0.392), (0.0, 1.570796, 0.0), (0.0, 1.0, 0.0)),
    ((0.0,  0.127, 0.000), (0.0, 0.0,      0.0), (0.0, 0.0, 1.0)),
    ((0.0,  0.000, 0.100), (0.0, 0.0,      0.0), (0.0, 1.0, 0.0)),
]

_UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
_UR_LIMITS = [(-2 * PI, 2 * PI)] * 2 + [(-PI, PI)] + [(-2 * PI, 2 * PI)] * 3
_UR_HOME = [0.0, -1.0, 1.4, -1.2, -1.57, 0.0]
_UR_WAVE = [0.15, 0.0, 0.0, 0.0, 0.4, 0.7]

UR3E = {
    "model": "UR3e",
    "joint_names": _UR_JOINTS,
    "joint_limits": _UR_LIMITS,
    "home_pose": _UR_HOME,
    "wave_amplitudes": _UR_WAVE,
    # UR3e is smaller -- IK chain not pre-baked yet; presets only.
    "ik": None,
}

UR5E = {
    "model": "UR5e",
    "joint_names": _UR_JOINTS,
    "joint_limits": _UR_LIMITS,
    "home_pose": _UR_HOME,
    "wave_amplitudes": _UR_WAVE,
    "ik": {
        "chain": _UR5E_CHAIN,
        "tcp_offset": (0.0, 0.0, 0.0),
        "workspace_min_radius": 0.15,
        "workspace_max_radius": 0.82,
        "workspace_min_z": 0.05,
        "max_iters": 80,
        "tol": 5e-3,
        "damping": 0.08,
        "max_dq": 0.08,
    },
}

UR10E = {
    "model": "UR10e",
    "joint_names": _UR_JOINTS,
    "joint_limits": _UR_LIMITS,
    "home_pose": _UR_HOME,
    "wave_amplitudes": _UR_WAVE,
    "ik": None,
}

# ── Franka Emika Panda (7-DOF + 2-finger gripper) ────────────────────
PANDA = {
    "model": "Panda",
    "joint_names": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ],
    "joint_limits": [
        (-2.8973, 2.8973),
        (-1.7628, 1.7628),
        (-2.8973, 2.8973),
        (-3.0718, -0.0698),
        (-2.8973, 2.8973),
        (-0.0175, 3.7525),
        (-2.8973, 2.8973),
    ],
    "home_pose": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "wave_amplitudes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6],
    "gripper_motors": ["panda_finger_joint1", "panda_finger_joint2"],
    "gripper_open_q": [0.04, 0.04],
    "gripper_close_q": [0.0, 0.0],
    "ik": None,
}

ARM_CONFIGS = {
    "ur3e":  UR3E,
    "ur5e":  UR5E,
    "ur10e": UR10E,
    "panda": PANDA,
}


# An arm whose robot package is held from distribution keeps its config in a
# sibling module that is not part of the public snapshot. Register it when the
# module is present; otherwise the bridge simply does not offer that arm.
try:
    from _arm_configs_held import HELD_ARMS
except ImportError:
    HELD_ARMS = {}
ARM_CONFIGS.update(HELD_ARMS)


def get_config(robot_id: str) -> dict:
    if robot_id not in ARM_CONFIGS:
        raise ValueError(
            f"Unknown arm '{robot_id}'. Known: {sorted(ARM_CONFIGS.keys())}"
        )
    return ARM_CONFIGS[robot_id]
