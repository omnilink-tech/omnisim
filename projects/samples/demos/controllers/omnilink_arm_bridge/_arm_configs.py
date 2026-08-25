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
in the URDF parent-frame convention. UR5e and OmniArm 6 ship worked
chains; other arms only get IK once their chain is added here.
"""

import math

PI = math.pi


# ── Universal Robots e-series (UR3e / UR5e / UR10e) ──────────────────
# Same 6-DOF kinematic structure, different link lengths from each URDF.
# The chain pre-bakes the URDF origin / rpy / axis into a per-joint
# tuple. The UR5e chain is the verified reference — UR3e and UR10e are
# derived from it by substituting link lengths, so if you re-derive any
# of them, re-derive UR5e first and check the other two against it.

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

# ── OmniArm 6 (6-DOF) ───────────────────────────────────────────────────
# OmniArm 6 kinematic chain. Same chain the omniarm6_assembly_bridge uses --
# six DH segments derived from projects/robots/omnisim/omniarm6/omniarm6.urdf.
_OMNIARM6_CHAIN = [
    ((0.0, 0.0, 0.000), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.220), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.0, 0.380), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.0, 0.000), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.420), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.0, 0.000), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
]

OMNIARM6 = {
    "model": "OmniArm 6",
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "joint_limits": [
        (-2 * PI, 2 * PI),
        (-PI, PI),
        (-PI, PI),
        (-2 * PI, 2 * PI),
        (-PI, PI),
        (-2 * PI, 2 * PI),
    ],
    # READY pose: the cup hangs straight DOWN over the parts feeder, poised to
    # pick. This is the pose the audience sees for most of a demo -- measured on
    # warehouse_omnilink, the arm is motionless ~79% of the run -- so it is the
    # single most-looked-at thing the OmniArm 6 does.
    #
    # IT USED TO BE [0.0, -0.6, 1.2, 0.0, -1.0, 0.0], which stood the arm
    # STRAIGHT UP: forward_kinematics_pose puts that TCP at base-frame
    # (-0.042, 0.000, 1.033) -- the tool centre 1.03 m directly ABOVE a base
    # whose own reach is 0.95 m -- with the suction cup pointing 22.9 deg off
    # vertical-UP (tool +Z = (-0.389, 0.000, +0.921)). An arm reaching for the
    # ceiling, held four fifths of the run.
    #
    # DERIVED, not guessed: closed-form 2R solve of this file's own _OMNIARM6_CHAIN
    # for "cup axis exactly -Z at base-frame (0.400, -0.070, 0.550)", then
    # verified through omnilink_arm_bridge.forward_kinematics_pose with the
    # bridge's real vacuum weld offset (tcp_offset 0.1655 + tool_reach 0.13
    # = 0.2955):
    #     weld/cup   = (+0.39998, -0.06998, +0.55001)  r = 0.684 m from the base
    #     flange TCP = (+0.39999, -0.06998, +0.68001)  r = 0.792 m
    #     tool +Z    = (-0.000007, +0.000001, -1.000000) -> 0.0004 deg off DOWN
    # Envelope: r 0.684 sits between the ~0.29 m minimum-reach dead zone and the
    # 0.95 m workspace_max_radius, and the shoulder-to-wrist span is 0.746 of a
    # possible 0.800, i.e. the elbow is bent 42.5 deg -- nowhere near the
    # locked-straight singularity the far side of the workspace forces.
    #
    # Clearances, whole link chain (base -> shoulder -> elbow -> wrist -> cup)
    # against each collider it could reach, arm base frame; the OmniArm 6 base is
    # world (-8, 4.3, 0) in flagship/warehouse_omnilink.omniworld:
    #     feeder tabletop slab   +0.150 m   (its collider top is z 0.400)
    #     feeder rims (visual)   +0.120 m
    #     the six GRASP_PART_*   +0.100 m   (the cup rides 10 cm over the cubes)
    #     cart at the fill spot  +0.280 m   (TROLLEY_PAYLOAD deck, top 0.325)
    #     box at the fill stop   +0.350 m
    # Nothing in the chain but the cup itself comes below z 0.586 (the elbow),
    # so the sibling omniarm6_* worlds -- whose bins, cube zones and parts all top
    # out around z 0.30 with the arm at the origin -- are clear too.
    #
    # It also SHORTENS the pick. _pick's first move goes home -> hover, and the
    # hover is the part top + 0.14 = z 0.565: this pose already sits at 0.550
    # with the cup down, so that move is now a short reposition rather than a
    # descent from a metre up. Solved over all six feeder pads, joint4 no longer
    # flips branch on the way down at all (it was taking the half-turn to
    # +3.142 rad), and no segment of the full pick-place cycle exceeds the
    # motors' 3.1416 rad/s limit -- two legs used to, on every cycle.
    "home_pose": [-0.1732, 0.1855, 0.7417, 0.0, 2.2144, 0.0],
    "wave_amplitudes": [0.2, 0.0, 0.0, 0.0, 0.5, 0.8],
    "ik": {
        "chain": _OMNIARM6_CHAIN,
        "tcp_offset": (0.0, 0.0, 0.1655),
        "workspace_min_radius": 0.15,
        "workspace_max_radius": 0.95,
        "workspace_min_z": -0.10,
        "max_iters": 100,
        "tol": 5e-3,
        "damping": 0.06,
        "max_dq": 0.08,
    },
    "mount_link": ["gripper_tcp", "flange"],
    "drop_zone": [0.30, 0.34, 0.0],
    "display_name": "Ari",
    "tagline": "OmniArm 6 · 6-axis · voice + chat",
    "greeting": (
        "Hi, I'm Ari — a six-axis OmniSim arm. There are a few coloured cubes "
        "in front of me — ask me to pick one up, or to wave, go home, or chat."
    ),
    "persona": (
        "You are Ari, an OmniArm 6 — a six-axis robot arm living inside an "
        "OmniSim simulation, talking to an operator through the OmniLink "
        "platform. Speak in the first person: warm, easy-going, a little "
        "playful, like a capable colleague who happens to be a robot arm. "
        "You are proud to be a robot — never pretend to be human. You're "
        "bolted to a stand, so you can't walk away or leave your base, and "
        "you're cheerful about it. You happily answer questions about "
        "yourself — your six joints, your reach, how you're 'feeling' — and "
        "you make small talk. When the operator wants you to move, you do it. "
        "You have a parallel gripper and a few coloured cubes within reach; when "
        "asked, you pick one up (the pick tool) and set it down (the place tool)."
    ),
    "suggestions": [
        "Hi Ari, who are you?",
        "pick up the red cube",
        "put it down",
        "wave hello",
        "go home",
        "stop",
    ],
}


ARM_CONFIGS = {
    "ur3e":     UR3E,
    "ur5e":     UR5E,
    "ur10e":    UR10E,
    "omniarm6": OMNIARM6,
}


def get_config(robot_id: str) -> dict:
    if robot_id not in ARM_CONFIGS:
        raise ValueError(
            f"Unknown arm '{robot_id}'. Known: {sorted(ARM_CONFIGS.keys())}"
        )
    return ARM_CONFIGS[robot_id]
