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

"""Gripper registry for omnilink_arm_bridge.

A gripper is decoupled from the arm: pick one at runtime with
`--gripper <id>`. The bridge wraps the chosen config in a
`GripperEffector` (see `gripper_effectors.py`) and exposes a uniform
surface no matter which gripper it is.

Adding a new gripper: drop an entry into GRIPPER_CONFIGS. No bridge code
changes -- exactly like adding an arm to ARM_CONFIGS.

Each config field:

    model        human label reported in /capabilities and chat.
    kind         family the effector dispatches on:
                 "parallel" | "angular" | "vacuum" | "magnetic".
    motors       sim motor device names (parallel/angular). Resolved as
                 `<name>_motor` then `<name>`, matching the URDF importer.
    open_q       per-motor joint targets, fully open.
    close_q      per-motor joint targets, fully closed.
    max_width    full-open opening in metres (enables width control).
    modes        (angular only) named grip modes -> per-motor joint lists.
    device       (vacuum/magnetic) Webots device name.
    flange_mount offset {xyz, rpy} for attaching the gripper Solid to the
                 arm's TCP frame (used by the world-assembly step, Phase 5).
    real_driver  key into the real-robot driver registry (Phase 4). The
                 sim bridge ignores this; the hardware bridge routes on it.

Width <-> motor mapping for parallel/angular grippers is linear between
`open_q` (width = max_width) and `close_q` (width = 0). Joint values and
strokes below are seeded from datasheets and marked where they still need
on-hardware calibration.
"""

from __future__ import annotations

from typing import Any, Dict


# ── Robotiq 2F-85 / 2F-140 (parallel, Modbus URCap) ──────────────────
# Single actuated finger joint per side in the Webots model; the URCap
# register protocol drives them together on the real hardware.
ROBOTIQ_2F85 = {
    "model": "Robotiq 2F-85",
    "kind": "parallel",
    "motors": ["finger_joint"],
    "open_q": [0.0],
    "close_q": [0.8],          # rad at full close  # TODO calibrate
    "max_width": 0.085,        # 85 mm stroke (datasheet)
    "flange_mount": {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
    "tool_reach": 0.14,        # m from flange to fingertips (grasp point)
    "grasp_radius": 0.16,      # kinematic-attach magnet radius (slack absorbs
                               # the bridge IK chain's ~12 cm pose error)
    "real_driver": "robotiq_2f",
}

# Physics-grasp variant: real prismatic fingers (needs an arm URDF whose
# gripper ships real finger joints).
# Drives the two finger motors to close on the part; contact friction
# holds it, no kinematic weld. `physics_grasp` tells the bridge to skip
# the magnet attach.
ROBOTIQ_2F85_PHYS = {
    "model": "Robotiq 2F-85 (physics)",
    "kind": "parallel",
    "motors": ["robotiq_2f85_finger", "robotiq_2f85_finger_mirror"],
    "open_q": [0.0425, 0.0425],   # prismatic: 0.0425 m = open per pad
    "close_q": [0.0, 0.0],        # 0 = closed; motor squeezes to grip
    "max_width": 0.085,
    "physics_grasp": True,
    # Flange -> finger-throat along the tool axis (the gripper base sits at the
    # flange, finger box centred ~0.085 above it). act_pick uses
    # tcp_offset_z + tool_reach to put the THROAT (not the flange) on the cube,
    # matching the validated pick-place tool point OZ=0.25.
    "tool_reach": 0.085,
    "real_driver": "robotiq_2f",
}

# Force-control variant of the physics 2F-85: the close target is driven 30 mm
# PAST contact (close_q < 0) so when the pads stall on a thin wall the position
# error saturates the actuator at its 50 N effort cap -- a faithful squeeze, the
# way a real force-controlled gripper grips. (A plain position servo on a thin
# wall develops almost no force: error ~3 mm x ke gives only a few N.) Needs the
# finger joints' lower limit extended to -0.03 in the arm URDF.
ROBOTIQ_2F85_GRIP = {
    "model": "Robotiq 2F-85 (force-grip)",
    "kind": "parallel",
    "motors": ["robotiq_2f85_finger", "robotiq_2f85_finger_mirror"],
    "open_q": [0.0425, 0.0425],
    "close_q": [-0.030, -0.030],   # overclose: squeeze to the effort cap
    "max_width": 0.085,
    "physics_grasp": True,
    "tool_reach": 0.085,           # flange -> finger throat (see _phys above)
    "real_driver": "robotiq_2f",
}

ROBOTIQ_2F140 = {
    "model": "Robotiq 2F-140",
    "kind": "parallel",
    "motors": ["finger_joint"],
    "open_q": [0.0],
    "close_q": [0.7],          # TODO calibrate
    "max_width": 0.140,        # 140 mm stroke (datasheet)
    "flange_mount": {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
    "real_driver": "robotiq_2f",
}

# ── Robotiq 3F (adaptive 3-finger, Modbus URCap) ─────────────────────
# Three fingers; here modelled with one representative joint per finger.
ROBOTIQ_3F = {
    "model": "Robotiq 3F",
    "kind": "angular",
    "motors": ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"],
    "open_q": [0.05, 0.05, 0.05],
    "close_q": [1.2, 1.2, 1.2],     # TODO calibrate
    "max_width": 0.155,             # ~155 mm grip span (datasheet)
    "default_mode": "basic",
    "modes": {
        "basic": [0.6, 0.6, 0.6],   # encompassing power grip
        "pinch": [0.9, 0.9, 0.3],   # fingertip pinch
        "wide":  [0.3, 0.3, 0.3],   # spread, large objects
    },
    "flange_mount": {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
    "real_driver": "robotiq_3f",
}

# ── Franka Panda 2-finger hand (parallel) ────────────────────────────
# The hand that ships inside the Panda URDF. Mirrors the legacy inline
# fields on the panda arm config so `--gripper panda_hand` and the
# default both resolve to identical motor targets.
PANDA_HAND = {
    "model": "Panda Hand",
    "kind": "parallel",
    "motors": ["panda_finger_joint1", "panda_finger_joint2"],
    "open_q": [0.04, 0.04],
    "close_q": [0.0, 0.0],
    "max_width": 0.08,         # 2 x 40 mm finger travel
    "real_driver": "franka_hand",
}

# ── OnRobot RG2 / RG6 (parallel, Compute Box / Modbus) ───────────────
ONROBOT_RG2 = {
    "model": "OnRobot RG2",
    "kind": "parallel",
    "motors": ["rg2_finger_joint"],
    "open_q": [0.0],
    "close_q": [0.6],          # TODO calibrate
    "max_width": 0.110,        # 110 mm stroke (datasheet)
    "real_driver": "onrobot_rg",
}

ONROBOT_RG6 = {
    "model": "OnRobot RG6",
    "kind": "parallel",
    "motors": ["rg6_finger_joint"],
    "open_q": [0.0],
    "close_q": [0.6],          # TODO calibrate
    "max_width": 0.160,        # 160 mm stroke (datasheet)
    "real_driver": "onrobot_rg",
}

# ── Schunk EGK parallel gripper ──────────────────────────────────────
SCHUNK_EGK40 = {
    "model": "Schunk EGK 40",
    "kind": "parallel",
    "motors": ["egk_finger_joint"],
    "open_q": [0.0],
    "close_q": [0.5],          # TODO calibrate
    "max_width": 0.084,        # 84 mm stroke (datasheet)
    "real_driver": "schunk_egx",
}

# ── Vacuum / suction (Webots VacuumGripper device) ───────────────────
VACUUM = {
    "model": "Vacuum Suction",
    "kind": "vacuum",
    "device": "vacuum gripper",
    "real_driver": "vacuum_io",
}

# ── Magnetic coupling (Webots Connector device) ──────────────────────
MAGNETIC = {
    "model": "Magnetic Coupler",
    "kind": "magnetic",
    "device": "connector",
    "real_driver": "magnetic_io",
}


GRIPPER_CONFIGS: Dict[str, Dict[str, Any]] = {
    "robotiq_2f85":  ROBOTIQ_2F85,
    "robotiq_2f85_phys": ROBOTIQ_2F85_PHYS,
    "robotiq_2f85_grip": ROBOTIQ_2F85_GRIP,
    "robotiq_2f140": ROBOTIQ_2F140,
    "robotiq_3f":    ROBOTIQ_3F,
    "panda_hand":    PANDA_HAND,
    "onrobot_rg2":   ONROBOT_RG2,
    "onrobot_rg6":   ONROBOT_RG6,
    "schunk_egk40":  SCHUNK_EGK40,
    "vacuum":        VACUUM,
    "magnetic":      MAGNETIC,
}


def get_gripper_config(gripper_id: str) -> Dict[str, Any]:
    if gripper_id not in GRIPPER_CONFIGS:
        raise ValueError(
            f"Unknown gripper '{gripper_id}'. "
            f"Known: {sorted(GRIPPER_CONFIGS.keys())}"
        )
    return GRIPPER_CONFIGS[gripper_id]


def legacy_gripper_config(arm_cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Build a parallel-gripper config from the deprecated inline fields
    on an arm config (`gripper_motors` / `gripper_open_q` /
    `gripper_close_q`). Keeps arms that predate the registry working with
    no `--gripper` flag."""
    open_q = list(arm_cfg.get("gripper_open_q") or [])
    close_q = list(arm_cfg.get("gripper_close_q") or [])
    return {
        "model": f"{arm_cfg.get('model', 'arm')} gripper",
        "kind": "parallel",
        "motors": list(arm_cfg.get("gripper_motors") or []),
        "open_q": open_q,
        "close_q": close_q,
        # Best-effort: assume the open target spans the full stroke.
        "max_width": float(max(open_q) if open_q else 0.0),
        "real_driver": None,
    }
