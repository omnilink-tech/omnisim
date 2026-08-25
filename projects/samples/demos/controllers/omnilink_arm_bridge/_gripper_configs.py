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
#
# ⚠ `physics_grasp` means the fingers are REAL and close on the part. It does
# NOT mean friction is what holds it: the bridge still layers a kinematic weld
# on top unless `assist_weld` is False, and the grasp response now says which
# of the two is holding (`hold_mechanism`). An earlier version of this comment
# claimed "contact friction holds it, no kinematic weld", which was wrong on
# both counts. Measured 2026-08-02: the honest friction-only hold of a 50 mm
# 0.2 kg block on this gripper lasts 1.376 s before slipping 15 mm.
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

# ⚠ THE KINEMATIC / MAGNET FLAVOUR: the bridge WELDS the part to the tool and
# this entry only decides how the jaws are POSED while it rides along. It is
# NOT the entry for a real friction grasp -- that is ROBOTIQ_2F140_GRIP below.
#
# ⚠⚠ REPAIRED 2026-08-11, AND THE OLD VALUES WERE A SILENT FAKE. This entry
# used to name a single motor `finger_joint` with targets in RADIANS
# ([0.0] -> [0.7], marked "# TODO calibrate"). NO URDF IN THIS TREE DEFINES A
# JOINT CALLED `finger_joint` -- not the OmniArm 6 variants, not the Robotiq xacro
# (which uses `${prefix}_finger` / `_finger_mirror`), nothing. So every motor
# lookup returned None, `_apply` skipped them all, and the gripper reported
# `holding: True` having moved nothing whatsoever. It was left alone in the
# previous pass "so anything already selecting robotiq_2f140 keeps its
# behaviour", but the behaviour being preserved was a lie, and this demo's
# entire premise is that a grasp is proved rather than asserted.
#
# It now names the SAME REAL PRISMATIC JOINTS as the force-grip entry, so the
# jaws actually pose. Note the unit change that came with it: those joints are
# prismatic and their q is a per-pad HALF-OPENING IN METRES, not an angle --
# 0.070 = fully open (140 mm stroke), 0.0 = pads meeting on the axis. The
# overclose trick (-0.030) is deliberately NOT copied here: it exists to
# saturate the effort cap for a physics squeeze, and there is no squeeze on a
# welded hold.
#
# `tool_reach` is also new here and is not cosmetic. Without it the bridge fell
# back to a default 0.13 m (omnilink_arm_bridge.py) -- a number derived for the
# much shorter 2F-85 -- against the 0.17645 m measured off this gripper's own
# CAD. That is ~46 mm of tool-length error on a 213 mm gripper, in the
# direction that drives the tool too deep.
ROBOTIQ_2F140 = {
    "model": "Robotiq 2F-140",
    "kind": "parallel",
    "motors": ["robotiq_2f140_finger", "robotiq_2f140_finger_mirror"],
    "open_q": [0.070, 0.070],   # metres, per-pad half-opening (NOT radians)
    "close_q": [0.0, 0.0],      # pads meeting; no overclose without physics
    "max_width": 0.140,         # 140 mm stroke (datasheet)
    "tool_reach": 0.17645,      # flange -> pad mid-height, measured off the CAD
    "flange_mount": {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
    "real_driver": "robotiq_2f",
}

# Physics-grasp 2F-140: two REAL mirrored prismatic jaws, in METRES, carrying
# the customer's CAD as their visual and a hand-authored box pad as their only
# collider. Needs an arm URDF that ships those joints --
# projects/robots/omnisim/omniarm6/omniarm6_2f140_grip.urdf is the reference, and
# omniarm6_2f140_pick_place is the demo that proves the hold by contact.
#
# open_q/close_q are per-pad HALF-openings, so q is literally the pad's inner
# face on the grasp axis (the URDF cancels the joint origin against the pad's
# half-thickness to make that exact). 0.070 = the 140 mm stroke fully open;
# -0.030 is a 30 mm overclose so a squeeze that stalls on the part saturates the
# 125 N effort cap instead of parking at zero error -- the same trick
# ROBOTIQ_2F85_GRIP uses.
#
# ⚠ tool_reach is flange -> PAD MID-HEIGHT (0.17645 m), MEASURED off the CAD:
# the pad's 14.32 cm2 inner face runs 0.1437..0.2092 above the mounting face.
# It is more than DOUBLE the 2F-85's 0.085 because the 2F-140 is a 213 mm long
# gripper, so an arm that swaps 2F-85 -> 2F-140 without updating this will drive
# the tool ~92 mm too deep. The demo's own tool point is the same number plus
# link6->flange: OZ = 0.1655 + 0.17645 = 0.34195.
ROBOTIQ_2F140_GRIP = {
    "model": "Robotiq 2F-140 (force-grip)",
    "kind": "parallel",
    "motors": ["robotiq_2f140_finger", "robotiq_2f140_finger_mirror"],
    "open_q": [0.070, 0.070],      # prismatic: 0.070 m = half of the 140 mm stroke
    "close_q": [-0.030, -0.030],   # overclose: squeeze to the 125 N effort cap
    "max_width": 0.140,
    "physics_grasp": True,
    # ⚠ assist_weld FALSE, AND IT IS THE WHOLE POINT OF THIS ENTRY.
    # act_grasp's default is to WELD the nearest DEF GRASP_* node to the tool
    # and teleport it to the TCP every tick. Its own source says why that is
    # not a grasp: the hold "would work with the fingers wide open, for ever",
    # so a demo built on it cannot demonstrate anything. `physics_grasp: True`
    # alone does NOT turn it off -- act_grasp welds on that path too unless
    # assist_weld is false, which is exactly the trap this flag exists for.
    #
    # With it false the fingers and contact friction are the only thing holding
    # the part. That is a real claim and it can fail, which is what makes it
    # worth making. It also makes a DEF GRASP_* name safe again: the weld
    # cannot engage, so a block can be NAMED for the bridge's target picker
    # without becoming fake-holdable.
    "assist_weld": False,
    "tool_reach": 0.17645,         # flange -> pad mid-height (measured, see above)
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
# NO `tool_reach` HERE ON PURPOSE -- and it is NOT an oversight. Read this
# before "fixing" it; the obvious fix was measured and is a regression.
#
# What happens today: no tool_reach -> the bridge takes its 0.13 default
# (omnilink_arm_bridge.py:987), so the weld point sits at OZ = tcp_offset
# 0.1655 + 0.13 = 0.2955 from link6. On the OmniArm 6's gum-pad rig the PAD FACE
# is at OZ 0.25 (omniarm6_gumgrip.urdf:215), and the held part's translation --
# i.e. its CENTRE -- is snapped to the weld point every tick, so a 0.05 m part
# rides with its top face 0.130 - 0.085 - 0.025 = 0.020 m BELOW the pad. A
# visible 20 mm air gap under the cup, on camera, every pick.
#
# WHY 0.085 (the pad face) IS NOT THE ANSWER. tool_reach means flange -> the
# point the PART'S CENTRE occupies when held -- that is what the two physics
# 2F85 entries above measure to (the finger throat), and what the weld does
# here. Setting it to the pad face would put the part's centre ON the face,
# i.e. the pad buried 25 mm inside the part for the whole carry: a worse
# artefact than the gap it removes. The geometrically right value for a 0.05 m
# part is 0.085 + 0.025 = 0.110, and it is PART-SIZE DEPENDENT, which a gripper
# config cannot know.
#
# AND IT MOVES THE ARM. `ArmIdleLoop._solve` aims the WELD POINT at the target,
# so shortening tool_reach drives the flange that much lower/further and picks
# a different elbow branch. Replayed offline through this repo's own
# dls_ik_pose / forward_kinematics_pose over warehouse_omnilink's six feeder
# pads x (hover, attach, lift) + carry + three box slots x (approach, release):
# base-frame radius and weld-to-part capture distance are unchanged, but the
# link2 collider's clearance to the feeder slab DEGRADES on every east-column
# pose -- at the attach pose C -0.082 -> -0.100 m and F -0.062 -> -0.082 m at
# tool_reach 0.085 (-0.090 / -0.071 at 0.110). That collider sweeping the tray
# is exactly what broke this demo in e1714736 and had to be reverted in
# a2b8331d, and the only configuration with live evidence (6 picks, 6 pads,
# zero no-grips) is the one below. Do not change it without a live re-verify.
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
    "robotiq_2f140_grip": ROBOTIQ_2F140_GRIP,
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
