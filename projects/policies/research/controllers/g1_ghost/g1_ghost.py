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

"""Kinematic GHOST controller -- "the ideal we optimise toward".

Drives a SECOND, physics-free G1 (staticBase TRUE) with the PURE human-gait
MODEL reference (projects/policies/control/gait/g1_human_gait): no RL, no balance, no
physics. The ghost stands beside the real robot (DEF G1_REAL) and its leg
cycle is locked to the real robot's FORWARD PROGRESS, so the two stay in
step: wherever the real robot is, the ghost shows the ideal pose it is
tracking. The visible gap between them IS the RL correction + sim2deploy
error.

Reads the same G1_GAIT_* env as the real controller so model params match.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.gait import g1_human_gait as ghg  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


_VISIT_LOG = []


def _ghostify(node, transparency, tint, depth=0):
    """Recursively make every Shape under `node` translucent + tinted, so the
    ghost reads as a hologram rather than a second solid robot. Walks the
    scene tree via supervisor field access (children / endPoint); sets
    PBRAppearance.transparency and baseColor in place. Returns #shapes hit."""
    if node is None or depth > 40:
        return 0
    n = 0
    try:
        type_name = node.getTypeName()
    except Exception:
        return 0
    _VISIT_LOG.append("  " * depth + type_name)
    if type_name == "Shape":
        try:
            app = node.getField("appearance").getSFNode()
            if app is not None:
                tf = app.getField("transparency")
                if tf is not None:
                    tf.setSFFloat(transparency)
                if tint is not None:
                    cf = app.getField("baseColor")
                    if cf is not None:
                        cf.setSFColor(list(tint))
                n += 1
        except Exception:
            pass
        return n
    for fname in ("children", "endPoint", "device"):
        fields = []
        for getter in ("getField", "getProtoField"):
            try:
                f = getattr(node, getter)(fname)
                if f is not None:
                    fields.append(f)
            except Exception:
                pass
        for f in fields:
            # getCount() returns -1 (not an exception) on SF fields.
            try:
                cnt = f.getCount()
            except Exception:
                cnt = -1
            if cnt is not None and cnt >= 0:
                for i in range(cnt):
                    n += _ghostify(f.getMFNode(i), transparency, tint, depth + 1)
            else:
                try:
                    n += _ghostify(f.getSFNode(), transparency, tint, depth + 1)
                except Exception:
                    pass
    return n


def main() -> int:
    GP = ghg.GaitParams(
        vx=_genv("G1_GAIT_VX", 0.4),
        freq=_genv("G1_GAIT_FREQ", 1.3),
        sway=_genv("G1_GAIT_A_LAT", 0.05),
        arm_swing=_genv("G1_GAIT_A_ARM", 0.25),
        ramp_s=_genv("G1_GAIT_RAMP_S", 2.0),
        style=os.environ.get("G1_GAIT_STYLE", "ik"),
        winter_hip_scale=_genv("G1_GAIT_HIP_SCALE", 0.75),
        # ── improved-shadow modes (A LIPM / B achieved / C human-3D) ──
        lateral=os.environ.get("G1_GAIT_LATERAL", "sway"),
        yaw=os.environ.get("G1_GAIT_YAW", "none"),
        lat_hip_amp=_genv("G1_GAIT_LAT_HIP_AMP", 0.09),
        step_width=_genv("G1_GAIT_STEP_WIDTH", 0.12),
    )
    Y_OFFSET = _genv("G1_GHOST_Y", 1.1)      # sideways offset from the real robot
    Z_BASE = _genv("G1_GHOST_Z", 0.78)

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    motors = {}
    for jn in LEGS_JOINTS + ARM_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m

    self_node = robot.getSelf()
    try:
        trans_field = self_node.getField("translation")
        rot_field = self_node.getField("rotation")
    except Exception:
        trans_field = rot_field = None
    if rot_field is not None:
        rot_field.setSFRotation([0, 0, 1, 0])    # upright, facing +x

    # The real robot, to lock the ghost's phase to its forward progress.
    real = None
    try:
        real = robot.getFromDef("G1_REAL")
    except Exception:
        real = None
    # SELF-WALK: with no real robot (or G1_GHOST_SELF_WALK=1) the ghost walks
    # on its OWN clock and translates forward at vx -> shows the pure shadow
    # walking across the floor (it is physics-free, so it never falls).
    self_walk = (real is None) or os.environ.get("G1_GHOST_SELF_WALK", "") == "1"
    x_start = None

    _glog_path = os.environ.get("G1_GHOST_LOG")
    _glog = open(_glog_path, "w", buffering=1) if _glog_path else None

    # Hologram look: translucent + pale tint (G1_GHOST_ALPHA=0 -> solid).
    alpha = _genv("G1_GHOST_ALPHA", 0.6)
    if alpha > 0.0:
        tint_s = os.environ.get("G1_GHOST_TINT", "0.62,0.82,1.0")
        try:
            tint = tuple(float(x) for x in tint_s.split(","))
        except ValueError:
            tint = (0.62, 0.82, 1.0)
        n_shapes = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[g1_ghost] ghostified {n_shapes} shapes "
                         f"(transparency={alpha})\n")
        if _glog is not None:
            _glog.write(f"ghostified={n_shapes} alpha={alpha}\n")
            if os.environ.get("G1_GHOST_DEBUG_TREE"):
                for line in _VISIT_LOG:
                    _glog.write(f"TREE {line}\n")

    sys.stderr.write("[g1_ghost] kinematic gait-model ghost running "
                     f"(style={GP.style}, y_offset={Y_OFFSET})\n")
    sys.stderr.flush()
    _sim_ms = 0
    _last_log = -1000

    while robot.step(step_ms) != -1:
        _sim_ms += step_ms
        if self_walk:
            # Walk on the ghost's own clock; translate forward at vx.
            t_equiv = _sim_ms / 1000.0
            pos_x = GP.vx * t_equiv
        else:
            # Lock phase + stride ramp to the real robot's forward progress.
            real_x = 0.0
            if real is not None:
                try:
                    real_x = float(real.getPosition()[0])
                except Exception:
                    real_x = 0.0
            if x_start is None:
                x_start = real_x
            t_equiv = max(0.0, real_x - x_start) / max(GP.vx, 1e-3)
            pos_x = real_x
        phase = (ghg.DS_PHASE + 2.0 * math.pi * GP.freq * t_equiv) % (2.0 * math.pi)

        legs, arms, _ = ghg.targets_np(phase, GP, t_since_start=t_equiv)
        for i, jn in enumerate(LEGS_JOINTS):
            m = motors.get(jn)
            if m is not None:
                m.setPosition(float(legs[i]))
        for k, jn in enumerate(ARM_JOINTS):
            m = motors.get(jn)
            if m is not None:
                m.setPosition(float(arms[k]))

        # Position the ghost (self-walk: forward at vx; else beside the robot).
        if trans_field is not None:
            trans_field.setSFVec3f([pos_x, Y_OFFSET, Z_BASE])

        if _glog is not None and _sim_ms - _last_log >= 1000:
            _last_log = _sim_ms
            _glog.write(f"t={_sim_ms/1000:5.1f} x={pos_x:+.2f} "
                        f"phase={phase:.2f} L_knee={legs[3]:+.3f} "
                        f"L_hip={legs[0]:+.3f} L_ankle={legs[4]:+.3f} "
                        f"L_hipR={legs[1]:+.3f} L_hipY={legs[2]:+.3f}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
