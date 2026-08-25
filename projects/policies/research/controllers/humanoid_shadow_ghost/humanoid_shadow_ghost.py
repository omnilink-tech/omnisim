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

"""Generic kinematic SHADOW/GHOST controller for humanoids (H1, Valkyrie, ...).

Drives a physics-free humanoid (staticBase TRUE) with the PURE human-gait MODEL
reference (projects/policies/control/gait/<robot>_human_gait) -- no RL, no balance, no physics.
This is the "ghost first" preview: it shows the IDEAL walking motion the RL
policy will be trained to track. Because it ignores physics it can never fall;
it just walks the designed shadow across the floor so the motion can be judged
(natural? stable-looking? feet flat? CoM over the feet?) BEFORE any training.

One controller, many robots: the robot is chosen by HUMANOID_GHOST_ROBOT and its
per-robot joint-name map + gait module are selected below. GaitParams defaults
come straight from the robot's own gait module (already size-tuned); env knobs
(HG_*) override. Self-walk is the default (no real robot needed).
"""
from __future__ import annotations

import importlib
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

# Per-robot joint maps, in the gait's internal 13-slot LEG order
# (L: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll; then R; then
# waist) and 10-slot ARM order (L: sh_pitch, sh_roll, sh_yaw, elbow, wrist; R).
# None = the robot has no such joint (slot is skipped).
ROBOTS = {
    "h1": dict(
        gait="h1_human_gait",
        legs=["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
              "left_knee_joint", "left_ankle_joint", None,
              "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_joint", None,
              "torso_joint"],
        arms=["left_shoulder_pitch_joint", "left_shoulder_roll_joint",
              "left_shoulder_yaw_joint", "left_elbow_joint", None,
              "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
              "right_shoulder_yaw_joint", "right_elbow_joint", None],
        z_base=1.00,
    ),
}


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


_VISIT = []


def _ghostify(node, transparency, tint, depth=0):
    """Recursively make every Shape under `node` translucent + tinted."""
    if node is None or depth > 40:
        return 0
    n = 0
    try:
        type_name = node.getTypeName()
    except Exception:
        return 0
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
    which = os.environ.get("HUMANOID_GHOST_ROBOT", "h1").strip()
    if which not in ROBOTS:
        sys.stderr.write(f"[humanoid_ghost] unknown robot '{which}'\n")
        return 1
    cfg = ROBOTS[which]
    gait = importlib.import_module(f"projects.policies.control.gait.{cfg['gait']}")

    # GaitParams: the robot's own (size-tuned) defaults, env-overridable.
    d = gait.GaitParams()
    GP = gait.GaitParams(
        vx=_genv("HG_VX", d.vx),
        freq=_genv("HG_FREQ", d.freq),
        ramp_s=_genv("HG_RAMP_S", 2.0),
        style=os.environ.get("HG_STYLE", d.style),
        lateral=os.environ.get("HG_LATERAL", d.lateral),
        yaw=os.environ.get("HG_YAW", d.yaw),
        step_height=_genv("HG_STEP_H", d.step_height),
        pelvis_height=_genv("HG_PELVIS_H", d.pelvis_height),
    )
    Y_OFFSET = _genv("HUMANOID_GHOST_Y", 0.0)
    Z_BASE = _genv("HUMANOID_GHOST_Z", cfg["z_base"])

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    names = [n for n in (cfg["legs"] + cfg["arms"]) if n]
    motors = {}
    for jn in names:
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

    _glog_path = os.environ.get("HUMANOID_GHOST_LOG")
    _glog = open(_glog_path, "w", buffering=1) if _glog_path else None

    alpha = _genv("HUMANOID_GHOST_ALPHA", 0.45)
    if alpha > 0.0:
        tint_s = os.environ.get("HUMANOID_GHOST_TINT", "0.62,0.82,1.0")
        try:
            tint = tuple(float(x) for x in tint_s.split(","))
        except ValueError:
            tint = (0.62, 0.82, 1.0)
        n_shapes = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[humanoid_ghost:{which}] ghostified {n_shapes} shapes\n")

    sys.stderr.write(f"[humanoid_ghost:{which}] shadow running (gait={cfg['gait']}, "
                     f"style={GP.style}, lateral={GP.lateral}, yaw={GP.yaw}, "
                     f"vx={GP.vx}, freq={GP.freq})\n")
    sys.stderr.flush()

    _sim_ms = 0
    _last_log = -1000
    while robot.step(step_ms) != -1:
        _sim_ms += step_ms
        t = _sim_ms / 1000.0
        pos_x = GP.vx * t
        phase = (gait.DS_PHASE + 2.0 * math.pi * GP.freq * t) % (2.0 * math.pi)

        legs, arms, _ = gait.targets_np(phase, GP, t_since_start=t)
        for i, jn in enumerate(cfg["legs"]):
            if jn and jn in motors:
                motors[jn].setPosition(float(legs[i]))
        for k, jn in enumerate(cfg["arms"]):
            if jn and jn in motors:
                motors[jn].setPosition(float(arms[k]))

        if trans_field is not None:
            trans_field.setSFVec3f([pos_x, Y_OFFSET, Z_BASE])

        if _glog is not None and _sim_ms - _last_log >= 1000:
            _last_log = _sim_ms
            _glog.write(f"t={t:5.1f} x={pos_x:+.2f} phase={phase:.2f} "
                        f"L_hip={legs[0]:+.3f} L_knee={legs[3]:+.3f} "
                        f"L_ankle={legs[4]:+.3f} L_hipR={legs[1]:+.3f}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
