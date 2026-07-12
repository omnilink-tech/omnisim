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

"""Kinematic GHOST controller for the Unitree Go2 -- "the ideal we track".

The Spot/G1 ghost ported to Go2: drives a SECOND, physics-free Go2
(visual-only URDF, staticBase TRUE) with the PURE trot model reference
(projects/policies/control/gait/go2_trot_gait) -- no RL, no physics. The ghost trots beside
the real robot (DEF GO2_REAL) with its leg cycle locked to the real robot's
FORWARD PROGRESS; the visible gap between them IS the RL correction +
tracking error.

Reads the same GO2_GAIT_* env as the real controller so the model params
match. Hologram look via GO2_GHOST_ALPHA / GO2_GHOST_TINT. With no real robot
(or GO2_GHOST_SELF_WALK=1) it trots forward across the floor on its own clock.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.gait import go2_trot_gait as stg  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JOINT_ORDER = [f"{leg}_{part}_joint" for leg in LEGS for part in PARTS]


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


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
                    n += _ghostify(f.getMFNode(i), transparency, tint,
                                   depth + 1)
            else:
                try:
                    n += _ghostify(f.getSFNode(), transparency, tint,
                                   depth + 1)
                except Exception:
                    pass
    return n


def main() -> int:
    gp = stg.GaitParams(
        vx=_genv("GO2_GAIT_VX", 0.4),
        freq=_genv("GO2_GAIT_FREQ", 1.8),
        duty=_genv("GO2_GAIT_DUTY", 0.6),
        step_height=_genv("GO2_GAIT_STEP_H", 0.05),
        body_height=_genv("GO2_GAIT_BODY_H", 0.30),
        x0=_genv("GO2_GAIT_X0", 0.0),
        ramp_s=_genv("GO2_GAIT_RAMP_S", 1.0))
    Y_OFFSET = _genv("GO2_GHOST_Y", 0.7)
    Z_BASE = _genv("GO2_GHOST_Z", gp.body_height + 0.022)  # match real body z
    nominal = stg.standing_pose(gp)

    # WALK<->STOP schedule (mirrors go2_walk_deploy.py so the ghost stops
    # exactly when the real robot does, holding the standing pose).
    WALK_FOR_S = _genv("GO2_WALK_FOR_S", 0.0)
    STAND_FOR_S = _genv("GO2_STAND_FOR_S", 5.0)
    STAND_AT_S = _genv("GO2_STAND_AT_S", 0.0)
    MODE_BLEND_S = _genv("GO2_MODE_BLEND_S", 1.0)
    SETTLE_S = _genv("GO2_SETTLE_S", 1.5)

    def _want_stand(elapsed):
        if WALK_FOR_S > 0.0:
            period = WALK_FOR_S + STAND_FOR_S
            return (elapsed % period) >= WALK_FOR_S
        if STAND_AT_S > 0.0:
            return STAND_AT_S <= elapsed < (STAND_AT_S + STAND_FOR_S)
        return False

    _w = 1.0

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    motors = {}
    for jn in JOINT_ORDER:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m
    sys.stderr.write(f"[go2_ghost] {len(motors)}/12 motors found\n")

    self_node = robot.getSelf()
    try:
        trans_field = self_node.getField("translation")
        rot_field = self_node.getField("rotation")
    except Exception:
        trans_field = rot_field = None
    if rot_field is not None:
        rot_field.setSFRotation([0, 0, 1, 0])     # upright, facing +x

    real = None
    try:
        real = robot.getFromDef("GO2_REAL")
    except Exception:
        real = None
    self_walk = (real is None) or os.environ.get("GO2_GHOST_SELF_WALK", "") == "1"
    x_start = None

    _glog_path = os.environ.get("GO2_GHOST_LOG")
    _glog = open(_glog_path, "w", buffering=1) if _glog_path else None

    alpha = _genv("GO2_GHOST_ALPHA", 0.6)
    if alpha > 0.0:
        tint_s = os.environ.get("GO2_GHOST_TINT", "0.62,0.82,1.0")
        try:
            tint = tuple(float(x) for x in tint_s.split(","))
        except ValueError:
            tint = (0.62, 0.82, 1.0)
        n_shapes = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[go2_ghost] ghostified {n_shapes} shapes "
                         f"(transparency={alpha})\n")

    sys.stderr.write(f"[go2_ghost] kinematic trot-model ghost running "
                     f"(y_offset={Y_OFFSET}, self_walk={self_walk})\n")
    sys.stderr.flush()
    _sim_ms = 0
    _last_log = -1000

    while robot.step(step_ms) != -1:
        _sim_ms += step_ms
        if self_walk:
            t_equiv = _sim_ms / 1000.0
            pos_x = gp.vx * t_equiv
        else:
            real_x = 0.0
            if real is not None:
                try:
                    real_x = float(real.getPosition()[0])
                except Exception:
                    real_x = 0.0
            if x_start is None:
                x_start = real_x
            t_equiv = max(0.0, real_x - x_start) / max(gp.vx, 1e-3)
            pos_x = real_x
        phase = stg.QS_PHASE + 2.0 * math.pi * gp.freq * t_equiv

        _elapsed = max(0.0, _sim_ms / 1000.0 - SETTLE_S)
        _w_tgt = 0.0 if _want_stand(_elapsed) else 1.0
        _dw = (step_ms / 1000.0) / max(MODE_BLEND_S, 1e-3)
        _w = _w + max(-_dw, min(_dw, _w_tgt - _w))

        legs, swings = stg.targets_np(phase, gp, t_since_start=t_equiv)
        legs, _ = stg.speed_scale(legs, swings, nominal, _w)
        for i, jn in enumerate(JOINT_ORDER):
            m = motors.get(jn)
            if m is not None:
                m.setPosition(float(legs[i]))

        if trans_field is not None:
            trans_field.setSFVec3f([pos_x, Y_OFFSET, Z_BASE])

        if _glog is not None and _sim_ms - _last_log >= 1000:
            _last_log = _sim_ms
            _glog.write(f"t={_sim_ms/1000:5.1f} x={pos_x:+.2f} "
                        f"phase={phase:.2f} "
                        f"FL=({legs[0]:+.3f},{legs[1]:+.3f},{legs[2]:+.3f})\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
