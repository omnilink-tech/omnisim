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

"""g1_sitstand_ghost -- the kinematic, translucent "ghost" for the sit-stand-sit
mimic demo.

A physics-free G1 (staticBase TRUE, no physicsBackend) that DISPLAYS the shared
sit->stand->sit reference (projects/policies/control/gait/g1_sitstand). Rendered as a pale-blue
hologram = "the motion the robot should reproduce". No RL, no physics.

Two display modes:
  - DEFAULT (idealized): play full_targets(t); raise the base to ref_pelvis_z(t)
    and step forward ref_pelvis_x(t) so the hologram visibly stands up + forward.
  - ACHIEVABLE (G1_GHOST_REPLAY=<csv>): replay the robot's RECORDED achieved
    trajectory (base x, z, rotation + 23 joints) so the ghost shows EXACTLY what
    the robot physically did -- the achievable ghost the recipe requires.

The real robot (controller g1_sitstand_mimic) consumes the SAME reference, so it
mimics what this ghost shows.
"""

import os
import sys
from pathlib import Path

try:
    from omnisim import Supervisor as _Robot
except Exception:  # pragma: no cover
    from omnisim import Robot as _Robot

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
if str(_REPO) not in sys.path:
    sys.path.append(str(_REPO))

from projects.policies.control.gait.g1_sitstand import (
    full_targets, ref_pelvis_z, ref_pelvis_x, ref_pelvis_pitch, T_TOTAL)
import math as _math


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _ghostify(node, transparency, tint, depth=0):
    if node is None or depth > 40:
        return 0
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
                return 1
        except Exception:
            pass
        return 0
    n = 0
    for fname in ("children", "endPoint", "device"):
        f = None
        for getter in ("getField", "getProtoField"):
            try:
                f = getattr(node, getter)(fname)
            except Exception:
                f = None
            if f is not None:
                break
        if f is None:
            continue
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


def main():
    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    _ALL = list(full_targets(0.0))
    motors = {}
    for jn in _ALL:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m
    sys.stderr.write(f"[sitstand_ghost] {len(motors)}/{len(_ALL)} motors\n")

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None
    alpha = _genv("G1_GHOST_ALPHA", 0.55)
    if self_node is not None and alpha > 0.0:
        # green for the "upright" variant, blue otherwise -- A/B compare clarity
        try:
            _nm = robot.getName() or ""
        except Exception:
            _nm = ""
        _default_tint = "0.50,0.88,0.60" if "upright" in _nm else "0.62,0.82,1.0"
        tint_s = os.environ.get("G1_GHOST_TINT", _default_tint)
        try:
            tint = tuple(float(x) for x in tint_s.split(","))
        except Exception:
            tint = (0.62, 0.82, 1.0)
        n = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[sitstand_ghost] ghostified {n} shapes (alpha={alpha})\n")

    spawn_xy = None
    if self_node is not None:
        try:
            _tt = self_node.getField("translation").getSFVec3f()
            spawn_xy = (float(_tt[0]), float(_tt[1]))
        except Exception:
            spawn_xy = None

    replay = None
    # Per-robot replay CSV via customData (lets two ghosts replay DIFFERENT clips in
    # one world for A/B comparison); falls back to the G1_GHOST_REPLAY env var.
    _rp = None
    try:
        _cd = robot.getCustomData()
        if _cd and os.path.exists(_cd):
            _rp = _cd
    except Exception:
        _rp = None
    if _rp is None:
        _rp = os.environ.get("G1_GHOST_REPLAY")
    if _rp and os.path.exists(_rp):
        try:
            import csv as _csv
            with open(_rp) as _f:
                replay = list(_csv.DictReader(_f))
            sys.stderr.write(f"[sitstand_ghost] REPLAY {len(replay)} frames from {_rp}\n")
        except Exception as e:
            sys.stderr.write(f"[sitstand_ghost] replay load failed ({e})\n")
            replay = None

    # Settle into the seated start pose.
    for jn, q in full_targets(0.0).items():
        if jn in motors:
            motors[jn].setPosition(float(q))
    for _ in range(max(1, int(0.4 / step_dt))):
        if robot.step(step_ms) == -1:
            return 0

    _loop = os.environ.get("G1_SITSTAND_LOOP", "0") != "0"
    sim_ms = 0
    k = 0
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        t_abs = sim_ms / 1000.0
        t = (t_abs % T_TOTAL) if _loop else min(t_abs, T_TOTAL - 0.05)
        if replay is not None:
            ri = (k % len(replay)) if _loop else min(k, len(replay) - 1)
            row = replay[ri]
            for jn in _ALL:
                m = motors.get(jn)
                if m is not None and jn in row:
                    try:
                        m.setPosition(float(row[jn]))
                    except Exception:
                        pass
            if self_node is not None and spawn_xy is not None:
                try:
                    self_node.getField("translation").setSFVec3f(
                        [spawn_xy[0] + float(row["x"]), spawn_xy[1], float(row["z"])])
                    self_node.getField("rotation").setSFRotation(
                        [float(row["rx"]), float(row["ry"]), float(row["rz"]), float(row["ra"])])
                except Exception:
                    pass
            k += 1
        else:
            for jn, q in full_targets(t).items():
                m = motors.get(jn)
                if m is not None:
                    m.setPosition(float(q))
            # Raise + step forward + lean (base pitch) so it visibly stands up.
            if self_node is not None and spawn_xy is not None:
                try:
                    self_node.getField("translation").setSFVec3f(
                        [spawn_xy[0] + float(ref_pelvis_x(t)), spawn_xy[1], float(ref_pelvis_z(t))])
                    # forward lean = rotation about the lateral (y) axis
                    self_node.getField("rotation").setSFRotation(
                        [0.0, 1.0, 0.0, float(ref_pelvis_pitch(t))])
                except Exception:
                    pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
