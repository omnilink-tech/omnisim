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

"""g1_standwave_ghost -- the kinematic, translucent "ghost" for the stand-and-wave demo.

A physics-free G1 (staticBase, no Newton) that DISPLAYS the generated stand+wave reference
(projects/policies/research/shadowing/generate_g1_standwave.py -> a replay CSV) as a pale-blue hologram:
the robot stands, balances, and WAVES its right hand. No RL, no physics -- this is the
ghost-first preview, so we agree on the motion BEFORE training the tracker to shadow it.

Replay CSV (x,z,rx,ry,rz,ra + 23 joints) via G1_GHOST_REPLAY=<csv>, looped.
"""

import os
import sys
from pathlib import Path

try:
    from omnisim import Supervisor as _Robot
except Exception:  # pragma: no cover
    from omnisim import Robot as _Robot

_ALL = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",
)


_LOG = os.path.join(os.environ.get("OMNISIM_HOME", str(next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists()))),
                    "_scratch", "standwave_ghost.dbg")


def _log(m):
    try:
        with open(_LOG, "a", buffering=1) as f:
            f.write(str(m) + "\n")
    except Exception:
        pass
    try:
        sys.stderr.write(str(m) + "\n")
    except Exception:
        pass


_log("=== standwave_ghost module load ===")


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

    motors = {}
    for jn in _ALL:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m
    _log(f"[standwave_ghost] {len(motors)}/{len(_ALL)} motors")

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None

    alpha = _genv("G1_GHOST_ALPHA", 0.55)
    if self_node is not None and alpha > 0.0:
        tint_s = os.environ.get("G1_GHOST_TINT", "0.62,0.82,1.0")
        try:
            tint = tuple(float(x) for x in tint_s.split(","))
        except Exception:
            tint = (0.62, 0.82, 1.0)
        n = _ghostify(self_node, alpha, tint)
        _log(f"[standwave_ghost] ghostified {n} shapes (alpha={alpha})\n")

    spawn_xy = None
    if self_node is not None:
        try:
            _tt = self_node.getField("translation").getSFVec3f()
            spawn_xy = (float(_tt[0]), float(_tt[1]))
        except Exception:
            spawn_xy = None

    replay = None
    _rp = os.environ.get("G1_GHOST_REPLAY")
    if _rp and os.path.exists(_rp):
        try:
            import csv as _csv
            with open(_rp) as _f:
                replay = list(_csv.DictReader(_f))
            _log(f"[standwave_ghost] REPLAY {len(replay)} frames from {_rp}")
        except Exception as e:
            _log(f"[standwave_ghost] replay load failed ({e})")
            replay = None
    if not replay:
        _log("[standwave_ghost] no replay CSV (set G1_GHOST_REPLAY); idle.")

    _loop = os.environ.get("G1_STANDWAVE_LOOP", "1") != "0"
    k = 0
    while robot.step(step_ms) != -1:
        if not replay:
            continue
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
