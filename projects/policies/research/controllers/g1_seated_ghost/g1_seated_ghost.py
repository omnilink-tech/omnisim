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

"""g1_seated_ghost -- the kinematic, translucent "ghost" for the sit-mimic demo.

A physics-free G1 (staticBase TRUE, no physicsBackend) that DISPLAYS the shared
seated gesture reference (projects/policies/control/gait/g1_sit_gesture.full_targets). It is
rendered as a pale-blue hologram so it reads as "the motion the robot should
reproduce". No RL, no physics -- pure kinematic playback.

The real robot (controller g1_seated_mimic) consumes the SAME full_targets(t),
so it mimics exactly what this ghost shows.
"""

import os
import sys
from pathlib import Path

try:
    from omnisim import Supervisor as _Robot
except Exception:  # pragma: no cover
    from omnisim import Robot as _Robot

# --- make `projects.policies.*` importable from the controller process ---
_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())   # .../g1_seated_ghost -> repo root
if str(_REPO) not in sys.path:
    sys.path.append(str(_REPO))

from projects.policies.control.gait.g1_sit_gesture import SEATED_POSE, full_targets


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _ghostify(node, transparency, tint, depth=0):
    """Recursively set PBRAppearance.transparency + baseColor on every Shape
    under `node`, turning the robot into a translucent hologram. Returns the
    number of shapes touched. Swallows errors on non-matching nodes."""
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

    motors = {}
    for jn in SEATED_POSE:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m
    sys.stderr.write(f"[seated_ghost] {len(motors)}/{len(SEATED_POSE)} motors\n")

    # Translucent hologram look (controller-side; the ghost URDF ships opaque).
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
        sys.stderr.write(f"[seated_ghost] ghostified {n} shapes (alpha={alpha})\n")

    # ACHIEVABLE-GHOST replay (G1_GHOST_REPLAY=<csv>): instead of the idealized
    # full_targets, play back the robot's RECORDED achieved trajectory so the
    # ghost shows exactly the physically-achievable seated wave (incl. the slight
    # rock). The robot then reproduces it -> the two match.
    replay = None
    _rp = os.environ.get("G1_GHOST_REPLAY")
    if _rp and os.path.exists(_rp):
        try:
            import csv as _csv
            with open(_rp) as _f:
                replay = list(_csv.DictReader(_f))
            sys.stderr.write(f"[seated_ghost] REPLAY {len(replay)} frames from {_rp}\n")
        except Exception as e:
            sys.stderr.write(f"[seated_ghost] replay load failed ({e})\n")
            replay = None
    spawn_xy = None
    if self_node is not None:
        try:
            _tt = self_node.getField("translation").getSFVec3f()
            spawn_xy = (float(_tt[0]), float(_tt[1]))
        except Exception:
            spawn_xy = None

    # Settle into the seated pose before the gesture starts.
    for jn, q in SEATED_POSE.items():
        if jn in motors:
            motors[jn].setPosition(float(q))
    for _ in range(max(1, int(0.4 / step_dt))):
        if robot.step(step_ms) == -1:
            return 0

    sim_ms = 0
    k = 0
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        t = sim_ms / 1000.0
        if replay is not None:
            row = replay[k % len(replay)]  # loop the achievable clip for long demos
            for jn in SEATED_POSE:
                m = motors.get(jn)
                if m is not None and jn in row:
                    try:
                        m.setPosition(float(row[jn]))
                    except Exception:
                        pass
            # Also mirror the base z + rock so the ghost looks exactly achievable.
            if self_node is not None and spawn_xy is not None:
                try:
                    self_node.getField("translation").setSFVec3f(
                        [spawn_xy[0], spawn_xy[1], float(row["z"])])
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
