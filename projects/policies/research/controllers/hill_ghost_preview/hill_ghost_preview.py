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

"""Standalone KINEMATIC preview of a hill-walk ghost (Shadowing, ghost-first).

Replays a generated up-over-down hill ghost (projects/policies/research/shadowing/ghosts/
<robot>_hill_ghost.npz) on a physics-free, visual-only robot (staticBase): each
tick it sets the base translation + pitch (about Y) from base[t] and the 12 joint
motors from ctrl[t], so the translucent hologram walks up, over, and down the
hill exactly as designed -- the reference the RL tracker will shadow. No RL, no
physics; this is what we SHOW for sign-off before training the mimic.

Env:
  HILL_GHOST_NPZ   ghost file (default: ghosts/<HILL_ROBOT>_hill_ghost.npz)
  HILL_ROBOT       b2 | omniquad | go2  (selects the URDF motor naming)
  HILL_GHOST_Y     lateral offset of the hologram (default 0)
  HILL_GHOST_ALPHA hologram transparency (default 0.55; 0 = opaque)
  HILL_GHOST_TINT  "r,g,b" hologram colour (default pale blue)
  HILL_LOOP        1 = repeat the traverse (default 1)
"""
from __future__ import annotations

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

# ghost joint (controller order) -> URDF motor device name, per robot.
_OMNIQUAD_LEG = {"FL": "front_left", "FR": "front_right", "RL": "rear_left", "RR": "rear_right"}


def _motor_candidates(robot, gj):
    """Candidate motor device names for a ghost joint name like 'FL_hip' /
    'FL_hip_x'."""
    if robot == "omniquad":
        leg, part = gj.split("_", 1)                 # FL, hip_x
        return [f"{_OMNIQUAD_LEG.get(leg, leg)}_{part}_motor"]
    return [f"{gj}_joint_motor", f"{gj}_motor"]       # b2 / go2


def _ghostify(node, transparency, tint, depth=0):
    if node is None or depth > 40:
        return 0
    try:
        tn = node.getTypeName()
    except Exception:
        return 0
    n = 0
    if tn == "Shape":
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
        for getter in ("getField", "getProtoField"):
            try:
                f = getattr(node, getter)(fname)
            except Exception:
                f = None
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


def main() -> int:
    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    kind = os.environ.get("HILL_ROBOT", "b2")
    npz = os.environ.get("HILL_GHOST_NPZ") or str(
        _REPO / "projects/policies/research/shadowing/ghosts" / f"{kind}_hill_ghost.npz")
    g = np.load(npz, allow_pickle=True)
    base = g["base"].astype(float)            # (n,7) x,y,z,qw,qx,qy,qz
    ctrl = g["ctrl"].astype(float)            # (n,12) joint targets, controller order
    jnames = [str(j) for j in g["joints"]]
    dt_ref = float(g["dt"])
    n_ref = base.shape[0]
    y_off = float(os.environ.get("HILL_GHOST_Y", "0") or 0.0)
    loop = os.environ.get("HILL_LOOP", "1") != "0"
    sys.stderr.write(f"[hill_ghost_preview] {kind}: {n_ref} frames @ {dt_ref*1000:.0f}ms, "
                     f"x {base[0,0]:.2f}->{base[:,0].max():.2f} m, crest z {base[:,2].max():.2f}\n")

    motors = []
    for gj in jnames:
        dev = None
        for cand in _motor_candidates(kind, gj):
            dev = robot.getDevice(cand)
            if dev is not None:
                break
        motors.append(dev)
        if dev is None:
            sys.stderr.write(f"[hill_ghost_preview] MISSING motor for {gj} "
                             f"(tried {_motor_candidates(kind, gj)})\n")
    sys.stderr.write(f"[hill_ghost_preview] {sum(m is not None for m in motors)}/{len(motors)} motors\n")

    self_node = robot.getSelf()
    trans_f = self_node.getField("translation") if self_node else None
    rot_f = self_node.getField("rotation") if self_node else None

    alpha = float(os.environ.get("HILL_GHOST_ALPHA", "0.55") or 0.0)
    if alpha > 0.0 and self_node is not None:
        try:
            tint = tuple(float(x) for x in os.environ.get("HILL_GHOST_TINT", "0.62,0.82,1.0").split(","))
        except ValueError:
            tint = (0.62, 0.82, 1.0)
        ns = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[hill_ghost_preview] ghostified {ns} shapes\n")

    # optional headless self-check: env-gated status file (off by default) so a
    # GUI-less run can confirm the hologram bound its motors and moved.
    status = None
    sp = os.environ.get("HILL_PREVIEW_STATUS")
    if sp:
        status = open(sp, "w", buffering=1)
        status.write(f"motors_bound={sum(m is not None for m in motors)}/{len(motors)}\n")

    sim_ms = 0
    next_status = 1000
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        f = (sim_ms / 1000.0) / dt_ref
        k = int(f) % n_ref if loop else min(int(f), n_ref - 1)
        b = base[k]
        theta_y = 2.0 * math.atan2(float(b[5]), float(b[3]))       # pure Y-rotation
        if trans_f is not None:
            trans_f.setSFVec3f([float(b[0]), y_off, float(b[2])])
        if rot_f is not None:
            rot_f.setSFRotation([0.0, 1.0, 0.0, theta_y])
        for i, m in enumerate(motors):
            if m is not None:
                m.setPosition(float(ctrl[k][i]))
        if status is not None and sim_ms >= next_status:
            next_status += 1000
            px = float(self_node.getPosition()[0]) if self_node else float("nan")
            status.write(f"t={sim_ms/1000:4.1f} k={k} ref_x={b[0]:+.2f} ref_z={b[2]:.2f} "
                         f"pitch={math.degrees(theta_y):+5.1f} node_x={px:+.2f}\n")
    if status is not None:
        status.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
