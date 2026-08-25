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

"""Kinematic GHOST for the OmniQuad GET-UP -- "the ideal the robot shadows".

A physics-free translucent OmniQuad (omniquad_ghost.urdf, staticBase) that REPLAYS the
MPPI-generated, verifier-certified get-up reference (shadowing/ghosts/
omniquad_getup_ghost.npz) beside the real RL-stabilised robot (DEF OMNIQUAD_REAL). The
ghost plays the IDEAL belly-flat->stand trajectory (base pose from q[t] + joints
from ctrl[t]); the gap to the real robot is the tracking error. Synced to the
real controller's schedule via OMNIQUAD_SETTLE_S, and (like the real deploy) ramps to
the crisp trot stand after the get-up ends. Hologram look: OMNIQUAD_GHOST_ALPHA/TINT.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))
from projects.policies.control.gait import omniquad_trot_gait as _stg  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = [f"{leg}_{j}" for leg in URDF_LEGS for j in ("hip_x", "hip_y", "knee")]
NJ = 12
DT = 0.016


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _ghostify(node, transparency, tint, depth=0):
    """Make every Shape under `node` translucent + tinted (omniquad_ghost recipe:
    getCount()==-1 on SF fields, walk getField AND getProtoField)."""
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


def _resample(arr, dt_ref, n_out):
    t = np.arange(n_out) * DT
    src = np.arange(arr.shape[0]) * dt_ref
    return np.stack([np.interp(t, src, arr[:, c]) for c in range(arr.shape[1])], axis=1)


def main():
    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    ref_path = os.environ.get("OMNIQUAD_GETUP_REF") or str(
        _REPO / "projects/policies/research/shadowing/ghosts/omniquad_getup_ghost.npz")
    g = np.load(ref_path, allow_pickle=True)
    dt_ref = float(g["dt"])
    n_out = int(round((g["ctrl"].shape[0] - 1) * dt_ref / DT)) + 1
    CTRL = _resample(g["ctrl"].astype(np.float32), dt_ref, n_out).astype(np.float32)  # (n,12)
    BASE = _resample(g["base"].astype(np.float32), dt_ref, n_out).astype(np.float32)  # (n,7)
    N = n_out
    TROT_STAND = _stg.standing_pose(_stg.GaitParams(body_height=0.55)).astype(np.float32)
    Y_OFFSET = _genv("OMNIQUAD_GHOST_Y", 1.4)
    SETTLE_S = _genv("OMNIQUAD_SETTLE_S", 1.0)
    FADE_S = _genv("OMNIQUAD_GETUP_HOLD_FADE_S", 1.2)

    motors = {}
    for jn in JOINT_ORDER:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m
    sys.stderr.write(f"[omniquad_getup_ghost] {len(motors)}/12 motors, ref {N} steps\n")

    self_node = robot.getSelf()
    try:
        trans_field = self_node.getField("translation")
        rot_field = self_node.getField("rotation")
    except Exception:
        trans_field = rot_field = None

    alpha = _genv("OMNIQUAD_GHOST_ALPHA", 0.6)
    if alpha > 0.0:
        try:
            tint = tuple(float(x) for x in
                         os.environ.get("OMNIQUAD_GHOST_TINT", "0.62,0.82,1.0").split(","))
        except ValueError:
            tint = (0.62, 0.82, 1.0)
        ns = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[omniquad_getup_ghost] ghostified {ns} shapes\n")

    sim_ms = 0
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        # Step index synced to the real controller (offset by its settle).
        ph = (sim_ms / 1000.0 - SETTLE_S) / DT
        ei = int(max(0, min(N - 1, ph)))
        legs = CTRL[ei].copy()
        bz = float(BASE[ei][2])
        quat = BASE[ei][3:7]
        # Once up, ramp legs + base to the crisp trot stand (match the real deploy).
        if ph >= N - 1:
            prog = min(1.0, (ph - (N - 1)) / max(1.0, FADE_S / DT))
            legs = (1.0 - prog) * legs + prog * TROT_STAND
            bz = (1.0 - prog) * bz + prog * 0.55
            quat = (1.0 - prog) * quat + prog * np.array([1.0, 0, 0, 0], np.float32)
        for i, jn in enumerate(JOINT_ORDER):
            if jn in motors:
                motors[jn].setPosition(float(legs[i]))
        if trans_field is not None:
            trans_field.setSFVec3f([float(BASE[ei][0]), Y_OFFSET + float(BASE[ei][1]), bz])
        if rot_field is not None:
            qw = float(quat[0]); s = math.sqrt(max(1e-9, 1.0 - qw * qw))
            if s > 1e-6:
                rot_field.setSFRotation([float(quat[1]) / s, float(quat[2]) / s,
                                         float(quat[3]) / s,
                                         2.0 * math.acos(max(-1.0, min(1.0, qw)))])
            else:
                rot_field.setSFRotation([0.0, 0.0, 1.0, 0.0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
