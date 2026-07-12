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

"""Closed-loop POLICY parity probe -- DEPLOY side (runs in omnisim-bin).

Runs a trained G1 stand ONNX policy closed-loop in the real binary and dumps a
per-tick trace, so it can be diffed against the SAME policy run in the trainer
(gpu_mjwarp_g1_stand_trainer.py --eval --dump-trace). This is the end-to-end
train==deploy validation: same network + same obs + same action law + (proven)
same physics => identical trajectory.

Control law (IDENTICAL to the trainer, parity mode):
    obs   = [ joint_q - NOMINAL (13), proj_gravity (3), last_action (13) ]  (29)
    action = clamp(onnx(obs), -1, 1)
    target = clamp(NOMINAL + 0.3*action, LIM_LO, LIM_HI)
No joint velocity, no base linear velocity in the obs -- the two terms that are
hardest to reproduce identically in deploy are deliberately dropped.

Env: G1_POLICY_ONNX (path), PROBE_TRACE (out), POLICY_SETTLE (hold-NOMINAL steps
before the policy engages), POLICY_TICKS.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

try:
    from controller import Supervisor as _Robot
except Exception:  # pragma: no cover
    from controller import Robot as _Robot

# Must match gpu_mjwarp_g1_stand_trainer.py exactly.
LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
NJ = 13
NOMINAL = np.array([
    -0.30, 0.00, 0.00, 0.52, -0.23, 0.00,
    -0.30, 0.00, 0.00, 0.52, -0.23, 0.00,
    0.00,
], dtype=np.float32)
LIM_LO = np.array([-2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
                   -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618], np.float32)
LIM_HI = np.array([2.880, 2.967, 2.758, 2.880, 0.524, 0.262,
                   2.880, 0.524, 2.758, 2.880, 0.524, 0.262, 2.618], np.float32)
RES_SCALE = 0.3
# 32-dim obs (q-NOMINAL, proj_gravity, base ang-vel, last_action) + analytic
# ankle-balance baseline -- must match gpu_newton_g1_stand_trainer exactly.
KP_ANK, KD_ANK, BAL_CLAMP, DT = -1.5, -0.2, 0.2, 0.016
_L_AP, _R_AP, _L_AR, _R_AR = 4, 10, 5, 11


def _say(m):
    sys.stderr.write(m); sys.stderr.flush()


def main() -> int:
    onnx_path = os.environ.get("G1_POLICY_ONNX")
    out_path = Path(os.environ.get("PROBE_TRACE", str(_REPO / "_scratch/parity/g1_policy_deploy.json")))
    settle = int(os.environ.get("POLICY_SETTLE", "40"))
    n_ticks = int(os.environ.get("POLICY_TICKS", "200"))
    record_settle = os.environ.get("PROBE_RECORD_SETTLE", "0").strip() != "0"
    if not onnx_path or not Path(onnx_path).exists():
        _say(f"[g1_policy_probe] missing ONNX {onnx_path}\n"); return 1
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    motors, sensors = [], []
    for jn in LEGS_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            _say(f"[g1_policy_probe] missing motor {jn}_motor\n"); return 1
        motors.append(m)
        s = m.getPositionSensor()
        if s is not None:
            s.enable(step_ms)
        sensors.append(s)
    self_node = robot.getSelf()

    _say(f"[g1_policy_probe] onnx={Path(onnx_path).name} settle={settle} ticks={n_ticks}\n")
    last_action = np.zeros(NJ, dtype=np.float32)
    ticks = []

    def read_q():
        return np.array([float(s.getValue()) if s is not None else 0.0 for s in sensors], np.float32)

    def proj_g():
        o = self_node.getOrientation()   # row-major body->world R
        return np.array([-float(o[6]), -float(o[7]), -float(o[8])], np.float32)

    def ang_vel_body():
        # base angular velocity in BODY frame = R^T * getVelocity()[3:6] (world).
        v = self_node.getVelocity()
        o = self_node.getOrientation()
        wx, wy, wz = float(v[3]), float(v[4]), float(v[5])
        return np.array([o[0]*wx + o[3]*wy + o[6]*wz,
                         o[1]*wx + o[4]*wy + o[7]*wz,
                         o[2]*wx + o[5]*wy + o[8]*wz], np.float32)

    def base_pose():
        return [float(x) for x in self_node.getPosition()], [float(x) for x in self_node.getOrientation()]

    def capture(k, target, phase, obs=None):
        bp, br = base_pose()
        rec = {"k": k, "phase": phase, "target": [float(x) for x in target],
               "q": [float(x) for x in read_q()], "base_pos": bp, "base_rot": br}
        if obs is not None:
            rec["obs"] = [float(x) for x in obs]
        ticks.append(rec)

    _prev = {"roll": 0.0, "pitch": 0.0}

    def baseline():
        # NOMINAL + analytic ankle balance PD (matches the trainer's baseline).
        o = self_node.getOrientation()
        roll = math.atan2(float(o[7]), float(o[8]))
        pitch = -math.asin(max(-1.0, min(1.0, float(o[6]))))
        rr = (roll - _prev["roll"]) / DT
        pr = (pitch - _prev["pitch"]) / DT
        _prev["roll"], _prev["pitch"] = roll, pitch
        ap = max(-BAL_CLAMP, min(BAL_CLAMP, KP_ANK * pitch + KD_ANK * pr))
        ar = max(-BAL_CLAMP, min(BAL_CLAMP, KP_ANK * roll + KD_ANK * rr))
        b = NOMINAL.copy()
        b[_L_AP] += ap; b[_R_AP] += ap
        b[_L_AR] += ar; b[_R_AR] += ar
        return b

    # SETTLE: ramp straight->NOMINAL (no seed); advance _prev each step.
    for s in range(settle):
        frac = min(1.0, (s + 1) / max(1.0, 0.6 * settle))
        tgt = frac * NOMINAL
        baseline()
        for j in range(NJ):
            motors[j].setPosition(float(tgt[j]))
        if robot.step(step_ms) == -1:
            break
        if record_settle:
            capture(s - settle, tgt, "settle")

    # POLICY: closed loop. target = baseline (NOMINAL+ankle PD) + RES_SCALE*action.
    # Angular-rate channel = finite-diff of proj_gravity (REPRODUCIBLE), NOT the
    # engine base ang-vel (which is in a different frame/scale here than in the
    # trainer -- proven by the closed-loop parity test). First tick -> 0.
    prev_pg = [None]
    for k in range(n_ticks):
        q = read_q()
        pg = proj_g()
        dpg = np.zeros(3, np.float32) if prev_pg[0] is None else ((pg - prev_pg[0]) / DT).astype(np.float32)
        prev_pg[0] = pg
        obs = np.concatenate([q - NOMINAL, pg, dpg, last_action]).astype(np.float32)
        out = sess.run(None, {in_name: obs[None, :]})[0][0]
        action = np.clip(out, -1.0, 1.0).astype(np.float32)
        target = np.clip(baseline() + RES_SCALE * action, LIM_LO, LIM_HI)
        for j in range(NJ):
            motors[j].setPosition(float(target[j]))
        if robot.step(step_ms) == -1:
            _say(f"[g1_policy_probe] sim ended at k={k}\n"); break
        last_action = action
        capture(k, target, "probe", obs)

    out = {"schema": 1, "side": "deploy",
           "meta": {"robot": "g1", "construction": "add_link (C++ WbNewtonBackend)",
                    "sequence": "policy", "njoints": NJ, "joint_order": list(LEGS_JOINTS),
                    "onnx": Path(onnx_path).name,
                    "target_ke": os.environ.get("OMNISIM_NEWTON_TARGET_KE", "unset"),
                    "target_kd": os.environ.get("OMNISIM_NEWTON_TARGET_KD", "unset")},
           "ticks": ticks}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out), encoding="utf-8")
    nfell = sum(1 for t in ticks if t["base_pos"][2] < 0.45)
    _say(f"[g1_policy_probe] wrote {out_path} ticks={len(ticks)} "
         f"final_base_z={ticks[-1]['base_pos'][2] if ticks else float('nan'):.4f} "
         f"low_z_ticks={nfell}\n")
    try:
        robot.simulationQuit(0)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
