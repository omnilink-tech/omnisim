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

"""spot_crouch_mimic -- the real, OmniSim/Newton Spot SHADOWING its MPPI-generated
crouch-recover ghost (shadowing-pipeline Stage D / deploy).

The ghost (projects/policies/research/shadowing/ghosts/spot_crouch_ghost.npz) was DISCOVERED by the
MPPI ghost_generator over Spot's real dynamics and CERTIFIED dynamically feasible
(open-loop sustainable) by ghost_verifier -- see projects/policies/research/tools/generate_spot_crouch.py.
This controller replays that certified, feasible POSITION-TARGET trajectory on the
URDF-imported Spot in OmniSim Newton, so the robot physically shadows it:

  q_target(step) = clamp(ref_ctrl(step) + res_scale * policy(obs), joint_limits)   # 12

Because the ghost is feasible by construction (it IS the result of realizable position
targets over the real dynamics) and the crouch is quasi-static, FEEDFORWARD (no policy)
should already track it. SPOT_CROUCH_POLICY=<onnx> adds an RL residual (Component 3) for
robustness / domain-gap; the obs then mirrors a tracking trainer.

Env:
  SPOT_CROUCH_REF=<npz>     ghost (default: shadowing/ghosts/spot_crouch_ghost.npz)
  SPOT_CROUCH_POLICY=<onnx> optional RL residual (else feedforward)
  SPOT_CROUCH_RES_SCALE     residual authority (default 0.15)
  SPOT_CROUCH_LOOP=1        repeat the crouch cycle (default: play once then hold)
  SPOT_CROUCH_LOG=<csv>     per-second telemetry (t, base_z, ref_z, tilt, joint_sim)
  SPOT_SETTLE_S             settle seconds at the stand before the motion (default 1.0)
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

# warmup_reload dodges the COLD-FIRST-LOAD under-tracking (fresh articulation
# undershoots its targets) -- matters for crisp crouch tracking.
_warmup_reload = None
try:
    _BRIDGE = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
    if str(_BRIDGE) not in sys.path:
        sys.path.insert(0, str(_BRIDGE))
    from omnilink_arm_bridge import warmup_reload as _warmup_reload
except Exception:
    _warmup_reload = None

URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = [(leg, j) for leg in URDF_LEGS for j in ("hip_x", "hip_y", "knee")]
NJ = 12
JL_LO = np.array([-1.50, -0.50, -1.20] * 4, dtype=np.float32)
JL_HI = np.array([+1.50, +3.13, -0.01] * 4, dtype=np.float32)


def _envf(k, d):
    v = os.environ.get(k)
    try:
        return float(v) if v not in (None, "") else d
    except ValueError:
        return d


def _resample(ref, dt_ref, step_dt, n_out):
    """Linear-interp the (n_ref, 12) reference (sampled at dt_ref) onto n_out deploy
    ticks (step_dt). Past the end, hold the last frame (the recovered stand)."""
    n_ref = ref.shape[0]
    out = np.empty((n_out, NJ), dtype=np.float32)
    for k in range(n_out):
        f = (k * step_dt) / dt_ref
        i0 = int(math.floor(f))
        if i0 >= n_ref - 1:
            out[k] = ref[-1]
        else:
            a = f - i0
            out[k] = (1.0 - a) * ref[i0] + a * ref[i0 + 1]
    return out


def main():
    robot = _Robot()
    if _warmup_reload is not None:
        try:
            _warmup_reload(robot)
        except Exception:
            pass
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    ref_path = os.environ.get("SPOT_CROUCH_REF") or str(
        _REPO / "projects/policies/research/shadowing/ghosts/spot_crouch_ghost.npz")
    g = np.load(ref_path, allow_pickle=True)
    ctrl = g["ctrl"].astype(np.float32)              # (n_ref, 12) controller order
    dt_ref = float(g["dt"])
    ref_z = g["base"][:, 2].astype(np.float32)
    n_ref = ctrl.shape[0]
    motion_s = (n_ref - 1) * dt_ref
    sys.stderr.write(f"[spot_crouch_mimic] ghost {ref_path} -- {n_ref} steps @ "
                     f"{dt_ref*1000:.0f}ms ({motion_s:.1f}s), base_z "
                     f"{ref_z[0]:.3f}->{ref_z.min():.3f}->{ref_z[-1]:.3f}\n")

    motors, sensors = {}, {}
    for leg, j in JOINT_ORDER:
        m = robot.getDevice(f"{leg}_{j}_motor")
        if m is None:
            sys.stderr.write(f"[spot_crouch_mimic] MISSING motor {leg}_{j}_motor\n")
            return 1
        motors[(leg, j)] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
                sensors[(leg, j)] = s
        except Exception:
            pass

    res_scale = _envf("SPOT_CROUCH_RES_SCALE", 0.15)
    loop = os.environ.get("SPOT_CROUCH_LOOP", "0") != "0"
    settle = max(1, int(_envf("SPOT_SETTLE_S", 1.0) / step_dt))

    # Optional RL residual (Component 3). Feedforward when absent.
    sess = None
    pol = os.environ.get("SPOT_CROUCH_POLICY")
    if pol and os.path.exists(pol):
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(pol, providers=["CPUExecutionProvider"])
            sys.stderr.write(f"[spot_crouch_mimic] RL residual: {pol}\n")
        except Exception as e:
            sys.stderr.write(f"[spot_crouch_mimic] policy load failed ({e}); FEEDFORWARD\n")
    else:
        sys.stderr.write("[spot_crouch_mimic] FEEDFORWARD (no policy) -- shadowing the "
                         "certified ghost open-loop\n")

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    # Resample the reference onto the deploy clock (one motion-length; loop re-indexes).
    n_play = int(round(motion_s / step_dt)) + 1
    REF = _resample(ctrl, dt_ref, step_dt, n_play)

    # Settle at the standing pose (REF[0]).
    for (leg, j), m in motors.items():
        i = JOINT_ORDER.index((leg, j))
        m.setPosition(float(REF[0][i]))
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            return 0

    log_f = None
    log_path = os.environ.get("SPOT_CROUCH_LOG")
    if log_path:
        try:
            log_f = open(log_path, "w", buffering=1)
            log_f.write("t,base_z,ref_z,tilt_deg,joint_sim_pct\n")
        except Exception:
            log_f = None

    last_action = np.zeros(NJ, dtype=np.float32)
    last_q = REF[0].copy()
    k = 0
    sim_ms = 0
    next_log = 1000
    fell = False
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        idx = (k % n_play) if loop else min(k, n_play - 1)
        ref = REF[idx]

        action = np.zeros(NJ, dtype=np.float32)
        if sess is not None:
            q = np.array([sensors[(leg, j)].getValue() if (leg, j) in sensors else last_q[i]
                          for i, (leg, j) in enumerate(JOINT_ORDER)], dtype=np.float32)
            qd = (q - last_q) / step_dt
            last_q = q.copy()
            try:
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                vel = self_node.getVelocity() or [0.0] * 6
            except Exception:
                ori, vel = [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.0] * 6
            _R = np.array(ori, dtype=np.float32).reshape(3, 3)
            vlin = np.array(vel[:3], dtype=np.float32)
            vang = (_R.T @ np.array(vel[3:6], dtype=np.float32)).astype(np.float32)
            pg = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
            phase = np.array([idx / max(1, n_play - 1)], dtype=np.float32)
            obs = np.concatenate([vlin, vang, pg, q - ref, qd, last_action, phase]).astype(np.float32)
            obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0), -10.0, 10.0)
            try:
                action = np.clip(sess.run(None, {"obs": obs[None, :]})[0][0], -1.0, 1.0).astype(np.float32)
            except Exception:
                action = np.zeros(NJ, dtype=np.float32)
            last_action = action

        tgt = np.clip(ref + res_scale * action, JL_LO, JL_HI)
        for i, (leg, j) in enumerate(JOINT_ORDER):
            motors[(leg, j)].setPosition(float(tgt[i]))
        k += 1

        if self_node is not None and sim_ms >= next_log:
            next_log += 1000
            try:
                bz = float(self_node.getPosition()[2])
                o = self_node.getOrientation()
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(o[8])))))
            except Exception:
                bz, tilt = float("nan"), float("nan")
            rz = float(ref_z[min(int(idx * step_dt / dt_ref), n_ref - 1)])
            errs = []
            for i, (leg, j) in enumerate(JOINT_ORDER):
                if (leg, j) in sensors:
                    try:
                        errs.append(abs(sensors[(leg, j)].getValue() - float(ref[i])))
                    except Exception:
                        pass
            jsim = 100.0 * sum(max(0.0, 1.0 - e / 0.5) for e in errs) / len(errs) if errs else float("nan")
            phase = "STAND" if abs(rz - ref_z[0]) < 0.02 else "CROUCH"
            if not fell and (bz < 0.25 or tilt > 50):
                fell = True
                sys.stderr.write(f"FALL@{sim_ms/1000:.2f}s bz={bz:.2f} tilt={tilt:.1f}\n")
            sys.stderr.write(f"[spot_crouch_mimic] t={sim_ms/1000:4.1f}s [{phase:6s}] "
                             f"base_z={bz:.3f} (ref {rz:.3f}) tilt={tilt:4.1f}deg "
                             f"joint_sim={jsim:.0f}%\n")
            if log_f is not None:
                log_f.write(f"{sim_ms/1000:.2f},{bz:.4f},{rz:.4f},{tilt:.2f},{jsim:.2f}\n")

    if log_f is not None:
        log_f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
