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

"""spot_getup_mimic -- the real OmniSim/Newton Spot SHADOWING its MPPI get-up ghost,
RL-stabilised (shadowing Stage D / deploy, Component 3 in the loop).

Unlike the crouch (feedforward), the get-up is a contact handoff the bare reference
can't hold open-loop, so the RL residual (gpu_mjwarp_spot_getup_trainer) supplies the
feedback. The robot SPAWNS LYING (belly on the floor, legs splayed = the ghost's first
frame), then tracks the certified ghost up to a stand:

  q_target(step) = clamp(ref_ctrl(step) + res_scale * policy(obs), joint_limits)   # 12

The obs MIRRORS gpu_mjwarp_spot_getup_trainer._build_obs_t EXACTLY (49-dim):
  [vlin_world(3), vang_BODY(3), proj_g(3), q-ref_legs(12), qd(12), last_action(12),
   phase(4)=[b, b_ahead(+0.2s), z_err, pitch_err]]

Env: SPOT_GETUP_REF / SPOT_GETUP_POLICY / SPOT_GETUP_RES_SCALE / SPOT_GETUP_LOG /
     SPOT_SETTLE_S (lying-settle seconds) / SPOT_GETUP_LOOP.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.gait import spot_trot_gait as _stg  # noqa: E402  (proven-stable stand)

_DBG = str(Path(__file__).resolve().parent / "spot_getup_mimic.dbg")


def _dbg(m):
    try:
        with open(_DBG, "a", buffering=1) as f:
            f.write(str(m) + "\n")
    except Exception:
        pass


_dbg("=== module load ===")

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

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
DT = 0.016
LOOKAHEAD_STEPS = int(round(0.2 / DT))
JL_LO = np.array([-1.50, -0.50, -1.20] * 4, dtype=np.float32)
JL_HI = np.array([+1.50, +3.13, -0.01] * 4, dtype=np.float32)


def _envf(k, d):
    v = os.environ.get(k)
    try:
        return float(v) if v not in (None, "") else d
    except ValueError:
        return d


def _resample(arr, dt_ref, n_out):
    """Linear-interp (n_ref, C) at dt_ref onto n_out ticks at DT (hold last)."""
    n_ref = arr.shape[0]
    t = np.arange(n_out) * DT
    src = np.arange(n_ref) * dt_ref
    return np.stack([np.interp(t, src, arr[:, c]) for c in range(arr.shape[1])], axis=1)


def main():
    robot = _Robot()
    if _warmup_reload is not None:
        try:
            _warmup_reload(robot)
        except Exception:
            pass
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    ref_path = os.environ.get("SPOT_GETUP_REF") or str(
        _REPO / "projects/policies/research/shadowing/ghosts/spot_getup_ghost.npz")
    g = np.load(ref_path, allow_pickle=True)
    dt_ref = float(g["dt"])
    n_out = int(round((g["ctrl"].shape[0] - 1) * dt_ref / DT)) + 1
    REF = _resample(g["ctrl"].astype(np.float32), dt_ref, n_out).astype(np.float32)   # (n,12)
    base = _resample(g["base"].astype(np.float32), dt_ref, n_out).astype(np.float32)  # (n,7)
    REF_Z = base[:, 2].astype(np.float32)
    _w, _x, _y, _z = base[:, 3], base[:, 4], base[:, 5], base[:, 6]
    REF_PITCH = np.arcsin(np.clip(2 * (_w * _y - _z * _x), -1, 1)).astype(np.float32)
    N = n_out
    _dbg(f"ghost loaded: {N} steps, base_z {REF_Z[0]:.3f}->{REF_Z.max():.3f}")
    sys.stderr.write(f"[spot_getup_mimic] ghost {ref_path}: {N} steps, base_z "
                     f"{REF_Z[0]:.3f}->{REF_Z.max():.3f}\n")

    motors, sensors = {}, {}
    for leg, j in JOINT_ORDER:
        m = robot.getDevice(f"{leg}_{j}_motor")
        if m is None:
            sys.stderr.write(f"[spot_getup_mimic] MISSING motor {leg}_{j}_motor\n")
            return 1
        motors[(leg, j)] = m
        s = m.getPositionSensor()
        if s is not None:
            s.enable(step_ms)
            sensors[(leg, j)] = s

    res_scale = _envf("SPOT_GETUP_RES_SCALE", 0.15)
    loop = os.environ.get("SPOT_GETUP_LOOP", "0") != "0"
    settle = max(1, int(_envf("SPOT_SETTLE_S", 1.5) / step_dt))

    sess = None
    pol = os.environ.get("SPOT_GETUP_POLICY")
    if pol and os.path.exists(pol):
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(pol, providers=["CPUExecutionProvider"])
            sys.stderr.write(f"[spot_getup_mimic] RL policy: {pol}\n")
        except Exception as e:
            sys.stderr.write(f"[spot_getup_mimic] policy load failed ({e}); FEEDFORWARD\n")
    else:
        sys.stderr.write("[spot_getup_mimic] FEEDFORWARD (no policy) -- expect it to sag\n")

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    _dbg(f"motors={len(motors)} sensors={len(sensors)} policy={'yes' if sess else 'no'} "
         f"settle={settle} -> teleport to the ghost lying pose")
    # TELEPORT to the EXACT ghost lying pose (base + joints) so the deploy START
    # matches the trainer's RSI start. Newton's FREE settle can't form the belly-
    # flat pose (it rests half-propped at 0.25/22deg -> OOD), so set the base
    # translation/rotation + joints to the ghost's first frame directly (the robot
    # is a Supervisor), zero the velocity (avoid the warm-solver teleport bounce),
    # then hold briefly. base0 = [x,y,z, qw,qx,qy,qz].
    b0 = base[0]
    # Start joints = the ghost's first frame. For a STANDING-start ghost (jump),
    # ctrl[0] is a NOISY MPPI control, not a clean stand -> teleporting to it
    # collapses the robot (the trainer RSI starts from the achieved clean stand).
    # Use the clean trot stand for a standing start; ctrl[0] for a low start (get-up).
    if float(b0[2]) > 0.45:
        start_legs = _stg.standing_pose(_stg.GaitParams(body_height=0.55)).astype(np.float32)
    else:
        start_legs = REF[0]
    for i, (leg, j) in enumerate(JOINT_ORDER):
        motors[(leg, j)].setPosition(float(start_legs[i]))
    if self_node is not None:
        try:
            self_node.getField("translation").setSFVec3f([0.0, 0.0, float(b0[2])])
            qw = float(b0[3]); s = math.sqrt(max(1e-9, 1.0 - qw * qw))
            if s > 1e-6:
                self_node.getField("rotation").setSFRotation(
                    [float(b0[4]) / s, float(b0[5]) / s, float(b0[6]) / s,
                     2.0 * math.acos(max(-1.0, min(1.0, qw)))])
            else:
                self_node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
            self_node.setVelocity([0.0] * 6)
        except Exception as e:
            _dbg(f"teleport failed: {e}")
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            _dbg("sim ended DURING settle")
            return 0
    try:
        _dbg(f"teleport+settle done base_z={float(self_node.getPosition()[2]):.3f} -> get-up loop")
    except Exception:
        _dbg("settle done -> get-up loop")

    log_f = None
    log_path = os.environ.get("SPOT_GETUP_LOG")
    if log_path:
        try:
            log_f = open(log_path, "w", buffering=1)
            log_f.write("t,base_z,ref_z,tilt_deg,joint_sim_pct\n")
        except Exception:
            log_f = None

    # After the get-up REACHES the stand (ref ends), FADE the residual to 0 over
    # SPOT_GETUP_HOLD_FADE_S: the reference's final pose IS the stable trot stand
    # (it holds open-loop), and the rise-trained residual otherwise keeps injecting
    # corrections into the now-static stand and topples it. Feedforward hold = stable.
    hold_fade_s = _envf("SPOT_GETUP_HOLD_FADE_S", 1.0)
    fade_steps = max(1, int(hold_fade_s / step_dt))
    # The crisp, proven-stable stand to settle into once up (the walk's bare model
    # holds it open-loop). Ramp the baseline REF[-1] -> this as the residual fades.
    TROT_STAND = _stg.standing_pose(_stg.GaitParams(body_height=0.55)).astype(np.float32)
    done_k = None
    peak_bz = 0.0
    min_foot_clear = 9.9   # min over time of the lowest foot's height proxy (base_z - ~stand)
    last_action = np.zeros(NJ, dtype=np.float32)
    last_q = REF[0].copy()
    k = 0
    sim_ms = 0
    next_log = 1000
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        ei = (k % N) if loop else min(k, N - 1)
        ref = REF[ei]

        q = np.array([sensors[(leg, j)].getValue() if (leg, j) in sensors else last_q[i]
                      for i, (leg, j) in enumerate(JOINT_ORDER)], dtype=np.float32)
        qd = (q - last_q) / step_dt
        last_q = q.copy()
        try:
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel = self_node.getVelocity() or [0.0] * 6
            bz = float(self_node.getPosition()[2])
        except Exception:
            ori, vel, bz = [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.0] * 6, 0.0
        _R = np.array(ori, dtype=np.float32).reshape(3, 3)
        vlin = np.array(vel[:3], dtype=np.float32)
        vang = (_R.T @ np.array(vel[3:6], dtype=np.float32)).astype(np.float32)
        pg = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        pitch = math.asin(max(-1.0, min(1.0, float(pg[0]))))
        b = ei / max(1, N - 1)
        b_a = min(ei + LOOKAHEAD_STEPS, N - 1) / max(1, N - 1)
        z_err = bz - float(REF_Z[ei])
        pitch_err = pitch - float(REF_PITCH[ei])
        phase = np.array([b, b_a, z_err, pitch_err], dtype=np.float32)
        obs = np.concatenate([vlin, vang, pg, q - ref, qd, last_action, phase]).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0), -10.0, 10.0)

        action = np.zeros(NJ, dtype=np.float32)
        if sess is not None:
            try:
                action = np.clip(sess.run(None, {"obs": obs[None, :]})[0][0],
                                 -1.0, 1.0).astype(np.float32)
            except Exception:
                action = np.zeros(NJ, dtype=np.float32)
        last_action = action

        # Once up (ref ended): ramp the baseline REF[-1] -> the crisp trot stand and
        # FADE the residual to 0, so it settles into a stable upright stand instead
        # of the rise-trained policy toppling the static hold.
        if not loop and ei >= N - 1 and done_k is None:
            done_k = k
        fade = 1.0
        base_tgt = ref
        if done_k is not None:
            prog = min(1.0, (k - done_k) / fade_steps)
            fade = 1.0 - prog
            base_tgt = (1.0 - prog) * ref + prog * TROT_STAND
        tgt = np.clip(base_tgt + (res_scale * fade) * action, JL_LO, JL_HI)
        for i, (leg, j) in enumerate(JOINT_ORDER):
            motors[(leg, j)].setPosition(float(tgt[i]))
        k += 1

        peak_bz = max(peak_bz, bz)
        if sim_ms >= next_log:
            next_log += 1000
            _dbg(f"t={sim_ms/1000:.1f} bz={bz:.3f} peak_bz={peak_bz:.3f} tilt~")
            try:
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ori[8])))))
            except Exception:
                tilt = float("nan")
            errs = [abs(sensors[(leg, j)].getValue() - float(ref[i]))
                    for i, (leg, j) in enumerate(JOINT_ORDER) if (leg, j) in sensors]
            jsim = 100.0 * sum(max(0.0, 1.0 - e / 0.5) for e in errs) / len(errs) if errs else float("nan")
            ph = "LYING" if b < 0.05 else ("STAND" if b > 0.95 else "RISING")
            sys.stderr.write(f"[spot_getup_mimic] t={sim_ms/1000:4.1f}s [{ph:6s}] "
                             f"base_z={bz:.3f} (ref {REF_Z[ei]:.3f}) tilt={tilt:4.1f}deg "
                             f"joint_sim={jsim:.0f}%\n")
            if log_f is not None:
                log_f.write(f"{sim_ms/1000:.2f},{bz:.4f},{REF_Z[ei]:.4f},{tilt:.2f},{jsim:.2f}\n")

    if log_f is not None:
        log_f.close()
    return 0


if __name__ == "__main__":
    import traceback
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException as e:
        _dbg("CRASH: " + repr(e) + "\n" + traceback.format_exc())
        raise
