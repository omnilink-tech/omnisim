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

"""b2_getup_mimic -- the real OmniSim/Newton Spot SHADOWING its MPPI get-up ghost,
RL-stabilised (shadowing Stage D / deploy, Component 3 in the loop).

Unlike the crouch (feedforward), the get-up is a contact handoff the bare reference
can't hold open-loop, so the RL residual (gpu_mjwarp_spot_getup_trainer) supplies the
feedback. The robot SPAWNS LYING (belly on the floor, legs splayed = the ghost's first
frame), then tracks the certified ghost up to a stand:

  q_target(step) = clamp(ref_ctrl(step) + res_scale * policy(obs), joint_limits)   # 12

The obs MIRRORS gpu_mjwarp_spot_getup_trainer._build_obs_t EXACTLY (49-dim):
  [vlin_world(3), vang_BODY(3), proj_g(3), q-ref_legs(12), qd(12), last_action(12),
   phase(4)=[b, b_ahead(+0.2s), z_err, pitch_err]]

Env: B2_GETUP_REF / B2_GETUP_POLICY / B2_GETUP_RES_SCALE / B2_GETUP_LOG /
     B2_SETTLE_S (lying-settle seconds) / B2_GETUP_LOOP.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.control.gait import b2_trot_gait as _stg  # noqa: E402  (proven-stable stand)

_DBG = str(Path(__file__).resolve().parent / "b2_getup_mimic.dbg")


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

URDF_LEGS = ("FL", "FR", "RL", "RR")
JOINT_ORDER = [(leg, j) for leg in URDF_LEGS for j in ("hip", "thigh", "calf")]
NJ = 12
DT = 0.016
LOOKAHEAD_STEPS = int(round(0.2 / DT))
JL_LO = np.array([-0.87, -0.94, -2.82] * 4, dtype=np.float32)
JL_HI = np.array([+0.87, +3.1316, -0.43] * 4, dtype=np.float32)


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

    ref_path = os.environ.get("B2_GETUP_REF") or str(
        _REPO / "projects/policies/research/shadowing/ghosts/b2_getup_ghost.npz")
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
    _dbg(f"[b2_getup_mimic] ghost {ref_path}: {N} steps, base_z "
                     f"{REF_Z[0]:.3f}->{REF_Z.max():.3f}\n")

    motors, sensors = {}, {}
    for leg, j in JOINT_ORDER:
        m = robot.getDevice(f"{leg}_{j}_joint_motor")
        if m is None:
            _dbg(f"[b2_getup_mimic] MISSING motor {leg}_{j}_motor\n")
            return 1
        motors[(leg, j)] = m
        s = m.getPositionSensor()
        if s is not None:
            s.enable(step_ms)
            sensors[(leg, j)] = s

    res_scale = _envf("B2_GETUP_RES_SCALE", 0.15)
    loop = os.environ.get("B2_GETUP_LOOP", "0") != "0"
    settle = max(1, int(_envf("B2_SETTLE_S", 1.5) / step_dt))

    sess = None
    pol = os.environ.get("B2_GETUP_POLICY")
    if pol and os.path.exists(pol):
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(pol, providers=["CPUExecutionProvider"])
            _dbg(f"[b2_getup_mimic] RL policy: {pol}\n")
        except Exception as e:
            _dbg(f"[b2_getup_mimic] policy load failed ({e}); FEEDFORWARD\n")
    else:
        _dbg("[b2_getup_mimic] FEEDFORWARD (no policy) -- expect it to sag\n")

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
    for i, (leg, j) in enumerate(JOINT_ORDER):
        motors[(leg, j)].setPosition(float(REF[0][i]))
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
    log_path = os.environ.get("B2_GETUP_LOG")
    if log_path:
        try:
            log_f = open(log_path, "w", buffering=1)
            log_f.write("t,base_z,ref_z,tilt_deg,joint_sim_pct\n")
        except Exception:
            log_f = None

    # After the get-up REACHES the stand (ref ends), FADE the residual to 0 over
    # B2_GETUP_HOLD_FADE_S: the reference's final pose IS the stable trot stand
    # (it holds open-loop), and the rise-trained residual otherwise keeps injecting
    # corrections into the now-static stand and topples it. Feedforward hold = stable.
    hold_fade_s = _envf("B2_GETUP_HOLD_FADE_S", 0.25)
    fade_steps = max(1, int(hold_fade_s / step_dt))
    HANDOFF_B = _envf("B2_GETUP_HANDOFF_B", 0.42)  # ghost phase to start the hold hand-off
    up_hits = 0      # consecutive steps detected upright-at-stand-height past HANDOFF_B
    # The crisp, proven-stable stand to settle into once up (the walk's bare model
    # holds it open-loop). Ramp the baseline REF[-1] -> this as the residual fades.
    TROT_STAND = _stg.standing_pose(_stg.GaitParams(body_height=0.50)).astype(np.float32)
    done_k = None

    # ── WALK-HOLD hand-off: the rise arrives with momentum a STATIC PD stand
    # can't catch, so once the get-up reaches the clean stand we hand off to the
    # PROVEN B2 walk balancer (gpu_b2_walk_main) run at vx=0 -- its gait collapses
    # to the standing pose (speed_scale s=0) and the policy ACTIVELY balances the
    # stand (it walks B2 100m+ with roll/pitch ~3deg). Its standing pose ==
    # TROT_STAND == the ghost stand (all body_height=0.50), so the robot is
    # in-distribution at the catch. obs = the b2_walk_deploy 48-dim contract. ──
    walk_sess = None
    walk_pol = os.environ.get("B2_WALK_POLICY") or str(
        _REPO / "projects/policies/research/inference/policies/gpu_b2_walk_main/policy.onnx")
    if os.path.exists(walk_pol):
        try:
            import onnxruntime as _ort2
            walk_sess = _ort2.InferenceSession(
                walk_pol, providers=["CPUExecutionProvider"])
            _dbg(f"[b2_getup_mimic] WALK-HOLD balancer: {walk_pol}\n")
        except Exception as e:
            _dbg(f"[b2_getup_mimic] walk-hold policy load failed ({e})\n")
    _walk_gp = _stg.GaitParams(
        vx=_envf("B2_GAIT_VX", 0.5), freq=_envf("B2_GAIT_FREQ", 1.3),
        duty=_envf("B2_GAIT_DUTY", 0.6), step_height=_envf("B2_GAIT_STEP_H", 0.08),
        body_height=_envf("B2_GAIT_BODY_H", 0.50), x0=0.0, ramp_s=1.0)
    _walk_nominal = _stg.standing_pose(_walk_gp).astype(np.float32)
    _walk_res = _envf("B2_WALK_RES_SCALE", 0.15)
    _walk_omega = 2.0 * math.pi * _walk_gp.freq
    walk_gait_t = 0.0
    walk_last_action = np.zeros(NJ, dtype=np.float32)

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
        # proj_gravity = body-frame gravity = -(THIRD ROW of body->world R)
        # = -(ori[6,7,8]); the stock code used the third COLUMN (ori[2,5,8]) --
        # ~equal only when UPRIGHT (walking tolerated it) but TRANSPOSED garbage
        # during a get-up large tilt -> wrong "up" -> cannot balance the rise.
        pg = np.array([-ori[6], -ori[7], -ori[8]], dtype=np.float32)
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
        # Hand the rise-policy off to a STATIC PD stand once the GHOST is past
        # its stand keyframe (phase b>=HANDOFF_B, ~t=3.2s) -- deterministic, so
        # it never fires mid-push-up (a robot-state trigger did, on a transient
        # 0.45 m crossing, and disrupted the rise). The deploy trace showed a
        # clean stand (z=0.52, tilt~2deg) at t=3s that the rise-trained residual
        # then toppled by t=4s; once up we fade residual->0 + baseline->
        # TROT_STAND (B2 holds that pose open-loop under the stiff PD).
        tilt_rad = math.acos(max(-1.0, min(1.0, float(ori[8]))))
        # STRICT catch: only the CLEAN full-stand arrival (near full height AND
        # nearly upright) -- this state never occurs during the rise (which
        # passes through lower z / higher tilt), so the hand-off can't fire
        # mid-push-up and disrupt the rise. The instant it's reached we freeze
        # to a static PD stand (fade residual->0, baseline->TROT_STAND), before
        # the rise-trained residual can perturb the held stand and topple it.
        # Fire the hand-off only at the PEAK/settle of the rise (vertical vel
        # ~0), NOT while the robot is still rising through z=0.50 -- a balancer's
        # "pull to nominal" fights the upward momentum mid-rise and topples it.
        reached = ((bz > _envf("B2_HANDOFF_Z", 0.50))
                   and (tilt_rad < _envf("B2_HANDOFF_TILT", 0.18))
                   and (float(vel[2]) < _envf("B2_HANDOFF_VZ", 0.2)))
        if reached:
            up_hits += 1
        else:
            up_hits = 0
        if not loop and done_k is None and (ei >= N - 1 or up_hits >= 1):
            done_k = k
        if done_k is not None and walk_sess is not None:
            # WALK-HOLD: the proven B2 walk balancer at vx=0 actively holds the
            # stand (its gait collapses to the standing pose; the policy supplies
            # the balance feedback the static fade lacked). 48-dim walk obs
            # (b2_walk_deploy contract), reusing this tick's vlin/vang/pg/qd.
            gait_phase = _stg.QS_PHASE + _walk_omega * walk_gait_t
            gait_obs = np.array([math.sin(gait_phase), math.cos(gait_phase)],
                                dtype=np.float32)
            wobs = np.concatenate([
                vlin, vang, pg, q - _walk_nominal, qd, walk_last_action,
                gait_obs, [np.float32(0.0)]]).astype(np.float32)
            wobs = np.clip(np.nan_to_num(wobs, nan=0.0, posinf=10.0, neginf=-10.0),
                           -10.0, 10.0)
            try:
                wact = np.clip(walk_sess.run(None, {"obs": wobs[None, :]})[0][0],
                               -1.0, 1.0).astype(np.float32)
            except Exception:
                wact = np.zeros(NJ, dtype=np.float32)
            walk_last_action = wact
            qm, swm = _stg.targets_np(gait_phase, _walk_gp,
                                      t_since_start=walk_gait_t, wz=0.0)
            qm, _ = _stg.speed_scale(qm, swm, _walk_nominal, 0.0)   # s=0 -> stand
            tgt = np.clip(qm.astype(np.float32) + wact * _walk_res, JL_LO, JL_HI)
            walk_gait_t += step_dt
        else:
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

        if sim_ms >= next_log:
            next_log += 1000
            try:
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ori[8])))))
            except Exception:
                tilt = float("nan")
            errs = [abs(sensors[(leg, j)].getValue() - float(ref[i]))
                    for i, (leg, j) in enumerate(JOINT_ORDER) if (leg, j) in sensors]
            jsim = 100.0 * sum(max(0.0, 1.0 - e / 0.5) for e in errs) / len(errs) if errs else float("nan")
            ph = "LYING" if b < 0.05 else ("STAND" if b > 0.95 else "RISING")
            _dbg(f"[b2_getup_mimic] t={sim_ms/1000:4.1f}s [{ph:6s}] "
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
