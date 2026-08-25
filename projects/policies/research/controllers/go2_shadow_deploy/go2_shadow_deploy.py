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

"""Unitree Go2 SHADOWING walk deploy controller -- the ghost-lut sibling of
go2_walk_deploy.py (which stays byte-identical for the legacy champion).

Deploys policies trained by quad_walk_recipe.py in SHADOWING mode
(QUAD_GHOST=<lut.json>): the baseline the ONNX residual rides is the CERTIFIED
achieved ghost lut (recorded from the real dynamics and folded on the gait
clock -- build_go2_shadow_ghost.py), NOT the analytic kinematic trot. The
achieved gait sags ~0.075 rad mean from the analytic trot (PD equilibrium under
the gait torques), so deploying a shadow policy on the analytic baseline would
shift the corridor centre off-training -- hence this sibling.

Obs/action contract == go2_walk_deploy == the trainer, UNCHANGED (48-dim obs,
+-GO2_ACT_SCALE rad residual; GO2_ACT_SCALE must equal the training corridor
QUAD_RES_SCALE). Differences, mirrored 1:1 from the trainer's ghost mode:

  * baseline = nominal + r * (lut(phase) - nominal), r = stride ramp
    clamp(t/ramp_s) (the trainer's _ghost_center); phase = QS_PHASE + omega*t,
    omega from the lut's OWN freq.
  * GO2_GHOST_FF=1 additionally shifts the COMMAND centre by r * ffdq(phase)
    (the ghost's declared feedforward, GHOST-FF; must match training).
  * yaw steering: the analytic trot's stance-sweep DELTA
    (targets(wz) - targets(0)) is added on top of the lut baseline -- the same
    model-layer steering mechanism the legacy deploy uses (zero at wz=0, so
    training parity holds; training runs wz_cmd=0 throughout).
  * logs GMATCH (ghost similarity: exp(-mean((q - ref_pose)^2)/sig^2), the
    trainer's eval metric, sig=GO2_GHOST_SIG=0.35) per second + running mean.

Env: GO2_GHOST_LUT=<lut.json> (REQUIRED), GO2_GHOST_FF, GO2_GHOST_SIG, plus the
whole GO2_* set of go2_walk_deploy (gait envs are used for the ramp/steer/
standing pose and must match the lut's own gait block).
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))  # lowest priority: don't shadow runtime `import omnisim`

from projects.policies.control.gait import go2_trot_gait as stg  # noqa: E402
from projects.policies.control.omniquad_motor_safety import (  # noqa: E402
    apply_realistic_limits, RateLimitedMotorBank,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JOINT_NAMES = [f"{leg}_{part}_joint" for leg in LEGS for part in PARTS]

OBS_DIM = 48
ACT_DIM = 12
NJ = 12
JOINT_LIMITS_LO = np.array([-1.0472, -0.5236, -2.7227] * 4, dtype=np.float32)
JOINT_LIMITS_HI = np.array([+1.0472, +3.1316, -0.83776] * 4, dtype=np.float32)


def _env_float(key: str, default: float) -> float:
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _lut_interp(table: np.ndarray, phase: float, nb: int) -> np.ndarray:
    """Circular linear interpolation -- the trainer's _lut, scalar form."""
    x = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * nb
    b0 = int(math.floor(x)) % nb
    b1 = (b0 + 1) % nb
    f = x - math.floor(x)
    return table[b0] * (1.0 - f) + table[b1] * f


def main() -> int:
    side_log_path = os.environ.get("GO2_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg):
        try:
            sys.stderr.write(msg); sys.stderr.flush()
        except Exception:
            pass
        if side_log is not None:
            try:
                side_log.write(msg); side_log.flush()
            except Exception:
                pass

    # ── the GHOST lut (required) ──
    lut_path = os.environ.get("GO2_GHOST_LUT")
    if not lut_path or not Path(lut_path).exists():
        say(f"[go2_shadow_deploy] GO2_GHOST_LUT missing/not found: {lut_path!r}\n")
        return 1
    gd = json.loads(open(lut_path).read())
    exp = JOINT_NAMES
    # `joints` = the REAL model names (the old humanoid-alias hack is gone; `joints_go2`
    # is the back-compat fallback for a stale lut).
    got = list(gd.get("joints") or gd.get("joints_go2") or [])
    if got != exp:
        say(f"[go2_shadow_deploy] lut joint-order mismatch: {got} != {exp}\n")
        return 1
    nb = int(gd["nb"])
    leg_lut = np.asarray(gd["leg_lut"], dtype=np.float32)
    ffdq_lut = None
    if os.environ.get("GO2_GHOST_FF", "").strip() == "1" and "ffdq_lut" in gd:
        ffdq_lut = np.asarray(gd["ffdq_lut"], dtype=np.float32)
    gsig = _env_float("GO2_GHOST_SIG", 0.35)

    # ── Gait model: used for the standing pose, ramp, and steer delta ──
    gp = stg.GaitParams(
        vx=_env_float("GO2_GAIT_VX", 0.4),
        freq=_env_float("GO2_GAIT_FREQ", float(gd.get("freq", 1.8))),
        duty=_env_float("GO2_GAIT_DUTY", 0.6),
        step_height=_env_float("GO2_GAIT_STEP_H", 0.05),
        body_height=_env_float("GO2_GAIT_BODY_H", 0.30),
        x0=_env_float("GO2_GAIT_X0", 0.0),
        ramp_s=_env_float("GO2_GAIT_RAMP_S", 1.0))
    res_scale = _env_float("GO2_ACT_SCALE", 0.15)
    nominal = stg.standing_pose(gp).astype(np.float32)
    omega = 2.0 * math.pi * float(gd.get("freq", gp.freq))   # lut clock == the clock
    say(f"[go2_shadow_deploy] GHOST={Path(lut_path).name} nb={nb} "
        f"freq={gd.get('freq')}Hz vx_ref={gd.get('vx')} corridor={res_scale} "
        f"ff={'ON' if ffdq_lut is not None else 'off'} sig={gsig}\n")

    policy_path = Path(os.environ.get("GO2_POLICY_ONNX") or "")
    sess = None
    if policy_path.exists():
        try:
            import onnxruntime as ort
            # single-threaded on purpose: multi-threaded CPU reductions sum in
            # nondeterministic order, and this policy is small enough that one
            # thread costs nothing. Keeps the same checkpoint bit-stable across
            # runs and machines (the cross-machine determinism plan, 2026-07-16).
            so = ort.SessionOptions()
            so.intra_op_num_threads = 1
            so.inter_op_num_threads = 1
            sess = ort.InferenceSession(str(policy_path), sess_options=so,
                                        providers=["CPUExecutionProvider"])
            say(f"[go2_shadow_deploy] ONNX loaded: {policy_path}\n")
        except Exception as e:
            # FATAL -- see go2_walk_deploy for the full rationale. A ghost LUT
            # replays well enough on its own (it is a certified achievable
            # reference) that a zero-residual run looks like a GOOD result: it
            # walks, it does not fall, and it scores a near-ceiling gmatch
            # because it is literally the ghost. That is the trap. If the policy
            # cannot load, say so and stop.
            say(f"[go2_shadow_deploy] FATAL: ONNX policy exists but failed to "
                f"load ({e}).\n")
            say(f"[go2_shadow_deploy] path: {policy_path}\n")
            say("[go2_shadow_deploy] refusing to run the bare ghost and report "
                "it as a Shadowing-policy result. Fix the controller "
                "interpreter's deps (pip install onnxruntime).\n")
            raise SystemExit(2)
    else:
        say(f"[go2_shadow_deploy] policy not found at {policy_path}; "
            "running the BARE ghost baseline\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[go2_shadow_deploy] missing motor {jn}_motor\n")
            return 1
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    apply_realistic_limits(motors,
                           max_torque_nm=_env_float("GO2_MAX_TORQUE_NM", 50.0),
                           max_vel_rad_s=_env_float("GO2_MAX_VEL_RAD_S", 30.0))
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("GO2_TARGET_RATE_RAD_S", 1e6))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    motor_bank.set_pose(nominal.tolist())

    # Settle at the standing pose (the trainer's rest-start state).
    settle = max(1, int(_env_float("GO2_SETTLE_S", 1.5) / step_dt))
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            return 0

    wz_user = _env_float("GO2_WZ", 0.0)
    HOLD = (os.environ.get("GO2_HEADING_HOLD", "1").strip() != "0")
    HOLD_KP_YAW = _env_float("GO2_HOLD_KP_YAW", 1.0)
    HOLD_KP_LAT2YAW = _env_float("GO2_HOLD_KP_LAT2YAW", 0.3)
    HOLD_WZ_MAX = _env_float("GO2_HOLD_WZ_MAX", 0.3)
    yaw_ref = None
    lat_ref = 0.0
    wz_cmd = wz_user

    # walk<->stand schedule (kept for parity with go2_walk_deploy)
    VX_NOMINAL = _env_float("GO2_GAIT_VX", 0.4)
    VX_CMD_MAX = _env_float("GO2_VX_CMD_MAX", 0.0)
    WALK_FOR_S = _env_float("GO2_WALK_FOR_S", 0.0)
    STAND_FOR_S = _env_float("GO2_STAND_FOR_S", 5.0)
    STAND_AT_S = _env_float("GO2_STAND_AT_S", 0.0)
    MODE_BLEND_S = _env_float("GO2_MODE_BLEND_S", 1.0)
    _w = 1.0
    _ctrl_t0 = None

    def _want_stand(elapsed):
        if WALK_FOR_S > 0.0:
            period = WALK_FOR_S + STAND_FOR_S
            return (elapsed % period) >= WALK_FOR_S
        if STAND_AT_S > 0.0:
            return STAND_AT_S <= elapsed < (STAND_AT_S + STAND_FOR_S)
        return False

    gait_t = 0.0
    sim_ms = 0
    last_action = np.zeros(ACT_DIM, dtype=np.float32)
    last_q = None
    fall_logged = False
    gm_sum, gm_n = 0.0, 0

    while robot.step(step_ms) != -1:
        sim_ms += step_ms

        _now_s = sim_ms / 1000.0
        if _ctrl_t0 is None:
            _ctrl_t0 = _now_s
        _elapsed = _now_s - _ctrl_t0
        _w_tgt = 0.0 if _want_stand(_elapsed) else 1.0
        _dw = step_dt / max(MODE_BLEND_S, 1e-3)
        _w = _w + max(-_dw, min(_dw, _w_tgt - _w))
        s_speed = _w

        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0, 0, 0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                vel = self_node.getVelocity() or [0] * 6
            except Exception:
                pos = [0, 0, 0]; ori = [1, 0, 0, 0, 1, 0, 0, 0, 1]; vel = [0] * 6
        else:
            pos = [0, 0, 0]; ori = [1, 0, 0, 0, 1, 0, 0, 0, 1]; vel = [0] * 6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))

        if HOLD and abs(wz_user) < 1e-6:
            _yaw_now = math.atan2(ori[3], ori[0])
            if yaw_ref is None:
                yaw_ref = _yaw_now
                lat_ref = by
            _dyaw = _yaw_now - yaw_ref
            while _dyaw > math.pi:
                _dyaw -= 2 * math.pi
            while _dyaw < -math.pi:
                _dyaw += 2 * math.pi
            wz_cmd = max(-HOLD_WZ_MAX, min(
                HOLD_WZ_MAX,
                -HOLD_KP_YAW * _dyaw - HOLD_KP_LAT2YAW * (by - lat_ref)))

        if not fall_logged and (bz < 0.18 or abs(roll) > 0.8 or abs(pitch) > 0.8):
            say(f"FALL@{sim_ms / 1000.0:.2f}s bz={bz:.2f} roll={roll:.2f} "
                f"pitch={pitch:.2f} x={bx:+.2f}\n")
            fall_logged = True

        # ── 48-dim obs, trainer layout ──
        q = np.zeros(NJ, dtype=np.float32)
        for i, s in enumerate(sensors):
            if s is None:
                continue
            try:
                q[i] = s.getValue()
            except Exception:
                q[i] = 0.0
        if last_q is None:
            last_q = q.copy()
        qd = (q - last_q) / step_dt
        last_q = q.copy()

        v_lin = np.array(vel[:3], dtype=np.float32)
        _R = np.array(ori, dtype=np.float32).reshape(3, 3)
        v_ang = (_R.T @ np.array(vel[3:6], dtype=np.float32)).astype(np.float32)
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        phase = stg.QS_PHASE + omega * gait_t
        gait_obs = np.array([math.sin(phase), math.cos(phase)], dtype=np.float32)

        obs = np.concatenate([v_lin, v_ang, proj_g, q - nominal, qd,
                              last_action, gait_obs,
                              [np.float32(wz_cmd)]]).astype(np.float32)
        if VX_CMD_MAX > 0.0:
            obs = np.concatenate(
                [obs, [np.float32(s_speed * VX_NOMINAL / VX_CMD_MAX)]]
            ).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0),
                      -10.0, 10.0)

        # ── Residual ──
        if sess is not None:
            try:
                action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
                action = np.clip(action, -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[go2_shadow_deploy] inference failed: {e}\n")
                action = np.zeros(ACT_DIM, dtype=np.float32)
        else:
            action = np.zeros(ACT_DIM, dtype=np.float32)

        # ── Model layer: GHOST lut baseline (the trainer's _ghost_center) ──
        r_ramp = min(1.0, max(0.0, gait_t / gp.ramp_s)) if gp.ramp_s > 0 else 1.0
        ref_pose = nominal + r_ramp * (
            _lut_interp(leg_lut, phase, nb) - nominal)          # the POSE (scored)
        q_model = ref_pose.copy()                               # the COMMAND centre
        if ffdq_lut is not None:
            q_model = q_model + r_ramp * _lut_interp(ffdq_lut, phase, nb)
        # yaw-steer delta from the analytic model (zero at wz=0 -> training parity)
        if abs(wz_cmd) > 1e-9:
            t_l, _ = stg.targets_np(phase, gp, t_since_start=gait_t, wz=wz_cmd)
            t_0, _ = stg.targets_np(phase, gp, t_since_start=gait_t, wz=0.0)
            q_model = q_model + (t_l - t_0).astype(np.float32)
        # walk<->stand blend (parity with the legacy deploy; s_speed=1 normally)
        q_model = nominal + s_speed * (q_model - nominal)

        # GMATCH: achieved pose vs the reference POSE (never the ff-shifted command)
        gm = math.exp(-float(np.mean((q - ref_pose) ** 2)) / (gsig * gsig))
        gm_sum += gm; gm_n += 1

        if sim_ms % 1000 < step_ms:
            say(f"[t={sim_ms / 1000.0:.0f}s] x={bx:+.2f} y={by:+.2f} "
                f"bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f} "
                f"vx={float(vel[0]):+.2f} gm={gm:.3f} gmavg={gm_sum / max(gm_n, 1):.3f}\n")

        q_cmd = np.clip(q_model.astype(np.float32) + action * res_scale,
                        JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        last_action = action
        motor_bank.set_pose(q_cmd.tolist())

        gait_t += step_dt

    say(f"GMATCH FINAL mean={gm_sum / max(gm_n, 1):.3f} over {gm_n} ticks\n")
    if side_log is not None:
        side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
