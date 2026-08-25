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

"""Unitree Go2 WALK deploy controller -- the OmniQuad walk-deploy recipe ported.

Runs the foot-space TROT MODEL (projects/policies/control/gait/go2_trot_gait.py) as the
baseline and adds the ONNX residual trained by
gpu_mjwarp_go2_walk_trainer.py. The obs/action contract mirrors that
trainer EXACTLY:

  * 48-dim obs: vlin_world(3), vang_BODY(3), proj_gravity(3),
    q - NOMINAL(12), qd finite-diff(12), last_action(12),
    gait phase sin/cos(2), wz command(1).
  * HEADING HOLD through the command channel (GO2_HEADING_HOLD=0 disables).
  * +-GO2_ACT_SCALE rad joint-space residual on the model targets.
  * NOMINAL = the gait model's standing pose; the clock starts at QS_PHASE
    (all four feet planted) and the stride RAMPS in from zero.

Go2 motors are imported as "<joint>_motor" with joint names
FL/FR/RL/RR_{hip,thigh,calf}_joint. Gait params are env-configurable
(GO2_GAIT_*) so one controller deploys any policy from that trainer -- set
them to the TRAINING values.

Deploy env (set by scripts/dev/run_go2_walk_deploy.ps1):
    OMNISIM_NEWTON_TARGET_KE=250 KD=6   <-> trainer MJCF kp/kv (MATCHED)
    OMNISIM_NEWTON_SUBSTEPS=8           <-> trainer 8 x 2 ms decimation
    OMNISIM_NEWTON_MJWARP=1             <-> trainer engine
(This block previously read "KE=80 KD=2.0 (MATCHED)", which was WRONG and cost a
mis-trained run: the gains are baked into the MJCF the champion was trained on --
research/training/mjcf/go2_newton.xml has kp=250/kv=6, and run_go2_walk_deploy.ps1
+ run_quad_rough_track.ps1 both correctly set 250/6. Read the MJCF, not a comment.)
"""
from __future__ import annotations

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
# Go2 URDF joint names (controller order FL,FR,RL,RR x hip,thigh,calf). The
# importer makes a RotationalMotor "<joint>_motor" + PositionSensor
# "<joint>_sensor" for each.
JOINT_NAMES = [f"{leg}_{part}_joint" for leg in LEGS for part in PARTS]

OBS_DIM = 48
ACT_DIM = 12
NJ = 12
# Match gpu_mjwarp_go2_walk_trainer JOINT_LIMITS (Go2 MJCF ranges).
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


def find_policy_path() -> Path:
    p = os.environ.get("GO2_POLICY_ONNX") or os.environ.get(
        "OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    return (_REPO / "projects" / "rl" / "inference" / "policies"
            / "gpu_go2_walk_main" / "policy.onnx")


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

    # ── Gait model: params MUST match the policy's training run ──
    gp = stg.GaitParams(
        vx=_env_float("GO2_GAIT_VX", 0.4),
        freq=_env_float("GO2_GAIT_FREQ", 1.8),
        duty=_env_float("GO2_GAIT_DUTY", 0.6),
        step_height=_env_float("GO2_GAIT_STEP_H", 0.05),
        body_height=_env_float("GO2_GAIT_BODY_H", 0.30),
        x0=_env_float("GO2_GAIT_X0", 0.0),
        ramp_s=_env_float("GO2_GAIT_RAMP_S", 1.0))
    res_scale = _env_float("GO2_ACT_SCALE", 0.15)
    nominal = stg.standing_pose(gp).astype(np.float32)
    say(f"[go2_walk_deploy] gait vx={gp.vx} freq={gp.freq} duty={gp.duty} "
        f"step_h={gp.step_height} body_h={gp.body_height} "
        f"res_scale={res_scale}\n")

    policy_path = find_policy_path()
    sess = None
    if policy_path.exists():
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say(f"[go2_walk_deploy] ONNX loaded: {policy_path}\n")
        except Exception as e:
            # FATAL, deliberately. The policy file is RIGHT THERE and we could
            # not load it -- that is a broken environment (a missing
            # onnxruntime in the controller interpreter is the classic one), not
            # a mode. Degrading to zero residual here would keep the robot
            # walking on the bare gait model and exit 0, so the run reports PASS
            # while the policy under test never ran. Judge nothing on that.
            # To run without a policy on purpose, use --bare (the branch below).
            say(f"[go2_walk_deploy] FATAL: ONNX policy exists but failed to "
                f"load ({e}).\n")
            say(f"[go2_walk_deploy] path: {policy_path}\n")
            say("[go2_walk_deploy] refusing to run with ZERO residual and "
                "report it as a policy result. Fix the controller "
                "interpreter's deps (pip install onnxruntime), or pass --bare "
                "to run the gait model deliberately.\n")
            raise SystemExit(2)
    else:
        say(f"[go2_walk_deploy] policy not found at {policy_path}; "
            "running the BARE gait model\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[go2_walk_deploy] missing motor {jn}_motor\n")
            return 1
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    # Go2 hip/thigh effort 23.7 Nm, calf 45.43 Nm -- the Newton backend already
    # caps per-joint from the URDF; keep the controller cap above all of them so
    # it is a no-op (matching the trainer's per-joint actuatorfrcrange).
    apply_realistic_limits(motors,
                           max_torque_nm=_env_float("GO2_MAX_TORQUE_NM", 50.0),
                           max_vel_rad_s=_env_float("GO2_MAX_VEL_RAD_S", 30.0))
    # No slew by default: the trainer applies targets instantly each tick.
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("GO2_TARGET_RATE_RAD_S", 1e6))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    motor_bank.set_pose(nominal.tolist())

    trace_file = None
    if os.environ.get("GO2_DEPLOY_TRACE") or os.environ.get(
            "OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\go2_walk_deploy.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[go2_walk_deploy] trace -> {trace_path}\n")

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

    # ── Velocity conditioning + the WALK<->STOP schedule (the OmniQuad port) ──
    VX_NOMINAL = _env_float("GO2_GAIT_VX", 0.4)
    VX_CMD_MAX = _env_float("GO2_VX_CMD_MAX", 0.0)      # >0 = VC policy (obs+1)
    WALK_FOR_S = _env_float("GO2_WALK_FOR_S", 0.0)      # 0 = no repeating cycle
    STAND_FOR_S = _env_float("GO2_STAND_FOR_S", 5.0)
    STAND_AT_S = _env_float("GO2_STAND_AT_S", 0.0)      # 0 = no single window
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
    last_bx = 0.0
    last_t_ms = 0
    fall_logged = False
    omega = 2.0 * math.pi * gp.freq

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

        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx_obs = (bx - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            yaw = math.atan2(ori[3], ori[0])
            trace_file.write(f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                             f"{roll:.4f},{pitch:.4f},{yaw:.4f},{vx_obs:.4f}\n")
            last_bx = bx
            last_t_ms = sim_ms
        if not fall_logged and (bz < 0.18 or abs(roll) > 0.8 or abs(pitch) > 0.8):
            say(f"FALL@{sim_ms / 1000.0:.2f}s bz={bz:.2f} roll={roll:.2f} "
                f"pitch={pitch:.2f} x={bx:+.2f}\n")
            fall_logged = True
        if sim_ms % 1000 < step_ms:
            say(f"[t={sim_ms / 1000.0:.0f}s] x={bx:+.2f} y={by:+.2f} "
                f"bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f} "
                f"vx={float(vel[0]):+.2f} w={_w:.2f}\n")

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
        if os.environ.get("GO2_OBS_DEBUG") and sim_ms % 320 == 0:
            say(f"[obsdbg] t={gait_t:.2f} vlin={v_lin.round(3).tolist()} "
                f"vang={v_ang.round(3).tolist()} q-n0_3="
                f"{(q - nominal)[:3].round(3).tolist()}\n")

        # ── Residual ──
        if sess is not None:
            try:
                action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
                action = np.clip(action, -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[go2_walk_deploy] inference failed: {e}\n")
                action = np.zeros(ACT_DIM, dtype=np.float32)
        else:
            action = np.zeros(ACT_DIM, dtype=np.float32)

        # ── Model layer: trot reference at this tick's phase + ramp ──
        q_model, sw_model = stg.targets_np(phase, gp, t_since_start=gait_t,
                                           wz=wz_cmd)
        q_model, _ = stg.speed_scale(q_model, sw_model, nominal, s_speed)
        q_cmd = np.clip(q_model.astype(np.float32) + action * res_scale,
                        JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        last_action = action
        motor_bank.set_pose(q_cmd.tolist())

        gait_t += step_dt

    if trace_file is not None:
        trace_file.close()
    if side_log is not None:
        side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
