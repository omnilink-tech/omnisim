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

"""go2_latency_probe -- go2_shadow_deploy.py with a CONTROL-LOOP DELAY knob.

Instrumented copy of projects/policies/research/controllers/go2_shadow_deploy/
go2_shadow_deploy.py.  The gait, the ghost lut, the ONNX residual, the obs
layout and the command path are IDENTICAL -- the only behavioural changes are:

  1. LAT_TICKS  (env GO2_LAT_TICKS, default 0)
     The controller's SENSED STATE is delayed by N control ticks.  Everything
     the controller reads about the robot -- joint position sensors, base
     position, base orientation, base velocity -- is taken from a FIFO N ticks
     deep.  The gait clock is NOT delayed (a real robot's timer is accurate;
     it is the *sensing* that is stale).  The in-sim joint PD servo is NOT
     delayed either: it keeps running at the physics rate on whatever target
     was last commanded.  That mirrors an RC-servo robot, where the servo has
     its own fast internal loop and the slow outer program only sets targets.

  2. STALL model (env GO2_STALL_P, GO2_STALL_TICKS, GO2_STALL_SEED)
     With probability GO2_STALL_P per control tick, the controller FREEZES for
     GO2_STALL_TICKS ticks: it does not sense, does not infer, and does not
     issue a new command -- the motors hold the last target.  This is the
     jitter/GC-pause model: intermittently slow, not uniformly slow.  The gait
     clock keeps running through a stall, so on resume the commanded phase has
     jumped forward, exactly as it would for a program that lost the CPU.

  3. JOINT_FB (env GO2_JOINT_FB: "measured" | "cmd" | "frozen")
     Does the policy get real MOTOR POSITION FEEDBACK?  "measured" is the
     shipped behaviour (the delayed joint position sensors).  "cmd" removes the
     measurement entirely and feeds the policy the controller's own last ISSUED
     target instead -- the open-loop hobby-servo model, where nothing reports
     back and the only available estimate is "it probably went where I told
     it".  "frozen" kills the channel outright (the pose observed at the start
     of walking, held forever).  The base/IMU channel is untouched in all three.

  4. ACT_HZ (env GO2_ACT_HZ, default 0 = off)
     Command-path quantisation.  The actuation bus latches at most ACT_HZ new
     targets per second on a fixed grid; commands produced between grid points
     are DROPPED and the motors hold the last latched target.  The in-sim joint
     PD servo keeps running at the physics substep on that held target.

  5. TRACE (env GO2_TRACE)
     Per-control-tick CSV of GROUND TRUTH (never the delayed copy): sim time,
     base x/y/z, roll/pitch/yaw, base linear velocity, and a per-foot contact
     flag derived from Supervisor getContactPoints(includeDescendants=True).
     The trace is the measurement instrument and is deliberately outside the
     delay, so a delayed run is still measured honestly.

Everything else -- env var names, gait params, corridor, FF handling, heading
hold, fall criterion -- is copied verbatim from go2_shadow_deploy.py.
"""
from __future__ import annotations

import json
import math
import os
import random
import sys
from collections import deque
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


def _env_int(key: str, default: int) -> int:
    return int(round(_env_float(key, float(default))))


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

    trace_path = os.environ.get("GO2_TRACE")
    trace = open(trace_path, "w", buffering=1) if trace_path else None
    if trace is not None:
        trace.write("t_s,x,y,z,roll,pitch,yaw,vx,vy,vz,c_FL,c_FR,c_RL,c_RR,"
                    "n_contacts,stalled\n")

    # -- the GHOST lut (required) --
    lut_path = os.environ.get("GO2_GHOST_LUT")
    if not lut_path or not Path(lut_path).exists():
        say(f"[go2_latency_probe] GO2_GHOST_LUT missing/not found: {lut_path!r}\n")
        return 1
    gd = json.loads(open(lut_path).read())
    exp = JOINT_NAMES
    got = list(gd.get("joints") or gd.get("joints_go2") or [])
    if got != exp:
        say(f"[go2_latency_probe] lut joint-order mismatch: {got} != {exp}\n")
        return 1
    nb = int(gd["nb"])
    leg_lut = np.asarray(gd["leg_lut"], dtype=np.float32)
    ffdq_lut = None
    if os.environ.get("GO2_GHOST_FF", "").strip() == "1" and "ffdq_lut" in gd:
        ffdq_lut = np.asarray(gd["ffdq_lut"], dtype=np.float32)
    gsig = _env_float("GO2_GHOST_SIG", 0.35)

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
    omega = 2.0 * math.pi * float(gd.get("freq", gp.freq))

    # -- the delay knobs --
    LAT_TICKS = max(0, _env_int("GO2_LAT_TICKS", 0))
    STALL_P = _env_float("GO2_STALL_P", 0.0)
    STALL_TICKS = max(0, _env_int("GO2_STALL_TICKS", 0))
    STALL_SEED = _env_int("GO2_STALL_SEED", 12345)
    rng = random.Random(STALL_SEED)
    TRUE_DT = os.environ.get("GO2_TRUE_DT", "").strip() == "1"
    # -- OPEN-LOOP arm: what does the policy get told about the JOINTS? --
    #   "measured" (default) -- the (delayed) joint position sensors, as shipped.
    #   "cmd"      -- NO joint measurement exists.  The controller feeds the
    #                 policy its OWN last commanded joint pose instead, i.e. it
    #                 assumes the servo reached its target.  This is the honest
    #                 model of a hobby-servo machine: the estimate is perfectly
    #                 fresh but may be wrong, because nothing reports back.
    #   "frozen"   -- the joint channel is dead: the policy is fed the pose it
    #                 observed at the start of walking, forever.
    JOINT_FB = (os.environ.get("GO2_JOINT_FB", "measured").strip().lower()
                or "measured")
    if JOINT_FB not in ("measured", "cmd", "frozen"):
        say(f"[go2_latency_probe] bad GO2_JOINT_FB={JOINT_FB!r}\n")
        return 1
    # -- COMMAND-PATH QUANTISATION: the actuation bus accepts at most ACT_HZ
    #    new targets per second.  Commands produced between grid points are
    #    DROPPED; the motors hold the last target that made it through.  The
    #    in-sim PD servo still runs at the physics substep on that held target.
    ACT_HZ = _env_float("GO2_ACT_HZ", 0.0)
    # Fixed SIM-time budget so every run in a sweep is comparable in sim seconds,
    # not in wall seconds (wall throughput varies with machine load).
    MAX_S = _env_float("GO2_MAX_S", 40.0)
    # Once it has fallen, keep going briefly to confirm it stays down, then stop.
    POST_FALL_S = _env_float("GO2_POST_FALL_S", 2.0)

    say(f"[go2_latency_probe] GHOST={Path(lut_path).name} nb={nb} "
        f"freq={gd.get('freq')}Hz corridor={res_scale} "
        f"ff={'ON' if ffdq_lut is not None else 'off'}\n")
    say(f"[go2_latency_probe] LAT_TICKS={LAT_TICKS} STALL_P={STALL_P} "
        f"STALL_TICKS={STALL_TICKS} SEED={STALL_SEED}\n")
    say(f"[go2_latency_probe] JOINT_FB={JOINT_FB} ACT_HZ={ACT_HZ}\n")

    policy_path = Path(os.environ.get("GO2_POLICY_ONNX") or "")
    sess = None
    if policy_path.exists():
        try:
            import onnxruntime as ort
            so = ort.SessionOptions()
            so.intra_op_num_threads = 1
            so.inter_op_num_threads = 1
            sess = ort.InferenceSession(str(policy_path), sess_options=so,
                                        providers=["CPUExecutionProvider"])
            say(f"[go2_latency_probe] ONNX loaded: {policy_path}\n")
        except Exception as e:
            say(f"[go2_latency_probe] FATAL: ONNX policy exists but failed to "
                f"load ({e}).\n")
            raise SystemExit(2)
    else:
        say(f"[go2_latency_probe] FATAL: policy not found at {policy_path}; "
            "refusing to run the bare ghost and call it a policy result\n")
        raise SystemExit(2)

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0
    say(f"[go2_latency_probe] control period = {step_ms} ms "
        f"({1000.0 / step_ms:.2f} Hz); injected delay = "
        f"{LAT_TICKS * step_ms} ms\n")

    motors = []
    sensors = []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[go2_latency_probe] missing motor {jn}_motor\n")
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

    # node-id -> leg prefix, resolved lazily from contact points
    _idname = {}

    def _leg_of(nid):
        if nid in _idname:
            return _idname[nid]
        leg = None
        try:
            n = robot.getFromId(nid)
            nm = ""
            if n is not None:
                try:
                    f = n.getField("name")
                    if f is not None:
                        nm = f.getSFString() or ""
                except Exception:
                    nm = ""
                if not nm:
                    try:
                        nm = n.getDef() or ""
                    except Exception:
                        nm = ""
            for L in LEGS:
                if nm.startswith(L + "_") or nm.startswith(L.lower() + "_"):
                    leg = L
                    break
        except Exception:
            leg = None
        _idname[nid] = leg
        return leg

    motor_bank.set_pose(nominal.tolist())

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

    VX_NOMINAL = _env_float("GO2_GAIT_VX", 0.4)
    VX_CMD_MAX = _env_float("GO2_VX_CMD_MAX", 0.0)
    _w = 1.0

    gait_t = 0.0
    sim_ms = 0
    last_action = np.zeros(ACT_DIM, dtype=np.float32)
    last_q_sensed = None
    fall_logged = False
    fall_t = -1.0
    fall_x = 0.0
    gm_sum, gm_n = 0.0, 0
    peak_tilt = 0.0
    peak_tilt_t = 0.0
    max_x = 0.0
    stall_remaining = 0
    sense_gap = 0
    stall_ticks_total = 0
    stall_events = 0
    ticks = 0
    q_frozen = None
    act_abs_sum = 0.0
    act_abs_max = 0.0
    act_bucket = -1
    act_writes = 0
    act_drops = 0
    last_act_s = 0.0
    max_act_gap_s = 0.0

    # sensing FIFO: holds (q, pos, ori, vel) tuples, LAT_TICKS deep
    fifo = deque()
    q_cmd = nominal.astype(np.float32).copy()
    # the last target that actually reached the motors (== q_cmd unless the
    # ACT_HZ command bus dropped it).  This is what an open-loop controller can
    # honestly believe about its own joints.
    q_issued = nominal.astype(np.float32).copy()

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        ticks += 1
        gait_t += step_dt          # the clock never stalls

        # ---- GROUND TRUTH (measurement instrument; never delayed) ----
        if self_node is not None:
            try:
                t_pos = self_node.getPosition() or [0, 0, 0]
                t_ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                t_vel = self_node.getVelocity() or [0] * 6
            except Exception:
                t_pos = [0, 0, 0]; t_ori = [1, 0, 0, 0, 1, 0, 0, 0, 1]; t_vel = [0] * 6
        else:
            t_pos = [0, 0, 0]; t_ori = [1, 0, 0, 0, 1, 0, 0, 0, 1]; t_vel = [0] * 6
        tbx, tby, tbz = float(t_pos[0]), float(t_pos[1]), float(t_pos[2])
        t_roll = math.atan2(t_ori[7], t_ori[8])
        t_pitch = math.asin(max(-1.0, min(1.0, -t_ori[6])))
        t_yaw = math.atan2(t_ori[3], t_ori[0])
        tilt = max(abs(t_roll), abs(t_pitch))
        if not fall_logged and tilt > peak_tilt:
            peak_tilt = tilt
            peak_tilt_t = sim_ms / 1000.0
        if not fall_logged and tbx > max_x:
            max_x = tbx

        if not fall_logged and (tbz < 0.18 or abs(t_roll) > 0.8 or abs(t_pitch) > 0.8):
            say(f"FALL@{sim_ms / 1000.0:.2f}s bz={tbz:.2f} roll={t_roll:.2f} "
                f"pitch={t_pitch:.2f} x={tbx:+.2f}\n")
            fall_logged = True
            fall_t = sim_ms / 1000.0
            fall_x = tbx

        _t_now = sim_ms / 1000.0
        if _t_now >= MAX_S or (fall_logged and _t_now >= fall_t + POST_FALL_S):
            say(f"BUDGET_END t={_t_now:.2f}s (max_s={MAX_S} fell={int(fall_logged)})\n")
            break

        # ---- true joint positions (for the trace + gmatch) ----
        q_true = np.zeros(NJ, dtype=np.float32)
        for i, s in enumerate(sensors):
            if s is None:
                continue
            try:
                q_true[i] = s.getValue()
            except Exception:
                q_true[i] = 0.0

        # ---- per-foot contact (ground truth) ----
        cflag = {L: 0 for L in LEGS}
        ncp = 0
        if self_node is not None and trace is not None:
            try:
                cps = self_node.getContactPoints(True) or []
                ncp = len(cps)
                for cp in cps:
                    L = _leg_of(cp.getNodeId())
                    if L is not None:
                        cflag[L] = 1
            except Exception:
                ncp = -1

        # ---- the STALL model: lose the CPU for STALL_TICKS ----
        if stall_remaining > 0:
            stall_remaining -= 1
            stall_ticks_total += 1
            sense_gap += 1
            if trace is not None:
                trace.write(f"{sim_ms / 1000.0:.4f},{tbx:.5f},{tby:.5f},{tbz:.5f},"
                            f"{t_roll:.5f},{t_pitch:.5f},{t_yaw:.5f},"
                            f"{float(t_vel[0]):.4f},{float(t_vel[1]):.4f},"
                            f"{float(t_vel[2]):.4f},"
                            f"{cflag['FL']},{cflag['FR']},{cflag['RL']},"
                            f"{cflag['RR']},{ncp},1\n")
            continue          # no sensing, no inference, no new command
        if STALL_TICKS > 0 and STALL_P > 0.0 and rng.random() < STALL_P:
            stall_remaining = STALL_TICKS
            stall_events += 1

        # ---- SENSING, delayed by LAT_TICKS ----
        fifo.append((q_true.copy(), list(t_pos), list(t_ori), list(t_vel)))
        while len(fifo) > LAT_TICKS + 1:
            fifo.popleft()
        if len(fifo) >= LAT_TICKS + 1:
            q, pos, ori, vel = fifo[0]
        else:
            q, pos, ori, vel = fifo[0]      # warm-up: oldest available

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

        # ---- OPEN-LOOP arm: what replaces the joint MEASUREMENT ----
        # NB: the "cmd" estimate is NOT delayed by LAT_TICKS -- a controller
        # always knows what it just commanded.  Only the BASE/IMU channel stays
        # stale.  That is the honest shape of an open-loop servo machine: no
        # joint measurement exists to be late, only to be wrong.
        if JOINT_FB == "cmd":
            q_obs = np.asarray(q_issued, dtype=np.float32).copy()
        elif JOINT_FB == "frozen":
            if q_frozen is None:
                q_frozen = q.copy()
            q_obs = q_frozen
        else:
            q_obs = q
        if last_q_sensed is None:
            last_q_sensed = q_obs.copy()
        # dt used for the joint-velocity finite difference. A fixed-dt
        # implementation (the common case, and what go2_shadow_deploy does)
        # always divides by the NOMINAL period, so after a stall it
        # over-estimates joint velocity by the stall factor. TRUE_DT=1 divides
        # by the time that actually elapsed -- the isolation arm.
        dt_eff = (sense_gap + 1) * step_dt if TRUE_DT else step_dt
        qd = (q_obs - last_q_sensed) / dt_eff
        last_q_sensed = q_obs.copy()
        sense_gap = 0

        v_lin = np.array(vel[:3], dtype=np.float32)
        _R = np.array(ori, dtype=np.float32).reshape(3, 3)
        v_ang = (_R.T @ np.array(vel[3:6], dtype=np.float32)).astype(np.float32)
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        phase = stg.QS_PHASE + omega * gait_t
        gait_obs = np.array([math.sin(phase), math.cos(phase)], dtype=np.float32)

        obs = np.concatenate([v_lin, v_ang, proj_g, q_obs - nominal, qd,
                              last_action, gait_obs,
                              [np.float32(wz_cmd)]]).astype(np.float32)
        if VX_CMD_MAX > 0.0:
            obs = np.concatenate(
                [obs, [np.float32(_w * VX_NOMINAL / VX_CMD_MAX)]]
            ).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0),
                      -10.0, 10.0)

        try:
            action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
            action = np.clip(action, -1.0, 1.0).astype(np.float32)
        except Exception as e:
            say(f"[go2_latency_probe] inference failed: {e}\n")
            action = np.zeros(ACT_DIM, dtype=np.float32)

        r_ramp = min(1.0, max(0.0, gait_t / gp.ramp_s)) if gp.ramp_s > 0 else 1.0
        ref_pose = nominal + r_ramp * (
            _lut_interp(leg_lut, phase, nb) - nominal)
        q_model = ref_pose.copy()
        if ffdq_lut is not None:
            q_model = q_model + r_ramp * _lut_interp(ffdq_lut, phase, nb)
        if abs(wz_cmd) > 1e-9:
            t_l, _ = stg.targets_np(phase, gp, t_since_start=gait_t, wz=wz_cmd)
            t_0, _ = stg.targets_np(phase, gp, t_since_start=gait_t, wz=0.0)
            q_model = q_model + (t_l - t_0).astype(np.float32)

        # GMATCH against the TRUE achieved pose
        gm = math.exp(-float(np.mean((q_true - ref_pose) ** 2)) / (gsig * gsig))
        gm_sum += gm; gm_n += 1
        # How hard is the LEARNED RESIDUAL actually working?  Pure logging; it
        # touches nothing the sim sees.  |action| is in policy units (the
        # corridor is +-1 -> +-GO2_ACT_SCALE rad).
        act_abs_sum += float(np.mean(np.abs(action)))
        act_abs_max = max(act_abs_max, float(np.max(np.abs(action))))

        if sim_ms % 1000 < step_ms:
            say(f"[t={sim_ms / 1000.0:.0f}s] x={tbx:+.2f} y={tby:+.2f} "
                f"bz={tbz:.2f} roll={t_roll:+.2f} pitch={t_pitch:+.2f} "
                f"vx={float(t_vel[0]):+.2f} gm={gm:.3f} "
                f"gmavg={gm_sum / max(gm_n, 1):.3f}\n")

        q_cmd = np.clip(q_model.astype(np.float32) + action * res_scale,
                        JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        last_action = action
        # ---- COMMAND-PATH QUANTISATION ----
        # The bus latches at most ACT_HZ targets per second, on a fixed grid.
        # A command produced between grid points is DROPPED and the motors hold
        # the previous target -- so a new target can go unacted on for up to
        # one grid period (measured as max_act_gap_ms in the SUMMARY).
        _now_s = sim_ms / 1000.0
        if ACT_HZ > 0.0:
            _b = int(_now_s * ACT_HZ)
            if _b != act_bucket:
                act_bucket = _b
                max_act_gap_s = max(max_act_gap_s, _now_s - last_act_s)
                last_act_s = _now_s
                act_writes += 1
                q_issued = q_cmd.copy()
                motor_bank.set_pose(q_cmd.tolist())
            else:
                act_drops += 1
        else:
            max_act_gap_s = max(max_act_gap_s, _now_s - last_act_s)
            last_act_s = _now_s
            act_writes += 1
            q_issued = q_cmd.copy()
            motor_bank.set_pose(q_cmd.tolist())

        if trace is not None:
            trace.write(f"{sim_ms / 1000.0:.4f},{tbx:.5f},{tby:.5f},{tbz:.5f},"
                        f"{t_roll:.5f},{t_pitch:.5f},{t_yaw:.5f},"
                        f"{float(t_vel[0]):.4f},{float(t_vel[1]):.4f},"
                        f"{float(t_vel[2]):.4f},"
                        f"{cflag['FL']},{cflag['FR']},{cflag['RL']},"
                        f"{cflag['RR']},{ncp},0\n")

    say(f"GMATCH FINAL mean={gm_sum / max(gm_n, 1):.3f} over {gm_n} ticks\n")
    say(f"SUMMARY lat_ticks={LAT_TICKS} lat_ms={LAT_TICKS * step_ms} "
        f"true_dt={int(TRUE_DT)} "
        f"ctrl_ms={step_ms} fell={1 if fall_logged else 0} "
        f"fall_t={fall_t:.2f} fall_x={fall_x:.3f} "
        f"max_x={max_x:.3f} peak_tilt_rad={peak_tilt:.4f} "
        f"peak_tilt_t={peak_tilt_t:.2f} "
        f"stall_events={stall_events} stall_ticks={stall_ticks_total} "
        f"ticks={ticks} "
        f"joint_fb={JOINT_FB} act_hz={ACT_HZ:.1f} "
        f"act_writes={act_writes} act_drops={act_drops} "
        f"act_rate_hz={act_writes / max(sim_ms / 1000.0, 1e-9):.2f} "
        f"max_act_gap_ms={max_act_gap_s * 1000.0:.1f} "
        f"res_l1={act_abs_sum / max(gm_n, 1):.4f} res_max={act_abs_max:.4f}\n")
    if trace is not None:
        trace.close()
    if side_log is not None:
        side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
