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

"""OmniQuad SHADOWING walk deploy -- the ghost-lut sibling of
omniquad_walk_deploy.py (which stays byte-identical for the legacy champion).

The Go2 shadow deploy, ported to a SECOND quadruped. Everything robot-specific
is in three places -- the joint names, the joint limits, and the gait module --
and NOTHING else had to change. That is the claim this file exists to support:
Shadowing is a method, not a Go2 script.

Deploys policies trained by quad_walk_recipe.py in SHADOWING mode
(QUAD_GHOST=<lut.json>): the baseline the ONNX residual rides is the CERTIFIED
achieved ghost lut (recorded from the real dynamics and folded on the gait
clock), NOT the analytic trot of omniquad_trot_gait.py. The achieved gait SAGS from
the analytic reference (it is the PD equilibrium under the gait torques, not the
kinematic ideal), so deploying a shadow policy on the analytic baseline would
shift the corridor centre off-training -- hence this sibling rather than a flag
on omniquad_walk_deploy.

Obs/action contract == omniquad_walk_deploy == the trainer, UNCHANGED (48-dim obs,
+-OMNIQUAD_ACT_SCALE rad residual; OMNIQUAD_ACT_SCALE must equal the training corridor
QUAD_RES_SCALE). Differences, mirrored 1:1 from the trainer's ghost mode:

  * baseline = nominal + r * (lut(phase) - nominal), r = stride ramp
    clamp(t/ramp_s) (the trainer's _ghost_center); phase = QS_PHASE + omega*t,
    omega from the lut's OWN freq (the lut carries its clock; the gait env must
    agree with it or the reference walks at a different cadence than it was
    recorded at).
  * OMNIQUAD_GHOST_FF=1 additionally shifts the COMMAND centre by r * ffdq(phase)
    (the ghost's declared feedforward, GHOST-FF). ⭐ The corridor law: a ghost
    recorded under torque tau_ff is UNTRACKABLE by a pure position PD unless the
    corridor exceeds tau_ff/kp -- so either the corridor is wide enough or the
    feedforward is replayed. It MUST match training either way.
  * yaw steering: the analytic trot's stance-sweep DELTA
    (targets(wz) - targets(0)) is added on top of the lut baseline -- the same
    model-layer steering mechanism the legacy deploy uses (exactly zero at wz=0,
    so training parity holds; training runs wz_cmd=0 throughout). The heading
    hold from omniquad_walk_deploy drives wz_cmd through that channel.
  * logs GMATCH (ghost similarity: exp(-mean((q - ref_pose)^2)/sig^2), the
    trainer's eval metric, sig=OMNIQUAD_GHOST_SIG=0.35) per second + running mean.

⛔ GMATCH IS A POSE METRIC AND CANNOT SEE THAT YOU ARE UPSIDE DOWN. A Go2 that
flipped onto its back and kept cycling its legs still scored gmatch 0.92. Read
the FALL line and the x-progress, never gmatch alone.

Env: OMNIQUAD_GHOST_LUT=<lut.json> (REQUIRED), OMNIQUAD_GHOST_FF, OMNIQUAD_GHOST_SIG,
OMNIQUAD_POLICY_ONNX, plus the whole OMNIQUAD_* set of omniquad_walk_deploy (the gait envs
drive the ramp/steer/standing pose and must match the lut's own gait block).
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
# Append (lowest priority): the repo root enables `from projects.policies...`
# WITHOUT shadowing the controller runtime's `import omnisim` with the repo's own
# omnisim/ CLI package.
sys.path.append(str(_REPO))

# Gait reference: trot by default; OMNIQUAD_GAIT_MODULE selects another -- MUST match
# the module the ghost was recorded on.
_GM = os.environ.get("OMNIQUAD_GAIT_MODULE")
if _GM:
    import importlib
    stg = importlib.import_module(_GM)
else:
    from projects.policies.control.gait import omniquad_trot_gait as stg  # noqa: E402
from projects.policies.control.omniquad_motor_safety import (  # noqa: E402
    apply_realistic_limits, RateLimitedMotorBank,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


# ⛔ OMNIQUAD'S JOINT NAMES HAVE NO "_joint" SUFFIX (the Go2's do: FL_hip_joint).
# The device name is "<name>_motor", so the Go2's f"{leg}_{part}_joint_motor"
# spelling would silently find NO motors here.
URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
PARTS = ("hip_x", "hip_y", "knee")
JOINT_NAMES = [f"{leg}_{part}" for leg in URDF_LEGS for part in PARTS]

OBS_DIM = 48
ACT_DIM = 12
NJ = 12
JOINT_LIMITS_LO = np.array([-1.50, -0.50, -1.20] * 4, dtype=np.float32)
JOINT_LIMITS_HI = np.array([+1.50, +3.13, -0.01] * 4, dtype=np.float32)


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
    side_log_path = os.environ.get("OMNIQUAD_DEPLOY_LOG") or os.environ.get(
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
    lut_path = os.environ.get("OMNIQUAD_GHOST_LUT")
    if not lut_path or not Path(lut_path).exists():
        say(f"[omniquad_shadow_deploy] OMNIQUAD_GHOST_LUT missing/not found: {lut_path!r}\n")
        return 1
    gd = json.loads(open(lut_path).read())
    got = list(gd.get("joints") or [])
    if got != JOINT_NAMES:
        # A positional fallback would "work" and score well while every joint
        # tracked the wrong reference -- the in-engine name-matching trap that
        # already cost one campaign. Refuse instead.
        say(f"[omniquad_shadow_deploy] lut joint-order mismatch: {got} != {JOINT_NAMES}\n")
        return 1
    nb = int(gd["nb"])
    leg_lut = np.asarray(gd["leg_lut"], dtype=np.float32)
    ffdq_lut = None
    if os.environ.get("OMNIQUAD_GHOST_FF", "").strip() == "1" and "ffdq_lut" in gd:
        ffdq_lut = np.asarray(gd["ffdq_lut"], dtype=np.float32)
    gsig = _env_float("OMNIQUAD_GHOST_SIG", 0.35)

    # ── Gait model: used for the standing pose, the stride ramp, and the steer
    #    delta. These MUST equal the lut's own `gait` block (and the training
    #    run's): the lut is a phase-folded recording, so a different duty/body_h
    #    means a different nominal, which shifts the whole corridor centre.
    gp = stg.GaitParams(
        vx=_env_float("OMNIQUAD_GAIT_VX", 0.4),
        freq=_env_float("OMNIQUAD_GAIT_FREQ", float(gd.get("freq", 1.4))),
        duty=_env_float("OMNIQUAD_GAIT_DUTY", 0.6),
        step_height=_env_float("OMNIQUAD_GAIT_STEP_H", 0.06),
        body_height=_env_float("OMNIQUAD_GAIT_BODY_H", 0.55),
        x0=_env_float("OMNIQUAD_GAIT_X0", 0.0),
        ramp_s=_env_float("OMNIQUAD_GAIT_RAMP_S", 1.0))
    res_scale = _env_float("OMNIQUAD_ACT_SCALE", 0.15)
    nominal = stg.standing_pose(gp).astype(np.float32)
    omega = 2.0 * math.pi * float(gd.get("freq", gp.freq))   # lut clock == the clock
    say(f"[omniquad_shadow_deploy] GHOST={Path(lut_path).name} nb={nb} "
        f"freq={gd.get('freq')}Hz vx_ref={gd.get('vx')} corridor={res_scale} "
        f"ff={'ON' if ffdq_lut is not None else 'off'} sig={gsig}\n")
    say(f"[omniquad_shadow_deploy] gait vx={gp.vx} freq={gp.freq} duty={gp.duty} "
        f"step_h={gp.step_height} body_h={gp.body_height}\n")

    policy_path = Path(os.environ.get("OMNIQUAD_POLICY_ONNX")
                       or os.environ.get("OMNISIM_POLICY_ONNX") or "")
    sess = None
    if policy_path.exists():
        try:
            import onnxruntime as ort
            # single-threaded on purpose: multi-threaded CPU reductions sum in
            # nondeterministic order; one thread keeps the same checkpoint
            # bit-stable across runs and machines at zero cost for a net this size.
            so = ort.SessionOptions()
            so.intra_op_num_threads = 1
            so.inter_op_num_threads = 1
            sess = ort.InferenceSession(str(policy_path), sess_options=so,
                                        providers=["CPUExecutionProvider"])
            say(f"[omniquad_shadow_deploy] ONNX loaded: {policy_path}\n")
        except Exception as e:
            # FATAL, deliberately. The policy file is RIGHT THERE and would not
            # load -- classically: onnxruntime missing from the CONTROLLER
            # interpreter, which is a DIFFERENT python than the one the engine
            # embeds. Degrading to zero residual keeps the robot moving on the
            # ghost baseline and still exits 0, so the run reports PASS while the
            # policy under test never ran. And a ghost lut replays well enough on
            # its own (it IS a certified achievable reference) that a bare run
            # LOOKS like a good result: it walks, it does not fall, and it scores
            # a near-ceiling gmatch because it is literally the ghost. That trap
            # silently voided an entire Go2 head-to-head on 2026-07-12 -- and the
            # broken run looked BETTER than the real one.
            say(f"[omniquad_shadow_deploy] FATAL: ONNX policy exists but failed to "
                f"load ({e}).\n")
            say(f"[omniquad_shadow_deploy] path: {policy_path}\n")
            say("[omniquad_shadow_deploy] refusing to run the bare ghost and report "
                "it as a Shadowing-policy result. Install onnxruntime for the "
                "CONTROLLER interpreter (the python that runs this file), not "
                "just the engine's.\n")
            raise SystemExit(2)
    else:
        # The deliberate bare-ghost path (run_omniquad_shadow_deploy.sh --bare). It is
        # an ABLATION, not a result: read the "BARE" banner in the log before you
        # quote any number out of this run.
        say(f"[omniquad_shadow_deploy] policy not found at {policy_path}; "
            "running the BARE ghost baseline\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[omniquad_shadow_deploy] missing motor {jn}_motor\n")
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
                           max_torque_nm=_env_float("OMNIQUAD_MAX_TORQUE_NM", 80.0),
                           max_vel_rad_s=_env_float("OMNIQUAD_MAX_VEL_RAD_S", 20.0))
    # No slew by default: the trainer applies targets instantly each tick.
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("OMNIQUAD_TARGET_RATE_RAD_S", 1e6))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    motor_bank.set_pose(nominal.tolist())

    # Settle at the standing pose (the trainer's rest-start state).
    settle = max(1, int(_env_float("OMNIQUAD_SETTLE_S", 1.5) / step_dt))
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            return 0

    # ── Heading hold through the command channel (omniquad_walk_deploy's, verbatim).
    # The trainer steers the gait model by tangential stance sweep and tracks
    # wz_cmd, so deploy holds heading by writing wz_cmd from a yaw PD each tick.
    # The lateral term (lat2yaw) is what kept the legacy champion dead straight
    # over +119 m; without it the robot holds HEADING but still crabs sideways.
    wz_user = _env_float("OMNIQUAD_WZ", 0.0)
    HOLD = (os.environ.get("OMNIQUAD_HEADING_HOLD", "1").strip() != "0")
    HOLD_KP_YAW = _env_float("OMNIQUAD_HOLD_KP_YAW", 1.0)
    HOLD_KP_LAT2YAW = _env_float("OMNIQUAD_HOLD_KP_LAT2YAW", 0.3)
    HOLD_WZ_MAX = _env_float("OMNIQUAD_HOLD_WZ_MAX", 0.3)
    yaw_ref = None
    lat_ref = 0.0
    wz_cmd = wz_user

    # walk<->stand schedule (kept for parity with omniquad_walk_deploy; the BATON
    # controller is the real way to sequence modes -- this is the single-policy
    # velocity-conditioned blend, not a hand-over).
    VX_NOMINAL = _env_float("OMNIQUAD_GAIT_VX", 0.4)
    VX_CMD_MAX = _env_float("OMNIQUAD_VX_CMD_MAX", 0.0)      # >0 = VC policy (obs+1)
    WALK_FOR_S = _env_float("OMNIQUAD_WALK_FOR_S", 0.0)
    STAND_FOR_S = _env_float("OMNIQUAD_STAND_FOR_S", 5.0)
    STAND_AT_S = _env_float("OMNIQUAD_STAND_AT_S", 0.0)
    MODE_BLEND_S = _env_float("OMNIQUAD_MODE_BLEND_S", 1.0)
    _w = 1.0
    _ctrl_t0 = None

    def _want_stand(elapsed):
        if WALK_FOR_S > 0.0:
            period = WALK_FOR_S + STAND_FOR_S
            return (elapsed % period) >= WALK_FOR_S
        if STAND_AT_S > 0.0:
            return STAND_AT_S <= elapsed < (STAND_AT_S + STAND_FOR_S)
        return False

    gait_t = 0.0            # time since gait start (drives the stride ramp)
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

        # OmniQuad stands at body_height 0.55 (the Go2 at 0.30) -- the fall floor is
        # omniquad_walk_deploy's 0.30, not the Go2's 0.18.
        if not fall_logged and (bz < 0.30 or abs(roll) > 0.8 or abs(pitch) > 0.8):
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
        # Trainer vang = MuJoCo free-joint qvel angular = BODY frame.
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
                say(f"[omniquad_shadow_deploy] inference failed: {e}\n")
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

        # GMATCH: achieved pose vs the reference POSE (never the ff-shifted command
        # -- scoring against the command would credit the feedforward offset as
        # tracking error and quietly change the ruler).
        gm = math.exp(-float(np.mean((q - ref_pose) ** 2)) / (gsig * gsig))
        gm_sum += gm; gm_n += 1

        if sim_ms % 1000 < step_ms:
            say(f"[t={sim_ms / 1000.0:.0f}s] x={bx:+.2f} y={by:+.2f} "
                f"bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f} "
                f"vx={float(vel[0]):+.2f} gm={gm:.3f} "
                f"gmavg={gm_sum / max(gm_n, 1):.3f}\n")

        q_cmd = np.clip(q_model.astype(np.float32) + action * res_scale,
                        JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        last_action = action
        motor_bank.set_pose(q_cmd.tolist())

        gait_t += step_dt   # advance AFTER use: first tick == rest-start

    say(f"GMATCH FINAL mean={gm_sum / max(gm_n, 1):.3f} over {gm_n} ticks\n")
    if side_log is not None:
        side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
