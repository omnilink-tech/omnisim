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

"""OmniSim deploy controller for the G1 standing PPO policy.

Loads the ONNX policy trained by `projects/policies/research/training/train_g1_stand.py`
and runs it inside an OmniSim world. The observation/action layout must
mirror the training env (`projects/policies/research/envs/g1_stand_env.py`) exactly:

Obs (48-dim float32):
    pelvis_lin_vel(3) + pelvis_ang_vel(3) + proj_gravity(3) +
    joint_q_minus_nominal(13) + joint_qd(13) + last_action(13)

Action (13-dim, [-1,1]):
    Joint-space delta in radians, scaled by ACT_SCALE = 0.3 rad,
    added to NOMINAL_POSE to form the position target.

Env vars:
    G1_POLICY_ONNX   path to policy.onnx (default: trained run path)
    G1_BALANCE_FALLBACK=1
                     if ONNX missing/broken, fall back to NOMINAL + ankle PD
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends.g1_robot_spec import (  # noqa: E402
    JOINT_NAMES, NOMINAL_POSE,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


# The legs-only URDF exposes these 13 actuated joints.
LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
NOMINAL_BY_JOINT = dict(zip(JOINT_NAMES, NOMINAL_POSE))
# STABLE stand pose for the DEPLOY model. The old NOMINAL (hip -0.20 / knee 0.42)
# put the whole-body CoM_x at +0.005 m -- 5 mm AHEAD of this model's foot front
# (x=0.0), so it TIPPED FORWARD at ~1.3 s under ANY control (pure PD, baseline, or
# RL) -- the real reason every G1 deploy attempt fell (NOT a sim2sim gap; verified
# in plain mujoco). The OmniSim URDF importer places the foot ~35 mm further back
# than newton's native add_urdf (which stands at the old pose), so this model needs
# a deeper squat: hip_pitch -0.30 / knee 0.52 drops + recentres the CoM behind the
# foot front. Verified statically stable 15 s in plain mujoco (pitch settles +0.04).
NOMINAL_LEGS = np.array([
    -0.30, 0.00, 0.00, 0.52, -0.23, 0.00,   # left leg
    -0.30, 0.00, 0.00, 0.52, -0.23, 0.00,   # right leg
    0.00,                                    # waist_yaw
], dtype=np.float32)
# Joint limits in LEGS_JOINTS order. The GPU trainer CLAMPS its position targets
# to these (baseline+residual); the deploy must too, else the policy's ±0.3 rad
# residual drives a joint PAST its limit (e.g. ankle_roll ±0.262) -> the joint slams
# into the limit -> reaction kick (the deploy diverged in ROLL). Clamping matches the
# trainer and removes the kick.
LIM_LO = np.array([-2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
                   -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618], np.float32)
LIM_HI = np.array([2.880, 2.967, 2.758, 2.880, 0.524, 0.262,
                   2.880, 0.524, 2.758, 2.880, 0.524, 0.262, 2.618], np.float32)
NJ = 13
OBS_DIM = 48
ACT_SCALE = 0.3

# Analytic balance PD baseline -- defaults match g1_stand_env.py exactly.
KP_ANKLE_PITCH = -1.5
KD_ANKLE_PITCH = -0.2
KP_ANKLE_ROLL = -1.5
KD_ANKLE_ROLL = -0.2
BAL_CLAMP = 0.2
_L_AP = LEGS_JOINTS.index("left_ankle_pitch_joint")
_R_AP = LEGS_JOINTS.index("right_ankle_pitch_joint")
_L_AR = LEGS_JOINTS.index("left_ankle_roll_joint")
_R_AR = LEGS_JOINTS.index("right_ankle_roll_joint")
_L_HP = LEGS_JOINTS.index("left_hip_pitch_joint")
_R_HP = LEGS_JOINTS.index("right_hip_pitch_joint")
_L_KN = LEGS_JOINTS.index("left_knee_joint")
_R_KN = LEGS_JOINTS.index("right_knee_joint")


def _balance_gains() -> dict:
    """Layer-1 active-balance gains, env-overridable. Defaults are now OFF (0):
    the analytic ankle PD DESTABILISES the deploy -- its finite-diff roll_rate
    spikes at handover and kicks ankle_roll, so the robot fell in ROLL at ~0.9 s
    even though the deeper-squat NOMINAL is statically stable (verified: with the
    PD off the deploy STANDS, roll=0, pitch converges to +0.04). Active balance is
    now the RL policy's job (residual on top of pure NOMINAL); the analytic PD is
    kept only as an opt-in (set G1_BAL_KP_* ) for experiments. Capture-point sign:
    a forward fall has pitch>0, restoring ankle offset is NEGATIVE, coeffs <=0."""
    def f(name, default):
        try:
            return float(os.environ.get(name, default))
        except (TypeError, ValueError):
            return default
    return dict(
        kp_p=f("G1_BAL_KP_P", 0.0),              # ankle-pitch on body pitch (OFF)
        kd_p=f("G1_BAL_KD_P", 0.0),              # ankle-pitch on pitch rate (OFF)
        kv_p=f("G1_BAL_KV_P", 0.0),              # ankle-pitch on CoM/pelvis vx (DCM)
        kp_r=f("G1_BAL_KP_R", 0.0),              # ankle-roll on body roll (OFF -- destabilised)
        kd_r=f("G1_BAL_KD_R", 0.0),              # ankle-roll on roll rate (OFF)
        kv_r=f("G1_BAL_KV_R", 0.0),              # ankle-roll on vy
        clamp=f("G1_BAL_CLAMP", BAL_CLAMP),      # ankle offset clamp (rad)
        kh_p=f("G1_BAL_KH_P", 0.0),              # hip-pitch on ankle saturation
        kk=f("G1_BAL_KK", 0.0),                  # knee extend on height (bz) sink
        z_ref=f("G1_BAL_ZREF", 0.78),            # pelvis-z target for the knee term
    )


def _baseline_targets(roll: float, pitch: float,
                      roll_rate: float, pitch_rate: float,
                      vx: float = 0.0, vy: float = 0.0, bz: float = 0.78,
                      g: dict = None) -> np.ndarray:
    """NOMINAL + active position-mode balance (capture-point ankle + hip strategy
    + knee height-hold). Defaults (no env overrides) == the legacy ankle PD."""
    if g is None:
        g = _balance_gains()
    # Sagittal: capture-point ankle offset (attitude + rate + CoM velocity).
    ap_raw = g["kp_p"] * pitch + g["kd_p"] * pitch_rate + g["kv_p"] * vx
    ap = float(np.clip(ap_raw, -g["clamp"], g["clamp"]))
    # Hip strategy: the part of the demand the saturated ankle couldn't take.
    hp = g["kh_p"] * (ap_raw - ap)
    # Lateral ankle-roll offset.
    ar_raw = g["kp_r"] * roll + g["kd_r"] * roll_rate + g["kv_r"] * vy
    ar = float(np.clip(ar_raw, -g["clamp"], g["clamp"]))
    # Knee height-hold: extend (knee offset>0 = more bend? sign via gain) on sink.
    kn = g["kk"] * (g["z_ref"] - bz)
    targets = NOMINAL_LEGS.copy()
    targets[_L_AP] += ap;  targets[_R_AP] += ap
    targets[_L_AR] += ar;  targets[_R_AR] += ar
    targets[_L_HP] += hp;  targets[_R_HP] += hp
    targets[_L_KN] += kn;  targets[_R_KN] += kn
    return targets


def _quat_to_rpy(qw, qx, qy, qz):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _ori_matrix_to_rpy(o):
    """o is the 9-element row-major rotation matrix from getOrientation()."""
    # Convert to roll/pitch/yaw using the matrix the OmniSim g1_model_walk
    # controller uses (so the deploy controller agrees with the training env).
    return (math.atan2(o[7], o[8]),
            math.asin(max(-1.0, min(1.0, -o[6]))),
            math.atan2(o[3], o[0]))


def _proj_gravity_from_ori(o):
    """Project world gravity (0,0,-1) into the body frame.

    OmniSim's getOrientation() returns body-to-world R row-major (9 floats).
    Body-frame gravity = R^T * (0, 0, -1).
    R^T * (0,0,-1) = (-R[6], -R[7], -R[8]).
    """
    return (-float(o[6]), -float(o[7]), -float(o[8]))


def find_policy_path() -> Path:
    p = os.environ.get("G1_POLICY_ONNX") or os.environ.get("OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    # Prefer the Newton-matched MJCF policy (kp=20/kv=3, sim2sim parity).
    # Earlier runs used kp=400/kv=20 which deploys to soft Newton joints
    # poorly. Then the DR run, then plain run, then CPU SB3 fallback.
    for cand in (
        # Robust-DR run: heavy domain randomization so the policy is
        # invariant to the wrapper-introduced sim2sim quirks
        # (control latency, per-body mass jitter, actuator-gain
        # jitter, big random pushes). This is the sim-to-real path.
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx",
        # Trained on Newton-dumped MJCF — same physics, but a single
        # operating point, no robustness against wrapper drift.
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_newtonmjcf/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_v3/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_matched/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_dr/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand/policy.onnx",
        _REPO / "projects/policies/research/training/runs/g1_stand/policy.onnx",
    ):
        if cand.exists():
            return cand
    return _REPO / "projects/policies/research/training/runs/gpu_g1_stand/policy.onnx"


def main() -> int:
    side_log_path = os.environ.get("G1_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg: str) -> None:
        sys.stderr.write(msg)
        sys.stderr.flush()
        if side_log is not None:
            side_log.write(msg)
            side_log.flush()

    # ONNX runtime is imported here so the controller can fall back to
    # the analytic baseline if ort isn't installed.
    sess = None
    policy_path = find_policy_path()
    if policy_path.exists():
        try:
            import onnxruntime as ort  # type: ignore
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say(f"[g1_stand_deploy] loaded ONNX {policy_path.name}\n")
        except Exception as e:
            say(f"[g1_stand_deploy] ONNX load failed ({e}); falling back\n")
    else:
        say(f"[g1_stand_deploy] no ONNX at {policy_path}; falling back to balance PD\n")

    fallback = (sess is None) or (
        os.environ.get("G1_BALANCE_FALLBACK", "0").strip() != "0")

    # Layer-1 active-balance gains (env-overridable; defaults == legacy ankle PD).
    bal_gains = _balance_gains()
    say(f"[g1_stand_deploy] balance gains: {bal_gains}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = {}
    sensors = {}
    for jn in LEGS_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[g1_stand_deploy] missing motor {jn}_motor\n")
            return 1
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors[jn] = s
        except Exception:
            sensors[jn] = None

    # NB: do NOT snap to NOMINAL here. The deploy spawns straight-legged; snapping
    # to the squat at stiff ke folds the legs fast and flings the CoM forward
    # (-> forward tip ~1.4 s). Instead ramp gently into NOMINAL below.

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    last_action = np.zeros(NJ, dtype=np.float32)
    _dbg = bool(os.environ.get("G1_DEBUG"))
    _dbg_n = 0
    last_log_ms = 0
    survived = 0
    fell_at = None
    sim_ms = 0

    # Joint-q -> joint-qd finite-difference state. Warm-started to
    # NOMINAL after settle so the very first policy tick doesn't see
    # a fake (joint_q_now - 0)/dt spike.
    prev_joint_q = NOMINAL_LEGS.copy()
    # Joint qd: the GPU trainer's obs qd is EXACT qvel (no lag). The old 30 ms
    # low-pass here added ~30 ms of velocity lag to 13 of 48 obs dims, which a
    # velocity-feedback balance policy is very sensitive to -> a train<->deploy
    # gap. Default now G1_QD_TAU=0 = raw finite-diff (lowest lag, matches the
    # trainer; the trainer's obs_noise DR covers the extra quantization noise).
    _qd_tau = float(os.environ.get("G1_QD_TAU", "0.0"))
    qd_alpha = 1.0 if _qd_tau <= 0 else max(0.05, min(1.0, step_dt / (step_dt + _qd_tau)))
    joint_qd_smoothed = np.zeros(NJ, dtype=np.float32)
    # Roll/pitch-rate for the baseline (FINITE-DIFF, matching the trainer's
    # _baseline_targets_t which uses (roll-prev)/DT, NOT world ang-vel components).
    prev_roll = 0.0
    prev_pitch = 0.0

    # Gentle ramp from the spawn pose into NOMINAL, then a short settle, before
    # the policy/balance takes over. The straight->squat FOLD is what tips the
    # deploy; ramping over G1_RAMP_S s lowers the body without flinging the CoM
    # forward. G1_RAMP_S=0 restores the old immediate-snap + 0.5 s settle.
    # Opt-in (default 0 = legacy snap). The ramp delays the fold-induced tip
    # (~1.4 s -> ~2.0 s) but does NOT make the deploy stand -- a deeper deploy-vs-
    # harness physics gap (likely the STATICS-floor ground) remains. Kept as a lever.
    ramp_s = float(os.environ.get("G1_RAMP_S", "0"))
    if ramp_s > 0.0:
        # One step so the position sensors report the spawn (straight) pose.
        if robot.step(step_ms) == -1:
            return 0
        sim_ms += step_ms
        q0 = NOMINAL_LEGS.copy()
        for i, jn in enumerate(LEGS_JOINTS):
            s = sensors.get(jn)
            if s is not None:
                try:
                    q0[i] = float(s.getValue())
                except Exception:
                    pass
        n_ramp = max(1, int(ramp_s / step_dt))
        for r in range(1, n_ramp + 1):
            a = r / float(n_ramp)
            tgt = (1.0 - a) * q0 + a * NOMINAL_LEGS
            for i, jn in enumerate(LEGS_JOINTS):
                motors[jn].setPosition(float(tgt[i]))
            if robot.step(step_ms) == -1:
                return 0
            sim_ms += step_ms
        # short hold at NOMINAL to let it settle
        for jn, q in zip(LEGS_JOINTS, NOMINAL_LEGS):
            motors[jn].setPosition(float(q))
        for _ in range(max(1, int(0.5 / step_dt))):
            if robot.step(step_ms) == -1:
                return 0
            sim_ms += step_ms
    else:
        for jn, q in zip(LEGS_JOINTS, NOMINAL_LEGS):
            motors[jn].setPosition(float(q))
        for _ in range(max(1, int(0.5 / step_dt))):
            if robot.step(step_ms) == -1:
                return 0
            sim_ms += step_ms
        sim_ms += step_ms
    # (Not needed: with the deeper-squat NOMINAL the spawn-seeded pose is statically
    # stable, so no rest-settle / base-velocity zeroing is required. An earlier
    # self_node.setVelocity() handover-rest hack CRASHED the URDFRobot root under
    # Newton and is removed.)
    # Warm-start prev_joint_q from the actual settled sensor reading
    # so the policy's joint_q - NOMINAL signal has a clean baseline.
    for i, jn in enumerate(LEGS_JOINTS):
        s = sensors.get(jn)
        if s is not None:
            try:
                prev_joint_q[i] = float(s.getValue())
            except Exception:
                pass

    while robot.step(step_ms) != -1:
        sim_ms += step_ms

        if self_node is None:
            continue

        try:
            pos = self_node.getPosition() or [0.0, 0.0, 0.0]
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
        except Exception:
            continue

        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll, pitch, yaw = _ori_matrix_to_rpy(ori)

        # Pelvis 6-DOF velocity, world frame. Supervisor.getVelocity()
        # returns [vx, vy, vz, wx, wy, wz] in world coords, matching
        # MuJoCo's qvel[0:6] for the free joint. This is what the
        # policy was trained on -- using finite-difference position
        # deltas (the previous implementation) added noise + a fake
        # first-tick spike that the policy reacts to and falls.
        try:
            vel6 = self_node.getVelocity() or [0.0] * 6
        except Exception:
            vel6 = [0.0] * 6
        lin_vel = np.array(vel6[0:3], dtype=np.float32)
        ang_vel = np.array(vel6[3:6], dtype=np.float32)

        # Joint q from position sensors.
        joint_q = np.zeros(NJ, dtype=np.float32)
        for i, jn in enumerate(LEGS_JOINTS):
            s = sensors.get(jn)
            if s is not None:
                try:
                    joint_q[i] = float(s.getValue())
                except Exception:
                    joint_q[i] = prev_joint_q[i]
            else:
                joint_q[i] = prev_joint_q[i]
        # Joint qd via finite-diff with low-pass smoothing. Position
        # sensors don't expose velocity directly, so we compute it.
        # PositionSensor.getValue() is quantized; without the smoother
        # the policy sees a noisy qd that the trainer never trained on.
        joint_qd_raw = (joint_q - prev_joint_q) / max(step_dt, 1e-6)
        joint_qd_smoothed = (qd_alpha * joint_qd_raw
                             + (1.0 - qd_alpha) * joint_qd_smoothed)
        joint_qd = joint_qd_smoothed.astype(np.float32)
        prev_joint_q = joint_q.copy()

        proj_g = np.array(_proj_gravity_from_ori(ori), dtype=np.float32)

        # OBS FIX 2026-06-09: the GPU trainer's obs angular velocity is MuJoCo
        # free-joint qvel[3:6] = BODY frame (verified empirically). getVelocity()
        # returns WORLD-frame omega, so the policy was being fed the wrong-frame
        # ang-vel (the reason the trained policy DESTABILISED the deploy). Rotate
        # world omega into the body frame: omega_body = R^T * omega_world, where
        # ori = getOrientation() is the body->world rotation (row-major; confirmed
        # by the roll/pitch formula in _ori_matrix_to_rpy). Linear vel is WORLD in
        # both (also verified) so it is left unchanged.
        _o = ori
        _wx, _wy, _wz = float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2])
        ang_vel_body = np.array([
            _o[0] * _wx + _o[3] * _wy + _o[6] * _wz,
            _o[1] * _wx + _o[4] * _wy + _o[7] * _wz,
            _o[2] * _wx + _o[5] * _wy + _o[8] * _wz,
        ], dtype=np.float32)

        # Spot-style residual recipe: targets = NOMINAL + analytic balance + policy delta.
        # roll/pitch-rate via FINITE-DIFF (matches the trainer's baseline, which uses
        # (roll-prev)/DT, not the world ang-vel components the deploy used before).
        roll_rate = (roll - prev_roll) / step_dt
        pitch_rate = (pitch - prev_pitch) / step_dt
        prev_roll = roll
        prev_pitch = pitch
        baseline = _baseline_targets(roll, pitch, roll_rate, pitch_rate,
                                     vx=float(lin_vel[0]), vy=float(lin_vel[1]),
                                     bz=bz, g=bal_gains)
        if fallback:
            # No policy available — run pure baseline (will fall in ~0.5 s,
            # but at least the controller proves end-to-end the path works).
            targets = baseline
            action = np.zeros(NJ, dtype=np.float32)
        else:
            obs = np.concatenate([
                lin_vel, ang_vel_body, proj_g,
                joint_q - NOMINAL_LEGS, joint_qd, last_action,
            ]).astype(np.float32)
            assert obs.shape[0] == OBS_DIM, f"obs shape {obs.shape} != {OBS_DIM}"
            try:
                out = sess.run(None, {"obs": obs[None, :]})
                action = np.clip(out[0][0], -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[g1_stand_deploy] ONNX inference failed: {e}; falling back\n")
                fallback = True
                action = np.zeros(NJ, dtype=np.float32)
            targets = baseline + ACT_SCALE * action

        # CLAMP to joint limits exactly as the GPU trainer does
        # (clamp(baseline+RES_SCALE*applied, jl_lo, jl_hi)). Without this the
        # policy's residual drives ankle_roll past ±0.262 -> limit slam -> kick.
        targets = np.clip(targets, LIM_LO, LIM_HI)

        for i, jn in enumerate(LEGS_JOINTS):
            motors[jn].setPosition(float(targets[i]))

        last_action = action

        if _dbg and not fallback and _dbg_n < 30:
            _dbg_n += 1
            say(f"[dbg {_dbg_n:02d}] t={sim_ms/1000:5.2f} bz={bz:.3f} "
                f"roll={roll:+.3f} pit={pitch:+.3f} "
                f"lin=({lin_vel[0]:+.2f},{lin_vel[1]:+.2f},{lin_vel[2]:+.2f}) "
                f"angB=({ang_vel_body[0]:+.2f},{ang_vel_body[1]:+.2f},{ang_vel_body[2]:+.2f}) "
                f"|dq|max={np.abs(joint_q-NOMINAL_LEGS).max():.3f} "
                f"|qd|max={np.abs(joint_qd).max():.2f} "
                f"|act|max={np.abs(action).max():.2f}\n")

        upright = (bz > 0.45 and abs(roll) < 0.8 and abs(pitch) < 0.8)
        if upright:
            survived += 1
        elif fell_at is None:
            fell_at = sim_ms / 1000.0

        if sim_ms - last_log_ms >= 1000:
            tag = "OK" if upright else f"FALL@{fell_at:.2f}s"
            say(f"[g1_stand_deploy] t={sim_ms/1000:5.1f}s "
                f"bx={bx:+.2f} bz={bz:.3f} roll={roll:+.3f} "
                f"pitch={pitch:+.3f}  {tag}  "
                f"{'ONNX' if not fallback else 'PD'}\n")
            last_log_ms = sim_ms

    return 0


if __name__ == "__main__":
    sys.exit(main())
