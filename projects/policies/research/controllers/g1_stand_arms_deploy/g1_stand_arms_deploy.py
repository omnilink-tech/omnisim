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

"""G1 standing deploy controller — FULL 23-DOF body (legs + arms).

Experiment: does the 13-DOF legs-only standing policy still hold the
robot up once the 10 arm joints (and their mass, ~3 kg above the waist)
are added to the body?

The policy is UNCHANGED — it reads the same 13 leg/waist joints + the
same pelvis 6-DOF state, and outputs the same 13-dim residual. The 10
arm joints are simply commanded to a fixed relaxed nominal pose; they
are along for the ride, contributing mass and inertia but not actively
balancing. If the robot stands, it means the leg policy is robust to
the CoM shift from the arms (it was never trained with arm mass).

Mirrors `g1_stand_deploy.py` exactly for the 13 driven joints; the only
addition is the ARM_JOINTS block held at ARM_NOMINAL.

Env vars:
    G1_POLICY_ONNX        path to policy.onnx (defaults to the trained run)
    G1_BALANCE_FALLBACK=1 run NOMINAL + ankle PD only (no policy)
    G1_DEPLOY_LOG         side-log path
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


# ── The 13 joints the policy drives (identical to g1_stand_deploy). ──
LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
# ── The 10 arm joints held at nominal (NOT driven by the policy). ──
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)
NOMINAL_BY_JOINT = dict(zip(JOINT_NAMES, NOMINAL_POSE))
# DEEPER-squat stand pose, MATCHING the trainer (gpu_mjwarp_g1_stand_trainer NOMINAL) and
# g1_stand_deploy. The g1_robot_spec NOMINAL_POSE is the OLD shallow pose (hip -0.20/knee 0.42)
# whose CoM sits ahead of the foot front -> tips/rolls; the policy was ALSO trained on (and its
# obs q-NOMINAL is referenced to) the deeper squat, so using the spec pose here both destabilises
# the stance AND feeds the policy an offset obs. This was the full-body deploy roll-at-0.35s bug.
NOMINAL_LEGS = np.array([
    -0.30, 0.0, 0.0, 0.52, -0.23, 0.0,    # left leg
    -0.30, 0.0, 0.0, 0.52, -0.23, 0.0,    # right leg
    0.0,                                   # waist
], dtype=np.float32)
# DEPLOY-GEOMETRY pose re-center (diagnostic 2026-06-23): the OmniSim deploy tips the
# OPPOSITE way from plain MuJoCo at the same NOMINAL (deploy leans BACKWARD, plain mujoco
# forward) -- the importer's foot placement shifts the CoM-vs-support relationship. These
# static biases shift the CoM to sit over the DEPLOY's actual support. G1_ANK_PITCH_BIAS
# rotates the body fore/aft about the ankle; G1_HIP_PITCH_BIAS leans the torso. Default 0.
import os as _biasos
_ANK_BIAS = float(_biasos.environ.get("G1_ANK_PITCH_BIAS", "0"))
_HIP_BIAS = float(_biasos.environ.get("G1_HIP_PITCH_BIAS", "0"))
NOMINAL_LEGS[4] += _ANK_BIAS;  NOMINAL_LEGS[10] += _ANK_BIAS   # L/R ankle pitch
NOMINAL_LEGS[0] += _HIP_BIAS;  NOMINAL_LEGS[6]  += _HIP_BIAS   # L/R hip pitch

# ── Layer A: SECURE-STANCE deltas (BraceGuard, env-gated; ALL default 0 so the
# committed pose is byte-identical unless the braceguard launcher opts in). The
# more-secure defensive stance LOWERS the CoM (deeper squat via the knee) and WIDENS
# the support polygon (hip-roll abduction), so the stiff KE position-hold resists cube
# impacts from more directions before the CoM escapes the foot edge. A matched
# ankle-roll counter-rotation keeps the abducted SOLES FLAT on the floor (HARD RULE:
# feet planted -- abducting the hips alone would roll the robot onto the inner sole
# edge and SHRINK the contact, the opposite of the intent). All additive on NOMINAL_LEGS.
_KNEE_BIAS = float(_biasos.environ.get("G1_KNEE_BIAS", "0"))            # +=> deeper squat (bend knees)
_HIP_ROLL  = float(_biasos.environ.get("G1_HIP_ROLL_NOMINAL", "0"))    # +=> abduct, wider feet (L +, R -)
_ANK_ROLL  = float(_biasos.environ.get("G1_ANKLE_ROLL_NOMINAL", "0"))  # +=> sole-flat counter-rotation
NOMINAL_LEGS[3]  += _KNEE_BIAS;  NOMINAL_LEGS[9]  += _KNEE_BIAS    # L/R knee
NOMINAL_LEGS[1]  += _HIP_ROLL;   NOMINAL_LEGS[7]  -= _HIP_ROLL     # L/R hip roll (mirror)
NOMINAL_LEGS[5]  += _ANK_ROLL;   NOMINAL_LEGS[11] -= _ANK_ROLL     # L/R ankle roll (mirror)

# Arm hold pose. The spec NOMINAL bends each elbow +0.5 rad, which kicks
# the forearms ~28° FORWARD and shifts the CoM forward enough that the
# legs-only policy (trained with zero arm mass) pitches forward and falls
# in <0.5 s. Instead, hang both arms straight down (elbow=0), with only
# the small ±0.2 rad shoulder-roll splay to keep them off the torso. This
# keeps the ~3 kg of arm mass low and laterally symmetric — as close to
# CoM-neutral as a fixed arm pose gets. Override per-joint via G1_ARM_POSE
# ("spec" restores the bent-elbow pose for comparison).
_ARM_DOWN = {
    "left_shoulder_pitch_joint":  0.00,
    "left_shoulder_roll_joint":   0.20,
    "left_shoulder_yaw_joint":    0.00,
    "left_elbow_joint":           0.00,
    "left_wrist_roll_joint":      0.00,
    "right_shoulder_pitch_joint": 0.00,
    "right_shoulder_roll_joint": -0.20,
    "right_shoulder_yaw_joint":   0.00,
    "right_elbow_joint":          0.00,
    "right_wrist_roll_joint":     0.00,
}
# ── Layer B arm poses: GUARD-REST (the secure-stance ready pose) + BAR-OUT (the
# two-hand intercept). The defense throws BOTH forearms up into the cube's flight
# line as a wide, symmetric "bar" across the torso front -- a far better intercept
# than a thin one-arm swat (large contact area; symmetric => net lateral CoM ~0; both
# ~3 kg links). Guard-rest keeps the forearms low/forward at lower-chest height
# (matching the THROW_Z=0.12 aim) and is strictly LESS CoM-forward than the
# destabilising spec pose (shoulders only +0.30, not raised high). Every value is
# inside the G1 23-DOF arm limits.
_ARM_GUARD = {
    "left_shoulder_pitch_joint":  0.30, "left_shoulder_roll_joint":   0.22,
    "left_shoulder_yaw_joint":    0.20, "left_elbow_joint":           0.90,
    "left_wrist_roll_joint":      0.00,
    "right_shoulder_pitch_joint": 0.30, "right_shoulder_roll_joint": -0.22,
    "right_shoulder_yaw_joint":  -0.20, "right_elbow_joint":          0.90,
    "right_wrist_roll_joint":     0.00,
}
_ARM_BAR = {  # forearms swing up + tuck inward so they MEET centrally = a continuous bar
    "left_shoulder_pitch_joint": -0.10, "left_shoulder_roll_joint":   0.05,
    "left_shoulder_yaw_joint":    0.35, "left_elbow_joint":           0.45,
    "left_wrist_roll_joint":      0.00,
    "right_shoulder_pitch_joint": -0.10, "right_shoulder_roll_joint": -0.05,
    "right_shoulder_yaw_joint":  -0.35, "right_elbow_joint":          0.45,
    "right_wrist_roll_joint":     0.00,
}
_arm_pose_name = os.environ.get("G1_ARM_POSE", "down").strip().lower()
if _arm_pose_name == "spec":
    ARM_NOMINAL = np.array([NOMINAL_BY_JOINT[j] for j in ARM_JOINTS], dtype=np.float32)
elif _arm_pose_name == "guard":
    ARM_NOMINAL = np.array([_ARM_GUARD[j] for j in ARM_JOINTS], dtype=np.float32)
else:
    ARM_NOMINAL = np.array([_ARM_DOWN[j] for j in ARM_JOINTS], dtype=np.float32)
ARM_BAR = np.array([_ARM_BAR[j] for j in ARM_JOINTS], dtype=np.float32)
NJ = 13
OBS_DIM = 48
ACT_SCALE = float(os.environ.get("G1_ACT_SCALE", "0.3"))  # must MATCH the trainer's G1_RES_SCALE

# Analytic balance PD baseline -- defaults match g1_stand_env.py exactly, but are
# now ENV-OVERRIDABLE. The analytic ankle PD DESTABILISES roll in deploy (its
# finite-diff roll_rate kicks ankle_roll at handover -- the exact bug documented in
# g1_stand_deploy.py), so the PURE-POSE stand+wave path turns it OFF entirely
# (G1_ARMS_ANKLE_KP=0 G1_ARMS_ANKLE_KD=0) and relies on the statically-stable
# deeper-squat NOMINAL + a stiff position hold to balance THROUGH the feedforward
# wave -- no policy, no ankle PD. Default (no env) is unchanged.
def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default
KP_ANKLE_PITCH = _envf("G1_ARMS_ANKLE_KP", -1.5)
KD_ANKLE_PITCH = _envf("G1_ARMS_ANKLE_KD", -0.2)
# Roll gains independently overridable (default to the pitch env for back-compat). The
# 2026-06-23 deploy diagnostic showed the sagittal sign was the problem: +ankle offset
# leans the body BACK, so the RESTORING gain for a forward lean (pitch>0) is POSITIVE,
# not the legacy -1.5. Roll can be tuned/disabled separately while stabilising pitch.
KP_ANKLE_ROLL = _envf("G1_ARMS_ANKLE_KP_R", _envf("G1_ARMS_ANKLE_KP", -1.5))
KD_ANKLE_ROLL = _envf("G1_ARMS_ANKLE_KD_R", _envf("G1_ARMS_ANKLE_KD", -0.2))
BAL_CLAMP = float(_envf("G1_ARMS_BAL_CLAMP", 0.2))
# Capture-point (CoM-velocity) + hip-strategy gains (env-gated, default 0 = ankle-only legacy).
# These turn the ankle-only PD into a proper deterministic balancer that can actively HOLD the
# deploy's (bridge-model) standing equilibrium, which a pure pose-hold can't (it leans/tips).
KV_ANKLE_PITCH = _envf("G1_ARMS_ANKLE_KV", 0.0)   # ankle-pitch on CoM forward-vel vx (DCM)
KV_ANKLE_ROLL  = _envf("G1_ARMS_ANKLE_KV_R", 0.0)  # ankle-roll on lateral-vel vy
KH_PITCH = _envf("G1_ARMS_HIP_KH", 0.0)            # hip-pitch picks up the saturated ankle demand
_L_AP = LEGS_JOINTS.index("left_ankle_pitch_joint")
_R_AP = LEGS_JOINTS.index("right_ankle_pitch_joint")
_L_AR = LEGS_JOINTS.index("left_ankle_roll_joint")
_R_AR = LEGS_JOINTS.index("right_ankle_roll_joint")
_L_HP = LEGS_JOINTS.index("left_hip_pitch_joint")
_R_HP = LEGS_JOINTS.index("right_hip_pitch_joint")
_WAIST = LEGS_JOINTS.index("waist_yaw_joint")


def _baseline_targets(roll: float, pitch: float,
                      roll_rate: float, pitch_rate: float,
                      vx: float = 0.0, vy: float = 0.0) -> np.ndarray:
    # Sagittal capture-point ankle: attitude + rate + CoM-velocity (DCM ~ pitch + pitch_rate/w + vx).
    ap_raw = KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate + KV_ANKLE_PITCH * vx
    ap = float(np.clip(ap_raw, -BAL_CLAMP, BAL_CLAMP))
    hp = KH_PITCH * (ap_raw - ap)               # hip strategy: take what the saturated ankle couldn't
    ar = KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate + KV_ANKLE_ROLL * vy
    ar = float(np.clip(ar, -BAL_CLAMP, BAL_CLAMP))
    targets = NOMINAL_LEGS.copy()
    targets[_L_AP] += ap;  targets[_R_AP] += ap
    targets[_L_AR] += ar;  targets[_R_AR] += ar
    targets[_L_HP] += hp;  targets[_R_HP] += hp
    return targets


def _ori_matrix_to_rpy(o):
    return (math.atan2(o[7], o[8]),
            math.asin(max(-1.0, min(1.0, -o[6]))),
            math.atan2(o[3], o[0]))


def _proj_gravity_from_ori(o):
    return (-float(o[6]), -float(o[7]), -float(o[8]))


# ── Layer B: reactive ORIENT-AND-BAR block (deterministic feedforward, env-gated) ──
# The cube_thrower is a supervisor that KNOWS each cube's bearing and time-of-flight, so
# it acts as a perfect threat oracle: at every throw it writes a one-shot JSON
# {throw_idx, theta, ttf_ms, bearing_reachable} to G1_THREAT_CHANNEL. We poll it, and for
# a reachable (roughly front-hemisphere) cube we (1) yaw the waist to FACE the cube
# (planted feet -- a torso rotation, NOT a step) and (2) play the two-hand BAR so it is
# OUT and held across the cube's arrival, timed to lead the impact by G1_BLOCK_LEAD_MS.
# Uses RELATIVE time-to-impact (no cross-process clock sync needed): we schedule off the
# robot's own clock at the moment a fresh throw is seen. Rear/side cubes (not reachable)
# are left to Layer A's passive resistance -- the honest, conceded coverage limit.
_THREAT_PATH = os.environ.get("G1_THREAT_CHANNEL", "")
_BLOCK_LEAD_MS = float(os.environ.get("G1_BLOCK_LEAD_MS", "0"))   # 0 => block OFF (A/B "off" arm)
_BLOCK_ON = bool(_THREAT_PATH) and _BLOCK_LEAD_MS > 0.0
_WAIST_CLAMP = float(os.environ.get("G1_BLOCK_WAIST_CLAMP", "1.0"))  # max waist yaw to face a threat
_BAR_RISE = float(os.environ.get("G1_BLOCK_RISE_S", "0.18"))   # guard-rest -> BAR-OUT
_BAR_HOLD = float(os.environ.get("G1_BLOCK_HOLD_S", "0.14"))   # hold the bar across arrival
_BAR_FALL = float(os.environ.get("G1_BLOCK_FALL_S", "0.20"))   # BAR-OUT -> guard-rest


def _smoothstep(x: float) -> float:
    x = 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
    return x * x * (3.0 - 2.0 * x)


class _Defense:
    """Schedules + plays the ORIENT-AND-BAR primitive from the threat channel.

    targets(sim_ms) -> (arm_vec[10], waist_yaw) where arm_vec is in ARM_JOINTS order.
    When idle it returns (ARM_NOMINAL, 0.0) so the stance/guard pose is held untouched.
    """

    def __init__(self, say):
        self._say = say
        self.last_idx = -1
        self.fire_at = None     # robot-clock ms at which to start the motion
        self.theta = 0.0        # waist yaw to face the threat (rad, clamped)
        self.t0 = None          # robot-clock ms the motion started
        self._mtime = 0.0
        self._dur = _BAR_RISE + _BAR_HOLD + _BAR_FALL

    def _poll(self, sim_ms: float) -> None:
        if not _THREAT_PATH:
            return
        try:
            mt = os.path.getmtime(_THREAT_PATH)
        except OSError:
            return
        if mt <= self._mtime:
            return
        self._mtime = mt
        try:
            import json
            with open(_THREAT_PATH) as f:
                d = json.load(f)
        except Exception:
            return
        idx = int(d.get("throw_idx", -1))
        if idx == self.last_idx:
            return
        self.last_idx = idx
        if not bool(d.get("bearing_reachable", False)):
            return  # rear/side cube -> Layer A passive resist, no block
        ttf = float(d.get("ttf_ms", 0.0))
        self.theta = float(np.clip(float(d.get("theta", 0.0)),
                                   -_WAIST_CLAMP, _WAIST_CLAMP))
        # Fire so BAR-OUT is reached ~G1_BLOCK_LEAD_MS before impact; clamp >= now.
        self.fire_at = sim_ms + max(0.0, ttf - _BLOCK_LEAD_MS)
        self._say(f"[g1_arms_deploy] BLOCK scheduled idx={idx} theta={self.theta:+.2f} "
                  f"ttf={ttf:.0f}ms fire_in={max(0.0, ttf - _BLOCK_LEAD_MS):.0f}ms\n")

    def targets(self, sim_ms: float):
        self._poll(sim_ms)
        if self.fire_at is not None and self.t0 is None and sim_ms >= self.fire_at:
            self.t0 = sim_ms
            self.fire_at = None
        if self.t0 is None:
            return ARM_NOMINAL, 0.0
        e = (sim_ms - self.t0) / 1000.0
        if e < _BAR_RISE:
            s = _smoothstep(e / max(_BAR_RISE, 1e-6))
        elif e < _BAR_RISE + _BAR_HOLD:
            s = 1.0
        elif e < self._dur:
            s = 1.0 - _smoothstep((e - _BAR_RISE - _BAR_HOLD) / max(_BAR_FALL, 1e-6))
        else:
            self.t0 = None
            return ARM_NOMINAL, 0.0
        arm = ARM_NOMINAL + s * (ARM_BAR - ARM_NOMINAL)
        return arm.astype(np.float32), s * self.theta


def find_policy_path() -> Path:
    p = os.environ.get("G1_POLICY_ONNX") or os.environ.get("OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    for cand in (
        # Full-body policy: trained with the arm mass present (arms held
        # at nominal). This is the one that should actually balance the
        # full body — prefer it over the legs-only policies below.
        _REPO / "projects/policies/research/training/runs/gpu_g1_arms_stand_robust/policy.onnx",
        # Legs-only policies (trained without arm mass) — fall back so the
        # controller still runs, but expect a forward faceplant.
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand_newtonmjcf/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_stand/policy.onnx",
        _REPO / "projects/policies/research/training/runs/g1_stand/policy.onnx",
    ):
        if cand.exists():
            return cand
    return _REPO / "projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx"


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

    sess = None
    policy_path = find_policy_path()
    if policy_path.exists():
        try:
            import onnxruntime as ort  # type: ignore
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say(f"[g1_arms_deploy] loaded ONNX {policy_path}\n")
        except Exception as e:
            say(f"[g1_arms_deploy] FATAL: ONNX policy exists but failed to load "
                f"({e}) -- refusing to fall back to the bare balance PD and "
                "report it as a policy result\n")
            # FATAL, deliberately: the policy file EXISTS and would not load -- a broken
            # environment (classically: onnxruntime missing from the CONTROLLER interpreter,
            # a DIFFERENT python than the engine embeds), not a mode. Falling back here keeps
            # the robot moving on its bare baseline and still exits 0, so the harness records
            # a PASS for a policy that never ran. That silently voided an entire Go2
            # head-to-head (2026-07-12) -- and the broken run scored BETTER than the real one.
            raise SystemExit(2)
    else:
        say(f"[g1_arms_deploy] no ONNX at {policy_path}; falling back to balance PD\n")

    fallback = (sess is None) or (
        os.environ.get("G1_BALANCE_FALLBACK", "0").strip() != "0")

    robot = _Robot()

    # Cold-load fix: a FRESH OmniSim process under-tracks position targets (the articulation
    # is "cold"), so the balance policy's corrections don't take and the robot drifts/falls
    # within a fraction of a second. warmup_reload() reloads the world once so the articulation
    # tracks crisply (warm). Critical for a balance policy. Opt out with G1_WARMUP_RELOAD=0.
    _warm = os.environ.get("G1_WARMUP_RELOAD", "0").strip() != "0"
    if _warm:
        try:
            _bridge = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
            if str(_bridge) not in sys.path:
                sys.path.insert(0, str(_bridge))
            from omnilink_arm_bridge import warmup_reload as _warmup_reload
            _warmup_reload(robot)
            say("[g1_arms_deploy] warmup_reload done (warm articulation)\n")
        except Exception as e:
            say(f"[g1_arms_deploy] warmup_reload skipped ({e})\n")
            _warm = False

    # ── Environment fingerprint ──
    # Report what ACTUALLY drove the world on this machine (Newton/MuJoCo vs a
    # silent ODE fallback, GPU+driver, warm/cold, build) to the deploy log AND
    # as an on-screen overlay. The humanoid stand's basin is razor-thin and
    # behaves differently under ODE, so "regressed on another computer" is most
    # often the Newton runtime being absent here -- this makes that visible.
    # Pre-first-step: the engine finalizes the physics backend on the FIRST
    # tick, so no verdict exists yet -- pre_step=True draws a neutral label and
    # the real verdict is redrawn after the settle loop below.
    _envfp = None
    _envfp_fp = {}
    try:
        from projects.policies.common import env_fingerprint as _envfp
        _envfp_fp = _envfp.report(robot, say, warm=_warm, repo_root=_REPO,
                                  pre_step=True)
    except Exception as _e:
        say(f"[g1_arms_deploy] env fingerprint skipped ({_e})\n")

    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    # Leg motors (driven by policy) + their position sensors.
    motors = {}
    sensors = {}
    for jn in LEGS_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[g1_arms_deploy] missing leg motor {jn}_motor\n")
            return 1
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors[jn] = s
        except Exception:
            sensors[jn] = None

    # Arm motors (held at nominal). Missing ones warn but don't fail —
    # the body still has the arm mass even if a motor isn't exposed.
    # Also enable their position sensors so we can verify the arms are
    # actually being held (a sagging arm shifts the CoM forward past what
    # the policy trained against — prime sim-to-deploy suspect).
    arm_motors = {}
    arm_sensors = {}
    for jn in ARM_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[g1_arms_deploy] WARN: missing arm motor {jn}_motor "
                f"(arm joint will be passive)\n")
        arm_motors[jn] = m
        try:
            s = m.getPositionSensor() if m is not None else None
            if s is not None:
                s.enable(step_ms)
            arm_sensors[jn] = s
        except Exception:
            arm_sensors[jn] = None
        # Crank up the motor's available torque + stiffness so the arm
        # actually holds its commanded pose. URDF-import defaults can be
        # too soft, letting the arm sag forward under gravity.
        if m is not None:
            try:
                m.setAvailableTorque(60.0)
            except Exception:
                pass
            try:
                m.setControlPID(40.0, 0.0, 1.0)
            except Exception:
                pass

    # Optional WAVE reference (Shadowing stand+wave demo): drive the arms from a ghost replay
    # CSV (G1_STANDWAVE_REF) instead of holding ARM_NOMINAL -> the policy keeps balancing the
    # full body (the solved standing primitive) while the arms SHADOW the wave (the ghost).
    # Default (no env var) is unchanged: arms held at ARM_NOMINAL.
    wave_arms = None
    _wp = os.environ.get("G1_STANDWAVE_REF")
    if _wp and os.path.exists(_wp):
        try:
            import csv as _csv
            with open(_wp) as _f:
                _rows = list(_csv.DictReader(_f))
            wave_arms = np.array([[float(r[jn]) for jn in ARM_JOINTS] for r in _rows],
                                 dtype=np.float32)
            say(f"[g1_arms_deploy] WAVE ref {len(wave_arms)} frames from {_wp}\n")
        except Exception as e:
            say(f"[g1_arms_deploy] WAVE ref load failed ({e}); holding ARM_NOMINAL\n")
            wave_arms = None

    def arm_targets(frame):
        if wave_arms is not None:
            return wave_arms[frame % len(wave_arms)]
        return ARM_NOMINAL

    defense = _Defense(say) if _BLOCK_ON else None
    if _BLOCK_ON:
        say(f"[g1_arms_deploy] BLOCK on: lead={_BLOCK_LEAD_MS:.0f}ms "
            f"waist_clamp={_WAIST_CLAMP:.2f} rise/hold/fall="
            f"{_BAR_RISE:.2f}/{_BAR_HOLD:.2f}/{_BAR_FALL:.2f}s ch={_THREAT_PATH}\n")

    for jn, q in zip(LEGS_JOINTS, NOMINAL_LEGS):
        motors[jn].setPosition(float(q))
    for jn, q in zip(ARM_JOINTS, arm_targets(0)):
        if arm_motors[jn] is not None:
            arm_motors[jn].setPosition(float(q))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    last_action = np.zeros(NJ, dtype=np.float32)
    last_log_ms = 0
    survived = 0
    fell_at = None
    sim_ms = 0

    prev_joint_q = NOMINAL_LEGS.copy()
    qd_alpha = max(0.05, min(1.0, step_dt / (step_dt + 0.030)))
    joint_qd_smoothed = np.zeros(NJ, dtype=np.float32)

    # Settle at nominal before the policy takes over. Configurable via
    # G1_SETTLE_S (default 0.5 s) — the arm-loaded full body tips faster
    # than the legs-only body during a policy-free settle, so a shorter
    # settle (or 0) lets the policy catch the forward lean sooner.
    # Always step at least twice so the position sensors and
    # getVelocity() return valid (non-NaN) data before we warm-start
    # prev_joint_q and build the first obs.
    settle_s = float(os.environ.get("G1_SETTLE_S", "0.1"))
    n_settle = max(2, int(settle_s / step_dt))
    for _ in range(n_settle):
        # Keep arms at the wave's first frame (arms at side) during settle.
        for jn, q in zip(ARM_JOINTS, arm_targets(0)):
            if arm_motors[jn] is not None:
                arm_motors[jn].setPosition(float(q))
        if robot.step(step_ms) == -1:
            return 0
        sim_ms += step_ms

    # World has ticked -> the backend verdict (sidecar + finalise line) exists;
    # resolve the deferred fingerprint and redraw the physics label for real.
    if _envfp is not None and _envfp_fp.get("pending"):
        try:
            _envfp_fp = _envfp.report(robot, say, warm=_warm, repo_root=_REPO)
        except Exception as _e:
            say(f"[g1_arms_deploy] env fingerprint recheck skipped ({_e})\n")

    for i, jn in enumerate(LEGS_JOINTS):
        s = sensors.get(jn)
        if s is not None:
            try:
                prev_joint_q[i] = float(s.getValue())
            except Exception:
                pass
    # Diagnostic: robot state at the moment the policy engages.
    if self_node is not None:
        try:
            _p = self_node.getPosition() or [0, 0, 0]
            _o = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            _r, _pi, _ = _ori_matrix_to_rpy(_o)
            say(f"[g1_arms_deploy] settle={settle_s:.2f}s done: "
                f"bz={_p[2]:.3f} roll={_r:+.3f} pitch={_pi:+.3f} "
                f"(policy engages now)\n")
        except Exception:
            pass
    # POSE DIAG (G1_POSE_DIAG=1): per-leg-joint COMMANDED (NOMINAL) vs ACHIEVED (sensor)
    # at settle. A systematic achieved!=commanded offset (esp. ankle/knee/hip pitch) is the
    # SolverMuJoCo control-application bias that leans the deploy backward vs plain mujoco.
    if os.environ.get("G1_POSE_DIAG", "0").strip() != "0":
        say("[pose-diag] joint  cmd     achieved   delta\n")
        for i, jn in enumerate(LEGS_JOINTS):
            say(f"[pose-diag] {jn:24s} {NOMINAL_LEGS[i]:+.4f}  {prev_joint_q[i]:+.4f}  "
                f"{prev_joint_q[i]-NOMINAL_LEGS[i]:+.4f}\n")
    _dbg_ticks = 0
    _DEBUG = os.environ.get("G1_DEBUG_TRACE", "0").strip() != "0"
    wave_k = 0

    while robot.step(step_ms) != -1:
        sim_ms += step_ms

        # Drive the arms every tick. Layer B (block) overrides when a reachable cube is
        # scheduled: it plays the two-hand BAR and yaws the waist to face the threat.
        # Otherwise: the WAVE ref (if loaded, looped) or ARM_NOMINAL (guard/down pose).
        waist_yaw_cmd = None
        if defense is not None:
            arm_vec, waist_yaw_cmd = defense.targets(sim_ms)
        else:
            arm_vec = arm_targets(wave_k)
        for jn, q in zip(ARM_JOINTS, arm_vec):
            if arm_motors[jn] is not None:
                arm_motors[jn].setPosition(float(q))
        wave_k += 1

        if self_node is None:
            continue

        try:
            pos = self_node.getPosition() or [0.0, 0.0, 0.0]
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
        except Exception:
            continue

        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll, pitch, yaw = _ori_matrix_to_rpy(ori)

        try:
            vel6 = self_node.getVelocity() or [0.0] * 6
        except Exception:
            vel6 = [0.0] * 6
        lin_vel = np.array(vel6[0:3], dtype=np.float32)
        # MuJoCo free-joint qvel[3:6] (the trainer's obs ang_vel) is in the BODY frame, but
        # Supervisor.getVelocity() returns WORLD angular velocity. Rotate world->body (R^T w)
        # so the obs matches training -- otherwise the policy's roll/pitch-rate channel is wrong
        # and active balance drives roll the wrong way (the legs-only static stand masked this).
        # Opt out with G1_ANGVEL_BODY=0.
        w0, w1, w2 = float(vel6[3]), float(vel6[4]), float(vel6[5])
        if os.environ.get("G1_ANGVEL_BODY", "1").strip() != "0":
            ang_vel = np.array([
                ori[0] * w0 + ori[3] * w1 + ori[6] * w2,
                ori[1] * w0 + ori[4] * w1 + ori[7] * w2,
                ori[2] * w0 + ori[5] * w1 + ori[8] * w2,
            ], dtype=np.float32)
        else:
            ang_vel = np.array([w0, w1, w2], dtype=np.float32)

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
        joint_qd_raw = (joint_q - prev_joint_q) / max(step_dt, 1e-6)
        joint_qd_smoothed = (qd_alpha * joint_qd_raw
                             + (1.0 - qd_alpha) * joint_qd_smoothed)
        joint_qd = joint_qd_smoothed.astype(np.float32)
        prev_joint_q = joint_q.copy()

        proj_g = np.array(_proj_gravity_from_ori(ori), dtype=np.float32)

        baseline = _baseline_targets(roll, pitch, ang_vel[0], ang_vel[1],
                                     vx=float(lin_vel[0]), vy=float(lin_vel[1]))
        if fallback:
            targets = baseline
            action = np.zeros(NJ, dtype=np.float32)
        else:
            obs = np.concatenate([
                lin_vel, ang_vel, proj_g,
                joint_q - NOMINAL_LEGS, joint_qd, last_action,
            ]).astype(np.float32)
            obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
            assert obs.shape[0] == OBS_DIM, f"obs shape {obs.shape} != {OBS_DIM}"
            try:
                out = sess.run(None, {"obs": obs[None, :]})
                action = np.clip(out[0][0], -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[g1_arms_deploy] ONNX inference failed: {e}; falling back\n")
                fallback = True
                action = np.zeros(NJ, dtype=np.float32)
            targets = baseline + ACT_SCALE * action
            # Per-tick knee-tracking trace (commanded vs actual), gated
            # behind G1_DEBUG_TRACE. This is the diagnostic that revealed
            # the 2026-05-29 Newton regression: leg joints stopped tracking
            # their position targets (knee_cmd~0.42 but knee_act~0), so the
            # robot stood on straight legs and faceplanted regardless of the
            # policy. If you see knee_act not following knee_cmd, run the
            # canonical g1_stand_deploy.omniworld as a control — the build is bad,
            # not your policy.
            if _DEBUG and _dbg_ticks < 60:
                _ki = LEGS_JOINTS.index("left_knee_joint")
                if _dbg_ticks % 4 == 0:
                    say(f"[trace] k{_dbg_ticks:02d} t={sim_ms/1000:.2f} "
                        f"bz={bz:.3f} pitch={pitch:+.3f} vx={lin_vel[0]:+.2f} "
                        f"knee_cmd={float(targets[_ki]):+.3f} "
                        f"knee_act={float(joint_q[_ki]):+.3f} "
                        f"|a|={np.abs(action).max():.2f}\n")
                _dbg_ticks += 1

        # Layer B waist override: face the incoming cube (planted-feet rotation).
        if waist_yaw_cmd is not None:
            targets[_WAIST] = float(waist_yaw_cmd)

        for i, jn in enumerate(LEGS_JOINTS):
            motors[jn].setPosition(float(targets[i]))

        last_action = action

        upright = (bz > 0.45 and abs(roll) < 0.8 and abs(pitch) < 0.8)
        if upright:
            survived += 1
        elif fell_at is None:
            fell_at = sim_ms / 1000.0

        if sim_ms - last_log_ms >= 1000:
            tag = "OK" if upright else f"FALL@{fell_at:.2f}s"
            # Read back two arm joints (commanded: l_sh_pitch=0, l_elbow=0)
            # to confirm the arms are held, not sagging forward.
            shp = arm_sensors.get("left_shoulder_pitch_joint")
            elb = arm_sensors.get("left_elbow_joint")
            shp_v = f"{shp.getValue():+.2f}" if shp is not None else "?"
            elb_v = f"{elb.getValue():+.2f}" if elb is not None else "?"
            say(f"[g1_arms_deploy] t={sim_ms/1000:5.1f}s "
                f"bx={bx:+.2f} bz={bz:.3f} roll={roll:+.3f} "
                f"pitch={pitch:+.3f}  l_shP={shp_v} l_elb={elb_v}  {tag}  "
                f"{'ONNX' if not fallback else 'PD'}  (+arms)\n")
            last_log_ms = sim_ms

    return 0


if __name__ == "__main__":
    sys.exit(main())
