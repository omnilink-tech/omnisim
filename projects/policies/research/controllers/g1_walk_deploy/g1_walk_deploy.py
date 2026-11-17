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
# SINGLE SOURCE OF TRUTH: the physics model (gains, substeps, friction,
# clamp), the joint ORDER, the nominal poses, and the per-joint URDF limits
# all derive from g1_physics_spec -- the ONE place the trainer and this deploy
# share. Nothing physics-model is re-declared as a literal here anymore.
from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402
# SINGLE SOURCE for the shared OBS primitives (projected gravity, the
# world->body ang-vel R^T rotation, and the finite-diff joint-velocity
# estimator). The deploy controller and the GPU trainer assemble the same
# 50-d obs; defining these primitives ONCE here means they can never drift.
from projects.policies.research.backends import g1_env_core as core  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


# The legs-only URDF exposes these 13 actuated joints. Sourced from
# g1_physics_spec (the single source of truth for joint ORDER) -- byte-identical
# to the previous hardcoded tuple.
LEGS_JOINTS = SPEC.LEGS_JOINTS
# Full-body (23-DOF URDF) arm joints. AUTO-DETECTED: when the loaded robot
# exposes these motors (g1_23dof_omnisim.urdf), the controller pins them at
# ARM_NOMINAL every tick -- matching the trainer's --hold-arms mode exactly
# (the policy stays 13-DOF; the +6.1 kg of arm mass is a passive balance
# load it trained against). On the legs-only URDF these devices don't
# exist and the block is skipped.
ARM_JOINTS = SPEC.ARM_JOINTS
# MUST match the trainer's ARM_NOMINAL (arms hanging, slight shoulder-roll
# splay). Now sourced from g1_physics_spec (the single source of truth) instead
# of a transcribed literal.
ARM_NOMINAL = np.array(SPEC.ARM_NOMINAL, dtype=np.float32)
NOMINAL_BY_JOINT = dict(zip(JOINT_NAMES, NOMINAL_POSE))
# STABLE stand pose for the DEPLOY model. The old NOMINAL (hip -0.20 / knee 0.42)
# put the whole-body CoM_x at +0.005 m -- 5 mm AHEAD of this model's foot front
# (x=0.0), so it TIPPED FORWARD at ~1.3 s under ANY control (pure PD, baseline, or
# RL) -- the real reason every G1 deploy attempt fell (NOT a sim2sim gap; verified
# in plain mujoco). The OmniSim URDF importer places the foot ~35 mm further back
# than newton's native add_urdf (which stands at the old pose), so this model needs
# a deeper squat: hip_pitch -0.30 / knee 0.52 drops + recentres the CoM behind the
# foot front. Verified statically stable 15 s in plain mujoco (pitch settles +0.04).
# The default leg nominal now comes from g1_physics_spec (the single source of
# truth); byte-identical to the previous hardcoded squat pose. The
# G1_NOM_HIP/KNEE/ANKLE override loop below is unchanged.
NOMINAL_LEGS = np.array(SPEC.NOMINAL_LEGS, dtype=np.float32)
# GAIT-V2: nominal pose knobs -- MUST match the trainer's --nominal-* for the
# deployed policy. The deep-squat default was a standing-stability fix; the
# natural-gait policies carry a TALL posture (e.g. hip -0.16 knee 0.32
# ankle -0.16).
_nh = os.environ.get("G1_NOM_HIP")
_nk = os.environ.get("G1_NOM_KNEE")
_na = os.environ.get("G1_NOM_ANKLE")
for _base in (0, 6):
    if _nh is not None:
        NOMINAL_LEGS[_base + 0] = float(_nh)
    if _nk is not None:
        NOMINAL_LEGS[_base + 3] = float(_nk)
    if _na is not None:
        NOMINAL_LEGS[_base + 4] = float(_na)
# Joint limits in LEGS_JOINTS order. The GPU trainer CLAMPS its position targets
# to these (baseline+residual); the deploy must too, else the policy's ±0.3 rad
# residual drives a joint PAST its limit (e.g. ankle_roll ±0.262) -> the joint slams
# into the limit -> reaction kick (the deploy diverged in ROLL). Clamping matches the
# trainer and removes the kick. These now come from g1_physics_spec (the single
# source of truth), read LIVE from the prim URDF, instead of transcribed arrays
# -- verified np.allclose to the old literals (atol 2e-3); same chain the trainer
# clamps with, so train<->deploy clamp parity is guaranteed.
LIM_LO, LIM_HI, _LIM_VEL, _LIM_EFF = SPEC.leg_limits()
NJ = SPEC.NJ
OBS_DIM = SPEC.OBS_DIM   # 50 = stand obs (48) + 2 gait-phase (sin,cos) dims
# Residual scale -- MUST match the trainer's --res-scale for the deployed policy
# (sourced from g1_physics_spec, the single source of truth; env-overridable).
ACT_SCALE = float(os.environ.get("G1_ACT_SCALE", str(SPEC.ACT_SCALE)))
# Per-joint residual scale -- MUST match the trainer's --frontal-res-scale.
# Scaling DOWN hip-roll(1,7)/yaw(2,8) stops the policy splaying on top of the
# commanded weight transfer. 1.0 -> uniform ACT_SCALE (unchanged).
_FRONTAL_RES_SCALE = float(os.environ.get("G1_FRONTAL_RES_SCALE", "1.0"))
ACT_SCALE_VEC = np.full(13, ACT_SCALE, dtype=np.float32)
for _j in (1, 7, 2, 8):
    ACT_SCALE_VEC[_j] *= _FRONTAL_RES_SCALE
STEP_DT = SPEC.DT    # control dt -- sourced from g1_physics_spec (must match the trainer DT)
# WALK gait reference -- MUST match gpu_mjwarp_g1_walk_trainer.py exactly.
# Gait params -- MUST match the deployed policy's training. Env-configurable so one
# controller can deploy any walk policy (set G1_GAIT_* to the trainer's values).
GAIT_FREQ = float(os.environ.get("G1_GAIT_FREQ", "1.3"))
GAIT_A_HIP = float(os.environ.get("G1_GAIT_A_HIP", "0.35"))
GAIT_A_KNEE = float(os.environ.get("G1_GAIT_A_KNEE", "0.45"))
GAIT_A_LAT = float(os.environ.get("G1_GAIT_A_LAT", "0.0"))
# Ankle-pitch counter-rotation (keeps the foot flat through the hip swing) --
# MUST match the trainer's --gait-a-ankle for the deployed policy.
GAIT_A_ANKLE = float(os.environ.get("G1_GAIT_A_ANKLE", "0.0"))
# GAIT-V2: counter-phase shoulder-pitch arm swing (full-body worlds) and a
# late-stance ankle push-off bump -- MUST match the trainer's --gait-a-arm /
# --gait-a-push.
GAIT_A_ARM = float(os.environ.get("G1_GAIT_A_ARM", "0.0"))
GAIT_A_PUSH = float(os.environ.get("G1_GAIT_A_PUSH", "0.0"))

# HUMAN GAIT MODEL (G1_GAIT_MODEL=human): foot-space planned, IK-realized
# reference from projects/policies/control/gait/g1_human_gait.py -- replaces the sine CPG
# terms AND the nominal pose (the model's standing pose is its own phase-0
# zero-stride output, so the walk grows out of standing with no snap). All
# params MUST match the trainer's --gait-* values for the deployed policy.
GAIT_MODEL = os.environ.get("G1_GAIT_MODEL", "")
GP = None
_LAST_ARMS = None
if GAIT_MODEL == "human":
    from projects.policies.control.gait import g1_human_gait as _ghg  # noqa: E402

    def _genv(name, default):
        try:
            return float(os.environ.get(name, default))
        except (TypeError, ValueError):
            return default

    GP = _ghg.GaitParams(
        vx=_genv("G1_GAIT_VX", 0.4),
        freq=GAIT_FREQ,
        duty=_genv("G1_GAIT_DUTY", 0.6),
        step_height=_genv("G1_GAIT_STEP_H", 0.05),
        pelvis_height=_genv("G1_GAIT_PELVIS_H", 0.755),
        bob=_genv("G1_GAIT_BOB", 0.020),
        sway=GAIT_A_LAT,
        arm_swing=GAIT_A_ARM,
        elbow_bend=_genv("G1_GAIT_ELBOW", 0.15),
        ankle_clear=_genv("G1_GAIT_ANKLE_CLEAR", 0.08),
        x0=_genv("G1_GAIT_X0", -0.02),
        ramp_s=_genv("G1_GAIT_RAMP_S", 1.0),
        # "winter" = measured human joint kinematics (knee double-bend +
        # ankle push-off). MUST match the trainer's --gait-style.
        style=os.environ.get("G1_GAIT_STYLE", "ik"),
        # Feed-forward pre-compensation: the soft kp=100 joints under-achieve
        # the commanded arc (deploy hip lands ~0.64x the model). Raising the
        # commanded amplitude above the trainer value (0.75) makes the ACHIEVED
        # arc land closer to the nominal model the ghost displays. Deploy-only
        # knob; the ghost stays nominal.
        winter_hip_scale=_genv("G1_GAIT_HIP_SCALE", 0.75),
        # IMPROVED-SHADOW frontal/transverse plane (MUST match the trainer's
        # --gait-lateral/--gait-yaw). default sway/none = legacy behaviour.
        lateral=os.environ.get("G1_GAIT_LATERAL", "sway"),
        yaw=os.environ.get("G1_GAIT_YAW", "none"),
        lat_hip_amp=_genv("G1_GAIT_LAT_HIP_AMP", 0.09),
        step_width=_genv("G1_GAIT_STEP_WIDTH", 0.12),
    )
    NOMINAL_LEGS = _ghg.standing_pose(GP).astype(np.float32)
    _LAST_ARMS = _ghg.targets_np(0.0, GP, t_since_start=0.0)[1]

# DETERMINISTIC BRAIN (G1_BRAIN=1): replace the ONNX policy with the hand-written
# feedforward + balance controller in projects/policies/control/g1_brain.py. The brain
# owns the gait clock and produces the full 13-joint target each tick; it reuses
# this controller's state estimation, settle ramp, clamping, tracing and logging.
BRAIN = None
if os.environ.get("G1_BRAIN", "0").strip() not in ("0", "", "false", "False"):
    # ONE consolidated deterministic brain (projects.policies.control.g1_brain). The
    # module stays SELECTABLE (set G1_BRAIN_MODULE=...) so a future attempt can
    # share this harness + scorer, but the default is now the canonical brain --
    # the old projects.policies.control.brain.g1_brain scratch effort was folded into it.
    import importlib as _il  # noqa: E402
    _bm = _il.import_module(
        os.environ.get("G1_BRAIN_MODULE", "projects.policies.control.g1_brain"))
    G1Brain, BrainState, BrainConfig = _bm.G1Brain, _bm.BrainState, _bm.BrainConfig
    BRAIN = G1Brain(BrainConfig.from_env())
    NOMINAL_LEGS = BRAIN.standing_pose()
CP_GAIN = float(os.environ.get("G1_CP_GAIN", "0.0"))  # capture-point foot-placement gain
GAIT_PHASE_DT = 2.0 * math.pi * GAIT_FREQ * STEP_DT
# ADAPTIVE PHASE: slow the gait clock when off-balance (MUST match the
# trainer's --phase-gate-tilt/-rate/-floor). 0 = fixed clock (legacy).
PHASE_GATE_TILT = float(os.environ.get("G1_PHASE_GATE_TILT", "0.0"))
PHASE_GATE_RATE = float(os.environ.get("G1_PHASE_GATE_RATE", "0.0"))
PHASE_GATE_FLOOR = float(os.environ.get("G1_PHASE_GATE_FLOOR", "0.2"))

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
_L_HR = LEGS_JOINTS.index("left_hip_roll_joint")
_R_HR = LEGS_JOINTS.index("right_hip_roll_joint")


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
                      g: dict = None, phase: float = 0.0,
                      walk_t: float = 0.0) -> np.ndarray:
    """NOMINAL + open-loop WALK gait reference + (off-by-default) analytic balance.
    The gait reference MUST match gpu_mjwarp_g1_walk_trainer._baseline_targets_t:
    left leg phase th, right th+pi; hip_pitch += -A_HIP*sin(th); knee += A_KNEE*relu(sin(th)).
    With G1_GAIT_MODEL=human the foot-space gait model replaces the CPG terms
    (walk_t drives its stride ramp-in)."""
    if g is None:
        g = _balance_gains()
    if GP is not None:
        global _LAST_ARMS
        legs, arms, _sw = _ghg.targets_np(phase, GP, t_since_start=walk_t)
        _LAST_ARMS = arms
        targets = legs.astype(np.float32)
        # analytic balance offsets still add on top (matches the trainer)
        ap_h = float(np.clip(g["kp_p"] * pitch + g["kd_p"] * pitch_rate + g["kv_p"] * vx,
                             -g["clamp"], g["clamp"]))
        ar_h = float(np.clip(g["kp_r"] * roll + g["kd_r"] * roll_rate + g["kv_r"] * vy,
                             -g["clamp"], g["clamp"]))
        targets[_L_AP] += ap_h
        targets[_R_AP] += ap_h
        targets[_L_AR] += ar_h
        targets[_R_AR] += ar_h
        return targets
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
    # WALK gait reference (open-loop CPG; the RL residual stabilises + propels).
    sL = math.sin(phase); sR = math.sin(phase + math.pi)
    targets[_L_HP] += -GAIT_A_HIP * sL
    targets[_R_HP] += -GAIT_A_HIP * sR
    targets[_L_KN] += GAIT_A_KNEE * max(0.0, sL)
    targets[_R_KN] += GAIT_A_KNEE * max(0.0, sR)
    if GAIT_A_ANKLE != 0.0:
        targets[_L_AP] += GAIT_A_ANKLE * sL
        targets[_R_AP] += GAIT_A_ANKLE * sR
    if GAIT_A_PUSH != 0.0:
        # Late-stance plantarflex bump (matches the trainer: squared half-sine
        # peaking at the end of each leg's stance).
        targets[_L_AP] += GAIT_A_PUSH * max(0.0, math.sin(phase - 1.5 * math.pi)) ** 2
        targets[_R_AP] += GAIT_A_PUSH * max(0.0, math.sin(phase - 0.5 * math.pi)) ** 2
    if GAIT_A_LAT != 0.0:
        # Lateral weight-shift sway (sine) -- MUST match the trainer.
        sway = GAIT_A_LAT * sL
        targets[_L_HR] += sway
        targets[_R_HR] += sway
    if CP_GAIN != 0.0:
        # Capture-point lateral foot placement on the swing leg -- MUST match trainer.
        cp = CP_GAIN * vy
        targets[_L_HR] += cp * max(0.0, sL)
        targets[_R_HR] += cp * max(0.0, math.sin(phase + math.pi))
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
    Body-frame gravity = R^T * (0, 0, -1) = (-R[6], -R[7], -R[8]).

    Delegates to ``g1_env_core.proj_gravity_from_matrix`` (the SINGLE source
    the trainer also uses) -- byte-identical to the previous inline formula.
    """
    return core.proj_gravity_from_matrix(o)


def find_policy_path() -> Path:
    p = os.environ.get("G1_POLICY_ONNX") or os.environ.get("OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    # The WALK policy (gait-residual + forward-velocity tracking + big lateral sway,
    # heavy DR). gpu_g1_walk5 is the best walker (deploy gait: G1_GAIT_A_LAT=0.22,
    # G1_START_PHASE=4.71; see run_g1_walk_deploy.ps1).
    for cand in (
        _REPO / "projects/policies/research/training/runs/gpu_g1_walk5/policy.onnx",
        _REPO / "projects/policies/research/training/runs/gpu_g1_walk/policy.onnx",
    ):
        if cand.exists():
            return cand
    return _REPO / "projects/policies/research/training/runs/gpu_g1_walk5/policy.onnx"


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

    # ONNX runtime is imported here; the analytic-baseline fallback exists
    # ONLY for the "no policy file present" branch below.
    sess = None
    policy_path = find_policy_path()
    if policy_path.exists():
        try:
            import onnxruntime as ort  # type: ignore
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say(f"[g1_stand_deploy] loaded ONNX {policy_path.name}\n")
        except Exception as e:
            say(f"[g1_stand_deploy] FATAL: WALK ONNX policy exists but failed to "
                f"load ({e}) -- refusing to fall back to the bare balance PD and "
                "report it as a policy result\n")
            # FATAL, deliberately: the policy file EXISTS and would not load -- a broken
            # environment (classically: onnxruntime missing from the CONTROLLER interpreter,
            # a DIFFERENT python than the engine embeds), not a mode. Falling back here keeps
            # the robot moving on its bare baseline and still exits 0, so the harness records
            # a PASS for a policy that never ran. That silently voided an entire Go2
            # head-to-head (2026-07-12) -- and the broken run scored BETTER than the real one.
            raise SystemExit(2)
    else:
        say(f"[g1_stand_deploy] no ONNX at {policy_path}; falling back to balance PD\n")

    # ── TWO-POLICY mode: optional STAND policy (G1_STAND_POLICY_ONNX). When
    # set, the controller can switch WALK <-> STAND on command (the milestone:
    # stop in the middle, then continue). The stand policy is the proven
    # Newton-robust deeper-squat stander: 48-dim obs (walk proprio MINUS the
    # 2 gait-phase dims), holds STAND_NOMINAL_LEGS. ─────────────────────────
    stand_sess = None
    _stand_p = os.environ.get("G1_STAND_POLICY_ONNX")
    if _stand_p and Path(_stand_p).exists():
        try:
            import onnxruntime as ort  # type: ignore
            stand_sess = ort.InferenceSession(_stand_p,
                                              providers=["CPUExecutionProvider"])
            say(f"[g1_stand_deploy] loaded STAND policy {Path(_stand_p).name}\n")
        except Exception as e:
            say(f"[g1_stand_deploy] FATAL: STAND policy exists but failed to "
                f"load ({e}) -- refusing to run without it\n")
            # FATAL, deliberately: the policy file EXISTS and would not load -- a broken
            # environment (classically: onnxruntime missing from the CONTROLLER interpreter,
            # a DIFFERENT python than the engine embeds), not a mode. Falling back here keeps
            # the robot moving on its bare baseline and still exits 0, so the harness records
            # a PASS for a policy that never ran. That silently voided an entire Go2
            # head-to-head (2026-07-12) -- and the broken run scored BETTER than the real one.
            raise SystemExit(2)
    # Statically-stable deeper-squat stand pose (matches g1_stand_deploy).
    STAND_NOMINAL_LEGS = np.array([
        -0.30, 0.00, 0.00, 0.52, -0.23, 0.00,
        -0.30, 0.00, 0.00, 0.52, -0.23, 0.00, 0.00], dtype=np.float32)

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

    # Full-body mode (auto-detect): pin any present arm motors at ARM_NOMINAL
    # every tick, mirroring the trainer's --hold-arms.
    arm_motors = {}
    for jn in ARM_JOINTS:
        try:
            m = robot.getDevice(f"{jn}_motor")
        except Exception:
            m = None
        if m is not None:
            arm_motors[jn] = m
    if arm_motors:
        say(f"[g1_stand_deploy] full-body: holding {len(arm_motors)} arm joints at nominal\n")

    def hold_arms(ph=0.0):
        # DETERMINISTIC BRAIN: arms come straight from the brain's reference.
        if BRAIN is not None:
            for k, jn in enumerate(ARM_JOINTS):
                m = arm_motors.get(jn)
                if m is not None:
                    m.setPosition(float(BRAIN.arms[k]))
            return
        # HUMAN gait model: arms come from the model (counter-phase swing +
        # soft elbows), refreshed by _baseline_targets each tick.
        if GP is not None and _LAST_ARMS is not None:
            for k, jn in enumerate(ARM_JOINTS):
                m = arm_motors.get(jn)
                if m is not None:
                    m.setPosition(float(_LAST_ARMS[k]))
            return
        # GAIT-V2: shoulder-pitch swings counter-phase to the same-side leg
        # when GAIT_A_ARM != 0 (matches the trainer's hold-arms swing).
        sw = GAIT_A_ARM * math.sin(ph) if GAIT_A_ARM != 0.0 else 0.0
        for k, (jn, q) in enumerate(zip(ARM_JOINTS, ARM_NOMINAL)):
            m = arm_motors.get(jn)
            if m is None:
                continue
            q = float(q)
            if k == 0:        # left_shoulder_pitch
                q += sw
            elif k == 5:      # right_shoulder_pitch
                q -= sw
            m.setPosition(q)

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
    _DBG_N = int(os.environ.get("G1_DEBUG_N", "30"))   # how many per-tick dbg lines
    _dbg_n = 0
    last_log_ms = 0
    survived = 0
    fell_at = None
    sim_ms = 0

    # Joint-q -> joint-qd finite-difference state. Warm-started to
    # NOMINAL after settle so the very first policy tick doesn't see
    # a fake (joint_q_now - 0)/dt spike. prev_joint_q is ALSO used as the
    # per-tick fallback when a position sensor read fails, so it is kept here
    # even though the qd finite-diff itself now lives in JointVelEstimator.
    prev_joint_q = NOMINAL_LEGS.copy()
    # Joint qd: the GPU trainer's obs qd is EXACT qvel (no lag). The old 30 ms
    # low-pass here added ~30 ms of velocity lag to 13 of 48 obs dims, which a
    # velocity-feedback balance policy is very sensitive to -> a train<->deploy
    # gap. Default now G1_QD_TAU=0 = raw finite-diff (lowest lag, matches the
    # trainer; the trainer's obs_noise DR covers the extra quantization noise).
    # The finite-diff(+low-pass) is now the SHARED g1_env_core.JointVelEstimator
    # (one definition for trainer + deploy); behaviour is byte-identical to the
    # previous inline qd_alpha + joint_qd_smoothed update.
    _qd_tau = float(os.environ.get("G1_QD_TAU", "0.0"))
    qd_est = core.JointVelEstimator(step_dt, _qd_tau)
    qd_est.reset(NOMINAL_LEGS.copy())
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
            hold_arms()
            if robot.step(step_ms) == -1:
                return 0
            sim_ms += step_ms
        # short hold at NOMINAL to let it settle
        for jn, q in zip(LEGS_JOINTS, NOMINAL_LEGS):
            motors[jn].setPosition(float(q))
        hold_arms()
        for _ in range(max(1, int(0.5 / step_dt))):
            if robot.step(step_ms) == -1:
                return 0
            sim_ms += step_ms
    else:
        for jn, q in zip(LEGS_JOINTS, NOMINAL_LEGS):
            motors[jn].setPosition(float(q))
        hold_arms()
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
    # Seed the qd estimator's finite-diff baseline with the SAME warm-started
    # settled pose (zeroing its smoothed state, exactly as the old inline
    # joint_qd_smoothed=0 did), so the first policy tick's qd is identical.
    qd_est.reset(prev_joint_q.copy())

    # WALK: gait clock (rad), advanced per tick. Human model default: start
    # in DOUBLE SUPPORT (DS_PHASE) -- phase 0 has the right foot mid-swing.
    _default_phase = str(_ghg.DS_PHASE) if GP is not None else "0.0"
    phase = float(os.environ.get("G1_START_PHASE", _default_phase))
    walk_start_ms = sim_ms + step_ms   # stride ramp clock origin: +1 tick so the
    # FIRST applied baseline has walk_t=0, matching the trainer's first-baseline
    # t_since=0 (deploy was 1 tick / ~0.8% ahead on the stride ramp -> nudged the
    # swing-foot/CoM forward earlier than the policy learned; trainer<->deploy diag)
    # STABILIZE-THEN-WALK launch: for the first LAUNCH_HOLD_S the gait clock
    # is FROZEN (phase held at DS_PHASE) and the stride ramp held at 0, so the
    # robot holds the STAND pose and settles to full height under the balance
    # PD + policy BEFORE it starts stepping -- instead of trying to step while
    # the cold articulation is still sagging (the recurring ~1.1 s launch
    # face-plant). After the hold, the gait clock + ramp engage. The hold can
    # extend until the robot is actually upright (tilt < LAUNCH_HOLD_TILT).
    LAUNCH_HOLD_S = float(os.environ.get("G1_LAUNCH_HOLD_S", "0.0"))
    LAUNCH_HOLD_TILT = float(os.environ.get("G1_LAUNCH_HOLD_TILT", "0.08"))
    # During the launch hold, deepen the squat into the STATICALLY-STABLE
    # stand (hip -0.30/knee 0.52 puts the CoM behind the foot, stands with
    # zero control) -- the walk's own launch pose (hip -0.22/knee 0.40) is
    # CoM-FORWARD and tips. After the hold the squat blends OUT and the walk
    # policy + stride blend IN over LAUNCH_BLEND_S.
    LAUNCH_SQUAT_HIP = float(os.environ.get("G1_LAUNCH_SQUAT_HIP", "-0.08"))
    LAUNCH_SQUAT_KNEE = float(os.environ.get("G1_LAUNCH_SQUAT_KNEE", "0.12"))
    LAUNCH_BLEND_S = float(os.environ.get("G1_LAUNCH_BLEND_S", "1.0"))
    _launch_start_ms = sim_ms
    _launched = (LAUNCH_HOLD_S <= 0.0)
    _launch_done_ms = sim_ms

    # ── WALK<->STAND mode machine (two-policy). mu: 1=walk, 0=stand. When a
    # STAND policy is loaded, mu blends control between the walk policy and the
    # stand policy. Schedule (for the milestone demo): stand at G1_STAND_AT_S
    # for G1_STAND_FOR_S, else walk. mu ramps over G1_MODE_BLEND_S. The gait
    # clock pauses while standing so the stride resumes where it left off.
    STAND_AT_S = float(os.environ.get("G1_STAND_AT_S", "0.0"))      # 0 = never
    STAND_FOR_S = float(os.environ.get("G1_STAND_FOR_S", "3.0"))
    MODE_BLEND_S = float(os.environ.get("G1_MODE_BLEND_S", "1.5"))
    _GAIT_RAMP_S = float(os.environ.get("G1_GAIT_RAMP_S", "2.0"))
    stand_last_action = np.zeros(NJ, dtype=np.float32)

    # Reload regression harness: G1_RELOAD_AT_S=<sim s> triggers a world
    # reload (same as the GUI's Ctrl+Shift+R) at that sim time, so the
    # post-reload behaviour can be tested headless. The reloaded world
    # starts a fresh controller instance (which truncates G1_DEPLOY_LOG,
    # so the log left after the run IS the post-reload run).
    reload_at_s = float(os.environ.get("G1_RELOAD_AT_S", "0"))
    reloaded_marker = False
    # Once-only across instances: the reloaded controller would otherwise
    # also reload at ITS t=reload_at_s, looping forever.
    _reload_flag = os.environ.get("G1_RELOAD_FLAG_FILE", "")
    if reload_at_s > 0 and _reload_flag and os.path.exists(_reload_flag):
        reloaded_marker = True

    _trace_ref_path = os.environ.get("G1_TRACE_REF")
    _trace_ref = open(_trace_ref_path, "w", buffering=1) if _trace_ref_path else None

    # Frame-stack memory (G1_OBS_STACK > 1 -> stacked-obs policy).
    from collections import deque
    OBS_STACK = int(os.environ.get("G1_OBS_STACK", "1"))
    obs_hist = deque()
    # Reference lookahead (G1_OBS_LOOKAHEAD="0.1,0.4" must match trainer's
    # --obs-lookahead): exact future model targets appended AFTER the stack.
    OBS_LOOK = [float(x) for x in
                os.environ.get("G1_OBS_LOOKAHEAD", "").split(",") if x.strip()]
    # Velocity-conditioned policy (G1_VX_CMD_MAX > 0, must match trainer
    # --vx-cmd-max): the NORMALISED forward-speed command is appended to the
    # obs as the LAST element. The command itself = _w*VX_NOMINAL (_w is the
    # stride scale, 1=full walk, 0=stand), exactly mirroring the trainer where
    # vx_cmd scales the gait toward nominal the same way. Commanding vx->0
    # (via the G1_STAND_AT_S schedule, which drives _w->0) makes the SINGLE
    # policy decelerate and settle into a stand -- the stop-in-the-middle
    # milestone with no second policy and no fragile hand-off.
    VX_CMD_MAX = float(os.environ.get("G1_VX_CMD_MAX", "0.0"))
    VX_NOMINAL = float(os.environ.get("G1_GAIT_VX", "0.4"))
    # VC launch-from-stand: launch STOPPED (vx_cmd=0 -> deep-squat nominal,
    # consistent obs+pose), settle, then accelerate to walk. Verified the robust
    # launch in the no-freeze regime (walked +11.65 m). (It only broke when
    # combined with the gait-clock FREEZE, which made the launch phase frozen
    # == out-of-distribution; the freeze is now off, so launch-from-stand is
    # back.) Set G1_VX_LAUNCH_HOLD_S=0 to launch walking instead.
    VX_LAUNCH_HOLD_S = (float(os.environ.get("G1_VX_LAUNCH_HOLD_S", "1.2"))
                        if VX_CMD_MAX > 0.0 else 0.0)
    # Speed below which the gait clock freezes -- MUST MATCH the trainer's
    # --vx-phase-freeze. Default OFF (0): the shipped policy trains with the
    # freeze off, and freezing the phase in deploy for a no-freeze policy is
    # OUT-OF-DISTRIBUTION -> it tips at the launch-hold stand (this exact
    # train/deploy mismatch caused a string of phantom "launch falls"). Only
    # set >0 when deploying a policy that was TRAINED with the freeze on.
    VX_PHASE_FREEZE = float(os.environ.get("G1_VX_PHASE_FREEZE", "0.0"))
    # STAND action-fade (DISABLED by default): the idea was to fade the policy
    # out as the robot stands (_w->0) so it holds a pure-pose stand. But the
    # deep-squat walk nominal is NOT laterally stable as a pure pose -- the
    # policy is doing essential roll balancing, so fading it tips the robot in
    # ROLL (verified: pure-pose stand fell at ~1.2 s, roll>1.6). The policy is
    # required at the stand; the ~0.13 m/s residual creep is the cost of a
    # policy-based stand. Kept gated for a future statically-stable stand pose.
    VX_STAND_FADE_W = float(os.environ.get("G1_VX_STAND_FADE_W", "0.0"))
    _w = 0.0 if VX_CMD_MAX > 0.0 else 1.0   # VC launches from a STAND (_w=0)

    # Wall-clock split: time inside robot.step() (simulator+physics) vs the
    # controller's own work (obs build + ONNX + motor writes). Logged on the
    # 1 Hz status line as a realtime factor so slowdowns are attributable.
    import time as _time
    wall_sim = 0.0     # seconds spent inside robot.step()
    wall_ctrl = 0.0    # seconds spent in controller computation
    wall_win0 = _time.perf_counter()
    sim_win0_ms = sim_ms
    _gerr_sum = 0.0   # ghost-tracking error: mean|achieved_leg - ghost_baseline|
    _gerr_n = 0       #   accumulated per 1 Hz log window (lower = more ghost-like)
    _t_loop = _time.perf_counter()

    while True:
        _t0 = _time.perf_counter()
        wall_ctrl += _t0 - _t_loop
        if robot.step(step_ms) == -1:
            break
        _t_loop = _time.perf_counter()
        wall_sim += _t_loop - _t0
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

        if reload_at_s > 0 and not reloaded_marker and sim_ms >= reload_at_s * 1000.0:
            reloaded_marker = True
            if _reload_flag:
                try:
                    open(_reload_flag, "w").write("reloaded\n")
                except Exception:
                    pass
            say(f"[g1_stand_deploy] RELOADING WORLD at t={sim_ms/1000:.2f}s (test)\n")
            try:
                robot.worldReload()
                continue
            except Exception as e:
                say(f"[g1_stand_deploy] worldReload failed: {e}\n")

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
        # Computed by the SHARED g1_env_core.JointVelEstimator (identical to the
        # prior inline finite-diff + low-pass). prev_joint_q is still advanced
        # below because it doubles as the per-tick sensor-read fallback above.
        joint_qd = qd_est.update(joint_q).astype(np.float32)
        prev_joint_q = joint_q.copy()

        proj_g = np.array(_proj_gravity_from_ori(ori), dtype=np.float32)

        # OBS FIX 2026-06-09: the GPU trainer's obs angular velocity is MuJoCo
        # free-joint qvel[3:6] = BODY frame (verified empirically). getVelocity()
        # returns WORLD-frame omega, so the policy was being fed the wrong-frame
        # ang-vel (the reason the trained policy DESTABILISED the deploy). Rotate
        # world omega into the body frame: omega_body = R^T * omega_world, where
        # ori = getOrientation() is the body->world rotation (row-major; confirmed
        # by the roll/pitch formula in _ori_matrix_to_rpy). Linear vel is WORLD in
        # both (also verified) so it is left unchanged. The R^T rotation is now
        # the SHARED g1_env_core.world_ang_to_body_matrix (one definition for
        # trainer + deploy); identical math, re-wrapped to float32 for the obs.
        ang_vel_body = np.array(
            core.world_ang_to_body_matrix(ori, ang_vel), dtype=np.float32)

        # OmniQuad-style residual recipe: targets = NOMINAL + analytic balance + policy delta.
        # roll/pitch-rate via FINITE-DIFF (matches the trainer's baseline, which uses
        # (roll-prev)/DT, not the world ang-vel components the deploy used before).
        roll_rate = (roll - prev_roll) / step_dt
        pitch_rate = (pitch - prev_pitch) / step_dt
        prev_roll = roll
        prev_pitch = pitch
        # STABILIZE-THEN-WALK: hold the stand (freeze clock + ramp at 0) until
        # LAUNCH_HOLD_S has passed AND the robot is upright, then release into
        # the walk. While holding, walk_start_ms tracks now so walk_t stays 0.
        if not _launched:
            held_s = (sim_ms - _launch_start_ms) / 1000.0
            tilt_ok = (roll * roll + pitch * pitch) < (LAUNCH_HOLD_TILT ** 2)
            if held_s >= LAUNCH_HOLD_S and tilt_ok:
                _launched = True           # release: ramp clock starts now
                _launch_done_ms = sim_ms   # blend origin
            else:
                walk_start_ms = sim_ms     # keep walk_t = 0 (stand, no stride)
        # WALK<->STAND progress _w. When standing is commanded _w ramps 1->0;
        # the STRIDE is scaled by _w (walk_t capped at _w*ramp) so the steps
        # SHRINK to nothing -> the robot DECELERATES instead of toppling
        # forward on its momentum, then settles into the stand.
        _elapsed = (sim_ms - _launch_start_ms) / 1000.0
        # Stand is commanded (a) during the VC launch settle (_elapsed <
        # VX_LAUNCH_HOLD_S, so the robot launches stopped then accelerates), or
        # (b) during the scheduled stop-in-the-middle window.
        _want_stand = ((_elapsed < VX_LAUNCH_HOLD_S) or
                       (STAND_AT_S > 0.0 and
                        STAND_AT_S <= _elapsed < STAND_AT_S + STAND_FOR_S))
        # VC drives _w straight from the command (no stabilize-then-walk gate);
        # plain walk policies keep the original _launched guard.
        _w_tgt = 0.0 if (_want_stand and (_launched or VX_CMD_MAX > 0.0)) else 1.0
        _dw = step_dt / max(MODE_BLEND_S, 1e-3)
        _w = _w + max(-_dw, min(_dw, _w_tgt - _w))
        # Policy action-fade for the VC stand: 1 while walking, ->0 as it stands.
        if VX_CMD_MAX > 0.0 and VX_STAND_FADE_W > 0.0:
            _afade = min(1.0, _w / VX_STAND_FADE_W)
        else:
            _afade = 1.0
        walk_t = min((sim_ms - walk_start_ms) / 1000.0, _w * _GAIT_RAMP_S)
        if BRAIN is not None:
            # The deterministic brain owns the clock and produces the full
            # target directly (feedforward ghost + balance feedback layers).
            # Body-frame angular velocity for ALL three rates: the brain is
            # tuned in the offline harness against d.qvel[3:6] (body-frame omega),
            # so feed it the same here -- not the finite-diff Euler roll/pitch
            # rates the policy path uses (those are inconsistent across axes).
            _bs = BrainState(
                roll=roll, pitch=pitch, yaw=yaw,
                roll_rate=float(ang_vel_body[0]), pitch_rate=float(ang_vel_body[1]),
                yaw_rate=float(ang_vel_body[2]),
                vx=float(lin_vel[0]), vy=float(lin_vel[1]), vz=float(lin_vel[2]),
                bz=bz, joint_q=joint_q, joint_qd=joint_qd, t=walk_t)
            baseline = BRAIN.step(_bs, step_dt)
            phase = BRAIN.phase
        else:
            baseline = _baseline_targets(roll, pitch, roll_rate, pitch_rate,
                                         vx=float(lin_vel[0]), vy=float(lin_vel[1]),
                                         bz=bz, g=bal_gains, phase=phase,
                                         walk_t=walk_t)
        # Ghost-tracking error this tick: |achieved leg pose - open-loop ghost
        # reference (baseline)|, captured BEFORE any launch-squat blend modifies
        # baseline. Lower mean over the walk = the gait is closer to the ghost.
        _gerr_sum += float(np.mean(np.abs(joint_q - np.asarray(baseline[:NJ], dtype=np.float32))))
        _gerr_n += 1
        if fallback or BRAIN is not None:
            # No policy — the brain (or open-loop gait) drives targets directly.
            targets = baseline
            action = np.zeros(NJ, dtype=np.float32)
        else:
            gait_obs = np.array([math.sin(phase), math.cos(phase)], dtype=np.float32)
            obs = np.concatenate([
                lin_vel, ang_vel_body, proj_g,
                joint_q - NOMINAL_LEGS, joint_qd, last_action, gait_obs,
            ]).astype(np.float32)
            assert obs.shape[0] == OBS_DIM, f"obs shape {obs.shape} != {OBS_DIM}"
            # Frame stack (G1_OBS_STACK=K, must match the trainer's
            # --obs-stack): policy sees the last K obs, NEWEST FIRST.
            if OBS_STACK > 1:
                if not obs_hist:
                    obs_hist.extend([obs] * OBS_STACK)
                else:
                    obs_hist.appendleft(obs)
                    while len(obs_hist) > OBS_STACK:
                        obs_hist.pop()
                obs = np.concatenate(list(obs_hist)).astype(np.float32)
            if OBS_LOOK and GP is not None:
                _om = 2.0 * math.pi * GAIT_FREQ
                _wt = walk_t
                _la = []
                for _dt in OBS_LOOK:
                    _legs, _, _ = _ghg.targets_np(
                        (phase + _om * _dt) % (2.0 * math.pi), GP,
                        t_since_start=_wt + _dt)
                    _la.append(_legs - NOMINAL_LEGS)
                obs = np.concatenate([obs] + _la).astype(np.float32)
            if VX_CMD_MAX > 0.0:
                # Velocity command (normalised), appended LAST to match the
                # trainer's _obs_full. _w is the live stride scale, so the
                # commanded speed = _w*VX_NOMINAL: full walk at _w=1, stand at
                # _w=0. The gait baseline + lookahead are already scaled by _w
                # via walk_t, so this obs is consistent with what the legs do.
                obs = np.concatenate([
                    obs, np.array([_w * VX_NOMINAL / VX_CMD_MAX],
                                  dtype=np.float32)]).astype(np.float32)
            try:
                out = sess.run(None, {"obs": obs[None, :]})
                action = np.clip(out[0][0], -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[g1_stand_deploy] ONNX inference failed: {e}; falling back\n")
                fallback = True
                action = np.zeros(NJ, dtype=np.float32)
            # STABILIZE-THEN-WALK blend: during the hold (beta=0) hold the
            # deeper STATICALLY-STABLE squat with ZERO policy action; after
            # hand-off, beta 0->1 over LAUNCH_BLEND_S blends the squat OUT and
            # the walk policy + stride IN. (No-op when the launch hold is off.)
            if LAUNCH_HOLD_S > 0.0:
                if not _launched:
                    _beta = 0.0
                else:
                    _beta = min(1.0, (sim_ms - _launch_done_ms)
                                / (LAUNCH_BLEND_S * 1000.0 + 1e-6))
                _sq = 1.0 - _beta
                baseline[0] += LAUNCH_SQUAT_HIP * _sq      # L hip pitch
                baseline[6] += LAUNCH_SQUAT_HIP * _sq      # R hip pitch
                baseline[3] += LAUNCH_SQUAT_KNEE * _sq     # L knee
                baseline[9] += LAUNCH_SQUAT_KNEE * _sq     # R knee
                targets = baseline + ACT_SCALE_VEC * action * _beta * _afade
            else:
                targets = baseline + ACT_SCALE_VEC * action * _afade

            # ── TWO-POLICY WALK<->STAND blend (the milestone) ──
            if stand_sess is not None:
                # STAND obs = walk proprio MINUS the 2 gait-phase dims (48-d).
                s_obs = np.concatenate([
                    lin_vel, ang_vel_body, proj_g,
                    joint_q - STAND_NOMINAL_LEGS, joint_qd, stand_last_action,
                ]).astype(np.float32)
                try:
                    s_action = np.clip(stand_sess.run(
                        None, {stand_sess.get_inputs()[0].name: s_obs[None, :]}
                    )[0][0], -1.0, 1.0).astype(np.float32)
                except Exception:
                    s_action = np.zeros(NJ, dtype=np.float32)
                stand_last_action = s_action
                stand_targets = STAND_NOMINAL_LEGS + ACT_SCALE * s_action
                # Blend control by _w (1=walk policy, 0=stand policy). The
                # stride has already shrunk with _w, so by the time the stand
                # policy takes over the robot is ~stopped.
                targets = (1.0 - _w) * stand_targets + _w * targets

        # CLAMP to joint limits exactly as the GPU trainer does
        # (clamp(baseline+RES_SCALE*applied, jl_lo, jl_hi)). Without this the
        # policy's residual drives ankle_roll past ±0.262 -> limit slam -> kick.
        targets = np.clip(targets, LIM_LO, LIM_HI)

        for i, jn in enumerate(LEGS_JOINTS):
            motors[jn].setPosition(float(targets[i]))
        if arm_motors:
            hold_arms(phase)

        # MODEL-vs-REALITY trace (G1_TRACE_REF=<path>): per tick dump the gait
        # MODEL reference (baseline), the model+RL command (targets) and the
        # ACTUAL measured pose (joint_q) so the model can be visualised against
        # what the robot physically did. One CSV row per tick.
        if _trace_ref is not None:
            _trace_ref.write(
                "%.3f,%.4f,%.4f," % (sim_ms / 1000.0, bx, phase)
                + ",".join("%.4f" % v for v in joint_q)
                + "," + ",".join("%.4f" % v for v in baseline)
                + "," + ",".join("%.4f" % v for v in targets) + "\n")

        # Feed back the APPLIED (faded) action so the last_action obs reflects
        # what physically happened (during the stand fade the policy's intended
        # action is scaled down; an un-faded last_action would tell the policy
        # it stepped when it stood).
        last_action = action * _afade
        # WALK: advance the gait clock for the next tick (obs + baseline).
        # ADAPTIVE PHASE: slow the clock when off-balance (linger to recover);
        # full rate when upright. Mirrors the trainer's state-dependent gate.
        if PHASE_GATE_TILT != 0.0 or PHASE_GATE_RATE != 0.0:
            _gate = 1.0 - PHASE_GATE_TILT * (roll * roll + pitch * pitch) \
                - PHASE_GATE_RATE * (roll_rate * roll_rate + pitch_rate * pitch_rate)
            _gate = max(PHASE_GATE_FLOOR, min(1.0, _gate))
        else:
            _gate = 1.0
        # During the launch hold the stride is already 0 (walk_t=0 -> ramp 0
        # -> the legs sit in the stand), so we do NOT need to freeze the
        # phase; freezing is OUT-OF-DISTRIBUTION for an adaptive-phase policy
        # (it never saw a frozen clock). Let the phase advance (in-dist);
        # only G1_LAUNCH_FREEZE_PHASE=1 restores the hard freeze.
        if (not _launched) and os.environ.get("G1_LAUNCH_FREEZE_PHASE", "0") == "1":
            _gate = 0.0
        # Pause the gait clock once basically standing (_w<0.15) so the stride
        # resumes from where it left off when walking continues.
        if stand_sess is not None and _w < 0.15:
            _gate = 0.0
        # VELOCITY-CONDITIONED phase freeze (mirrors the trainer): scale the
        # clock by the speed command so it FREEZES as vx_cmd -> 0 -> a static
        # stand. vx_cmd = _w*VX_NOMINAL.
        if VX_CMD_MAX > 0.0 and VX_PHASE_FREEZE > 0.0:
            _gate = _gate * min(1.0, (_w * VX_NOMINAL) / VX_PHASE_FREEZE)
        # The deterministic brain advances its own clock inside step(); leave it.
        if BRAIN is None:
            phase = (phase + GAIT_PHASE_DT * _gate) % (2.0 * math.pi)

        if os.environ.get("G1_DEBUG_OBS") and not fallback and _dbg_n < 3:
            _dbg_n += 1
            say(f"[obs {_dbg_n}] {np.array2string(obs, precision=3, max_line_width=200, separator=',')}\n")

        if _dbg and _dbg_n < _DBG_N:
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
            now = _time.perf_counter()
            win_wall = max(now - wall_win0, 1e-9)
            win_sim = (sim_ms - sim_win0_ms) / 1000.0
            rtf = win_sim / win_wall      # realtime factor (1.0 = realtime)
            tot = max(wall_sim + wall_ctrl, 1e-9)
            _gerr = _gerr_sum / max(_gerr_n, 1)
            say(f"[g1_stand_deploy] t={sim_ms/1000:5.1f}s "
                f"bx={bx:+.2f} bz={bz:.3f} roll={roll:+.3f} "
                f"pitch={pitch:+.3f} gerr={_gerr:.3f}  {tag}  "
                f"{'ONNX' if not fallback else 'PD'}  "
                f"rtf={rtf:.2f}x sim%={100*wall_sim/tot:.0f} ctrl%={100*wall_ctrl/tot:.0f}\n")
            last_log_ms = sim_ms
            wall_win0 = now
            sim_win0_ms = sim_ms
            _gerr_sum = 0.0
            _gerr_n = 0
            wall_sim = 0.0
            wall_ctrl = 0.0

    return 0


if __name__ == "__main__":
    sys.exit(main())
