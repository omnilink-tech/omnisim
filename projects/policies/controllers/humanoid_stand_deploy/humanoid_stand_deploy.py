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

"""Generic DETERMINISTIC humanoid stand deploy controller (no RL).

This is the robot-agnostic generalisation of the proven G1 "pure-pose"
stand (see ``g1_stand_arms_deploy.py`` / ``run_g1_standwave_pose_deploy.ps1``).
The recipe, ported verbatim:

  * Hold a STATICALLY-STABLE squat pose (CoM over the feet) with a stiff
    position hold. The stiffness is the Newton position PD set globally by
    the launch script (``OMNISIM_NEWTON_TARGET_KE`` / ``_KD``), so here we
    simply ``setPosition`` every actuated joint at its nominal every tick.
  * NO RL policy, NO analytic ankle PD by default — the G1 work proved both
    of those DESTABILISE a stiff-contact Newton/MuJoCo stand at handover.
  * Optional ankle-pitch / ankle-roll balance PD (off by default) for robots
    whose static squat is too marginal to hold open-loop.

The robot is described entirely by a JSON spec (``HUMANOID_STAND_SPEC``), so
the same controller drives H1, Valkyrie, or any future humanoid:

    {
      "name": "h1",
      "joint_order": ["left_hip_yaw_joint", ...],   # every actuated joint
      "nominal":     {"left_hip_yaw_joint": 0.0, ...},  # rad, per joint
      "ankle_pitch_joints": ["left_ankle_joint", "right_ankle_joint"],
      "ankle_roll_joints":  [],
      "upright": {"bz_min": 0.55, "roll_max": 0.8, "pitch_max": 0.8},
      "settle_s": 0.2,
      "arm_hold": ["left_shoulder_pitch_joint", ...]   # optional: joints to
                                                       # crank torque/PID on
    }

Any joint present in ``joint_order`` but missing a ``nominal`` entry is held
at 0.0. Missing motors warn but do not fail (the link still adds mass).

Env vars:
    HUMANOID_STAND_SPEC   path to the robot stand spec JSON (required)
    HSTAND_ANKLE_KP/_KD   ankle-pitch balance gains (default 0 = pure pose)
    HSTAND_ANKLE_KP_R/_KD_R   ankle-roll gains (default = pitch gains)
    HSTAND_BAL_CLAMP      clamp on the balance correction, rad (default 0.2)
    HSTAND_SETTLE_S       override the spec settle time
    HSTAND_WARMUP_RELOAD  1 = reload once at startup (warm articulation)
    HSTAND_SQUAT          1 = run the squat overlay (spec `squat` block): the legs
                          blend deeper into a knee-bend, hold, and rise back, x reps
    HSTAND_SQUAT_START / _DESCEND / _HOLD / _RISE / _BETWEEN   squat phase times, s
    HSTAND_SQUAT_REPS     number of squat reps before holding the stand
    HSTAND_SQUAT_FF_ANK / _FF_HIP   open-loop depth-proportional lean-back gains
    HSTAND_SQUAT_BZ_DROP  expected pelvis sink (m) — relaxes the fall-height floor
    HUMANOID_STAND_LOG / OMNISIM_DEPLOY_LOG   side-log path
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[4]

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _ori_matrix_to_rpy(o):
    return (math.atan2(o[7], o[8]),
            math.asin(max(-1.0, min(1.0, -o[6]))),
            math.atan2(o[3], o[0]))


def _load_spec() -> dict:
    p = os.environ.get("HUMANOID_STAND_SPEC")
    if not p:
        raise SystemExit("[hstand] HUMANOID_STAND_SPEC not set (path to robot stand spec JSON)")
    p = Path(p)
    if not p.is_absolute():
        p = _REPO / p
    with open(p) as f:
        return json.load(f)


def main() -> int:
    spec = _load_spec()
    name = spec.get("name", "humanoid")
    joint_order = list(spec["joint_order"])
    nominal_map = {j: 0.0 for j in joint_order}
    nominal_map.update({k: float(v) for k, v in spec.get("nominal", {}).items()})

    ankle_pitch = list(spec.get("ankle_pitch_joints", []))
    ankle_roll = list(spec.get("ankle_roll_joints", []))
    hip_pitch = list(spec.get("hip_pitch_joints", []))
    # Shoulder joints used by the reactive ARM-balance strategy (move the arms to
    # help balance instead of holding them rigidly). Derived by name so it needs
    # no extra spec; falls back to spec-declared lists if present.
    shoulder_pitch = list(spec.get("shoulder_pitch_joints",
                                   [j for j in joint_order if "shoulder_pitch" in j]))
    shoulder_roll = list(spec.get("shoulder_roll_joints",
                                  [j for j in joint_order if "shoulder_roll" in j]))

    # Fore/aft CoM trim (deterministic balance over the feet), mirroring G1's
    # G1_ANK_PITCH_BIAS / G1_HIP_PITCH_BIAS. A negative ankle bias leans the
    # planted body backward (pulls CoM behind the ankle); the hip bias leans
    # the torso. Used to centre a marginal squat that tips fore/aft. Default 0.
    ank_bias = _envf("HSTAND_ANK_BIAS", spec.get("ank_bias", 0.0))
    hip_bias = _envf("HSTAND_HIP_BIAS", spec.get("hip_bias", 0.0))
    for jn in ankle_pitch:
        if jn in nominal_map:
            nominal_map[jn] += ank_bias
    for jn in hip_pitch:
        if jn in nominal_map:
            nominal_map[jn] += hip_bias
    arm_hold = set(spec.get("arm_hold", []))
    upright = spec.get("upright", {})
    bz_min = float(upright.get("bz_min", 0.45))
    roll_max = float(upright.get("roll_max", 0.8))
    pitch_max = float(upright.get("pitch_max", 0.8))
    settle_s = _envf("HSTAND_SETTLE_S", spec.get("settle_s", 0.2))

    # Balance PD (default OFF — pure pose). Restoring an angle bias on the
    # ankle joints proportional to body roll/pitch. The G1 work showed this
    # destabilises a stiff stand, so default is 0; enable per-robot only if a
    # static squat is too marginal.
    kp_p = _envf("HSTAND_ANKLE_KP", spec.get("ankle_kp", 0.0))
    kd_p = _envf("HSTAND_ANKLE_KD", spec.get("ankle_kd", 0.0))
    kp_r = _envf("HSTAND_ANKLE_KP_R", spec.get("ankle_kp_r", kp_p))
    kd_r = _envf("HSTAND_ANKLE_KD_R", spec.get("ankle_kd_r", kd_p))
    bal_clamp = _envf("HSTAND_BAL_CLAMP", 0.2)

    # ── Reactive fore/aft LEAN (the ankle strategy) ──
    # A small pitch-ONLY feedback (roll PD destabilised, so we leave it out) that
    # leans the ankles BACK when the body is pitching/moving forward -- exactly the
    # case a cube-from-behind creates (it shoves the CoM forward past the foot, and
    # the passive stand face-plants). The base forward velocity vx is the FASTEST
    # signal (it spikes the instant the cube lands, before the tilt develops), so
    # it gets the most weight. Pitch + pitch-rate add the slower restoring terms.
    # Symmetric, so it also catches a front-hit (lean forward). Default OFF.
    lean = spec.get("lean") or {}
    lean_on = (os.environ.get("HSTAND_LEAN", "1" if lean.get("enabled") else "0").strip() != "0")
    lean_kv = _envf("HSTAND_LEAN_KV", lean.get("kv", 0.14))   # forward-velocity gain (fast)
    lean_kp = _envf("HSTAND_LEAN_KP", lean.get("kp", 1.6))    # forward-tilt gain
    lean_kd = _envf("HSTAND_LEAN_KD", lean.get("kd", 0.25))   # tilt-rate damping
    lean_hip = _envf("HSTAND_LEAN_HIP", lean.get("hip", 0.35))# hip share of the lean
    lean_clamp = _envf("HSTAND_LEAN_CLAMP", lean.get("clamp", 0.30))
    # Deadband: zero the lean below this (rad) so sensor noise doesn't make the
    # ankles twitch in the steady stand (the "vibration"); a real cube spike is
    # well above it. Soft (subtracts the band) so there's no discontinuity.
    # 0.008 is the tuned sweet spot: ~7x less steady-state jitter (0.057->0.008)
    # yet still catches the cubes. Bigger (0.014+) starts to drift/fall; input
    # smoothing < 1 adds loop lag that makes the lean RING, so leave it at 1 (off).
    lean_db = _envf("HSTAND_LEAN_DEADBAND", lean.get("deadband", 0.008))
    lean_smooth = _envf("HSTAND_LEAN_SMOOTH", lean.get("smooth", 1.0))  # input low-pass (1 = off)

    # ── Reactive ARM balance (the "move your arms to keep your balance" strategy) ──
    # The arms are heavy enough (~3 kg/pair) to act as a CoM-shift + counter-rotation
    # effector. We feed body pitch/roll back into the shoulders: lean FORWARD and the
    # arms swing BACK (shoulder_pitch up) to pull the CoM behind the ankle; tip to one
    # side and the arms swing the OTHER way. Proportional (CoM shift, holds a steady
    # lean) + rate damping (catches the cube impulse). Returns to the down rest pose
    # when calm. Spec block "arm_balance": {enabled, kp_pitch, kd_pitch, kp_roll,
    # clamp}. Enable with HSTAND_ARM_BALANCE=1.
    armb = spec.get("arm_balance") or {}
    armb_on = (os.environ.get("HSTAND_ARM_BALANCE", "1" if armb.get("enabled") else "0").strip() != "0")
    armb_kp = _envf("HSTAND_ARM_KP", armb.get("kp_pitch", 1.8))   # P: pitch -> shoulder_pitch
    armb_ki = _envf("HSTAND_ARM_KI", armb.get("ki_pitch", 0.0))   # I: drives the body BACK to 0 rad (home)
    armb_kd = _envf("HSTAND_ARM_KD", armb.get("kd_pitch", 0.30))  # D: pitch-rate damping
    armb_kr = _envf("HSTAND_ARM_KR", armb.get("kp_roll", 1.2))    # P: roll  -> shoulder_roll
    armb_kir = _envf("HSTAND_ARM_KIR", armb.get("ki_roll", 0.0))  # I (roll): drives roll back to 0
    armb_clamp = _envf("HSTAND_ARM_CLAMP", armb.get("clamp", 1.0))  # max arm swing (rad)
    armb_iclamp = _envf("HSTAND_ARM_ICLAMP", armb.get("iclamp", 1.2))  # integral windup bound (rad)

    # ── Integral auto-trim ──
    # The ankle "lean" reacts only to FAST disturbances (relative pitch), so a slow
    # CoM creep -- e.g. the cumulative forward shove of cube after cube -- ratchets
    # the lean up until it topples. A slow integral of ABSOLUTE pitch/roll nudges the
    # ankle/hip trim to hold the body upright over minutes, the strongest effector for
    # a steady offset (the arms catch the fast hits). ki small => no fight with impulses.
    trim_on = (os.environ.get("HSTAND_TRIM", "1" if spec.get("auto_trim", {}).get("enabled") else "0").strip() != "0")
    trim_kp = _envf("HSTAND_TRIM_KP", spec.get("auto_trim", {}).get("kp", 1.0))      # P on absolute pitch
    trim_ki = _envf("HSTAND_TRIM_KI", spec.get("auto_trim", {}).get("ki", 0.20))     # I on absolute pitch
    # Optional D (rate) term -> the hip becomes a full PID FAST balancer, not just a
    # slow return-to-home. Default 0 (the slow P+I behaviour the cube-defense stand
    # relies on); the arm-motion skill turns it up so the LEGS catch fore/aft tilts
    # while the arms are busy with the exercises (the arms can't balance when raised).
    trim_kd = _envf("HSTAND_TRIM_KD", spec.get("auto_trim", {}).get("kd", 0.0))      # D on pitch rate
    trim_clamp = _envf("HSTAND_TRIM_CLAMP", spec.get("auto_trim", {}).get("clamp", 0.22))
    # Separate integral-windup bound so a strong output clamp doesn't let the slow
    # integral wind all the way up (defaults to clamp for backward compatibility).
    trim_iclamp = _envf("HSTAND_TRIM_ICLAMP", spec.get("auto_trim", {}).get("iclamp", trim_clamp))
    trim_hip = _envf("HSTAND_TRIM_HIP", spec.get("auto_trim", {}).get("hip", 0.35))

    # ── One-leg balance ──
    # Phased: (1) SHIFT the CoM laterally over the support foot via coordinated
    # ankle+hip roll (pelvis stays ~level), ramp the arms OUT as outriggers; (2) LIFT
    # the swing leg (hip flex + knee tuck); (3) HOLD with a support-ankle roll PD +
    # the arms swinging for roll. Spec "one_leg": {enabled, support, shift_s, lift_s,
    # lat_ankle, lat_hip, lift_hip, lift_knee, arm_out, ankle_kp, ankle_kd}.
    oneleg = spec.get("one_leg") or {}
    oneleg_on = (os.environ.get("HSTAND_ONELEG", "1" if oneleg.get("enabled") else "0").strip() != "0")
    ol_support = os.environ.get("HSTAND_OL_SUPPORT", oneleg.get("support", "left")).strip()
    ol_shift_s = _envf("HSTAND_OL_SHIFT_S", oneleg.get("shift_s", 3.0))
    ol_lift_s = _envf("HSTAND_OL_LIFT_S", oneleg.get("lift_s", 2.5))
    # Lateral CoM regulation: drive base y onto the support foot y (PI -> a body
    # lean via support ankle + hips). +lat moves base -y (measured), so the
    # negative-feedback gain is positive: lat_cmd = kp*(by-target)+i.
    ol_y_target = _envf("HSTAND_OL_YTARGET", oneleg.get("y_target", 0.085))  # |foot y|; signed by support
    ol_lat_kp = _envf("HSTAND_OL_LAT_KP", oneleg.get("lat_kp", 1.6))
    ol_lat_ki = _envf("HSTAND_OL_LAT_KI", oneleg.get("lat_ki", 1.1))
    ol_lat_kd = _envf("HSTAND_OL_LAT_KD", oneleg.get("lat_kd", 0.10))
    ol_lat_clamp = _envf("HSTAND_OL_LAT_CLAMP", oneleg.get("lat_clamp", 0.40))
    ol_hip_share = _envf("HSTAND_OL_HIP_SHARE", oneleg.get("hip_share", 0.5))
    ol_lift_hip = _envf("HSTAND_OL_LIFT_HIP", oneleg.get("lift_hip", -0.50))    # swing thigh up
    ol_lift_knee = _envf("HSTAND_OL_LIFT_KNEE", oneleg.get("lift_knee", 1.10))  # swing foot tuck
    ol_arm_out = _envf("HSTAND_OL_ARM_OUT", oneleg.get("arm_out", 1.20))        # arms out as outriggers
    # Capture-point (LIPM) LATERAL stabiliser: the support ankle reacts to the
    # capture point CP = com_y + com_vy/omega (the spot the CoP must cover to stop the
    # fall). The velocity term is what catches a developing tip -- the piece the
    # position-only PI missed. ankle = fast CoP, hip = slow CoM placement, arm = momentum.
    ol_cp_ank = _envf("HSTAND_OL_CP_ANK", oneleg.get("cp_ank", 1.2))    # CP err -> support ankle_roll
    ol_cp_arm = _envf("HSTAND_OL_CP_ARM", oneleg.get("cp_arm", 2.5))    # CP err -> arm roll (momentum)
    ol_cp_clamp = _envf("HSTAND_OL_CP_CLAMP", oneleg.get("cp_clamp", 0.30))
    ol_omega = _envf("HSTAND_OL_OMEGA", oneleg.get("omega", 0.0))       # 0 => compute sqrt(g/z)
    # Option B -- ankle CoP TORQUE via position-PD inversion. The support ankle_roll
    # applies a bounded torque tau (capture-point CoP law) realised as the position
    # target q + (tau + kd*qd)/ke, so the existing ke/kd PD produces exactly tau.
    # tau is clamped to the physical CoP limit so it regulates the contact point
    # rather than levering the whole body (the piece the earlier angle command lacked).
    ol_ankle_torque = (os.environ.get("HSTAND_OL_ANKLE_TORQUE",
                                      "1" if oneleg.get("ankle_torque") else "0").strip() != "0")
    ol_cop_k = _envf("HSTAND_OL_COP_K", oneleg.get("cop_k", 140.0))     # N*m per rad of capture-point err
    ol_tau_max = _envf("HSTAND_OL_TAU_MAX", oneleg.get("tau_max", 14.0))  # CoP torque clamp (F*foot_hw)
    ol_pd_ke = _envf("HSTAND_OL_PD_KE", oneleg.get("pd_ke", 400.0))     # must match the ankle's Newton ke
    ol_pd_kd = _envf("HSTAND_OL_PD_KD", oneleg.get("pd_kd", 60.0))      # must match the ankle's Newton kd
    # Support-leg HEIGHT hold: bearing the whole body on one leg makes the stiff-PD
    # support leg sag (angle error = torque/ke doubles). Extend the support knee
    # (+ hip/ankle compensation to keep the torso vertical) to hold base z.
    ol_height_kp = _envf("HSTAND_OL_HEIGHT_KP", oneleg.get("height_kp", 2.5))
    ol_height_comp = _envf("HSTAND_OL_HEIGHT_COMP", oneleg.get("height_comp", 0.5))
    ol_ext_max = _envf("HSTAND_OL_EXT_MAX", oneleg.get("ext_max", 0.40))
    # Swing-leg MOMENTUM: the lifted leg is a far heavier roll effector than the arms.
    # Ab/adducting it (swing hip_roll) on the capture point throws counter-roll
    # angular momentum -- the strongest deterministic lever for single-support roll.
    ol_swing_k = _envf("HSTAND_OL_SWING_K", oneleg.get("swing_k", 0.0))
    ol_swing_clamp = _envf("HSTAND_OL_SWING_CLAMP", oneleg.get("swing_clamp", 0.6))
    _sup = ol_support if ol_support in ("left", "right") else "left"
    _sw = "right" if _sup == "left" else "left"
    def _legj(side, part):
        return f"{side}_{part}_joint"

    # Optional WAVE: a slow feedforward arm motion ridden on top of the stiff
    # stance (the stand is the solved primitive; the stance absorbs the small,
    # slow arm perturbation — same idea as the G1 stand+wave). Spec block:
    #   "wave": {"pose": {joint: rad, ...},      # raised-arm hold pose
    #            "osc": {"joint": j, "amp": a, "freq_hz": f},  # the waving DOF
    #            "ramp_s": 1.5}                   # ramp nominal->pose
    # Enable with HSTAND_WAVE=1 (or spec wave.enabled). Keep the oscillating DOF
    # distal/low-mass (forearm/elbow) so the CoM barely moves.
    wave = spec.get("wave") or {}
    wave_on = (os.environ.get("HSTAND_WAVE", "1" if wave.get("enabled") else "0").strip() != "0") and bool(wave)
    wave_pose = {k: float(v) for k, v in wave.get("pose", {}).items()}
    wave_osc = wave.get("osc", {})
    wave_ramp_s = float(wave.get("ramp_s", 1.5))

    # ── Table MANIPULATION overlay config (deterministic pick/lift/move/place) ──
    # While the lower body holds the deterministic stand, reach with ONE arm via
    # position-only IK to pick DEF GRASP_* cubes off a table, then lift/move/place
    # them; the OTHER arm keeps arm_balance running as the fast balancer. The
    # fingerless rubber hand grasps by KINEMATIC WELD (attach + carry), not a
    # physical fingered grasp. Spec block "manip" + sibling arm_ik.py. The full
    # runtime (IK chain, cube nodes, state machine) is built just before the loop.
    manip = spec.get("manip") or {}
    manip_on = (os.environ.get("HSTAND_MANIP", "1" if manip.get("enabled") else "0").strip() != "0") and bool(manip)
    manip_reach_side = os.environ.get("HSTAND_MANIP_REACH_ARM", manip.get("reach_arm", "right")).strip()
    manip_reach_side = manip_reach_side if manip_reach_side in ("left", "right") else "right"
    # When manipulating, arm_balance must NOT drive the reaching arm (the overlay
    # owns those 5 joints). Filter it to the BALANCE arm only so the proven fast
    # balancer stays alive on the other arm while this arm does the task.
    armb_pitch_joints = list(shoulder_pitch)
    armb_roll_joints = list(shoulder_roll)
    if manip_on:
        armb_pitch_joints = [j for j in shoulder_pitch if manip_reach_side not in j]
        armb_roll_joints = [j for j in shoulder_roll if manip_reach_side not in j]

    # ── ARM-MOTION overlay config (the "balance while the arms do 3-D exercises" skill) ──
    # Ride a looping library of bilateral arm EXERCISES (front/lateral raises, arm
    # circles, elbow curls, overhead reach, wave) on top of the deterministic stand.
    # The arms are driven by smooth per-joint sinusoids (eased in/out to the rest pose
    # between exercises) ON TOP OF the proven arm_balance correction (which stays live as
    # a safety net); the legs+hips keep the stand and a forward-kinematics CoM-back
    # feedforward pre-compensates the slow CoM shift of the extended arms, so the stand
    # stays centred while both arms move freely. The hand positions are computed each tick
    # by FK (the robot is "aware of its hands") and logged. Unlike manip, this does NOT
    # take the arms away from arm_balance -- the exercise is SUPERIMPOSED, so the fast
    # balancer keeps working through the routine. Spec block "arm_motion" + sibling
    # arm_ik.py FK. Enable with HSTAND_ARM_MOTION=1.
    armm = spec.get("arm_motion") or {}
    armm_on = (os.environ.get("HSTAND_ARM_MOTION", "1" if armm.get("enabled") else "0").strip() != "0") and bool(armm)
    armm_start_s = _envf("HSTAND_ARM_MOTION_START", armm.get("start_s", 3.0))
    armm_ramp_s = float(armm.get("ramp_s", 1.2))
    armm_rest_s = _envf("HSTAND_ARM_MOTION_REST", armm.get("rest_s", 2.0))   # rest dwell between exercises
    armm_loop = bool(armm.get("loop", True))
    armm_exercises = list(armm.get("exercises", []))
    armm_limits = {k: tuple(v) for k, v in armm.get("limits", {}).items()}
    _armm_ff = armm.get("ff", {})
    armm_c_hip = _envf("HSTAND_ARM_MOTION_C_HIP", _armm_ff.get("c_hip", 0.16))
    armm_hip_clamp = _envf("HSTAND_ARM_MOTION_HIP_CLAMP", _armm_ff.get("hip_clamp", 0.10))
    armm_c_ank = _envf("HSTAND_ARM_MOTION_C_ANK", _armm_ff.get("c_ank", 0.28))
    armm_ank_clamp = _envf("HSTAND_ARM_MOTION_ANK_CLAMP", _armm_ff.get("ank_clamp", 0.10))
    armm_ref_pad = float(_armm_ff.get("ref_pad", 0.0))

    side_log_path = os.environ.get("HUMANOID_STAND_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
    # Append (not "w"): a warmup_reload restarts this controller process, and "w"
    # truncated the log so the post-reload (actual) run's monitor/STEP! lines were
    # lost. Append keeps both; clear the file before launch if you want a fresh log.
    side_log = open(side_log_path, "a", buffering=1) if side_log_path else None

    def say(msg: str) -> None:
        sys.stderr.write(msg)
        sys.stderr.flush()
        if side_log is not None:
            side_log.write(msg)
            side_log.flush()

    dbg = os.environ.get("HSTAND_DEBUG", "0").strip() != "0"

    def dsay(msg: str) -> None:
        if dbg:
            say(msg)

    say(f"[hstand:{name}] spec: {len(joint_order)} joints, "
        f"ankle_pd=({kp_p},{kd_p}) ank_bias={ank_bias:+.3f} settle={settle_s:.2f}s "
        f"lean={'ON' if lean_on else 'off'} "
        f"arm_balance={'ON kp=%.2f kd=%.2f kr=%.2f' % (armb_kp, armb_kd, armb_kr) if armb_on else 'off'}\n")

    robot = _Robot()
    dsay(f"[hstand:{name}] robot constructed\n")

    # Cold-load fix (see reference_cold_first_load_trap): a FRESH process
    # under-tracks position targets, so a marginal squat tips before it
    # settles. Reload once so the articulation tracks crisply (warm).
    _warm = os.environ.get("HSTAND_WARMUP_RELOAD", "0").strip() != "0"
    if _warm:
        try:
            _bridge = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
            if str(_bridge) not in sys.path:
                sys.path.insert(0, str(_bridge))
            from omnilink_arm_bridge import warmup_reload as _warmup_reload
            _warmup_reload(robot)
            say(f"[hstand:{name}] warmup_reload done\n")
        except Exception as e:
            say(f"[hstand:{name}] warmup_reload skipped ({e})\n")
            _warm = False

    # ── Environment fingerprint ──
    # Report what ACTUALLY drove the world on this machine (Newton/MuJoCo vs a
    # silent ODE fallback, GPU+driver, warm/cold, build) to the deploy log AND
    # as an on-screen overlay, so a cross-machine regression carries a diagnostic
    # instead of a guess.
    # This runs BEFORE the first robot.step(): the engine finalizes the physics
    # backend (and writes its verdict) on the FIRST simulation tick, so a plain
    # report() here always races it and cries "UNCONFIRMED". pre_step=True draws
    # a neutral "verifying" label instead; the verdict is redrawn for real right
    # after the settle loop below, once the world has provably ticked.
    _envfp = None
    _envfp_fp = {}
    try:
        if str(_REPO) not in sys.path:
            sys.path.insert(0, str(_REPO))
        from projects.policies.common import env_fingerprint as _envfp
        _envfp_fp = _envfp.report(robot, say, warm=_warm, repo_root=_REPO,
                                  pre_step=True)
    except Exception as _e:
        say(f"[hstand:{name}] env fingerprint skipped ({_e})\n")

    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = {}
    sensors = {}
    missing = []
    for jn in joint_order:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            missing.append(jn)
            motors[jn] = None
            sensors[jn] = None
            continue
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors[jn] = s
        except Exception:
            sensors[jn] = None
        # Crank torque + stiffness on explicit hold joints (arms/torso/fingers)
        # so they don't sag under gravity and shift the CoM.
        if jn in arm_hold:
            try:
                m.setAvailableTorque(120.0)
            except Exception:
                pass
            try:
                m.setControlPID(60.0, 0.0, 2.0)
            except Exception:
                pass
    if missing:
        say(f"[hstand:{name}] WARN missing {len(missing)} motors: {missing[:6]}"
            f"{' ...' if len(missing) > 6 else ''}\n")

    # Command the nominal pose immediately.
    for jn in joint_order:
        if motors[jn] is not None:
            motors[jn].setPosition(float(nominal_map[jn]))
    dsay(f"[hstand:{name}] devices ready ({len(joint_order)-len(missing)} motors), nominal commanded\n")

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception as e:
        say(f"[hstand:{name}] getSelf failed: {e}\n")

    # ── SAMPLING-MPC brain (HSTAND_MPC=1): replace the reactive lean/arm/trim
    # overlays with a model-predictive planner. Carries a MuJoCo "sidecar" (the
    # kp400 G1 model) in THIS controller process; each tick it copies the live
    # Newton state into the sidecar, runs MPPI rollouts, and returns leg+shoulder
    # targets. See projects/policies/control/g1_mpc_stand.py. ──
    mpc = None
    mpc_on = os.environ.get("HSTAND_MPC", "0") == "1"
    if mpc_on:
        try:
            import sys as _sys
            _root = os.environ.get("OMNISIM_HOME") or os.getcwd()
            if _root not in _sys.path:
                _sys.path.insert(0, _root)
            import numpy as _np
            import projects.policies.control.g1_brain_sim as _Hsim
            from projects.policies.control.g1_mpc_stand import (
                G1StandMPC, StandPlant, PLANTS)
            mpc_leg_names = list(_Hsim.LEGS_JOINTS)
            mpc_arm_names = list(_Hsim.ARM_JOINTS)
            mpc_leg_nom = _np.array([nominal_map.get(j, 0.0) for j in mpc_leg_names])
            mpc_arm_nom = _np.array([nominal_map.get(j, 0.0) for j in mpc_arm_names])
            _cfg = dict(K=int(os.environ.get("HSTAND_MPC_K", "96")),
                        H=int(os.environ.get("HSTAND_MPC_H", "22")),
                        nthread=int(os.environ.get("HSTAND_MPC_NTHREAD", "8")))
            _mkp = os.environ.get("HSTAND_MPC_KP")    # opt-in actuator-gain override
            _mkd = os.environ.get("HSTAND_MPC_KD")    # (needed only for the deploy MJCF)
            mpc_plant = StandPlant(
                PLANTS[os.environ.get("HSTAND_MPC_PLANT", "kp400")],
                foot_shift_x=float(os.environ.get("HSTAND_MPC_FOOTSHIFT", "0.0")),
                kp=float(_mkp) if _mkp else None,
                kd=float(_mkd) if _mkd else None)
            mpc = G1StandMPC(mpc_plant, mpc_leg_nom, cfg=_cfg, nominal_arms=mpc_arm_nom)
            mpc_rng = _np.random.default_rng(0)
            mpc_prevq = {j: nominal_map.get(j, 0.0) for j in mpc_leg_names}
            mpc_trim_on = os.environ.get("HSTAND_MPC_TRIM", "0") == "1"
            say(f"[hstand:{name}] MPC brain ON: K={_cfg['K']} H={_cfg['H']} "
                f"nthread={_cfg['nthread']} (sidecar={PLANTS[os.environ.get('HSTAND_MPC_PLANT','kp400')].name})\n")
        except Exception as e:
            say(f"[hstand:{name}] MPC brain FAILED to load ({e}); reactive fallback\n")
            mpc = None; mpc_on = False

    # Settle at nominal before any balance correction engages.
    n_settle = max(2, int(settle_s / step_dt))
    for _si in range(n_settle):
        if robot.step(step_ms) == -1:
            return 0

    # The world has now stepped >= 2 ticks, so the engine's backend verdict
    # (the .newton.json sidecar + 'world finalised' log line, both written by
    # finalizeWorld on tick 1) exists -- resolve the deferred fingerprint and
    # redraw the physics label with the real verdict (green when Newton/MuJoCo
    # cleanly drove the world, orange otherwise).
    if _envfp is not None and _envfp_fp.get("pending"):
        try:
            _envfp_fp = _envfp.report(robot, say, warm=_warm, repo_root=_REPO)
        except Exception as _e:
            say(f"[hstand:{name}] env fingerprint recheck skipped ({_e})\n")

    if self_node is not None:
        try:
            _p = self_node.getPosition() or [0, 0, 0]
            _o = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            _r, _pi, _ = _ori_matrix_to_rpy(_o)
            say(f"[hstand:{name}] settle done: bz={_p[2]:.3f} "
                f"roll={_r:+.3f} pitch={_pi:+.3f}\n")
        except Exception:
            pass

    use_balance = (kp_p != 0.0 or kd_p != 0.0 or kp_r != 0.0 or kd_r != 0.0) and (
        ankle_pitch or ankle_roll)

    # ── Cube-throw test (real external perturbation in deploy) ──
    # When HSTAND_THROW=1, this supervisor periodically launches DEF CUBE0..CUBEn
    # at the robot's torso. Each cube is teleported to a point ~throw_dist away,
    # given a velocity straight at the torso, so it hits and shoves the robot. The
    # direction rotates around the robot so it gets hit from every side. This is the
    # deploy-side analogue of the trainer's --dr-push DR.
    throw_on = os.environ.get("HSTAND_THROW", "0").strip() != "0"
    throw_cubes = []
    throw_tr = []
    if throw_on:
        i = 0
        while True:
            c = robot.getFromDef(f"CUBE{i}")
            if c is None:
                break
            throw_cubes.append(c)
            try:
                throw_tr.append(c.getField("translation"))
            except Exception:
                throw_tr.append(None)
            i += 1
        say(f"[hstand:{name}] THROW on: {len(throw_cubes)} cubes\n")
    throw_speed = _envf("HSTAND_THROW_SPEED", 4.5)
    throw_dist = _envf("HSTAND_THROW_DIST", 1.6)
    throw_period_ms = _envf("HSTAND_THROW_PERIOD", 2.0) * 1000.0
    throw_z = _envf("HSTAND_THROW_Z", 0.12)   # aim height above the base (lower torso = central, reliable)
    throw_idx = 0
    last_throw_ms = -1e9
    traced = None       # cube tracer: confirm the thrown cube actually moves + reaches the torso
    trace_left = 0
    launch_cube = None  # the cube currently being launched (velocity re-applied to flush)
    launch_vel = [0.0] * 6
    launch_left = 0
    trace_min = 1e9
    pitch_lp = 0.0      # slow low-pass of pitch = the lean controller's drifting equilibrium
    lean_vx_lp = 0.0    # light low-pass of the lean's noisy disturbance inputs
    lean_pr_lp = 0.0
    trim_pi = 0.0       # slow integral auto-trim (pitch): arrests steady fore/aft creep
    trim_ri = 0.0       # slow integral auto-trim (roll): arrests steady lateral creep
    arm_ipitch = 0.0    # arm-balance integral (pitch): returns the body to 0 rad (home)
    arm_iroll = 0.0     # arm-balance integral (roll): returns the body to 0 rad (home)
    ol_lat_i = 0.0      # one-leg lateral CoM integral (drives base y onto the support foot)
    ol_by_prev = 0.0    # previous base y (for the lateral derivative term)
    ol_lift_t0 = -1.0   # latched time the swing-leg lift actually started (-1 = not yet)
    ol_by0 = None       # latched base y at one-leg start = lateral origin for the foot target
    ol_ar_prev = None    # previous support ankle_roll angle (for the torque-inversion qd)
    ol_bz0 = None        # latched base z at one-leg start = the height-hold target

    # ── Deterministic CAPTURE-STEP recovery (the 3rd balance strategy) ──
    # The stiff stand only has the ankle strategy: a big push that carries the CoM
    # past the foot edge is unrecoverable -> it tips. This adds a protective STEP:
    # when the body tilt + horizontal velocity say the CoM is escaping, swing the
    # leg on the falling side OUT in the fall direction (capture-point idea: foot
    # goes where the CoM is heading), plant it to widen the base and catch the
    # fall, then settle back to the stand. Pure geometry + a state machine, no RL.
    cap = spec.get("capture_step") or {}
    cap_on = (os.environ.get("HSTAND_CAPTURE_STEP", "1" if cap.get("enabled") else "0").strip() != "0") and bool(cap)
    cap_legs = cap.get("legs", {})
    cap_trigger = _envf("HSTAND_STEP_TRIGGER", cap.get("trigger_tilt", 0.16))
    cap_vtrig = _envf("HSTAND_STEP_VTRIG", cap.get("trigger_vel", 0.25))
    cap_reach = _envf("HSTAND_STEP_REACH", cap.get("reach", 0.55))
    cap_lift = _envf("HSTAND_STEP_LIFT", cap.get("lift", 0.30))
    cap_extend = _envf("HSTAND_STEP_EXTEND", cap.get("extend", 0.42))  # knee straighten to plant forward-down
    cap_shift = _envf("HSTAND_STEP_SHIFT", cap.get("shift", 0.18))      # lateral weight-shift onto stance foot
    cap_shift_sign = _envf("HSTAND_STEP_SHIFT_SIGN", cap.get("shift_sign", 1.0))
    cap_dur = _envf("HSTAND_STEP_DUR", cap.get("dur", 0.34))
    cap_settle = _envf("HSTAND_STEP_SETTLE", cap.get("settle", 0.8))
    cap_state = "STAND"
    cap_t = 0.0
    cap_leg = "right"
    cap_fx = 0.0   # fore/aft step amount (rad of hip pitch)
    cap_fy = 0.0   # lateral step amount (rad of hip roll)
    # ── Capture-point IK stepping (the PROPER version: world-frame capture point ->
    # swing-foot target -> leg IK, vs the old hip-angle heuristic). capture_step.py is
    # a sibling module. ──
    cap_cs = None
    if cap_on:
        try:
            import capture_step as cap_cs
        except Exception as _ce:
            say(f"[hstand:{name}] capture_step import FAILED ({_ce}); step disabled\n")
            cap_cs = None; cap_on = False
    cap_cp_trig = _envf("HSTAND_STEP_CPTRIG", cap.get("cp_trigger", 0.12))  # CP-escape (m) to step
    cap_lift_h = _envf("HSTAND_STEP_LIFTH", cap.get("lift_h", 0.07))        # swing-foot lift (m)
    cap_boxx = _envf("HSTAND_STEP_BOXX", cap.get("box_x", 0.30))            # max fore/aft foot target (m)
    cap_boxy = _envf("HSTAND_STEP_BOXY", cap.get("box_y", 0.34))            # max lateral foot target (m)
    cap_wshift = _envf("HSTAND_STEP_WSHIFT", cap.get("wshift", 0.12))       # stance ankle weight-shift (rad)
    cap_foot_target = None; cap_foot_start = None; cap_qsw = None

    def _legq_nom(_lg):
        return [float(nominal_map.get(f"{_lg}_hip_pitch_joint", 0.0)),
                float(nominal_map.get(f"{_lg}_hip_roll_joint", 0.0)),
                float(nominal_map.get(f"{_lg}_hip_yaw_joint", 0.0)),
                float(nominal_map.get(f"{_lg}_knee_joint", 0.0)),
                float(nominal_map.get(f"{_lg}_ankle_pitch_joint", 0.0)),
                float(nominal_map.get(f"{_lg}_ankle_roll_joint", 0.0))]

    def _set_leg(_t, _lg, _q):
        for _jn, _v in zip(
            (f"{_lg}_hip_pitch_joint", f"{_lg}_hip_roll_joint", f"{_lg}_hip_yaw_joint",
             f"{_lg}_knee_joint", f"{_lg}_ankle_pitch_joint", f"{_lg}_ankle_roll_joint"), _q):
            if _jn in _t:
                _t[_jn] = float(_v)

    def _legq_cur(_lg, _t):
        return [float(_t.get(f"{_lg}_hip_pitch_joint", nominal_map.get(f"{_lg}_hip_pitch_joint", 0.0))),
                float(_t.get(f"{_lg}_hip_roll_joint", nominal_map.get(f"{_lg}_hip_roll_joint", 0.0))),
                float(_t.get(f"{_lg}_hip_yaw_joint", nominal_map.get(f"{_lg}_hip_yaw_joint", 0.0))),
                float(_t.get(f"{_lg}_knee_joint", nominal_map.get(f"{_lg}_knee_joint", 0.0))),
                float(_t.get(f"{_lg}_ankle_pitch_joint", nominal_map.get(f"{_lg}_ankle_pitch_joint", 0.0))),
                float(_t.get(f"{_lg}_ankle_roll_joint", nominal_map.get(f"{_lg}_ankle_roll_joint", 0.0)))]

    # MULTI-STEP reactive stepping: chain fast small steps (alternating legs, each
    # chasing the live capture point, feet straddling the CP by +/- half-stance) until
    # the CP is contained -> recovers impulses no single step can, and keeps stepping
    # under a sustained barrage.
    cap_dbl = _envf("HSTAND_STEP_DBL", cap.get("dbl_s", 0.05))      # double-support re-eval gap
    cap_rec = _envf("HSTAND_STEP_REC", cap.get("recover_s", 0.5))   # return-to-nominal ramp
    cap_cp_safe = _envf("HSTAND_STEP_CPSAFE", cap.get("cp_safe", 0.07))  # CP contained -> stop stepping
    cap_hstance = _envf("HSTAND_STEP_HSTANCE", cap.get("hstance", 0.10))  # half stance width (m)
    cap_footz = _envf("HSTAND_STEP_FOOTZ", cap.get("foot_z", -0.76))      # FIXED foot depth (m, pelvis frame)
    cap_maxstep = int(_envf("HSTAND_STEP_MAX", cap.get("max_steps", 12)))
    cap_cop = _envf("HSTAND_STEP_COP", cap.get("cop_k", 0.6))       # ankle CoP gain during steps (rad per m of CP)
    cap_q = {"left": _legq_nom("left"), "right": _legq_nom("right")}
    cap_active = "right"; cap_nstep = 0; cap_swing_start = _legq_nom("right")

    if cap_on:
        say(f"[hstand:{name}] CAPTURE-STEP (multi-step IK) on: cp_trig={cap_cp_trig} v>{cap_vtrig} "
            f"box=({cap_boxx},{cap_boxy}) lift={cap_lift_h} dur={cap_dur}s dbl={cap_dbl} hstance={cap_hstance}\n")

    # Optional clean TEST PUSH (isolates the step controller from cube physics):
    # apply a one-shot pelvis velocity impulse at a set time + direction.
    test_push_v = _envf("HSTAND_TEST_PUSH", 0.0)        # m/s (0 = off)
    test_push_t = _envf("HSTAND_TEST_PUSH_T", 1.5)      # when (s)
    test_push_ang = _envf("HSTAND_TEST_PUSH_ANG", 0.0)  # direction (rad, 0=+x forward)
    test_push_done = False

    if wave_on:
        say(f"[hstand:{name}] WAVE on: {len(wave_pose)} arm joints, "
            f"osc {wave_osc.get('joint')} amp={wave_osc.get('amp')} "
            f"freq={wave_osc.get('freq_hz')}Hz, ramp {wave_ramp_s}s\n")

    sim_ms = settle_s * 1000.0
    last_log_ms = 0.0
    survived = 0
    fell_at = None
    wave_t = 0.0

    # ── SQUAT overlay config (deterministic quasi-static knee-bend and return) ──
    # On top of the held stand, blend the legs DEEPER into a squat by the per-joint
    # `delta` offsets (foot kept flat: ankle delta = -(hip+knee)), HOLD at the bottom,
    # then RISE, repeated `reps` times. Pure feedforward leg targets; the reactive
    # lean + arm_balance + hip auto_trim keep the CoM centred through the transient.
    squat = spec.get("squat") or {}
    squat_on = (os.environ.get("HSTAND_SQUAT", "1" if squat.get("enabled") else "0").strip() != "0") and bool(squat)
    squat_delta = {k: float(v) for k, v in squat.get("delta", {}).items()}
    squat_start_s = _envf("HSTAND_SQUAT_START", squat.get("start_s", 2.0))
    squat_descend_s = _envf("HSTAND_SQUAT_DESCEND", squat.get("descend_s", 2.0))
    squat_hold_s = _envf("HSTAND_SQUAT_HOLD", squat.get("hold_s", 1.2))
    squat_rise_s = _envf("HSTAND_SQUAT_RISE", squat.get("rise_s", 2.0))
    squat_between_s = _envf("HSTAND_SQUAT_BETWEEN", squat.get("between_s", 1.0))
    squat_reps = int(_envf("HSTAND_SQUAT_REPS", squat.get("reps", 1)))
    squat_ff_ank = _envf("HSTAND_SQUAT_FF_ANK", squat.get("ff_ank", 0.0))
    squat_ff_hip = _envf("HSTAND_SQUAT_FF_HIP", squat.get("ff_hip", 0.0))
    squat_bz_drop = _envf("HSTAND_SQUAT_BZ_DROP", squat.get("bz_drop", 0.25))
    squat_phase = "WAIT"     # WAIT -> DESCEND -> HOLD -> RISE -> BETWEEN -> ... -> DONE
    squat_t = 0.0
    squat_rep = 0
    squat_frac = 0.0         # 0 = standing, 1 = bottom of squat (smoothstepped)
    squat_min_bz = 1e9       # deepest pelvis height seen at the bottom (telemetry)
    squat_done_logged = False
    if squat_on:
        say(f"[hstand:{name}] SQUAT on: reps={squat_reps} start={squat_start_s:.1f}s "
            f"descend={squat_descend_s:.1f}s hold={squat_hold_s:.1f}s rise={squat_rise_s:.1f}s "
            f"d_knee={squat_delta.get('left_knee_joint', 0.0):+.2f} ff_ank={squat_ff_ank:+.2f} "
            f"bz_drop={squat_bz_drop:.2f}\n")
    peak_spd = 0.0    # peak base linear speed since last log (impact signature)
    peak_tilt = 0.0   # peak |roll|/|pitch| since last log
    jit_accum = 0.0   # sum of |Δpitch| per second = the "vibration" metric
    prev_pitch_j = 0.0

    # ── Build the MANIPULATION runtime (IK chain, cube nodes, state machine) ──
    manip_ok = False
    manip_state = "INIT"
    manip_phases = []
    manip_phase_i = 0
    manip_enter = False
    manip_seg_t0 = 0.0
    manip_seg_dur = 1.0
    manip_cube_idx = 0
    manip_held = None
    manip_held_tf = None
    manip_pin_node = None       # a just-released cube pinned at its place while the hand leaves
    manip_pin_tf = None
    manip_pin_pos = None
    manip_pin_ticks = 0
    manip_placed = 0
    manip_q_cmd = []
    manip_cur_tcp = (0.0, 0.0, 0.0)   # current commanded WORLD TCP (interpolated)
    manip_tcp_from = (0.0, 0.0, 0.0)
    manip_tcp_to = (0.0, 0.0, 0.0)
    manip_started = False             # has the first reach trajectory begun
    manip_ik_count = 0                # decimation counter (per-tick IK is the cpu cost)
    manip_hand_node = None   # the real reach-arm wrist/hand link node (ground truth)
    # pelvis(root) -> torso_link origin offset (waist_yaw_joint, held at 0); the IK
    # chain is expressed in the torso_link frame, so world<->torso conversions use this.
    M_PELVIS_TO_TORSO = (-0.0039635, 0.0, 0.054)

    def _find_link_node(nm):
        """Walk the robot subtree for the Solid/link node whose name == nm (the URDF
        importer names link Solids after the URDF link). Used to ride the REAL hand
        pose for grasp/carry instead of trusting FK."""
        if self_node is None or not nm:
            return None
        stack = [self_node]
        while stack:
            n = stack.pop()
            try:
                f = n.getField("name")
                if f is not None and f.getSFString() == nm:
                    return n
            except Exception:
                pass
            for fn in ("children", "endPoint"):
                f = n.getField(fn)
                if f is None:
                    continue
                try:
                    c = f.getCount()
                    if c and c > 0:
                        for i in range(c):
                            ch = f.getMFNode(i)
                            if ch is not None:
                                stack.append(ch)
                        continue
                except Exception:
                    pass
                try:
                    ch = f.getSFNode()
                    if ch is not None:
                        stack.append(ch)
                except Exception:
                    pass
        return None

    def _hand_world_fk():
        """Reach-hand TCP world position from the LIVE joint sensors via the IK chain
        FK (used as a fallback + as a self-check against the real link node)."""
        qa = []
        for jn in mreach_joints:
            s = sensors.get(jn)
            qa.append(float(s.getValue()) if s is not None else 0.0)
        tt = arm_ik.forward_kinematics(mchain, qa, mtcp)   # torso-frame TCP
        bp = self_node.getPosition() or [0.0, 0.0, 0.0]
        bo = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
        fx = M_PELVIS_TO_TORSO[0] + tt[0]
        fy = M_PELVIS_TO_TORSO[1] + tt[1]
        fz = M_PELVIS_TO_TORSO[2] + tt[2]
        return (bp[0] + bo[0] * fx + bo[1] * fy + bo[2] * fz,
                bp[1] + bo[3] * fx + bo[4] * fy + bo[5] * fz,
                bp[2] + bo[6] * fx + bo[7] * fy + bo[8] * fz)

    def _hand_world_now():
        """Reach-hand TCP world position. Prefers the REAL hand link node pose (ground
        truth, robust to any FK/chain error); falls back to FK if the node is absent."""
        if manip_hand_node is not None:
            try:
                p = manip_hand_node.getPosition()
                o = manip_hand_node.getOrientation()  # row-major flat 9
                tx, ty, tz = mtcp
                return (p[0] + o[0] * tx + o[1] * ty + o[2] * tz,
                        p[1] + o[3] * tx + o[4] * ty + o[5] * tz,
                        p[2] + o[6] * tx + o[7] * ty + o[8] * tz)
            except Exception:
                pass
        return _hand_world_fk()

    def _ik_world(world_tcp):
        """Solve the reach arm to a WORLD TCP target (converts world->torso via the
        live base pose). Returns (q_solution, err_norm)."""
        bp = self_node.getPosition() or [0.0, 0.0, 0.0]
        bo = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
        tw0 = bp[0] + bo[0] * M_PELVIS_TO_TORSO[0] + bo[1] * M_PELVIS_TO_TORSO[1] + bo[2] * M_PELVIS_TO_TORSO[2]
        tw1 = bp[1] + bo[3] * M_PELVIS_TO_TORSO[0] + bo[4] * M_PELVIS_TO_TORSO[1] + bo[5] * M_PELVIS_TO_TORSO[2]
        tw2 = bp[2] + bo[6] * M_PELVIS_TO_TORSO[0] + bo[7] * M_PELVIS_TO_TORSO[1] + bo[8] * M_PELVIS_TO_TORSO[2]
        rx, ry, rz = world_tcp[0] - tw0, world_tcp[1] - tw1, world_tcp[2] - tw2
        # torso-frame target = R^T * (world - torso_origin)
        tgt = (bo[0] * rx + bo[3] * ry + bo[6] * rz,
               bo[1] * rx + bo[4] * ry + bo[7] * rz,
               bo[2] * rx + bo[5] * ry + bo[8] * rz)
        qsol, err, _ = arm_ik.dls_ik(mchain, manip_q_cmd, tgt, mikcfg, mlimits,
                                     q_rest=(mqrest or None), posture_gain=m_posture_gain)
        return qsol, err

    if manip_on:
        try:
            _ctrl_dir = str(Path(__file__).resolve().parent)
            if _ctrl_dir not in sys.path:
                sys.path.insert(0, _ctrl_dir)
            import arm_ik
            mreach_joints = list(manip["reach_joints_%s" % manip_reach_side])
            mchain = arm_ik.parse_chain(manip["chain_%s" % manip_reach_side])
            mtcp = tuple(manip["tcp_%s" % manip_reach_side])
            mlimits = [tuple(p) for p in manip["limits_%s" % manip_reach_side]]
            mikcfg = dict(manip.get("ik", {}))
            mikcfg["tcp_offset"] = mtcp
            # Null-space posture bias: keep the redundant arm DOF natural + body-clear.
            m_posture_gain = _envf("HSTAND_MANIP_POSTURE", manip.get("posture_gain", 0.0))
            mqrest = list(manip.get("q_rest_%s" % manip_reach_side, []))
            _mt = manip.get("timing", {})
            m_move_s = float(_mt.get("move_s", 1.3))
            m_dwell_s = float(_mt.get("dwell_s", 0.4))
            m_grasp_dwell_s = float(_mt.get("grasp_dwell_s", 0.6))
            _mo = manip.get("offsets", {})
            m_z_above = float(_mo.get("z_above", 0.08))
            m_grasp_dz = float(_mo.get("grasp_dz", 0.02))
            m_lift_dz = float(_mo.get("lift_dz", 0.16))
            m_place_dz = float(_mo.get("place_dz", 0.01))
            # Pin a released cube until the reach hand retracts at least this far from it
            # (so the hand-cube contact can't fling it off the table), capped at m_pin_max.
            m_pin_release_dist = float(manip.get("pin_release_dist", 0.22))
            m_pin_max = int(_envf("HSTAND_MANIP_PIN_TICKS", manip.get("pin_ticks", 400)))
            m_home_tcp = tuple(manip.get("home_tcp", [0.14, -0.20, 0.55]))
            m_grasp_radius = _envf("HSTAND_MANIP_GRASP_RADIUS", manip.get("grasp_radius", 0.12))
            m_start_s = _envf("HSTAND_MANIP_START_S", manip.get("start_s", 3.0))
            m_c_reach = _envf("HSTAND_MANIP_C_REACH", manip.get("c_reach", 0.30))
            m_ff_ref = float(manip.get("ff_ref", 0.0))
            m_ff_clamp = _envf("HSTAND_MANIP_FF_CLAMP", manip.get("ff_clamp", 0.12))
            m_c_ank = _envf("HSTAND_MANIP_C_ANK", manip.get("c_ank", 0.35))
            m_ank_clamp = _envf("HSTAND_MANIP_ANK_CLAMP", manip.get("ank_clamp", 0.14))
            m_c_cw = _envf("HSTAND_MANIP_C_CW", manip.get("c_cw", 1.0))
            m_cw_clamp = _envf("HSTAND_MANIP_CW_CLAMP", manip.get("cw_clamp", 0.6))
            m_cube_nodes = []
            for cd in manip.get("cubes", []):
                cn = robot.getFromDef(cd["def"])
                ctf = cn.getField("translation") if cn is not None else None
                m_cube_nodes.append({"def": cd["def"], "node": cn, "tf": ctf,
                                     "place": tuple(cd["place"])})
            manip_q_cmd = [float(nominal_map.get(jn, 0.0)) for jn in mreach_joints]
            manip_cur_tcp = tuple(m_home_tcp)
            _missing_reach = [jn for jn in mreach_joints if motors.get(jn) is None]
            _have_cubes = sum(1 for c in m_cube_nodes if c["node"] is not None)
            manip_ok = (self_node is not None and len(mreach_joints) == 5
                        and not _missing_reach and _have_cubes > 0)
            manip_hand_node = _find_link_node(manip.get("hand_link_%s" % manip_reach_side))
            if manip_ok:
                _bal_side = "left" if manip_reach_side == "right" else "right"
                _hw = _hand_world_now()       # real-node (ground truth) if found
                _hwfk = _hand_world_fk()      # chain-FK estimate
                _derr = math.sqrt(sum((_hw[k] - _hwfk[k]) ** 2 for k in range(3)))
                say(f"[hstand:{name}] MANIP on: reach={manip_reach_side} balance={_bal_side} "
                    f"cubes={_have_cubes} grasp_r={m_grasp_radius:.2f} start={m_start_s:.1f}s "
                    f"hand_node={'found' if manip_hand_node is not None else 'MISSING'}\n")
                say(f"[hstand:{name}] MANIP diag: hand_real=({_hw[0]:.3f},{_hw[1]:.3f},{_hw[2]:.3f}) "
                    f"hand_fk=({_hwfk[0]:.3f},{_hwfk[1]:.3f},{_hwfk[2]:.3f}) FK_err={_derr*1000:.0f}mm\n")
                for _c in m_cube_nodes:
                    if _c["node"] is not None:
                        _cp = _c["node"].getPosition() or [0, 0, 0]
                        say(f"[hstand:{name}] MANIP diag: {_c['def']} at=({_cp[0]:.3f},{_cp[1]:.3f},{_cp[2]:.3f})\n")
            else:
                say(f"[hstand:{name}] MANIP DISABLED: self={self_node is not None} "
                    f"missing_reach={_missing_reach} cubes={_have_cubes}\n")
                manip_on = False
        except Exception as _me:
            import traceback
            say(f"[hstand:{name}] MANIP setup failed: {_me}\n{traceback.format_exc()}\n")
            manip_on = False
            manip_ok = False

    # ── Build the ARM-MOTION runtime (FK chains, rest-hand reference, schedule) ──
    # FK both arm chains so the hand world/torso positions are known each tick (the
    # "awareness"); the rest-pose forward reach is the reference the CoM-back feedforward
    # measures the arms' forward extension against. Reuses arm_ik.forward_kinematics.
    armm_ok = False
    armm_fk = None
    armm_ex_i = 0
    armm_ex_t0 = None
    armm_resting = False
    armm_rest_fwd = 0.0
    armm_hand_l = (0.0, 0.0, 0.0)
    armm_hand_r = (0.0, 0.0, 0.0)
    if armm_on:
        try:
            _ctrl_dir = str(Path(__file__).resolve().parent)
            if _ctrl_dir not in sys.path:
                sys.path.insert(0, _ctrl_dir)
            import arm_ik as _armik
            _fk = armm.get("fk", {})
            armm_fk = {
                "chain_l": _armik.parse_chain(_fk["chain_left"]),
                "chain_r": _armik.parse_chain(_fk["chain_right"]),
                "tcp_l": tuple(_fk["tcp_left"]),
                "tcp_r": tuple(_fk["tcp_right"]),
                "jl": list(_fk["joints_left"]),
                "jr": list(_fk["joints_right"]),
                "fk": _armik.forward_kinematics,
            }
            # rest-hand forward reference (torso-frame x) from the rest arm pose
            _ql0 = [float(nominal_map.get(j, 0.0)) for j in armm_fk["jl"]]
            _qr0 = [float(nominal_map.get(j, 0.0)) for j in armm_fk["jr"]]
            _hl0 = armm_fk["fk"](armm_fk["chain_l"], _ql0, armm_fk["tcp_l"])
            _hr0 = armm_fk["fk"](armm_fk["chain_r"], _qr0, armm_fk["tcp_r"])
            armm_rest_fwd = 0.5 * (_hl0[0] + _hr0[0])
            armm_hand_l, armm_hand_r = _hl0, _hr0
            armm_ok = len(armm_exercises) > 0
            if armm_ok:
                say(f"[hstand:{name}] ARM_MOTION on: {len(armm_exercises)} exercises "
                    f"({', '.join(e.get('name', '?') for e in armm_exercises)}), "
                    f"start={armm_start_s:.1f}s loop={armm_loop} rest_fwd={armm_rest_fwd:.3f}\n")
            else:
                armm_on = False
                say(f"[hstand:{name}] ARM_MOTION DISABLED: no exercises in spec\n")
        except Exception as _ae:
            import traceback
            say(f"[hstand:{name}] ARM_MOTION setup failed: {_ae}\n{traceback.format_exc()}\n")
            armm_on = False
            armm_ok = False

    # ── Optional WEB-ANIMATION capture (HSTAND_ANIM=<path.html>) ──
    # Record the scene transforms each step to an X3D/HTML animation that replays
    # in a browser — works in the reliable headless path (no GUI window, no ffmpeg;
    # it serialises poses, not pixels). Stopped cleanly when the motion (e.g. the
    # squat reps) finishes, so the file is finalised before the process is killed.
    anim_path = os.environ.get("HSTAND_ANIM", "").strip()
    anim_active = False
    if anim_path and self_node is not None:
        try:
            _ap = anim_path if os.path.isabs(anim_path) else str(_REPO / anim_path)
            os.makedirs(os.path.dirname(_ap), exist_ok=True)
            robot.animationStartRecording(_ap)
            anim_active = True
            say(f"[hstand:{name}] ANIM recording -> {_ap}\n")
        except Exception as _ae:
            say(f"[hstand:{name}] ANIM start failed: {_ae}\n")

    while robot.step(step_ms) != -1:
        sim_ms += step_ms

        targets = dict(nominal_map)

        # WAVE overlay: ramp the chosen arm from nominal to the raised pose, then
        # oscillate the one waving DOF. Pure feedforward — the legs/torso keep
        # holding NOMINAL and the stiff stance absorbs the perturbation.
        if wave_on:
            wave_t += step_dt
            r = min(1.0, wave_t / max(wave_ramp_s, 1e-3))
            for jn, pv in wave_pose.items():
                if jn in targets:
                    targets[jn] = nominal_map[jn] + r * (pv - nominal_map[jn])
            oj = wave_osc.get("joint")
            if oj and oj in targets:
                amp = float(wave_osc.get("amp", 0.0))
                f = float(wave_osc.get("freq_hz", 0.7))
                targets[oj] += r * amp * math.sin(2.0 * math.pi * f * wave_t)

        # SQUAT overlay: advance the descend/hold/rise state machine and blend the
        # legs toward the squat by `squat_frac` (smoothstepped). Feedforward leg
        # targets only -- the reactive balance below stabilises the transient.
        if squat_on:
            t_sq = sim_ms / 1000.0 - settle_s
            if squat_phase == "WAIT":
                if t_sq >= squat_start_s:
                    squat_phase = "DESCEND"; squat_t = 0.0
            elif squat_phase == "DESCEND":
                squat_t += step_dt
                x = min(1.0, squat_t / max(squat_descend_s, 1e-3))
                squat_frac = x * x * (3.0 - 2.0 * x)              # smoothstep up
                if squat_t >= squat_descend_s:
                    squat_frac = 1.0; squat_phase = "HOLD"; squat_t = 0.0
            elif squat_phase == "HOLD":
                squat_t += step_dt; squat_frac = 1.0
                if squat_t >= squat_hold_s:
                    squat_phase = "RISE"; squat_t = 0.0
            elif squat_phase == "RISE":
                squat_t += step_dt
                x = min(1.0, squat_t / max(squat_rise_s, 1e-3))
                squat_frac = 1.0 - x * x * (3.0 - 2.0 * x)        # smoothstep down
                if squat_t >= squat_rise_s:
                    squat_frac = 0.0; squat_rep += 1
                    if squat_rep >= squat_reps:
                        squat_phase = "DONE"
                    else:
                        squat_phase = "BETWEEN"; squat_t = 0.0
            elif squat_phase == "BETWEEN":
                squat_t += step_dt; squat_frac = 0.0
                if squat_t >= squat_between_s:
                    squat_phase = "DESCEND"; squat_t = 0.0
            # DONE -> squat_frac stays 0.0 (hold the stand)

            if squat_frac > 0.0:
                for jn, dv in squat_delta.items():
                    if jn in targets:
                        targets[jn] += squat_frac * dv
                # Optional open-loop depth-proportional CoM-back (like the manip overlay):
                # lean the ankle (and hip) back as the body sinks, no destabilising feedback.
                if squat_ff_ank != 0.0:
                    for jn in ankle_pitch:
                        if jn in targets:
                            targets[jn] -= squat_frac * squat_ff_ank   # neg ankle = lean back
                if squat_ff_hip != 0.0:
                    for jn in hip_pitch:
                        if jn in targets:
                            targets[jn] += squat_frac * squat_ff_hip

        bx = by = bz = roll = pitch = yaw = 0.0
        ori = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0.0, 0.0, 0.0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
                roll, pitch, yaw = _ori_matrix_to_rpy(ori)
                vel6 = self_node.getVelocity() or [0.0] * 6
                # world->body angular velocity, like the G1 controller.
                w0, w1, w2 = float(vel6[3]), float(vel6[4]), float(vel6[5])
                roll_rate = ori[0] * w0 + ori[3] * w1 + ori[6] * w2
                pitch_rate = ori[1] * w0 + ori[4] * w1 + ori[7] * w2
                vx, vy = float(vel6[0]), float(vel6[1])
                base_spd = math.hypot(vx, vy)
            except Exception:
                roll_rate = pitch_rate = 0.0
                base_spd = 0.0
                vx = vy = 0.0
        else:
            roll_rate = pitch_rate = 0.0
            base_spd = 0.0
            vx = vy = 0.0

        # ── SAMPLING-MPC brain: plan leg+shoulder targets from the live Newton
        # state (self-contained read; the reactive overlays below stay disabled). ──
        if mpc_on and mpc is not None and self_node is not None:
            try:
                _pos = self_node.getPosition() or [0.0, 0.0, 0.0]
                _ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                _v = self_node.getVelocity() or [0.0] * 6
                _roll, _pitch, _yaw = _ori_matrix_to_rpy(_ori)
                _w0, _w1, _w2 = float(_v[3]), float(_v[4]), float(_v[5])
                _rr = _ori[0] * _w0 + _ori[3] * _w1 + _ori[6] * _w2
                _pr = _ori[1] * _w0 + _ori[4] * _w1 + _ori[7] * _w2
                _yr = _ori[2] * _w0 + _ori[5] * _w1 + _ori[8] * _w2
                _jq = []; _jqd = []
                for _j in mpc_leg_names:
                    _s = sensors.get(_j)
                    _q = float(_s.getValue()) if _s is not None else nominal_map.get(_j, 0.0)
                    _jqd.append((_q - mpc_prevq.get(_j, _q)) / max(step_dt, 1e-4))
                    mpc_prevq[_j] = _q; _jq.append(_q)
                if mpc_trim_on:
                    mpc.update_trim(_roll, _pitch, step_dt)
                mpc.sync_state(float(_pos[2]), _roll, _pitch, _yaw,
                               float(_v[0]), float(_v[1]), float(_v[2]),
                               _rr, _pr, _yr, _jq, _jqd)
                _legs_t, _arms_t = mpc.plan_targets(mpc_rng)
                for _j, _val in zip(mpc_leg_names, _legs_t):
                    if _j in targets:
                        targets[_j] = float(_val)
                for _j, _val in zip(mpc_arm_names, _arms_t):
                    if _j in targets:
                        targets[_j] = float(_val)
            except Exception as _e:
                say(f"[hstand:{name}] MPC step error: {_e}\n")

        # ── One-leg balance: capture-point (LIPM) lateral control ──
        # Three lateral effectors, all driven off the support-foot frame:
        #   HIP (slow, large range): integral on CoM position error -> bodily shift the
        #     CoM over the support foot (the only way to move the CoM ~10 cm).
        #   ANKLE (fast, velocity-aware): the CAPTURE POINT cp = (by-foot) + vy/omega
        #     -> support ankle_roll. The vy/omega term reacts to a developing tip the
        #     instant it starts moving -- the piece the position-only PI lacked.
        #   ARMS: out as outriggers + roll-swing on the capture point (angular momentum).
        # Fore/aft stays on the arm-pitch + hip-pitch trim from above.
        if oneleg_on:
            t_ol = sim_ms / 1000.0 - settle_s
            if ol_by0 is None and t_ol > 0.0:
                ol_by0 = by
            arm_r = max(0.0, min(1.0, t_ol / max(ol_shift_s, 1e-3)))
            foot_y = (ol_by0 if ol_by0 is not None else 0.0) + (ol_y_target if _sup == "left" else -ol_y_target)
            ey = by - foot_y                         # CoM lateral position error
            omega = ol_omega if ol_omega > 1e-3 else math.sqrt(9.81 / max(bz, 0.3))
            cp_ey = ey + vy / omega                  # lateral capture-point error
            # Latch the lift once the weight is over the foot (CoM near + slow).
            _lifting_cfg = abs(ol_lift_hip) > 1e-6 or abs(ol_lift_knee) > 1e-6
            if ol_lift_t0 < 0.0 and _lifting_cfg and ((abs(ey) < 0.045 and abs(vy) < 0.18) or t_ol > ol_shift_s):
                ol_lift_t0 = t_ol
            lift_r = 0.0 if ol_lift_t0 < 0.0 else max(0.0, min(1.0, (t_ol - ol_lift_t0) / max(ol_lift_s, 1e-3)))
            # Arms OUT to the sides as outriggers (left +roll, right -roll).
            for jn in shoulder_roll:
                if jn in targets:
                    targets[jn] += (ol_arm_out if "left" in jn else -ol_arm_out) * arm_r
            # Slow CoM placement (proven ankle-dominant translate: ankle full, hip the
            # opposite share -> pelvis stays ~level). +lat_cmd moves CoM -y (empirical),
            # so the loop output is negated to drive CoM toward a +foot_y target.
            # Anti-windup: stop integrating once the CoM is within 3 cm of the foot,
            # else a small residual error keeps winding the lean -> the body leans
            # progressively more and the pelvis sinks (the slow-sink failure).
            if t_ol > 0.2 and lift_r <= 0.0 and abs(ey) > 0.03:
                ol_lat_i = float(np.clip(ol_lat_i + ol_lat_ki * ey * step_dt,
                                         -ol_lat_clamp, ol_lat_clamp))
            lat_cmd = -float(np.clip(ol_lat_kp * ey + ol_lat_i, -ol_lat_clamp, ol_lat_clamp))
            lat_cmd *= (1.0 - 0.45 * lift_r)
            sup_ar = _legj(_sup, "ankle_roll")
            sup_hr = _legj(_sup, "hip_roll")
            # HIP does the large CoM placement (slow translate over the support foot).
            if sup_hr in targets:
                targets[sup_hr] += lat_cmd
            if ol_ankle_torque:
                # ANKLE CoP torque (Option B): bounded capture-point torque via PD
                # inversion. tau clamped to the physical CoP limit; +ankle_roll moves
                # CoM -y so the restoring torque for a +cp_ey (tipping +y) is +tau.
                q_ar = sensors[sup_ar].getValue() if sensors.get(sup_ar) is not None else 0.0
                qd_ar = (q_ar - ol_ar_prev) / step_dt if (ol_ar_prev is not None and step_dt > 0) else 0.0
                ol_ar_prev = q_ar
                tau_cop = float(np.clip(ol_cop_k * cp_ey, -ol_tau_max, ol_tau_max))
                if sup_ar in targets:
                    targets[sup_ar] = q_ar + (tau_cop + ol_pd_kd * qd_ar) / max(ol_pd_ke, 1e-3)
            else:
                # legacy angle command (kept for comparison)
                ank_cp = -float(np.clip(ol_cp_ank * cp_ey, -ol_cp_clamp, ol_cp_clamp))
                if sup_ar in targets:
                    targets[sup_ar] += -lat_cmd * ol_hip_share + ank_cp
            # ARMS roll-swing on the capture point (angular momentum). Swing both arms
            # the SAME world-lateral way to throw momentum against the fall.
            arm_cp = float(np.clip(ol_cp_arm * cp_ey, -1.0, 1.0))
            for jn in shoulder_roll:
                if jn in targets:
                    targets[jn] += arm_cp
            # Support-leg HEIGHT hold: extend the support knee (+ hip/ankle pitch
            # compensation to keep the torso vertical) to counter the sag from bearing
            # the whole body on one leg. Engages with the lift.
            if ol_bz0 is None and t_ol > 0.0:
                ol_bz0 = bz
            # Engage height-hold with the lift (one-foot load). During the two-foot
            # shift it would tilt the pelvis and fight the weight-shift.
            if ol_bz0 is not None and lift_r > 0.0:
                bz_err = ol_bz0 - bz                      # >0 when sinking
                ext = float(np.clip(ol_height_kp * bz_err, -0.1, ol_ext_max)) * lift_r
                kn = _legj(_sup, "knee"); hp = _legj(_sup, "hip_pitch"); apj = _legj(_sup, "ankle_pitch")
                if kn in targets:
                    targets[kn] -= ext                   # straighten knee -> lift pelvis
                if hp in targets:
                    targets[hp] += ext * ol_height_comp  # keep torso vertical
                if apj in targets:
                    targets[apj] += ext * ol_height_comp
            # Lift the swing leg once the weight is over the support foot (latched).
            if ol_lift_t0 >= 0.0:
                sh = _legj(_sw, "hip_pitch"); sk = _legj(_sw, "knee"); sap = _legj(_sw, "ankle_pitch")
                if sh in targets:
                    targets[sh] += ol_lift_hip * lift_r
                if sk in targets:
                    targets[sk] += ol_lift_knee * lift_r
                if sap in targets:
                    targets[sap] += -0.20 * lift_r
                # Swing-leg roll MOMENTUM: ab/adduct the lifted leg on the capture point
                # to throw counter-roll momentum (heavier lever than the arms).
                sw_hr = _legj(_sw, "hip_roll")
                if ol_swing_k != 0.0 and sw_hr in targets:
                    targets[sw_hr] += float(np.clip(ol_swing_k * cp_ey,
                                                    -ol_swing_clamp, ol_swing_clamp)) * lift_r

        # Clean TEST PUSH: one-shot pelvis velocity impulse to trigger a fall in
        # isolation (no cubes) so the capture-step can be developed/tuned fast.
        if (test_push_v > 0.0 and not test_push_done and self_node is not None
                and sim_ms / 1000.0 >= test_push_t):
            try:
                pvx = test_push_v * math.cos(test_push_ang)
                pvy = test_push_v * math.sin(test_push_ang)
                self_node.setVelocity([pvx, pvy, 0.0, 0.0, 0.0, 0.0])
                say(f"[hstand:{name}] TEST PUSH {test_push_v:.2f} m/s "
                    f"dir=({math.cos(test_push_ang):+.2f},{math.sin(test_push_ang):+.2f}) "
                    f"at t={sim_ms/1000:.2f}s\n")
            except Exception as e:
                say(f"[hstand:{name}] test push failed: {e}\n")
            test_push_done = True

        if use_balance:
            ap = float(np.clip(kp_p * pitch + kd_p * pitch_rate, -bal_clamp, bal_clamp))
            ar = float(np.clip(kp_r * roll + kd_r * roll_rate, -bal_clamp, bal_clamp))
            for jn in ankle_pitch:
                targets[jn] = nominal_map[jn] + ap
            for jn in ankle_roll:
                targets[jn] = nominal_map[jn] + ar

        # Reactive fore/aft lean (ankle strategy): drive the ankles BACK (and hips
        # slightly) when the body is pitching/moving FORWARD -- catches a cube from
        # behind before it face-plants. Driven by the DISTURBANCE signals (forward
        # velocity vx + pitch-rate), which are ~0 in the steady stand and spike the
        # instant a cube lands. A small position term uses pitch relative to a slow
        # low-pass (its drifting equilibrium), NOT the absolute pitch -- the steady
        # +0.13 lean would otherwise be mis-read as a disturbance and over-corrected.
        if lean_on and self_node is not None:
            pitch_lp += 0.02 * (pitch - pitch_lp)
            # Low-pass the noisy disturbance inputs (kills the steady-state twitch
            # but keeps a real cube spike, which is sustained over many ticks).
            lean_vx_lp += lean_smooth * (vx - lean_vx_lp)
            lean_pr_lp += lean_smooth * (pitch_rate - lean_pr_lp)
            fwd = (lean_kv * lean_vx_lp
                   + lean_kd * (-lean_pr_lp)
                   + lean_kp * (-(pitch - pitch_lp)))
            # Soft deadband: sit still under noise, react only to real disturbances.
            if abs(fwd) <= lean_db:
                fwd = 0.0
            else:
                fwd = math.copysign(abs(fwd) - lean_db, fwd)
            fwd = float(np.clip(fwd, -lean_clamp, lean_clamp))
            for jn in ankle_pitch:
                if jn in targets:
                    targets[jn] -= fwd                  # negative ankle = lean back
            for jn in hip_pitch:
                if jn in targets:
                    targets[jn] += fwd * lean_hip       # thigh back = CoM back

        # Reactive ARM balance: swing the arms to fight a fore/aft (and lateral) tip
        # instead of holding them rigid. Forward lean (pitch>0) -> arms swing BACK
        # (shoulder_pitch up, +1.40 toward straight-down/behind) which pulls the CoM
        # behind the ankle; rate term adds a counter-rotation kick on the cube impulse.
        # Absolute pitch/roll (not the lean's relative signal), so it also arrests a
        # SLOW forward drift the ankle lean ignores -- the exact arms-down failure.
        if armb_on and self_node is not None:
            # Integrate absolute pitch/roll (after settle) so the arms hold whatever
            # steady swing is needed to bring the body all the way back to 0 rad
            # (HOME/upright), not just to a proportional offset. Clamped vs windup.
            if sim_ms / 1000.0 > settle_s + 0.5:
                arm_ipitch = float(np.clip(arm_ipitch + armb_ki * pitch * step_dt,
                                           -armb_iclamp, armb_iclamp))
                arm_iroll = float(np.clip(arm_iroll + armb_kir * roll * step_dt,
                                          -armb_iclamp, armb_iclamp))
            arm_p = float(np.clip(armb_kp * pitch + arm_ipitch + armb_kd * pitch_rate,
                                  -armb_clamp, armb_clamp))
            # Exclude the reach arm from arm_balance ONLY while manipulation is active;
            # before it starts and after it finishes (DONE) both arms balance again so
            # the stand stays durable long after the cubes are placed.
            # arm_motion KEEPS arm_balance live on the arms (superimposed): G1's stand is
            # marginal and NEEDS the arms as its fast balancer, so the exercise rides on
            # top of the balancer rather than replacing it. Only manip (one arm at a time)
            # hands an arm OFF; arm_motion's amplitudes stay moderate so the balancer keeps
            # its authority through the routine.
            _meng = manip_on and manip_ok and manip_started and manip_state != "DONE"
            _abp = armb_pitch_joints if _meng else shoulder_pitch
            _abr = armb_roll_joints if _meng else shoulder_roll
            for jn in _abp:
                if jn in targets:
                    targets[jn] += arm_p                 # +shoulder_pitch = arms back -> CoM back
            arm_r = float(np.clip(armb_kr * roll + arm_iroll, -armb_clamp, armb_clamp))
            for jn in _abr:
                if jn in targets:
                    targets[jn] += arm_r                 # swing arms laterally to counter roll

        # Integral auto-trim: integrate absolute pitch/roll and feed the ankles/hips
        # so a slow CoM creep (the cumulative cube shove) is held off, not just the
        # fast hits. Only runs after the settle so the initial transient doesn't wind
        # it up. Clamped to bound authority.
        # HIP-ONLY return-to-home trim. The hip is a strong CoM effector (heavy torso)
        # and -- unlike the ANKLE, whose feedback destabilises this stiff position-held
        # stand -- it is safe to drive. Integral of absolute pitch leans the torso back
        # to bring the body all the way to 0 rad, assisting the arms. P + I.
        if trim_on and self_node is not None and sim_ms / 1000.0 > settle_s + 0.5:
            trim_pi = float(np.clip(trim_pi + trim_ki * pitch * step_dt, -trim_iclamp, trim_iclamp))
            corr_p = float(np.clip(trim_kp * pitch + trim_pi + trim_kd * pitch_rate,
                                   -trim_clamp, trim_clamp))
            for jn in hip_pitch:
                if jn in targets:
                    targets[jn] += corr_p                # forward lean -> thigh/torso back -> CoM back

        # ── ARM-MOTION overlay: ride a looping exercise routine on the arms ──
        # Smooth per-joint sinusoids (dev = ramp * (bias + amp*sin(2*pi*f*t + phase)))
        # eased in/out to the rest pose at each exercise boundary, SUPERIMPOSED on the
        # arm_balance correction already written above. FK gives the live hand positions
        # (awareness); a forward CoM-back feedforward (hip + ankle, proportional to how
        # far the hands extend ahead of their rest reach) pre-compensates the arms' CoM
        # shift so the stand stays centred while both arms move.
        if armm_on and armm_ok and self_node is not None:
            t_am = sim_ms / 1000.0
            if t_am >= armm_start_s:
                if armm_ex_t0 is None:
                    armm_ex_t0 = t_am
                te = t_am - armm_ex_t0
                arm_dev = {}
                if armm_resting:
                    # REST DWELL between exercises: hold the rest pose (arm_dev empty) so the
                    # stand re-centres. An exercise can leave a small forward CoM lean (e.g.
                    # the wave holds one arm up for several seconds); without this dwell the
                    # NEXT exercise's transition tips the already-leaned body over. The dwell
                    # lets lean + hip trim bring pitch back to ~0 before the next perturbation.
                    if te >= armm_rest_s:
                        armm_ex_i += 1
                        if armm_ex_i >= len(armm_exercises):
                            armm_ex_i = 0 if armm_loop else len(armm_exercises) - 1
                        armm_resting = False
                        armm_ex_t0 = t_am
                        say(f"[hstand:{name}] ARM_MOTION -> {armm_exercises[armm_ex_i].get('name', '?')} "
                            f"t={t_am:.1f}s\n")
                else:
                    _ex = armm_exercises[armm_ex_i]
                    _ex_dur = float(_ex.get("dur_s", 6.0))
                    # ease 0->1 over ramp_s in, 1->0 over ramp_s out so the arms pass through
                    # the rest pose at each boundary (smooth, no step into the rest dwell).
                    ramp = 1.0
                    if armm_ramp_s > 1e-3:
                        ramp = max(0.0, min(te / armm_ramp_s, (_ex_dur - te) / armm_ramp_s, 1.0))
                    for term in _ex.get("terms", []):
                        jn = term.get("joint")
                        if jn is None:
                            continue
                        dv = (float(term.get("bias", 0.0))
                              + float(term.get("amp", 0.0))
                              * math.sin(2.0 * math.pi * float(term.get("freq_hz", 0.0)) * te
                                         + float(term.get("phase", 0.0))))
                        arm_dev[jn] = arm_dev.get(jn, 0.0) + ramp * dv
                    # end of this exercise -> enter the rest dwell before the next one
                    if te >= _ex_dur:
                        armm_resting = True
                        armm_ex_t0 = t_am
                        say(f"[hstand:{name}] ARM_MOTION rest {armm_rest_s:.1f}s "
                            f"(after {_ex.get('name', '?')}) t={t_am:.1f}s\n")
                # apply the exercise deviation ON TOP of arm_balance, clamped to joint limits
                for jn, dv in arm_dev.items():
                    if jn in targets:
                        v = targets[jn] + dv
                        lim = armm_limits.get(jn)
                        if lim is not None:
                            v = lim[0] if v < lim[0] else lim[1] if v > lim[1] else v
                        targets[jn] = v
                # FK the commanded arm pose -> live hand positions (torso frame) = awareness
                _qlh = [float(nominal_map.get(j, 0.0)) + arm_dev.get(j, 0.0) for j in armm_fk["jl"]]
                _qrh = [float(nominal_map.get(j, 0.0)) + arm_dev.get(j, 0.0) for j in armm_fk["jr"]]
                armm_hand_l = armm_fk["fk"](armm_fk["chain_l"], _qlh, armm_fk["tcp_l"])
                armm_hand_r = armm_fk["fk"](armm_fk["chain_r"], _qrh, armm_fk["tcp_r"])
                # forward CoM-back feedforward (hands ahead of rest reach -> lean body back)
                fwd_ext = max(0.0, 0.5 * (armm_hand_l[0] + armm_hand_r[0]) - armm_rest_fwd - armm_ref_pad)
                ff_hip = float(np.clip(armm_c_hip * fwd_ext, 0.0, armm_hip_clamp))
                ff_ank = float(np.clip(armm_c_ank * fwd_ext, 0.0, armm_ank_clamp))
                for jn in hip_pitch:
                    if jn in targets:
                        targets[jn] += ff_hip
                for jn in ankle_pitch:
                    if jn in targets:
                        targets[jn] -= ff_ank            # negative ankle = lean back

        # ── MULTI-STEP capture state machine ──
        # CP = CoM_xy + CoM_vel_xy*sqrt(h/g), expressed in the FULL pelvis frame so the
        # foot lands at the right WORLD spot under roll/pitch. STAND -> STEP (swing the
        # active leg to straddle the CP via IK) -> EVAL (brief double support; if the CP
        # is still escaping, step the OTHER leg = chain) -> RECOVER (ramp both legs back
        # to the nominal stance). Fast small steps alternate legs to chase the CoM.
        if cap_on and cap_cs is not None and self_node is not None:
            tilt = max(abs(roll), abs(pitch))
            tau = math.sqrt(max(bz, 0.3) / 9.81)
            cwx, cwy = vx * tau, vy * tau                   # CP offset from CoM (world)
            _dwz = -(bz)
            cp_fwd = ori[0]*cwx + ori[3]*cwy + ori[6]*_dwz  # CP in full pelvis frame
            cp_lat = ori[1]*cwx + ori[4]*cwy + ori[7]*_dwz
            cp_dwn = ori[2]*cwx + ori[5]*cwy + ori[8]*_dwz
            cp_mag = math.hypot(cp_fwd, cp_lat)

            def _begin_step(leg):
                # foot target straddles the CP by +/- half-stance on the leg's own side
                sgn = 1.0 if leg == "left" else -1.0
                tx = float(np.clip(cp_fwd, -0.22, cap_boxx))
                ty = float(np.clip(cp_lat + sgn * cap_hstance, -cap_boxy, cap_boxy))
                # foot sole target z is a FIXED leg depth (the squat foot height), NOT
                # -base_height: if it tracked the sinking pelvis the legs would just bend
                # shorter and the body would keep sinking. Fixed depth means a sunk pelvis
                # puts the foot target below the floor -> the leg EXTENDS and pushes the
                # body back up (height regulation).
                return np.array([tx, ty, cap_footz])

            if cap_state == "STAND":
                if cp_mag > cap_cp_trig and base_spd > cap_vtrig:
                    cap_q["left"] = _legq_cur("left", targets)   # snapshot current pose (no jump)
                    cap_q["right"] = _legq_cur("right", targets)
                    cap_active = "left" if cp_lat > 0.0 else "right"
                    cap_swing_start = list(cap_q[cap_active])
                    cap_foot_target = _begin_step(cap_active)
                    cap_nstep = 1; cap_t = 0.0; cap_state = "STEP"
                    say(f"[hstand:{name}] STEP {cap_nstep}! cp=({cp_fwd:+.2f},{cp_lat:+.2f}) "
                        f"v={base_spd:.2f} leg={cap_active}\n")
            elif cap_state == "STEP":
                cap_t += step_dt
                frac = min(1.0, cap_t / max(cap_dur, 1e-3))
                ss = frac * frac * (3.0 - 2.0 * frac)
                sfoot = cap_cs.leg_fk(cap_swing_start, cap_active)
                foot = (sfoot + (cap_foot_target - sfoot) * ss).astype(float).copy()
                foot[2] += cap_lift_h * math.sin(math.pi * frac)
                cap_q[cap_active] = list(cap_cs.leg_ik(foot, cap_active, q0=cap_q[cap_active]))
                if frac >= 1.0:
                    cap_state = "EVAL"; cap_t = 0.0
            elif cap_state == "EVAL":
                cap_t += step_dt
                if cap_t >= cap_dbl:
                    if cp_mag > cap_cp_safe and base_spd > cap_vtrig and cap_nstep < cap_maxstep:
                        cap_active = "right" if cap_active == "left" else "left"   # chain: other leg
                        cap_swing_start = list(cap_q[cap_active])
                        cap_foot_target = _begin_step(cap_active)
                        cap_nstep += 1; cap_t = 0.0; cap_state = "STEP"
                        say(f"[hstand:{name}] STEP {cap_nstep}! cp=({cp_fwd:+.2f},{cp_lat:+.2f}) "
                            f"v={base_spd:.2f} leg={cap_active}\n")
                    else:
                        cap_t = 0.0; cap_state = "RECOVER"
            elif cap_state == "RECOVER":
                cap_t += step_dt
                rr = min(1.0, cap_t / max(cap_rec, 1e-3))
                for _lg in ("left", "right"):
                    nq = _legq_nom(_lg)
                    cap_q[_lg] = [cap_q[_lg][i] + (nq[i] - cap_q[_lg][i]) * rr for i in range(6)]
                if rr >= 1.0 and cp_mag < cap_cp_safe and tilt < 0.12:
                    cap_state = "STAND"
                    say(f"[hstand:{name}] recovered -> STAND ({cap_nstep} steps) at t={sim_ms/1000:.2f}s\n")
            # While stepping, command BOTH legs from the tracked poses (stance holds,
            # swing updates) so the IK fully owns the legs (overriding the lean overlay).
            if cap_state != "STAND":
                _set_leg(targets, "left", cap_q["left"])
                _set_leg(targets, "right", cap_q["right"])
                # Active ankle CoP control on top of the IK flat-foot pose -- the fast
                # balance lever the lean overlay normally provides (and that the IK would
                # otherwise lock out). Lean the ankles toward the capture point.
                _cop_p = float(np.clip(-cap_cop * cp_fwd, -0.20, 0.20))   # CP forward -> lean back
                _cop_r = float(np.clip(cap_cop * cp_lat, -0.20, 0.20))    # CP lateral -> lean toward it
                for _lg in ("left", "right"):
                    _apj = f"{_lg}_ankle_pitch_joint"; _arj = f"{_lg}_ankle_roll_joint"
                    if _apj in targets: targets[_apj] += _cop_p
                    if _arj in targets: targets[_arj] += _cop_r

        # ── MANIPULATION overlay (runs LAST so it owns the reach arm) ──
        # Deterministic per-cube state machine: ABOVE -> DESCEND -> GRASP(weld) ->
        # LIFT -> MOVE -> PLACE -> RELEASE -> RETRACT, then the next cube. Each
        # phase has a WORLD TCP target solved once on entry by position-only IK;
        # the arm joints are linearly interpolated in joint space over the phase.
        # The reach-arm targets are FULLY OVERRIDDEN here (the balance arm + legs
        # keep the stand). A small feedforward hip pre-lean offsets the forward CoM
        # shift of the extended arm.
        if manip_on and manip_ok:
            tnow = sim_ms / 1000.0
            if manip_state == "INIT" and tnow >= m_start_s:
                if manip_cube_idx >= len(m_cube_nodes):
                    # exhausted (all remaining cubes skipped) -> done
                    manip_state = "DONE"
                    say(f"[hstand:{name}] MANIP_SUCCESS: {manip_placed}/{len(m_cube_nodes)} cubes placed\n")
                else:
                    _mc = m_cube_nodes[manip_cube_idx]
                    cp = _mc["node"].getPosition() if _mc["node"] is not None else None
                    if cp is None:
                        say(f"[hstand:{name}] MANIP skip {_mc['def']} (no node)\n")
                        manip_cube_idx += 1
                    else:
                        cx, cy, cz = float(cp[0]), float(cp[1]), float(cp[2])
                        px, py, pz = _mc["place"]
                        manip_phases = []
                        if not manip_started:
                            # FIRST cube only: move the arm from the behind/down nominal up
                            # to the forward home pose first, so the ABOVE approach starts
                            # from in front of the table (like every later cube does after
                            # RETRACT) and never sweeps the arm THROUGH the cube.
                            manip_phases.append(("READY", tuple(m_home_tcp), m_move_s, None))
                        manip_phases += [
                            ("ABOVE",   (cx, cy, cz + m_z_above),  m_move_s,        None),
                            ("DESCEND", (cx, cy, cz + m_grasp_dz), m_move_s,        None),
                            ("GRASP",   (cx, cy, cz + m_grasp_dz), m_grasp_dwell_s, "grasp"),
                            ("LIFT",    (cx, cy, cz + m_lift_dz),  m_move_s,        None),
                            ("MOVE",    (px, py, pz + m_lift_dz),  m_move_s,        None),
                            ("PLACE",   (px, py, pz + m_place_dz), m_move_s,        None),
                            ("RELEASE", (px, py, pz + m_place_dz), m_dwell_s,       "release"),
                            ("RETRACT", tuple(m_home_tcp),         m_move_s,        None),
                        ]
                        manip_phase_i = 0
                        manip_enter = True
                        manip_state = "RUN"
            if manip_state == "RUN" and manip_phase_i < len(manip_phases):
                ph_name, ph_tcp, ph_dur, ph_act = manip_phases[manip_phase_i]
                _cd = m_cube_nodes[manip_cube_idx]["def"]
                if manip_enter:
                    manip_enter = False
                    manip_seg_t0 = tnow
                    manip_seg_dur = max(ph_dur, 1e-3)
                    if not manip_started:
                        # start the very first trajectory from where the hand REALLY is
                        manip_cur_tcp = _hand_world_now()
                        manip_started = True
                    manip_tcp_from = tuple(manip_cur_tcp)
                    manip_tcp_to = tuple(ph_tcp)
                    manip_ik_count = 0
                    say(f"[hstand:{name}] MANIP {ph_name} {_cd} tcp=({ph_tcp[0]:.2f},{ph_tcp[1]:.2f},{ph_tcp[2]:.2f}) t={tnow:.1f}s\n")
                    # RELEASE drops the held cube at the start of the dwell (it is already
                    # at the place pose from the PLACE phase).
                    if ph_act == "release" and manip_held is not None:
                        # Clean kinematic set-down (consistent with the kinematic weld
                        # grasp): place the cube exactly at the resting target with zero
                        # velocity. A free "drop" from the carried pose instead gets flung
                        # by the residual teleport-carry velocity in the warp solver.
                        tgt = m_cube_nodes[manip_cube_idx]["place"]
                        try:
                            manip_held_tf.setSFVec3f([tgt[0], tgt[1], tgt[2]])
                            manip_held.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                            manip_held.resetPhysics()
                        except Exception:
                            pass
                        # PIN the cube at the target (zero velocity) for a beat: a plain
                        # release leaves the cube interpenetrating the rubber hand, whose
                        # contact then FLINGS it off the table. Pinning holds it put until
                        # the hand has retracted clear, after which it rests under gravity.
                        manip_pin_node = manip_held
                        manip_pin_tf = manip_held_tf
                        manip_pin_pos = [tgt[0], tgt[1], tgt[2]]
                        manip_pin_ticks = 0
                        # The set-down is deterministic (setSFVec3f to the target); getPosition
                        # lags setSFVec3f by one tick, so we report the commanded target.
                        manip_placed += 1
                        say(f"[hstand:{name}] MANIP_PLACE {_cd} set_at=({tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f}) OK\n")
                        manip_held = None
                        manip_held_tf = None
                # Live cube servoing on the VERTICAL approach only (DESCEND/GRASP): retarget
                # the endpoint to the cube's CURRENT position so the hand homes straight down
                # onto it. NOT on ABOVE (which goes to the planned spot from high up, so the
                # swing-in never sweeps through the cube). Gated to a sane reach window so a
                # knocked-away cube is never chased (which would over-extend the arm / topple).
                if ph_name in ("DESCEND", "GRASP") and manip_held is None:
                    _cn = m_cube_nodes[manip_cube_idx]["node"]
                    _cp = _cn.getPosition() if _cn is not None else None
                    if (_cp is not None and 0.18 <= _cp[0] <= 0.40
                            and -0.40 <= _cp[1] <= 0.10 and _cp[2] >= 0.75):
                        manip_tcp_to = (_cp[0], _cp[1], _cp[2] + m_grasp_dz)
                # Per-tick weld attempt during the DESCEND approach + GRASP phase: attach
                # as soon as the REAL hand is within grasp_radius of the cube. Welding on
                # approach (before the hand presses down) avoids knocking the light cube
                # off its spot, which a press-then-grasp does.
                if (ph_act == "grasp" or ph_name == "DESCEND") and manip_held is None:
                    cn = m_cube_nodes[manip_cube_idx]["node"]
                    cpp = cn.getPosition() if cn is not None else None
                    if cpp is not None:
                        hw = _hand_world_now()
                        d = math.sqrt(sum((hw[k] - cpp[k]) ** 2 for k in range(3)))
                        if d <= m_grasp_radius:
                            manip_held = cn
                            manip_held_tf = m_cube_nodes[manip_cube_idx]["tf"]
                            try:
                                manip_held.resetPhysics()
                            except Exception:
                                pass
                            say(f"[hstand:{name}] MANIP_GRASP attached:{_cd} dist={d*1000:.0f}mm t={tnow:.1f}s\n")
                # World-space trajectory: interpolate the TCP target and re-solve IK
                # against the LIVE base each tick, so the hand tracks the world point even
                # as the body leans back to balance (decouples reach from balance).
                frac = min(1.0, (tnow - manip_seg_t0) / manip_seg_dur)
                manip_cur_tcp = (
                    manip_tcp_from[0] + frac * (manip_tcp_to[0] - manip_tcp_from[0]),
                    manip_tcp_from[1] + frac * (manip_tcp_to[1] - manip_tcp_from[1]),
                    manip_tcp_from[2] + frac * (manip_tcp_to[2] - manip_tcp_from[2]))
                # Re-solve IK at 1/3 the tick rate (the per-tick numerical-Jacobian IK is
                # the dominant CPU cost; the target + body lean move slowly so a stepped
                # setpoint that the stiff PD tracks is smooth enough). Always solve on the
                # first tick of a phase (count==0) and the last (frac>=1) for accuracy.
                if manip_ik_count % 3 == 0 or frac >= 1.0:
                    qsol, ikerr = _ik_world(manip_cur_tcp)
                    manip_q_cmd = qsol
                manip_ik_count += 1
                if frac >= 1.0:
                    if ph_act == "grasp" and manip_held is None:
                        say(f"[hstand:{name}] MANIP_GRASP_MISS {_cd} (no cube within {m_grasp_radius*1000:.0f}mm) t={tnow:.1f}s\n")
                    manip_phase_i += 1
                    manip_enter = True
                    if manip_phase_i >= len(manip_phases):
                        manip_cube_idx += 1
                        if manip_cube_idx >= len(m_cube_nodes):
                            manip_state = "DONE"
                            say(f"[hstand:{name}] MANIP_SUCCESS: {manip_placed}/{len(m_cube_nodes)} cubes placed\n")
                        else:
                            manip_state = "INIT"
            # While manipulating (not yet DONE): override the reach-arm joints with the
            # manip command + apply the CoM-back feedforward. Once DONE, skip BOTH so the
            # reach arm returns to nominal and the two-arm balance resumes (durable stand).
            if manip_state != "DONE":
                for i, jn in enumerate(mreach_joints):
                    if jn in targets:
                        targets[jn] = manip_q_cmd[i]
                # ── Feedforward CoM-back compensation while reaching ──
                # The reach arm extended forward shifts the CoM forward (toward the toes);
                # holding it there topples the stand. Counter it OPEN-LOOP (proportional to
                # the commanded forward reach past the body center, so no destabilising
                # feedback) on THREE levers, strongest first:
                #   ankle: lean the whole body back at the ankle (biggest CoM lever)
                #   hip:   lean the torso back (heavy torso, same sign as auto_trim)
                #   arm:   swing the FREE (balance) arm back as a counterweight (human reach)
                reach_ext = max(0.0, manip_cur_tcp[0] - m_ff_ref)
                ff_hip = float(np.clip(m_c_reach * reach_ext, 0.0, m_ff_clamp))
                ff_ank = float(np.clip(m_c_ank * reach_ext, 0.0, m_ank_clamp))
                ff_cw = float(np.clip(m_c_cw * reach_ext, 0.0, m_cw_clamp))
                for jn in hip_pitch:
                    if jn in targets:
                        targets[jn] += ff_hip
                for jn in ankle_pitch:
                    if jn in targets:
                        targets[jn] -= ff_ank      # negative ankle = lean back (like ank_bias)
                for jn in armb_pitch_joints:       # the balance arm (reach arm excluded)
                    if jn in targets:
                        targets[jn] += ff_cw       # +shoulder_pitch = arm swings back -> CoM back

        for jn in joint_order:
            if motors[jn] is not None:
                motors[jn].setPosition(float(targets[jn]))

        # ── Weld carry: ride the held cube on the live hand pose, frozen ──
        if manip_on and manip_ok and manip_held is not None and manip_held_tf is not None:
            hw = _hand_world_now()
            try:
                manip_held_tf.setSFVec3f([hw[0], hw[1], hw[2]])
                # setVelocity(0) every tick is the warm-solver bounce fix; resetPhysics is
                # done once at grasp (per-tick resetPhysics on a Newton body is costly).
                manip_held.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass

        # Hold a just-released cube pinned at its place target (zero velocity) UNTIL the
        # reach hand has retracted clear of it (distance > m_pin_release_dist), so the
        # hand-cube contact can't fling it off the table. Then let it rest under gravity.
        if manip_on and manip_ok and manip_pin_node is not None:
            hwp = _hand_world_now()
            hand_clear = math.sqrt(sum((hwp[k] - manip_pin_pos[k]) ** 2 for k in range(3)))
            if hand_clear < m_pin_release_dist and manip_pin_ticks < m_pin_max:
                try:
                    manip_pin_tf.setSFVec3f(manip_pin_pos)
                    manip_pin_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    manip_pin_node.resetPhysics()
                except Exception:
                    pass
                manip_pin_ticks += 1
            else:
                # hand has retracted clear -> stop pinning; the cube rests on the table
                say(f"[hstand:{name}] MANIP_PIN released (hand {hand_clear*100:.0f}cm clear) at t={sim_ms/1000:.1f}s\n")
                manip_pin_node = None
                manip_pin_tf = None

        # Throw the next cube at the torso, ONE AT A TIME, on an ARC from where it's
        # parked (off to the side) -- NO teleport (teleport+setVelocity froze the cube
        # in the warp state). Just setVelocity on the resting cube so it launches
        # toward the torso and arcs up to hit it. Re-applied for a few ticks so the
        # launch velocity is reliably flushed to mujoco_warp.
        if (throw_on and throw_cubes and self_node is not None
                and launch_left <= 0 and throw_idx < len(throw_cubes)
                and sim_ms - last_throw_ms >= throw_period_ms
                and not (manip_on and manip_ok and (
                    manip_held is not None
                    or (manip_state == "RUN" and manip_phase_i < len(manip_phases)
                        and manip_phases[manip_phase_i][0] in ("DESCEND", "GRASP"))))):
            cube = throw_cubes[throw_idx]
            try:
                cp = cube.getPosition() or [0.0, 0.0, 0.0]
                # Aim at the body CENTRE at torso height. Exact projectile solve:
                # cross the horizontal gap dh at speed throw_speed (time T=dh/speed),
                # and arrive at the target height at exactly that time -> the cube
                # passes THROUGH the body centre = a solid hit (not a glance).
                tx, ty, tz = bx, by, bz + throw_z
                ddx, ddy, ddz = tx - cp[0], ty - cp[1], tz - cp[2]
                dh = math.hypot(ddx, ddy)
                if dh > 1e-3:
                    ux, uy = ddx / dh, ddy / dh
                    T = dh / max(throw_speed, 0.5)           # time to reach the body
                    vz = (ddz + 0.5 * 9.81 * T * T) / T      # so z hits the target at time T
                    launch_vel = [ux * throw_speed, uy * throw_speed, vz, 0.0, 0.0, 0.0]
                    cube.setVelocity(launch_vel)
                    launch_cube = cube
                    launch_left = 3                          # re-apply for 3 ticks (flush)
                    traced = cube
                    trace_left = max(6, int(T / max(step_dt, 1e-3)) + 6)
                    trace_min = 1e9
                    say(f"[hstand:{name}] THROW #{throw_idx} t={sim_ms/1000:.1f}s "
                        f"from=({cp[0]:+.1f},{cp[1]:+.1f}) speed={throw_speed:.1f} T={T:.2f}\n")
            except Exception as e:
                say(f"[hstand:{name}] throw failed: {e}\n")
            throw_idx += 1
            last_throw_ms = sim_ms

        # Keep re-applying the launch velocity for a few ticks so it flushes to warp.
        if launch_left > 0 and launch_cube is not None:
            try:
                launch_cube.setVelocity(launch_vel)
            except Exception:
                pass
            launch_left -= 1

        # Track each thrown cube's CLOSEST approach to the body -> confirms a hit.
        if traced is not None and trace_left > 0:
            try:
                cp = traced.getPosition() or [0, 0, 0]
                d = math.sqrt((cp[0]-bx)**2 + (cp[1]-by)**2 + (cp[2]-(bz+throw_z))**2)
                trace_min = min(trace_min, d)
                if trace_left == 1:
                    verdict = "HIT" if trace_min < 0.28 else "miss"
                    say(f"[hstand:{name}]   cube closest={trace_min:.2f}m -> {verdict}\n")
            except Exception:
                pass
            trace_left -= 1

        peak_spd = max(peak_spd, base_spd)
        peak_tilt = max(peak_tilt, abs(roll), abs(pitch))
        # Jitter = tick-to-tick pitch change accumulated per second (the "vibration").
        jit_accum += abs(pitch - prev_pitch_j)
        prev_pitch_j = pitch

        # During a deliberate squat the pelvis is SUPPOSED to drop, so relax the
        # height floor by the commanded depth (frac*bz_drop) -- only a true collapse
        # far below the commanded crouch counts as a fall. Tilt limits stay strict.
        eff_bz_min = bz_min - (squat_frac * squat_bz_drop if squat_on else 0.0)
        if squat_on and squat_frac > 0.5:
            squat_min_bz = min(squat_min_bz, bz)
        upright = (bz > eff_bz_min and abs(roll) < roll_max and abs(pitch) < pitch_max)
        if upright:
            survived += 1
        elif fell_at is None and self_node is not None:
            fell_at = sim_ms / 1000.0

        if sim_ms - last_log_ms >= 1000:
            tag = "OK" if upright else f"FALL@{fell_at:.2f}s" if fell_at else "?"
            extra = f" peakV={peak_spd:.2f} peakTilt={peak_tilt:.3f} jit={jit_accum:.3f}"
            olx = f" by={by:+.3f} vy={vy:+.3f}" if oneleg_on else ""
            sqx = f" sq={squat_phase}:{squat_frac:.2f} rep={squat_rep}/{squat_reps}" if squat_on else ""
            mlx = ""
            if manip_on and manip_ok:
                _cps = []
                for _c in m_cube_nodes:
                    if _c["node"] is not None:
                        _q = _c["node"].getPosition() or [0, 0, 0]
                        _cps.append(f"{_c['def'][-1]}=({_q[0]:.2f},{_q[1]:.2f},{_q[2]:.2f})")
                mlx = f" held={'Y' if manip_held is not None else 'n'} st={manip_state} " + " ".join(_cps)
            amx = ""
            if armm_on and armm_ok:
                _exn = armm_exercises[armm_ex_i].get("name", "?") if armm_ex_t0 is not None else "wait"
                amx = (f" ex={_exn} Lh=({armm_hand_l[0]:.2f},{armm_hand_l[1]:.2f},{armm_hand_l[2]:.2f})"
                       f" Rh=({armm_hand_r[0]:.2f},{armm_hand_r[1]:.2f},{armm_hand_r[2]:.2f})")
            say(f"[hstand:{name}] t={sim_ms/1000:5.1f}s bz={bz:.3f} "
                f"roll={roll:+.3f} pitch={pitch:+.3f}  {tag}  "
                f"{'BAL' if use_balance else 'POSE'}{extra}{olx}{sqx}{mlx}{amx}\n")
            last_log_ms = sim_ms
            if squat_on and squat_phase == "DONE" and not squat_done_logged:
                squat_done_logged = True
                _deepest = squat_min_bz if squat_min_bz < 1e8 else bz
                _verdict = "SUCCESS" if fell_at is None else f"FELL@{fell_at:.2f}s"
                say(f"[hstand:{name}] SQUAT_DONE reps={squat_reps} deepest_bz={_deepest:.3f} "
                    f"final upright={upright} -> {_verdict}\n")
                if anim_active:
                    try:
                        robot.animationStopRecording()
                        anim_active = False
                        say(f"[hstand:{name}] ANIM saved\n")
                    except Exception as _ae:
                        say(f"[hstand:{name}] ANIM stop failed: {_ae}\n")
            peak_spd = 0.0
            peak_tilt = 0.0
            jit_accum = 0.0

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException as _e:
        import traceback
        _lp = os.environ.get("HUMANOID_STAND_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
        _tb = traceback.format_exc()
        sys.stderr.write(_tb)
        sys.stderr.flush()
        if _lp:
            try:
                with open(_lp, "a", buffering=1) as _f:
                    _f.write("[hstand] FATAL:\n" + _tb)
            except Exception:
                pass
        sys.exit(1)
