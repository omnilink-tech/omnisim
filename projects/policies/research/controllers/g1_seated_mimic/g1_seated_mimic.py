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

"""g1_seated_mimic -- the real, physics-simulated G1 that MIMICS the ghost.

A Newton-backed G1 with its pelvis pinned (staticBase TRUE in the world, so
there is NO balance problem) seated on a chair. Every tick it drives all 23
joints toward the SAME shared reference the ghost displays
(projects/policies/control/gait/g1_sit_gesture.full_targets) via position control -- so the
robot physically reproduces the ghost's seated wave.

This is the deterministic baseline of the pipeline we are nailing down:
    reference(t) -> position-track under the Newton deploy solver.
An RL residual later replaces the bare copy with
    target = full_targets(t) + ACT_SCALE * residual(obs).

Telemetry: set G1_SIT_LOG=<path> to record a per-second CSV
(t, right_elbow_cmd, right_elbow_act, max_abs_track_err).
"""

import os
import sys
import traceback
from pathlib import Path

_DBG = (os.environ.get("G1_SIT_LOG") or str(Path(__file__).resolve().parent / "seated_mimic")) + ".dbg"


def _dbg(m):
    try:
        with open(_DBG, "a", buffering=1) as f:
            f.write(str(m) + "\n")
    except Exception:
        pass


_dbg("=== module load ===")

try:
    from omnisim import Supervisor as _Robot
    _dbg("controller import OK (Supervisor)")
except Exception as e:  # pragma: no cover
    _dbg(f"Supervisor import failed: {e!r}; trying Robot")
    from omnisim import Robot as _Robot

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())   # .../g1_seated_mimic -> repo root
_dbg(f"_REPO={_REPO}")
if str(_REPO) not in sys.path:
    sys.path.append(str(_REPO))

try:
    from projects.policies.control.gait.g1_sit_gesture import SEATED_POSE, full_targets
    _dbg("gait import OK")
except Exception as e:
    _dbg(f"gait import FAILED: {e!r}")
    raise

# warmup_reload dodges the COLD-FIRST-LOAD under-tracking (a fresh Newton/MuJoCo
# articulation undershoots its targets). For the seated legs this matters: cold,
# they droop during the first settle, the feet stick on the floor, and the
# under-tracked servo can't re-fold them -> the robot sprawls. A warm restart
# tracks the seated pose crisply so the legs hold and only the arms move.
_warmup_reload = None
try:
    _BRIDGE = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
    if str(_BRIDGE) not in sys.path:
        sys.path.insert(0, str(_BRIDGE))
    from omnilink_arm_bridge import warmup_reload as _warmup_reload
    _dbg("warmup_reload import OK")
except Exception as e:
    _dbg(f"warmup_reload import FAILED (continuing cold): {e!r}")

# Joints we read back to measure tracking fidelity (the waving arm).
_TRACK = ("right_elbow_joint", "right_shoulder_roll_joint", "right_wrist_roll_joint")
# Leg joints -- diagnostic: are the seated legs actually HOLDING the commanded
# pose, or sagging (cmd vs act)? A big gap = the legs collapse to the floor.
_LEG = ("left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint")


def main():
    robot = _Robot()

    # Warm the articulation BEFORE any setup (reloads the world once, then this
    # controller restarts warm). See the cold-first-load note above.
    if _warmup_reload is not None:
        try:
            _warmup_reload(robot)
            _dbg("warmup_reload returned (no reload this launch)")
        except Exception as e:
            _dbg(f"warmup_reload raised (continuing): {e!r}")

    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    # Base (pelvis) pin. The whole point is that the pelvis stays fixed (no
    # balance problem) while the arms wave. A Newton FIXED-joint weld on a
    # floating-base humanoid does NOT hold under the leg load -- the pelvis sinks
    # (verified via STEPDIAG body_z: the welded pelvis fell from 0.47 to ~0 while
    # only the massless ghost held). So we PIN it directly each step from this
    # supervisor: reset the free-root pelvis to its spawn pose + zero velocity
    # (the reset_body_pose path the bin/suction demos use). getPosition() is now
    # a real dynamic-body read, so base_drift telemetry is meaningful again.
    self_node = None
    base_p0 = None
    pin_t = None
    pin_r = None
    try:
        self_node = robot.getSelf()
        if self_node is not None:
            base_p0 = list(self_node.getPosition())
            tf = self_node.getField("translation")
            rf = self_node.getField("rotation")
            pin_t = list(tf.getSFVec3f()) if tf is not None else None
            pin_r = list(rf.getSFRotation()) if rf is not None else None
    except Exception:
        self_node = None
    _dbg(f"base spawn pos={base_p0}  pin_t={pin_t} pin_r={pin_r}")

    # G1_SIT_PIN=0 disables the base pin -> the robot must hold itself SEATED on
    # the chair under gravity (the baseline-stability test before adding an RL
    # balance policy). Default 1 (pinned) = the stable deterministic demo.
    _pin_on = os.environ.get("G1_SIT_PIN", "1") != "0"
    _dbg(f"pin_on={_pin_on}")

    def _pin_base():
        """Force the pelvis back to its spawn pose with zero velocity. Holds the
        free-root base fixed so the legs/arms articulate off a stationary pelvis
        (no balance problem) -- a reliable substitute for the weld, which sags."""
        if not _pin_on or self_node is None or pin_t is None:
            return
        try:
            self_node.getField("translation").setSFVec3f(pin_t)
            if pin_r is not None:
                self_node.getField("rotation").setSFRotation(pin_r)
            self_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass

    motors = {}
    sensors = {}
    missing = []
    for jn in SEATED_POSE:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            missing.append(jn)
            continue
        motors[jn] = m
        # (Physics-deploy step: bump per-joint torque/PID here so the arm holds
        # crisply under load -- but keep setAvailableTorque <= the URDF maxTorque,
        # which is 25 for the arms / 35 for the ankles, or the backend warns.)
        try:
            ps = m.getPositionSensor()
            if ps is not None:
                ps.enable(step_ms)
                sensors[jn] = ps
        except Exception:
            pass
    sys.stderr.write(
        f"[seated_mimic] {len(motors)}/{len(SEATED_POSE)} motors"
        + (f"; MISSING {missing}" if missing else "")
        + "\n"
    )
    if missing:
        # A legs-only URDF has no arm motors -> the wave would silently no-op.
        sys.stderr.write("[seated_mimic] WARNING: arm motors missing -- load g1_23dof_omnisim.urdf\n")

    _dbg(f"main: robot ok step_ms={step_ms} motors={len(motors)} sensors={len(sensors)}")

    # ── Optional RL balance policy (G1_SIT_POLICY=<onnx>) ──────────────
    # The 13 leg+waist joints learn to keep the robot SEATED + balanced
    # (UNPINNED) while the arms still follow the ghost wave open-loop. Obs +
    # residual mirror gpu_mjwarp_g1_sit_trainer / g1_stand_arms_deploy exactly:
    #   obs = [lin_vel(3), ang_vel(3), proj_g(3), q-NOMINAL(13), qd(13), last_a(13)]
    #   leg targets = SEATED_NOMINAL + 0.3 * tanh(policy(obs)); arms = wave.
    import numpy as _np
    _LEGS = ("left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
             "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
             "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
             "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
             "waist_yaw_joint")
    _NOMINAL_LEGS = _np.array([SEATED_POSE[j] for j in _LEGS], dtype=_np.float32)
    # Arm feedforward joints, SAME order as gpu_mjwarp_g1_sit_trainer.ARM_JOINTS.
    _ARMS = ("left_shoulder_pitch_joint", "left_shoulder_roll_joint",
             "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
             "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
             "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint")
    _ARM_LOOKAHEAD = 0.12  # s; must match the trainer
    _ACT_SCALE = 0.3
    policy_sess = None
    _policy_path = os.environ.get("G1_SIT_POLICY")
    if _policy_path and os.path.exists(_policy_path):
        try:
            import onnxruntime as _ort
            policy_sess = _ort.InferenceSession(
                _policy_path, providers=["CPUExecutionProvider"])
            _pin_on = False  # the policy does the balancing -> run unpinned
            sys.stderr.write(f"[seated_mimic] RL balance policy: {_policy_path} (UNPINNED)\n")
            _dbg(f"policy loaded {_policy_path}; pin off")
        except Exception as e:
            sys.stderr.write(f"[seated_mimic] policy load FAILED ({e}); deterministic mode\n")
            policy_sess = None
    elif _policy_path:
        sys.stderr.write(f"[seated_mimic] G1_SIT_POLICY not found: {_policy_path}; deterministic\n")
    # Policy obs state.
    _prev_q = _NOMINAL_LEGS.copy()
    _qd_alpha = max(0.05, min(1.0, step_dt / (step_dt + 0.030)))
    _qd_sm = _np.zeros(13, dtype=_np.float32)
    _last_action = _np.zeros(13, dtype=_np.float32)

    def _policy_step(tgt, tgt_ahead):
        """One RL step: read state -> obs -> policy -> leg targets; arms=wave."""
        nonlocal _prev_q, _qd_sm, _last_action
        try:
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel6 = self_node.getVelocity() or [0.0] * 6
        except Exception:
            return
        lin = _np.array(vel6[0:3], dtype=_np.float32)
        ang = _np.array(vel6[3:6], dtype=_np.float32)
        proj_g = _np.array([-ori[6], -ori[7], -ori[8]], dtype=_np.float32)
        jq = _np.empty(13, dtype=_np.float32)
        for i, jn in enumerate(_LEGS):
            ps = sensors.get(jn)
            try:
                jq[i] = ps.getValue() if ps is not None else _prev_q[i]
            except Exception:
                jq[i] = _prev_q[i]
        qd_raw = (jq - _prev_q) / max(step_dt, 1e-6)
        _qd_sm = _qd_alpha * qd_raw + (1.0 - _qd_alpha) * _qd_sm
        _prev_q = jq.copy()
        # Arm feedforward (current + lookahead), same order/scaling as the trainer.
        arm_now = _np.array([tgt[j] for j in _ARMS], dtype=_np.float32)
        arm_ahead = _np.array([tgt_ahead[j] for j in _ARMS], dtype=_np.float32)
        obs = _np.concatenate([lin, ang, proj_g, jq - _NOMINAL_LEGS,
                               _qd_sm, _last_action, arm_now, arm_ahead]).astype(_np.float32)
        obs = _np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        try:
            action = _np.clip(policy_sess.run(None, {"obs": obs[None, :]})[0][0],
                              -1.0, 1.0).astype(_np.float32)
        except Exception as e:
            _dbg(f"policy inference failed: {e}")
            action = _np.zeros(13, dtype=_np.float32)
        leg_tgt = _NOMINAL_LEGS + _ACT_SCALE * action
        for i, jn in enumerate(_LEGS):
            if jn in motors:
                motors[jn].setPosition(float(leg_tgt[i]))
        _last_action = action
        # arms (+ anything not leg/waist) follow the ghost wave
        for jn, q in tgt.items():
            if jn not in _LEGS and jn in motors:
                motors[jn].setPosition(float(q))

    # Settle into the seated pose before the gesture starts.
    for jn, q in SEATED_POSE.items():
        if jn in motors:
            motors[jn].setPosition(float(q))
    _pin_base()
    for _ in range(max(1, int(0.5 / step_dt))):
        if robot.step(step_ms) == -1:
            _dbg("settle: robot.step returned -1 -> EXIT before main loop")
            return 0
        _pin_base()
    _dbg("settle done -> entering main loop")

    log_path = os.environ.get("G1_SIT_LOG")
    log_f = None
    if log_path:
        try:
            # Line-buffered so each row hits disk immediately and survives the
            # abrupt process kill at duration-end.
            log_f = open(log_path, "w", buffering=1)
            log_f.write(f"# seated_mimic init: motors={len(motors)} sensors={len(sensors)} step_ms={step_ms}\n")
            log_f.write("t,relbow_cmd,relbow_act,max_track_err,base_drift_m\n")
        except Exception:
            log_f = None

    # ── Optional trajectory RECORD (G1_SIT_RECORD=<csv>) for the ACHIEVABLE
    # GHOST: dump the robot's achieved per-tick state (all 23 joint angles +
    # pelvis z + base rotation) so g1_seated_ghost can REPLAY it -> the ghost
    # then shows exactly the physically-achievable seated wave (incl. the slight
    # rock), and the robot reproduces it. ──
    _ALL23 = list(SEATED_POSE)
    _rec_f = None
    _rec_path = os.environ.get("G1_SIT_RECORD")
    if _rec_path:
        try:
            _rec_f = open(_rec_path, "w", buffering=1)
            _rec_f.write("z,rx,ry,rz,ra," + ",".join(_ALL23) + "\n")
        except Exception:
            _rec_f = None

    sim_ms = 0
    next_log_ms = 1000
    _dbg("while-loop start")
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        if sim_ms == step_ms:
            _dbg("while-loop: first step executed")
        t = sim_ms / 1000.0
        tgt = full_targets(t)
        if policy_sess is not None:
            # RL balance: legs+waist from the policy (+ arm feedforward), arms=wave.
            _policy_step(tgt, full_targets(t + _ARM_LOOKAHEAD))
        else:
            # Deterministic: every joint open-loop to the reference.
            for jn, q in tgt.items():
                m = motors.get(jn)
                if m is not None:
                    m.setPosition(float(q))
        _pin_base()

        # Record the achieved state for the achievable-ghost replay.
        if _rec_f is not None and self_node is not None:
            try:
                p = self_node.getPosition()
                r = self_node.getField("rotation").getSFRotation()
                jv = []
                for jn in _ALL23:
                    ps = sensors.get(jn)
                    jv.append(ps.getValue() if ps is not None else SEATED_POSE[jn])
                _rec_f.write(f"{p[2]:.5f},{r[0]:.5f},{r[1]:.5f},{r[2]:.5f},{r[3]:.5f},"
                             + ",".join(f"{v:.5f}" for v in jv) + "\n")
            except Exception:
                pass

        if sim_ms >= next_log_ms:
            next_log_ms += 1000
            # Tracking fidelity on the waving arm.
            errs = []
            for jn in _TRACK:
                ps = sensors.get(jn)
                if ps is not None and jn in tgt:
                    try:
                        errs.append(abs(ps.getValue() - tgt[jn]))
                    except Exception:
                        pass
            max_err = max(errs) if errs else float("nan")
            re_cmd = tgt.get("right_elbow_joint", float("nan"))
            re_ps = sensors.get("right_elbow_joint")
            re_act = re_ps.getValue() if re_ps is not None else float("nan")
            # Base drift: distance the pelvis has moved from its spawn pose.
            base_drift = float("nan")
            if self_node is not None and base_p0 is not None:
                try:
                    p = self_node.getPosition()
                    base_drift = (
                        (p[0] - base_p0[0]) ** 2
                        + (p[1] - base_p0[1]) ** 2
                        + (p[2] - base_p0[2]) ** 2
                    ) ** 0.5
                except Exception:
                    pass
            line = (f"t={t:5.1f}s  relbow cmd={re_cmd:+.3f} act={re_act:+.3f}  "
                    f"max_track_err={max_err:.4f}  base_drift={base_drift:.4f}m")
            sys.stderr.write("[seated_mimic] " + line + "\n")
            # Leg-hold diagnostic: commanded vs actual for hip/knee/ankle.
            leg_parts = []
            for jn in _LEG:
                ps = sensors.get(jn)
                if ps is not None and jn in tgt:
                    try:
                        leg_parts.append(f"{jn.split('_joint')[0]} cmd={tgt[jn]:+.2f} act={ps.getValue():+.2f}")
                    except Exception:
                        pass
            if leg_parts:
                sys.stderr.write("[seated_mimic][legs] " + "  ".join(leg_parts) + "\n")
            # ── POSTURE SIMILARITY vs the ghost (verification metric) ──
            # All 23 joints: actual (sensor) vs the ghost reference (full_targets).
            # per-joint match = max(0, 1 - |err|/TOL); TOL=0.5 rad (~29 deg, a
            # clearly-different posture). Plus the "sit straight" check: pelvis
            # upright (torso z-axis vs world up) and pelvis height vs the ghost.
            import math as _m
            errs_all = []
            worst = ("", 0.0)
            for jn in tgt:
                ps = sensors.get(jn)
                if ps is not None:
                    try:
                        e = abs(ps.getValue() - tgt[jn])
                        errs_all.append(e)
                        if e > worst[1]:
                            worst = (jn, e)
                    except Exception:
                        pass
            if errs_all:
                TOL = 0.5
                joint_sim = 100.0 * sum(max(0.0, 1.0 - e / TOL) for e in errs_all) / len(errs_all)
                mean_e = sum(errs_all) / len(errs_all)
                tilt_deg = float("nan")
                bz_now = float("nan")
                if self_node is not None:
                    try:
                        o = self_node.getOrientation()
                        tilt_deg = _m.degrees(_m.acos(max(-1.0, min(1.0, float(o[8])))))
                        bz_now = float(self_node.getPosition()[2])
                    except Exception:
                        pass
                up_sim = max(0.0, 1.0 - (tilt_deg / 29.0)) * 100.0 if tilt_deg == tilt_deg else float("nan")
                ht_sim = max(0.0, 1.0 - abs(bz_now - 0.47) / 0.10) * 100.0 if bz_now == bz_now else float("nan")
                overall = (joint_sim + up_sim + ht_sim) / 3.0
                msg = (f"[seated_mimic][SIM] OVERALL={overall:.1f}%  joints={joint_sim:.1f}% "
                       f"(mean_err={_m.degrees(mean_e):.1f}deg worst={worst[0]} {_m.degrees(worst[1]):.0f}deg)  "
                       f"upright={up_sim:.0f}% (tilt={tilt_deg:.1f}deg)  height={ht_sim:.0f}% "
                       f"(pelvis_z={bz_now:.3f} vs ghost 0.47)\n")
                sys.stderr.write(msg)
                _dbg(msg.strip())
                _dbg(f"legs t={t:.1f}  " + "  ".join(leg_parts))
            if log_f is not None:
                log_f.write(f"{t:.1f},{re_cmd:.4f},{re_act:.4f},{max_err:.4f},{base_drift:.4f}\n")
                log_f.flush()

    _dbg(f"loop exited (sim_ms={sim_ms})")
    if log_f is not None:
        log_f.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException as e:
        _dbg("main CRASHED: " + repr(e) + "\n" + traceback.format_exc())
        raise
