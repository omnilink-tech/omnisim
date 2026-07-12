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

"""Deterministic WALK-DEPLOY: drive the PHYSICS robot in OmniSim Newton to track
the kinematic walking SHADOW (projects/policies/control/gait/<robot>_human_gait) with a stiff
position-PD -- NO RL, NO balance feedback. This is the bare Layer-1 feedforward:
its job is to MEASURE the sim-to-deploy gap (how well does the real Newton
articulation track the verified-feasible shadow, and how long does it stay up?)
before any RL compute is spent. The RL residual (Layer 2) will later add onto the
same shadow targets here.

Robot is chosen by HUMANOID_WALK_ROBOT; the Newton solver config + stiffness come
from the launcher env (same proven config as the deterministic stand). It settles
into the shadow's standing pose, then ramps into the walk and logs forward
distance + roll/pitch + a FALL verdict each tick.
"""
from __future__ import annotations

import importlib
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

# Per-robot joint maps (gait 13-slot LEG order + 10-slot ARM order; None = absent)
# and deploy params. Mirrors humanoid_shadow_ghost.
ROBOTS = {
    "h1": dict(
        gait="h1_human_gait",
        legs=["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
              "left_knee_joint", "left_ankle_joint", None,
              "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_joint", None, "torso_joint"],
        arms=["left_shoulder_pitch_joint", "left_shoulder_roll_joint",
              "left_shoulder_yaw_joint", "left_elbow_joint", None,
              "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
              "right_shoulder_yaw_joint", "right_elbow_joint", None],
        spawn_z=1.01, fall_bz=0.55,
    ),
    "valkyrie": dict(
        gait="valkyrie_human_gait",
        legs=["leftHipPitch", "leftHipRoll", "leftHipYaw",
              "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
              "rightHipPitch", "rightHipRoll", "rightHipYaw",
              "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw"],
        arms=["leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw",
              "leftElbowPitch", "leftWristRoll",
              "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
              "rightElbowPitch", "rightWristRoll"],
        spawn_z=1.08, fall_bz=0.62,
    ),
}


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _rpy(R):
    """roll, pitch from a flat 9-vector row-major rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -R[6])))
    roll = math.atan2(R[7], R[8])
    return roll, pitch


def main() -> int:
    which = os.environ.get("HUMANOID_WALK_ROBOT", "h1").strip()
    cfg = ROBOTS[which]
    gait = importlib.import_module(f"projects.policies.control.gait.{cfg['gait']}")
    d = gait.GaitParams()
    GP = gait.GaitParams(
        vx=_envf("HW_VX", d.vx),
        freq=_envf("HW_FREQ", d.freq),
        ramp_s=_envf("HW_RAMP_S", 2.0),
        style=os.environ.get("HW_STYLE", d.style),
        lateral=os.environ.get("HW_LATERAL", d.lateral),
        yaw=os.environ.get("HW_YAW", d.yaw),
        cp_gain=_envf("HW_CP_GAIN", getattr(d, "cp_gain", 0.0)),     # capture-point step placement
        cp_gain_y=_envf("HW_CP_GAIN_Y", getattr(d, "cp_gain_y", 0.0)),
    )
    settle_s = _envf("HW_SETTLE_S", 1.0)
    fall_bz = _envf("HW_FALL_BZ", cfg["fall_bz"])
    ank_bias = _envf("HW_ANK_BIAS", 0.0)   # backward ankle-pitch trim (neg=lean back)
    # ankle-pitch slots in the gait's 13-vector are 4 (L) and 10 (R).
    _AP_SLOTS = (4, 10)

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0

    # Cold-load fix: reload once so the articulation tracks crisply (warm).
    if os.environ.get("HW_NO_WARMUP", "0").strip() == "0":
        try:
            _b = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
            if str(_b) not in sys.path:
                sys.path.insert(0, str(_b))
            from omnilink_arm_bridge import warmup_reload as _wr
            _wr(robot)
        except Exception as e:
            sys.stderr.write(f"[walk:{which}] warmup skipped: {e}\n")

    names = [n for n in (cfg["legs"] + cfg["arms"]) if n]
    motors = {}
    for jn in names:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m

    self_node = robot.getSelf()

    log_path = os.environ.get("HUMANOID_WALK_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
    log = open(log_path, "w", buffering=1) if log_path else None

    # ── Optional RL RESIDUAL (Layer 2). If HUMANOID_WALK_ONNX is set, load the
    # trained policy and ADD its bounded residual to the shadow baseline. The obs
    # MUST match the trainer's _build_obs_t exactly (order is fatal):
    #   lin_vel(3) + ang_vel_BODY(3) + proj_g(3) + (q-nominal)(nj) + qd(nj)
    #   + last_action(nj) + [sin(phase), cos(phase)]
    # The driven joints are the gait slots this robot actually has (H1: the 11
    # non-ankle-roll slots), in the gait's slot order = the trainer's LEGS_JOINTS.
    onnx_path = os.environ.get("HUMANOID_WALK_ONNX", "")
    sess = None
    keep = [i for i, n in enumerate(cfg["legs"]) if n]      # gait slots driven
    driven = [cfg["legs"][i] for i in keep]
    nj = len(driven)
    nominal_r = gait.standing_pose(GP).astype(np.float32)[keep]
    res_scale = _envf("HUMANOID_WALK_RES_SCALE",
                      float(os.environ.get("H1_RES_SCALE", "0.1")))
    # PURE-RL (full-authority) deploy: target = nominal + act_scale*action, NO gait
    # baseline -- mirrors the trainer's H1_PURE_RL step exactly. The arms are NOT in
    # the action (trainer NJ=11 = legs+torso); the trainer PD-holds the arm DOFs at
    # their MJCF zero target, so here we hold them at 0 too (matching dynamics).
    pure_rl = os.environ.get("HUMANOID_WALK_PURE_RL",
                             os.environ.get("H1_PURE_RL", "0")).strip() == "1"
    act_scale = _envf("HUMANOID_WALK_ACT_SCALE", _envf("H1_ACT_SCALE", 1.0))
    # closed-loop: frame-stack the last K obs (MUST equal the trainer's --obs-history).
    obs_hist_k = max(1, int(_envf("HUMANOID_WALK_OBS_HISTORY",
                                  _envf("H1_OBS_HISTORY", 1))))
    obs_hist = []          # last K single-frame obs (filled on the first pure-RL tick)
    pos_sensors = {}
    last_action = np.zeros(nj, dtype=np.float32)
    prev_q_sensor = nominal_r.copy()
    if onnx_path and os.path.exists(onnx_path):
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
            for jn in driven:
                mtr = motors.get(jn)
                if mtr is not None:
                    s = mtr.getPositionSensor()
                    if s is not None:
                        s.enable(step_ms)
                        pos_sensors[jn] = s
        except Exception as e:
            sys.stderr.write(f"[walk:{which}] residual load failed ({e}); shadow-only\n")
            sess = None
    _dbg = {"res": 0.0, "amax": 0.0, "vx": 0.0}   # launch-trace telemetry

    def say(msg):
        sys.stderr.write(msg)
        sys.stderr.flush()
        if log:
            log.write(msg)
            log.flush()

    if sess is not None and pure_rl:
        mode = "pure-RL (full-authority)"
    elif sess is not None:
        mode = "shadow+RL-residual"
    else:
        mode = "shadow-only"
    say(f"[walk:{which}] {mode} deploy: gait={cfg['gait']} vx={GP.vx} "
        f"freq={GP.freq} lateral={GP.lateral} settle={settle_s}s cp_gain={GP.cp_gain}"
        f"{f' res_scale={res_scale}' if (sess is not None and not pure_rl) else ''}"
        f"{f' act_scale={act_scale}' if (sess is not None and pure_rl) else ''}\n")

    def _hold_nominal():
        """Pure-RL settle / hold: legs+torso at nominal, arms at 0 (trainer match)."""
        for k, jn in enumerate(driven):
            if jn in motors:
                motors[jn].setPosition(float(nominal_r[k]))
        for jn in cfg["arms"]:
            if jn and jn in motors:
                motors[jn].setPosition(0.0)

    def drive(phase, tss):
        _vm = float((self_node.getVelocity() or [0.0])[0])   # measured forward vel (capture-point)
        legs, arms, _ = gait.targets_np(phase, GP, t_since_start=tss, v_meas=_vm)
        if ank_bias:
            for s in _AP_SLOTS:
                legs[s] += ank_bias            # lean the stance back (CoM trim)
        for i, jn in enumerate(cfg["legs"]):
            if jn and jn in motors:
                motors[jn].setPosition(float(legs[i]))
        for k, jn in enumerate(cfg["arms"]):
            if jn and jn in motors:
                motors[jn].setPosition(float(arms[k]))

    def drive_residual(phase, tss):
        """Shadow baseline (+ capture-point foot placement) + the trained RL residual."""
        nonlocal last_action, prev_q_sensor
        # obs (must match the trainer's _build_obs_t order exactly)
        vel6 = self_node.getVelocity() or [0.0] * 6
        lin = np.asarray(vel6[0:3], dtype=np.float32)
        legs, arms, _ = gait.targets_np(phase, GP, t_since_start=tss, v_meas=float(lin[0]))
        if ank_bias:
            for s in _AP_SLOTS:
                legs[s] += ank_bias
        base_r = np.array([legs[i] for i in keep], dtype=np.float32)
        wx, wy, wz = float(vel6[3]), float(vel6[4]), float(vel6[5])
        o = self_node.getOrientation()
        ang_b = np.array([o[0] * wx + o[3] * wy + o[6] * wz,     # R^T * omega_world
                          o[1] * wx + o[4] * wy + o[7] * wz,
                          o[2] * wx + o[5] * wy + o[8] * wz], dtype=np.float32)
        pg = np.array([-o[6], -o[7], -o[8]], dtype=np.float32)   # R^T * (0,0,-1)
        qpos = np.array([float(pos_sensors[jn].getValue()) if jn in pos_sensors
                         else nominal_r[k] for k, jn in enumerate(driven)],
                        dtype=np.float32)
        qd = (qpos - prev_q_sensor) / dt
        prev_q_sensor = qpos
        obs = np.concatenate([lin, ang_b, pg, qpos - nominal_r, qd, last_action,
                              np.array([math.sin(phase), math.cos(phase)],
                                       dtype=np.float32)]).astype(np.float32)
        act = sess.run(None, {sess.get_inputs()[0].name: obs[None, :]})[0][0]
        act = np.clip(act, -1.0, 1.0).astype(np.float32)
        last_action = act
        _dbg["res"] = float(np.abs(act).mean())
        _dbg["amax"] = float(np.abs(act).max())
        _dbg["vx"] = float(lin[0])
        tgt = base_r + res_scale * act
        for k, jn in enumerate(driven):
            if jn in motors:
                motors[jn].setPosition(float(tgt[k]))
        for k, jn in enumerate(cfg["arms"]):
            if jn and jn in motors:
                motors[jn].setPosition(float(arms[k]))

    def drive_purerl(phase, tss):
        """Full-authority pure-RL: target = nominal + act_scale*action (NO gait
        baseline). Obs is IDENTICAL to drive_residual (the trainer's _build_obs_t),
        incl. the sin/cos phase clock the policy uses as a rhythm cue. Arms held at 0."""
        nonlocal last_action, prev_q_sensor
        vel6 = self_node.getVelocity() or [0.0] * 6
        lin = np.asarray(vel6[0:3], dtype=np.float32)
        wx, wy, wz = float(vel6[3]), float(vel6[4]), float(vel6[5])
        o = self_node.getOrientation()
        ang_b = np.array([o[0] * wx + o[3] * wy + o[6] * wz,
                          o[1] * wx + o[4] * wy + o[7] * wz,
                          o[2] * wx + o[5] * wy + o[8] * wz], dtype=np.float32)
        pg = np.array([-o[6], -o[7], -o[8]], dtype=np.float32)
        qpos = np.array([float(pos_sensors[jn].getValue()) if jn in pos_sensors
                         else nominal_r[k] for k, jn in enumerate(driven)],
                        dtype=np.float32)
        qd = (qpos - prev_q_sensor) / dt
        prev_q_sensor = qpos
        frame = np.concatenate([lin, ang_b, pg, qpos - nominal_r, qd, last_action,
                                np.array([math.sin(phase), math.cos(phase)],
                                         dtype=np.float32)]).astype(np.float32)
        if obs_hist_k > 1:
            if not obs_hist:
                obs_hist.extend([frame] * obs_hist_k)   # fill on first tick (no stale)
            else:
                obs_hist.append(frame)
                del obs_hist[0]
            obs = np.concatenate(obs_hist).astype(np.float32)   # oldest..newest
        else:
            obs = frame
        act = sess.run(None, {sess.get_inputs()[0].name: obs[None, :]})[0][0]
        act = np.clip(act, -1.0, 1.0).astype(np.float32)
        last_action = act
        _dbg["res"] = float(np.abs(act).mean())
        _dbg["amax"] = float(np.abs(act).max())
        _dbg["vx"] = float(lin[0])
        tgt = nominal_r + act_scale * act
        for k, jn in enumerate(driven):
            if jn in motors:
                motors[jn].setPosition(float(tgt[k]))
        for jn in cfg["arms"]:
            if jn and jn in motors:
                motors[jn].setPosition(0.0)

    # Seed the standing pose BEFORE the first step() so OMNISIM_NEWTON_SEED_POSE
    # spawns the articulation already in the squat. Without this the seed reads
    # zero targets (straight legs from the URDF default), then the stiff PD snaps
    # the legs into the deep squat and the transient explodes (pitch -1.3 in one
    # tick). The stand avoids this the same way.
    _hold_nominal() if pure_rl else drive(gait.DS_PHASE, 0.0)

    t = 0.0
    t_walk = 0.0
    x0 = None
    fell_at = None
    peak_x = 0.0
    last_log = -1.0
    trace_last = -1.0
    while robot.step(step_ms) != -1:
        t += dt
        if t < settle_s:
            phase, tss = gait.DS_PHASE, 0.0
            _hold_nominal() if pure_rl else drive(phase, tss)   # settle
        else:
            t_walk += dt
            phase = (gait.DS_PHASE + 2.0 * math.pi * GP.freq * t_walk) % (2.0 * math.pi)
            tss = t_walk
            if sess is not None and pure_rl:
                drive_purerl(phase, tss)
            elif sess is not None:
                drive_residual(phase, tss)
            else:
                drive(phase, tss)

        pos = self_node.getPosition()
        R = self_node.getOrientation()
        roll, pitch = _rpy(R)
        bx, bz = pos[0], pos[2]
        if x0 is None:
            x0 = bx
        dist = bx - x0
        peak_x = max(peak_x, dist)
        down = (bz < fall_bz) or (abs(roll) > 0.8) or (abs(pitch) > 0.8)
        if down and fell_at is None and t > settle_s:
            fell_at = t_walk
            say(f"[walk:{which}] FALL@{t_walk:.2f}s  dist={dist:+.2f}m "
                f"bz={bz:.3f} roll={roll:+.2f} pitch={pitch:+.2f}\n")
        # LAUNCH TRACE: fine-grained telemetry through the first ~1.5 s of walking
        # so the residual's effect (or absence) at the deploy launch is visible.
        if sess is not None and t >= settle_s and t_walk <= 1.5 and t - trace_last >= 0.15:
            trace_last = t
            say(f"[trace] t_walk={t_walk:.2f} pitch={pitch:+.3f} vx_obs={_dbg['vx']:+.3f} "
                f"|res|={_dbg['res']:.3f} res_max={_dbg['amax']:.3f} bz={bz:.3f}\n")
        if t - last_log >= 1.0:
            last_log = t
            tag = "WALK" if t >= settle_s else "settle"
            say(f"[walk:{which}] t={t:5.1f}s {tag} dist={dist:+.2f}m peak={peak_x:+.2f}m "
                f"bz={bz:.3f} roll={roll:+.2f} pitch={pitch:+.2f} "
                f"{'FALLEN' if fell_at else 'up'}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
