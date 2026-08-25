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

"""g1_sitstand_mimic -- the real, OmniSim/Newton-simulated G1 that TRACKS a
sit->stand->sit reference, dynamically balancing the WHOLE way (NEVER pinned).

This is Stage D (DEPLOY) of the ghost-tracking pipeline: the policy trained by
gpu_mjwarp_g1_sitstand_trainer to TRACK a planner reference runs here, in OmniSim's
Newton (MuJoCo-solver) physics, on the URDF-imported G1 -- so we can watch the real
deployment and measure the trainer->deploy model gap.

  leg_target(t) = clamp(ref_legs(step) + res_scale * policy(obs), joint_limits)  # 13
  arm_target(t) = ref_arms(step)                                                 # 10 feedforward

The reference is a STEP-INDEXED lookup table, identical to the trainer:
  SITSTAND_REF_NPZ=<file>  -> a PLANNER (trajectory-opt) reference (g1_sitstand_trajopt);
  else                     -> the legacy hand-drawn g1_sitstand IK reference.

obs (53) MIRRORS gpu_mjwarp_g1_sitstand_trainer._build_obs_t EXACTLY:
  [lin_vel(3), ang_vel_BODY(3), proj_g(3), q_legs-ref_legs(13), qd_legs(13),
   last_action(13), phase(5)]
  phase = [b, b_ahead(+0.3s), z_err, x_err, pitch_err]
NOTE: the policy ONNX must use CLAMP squashing (reexport_sitstand_onnx.py), NOT the
auto-exported tanh -- the trainer eval uses clamp(mu,-1,1) (see step()).

Telemetry: G1_SITSTAND_LOG=<csv> -> per-second (t, pelvis_z, ref_z, tilt, joint_sim).
Record the achieved motion: G1_SITSTAND_RECORD=<csv> -> per-tick (base x, z, rotation
+ all 23 joints) so the ghost can replay exactly what the robot physically did.
"""

import os
import sys
import math
import traceback
from pathlib import Path

import numpy as np

_DBG = (os.environ.get("G1_SITSTAND_LOG") or
        str(Path(__file__).resolve().parent / "sitstand_mimic")) + ".dbg"


def _dbg(m):
    try:
        with open(_DBG, "a", buffering=1) as f:
            f.write(str(m) + "\n")
    except Exception:
        pass


def _info(m):
    """Log to the dbg file (reliable) AND try stderr (which can raise OSError 22 in
    the OmniSim controller process on Windows -- never let that crash the run)."""
    _dbg(m.rstrip("\n"))
    try:
        sys.stderr.write(m if m.endswith("\n") else m + "\n")
    except Exception:
        pass


_dbg("=== module load ===")

try:
    from omnisim import Supervisor as _Robot
except Exception as e:  # pragma: no cover
    _dbg(f"Supervisor import failed: {e!r}; trying Robot")
    from omnisim import Robot as _Robot

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
if str(_REPO) not in sys.path:
    sys.path.append(str(_REPO))

# Z bounds for the stand-fraction b (MUST match the trainer's g1_sitstand import).
from projects.policies.control.gait.g1_sitstand import Z_SEATED, Z_STAND
import projects.policies.control.gait.g1_sitstand as GS

# warmup_reload dodges the COLD-FIRST-LOAD under-tracking (a fresh articulation
# undershoots its targets). Critical here: standing balance needs crisp tracking.
_warmup_reload = None
try:
    _BRIDGE = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
    if str(_BRIDGE) not in sys.path:
        sys.path.insert(0, str(_BRIDGE))
    from omnilink_arm_bridge import warmup_reload as _warmup_reload
except Exception as e:
    _dbg(f"warmup_reload import FAILED (continuing cold): {e!r}")

# Joint orderings -- MUST match gpu_mjwarp_g1_sitstand_trainer LEGS_JOINTS/ARM_JOINTS.
_LEGS = ("left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
         "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
         "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
         "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
         "waist_yaw_joint")
_ARMS = ("left_shoulder_pitch_joint", "left_shoulder_roll_joint",
         "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
         "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
         "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint")
_ALL23 = _LEGS + _ARMS

_DT = 0.016
_LOOKAHEAD_STEPS = int(round(0.3 / _DT))   # phase feedforward, matches the trainer
# leg joint limits (clamp the target) -- MUST match the trainer JOINT_LIMITS_*.
_JL_LO = np.array([-2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
                   -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618], dtype=np.float32)
_JL_HI = np.array([+2.880, +2.967, +2.758, +2.880, +0.524, +0.262,
                   +2.880, +0.524, +2.758, +2.880, +0.524, +0.262, +2.618], dtype=np.float32)
# residual authority -- MUST equal the trainer's --res-scale (G1_SITSTAND_RES_SCALE).
try:
    _ACT_SCALE = float(os.environ.get("G1_SITSTAND_RES_SCALE", "0.3"))
except (TypeError, ValueError):
    _ACT_SCALE = 0.3


def _build_reference():
    """STEP-INDEXED reference tables, identical to gpu_mjwarp_g1_sitstand_trainer."""
    # OmniSim forwards G1_SITSTAND_*-prefixed env vars to the controller process;
    # bare SITSTAND_* does NOT reach it -- read the prefixed one first.
    npz = os.environ.get("G1_SITSTAND_REF_NPZ") or os.environ.get("SITSTAND_REF_NPZ")
    if npz and os.path.exists(npz):
        d = np.load(npz, allow_pickle=True)
        traj = d["traj"]
        pdt = float(d["dt"])
        pnames = [str(s) for s in d["joints"]]
        tr = traj[:: max(1, int(round(_DT / pdt)))]          # downsample to the deploy DT
        n = tr.shape[0]
        ni = {nm: k for k, nm in enumerate(pnames)}
        REF_LEGS = np.array([[tr[i, ni[j]] for j in _LEGS] for i in range(n)], dtype=np.float32)
        REF_ARMS = np.array([[tr[i, ni[j]] for j in _ARMS] for i in range(n)], dtype=np.float32)
        REF_X = tr[:, 23].astype(np.float32)
        REF_Z = tr[:, 24].astype(np.float32)
        REF_PITCH = (-tr[:, 25]).astype(np.float32)          # planner saved -pitch; obs uses +pitch
        src = f"PLANNER {npz}"
    else:
        n = int(round(GS.T_TOTAL / _DT)) + 30
        FT = [GS.full_targets(i * _DT) for i in range(n)]
        REF_LEGS = np.array([[ft[j] for j in _LEGS] for ft in FT], dtype=np.float32)
        REF_ARMS = np.array([[ft[j] for j in _ARMS] for ft in FT], dtype=np.float32)
        REF_X = np.array([GS.ref_pelvis_x(i * _DT) for i in range(n)], dtype=np.float32)
        REF_Z = np.array([GS.ref_pelvis_z(i * _DT) for i in range(n)], dtype=np.float32)
        REF_PITCH = np.array([GS.ref_pelvis_pitch(i * _DT) for i in range(n)], dtype=np.float32)
        src = "legacy g1_sitstand IK"
    REF_B = np.clip((REF_Z - Z_SEATED) / (Z_STAND - Z_SEATED), 0.0, 1.0).astype(np.float32)
    _info(f"[sitstand_mimic] reference: {src} -> {n} steps")
    return REF_LEGS, REF_ARMS, REF_X, REF_Z, REF_PITCH, REF_B, n


def main():
    robot = _Robot()
    if _warmup_reload is not None:
        try:
            _warmup_reload(robot)
        except Exception as e:
            _dbg(f"warmup_reload raised (continuing): {e!r}")

    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    REF_LEGS, REF_ARMS, REF_X, REF_Z, REF_PITCH, REF_B, MAX_EP = _build_reference()

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None

    motors, sensors, missing = {}, {}, []
    for jn in _ALL23:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            missing.append(jn)
            continue
        motors[jn] = m
        try:
            ps = m.getPositionSensor()
            if ps is not None:
                ps.enable(step_ms)
                sensors[jn] = ps
        except Exception:
            pass
    _info(f"[sitstand_mimic] {len(motors)}/{len(_ALL23)} motors"
          + (f"; MISSING {missing}" if missing else ""))

    # ── RL tracking policy (ONNX, CLAMP squashing) ──────────────────────
    policy_sess = None
    _policy_path = os.environ.get("G1_SITSTAND_POLICY")
    if _policy_path and os.path.exists(_policy_path):
        try:
            import onnxruntime as _ort
            policy_sess = _ort.InferenceSession(_policy_path, providers=["CPUExecutionProvider"])
            _info(f"[sitstand_mimic] RL policy: {_policy_path}\n")
        except Exception as e:
            _info(f"[sitstand_mimic] FATAL: policy EXISTS but failed to load ({e}) -- refusing to run the bare feedforward and report it as a policy result\n")
            policy_sess = None
            # FATAL, deliberately: the policy file EXISTS and would not load. That is a
            # broken environment (classically: onnxruntime missing from the CONTROLLER
            # interpreter -- a DIFFERENT python than the engine embeds), not a mode.
            # Degrading here keeps the robot moving on the bare reference and still exits
            # 0, so the run reports PASS while the policy under test never ran. That
            # silently voided a whole Go2 head-to-head (2026-07-12). The 'policy not
            # found' branch remains the intentional way to run without a policy.
            raise SystemExit(2)
    elif _policy_path:
        _info(f"[sitstand_mimic] policy not found: {_policy_path}; FEEDFORWARD-only\n")

    _prev_q = REF_LEGS[0].copy()
    _qd_alpha = max(0.05, min(1.0, step_dt / (step_dt + 0.030)))
    _qd_sm = np.zeros(13, dtype=np.float32)
    _last_action = np.zeros(13, dtype=np.float32)
    _x_origin = [None]   # set on the first policy tick: maps world-x -> reference frame

    def _read_legs():
        jq = np.empty(13, dtype=np.float32)
        for i, jn in enumerate(_LEGS):
            ps = sensors.get(jn)
            try:
                jq[i] = ps.getValue() if ps is not None else _prev_q[i]
            except Exception:
                jq[i] = _prev_q[i]
        return jq

    def _policy_step(ei):
        """Read state -> 53-dim obs (mirrors the trainer) -> policy -> leg targets."""
        nonlocal _prev_q, _qd_sm, _last_action
        rl = REF_LEGS[ei]
        ra = REF_ARMS[ei]
        ref_x = float(REF_X[ei]); ref_z = float(REF_Z[ei]); ref_pitch = float(REF_PITCH[ei])
        b = float(REF_B[ei])
        b_ahead = float(REF_B[min(ei + _LOOKAHEAD_STEPS, MAX_EP - 1)])
        try:
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel6 = self_node.getVelocity() or [0.0] * 6
            pos = self_node.getPosition()
            bz = float(pos[2]); bx = float(pos[0])
        except Exception:
            ori, vel6, bz, bx = [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.0] * 6, 0.0, 0.0
        lin = np.array(vel6[0:3], dtype=np.float32)                 # global linear (= MuJoCo qvel[0:3])
        # MuJoCo free-joint qvel[3:6] is angular velocity in the BODY frame; getVelocity()
        # returns WORLD angular -> rotate world->body (R^T w) to match the trainer.
        w0, w1, w2 = vel6[3], vel6[4], vel6[5]
        ang = np.array([
            ori[0] * w0 + ori[3] * w1 + ori[6] * w2,
            ori[1] * w0 + ori[4] * w1 + ori[7] * w2,
            ori[2] * w0 + ori[5] * w1 + ori[8] * w2,
        ], dtype=np.float32)
        # projected gravity = R^T[0,0,-1] = [-R20,-R21,-R22] (matches trainer pg from quat)
        proj_g = np.array([-ori[6], -ori[7], -ori[8]], dtype=np.float32)
        # pitch = asin(2(wy-zx)) = asin(proj_g[0])  (algebraically identical)
        pitch = math.asin(max(-1.0, min(1.0, float(proj_g[0]))))
        jq = _read_legs()
        qd_raw = (jq - _prev_q) / max(step_dt, 1e-6)
        _qd_sm = _qd_alpha * qd_raw + (1.0 - _qd_alpha) * _qd_sm
        _prev_q = jq.copy()
        if _x_origin[0] is None:
            _x_origin[0] = bx - ref_x            # so x_err == 0 at the first tick
        z_err = bz - ref_z
        x_err = (bx - _x_origin[0]) - ref_x
        pitch_err = pitch - ref_pitch
        phase = np.array([b, b_ahead, z_err, x_err, pitch_err], dtype=np.float32)
        obs = np.concatenate([lin, ang, proj_g, jq - rl, _qd_sm,
                              _last_action, phase]).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0), -10.0, 10.0)
        if policy_sess is not None:
            try:
                action = np.clip(policy_sess.run(None, {"obs": obs[None, :]})[0][0],
                                 -1.0, 1.0).astype(np.float32)
            except Exception as e:
                _dbg(f"policy inference failed: {e}")
                action = np.zeros(13, dtype=np.float32)
        else:
            action = np.zeros(13, dtype=np.float32)
        leg_tgt = np.clip(rl + _ACT_SCALE * action, _JL_LO, _JL_HI)
        for i, jn in enumerate(_LEGS):
            if jn in motors:
                motors[jn].setPosition(float(leg_tgt[i]))
        for i, jn in enumerate(_ARMS):
            if jn in motors:
                motors[jn].setPosition(float(ra[i]))
        _last_action = action

    # Settle into the seated start pose (open-loop) before the cycle begins.
    for i, jn in enumerate(_LEGS):
        if jn in motors:
            motors[jn].setPosition(float(REF_LEGS[0][i]))
    for i, jn in enumerate(_ARMS):
        if jn in motors:
            motors[jn].setPosition(float(REF_ARMS[0][i]))
    for _ in range(max(1, int(0.5 / step_dt))):
        if robot.step(step_ms) == -1:
            return 0
    _dbg("settle done -> entering main loop")

    # Telemetry + record sinks.
    log_f = None
    log_path = os.environ.get("G1_SITSTAND_LOG")
    if log_path:
        try:
            log_f = open(log_path, "w", buffering=1)
            log_f.write("t,pelvis_z,ref_z,tilt_deg,joint_sim_pct\n")
        except Exception:
            log_f = None
    rec_f = None
    rec_path = os.environ.get("G1_SITSTAND_RECORD")
    if rec_path:
        try:
            rec_f = open(rec_path, "w", buffering=1)
            rec_f.write("x,z,rx,ry,rz,ra," + ",".join(_ALL23) + "\n")
        except Exception:
            rec_f = None

    # G1_SITSTAND_LOOP=1 repeats the cycle; default = play ONCE then hold the last
    # reference step (so the demo shows one sit->stand->sit attempt).
    _loop = os.environ.get("G1_SITSTAND_LOOP", "0") != "0"
    k = 0
    next_log_ms = 1000
    sim_ms = 0
    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        ei = (k % MAX_EP) if _loop else min(k, MAX_EP - 1)
        _policy_step(ei)
        k += 1

        if rec_f is not None and self_node is not None and k <= MAX_EP:
            try:
                p = self_node.getPosition()
                r = self_node.getField("rotation").getSFRotation()
                jv = []
                for jn in _ALL23:
                    ps = sensors.get(jn)
                    jv.append(ps.getValue() if ps is not None else 0.0)
                rec_f.write(f"{p[0]:.5f},{p[2]:.5f},{r[0]:.5f},{r[1]:.5f},{r[2]:.5f},{r[3]:.5f},"
                            + ",".join(f"{v:.5f}" for v in jv) + "\n")
            except Exception:
                pass

        if sim_ms >= next_log_ms:
            next_log_ms += 1000
            errs = []
            for i, jn in enumerate(_LEGS):
                ps = sensors.get(jn)
                if ps is not None:
                    try:
                        errs.append(abs(ps.getValue() - float(REF_LEGS[ei][i])))
                    except Exception:
                        pass
            TOL = 0.5
            joint_sim = 100.0 * sum(max(0.0, 1.0 - e / TOL) for e in errs) / len(errs) if errs else float("nan")
            tilt_deg = float("nan"); bz = float("nan")
            if self_node is not None:
                try:
                    o = self_node.getOrientation()
                    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(o[8])))))
                    bz = float(self_node.getPosition()[2])
                except Exception:
                    pass
            rz = float(REF_Z[ei])
            b = float(REF_B[ei])
            phase_name = "SIT" if b < 0.02 else ("STAND" if b > 0.98 else "MOVE")
            _info(
                f"[sitstand_mimic] t={sim_ms/1000.0:4.1f}s [{phase_name:5s}] pelvis_z={bz:.3f} "
                f"(ref {rz:.3f})  tilt={tilt_deg:4.1f}deg  joint_sim={joint_sim:.1f}%")
            if log_f is not None:
                log_f.write(f"{sim_ms/1000.0:.2f},{bz:.4f},{rz:.4f},{tilt_deg:.2f},{joint_sim:.2f}\n")

    if log_f is not None:
        log_f.close()
    if rec_f is not None:
        rec_f.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException as e:
        _dbg("main CRASHED: " + repr(e) + "\n" + traceback.format_exc())
        raise
