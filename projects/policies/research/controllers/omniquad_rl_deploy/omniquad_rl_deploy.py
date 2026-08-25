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

"""OmniQuad RL deploy controller — loads an ONNX policy and runs it.

A drop-in replacement for omniquad_simple_pose / omniquad_rl_agent that:
  - reads the ONNX policy from a path passed via env var OMNIQUAD_POLICY_ONNX
    (default: projects/policies/research/inference/policies/omniquad_ppo_main/policy.onnx)
  - builds the same 49-D observation the training controller built
  - runs inference each tick via onnxruntime (CPU is fine, ~1-2 ms / call)
  - sends the resulting residual joint deltas to the motors, scaled by
    ACTION_SCALE and clamped to URDF joint limits

The world file points its URDFRobot at this controller; the agent runs
forever (no episode reset) until simulation ends.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = []
for leg in LEGS:
    for joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((leg, joint))

OBS_DIM = 49
ACT_DIM = 12

HIP_X_LEFT = 0.30
HIP_X_RIGHT = -0.30
HIP_Y_STAND = 0.30
KNEE_STAND = -0.60

NOMINAL_POSE = np.zeros(ACT_DIM, dtype=np.float32)
for i, (leg, joint) in enumerate(JOINT_ORDER):
    if joint == "hip_x":
        NOMINAL_POSE[i] = HIP_X_LEFT if "left" in leg else HIP_X_RIGHT
    elif joint == "hip_y":
        NOMINAL_POSE[i] = HIP_Y_STAND
    else:
        NOMINAL_POSE[i] = KNEE_STAND

ACTION_SCALE = 0.15  # must match the value in omniquad_rl_agent.py at train time

_DEPLOY_LOOP = bool(os.environ.get("OMNISIM_DEPLOY_LOOP") or os.environ.get("OMNIQUAD_DEPLOY_LOOP"))
_DEPLOY_DBG = bool(os.environ.get("OMNISIM_DEPLOY_DBG"))


def _pitch_trim_vec(pitch_trim: float) -> np.ndarray:
    """Per-joint feedforward pitch-trim offset (added to the fore/aft hip
    joints only). MUST match what the policy trained with (the GPU trainer
    adds PITCH_TRIM=0.20 to every hip_y joint) or the gait equilibrium is
    wrong and OmniQuad tips. Resolved from the sidecar/env in main()."""
    vec = np.zeros(ACT_DIM, dtype=np.float32)
    for _i, (_leg, _joint) in enumerate(JOINT_ORDER):
        if _joint == "hip_y":
            vec[_i] = pitch_trim
    return vec


# ── CPG trotting prior — mirror of the training controller ──
# When OMNIQUAD_CPG_FREQ_HZ > 0, joint target becomes:
#     target = NOMINAL_POSE + CPG_BASE(t) + ACTION_SCALE * policy_action
# Matching what the training agent does so the policy sees the same
# observation-target distribution as it was trained against. Reading
# from env vars (set in the launching script, e.g. tools/eval_walk.py
# or whoever spawns webots).
CPG_LEG_PHASE = {
    "front_left":  0.0,
    "front_right": 0.5,
    "rear_left":   0.5,
    "rear_right":  0.0,
}


def _env_float(key, default):
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


# Gait mode. "legacy" = the sin^2 knee flex EVERY shipped policy
# (omniquad_walk_v12, omniquad_newton_v*, ...) was TRAINED against -- it must stay
# the default so deploying a trained policy uses the exact gait prior it
# learned to refine. "trot" = the single-flex diagonal trot that nets
# forward motion open-loop; only correct for a policy trained with it (or
# for OMNISIM_GAIT_ONLY open-loop demos). Select via OMNISIM_CPG_GAIT.
_GAIT_MODE = (os.environ.get("OMNISIM_CPG_GAIT") or os.environ.get("OMNIQUAD_CPG_GAIT")
              or "legacy").strip().lower()


# Knee phase offset (radians) added to the knee's theta only, shifting the
# foot-lift timing relative to the hip swing. This sets the trot's PROPULSION
# DIRECTION: the open-loop gait in the Newton deploy drives the body backward
# at the default 0.0 offset; a ~pi offset reverses it to forward. Swept to
# find a stable forward walk. Opt-in via OMNISIM_CPG_KNEE_PHASE (default 0 =
# legacy behaviour unchanged).
_KNEE_PHASE = _env_float("OMNISIM_CPG_KNEE_PHASE", 0.0)


def cpg_base_offsets(t, freq_hz, hip_y_amp, knee_amp):
    """Per-joint CPG offsets added on top of NOMINAL_POSE. Diagonal trot
    phasing (FL/RR vs FR/RL). Two knee-flex shapes selectable via
    _GAIT_MODE (see above):

      legacy: knee flexes with sin^2(theta) -- foot lifts TWICE per thigh
              cycle (a near-march-in-place; this is what shipped policies
              learned on, so it's the default).
      trot:   knee flexes ONCE per cycle, gated to the forward-swing half
              (sin<0), foot planted through the rearward stance sweep so a
              planted foot pushes the body forward -- nets real forward
              displacement open-loop, but only matches a policy trained
              with it.
    """
    base = np.zeros(ACT_DIM, dtype=np.float32)
    if freq_hz <= 0.0 or (hip_y_amp == 0.0 and knee_amp == 0.0):
        return base
    omega = 2.0 * math.pi * freq_hz
    trot = _GAIT_MODE == "trot"
    for i, (leg, joint) in enumerate(JOINT_ORDER):
        phase = CPG_LEG_PHASE[leg]
        theta = omega * t + 2.0 * math.pi * phase
        if joint == "hip_y":
            base[i] = hip_y_amp * math.cos(theta)
        elif joint == "knee":
            ktheta = theta + _KNEE_PHASE   # shift foot-lift timing -> sets direction
            if trot:
                base[i] = -knee_amp * max(0.0, -math.sin(ktheta))
            else:
                s = max(0.0, math.sin(ktheta))
                base[i] = -knee_amp * (s * s)
    return base

JOINT_LIMITS_LO = np.array([
    +0.001, +0.001, -1.20,
    -0.60,  +0.001, -1.20,
    +0.001, +0.001, -1.20,
    -0.60,  +0.001, -1.20,
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +0.60,  +0.60,  -0.01,
    -0.001, +0.60,  -0.01,
    +0.60,  +0.60,  -0.01,
    -0.001, +0.60,  -0.01,
], dtype=np.float32)


def projected_gravity(ori9):
    return np.array([-ori9[2], -ori9[5], -ori9[8]], dtype=np.float32)


def find_policy_path() -> Path:
    """Return the first path that exists, in priority order."""
    candidates = []
    # Webots' batch mode drops OMNIQUAD_-prefixed env vars before they reach
    # the controller subprocess for some --batch/--mode=fast combos
    # (same failure that hit OMNIQUAD_DEPLOY_TRACE). OMNISIM_-prefixed vars
    # survive, so check both -- without the OMNISIM_ fallback every
    # headless eval silently loaded the omniquad_ppo_main default instead of
    # the policy under test, making four different trained policies all
    # produce byte-identical "falls at 0.688 s" traces.
    env_path = os.environ.get("OMNIQUAD_POLICY_ONNX") or os.environ.get("OMNISIM_POLICY_ONNX")
    if env_path:
        candidates.append(Path(env_path))
    repo_root = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
    # Default walker: omniquad_walk_v12_200k is the verified-stable ODE walker
    # (upright 100%, never falls, trots ~0.5 m/s) -- the current shipped
    # OmniQuad solution. Tried before the legacy omniquad_ppo_main slot so the
    # deploy world "just works" with no env var.
    candidates.append(repo_root / "projects" / "rl" / "inference" / "policies" / "omniquad_walk_v12_200k" / "policy.onnx")
    # Legacy canonical slot -- eval_policy.py copies here.
    candidates.append(repo_root / "projects" / "rl" / "inference" / "policies" / "omniquad_ppo_main" / "policy.onnx")
    # Sometimes Webots starts the controller with a weird cwd; also look
    # relative to the controller dir.
    candidates.append(Path(__file__).parent / "policy.onnx")
    for c in candidates:
        if c.exists():
            return c
    # Return the last candidate so the caller's error message points somewhere.
    return candidates[-1]


def _load_sidecar(policy_path: Path) -> dict:
    """Read robot_spec.json next to the policy. Returns {} if missing so
    the legacy defaults below still kick in for old policies."""
    spec_path = policy_path.parent / "robot_spec.json"
    if not spec_path.exists():
        return {}
    try:
        import json
        return json.loads(spec_path.read_text())
    except Exception as _e:
        sys.stderr.write(f"[omniquad_rl_deploy] sidecar parse failed ({_e}); using legacy defaults\n")
        return {}


def main() -> int:
    # Mirror everything we send to stderr into a known file too. Webots'
    # batch-mode pipe to host stdout drops controller output silently
    # under some flag combinations -- this side-channel lets the demo
    # harness still see what the controller did.
    # OMNIQUAD_DEPLOY_LOG is dropped by Webots batch mode; OMNISIM_DEPLOY_LOG
    # survives. Check both so the side log captures reliably in headless
    # --batch runs (where OMNIQUAD_-prefixed env vars never reach us).
    _side_log_path = os.environ.get("OMNIQUAD_DEPLOY_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
    _side_log = None
    if _side_log_path:
        try:
            _side_log = open(_side_log_path, "w", buffering=1)
        except Exception:
            _side_log = None
    def _say(msg: str) -> None:
        try:
            sys.stderr.write(msg)
            sys.stderr.flush()
        except Exception:
            pass
        if _side_log is not None:
            try:
                _side_log.write(msg)
                _side_log.flush()  # survive a hard kill / Newton segfault
            except Exception:
                pass
    _say("[omniquad_rl_deploy] starting\n")

    policy_path = find_policy_path()
    _say(f"[omniquad_rl_deploy] policy: {policy_path}\n")
    if not policy_path.exists():
        sys.stderr.write(f"[omniquad_rl_deploy] ERROR: policy not found at {policy_path}\n")
        sys.stderr.write("  Set OMNIQUAD_POLICY_ONNX env var, or export a policy via\n")
        sys.stderr.write("  projects/policies/research/inference/export_onnx.py\n")
        return 1

    # Read motor_pid + action_scale from the policy's sidecar. Different
    # trained policies use different gains (CPU-SB3 trained at kp=20 /
    # action_scale=0.15; MJX trained at kp=500 / action_scale=0.05) and
    # the deploy controller MUST match the training-side values or the
    # policy's joint commands either underwhelm or overpower the motors.
    sidecar = _load_sidecar(policy_path)
    motor_pid_arg = tuple(sidecar.get("motor_pid", (20.0, 0.0, 0.3)))
    action_scale_arg = float(sidecar.get("action_scale", ACTION_SCALE))
    # Pitch trim: env override first (ad-hoc deploys), else the sidecar value
    # (authoritative for a trained policy). Default 0.0 for legacy policies
    # that trained without trim. Previously this read ONLY the env var and
    # ignored the sidecar -- so headless evals (which don't set the env var)
    # deployed the policy with zero trim while it trained with +0.20, a
    # wrong-equilibrium mismatch that tipped the robot.
    _pt_env = os.environ.get("OMNIQUAD_PITCH_TRIM") or os.environ.get("OMNISIM_PITCH_TRIM")
    if _pt_env is not None and _pt_env.strip() != "":
        try:
            pitch_trim = float(_pt_env)
        except ValueError:
            pitch_trim = float(sidecar.get("pitch_trim", 0.0))
    else:
        pitch_trim = float(sidecar.get("pitch_trim", 0.0))
    PITCH_TRIM_VEC = _pitch_trim_vec(pitch_trim)
    _say(f"[omniquad_rl_deploy] pitch_trim={pitch_trim} (sidecar+env)\n")
    # OMNIQUAD_KP env var overrides the sidecar kp. Used by the Newton demo
    # where the actuator is softer than ODE's internal PD and the
    # sidecar's ODE-tuned kp=20 leaves the legs collapsing under
    # gravity. Setting OMNIQUAD_KP=200 (10x stiffer) gives Newton's
    # POSITION_VELOCITY actuator enough force authority to hold the
    # standing pose against the chassis weight.
    # OMNIQUAD_KP is dropped by Webots batch mode; OMNISIM_KP survives. Check
    # both so the stiffer-actuator override actually reaches the controller
    # in headless --batch runs (where the ODE-tuned kp=20 collapses).
    _kp_override = os.environ.get("OMNIQUAD_KP") or os.environ.get("OMNISIM_KP")
    if _kp_override:
        try:
            new_kp = float(_kp_override)
            motor_pid_arg = (new_kp,) + motor_pid_arg[1:]
        except ValueError:
            pass
    _say(f"[omniquad_rl_deploy] sidecar motor_pid={motor_pid_arg} "
         f"action_scale={action_scale_arg}\n")

    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(policy_path),
                                    providers=["CPUExecutionProvider"])
    except Exception as e:
        _say(f"[omniquad_rl_deploy] FATAL: ONNX policy exists but failed to load ({e}) -- "
             "refusing to run and report a bare-baseline result\n")
        # FATAL, deliberately: the policy file EXISTS and would not load -- a broken
        # environment (classically: onnxruntime missing from the CONTROLLER interpreter,
        # a DIFFERENT python than the engine embeds), not a mode. Falling back here keeps
        # the robot moving on its bare baseline and still exits 0, so the harness records
        # a PASS for a policy that never ran. That silently voided an entire Go2
        # head-to-head (2026-07-12) -- and the broken run scored BETTER than the real one.
        raise SystemExit(2)
    _say(f"[omniquad_rl_deploy] ONNX loaded: {policy_path}\n")
    # Auto-detect the policy's expected obs dim from the ONNX input shape.
    # 49 = legacy layout (v12_200k and friends); 50 = obs[49] is heading
    # deviation (added 2026-05-23 to support a real straight-walker).
    # Best-effort only: a symbolic/dynamic input shape keeps the default
    # OBS_DIM -- an autodetect miss is NOT a load failure and must not abort.
    try:
        _shape = sess.get_inputs()[0].shape
        _detected = int([d for d in _shape if isinstance(d, int)][-1])
        if _detected in (49, 50):
            OBS_DIM = _detected
            _say(f"[omniquad_rl_deploy] policy obs_dim auto-detected = {OBS_DIM}\n")
    except Exception:
        pass

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0
    _say(f"[omniquad_rl_deploy] robot ok, step_ms={step_ms}\n")

    motors = []
    sensors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            sys.stderr.write(f"[omniquad_rl_deploy] missing motor {name}\n")
            return 1
        motors.append(m)
        try:
            if hasattr(m, "setControlPID"):
                kp_w, ki_w, kd_w = motor_pid_arg
                m.setControlPID(float(kp_w), float(ki_w), float(kd_w))
        except Exception:
            pass
        s = m.getPositionSensor()
        if s is not None:
            s.enable(step_ms)
        sensors.append(s)

    self_node = robot.getSelf()
    if self_node is None:
        sys.stderr.write("[omniquad_rl_deploy] no supervisor handle\n")
        return 1

    _say("[omniquad_rl_deploy] motors/sensors registered, applying nominal pose\n")
    # Apply nominal standing pose t=0.
    for i, m in enumerate(motors):
        m.setPosition(float(NOMINAL_POSE[i]))

    # Settle phase: step the sim 20 ticks holding nominal pose so the body
    # stops bouncing from the spawn height before the policy starts driving.
    # Matches the training controller's 20-tick post-reset settle. Without
    # this, the first policy action runs against an obs from a still-falling
    # body and the robot tips immediately on every deploy.
    settled_ok = True
    for _ in range(20):
        if robot.step(step_ms) == -1:
            settled_ok = False
            break
    _say(f"[omniquad_rl_deploy] settle complete (ok={settled_ok})\n")

    prev_joint_pos = NOMINAL_POSE.copy()
    for i, s in enumerate(sensors):
        if s is not None:
            try:
                prev_joint_pos[i] = float(s.getValue())
            except Exception:
                pass
    last_action = np.zeros(ACT_DIM, dtype=np.float32)

    # CPG params: sidecar first, env-var override second. Sidecar is the
    # authoritative source for a trained policy (its CPG was the prior the
    # policy learned to refine); env vars are useful for ad-hoc deploys.
    # Default 0.0 freq disables the CPG entirely.
    # OMNIQUAD_-prefixed env vars are dropped by Webots batch mode before they
    # reach the controller; OMNISIM_-prefixed survive. Check both so the
    # gait stays tunable from a headless eval (env) AND from a trained
    # policy's sidecar (the authoritative source once a policy ships).
    def _cpg(omniquad_key, omni_key, sidecar_key):
        v = os.environ.get(omniquad_key) or os.environ.get(omni_key)
        if v is not None and v.strip() != "":
            try:
                return float(v)
            except ValueError:
                pass
        return float(sidecar.get(sidecar_key, 0.0))
    cpg_freq = _cpg("OMNIQUAD_CPG_FREQ_HZ", "OMNISIM_CPG_FREQ_HZ", "cpg_freq_hz")
    cpg_hip_amp = _cpg("OMNIQUAD_CPG_HIP_Y_AMP", "OMNISIM_CPG_HIP_Y_AMP", "cpg_hip_y_amp")
    cpg_knee_amp = _cpg("OMNIQUAD_CPG_KNEE_AMP", "OMNISIM_CPG_KNEE_AMP", "cpg_knee_amp")
    _say(
        f"[omniquad_rl_deploy] cpg: freq={cpg_freq}Hz hip={cpg_hip_amp} "
        f"knee={cpg_knee_amp} (from sidecar+env)\n"
    )
    # Open-loop gait test: zero the policy action so the motor target is
    # pure NOMINAL_POSE + CPG. Lets us validate that the scripted trot
    # alone walks the robot (decouples "can it physically walk" from "did
    # RL learn to walk"). Either prefix -- OMNISIM_ survives Webots batch
    # mode, OMNIQUAD_ is convenient from a shell.
    _go = os.environ.get("OMNISIM_GAIT_ONLY") or os.environ.get("OMNIQUAD_GAIT_ONLY")
    gait_only = bool(_go) and _go.strip() not in ("", "0", "false", "False")
    if gait_only:
        _say("[omniquad_rl_deploy] GAIT-ONLY mode: policy action zeroed\n")
    # Default forward walk command. Env vars don't always propagate through
    # Webots to the controller subprocess, so we also accept a sentinel
    # file at projects/policies/research/inference/current_command.txt with three lines:
    # vx, vy, wz.
    vx = float(os.environ.get("OMNIQUAD_VX", "0.5"))
    vy = float(os.environ.get("OMNIQUAD_VY", "0.0"))
    wz = float(os.environ.get("OMNIQUAD_WZ", "0.0"))
    try:
        repo_root = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
        cmd_file = repo_root / "projects" / "rl" / "inference" / "current_command.txt"
        if cmd_file.exists():
            lines = cmd_file.read_text().strip().splitlines()
            if len(lines) >= 3:
                vx, vy, wz = float(lines[0]), float(lines[1]), float(lines[2])
    except Exception:
        pass
    vel_command = np.array([vx, vy, wz], dtype=np.float32)
    _say(f"[omniquad_rl_deploy] vel command = ({vx:.2f}, {vy:.2f}, {wz:.2f})\n")
    sim_time = 0.0
    sim_ms = 0

    # Optional body trace at 10 Hz, opt-in via OMNIQUAD_DEPLOY_TRACE env var
    # (or OMNISIM_DEPLOY_TRACE -- the Newton-demo harness sets that one
    # because Webots' batch mode drops OMNIQUAD_-prefixed vars on the floor
    # before they reach the controller subprocess).
    trace = None
    if os.environ.get("OMNIQUAD_DEPLOY_TRACE") or os.environ.get("OMNISIM_DEPLOY_TRACE"):
        try:
            os.makedirs(r"C:\tmp\husky_trace", exist_ok=True)
            trace = open(r"C:\tmp\husky_trace\omniquad_deploy.csv", "w", buffering=1)
            trace.write("t_ms,bx,by,bz,roll,pitch,yaw,vx,vy,vz\n")
            _say("[omniquad_rl_deploy] trace opened\n")
        except Exception as _e:
            _say(f"[omniquad_rl_deploy] trace open failed: {_e}\n")
            trace = None
    else:
        _say("[omniquad_rl_deploy] OMNIQUAD_DEPLOY_TRACE unset, trace disabled\n")
    next_trace_ms = 0

    # Optional rendered-frame capture: OMNISIM_DEPLOY_SNAP=1 saves the ACTUAL
    # 3D view (what the user sees on screen) to JPGs via Supervisor.exportImage,
    # so behaviour can be verified against the RENDER, not just the chassis
    # trace (which is blind to the legs flying apart / a render desync).
    _snap = bool(os.environ.get("OMNISIM_DEPLOY_SNAP"))
    _snap_n = 0
    _snap_next_ms = 0
    _snap_period = int(os.environ.get("OMNISIM_DEPLOY_SNAP_MS", "1000") or "1000")
    if _snap:
        try:
            os.makedirs(r"C:\tmp\omniquad_snap", exist_ok=True)
            _say("[omniquad_rl_deploy] frame snapshots ON -> C:\\tmp\\omniquad_snap\n")
        except Exception:
            _snap = False

    # Per-link world-position logging (OMNISIM_DEPLOY_LINKLOG=1): walk the
    # robot's actual Solid tree and record every link's RENDERED scene-tree
    # position (Supervisor.getPosition), so we can verify the WHOLE robot
    # (every leg segment), not just the chassis -- and confirm what's rendered
    # matches what we report. Detects: real translation (all links move
    # together), gait execution (feet cycle), tornado (links fly apart =
    # inter-link distances explode / NaN).
    _linklog = None
    _link_nodes = []
    if os.environ.get("OMNISIM_DEPLOY_LINKLOG"):
        def _collect(node, acc, seen, depth=0):
            if node is None or depth > 12:
                return
            try:
                nid = node.getId()
            except Exception:
                nid = id(node)
            if nid in seen:
                return
            seen.add(nid)
            try:
                if node.getBaseTypeName() == "Solid":
                    acc.append(node)
            except Exception:
                pass
            for fld in ("children",):
                try:
                    f = node.getField(fld)
                    if f is not None:
                        for i in range(f.getCount()):
                            _collect(f.getMFNode(i), acc, seen, depth + 1)
                except Exception:
                    pass
            try:
                ep = node.getField("endPoint")
                if ep is not None:
                    _collect(ep.getSFNode(), acc, seen, depth + 1)
            except Exception:
                pass
        try:
            _seen = set()
            _collect(self_node, _link_nodes, _seen)
            os.makedirs(r"C:\tmp", exist_ok=True)
            _linklog = open(r"C:\tmp\omniquad_links.csv", "w", buffering=1)
            hdr = "t_ms," + ",".join(
                f"{(n.getDef() or 'L%d' % i)}_{ax}" for i, n in enumerate(_link_nodes) for ax in "xyz")
            _linklog.write(hdr + "\n")
            _say(f"[omniquad_rl_deploy] LINKLOG on: {len(_link_nodes)} solids tracked\n")
        except Exception as _le:
            _say(f"[omniquad_rl_deploy] linklog setup failed: {_le}\n")
            _linklog = None
    _linklog_next_ms = 0

    # DIAGNOSTIC: every 5s log getPosition() vs the translation field that the
    # render reads. If they DIVERGE, Newton is updating only its internal pose
    # while the rendered Solid stays at spawn (=> user sees stationary robot
    # while per-link getPosition reports motion).
    _pos_field = None
    try:
        _pos_field = self_node.getField("translation")
    except Exception:
        _pos_field = None
    _pos_diag_next = 0

    _say("[omniquad_rl_deploy] entering main loop\n")
    _dbg_step = 0
    prev_body_pos = None
    prev_body_R = None
    # Heading-lock controller: trained policies track a YAW-RATE command well,
    # but with cmd=0 they still slow-drift over minutes. Close a proportional
    # heading loop in software: each step measure heading error from the
    # initial yaw, set the policy's wz command to -kp*error. The policy treats
    # this as just another yaw-rate command (well within its training range
    # [-0.3, +0.3]) and steers back to straight. Set OMNIQUAD_HEADING_LOCK=0 to
    # disable and let the user-supplied wz command pass through unchanged.
    _heading_lock = (os.environ.get("OMNIQUAD_HEADING_LOCK", "1").strip() != "0")
    _heading_kp = float(os.environ.get("OMNIQUAD_HEADING_KP", "1.0"))
    _heading_clip = float(os.environ.get("OMNIQUAD_HEADING_CLIP", "0.25"))
    _heading_ref = None  # initial yaw, captured on first loop tick
    _base_wz = float(vel_command[2])  # user-supplied wz; lock only overrides if base is 0
    while robot.step(step_ms) != -1:
        sim_time += step_dt
        sim_ms += step_ms
        try:
            pos = self_node.getPosition() or [0, 0, 0]
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel = self_node.getVelocity() or [0]*6
        except Exception:
            pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        if _snap and sim_ms >= _snap_next_ms:
            try:
                robot.exportImage(rf"C:\tmp\omniquad_snap\f{_snap_n:04d}.jpg", 80)
                _snap_n += 1
                _snap_next_ms = sim_ms + _snap_period
            except Exception as _se:
                _say(f"[omniquad_rl_deploy] exportImage failed: {_se}\n")
                _snap = False
        if _pos_field is not None and sim_ms >= _pos_diag_next:
            try:
                gp = self_node.getPosition() or [0,0,0]
                fp = _pos_field.getSFVec3f() or [0,0,0]
                _say(f"[posdiag] t={sim_ms} getPosition=({gp[0]:+.2f},{gp[1]:+.2f},{gp[2]:+.2f})  field translation=({fp[0]:+.2f},{fp[1]:+.2f},{fp[2]:+.2f})  diff_x={gp[0]-fp[0]:+.2f}\n")
                _pos_diag_next = sim_ms + 5000
            except Exception:
                pass
        if _linklog is not None and sim_ms >= _linklog_next_ms:
            try:
                vals = []
                for n in _link_nodes:
                    p = n.getPosition() or [float('nan')] * 3
                    vals += [f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}"]
                _linklog.write(f"{sim_ms}," + ",".join(vals) + "\n")
                _linklog_next_ms = sim_ms + 200
            except Exception:
                pass
        # Demo loop: auto-reset to the standing pose on a fall so the walk
        # replays continuously (opt-in via OMNISIM_DEPLOY_LOOP). Off by
        # default so eval/normal deploy see a single uninterrupted run.
        if _DEPLOY_LOOP:
            _roll = math.atan2(ori[7], ori[8])
            _pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
            if abs(_roll) > 1.0 or abs(_pitch) > 1.0 or pos[2] < 0.30:
                try:
                    _tf = self_node.getField("translation"); _rf = self_node.getField("rotation")
                    if _tf: _tf.setSFVec3f([0.0, 0.0, 0.70])
                    if _rf: _rf.setSFRotation([0.0, 0.0, 1.0, 0.0])
                    self_node.resetPhysics()
                except Exception:
                    pass
                for _i, _m in enumerate(motors):
                    _m.setPosition(float(NOMINAL_POSE[_i] + PITCH_TRIM_VEC[_i]))
                sim_time = 0.0; prev_joint_pos = NOMINAL_POSE.copy()
                # CRUCIAL: clear the finite-diff velocity history so the
                # teleport back to standing doesn't produce a huge bogus
                # velocity spike next step (which makes the policy flail).
                prev_body_pos = None; prev_body_R = None
                _say("[omniquad_rl_deploy] fell -> reset (demo loop)\n")
                continue
        if trace is not None and sim_ms >= next_trace_ms:
            roll = math.atan2(ori[7], ori[8])
            pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
            yaw = math.atan2(ori[3], ori[0])
            trace.write(f"{sim_ms},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                        f"{roll:.4f},{pitch:.4f},{yaw:.4f},"
                        f"{vel[0]:.4f},{vel[1]:.4f},{vel[2]:.4f}\n")
            next_trace_ms = sim_ms + 100

        # NOTE: self_node.getVelocity() returns ZEROS under the Newton
        # backend (it updates poses but not Webots node velocities). The
        # policy is a velocity-feedback balancer, so a zero velocity obs
        # makes it diverge. getVelocity() now returns the real Newton
        # velocity [vx,vy,vz, wx,wy,wz] WORLD-frame (backend fix). Match the
        # trainer's frames: v_lin in WORLD, v_ang in BODY-LOCAL (MuJoCo
        # free-joint qvel convention) -> rotate world angular into body.
        _R = np.array(ori, dtype=np.float64).reshape(3, 3)
        v_lin = np.array(vel[:3], dtype=np.float32)
        v_ang = (_R.T @ np.array(vel[3:6], dtype=np.float64)).astype(np.float32)
        # finite-diff fallback if getVelocity() ever reads ~0 (e.g. ODE world)
        if abs(v_lin).max() < 1e-6 and abs(v_ang).max() < 1e-6 and prev_body_pos is not None:
            _p = np.array(pos, dtype=np.float64)
            v_lin = ((_p - prev_body_pos) / max(step_dt, 1e-4)).astype(np.float32)
            _dR = _R @ prev_body_R.T
            _w = np.array([_dR[2, 1] - _dR[1, 2], _dR[0, 2] - _dR[2, 0],
                           _dR[1, 0] - _dR[0, 1]]) / (2.0 * max(step_dt, 1e-4))
            v_ang = (_R.T @ _w).astype(np.float32)
        prev_body_pos = np.array(pos, dtype=np.float64)
        prev_body_R = _R
        proj_g = projected_gravity(ori)
        q = np.zeros(ACT_DIM, dtype=np.float32)
        for i, s in enumerate(sensors):
            if s is not None:
                try:
                    q[i] = float(s.getValue())
                except Exception:
                    q[i] = 0.0
        qd = (q - prev_joint_pos) / max(step_dt, 1e-4)
        prev_joint_pos = q

        # Standing-diagnostic: for the first ~1.5 s, log how well the joints
        # actually track NOMINAL_POSE (the commanded setpoint in gait-only
        # mode). Big tracking error => the Newton actuator can't hold the
        # leg against gravity (authority problem). Small error while the
        # body still rolls => the legs hold but the stance is unstable /
        # the feet aren't carrying load (contact problem). Opt-in via
        # OMNISIM_DEPLOY_JOINTLOG so it never spams a normal run.
        if os.environ.get("OMNISIM_DEPLOY_JOINTLOG") and sim_ms <= 1500 and (sim_ms <= step_ms or sim_ms % 96 < step_ms):
            jerr = q - NOMINAL_POSE
            _say(f"[jointlog] t={sim_ms} bz={pos[2]:.3f} "
                 f"roll={math.atan2(ori[7], ori[8]):+.3f} "
                 f"max|q-nom|={float(np.max(np.abs(jerr))):.3f} "
                 f"hipy_err=[{jerr[1]:+.2f},{jerr[4]:+.2f},{jerr[7]:+.2f},{jerr[10]:+.2f}] "
                 f"knee_err=[{jerr[2]:+.2f},{jerr[5]:+.2f},{jerr[8]:+.2f},{jerr[11]:+.2f}]\n")

        # Heading-lock: override commanded wz with a proportional yaw correction
        # so the robot holds the initial heading instead of slow-drifting. Only
        # engages when the user-supplied wz is 0 (so a deliberate turn command
        # still flows through). cur_yaw is in world frame from getOrientation
        # (yaw = atan2(R[1][0], R[0][0]) = atan2(ori[3], ori[0])).
        if _heading_lock and abs(_base_wz) < 1e-6:
            cur_yaw = math.atan2(float(ori[3]), float(ori[0]))
            if _heading_ref is None:
                _heading_ref = cur_yaw
            dyaw = cur_yaw - _heading_ref
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi  # wrap [-pi,pi]
            wz_cmd = max(-_heading_clip, min(_heading_clip, -_heading_kp * dyaw))
            vel_command[2] = wz_cmd

        # Clock phase-locked to CPG frequency when CPG is active so the
        # policy sees the same clock signal it was trained against.
        _clock_freq = cpg_freq if cpg_freq > 0.0 else 0.5
        clock = (sim_time * _clock_freq) % 1.0
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = v_lin
        obs[3:6] = v_ang
        obs[6:9] = proj_g
        obs[9:21] = q
        obs[21:33] = qd
        obs[33:45] = last_action
        obs[45:48] = vel_command
        obs[48] = clock
        # obs[49] = wrapped heading deviation from the first observed yaw.
        # Only filled for 50-dim policies (v12_200k stays untouched at 49).
        if OBS_DIM >= 50:
            cur_yaw_obs = math.atan2(float(ori[3]), float(ori[0]))
            if _heading_ref is None:
                _heading_ref = cur_yaw_obs
            dyaw_obs = cur_yaw_obs - _heading_ref
            dyaw_obs = (dyaw_obs + math.pi) % (2 * math.pi) - math.pi
            obs[49] = float(dyaw_obs)
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0).astype(np.float32)

        action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        last_action = action.copy()
        if gait_only:
            action = np.zeros(ACT_DIM, dtype=np.float32)
        cpg = cpg_base_offsets(sim_time, cpg_freq, cpg_hip_amp, cpg_knee_amp)
        target = NOMINAL_POSE + PITCH_TRIM_VEC + cpg + action_scale_arg * action
        target = np.clip(target, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        for i, m in enumerate(motors):
            m.setPosition(float(target[i]))

        if _DEPLOY_DBG and _dbg_step < 600 and _dbg_step % 10 == 0:
            _roll = math.atan2(ori[7], ori[8]); _pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
            _yaw = math.atan2(ori[3], ori[0])
            try:
                with open(r"C:\tmp\omniquad_dbg_steps.txt", "a") as _df:
                    _df.write(f"s={_dbg_step} bz={pos[2]:.3f} yaw={_yaw:+.2f} wz={float(v_ang[2]):+.2f} rp=({_roll:+.2f},{_pitch:+.2f}) "
                              f"vlin={float(np.linalg.norm(v_lin)):.2f} vang={float(np.linalg.norm(v_ang)):.2f} "
                              f"qdmax={float(np.max(np.abs(qd))):.2f} amax={float(np.max(np.abs(action))):.2f} "
                              f"tgtmax={float(np.max(np.abs(target-NOMINAL_POSE))):.2f} "
                              f"qerrmax={float(np.max(np.abs(q-NOMINAL_POSE))):.2f}\n")
            except Exception:
                pass
        _dbg_step += 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
