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

"""Spot residual-RL deploy controller.

Runs the model walker (gait + IK + balance) and applies an ONNX
residual policy on top: per-leg foot-position offsets (12 dims, ±1
each, scaled to ±3 cm).

Reads the ONNX policy from SPOT_POLICY_ONNX (default:
projects/policies/research/inference/policies/spot_residual_main/policy.onnx).

Writes a body-pose trace CSV (same format as spot_rl_deploy and
spot_model_walk) when SPOT_DEPLOY_TRACE is set, so verify_straight_walk
.py works against this controller too.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))  # lowest priority: don't shadow runtime `import omnisim`

from projects.policies.control.spot_kinematics import inverse_kinematics  # noqa: E402
from projects.policies.control.spot_gait import (  # noqa: E402
    GaitParams, foot_targets, neutral_foot_positions,
)
from projects.policies.control.spot_balance import (  # noqa: E402
    BalanceParams, balance_offsets,
)
from projects.policies.control.spot_recovery import (  # noqa: E402
    RecoveryFSM, RecoveryAction, righting_joint_targets,
)
from projects.policies.control.spot_motor_safety import (  # noqa: E402
    apply_realistic_limits, set_recovery_torque, RateLimitedMotorBank,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
URDF_TO_IK = {"front_left": "FL", "front_right": "FR",
              "rear_left": "RL", "rear_right": "RR"}
JOINT_ORDER = []
for leg in URDF_LEGS:
    for joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((leg, joint))

OBS_DIM = 18
ACT_DIM = 12
RES_SCALE = 0.03


def _env_float(key: str, default: float) -> float:
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _wrap_pi(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def find_policy_path() -> Path:
    p = os.environ.get("SPOT_POLICY_ONNX") or os.environ.get(
        "OMNISIM_POLICY_ONNX")
    if p:
        return Path(p)
    return (_REPO / "projects" / "rl" / "inference" / "policies"
            / "spot_residual_main" / "policy.onnx")


def main() -> int:
    side_log_path = os.environ.get("SPOT_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg):
        try: sys.stderr.write(msg); sys.stderr.flush()
        except Exception: pass
        if side_log is not None:
            try: side_log.write(msg); side_log.flush()
            except Exception: pass

    policy_path = find_policy_path()
    say(f"[spot_residual_deploy] policy: {policy_path}\n")
    use_policy = policy_path.exists()
    sess = None
    if use_policy:
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(str(policy_path),
                                        providers=["CPUExecutionProvider"])
            say("[spot_residual_deploy] ONNX loaded\n")
        except Exception as e:
            say(f"[spot_residual_deploy] ONNX load failed ({e}); zero residual\n")
            sess = None
    else:
        say("[spot_residual_deploy] no ONNX found; running model walker with zero residual\n")

    # Gait geometry env-overridable to match the training agent: under the
    # post-W1 honest leg geometry the default 0.05 m swing arc grazes the
    # ground and drags the body backward; the Newton recipe trains and
    # deploys with SPOT_GAIT_STEP_HEIGHT=0.09 (see spot-residual-rl.md).
    _gd = GaitParams()
    gait = GaitParams(
        step_height=_env_float("SPOT_GAIT_STEP_HEIGHT", _gd.step_height),
        ground_z=_env_float("SPOT_GAIT_GROUND_Z", _gd.ground_z),
    )
    # Balance-PD gains are env-tunable: the defaults were tuned for the
    # 2026-05-24 ODE physics and under-reject roll on the current engine
    # (a persistent ~0.1 rad lean slowly diverged after ~29 m). Slightly
    # stiffer roll + more headroom keeps the body level over a long walk.
    balance = BalanceParams(
        kp_pitch=_env_float("SPOT_BAL_KP_PITCH", 0.10),
        kd_pitch=_env_float("SPOT_BAL_KD_PITCH", 0.02),
        kp_roll=_env_float("SPOT_BAL_KP_ROLL", 0.10),
        kd_roll=_env_float("SPOT_BAL_KD_ROLL", 0.02),
        max_dz=_env_float("SPOT_BAL_MAX_DZ", 0.05),
    )
    vx = _env_float("SPOT_VX", 0.5)
    vy = _env_float("SPOT_VY", 0.0)
    wz = _env_float("SPOT_WZ", 0.0)
    say(f"[spot_residual_deploy] command vx={vx} vy={vy} wz={wz}\n")

    # Deterministic heading + lateral hold (deploy-side, model-layer).
    # The analytic gait + ONNX residual were tuned for the 2026-05-24 ODE
    # physics; on the current engine the open-loop gait curves (the
    # model-only walker drifts too), and the 18-dim residual can't pull a
    # multi-metre heading error back through +/-3 cm foot offsets. So we
    # close a PD loop on heading (yaw) and lateral position and feed the
    # correction as a steering command to `foot_targets` (wz = yaw rate,
    # vy = sidestep) -- the analytic kinematic layer, NOT the policy. We
    # deliberately keep the POLICY's command observation at the user's
    # [vx, vy, wz] so the residual never sees a non-zero wz: a previous
    # attempt that fed the corrective wz into the policy made it pivot in
    # place (it was trained to read wz!=0 as "turn"), see
    # docs/developer/spot-residual-rl.md. Tunable; default ON.
    HOLD = (os.environ.get("SPOT_HEADING_HOLD", "1").strip() != "0")
    HOLD_KP_YAW = _env_float("SPOT_HOLD_KP_YAW", 2.0)
    HOLD_KD_YAW = _env_float("SPOT_HOLD_KD_YAW", 0.4)
    # Direct lateral sidestep (vy) for fast lateral disturbance rejection,
    # working alongside the gentler steer-to-centreline term (KP_LAT2YAW).
    HOLD_KP_LAT = _env_float("SPOT_HOLD_KP_LAT", 0.6)
    # Lateral-position -> heading coupling (steer-to-centreline). Without
    # it, the heading-PD only HOLDS the initial heading; since the gait
    # has an intrinsic curve on the current physics, the heading-PD
    # saturates at wz_max leaving a steady ~10 deg offset that slowly
    # accumulates lateral drift. This term steers the heading back toward
    # y=0 (like a car returning to lane centre), driving lateral->0.
    HOLD_KP_LAT2YAW = _env_float("SPOT_HOLD_KP_LAT2YAW", 0.18)
    HOLD_WZ_MAX = _env_float("SPOT_HOLD_WZ_MAX", 0.35)
    HOLD_VY_MAX = _env_float("SPOT_HOLD_VY_MAX", 0.12)
    say(f"[spot_residual_deploy] heading_hold={'on' if HOLD else 'off'} "
        f"(kp_yaw={HOLD_KP_YAW} kd_yaw={HOLD_KD_YAW} kp_lat={HOLD_KP_LAT})\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = []
    sensors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            say(f"[spot_residual_deploy] missing motor {name}\n")
            return 1
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(_env_float("SPOT_MOTOR_KP", 20.0), 0.0,
                                _env_float("SPOT_MOTOR_KD", 0.3))
        except Exception:
            pass
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None: s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    # Realistic actuator constraints. The URDF tags each joint with
    # effort=80 Nm and velocity=20 rad/s. Without these caps the
    # position-PD can request arbitrary instant torques, which during
    # recovery transitions produced jumps the real hardware couldn't
    # make. setAvailableTorque caps the peak motor force; setVelocity
    # caps how fast the motor slews toward its position target.
    apply_realistic_limits(motors,
                           max_torque_nm=_env_float("SPOT_MAX_TORQUE_NM", 80.0),
                           max_vel_rad_s=_env_float("SPOT_MAX_VEL_RAD_S", 20.0))
    # Rate-limited target wrapper. Even with motor-side velocity caps,
    # we additionally rate-limit the COMMANDED position so a pose
    # change doesn't request "infinitely far away" -- the controller
    # walks the target there over many ticks at a realistic rate.
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("SPOT_TARGET_RATE_RAD_S", 6.0))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    # Self-righting recovery (leg-only, no supervisor teleport). When
    # the robot tips, the controller runs a scripted tuck → extend →
    # stand cycle that uses the legs themselves to push the body off
    # the ground and back upright. See projects/policies/control/spot_recovery
    # .py for the maneuver and pose tables.
    recovery_enabled = (os.environ.get("SPOT_RECOVERY", "1").strip() != "0")
    rec = RecoveryFSM(
        max_recovery_s=_env_float("SPOT_RECOVERY_MAX_S", 6.0),
    )
    say(f"[spot_residual_deploy] recovery={'on' if recovery_enabled else 'off'} "
        f"(model-based, orientation-aware)\n")

    # Precompute the standing joint targets from the gait's neutral
    # foot positions; used for initial stand-pose command.
    _neutrals = neutral_foot_positions(gait)
    _stand_q = [0.0] * len(motors)
    for i, (leg, joint) in enumerate(JOINT_ORDER):
        if joint != "hip_x":
            continue
        ik_leg = URDF_TO_IK[leg]
        _q = inverse_kinematics(ik_leg, _neutrals[ik_leg])
        if _q is None:
            continue
        _stand_q[i + 0] = _q.hip_x
        _stand_q[i + 1] = _q.hip_y
        _stand_q[i + 2] = _q.knee

    def _command_pose(q_vec):
        motor_bank.set_pose(q_vec)

    trace_file = None
    if os.environ.get("SPOT_DEPLOY_TRACE") or os.environ.get(
            "OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\spot_deploy.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[spot_residual_deploy] trace -> {trace_path}\n")

    # Initial stand-pose command.
    _command_pose(_stand_q)

    settle = max(1, int(0.5 / step_dt))
    for _ in range(settle):
        if robot.step(step_ms) == -1:
            return 0

    sim_t = 0.0
    sim_ms = 0
    last_bx = 0.0
    last_t_ms = 0
    prev_roll = 0.0
    prev_pitch = 0.0
    yaw_ref = None
    last_action = np.zeros(ACT_DIM, dtype=np.float32)
    vel_cmd = np.array([vx, vy, wz], dtype=np.float32)

    # Perturbation injection (deploy-time demo). Same mechanism as
    # spot_residual_agent: every PERTURB_INTERVAL_S past PERTURB_START_S,
    # add a random horizontal velocity impulse to the chassis. Default
    # DV_MAX=0 disables; SPOT_PERTURB_DV_MAX=0.4 reproduces what the
    # policy was trained to handle.
    #
    # If SPOT_PERTURB_THROW_CUBE=1 (default 1 when perturbation enabled),
    # also spawns a visible dynamic cube at the perturb moment, flying
    # at the robot from the same direction the impulse came from. The
    # cube is cosmetic — the actual force on Spot is the impulse — but
    # it sells the visual: "something was thrown at the robot."
    PERTURB_DV_MIN = _env_float("SPOT_PERTURB_DV_MIN", 0.0)
    PERTURB_DV_MAX = _env_float("SPOT_PERTURB_DV_MAX", 0.0)
    PERTURB_INTERVAL_S = _env_float("SPOT_PERTURB_INTERVAL_S", 3.0)
    PERTURB_START_S = _env_float("SPOT_PERTURB_START_S", 2.0)
    PERTURB_ENABLED = (PERTURB_DV_MAX > 0.0)
    PERTURB_THROW_CUBE = (os.environ.get("SPOT_PERTURB_THROW_CUBE", "1").strip() != "0")
    PERTURB_CUBE_DISTANCE = _env_float("SPOT_PERTURB_CUBE_DISTANCE", 1.5)  # m
    PERTURB_CUBE_SPEED = _env_float("SPOT_PERTURB_CUBE_SPEED", 5.0)        # m/s
    PERTURB_CUBE_SIZE = _env_float("SPOT_PERTURB_CUBE_SIZE", 0.12)         # m
    PERTURB_CUBE_MASS = _env_float("SPOT_PERTURB_CUBE_MASS", 1.0)          # kg
    # Auto-remove cubes this many seconds after spawn. Newton's contact
    # solver under fast cube impacts occasionally leaves a cube
    # interpenetrating Spot's chassis; auto-removal cleans up before
    # the stuck cube interferes with the next perturbation cycle.
    # 1.5s is well past collision time (~50ms) and well before the next
    # perturbation fires.
    PERTURB_CUBE_LIFETIME_S = _env_float("SPOT_PERTURB_CUBE_LIFETIME_S", 1.5)
    next_perturb_t = PERTURB_START_S
    _root_children = None
    if PERTURB_ENABLED:
        say(f"[spot_residual_deploy] perturbations: dv in "
            f"[{PERTURB_DV_MIN:.2f}, {PERTURB_DV_MAX:.2f}] m/s "
            f"every {PERTURB_INTERVAL_S}s (first at {PERTURB_START_S}s); "
            f"throw_cube={PERTURB_THROW_CUBE}\n")
        if PERTURB_THROW_CUBE:
            try:
                _root_children = robot.getRoot().getField("children")
            except Exception as e:
                say(f"[spot_residual_deploy] cannot access root.children: {e}; "
                    f"cube spawning disabled\n")
                _root_children = None
    _cube_id = 0
    # List of (cube_node, spawn_time) for lifetime tracking.
    _live_cubes: list = []

    # ── Joint state diagnostic ──
    # SPOT_DEBUG_JOINTS=1 prints a once-per-second summary of joint
    # angle range + max joint velocity since the last print. Used to
    # diagnose whether Newton is enforcing the URDF velocity_limit
    # (20 rad/s for Spot) under external impacts. With armature=0 in
    # WbNewtonBackend, the joint inertia is just the limb inertia
    # (small), so external forces can whip joints faster than the
    # motor's own velocity limit would allow.
    DEBUG_JOINTS = (os.environ.get("SPOT_DEBUG_JOINTS", "0").strip() != "0")
    JOINT_VEL_LIMIT = _env_float("SPOT_JOINT_VEL_LIMIT", 20.0)  # rad/s
    _prev_q: list = [None] * len(motors)
    _joint_vel_max = 0.0
    _joint_q_max = -1e9
    _joint_q_min = 1e9
    _last_debug_t = 0.0
    # SPOT_DEBUG_JOINTS_CSV=<path> emits one row per tick with all 12
    # joint angles + 12 velocities. Used for offline anomaly detection.
    _joints_csv = None
    _joints_csv_path = os.environ.get("SPOT_DEBUG_JOINTS_CSV", "").strip()
    if _joints_csv_path:
        try:
            _joints_csv = open(_joints_csv_path, "w", buffering=1)
            _joints_csv.write("t_ms," + ",".join(
                f"q_{leg}_{j}" for leg, j in JOINT_ORDER) + "," + ",".join(
                f"qd_{leg}_{j}" for leg, j in JOINT_ORDER) + "\n")
            say(f"[spot_residual_deploy] joint CSV -> {_joints_csv_path}\n")
        except Exception as e:
            say(f"[spot_residual_deploy] joint CSV open failed: {e}\n")
            _joints_csv = None
    if DEBUG_JOINTS:
        say(f"[spot_residual_deploy] joint diag ON "
            f"(vel limit={JOINT_VEL_LIMIT:.1f} rad/s; "
            f"reports max-over-last-1s every tick)\n")

    while robot.step(step_ms) != -1:
        sim_t += step_dt

        # Joint state sampling (cheap; happens every tick, prints once
        # per second). Reads position sensors → finite-difference vel.
        if DEBUG_JOINTS or _joints_csv is not None:
            cur_q = []
            for i, s in enumerate(sensors):
                if s is None:
                    cur_q.append(0.0)
                    continue
                try:
                    cur_q.append(float(s.getValue()))
                except Exception:
                    cur_q.append(0.0)
            cur_qd = [0.0] * len(cur_q)
            for i, q in enumerate(cur_q):
                pq = _prev_q[i]
                if pq is not None:
                    qd = (q - pq) / step_dt
                    cur_qd[i] = qd
                    v = abs(qd)
                    if v > _joint_vel_max:
                        _joint_vel_max = v
                if q > _joint_q_max: _joint_q_max = q
                if q < _joint_q_min: _joint_q_min = q
                _prev_q[i] = q
            if _joints_csv is not None:
                _joints_csv.write(
                    f"{sim_ms}," + ",".join(f"{q:.5f}" for q in cur_q) + "," +
                    ",".join(f"{v:.4f}" for v in cur_qd) + "\n")
            if sim_t - _last_debug_t >= 1.0:
                exceeded = "OK"
                if _joint_vel_max > JOINT_VEL_LIMIT:
                    exceeded = f"VIOLATION ({_joint_vel_max/JOINT_VEL_LIMIT:.1f}x limit)"
                say(f"[spot_residual_deploy] t={sim_t:5.1f}s  "
                    f"max_joint_vel={_joint_vel_max:6.2f} rad/s  "
                    f"q_range=[{_joint_q_min:+5.2f}, {_joint_q_max:+5.2f}]  "
                    f"{exceeded}\n")
                _last_debug_t = sim_t
                _joint_vel_max = 0.0
                _joint_q_max = -1e9
                _joint_q_min = 1e9

        # Remove cubes that have outlived PERTURB_CUBE_LIFETIME_S.
        if _live_cubes:
            _live_cubes_new = []
            for cn, t_spawn in _live_cubes:
                if sim_t - t_spawn > PERTURB_CUBE_LIFETIME_S:
                    try:
                        cn.remove()
                    except Exception:
                        pass
                else:
                    _live_cubes_new.append((cn, t_spawn))
            _live_cubes = _live_cubes_new
        sim_ms += step_ms

        if PERTURB_ENABLED and self_node is not None and sim_t >= next_perturb_t:
            try:
                cur_vel = self_node.getVelocity() or [0]*6
                cur_pos = self_node.getPosition() or [0, 0, 0]
                dv = PERTURB_DV_MIN + (PERTURB_DV_MAX - PERTURB_DV_MIN) * np.random.random()
                angle = 2.0 * math.pi * np.random.random()
                dvx = dv * math.cos(angle)
                dvy = dv * math.sin(angle)
                # Apply the artificial impulse ONLY when cubes are
                # disabled. When cubes are spawning, the cube collision
                # IS the disturbance — applying both delivers ~0.5
                # m/s of Δv per perturbation, well above the policy's
                # training distribution.
                if not PERTURB_THROW_CUBE:
                    new_vel = [
                        float(cur_vel[0]) + dvx,
                        float(cur_vel[1]) + dvy,
                        float(cur_vel[2]),
                        float(cur_vel[3]),
                        float(cur_vel[4]),
                        float(cur_vel[5]),
                    ]
                    if hasattr(self_node, "setVelocity"):
                        self_node.setVelocity(new_vel)
                say(f"[spot_residual_deploy] perturb t={sim_t:.2f}s "
                    f"dv=({dvx:+.2f}, {dvy:+.2f}) m/s "
                    f"(via {'cube' if PERTURB_THROW_CUBE else 'impulse'})\n")
                # Spawn a visible cube at the perturb direction, flying
                # toward Spot's center. Predates the impulse by a few cm
                # of travel (spawned 1.5m out, moving at 6 m/s → ~250 ms
                # of flight before impact, during which the impulse has
                # already nudged the body). The cube itself does NOT
                # physically push the robot (Newton's articulation builder
                # only sees scene at init), but it visually matches the
                # impulse direction.
                if PERTURB_THROW_CUBE and _root_children is not None:
                    _cube_id += 1
                    # Aim at where Spot will be when the cube arrives,
                    # not where Spot is now. flight_time = distance/speed.
                    # Without lead correction, the cube arrives 12 cm
                    # behind the chassis at vx=0.5 m/s — misses cleanly.
                    flight_time = PERTURB_CUBE_DISTANCE / max(PERTURB_CUBE_SPEED, 0.1)
                    lead_x = float(cur_vel[0]) * flight_time
                    lead_y = float(cur_vel[1]) * flight_time
                    target_x = float(cur_pos[0]) + lead_x
                    target_y = float(cur_pos[1]) + lead_y
                    target_z = float(cur_pos[2])  # chassis level
                    # Spawn opposite the impulse direction, raised by
                    # the gravity drop over flight_time so the cube's
                    # parabolic arc lands at target_z. Without this the
                    # cube falls ~31 cm during flight and hits the legs
                    # or floor.
                    g = 9.81
                    gravity_drop = 0.5 * g * flight_time * flight_time
                    spawn_x = target_x - PERTURB_CUBE_DISTANCE * math.cos(angle)
                    spawn_y = target_y - PERTURB_CUBE_DISTANCE * math.sin(angle)
                    spawn_z = target_z + gravity_drop
                    # Cube velocity: horizontal only, toward target.
                    # Gravity does the vertical work.
                    cvx = PERTURB_CUBE_SPEED * math.cos(angle)
                    cvy = PERTURB_CUBE_SPEED * math.sin(angle)
                    sz = PERTURB_CUBE_SIZE
                    cube_vrml = (
                        f'DEF PUSH_CUBE_{_cube_id} Solid {{ '
                        f'translation {spawn_x:.3f} {spawn_y:.3f} {spawn_z:.3f} '
                        f'children [ Shape {{ '
                        f'appearance PBRAppearance {{ baseColor 0.95 0.30 0.30 '
                        f'roughness 0.7 metalness 0 }} '
                        f'geometry Box {{ size {sz} {sz} {sz} }} '
                        f'}} ] '
                        f'name "push_cube_{_cube_id}" '
                        f'boundingObject Box {{ size {sz} {sz} {sz} }} '
                        f'physics Physics {{ density -1 mass {PERTURB_CUBE_MASS} }} '
                        f'}}'
                    )
                    try:
                        _root_children.importMFNodeFromString(-1, cube_vrml)
                        cube_node = _root_children.getMFNode(-1)
                        if cube_node is not None:
                            if hasattr(cube_node, "setVelocity"):
                                cube_node.setVelocity([cvx, cvy, 0.0, 0.0, 0.0, 0.0])
                            _live_cubes.append((cube_node, sim_t))
                    except Exception as e:
                        say(f"[spot_residual_deploy] cube spawn failed: {e}\n")
            except Exception:
                pass
            next_perturb_t = sim_t + PERTURB_INTERVAL_S

        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0, 0, 0]
                ori = self_node.getOrientation() or [1,0,0,0,1,0,0,0,1]
                vel = self_node.getVelocity() or [0]*6
            except Exception:
                pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        else:
            pos = [0,0,0]; ori = [1,0,0,0,1,0,0,0,1]; vel = [0]*6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
        yaw = math.atan2(ori[3], ori[0])

        # Self-righting recovery: on fall, run a tuck → extend → stand
        # cycle using the legs themselves (no supervisor teleport). The
        # FSM returns one of TUCK / EXTEND / STAND while in recovery;
        # the controller commands that pose and skips gait + IK +
        # policy this tick so the legs hold position instead of
        # flailing or chasing the gait clock.
        if recovery_enabled:
            rec_action, orient = rec.step(
                bz, roll, pitch, float(vel[2]),
                float(ori[2]), float(ori[5]), float(ori[8]),
                step_dt)
            if rec_action == RecoveryAction.RIGHTING:
                if not getattr(rec, "_logged_start", False):
                    say(f"[spot_residual_deploy] fall at t={sim_t:.1f}s "
                        f"(bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f}); "
                        f"orientation={orient.value}; running model righting\n")
                    rec._logged_start = True
                    rec._last_logged_orient = orient
                    set_recovery_torque(motors)
                elif getattr(rec, "_last_logged_orient", None) != orient:
                    say(f"[spot_residual_deploy]   t={sim_t:.1f}s "
                        f"orientation -> {orient.value}\n")
                    rec._last_logged_orient = orient
                _command_pose(righting_joint_targets(orient, JOINT_ORDER))
                continue
            # Just exited recovery; reset gait + heading state so the
            # walking phase starts fresh from the body's current pose.
            if getattr(rec, "_logged_start", False):
                actually_upright = (bz > 0.55 and abs(roll) < 0.40
                                    and abs(pitch) < 0.40)
                if actually_upright:
                    say(f"[spot_residual_deploy] recovered at t={sim_t:.1f}s "
                        f"(bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f}); "
                        f"resuming gait\n")
                    apply_realistic_limits(
                        motors,
                        max_torque_nm=_env_float("SPOT_MAX_TORQUE_NM", 80.0),
                        max_vel_rad_s=_env_float("SPOT_MAX_VEL_RAD_S", 8.0))
                else:
                    say(f"[spot_residual_deploy] recovery timed out at "
                        f"t={sim_t:.1f}s "
                        f"(bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f}); "
                        f"body still down -- holding recovery torque\n")
                rec._logged_start = False
                rec._last_logged_orient = None
                sim_t = 0.0
                yaw_ref = None
                prev_roll = 0.0
                prev_pitch = 0.0
                last_action = np.zeros(ACT_DIM, dtype=np.float32)

        if yaw_ref is None:
            yaw_ref = yaw
        dyaw = _wrap_pi(yaw - yaw_ref)
        dlat = by

        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx_obs = (bx - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            trace_file.write(f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                             f"{roll:.4f},{pitch:.4f},{yaw:.4f},{vx_obs:.4f}\n")
            last_bx = bx
            last_t_ms = sim_ms

        # Build obs + run ONNX residual.
        v_lin = np.array(vel[:3], dtype=np.float32)
        v_ang = np.array(vel[3:6], dtype=np.float32)
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], dtype=np.float32)
        phase = (sim_t / gait.period_s) % 1.0

        obs = np.zeros(OBS_DIM, dtype=np.float32)
        obs[0:3] = v_ang
        obs[3:6] = proj_g
        obs[6] = phase
        obs[7:10] = v_lin
        obs[10:13] = vel_cmd
        obs[13] = dyaw
        obs[14] = dlat
        obs[15:18] = last_action[:3]
        obs = np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = np.clip(obs, -10.0, 10.0).astype(np.float32)

        if sess is not None:
            try:
                action = sess.run(None, {"obs": obs.reshape(1, -1)})[0][0]
                action = np.clip(action, -1.0, 1.0).astype(np.float32)
            except Exception as e:
                say(f"[spot_residual_deploy] inference failed: {e}\n")
                action = np.zeros(ACT_DIM, dtype=np.float32)
        else:
            action = np.zeros(ACT_DIM, dtype=np.float32)
        last_action = action

        # Heading + lateral hold: steer the analytic gait back to the
        # initial heading (yaw_ref) and the path centreline (y=0). The
        # correction is fed ONLY to foot_targets (the gait's kinematic
        # turn), never to the policy's vel_cmd obs above.
        gait_vy = float(vel_cmd[1])
        gait_wz = float(vel_cmd[2])
        if HOLD:
            yaw_rate = float(v_ang[2])
            wz_corr = (-HOLD_KP_YAW * dyaw - HOLD_KD_YAW * yaw_rate
                       - HOLD_KP_LAT2YAW * dlat)
            wz_corr = max(-HOLD_WZ_MAX, min(HOLD_WZ_MAX, wz_corr))
            vy_corr = -HOLD_KP_LAT * dlat
            vy_corr = max(-HOLD_VY_MAX, min(HOLD_VY_MAX, vy_corr))
            gait_wz += wz_corr
            gait_vy += vy_corr

        # Model walker output + residual + balance.
        feet = foot_targets(sim_t, vx=float(vel_cmd[0]),
                            vy=gait_vy, wz=gait_wz, p=gait)
        roll_rate = (roll - prev_roll) / step_dt
        pitch_rate = (pitch - prev_pitch) / step_dt
        prev_roll, prev_pitch = roll, pitch
        bal = balance_offsets(roll, pitch, roll_rate, pitch_rate, balance)

        for i, leg_ik in enumerate(("FL", "FR", "RL", "RR")):
            fx, fy, fz = feet[leg_ik]
            off = action[i*3:(i+1)*3] * RES_SCALE
            fx += float(off[0])
            fy += float(off[1])
            fz += float(off[2]) + bal[leg_ik]
            feet[leg_ik] = (fx, fy, fz)

        # Build the full 12-dim joint target vector and route it
        # through the rate-limited bank so contact transitions are
        # smooth (no impulsive position jumps).
        gait_q = list(motor_bank._last)  # start from last commanded
        gait_q = [v if v is not None else 0.0 for v in gait_q]
        for i, (leg, joint) in enumerate(JOINT_ORDER):
            if joint != "hip_x":
                continue
            ik_leg = URDF_TO_IK[leg]
            q = inverse_kinematics(ik_leg, feet[ik_leg])
            if q is None:
                continue
            gait_q[i + 0] = q.hip_x
            gait_q[i + 1] = q.hip_y
            gait_q[i + 2] = q.knee
        motor_bank.set_pose(gait_q)

    if trace_file is not None: trace_file.close()
    if side_log is not None: side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
