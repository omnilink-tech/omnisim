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

"""Model-only OmniQuad walker — no policy, no RL, no neural network.

Hand-coded controller that drives OmniQuad at a commanded body velocity by:
  1. The gait engine (`omniquad_gait`) emits 4 foot targets in body frame.
  2. The inverse kinematics (`omniquad_kinematics`) converts each foot
     target to 3 joint angles.
  3. Motors are commanded to those joint angles.

This is the model-based controller that residual RL will sit on top of
in subsequent phases. By itself it should already walk OmniQuad straight at
the commanded vx with near-zero drift on flat ground.

Environment overrides:
  OMNIQUAD_VX=0.5   commanded forward velocity, m/s
  OMNIQUAD_VY=0.0   commanded lateral velocity, m/s
  OMNIQUAD_WZ=0.0   commanded yaw rate, rad/s
  OMNIQUAD_GAIT_FREQ_HZ=2.0     gait cycle frequency
  OMNIQUAD_GAIT_DUTY=0.5        stance duty factor
  OMNIQUAD_GAIT_STEP_HEIGHT=0.05  swing arc peak, m
  OMNIQUAD_GAIT_GROUND_Z=-0.56  foot z in body frame when planted, m
  OMNIQUAD_GAIT_LATERAL_Y=0.30  foot's lateral neutral, m from body center
  OMNIQUAD_GAIT_FRONT_X=0.30    front feet's forward neutral, m
  OMNIQUAD_GAIT_REAR_X=-0.28    rear feet's backward neutral, m

  OMNIQUAD_DEPLOY_TRACE=1       write a body-pose trace CSV (matches the RL
                            deploy controller's format so verify_straight
                            _walk.py works against this controller too).
  OMNIQUAD_DEPLOY_LOG=<path>    mirror controller stderr to this file.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

# Locate repo root for cross-imports (parents[5] = repo root; this file
# sits at projects/policies/research/controllers/omniquad_model_walk/omniquad_model_walk.py)
_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))  # lowest priority: don't shadow runtime `import omnisim`

from projects.policies.control.omniquad_kinematics import (  # noqa: E402
    JointAngles, inverse_kinematics,
)
from projects.policies.control.omniquad_gait import (  # noqa: E402
    GaitParams, LEGS as IK_LEGS, foot_targets,
)
from projects.policies.control.omniquad_balance import (  # noqa: E402
    BalanceParams, balance_offsets,
)
from projects.policies.control.omniquad_recovery import (  # noqa: E402
    RecoveryFSM, RecoveryAction, righting_joint_targets,
)
from projects.policies.control.omniquad_motor_safety import (  # noqa: E402
    apply_realistic_limits, set_recovery_torque, RateLimitedMotorBank,
)

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


# URDF joint name layout, in the same canonical order as the agent.
URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
URDF_TO_IK = {"front_left": "FL", "front_right": "FR",
              "rear_left": "RL", "rear_right": "RR"}
JOINT_ORDER = []
for leg in URDF_LEGS:
    for joint in ("hip_x", "hip_y", "knee"):
        JOINT_ORDER.append((leg, joint))


def _env_float(key: str, default: float) -> float:
    v = os.environ.get(key)
    if v is None or v.strip() == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _gait_from_env() -> GaitParams:
    """Build a GaitParams from env vars; defaults inherit from
    GaitParams' own defaults (set to the agent's nominal-pose FK)."""
    defaults = GaitParams()
    freq = _env_float("OMNIQUAD_GAIT_FREQ_HZ", 1.0 / defaults.period_s)
    period = 1.0 / max(freq, 0.1)
    return GaitParams(
        period_s=period,
        duty=_env_float("OMNIQUAD_GAIT_DUTY", defaults.duty),
        step_height=_env_float("OMNIQUAD_GAIT_STEP_HEIGHT", defaults.step_height),
        ground_z=_env_float("OMNIQUAD_GAIT_GROUND_Z", defaults.ground_z),
        neutral_lateral_y=_env_float("OMNIQUAD_GAIT_LATERAL_Y",
                                     defaults.neutral_lateral_y),
        neutral_front_x=_env_float("OMNIQUAD_GAIT_FRONT_X", defaults.neutral_front_x),
        neutral_rear_x=_env_float("OMNIQUAD_GAIT_REAR_X", defaults.neutral_rear_x),
    )


def main() -> int:
    side_log_path = os.environ.get("OMNIQUAD_DEPLOY_LOG") or os.environ.get(
        "OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg: str) -> None:
        try:
            sys.stderr.write(msg)
            sys.stderr.flush()
        except Exception:
            pass
        if side_log is not None:
            try:
                side_log.write(msg)
                side_log.flush()
            except Exception:
                pass

    say("[omniquad_model_walk] starting (model-only, no policy)\n")
    gait = _gait_from_env()
    balance = BalanceParams(
        kp_pitch=_env_float("OMNIQUAD_BAL_KP_PITCH", 0.10),
        kd_pitch=_env_float("OMNIQUAD_BAL_KD_PITCH", 0.02),
        kp_roll=_env_float("OMNIQUAD_BAL_KP_ROLL", 0.10),
        kd_roll=_env_float("OMNIQUAD_BAL_KD_ROLL", 0.02),
        max_dz=_env_float("OMNIQUAD_BAL_MAX_DZ", 0.05),
    )
    balance_enabled = (os.environ.get("OMNIQUAD_BALANCE", "1").strip() != "0")
    # Feedforward hip_y trim — added uniformly to every hip_y joint
    # after IK, both during gait and in STAND_ONLY mode. Used under
    # Newton to counter a structural nose-down moment that tips
    # OmniQuad pitch-forward from the standing pose within ~25 ticks.
    # Default 0.0 = legacy (ODE) behaviour.
    pitch_trim = _env_float("OMNIQUAD_PITCH_TRIM", 0.0)
    vx = _env_float("OMNIQUAD_VX", 0.5)
    vy = _env_float("OMNIQUAD_VY", 0.0)
    # Feedforward lateral trim added to the commanded vy (same idea as
    # OMNIQUAD_PITCH_TRIM): the trot has a small structural lateral thrust on
    # the current physics (the body crabs sideways at a steady rate while
    # the heading lock holds yaw), and a constant counter-vy cancels it
    # where closed-loop sidestepping (vy feedback) destabilizes the gait.
    vy += _env_float("OMNIQUAD_VY_TRIM", 0.0)
    wz = _env_float("OMNIQUAD_WZ", 0.0)
    # Closed-loop heading lock: if OMNIQUAD_WZ command is 0, override wz
    # each tick with -kp_yaw * (yaw - initial_yaw) so the gait steers
    # back to the spawn heading. Plugs directly into the gait engine
    # (the wz argument), which adjusts step lengths per leg to produce
    # the commanded yaw rate. Cleaner than the policy-side heading
    # lock since here we control the steering at the gait level, not
    # by modulating the policy's vel_command observation.
    # Default OFF: a pure P controller on heading either undershoots
    # (under-corrects the natural drift) or oscillates into a fall.
    # Tuning a model-based PD here is out of scope -- this is the kind
    # of closed-loop refinement the residual policy in Phase 5 handles
    # naturally (with proprioception + integrated state). Set =1 to
    # experiment with the model-only heading lock.
    heading_lock = (os.environ.get("OMNIQUAD_HEADING_LOCK", "0").strip() != "0")
    heading_kp = _env_float("OMNIQUAD_HEADING_KP", 0.5)
    heading_clip = _env_float("OMNIQUAD_HEADING_CLIP", 0.10)
    heading_lat2yaw = _env_float("OMNIQUAD_HEADING_LAT2YAW", 0.0)
    say(f"[omniquad_model_walk] gait: period={gait.period_s:.3f}s duty={gait.duty} "
        f"step_h={gait.step_height} ground_z={gait.ground_z} "
        f"neutrals (front_x={gait.neutral_front_x}, rear_x={gait.neutral_rear_x}, "
        f"lat_y={gait.neutral_lateral_y})\n")
    say(f"[omniquad_model_walk] command: vx={vx} vy={vy} wz={wz}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    # Get motors + position sensors.
    motors = []
    sensors = []
    for leg, joint in JOINT_ORDER:
        name = f"{leg}_{joint}_motor"
        m = robot.getDevice(name)
        if m is None:
            say(f"[omniquad_model_walk] ERROR: missing motor {name}\n")
            return 1
        # Match the agent's PD: kp=20, kd=0.3. The neutral foot
        # geometry in `omniquad_gait.GaitParams` is set to the FK of the
        # agent's nominal pose so the resulting joint targets reproduce
        # the standing pose OmniQuad is stable at under these gains. Stiff
        # PD (kp=200) causes physics explosions at contact transitions.
        kp = _env_float("OMNIQUAD_MOTOR_KP", 20.0)
        kd = _env_float("OMNIQUAD_MOTOR_KD", 0.3)
        try:
            if hasattr(m, "setControlPID"):
                m.setControlPID(kp, 0.0, kd)
        except Exception:
            pass
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)

    # Realistic actuator constraints from the OmniQuad URDF (80 Nm /
    # 20 rad/s per joint), plus a rate-limited target wrapper so a
    # pose change can't request an instant joint jump that the real
    # hardware couldn't make.
    apply_realistic_limits(motors,
                           max_torque_nm=_env_float("OMNIQUAD_MAX_TORQUE_NM", 80.0),
                           max_vel_rad_s=_env_float("OMNIQUAD_MAX_VEL_RAD_S", 20.0))
    motor_bank = RateLimitedMotorBank(
        motors, step_dt,
        max_vel_rad_s=_env_float("OMNIQUAD_TARGET_RATE_RAD_S", 6.0))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        pass

    # Model-based self-righting: classify body orientation each tick
    # and apply the geometrically-correct leg push to roll it toward
    # upright (no supervisor teleport, no generic flail). See
    # projects/policies/control/omniquad_recovery.py for the orientation classes
    # and per-orientation maneuvers.
    recovery_enabled = (os.environ.get("OMNIQUAD_RECOVERY", "1").strip() != "0")
    rec = RecoveryFSM(
        max_recovery_s=_env_float("OMNIQUAD_RECOVERY_MAX_S", 6.0),
    )
    say(f"[omniquad_model_walk] recovery={'on' if recovery_enabled else 'off'} "
        f"(model-based, orientation-aware)\n")

    # Trace setup: write the same CSV columns verify_straight_walk.py reads.
    trace_path = None
    trace_file = None
    if os.environ.get("OMNIQUAD_DEPLOY_TRACE") or os.environ.get(
            "OMNISIM_DEPLOY_TRACE"):
        trace_path = Path(r"C:\tmp\husky_trace\omniquad_deploy.csv")
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = open(trace_path, "w", buffering=1)
        trace_file.write("t_ms,bx,by,bz,roll,pitch,yaw,vx\n")
        say(f"[omniquad_model_walk] trace -> {trace_path}\n")

    # Send the initial standing pose (a feasible IK from foot positions
    # at the neutral stance) so the robot doesn't go through any wild
    # init transient.
    def stance_targets() -> list:
        out = [0.0] * len(motors)
        # Neutrals at t=0 with the gait's ground_z; these match the gait
        # FUNCTION's t=0 stance start positions but are simpler to reason
        # about as a pure standing pose.
        from projects.policies.control.omniquad_gait import neutral_foot_positions
        neutrals = neutral_foot_positions(gait)
        for i, (leg, _joint) in enumerate(JOINT_ORDER):
            if _joint != "hip_x":
                continue  # set all three at once below
            ik_leg = URDF_TO_IK[leg]
            q = inverse_kinematics(ik_leg, neutrals[ik_leg])
            if q is None:
                say(f"[omniquad_model_walk] WARN: nominal IK unreachable for {leg}\n")
                continue
            out[i + 0] = q.hip_x
            out[i + 1] = q.hip_y + pitch_trim
            out[i + 2] = q.knee
        return out

    init_targets = stance_targets()
    motor_bank.set_pose(init_targets)

    # Let the body settle to standing before starting the gait clock.
    settle_steps = max(1, int(0.5 / step_dt))
    for _ in range(settle_steps):
        if robot.step(step_ms) == -1:
            return 0

    # ── Main control loop ────────────────────────────────────────────
    t0 = 0.0
    sim_t = 0.0
    sim_ms = 0
    last_bx = 0.0
    last_by = 0.0
    last_t_ms = 0
    # Track previous roll/pitch for finite-difference angular rates
    # consumed by the balance PD.
    prev_roll = 0.0
    prev_pitch = 0.0
    yaw_ref = None  # captured on first tick if heading lock is on
    base_wz = wz  # remember user-supplied wz so heading lock only acts when 0
    say(f"[omniquad_model_walk] gait engaged (balance={'on' if balance_enabled else 'off'}, "
        f"kp_p={balance.kp_pitch} kp_r={balance.kp_roll}; "
        f"heading_lock={'on' if heading_lock else 'off'} "
        f"kp={heading_kp} clip={heading_clip})\n")
    while robot.step(step_ms) != -1:
        sim_t += step_dt
        sim_ms += step_ms

        # Read body pose for the trace.
        vz_body = 0.0
        # Body Z axis expressed in world frame -- columns of the
        # body-to-world rotation matrix R = ori (row-major). Used by
        # the recovery classifier to detect which side the body has
        # fallen onto.
        body_z_world_x = 0.0
        body_z_world_y = 0.0
        body_z_world_z = 1.0
        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0, 0, 0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                vel = self_node.getVelocity() or [0]*6
                bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
                roll = math.atan2(ori[7], ori[8])
                pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
                yaw = math.atan2(ori[3], ori[0])
                vz_body = float(vel[2])
                body_z_world_x = float(ori[2])
                body_z_world_y = float(ori[5])
                body_z_world_z = float(ori[8])
            except Exception:
                bx = by = bz = roll = pitch = yaw = 0.0
        else:
            bx = by = bz = roll = pitch = yaw = 0.0

        # Trace output. vx is a finite difference on body x.
        if trace_file is not None:
            dt_s = (sim_ms - last_t_ms) / 1000.0
            vx_obs = (bx - last_bx) / dt_s if dt_s > 1e-6 else 0.0
            trace_file.write(f"{sim_ms},{bx:.4f},{by:.4f},{bz:.4f},"
                             f"{roll:.4f},{pitch:.4f},{yaw:.4f},{vx_obs:.4f}\n")
            last_bx = bx
            last_by = by
            last_t_ms = sim_ms

        # Model-based self-righting: classify body orientation, then
        # apply the per-orientation leg push (no fixed flail cycle).
        if recovery_enabled:
            rec_action, orient = rec.step(
                bz, roll, pitch, vz_body,
                body_z_world_x, body_z_world_y, body_z_world_z,
                step_dt)
            if rec_action == RecoveryAction.RIGHTING:
                if not getattr(rec, "_logged_start", False):
                    say(f"[omniquad_model_walk] fall at t={sim_t:.1f}s "
                        f"(bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f}); "
                        f"orientation={orient.value}; running model righting\n")
                    rec._logged_start = True
                    rec._last_logged_orient = orient
                    # Drop torque + velocity to recovery profile so
                    # impulsive contact forces during righting can't
                    # launch the chassis. Restored on exit below.
                    set_recovery_torque(motors)
                elif getattr(rec, "_last_logged_orient", None) != orient:
                    say(f"[omniquad_model_walk]   t={sim_t:.1f}s "
                        f"orientation -> {orient.value}\n")
                    rec._last_logged_orient = orient
                pose = righting_joint_targets(orient, JOINT_ORDER)
                motor_bank.set_pose(pose)
                continue
            if getattr(rec, "_logged_start", False):
                # Only restore walking-mode torque if the body is
                # actually upright. A timeout-exit (body still down)
                # would otherwise pop torque back to 80 Nm and cause
                # the next tick's stand command to launch the chassis.
                actually_upright = (bz > 0.55 and abs(roll) < 0.40
                                    and abs(pitch) < 0.40)
                if actually_upright:
                    say(f"[omniquad_model_walk] recovered at t={sim_t:.1f}s "
                        f"(bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f}); "
                        f"resuming gait\n")
                    apply_realistic_limits(
                        motors,
                        max_torque_nm=_env_float("OMNIQUAD_MAX_TORQUE_NM", 80.0),
                        max_vel_rad_s=_env_float("OMNIQUAD_MAX_VEL_RAD_S", 8.0))
                else:
                    say(f"[omniquad_model_walk] recovery timed out at t={sim_t:.1f}s "
                        f"(bz={bz:.2f} roll={roll:+.2f} pitch={pitch:+.2f}); "
                        f"body still down -- holding stand pose at recovery torque\n")
                rec._logged_start = False
                rec._last_logged_orient = None
                sim_t = 0.0
                yaw_ref = None
                prev_roll = 0.0
                prev_pitch = 0.0

        # OMNIQUAD_MODEL_STAND_ONLY=1: just hold the agent's nominal stand
        # pose (0.30, 0.30, -0.60) -- a sanity check that the controller
        # plumbing is sound. Independent of gait/IK.
        if os.environ.get("OMNIQUAD_MODEL_STAND_ONLY") == "1":
            stand_vec = []
            for (_leg, joint) in JOINT_ORDER:
                if joint == "hip_x":
                    stand_vec.append(0.30 if "left" in _leg else -0.30)
                elif joint == "hip_y":
                    stand_vec.append(0.30 + pitch_trim)
                else:
                    stand_vec.append(-0.60)
            motor_bank.set_pose(stand_vec)
            continue

        # Heading lock: override wz if user command is 0. The lateral term
        # (steer-to-centreline, OMNIQUAD_HEADING_LAT2YAW, default 0 = legacy)
        # converts lateral drift from the spawn line into a heading bias,
        # so the walker crabs back instead of accumulating |y| while the
        # pure-yaw lock holds heading.
        if heading_lock and abs(base_wz) < 1e-6 and self_node is not None:
            if yaw_ref is None:
                yaw_ref = yaw
                lat_ref = by
            dyaw = yaw - yaw_ref
            # wrap to [-pi, pi]
            while dyaw > math.pi: dyaw -= 2 * math.pi
            while dyaw < -math.pi: dyaw += 2 * math.pi
            dlat = by - lat_ref
            wz_cmd = -heading_kp * dyaw - heading_lat2yaw * dlat
            if wz_cmd > heading_clip: wz_cmd = heading_clip
            elif wz_cmd < -heading_clip: wz_cmd = -heading_clip
            wz = wz_cmd

        # Compute foot targets in body frame for this tick.
        feet = foot_targets(sim_t, vx=vx, vy=vy, wz=wz, p=gait)

        # Balance PD: adjust foot z per leg to level the body.
        if balance_enabled and self_node is not None:
            roll_rate = (roll - prev_roll) / step_dt if step_dt > 1e-6 else 0.0
            pitch_rate = (pitch - prev_pitch) / step_dt if step_dt > 1e-6 else 0.0
            prev_roll, prev_pitch = roll, pitch
            offsets = balance_offsets(roll, pitch, roll_rate, pitch_rate, balance)
            feet = {leg: (fx, fy, fz + offsets[leg])
                    for leg, (fx, fy, fz) in feet.items()}

        # IK each foot to (hip_x, hip_y, knee) and build a full
        # 12-dim joint target vector; route through the rate-limited
        # bank so transitions are smooth (no impulsive position
        # jumps that would be unrealistic on real hardware).
        gait_q = list(motor_bank._last)
        gait_q = [v if v is not None else 0.0 for v in gait_q]
        for i, (leg, joint) in enumerate(JOINT_ORDER):
            if joint != "hip_x":
                continue
            ik_leg = URDF_TO_IK[leg]
            q = inverse_kinematics(ik_leg, feet[ik_leg])
            if q is None:
                # Foot target outside reach (shouldn't happen with default
                # gait params); hold previous command for this leg.
                continue
            gait_q[i + 0] = q.hip_x
            gait_q[i + 1] = q.hip_y + pitch_trim
            gait_q[i + 2] = q.knee
        motor_bank.set_pose(gait_q)

    if trace_file is not None:
        trace_file.close()
    if side_log is not None:
        side_log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
