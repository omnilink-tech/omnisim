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

"""Spot walking controller — pure-physics gait with instrumentation.

Drives the 12 leg motors of the Boston Dynamics Spot URDF. No
supervisor body-teleport, no rotation lock: every motion has to come
from the leg motors interacting with the floor.

Phases:
  1. DROP   -- legs almost-straight (knee=-0.30) absorb the spawn pose.
  2. CROUCH -- ramp to standing stance (hip_y=0.30, knee=-0.60).
  3. ACTION -- depends on SPOT_PROBE env var:
       "hold"     : hold standing pose forever (used to measure
                    equilibrium body height + foot positions).
       "lift_fr"  : hold standing, then ramp FR knee from -0.60 to
                    -1.30 over 2 s, hold 1 s, ramp back. Used to
                    measure how much knee delta lifts the foot.
       "sweep"    : hold standing, then sweep all four hip_y joints
                    in unison from 0.15 to 0.45 and back, period 4 s.
                    Used to measure forward translation per radian of
                    coordinated hip sweep (stance-only propulsion).
       "walk" (default) : full wave gait.

Per-tick CSV trace at C:\\tmp\\husky_trace\\spot_probe.csv with body
pose, per-joint cmd vs sensor, per-foot world position (via supervisor
node walk), body linear+angular velocity. Designed so all gait tuning
decisions can be made from numbers, not guesses.

Env vars:
  SPOT_PROBE             -- "hold" | "lift_fr" | "sweep" | "walk"
                            (default "walk")
  SPOT_SIM_DURATION_S    -- self-quit deadline for headless iteration
"""

from __future__ import annotations

import math
import os
import sys

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


LEGS = ("front_left", "front_right", "rear_left", "rear_right")

# --- Standing pose (URDF nominal) -------------------------------------
HIP_X_LEFT = 0.30
HIP_X_RIGHT = -0.30
HIP_Y_STAND = 0.30
KNEE_STAND = -0.60

def hip_x_for_leg(leg):
    return HIP_X_LEFT if "left" in leg else HIP_X_RIGHT

def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# --- Joint limits (from URDF) -----------------------------------------
KNEE_LIMIT_LO = -2.7929
KNEE_LIMIT_HI = -0.2548


# --- Phase timing -----------------------------------------------------
EXTEND_HIP_Y = 0.15
EXTEND_KNEE = -0.30
DROP_S = 1.5
CROUCH_S = 2.0


# --- Gait parameters (for SPOT_PROBE=walk) ----------------------------
# Conservative wave gait. 100% upright over 30s, legs visibly cycling.
WALK_PERIOD_S = 8.0
SWING_FRACTION = 0.15
STANCE_HIP_Y = 0.30
STANCE_KNEE = -0.60
SWING_KNEE_DELTA = -0.20
HIP_SWEEP_AMP = 0.08
AMP_RAMP_CYCLES = 2.0
# Lateral CoM shift via uniform hip_x delta, to keep CoM inside the
# 3-leg support triangle during single-leg swing. From the lift_fr probe
# we know that with no shift, lifting one leg flips the body within
# ~100 ms. With a ~0.06 m lateral body shift (matching the support
# triangle centroid offset from body origin), the body stays inside the
# triangle. 0.06 m / ~0.4 m leg ~= 0.15 rad of hip_x delta.
COM_SHIFT_AMP_Y = 0.22

LEG_PHASES = {
    "front_right": 0.00,
    "rear_left":   0.25,
    "front_left":  0.50,
    "rear_right":  0.75,
}


def find_leg_solids(self_node):
    """Walk the URDFRobot's solid tree to find each leg's hip / upper_leg /
    lower_leg Solid nodes. Returns dict keyed by URDF link name.

    URDFRobot emits a tree of Solids connected by HingeJoints. Solids
    have a `name` field matching the URDF link name; HingeJoints have an
    `endPoint` field pointing at the next Solid.
    """
    target_names = set()
    for leg in LEGS:
        for part in ("hip", "upper_leg", "lower_leg"):
            target_names.add(f"{leg}_{part}")
    found = {}

    def walk(node, depth=0):
        if node is None or depth > 30:
            return
        try:
            name_field = node.getField("name")
            n = name_field.getSFString() if name_field else None
            if n and n in target_names:
                found[n] = node
        except Exception:
            pass
        # Solid.children
        try:
            ch = node.getField("children")
            if ch is not None:
                for i in range(ch.getCount()):
                    child = ch.getMFNode(i)
                    if child is not None:
                        walk(child, depth + 1)
        except Exception:
            pass
        # HingeJoint.endPoint
        try:
            ep = node.getField("endPoint")
            if ep is not None:
                ep_node = ep.getSFNode()
                if ep_node is not None:
                    walk(ep_node, depth + 1)
        except Exception:
            pass

    walk(self_node)
    return found


def rotation_matrix_to_rpy(ori):
    """Convert Webots 3x3 row-major orientation matrix to roll, pitch, yaw.

    ori is a 9-element list: [m00, m01, m02, m10, m11, m12, m20, m21, m22]
    Z-up convention: yaw around Z, pitch around Y, roll around X.
    """
    m20 = ori[6]
    # pitch = asin(-m20)
    pitch = math.asin(max(-1.0, min(1.0, -m20)))
    # roll = atan2(m21, m22)
    roll = math.atan2(ori[7], ori[8])
    # yaw = atan2(m10, m00)
    yaw = math.atan2(ori[3], ori[0])
    return roll, pitch, yaw


def main() -> int:
    sys.stderr.write("[spot_probe] starting\n")
    sys.stderr.flush()

    probe_mode = os.environ.get("SPOT_PROBE", "walk").strip().lower()
    if probe_mode not in ("hold", "lift_fr", "sweep", "walk"):
        sys.stderr.write(f"[spot_probe] unknown SPOT_PROBE={probe_mode}; using 'walk'\n")
        probe_mode = "walk"
    sys.stderr.write(f"[spot_probe] mode={probe_mode}\n")
    sys.stderr.flush()

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    motors = {}
    sensors = {}
    for leg in LEGS:
        for joint in ("hip_x", "hip_y", "knee"):
            name = f"{leg}_{joint}_motor"
            m = robot.getDevice(name)
            if m is None:
                sys.stderr.write(f"[spot_probe] missing motor {name}; aborting\n")
                return 1
            motors[(leg, joint)] = m
            try:
                if hasattr(m, "setControlPID"):
                    m.setControlPID(20.0, 0.0, 0.3)
            except Exception:
                pass
            try:
                s = m.getPositionSensor()
                if s is not None:
                    s.enable(step_ms)
                    sensors[(leg, joint)] = s
            except Exception:
                pass

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None
    if self_node is None:
        sys.stderr.write("[spot_probe] no supervisor handle; aborting\n")
        return 1

    # Locate the per-leg Solid nodes so we can read foot world positions.
    leg_solids = find_leg_solids(self_node)
    sys.stderr.write(f"[spot_probe] found leg solids: {sorted(leg_solids.keys())}\n")
    sys.stderr.flush()

    # Self-quit support for headless iteration.
    sim_duration_s = None
    env_dur = os.environ.get("SPOT_SIM_DURATION_S", "").strip()
    if env_dur:
        try:
            sim_duration_s = float(env_dur)
            sys.stderr.write(f"[spot_probe] will quit at sim t={sim_duration_s}s\n")
            sys.stderr.flush()
        except Exception:
            sim_duration_s = None

    # CSV trace.
    trace_dir = r"C:\tmp\husky_trace"
    try:
        os.makedirs(trace_dir, exist_ok=True)
    except Exception:
        trace_dir = "/tmp/husky_trace"
        try:
            os.makedirs(trace_dir, exist_ok=True)
        except Exception:
            trace_dir = None
    csv = None
    if trace_dir is not None:
        try:
            csv = open(os.path.join(trace_dir, f"spot_probe_{probe_mode}.csv"), "w", buffering=1)
        except Exception:
            csv = None
    if csv is not None:
        # Header row
        cols = ["t_ms", "phase", "bx", "by", "bz", "roll", "pitch", "yaw",
                "vx", "vy", "vz", "wx", "wy", "wz"]
        for leg in LEGS:
            short = "".join(p[0] for p in leg.split("_"))
            for kind in ("hxc", "hyc", "knc", "hxa", "hya", "kna",
                          "fx", "fy", "fz",
                          "ux", "uy", "uz"):
                cols.append(f"{short}_{kind}")
        csv.write(",".join(cols) + "\n")

    # Apply STANDING pose from t=0. With the URDF importer now seeding the
    # knee at -0.60 (because we tightened the URDF knee limits to
    # [-1.20, -0.01], midpoint -0.60), the body spawns with knees already
    # at standing -- no DROP/CROUCH transition needed for probes.
    for leg in LEGS:
        motors[(leg, "hip_x")].setPosition(hip_x_for_leg(leg))
        motors[(leg, "hip_y")].setPosition(HIP_Y_STAND)
        motors[(leg, "knee")].setPosition(KNEE_STAND)

    sim_ms = 0
    next_log_ms = 0
    crouch_started_ms = -1
    action_started_ms = -1
    walk_anchor = None  # captured (x, y, z) at start of walk phase

    # Commanded joint values (for the CSV — record what we tell the motors).
    cmd_table = {(leg, j): 0.0 for leg in LEGS for j in ("hip_x", "hip_y", "knee")}

    def set_cmd(leg, joint, value):
        cmd_table[(leg, joint)] = value
        motors[(leg, joint)].setPosition(value)

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        t = robot.getTime()

        if sim_duration_s is not None and t >= sim_duration_s:
            try:
                if hasattr(robot, "simulationQuit"):
                    robot.simulationQuit(0)
            except Exception:
                pass
            break

        # ------------------------------------------------------------
        # Phase logic
        # ------------------------------------------------------------
        # Brief SETTLE: hold standing pose for 1.5 s so the body settles
        # to its rest height before the probe action begins. With the URDF
        # importer now seeding the knee at -0.60 there is no big drop, but
        # the body still falls a few cm from its spawn z.
        SETTLE_S = 1.5
        if t < SETTLE_S:
            phase_name = "settle"
            for leg in LEGS:
                set_cmd(leg, "hip_x", hip_x_for_leg(leg))
                set_cmd(leg, "hip_y", HIP_Y_STAND)
                set_cmd(leg, "knee", KNEE_STAND)
        else:
            # Action phase, depends on probe mode.
            if action_started_ms < 0:
                action_started_ms = sim_ms
            action_t = (sim_ms - action_started_ms) / 1000.0
            phase_name = probe_mode

            if probe_mode == "hold":
                # Just hold standing pose.
                for leg in LEGS:
                    set_cmd(leg, "hip_x", hip_x_for_leg(leg))
                    set_cmd(leg, "hip_y", HIP_Y_STAND)
                    set_cmd(leg, "knee", KNEE_STAND)

            elif probe_mode == "lift_fr":
                # Hold standing for everyone EXCEPT FR; FR knee bends deeper
                # over 2 s, holds 1 s, returns over 2 s. Cycle repeats.
                for leg in LEGS:
                    set_cmd(leg, "hip_x", hip_x_for_leg(leg))
                    set_cmd(leg, "hip_y", HIP_Y_STAND)
                    set_cmd(leg, "knee", KNEE_STAND)
                # Override FR knee.
                cycle_t = action_t % 6.0  # 6s cycle
                if cycle_t < 2.0:
                    # Ramp from -0.60 to -1.30
                    knee_cmd = KNEE_STAND + (-0.70) * (cycle_t / 2.0)
                elif cycle_t < 3.0:
                    knee_cmd = -1.30
                elif cycle_t < 5.0:
                    knee_cmd = -1.30 + 0.70 * ((cycle_t - 3.0) / 2.0)
                else:
                    knee_cmd = KNEE_STAND
                set_cmd("front_right", "knee", knee_cmd)

            elif probe_mode == "sweep":
                # All 4 hip_y sweep together: standing center, +/-0.15 amplitude,
                # 4s period.
                amp = 0.15
                hip_y = HIP_Y_STAND + amp * math.sin(2.0 * math.pi * action_t / 4.0)
                for leg in LEGS:
                    set_cmd(leg, "hip_x", hip_x_for_leg(leg))
                    set_cmd(leg, "hip_y", hip_y)
                    set_cmd(leg, "knee", KNEE_STAND)

            elif probe_mode == "walk":
                # Pure physics wave gait. No supervisor body cheats. Spot's
                # legs animate through swing/stance and the body's motion
                # comes entirely from foot-floor friction.
                walk_t = action_t
                cycle_phase = (walk_t / WALK_PERIOD_S) % 1.0
                amp_scale = clamp(walk_t / (AMP_RAMP_CYCLES * WALK_PERIOD_S), 0.0, 1.0)

                # Determine which leg is in swing and shift body CoM to keep
                # the body inside the remaining 3-leg support triangle.
                # Sign convention: a NEGATIVE com_shift_y added uniformly to
                # hip_x_for_leg makes left feet abduct LESS and right feet
                # abduct MORE -> body shifts to +Y (LEFT).
                #   FR swing -> support {FL,RL,RR}, centroid at +Y -> body LEFT  -> shift=-A
                #   RL swing -> support {FL,FR,RR}, centroid at -Y -> body RIGHT -> shift=+A
                #   FL swing -> support {FR,RL,RR}, centroid at -Y -> body RIGHT -> shift=+A
                #   RR swing -> support {FL,FR,RL}, centroid at +Y -> body LEFT  -> shift=-A
                shift_by_leg = {
                    "front_right": -COM_SHIFT_AMP_Y,
                    "rear_left":   +COM_SHIFT_AMP_Y,
                    "front_left":  +COM_SHIFT_AMP_Y,
                    "rear_right":  -COM_SHIFT_AMP_Y,
                }
                com_shift_y = 0.0
                for leg, leg_phase in LEG_PHASES.items():
                    offset = (cycle_phase - leg_phase) % 1.0
                    if offset < SWING_FRACTION:
                        # This leg is currently in swing. Use a half-sine
                        # ramp so the shift smoothly rises during the first
                        # half of the swing and decays during the second.
                        p = offset / SWING_FRACTION
                        weight = math.sin(math.pi * p)
                        com_shift_y = shift_by_leg[leg] * weight
                        break

                com_shift_y *= amp_scale

                for leg in LEGS:
                    offset = (cycle_phase - LEG_PHASES[leg]) % 1.0
                    if offset < SWING_FRACTION:
                        p = offset / SWING_FRACTION
                        smooth = p * p * (3.0 - 2.0 * p)
                        # Swing: hip_y from STANCE+A down to STANCE-A; knee flexes.
                        hip_y = (STANCE_HIP_Y
                                 + HIP_SWEEP_AMP * (1.0 - 2.0 * smooth) * amp_scale)
                        knee = (STANCE_KNEE
                                + SWING_KNEE_DELTA * math.sin(math.pi * p) * amp_scale)
                    else:
                        p = (offset - SWING_FRACTION) / (1.0 - SWING_FRACTION)
                        # Stance: hip_y from STANCE-A up to STANCE+A.
                        hip_y = (STANCE_HIP_Y
                                 + HIP_SWEEP_AMP * (-1.0 + 2.0 * p) * amp_scale)
                        knee = STANCE_KNEE
                    set_cmd(leg, "hip_x", hip_x_for_leg(leg) + com_shift_y)
                    set_cmd(leg, "hip_y", hip_y)
                    set_cmd(leg, "knee", knee)

        # ------------------------------------------------------------
        # Trace
        # ------------------------------------------------------------
        if csv is not None and sim_ms >= next_log_ms:
            try:
                pos = self_node.getPosition() or [0.0, 0.0, 0.0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                vel = self_node.getVelocity() or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                roll, pitch, yaw = rotation_matrix_to_rpy(ori)
                cells = [
                    str(sim_ms), phase_name,
                    f"{pos[0]:+.4f}", f"{pos[1]:+.4f}", f"{pos[2]:+.4f}",
                    f"{roll:+.4f}", f"{pitch:+.4f}", f"{yaw:+.4f}",
                    f"{vel[0]:+.4f}", f"{vel[1]:+.4f}", f"{vel[2]:+.4f}",
                    f"{vel[3]:+.4f}", f"{vel[4]:+.4f}", f"{vel[5]:+.4f}",
                ]
                for leg in LEGS:
                    hxc = cmd_table[(leg, "hip_x")]
                    hyc = cmd_table[(leg, "hip_y")]
                    knc = cmd_table[(leg, "knee")]
                    s_hx = sensors.get((leg, "hip_x"))
                    s_hy = sensors.get((leg, "hip_y"))
                    s_kn = sensors.get((leg, "knee"))
                    hxa = s_hx.getValue() if s_hx is not None else 0.0
                    hya = s_hy.getValue() if s_hy is not None else 0.0
                    kna = s_kn.getValue() if s_kn is not None else 0.0
                    # Foot world position: from lower_leg solid (its origin
                    # is the knee joint; the foot is ~0.32 m below in the
                    # lower_leg's body frame -- but the body frame is rotated
                    # by hip_x + hip_y + knee, so the analytical computation
                    # gets messy. Easier: read the lower_leg solid's world
                    # position directly. Its origin is at the knee; the foot
                    # tip is below that (in world frame) by whatever the mesh
                    # extends. Without the mesh extent we report the knee
                    # world pos; foot z will be roughly knee_z - 0.30 m.
                    lower = leg_solids.get(f"{leg}_lower_leg")
                    upper = leg_solids.get(f"{leg}_upper_leg")
                    fx = fy = fz = 0.0
                    ux = uy = uz = 0.0
                    if lower is not None:
                        lp = lower.getPosition()
                        if lp is not None and len(lp) >= 3:
                            fx, fy, fz = float(lp[0]), float(lp[1]), float(lp[2])
                    if upper is not None:
                        up = upper.getPosition()
                        if up is not None and len(up) >= 3:
                            ux, uy, uz = float(up[0]), float(up[1]), float(up[2])
                    cells.extend([
                        f"{hxc:+.3f}", f"{hyc:+.3f}", f"{knc:+.3f}",
                        f"{hxa:+.3f}", f"{hya:+.3f}", f"{kna:+.3f}",
                        f"{fx:+.4f}", f"{fy:+.4f}", f"{fz:+.4f}",
                        f"{ux:+.4f}", f"{uy:+.4f}", f"{uz:+.4f}",
                    ])
                csv.write(",".join(cells) + "\n")
            except Exception as e:
                csv.write(f"# trace error: {e}\n")
            next_log_ms = sim_ms + 50  # 50 ms cadence, 20 Hz

    return 0


if __name__ == "__main__":
    sys.exit(main())
