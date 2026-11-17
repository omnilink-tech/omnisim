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

"""Does OMNIPILOT's inverse kinematics actually produce the motion it commands?

The robot is his, rebuilt from his URDF (r = 0.0303, L = 0.14764, wheels at
0/120/240 deg) but with PHYSICAL ROLLERS, which is the wheel his plain-cylinder
collision plus Gazebo mu1/mu2/fdir1 is standing in for.

The inverse kinematics driven here is his, exactly:

    w_i = ( d_ix * vx + d_iy * vy + L * wz ) / r

with d_i the tangential drive direction of wheel i.  Nothing is closed-loop:
the wheel speeds are computed once per phase from the commanded body twist and
held, so what comes out is what open-loop dead reckoning would have promised
against what the contact patch actually delivered.

Four commands.  How much lateral slide each one demands of the wheels is
computed in `predict`, not assumed:

    spin              zero  -- every wheel rolls, nothing slides at all
    strafe_y          1.73 v
    forward_x         2.00 v  -- the WORST case, not the strafe
    strafe_and_spin   the body frame turns under the command, so this is the
                      one where open-loop dead reckoning has somewhere to go

The command is in the body frame and the measured pose is in the world frame,
so every comparison here rotates one into the other by the yaw measured at the
start of the phase.  `dead_reckoning_error_m` integrates the COMMANDED twist
from the measured start pose and reports the gap to where the robot actually
ended up -- which is the number a robot navigating on wheel odometry alone
would accumulate, and the only one that stays meaningful while it is turning.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260915",
                   "ahmed")

R = 0.0303
L = 0.14764
V = 0.20            # m/s, needs 6.6 rad/s -- inside his +/-10 command limit
WZ = 1.0            # rad/s

# tag -> tangential drive direction, from the URDF joint origins
DRIVE = {
    "front": (0.0, 1.0),
    "left": (-0.8660254, -0.5),
    "right": (0.8660254, -0.5),
}
TAGS = ["front", "left", "right"]

PHASES = [
    ("settle_0", 0.0, 0.0, 0.0, 1.5),
    ("strafe_y", 0.0, V, 0.0, 4.0),
    ("settle_1", 0.0, 0.0, 0.0, 1.5),
    ("forward_x", V, 0.0, 0.0, 4.0),
    ("settle_2", 0.0, 0.0, 0.0, 1.5),
    ("spin", 0.0, 0.0, WZ, 3.0),
    ("settle_3", 0.0, 0.0, 0.0, 1.5),
    ("strafe_and_spin", 0.0, V, 0.5, 4.0),
    ("settle_4", 0.0, 0.0, 0.0, 1.5),
]


def ik(vx, vy, wz):
    """His inverse kinematics, unmodified."""
    return {t: (DRIVE[t][0] * vx + DRIVE[t][1] * vy + L * wz) / R
            for t in TAGS}


def predict(vx, vy, wz):
    """Per-wheel rolling and lateral-slide speed the kinematics demands.

    A wheel's contact point moves with the chassis.  The component along the
    drive direction is what the wheel rolls off; the component across it is
    what the rollers have to absorb.  With a plain cylinder and isotropic
    friction that component is simply dragged.
    """
    out = {}
    slide_total = 0.0
    for t in TAGS:
        dx, dy = DRIVE[t]
        # the drive direction is tangential, d = zhat x p_hat, so the wheel
        # centre direction is p_hat = d x zhat = (dy, -dx).  Checked against
        # the URDF origins: left comes back (-0.0738, +0.1279) as authored.
        nx, ny = dy, -dx
        px, py = L * nx, L * ny
        cvx, cvy = vx - wz * py, vy + wz * px
        roll = cvx * dx + cvy * dy
        slide = cvx * nx + cvy * ny
        out[t] = {"roll_m_s": round(roll, 6), "slide_m_s": round(slide, 6)}
        slide_total += abs(slide)
    out["slide_total_m_s"] = round(slide_total, 6)
    return out


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    me = robot.getFromDef("KIWI")
    if me is None:
        print("[kiwi] FATAL: DEF KIWI not found", flush=True)
        sys.exit(1)

    motors, sensors = {}, {}
    for t in TAGS:
        m = robot.getDevice(t + "_wheel")
        s = robot.getDevice(t + "_wheel_pos")
        if m is None or s is None:
            print("[kiwi] FATAL: device missing for " + t, flush=True)
            sys.exit(1)
        m.setPosition(float("inf"))            # velocity mode
        m.setVelocity(0.0)
        s.enable(dt_ms)
        motors[t], sensors[t] = m, s

    def pose():
        p = me.getPosition()
        o = me.getOrientation()                # row-major 3x3
        return p[0], p[1], p[2], math.atan2(o[3], o[0])

    rec = {"sample_columns": ["t_s", "x", "y", "z", "yaw_rad",
                              "enc_front_rad", "enc_left_rad", "enc_right_rad"],
           "basic_time_step_ms": dt_ms, "wheel_radius_m": R,
           "wheel_offset_m": L, "commanded_speed_m_s": V,
           "ground_mu": 1.0, "n_rollers_per_wheel": 12,
           "phases": []}

    robot.step(dt_ms)
    yaw_prev = pose()[3]
    yaw_acc = yaw_prev                          # unwrapped

    for name, vx, vy, wz, dur in PHASES:
        cmd = ik(vx, vy, wz)
        for t in TAGS:
            motors[t].setVelocity(cmd[t])

        x0, y0, z0, _ = pose()
        yaw0 = yaw_acc
        dr_x, dr_y, dr_th = x0, y0, yaw0
        enc0 = {t: sensors[t].getValue() for t in TAGS}
        t_el = 0.0
        samples = []
        settle_mark = None
        while robot.step(dt_ms) != -1 and t_el < dur:
            t_el += dt
            x, y, z, yw = pose()
            d = yw - yaw_prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            yaw_acc += d
            yaw_prev = yw
            # dead reckoning, midpoint rule, on the COMMANDED twist alone
            th_mid = dr_th + 0.5 * wz * dt
            dr_x += (vx * math.cos(th_mid) - vy * math.sin(th_mid)) * dt
            dr_y += (vx * math.sin(th_mid) + vy * math.cos(th_mid)) * dt
            dr_th += wz * dt
            if t_el >= 1.0 and settle_mark is None:
                # after the acceleration transient: the steady-state ruler
                settle_mark = (t_el, x, y, yaw_acc,
                               {t: sensors[t].getValue() for t in TAGS})
            if len(samples) < 4000:
                # ground truth AND the raw encoder stream, so his own odometry
                # node can be replayed on exactly what his robot would publish
                samples.append([round(t_el, 4), round(x, 6), round(y, 6),
                                round(z, 6), round(yaw_acc, 6)]
                               + [round(sensors[t].getValue(), 8)
                                  for t in TAGS])

        x1, y1, z1, _ = pose()
        enc1 = {t: sensors[t].getValue() for t in TAGS}
        ach = {t: (enc1[t] - enc0[t]) / max(t_el, 1e-9) for t in TAGS}

        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        want = math.hypot(vx, vy) * t_el
        # world displacement expressed in the body frame the command was given in
        c0, s0 = math.cos(yaw0), math.sin(yaw0)
        bx, by = dx * c0 + dy * s0, -dx * s0 + dy * c0
        dr_err = math.hypot(x1 - dr_x, y1 - dr_y)
        dr_head = math.degrees(yaw_acc - dr_th)
        # commanded direction in the body frame at phase start
        entry = {
            "phase": name, "duration_s": round(t_el, 4),
            "cmd_vx": vx, "cmd_vy": vy, "cmd_wz": wz,
            "wheel_cmd_rad_s": {t: round(cmd[t], 6) for t in TAGS},
            "wheel_achieved_rad_s": {t: round(ach[t], 6) for t in TAGS},
            "wheel_slip_rad_s": {t: round(ach[t] - cmd[t], 6) for t in TAGS},
            "kinematics_demands": predict(vx, vy, wz),
            "start_xyz": [round(x0, 6), round(y0, 6), round(z0, 6)],
            "end_xyz": [round(x1, 6), round(y1, 6), round(z1, 6)],
            "displacement_m": [round(dx, 6), round(dy, 6)],
            "distance_m": round(dist, 6),
            "distance_commanded_m": round(want, 6),
            "yaw_drift_deg": round(math.degrees(yaw_acc - yaw0), 5),
            "displacement_body_m": [round(bx, 6), round(by, 6)],
            "dead_reckoned_end_xy": [round(dr_x, 6), round(dr_y, 6)],
            "dead_reckoning_error_m": round(dr_err, 6),
            "dead_reckoning_error_pct_of_path": (
                round(100.0 * dr_err / want, 4) if want > 1e-6 else None),
            "dead_reckoning_heading_error_deg": round(dr_head, 5),
        }
        if want > 1e-6:
            entry["speed_ratio"] = round(dist / want, 5)
            entry["heading_of_travel_body_deg"] = round(
                math.degrees(math.atan2(by, bx)), 4)
            entry["heading_commanded_body_deg"] = round(
                math.degrees(math.atan2(vy, vx)), 4)
            ux, uy = vx / math.hypot(vx, vy), vy / math.hypot(vx, vy)
            entry["along_track_m"] = round(bx * ux + by * uy, 6)
            entry["cross_track_m"] = round(-bx * uy + by * ux, 6)
        if settle_mark is not None:
            ts, xs, ys, yws, encs = settle_mark
            span = t_el - ts
            if span > 0.5:
                sdx, sdy = x1 - xs, y1 - ys
                entry["steady_state"] = {
                    "window_s": [round(ts, 3), round(t_el, 3)],
                    "distance_m": round(math.hypot(sdx, sdy), 6),
                    "speed_m_s": round(math.hypot(sdx, sdy) / span, 6),
                    "yaw_rate_deg_s": round(
                        math.degrees(yaw_acc - yws) / span, 5),
                    "wheel_achieved_rad_s": {
                        t: round((enc1[t] - encs[t]) / span, 6) for t in TAGS},
                }
                if want > 1e-6:
                    cs, ss_ = math.cos(yws), math.sin(yws)
                    entry["steady_state"]["heading_of_travel_body_deg"] = round(
                        math.degrees(math.atan2(-sdx * ss_ + sdy * cs,
                                                sdx * cs + sdy * ss_)), 4)
        entry["samples"] = samples
        rec["phases"].append(entry)

        tail = ""
        if want > 1e-6:
            tail = ("dist %.4f/%.4f m (%.1f%%) travel %.2f deg vs cmd %.2f "
                    % (dist, want, 100 * dist / want,
                       entry["heading_of_travel_body_deg"],
                       entry["heading_commanded_body_deg"]))
        print("[kiwi] %-14s %syaw %+.3f deg  dr_err %.4f m  slide %.3f m/s"
              % (name, tail, entry["yaw_drift_deg"],
                 entry["dead_reckoning_error_m"],
                 entry["kinematics_demands"]["slide_total_m_s"]), flush=True)

    for t in TAGS:
        motors[t].setVelocity(0.0)

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "kiwi_strafe.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[kiwi] wrote kiwi_strafe.json", flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        tb = traceback.format_exc()
        with open(os.path.join(OUT, "kiwi_tb.txt"), "a") as fh:
            fh.write(tb)
        print("[kiwi] FAILED " + tb, flush=True)
        raise
