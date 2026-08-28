#!/usr/bin/env python3

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

"""RoboLife docking gate (DESIGN.md "Verification gates", A). Runs ON the
probe Husky (`supervisor TRUE` only so it can read MODULE_0's pose; every
motion is driven through its own four wheel motors, GPS and InertialUnit,
exactly as B's `robolife_robot` will).

Timeline (dt = basicTimeStep):
  settle           -> rest height, initial socket presence
  A  2 m drive     -> {commanded, achieved, error} distance (open-loop wheel law)
  B  90 deg turn   -> {commanded, achieved, error} yaw   (open-loop wheel law)
  C  brake, bare   -> distance from the brake command at 0.8 m/s to rest
  D  dock          -> navigate to MODULE_0's plug, align, creep, lock();
                      presence before/after, isLocked, socket<->plug separation
  E  tow 4 m       -> max socket<->plug separation while driving, module displacement
  F  brake, docked -> same as C with the 6 kg battery on the front socket
  G  unlock        -> back away 2 m; the module must stay put (its pose before/after)
Every number is MEASURED from GPS / IMU / the module node -- never the
command echoed back. Wall time around every step() gives engine ms/step.
Output: --out <json>; then simulationQuit(0).
"""
import json
import math
import os
import sys
import time

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..")))
from rl import modules as M  # noqa: E402

WHEEL_R = 0.1651
TRACK = 0.555
SOCKET_X = M.SOCKET_X
SOCKET_Z = M.SOCKET_Z_ROBOT


def _arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default


OUT = _arg("--out", os.path.join(HERE, "..", "..", "_run", "probe_dock_result.json"))
MODULE_DEF = _arg("--module", "MODULE_0")
MODULE_TYPE = _arg("--module-type", "battery")

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
DT = dt / 1000.0

motors = {n: robot.getDevice(n + "_wheel_motor")
          for n in ("front_left", "front_right", "rear_left", "rear_right")}
for m in motors.values():
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
QUICK = "--quick" in sys.argv
DOCK_ONLY = "--dock-only" in sys.argv
wheel_sensors = {n: robot.getDevice(n + "_wheel_sensor")
                 for n in ("front_left", "front_right", "rear_left", "rear_right")}
for s in wheel_sensors.values():
    s.enable(dt)
self_node = robot.getSelf()
gps = robot.getDevice("navsat_fix")
gps.enable(dt)
imu = robot.getDevice("imu_data")
imu.enable(dt)
socket_f = robot.getDevice("socket_front")
socket_f.enablePresence(dt)
module = robot.getFromDef(MODULE_DEF)

step_ms = []
sim_ticks = [0]
_prev_p = [None]
_vel = [0.0, 0.0, 0.0]
log = []


def note(msg):
    log.append("t=%.2f %s" % (sim_ticks[0] * DT, msg))
    sys.stdout.write("[probe_dock] " + log[-1] + "\n")
    sys.stdout.flush()


def step():
    t0 = time.perf_counter()
    r = robot.step(dt)
    step_ms.append((time.perf_counter() - t0) * 1000.0)
    sim_ticks[0] += 1
    if r == -1:
        finish("engine stopped the controller")
    p = gps.getValues()
    if _prev_p[0] is not None:
        for k in range(3):
            _vel[k] = (p[k] - _prev_p[0][k]) / DT
    _prev_p[0] = list(p)
    return r


def steps(seconds):
    for _ in range(max(1, int(round(seconds / DT)))):
        step()


def pos():
    p = gps.getValues()
    return [float(p[0]), float(p[1]), float(p[2])]


def yaw():
    return float(imu.getRollPitchYaw()[2])


def yaw_supervisor():
    """Ground truth from the scene graph, to audit the InertialUnit."""
    o = self_node.getOrientation()
    return math.atan2(o[3], o[0])


_wheel_prev = {}


def wheel_rates():
    """Measured wheel angular velocities [rad/s] from the PositionSensors."""
    out = {}
    for n, s in wheel_sensors.items():
        q = float(s.getValue())
        out[n] = (q - _wheel_prev[n]) / DT if n in _wheel_prev else 0.0
        _wheel_prev[n] = q
    return out


def speed():
    return math.hypot(_vel[0], _vel[1])


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def drive(v, w):
    """Diff-drive wheel law: v [m/s], w [rad/s] -> wheel rad/s (left, right)."""
    wl = (v - w * TRACK / 2.0) / WHEEL_R
    wr = (v + w * TRACK / 2.0) / WHEEL_R
    motors["front_left"].setVelocity(wl)
    motors["rear_left"].setVelocity(wl)
    motors["front_right"].setVelocity(wr)
    motors["rear_right"].setVelocity(wr)


def stop():
    drive(0.0, 0.0)


def module_pose():
    p = module.getPosition()
    o = module.getOrientation()
    return [float(p[0]), float(p[1]), float(p[2])], math.atan2(o[3], o[0])


def socket_world():
    p, y = pos(), yaw()
    c, s = math.cos(y), math.sin(y)
    return [p[0] + c * SOCKET_X, p[1] + s * SOCKET_X, p[2] + SOCKET_Z]


def plug_world():
    p, y = module_pose()
    px, py, pz, _ = M.plug_pose(MODULE_TYPE, p, y)
    return [px, py, pz]


def separation():
    a, b = socket_world(), plug_world()
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


result = {"ok": False, "module": MODULE_DEF, "module_type": MODULE_TYPE, "dt_ms": dt}


def finish(reason=None):
    if reason:
        result["abort"] = reason
    ms = sorted(step_ms[20:]) or [0.0]
    result["engine_ms_per_step"] = {
        "mean": sum(ms) / len(ms), "median": ms[len(ms) // 2],
        "p95": ms[int(0.95 * (len(ms) - 1))], "n_steps": len(ms),
        "note": "wall time around robot.step() on the probe controller, --mode=fast; "
                "includes controller IPC, excludes the first 20 ticks",
    }
    result["sim_time_s"] = sim_ticks[0] * DT
    result["log"] = log
    os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    sys.stdout.write("[probe_dock] wrote %s\n" % OUT)
    sys.stdout.flush()
    robot.simulationQuit(0)
    sys.exit(0)


# ---------------------------------------------------------------- navigation
W_PIVOT = 3.0      # rad/s commanded: enough wheel-speed error to break loose


def turn_to(target_yaw, tol=0.05, timeout=10.0):
    """In-place pivot. Below W_PIVOT the wheels stall (see turn_test), so the
    pivot is bang-bang at W_PIVOT with a settle between pulses; tol 0.05 rad
    is well inside the Connector's 0.45 rad axisTolerance."""
    t_end = sim_ticks[0] * DT + timeout
    while sim_ticks[0] * DT < t_end:
        e = wrap(target_yaw - yaw())
        if abs(e) < tol:
            break
        drive(0.0, math.copysign(W_PIVOT, e))
        # pulse: at most 1/3 of the residual per pulse at the measured ~1.5 rad/s
        n = max(1, int(round(abs(e) / 1.5 / 3.0 / DT)))
        for _ in range(n):
            step()
        stop()
        steps(0.3)
    stop()
    steps(0.5)
    return wrap(target_yaw - yaw())


def goto(goal, tol=0.05, v_max=0.4, timeout=30.0):
    """Turn to face the goal, then drive with proportional heading correction."""
    p = pos()
    turn_to(math.atan2(goal[1] - p[1], goal[0] - p[0]))
    t_end = sim_ticks[0] * DT + timeout
    while sim_ticks[0] * DT < t_end:
        p = pos()
        d = dist(goal, p)
        if d < tol:
            break
        e = wrap(math.atan2(goal[1] - p[1], goal[0] - p[0]) - yaw())
        v = min(v_max, max(0.15, 0.6 * d))
        drive(v * max(0.2, math.cos(e)), max(-1.5, min(1.5, 3.0 * e)))
        step()
    stop()
    steps(0.5)
    return dist(goal, pos())


def brake_test(label):
    """Accelerate to 0.8 m/s, brake (setVelocity 0), measure distance to rest."""
    drive(0.8, 0.0)
    t0 = sim_ticks[0] * DT
    vmax = 0.0
    while sim_ticks[0] * DT - t0 < 6.0:
        step()
        vmax = max(vmax, speed())
        if speed() >= 0.78:
            break
    t_accel = sim_ticks[0] * DT - t0
    v_at_brake = speed()
    p0 = pos()
    stop()
    t1 = sim_ticks[0] * DT
    while sim_ticks[0] * DT - t1 < 5.0:
        step()
        if speed() < 0.02:
            break
    t_brake = sim_ticks[0] * DT - t1
    r = {"commanded_v": 0.8, "v_at_brake": v_at_brake, "vmax": vmax,
         "reached_0p78": v_at_brake >= 0.78, "t_accel_s": t_accel,
         "brake_distance_m": dist(pos(), p0), "t_brake_s": t_brake}
    result["brake_" + label] = r
    note("brake %s: v=%.3f dist=%.3f m in %.2f s" % (label, v_at_brake, r["brake_distance_m"], t_brake))
    steps(0.5)
    return r


# ---------------------------------------------------------------- timeline
steps(5 * DT)
steps(1.0)
result["rest"] = {"gps": pos(), "yaw": yaw(), "presence_initial": int(socket_f.getPresence()),
                  "module_pose": module_pose(), "separation_initial": separation()}
note("rest z=%.4f presence=%d module=%s" % (pos()[2], result["rest"]["presence_initial"],
                                            result["rest"]["module_pose"]))

if DOCK_ONLY:
    # skip A-C; put the robot where C would have left it
    result["skipped"] = "A-C (--dock-only)"

# A. 2 m straight drive, open loop on the wheel law
p0, y0 = pos(), yaw()
if DOCK_ONLY:
    p1 = p0
drive(0.5 if not DOCK_ONLY else 0.0, 0.0)
steps(2.0 / 0.5)
stop()
steps(1.0)
p1 = pos()
result["drive_2m"] = {"commanded_m": 2.0, "achieved_m": dist(p1, p0),
                      "error_m": dist(p1, p0) - 2.0,
                      "along_heading_m": (p1[0] - p0[0]) * math.cos(y0) + (p1[1] - p0[1]) * math.sin(y0),
                      "yaw_drift_rad": wrap(yaw() - y0), "commanded_v": 0.5}
note("drive 2 m: achieved %.4f (along heading %.4f), yaw drift %.4f"
     % (result["drive_2m"]["achieved_m"], result["drive_2m"]["along_heading_m"],
        result["drive_2m"]["yaw_drift_rad"]))

# B. 90 deg turns, open loop on the wheel law.
#    A skid-steer pivot has to BREAK the wheels loose from static friction:
#    the velocity servo gain is clamped to I_wheel/dt = 0.044/0.008 = 5.5
#    N m s/rad (omnisim_newton_runtime._clamp_velocity_servo_gains), so the
#    torque a wheel can raise is 5.5 x (commanded - actual rad/s), capped by
#    maxTorque. At w = 0.6 rad/s that is 5.5 N m against a ~20-28 N m
#    breakaway (mu 0.8-1.5) -> the wheels stall and the chassis does not move
#    (measured: 0.04 deg). So three laws are measured: a slow pivot, a fast
#    pivot (enough wheel-speed error to break loose) and an arc while driving.
def turn_test(label, w, v=0.0, target=math.pi / 2.0):
    y0, p0 = yaw(), pos()
    ys0 = yaw_supervisor()
    drive(v, w)
    wheel_rates()
    rates = []
    traj = []
    n_turn = int(round(target / w / DT))
    for i in range(n_turn + int(round(1.0 / DT))):
        if i == n_turn:
            stop()
        step()
        rates.append(wheel_rates())
        if i % int(round(0.1 / DT)) == 0:
            traj.append([round((i + 1) * DT, 3), round(math.degrees(wrap(yaw() - y0)), 2),
                         round(rates[-1]["front_left"], 2), round(rates[-1]["front_right"], 2)])
    mid = rates[n_turn // 2]
    dy = wrap(yaw() - y0)
    r = {"commanded_rad": target, "achieved_rad": dy, "error_rad": dy - target,
         "achieved_deg": math.degrees(dy), "achieved_fraction": dy / target,
         "achieved_rad_supervisor": wrap(yaw_supervisor() - ys0),
         "commanded_w": w, "commanded_v": v, "moved_m": dist(pos(), p0),
         "wheel_rates_mid": mid, "trajectory_t_deg_wl_wr": traj,
         "wheel_rates_commanded": {"left": (v - w * TRACK / 2.0) / WHEEL_R,
                                   "right": (v + w * TRACK / 2.0) / WHEEL_R}}
    result[label] = r
    note("%s: w=%.2f v=%.2f -> %.1f deg (%.0f%%), moved %.3f m, wheels L %.2f R %.2f"
         % (label, w, v, r["achieved_deg"], 100 * r["achieved_fraction"], r["moved_m"],
            mid["front_left"], mid["front_right"]))
    return r


if not DOCK_ONLY:
    turn_test("turn_90", 0.6)                       # the naive law B would inherit
    turn_test("turn_90_fast", 3.0)                  # pivot with breakaway
    turn_test("turn_90_arc", 0.6, v=0.4)            # arc while driving
    result["turn_90"]["note"] = "slow pivot: wheels stall against static friction (see turn_90_fast)"

# C. braking, bare
if not DOCK_ONLY:
    brake_test("bare")
result["yaw_audit"] = {"imu": yaw(), "supervisor": yaw_supervisor()}
if QUICK:
    result["ok"] = True
    result["verdict"] = {"quick": True}
    finish()

# D. dock to MODULE_0
mp, my = module_pose()
result["module_start"] = {"pos": mp, "yaw": my}
dock_xy = M.docked_robot_xy(MODULE_TYPE, mp, my)
pre = (dock_xy[0] - 1.5 * math.cos(my), dock_xy[1] - 1.5 * math.sin(my))
result["dock_geometry"] = {"docked_robot_xy": list(dock_xy), "pre_dock_xy": list(pre),
                           "plug_world": plug_world()}
gd = goto(pre, tol=0.06)
ye = turn_to(my, tol=0.015)
note("pre-dock: pos err %.3f m, yaw err %.4f rad, separation %.3f" % (gd, ye, separation()))
result["pre_dock"] = {"pos_error_m": gd, "yaw_error_rad": ye, "separation_m": separation()}

creep = {"presence_seen": 0, "min_separation": separation(), "ticks": 0}
mp0 = module_pose()[0]
t0 = sim_ticks[0] * DT
stop_reason = "timeout"
while sim_ticks[0] * DT - t0 < 20.0:
    e = wrap(my - yaw())
    sep = separation()
    creep["min_separation"] = min(creep["min_separation"], sep)
    pres = int(socket_f.getPresence())
    if pres == 1:
        creep["presence_seen"] = 1
        stop_reason = "presence"
        break
    moved = dist(module_pose()[0], mp0)
    if moved > 0.03:
        stop_reason = "module pushed %.3f m" % moved
        break
    if sep < 0.025:
        stop_reason = "separation < 0.025"
        break
    drive(0.12, max(-0.6, min(0.6, 3.0 * e)))
    step()
    creep["ticks"] += 1
stop()
steps(0.3)
creep["stop_reason"] = stop_reason
creep["separation_at_stop"] = separation()
creep["presence_before_lock"] = int(socket_f.getPresence())
creep["module_pushed_m"] = dist(module_pose()[0], mp0)
creep["socket_world"] = socket_world()
creep["plug_world"] = plug_world()
result["creep"] = creep
note("creep stop: %s, separation %.4f, presence %d" % (stop_reason, creep["separation_at_stop"],
                                                        creep["presence_before_lock"]))

socket_f.lock()
steps(5 * DT)
lock = {"is_locked": bool(socket_f.isLocked()), "presence_after_lock": int(socket_f.getPresence()),
        "separation_after_lock": separation()}
result["lock"] = lock
note("lock: isLocked=%s presence=%d sep=%.4f" % (lock["is_locked"], lock["presence_after_lock"],
                                                 lock["separation_after_lock"]))
steps(0.5)

# E. tow 4 m
p0 = pos()
mp_e0 = module_pose()[0]
sep0 = separation()
max_sep, sum_sep, n = 0.0, 0.0, 0
drive(0.5, 0.0)
while dist(pos(), p0) < 4.0 and n < int(15.0 / DT):
    step()
    s = separation()
    max_sep = max(max_sep, s)
    sum_sep += s
    n += 1
stop()
steps(1.0)
mp_e1 = module_pose()[0]
result["tow_4m"] = {"commanded_m": 4.0, "robot_moved_m": dist(pos(), p0),
                    "module_moved_m": dist(mp_e1, mp_e0),
                    "separation_start": sep0, "separation_max": max_sep,
                    "separation_mean": sum_sep / max(1, n), "separation_end": separation(),
                    "presence": int(socket_f.getPresence()), "is_locked": bool(socket_f.isLocked()),
                    "module_z_end": mp_e1[2]}
note("tow: robot %.3f m, module %.3f m, sep max %.4f end %.4f"
     % (result["tow_4m"]["robot_moved_m"], result["tow_4m"]["module_moved_m"], max_sep, separation()))

# F. braking, docked (only meaningful if the weld carried the module)
brake_test("docked")
result["brake_docked"]["module_still_attached"] = separation() < 0.15
result["brake_docked"]["separation_after"] = separation()

# G. unlock and drive on 2 m -- IN REVERSE. Measured first with a forward
#    drive: the weld released cleanly (the module sat still while the robot
#    advanced 11.6 cm) and then the chassis box, whose front is 5.6 cm behind
#    the socket, caught the module's rear face and PUSHED it 1.9 m. A front
#    module is released by backing away (or a rear one by driving on); B's
#    brain must do the same.
socket_f.unlock()
steps(3 * DT)
mp_g0 = module_pose()[0]
unl = {"is_locked": bool(socket_f.isLocked()), "presence": int(socket_f.getPresence()),
       "module_pos_at_unlock": mp_g0}
p0 = pos()
drive(-0.5, 0.0)
trace = []
for i in range(int(round(2.0 / 0.5 / DT))):
    step()
    if i % int(round(0.1 / DT)) == 0:
        s_w, p_w, mpos = socket_world(), plug_world(), module_pose()[0]
        trace.append([round((i + 1) * DT, 3), round(separation(), 4),
                      round(p_w[0] - s_w[0], 4), round(p_w[1] - s_w[1], 4),
                      round(dist(mpos, mp_g0), 4)])
unl["trace_t_sep_dx_dy_modulemoved"] = trace
stop()
steps(1.0)
mp_g1 = module_pose()[0]
unl.update({"robot_moved_m": dist(pos(), p0), "module_moved_m": dist(mp_g1, mp_g0),
            "module_pos_after": mp_g1, "separation_after": separation(),
            "presence_after": int(socket_f.getPresence())})
result["unlock"] = unl
note("unlock: robot %.3f m, module moved %.3f m, sep %.3f"
     % (unl["robot_moved_m"], unl["module_moved_m"], unl["separation_after"]))

carried = (lock["is_locked"] and result["tow_4m"]["module_moved_m"] > 3.0
           and result["tow_4m"]["separation_max"] < 0.15)
released = unl["module_moved_m"] < 0.1 and unl["separation_after"] > 1.0
result["verdict"] = {"weld_carries": carried, "weld_releases": released,
                     "pass": bool(carried and released)}
result["ok"] = True
note("VERDICT carries=%s releases=%s PASS=%s" % (carried, released, result["verdict"]["pass"]))
finish()
