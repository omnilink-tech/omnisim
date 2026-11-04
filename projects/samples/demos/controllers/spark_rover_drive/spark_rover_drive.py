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

"""SparkRover Mini's drive test: 1 m straight, a stop, a spin-turn in place, an arm wave, measured.

Each wheel motor is the Pololu #4844 (34:1, 12 V rated) on the pack's voltage: no-load speed
and stall torque scale with the voltage (Pololu's 6 V and 12 V rows agree), and every step
the available torque is set to stall x |1 - |speed| / no-load speed| with the velocity
commanded at the no-load speed, so a wheel settles where the motor's torque meets its load:
driving below no-load speed, BRAKING above it (setAvailableTorque(0) would make the joint
passive: the desk robot's spin overspeed, desk_robot_drive.available_torque). The current
is the motor's on that line (chassis.operating's model): I0 + (Is - I0) x torque / stall.
Newton has no motor torque feedback yet (OmMotor.cpp), so the current is read off the
measured wheel speed, not off a measured torque.

The arms are the XC330-M288-T position servos (the world's RotationalMotor limits): each
pose of WAVE is reached on the design's trapezoid profile (arms.WAVE: 90 deg in 0.5 s), held
HOLD_S, and its angle read over the last 0.25 s.

The face runs alongside, outside the physics: the head's camera (U9, the Camera Module 3,
"camera") is enabled every CAMERA_MS, and the screen (U8, the 5" 800 x 480 panel, "display")
shows a face that follows the test's phase (eyes, a mouth, "driving" / "spinning" / "waving")
with the camera's latest frame as an inset in its corner. Neither reads back into the drive.

The pack is spark_rover/battery.py's model of the 2S2P Molicel P28A (its data sheet's
discharge curves, R0 and an RC branch, coulomb counting, the 6.4 V cut-off): every step the
motors run on the bus voltage it gives under that step's load (the four motors' currents on
their line, plus the electronics and servo rails through their regulators: electrical's
operating points), so the robot is a little faster on a full pack than on a tired one.
--fixed-pack runs the motors on --pack-v instead (the design's nominal 7.2 V).

controllerArgs: --pack-v --rated-v --no-load-rpm --stall-kgcm (robot.py writes them from the
design) [--soc <0-1>] [--fixed-pack] [--result <json>] [--quit] [--no-face] [--frames <dir>]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

from omnisim import Supervisor

ROOT = Path(__file__).resolve().parents[5]
PRODUCT = ROOT / "projects" / "manufacturing" / "products" / "spark_rover"
# The rails' loads besides the motors (A at the regulators' outputs): battery.design_profiles(), i.e.
# electrical.operating_point(case, 7.2 V) of 2026-11-04; tests/test_spark_rover_battery pins them.
ELECTRONICS_A = {"idle": 1.2946, "arms": 2.489, "driving": 2.3713}     # the 5 V rail (Pi, screen, Pico, ...)
SERVO_A = {"idle": 0.068, "arms": 0.8, "driving": 0.4}                 # the servo rail
R_EXT_OHM = 0.0315     # pack terminals to the Motorons' VIN: connectors, fuse, switch, splices, wires
KGCM = 0.0980665
NO_LOAD_A, STALL_A_RATED = 0.30, 5.0          # Pololu #4844 at 12 V (chassis.BOUGHT["gearmotor"])
WHEELS = ("fl", "fr", "rl", "rr")
LEFT = {"fl": True, "fr": False, "rl": True, "rr": False}
STRAIGHT_M = 1.0
SPIN_S = 4.0
HOLD_S = 1.0           # after each move, the pose is held this long; its angle read over the last 0.25 s
# (shoulder, elbow) in degrees, both arms: through every limit (arms.joints(): shoulder
# -70..+90, elbow +-90), then back to hanging.
WAVE = [(90, 0), (-70, 0), (0, 90), (0, -90), (90, 90), (90, -90), (45, 45), (0, 0)]
# The arms' design motion (arms.WAVE): 90 deg in 0.5 s, a trapezoid in thirds, so the peak
# speed is 90 deg / (2/3 x 0.5 s) and the acceleration that speed over 1/6 s. Every move runs
# at those two limits (the XC330's Profile Velocity / Profile Acceleration), the joints of a
# pose synchronised to finish together. --raw-arms instead jumps each target (setPosition
# alone): the simulated servo then slams at its stall torque, and both arms together slid the
# braked 2.4 kg robot 10-27 mm per swing (2026-11-03; the engine's position servo has no rotor
# inertia, where the XC330's 288:1 gearbox reflects its motor's).
PROFILE_W = math.radians(90.0) / (2.0 / 3.0 * 0.5)
PROFILE_A = PROFILE_W / (0.5 / 3.0)
CAMERA_MS = 100        # the camera's frame period (10 fps: the inset and any vision code)
FACE_MS = 100          # the face is redrawn at most this often, and only when it changes
INSET = 4              # the inset is the camera frame at 1/INSET: 160 x 90 from 640 x 360
MOODS = {"settle": "ready", "straight": "driving", "stop": "stopped", "spin": "spinning", "stop2": "stopped",
         "move": "waving", "wave": "waving", "done": "done"}


def mood_of(phase):
    return MOODS.get(phase.split("_")[0], phase)


class Face:
    """The robot's face on its 800 x 480 screen: two eyes (blinking every 3 s), a mouth and the
    phase's word, and the camera's latest frame as an inset. Pixel (0, 0) is the top left as seen
    from outside: the robot's left is the viewer's right."""

    BG, EYE, PUPIL, INK, DIM = 0x0B1424, 0xF2F6FF, 0x101820, 0x46D7FF, 0x5A6A80

    def __init__(self, display, camera):
        self.d, self.cam = display, camera
        self.w, self.h = display.getWidth(), display.getHeight()
        self.key = None
        self.next_t = 0.0
        self.draws = 0
        self.insets = 0
        self.inset_t = None
        try:
            import numpy as np
            self.np = np
        except ImportError:          # the inset needs numpy; the face does not
            self.np = None

    def _smile(self, cx, cy, rx, ry, thick):
        xs, ys = [], []
        for i in range(17):                                   # the outer edge, left to right
            a = math.pi * i / 16
            xs.append(int(cx - rx * math.cos(a)))
            ys.append(int(cy + ry * math.sin(a)))
        for i in range(16, -1, -1):                           # the inner edge back
            a = math.pi * i / 16
            xs.append(int(cx - (rx - thick) * math.cos(a)))
            ys.append(int(cy + max(ry - thick, 1) * math.sin(a)))
        self.d.fillPolygon(xs, ys)

    def draw(self, mood, blink, power=None):
        d = self.d
        d.setColor(self.BG)
        d.fillRectangle(0, 0, self.w, self.h)
        look = 34 if mood == "spinning" else 0       # turning left: toward the robot's left, the viewer's right
        for cx in (260, 540):
            d.setColor(self.EYE)
            if blink:
                d.fillRectangle(cx - 70, 180, 140, 12)
            elif mood == "waving":                              # happy eyes: upturned arcs
                d.fillOval(cx, 200, 70, 60)
                d.setColor(self.BG)
                d.fillOval(cx, 228, 72, 56)
            else:
                d.fillOval(cx, 185, 66, 82)
                d.setColor(self.PUPIL)
                d.fillOval(cx + look, 195, 26, 32)
        d.setColor(self.EYE)
        if mood in ("driving", "waving"):
            big = mood == "waving"
            self._smile(400, 300, 120 if big else 95, 70 if big else 45, 22)
        elif mood == "spinning":
            d.fillOval(400, 330, 30, 28)
            d.setColor(self.BG)
            d.fillOval(400, 330, 16, 14)
        else:
            d.fillRectangle(330, 322, 140, 14)
        d.setColor(self.INK)
        d.setFont("LiberationSans-Bold", 56, True)
        d.drawText(mood, 40, 400)
        d.setColor(self.DIM)
        d.setFont("LiberationSans-Regular", 18, True)
        d.drawText("camera", self.w - 172, self.h - 128)
        if power is not None:                                  # the pack: charge and the bus voltage
            pct, volts = power
            d.setColor(self.INK if pct > 20 else 0xFF6A3D)
            d.drawRectangle(self.w - 150, 28, 96, 40)
            d.fillRectangle(self.w - 54, 40, 8, 16)
            d.fillRectangle(self.w - 146, 32, int(88 * max(0, min(pct, 100)) / 100), 32)
            d.setFont("LiberationSans-Regular", 22, True)
            d.drawText(f"{pct:.0f}%  {volts:.2f} V", self.w - 270, 80)
        self.draws += 1

    def inset(self):
        """The camera's latest frame at 1/INSET, pasted into the bottom right corner."""
        if self.np is None:
            return
        cw, ch = self.cam.getWidth(), self.cam.getHeight()
        raw = self.cam.getImage()
        if not raw:
            return
        img = self.np.frombuffer(raw, dtype=self.np.uint8).reshape(ch, cw, 4)[::INSET, ::INSET]
        h, w = img.shape[:2]
        ref = self.d.imageNew(self.np.ascontiguousarray(img).tobytes(), self.d.BGRA, w, h)
        self.d.imagePaste(ref, self.w - w - 12, self.h - h - 12, False)
        self.d.imageDelete(ref)
        self.insets += 1

    def update(self, phase, t, pack=None):
        """Called after every step; redraws at most every FACE_MS, and only on a change. pack:
        (state of charge 0-1, bus volts), shown to 1 % and 0.05 V."""
        if t + 1e-9 < self.next_t:
            return
        self.next_t = t + FACE_MS / 1000.0
        mood = mood_of(phase)
        blink = (t % 3.0) < 0.15
        power = None if pack is None else (round(100 * pack[0]), round(pack[1] * 20) / 20)
        if (mood, blink, power) != self.key:
            self.key = (mood, blink, power)
            self.draw(mood, blink, power)
            self.inset_t = None
        frame = math.floor(t * 1000.0 / CAMERA_MS + 1e-6)     # a new camera frame since the last inset
        if frame != self.inset_t:
            self.inset_t = frame
            self.inset()


def trapezoid(dist, w=PROFILE_W, acc=PROFILE_A):
    """(duration, s(t)) of a rest-to-rest move of `dist` (rad, >= 0) at speed w, acceleration acc."""
    if dist <= 0.0:
        return 0.0, lambda t: 0.0
    ta = w / acc
    if dist < w * ta:                                  # triangular: never reaches w
        ta = math.sqrt(dist / acc)
        w = acc * ta
    tc = (dist - w * ta) / w
    T = 2 * ta + tc

    def s(t):
        if t <= 0:
            return 0.0
        if t < ta:
            return 0.5 * acc * t * t
        if t < ta + tc:
            return 0.5 * acc * ta * ta + w * (t - ta)
        if t < T:
            r = T - t
            return dist - 0.5 * acc * r * r
        return dist
    return T, s


class Motor:
    def __init__(self, pack_v, rated_v, no_load_rpm, stall_kgcm):
        self.args = (rated_v, no_load_rpm, stall_kgcm)
        k = pack_v / rated_v
        self.v = pack_v
        self.w0 = no_load_rpm * k * 2 * math.pi / 60
        self.stall = stall_kgcm * k * KGCM
        self.stall_a = STALL_A_RATED * k
        self.i0 = NO_LOAD_A

    def available_torque(self, u, w):
        """Torque the motor can apply at wheel speed w (rad/s) commanded u (-1..1): the DC
        line's magnitude, which the servo aims at no-load speed from either side (drives
        below it, brakes above it). Stopping (u = 0) is the full stall torque, a brake."""
        return self.stall * abs(1.0 - abs(w) / self.w0) if u else self.stall

    def current(self, w):
        f = abs(1.0 - abs(w) / self.w0)
        return self.i0 + (self.stall_a - self.i0) * f

    def at(self, v):
        """The same motor on v volts (speed, stall torque and stall current scale with v)."""
        return self if v == self.v else Motor(v, *self.args)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pack-v", type=float, default=7.2)
    p.add_argument("--rated-v", type=float, default=12.0)
    p.add_argument("--no-load-rpm", type=float, default=300.0)
    p.add_argument("--stall-kgcm", type=float, default=11.0)
    p.add_argument("--result", default=str(ROOT / ".tmp/spark_rover/drive.json"))
    p.add_argument("--quit", action="store_true", help="end the simulation once the result is written")
    p.add_argument("--raw-arms", action="store_true", help="jump each arm target instead of the design's profile")
    p.add_argument("--trace", help="also write every 5th step's pose, wheel speeds and arm angles (JSON lines)")
    p.add_argument("--no-face", action="store_true", help="leave the camera and the screen off (the A/B for their cost)")
    p.add_argument("--frames", help="save the camera's frame and the screen 0.5 s into each mood, to this directory "
                   "(evidence only: after the first Display.imageSave the rest of the run measured ~2x slower)")
    p.add_argument("--soc", type=float, default=1.0, help="the pack's state of charge at the start (0-1; 1 = just charged)")
    p.add_argument("--fixed-pack", action="store_true", help="run the motors on --pack-v, not on the battery model")
    a = p.parse_args()
    mot = Motor(a.pack_v, a.rated_v, a.no_load_rpm, a.stall_kgcm)
    bat = None
    if not a.fixed_pack:
        sys.path.insert(0, str(PRODUCT))
        import battery as battery_model             # standard library only; electrical.py is not imported
        bat = battery_model.Battery(soc=a.soc)
        bat_start = {"soc": a.soc, "v_ocv": round(bat.v_ocv, 4)}
    i_pack = [0.0]
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    wm = {t: robot.getDevice(f"wheel_{t}_motor") for t in WHEELS}
    ws = {t: robot.getDevice(f"wheel_{t}_sensor") for t in WHEELS}
    for t in WHEELS:
        wm[t].setPosition(float("inf"))
        wm[t].setVelocity(0.0)
        ws[t].enable(dt_ms)
    joints = [f"{j}_{s}" for s in ("left", "right") for j in ("shoulder", "elbow")]
    am = {j: robot.getDevice(j) for j in joints}
    asn = {j: robot.getDevice(f"{j}_sensor") for j in joints}
    for j in joints:
        asn[j].enable(dt_ms)
        am[j].setPosition(0.0)
    face = None
    if not a.no_face:
        cam = robot.getDevice("camera")
        cam.enable(CAMERA_MS)
        face = Face(robot.getDevice("display"), cam)
    frames = Path(a.frames) if a.frames else None
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
    seen = {}                       # mood -> the time it began (for --frames)
    wall = {"step_s": 0.0, "face_s": 0.0, "t0": time.perf_counter()}
    me = robot.getSelf()
    last = {t: None for t in WHEELS}
    omega = {t: 0.0 for t in WHEELS}
    t_sim = [0.0]
    rows = []

    def tick(cmd, phase):
        """One step with wheel commands cmd {tag: u}; records a row. The motors run on the bus
        voltage the battery gave under the last step's load (or on --pack-v)."""
        v_bus = a.pack_v if bat is None else bat.v_terminal - i_pack[0] * R_EXT_OHM
        mo = mot.at(v_bus)
        for t in WHEELS:
            u = cmd.get(t, 0.0)
            wm[t].setAvailableTorque(min(mo.available_torque(u, omega[t]), wm[t].getMaxTorque()))
            wm[t].setVelocity(max(-wm[t].getMaxVelocity(), min(u * mo.w0, wm[t].getMaxVelocity())))
        w0 = time.perf_counter()
        if robot.step(dt_ms) == -1:
            return False
        w1 = time.perf_counter()
        wall["step_s"] += w1 - w0
        t_sim[0] += dt
        if face is not None:
            face.update(phase, t_sim[0], None if bat is None else (bat.soc, bat.v_terminal))
            mood = mood_of(phase)
            t_in = seen.setdefault(mood, t_sim[0])
            if frames and t_in is not None and t_sim[0] - t_in >= 0.5:
                seen[mood] = None                    # once per mood
                face.cam.saveImage(str(frames / f"camera_{mood}.png"), 100)
                face.d.imageSave(None, str(frames / f"display_{mood}.png"))
            wall["face_s"] += time.perf_counter() - w1
        for t in WHEELS:
            ang = ws[t].getValue()
            omega[t] = 0.0 if last[t] is None else (ang - last[t]) / dt
            last[t] = ang
        if bat is not None:
            # the pack's load this step: each driven motor's current on its line (a braked motor
            # is shorted by its bridge and draws nothing from the pack), plus the rails
            driven = [t for t in WHEELS if cmd.get(t, 0.0)]
            case = "driving" if driven else ("arms" if phase.startswith("move") else "idle")
            i_mot = [mo.current(omega[t]) for t in driven]
            i_pack[0] = battery_model.pack_load_a(i_mot, ELECTRONICS_A[case], servo_a=SERVO_A[case], v_bus=v_bus)
            bat.step(i_pack[0], dt)
        pos = me.getPosition()
        R = me.getOrientation()
        yaw = math.atan2(R[3], R[0])
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[6]))))
        roll = math.degrees(math.atan2(R[7], R[8]))
        rows.append({"t": t_sim[0], "phase": phase, "x": pos[0], "y": pos[1], "z": pos[2], "yaw": yaw,
                     "pitch": pitch, "roll": roll, "w": dict(omega),
                     "q": {j: asn[j].getValue() for j in joints}, "v_bus": v_bus, "i_pack": i_pack[0]})
        return True

    def hold(phase, seconds, cmd=None):
        for _ in range(int(round(seconds / dt))):
            if not tick(cmd or {}, phase):
                return False
        return True

    fwd = {t: 1.0 for t in WHEELS}
    spin = {t: (-1.0 if LEFT[t] else 1.0) for t in WHEELS}     # left back, right forward: counter-clockwise
    ok = hold("settle", 1.0)
    x0, y0 = me.getPosition()[0], me.getPosition()[1]
    n = 0
    while ok:
        ok = tick(fwd, "straight")
        n += 1
        px, py = me.getPosition()[0], me.getPosition()[1]
        if math.hypot(px - x0, py - y0) >= STRAIGHT_M or n * dt > 5.0:
            break
    ok = ok and hold("stop", 1.0) and hold("spin", SPIN_S, spin) and hold("stop2", 1.0)
    wave_rows = []
    cmd_q = {j: 0.0 for j in joints}
    track = {}
    for k, (qs, qe) in enumerate(WAVE):
        if not ok:
            break
        goal = {j: math.radians(qs if j.startswith("shoulder") else qe) for j in joints}
        start = dict(cmd_q)
        far = max(abs(goal[j] - start[j]) for j in joints)
        T, s = trapezoid(far)
        worst = 0.0
        if a.raw_arms or T == 0.0:
            for j in joints:
                am[j].setPosition(goal[j])
            T = 0.0
        else:
            for i in range(int(math.ceil(T / dt))):
                f = s((i + 1) * dt) / far
                for j in joints:
                    am[j].setPosition(start[j] + f * (goal[j] - start[j]))
                ok = tick({}, f"move_{k}")
                if not ok:
                    break
                worst = max([worst] + [abs(rows[-1]["q"][j] - (start[j] + f * (goal[j] - start[j]))) for j in joints])
        cmd_q = goal
        track[k] = {"move_s": round(T, 3), "max_tracking_error_deg": round(math.degrees(worst), 3)}
        ok = ok and hold(f"wave_{k}", HOLD_S if not a.raw_arms else 1.5)
        wave_rows.append((k, qs, qe))

    def phase_rows(name):
        return [r for r in rows if r["phase"] == name]

    out = {"dt_ms": dt_ms, "pack_v": a.pack_v, "motor": {"no_load_rad_s": round(mot.w0, 4), "stall_nm": round(mot.stall, 4),
                                                        "stall_a": round(mot.stall_a, 3), "no_load_a": mot.i0}}
    st = phase_rows("straight")
    if st:
        d_tot = math.hypot(st[-1]["x"] - x0, st[-1]["y"] - y0)
        # steady speed: the last 60 % of the run (the robot accelerates in its first ~0.1 m)
        k0 = next(i for i, r in enumerate(st) if math.hypot(r["x"] - x0, r["y"] - y0) >= 0.4 * d_tot)
        w = st[k0:]
        v = math.hypot(w[-1]["x"] - w[0]["x"], w[-1]["y"] - w[0]["y"]) / (w[-1]["t"] - w[0]["t"])
        wbar = sum(abs(r["w"][t]) for r in w for t in WHEELS) / (4 * len(w))
        v_mean = sum(r["v_bus"] for r in w) / len(w)
        mo = mot.at(v_mean) if bat is not None else mot      # the motor on the window's mean bus voltage
        out["straight"] = {"pack_v_mean": round(v_mean, 4),
            "distance_m": round(d_tot, 4), "time_s": round(st[-1]["t"] - st[0]["t"] + dt, 3),
            "steady_speed_m_s": round(v, 4), "steady_window_m": [round(0.4 * d_tot, 3), round(d_tot, 3)],
            "wheel_rad_s": round(wbar, 3), "wheel_rad_s_each": {t: round(sum(abs(r["w"][t]) for r in w) / len(w), 3) for t in WHEELS},
            "effective_radius_mm": round(1000 * v / wbar, 3) if wbar else None,
            "current_a_each": {t: round(mo.current(sum(abs(r["w"][t]) for r in w) / len(w)), 3) for t in WHEELS},
            "heading_drift_deg": round(math.degrees(math.remainder(st[-1]["yaw"] - st[0]["yaw"], 2 * math.pi)), 3),
            "lateral_drift_mm": round(1000 * (st[-1]["y"] - y0), 2),
            "pitch_deg_range": [round(min(r["pitch"] for r in st), 3), round(max(r["pitch"] for r in st), 3)],
            "chassis_z_mm_range": [round(1000 * min(r["z"] for r in st), 3), round(1000 * max(r["z"] for r in st), 3)]}
    sp = phase_rows("spin")
    if sp:
        tail = [r for r in sp if r["t"] >= sp[-1]["t"] - 2.0]          # the last 2 s: steady
        yaw = [r["yaw"] for r in tail]
        turned = sum(math.remainder(b - c, 2 * math.pi) for c, b in zip(yaw[:-1], yaw[1:]))
        rate = turned / (tail[-1]["t"] - tail[0]["t"])
        we = {t: sum(abs(r["w"][t]) for r in tail) / len(tail) for t in WHEELS}
        v_mean = sum(r["v_bus"] for r in tail) / len(tail)
        mo = mot.at(v_mean) if bat is not None else mot
        cur = {t: mo.current(we[t]) for t in WHEELS}
        drift = math.hypot(tail[-1]["x"] - tail[0]["x"], tail[-1]["y"] - tail[0]["y"])
        out["spin"] = {"pack_v_mean": round(v_mean, 4),
            "yaw_rate_rad_s": round(rate, 4), "wheel_rad_s_each": {t: round(v, 3) for t, v in we.items()},
            "wheel_rad_s_mean": round(sum(we.values()) / 4, 3),
            "wheel_rad_s_peak": round(max(abs(r["w"][t]) for r in tail for t in WHEELS), 3),
            "torque_nm_each": {t: round(mo.available_torque(1, we[t]), 4) for t in WHEELS},
            "current_a_each": {t: round(c, 3) for t, c in cur.items()},
            "current_a_mean": round(sum(cur.values()) / 4, 3),
            "current_frac_of_stall_mean": round(sum(cur.values()) / 4 / mo.stall_a, 4),
            "effective_track_mm": round(2000 * (sum(we.values()) / 4) * 0.0325 / abs(rate), 1) if rate else None,
            "centre_drift_mm_in_2s": round(1000 * drift, 2),
            "window_s": 2.0}
    wv = []
    for k, qs, qe in wave_rows:
        rs = phase_rows(f"wave_{k}")
        if not rs:
            continue
        tail = [r for r in rs if r["t"] >= rs[-1]["t"] - 0.25]
        row = {"cmd_deg": {"shoulder": qs, "elbow": qe}, **track.get(k, {})}
        mv = phase_rows(f"move_{k}") or rs
        row["robot_moved_mm"] = round(1000 * math.hypot(rs[-1]["x"] - mv[0]["x"], rs[-1]["y"] - mv[0]["y"]), 3)
        row["chassis_z_dip_mm"] = round(1000 * (min(r["z"] for r in mv + rs) - rs[-1]["z"]), 3)
        for j in joints:
            tgt = qs if j.startswith("shoulder") else qe
            got = sum(r["q"][j] for r in tail) / len(tail)
            # settle time: from the command until the joint stays within 0.5 deg of where it ends
            end = math.degrees(got)
            out_at = [i for i, x in enumerate(rs) if abs(math.degrees(x["q"][j]) - end) > 0.5]
            k_in = (out_at[-1] + 1) if out_at else 0
            t_set = rs[k_in]["t"] - rs[0]["t"] + dt if k_in < len(rs) else None
            row[j] = {"settled_deg": round(end, 3), "error_deg": round(end - tgt, 3),
                      "settle_s": None if t_set is None else round(t_set, 3),
                      "ripple_deg": round(max(math.degrees(abs(r["q"][j] - got)) for r in tail), 4)}
        wv.append(row)
    if wv:
        wv_start = (phase_rows("move_0") or phase_rows("wave_0"))[0]
        errs = [abs(r[j]["error_deg"]) for r in wv for j in joints]
        out["arms"] = {"hold_s": HOLD_S, "profile": None if a.raw_arms else {
            "peak_deg_s": round(math.degrees(PROFILE_W), 1), "accel_deg_s2": round(math.degrees(PROFILE_A), 1),
            "from": "arms.WAVE: 90 deg in 0.5 s, trapezoid in thirds"}, "poses": wv, "max_abs_error_deg": round(max(errs), 3),
                       "max_settle_s": max((r[j]["settle_s"] or 0) for r in wv for j in joints)}
        out["arms"]["robot_moved_mm_while_waving"] = round(1000 * math.hypot(
            rows[-1]["x"] - wv_start["x"], rows[-1]["y"] - wv_start["y"]), 3)
    moves = {}
    for r in rows:
        mv = moves.setdefault(r["phase"], {"first": r, "last": r})
        mv["last"] = r
    out["phases"] = {k: {"t0_s": round(v["first"]["t"], 3), "t1_s": round(v["last"]["t"], 3),
                         "moved_mm": round(1000 * math.hypot(v["last"]["x"] - v["first"]["x"], v["last"]["y"] - v["first"]["y"]), 3),
                         "yaw_deg": round(math.degrees(math.remainder(v["last"]["yaw"] - v["first"]["yaw"], 2 * math.pi)), 3)}
                     for k, v in moves.items()}
    out["chassis_z_mm_settle"] = round(1000 * phase_rows("settle")[-1]["z"], 3) if phase_rows("settle") else None
    if bat is not None:
        out["battery"] = {
            "model": "spark_rover/battery.py: 2S2P Molicel P28A, its data sheet's curves (23 C, typical cell)",
            "start": bat_start, "soc_end": round(bat.soc, 6), "v_ocv_end": round(bat.v_ocv, 4),
            "v_bus_min": round(min(r["v_bus"] for r in rows), 4), "v_bus_max": round(max(r["v_bus"] for r in rows), 4),
            "i_pack_peak_a": round(max(r["i_pack"] for r in rows), 3),
            "i_pack_mean_a": round(sum(r["i_pack"] for r in rows) / len(rows), 3),
            "charge_mah": round(1000 * bat.charge_ah_used, 3), "energy_wh": round(bat.energy_wh_used, 4),
            "cut_off": bool(bat.cut_off), "loads": "electrical.operating_point: idle / arms (moves) / driving rails"}
    else:
        out["battery"] = {"model": "fixed", "pack_v": a.pack_v}
    out["complete"] = bool(ok)
    if a.trace:
        with Path(a.trace).open("w", encoding="utf-8") as fh:
            for r in rows[::5]:
                fh.write(json.dumps(r) + "\n")
    path = Path(a.result)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print("[spark_rover_drive] " + json.dumps(out), flush=True)
    timing = {"steps": len(rows), "loop_wall_s": round(time.perf_counter() - wall["t0"], 3),
              "step_wall_s": round(wall["step_s"], 3), "face_wall_s": round(wall["face_s"], 3), "face": face is not None}
    if face is not None:
        timing.update(face_draws=face.draws, insets=face.insets)
    print("[spark_rover_drive] timing " + json.dumps(timing), flush=True)
    for t in WHEELS:
        wm[t].setVelocity(0.0)
    if face is not None:
        face.draw("done", False)
    if a.quit:
        robot.simulationQuit(0)
    while robot.step(dt_ms) != -1:
        pass


if __name__ == "__main__":
    main()
