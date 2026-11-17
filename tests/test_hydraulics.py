"""Engine-free tests for omnisim.hydraulics (2026-11-08).

WHY THIS EXISTS
---------------
In the 2026-11-07 Megahex agent test every one of seven sessions wrote its own hydraulic model
before it could make a full-scale excavator-legged machine move like one. omnisim.hydraulics is
that model as a part of the controller library: cylinders, pin geometry, pumps (wear, power
limit), proportional and open-centre valves (deadband, lag, slew, flow sharing), relief, leakage
creep and stroke end stops, driving an OmniSim joint through its position servo.

These tests check the circuit against hand calculations, and drive a one-joint arm whose plant is
the engine's own servo law (torque = ke*(target - q) - kd*qdot, clamped at the available torque)
integrated in small substeps, so the Drive is exercised exactly as the engine would see it. The
engine pin is tests/test_hydraulics_engine.py.

RUN:  python -m pytest tests/test_hydraulics.py -q
"""

import importlib.util
import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_MOD = os.path.join(_HERE, os.pardir, "lib", "controller", "python", "omnisim", "hydraulics.py")
_spec = importlib.util.spec_from_file_location("_omnisim_hydraulics_under_test", _MOD)
H = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = H          # dataclasses look their module up while the class is built
_spec.loader.exec_module(H)

G = 9.81


def boom_cylinder(**kw):
    return H.Cylinder.mm(bore=63, rod=35, stroke=300, retracted=480, **kw)


# --- a fake robot whose plant is the engine's servo law ------------------------------------------
class FakeMotor:
    ROTATIONAL, LINEAR = 0, 1

    def __init__(self, effort, lo, hi, kind=0):
        self.effort, self.lo, self.hi, self.kind = effort, lo, hi, kind
        self.target, self.avail = 0.0, effort

    def getType(self):
        return self.kind

    def getMaxTorque(self):
        return self.effort

    def getMaxForce(self):
        return self.effort

    def getMinPosition(self):
        return self.lo

    def getMaxPosition(self):
        return self.hi

    def setAvailableTorque(self, t):
        self.avail = min(t, self.effort)

    setAvailableForce = setAvailableTorque

    def setPosition(self, p):
        self.target = p

    def getTargetPosition(self):
        return self.target


class FakeSensor:
    def __init__(self, arm):
        self.arm = arm

    def enable(self, ms):
        pass

    def getValue(self):
        return self.arm.q


class Arm:
    """A boom: inertia I about its hinge, a payload m at radius l, gravity torque -m g l cos(q)."""

    def __init__(self, motor, inertia=60.0, mass=200.0, radius=1.5, q0=0.0, dt=0.002, sub=40):
        self.m, self.I, self.mass, self.l = motor, inertia, mass, radius
        self.q, self.qd, self.dt, self.sub = q0, 0.0, dt, sub
        self.ke, self.kd = 10.0 * motor.effort, 0.5 * motor.effort

    def gravity(self):
        return -self.mass * G * self.l * math.cos(self.q)

    def step(self):
        h = self.dt / self.sub
        for _ in range(self.sub):
            tau = self.ke * (self.m.target - self.q) - self.kd * self.qd
            tau = max(-self.m.avail, min(self.m.avail, tau))
            self.qd += (tau + self.gravity()) / self.I * h
            self.q += self.qd * h
            if not self.m.lo <= self.q <= self.m.hi:     # the joint's hard limits, as in the engine
                self.q = max(self.m.lo, min(self.m.hi, self.q))
                self.qd = 0.0


class FakeRobot:
    def __init__(self, devices, dt_ms=2):
        self.devices, self.dt_ms = devices, dt_ms

    def getBasicTimeStep(self):
        return float(self.dt_ms)

    def getDevice(self, name):
        return self.devices.get(name)


def make_rig(mass=200.0, valve=None, pump=None, cyl=None, effort_factor=10.0, load_holding=True):
    cyl = cyl or boom_cylinder()
    mount = H.Mount.fit(cyl, q_lo=-0.5, q_hi=0.8, ra=0.30)
    pump = pump or H.Pump(flow_lpm=10.5, relief_bar=160)
    circ = H.Circuit(pump, valve or H.Valve.proportional())
    circ.add("boom", cyl, mount, load_holding=load_holding)
    # stiffness: effort = factor x the cylinder's largest relief torque over the range
    cap = max(pump.relief * cyl.area_bore * mount.arm(q) for q in (-0.5, 0.0, 0.4, 0.8))
    motor = FakeMotor(effort_factor * cap, -0.5, 0.8)
    arm = Arm(motor, mass=mass)
    robot = FakeRobot({"m": motor, "s": FakeSensor(arm)})
    drive = H.Drive(robot, [circ], {"boom": ("m", "s")})
    return drive, arm, circ, mount, cyl, pump


def run(drive, arm, want, seconds, settle=0.5):
    """Settle with valves closed, then request `want` (a number or a function of q) for `seconds`."""
    rep = None
    for _ in range(int(settle / arm.dt)):
        rep = drive.step({})
        arm.step()
    hist = []
    for _ in range(int(seconds / arm.dt)):
        v = want(arm.q) if callable(want) else want
        rep = drive.step({"boom": v})
        arm.step()
        hist.append((arm.q, rep["boom"]))
    return rep, hist


# --- parts ----------------------------------------------------------------------------------------
class Parts(unittest.TestCase):
    def test_cylinder_areas_and_units(self):
        c = boom_cylinder(leak_lpm_at_100bar=0.1)
        self.assertAlmostEqual(c.area_bore, math.pi * 0.063 ** 2 / 4, places=9)
        self.assertAlmostEqual(c.area_annulus, math.pi * (0.063 ** 2 - 0.035 ** 2) / 4, places=9)
        self.assertAlmostEqual(c.extended, 0.78)
        self.assertAlmostEqual(c.leak * 100 * H.BAR, 0.1 * H.LPM, places=12)
        with self.assertRaises(ValueError):
            H.Cylinder(0.05, 0.06, 0.3, 0.4)

    def test_pin_mount_arm_is_the_derivative_of_length(self):
        m = H.Mount.pins(0.3, 0.6, 1.0, sign=1)
        for q in (-0.4, 0.0, 0.5):
            num = (m.length(q + 1e-6) - m.length(q - 1e-6)) / 2e-6
            self.assertAlmostEqual(m.dLdq(q), num, places=6)
        mneg = H.Mount.pins(0.3, 0.6, 1.0, sign=-1)
        self.assertLess(mneg.dLdq(0.2), 0.0)

    def test_fit_uses_the_stroke_over_the_range(self):
        c = boom_cylinder()
        for sign in (1, -1):
            m = H.Mount.fit(c, -0.5, 0.8, ra=0.3, sign=sign)
            short, long_ = c.retracted + 0.02 * c.stroke, c.extended - 0.02 * c.stroke
            a, b = (m.length(-0.5), m.length(0.8)) if sign > 0 else (m.length(0.8), m.length(-0.5))
            self.assertAlmostEqual(a, short, places=4)
            self.assertAlmostEqual(b, long_, places=4)
            self.assertTrue(m.dLdq(0.1) * sign > 0)
        with self.assertRaises(ValueError):
            H.Mount.fit(c, 0.0, 3.5, ra=0.3)

    def test_pump_wear_and_power_limit(self):
        p = H.Pump(flow_lpm=60, relief_bar=200, wear=0.2)
        self.assertAlmostEqual(p.flow_at(0.0), 60 * H.LPM)
        self.assertAlmostEqual(p.flow_at(200 * H.BAR), 48 * H.LPM)
        pw = H.Pump(flow_lpm=60, relief_bar=250, power_kw=10, efficiency=1.0)
        self.assertAlmostEqual(pw.flow_at(50 * H.BAR), 60 * H.LPM)          # 10 kW allows 120 L/min
        self.assertAlmostEqual(pw.flow_at(200 * H.BAR), 10e3 / 200e5)       # power-limited


# --- the circuit ----------------------------------------------------------------------------------
def two_on_one_pump(valve):
    c = H.Circuit(H.Pump(flow_lpm=20, relief_bar=150), valve)
    cyl = boom_cylinder()
    c.add("a", cyl, H.Mount.linear(0.6)).add("b", cyl, H.Mount.linear(0.6))
    states = [H.Circuit.state(act, 0.0) for act in c.actuators]
    return c, cyl, states


class CircuitPhysics(unittest.TestCase):
    def settle(self, c, states, want, tau, n=200, dt=0.002):
        out = None
        for _ in range(n):
            out = c.step(dt, states, want, tau)
        return out

    def test_proportional_shares_a_short_pump_in_proportion(self):
        c, cyl, states = two_on_one_pump(H.Valve.proportional())
        q_pump = 20 * H.LPM
        v_full = q_pump / cyl.area_bore             # one cylinder alone could take the whole pump
        got = self.settle(c, states, [v_full, 0.5 * v_full], [1000.0, 20000.0])
        self.assertAlmostEqual(got[0] / got[1], 2.0, places=2)           # path kept: ratio preserved
        self.assertAlmostEqual(c.flow, q_pump, delta=0.02 * q_pump)
        self.assertLess(got[0], v_full)

    def test_open_centre_feeds_the_lighter_load_first(self):
        c, cyl, states = two_on_one_pump(H.Valve.open_centre())
        v = 20 * H.LPM / cyl.area_bore
        light, heavy = 500.0, 0.8 * 150 * H.BAR * cyl.area_bore
        got = self.settle(c, states, [v, v], [light, heavy], n=400)
        self.assertGreater(got[0], 3.0 * max(got[1], 1e-9))
        same = self.settle(c, states, [v, v], [heavy, heavy], n=400)
        self.assertAlmostEqual(same[0], same[1], places=6)

    def test_load_above_relief_gets_no_flow(self):
        c, cyl, states = two_on_one_pump(H.Valve.proportional())
        over = 1.1 * 150 * H.BAR * cyl.area_bore
        got = self.settle(c, states, [0.05, 0.05], [over, 1000.0])
        self.assertEqual(got[0], 0.0)
        self.assertGreater(got[1], 0.0)
        self.assertTrue(c.at_relief)

    def test_deadband_passes_nothing_until_the_spool_clears_it(self):
        c, cyl, states = two_on_one_pump(H.Valve.open_centre())
        small = 0.05 * 20 * H.LPM / cyl.area_bore      # 5 % of the pump: inside the 15 % overlap
        got = self.settle(c, states, [small, 0.0], [1000.0, 0.0], n=400)
        self.assertEqual(got[0], 0.0)

    def test_spool_lag_delays_the_flow(self):
        c, cyl, states = two_on_one_pump(H.Valve.open_centre(lag_s=0.2))
        v = 20 * H.LPM / cyl.area_bore
        first = c.step(0.002, states, [v, 0.0], [1000.0, 0.0])
        self.assertEqual(first[0], 0.0)
        later = self.settle(c, states, [v, 0.0], [1000.0, 0.0], n=500)
        self.assertGreater(later[0], 0.5 * v)

    def test_creep_rate_is_leak_times_pressure_over_area(self):
        cyl = boom_cylinder(leak_lpm_at_100bar=0.2)
        c = H.Circuit(H.Pump(10, 160), H.Valve.proportional(spool_leak=cyl.leak))
        c.add("held", cyl, H.Mount.linear(0.6), load_holding=True)
        c.add("open", cyl, H.Mount.linear(0.6), load_holding=False)
        s = H.Circuit.state(c.actuators[0], 0.0)
        tau = 100 * H.BAR * cyl.area_bore             # a 100 bar load
        held, leaky = c.creep(0, s, tau), c.creep(1, s, tau)
        self.assertAlmostEqual(held, -0.2 * H.LPM / cyl.area_bore, places=9)
        self.assertAlmostEqual(leaky, 2 * held, places=9)


# --- the drive on a one-joint arm under the engine's servo law -------------------------------------
class DriveOnAnArm(unittest.TestCase):
    def test_estimated_torque_holds_the_payload(self):
        drive, arm, *_ = make_rig(mass=200.0)
        rep, _ = run(drive, arm, 0.0, 0.2)
        self.assertAlmostEqual(rep["boom"]["torque"], -arm.gravity(), delta=0.02 * abs(arm.gravity()))
        self.assertLess(rep["boom"]["use"], 1.0)

    def test_speed_is_flow_limited(self):
        drive, arm, circ, mount, cyl, pump = make_rig(mass=200.0)
        rep, hist = run(drive, arm, 2.0, 1.0)         # ask for far more than the pump can give
        q_mid = hist[len(hist) // 2][0]
        expected = pump.flow_at(circ.pressure) / (cyl.area_bore * mount.arm(q_mid))
        mid = hist[len(hist) // 2][1]
        self.assertAlmostEqual(mid["delivered"], expected, delta=0.05 * expected)
        self.assertAlmostEqual(mid["achieved"], mid["delivered"], delta=0.1 * expected)
        self.assertLess(mid["delivered"], 2.0)

    def test_reaches_a_target_and_stops_at_the_stroke_end(self):
        drive, arm, *_ = make_rig(mass=200.0)
        run(drive, arm, lambda q: H.velocity_toward(q, 0.5, gain=3.0), 6.0)
        self.assertAlmostEqual(arm.q, 0.5, delta=0.02)
        drive2, arm2, circ2, mount2, cyl2, _ = make_rig(mass=100.0)
        run(drive2, arm2, 1.0, 8.0)
        end = mount2.q_at_length(cyl2.extended, -0.5, 0.8)     # None: fit() leaves 2 % of stroke spare
        self.assertLessEqual(arm2.q, (end if end is not None else 0.8) + 0.02)
        self.assertGreater(arm2.q, 0.75)

    def test_a_load_above_relief_pushes_the_boom_down(self):
        drive, arm, *_ = make_rig(mass=1100.0)         # ~16 kN m at q=0 against a ~12 kN m relief
        rep, hist = run(drive, arm, 0.3, 0.3, settle=0.0)
        q_mid, mid = hist[len(hist) // 2]
        self.assertLess(arm.q, -0.02)                  # asked to rise, it is pushed down
        self.assertGreater(mid["use"], 0.95)
        self.assertTrue(mid["at_relief"])
        self.assertEqual(mid["delivered"], 0.0)

    def test_leakage_lets_a_held_load_creep(self):
        tight, arm_t, *_ = make_rig(mass=300.0)
        run(tight, arm_t, 0.0, 0.0)                    # settle: the servo takes up the load
        qt0 = arm_t.q
        run(tight, arm_t, 0.0, 5.0, settle=0.0)
        self.assertAlmostEqual(arm_t.q, qt0, delta=2e-4)   # no leak: it holds
        cyl = boom_cylinder(leak_lpm_at_100bar=0.5)
        leaky, arm_l, circ, mount, *_ = make_rig(mass=300.0, cyl=cyl)
        rep, _ = run(leaky, arm_l, 0.0, 0.0)
        q0, creep = arm_l.q, rep["boom"]["creep"]
        rep, _ = run(leaky, arm_l, 0.0, 5.0, settle=0.0)
        self.assertLess(creep, 0.0)
        self.assertAlmostEqual(arm_l.q - q0, 5.0 * creep, delta=0.15 * abs(5.0 * creep))

    def test_worn_pump_is_slower(self):
        new_drive, new_arm, *_ = make_rig(mass=300.0)
        _, h_new = run(new_drive, new_arm, 2.0, 0.6)
        worn_pump = H.Pump(flow_lpm=10.5, relief_bar=160, wear=0.3)
        worn_drive, worn_arm, *_ = make_rig(mass=300.0, pump=worn_pump)
        _, h_worn = run(worn_drive, worn_arm, 2.0, 0.6)
        v_new, v_worn = h_new[-1][1]["delivered"], h_worn[-1][1]["delivered"]
        p = h_worn[-1][1]["pressure_bar"] * H.BAR
        self.assertAlmostEqual(v_worn / v_new, 1.0 - 0.3 * p / worn_pump.relief, delta=0.02)


if __name__ == "__main__":
    unittest.main()
