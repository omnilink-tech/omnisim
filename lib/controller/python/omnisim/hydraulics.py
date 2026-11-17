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

"""Hydraulic actuators for OmniSim joints: cylinders, pumps, valves and the imperfections of real ones.

A physics engine moves a joint with a motor. A hydraulic machine moves it with a cylinder fed by a
pump through a valve, and that changes what the joint can do:

- its SPEED is set by oil flow: the pump's flow, shared between every cylinder on it, over the
  piston area on the pressure side, through the cylinder's moment arm about the joint;
- its FORCE is capped by the relief valve: relief pressure times that area times the arm, and the
  arm changes with the joint angle;
- a push (bore area) is stronger and slower than a pull (annulus area);
- under a load above relief the cylinder is pushed back;
- real parts are imperfect: valves have deadband, lag and slew; an open-centre valve sends oil to
  the LIGHTEST load first; pumps lose flow when worn and are capped by their engine's power;
  cylinders and spools leak, so a held load creeps; cylinders stop at the ends of their stroke.

This module models all of that and drives an OmniSim joint with it. It runs inside an ordinary
controller and needs no engine change:

    from omnisim import Robot
    from omnisim.hydraulics import Cylinder, Mount, Pump, Valve, Circuit, Drive

    robot = Robot()
    dt = robot.getBasicTimeStep() / 1000.0
    boom = Cylinder.mm(bore=63, rod=35, stroke=300, retracted=480)
    circuit = Circuit(Pump(flow_lpm=10.5, relief_bar=160), Valve.proportional())
    circuit.add("boom", boom, Mount.fit(boom, q_lo=-0.6, q_hi=0.9, ra=0.35))
    drive = Drive(robot, [circuit], {"boom": ("boom_motor", "boom_sensor")})
    while robot.step(int(dt * 1000)) != -1:
        report = drive.step({"boom": 0.3})      # ask for 0.3 rad/s; read what it delivered
        # report["boom"]: requested, delivered, q, torque, cap, use, pressure_bar, at_relief, ...

HOW IT DRIVES THE JOINT. The engine's joint is a position servo: torque = ke*(target - q) -
kd*qdot, clamped at the motor's available torque. Every step the drive (1) estimates the torque the
servo made last step from that law, which is the cylinder's load; (2) asks the circuit what joint
speed the oil can deliver against that load; (3) moves the servo's target at that speed and caps
the servo at the relief torque. A load above relief therefore pushes the joint back, a lack of flow
slows it, and a closed valve with a leak lets it creep. The servo law's gains come from the motor
(ke = 10 x effort, kd = 0.5 x effort; a short slider is floored at effort / travel), so set the
motor's maxTorque (maxForce) to SEVERAL TIMES the cylinder's relief torque: it is the joint's
stiffness, not its strength. At maxTorque = 10 x relief torque a hinge holding a full relief load
deflects 0.01 rad; the drive caps the strength at relief itself.

WHAT IT DOES NOT MODEL. Oil compressibility and hose volume (beyond the valve's lag), pressure
dynamics inside a step, line losses, heat, cavitation and pressure spikes: the circuit is
quasi-static each control step. Every default number below is a typical value, not a measurement
of any particular part: give your own parts' numbers.

Torque feedback is not measured on the Newton backend (Motor.getTorqueFeedback reads 0), which is
why the load is estimated from the servo law; the estimate is exact while the servo is within its
cap and the gains are the ones the engine used (pass ke/kd to Drive if you override them).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

BAR = 1.0e5                 # Pa
PSI = 6894.757              # Pa
LPM = 1.0 / 60000.0         # m^3/s per litre per minute

SIDE_HYSTERESIS = 0.05      # a load must reverse by 5 % of the cap before the cap changes side
CAP_PUSH_BAND = 0.02        # re-send a cap to the engine only when it moved by more than 2 %
RELIEF_TOL = 0.02           # a load within 2 % of relief counts as at relief (the pushed cap's band)


# --- parts --------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Cylinder:
    """A double-acting single-rod cylinder (SI units: metres).

    bore, rod: piston and rod diameters. stroke: travel. retracted: pin-to-pin length fully
    retracted. leak: internal leakage across the piston, m^3/s per Pa of load pressure (a worn
    seal leaks more); it lets a held load creep even behind a load-holding valve."""
    bore: float
    rod: float
    stroke: float
    retracted: float
    leak: float = 0.0

    def __post_init__(self):
        if not (self.bore > 0 and 0 <= self.rod < self.bore and self.stroke > 0 and self.retracted > 0):
            raise ValueError(f"cylinder needs bore > rod >= 0, stroke > 0, retracted > 0: {self}")

    @classmethod
    def mm(cls, bore: float, rod: float, stroke: float, retracted: float,
           leak_lpm_at_100bar: float = 0.0) -> "Cylinder":
        """Sizes in millimetres, leakage as litres per minute at a 100 bar load."""
        return cls(bore / 1000.0, rod / 1000.0, stroke / 1000.0, retracted / 1000.0,
                   leak=leak_lpm_at_100bar * LPM / (100.0 * BAR))

    @property
    def area_bore(self) -> float:
        return math.pi * self.bore ** 2 / 4.0

    @property
    def area_annulus(self) -> float:
        return math.pi * (self.bore ** 2 - self.rod ** 2) / 4.0

    @property
    def extended(self) -> float:
        return self.retracted + self.stroke

    def push_force(self, pressure: float) -> float:
        return pressure * self.area_bore

    def pull_force(self, pressure: float) -> float:
        return pressure * self.area_annulus


class Mount:
    """How a cylinder's pin-to-pin length L depends on its joint's position q.

    Mount.linear(): a slider driven straight (L = L0 + sign*q, arm 1).
    Mount.pins(): a hinge with the cylinder between a pin on the parent at radius ra and a pin on
        the child at radius rb: L^2 = ra^2 + rb^2 - 2 ra rb cos(phi), phi = phi0 + sign*q.
    Mount.fit(): Mount.pins() solved so the joint's range [q_lo, q_hi] uses the cylinder's stroke.
    Mount(length_fn): any other linkage, as a function q -> L."""

    def __init__(self, length_fn: Callable[[float], float], kind: str = "custom", **params):
        self._L = length_fn
        self.kind = kind
        self.params = params

    def length(self, q: float) -> float:
        return self._L(q)

    def dLdq(self, q: float, eps: float = 1e-6) -> float:
        if self.kind == "linear":
            return self.params["sign"]
        if self.kind == "pins":
            ra, rb, phi0, sign = (self.params[k] for k in ("ra", "rb", "phi0", "sign"))
            phi = phi0 + sign * q
            L = self.length(q)
            return sign * ra * rb * math.sin(phi) / max(L, 1e-12)
        return (self._L(q + eps) - self._L(q - eps)) / (2 * eps)

    def arm(self, q: float) -> float:
        """The cylinder's moment arm |dL/dq| (m for a hinge; 1 for a direct slider)."""
        return abs(self.dLdq(q))

    @classmethod
    def linear(cls, length_at_zero: float, sign: float = 1.0) -> "Mount":
        s = 1.0 if sign >= 0 else -1.0
        return cls(lambda q: length_at_zero + s * q, "linear", L0=length_at_zero, sign=s)

    @classmethod
    def pins(cls, ra: float, rb: float, phi0: float, sign: float = 1.0) -> "Mount":
        s = 1.0 if sign >= 0 else -1.0

        def L(q):
            phi = phi0 + s * q
            return math.sqrt(max(ra * ra + rb * rb - 2 * ra * rb * math.cos(phi), 0.0))
        return cls(L, "pins", ra=ra, rb=rb, phi0=phi0, sign=s)

    @classmethod
    def fit(cls, cylinder: Cylinder, q_lo: float, q_hi: float, ra: float, sign: float = 1.0,
            margin: float = 0.02) -> "Mount":
        """Pins for a hinge range: the cylinder is `margin` of its stroke short of fully retracted at
        one end of [q_lo, q_hi] and as short of fully extended at the other. sign=+1: it extends as q
        rises (a boom lift cylinder); -1: as q falls. Raises if no pin placement fits."""
        if not q_hi > q_lo:
            raise ValueError("fit() needs q_hi > q_lo")
        span = q_hi - q_lo
        if span >= math.pi:
            raise ValueError("one cylinder cannot turn a joint through 180 degrees or more")
        L_short = cylinder.retracted + margin * cylinder.stroke
        L_long = cylinder.extended - margin * cylinder.stroke

        def ends(rb):
            """The phi at the short end, and the length span - (L_long - L_short) it reaches."""
            c = (ra * ra + rb * rb - L_short * L_short) / (2 * ra * rb)
            if not -1.0 <= c <= 1.0:
                return None
            phi_lo = math.acos(c)
            if phi_lo + span > math.pi:
                return None
            L_hi = math.sqrt(ra * ra + rb * rb - 2 * ra * rb * math.cos(phi_lo + span))
            return phi_lo, L_hi - L_long

        lo_rb, hi_rb = abs(L_short - ra) * 1.0001 + 1e-9, (L_short + ra) * 0.9999
        best = None
        n = 400
        prev = None
        for i in range(n + 1):
            rb = lo_rb + (hi_rb - lo_rb) * i / n
            e = ends(rb)
            if e is None:
                prev = None
                continue
            if prev is not None and (prev[1][1] <= 0.0) != (e[1] <= 0.0):
                a, b = prev[0], rb
                for _ in range(60):
                    m = (a + b) / 2
                    em = ends(m)
                    if em is None:
                        break
                    if (em[1] <= 0.0) == (prev[1][1] <= 0.0):
                        a = m
                    else:
                        b = m
                rb_sol = (a + b) / 2
                es = ends(rb_sol)
                if es is not None:
                    best = (rb_sol, es[0])
                    break
            prev = (rb, e)
        if best is None:
            raise ValueError(f"no pin placement at ra={ra} m fits a {span:.3f} rad range to this "
                             f"cylinder's stroke; try another ra or a longer stroke")
        rb, phi_lo = best
        s = 1.0 if sign >= 0 else -1.0
        # phi grows with q when sign=+1 (L grows); with sign=-1, phi at q_hi is phi_lo
        phi0 = phi_lo - q_lo if s > 0 else phi_lo + q_hi
        return cls.pins(ra, rb, phi0, s)

    def q_at_length(self, L: float, q_lo: float, q_hi: float) -> Optional[float]:
        """The joint position in [q_lo, q_hi] where the cylinder is L long (None if not there)."""
        f_lo, f_hi = self.length(q_lo) - L, self.length(q_hi) - L
        if f_lo == 0.0:
            return q_lo
        if (f_lo > 0) == (f_hi > 0):
            return None
        a, b = q_lo, q_hi
        for _ in range(80):
            m = (a + b) / 2
            if (self.length(m) - L > 0) == (f_lo > 0):
                a = m
            else:
                b = m
        return (a + b) / 2


@dataclass
class Pump:
    """A fixed-displacement pump with a relief valve.

    flow_lpm: flow at no load. relief_bar: the relief valve's cracking pressure (the circuit's
    maximum). power_kw: the engine's power, if it limits flow at high pressure (Q <= eff*P/p).
    wear: the fraction of flow lost to internal pump leakage at relief pressure (0 new, 0.2 worn);
    the loss grows linearly with pressure."""
    flow_lpm: float
    relief_bar: float
    power_kw: Optional[float] = None
    efficiency: float = 0.85
    wear: float = 0.0

    @property
    def relief(self) -> float:
        return self.relief_bar * BAR

    def flow_at(self, pressure: float) -> float:
        """Delivered flow (m^3/s) at a pressure (Pa)."""
        q = self.flow_lpm * LPM * (1.0 - self.wear * min(max(pressure, 0.0) / self.relief, 1.0))
        if self.power_kw and pressure > 0:
            q = min(q, self.efficiency * self.power_kw * 1000.0 / pressure)
        return max(q, 0.0)


@dataclass
class Valve:
    """A directional valve section per cylinder, all on one pump.

    kind "proportional": electro-hydraulic, pressure-compensated: each spool meters the flow it is
        asked for whatever its load, and a controller in front scales every request by the same
        factor when together they ask for more than the pump gives (the machine slows down but stays
        on its path).
    kind "open_centre": a manual open-centre parallel sectional valve (levers, maybe pushed by hobby
        servos): the spools are orifices, so flow goes where the pressure is LOWEST: a lightly loaded
        cylinder takes the flow and a loaded one waits. The open centre bypasses flow to tank until
        the spools close it.
    deadband: spool overlap, the fraction of travel that passes no flow. lag_s, slew_per_s: how the
    spool follows its command. comp_deadband: how much of the deadband the controller knows and
    compensates. spool_leak: leakage past a closed spool, m^3/s per Pa (a held load creeps through
    it unless the joint has a load-holding valve)."""
    kind: str = "proportional"
    deadband: float = 0.02
    lag_s: float = 0.03
    slew_per_s: float = 20.0
    comp_deadband: float = 0.02
    dp_full_bar: float = 10.0        # open_centre: drop across a full-open spool at full pump flow
    bypass_dp_bar: float = 4.0       # open_centre: drop across the open centre at full pump flow
    spool_leak: float = 0.0

    @classmethod
    def proportional(cls, **kw) -> "Valve":
        return cls("proportional", **kw)

    @classmethod
    def open_centre(cls, **kw) -> "Valve":
        """Typical cheap sectional valve worked through hobby servos (ASSUMED values: 15 % overlap,
        ~0.12 s lag, 5 travels/s slew, no deadband compensation)."""
        base = dict(deadband=0.15, lag_s=0.12, slew_per_s=5.0, comp_deadband=0.0)
        base.update(kw)
        return cls("open_centre", **base)


# --- one circuit: a pump, a valve bank, its cylinders ------------------------------------------
@dataclass
class _Actuator:
    name: str
    cylinder: Cylinder
    mount: Mount
    load_holding: bool
    spool: float = 0.0
    side: int = 1


class Circuit:
    """One pump and one valve bank feeding several cylinders. Quasi-static each control step."""

    def __init__(self, pump: Pump, valve: Optional[Valve] = None):
        self.pump = pump
        self.valve = valve or Valve.proportional()
        self.actuators: List[_Actuator] = []
        self.pressure = 0.0          # Pa, last step
        self.at_relief = False
        self.flow = 0.0              # m^3/s delivered to cylinders, last step

    def add(self, name: str, cylinder: Cylinder, mount: Mount, load_holding: bool = True) -> "Circuit":
        """Add a cylinder. load_holding: the joint has a counterbalance / pilot-operated check valve,
        so it does not creep through its spool (it still creeps by the cylinder's own leak)."""
        if any(a.name == name for a in self.actuators):
            raise ValueError(f"duplicate actuator name {name!r}")
        self.actuators.append(_Actuator(name, cylinder, mount, load_holding))
        return self

    def names(self) -> List[str]:
        return [a.name for a in self.actuators]

    # geometry at a joint position
    @staticmethod
    def state(act: _Actuator, q: float) -> dict:
        """Length, arm, and the area that pushes the joint toward +q / -q at position q."""
        dLdq = act.mount.dLdq(q)
        c = act.cylinder
        if dLdq >= 0:            # extending turns +q
            a_pos, a_neg = c.area_bore, c.area_annulus
        else:
            a_pos, a_neg = c.area_annulus, c.area_bore
        return {"length": act.mount.length(q), "dLdq": dLdq, "arm": abs(dLdq),
                "area_pos": a_pos, "area_neg": a_neg}

    def caps(self, states: Sequence[dict], tau: Sequence[float]) -> List[float]:
        """Each joint's torque cap: relief times the area that holds its present load, times its arm.
        The side follows the load with hysteresis (a joint carrying almost nothing would otherwise
        flip between the bore and annulus caps every step)."""
        out = []
        p = self.pump.relief
        for act, s, t in zip(self.actuators, states, tau):
            cap_pos, cap_neg = p * s["area_pos"] * s["arm"], p * s["area_neg"] * s["arm"]
            if act.side > 0 and t < -SIDE_HYSTERESIS * cap_neg:
                act.side = -1
            elif act.side < 0 and t > SIDE_HYSTERESIS * cap_pos:
                act.side = 1
            out.append(cap_pos if act.side > 0 else cap_neg)
        return out

    def step(self, dt: float, states: Sequence[dict], qdot_want: Sequence[float],
             tau: Sequence[float]) -> List[float]:
        """The joint velocities the circuit delivers this step.

        states: Circuit.state() of each actuator; qdot_want: requested joint velocities; tau: the
        torque each actuator is making (+ along +q), which sets its load pressure."""
        n = len(self.actuators)
        v = self.valve
        q0 = self.pump.flow_at(0.0)
        need = []
        for s, w in zip(states, qdot_want):
            area = s["area_pos"] if w >= 0 else s["area_neg"]
            need.append(abs(w) * s["arm"] * area)
        if v.kind == "proportional":
            tot = sum(need)
            if tot > q0 > 0:
                need = [x * q0 / tot for x in need]
        # spool commands, then the spools follow with lag and slew
        a = min(1.0, dt / max(v.lag_s, 1e-6))
        step_max = v.slew_per_s * dt
        for act, x, w in zip(self.actuators, need, qdot_want):
            u = min(x / q0, 1.0) if q0 > 0 else 0.0
            travel = 0.0 if u <= 1e-9 else v.comp_deadband + u * (1.0 - v.comp_deadband)
            cmd = math.copysign(travel, w)
            target = act.spool + (cmd - act.spool) * a
            act.spool += max(-step_max, min(step_max, target - act.spool))
        opening = [max(0.0, abs(act.spool) - v.deadband) / (1.0 - v.deadband) for act in self.actuators]
        # the pressure each cylinder must beat to move the way its spool points
        p_load, areas, sgns = [], [], []
        for act, s, t in zip(self.actuators, states, tau):
            sg = 1.0 if act.spool >= 0 else -1.0
            area = s["area_pos"] if sg > 0 else s["area_neg"]
            p_load.append(max(0.0, t * sg) / max(area * s["arm"], 1e-12))
            areas.append(area)
            sgns.append(sg)
        flows = self._flows(opening, p_load)
        self.flow = sum(flows)
        return [sg * f / max(ar, 1e-12) / max(s["arm"], 1e-9)
                for sg, f, ar, s in zip(sgns, flows, areas, states)] if n else []

    def _flows(self, opening: Sequence[float], p_load: Sequence[float]) -> List[float]:
        relief = self.pump.relief
        n = len(opening)
        if self.valve.kind == "proportional":
            # pressure-compensated: the pump runs at the highest working load's pressure
            limit = relief * (1.0 - RELIEF_TOL)
            work = [j for j in range(n) if opening[j] > 0 and p_load[j] < limit]
            stalled = [j for j in range(n) if opening[j] > 0 and p_load[j] >= limit]
            P = relief if stalled else max([p_load[j] for j in work] or [0.0])
            Q = self.pump.flow_at(P)
            want = [opening[j] * Q if j in work else 0.0 for j in range(n)]
            tot = sum(want)
            k = 1.0 if tot <= Q or tot <= 0 else Q / tot
            self.pressure = P
            self.at_relief = bool(stalled)
            return [w * k for w in want]
        # open centre: orifices in parallel plus the centre to tank; find P with sum Q_j(P) = Q(P)
        Qn = self.pump.flow_at(0.0)
        K = Qn / math.sqrt(self.valve.dp_full_bar * BAR)
        Kc = Qn / math.sqrt(self.valve.bypass_dp_bar * BAR)
        centre = Kc * max(0.0, 1.0 - max(list(opening) + [0.0]))

        def excess(P):
            s = centre * math.sqrt(max(P, 0.0))
            for j in range(n):
                if opening[j] > 0:
                    s += K * opening[j] * math.sqrt(max(P - p_load[j], 0.0))
            return s - self.pump.flow_at(P)
        if excess(relief) <= 0:            # the circuit cannot take the pump's flow below relief
            P, self.at_relief = relief, True
        else:
            lo, hi = 0.0, relief
            for _ in range(50):
                mid = (lo + hi) / 2
                if excess(mid) < 0:
                    lo = mid
                else:
                    hi = mid
            P, self.at_relief = hi, False
        self.pressure = P
        return [K * opening[j] * math.sqrt(max(P - p_load[j], 0.0)) if opening[j] > 0 else 0.0
                for j in range(n)]

    def creep(self, act_index: int, state: dict, tau: float) -> float:
        """Joint velocity of leakage under load (in the load's direction, opposite tau)."""
        act = self.actuators[act_index]
        if abs(tau) <= 0.0 or state["arm"] <= 1e-9:
            return 0.0
        area = state["area_pos"] if tau >= 0 else state["area_neg"]
        p = abs(tau) / (area * state["arm"])
        leak = act.cylinder.leak + (0.0 if act.load_holding else self.valve.spool_leak)
        return -math.copysign(leak * p / area / state["arm"], tau)


# --- the engine side ---------------------------------------------------------------------------
def servo_gains(motor) -> Tuple[float, float]:
    """(ke, kd) of the engine's position servo on this motor, mirroring the Newton runtime:
    ke = 10 x effort, kd = 0.5 x effort; a LINEAR motor with a finite range shorter than 0.1 m is
    floored at ke = effort / travel (kd scaled alike) unless OMNISIM_NEWTON_SLIDER_GAIN_FLOOR=0.
    OMNISIM_NEWTON_TARGET_KE / _KD override every joint, as in the runtime."""
    import os
    linear = motor.getType() == getattr(motor, "LINEAR", 1)
    effort = motor.getMaxForce() if linear else motor.getMaxTorque()
    ke, kd = 10.0 * effort, 0.5 * effort
    if linear and os.environ.get("OMNISIM_NEWTON_SLIDER_GAIN_FLOOR", "1").strip().lower() not in (
            "0", "false", "off", "no"):
        lo, hi = motor.getMinPosition(), motor.getMaxPosition()
        travel = hi - lo
        if effort > 0 and travel > 0 and max(abs(lo), abs(hi)) < 1.0e6 and effort / travel > ke:
            scale = (effort / travel) / ke
            ke, kd = effort / travel, kd * scale
    for var, idx in (("OMNISIM_NEWTON_TARGET_KE", 0), ("OMNISIM_NEWTON_TARGET_KD", 1)):
        raw = os.environ.get(var)
        if raw:
            try:
                val = float(raw)
                ke, kd = (val, kd) if idx == 0 else (ke, val)
            except ValueError:
                pass
    return ke, kd


@dataclass
class _Bound:
    circuit: Circuit
    index: int
    motor: object
    sensor: object
    ke: float
    kd: float
    q_lo: float
    q_hi: float
    q_cmd: float = 0.0
    q_prev: Optional[float] = None
    cap: float = 0.0
    pushed: float = 0.0
    tau: float = 0.0
    started: bool = False
    extra: Dict[str, float] = field(default_factory=dict)


class Drive:
    """Binds circuits' cylinders to an OmniSim robot's motors and runs them every control step.

    joints: {actuator name: (motor name, position-sensor name)}; the names match Circuit.add().
    gains: optional {actuator name: (ke, kd)} when the engine's servo gains are not the defaults.
    The position sensor must be enabled at the control step (Drive enables it)."""

    def __init__(self, robot, circuits: Sequence[Circuit], joints: Dict[str, Tuple[str, str]],
                 gains: Optional[Dict[str, Tuple[float, float]]] = None, dt: Optional[float] = None):
        self.robot = robot
        self.circuits = list(circuits)
        self.dt = dt if dt is not None else robot.getBasicTimeStep() / 1000.0
        step_ms = max(1, int(round(self.dt * 1000.0)))
        self.joints: Dict[str, _Bound] = {}
        for c in self.circuits:
            for i, act in enumerate(c.actuators):
                if act.name not in joints:
                    raise KeyError(f"no motor/sensor given for actuator {act.name!r}")
                m_name, s_name = joints[act.name]
                motor, sensor = robot.getDevice(m_name), robot.getDevice(s_name)
                if motor is None or sensor is None:
                    raise KeyError(f"{act.name}: device {m_name!r} or {s_name!r} not found")
                sensor.enable(step_ms)
                ke, kd = (gains or {}).get(act.name) or servo_gains(motor)
                lo, hi = motor.getMinPosition(), motor.getMaxPosition()
                if lo == hi == 0.0:              # an unlimited joint: search the stroke near zero
                    lo, hi = (-math.pi, math.pi) if act.mount.kind != "linear" else (-10.0, 10.0)
                # the stroke's ends, where they fall inside the joint's range, are hard stops too
                q_lo, q_hi = lo, hi
                ends = [act.mount.q_at_length(L, lo, hi) for L in (act.cylinder.retracted, act.cylinder.extended)]
                if None not in ends:
                    q_lo, q_hi = max(lo, min(ends)), min(hi, max(ends))
                linear = motor.getType() == getattr(motor, "LINEAR", 1)
                self.joints[act.name] = _Bound(c, i, motor, sensor, ke, kd, q_lo, q_hi,
                                               extra={"linear": 1.0 if linear else 0.0})
        unknown = set(joints) - set(self.joints)
        if unknown:
            raise KeyError(f"joints with no actuator in any circuit: {sorted(unknown)}")

    def step(self, velocities: Dict[str, float]) -> Dict[str, dict]:
        """Request joint velocities (rad/s or m/s; missing joints hold at 0) and drive one step.

        Returns, per joint: the requested velocity, the velocity the oil delivered and the one the
        joint achieved last step (measured), q, the estimated actuator torque, its
        cap, use = |torque| / cap, the circuit's pressure (bar) and whether it is at relief, the
        cylinder's length and stroke fraction, and the creep velocity from leakage."""
        dt = self.dt
        report: Dict[str, dict] = {}
        per_circuit: Dict[int, list] = {}
        for name, b in self.joints.items():
            q = b.sensor.getValue()
            if not b.started:
                b.q_cmd, b.q_prev, b.started = q, q, True
            qdot = (q - b.q_prev) / dt
            b.extra["qdot"] = qdot
            # the torque the servo made last step: its law, clamped at the cap it had
            if b.pushed > 0:
                b.tau = max(-b.pushed, min(b.pushed, b.ke * (b.motor.getTargetPosition() - q) - b.kd * qdot))
            b.q_prev = q
            per_circuit.setdefault(id(b.circuit), []).append((name, b, q))
        for c in self.circuits:
            rows = sorted(per_circuit.get(id(c), []), key=lambda r: r[1].index)
            states = [Circuit.state(c.actuators[b.index], q) for _, b, q in rows]
            taus = [b.tau for _, b, _ in rows]
            want = [float(velocities.get(name, 0.0)) for name, _, _ in rows]
            got = c.step(dt, states, want, taus)
            caps = c.caps(states, taus)
            for (name, b, q), s, w, g, cap in zip(rows, states, want, got, caps):
                creep = c.creep(b.index, s, b.tau)
                b.q_cmd += (g + creep) * dt
                band = cap / max(b.ke, 1e-12)            # a load above relief pushes the cylinder back
                b.q_cmd = max(q - band, min(q + band, b.q_cmd))
                # the stroke's ends last: nothing, not even the yield band, drives past them
                b.q_cmd = max(b.q_lo, min(b.q_hi, b.q_cmd))
                if (b.q_cmd >= b.q_hi and g > 0) or (b.q_cmd <= b.q_lo and g < 0):
                    g = 0.0                              # bottomed out: no flow, no feed-forward
                if b.pushed <= 0 or abs(cap - b.pushed) > CAP_PUSH_BAND * b.pushed:
                    if b.extra.get("linear"):
                        b.motor.setAvailableForce(cap)
                    else:
                        b.motor.setAvailableTorque(cap)
                    b.pushed = cap
                b.cap = cap
                # feed-forward the servo's damping so the joint keeps the delivered speed
                b.motor.setPosition(b.q_cmd + b.kd / max(b.ke, 1e-12) * g)
                act = c.actuators[b.index]
                L = s["length"]
                report[name] = {
                    "requested": w, "delivered": g, "achieved": b.extra["qdot"],
                    "q": q, "torque": b.tau, "cap": cap,
                    "use": abs(b.tau) / cap if cap > 0 else 0.0,
                    "pressure_bar": c.pressure / BAR, "at_relief": c.at_relief,
                    "length": L, "stroke_fraction": (L - act.cylinder.retracted) / act.cylinder.stroke,
                    "creep": creep, "spool": act.spool,
                }
        return report

    def hold(self) -> Dict[str, dict]:
        """One step with every valve closed (loads are held, minus any leakage creep)."""
        return self.step({})


def velocity_toward(q: float, target: float, gain: float = 2.0, vmax: Optional[float] = None) -> float:
    """A simple velocity request that closes on a target position: gain*(target - q), limited."""
    v = gain * (target - q)
    if vmax is not None:
        v = max(-vmax, min(vmax, v))
    return v
