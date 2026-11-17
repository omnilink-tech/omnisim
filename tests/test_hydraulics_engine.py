"""omnisim.hydraulics drives a real OmniSim joint the way a hydraulic cylinder would (2026-11-08).

The engine-free half is tests/test_hydraulics.py (the circuit's physics against hand calculations,
and the Drive on a plant that integrates the engine's servo law). This half runs the Drive on the
engine itself, on a boom: a fixed-base Robot with one HingeJoint whose end point is a payload at
1.5 m, lifted by a 63/35 x 300 mm cylinder on a 10.5 L/min, 160 bar pump through a proportional
valve. Each arm is one headless run (CPU `newtonSolver "mujoco"`, ~5 s simulated):

  payload 300 kg, ask 2 rad/s   it rises at the FLOW-LIMITED speed (pump flow / bore area / arm),
                                and the torque the Drive infers from the servo law at rest matches
                                the payload's weight: the gains it mirrors are the engine's own
  payload 1100 kg, ask to rise  above relief: it is pushed DOWN and reports at_relief
  300 kg, leaky cylinder, hold  it creeps down at the predicted leak rate

    python -m pytest tests/test_hydraulics_engine.py -v

Honours OMNISIM_BINARY and an already-set OMNISIM_HOME, like the other engine pins.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
_MOD = REPO / "lib" / "controller" / "python" / "omnisim" / "hydraulics.py"
_spec = importlib.util.spec_from_file_location("_omnisim_hydraulics_engine_pin", _MOD)
H = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = H
_spec.loader.exec_module(H)

G = 9.81
RADIUS = 1.5
Q_LO, Q_HI = -0.5, 0.8


def _parts(leak=0.0):
    cyl = H.Cylinder.mm(bore=63, rod=35, stroke=300, retracted=480, leak_lpm_at_100bar=leak)
    mount = H.Mount.fit(cyl, q_lo=Q_LO, q_hi=Q_HI, ra=0.30)
    pump = H.Pump(flow_lpm=10.5, relief_bar=160)
    return cyl, mount, pump


def _effort():
    """The motor's maxTorque: 10 x the cylinder's largest relief torque over the range (stiffness)."""
    cyl, mount, pump = _parts()
    return 10.0 * max(pump.relief * cyl.area_bore * mount.arm(q / 20.0 * (Q_HI - Q_LO) + Q_LO)
                      for q in range(21))


CONTROLLER = '''"""Hold, then ask the boom for --speed rad/s for --move s, then hold; write the trace."""
import json, sys
from omnisim import Robot
from omnisim.hydraulics import Cylinder, Mount, Pump, Valve, Circuit, Drive
a = dict(zip(sys.argv[1::2], sys.argv[2::2]))
robot = Robot()
dt_ms = int(robot.getBasicTimeStep())
cyl = Cylinder.mm(bore=63, rod=35, stroke=300, retracted=480, leak_lpm_at_100bar=float(a["--leak"]))
circuit = Circuit(Pump(flow_lpm=10.5, relief_bar=160), Valve.proportional())
circuit.add("boom", cyl, Mount.fit(cyl, q_lo=-0.5, q_hi=0.8, ra=0.30))
drive = Drive(robot, [circuit], {"boom": ("boom_motor", "boom_sensor")})
hold0, move, hold1 = float(a["--hold"]), float(a["--move"]), float(a["--after"])
trace = []
while robot.step(dt_ms) != -1:
    t = robot.getTime()
    v = float(a["--speed"]) if hold0 <= t < hold0 + move else 0.0
    r = drive.step({"boom": v})["boom"]
    trace.append([t, r["q"], r["requested"], r["delivered"], r["achieved"], r["torque"], r["cap"],
                  r["use"], r["pressure_bar"], 1 if r["at_relief"] else 0, r["creep"]])
    if t >= hold0 + move + hold1:
        break
json.dump({"trace": trace}, open(a["--out"], "w"))
robot.simulationQuit(0)
'''


def _world(out: Path, mass: float, leak: float, speed: float, hold: float, move: float, after: float) -> str:
    effort = _effort()
    inertia = mass * 0.1 ** 2 / 6.0          # a 0.1 m cube's own inertia; the radius does the rest
    return f"""#OMNISIM R2025a utf8


WorldInfo {{
  basicTimeStep 2
  newtonSolver "mujoco"
}}
Viewpoint {{ position 1 -6 2 }}
Robot {{
  name "boom_rig"
  controller "hyd_probe"
  controllerArgs [ "--out" "{out.as_posix()}" "--leak" "{leak}" "--speed" "{speed}" "--hold" "{hold}" "--move" "{move}" "--after" "{after}" ]
  children [
    HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 -1 0 anchor 0 0 1 }}
      device [
        RotationalMotor {{ name "boom_motor" maxTorque {effort:.6g} minPosition {Q_LO} maxPosition {Q_HI} maxVelocity 10 }}
        PositionSensor {{ name "boom_sensor" }}
      ]
      endPoint Solid {{
        translation {RADIUS} 0 1
        children [ Shape {{ geometry Box {{ size 0.1 0.1 0.1 }} }} ]
        physics Physics {{ density -1 mass {mass} centerOfMass [ 0 0 0 ] inertiaMatrix [ {inertia:.6g} {inertia:.6g} {inertia:.6g} 0 0 0 ] }}
      }}
    }}
  ]
}}
"""


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


def _run(tmp_path: Path, tag: str, mass: float, leak: float = 0.0, speed: float = 0.0,
         hold: float = 1.0, move: float = 0.0, after: float = 0.0):
    proj = tmp_path / tag
    (proj / "worlds").mkdir(parents=True)
    ctrl = proj / "controllers" / "hyd_probe"
    ctrl.mkdir(parents=True)
    (ctrl / "hyd_probe.py").write_text(CONTROLLER, encoding="utf-8")
    out = proj / "result.json"
    world = proj / "worlds" / "boom.omniworld"
    world.write_text(_world(out, mass, leak, speed, hold, move, after), encoding="utf-8")
    log = proj / "engine.log"
    env = dict(os.environ)
    home = Path(env.get("OMNISIM_HOME") or REPO)
    env["OMNISIM_HOME"] = str(home)
    env["OMNISIM_LOG_PATH"] = str(log)
    env.pop("OMNISIM_NEWTON_MJWARP", None)
    for var in ("OMNISIM_NEWTON_TARGET_KE", "OMNISIM_NEWTON_TARGET_KD", "OMNISIM_NEWTON_TORQUE_MODE"):
        env.pop(var, None)
    proc = subprocess.run([sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration", "60"],
                          cwd=str(home), env=env, capture_output=True, text=True, timeout=300)
    assert out.is_file(), (f"{tag}: the probe wrote no result (see {log})\n"
                           + proc.stdout[-2000:] + proc.stderr[-2000:])
    rows = json.loads(out.read_text())["trace"]
    keys = ("t", "q", "requested", "delivered", "achieved", "torque", "cap", "use", "p_bar", "relief", "creep")
    return [dict(zip(keys, r)) for r in rows]


def _at(rows, t):
    return min(rows, key=lambda r: abs(r["t"] - t))


ENGINE = pytest.mark.engine
NO_BINARY = pytest.mark.skipif(_binary() is None, reason="no omnisim-bin in this clone")


@ENGINE
@NO_BINARY
def test_flow_limited_rise_and_the_torque_estimate(tmp_path):
    mass = 300.0
    rows = _run(tmp_path, "rise", mass=mass, speed=2.0, hold=1.0, move=1.5, after=0.5)
    rest = _at(rows, 0.95)
    weight = mass * G * RADIUS * math.cos(rest["q"])
    assert rest["torque"] == pytest.approx(weight, rel=0.05), \
        f"the inferred torque {rest['torque']:.0f} N m should hold the payload's {weight:.0f} N m"
    cyl, mount, pump = _parts()
    mid = _at(rows, 1.75)
    flow_speed = pump.flow_at(mid["p_bar"] * H.BAR) / (cyl.area_bore * mount.arm(mid["q"]))
    assert mid["delivered"] == pytest.approx(flow_speed, rel=0.05)
    assert mid["achieved"] == pytest.approx(flow_speed, rel=0.10), \
        f"the joint moved at {mid['achieved']:.3f} rad/s; the oil allows {flow_speed:.3f}"
    assert mid["delivered"] < 1.0                       # far below the 2 rad/s asked
    assert _at(rows, 2.6)["q"] > rest["q"] + 0.3


@ENGINE
@NO_BINARY
def test_a_load_above_relief_is_pushed_down(tmp_path):
    rows = _run(tmp_path, "overload", mass=1100.0, speed=0.3, hold=0.0, move=0.6, after=0.0)
    q0, mid = rows[0]["q"], _at(rows, 0.3)
    assert rows[-1]["q"] < q0 - 0.02, "asked to rise, an overloaded boom must be pushed down"
    assert mid["relief"] == 1 and mid["use"] > 0.95
    assert mid["delivered"] == 0.0


@ENGINE
@NO_BINARY
def test_a_leaky_cylinder_lets_the_load_creep(tmp_path):
    rows = _run(tmp_path, "leak", mass=300.0, leak=0.5, hold=4.0)
    a, b = _at(rows, 1.0), _at(rows, 4.0)
    predicted = a["creep"] * (b["t"] - a["t"])
    assert a["creep"] < 0.0
    assert b["q"] - a["q"] == pytest.approx(predicted, rel=0.2), \
        f"drift {b['q'] - a['q']:.5f} rad against the leak's {predicted:.5f}"
