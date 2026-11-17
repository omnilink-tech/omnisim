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

""""Sit down" is a command, and a sit is reported from what it MEASURED
(XGO Mini2S clip lane, public v9.2.6, 2026-10-20).

Two defects, found on the installed package and pinned here:

A. The parser's `sit` rule was `sit|sit down|lie down|crouch`. An alternation
   takes the first branch that matches, so "Sit down." read "Sit" (38% of the
   clause, under the gate) and went to the model. Worse, "Sit down, then stand
   up." was salvaged as an ADDRESS "Sit down" + "then stand up" and ran the
   stand ALONE at 0.95, saying nothing about the dropped clause.

B. The quadruped bridge called a sit / stand done the moment it switched the
   setpoint (`is_busy()` idle for "sit" and "stand"), so in "Lie down, then
   stand up." both legs ran in the same tick and the reply said "Sit. Stand."
   while the body never went down (min 0.1332 m; 0.088 m when it really lies).

⚠️ B DRIVES THE SHIPPED QuadrupedBridge (the real module, behind a stub
`omnisim` Supervisor) against a fake plant: first-order joint servos and a
body whose height follows the knees. That proves the bookkeeping -- the
order is sequenced on the measurement and the reply quotes it -- not the
physics, which was measured in the engine (EVIDENCE.md of the clip lane).
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import threading
import time
import types

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
QUAD_DIR = ROOT / "projects/samples/demos/controllers/omnilink_quadruped_bridge"
for _p in (str(PKG_SRC),):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from omnisim_bridges import interpret as I  # noqa: E402
from omnisim_bridges import route as R  # noqa: E402

ROBOT = "lite3"          # body_lock False: it stands on its own contacts
DT = 0.008


# ── A. the parser ────────────────────────────────────────────────────

@pytest.mark.parametrize("text,tools", [
    ("Sit down.", ["sit"]),
    ("sit down please", ["sit"]),
    ("Lie down.", ["sit"]),
    ("Crouch down.", ["sit"]),
    ("Stand up.", ["stand"]),
    ("Get up.", ["stand"]),
    ("Sit down, then stand up.", ["sit", "stand"]),
    ("Lie down, then stand up.", ["sit", "stand"]),
    ("Stand up, then sit down.", ["stand", "sit"]),
    ("Spot, sit down.", ["sit"]),            # a real address still strips
])
def test_multiword_pose_orders_parse_whole(text, tools):
    r = I.interpret(text, I.QUADRUPED)
    assert r.executable and r.confidence >= 0.8, r.explain()
    assert [f.tool for f in r.frames] == tools, r.explain()
    assert not r.residue, r.explain()


def test_a_clause_is_never_salvaged_away_as_an_address():
    # On a wheeled robot "Sit down" is not an order it can run. It used to
    # be stripped as a NAME ("Cart, stop!") and the drive ran alone at 0.95.
    r = I.interpret("Sit down, then drive forward 1 m.", I.MOBILE)
    assert r.confidence < 0.8, r.explain()            # not acted on unasked
    assert "sit down" in r.residue.lower(), r.explain()
    # ...and the parser-first path does not answer it: the model gets the
    # whole sentence, residue and all.
    assert R.parser_first_plan("Sit down, then drive forward 1 m.", I.MOBILE) is None
    # Should anything execute it anyway, the reply names the skipped part.
    bridge = types.SimpleNamespace(
        act_drive_forward=lambda distance: {"accepted": True})
    out = R.execute(bridge, r, I.MOBILE)
    assert "I did not act on 'Sit down'" in out["agent"], out


def test_ordinary_addresses_still_strip():
    assert [f.tool for f in I.interpret("Cart, stop!", I.MOBILE).frames] == ["stop"]
    r = I.interpret("Husky, drive forward one metre", I.MOBILE)
    assert r.executable and r.confidence >= 0.8
    assert [f.tool for f in r.frames] == ["drive_forward"]


@pytest.mark.parametrize("text", [
    "Sit down, then stand up.",
    "Stand up, then sit down.",
    "Lie down, then wave, then stand up.",
    "Sit down, then fly to the moon, then stand up.",
])
def test_every_clause_of_a_compound_is_acted_on_or_reported(text):
    """The invariant behind A: no clause disappears. Each clause either
    became a frame, or is in `residue` and the parse is below the acting
    floor, so the model (and the reply) see it."""
    r = I.interpret(text, I.QUADRUPED)
    clauses = [c for c in I._clauses(text) if I._core(c)]
    framed = {f.source.lower().strip(" .") for f in r.frames}
    for c in clauses:
        c0 = c.lower().strip(" .")
        if c0 in framed:
            continue
        assert r.confidence < 0.8 and c0 in r.residue.lower(), (c, r.explain())


# ── B. the bridge, against a fake plant ──────────────────────────────

class _Sensor:
    def __init__(self, m):
        self._m = m

    def enable(self, _ts):
        pass

    def getValue(self):
        return self._m.actual


class _Motor:
    TAU = 0.06

    def __init__(self):
        self.setpoint = None
        self.actual = 0.0
        self.stuck = False
        self._s = _Sensor(self)

    def setPosition(self, q):
        self.setpoint = float(q)

    def setVelocity(self, _v):
        pass

    def getPositionSensor(self):
        return self._s

    def getMinPosition(self):
        return -10.0

    def getMaxPosition(self):
        return 10.0

    def step(self, dt):
        if self.setpoint is not None and not self.stuck:
            self.actual += (self.setpoint - self.actual) * min(1.0, dt / self.TAU)


class _Field:
    def setSFVec3f(self, _v):
        pass

    def setSFRotation(self, _v):
        pass


class _Node:
    def __init__(self, robot):
        self._r = robot

    def getField(self, _n):
        return _Field()

    def getPosition(self):
        return [0.0, 0.0, self._r.body_z()]

    def getOrientation(self):
        return [1, 0, 0, 0, 1, 0, 0, 0, 1]


Z_STAND, Z_SIT = 0.30, 0.20


class _Supervisor:
    cfg = None          # set by the fixture

    def __init__(self):
        self._time = 0.0
        self.motors = {}

    def getBasicTimeStep(self):
        return DT * 1000.0

    def getDevice(self, name):
        if name not in self.motors:
            self.motors[name] = _Motor()
        return self.motors[name]

    def getSelf(self):
        return _Node(self)

    def getTime(self):
        return self._time

    def body_z(self):
        """The body follows the knees: stand height at the stand knee, sit
        height at the sit knee, linear between (clamped)."""
        fr = []
        for leg, names in self.cfg["legs"].items():
            m = self.motors.get(names["knee"])
            if m is None:
                continue
            k0, k1 = self.cfg["stand"][leg][1], self.cfg["sit"][leg][1]
            fr.append(max(0.0, min(1.0, (m.actual - k0) / (k1 - k0))))
        f = sum(fr) / len(fr) if fr else 0.0
        return Z_STAND + (Z_SIT - Z_STAND) * f


@pytest.fixture(scope="module")
def qb():
    """Import the SHIPPED bridge module behind a stub `omnisim`, then put the
    real `omnisim` (the repo's CLI package) back."""
    saved = {k: sys.modules.get(k) for k in ("omnisim",)}
    for p in (str(QUAD_DIR), str(QUAD_DIR.parent)):
        if p not in sys.path:
            sys.path.insert(0, p)
    stub = types.ModuleType("omnisim")
    stub.Supervisor = _Supervisor
    stub.Robot = _Supervisor
    sys.modules["omnisim"] = stub
    try:
        spec = importlib.util.spec_from_file_location(
            "_quadruped_bridge_under_test", QUAD_DIR / "omnilink_quadruped_bridge.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    _Supervisor.cfg = mod.QUADRUPED_CONFIGS[ROBOT]
    return mod


def _tick(bridge, robot):
    robot._time += DT
    for m in robot.motors.values():
        m.step(DT)
    bridge.tick(robot.getTime())


class _Ticker:
    """The sim thread: steps the fake world while the caller blocks."""

    def __init__(self, bridge, robot):
        self.bridge, self.robot = bridge, robot
        self.z = []                       # (sim_t, body_z, motion kind)
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            _tick(self.bridge, self.robot)
            self.z.append((self.robot._time, self.robot.body_z(), self.bridge.motion[0]))
            time.sleep(0.0002)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(5)


def _standing(qb):
    robot = _Supervisor()
    bridge = qb.QuadrupedBridge(robot, ROBOT)
    for _ in range(int(2.5 / DT)):        # the spawn settle into the stand
        _tick(bridge, robot)
    assert bridge.motion[0] == "stand"
    return bridge, robot


def test_a_sit_is_busy_until_it_is_measured(qb):
    """The root of B: switching the setpoint is not arriving."""
    bridge, robot = _standing(qb)
    out = bridge.act_sit(wait=False)
    assert out["settled"] is None                    # never claims it sat
    assert bridge.is_busy()
    for _ in range(int(3.0 / DT)):
        _tick(bridge, robot)
        if not bridge.is_busy():
            break
    assert not bridge.is_busy()
    res = bridge.get_state()["last_pose"]
    assert res["seq"] == out["seq"] and res["settled"] is True
    assert res["achieved"]["body_z_m"] == pytest.approx(Z_SIT, abs=0.005)
    assert res["achieved"]["joint_error_max_deg"] <= 3.0


def test_lie_down_then_stand_up_really_goes_down_first(qb):
    bridge, robot = _standing(qb)
    r = I.interpret("Lie down, then stand up.", I.QUADRUPED)
    with _Ticker(bridge, robot) as sim:
        out = R.execute(bridge, r, I.QUADRUPED)
    tools = out["tools"]
    assert [t[0] for t in tools] == ["sit", "stand"], out
    assert [t[1] for t in tools] == ["ok", "ok"], out
    # The body reached the sit height BEFORE the stand was commanded.
    first_stand = next(t for t, _z, k in sim.z if k == "stand" and t > sim.z[0][0])
    low = min(z for t, z, _k in sim.z if t < first_stand)
    assert low <= Z_SIT + 0.005, (low, first_stand)
    # ...and the reply quotes the measurement, not the tool name.
    reply = out["agent"]
    assert reply != "Sit. Stand."
    assert "In the sit pose" in reply and "In the standing pose" in reply, reply
    assert "body height" in reply, reply


def test_a_sit_that_cannot_happen_is_reported_as_not_reached(qb):
    bridge, robot = _standing(qb)
    for m in robot.motors.values():
        m.stuck = True                    # a blocked plant: nothing moves
    with _Ticker(bridge, robot):
        res = bridge.act_sit()
    assert res["settled"] is False and res["timed_out"] is True, res
    assert res["achieved"]["body_dz_m"] == pytest.approx(0.0, abs=1e-6)
    said, status, _detail, _rule = R._describe("sit", {}, res, [])
    assert status == "err", said
    assert said.startswith("I did not reach the sit pose"), said


def test_a_stop_supersedes_a_sit_in_flight(qb):
    bridge, robot = _standing(qb)
    out = bridge.act_sit(wait=False)
    _tick(bridge, robot)
    bridge.act_stop()
    _tick(bridge, robot)
    res = bridge.get_state()["last_pose"]
    assert res["seq"] == out["seq"] and res.get("superseded") is True
    assert not bridge.is_busy()
    said, status, detail, _ = R._describe("sit", {}, res, [])
    assert detail == "superseded" and "did not finish" in said


def test_the_sim_thread_never_waits_on_itself(qb):
    """handle_wwi runs on the sim thread; a wait there would deadlock."""
    bridge, robot = _standing(qb)
    bridge._sim_thread = threading.get_ident()
    t0 = time.time()
    res = bridge.act_sit()                            # wait=True by default
    assert time.time() - t0 < 1.0
    assert res["settled"] is None
    said, status, detail, _ = R._describe("sit", {}, res, [])
    assert detail == "unconfirmed" and "did not wait" in said
