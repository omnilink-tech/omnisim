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

"""The arm bridge HOLDS its commanded pose under gravity and LIFTS before a
lateral move (2026-10-17).

⚠️ THIS DRIVES THE SHIPPED ArmBridge (the real module, imported behind a stub
`omnisim` Supervisor), against a fake plant that behaves the way the engine's
position servo does: a spring-damper with NO integral term, so every loaded
joint settles short of its setpoint by a constant sag (tau_gravity / ke).

WHAT IT PINS, AND WHY. On the HEBI A-2240-06 pilot (EVIDENCE.md, take3_model)
every 5 cm hover landed 20-22 mm low and the completion still said
`settled: true`; leaving a hover for the next one, the joint-space move dipped
to ~2 mm above the tag it had just inspected. The fixes: an integral setpoint
trim in the bridge, `settled` as a convergence verdict, and a Cartesian
up/across/down path for lateral moves. Each has a value-parsed hatch
(OMNISIM_ARM_GRAVITY_TRIM, OMNISIM_ARM_LIFT_BEFORE_MOVE) that restores the old
behaviour exactly, and the hatch tests pin THAT too, so the A/B stays honest.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import types

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
ARM_DIR = ROOT / "projects/samples/demos/controllers/omnilink_arm_bridge"
PKG_SRC = pathlib.Path(__file__).resolve().parents[1] / "src"

DT = 0.016
# Per-joint steady-state sag of the fake servo (rad): what gravity pulls a
# loaded UR5e-like joint short by. Shoulder and elbow sag most, like the HEBI.
SAG = [0.0, 0.03, -0.015, 0.008, 0.0, 0.0]


# ── fake engine ──────────────────────────────────────────────────────

class _Sensor:
    def __init__(self, m):
        self._m = m

    def enable(self, _ts):
        pass

    def getValue(self):
        return self._m.actual


class _PlantMotor:
    """Position servo with a POSE-DEPENDENT gravity sag and no integral
    action: the sag scales with cos(joint angle), so a trim learned at one
    pose is wrong at the next and has to be re-converged, not replayed."""

    def __init__(self, sag):
        self.sag = sag
        self.setpoint = None
        self.actual = 0.0
        self._s = _Sensor(self)
        self.blocked_above = None     # a surface: cannot go further positive

    def sag_now(self, q=None):
        q = self.actual if q is None else q
        return self.sag * (0.75 + 0.25 * math.cos(q))

    def getMaxVelocity(self):
        return 3.0

    def setVelocity(self, _v):
        pass

    def setPosition(self, q):
        self.setpoint = float(q)
        if not hasattr(self, "_placed"):
            self._placed = True
            a = float(q)
            for _ in range(20):
                a = float(q) - self.sag_now(a)
            self.actual = a

    def getPositionSensor(self):
        return self._s

    def step(self):
        if self.setpoint is not None:
            # first-order tracking toward (setpoint - sag): tau ~ 0.05 s
            self.actual += (self.setpoint - self.sag_now() - self.actual) * 0.3
        if self.blocked_above is not None:
            self.actual = min(self.actual, self.blocked_above)


class _Field:
    def __init__(self, v):
        self.v = v

    def getSFString(self):
        return self.v


class _MF:
    def getCount(self):
        return 0


class _Node:
    def getField(self, n):
        if n == "name":
            return _Field("ur5e")
        if n == "children":
            return _MF()
        return None

    def getPosition(self):
        return [0.0, 0.0, 0.0]

    def getOrientation(self):
        return [1, 0, 0, 0, 1, 0, 0, 0, 1]

    def getDef(self):
        return ""


class _Supervisor:
    def __init__(self, sag=SAG, names=None):
        self._time = 0.0
        self._sag = list(sag)
        self.motors = {}
        self._names = list(names or [])

    def getBasicTimeStep(self):
        return DT * 1000.0

    def getDevice(self, name):
        if not name.endswith("_motor"):
            return None
        base = name[: -len("_motor")]
        if base not in self.motors:
            j = self._names.index(base) if base in self._names else 0
            self.motors[base] = _PlantMotor(self._sag[j])
        return self.motors[base]

    def getSelf(self):
        return _Node()

    def getRoot(self):
        return _Node()

    def getFromDef(self, _n):
        return None

    def getTime(self):
        return self._time


@pytest.fixture(scope="module")
def br():
    """Import the SHIPPED bridge module once, behind a stub `omnisim`, and put
    the real `omnisim` (the repo's CLI package) back afterwards."""
    saved = {k: sys.modules.get(k) for k in ("omnisim", "controller")}
    for p in (str(PKG_SRC), str(ARM_DIR), str(ARM_DIR.parent)):
        if p not in sys.path:
            sys.path.insert(0, p)
    stub = types.ModuleType("omnisim")
    stub.Supervisor = _Supervisor
    stub.Robot = _Supervisor
    sys.modules["omnisim"] = stub
    try:
        spec = importlib.util.spec_from_file_location(
            "_arm_bridge_under_test", ARM_DIR / "omnilink_arm_bridge.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


def _make(br, monkeypatch, trim=True, lift=True, sag=SAG):
    monkeypatch.setenv("OMNISIM_ARM_GRAVITY_TRIM", "1" if trim else "0")
    monkeypatch.setenv("OMNISIM_ARM_LIFT_BEFORE_MOVE", "1" if lift else "0")
    monkeypatch.delenv("OMNISIM_ARM_TCP_TRACE", raising=False)
    monkeypatch.setenv("OMNILINK_INTENTS", "0")
    cfg = br.get_config("ur5e")
    robot = _Supervisor(sag=sag, names=cfg["joint_names"])
    bridge = br.ArmBridge(robot, cfg, "ur5e")
    return bridge, robot, cfg


def _run(bridge, robot, seconds, on_tick=None):
    for _ in range(int(round(seconds / DT))):
        robot._time += DT
        for m in robot.motors.values():
            m.step()
        bridge.tick(robot.getTime())
        if on_tick is not None:
            on_tick()


def _run_until_done(bridge, robot, seq, limit_s=15.0, on_tick=None):
    t = 0.0
    while t < limit_s:
        _run(bridge, robot, DT, on_tick)
        t += DT
        done = bridge.last_completion
        if done is not None and done.get("seq") == seq:
            return done
    raise AssertionError(f"motion {seq} never completed in {limit_s}s sim")


def _fk(br, cfg, q):
    ik = cfg["ik"]
    return br.forward_kinematics(ik["chain"], q, ik["tcp_offset"])


# ── gravity trim / settled ───────────────────────────────────────────

def test_hold_converges_on_home_despite_servo_sag(br, monkeypatch):
    bridge, robot, _cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    out = bridge.act_reset_to_home(wait=False)
    rec = _run_until_done(bridge, robot, out["seq"])
    assert rec["settled"] is True
    assert rec["settle"]["converged"] is True
    assert rec["max_abs_error_rad"] <= bridge.SETTLE_TOL_RAD, rec
    # The trim IS the sag at this pose: the servo is asked for exactly the
    # offset gravity takes away, no more.
    for t, n in zip(bridge._trim, _cfg["joint_names"]):
        assert abs(t - robot.motors[n].sag_now()) < 2e-3


def test_hatch_off_reports_the_sagged_pose_exactly_as_before(br, monkeypatch):
    bridge, robot, _cfg = _make(br, monkeypatch, trim=False, lift=False)
    _run(bridge, robot, 0.5)
    out = bridge.act_reset_to_home(wait=False)
    rec = _run_until_done(bridge, robot, out["seq"])
    # The old verdict: settled after SETTLE_S whatever the error, no settle
    # block, the sag in plain sight in `error`.
    assert rec["settled"] is True
    assert "settle" not in rec and "path_min_tcp_z_m" not in rec
    assert abs(rec["max_abs_error_rad"]
               - max(abs(m.sag_now()) for m in robot.motors.values())) < 3e-3
    assert all(t == 0.0 for t in bridge._trim)
    assert bridge.motion[0] == "hold"


def test_a_pose_the_servo_cannot_reach_is_not_reported_settled(br, monkeypatch):
    # A sag bigger than the trim clamp: the joint never converges, and the
    # completion must SAY so instead of calling it settled.
    big = [0.0, 0.30, 0.0, 0.0, 0.0, 0.0]
    bridge, robot, _cfg = _make(br, monkeypatch, sag=big)
    _run(bridge, robot, 0.5)
    out = bridge.act_reset_to_home(wait=False)
    rec = _run_until_done(bridge, robot, out["seq"], limit_s=20.0)
    assert rec["settled"] is False
    assert rec["settle"]["converged"] is False
    assert rec["settle"]["max_abs_joint_error_rad"] > 0.1
    assert abs(bridge._trim[1]) <= bridge.trim_band + 1e-12
    # it gave up, so it rolled back to the last converged trim (none yet)
    assert rec["settle"].get("trim_rolled_back") is True
    assert bridge._trim == bridge._trim_good


def test_set_tcp_target_lands_within_the_stated_tolerance(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    target = (0.45, 0.15, 0.12)
    out = bridge.act_set_tcp_target(target, wait=False)
    assert out.get("accepted") is True, out
    rec = _run_until_done(bridge, robot, out["seq"])
    assert rec["settled"] is True
    assert rec["within_tolerance"] is True
    assert rec["tcp_tolerance_m"] == bridge.TCP_TOL_M
    assert rec["error_m"] <= bridge.TCP_TOL_M, rec
    # and the measurement is FK of the joint SENSORS, not the plan
    got = _fk(br, cfg, bridge._read_q())
    assert math.dist(got, rec["achieved"]) < 1e-6


def test_without_the_trim_the_same_move_lands_low(br, monkeypatch):
    # The defect the trim fixes, measured on the same fake: centimetre-scale
    # TCP error from a few hundredths of a radian of sag.
    bridge, robot, _cfg = _make(br, monkeypatch, trim=False, lift=False)
    _run(bridge, robot, 0.5)
    out = bridge.act_set_tcp_target((0.45, 0.15, 0.12), wait=False)
    rec = _run_until_done(bridge, robot, out["seq"])
    assert rec["settled"] is True          # the old, dishonest verdict
    assert rec["error_m"] > 0.01


# ── lift before a lateral move ───────────────────────────────────────

def _hover(br, bridge, robot, cfg, xyz, track):
    out = bridge.act_set_tcp_target(xyz, wait=False)
    assert out.get("accepted") is True, out
    return out, _run_until_done(bridge, robot, out["seq"], on_tick=track)


def test_a_lateral_move_between_low_hovers_lifts_and_never_dips(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    a, b = (0.45, 0.20, 0.08), (0.50, -0.15, 0.08)
    _hover(br, bridge, robot, cfg, a, None)
    zs = []
    track = lambda: zs.append(_fk(br, cfg, bridge._read_q())[2])  # noqa: E731
    _out, rec = _hover(br, bridge, robot, cfg, b, track)
    plan = rec["path"]
    assert plan["kind"] == "lift_traverse_lower"
    assert abs(plan["travel_z_m"] - (0.08 + bridge.LIFT_H_M)) < 1e-6
    assert max(zs) >= 0.08 + bridge.LIFT_H_M - 0.01      # it really went up
    # never meaningfully below the hover height (servo lag only)
    assert min(zs) > 0.08 - 0.005, min(zs)
    assert rec["path_min_tcp_z_m"] > 0.08 - 0.005
    assert rec["within_tolerance"] is True


def test_lift_hatch_off_is_the_old_joint_space_move(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch, lift=False)
    _run(bridge, robot, 0.5)
    bridge.act_set_tcp_target((0.45, 0.20, 0.08), wait=False)
    assert bridge.motion[0] == "interp"


def test_a_short_or_vertical_move_is_unchanged(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    _hover(br, bridge, robot, cfg, (0.45, 0.20, 0.08), None)
    bridge.act_set_tcp_target((0.45, 0.20, 0.15), wait=False)
    assert bridge.motion[0] == "interp"      # straight up: no detour needed


def test_reset_to_home_from_a_low_hover_retreats_straight_up_first(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    _hover(br, bridge, robot, cfg, (0.50, -0.15, 0.08), None)
    p0 = _fk(br, cfg, bridge._read_q())
    pts = []
    out = bridge.act_reset_to_home(wait=False)
    assert bridge.motion[0] == "traj"
    plan = bridge.motion[1]["_path_plan"]
    assert plan["kind"] == "retreat_then_home"
    T1 = plan["legs"][0]["duration_s"]
    track = lambda: pts.append(_fk(br, cfg, bridge._read_q()))  # noqa: E731
    rec = _run_until_done(bridge, robot, out["seq"], on_tick=track)
    first = [p for i, p in enumerate(pts) if i * DT < T1 * 0.8]
    assert first, "no samples on the retreat leg"
    # while retreating the tool stays over where it was and does not go down
    assert max(math.hypot(p[0] - p0[0], p[1] - p0[1]) for p in first) < 0.01
    assert min(p[2] for p in pts) > p0[2] - 0.005
    assert rec["settled"] is True
    assert rec["max_abs_error_rad"] <= bridge.SETTLE_TOL_RAD


# ── the reply answers the task's own question ────────────────────────

class _Tag:
    def __init__(self, d, name, xyz):
        self._d, self._n, self._t = d, name, xyz

    def getDef(self):
        return self._d

    def getField(self, n):
        if n == "name":
            return _Field(self._n)
        if n == "translation":
            return types.SimpleNamespace(getSFVec3f=lambda: list(self._t))
        return None


def test_a_hover_reports_its_measured_height_over_the_object(br, monkeypatch):
    # HEBI take4/take5 (2026-10-17): the model sent a WORLD z in the default
    # BASE frame and the hover landed 2 cm high while the reply said 5 cm.
    # The roster now gives both frames and the completion says, measured, how
    # high the tool ended up over the object below it.
    bridge, robot, _cfg = _make(br, monkeypatch)
    tag = _Tag("GRASP_TAG_A", "tag_a", [0.45, 0.20, 0.005])
    monkeypatch.setattr(bridge, "_iter_graspables",
                        lambda: iter([(tag, tag.getField("translation"))]))
    roster = bridge.act_locate_objects()["objects"][0]
    assert roster["base_xyz"] == [0.45, 0.2, 0.005]
    _run(bridge, robot, 0.5)
    out = bridge.act_set_tcp_target((0.45, 0.20, 0.055), wait=False)
    rec = _run_until_done(bridge, robot, out["seq"])
    near = rec["nearest_object"]
    assert near["name"] == "tag_a" and near["dxy_m"] < 0.003
    assert abs(near["tcp_above_object_origin_m"] - 0.05) < 0.003
    assert rec["achieved_world_xyz"] == [round(v, 5) for v in rec["achieved"]]


# ── review fixes (2026-10-17): races, windup, honest verdicts ────────

def _settled_at(br, bridge, robot, xyz):
    out = bridge.act_set_tcp_target(xyz, wait=False)
    assert out.get("accepted") is True, out
    return _run_until_done(bridge, robot, out["seq"])


def test_stop_during_planning_is_not_overwritten_by_the_plan(br, monkeypatch):
    # P1: the motion slot used to be claimed, then ~0.3 s of IK ran on the
    # HTTP thread, then the plan was written over whatever was there -- so a
    # stop that landed in that window was silently undone.
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    _settled_at(br, bridge, robot, (0.45, 0.20, 0.08))
    orig = bridge._plan_lifted
    seen = {}

    def plan_then_stop(*a, **k):
        seen["mode_while_planning"] = bridge.motion[0]
        # a second GOAL verb in the window is refused, not raced
        seen["busy"] = bridge.act_set_joint_positions_checked(
            list(bridge.home_pose), wait=False)
        r = orig(*a, **k)
        seen["stop"] = bridge.act_stop()       # the operator stop lands here
        return r

    monkeypatch.setattr(bridge, "_plan_lifted", plan_then_stop)
    p_before = _fk(br, cfg, bridge._read_q())
    out = bridge.act_set_tcp_target((0.50, -0.15, 0.08), wait=False)
    assert seen["mode_while_planning"] == "planning"
    assert seen["busy"]["error"] == "busy"
    assert seen["busy"]["http_status"] == 409
    assert out["superseded"] is True and out["started"] is False, out
    assert out["superseded_by_verb"] == "stop_robot"
    assert bridge.motion[0] == "hold", bridge.motion[0]
    pts = []
    _run(bridge, robot, 2.0,
         on_tick=lambda: pts.append(_fk(br, cfg, bridge._read_q())))
    assert bridge.motion[0] == "hold"
    assert max(math.dist(p, p_before) for p in pts) < 0.002   # did not move
    # the waiter is released honestly, never with a measured number
    assert bridge.last_completion["seq"] == out["seq"]
    assert bridge.last_completion["superseded"] is True
    assert bridge.last_completion["achieved"] is None


def test_stop_during_reset_retreat_planning_wins(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    _settled_at(br, bridge, robot, (0.50, -0.15, 0.08))
    orig = bridge._plan_home_retreat

    def plan_then_stop(*a, **k):
        r = orig(*a, **k)
        bridge.act_stop()
        return r

    monkeypatch.setattr(bridge, "_plan_home_retreat", plan_then_stop)
    out = bridge.act_reset_to_home(wait=False)
    assert out["superseded"] is True and out["started"] is False
    assert bridge.motion[0] == "hold"


def _blocked_hold(br, bridge, robot, cfg, short=0.02):
    """Converge at home, then command joint 1 `short` rad further than a
    surface lets it go (pressing on something)."""
    _run(bridge, robot, 0.5)
    rec = _run_until_done(bridge, robot,
                          bridge.act_reset_to_home(wait=False)["seq"])
    assert rec["settled"] is True
    good = list(bridge._trim)
    m = robot.motors[cfg["joint_names"][1]]
    m.blocked_above = m.actual
    q = list(rec["final_commanded_q"])
    q[1] += short
    out = bridge.act_set_joint_positions_checked(q, wait=False)
    return out, good, m


def test_a_joint_held_off_by_contact_does_not_wind_the_trim_past_the_band(
        br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    out, good, m = _blocked_hold(br, bridge, robot, cfg)
    peak = [0.0]

    def watch():
        peak[0] = max(peak[0], max(abs(t) for t in bridge._trim))

    rec = _run_until_done(bridge, robot, out["seq"], limit_s=20, on_tick=watch)
    assert rec["settled"] is False and rec["settle"]["converged"] is False
    assert peak[0] <= bridge.trim_band + 1e-12, peak[0]
    # gave up -> rolled back to the last CONVERGED trim, and frozen there
    assert rec["settle"]["trim_rolled_back"] is True
    assert bridge._trim == good
    _run(bridge, robot, 10.0, on_tick=watch)
    assert bridge._trim == good, "kept integrating into the surface"
    # and the next motion starts from the good trim, not the wound one
    m.blocked_above = None
    bridge.act_reset_to_home(wait=False)
    assert bridge._trim == good


def test_a_wound_trim_is_not_carried_into_a_motion_that_cuts_the_hold_short(
        br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    out, good, m = _blocked_hold(br, bridge, robot, cfg)
    # 1.5 s into the blocked hold: still inside the settle window, trim wound
    _run(bridge, robot, 1.5)
    assert bridge.motion[0] == "hold"
    assert abs(bridge._trim[1] - good[1]) > 5e-3, "the fixture did not wind"
    m.blocked_above = None
    out2 = bridge.act_reset_to_home(wait=False)
    assert bridge._trim == good
    # the cut-short blocked move is reported unmeasured, not as a failure
    rec = bridge.last_completion
    assert rec["seq"] == out["seq"] and rec["settled"] is None
    assert rec["cut_short"] is True
    rec2 = _run_until_done(bridge, robot, out2["seq"])
    assert rec2["settled"] is True


def test_back_to_back_commands_report_cut_short_not_a_failure(br, monkeypatch):
    # P2: the 409 hint and /action invite sending the next command as soon
    # as the mode is 'hold' -- which lands inside the settle window.
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    first = bridge.act_set_tcp_target((0.45, 0.20, 0.08), wait=False)
    called = []
    monkeypatch.setattr(bridge, "_nearest_graspable",
                        lambda *a, **k: called.append(a))
    while bridge.motion[0] != "hold":
        _run(bridge, robot, DT)
    st = bridge.get_state()
    assert st["mode"] == "hold" and st["settling"] is True
    bridge.act_set_tcp_target((0.45, 0.20, 0.14), wait=False)
    rec = bridge.last_completion
    assert rec["seq"] == first["seq"]
    assert rec["settled"] is None and rec["cut_short"] is True, rec
    assert rec["settle"]["converged"] is None
    # measured off the sim thread: no scene walk for nearest_object
    assert not called and "nearest_object_note" in rec
    assert bridge.events is not None
    events = bridge.events.since(0)["events"]
    assert not [e for e in events if e.get("type") == "motion.unsettled"], \
        events


def test_a_tcp_correction_seeds_from_the_measured_joints(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    cmd = [0.45, 0.15, 0.12]
    out = bridge.act_solve_ik(cmd)
    q_hold = list(out["q"])
    meas = list(q_hold)
    meas[1] += 0.01                       # the joints read 10 mrad off
    bridge.last_q = list(meas)
    got = _fk(br, cfg, meas)
    hold = {"q": list(q_hold),
            "_pending": {"verb": "set_tcp_target", "seq": 1,
                         "measure": {"kind": "tcp", "commanded": cmd}}}
    assert bridge._tcp_correction(hold) is True
    aim = [c + (c - g) for c, g in zip(cmd, got)]
    assert math.dist(_fk(br, cfg, hold["q"]), aim) < 1e-3
    assert hold["_tcp_corrections"] == 1
    assert hold["_tcp_correction_log"][0]["max_joint_change_rad"] > 1e-4
    # an unreachable aim changes nothing and is NOT counted
    far = {"q": list(q_hold),
           "_pending": {"verb": "set_tcp_target", "seq": 2,
                        "measure": {"kind": "tcp",
                                    "commanded": [3.0, 0.0, 0.1]}}}
    assert bridge._tcp_correction(far) is False
    assert "_tcp_corrections" not in far and far["q"] == q_hold


def test_a_lift_that_leaves_the_envelope_falls_back_without_a_refusal_line(
        br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    _settled_at(br, bridge, robot, (0.45, 0.20, 0.08))
    lines = []
    monkeypatch.setattr(bridge, "_log_refusal", lines.append)
    bridge.lift_h = 5.0                   # travel height far outside reach
    out = bridge.act_set_tcp_target((0.50, -0.15, 0.08), wait=False)
    assert out["accepted"] is True and bridge.motion[0] == "interp"
    rec = _run_until_done(bridge, robot, out["seq"])
    assert "outside the reach envelope" in rec["path"]["lift_skipped"]
    assert not [ln for ln in lines if "REFUSED" in ln], lines


def test_no_world_frame_numbers_when_the_base_pose_is_unknown(br, monkeypatch):
    bridge, robot, _cfg = _make(br, monkeypatch)
    tag = _Tag("GRASP_TAG_A", "tag_a", [0.45, 0.20, 0.005])
    monkeypatch.setattr(bridge, "_iter_graspables",
                        lambda: iter([(tag, tag.getField("translation"))]))
    monkeypatch.setattr(bridge, "_base_world_pose", lambda: None)
    roster = bridge.act_locate_objects()
    assert "base_xyz" not in roster["objects"][0]
    assert "base_xyz_note" in roster
    _run(bridge, robot, 0.5)
    rec = _run_until_done(bridge, robot, bridge.act_set_tcp_target(
        (0.45, 0.20, 0.055), wait=False)["seq"])
    assert "achieved_world_xyz" not in rec and "nearest_object" not in rec


def test_the_tcp_trace_names_the_robot_and_is_closed(br, monkeypatch, tmp_path):
    import json
    f = tmp_path / "trace.jsonl"
    monkeypatch.setenv("OMNISIM_ARM_TCP_TRACE", str(f))
    monkeypatch.setenv("OMNISIM_ARM_GRAVITY_TRIM", "1")
    monkeypatch.setenv("OMNISIM_ARM_LIFT_BEFORE_MOVE", "1")
    monkeypatch.setenv("OMNILINK_INTENTS", "0")
    cfg = br.get_config("ur5e")
    robot = _Supervisor(names=cfg["joint_names"])
    bridge = br.ArmBridge(robot, cfg, "ur5e")
    _run(bridge, robot, 0.2)
    bridge.close_trace()
    assert bridge._trace_f is None
    rows = [json.loads(ln) for ln in f.read_text().splitlines()]
    assert rows and all(r["robot"] == "ur5e" for r in rows)
    bridge.close_trace()                  # idempotent


def test_set_tcp_pose_reports_the_solve_it_moves_on(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    ik = cfg["ik"]
    oz = ik["tcp_offset"][2] + 0.05       # a retargeted tool point
    _run(bridge, robot, 0.5)
    out = bridge.act_set_tcp_pose((0.45, 0.20, 0.10), tcp_offset_z=oz)
    _run_until_done(bridge, robot, out["seq"])
    zs = []
    track = lambda: zs.append(br.forward_kinematics(  # noqa: E731
        ik["chain"], bridge._read_q(), (0.0, 0.0, oz))[2])
    out = bridge.act_set_tcp_pose((0.50, -0.15, 0.10), tcp_offset_z=oz)
    assert bridge.motion[0] == "traj"
    # residuals are FK of the joint vector the motion ends on
    p = br.forward_kinematics(ik["chain"], out["solved_q"], (0.0, 0.0, oz))
    assert abs(math.dist(p, [0.50, -0.15, 0.10]) - out["ik_residual_m"]) < 1e-9
    assert out["ik_residual_m"] < bridge.TCP_TOL_M
    rec = _run_until_done(bridge, robot, out["seq"], on_tick=track)
    # the clearance is measured at the COMMANDED tool point
    assert abs(rec["path_min_tcp_z_m"] - min(zs)) < 2e-3
    assert rec["within_tolerance"] is True


class _FakeHw:
    def __init__(self):
        self.connected = True
        self.calls = []

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return lambda *a: self.calls.append((name, a))


def test_hardware_gets_one_straight_move_and_the_reply_says_so(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    _settled_at(br, bridge, robot, (0.45, 0.20, 0.08))
    hw = _FakeHw()
    bridge.hw, bridge.hw_mirror = hw, False
    out = bridge.act_set_tcp_target((0.50, -0.15, 0.08), wait=False)
    assert bridge.motion[0] == "traj"
    assert [c[0] for c in hw.calls] == ["move_linear"]
    hp = out["hardware_path"]
    assert hp["path_on_hardware"] is False
    assert "SIMULATION only" in hp["note"]
    _run_until_done(bridge, robot, out["seq"])
    out = bridge.act_reset_to_home(wait=False)
    assert out["hardware_path"]["forwarded"] == "reset_to_home"
    desc = {t.name: t.description for t in br.build_arm_tools(bridge)}
    assert "NOT known to retreat" in desc["reset_to_home"]
    assert "does NOT lift" in desc["set_tcp_target"]
