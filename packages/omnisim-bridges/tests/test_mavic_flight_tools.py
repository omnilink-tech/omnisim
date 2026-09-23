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

"""The drone's model-facing flight verbs must WAIT, and report what happened.

WHY THIS EXISTS. Measured 2026-09-23 on a live Gemini take: asked to "land on
the blue pad", the model sent `move_body` and `land` in ONE turn. Each verb
returned {"accepted": true} the instant it set a target, and `land` pins the
touchdown point to the CURRENT position -- so the aircraft came straight down
where it hovered, 2.55 m short, while the reply said it was "moving to the
blue pad and landing". The fix makes `takeoff` / `move_body` / `turn` / `land`
block until the aircraft settles and return {commanded, achieved, settled,
error_m}, with budgets in SIM seconds (under capture that world ran at 0.06x
real time) and a wall cap for a held world.

The bridge module needs the simulator's `omnisim` library at import time, so,
like test_mavic_stop_verb.py, this reads the SOURCE and executes only the
flight helpers against a small fake flight loop.
"""
from __future__ import annotations

import ast
import importlib.util
import math
import pathlib
import threading
import time
from types import SimpleNamespace

import pytest

CTRL = (pathlib.Path(__file__).resolve().parents[3]
        / "projects" / "samples" / "demos" / "controllers" / "mavic_omnilink_bridge")
BRIDGE = CTRL / "mavic_omnilink_bridge.py"
HELPERS = {"_drone_tools", "_flight_snapshot", "_still", "_await_flight", "_flight_report"}
CONSTS = {"_REACH_XY_M", "_REACH_Z_M", "_STILL_V_XY", "_STILL_V_Z", "_FLIGHT_WALL_CAP_S",
          "DEFAULT_TAKEOFF_ALTITUDE", "_MODEL_TAKEOFF_DEFAULT_M"}


class FakeStateBridge:
    """The `_StateBridge` verbs, with the real semantics that matter here:
    targets are set and the call returns at once; `land` pins the touchdown
    point to the CURRENT position."""

    def __init__(self, state):
        self.state = state

    def act_takeoff(self, altitude):
        with self.state.lock:
            self.state.target_altitude = max(0.5, float(altitude))
            self.state.target_x, self.state.target_y = self.state.x, self.state.y
            self.state.mode = "takeoff"

    def act_move_body(self, forward=0.0, vertical=0.0):
        with self.state.lock:
            s = self.state
            s.target_x = s.x + forward * math.cos(s.yaw)
            s.target_y = s.y + forward * math.sin(s.yaw)
            s.target_altitude = max(0.5, s.target_altitude + vertical)
            s.mode = "goto"

    def act_land(self):
        with self.state.lock:
            s = self.state
            s.target_x, s.target_y, s.target_altitude, s.mode = s.x, s.y, 0.0, "land"

    def act_turn(self, angle_rad):
        with self.state.lock:
            self.state.target_yaw = self.state.yaw + float(angle_rad)

    def act_hover(self): return {"accepted": True}
    def act_stop(self): return {"accepted": True}
    def act_reset_to_home(self): return {"accepted": True}


def _load(wall_cap_s: float = 300.0) -> dict:
    tree = ast.parse(BRIDGE.read_text(encoding="utf-8"))
    body = [n for n in tree.body
            if (isinstance(n, ast.FunctionDef) and n.name in HELPERS)
            or (isinstance(n, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in CONSTS for t in n.targets))]
    found = {n.name for n in body if isinstance(n, ast.FunctionDef)}
    assert found == HELPERS, f"missing helpers: {HELPERS - found}"
    ns = {"math": math, "time": time, "_StateBridge": FakeStateBridge}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(BRIDGE), "exec"), ns)
    ns["_FLIGHT_WALL_CAP_S"] = wall_cap_s
    return ns


def _state(**kw):
    s = SimpleNamespace(lock=threading.Lock(), x=0.0, y=0.0, z=0.04, yaw=0.0,
                        v_xy=0.0, v_z=0.0, mode="idle", fault=None, sim_time=0.0,
                        target_x=None, target_y=None, target_altitude=0.0,
                        target_yaw=None)
    s.__dict__.update(kw)
    return s


class FakeFlight:
    """Moves the fake aircraft toward its targets, one 16 ms SIM step per
    ~1 ms of wall time. `rate=0` freezes sim time (a held world)."""

    def __init__(self, state, rate: float = 1.0):
        self.s, self.rate, self.stop = state, rate, threading.Event()
        self.t = threading.Thread(target=self.run, daemon=True)

    def __enter__(self):
        self.t.start()
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.t.join()

    def run(self):
        dt, v = 0.016, 1.0
        while not self.stop.is_set():
            time.sleep(0.001)
            if not self.rate:
                continue
            with self.s.lock:
                s = self.s
                s.sim_time += dt
                vx = vy = vz = 0.0
                if s.mode in ("takeoff", "goto", "hover", "land") and s.target_x is not None:
                    dx, dy = s.target_x - s.x, s.target_y - s.y
                    d = math.hypot(dx, dy)
                    if d > 1e-4:
                        step = min(d, v * dt)
                        vx, vy = dx / d * step / dt, dy / d * step / dt
                        s.x += dx / d * step
                        s.y += dy / d * step
                    dz = s.target_altitude - s.z
                    if abs(dz) > 1e-4:
                        stepz = math.copysign(min(abs(dz), v * dt), dz)
                        s.z += stepz
                        vz = stepz / dt
                    if s.mode == "land" and s.z < 0.4:
                        s.mode, s.z, vz = "landed", 0.04, 0.0
                    elif s.mode == "takeoff" and abs(s.target_altitude - s.z) < 0.05:
                        s.mode = "hover"
                if s.target_yaw is not None:
                    s.yaw += max(-0.05, min(0.05, s.target_yaw - s.yaw))
                s.v_xy, s.v_z = math.hypot(vx, vy), vz


def test_land_after_move_touches_down_at_the_destination_not_the_start():
    ns = _load()
    st = _state(z=1.0, mode="hover", target_altitude=1.0)
    tools = ns["_drone_tools"](st)
    with FakeFlight(st):
        moved = tools["move_body"][0]({"forward": 2.0})
        landed = tools["land"][0]({})
    assert moved["settled"] is True and moved["error_m"] < 0.10
    assert moved["achieved"]["x"] == pytest.approx(2.0, abs=0.15)
    assert landed["settled"] is True and landed["mode"] == "landed"
    # The defect: an immediate land used to come down at x = 0.
    assert landed["achieved"]["x"] == pytest.approx(2.0, abs=0.15)


def test_takeoff_waits_for_altitude_and_reports_it():
    ns = _load()
    st = _state()
    tools = ns["_drone_tools"](st)
    with FakeFlight(st):
        out = tools["takeoff"][0]({"altitude": 1.0})
    assert out["settled"] is True
    assert out["achieved"]["altitude_m"] == pytest.approx(1.0, abs=0.15)
    assert out["commanded"] == {"altitude_m": 1.0}


def test_a_takeoff_with_no_height_is_a_low_hop_never_the_12_m_default():
    # Measured on the x500 take: refused twice for an invented height, the
    # model sent takeoff{} and the 12 m default took it far out of frame --
    # a bigger climb than either refused request. Omission must never pay.
    ns = _load()
    ns["_await_flight"] = lambda state, done, budget: (ns["_flight_snapshot"](state), False, "timeout")
    st = _state()
    out = ns["_drone_tools"](st)["takeoff"][0]({})
    assert out["commanded"]["altitude_m"] == ns["_MODEL_TAKEOFF_DEFAULT_M"] == 1.5
    assert ns["_MODEL_TAKEOFF_DEFAULT_M"] < ns["DEFAULT_TAKEOFF_ALTITUDE"]
    with st.lock:
        assert st.target_altitude == 1.5


def test_reset_returns_to_the_authored_pose_not_the_mavic_world_spawn():
    spec = importlib.util.spec_from_file_location("mavic_chat_router_ut", CTRL / "mavic_chat_router.py")
    router = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(router)
    st = _state(authored_pose={"x": 0.0, "y": 0.0, "z": 0.25, "yaw": 0.0},
                xy_i_fwd=1.2, xy_i_right=-0.7, mission_complete=True, mission_log=[1])
    router._act_reset(st)
    assert st.reset_request == {"x": 0.0, "y": 0.0, "z": 0.25, "yaw": 0.0}
    assert st.reset_anchor == st.reset_request and st.reset_anchor is not st.reset_request
    assert st.xy_i_fwd == 0.0 and st.xy_i_right == 0.0


def test_a_held_world_ends_the_wait_on_the_wall_cap():
    ns = _load(wall_cap_s=0.3)
    st = _state(z=1.0, mode="hover", target_altitude=1.0)
    with FakeFlight(st, rate=0):
        t0 = time.time()
        out = ns["_drone_tools"](st)["move_body"][0]({"forward": 2.0})
    assert out["settled"] is False and out["reason"] == "wall_timeout"
    assert time.time() - t0 < 2.0


def test_the_budget_is_counted_in_sim_seconds():
    ns = _load(wall_cap_s=30.0)
    st = _state(z=1.0, mode="hover", target_altitude=1.0)
    with FakeFlight(st):
        t0 = time.time()
        s, ok, why = ns["_await_flight"](st, lambda s: False, 0.5)
    # The fake runs ~16x real time and the helper polls every 50 ms of wall
    # time, so it may overshoot by one poll -- but it ends on the SIM budget,
    # long before the 30 s wall cap.
    assert ok is False and why == "timeout"
    assert 0.5 <= s["sim_time"] < 2.0
    assert time.time() - t0 < 5.0


def test_no_progress_is_advisory_but_a_hard_fault_ends_the_wait():
    ns = _load()
    st = _state(fault="no_progress")
    with FakeFlight(st):
        _, ok, why = ns["_await_flight"](st, lambda s: s["sim_time"] > 0.3, 5.0)
    assert ok is True and why is None
    st = _state(fault="crashed")
    _, ok, why = ns["_await_flight"](st, lambda s: True, 5.0)
    assert ok is False and why == "crashed"


def test_state_read_carries_heading():
    ns = _load()
    st = _state(yaw=math.pi / 2)
    out = ns["_drone_tools"](st)["get_robot_state"][0]({})
    assert out["heading_deg"] == pytest.approx(90.0)
    assert set(out) >= {"x", "y", "altitude_m", "mode", "fault"}


# ── mavic_dynamics: a declared airframe overrides; none changes nothing ──

def _dynamics():
    spec = importlib.util.spec_from_file_location("mavic_dynamics_ut", CTRL / "mavic_dynamics.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Node:
    def __init__(self):
        self.forces, self.torques = [], []

    def addForceWithOffset(self, f, o, rel):
        self.forces.append((f, o))

    def addTorque(self, t, rel):
        self.torques.append(t)


def test_no_airframe_keeps_the_mavic_constants_exactly():
    md = _dynamics()
    rd = md.RotorDynamics(_Node())
    assert rd.k_thrust == 0.00054 and rd.k_torque == 0.0000052
    assert rd.props == (md._PROP_FL, md._PROP_FR, md._PROP_RL, md._PROP_RR)


def test_a_declared_airframe_moves_the_anchors_and_the_coefficients():
    md = _dynamics()
    node = _Node()
    af = {"k_thrust": 0.0011, "k_torque": 1e-5,
          "props": {"fl": [0.174, 0.174, 0.06], "fr": [0.174, -0.174, 0.06],
                    "rl": [-0.174, 0.174, 0.06], "rr": [-0.174, -0.174, 0.06]}}
    md.RotorDynamics(node, af).step(10.0, 10.0, 10.0, 10.0)
    assert [o for _, o in node.forces] == [[0.174, 0.174, 0.06], [0.174, -0.174, 0.06],
                                           [-0.174, 0.174, 0.06], [-0.174, -0.174, 0.06]]
    assert node.forces[0][0][2] == pytest.approx(0.0011 * 100.0)


@pytest.mark.parametrize("raw", ["", "not json", "[]", '{"other": 1}', '{"rotor_dynamics": 5}'])
def test_custom_data_without_an_airframe_is_ignored(raw):
    assert _dynamics().airframe_from_custom_data(raw) is None


def test_custom_data_airframe_is_read():
    got = _dynamics().airframe_from_custom_data('{"rotor_dynamics": {"k_thrust": 0.001}}')
    assert got == {"k_thrust": 0.001}
