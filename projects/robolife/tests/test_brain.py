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

"""Unit tests for the pure RoboLife robot brain (no engine, no omnisim).

Run: python -m pytest projects/robolife/tests/test_brain.py -q
"""
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from rl import brain as B  # noqa: E402

PI = math.pi


def close(a, b, tol=1e-6):
    return abs(B.wrap_pi(a - b)) < tol if isinstance(a, float) and abs(a) > 3 else abs(a - b) < tol


# ------------------------------------------------------------------ bus
class TestParseBus:
    def test_empty_and_garbage_yield_defaults(self):
        for raw in ("", None, "not json", "[1,2,3]", "42", b""):
            bus = B.parse_bus(raw)
            assert set(bus) == set(B.DEFAULT_BUS)
            assert bus["genome"] == B.DEFAULT_GENOME
            assert bus["modules"] == [] and bus["orders"] == []
            assert bus["batt"] == 1.0

    def test_partial_is_merged_and_coerced(self):
        raw = json.dumps({"batt": "0.42", "orders": "stop",
                          "modules": [{"id": "3", "type": "solar", "x": 1, "y": 2},
                                      {"no_id": 1}, "junk"],
                          "pads": [[1, 2], "bad", [3]],
                          "genome": {"charge_at": 0.4, "module_pref": {"mast": "7"},
                                     "greed": 5}})
        bus = B.parse_bus(raw)
        assert bus["batt"] == 0.42
        assert bus["orders"] == ["stop"]
        assert bus["modules"] == [{"id": 3, "type": "solar", "x": 1.0, "y": 2.0,
                                   "yaw": 0.0, "loose": True}]
        assert bus["pads"] == [[1.0, 2.0]]
        g = bus["genome"]
        assert g["charge_at"] == 0.4
        assert g["module_pref"]["mast"] == 7.0 and g["module_pref"]["battery"] == 1.0
        assert g["greed"] == 1.0            # clamped
        assert g["cruise_speed"] == B.DEFAULT_GENOME["cruise_speed"]

    def test_parse_bus_does_not_alias_defaults(self):
        a = B.parse_bus("")
        a["genome"]["module_pref"]["battery"] = 99
        assert B.DEFAULT_GENOME["module_pref"]["battery"] == 1.0
        assert B.parse_bus("")["genome"]["module_pref"]["battery"] == 1.0


# ------------------------------------------------------------------ kinematics
class TestDiffDrive:
    @pytest.mark.parametrize("v,w", [(0.0, 0.0), (0.5, 0.0), (0.0, 1.0), (0.7, -0.6), (-0.2, 0.3)])
    def test_round_trip(self, v, w):
        left, right = B.diff_drive(v, w)
        v2, w2 = B.wheel_speeds_to_twist(left, right)
        assert abs(v2 - v) < 1e-9 and abs(w2 - w) < 1e-9

    def test_pure_forward_and_pivot_signs(self):
        left, right = B.diff_drive(0.5, 0.0)
        assert abs(left - right) < 1e-12 and left > 0
        assert abs(left - 0.5 / B.WHEEL_RADIUS) < 1e-9
        left, right = B.diff_drive(0.0, 1.0)        # left turn: right wheel faster
        assert right > 0 > left and abs(right + left) < 1e-12
        assert abs((right - left) * B.WHEEL_RADIUS / B.TRACK - 1.0) < 1e-9

    def test_saturation_preserves_arc(self):
        left, right = B.diff_drive(2.0, 1.0)
        assert max(abs(left), abs(right)) > B.WHEEL_MAX_RADPS
        l2, r2 = B.saturate_wheels(left, right)
        assert max(abs(l2), abs(r2)) == pytest.approx(B.WHEEL_MAX_RADPS)
        assert l2 / r2 == pytest.approx(left / right)
        assert B.saturate_wheels(1.0, -1.0) == (1.0, -1.0)


# ------------------------------------------------------------------ geometry
class TestApproachPose:
    @pytest.mark.parametrize("plug_yaw,exp_xy,exp_yaw", [
        (0.0, (1.45, 0.0), PI),          # plug normal +x -> robot east of it, facing -x
        (PI / 2, (0.0, 1.45), -PI / 2),  # plug normal +y -> robot north, facing -y
        (PI, (-1.45, 0.0), 0.0),         # plug normal -x -> robot west, facing +x
    ])
    def test_front_socket(self, plug_yaw, exp_xy, exp_yaw):
        x, y, yaw = B.approach_pose(0.0, 0.0, plug_yaw, 0.9)
        assert (x, y) == pytest.approx(exp_xy, abs=1e-9)
        assert B.wrap_pi(yaw - exp_yaw) == pytest.approx(0.0, abs=1e-9)
        # The FRONT socket sits exactly `standoff` out along the plug normal.
        sx, sy = B.socket_position((x, y, yaw), "front")
        assert math.hypot(sx, sy) == pytest.approx(0.9, abs=1e-9)
        assert B.wrap_pi(math.atan2(sy, sx) - plug_yaw) == pytest.approx(0.0, abs=1e-9)
        # ...and a straight creep along the robot's heading reaches the plug.
        creep = (sx + 0.9 * math.cos(yaw), sy + 0.9 * math.sin(yaw))
        assert creep == pytest.approx((0.0, 0.0), abs=1e-9)

    @pytest.mark.parametrize("plug_yaw", [0.0, PI / 2, PI, -2.1])
    def test_rear_socket_backs_in(self, plug_yaw):
        x, y, yaw = B.approach_pose(3.0, -1.0, plug_yaw, 0.9, socket="rear")
        assert B.wrap_pi(yaw - plug_yaw) == pytest.approx(0.0, abs=1e-9)
        sx, sy = B.socket_position((x, y, yaw), "rear")
        assert math.hypot(sx - 3.0, sy + 1.0) == pytest.approx(0.9, abs=1e-9)

    def test_offset_plug_translates(self):
        x, y, yaw = B.approach_pose(2.0, 5.0, PI / 2, 0.5)
        assert (x, y) == pytest.approx((2.0, 5.0 + 0.5 + B.SOCKET_OFFSET))

    def test_module_plug_pose_from_bus_record(self):
        m = {"id": 1, "type": "battery", "x": 4.0, "y": 0.0, "yaw": 0.0}
        px, py, pyaw = B.module_plug_pose(m)
        assert (px, py) == pytest.approx((4.0 - 0.17, 0.0))   # the -x face
        assert abs(B.wrap_pi(pyaw - PI)) < 1e-9
        m["yaw"] = PI / 2                                      # -x face now points -y
        px, py, pyaw = B.module_plug_pose(m)
        assert (px, py) == pytest.approx((4.0, -0.17))
        assert pyaw == pytest.approx(-PI / 2)


class TestHeadingAndGoTo:
    def test_heading_error_sign(self):
        assert B.heading_error((0, 0, 0), (1, 1)) == pytest.approx(PI / 4)     # left
        assert B.heading_error((0, 0, 0), (1, -1)) == pytest.approx(-PI / 4)   # right
        assert abs(B.heading_error((0, 0, PI / 2), (0, 5))) < 1e-12

    def test_go_to_turns_toward_and_slows_near(self):
        v, w = B.go_to((0, 0, 0), (0, 5), 1.0)          # target 90 deg left
        assert w > 0 and v == 0.0                        # pivot first
        v, w = B.go_to((0, 0, 0), (5, 0), 1.0)
        assert v == pytest.approx(1.0) and w == 0.0
        v_near, _ = B.go_to((0, 0, 0), (0.5, 0), 1.0)
        assert 0 < v_near < 1.0
        assert B.go_to((0, 0, 0), (0.05, 0), 1.0) == (0.0, 0.0)

    def test_go_to_final_yaw_and_caution(self):
        v, w = B.go_to((0, 0, 0), (0.05, 0, PI / 2), 1.0)
        assert v == 0.0 and w > 0
        v, _ = B.go_to((0, 0, 0), (5, 0), 1.0, caution_stop=0.3)
        assert v == 0.0
        v, _ = B.go_to((0, 0, 0), (5, 0), 1.0, caution_stop=0.8)
        assert 0 < v < 1.0


# ------------------------------------------------------------------ lidar
def scan(n=180, fov=PI, fill=10.0, hits=()):
    """A left-to-right scan; hits = [(angle_deg, range)] with + = left."""
    r = [fill] * n
    step = fov / (n - 1)
    for deg, rng in hits:
        i = int(round((0.5 * fov - math.radians(deg)) / step))
        r[i] = rng
    return r


class TestLidarGuard:
    def test_clear_scan(self):
        assert B.lidar_guard(scan(), PI, 1.0) == (False, 0.0)
        assert B.lidar_guard([], PI, 1.0) == (False, 0.0)
        assert B.lidar_guard(None, PI, 1.0) == (False, 0.0)

    def test_blocks_straight_ahead_only(self):
        blocked, _ = B.lidar_guard(scan(hits=[(0, 0.5)]), PI, 1.0)
        assert blocked
        for deg in (60, -60, 85, -85):                      # outside +-40 deg
            blocked, bias = B.lidar_guard(scan(hits=[(deg, 0.5)]), PI, 1.0)
            assert not blocked and bias == 0.0
        blocked, _ = B.lidar_guard(scan(hits=[(0, 1.5)]), PI, 1.0)   # beyond caution
        assert not blocked

    def test_bias_steers_away(self):
        blocked, bias = B.lidar_guard(scan(hits=[(20, 0.5)]), PI, 1.0)   # obstacle LEFT
        assert blocked and bias < 0                                       # -> turn right
        blocked, bias = B.lidar_guard(scan(hits=[(-20, 0.5)]), PI, 1.0)  # obstacle RIGHT
        assert blocked and bias > 0
        # nearer = stronger
        _, b_near = B.lidar_guard(scan(hits=[(-20, 0.2)]), PI, 1.0)
        _, b_far = B.lidar_guard(scan(hits=[(-20, 0.9)]), PI, 1.0)
        assert b_near > b_far > 0

    def test_dead_ahead_picks_freer_side(self):
        # wall ahead, extra clutter on the right -> go left
        s = scan(hits=[(0, 0.5)] + [(-d, 1.2) for d in range(5, 40, 5)])
        blocked, bias = B.lidar_guard(s, PI, 1.0)
        assert blocked and bias == 1.0

    def test_non_returns_ignored(self):
        s = scan(hits=[(0, float("inf")), (5, float("nan")), (-5, 0.0)])
        assert B.lidar_guard(s, PI, 1.0) == (False, 0.0)
        assert B.lidar_min(s) == 10.0
        assert B.lidar_min([float("inf")]) is None


# ------------------------------------------------------------------ module choice
class TestChooseModule:
    MODS = [{"id": 1, "type": "battery", "x": 3.0, "y": 0.0, "yaw": 0.0, "loose": True},
            {"id": 2, "type": "solar", "x": 0.0, "y": 3.0, "yaw": 0.0, "loose": True}]

    def test_preference_weights_decide_between_equidistant(self):
        g = B.parse_genome({"module_pref": {"battery": 2.0, "solar": 0.5}, "greed": 1.0})
        m, s = B.choose_module(self.MODS, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0)
        assert m["id"] == 1 and s == "front"
        g = B.parse_genome({"module_pref": {"battery": 0.5, "solar": 2.0}, "greed": 1.0})
        m, _ = B.choose_module(self.MODS, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0)
        assert m["id"] == 2

    def test_distance_divides(self):
        mods = [dict(self.MODS[0], x=1.0), dict(self.MODS[1], y=6.0, type="battery")]
        g = B.parse_genome({"greed": 1.0})
        m, _ = B.choose_module(mods, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0)
        assert m["id"] == 1
        # a 7x preference beats a 6x distance
        g = B.parse_genome({"greed": 1.0, "module_pref": {"battery": 1.0}})
        mods[1]["type"] = "mast"
        g["module_pref"]["mast"] = 7.0
        m, _ = B.choose_module(mods, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0)
        assert m["id"] == 2

    def test_zero_weight_greed_blacklist_and_sockets(self):
        g = B.parse_genome({"module_pref": {"battery": 0.0, "solar": 0.0}, "greed": 1.0})
        assert B.choose_module(self.MODS, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0) == (None, None)
        g = B.parse_genome({"greed": 0.0})
        assert B.choose_module(self.MODS, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0) == (None, None)
        g = B.parse_genome({"greed": 1.0})
        m, s = B.choose_module(self.MODS, (0, 0, 0), g, {"front": 7, "rear": None}, {}, 0.0)
        assert m is not None and s == "rear"
        assert B.choose_module(self.MODS, (0, 0, 0), g, {"front": 7, "rear": 8}, {}, 0.0) == (None, None)
        m, _ = B.choose_module(self.MODS, (0, 0, 0), g, {"front": None, "rear": None}, {1: 100.0}, 5.0)
        assert m["id"] == 2
        taken = [dict(self.MODS[0], loose=False), self.MODS[1]]
        m, _ = B.choose_module(taken, (0, 0, 0), g, {"front": None, "rear": None}, {}, 0.0)
        assert m["id"] == 2


# ------------------------------------------------------------------ the brain
def bus(**kw):
    b = B.parse_bus("")
    b["pads"] = [[10.0, 10.0]]
    b["genome"] = B.parse_genome({"greed": 1.0, "charge_at": 0.3, "cruise_speed": 0.8})
    b.update(kw)
    return b


NO_LIDAR = None


class TestBrainTransitions:
    def test_battery_threshold_seek_charge_charging_explore(self):
        br = B.Brain(0)
        out = br.step(bus(batt=0.8), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.0)
        assert out["state"] == "explore"
        out = br.step(bus(batt=0.29), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.1)
        assert out["state"] == "seek_charge" and "seek_charge" in out["note"]
        # heading toward the pad at (10,10): pivots left first, never backwards
        assert out["w"] > 0 and out["v"] >= 0
        # still below threshold, at the pad -> charging, wheels stopped
        out = br.step(bus(batt=0.31), (9.8, 9.9, 0.5), (0.3, 0), NO_LIDAR, 0, 0, 0.2)
        assert out["state"] == "charging" and out["v"] == 0.0 and out["w"] == 0.0
        out = br.step(bus(batt=0.85), (9.8, 9.9, 0.5), (0, 0), NO_LIDAR, 0, 0, 0.3)
        assert out["state"] == "charging"
        out = br.step(bus(batt=0.91), (9.8, 9.9, 0.5), (0, 0), NO_LIDAR, 0, 0, 0.4)
        assert out["state"] == "explore"

    def test_no_pads_means_no_seek_charge(self):
        br = B.Brain(0)
        out = br.step(bus(batt=0.1, pads=[]), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.0)
        assert out["state"] == "explore"

    def test_stop_order_overrides_and_lifts(self):
        br = B.Brain(1)
        br.step(bus(batt=0.9), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.0)
        out = br.step(bus(batt=0.9, orders=["stop"]), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.1)
        assert out["state"] == "stopped" and out["v"] == 0.0 and out["w"] == 0.0
        # stays stopped and silent while the order persists, even at low battery
        out = br.step(bus(batt=0.05, orders=["stop"]), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.2)
        assert out["state"] == "stopped" and out["v"] == 0.0
        out = br.step(bus(batt=0.05, state_hint="dead"), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.3)
        assert out["state"] == "stopped"
        out = br.step(bus(batt=0.9, orders=["none"]), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.4)
        assert out["state"] == "explore"

    def test_release_orders_yield_unlock(self):
        br = B.Brain(2)
        br.docked = {"front": 4, "rear": 9}
        out = br.step(bus(orders=["release_rear"]), (0, 0, 0), (0, 0), NO_LIDAR, 1, 1, 0.0)
        assert out["unlock_rear"] is True and out["unlock_front"] is False
        assert br.docked == {"front": 4, "rear": None}
        # the dropped module is ignored for a cooldown and the robot pulls
        # FORWARD off a rear release (the module is sitting at the socket)
        assert br.blacklist[9] == pytest.approx(0.0 + B.RELEASE_COOLDOWN_S)
        assert out["v"] > 0 and out["w"] == 0.0
        out = br.step(bus(orders=["release_rear"]), (0, 0, 0), (0, 0), NO_LIDAR, 1, 1, 0.1)
        assert out["unlock_rear"] is False                     # nothing left to release
        out = br.step(bus(orders=["release_front", "stop"]), (0, 0, 0), (0, 0), NO_LIDAR, 1, 1, 0.2)
        assert out["unlock_front"] is True and out["state"] == "stopped"
        assert br.docked == {"front": None, "rear": None}
        assert out["v"] == 0.0                                 # stop wins over the back-away

    def test_front_release_backs_away_then_resumes(self):
        mods = [{"id": 4, "type": "battery", "x": 0.72, "y": 0.0, "yaw": 0.0, "loose": True}]
        br = B.Brain(2)
        br.docked["front"] = 4
        out = br.step(bus(orders=["release_front"], modules=mods), (0, 0, 0), (0, 0), NO_LIDAR, 1, 0, 0.0)
        assert out["unlock_front"] and out["v"] < 0 and out["state"] == "explore"
        # while still within CLEAR_M of the release point it keeps reversing,
        # even though the module is loose, adjacent and preferred
        out = br.step(bus(modules=mods), (-0.3, 0, 0), (-0.3, 0), NO_LIDAR, 1, 0, 1.0)
        assert out["v"] < 0 and out["state"] == "explore"
        out = br.step(bus(modules=mods), (-0.7, 0, 0), (-0.3, 0), NO_LIDAR, 0, 0, 2.5)
        assert out["v"] >= 0 and out["state"] == "explore"     # clear done, module blacklisted
        out = br.step(bus(modules=mods), (-0.7, 0, 0), (0, 0), NO_LIDAR, 0, 0, B.RELEASE_COOLDOWN_S + 1)
        assert out["state"] == "seek_module" and out["target"] == 4

    def test_guard_side_is_committed_while_blocked(self):
        br = B.Brain(8)
        b = bus(batt=1.0)
        wall = scan(hits=[(0, 0.4)])                    # dead ahead
        clutter_r = scan(hits=[(0, 0.4)] + [(-d, 1.2) for d in range(5, 40, 5)])
        clutter_l = scan(hits=[(0, 0.4)] + [(d, 1.2) for d in range(5, 40, 5)])
        br._waypoint = [20.0, 0.0]
        br._wp_t0 = 0.0
        first = br.step(b, (0, 0, 0), (0, 0), (clutter_r, PI), 0, 0, 0.1)
        assert first["w"] > 0                            # freer side is left
        signs = set()
        for i, s in enumerate((clutter_l, clutter_r, wall, clutter_l)):
            br._waypoint = [20.0, 0.0]
            out = br.step(b, (0, 0, 0), (0, 0), (s, PI), 0, 0, 0.2 + i * 0.008)
            signs.add(out["w"] > 0)
        assert signs == {True}                           # never flipped
        br._waypoint = [20.0, 0.0]
        out = br.step(b, (0, 0, 0), (0, 0), (scan(), PI), 0, 0, 0.5)
        assert out["v"] > 0 and abs(out["w"]) < 1e-9     # unblocked -> resumes
        assert br._avoid_dir == 0.0

    def test_blocked_too_long_escapes_backwards(self):
        br = B.Brain(9)
        b = bus(batt=1.0)
        wall = scan(hits=[(0, 0.4)])
        t = 0.0
        out = None
        for _ in range(int(B.BLOCKED_ESCAPE_S / 0.008) + 5):
            br._waypoint = [20.0, 0.0]
            br._wp_t0 = t
            out = br.step(b, (0, 0, 0), (0, 0), (wall, PI), 0, 0, t)
            t += 0.008
        assert out["v"] < 0 and out["w"] == 0.0
        assert br._escape_until is not None and br._escape_until > t

    def test_explore_picks_module_by_preference(self):
        mods = [{"id": 1, "type": "battery", "x": 3.0, "y": 0.0, "yaw": 0.0, "loose": True},
                {"id": 2, "type": "solar", "x": -3.0, "y": 0.0, "yaw": 0.0, "loose": True}]
        b = bus(modules=mods)
        b["genome"]["module_pref"] = {"battery": 0.1, "solar": 3.0, "mast": 1, "armor": 1}
        br = B.Brain(3)
        out = br.step(b, (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.0)
        assert out["state"] == "seek_module" and out["target"] == 2
        b["modules"][1]["loose"] = False
        out = br.step(b, (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.1)
        assert out["state"] == "explore" and out["target"] is None

    def test_lost_module_returns_to_explore(self):
        mods = [{"id": 1, "type": "battery", "x": 3.0, "y": 0.0, "yaw": 0.0, "loose": True}]
        br = B.Brain(3)
        assert br.step(bus(modules=mods), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.0)["state"] == "seek_module"
        assert br.step(bus(modules=[]), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 2.0)["state"] == "seek_module"
        assert br.step(bus(modules=[]), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 5.0)["state"] == "explore"

    def test_explore_lidar_guard_slows_and_steers(self):
        br = B.Brain(4)
        b = bus(batt=1.0)
        b["genome"]["caution"] = 1.0
        # aim the waypoint straight ahead so the unguarded command is pure forward
        br._waypoint = [20.0, 0.0]
        br._wp_t0 = 0.0
        free = br.step(b, (0, 0, 0), (0, 0), (scan(), PI), 0, 0, 0.1)
        assert free["v"] > 0.5 and abs(free["w"]) < 1e-9
        br._waypoint = [20.0, 0.0]
        blocked = br.step(b, (0, 0, 0), (0, 0), (scan(hits=[(10, 0.4)]), PI), 0, 0, 0.2)
        assert blocked["v"] == 0.0 and blocked["w"] < 0        # obstacle left -> pivot right

    def test_status_shape(self):
        br = B.Brain(5)
        br.docked["front"] = 3
        s = br.status(0.123456, -0.2, 1.23456)
        assert s == {"state": "explore", "v": 0.1235, "w": -0.2,
                     "docked": {"front": 3, "rear": None}, "target": None, "lidar_min": 1.235}
        assert br.status(0, 0, None)["lidar_min"] is None
        assert len(json.dumps(s)) < 1024

    def test_w_gain_learns_from_measured_rate(self):
        br = B.Brain(6)
        b = bus(batt=1.0)
        pose = (0.0, 0.0, 0.0)
        br._waypoint = [0.0, 20.0]        # 90 deg left -> steady pivot command
        t = 0.0
        for _ in range(400):
            br._waypoint = [0.0, 20.0]
            br._wp_t0 = t
            out = br.step(b, pose, (0.0, 0.35 * br._w_cmd_prev), NO_LIDAR, 0, 0, t)
            t += 0.008
        assert out["w"] > 0
        assert abs(br.w_gain - 0.35) < 0.03
        # and the correction pushes harder than the desired rate
        assert out["w"] > B.W_MAX


# ------------------------------------------------------------------ docking flow
class KinSim:
    """Perfect-actuation kinematic robot for the docking integration test."""

    def __init__(self, pose, dt=0.008):
        self.pose = list(pose)
        self.dt = dt
        self.v = self.w = 0.0

    def apply(self, v, w):
        x, y, yaw = self.pose
        self.pose = [x + v * math.cos(yaw) * self.dt, y + v * math.sin(yaw) * self.dt,
                     B.wrap_pi(yaw + w * self.dt)]
        self.v, self.w = v, w


def presence_for(pose, plug, socket, tol=0.08):
    sx, sy = B.socket_position(pose, socket)
    return 1 if math.hypot(sx - plug[0], sy - plug[1]) < tol else 0


def run_dock(module, start, socket_presence=True, docked=None, max_s=120.0):
    b = bus(modules=[module])
    b["genome"]["module_pref"] = {"battery": 1, "solar": 1, "mast": 1, "armor": 1}
    br = B.Brain(7)
    if docked:
        br.docked.update(docked)
    sim = KinSim(start)
    plug = B.module_plug_pose(module)
    t, states, notes, out = 0.0, [], [], None
    while t < max_s:
        pf = presence_for(sim.pose, plug, "front") if socket_presence else 0
        pr = presence_for(sim.pose, plug, "rear") if socket_presence else 0
        out = br.step(b, sim.pose, (sim.v, sim.w), NO_LIDAR, pf, pr, t)
        if out["note"]:
            notes.append((round(t, 2), out["note"]))
        if not states or states[-1] != out["state"]:
            states.append(out["state"])
        if out["lock_front"] or out["lock_rear"]:
            return br, sim, states, notes, out, t
        if br.state == "explore" and br.blacklist:
            return br, sim, states, notes, out, t
        sim.apply(out["v"], out["w"])
        t += sim.dt
    return br, sim, states, notes, out, t


class TestDockingFlow:
    @pytest.mark.parametrize("myaw", [0.0, PI / 2, PI, -0.7])
    def test_front_dock_locks_on_presence(self, myaw):
        module = {"id": 5, "type": "battery", "x": 4.0, "y": 2.0, "yaw": myaw, "loose": True}
        br, sim, states, notes, out, t = run_dock(module, (0.0, 0.0, 0.3))
        assert states == ["seek_module", "dock", "explore"], (states, notes)
        assert out["lock_front"] is True and out["lock_rear"] is False
        assert br.docked == {"front": 5, "rear": None} and br.target is None
        # the socket really is at the plug, and the robot is nose-on to it
        plug = B.module_plug_pose(module)
        sx, sy = B.socket_position(sim.pose, "front")
        assert math.hypot(sx - plug[0], sy - plug[1]) < 0.08
        assert abs(B.wrap_pi(sim.pose[2] - (plug[2] + PI))) < 0.1
        assert t < 60.0

    def test_rear_dock_when_front_is_full(self):
        module = {"id": 6, "type": "armor", "x": -3.0, "y": 1.0, "yaw": 1.0, "loose": True}
        br, sim, states, notes, out, t = run_dock(module, (0.0, 0.0, 0.0), docked={"front": 1})
        assert states == ["seek_module", "dock", "explore"], (states, notes)
        assert out["lock_rear"] is True and br.docked == {"front": 1, "rear": 6}
        plug = B.module_plug_pose(module)
        sx, sy = B.socket_position(sim.pose, "rear")
        assert math.hypot(sx - plug[0], sy - plug[1]) < 0.08

    def test_no_presence_backs_off_retries_once_then_blacklists(self):
        module = {"id": 8, "type": "solar", "x": 3.0, "y": 0.0, "yaw": 0.0, "loose": True}
        br, sim, states, notes, out, t = run_dock(module, (0.0, 0.0, 0.0), socket_presence=False)
        assert out["lock_front"] is False and br.docked["front"] is None
        assert br.blacklist.get(8, 0) > t
        assert states == ["seek_module", "dock", "explore"]
        joined = " | ".join(n for _, n in notes)
        assert "back off" in joined and "retry 1" in joined and "blacklisted" in joined
        assert joined.count("back off") == 2          # first attempt + one retry
        # the module is ignored while blacklisted, chased again after
        b = bus(modules=[module])
        assert br.step(b, sim.pose, (0, 0), NO_LIDAR, 0, 0, t + 1.0)["state"] == "explore"
        assert br.step(b, sim.pose, (0, 0), NO_LIDAR, 0, 0, t + B.BLACKLIST_S + 1.0)["state"] == "seek_module"

    def test_low_battery_aborts_dock(self):
        module = {"id": 9, "type": "mast", "x": 3.0, "y": 0.0, "yaw": 0.0, "loose": True}
        b = bus(modules=[module])
        br = B.Brain(7)
        br.step(b, (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.0)
        assert br.state == "seek_module"
        out = br.step(bus(modules=[module], batt=0.1), (0, 0, 0), (0, 0), NO_LIDAR, 0, 0, 0.1)
        assert out["state"] == "seek_charge" and br.target is None and br._dock == {}
