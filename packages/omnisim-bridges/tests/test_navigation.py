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
"""Route planning around keep-out zones and learned obstacles."""
import math

from omnisim_bridges.navigation import NavMap, obstacle_ahead


def _legs_clear(nav, start, wps, zones=(), bounds=()):
    pts = [start] + list(wps)
    return all(nav.segment_clear(a, b, zones, bounds) for a, b in zip(pts, pts[1:]))


def test_a_clear_line_is_one_direct_leg():
    nav = NavMap(6, 6)
    r = nav.plan((0, 0), (3, 0))
    assert r["ok"] and r["direct"] and r["waypoints"] == [(3, 0)]


def test_routes_around_an_aisle_instead_of_through_it():
    nav = NavMap(6, 6)
    aisle = [{"name": "aisle", "x_min": 1.0, "x_max": 2.0, "y_min": -2.0, "y_max": 2.0}]
    r = nav.plan((0, 0), (3.5, 0), zones=aisle)
    assert r["ok"] and not r["direct"] and len(r["waypoints"]) >= 2
    assert _legs_clear(nav, (0, 0), r["waypoints"], aisle)


def test_a_goal_inside_a_zone_is_refused_as_impossible():
    nav = NavMap(6, 6)
    aisle = [{"name": "aisle", "x_min": 1.0, "x_max": 2.0, "y_min": -6.0, "y_max": 6.0}]
    r = nav.plan((0, 0), (1.5, 0), zones=aisle)
    assert not r["ok"] and r["hard"]


def test_a_full_width_aisle_leaves_no_route():
    nav = NavMap(6, 6)
    wall = [{"name": "aisle", "x_min": 1.0, "x_max": 2.0, "y_min": -6.0, "y_max": 6.0}]
    r = nav.plan((0, 0), (4, 0), zones=wall)
    assert not r["ok"] and not r["hard"]


def test_routes_around_a_learned_obstacle_and_rejoins_the_goal():
    nav = NavMap(6, 6)
    ob = obstacle_ahead(1.35, 0.0, 0.0, half_length=0.5)
    nav.learn_obstacle(ob["x"], ob["y"], ob["radius"])
    # The robot stands right against it: leaving the margin is allowed.
    r = nav.plan((1.35, 0.0), (4.0, 0.0))
    assert r["ok"] and not r["direct"]
    assert r["waypoints"][-1] == (4.0, 0.0)
    worst = min(math.hypot(x - ob["x"], y - ob["y"]) for x, y in r["waypoints"][:-1])
    assert worst >= ob["radius"]


def test_a_line_boundary_can_be_driven_up_to_but_not_past():
    nav = NavMap(6, 6)
    b = [{"axis": "x", "max": 3.0}]
    assert nav.plan((0, 0), (3.0, 0), bounds=b)["ok"]
    r = nav.plan((0, 0), (4.0, 0), bounds=b)
    assert not r["ok"] and r["hard"]


def test_site_edges_are_respected():
    nav = NavMap(6, 6)
    assert not nav.plan((0, 0), (5.9, 0))["ok"]
