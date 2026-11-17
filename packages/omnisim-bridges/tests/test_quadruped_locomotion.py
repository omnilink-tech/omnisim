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

"""The quadruped bridge's `--locomotion crawl|trot`: engine-free checks.

`_crawl_motion.CrawlDriver` is pure Python (the gait model it drives imports
the simulator only inside its own `main`), so its state machine runs here
against an idealised body. That proves the BOOKKEEPING -- every verb ends
measured, superseded orders say so, corrections finish short motions, routes
pass their via points -- not the physics; the physics numbers quoted in the
module were measured in the engine (x30_plant_inspection.omniworld).

The bridge's place lookup and walkway routing are lifted out of the
controller with the AST trick the other bridge tests use, because the
controller imports `omnisim` at module scope.
"""

from __future__ import annotations

import ast
import json
import math
import pathlib
import re
import sys
from types import SimpleNamespace
from typing import Any, Dict

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
QUAD_DIR = ROOT / "projects/samples/demos/controllers/omnilink_quadruped_bridge"
QUAD = QUAD_DIR / "omnilink_quadruped_bridge.py"
WORLD = ROOT / "projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld"
if str(QUAD_DIR) not in sys.path:
    sys.path.insert(0, str(QUAD_DIR))

import _crawl_motion as cm  # noqa: E402


def _run(d: "cm.CrawlDriver", seq: int, body: Dict[str, float], efficiency: float = 1.0,
         limit_s: float = 200.0) -> Dict[str, Any]:
    """Integrate an idealised body from the driver's low-passed rates (a
    robot that delivers `efficiency` of what its stride asks) until motion
    `seq` reports."""
    t_end = body["t"] + limit_s
    while body["t"] < t_end:
        d.tick(body["t"], body["x"], body["y"], body["yaw"])
        k = efficiency * d.blend * d.dt
        c, s = math.cos(body["yaw"]), math.sin(body["yaw"])
        body["x"] += k * (d.vx * c - d.vy * s)
        body["y"] += k * (d.vx * s + d.vy * c)
        body["yaw"] = cm.wrap_pi(body["yaw"] + k * d.wz)
        body["t"] += d.dt
        done = d.completed
        if done is not None and done["seq"] == seq:
            return done
    raise AssertionError(f"motion {seq} never reported")


def _body() -> Dict[str, float]:
    return {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0}


@pytest.mark.parametrize("gait", ["crawl", "trot"])
def test_every_verb_ends_measured_and_within_tolerance(gait) -> None:
    d = cm.CrawlDriver("x30", dt=0.008, gait=gait)
    b = _body()
    r = _run(d, d.walk(2.0), b)
    assert r["verb"] == "walk" and r["settled"] is True
    assert abs(r["achieved"] - 2.0) <= d.WALK_TOL_M
    r = _run(d, d.turn(math.pi / 2), b)
    assert r["settled"] is True and abs(r["error"]) <= d.TURN_TOL_RAD
    if gait == "trot":
        # (The idealised body delivers ALL of its stride, so it coasts twice
        # as far as the real robot the crawl's stop leads are tuned for; the
        # crawl's walk-to accuracy is an engine measurement, not this one.)
        r = _run(d, d.walk_to(1.0, 3.0, 0.0), b)
        assert r["settled"] is True and r["error_m"] <= d.GOTO_TOL_M
    # the gait always ends in its standing pose
    assert d.blend == 0.0 and not d.busy()


def test_a_robot_that_delivers_less_is_finished_by_corrections() -> None:
    # The stance legs lag their targets: in the engine the X30 delivers about
    # half its stride. A short stop must be FINISHED toward the same target,
    # not reported as done.
    d = cm.CrawlDriver("x30", dt=0.008, gait="trot", lead_walk_s=1.5)
    r = _run(d, d.walk(3.0), _body(), efficiency=0.55)
    assert r["settled"] is True and abs(r["error"]) <= max(d.WALK_TOL_M, 0.05 * 3.0)
    assert 0 <= r["corrections"] <= d.MAX_CORRECTIONS


def test_a_superseded_order_says_so_and_measures_nothing() -> None:
    d = cm.CrawlDriver("x30", dt=0.008, gait="trot")
    b = _body()
    first = d.walk(5.0)
    for _ in range(300):
        d.tick(b["t"], b["x"], b["y"], b["yaw"])
        b["t"] += d.dt
    second = d.halt()
    assert d.completed["seq"] == first and d.completed["superseded"] is True
    r = _run(d, second, b)
    assert r["verb"] == "halt" and d.blend == 0.0


def test_a_route_passes_its_via_points_without_stopping() -> None:
    d = cm.CrawlDriver("x30", dt=0.008, gait="trot")
    b = _body()
    blends, where = [], []
    via = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
    seq = d.walk_to(0.0, 0.0, math.pi, via=via)
    while True:
        d.tick(b["t"], b["x"], b["y"], b["yaw"])
        k = d.blend * d.dt
        b["x"] += k * d.vx * math.cos(b["yaw"])
        b["y"] += k * d.vx * math.sin(b["yaw"])
        b["yaw"] = cm.wrap_pi(b["yaw"] + k * d.wz)
        b["t"] += d.dt
        blends.append(d.blend)
        where.append((b["x"], b["y"]))
        if d.completed is not None and d.completed["seq"] == seq:
            break
        assert b["t"] < 300
    r = d.completed
    assert r["settled"] is True and r["path_m"] > 9.0
    # it never came to a standstill at a corner (a stop at the destination,
    # and a correction pass there, are fine)
    stops = [where[k + 1] for k, (a, c) in enumerate(zip(blends, blends[1:]))
             if a > 0.0 and c == 0.0]
    assert stops
    for sx, sy in stops:
        assert all(math.hypot(sx - vx, sy - vy) > 0.6 for vx, vy in via), (sx, sy)


def test_the_lateral_stride_mirrors_the_forward_one() -> None:
    d = cm.CrawlDriver("x30", dt=0.008, gait="trot")
    d.blend, d.vy, d.gait_time = 1.0, 0.1, 5.0
    d.phase = cm._dr.QS_PHASE + 1.3
    feet, _ = d.gait.foot_targets(d.phase, d.gait_time, 0.0)
    moved = d._add_lateral_stride(feet)
    st = d.gait.stride(d.phase, d.gait_time, 0.0)
    for leg in cm.LEGS:
        dy = moved[leg][1] - feet[leg][1]
        if st[leg][3] is None:          # planted: the foot slides the other way
            Ly = d.vy * d.gait.duty / d.gait.freq
            assert dy == pytest.approx(Ly * (0.5 - st[leg][2] / d.gait.duty))
        assert moved[leg][0] == feet[leg][0] and moved[leg][2] == feet[leg][2]


def test_the_trot_starts_from_all_four_feet_planted() -> None:
    g = cm.CrawlDriver("x30", dt=0.008, gait="trot").gait
    _, swings = g.foot_targets(cm._dr.QS_PHASE, 0.0)
    assert all(v == 0.0 for v in swings.values())
    assert g.offsets == cm._dr.TROT_OFFSET


# ── the bridge's site: places and walkway routing ─────────────────────

def _load(name: str, ns: Dict[str, Any]):
    tree = ast.parse(QUAD.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    mod = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")],
                                          level=0), fn], type_ignores=[])
    ast.fix_missing_locations(mod)
    exec(compile(mod, str(QUAD), "exec"), ns)
    return ns[name]


def _site_from_world() -> SimpleNamespace:
    """The shipped world's customData, parsed by the shipped _read_places."""
    text = WORLD.read_text(encoding="utf-8")
    raw = re.search(r'customData "((?:[^"\\]|\\.)*)"', text).group(1).replace('\\"', '"')
    fake = SimpleNamespace(robot=SimpleNamespace(getCustomData=lambda: raw),
                           walkway_nodes={}, walkway_edges={}, site_bounds=None)
    from typing import Dict as _D, List as _L, Optional as _O, Tuple as _T
    ns = {"json": json, "math": math, "re": re, "Dict": _D, "List": _L, "Optional": _O, "Tuple": _T}
    fake.places = _load("_read_places", ns)(fake)
    fake._walkway_route = lambda x, y, g: _load("_walkway_route", ns)(fake, x, y, g)
    fake._find_place = lambda p: _load("_find_place", ns)(fake, p)
    return fake


def test_the_world_declares_seven_places_on_its_walkway() -> None:
    site = _site_from_world()
    assert len(site.places) == 7
    for name, p in site.places.items():
        assert p["node"] in site.walkway_nodes, name
        b = site.site_bounds
        assert b[0] <= p["x"] <= b[1] and b[2] <= p["y"] <= b[3], name


def test_go_to_follows_the_walkway_instead_of_cutting_across() -> None:
    site = _site_from_world()
    dock = site.places["charging dock"]
    pump = site.places["pump P-101"]
    via = site._walkway_route(dock["x"], dock["y"], pump["node"])
    # dock -> along the south walkway -> the pump's junction
    assert via[-1] == site.walkway_nodes["S_P101"]
    assert all(abs(y - (-5.0)) < 1e-9 for _, y in via)
    t2 = site.places["tank T-2"]
    via = site._walkway_route(pump["x"], pump["y"], t2["node"])
    # every leg of the route is axis-aligned: it stays on painted walkway
    pts = [(pump["x"], pump["y"])] + via
    for (x0, y0), (x1, y1) in zip(pts[1:], pts[2:]):
        assert abs(x1 - x0) < 1e-9 or abs(y1 - y0) < 1e-9


def test_place_names_match_loosely_but_never_guess() -> None:
    site = _site_from_world()
    assert site._find_place("pump p101") == "pump P-101"
    assert site._find_place("Tank T-2") == "tank T-2"
    assert site._find_place("tank") is None          # T-1 or T-2: ask, do not pick
    assert site._find_place("boiler house") is None
