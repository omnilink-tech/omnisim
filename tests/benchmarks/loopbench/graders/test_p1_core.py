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

"""P1's core, and every way a wrong answer could look like a right one.

Red evidence: each clause is exercised on a route built to FAIL it. The
interesting cases are the near-misses -- a circle instead of a square, two laps
then a stop, a body that jitters at the start line for ever -- because those
are what a real agent actually produces.
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from loopbench.graders import p1_core                      # noqa: E402

C = {"SIDE_M": 5.0, "SIDE_TOL_M": 1.5, "MIN_LAPS": 2,
     "LAP_EXIT_M": 2.0, "LAP_RETURN_M": 1.0,
     "MAX_STEP_JUMP_M": 0.5, "MAX_SAMPLE_DT_S": 0.05, "Z_BAND_M": 0.5}


def square_route(laps=3, side=5.0, dt=0.02, speed=1.0, z0=0.1, stop_after=None):
    """A body walking the perimeter of a square, corner to corner."""
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
    t, xy, z = [], [], []
    now = 0.0
    for lap in range(laps):
        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(2, int(seg / (speed * dt)))
            for k in range(n):
                f = k / float(n)
                t.append(now)
                xy.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
                z.append(z0)
                now += dt
        if stop_after is not None and lap + 1 >= stop_after:
            break
    if stop_after is not None:
        # park at the last point for as long again, so the run "ends" idle
        for _ in range(len(t)):
            t.append(now)
            xy.append(xy[-1])
            z.append(z0)
            now += dt
    return t, xy, z


def test_a_real_patrol_passes():
    t, xy, z = square_route(laps=3)
    out, cl, n = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert out == "PASS", {k: v.detail for k, v in cl.items()}
    assert n["laps"] >= 2


def test_one_lap_is_not_a_patrol():
    t, xy, z = square_route(laps=1)
    out, cl, _ = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert out == "FAIL" and p1_core.first_failure(cl) == "P1.1"


def test_two_laps_then_parking_fails_the_still_going_clause():
    """The prompt says KEEP going. Stopping after two laps is a different
    behaviour from patrolling, and a lap count alone cannot tell them apart."""
    t, xy, z = square_route(laps=4, stop_after=2)
    out, cl, _ = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert cl["P1.1"].ok            # it did do two laps
    assert not cl["P1.3"].ok        # ...and then gave up
    assert out == "FAIL"


def test_a_circle_of_the_right_size_is_not_a_square():
    """A near-miss an agent really produces: it loops, it is 5 m across, and
    the extent test alone would wave it through -- so the tolerance has to be
    tight enough to matter, and this pins that it is."""
    t, xy, z = [], [], []
    r, now, dt = 2.5, 0.0, 0.02
    for k in range(3 * 400):
        a = 2 * math.pi * (k % 400) / 400.0
        t.append(now)
        xy.append((2.5 + r * math.cos(a), 2.5 + r * math.sin(a)))
        z.append(0.1)
        now += dt
    out, cl, n = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    # A circle of diameter 5 sweeps exactly 5x5, so the EXTENT clause passes.
    # Recorded rather than asserted-against: P1.2 is a size check, not a
    # shape check, and pretending otherwise would be a false claim about what
    # this grader measures. The tier's own note says so.
    assert cl["P1.2"].ok
    assert n["extent_m"][0] > 4.0


def test_jittering_at_the_start_line_scores_no_laps():
    t, xy, z = [], [], []
    now = 0.0
    for k in range(2000):
        t.append(now)
        xy.append((0.1 * math.sin(k), 0.1 * math.cos(k)))
        z.append(0.1)
        now += 0.02
    out, cl, n = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert n["laps"] == 0 and not cl["P1.1"].ok


def test_a_square_of_the_wrong_size_fails_P1_2():
    t, xy, z = square_route(laps=3, side=12.0)
    out, cl, _ = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert not cl["P1.2"].ok and "12" in cl["P1.2"].detail


def test_a_teleporting_body_fails_P1_4():
    t, xy, z = square_route(laps=3)
    xy[500] = (xy[500][0] + 3.0, xy[500][1])
    out, cl, _ = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert not cl["P1.4"].ok and "teleport" in cl["P1.4"].detail


def test_a_body_that_flies_fails_P1_5():
    t, xy, z = square_route(laps=3)
    z = [0.1 + 0.002 * i for i in range(len(z))]
    out, cl, _ = p1_core.grade(t=t, xy=xy, z=z, constants=C)
    assert not cl["P1.5"].ok


def test_an_empty_run_is_ERROR_and_every_clause_is_vacuous():
    out, cl, _ = p1_core.grade(t=[], xy=[], z=[], constants=C)
    assert out == "ERROR" and all(c.vacuous for c in cl.values())


def test_a_missing_height_series_is_vacuous_not_a_pass():
    t, xy, _ = square_route(laps=3)
    out, cl, _ = p1_core.grade(t=t, xy=xy, z=[], constants=C)
    assert cl["P1.5"].vacuous and not cl["P1.5"].ok


SIM_TOKENS = ("omnisim", "webots", "mujoco", "isaac", "gazebo", "wbt", "urdf",
              "mjcf", "vrml", "newton", "ode", "harness")


def test_the_core_names_no_simulator_in_code():
    path = Path(p1_core.__file__)
    tree = ast.parse(path.read_text(encoding="utf-8"))
    docs = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)):
            body = getattr(node, "body", None) or []
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docs.add(id(body[0].value))
    for node in ast.walk(tree):
        text = None
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docs:
                text = node.value
        elif isinstance(node, ast.Name):
            text = node.id
        elif isinstance(node, ast.Attribute):
            text = node.attr
        if not text:
            continue
        for token in SIM_TOKENS:
            assert token not in text.lower(), (
                "p1_core mentions %r in code: %r" % (token, text))
