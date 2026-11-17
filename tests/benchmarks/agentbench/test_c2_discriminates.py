#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""C2 must be able to FAIL -- the geometry that makes it a discriminator.

AgentBench C2 (``fall_through_floor``) presents a floor Solid authored with no
``boundingObject`` and asks the agent to make the crate stay up. Criterion C2.5
then checks the crate rests on the surface the file declares::

    |z_end - (floor_top + half_height)| <= REST_TOL_M

The engine carries an **implicit ground plane at z=0**. So a crate that never
touches the authored floor at all does not fall for ever -- it lands on that
plane. With the floor originally at ``translation 0 0 0``, top surface 0.05:

    forged rest (implicit plane) = 0.000 + 0.200 = 0.200
    expected rest (authored)     = 0.050 + 0.200 = 0.250
    error                        = 0.050  <=  REST_TOL_M (0.150)   -> PASS

The **unfixed** world passed 5/5 (measured 2026-08-08), so the task could not
discriminate a fix from no fix, and every C2 result ever recorded was uninformative.

The property that makes it discriminate is one inequality, independent of the
crate's size (the half-height cancels)::

    floor_top > REST_TOL_M

This test asserts exactly that, on both arms, reading the tolerance from the
grader so the two cannot drift apart. It is pure arithmetic on the authored
world text -- no simulator, no GPU -- so it runs in the ordinary unit lane and
guards against a future "tidy the world back to the origin" edit.
"""

import pathlib
import re
import sys

import pytest

_HERE = pathlib.Path(__file__).resolve().parent
_TASK = _HERE / "tasks" / "C2_fall_through_floor"

# Loaded by path: `tests` is not a package, and the grader package root is
# tests/benchmarks/ (meta.json names it "agentbench.graders.c2"), so neither a
# plain import nor a repo-root sys.path entry resolves it from here.
sys.path.insert(0, str(_HERE.parent))  # tests/benchmarks -> `agentbench` package
from agentbench.graders import c2_core  # noqa: E402

IMPLICIT_GROUND_PLANE_Z = 0.0

ARMS = [
    pytest.param(_TASK / "initial" / "fall_through.wbt", id="omnisim"),
    pytest.param(_TASK / "initial_webots" / "fall_through.wbt", id="webots"),
]


def _floor_geometry(world: pathlib.Path):
    """Return (centre_z, box_size_z) of the ``DEF FLOOR`` Solid, from the text.

    Deliberately a text read rather than an engine load: this test's whole job
    is to check what the FILE says, and it must run without a simulator.
    """
    text = world.read_text(encoding="utf-8")
    start = text.index("DEF FLOOR Solid {")
    block = text[start:]

    # `translation x y z` -- the first one inside the block is the Solid's own.
    m = re.search(r"^\s*translation\s+(\S+)\s+(\S+)\s+(\S+)\s*$",
                  block, re.M)
    assert m, "no translation on DEF FLOOR in %s" % world
    centre_z = float(m.group(3))

    # `size sx sy sz` of its Box geometry.
    m = re.search(r"^\s*size\s+(\S+)\s+(\S+)\s+(\S+)\s*$", block, re.M)
    assert m, "no Box size under DEF FLOOR in %s" % world
    return centre_z, float(m.group(3))


@pytest.mark.parametrize("world", ARMS)
def test_floor_top_is_above_the_rest_tolerance(world):
    """The authored surface must be further from z=0 than C2.5's tolerance.

    Otherwise a body resting on the implicit ground plane satisfies C2.5 and the
    task passes without the bug being fixed.
    """
    centre_z, size_z = _floor_geometry(world)
    floor_top = centre_z + size_z / 2.0
    tol = c2_core.REST_TOL_M

    assert floor_top > tol, (
        "C2 cannot discriminate: the floor's top surface is %.3f m above z=0, "
        "within C2.5's REST_TOL_M of %.3f m. A crate resting on the engine's "
        "implicit ground plane at z=0 -- i.e. one that never touched the "
        "authored floor at all -- would satisfy C2.5, so the UNFIXED world "
        "passes. Raise DEF FLOOR so its top is more than %.3f m above the "
        "origin. (%s)" % (floor_top, tol, tol, world.name))


@pytest.mark.parametrize("world", ARMS)
def test_resting_on_the_implicit_plane_fails_c2_5(world):
    """State the same thing the way the grader computes it, end to end.

    The half-height cancels out of the inequality above, but spelling the
    arithmetic out means a future change to C2.5's *formula* -- not just its
    tolerance -- also trips this test.
    """
    centre_z, size_z = _floor_geometry(world)
    floor_top = centre_z + size_z / 2.0

    text = world.read_text(encoding="utf-8")
    crate = text[text.index("DEF CRATE_BOT Robot {"):]
    m = re.search(r"^\s*size\s+(\S+)\s+(\S+)\s+(\S+)\s*$", crate, re.M)
    assert m, "no Box size under DEF CRATE_BOT in %s" % world
    half_h = float(m.group(3)) / 2.0

    expected_rest = floor_top + half_h
    forged_rest = IMPLICIT_GROUND_PLANE_Z + half_h
    error = abs(forged_rest - expected_rest)

    assert error > c2_core.REST_TOL_M, (
        "a crate resting on the implicit z=0 plane reads z=%.3f, the authored "
        "surface expects z=%.3f, error %.3f m -- inside REST_TOL_M (%.3f), so "
        "C2.5 would PASS a world nobody fixed. (%s)"
        % (forged_rest, expected_rest, error, c2_core.REST_TOL_M, world.name))


def test_the_two_arms_share_the_floor_geometry():
    """OmniSim and upstream Webots must present the SAME scene.

    The arms exist to compare simulators; a geometry difference between them
    would silently become a difference in the thing being measured.
    """
    omnisim = _floor_geometry(_TASK / "initial" / "fall_through.wbt")
    webots = _floor_geometry(_TASK / "initial_webots" / "fall_through.wbt")
    assert omnisim == webots, (
        "the C2 arms disagree on the floor: OmniSim (centre_z, size_z)=%s vs "
        "upstream Webots %s. The cross-simulator comparison is only valid if "
        "both arms author the same geometry." % (omnisim, webots))


def test_meta_json_constants_match_the_grader():
    """meta.json duplicates the grader's constants; drift would mislead.

    ``REST_TOL_M`` in particular is the number the geometry above is chosen
    against, so a copy of it that disagrees is worse than no copy.
    """
    import json
    meta = json.loads((_TASK / "meta.json").read_text(encoding="utf-8"))
    for name, declared in meta["constants"].items():
        actual = getattr(c2_core, name)
        assert actual == pytest.approx(declared), (
            "meta.json says %s=%s but the grader uses %s" % (name, declared, actual))


def test_the_floor_still_has_no_bounding_object():
    """The bug the task is ABOUT must still be present in the initial state.

    Raising the floor is a fix to the *grading*, not to the world. If a future
    edit also gave the floor a boundingObject, C2 would start fully solved --
    and would once again fail to discriminate, from the opposite direction.
    """
    for world in (_TASK / "initial" / "fall_through.wbt",
                  _TASK / "initial_webots" / "fall_through.wbt"):
        text = world.read_text(encoding="utf-8")
        floor = text[text.index("DEF FLOOR Solid {"):]
        # The floor block runs to the next top-level DEF, or to EOF.
        nxt = floor.find("\nDEF ", 1)
        floor = floor[:nxt] if nxt != -1 else floor
        assert "boundingObject" not in floor, (
            "%s: DEF FLOOR has a boundingObject -- the C2 initial state is "
            "supposed to be BROKEN (that missing field is the whole task). "
            "The fix belongs in the agent's answer, not in the fixture."
            % world.name)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
