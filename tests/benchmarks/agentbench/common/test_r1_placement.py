#!/usr/bin/env python3
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

"""R1's grade-time placement, as a mechanism -- no simulator, no agent.

What is being pinned here is the property the whole anti-hardcode argument
rests on: **the world that gets graded is not the world that was published**,
the grader is told which world that is, and anything less than that is a
failure rather than a quieter success.

The three things a reader should check first:

    test_the_graded_world_is_not_the_published_layout   the obstacles MOVED
    test_the_sidecar_is_what_the_grader_resolves        the grader is TOLD
    test_a_missing_obstacle_blocks_instead_of_...       failure is LOUD

Everything else is the machinery those three depend on: matching by geometry
rather than by name, a minimal rewrite, and a verification pass that re-reads
the placed artifact instead of trusting that the edit landed.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.common import mjcftext, r1_placement, worldtext  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402

AGENTBENCH = Path(__file__).resolve().parents[1]
WBT_FIXTURE = (AGENTBENCH / "adapters" / "omnisim" / "omnisim_lane" / "worlds"
               / "r1_null.wbt")
MJCF_FIXTURE = (AGENTBENCH / "adapters" / "mujoco" / "mujoco_lane"
                / "r1_oracle.xml")

SEED = "test/r1_placement/1"


def _published():
    return r1_core.obstacle_spec()


def _centres(bodies):
    return {b.name: (round(b.centre[0], 4), round(b.centre[1], 4))
            for b in bodies}


# --- the headline property --------------------------------------------------

@pytest.mark.parametrize("fixture,fmt,scanner", [
    (WBT_FIXTURE, "wbt", worldtext),
    (MJCF_FIXTURE, "mjcf", mjcftext),
])
def test_the_graded_world_is_not_the_published_layout(fixture, fmt, scanner):
    """⭐ The whole point, on both deliverable formats.

    Before this step existed, a campaign run graded the world exactly as the
    agent delivered it -- at the positions ``benchmark_assets/obstacles.json``
    publishes. A controller that read that file and cast no beam therefore
    passed 6/6 (measured: ``adapters/mujoco/mujoco_lane/r1_hardcode.py``).

    So the assertion is not "placement ran" but "the poses in the graded
    artifact are DIFFERENT from the published ones, and they are the drawn
    ones". Both halves matter: moving them somewhere arbitrary would be as
    useless as not moving them, because the grader would then score a layout
    nobody declared.
    """
    text = fixture.read_text(encoding="utf-8")
    layout = r1_core.sample_layout(SEED)
    placed, report = r1_placement.place_text(text, layout, fmt)

    before = _centres(scanner.scan_bodies(text))
    after = _centres(scanner.scan_bodies(placed))
    drawn = {o["name"]: (round(float(o["position"][0]), 4),
                         round(float(o["position"][1]), 4)) for o in layout}

    for pub in _published():
        name = pub["name"]
        assert after[name] != before[name], (
            "%s was not moved: the graded world still carries the PUBLISHED "
            "layout, which is the memorising agent's best case" % name)
        assert after[name] == drawn[name], (
            "%s ended at %s, not at the drawn %s" % (name, after[name],
                                                     drawn[name]))
    assert report["verification"]["matched"] == r1_core.N_OBSTACLES
    assert report["verification"]["max_centre_error_m"] <= \
        r1_placement.VERIFY_TOL_M


def test_the_placed_world_still_blocks_the_straight_line(tmp_path):
    """The sensing argument, re-derived from the PLACED text.

    R1.4-R1.6 mean nothing unless a blind straight run would hit something.
    The layout sampler makes that an acceptance criterion; this checks it
    survived the rewrite, which is a different claim (a placement that moved
    the wrong bodies could satisfy the first and not the second).
    """
    layout = r1_core.sample_layout(SEED)
    _placed, report = r1_placement.place_text(
        WBT_FIXTURE.read_text(encoding="utf-8"), layout, "wbt")
    v = report["verification"]
    assert v["straight_line_blocked_by"], v
    assert v["straight_line_blocked_m"] >= r1_core.PLACEMENT_MIN_BLOCK_M, v


# --- the grader is TOLD -----------------------------------------------------

def test_the_sidecar_is_what_the_grader_resolves(tmp_path):
    """⭐ The handshake, checked from the GRADER's side.

    ``place_and_declare`` writes ``r1_graded_layout.json``; the assertion is
    that ``r1_core.resolve_graded_layout`` -- the function ``graders/r1.py``
    actually calls -- resolves that file and returns the layout the artifact
    was placed with. A sidecar the grader does not read is a sidecar that
    silently leaves it scoring the published layout.
    """
    artifact = tmp_path / "lidar_nav.wbt"
    artifact.write_text(WBT_FIXTURE.read_text(encoding="utf-8"),
                        encoding="utf-8")
    run_dir = tmp_path / "run"
    report = r1_placement.place_and_declare(artifact, seed=SEED,
                                            declare_dirs=(run_dir,))

    resolved, source = r1_core.resolve_graded_layout(directories=(run_dir,))
    assert resolved is not None, "the grader found no declaration"
    assert r1_core.LAYOUT_SIDECAR in source and SEED in source
    assert resolved == r1_core.sample_layout(SEED)

    # ...and the world on disk IS that layout, matched by the grader's own
    # matcher at the grader's own tolerance.
    bodies = worldtext.scan_bodies(artifact.read_text(encoding="utf-8"))
    found, missing = r1_core.match_spec_obstacles(bodies, resolved)
    assert missing == [] and len(found) == r1_core.N_OBSTACLES
    assert report["sidecars"] == [str(run_dir / r1_core.LAYOUT_SIDECAR)]


def test_the_declaration_names_the_seed_and_the_mechanism(tmp_path):
    """A layout on disk with no provenance cannot be re-derived or argued
    with. The sidecar carries the task, the seed and the mechanism id."""
    artifact = tmp_path / "lidar_nav.xml"
    artifact.write_text(MJCF_FIXTURE.read_text(encoding="utf-8"),
                        encoding="utf-8")
    r1_placement.place_and_declare(artifact, seed=SEED, declare_dirs=(tmp_path,))
    doc = json.loads((tmp_path / r1_core.LAYOUT_SIDECAR)
                     .read_text(encoding="utf-8"))
    assert doc["task"] == r1_core.TASK
    assert doc["seed"] == SEED
    assert doc["mechanism"] == r1_core.PLACEMENT_MECHANISM
    assert [o["name"] for o in doc["obstacles"]] == \
        [o["name"] for o in _published()]


# --- failure is LOUD --------------------------------------------------------

def test_a_missing_obstacle_blocks_instead_of_scoring_the_published_layout():
    """⭐ No fallback, end to end.

    The superseded perturbation had one: a layout whose straight line came
    out unblocked was replaced by the PUBLISHED layout -- the memoriser's best
    case -- and the substitution was invisible in the row. Nothing here may
    behave that way. A deliverable the placer cannot place raises, and the
    error text says what it could not find, so the cell blocks with a reason
    rather than being graded on a world nobody moved.
    """
    text = WBT_FIXTURE.read_text(encoding="utf-8")
    # Delete one obstacle the way an agent that built four would: the whole
    # node, leaving the other four exactly as they were.
    start = text.index("DEF OBSTACLE_5 Solid {")
    end = text.index("\n}", start) + 2
    four = text[:start] + text[end:]

    layout = r1_core.sample_layout(SEED)
    with pytest.raises(r1_placement.PlacementError) as exc:
        r1_placement.place_text(four, layout, "wbt")
    assert "OBSTACLE_5" in str(exc.value)
    assert exc.value.report["missing"] == ["OBSTACLE_5"]
    # ...and the four that WERE found are named, so a reader can tell "the
    # agent built four boxes" from "our scanner is blind".
    assert len(exc.value.report["matched"]) == 4


def test_a_rewrite_that_does_not_land_is_caught_by_verification(monkeypatch):
    """The placer checks its OWN output, and fails when the edit did nothing.

    Simulated by making the rewrite a no-op -- the shape of any bug that
    edits the wrong node, or a node whose parent puts it back. Without this
    pass such a bug would ship a world at the PUBLISHED poses together with a
    sidecar claiming the drawn ones, and every obstacle would read as
    "missing" in the verdict: an honest agent failed by our arithmetic.
    """
    monkeypatch.setattr(worldtext, "move_bodies",
                        lambda text, moves: (text, []))
    layout = r1_core.sample_layout(SEED)
    with pytest.raises(r1_placement.PlacementError) as exc:
        r1_placement.place_text(WBT_FIXTURE.read_text(encoding="utf-8"),
                                layout, "wbt")
    assert "did not land" in str(exc.value)


def test_an_unknown_deliverable_format_is_refused(tmp_path):
    """R1 is graded on a ``.wbt`` or an MJCF ``.xml``. Anything else is a
    refusal with a reason, never a pass-through."""
    art = tmp_path / "world.sdf"
    art.write_text("<sdf/>", encoding="utf-8")
    with pytest.raises(r1_placement.PlacementError) as exc:
        r1_placement.place_and_declare(art, seed=SEED)
    assert ".sdf" in str(exc.value)


def test_an_unplaceable_layout_seed_raises_rather_than_substituting(
        monkeypatch):
    """``sample_layout`` exhausting its budget must reach the caller as a
    placement failure, not as a quiet published-layout run."""
    def _boom(seed):
        raise r1_core.LayoutError("no legal layout for %r" % seed)

    monkeypatch.setattr(r1_core, "sample_layout_with_report",
                        lambda seed, **kw: _boom(seed))
    with pytest.raises(r1_placement.PlacementError) as exc:
        r1_placement.draw("whatever")
    assert "no legal obstacle layout" in str(exc.value)


# --- found by GEOMETRY, never by name ---------------------------------------

def test_obstacles_are_found_when_the_agent_named_them_its_own_way():
    """Measured need, twice over: one real R1 run called its boxes ``crate
    A``..``crate E`` and another ``obstacle_1``..``_6``. A placer keyed on the
    published names would refuse both -- and refuse them as "the agent did not
    build the obstacles", which is grading our convention."""
    text = WBT_FIXTURE.read_text(encoding="utf-8")
    for i, alias in enumerate(["crate A", "crate B", "crate C", "crate D",
                               "crate E"], start=1):
        text = text.replace("DEF OBSTACLE_%d Solid" % i,
                            "DEF CRATE_%d Solid" % i)
        text = text.replace('name "OBSTACLE_%d"' % i, 'name "%s"' % alias)
    assert "OBSTACLE_" not in text.split("# --- the rover")[0].split(
        "the five frozen obstacles")[-1]

    layout = r1_core.sample_layout(SEED)
    placed, report = r1_placement.place_text(text, layout, "wbt")
    assert report["missing"] == []
    assert all(o["channel"] == "position+footprint"
               for o in report["obstacles"])
    after = _centres(worldtext.scan_bodies(placed))
    for o in layout:
        # the body kept the agent's DEF name; it is the GEOMETRY that was
        # matched, and the drawn pose is what it now carries
        assert (round(float(o["position"][0]), 4),
                round(float(o["position"][1]), 4)) in after.values()


def test_an_agent_that_put_its_boxes_elsewhere_is_still_placed():
    """The footprint-only channel.

    An agent may build the five specified boxes and lay them out somewhere of
    its own -- wrongly, by the prompt, but it has still built them. Placement
    is about to overwrite their positions anyway, so refusing to recognise
    them would block the cell over a difference the step erases. R1.3 still
    judges the delivered world, on the drawn layout, afterwards.
    """
    text = WBT_FIXTURE.read_text(encoding="utf-8")
    text = text.replace("DEF OBSTACLE_2 Solid {\n  translation -2 -1.4 0.25",
                        "DEF OBSTACLE_2 Solid {\n  translation -3.7 3.1 0.25")
    layout = r1_core.sample_layout(SEED)
    placed, report = r1_placement.place_text(text, layout, "wbt")
    channels = {o["name"]: o["channel"] for o in report["obstacles"]}
    assert channels["OBSTACLE_2"] == "footprint"
    assert set(channels.values()) == {"footprint", "position+footprint"}
    after = _centres(worldtext.scan_bodies(placed))
    drawn = {o["name"]: (round(float(o["position"][0]), 4),
                         round(float(o["position"][1]), 4)) for o in layout}
    assert after["OBSTACLE_2"] == drawn["OBSTACLE_2"]


def test_the_robot_is_never_treated_as_an_obstacle():
    """A ``Robot`` subtree is skipped whole on the ``.wbt`` side and an
    articulated ``<body>`` on the MJCF side, so no chassis or wheel can be
    matched, moved, or counted as scenery."""
    wbt = worldtext.scan_bodies(WBT_FIXTURE.read_text(encoding="utf-8"))
    assert not any("wheel" in (b.name or "").lower() or
                   (b.name or "").upper() == "ROVER" for b in wbt), \
        [b.name for b in wbt]
    mj = mjcftext.scan_bodies(MJCF_FIXTURE.read_text(encoding="utf-8"))
    assert not any(b.name in ("chassis", "caster", "tyre_left", "tyre_right")
                   for b in mj), [b.name for b in mj]


# --- the rewrite is MINIMAL -------------------------------------------------

def test_only_the_moved_bodies_own_pose_field_changes():
    """The deliverable that gets graded must still be the one the agent
    wrote. Exactly five lines differ, each of them a ``translation``."""
    text = WBT_FIXTURE.read_text(encoding="utf-8")
    placed, _r = r1_placement.place_text(text, r1_core.sample_layout(SEED),
                                         "wbt")
    a, b = text.splitlines(), placed.splitlines()
    assert len(a) == len(b), "the rewrite changed the line structure"
    differing = [(i, x, y) for i, (x, y) in enumerate(zip(a, b)) if x != y]
    assert len(differing) == r1_core.N_OBSTACLES, differing
    assert all("translation" in x for _i, x, _y in differing), differing


def test_the_mjcf_rewrite_touches_only_pos_attributes():
    text = MJCF_FIXTURE.read_text(encoding="utf-8")
    placed, _r = r1_placement.place_text(text, r1_core.sample_layout(SEED),
                                         "mjcf")
    a, b = text.splitlines(), placed.splitlines()
    assert len(a) == len(b)
    differing = [(x, y) for x, y in zip(a, b) if x != y]
    assert len(differing) == r1_core.N_OBSTACLES, differing
    assert all('pos="' in x and "OBSTACLE" in x for x, _y in differing), \
        differing


def test_a_body_with_no_translation_field_gets_one():
    """A Solid that relied on the ``0 0 0`` default is still movable."""
    world = """#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 16 }
DEF BOX Solid {
  children [
    Shape { geometry Box { size 1.6 1.6 0.5 } }
  ]
  name "the only box"
}
"""
    spec = [{"name": "OBSTACLE_1", "position": [0.0, 0.0, 0.25],
             "size": [1.6, 1.6, 0.5]}]
    layout = [{"name": "OBSTACLE_1", "position": [2.5, -1.25, 0.25],
               "size": [1.6, 1.6, 0.5]}]
    bodies = worldtext.scan_bodies(world)
    moved, applied = worldtext.move_bodies(world, [(bodies[0], (2.5, -1.25,
                                                                0.0))])
    assert applied[0]["field"] == "inserted"
    assert "translation 2.5 -1.25 0" in moved
    again = worldtext.scan_bodies(moved)
    found, missing = r1_core.match_spec_obstacles(again, layout)
    assert missing == [] and len(found) == 1
    assert r1_core.match_spec_obstacles(again, spec)[1] == ["OBSTACLE_1"]


def test_a_body_under_a_rotated_parent_lands_where_it_was_asked():
    """The delta is written in the body's PARENT frame.

    A group rotated 90 degrees about z means "move +1 in world x" is "move +1
    in local y" for anything under it. Writing the world delta straight into
    the child's field would put the obstacle somewhere nobody asked for -- and
    the verification pass would then (correctly) fail the cell, so getting
    this right is what keeps an honest agent's nested world gradeable.
    """
    world = """#OMNISIM R2025a utf8
DEF GROUP Pose {
  translation 1 0 0
  rotation 0 0 1 1.5707963267948966
  children [
    DEF BOX Solid {
      translation 0.5 0 0.25
      children [ Shape { geometry Box { size 1 1 0.5 } } ]
    }
  ]
}
"""
    bodies = {b.name: b for b in worldtext.scan_bodies(world)}
    box = bodies["BOX"]
    assert (round(box.centre[0], 6), round(box.centre[1], 6)) == (1.0, 0.5)
    moved, _applied = worldtext.move_bodies(world, [(box, (2.0, -1.5, 0.0))])
    after = {b.name: b for b in worldtext.scan_bodies(moved)}["BOX"]
    assert (round(after.centre[0], 6), round(after.centre[1], 6)) == (3.0, -1.0)


# --- reproducibility --------------------------------------------------------

def test_the_same_seed_gives_the_same_placement_and_another_does_not():
    """The seed is recorded on the row precisely so a layout can be
    re-derived from it later; that is only true if the draw is pure."""
    text = WBT_FIXTURE.read_text(encoding="utf-8")
    a, _ra = r1_placement.place_text(text, r1_core.sample_layout(SEED), "wbt")
    b, _rb = r1_placement.place_text(text, r1_core.sample_layout(SEED), "wbt")
    c, _rc = r1_placement.place_text(text,
                                     r1_core.sample_layout(SEED + "/other"),
                                     "wbt")
    assert a == b
    assert a != c


def test_the_default_seed_is_per_task_and_repeat_and_arm_free():
    """Two arms' r0 must draw the SAME layout -- otherwise a sim-vs-sim
    comparison is also a layout-vs-layout one -- while r0 and r1 differ."""
    s0 = r1_placement.default_seed(r1_core.TASK, 0, campaign_id="c1")
    s1 = r1_placement.default_seed(r1_core.TASK, 1, campaign_id="c1")
    assert s0 == r1_placement.default_seed(r1_core.TASK, 0, campaign_id="c1")
    assert s0 != s1
    assert "omnisim" not in s0 and "mujoco" not in s0
    assert r1_core.sample_layout(s0) != r1_core.sample_layout(s1)


def test_the_drawn_layout_keeps_the_published_count_and_sizes():
    """The contract the task publishes -- five boxes, these extents -- is not
    what is redrawn. Only the positions are."""
    layout = r1_core.sample_layout(SEED)
    assert r1_core.obstacle_sizes(layout) == r1_core.obstacle_sizes()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
