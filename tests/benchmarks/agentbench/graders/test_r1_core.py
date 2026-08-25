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

"""R1 lidar_nav -- unit tests for the sim-neutral core.

These never launch a simulator. They build evidence structures directly, which
is the point: if an assertion needs a simulator to be exercised, it is not
sim-neutral. The end-to-end half -- that grade-time placement actually catches
a memorising driver and actually spares a sensing one -- is measured on real
runs in ``adapters/mujoco/test_r1_discriminates_mujoco.py``.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders import r1_core  # noqa: E402

TASK_DIR = (Path(__file__).resolve().parents[1] / "tasks" / "R1_lidar_nav")
META = json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))


# --- helpers ------------------------------------------------------------------

def test_path_length_and_max_step_are_metres():
    xy = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
    assert r1_core.path_length_xy(xy) == pytest.approx(7.0)
    assert r1_core.max_step_xy(xy) == pytest.approx(4.0)


class _Body:
    """Minimal stand-in for graders.evidence.Body."""

    def __init__(self, name, lo=None, hi=None, robot=False, joints=0,
                 member_of=None):
        self.name = name
        self.body_id = name
        self.robot_class = robot
        self.n_joints = joints
        self.member_of = member_of
        self._lo, self._hi = lo, hi

    @property
    def has_aabb(self):
        return self._lo is not None

    @property
    def aabb(self):
        return (self._lo, self._hi)

    @property
    def top_z(self):
        return None if self._hi is None else float(self._hi[2])


def _obstacle(name, cx, cy, sx, sy):
    return _Body(name, (cx - sx / 2, cy - sy / 2, 0.0),
                 (cx + sx / 2, cy + sy / 2, 0.5))


def _bodies_for(layout):
    return [_obstacle(o["name"], o["position"][0], o["position"][1],
                      o["size"][0], o["size"][1]) for o in layout]


# --- the sensing argument -----------------------------------------------------

def test_the_shipped_obstacles_really_do_block_the_straight_line():
    """The published layout is what the agent authors against, so it must
    still pose the task it describes: a direct path that is physically
    blocked. (Every GRADED layout is required to as well -- that is an
    acceptance criterion of the sampler, asserted below.)"""
    bodies = _bodies_for(r1_core.obstacle_spec())
    assert r1_core.segment_blocked_by(bodies), (
        "the straight start->goal path must be BLOCKED by at least one "
        "shipped obstacle, or reaching the goal is no evidence of perception")


def test_both_arms_ship_identical_obstacle_geometry():
    """A cross-simulator comparison is only like-for-like if the scene is."""
    a = json.loads((TASK_DIR / "initial" / "benchmark_assets"
                    / "obstacles.json").read_text(encoding="utf-8"))
    b = json.loads((TASK_DIR / "initial_webots" / "benchmark_assets"
                    / "obstacles.json").read_text(encoding="utf-8"))
    assert a == b


def test_blocked_length_is_more_than_a_clipped_corner():
    """A boolean "blocked" is satisfied by a centimetre of corner, which
    proves nothing about what a blind robot would hit. The sampler's criterion
    is a LENGTH, and the published layout clears it too."""
    got = r1_core.segment_blocked_length(_bodies_for(r1_core.obstacle_spec()))
    assert got >= r1_core.PLACEMENT_MIN_BLOCK_M


# --- the contract: nothing is written down twice -----------------------------

def test_every_threshold_comes_from_meta_json():
    """The grader READS meta.json's constants; this proves it, by checking the
    module's value against the file for every declared name."""
    c = META["constants"]
    assert c, "the constants block must not be empty"
    for name, declared in c.items():
        got = getattr(r1_core, name)
        if isinstance(declared, list):
            assert list(got) == pytest.approx(declared), name
        else:
            assert got == pytest.approx(declared), (
                "meta.json says %s=%s but the grader uses %s"
                % (name, declared, got))
    from agentbench import tasks as _t
    assert META["timeout_s"] == min(3 * META["par_s"],
                                    _t.TASK_HARD_CEILING_S)


def test_the_straight_line_constant_is_the_actual_straight_line():
    assert r1_core.STRAIGHT_LINE_M == pytest.approx(
        math.hypot(r1_core.GOAL_XY[0] - r1_core.START_XY[0],
                   r1_core.GOAL_XY[1] - r1_core.START_XY[1]), abs=1e-3)


def test_meta_states_the_hole_the_measured_rates_and_which_mechanism_wins():
    """A task with a known hole says so, in the numbers it was measured in.

    Pinned as PROPERTIES rather than phrases: the wording has been rewritten
    twice already (2026-08-09, when 510 graded runs showed the declared fix
    could not be tuned into working), and a test pinned to a slogan fails on
    an honest rewrite while a test pinned to the claim keeps guarding it.
    """
    ah = META["anti_hardcode"]
    assert "the_hole" in ah and "6/6" in ah["the_hole"], (
        "the measured hole -- a memoriser passes 6/6 on the published layout "
        "-- must be stated, not softened")
    assert "grade_time" in ah["authoritative_mechanism"].lower() \
        or "grade-time" in ah["authoritative_mechanism"].lower()
    assert ah["legality_constraints"], "name the constraints, one by one"
    assert "superseded" in ah["superseded_mechanism"].lower(), (
        "two mechanisms exist in the module; the contract must say which one "
        "is authoritative")
    assert "measured_catch_rate" in ah and "%" in ah["measured_catch_rate"]
    # The mechanism has to be INVOCABLE, and the contract has to say by what.
    # It was a table for one commit -- sample_layout drew layouts nobody
    # placed -- and "declared but unwired" is the state a reader is most
    # likely to be misled about, in either direction.
    assert "injection_step" in ah, (
        "the step that PLACES the drawn layout must be named; a mechanism "
        "nobody can invoke is a table, not a defence")
    assert "r1_placement" in ah["injection_step"], (
        "name the module that does it, so a reader can go and read it")
    assert "measured_catch_rate_wired" in ah and "%" in \
        ah["measured_catch_rate_wired"], (
        "the rates measured THROUGH the wired path are a different "
        "measurement from the ones taken by placing the layout by hand, and "
        "the contract must carry both rather than pool them")
    assert "what_is_still_unbuilt" in ah, (
        "whatever is left is what stops the numbers being published; it must "
        "be named rather than left to a reader to infer")
    status = META["status"].lower()
    assert "may not be published" in status, (
        "until the placement is wired into a graded run, a memorising agent "
        "still passes and the task must say its numbers are not publishable")


# --- the discriminating behaviour --------------------------------------------

def test_a_teleporting_run_is_caught_by_the_step_bound():
    jump = [(-4.0, -4.0), (4.0, 4.0)]
    assert r1_core.max_step_xy(jump) > r1_core.MAX_STEP_DISPLACEMENT_M


def test_a_straight_blind_run_passes_THROUGH_the_boxes_and_is_caught():
    """R1.6 asserts "it drove AROUND" directly, from measured geometry.

    This is the property the retired 11.5 m path floor was standing in for --
    and it is strictly stronger, because it does not have to infer "it must
    have gone around" from a length.
    """
    spec = r1_core.obstacle_spec()
    straight = [(-4.0 + 8.0 * i / 400.0, -4.0 + 8.0 * i / 400.0)
                for i in range(401)]
    through = r1_core.samples_inside(straight, _bodies_for(spec))
    assert through, "a straight run crosses the blocking obstacles"


def test_a_grazing_run_is_not_accused_of_driving_through():
    """The margin that keeps the "through" clause off honest agents: a robot
    that skims a box keeps its own origin outside it."""
    b = _obstacle("OBSTACLE_X", 0.0, 0.0, 1.0, 1.0)
    graze = [(x / 100.0, 0.501) for x in range(-100, 101)]
    assert r1_core.samples_inside(graze, [b]) == []


# --- R1.6's floor: derived from the layout being scored (defect B) ------------

def test_the_path_floor_can_never_punish_a_collision_free_arrival():
    """Defect B, as a property rather than an anecdote.

    The retired ``MIN_PATH_LENGTH_M = 11.5`` was arithmetic about ONE layout,
    and five honest oracle runs that arrived within 0.10 m of the goal on
    11.31-11.49 m paths failed R1.6 for taking a legitimately shorter legal
    route. The floor must therefore sit under the shortest route the layout
    admits, less the tolerance a run may stop short by -- on EVERY layout the
    sampler can produce, not just on the published one.
    """
    layouts = [r1_core.obstacle_spec()] + [r1_core.sample_layout(s)
                                           for s in range(1, 8)]
    for layout in layouts:
        shortest = r1_core.shortest_route_m(layout, inflate_m=0.0)
        floor = r1_core.min_path_length(layout)
        assert floor <= shortest - r1_core.GOAL_TOL_M, (
            "floor %.3f is above the shortest point-robot route %.3f, so a "
            "legal route could fail it" % (floor, shortest))
        assert floor > 0.0, "a floor of zero asserts nothing"


def test_the_path_floor_moves_with_the_layout():
    """It is DERIVED, not a constant wearing a function's clothes."""
    tight = [{"name": "OBSTACLE_1", "position": [0.0, 0.0, 0.25],
              "size": [6.0, 0.6, 0.5]}]
    assert r1_core.min_path_length(tight) > \
        r1_core.min_path_length(r1_core.obstacle_spec())


def test_the_retired_constant_is_gone():
    """``MIN_PATH_LENGTH_M`` was the defect. Leaving the name behind invites
    the next reader to use it."""
    assert not hasattr(r1_core, "MIN_PATH_LENGTH_M")
    assert "MIN_PATH_LENGTH_M" not in META["constants"]


# --- GRADE-TIME PLACEMENT: the authoritative anti-hardcode mechanism ----------

def test_placement_is_pure_and_reproducible_from_the_seed():
    a = r1_core.sample_layout(7)
    b = r1_core.sample_layout(7)
    assert a == b, "the graded layout must be reproducible from the row"
    assert r1_core.sample_layout(8) != a, "a different seed, a different layout"


def test_placement_publishes_the_count_and_the_sizes_and_draws_the_rest():
    """The contract: the agent is told how many boxes and how big; where they
    go is drawn at grade time. If a size moved, the agent would be graded on a
    scene it could not have authored."""
    published = r1_core.obstacle_sizes()
    for seed in (1, 2, 3, 11, 99):
        layout = r1_core.sample_layout(seed)
        assert len(layout) == r1_core.N_OBSTACLES
        assert r1_core.obstacle_sizes(layout) == published


def test_every_sampled_layout_is_legal_by_construction():
    """Each acceptance criterion, re-checked independently of the sampler that
    claims to have enforced it."""
    for seed in range(1, 26):
        layout = r1_core.sample_layout(seed)
        legal = r1_core.layout_legality(layout)
        assert legal["legal"], (seed, legal)
        assert not legal["outside_arena"]
        assert not legal["too_close"]
        assert not legal["near_start"] and not legal["near_goal"]
        assert legal["route_exists"]
        assert legal["blocked_length_m"] >= r1_core.PLACEMENT_MIN_BLOCK_M
        assert legal["blocking_straight_line"], (
            "the sensing proof: the direct path must still be blocked")


def test_the_legality_rules_are_the_ones_meta_json_publishes():
    """Each numeric constraint, asserted at its declared value on real
    layouts -- so a rule that quietly stopped being enforced fails here."""
    c = META["constants"]
    for seed in range(1, 16):
        layout = r1_core.sample_layout(seed)
        for o in layout:
            x0, y0, x1, y1 = r1_core._aabb_xy(o)
            assert min(x0, y0) >= -c["PLACEMENT_ARENA_INNER_M"] - 1e-9
            assert max(x1, y1) <= c["PLACEMENT_ARENA_INNER_M"] + 1e-9
            assert r1_core._box_point_gap(o, r1_core.START_XY) >= \
                c["PLACEMENT_ENDPOINT_CLEAR_M"] - 1e-9
            assert r1_core._box_point_gap(o, r1_core.GOAL_XY) >= \
                c["PLACEMENT_ENDPOINT_CLEAR_M"] - 1e-9
        for i, a in enumerate(layout):
            for b in layout[i + 1:]:
                assert r1_core._box_box_gap(a, b) >= \
                    c["PLACEMENT_MIN_GAP_M"] - 1e-9
        assert r1_core.route_exists(
            layout, inflate_m=c["PLACEMENT_ROUTE_INFLATE_M"])


def test_an_illegal_layout_is_rejected_rather_than_repaired():
    """Each rule, shown to actually reject something."""
    def one(pos, size=(1.0, 1.0, 0.5), name="OBSTACLE_1"):
        return {"name": name, "position": [pos[0], pos[1], 0.25],
                "size": list(size)}

    assert not r1_core.layout_legality(
        [one((4.9, 0.0))])["legal"]                       # through the wall
    assert r1_core.layout_legality([one((4.9, 0.0))])["outside_arena"]
    assert r1_core.layout_legality(
        [one((-4.0, -3.4))])["near_start"]                # sat on the start
    assert r1_core.layout_legality([one((4.0, 3.4))])["near_goal"]
    pair = [one((0.0, 0.0), name="A"), one((1.05, 0.0), name="B")]
    assert r1_core.layout_legality(pair)["too_close"]
    # a wall right across the arena: solvable? no.
    assert not r1_core.layout_legality(
        [one((0.0, 0.0), size=(0.4, 9.6, 0.5))])["route_exists"]
    # ...and a layout that leaves the direct path clear is refused, which is
    # the acceptance criterion the superseded perturbation could only bail out
    # of.
    off = r1_core.layout_legality([one((-4.0, 4.0))])
    assert not off["blocking_straight_line"] and not off["legal"]


def test_placement_never_falls_back_to_a_layout_the_agent_has_seen():
    """The perturbation's fallback returned the PUBLISHED layout -- the
    memoriser's best case -- and its rate climbed with the displacement. This
    mechanism raises instead."""
    with pytest.raises(r1_core.LayoutError):
        # one impossible size: a box wider than the arena can never be placed
        r1_core.sample_layout(1, sizes=[("OBSTACLE_1", (99.0, 99.0, 0.5))],
                              max_attempts=5)


def test_the_graded_layout_is_not_the_published_one():
    """If the drawn layout could coincide with the published one, the whole
    mechanism would be a no-op on that seed."""
    published = {o["name"]: tuple(o["position"][:2])
                 for o in r1_core.obstacle_spec()}
    for seed in range(1, 21):
        drawn = {o["name"]: tuple(o["position"][:2])
                 for o in r1_core.sample_layout(seed)}
        moved = [n for n in published
                 if math.dist(published[n], drawn[n]) > 0.5]
        assert len(moved) >= 3, (
            "seed %d moved only %d of %d obstacles more than 0.5 m; a fixed "
            "path would survive it" % (seed, len(moved), len(published)))


# --- how grade() learns which layout it is scoring ---------------------------

def test_resolve_prefers_the_argument_then_the_sidecar_then_the_seed(tmp_path):
    explicit = r1_core.sample_layout(3)
    got, source = r1_core.resolve_graded_layout(layout=explicit, seed=99)
    assert got == explicit and source == "argument"

    r1_core.write_graded_layout(tmp_path, 5)
    got, source = r1_core.resolve_graded_layout(seed=99,
                                                directories=(tmp_path,))
    assert got == r1_core.sample_layout(5), "the sidecar wins over the seed"
    assert r1_core.LAYOUT_SIDECAR in source and "5" in source

    got, source = r1_core.resolve_graded_layout(seed=5,
                                                directories=(tmp_path / "no",))
    assert got == r1_core.sample_layout(5) and source == "seed 5"


def test_no_placement_resolves_to_nothing_rather_than_to_the_published_layout():
    """"I do not know which layout this was" must not be spelled the same way
    as "it was the published one", or the row silently claims evidence it does
    not have."""
    assert r1_core.resolve_graded_layout() == (None, None)


def test_the_sidecar_round_trips(tmp_path):
    path = r1_core.write_graded_layout(tmp_path, 12)
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["task"] == r1_core.TASK and data["seed"] == 12
    assert data["mechanism"] == r1_core.PLACEMENT_MECHANISM
    assert data["obstacles"] == r1_core.sample_layout(12)


# --- what counts as a collision (defect A) -----------------------------------

def test_a_moved_obstacle_is_still_matched_when_the_GRADED_layout_is_scored():
    """Defect A at its root: the matcher was handed the PUBLISHED layout, so
    on a placed world nothing matched and R1.5's hit set was empty."""
    layout = r1_core.sample_layout(4)
    bodies = _bodies_for(layout)
    found, missing = r1_core.match_spec_obstacles(bodies, layout)
    assert len(found) == r1_core.N_OBSTACLES and not missing
    # ...and against the published layout, the same world matches nothing.
    found_pub, missing_pub = r1_core.match_spec_obstacles(
        bodies, r1_core.obstacle_spec())
    assert not found_pub and len(missing_pub) == r1_core.N_OBSTACLES


def test_a_contact_is_countable_even_when_nothing_matched():
    """The second half of the defect A fix: the hit set is not a name list.

    A body the robot could not have driven over counts, whatever it is called
    and whether or not it matched a spec entry -- so an agent that clipped a
    box can never be reported as having touched nothing.
    """
    robot = _Body("rover", (-4.2, -4.2, 0.005), (-3.8, -3.8, 0.21),
                  robot=True, joints=2)
    floor = _Body("floor", (-5.0, -5.0, 0.0), (5.0, 5.0, 0.0))
    wall = _Body("wall_north", (-5.0, 4.9, 0.0), (5.0, 5.1, 0.5))
    crate = _obstacle("crate A", 1.0, 1.0, 1.0, 1.0)
    link = _Body("wheel_left", (-4.3, -4.3, 0.0), (-4.1, -4.1, 0.26),
                 member_of="rover")
    hittable, unbounded = r1_core.hittable_bodies(
        [floor, wall, crate, link], robot, matched=[])
    assert "crate a" in hittable, "an unmatched box is still a box"
    assert "wall_north" in hittable
    assert "floor" not in hittable, "the robot drove over the floor"
    assert "wheel_left" not in hittable, "the robot's own link is not a hit"
    assert unbounded == []


def test_a_body_we_could_not_measure_is_named_rather_than_guessed():
    """On upstream Webots a Mesh hull arrives with no bounds. Failing an agent
    for touching a body we could not measure would be scoring our instrument;
    so would silently passing it. It is reported."""
    robot = _Body("rover", (-4.2, -4.2, 0.0), (-3.8, -3.8, 0.2), robot=True,
                  joints=2)
    mystery = _Body("decor")
    hittable, unbounded = r1_core.hittable_bodies([mystery], robot, matched=[])
    assert "decor" not in hittable
    assert unbounded == ["decor"]


def test_a_raised_floor_is_still_a_floor():
    """The drive-over rule is geometric, not a name list: it reads the robot's
    own underside, so a world built on a platform behaves the same."""
    robot = _Body("rover", (-4.2, -4.2, 1.0), (-3.8, -3.8, 1.2), robot=True,
                  joints=2)
    deck = _Body("platform_tile_3", (-5.0, -5.0, 0.9), (0.0, 0.0, 1.0))
    hittable, _ = r1_core.hittable_bodies([deck], robot, matched=[])
    assert "platform_tile_3" not in hittable


# --- SUPERSEDED: the perturbation is kept, marked, and NOT used ---------------

def test_the_perturbation_is_still_reproducible_for_the_sweep_that_retired_it():
    spec = r1_core.obstacle_spec()
    a, oa, _ = r1_core.perturbed_spec(spec, 7)
    b, ob, _ = r1_core.perturbed_spec(spec, 7)
    assert a == b and oa == ob
    for seed in (1, 7, 42, 1234):
        moved, _, _ = r1_core.perturbed_spec(spec, seed)
        for m, o in zip(moved, spec):
            d = math.hypot(m["position"][0] - o["position"][0],
                           m["position"][1] - o["position"][1])
            assert d <= r1_core.PERTURBATION_MAX_M + 1e-6


def test_the_superseded_mechanism_says_so_in_its_own_docstrings():
    """Two mechanisms live in the module. Which one is authoritative must be
    readable from the code, not only from the contract."""
    for fn in (r1_core.seeded_offsets, r1_core.perturbed_spec):
        assert "SUPERSEDED" in fn.__doc__, fn.__name__
    assert "sample_layout" in r1_core.perturbed_spec.__doc__


def test_the_robot_row_is_found_by_body_id_not_roster_position():
    """The roster and the trajectory are DIFFERENT index spaces.

    Measured 2026-08-09 on the webots arm: R1 indexed xyz with a roster
    position and died with "index 11 is out of bounds for axis 0 with size 2".
    On OmniSim the same code had "worked" only because the two orders happened
    to coincide -- which means it could equally have measured the wrong body
    and reported a confident number. This pins the mapping.
    """
    from agentbench.graders.evidence import Trajectory
    import numpy as np
    # roster order: five obstacles first, robot LAST (position 5)
    ids = ["rover"]                       # ...but only the robot is tracked
    traj = Trajectory(body_ids=ids, t=np.array([0.0, 1.0]),
                      xyz=np.zeros((1, 2, 3)))
    assert traj.index_of("rover") == 0, "lookup is by id, not by position"
    assert traj.index_of("crate A") is None, "an untracked body has no row"
