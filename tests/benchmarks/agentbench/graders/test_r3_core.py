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

"""R3 pick_and_place -- unit tests for the sim-neutral core.

These never launch a simulator. They build evidence structures directly, which
is the point: if an assertion needs a simulator to be exercised, it is not
sim-neutral.

The centre of gravity here is the **discrimination** block. C2 shipped a world
whose UNFIXED form passed 5/5 for a whole campaign because nobody asserted the
task could tell a fix from no fix, and every C2 number ever recorded was
uninformative as a result. So R3 states its four named cheats as tests --
do nothing, spawn the cube in the bin, slide it in along the table, teleport it
-- and each one names the assertion that catches it. A future edit that
loosens a threshold past the point of discrimination fails here, in the unit
lane, before it costs a campaign.
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders import r3_core  # noqa: E402
from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, EngineAttribution, EvidenceBundle, ProcessFacts,
    Trajectory)

TASK_DIR = Path(__file__).resolve().parents[1] / "tasks" / "R3_pick_and_place"
DT = 0.016


# --- fixtures in code ---------------------------------------------------------


def _box_body(name, centre, size, **kw):
    lo = tuple(centre[k] - size[k] / 2.0 for k in range(3))
    hi = tuple(centre[k] + size[k] / 2.0 for k in range(3))
    return Body(body_id=kw.pop("body_id", name.upper()), name=name,
                position=tuple(centre), aabb_min=lo, aabb_max=hi, **kw)


def _cube_body(centre=None, dynamic=True):
    centre = centre or r3_core.CUBE_START_XYZ
    s = r3_core.CUBE_SIZE_M
    return _box_body("cube", centre, (s, s, s), body_id="CUBE", kind="Robot",
                     n_joints=0, dynamic=dynamic, robot_class=True,
                     identity_evidence="test fixture")


def _arm_body(n_joints=6):
    return _box_body("arm", (0.0, 0.0, 0.55), (0.30, 0.30, 1.10),
                     body_id="ARM", kind="Robot", n_joints=n_joints,
                     dynamic=True, robot_class=True,
                     identity_evidence="test fixture")


def _scene_bodies(bin_scale=1.0):
    lo, hi = r3_core.TABLE_AABB
    t_centre = tuple((lo[k] + hi[k]) / 2.0 for k in range(3))
    t_size = tuple(hi[k] - lo[k] for k in range(3))
    blo, bhi = r3_core.BIN_AABB
    b_centre = tuple((blo[k] + bhi[k]) / 2.0 for k in range(3))
    b_size = tuple((bhi[k] - blo[k]) * bin_scale for k in range(3))
    return [_box_body("table", t_centre, t_size, kind="Solid",
                      robot_class=False),
            _box_body("bin", b_centre, b_size, kind="Solid",
                      robot_class=False)]


def _bundle(t, cube_xyz, *, cube_body=None, arm_joints=6, bin_scale=1.0,
            clean=True, artifact="pick_and_place.wbt"):
    """One complete evidence bundle, shaped like the OmniSim arm's."""
    cube = cube_body if cube_body is not None else _cube_body()
    arm = _arm_body(arm_joints)
    t0 = BodyInventory(bodies=[cube, arm] + _scene_bodies(bin_scale),
                       frozen=True, t_s=0.0, source="test t=0 scan")
    roster = BodyInventory(bodies=[cube, arm], frozen=True, t_s=0.0,
                           source="test roster")
    arm_xyz = np.tile(np.array([0.0, 0.0, 0.55]), (len(t), 1))
    xyz = np.stack([np.asarray(cube_xyz, dtype=float), arm_xyz])
    traj = Trajectory(body_ids=["CUBE", "ARM"], t=np.asarray(t, dtype=float),
                      xyz=xyz, dt_s=DT,
                      recorded_s=float(t[-1] - t[0]), complete=True,
                      source="test recorder")
    proc = ProcessFacts(exit_code=0 if clean else 1,
                        error_lines=[] if clean else ["ERROR: boom"],
                        log_available=True, log_source="test log",
                        driver_completed=True)
    return EvidenceBundle(
        task=r3_core.TASK, sim="test", adapter="test",
        artifact=artifact, roster=roster, t0=t0, trajectory=traj,
        process=proc,
        attribution=EngineAttribution(backend="newton", solver="mujoco",
                                      source="test"))


def _waypoints(points, dt=DT):
    """Linearly interpolate ``(t, x, y, z)`` waypoints onto a uniform grid."""
    t_end = points[-1][0]
    n = int(round(t_end / dt)) + 1
    ts = np.array([i * dt for i in range(n)])
    out = np.zeros((n, 3))
    for i, tv in enumerate(ts):
        seg = 0
        while seg + 2 < len(points) and points[seg + 1][0] < tv:
            seg += 1
        t0, t1 = points[seg][0], points[seg + 1][0]
        f = 0.0 if t1 <= t0 else min(1.0, max(0.0, (tv - t0) / (t1 - t0)))
        for k in range(3):
            a, b = points[seg][1 + k], points[seg + 1][1 + k]
            out[i][k] = a + (b - a) * f
    return ts, out


# The reference solution's shape: settle, lift clear, translate, lower, let go,
# and leave it there. Every number is derived from the shipped scene.
REST_ON_TABLE = (r3_core.CUBE_START_XYZ[0], r3_core.CUBE_START_XYZ[1],
                 r3_core.CUBE_REST_Z_M)
REST_IN_BIN = (r3_core.BIN_CENTER_XYZ[0], r3_core.BIN_CENTER_XYZ[1],
               r3_core.BIN_INNER_FLOOR_Z_M + r3_core.CUBE_SIZE_M / 2.0)
CARRY_Z = 1.00


def _oracle():
    return _waypoints([
        (0.0,) + REST_ON_TABLE,
        (3.0,) + REST_ON_TABLE,
        (5.0, REST_ON_TABLE[0], REST_ON_TABLE[1], CARRY_Z),
        (8.0, REST_IN_BIN[0], REST_IN_BIN[1], CARRY_Z),
        (9.5, REST_IN_BIN[0], REST_IN_BIN[1], 0.90),
        (10.0,) + REST_IN_BIN,
        (20.0,) + REST_IN_BIN,
    ])


def _grade(t, xyz, **kw):
    return r3_core.grade(_bundle(t, xyz, **kw))


# --- helpers ------------------------------------------------------------------


def test_speed_profile_is_metres_and_seconds():
    t = [0.0, 1.0, 2.0]
    xyz = [(0, 0, 0), (3, 4, 0), (3, 4, 1)]
    speed, step = r3_core.speed_profile(t, xyz)
    assert step == pytest.approx(5.0)
    assert speed == pytest.approx(5.0)


def test_longest_true_run_is_contiguous_not_cumulative():
    """A cube that flickers above the threshold on bounces accumulates time
    without ever having been carried; only a contiguous run means 'held'."""
    t = [i * 0.1 for i in range(10)]
    mask = [True, False, True, False, True, True, True, False, True, False]
    i0, i1, span = r3_core.longest_true_run(mask, t)
    assert (i0, i1) == (4, 6)
    assert span == pytest.approx(0.2)
    assert r3_core.longest_true_run([False] * 5, t[:5]) == (None, None, 0.0)


def test_in_bin_requires_the_WHOLE_cube_between_the_walls():
    bx, by, _bz = r3_core.BIN_CENTER_XYZ
    z = r3_core.BIN_INNER_FLOOR_Z_M + r3_core.CUBE_SIZE_M / 2.0
    assert r3_core.in_bin(bx, by, z)
    # just inside / just outside the inset
    assert r3_core.in_bin(bx + r3_core.BIN_INSET_M - 1e-6, by, z)
    assert not r3_core.in_bin(bx + r3_core.BIN_INSET_M + 1e-3, by, z)
    # perched ON the rim is not IN the bin
    assert not r3_core.in_bin(bx, by, r3_core.BIN_RIM_Z_M
                              + r3_core.CUBE_SIZE_M / 2.0)
    # resting on the table beside it is not in it either
    assert not r3_core.in_bin(r3_core.CUBE_START_XYZ[0],
                              r3_core.CUBE_START_XYZ[1], r3_core.CUBE_REST_Z_M)


def test_the_cube_is_found_by_geometry_never_by_name():
    """R1's lesson: an agent that builds the specified scene and calls the
    body something else has still built the specified scene."""
    odd = _cube_body()
    odd.name = "widget"
    odd.body_id = "WIDGET_7"
    idx, cube, how, n = r3_core.find_cube([odd, _arm_body()]
                                          + _scene_bodies())
    assert cube is odd and how == "aabb" and n == 1
    assert idx == 0


def test_a_body_of_the_wrong_size_is_not_the_cube():
    fat = _box_body("cube", r3_core.CUBE_START_XYZ, (0.20, 0.20, 0.20),
                    kind="Robot", n_joints=0, dynamic=True, robot_class=True)
    _idx, cube, how, _n = r3_core.find_cube([fat, _arm_body()])
    assert cube is None and how == "none"


def test_the_pose_row_resolves_by_id_and_by_name():
    """The two arms disagree about what Trajectory.body_ids holds; both
    channels must resolve without ever guessing a position."""
    cube = _cube_body()
    by_id = Trajectory(body_ids=["ARM", "CUBE"], xyz=np.zeros((2, 3, 3)))
    assert r3_core.trajectory_row(by_id, cube) == (1, "body_id")
    by_name = Trajectory(body_ids=["arm", "cube"], xyz=np.zeros((2, 3, 3)))
    assert r3_core.trajectory_row(by_name, cube) == (1, "name")
    neither = Trajectory(body_ids=["a", "b"], xyz=np.zeros((2, 3, 3)))
    assert r3_core.trajectory_row(neither, cube) == (None, "unresolved")


# --- the contract: nothing is written down twice ------------------------------


def test_every_threshold_comes_from_meta_json():
    """The grader READS meta.json's constants; this proves it, by checking the
    module's value against the file for every declared name."""
    meta = json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))
    assert meta["constants"], "the constants block must not be empty"
    for name, declared in meta["constants"].items():
        assert getattr(r3_core, name) == pytest.approx(declared), (
            "meta.json says %s=%s but the grader uses %s"
            % (name, declared, getattr(r3_core, name)))
    from agentbench import tasks as _t
    assert meta["timeout_s"] == min(3 * meta["par_s"],
                                    _t.TASK_HARD_CEILING_S)
    assert meta["par_s"] == 600
    # ONE run per (task, arm) since 2026-08-10 (SPEC 3.5): the suite
    # stopped running repeats, so this is 1 and a single observation is
    # an outcome rather than a rate. It used to be 5.
    assert meta["repeats_default"] == 1
    assert meta["budget"]["runs_per_cell"] == 1
    assert meta["budget"]["variance_measured"] is False
    assert meta["tier"] == "robotics"
    assert meta["grader"] == "agentbench.graders.r3"


def test_meta_declares_what_it_does_not_measure():
    """Retraction and release are not in the evidence on either arm. Saying so
    is the difference between a limit and a lie."""
    meta = json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))
    assert meta["not_measured"], "state the gaps"
    assert "known_hole" in meta
    assert "not_pre_tuned" in meta, (
        "the fixture must declare that no grasp setting is pre-baked on "
        "either arm, or the comparison is tilted in the fixture itself")


def test_the_prompt_is_the_spec_text_and_nothing_else():
    text = (TASK_DIR / "prompt.txt").read_text(encoding="utf-8")
    assert text.startswith("Build a robotic pick-and-place simulation using "
                           "the benchmark arm asset.")
    assert "remain inside the bin for at least 2 simulated seconds" in text
    assert "Do not teleport the cube" in text
    # the prompt must not leak the grader's thresholds
    for leaked in ("REST_BAND", "AIRBORNE", "grader", "assertion"):
        assert leaked not in text


# --- the scene the arms actually ship -----------------------------------------


def _block(world_text, header):
    start = world_text.index(header)
    rest = world_text[start:]
    nxt = rest.find("\nDEF ", 1)
    return rest[:nxt] if nxt != -1 else rest


_NUM = r"(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"
_TOKEN = re.compile(r"\b(translation|size)\s+%s\s+%s\s+%s" % (_NUM, _NUM, _NUM))


def _aabb_from_block(block):
    """World AABB of a DEF block, from its own translation + every Box size.

    A tiny reader rather than an engine load, for C2's reason: this test's job
    is to check what the FILE says, and it must run without a simulator. The
    file pairs each Box with the Pose translation immediately above it, and
    the block's own translation comes first.
    """
    base = None
    pose = (0.0, 0.0, 0.0)
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for m in _TOKEN.finditer(block):
        vals = tuple(float(m.group(i)) for i in (2, 3, 4))
        if m.group(1) == "translation":
            if base is None:
                base = vals
            else:
                pose = vals
            continue
        centre = [base[k] + pose[k] for k in range(3)]
        for k in range(3):
            lo[k] = min(lo[k], centre[k] - vals[k] / 2.0)
            hi[k] = max(hi[k], centre[k] + vals[k] / 2.0)
    assert base is not None and lo[0] != math.inf
    return tuple(round(v, 6) for v in lo), tuple(round(v, 6) for v in hi)


ARMS = [pytest.param(TASK_DIR / "initial" / "pick_and_place.wbt", id="omnisim"),
        pytest.param(TASK_DIR / "initial_webots" / "pick_and_place.wbt",
                     id="webots")]


@pytest.mark.parametrize("world", ARMS)
def test_the_shipped_world_matches_the_frozen_scene_spec(world):
    """The world is the fixture, scene.json is the contract, and the grader
    reads the contract. A drift between them would grade a scene nobody ships."""
    spec = r3_core.scene_spec()
    text = world.read_text(encoding="utf-8")

    lo, hi = _aabb_from_block(_block(text, "DEF TABLE Solid {"))
    assert lo == tuple(spec["table"]["aabb_min"])
    assert hi == tuple(spec["table"]["aabb_max"])
    assert hi[2] == pytest.approx(spec["table"]["top_z"]), (
        "the table's top surface is what R3.4 and R3.5 are stated against")

    lo, hi = _aabb_from_block(_block(text, "DEF BIN Solid {"))
    assert lo == tuple(spec["bin"]["aabb_min"])
    assert hi == tuple(spec["bin"]["aabb_max"])

    lo, hi = _aabb_from_block(_block(text, "DEF CUBE Robot {"))
    s = spec["cube"]["size"]
    for k in range(3):
        assert hi[k] - lo[k] == pytest.approx(s)
        assert (hi[k] + lo[k]) / 2.0 == pytest.approx(
            spec["cube"]["position"][k])


@pytest.mark.parametrize("world", ARMS)
def test_the_cube_is_authored_as_a_tracked_body(world):
    """The pose recorder samples Robot nodes on BOTH arms and a plain dynamic
    Solid on only one of them. If the cube ever stops being a Robot node it
    silently loses its trajectory on OmniSim and R3 becomes ungradable there."""
    text = world.read_text(encoding="utf-8")
    assert "DEF CUBE Robot {" in text
    cube = _block(text, "DEF CUBE Robot {")
    assert "physics Physics {" in cube, "R3.3's dynamic clause needs this"
    assert "boundingObject" in cube, "a cube with no collider cannot be grasped"


@pytest.mark.parametrize("world", ARMS)
def test_no_remote_protos(world):
    text = world.read_text(encoding="utf-8")
    assert "http://" not in text and "https://" not in text, (
        "benchmark worlds are local-asset-only")


def test_both_arms_ship_identical_scene_geometry():
    """A cross-simulator comparison is only like-for-like if the scene is.
    Compared as TEXT from the first body onwards, which is stricter than
    comparing AABBs: it also catches a mass, a colour or a field ordering
    drifting between the arms."""
    a = (TASK_DIR / "initial" / "pick_and_place.wbt").read_text(
        encoding="utf-8")
    b = (TASK_DIR / "initial_webots" / "pick_and_place.wbt").read_text(
        encoding="utf-8")
    marker = "DEF FLOOR Solid {"
    assert a[a.index(marker):] == b[b.index(marker):], (
        "the R3 arms disagree about the scene; the comparison would be "
        "measuring the fixture, not the simulators")


def test_both_arms_ship_identical_benchmark_assets():
    for name in ("scene.json", "arm.json"):
        a = (TASK_DIR / "initial" / "benchmark_assets" / name).read_bytes()
        b = (TASK_DIR / "initial_webots" / "benchmark_assets" / name
             ).read_bytes()
        assert a == b


def test_the_geometry_is_actually_gradeable():
    """The scene numbers and the thresholds have to be mutually consistent, or
    an assertion is unsatisfiable (or free) no matter what the agent does."""
    # a cube fits inside the bin with room to spare
    assert r3_core.BIN_INSET_M > 0.02, (
        "the bin must accept the cube with real clearance, or R3.7 is a "
        "millimetre-accuracy test of the agent's placement rather than a "
        "containment test")
    # 'airborne' is above the rim, which is above the table
    assert (r3_core.Z_AIRBORNE_M - r3_core.CUBE_SIZE_M / 2.0
            > r3_core.BIN_RIM_Z_M > r3_core.TABLE_TOP_Z_M)
    # the transfer is a real move, not a nudge
    reach = math.hypot(r3_core.CUBE_START_XYZ[0] - r3_core.BIN_CENTER_XYZ[0],
                       r3_core.CUBE_START_XYZ[1] - r3_core.BIN_CENTER_XYZ[1])
    assert reach > 2 * r3_core.CARRY_MIN_XY_M, (
        "the carry floor must be well inside the distance the task requires")
    # the cube starts on the table's top surface, inside its footprint
    lo, hi = r3_core.TABLE_AABB
    for k in (0, 1):
        assert lo[k] + 0.05 < r3_core.CUBE_START_XYZ[k] < hi[k] - 0.05
    assert hi[2] == pytest.approx(r3_core.TABLE_TOP_Z_M)
    # the bin sits on the table, wholly inside its footprint
    blo, bhi = r3_core.BIN_AABB
    assert blo[2] == pytest.approx(r3_core.TABLE_TOP_Z_M)
    for k in (0, 1):
        assert lo[k] < blo[k] and bhi[k] < hi[k]
    # the cube does not start inside the bin
    assert not r3_core.in_bin(*r3_core.CUBE_START_XYZ)


# --- the oracle ---------------------------------------------------------------


def test_a_real_pick_and_place_passes():
    t, xyz = _oracle()
    v = r3_core.grade(_bundle(t, xyz))
    assert v.outcome == "PASS", v.summary()
    assert len(v.assertions) == 8
    assert not v.vacuous, v.vacuous


def test_the_run_being_dirty_fails_only_r3_1():
    t, xyz = _oracle()
    v = r3_core.grade(_bundle(t, xyz, clean=False))
    assert v.failed == ["R3.1"], v.summary()


# --- discrimination: the four named cheats ------------------------------------
#
# C2 shipped a task whose unfixed world passed 5/5 for a whole campaign. Each
# test below names the cheat, the trajectory it produces, and the assertion
# that must catch it.


def test_cheat_a_doing_nothing_fails():
    """(a) The agent does nothing at all. The cube sits where it was put."""
    t, xyz = _waypoints([(0.0,) + REST_ON_TABLE, (20.0,) + REST_ON_TABLE])
    v = _grade(t, xyz)
    assert v.outcome == "FAIL"
    for aid in ("R3.5", "R3.6", "R3.7"):
        assert aid in v.failed, v.summary()


def test_cheat_a_no_artifact_at_all_fails_every_assertion():
    t, xyz = _oracle()
    b = _bundle(t, xyz)
    b.artifact = None
    v = r3_core.grade(b)
    assert v.outcome == "FAIL" and v.progress == 0
    assert len(v.failed) == 8


def test_cheat_b_spawning_the_cube_in_the_bin_fails():
    """(b) The agent edits the world so the cube starts in the bin and then
    'does' the task by leaving it there."""
    t, xyz = _waypoints([(0.0,) + REST_IN_BIN, (20.0,) + REST_IN_BIN])
    v = _grade(t, xyz, cube_body=_cube_body(centre=REST_IN_BIN))
    assert v.outcome == "FAIL"
    assert "R3.3" in v.failed, (
        "a cube authored somewhere other than the specified start is not the "
        "benchmark's cube: %s" % v.summary())
    assert "R3.4" in v.failed and "R3.5" in v.failed


def test_cheat_b_variant_start_pose_moved_but_body_still_matched():
    """The same cheat where only the RECORDED start moved (the t=0 body is
    still the shipped one). R3.4 is the clause that has to hold here."""
    t, xyz = _waypoints([(0.0,) + REST_IN_BIN, (20.0,) + REST_IN_BIN])
    v = _grade(t, xyz)
    assert "R3.4" in v.failed and "R3.5" in v.failed, v.summary()


def test_cheat_c_sliding_it_in_along_the_table_fails_on_r3_5():
    """(c) No lift: the cube is pushed across the table, rides up the outer
    wall and tips in. Deliberately the GENEROUS version of this cheat -- it
    ends up genuinely inside the bin, at rest, having descended, so R3.4,
    R3.6, R3.7 and R3.8 are all satisfied and R3.5 is the only thing standing
    between it and a PASS. If this test ever goes green, the task has stopped
    measuring a grasp."""
    t, xyz = _waypoints([
        (0.0,) + REST_ON_TABLE,
        (3.0,) + REST_ON_TABLE,
        (7.0, 0.32, 0.33, r3_core.CUBE_REST_Z_M),
        (7.30, REST_IN_BIN[0], REST_IN_BIN[1], 0.90),
        (7.35, REST_IN_BIN[0], REST_IN_BIN[1], 0.90),
        (7.80,) + REST_IN_BIN,
        (20.0,) + REST_IN_BIN,
    ])
    v = _grade(t, xyz)
    assert v.outcome == "FAIL"
    assert v.failed == ["R3.5"], (
        "R3.5 must be the discriminator for the no-lift cheat: %s"
        % v.summary())
    a5 = {a.id: a for a in v.assertions}["R3.5"]
    assert a5.measured["longest contiguous time clear of everything (s)"] \
        < r3_core.AIRBORNE_MIN_S


def test_cheat_c_pure_slide_never_leaves_the_table():
    t, xyz = _waypoints([
        (0.0,) + REST_ON_TABLE,
        (3.0,) + REST_ON_TABLE,
        (9.0, REST_IN_BIN[0], REST_IN_BIN[1], r3_core.CUBE_REST_Z_M),
        (20.0, REST_IN_BIN[0], REST_IN_BIN[1], r3_core.CUBE_REST_Z_M),
    ])
    v = _grade(t, xyz)
    assert "R3.5" in v.failed and "R3.6" in v.failed, v.summary()


def test_cheat_d_teleporting_it_fails_on_r3_8():
    """(d) One frame on the table, the next in the bin."""
    n1, n2 = 313, 938
    t = np.array([i * DT for i in range(n1 + n2)])
    xyz = np.array([list(REST_ON_TABLE)] * n1 + [list(REST_IN_BIN)] * n2)
    v = _grade(t, xyz)
    assert v.outcome == "FAIL"
    assert "R3.8" in v.failed, v.summary()
    a8 = {a.id: a for a in v.assertions}["R3.8"]
    assert a8.measured["fastest the cube ever moved (m/s)"] \
        > r3_core.MAX_SPEED_MPS
    assert a8.measured["largest single-sample step (m)"] \
        > r3_core.MAX_STEP_DISPLACEMENT_M


def test_the_teleport_bound_is_dt_independent():
    """A coarser recording must not buy a bigger jump. The speed clause is why
    this holds where R1's per-sample distance bound would not."""
    for dt in (0.008, 0.016, 0.032, 0.064):
        n = int(round(6.0 / dt))
        t = np.array([i * dt for i in range(2 * n)])
        xyz = np.array([list(REST_ON_TABLE)] * n + [list(REST_IN_BIN)] * n)
        v = _grade(t, xyz)
        assert "R3.8" in v.failed, "dt=%s let a teleport through" % dt


# --- further cheats the same evidence has to catch ----------------------------


def test_dropping_it_beside_the_bin_fails_on_r3_7():
    t, xyz = _waypoints([
        (0.0,) + REST_ON_TABLE,
        (3.0,) + REST_ON_TABLE,
        (5.0, REST_ON_TABLE[0], REST_ON_TABLE[1], CARRY_Z),
        (8.0, 0.30, 0.20, CARRY_Z),
        (9.5, 0.30, 0.20, 0.90),
        (10.0, 0.30, 0.20, r3_core.CUBE_REST_Z_M),
        (20.0, 0.30, 0.20, r3_core.CUBE_REST_Z_M),
    ])
    v = _grade(t, xyz)
    assert v.failed == ["R3.7"], v.summary()


def test_bouncing_back_out_of_the_bin_fails_on_r3_7():
    t, xyz = _oracle()
    xyz[-40:, 0] = r3_core.BIN_CENTER_XYZ[0] + 0.5   # it left, late
    v = _grade(t, xyz)
    assert "R3.7" in v.failed, v.summary()


def test_still_being_carried_at_the_end_fails():
    t, xyz = _waypoints([
        (0.0,) + REST_ON_TABLE,
        (3.0,) + REST_ON_TABLE,
        (5.0, REST_ON_TABLE[0], REST_ON_TABLE[1], CARRY_Z),
        (8.0, REST_IN_BIN[0], REST_IN_BIN[1], CARRY_Z),
        (20.0, REST_IN_BIN[0], REST_IN_BIN[1], CARRY_Z),
    ])
    v = _grade(t, xyz)
    assert "R3.6" in v.failed and "R3.7" in v.failed, v.summary()


def test_a_cube_with_its_dynamics_removed_fails_r3_3():
    """The 'hold it by nailing it down' cheat, C2.3's clause applied here."""
    t, xyz = _oracle()
    v = _grade(t, xyz, cube_body=_cube_body(dynamic=False))
    assert "R3.3" in v.failed, v.summary()


def test_widening_the_bin_fails_r3_2():
    """Every later assertion is stated against the bin volume, so the bin has
    to be measured and pinned or 'inside the bin' means whatever the agent
    wants it to mean."""
    t, xyz = _oracle()
    v = _grade(t, xyz, bin_scale=3.0)
    assert "R3.2" in v.failed, v.summary()


def test_no_arm_at_all_fails_r3_3():
    t, xyz = _oracle()
    v = _grade(t, xyz, arm_joints=0)
    assert "R3.3" in v.failed, v.summary()


def test_a_missing_fixture_body_fails_r3_2_and_says_which():
    t, xyz = _oracle()
    b = _bundle(t, xyz)
    b.t0.bodies = [x for x in b.t0.bodies if x.name != "bin"]
    v = r3_core.grade(b)
    assert "R3.2" in v.failed
    a2 = {a.id: a for a in v.assertions}["R3.2"]
    assert a2.measured["fixture bodies not found"] == ["bin"]


# --- honesty about the instrument ---------------------------------------------


def test_an_unattributed_run_is_invalid_not_failed():
    t, xyz = _oracle()
    b = _bundle(t, xyz)
    b.attribution = None
    v = r3_core.grade(b)
    assert v.outcome == "INVALID", v.outcome


def test_a_run_with_no_samples_is_artifact_invalid():
    t, xyz = _oracle()
    b = _bundle(t, xyz)
    b.trajectory = None
    b.motion_error = "the recorder produced no samples"
    v = r3_core.grade(b)
    assert v.progress == 1 and v.outcome == "FAIL"
    assert len(v.failed) == 8


def test_matching_the_cube_without_an_aabb_marks_the_size_clause_vacuous():
    """An adapter with no bounds query can still find the cube by its t=0
    point -- but it has NOT checked the size, and the verdict has to say so."""
    t, xyz = _oracle()
    b = _bundle(t, xyz)
    for body in b.t0.bodies:
        body.aabb_min = body.aabb_max = None
    v = r3_core.grade(b)
    a3 = {a.id: a for a in v.assertions}["R3.3"]
    assert a3.measured["cube matched at t=0 by"] == "position"
    assert a3.vacuous, "the unverified size clause must be marked vacuous"
    # ...and the scene it could not measure is a FAIL, never a silent pass
    assert "R3.2" in v.failed


def test_a_pose_series_with_no_time_base_declines_rather_than_guessing():
    """Every threshold from R3.4 on is in seconds. Substituting sample indices
    would keep grading and quietly mean something else."""
    t, xyz = _oracle()
    b = _bundle(t, xyz)
    b.trajectory.t = None
    v = r3_core.grade(b)
    assert v.outcome == "FAIL"
    for aid in ("R3.4", "R3.5", "R3.6", "R3.7", "R3.8"):
        assert aid in v.failed
    assert "R3.1" not in v.failed and "R3.2" not in v.failed


def test_the_known_hole_is_real_and_declared():
    """R3.8 bounds a rate. A supervisor animating the cube smoothly along the
    oracle's own arc produces the oracle's own numbers, so it passes -- there
    is nothing in either arm's evidence that separates the two. The hole is
    pinned here so it stays a published fact rather than a surprise."""
    t, xyz = _oracle()
    assert r3_core.grade(_bundle(t, xyz)).outcome == "PASS"
    meta = json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))
    assert "supervisor" in meta["known_hole"]
    assert "R3.8" in meta["known_hole"]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
