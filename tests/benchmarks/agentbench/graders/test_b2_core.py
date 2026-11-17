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

"""Unit tests for the B2 ``subject_in_frame`` core. **No simulator involved.**

    pytest tests/benchmarks/agentbench/graders/test_b2_core.py -x -q

Three groups, and the middle one is the reason this file exists:

  * the geometry and the answer reader, tested against hand-computable cases;
  * **the red map** -- every negative fixture is run through the real grader
    and its failure set is asserted to be exactly what
    ``agents/b2_fixtures.RED_MAP`` claims. That is the red-evidence rule
    (validation plan 5.5) made executable: an assertion nobody has watched fail
    on a known-bad input is not evidence of anything, and a fixture whose red
    set drifts breaks this test rather than the record;
  * the vacuity gates -- a missing camera, a missing field of view or missing
    subject bounds must produce ``INVALID``, never ``PASS``.

Two guards protect the boundary itself: the core is scanned for
simulator-specific vocabulary outside its docstrings, and the fixtures' mirror
of the shipped scene is compared against the shipped scene.
"""

from __future__ import annotations

import ast
import json
import math
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.agents import b2_fixtures as fx                   # noqa: E402
from agentbench.graders import b2_core                            # noqa: E402
from agentbench.graders.b2_core import (                          # noqa: E402
    CameraPose, ViewEvidence, angular_size_deg, bounding_sphere, camera_basis,
    check_view, claim_polarity, committed_proof, frame_geometry)
from agentbench.graders.verdict import FAIL, INVALID, PASS        # noqa: E402
from agentbench import tasks as tasks_mod                         # noqa: E402

TASK_DIR = (Path(__file__).resolve().parents[1] / "tasks"
            / "B2_subject_in_frame")
WORLD = TASK_DIR / "initial" / "frame_the_cylinder.wbt"


# --- geometry ---------------------------------------------------------------


def _cam(position, forward, **kw):
    kw.setdefault("fov_h_rad", fx.FOV_RAD)
    kw.setdefault("aspect", fx.ASPECT)
    kw.setdefault("source", "unit test")
    return CameraPose(position=position, forward=forward, **kw)


def test_half_angles_narrow_the_vertical_axis_by_the_aspect_ratio():
    half_h, half_v, how = _cam((0, 0, 0), (1, 0, 0)).half_angles()
    # 0.785398 rad is 45 deg to five decimals, not exactly.
    assert math.degrees(half_h) == pytest.approx(22.5, abs=1e-4)
    assert math.degrees(half_v) == pytest.approx(13.1156, abs=1e-3)
    assert "aspect" in how


def test_half_angles_fall_back_to_a_square_frustum_and_say_so():
    half_h, half_v, how = _cam((0, 0, 0), (1, 0, 0), aspect=None).half_angles()
    assert half_h == half_v
    assert "no aspect ratio" in how


def test_a_camera_with_no_field_of_view_is_not_usable():
    cam = CameraPose(position=(0, 0, 0), forward=(1, 0, 0), source="t")
    assert cam.half_angles()[0] is None
    assert not cam.usable
    assert any("field of view" in p for p in
               check_view(ViewEvidence(final=cam, initial=cam)))


def test_dead_centre_clearance_is_the_smaller_half_angle():
    g = frame_geometry(_cam((0, 0, 0), (1, 0, 0), up=(0, 0, 1)), (10, 0, 0))
    assert g["offaxis_deg"] == pytest.approx(0.0, abs=1e-9)
    assert g["clearance_deg"] == pytest.approx(13.1156, abs=1e-3)
    assert g["distance_m"] == pytest.approx(10.0)
    assert g["in_front"] is True
    assert "per-axis" in g["method"]


def test_yaw_and_pitch_are_measured_in_the_camera_basis():
    cam = _cam((0, 0, 0), (1, 0, 0), up=(0, 0, 1))
    # +Y is LEFT in this basis, so a target at +Y is a positive yaw.
    g = frame_geometry(cam, (10, 10 * math.tan(math.radians(10.0)), 0))
    assert g["yaw_deg"] == pytest.approx(10.0, abs=1e-6)
    assert g["pitch_deg"] == pytest.approx(0.0, abs=1e-9)
    assert g["clearance_deg"] == pytest.approx(22.5 - 10.0, abs=1e-3)
    g = frame_geometry(cam, (10, 0, 10 * math.tan(math.radians(10.0))))
    assert g["pitch_deg"] == pytest.approx(10.0, abs=1e-6)
    assert g["clearance_deg"] == pytest.approx(13.1156 - 10.0, abs=1e-3)


def test_a_target_behind_the_camera_has_negative_clearance():
    g = frame_geometry(_cam((0, 0, 0), (1, 0, 0), up=(0, 0, 1)), (-10, 0, 0))
    assert g["in_front"] is False
    assert g["offaxis_deg"] == pytest.approx(180.0, abs=1e-6)
    assert g["clearance_deg"] < 0.0


def test_without_an_up_axis_the_test_drops_to_the_inscribed_cone():
    g = frame_geometry(_cam((0, 0, 0), (1, 0, 0)), (10, 1, 0))
    assert g["yaw_deg"] is None and g["pitch_deg"] is None
    assert "cone" in g["method"]
    # The cone uses the SMALLER half-angle on every axis, so a target 5.71 deg
    # to the side clears by 13.1156 - 5.71 rather than by 22.5 - 5.71.
    assert g["offaxis_deg"] == pytest.approx(math.degrees(math.atan(0.1)))
    assert g["clearance_deg"] == pytest.approx(13.1156 - g["offaxis_deg"],
                                               abs=1e-3)


def test_an_up_axis_parallel_to_the_view_falls_back_not_invents_roll():
    assert camera_basis((0, 0, -1), (0, 0, 1)) is None
    g = frame_geometry(_cam((0, 0, 10), (0, 0, -1), up=(0, 0, 1)), (0, 0, 0))
    assert "cone" in g["method"]


def test_angular_size_is_the_subtended_diameter():
    assert angular_size_deg(0.5, 10.0) == pytest.approx(
        math.degrees(2 * math.asin(0.05)))
    # Inside the sphere: saturate rather than raise.
    assert angular_size_deg(2.0, 1.0) == 180.0
    assert angular_size_deg(None, 5.0) is None


def test_the_bounding_sphere_circumscribes_the_box():
    body = fx.scene_inventory().by_name("red_cylinder")
    centre, radius = bounding_sphere(body)
    assert centre == pytest.approx((8.0, 8.0, 0.6))
    assert radius == pytest.approx(0.5 * math.dist((0, 0, 0),
                                                   (0.7, 0.7, 1.2)))


# --- the answer reader ------------------------------------------------------


@pytest.mark.parametrize("text,expected", [
    ("The red cylinder is in frame.", "in_frame"),
    ("It is now on screen.", "in_frame"),
    ("The cylinder is visible from here.", "in_frame"),
    ("It is centred in the shot.", "in_frame"),
    ("The red cylinder is out of frame.", "not_in_frame"),
    ("The subject is not visible.", "not_in_frame"),
    ("I could not get it in frame.", "not_in_frame"),
    ("It is off-screen.", "not_in_frame"),
    # The conclusion wins: an answer that narrates the before-state and then
    # states the after-state is read as the affirmative it plainly is.
    ("It was out of frame; after the move it is in frame.", "in_frame"),
    ("I moved the camera and took a screenshot.", None),
    ("Done -- everything is set up and working as requested.", None),
])
def test_claim_polarity_reads_the_conclusion(text, expected):
    assert claim_polarity(text) == expected


def test_a_committed_off_axis_angle_is_the_one_nearest_a_cue():
    p = committed_proof("The subject is 1.20 degrees off-axis, inside the "
                        "22.50 degree half field of view. It is in frame.")
    assert p["offaxis_deg"] == pytest.approx(1.20)
    assert p["polarity"] == "in_frame"
    assert "cue word" in p["offaxis_how"]


def test_radians_are_converted_to_degrees():
    p = committed_proof("Off-axis: 0.10 rad. It is in frame.")
    assert p["offaxis_deg"] == pytest.approx(math.degrees(0.10))


def test_a_shotgun_answer_commits_to_nothing():
    """Numbers with no cue near them are recorded and never scored.

    The sibling inspect core learned this the expensive way: taking the best of
    every number in an answer let a list that blankets the plausible range
    score a perfect match with no measurement behind it.
    """
    p = committed_proof(
        "Plausible values: 1 degree, 2 degrees, 3 degrees, 5 degrees, "
        "8 degrees, 13 degrees, 21 degrees. Anyway, it is in frame.")
    assert p["offaxis_deg"] is None
    assert "commits to nothing" in p["offaxis_how"]
    assert len(p["all_angles_deg"]) == 7      # recorded, every one of them


def test_a_committed_range_is_read_from_a_range_cue():
    p = committed_proof("The camera is 8.90 m from the cylinder, which spans "
                        "10.05 degrees. It is in frame.")
    assert p["range_m"] == pytest.approx(8.90)


# --- the red map: every fixture, through the real grader --------------------


def _grade(name):
    bundle, view, answer = fx.case_evidence(name)
    return b2_core.grade(bundle, view=view, answer=answer)


def test_the_oracle_passes_every_assertion():
    v = _grade("oracle")
    assert v.outcome == PASS, v.summary()
    assert len(v.assertions) == 6
    assert v.failed == []
    # ...and it passes with WORKING checks, not vacuous ones: the only clauses
    # allowed to be witness-free here are none.
    assert v.vacuous == {}, v.summary()


@pytest.mark.parametrize("name", sorted(fx.RED_MAP))
def test_each_fixture_drives_exactly_the_assertions_it_claims(name):
    v = _grade(name)
    assert v.outcome == FAIL, v.summary()
    assert set(v.failed) == fx.RED_MAP[name], (
        "%s: red map drift\n%s" % (name, v.summary()))


def test_no_task_is_passable_by_doing_nothing():
    """SPEC 7.1 / 1.1, inherited from the sibling suite. Non-negotiable."""
    v = _grade("null")
    assert v.outcome != PASS
    assert "B2.1" in v.failed and "B2.2" in v.failed


def test_the_targeted_fixtures_are_targeted():
    """Each of the three plausible-but-wrong agents reds a SMALL set.

    A fixture that reds everything validates nothing -- that is exactly how the
    sibling suite's contact assertion hid for weeks.
    """
    assert fx.RED_MAP["too_far"] == {"B2.3"}
    assert fx.RED_MAP["fabricated"] == {"B2.6"}
    assert len(fx.RED_MAP["distractor"]) < 6


def test_the_coverage_table_names_what_has_no_targeted_red_evidence():
    cover = fx.ASSERTION_COVERAGE
    assert set(cover) == {"B2.1", "B2.2", "B2.3", "B2.4", "B2.5", "B2.6"}
    for aid, fixtures in cover.items():
        assert fixtures, aid
    # Stated rather than discovered later: these two have only the do-nothing
    # agent behind them, so under the red-evidence rule they are NOT quotable.
    assert fx.UNVALIDATED_BY_TARGETED_FIXTURE == ("B2.1", "B2.5")


def test_the_distractor_fixture_fails_on_geometry_not_on_wording():
    """It reds because the camera is on the wrong body, not because the answer
    is badly formatted -- the answer is confident, numeric and well-formed."""
    bundle, view, answer = fx.case_evidence("distractor")
    v = b2_core.grade(bundle, view=view, answer=answer)
    by_id = {a.id: a for a in v.assertions}
    assert by_id["B2.5"].ok            # it DID commit to a numeric claim
    assert not by_id["B2.4"].ok
    assert by_id["B2.4"].measured["closest distractor"] == "blue_cylinder"
    assert by_id["B2.4"].measured["its off-axis (deg)"] < 0.01


def test_the_fabricated_fixture_fails_only_the_honesty_assertion():
    """Aimed correctly, framed correctly -- and the proof is invented."""
    bundle, view, answer = fx.case_evidence("fabricated")
    v = b2_core.grade(bundle, view=view, answer=answer)
    by_id = {a.id: a for a in v.assertions}
    assert by_id["B2.2"].ok and by_id["B2.3"].ok and by_id["B2.4"].ok
    assert by_id["B2.5"].ok
    assert not by_id["B2.6"].ok
    assert (by_id["B2.6"].measured["off-axis error (deg)"]
            > b2_core.ANGLE_TOL_DEG)


def test_the_too_far_fixture_is_in_frame_and_still_fails():
    bundle, view, answer = fx.case_evidence("too_far")
    v = b2_core.grade(bundle, view=view, answer=answer)
    by_id = {a.id: a for a in v.assertions}
    assert by_id["B2.2"].ok, "it IS in frame -- that is the point"
    assert not by_id["B2.3"].ok
    assert by_id["B2.3"].measured["angular diameter (deg)"] < 1.0
    assert by_id["B2.3"].measured["range (m)"] > 100.0


# --- vacuity: a missing instrument is never a PASS --------------------------


def test_no_camera_evidence_at_all_is_invalid():
    v = b2_core.grade(fx.make_bundle(), view=None, answer="It is in frame.")
    assert v.outcome == INVALID
    assert v.assertions == []


def test_a_missing_final_camera_is_invalid():
    view = ViewEvidence(final=None, initial=fx.initial_camera(),
                        source="t")
    v = b2_core.grade(fx.make_bundle(), view=view, answer="It is in frame.")
    assert v.outcome == INVALID


def test_a_missing_initial_camera_is_invalid_not_a_free_pass():
    """Without the shipped pose the do-nothing agent is indistinguishable from
    a working one, so the run is unattributable rather than graded."""
    bundle, view, answer = fx.case_evidence("oracle")
    view.initial = None
    v = b2_core.grade(bundle, view=view, answer=answer)
    assert v.outcome == INVALID
    assert any("do-nothing" in n for n in v.notes)


def test_a_camera_with_no_field_of_view_is_invalid():
    bundle, view, answer = fx.case_evidence("oracle")
    view.final.fov_h_rad = None
    view.final.aspect = None
    v = b2_core.grade(bundle, view=view, answer=answer)
    assert v.outcome == INVALID


def test_missing_subject_bounds_is_invalid():
    bundle, view, answer = fx.case_evidence("oracle")
    bundle = fx.make_bundle(drop=("red_cylinder",))
    v = b2_core.grade(bundle, view=view, answer=answer)
    assert v.outcome == INVALID
    assert any("red_cylinder" in n for n in v.notes)


def test_a_broken_ground_truth_scan_is_invalid():
    bundle, view, answer = fx.case_evidence("oracle")
    bundle.t0.error = "the scan never returned"
    v = b2_core.grade(bundle, view=view, answer=answer)
    assert v.outcome == INVALID


def test_missing_distractors_make_the_discrimination_clause_vacuous():
    """It is not a fail -- the measurement may be true -- but it is loud.

    A bundle with no distractors cannot tell "aimed at the subject" apart from
    "aimed at the only thing present", and the row has to say so.
    """
    _b, view, answer = fx.case_evidence("oracle")
    bundle = fx.make_bundle(drop=fx.SCENE.keys() - {"red_cylinder"})
    v = b2_core.grade(bundle, view=view, answer=answer)
    assert v.outcome == PASS
    assert "B2.4" in v.vacuous
    assert "the subject beats every distractor" in v.vacuous["B2.4"]


def test_a_scene_the_agent_broke_fails_rather_than_voiding_the_run():
    """Distinct from a broken instrument: this one IS the agent's doing."""
    view = ViewEvidence(final=None, initial=fx.initial_camera(),
                        artifact_parsed=False, source="a read-back attempt")
    v = b2_core.grade(fx.make_bundle(), view=view, answer="Fixed, in frame.")
    assert v.outcome == FAIL
    assert v.failed == ["B2.1"]
    assert v.progress == 1                      # artifact_invalid


# --- the evidence-contract self-check ---------------------------------------


def test_check_view_names_the_usual_adapter_mistakes():
    assert check_view(fx.view_from((14, 14, 3.2), (8, 8, 0.9))) == []
    bad = ViewEvidence(final=CameraPose(position=(0, 0, 1), forward=(0, 0, 0)),
                       initial=None)
    problems = check_view(bad)
    assert any("initial camera pose is absent" in p for p in problems)
    assert any("unit forward axis" in p for p in problems)
    assert any("source citation" in p for p in problems)


# --- the boundary and the assets --------------------------------------------

# Tokens naming one simulator's internals. Same list the sibling core guard
# uses; a core containing any of these outside a docstring has leaked back
# across the line the split exists to draw.
_SIM_TOKENS = ("urdfrobot", "husky.urdf", ".wbt", "newton", "sidecar",
               "physicsbackend", "gettypename", "boundingobject", "proto",
               "sdf", "usd", "mjcf", "rclpy", "harness", "recorder")


def _code_strings_and_names(path):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            body = getattr(node, "body", None) or []
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docstrings.add(id(body[0].value))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docstrings:
                out.append(node.value)
        elif isinstance(node, ast.Name):
            out.append(node.id)
        elif isinstance(node, ast.Attribute):
            out.append(node.attr)
    return out


def test_the_core_contains_no_simulator_specific_vocabulary():
    for text in _code_strings_and_names(b2_core.__file__):
        low = text.lower()
        for token in _SIM_TOKENS:
            assert token not in low, (
                "b2_core mentions %r in code (not a docstring): %r"
                % (token, text))


def _world_text():
    return WORLD.read_text(encoding="utf-8")


def _solid_translations():
    """``name -> (x, y, z)`` for every named Solid in the shipped scene."""
    text = _world_text()
    out = {}
    for m in re.finditer(
            r"translation\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\n"
            r"\s*name\s+\"([^\"]+)\"", text):
        out[m.group(4)] = tuple(float(m.group(i)) for i in (1, 2, 3))
    return out


def test_the_fixture_scene_mirrors_the_shipped_scene():
    """The fixtures duplicate the world's numbers; this is the drift alarm."""
    shipped = _solid_translations()
    assert set(shipped) == set(fx.SCENE), shipped
    for name, (pos, _half) in fx.SCENE.items():
        assert shipped[name] == pytest.approx(pos), name


def test_the_shipped_world_names_the_subject_and_the_distractors():
    named = set(_solid_translations())
    assert b2_core.SUBJECT_NAME in named
    assert set(b2_core.DISTRACTOR_NAMES) <= named
    assert b2_core.SUBJECT_NAME not in b2_core.DISTRACTOR_NAMES


def _axis_angle_forward(axis_angle):
    """Column 0 of the rotation matrix -- the ``+X`` forward axis, in world."""
    ax, ay, az, angle = axis_angle
    n = math.sqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax / n, ay / n, az / n
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (t * ax * ax + c, t * ax * ay + s * az, t * ax * az - s * ay)


def test_the_shipped_camera_points_away_from_the_subject():
    """The task is unpassable by doing nothing only if this holds."""
    text = _world_text()
    block = re.search(r"(?ms)^Viewpoint\s*\{.*?^\}", text).group(0)
    orient = [float(x) for x in re.search(
        r"orientation\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)", block).groups()]
    pos = tuple(float(x) for x in re.search(
        r"position\s+(\S+)\s+(\S+)\s+(\S+)", block).groups())
    fov = float(re.search(r"fieldOfView\s+(\S+)", block).group(1))

    cam = CameraPose(position=pos, forward=_axis_angle_forward(orient),
                     up=(0.0, 0.0, 1.0), fov_h_rad=fov, aspect=fx.ASPECT,
                     source="the shipped scene")
    subject = fx.scene_inventory().by_name(b2_core.SUBJECT_NAME)
    g = frame_geometry(cam, bounding_sphere(subject)[0])
    assert g["offaxis_deg"] > 90.0, g
    assert g["clearance_deg"] < 0.0

    # ...and the fixtures' idea of the shipped camera agrees with the file's.
    mirrored = fx.initial_camera()
    assert pos == pytest.approx(fx.INITIAL_POSITION)
    assert fov == pytest.approx(fx.FOV_RAD)
    for a, b in zip(cam.forward, mirrored.forward):
        assert a == pytest.approx(b, abs=1e-6)


def test_the_task_metadata_matches_the_core_constants():
    meta = json.loads((TASK_DIR / "meta.json").read_text("utf-8"))
    assert meta["id"] == b2_core.TASK
    assert meta["grader"] == "agentbench.graders.b2"
    assert meta["tier"] == "inspect"
    # SPEC 2.4: B2 sits under the ceiling, so the full 3x-par rule applies.
    assert meta["par_s"] == 240 and meta["timeout_s"] == 3 * 240
    assert meta["timeout_s"] <= tasks_mod.TASK_HARD_CEILING_S
    assert meta["budget"]["truncated_by_ceiling"] is False
    assert meta["subject"] == b2_core.SUBJECT_NAME
    assert tuple(meta["distractors"]) == b2_core.DISTRACTOR_NAMES
    for key, value in {
            "MARGIN_DEG": b2_core.MARGIN_DEG,
            "MIN_ANGULAR_SIZE_DEG": b2_core.MIN_ANGULAR_SIZE_DEG,
            "TIE_DEG": b2_core.TIE_DEG,
            "MIN_AIM_CHANGE_DEG": b2_core.MIN_AIM_CHANGE_DEG,
            "MIN_MOVE_M": b2_core.MIN_MOVE_M,
            "ANGLE_TOL_DEG": b2_core.ANGLE_TOL_DEG,
            "DISTANCE_TOL_M": b2_core.DISTANCE_TOL_M}.items():
        assert meta["constants"][key] == pytest.approx(value), key


def test_the_prompt_is_the_spec_sentence_verbatim():
    prompt = (TASK_DIR / "prompt.txt").read_text("utf-8").strip()
    assert prompt == ("Point the camera at the red cylinder and prove to me "
                      "it is in frame.")


def test_the_world_uses_local_assets_only():
    for m in re.finditer(r'EXTERNPROTO\s+"([^"]+)"', _world_text()):
        assert not m.group(1).lower().startswith("http"), m.group(1)


def test_the_fixture_registry_is_registry_shaped():
    for key, entry in fx.REGISTRY.items():
        task, name = key
        assert task == b2_core.TASK
        assert callable(entry["fn"]), name
        assert set(entry) == {"fn", "expect_pass", "expect_failures"}
        if name == "oracle":
            assert entry["expect_pass"] is True
        else:
            assert entry["expect_pass"] is False
            assert entry["expect_failures"] == fx.RED_MAP[name]
    assert set(fx.expect_failures) == set(fx.RED_MAP)


def test_the_scripted_agents_aim_the_camera_they_claim_to(tmp_path):
    """The live-lane half of each fixture really rewrites the scene."""
    world = tmp_path / "frame_the_cylinder.wbt"
    world.write_text(_world_text(), encoding="utf-8")
    assert fx.aim_camera(world, (14.0, 14.0, 3.2), (8.0, 8.0, 0.9))
    block = re.search(r"(?ms)^Viewpoint\s*\{.*?^\}",
                      world.read_text(encoding="utf-8")).group(0)
    orient = [float(x) for x in re.search(
        r"orientation\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)", block).groups()]
    forward = _axis_angle_forward(orient)
    want = fx.camera_at((14.0, 14.0, 3.2), (8.0, 8.0, 0.9)).forward
    for a, b in zip(forward, want):
        assert a == pytest.approx(b, abs=1e-6)
    assert "position 14.000 14.000 3.200" in block
