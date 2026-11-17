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

"""Tests for the viewpoint-framing validator.

The check answers "does this world open looking at its subject?" — see
``docs/developer/viewpoint-convention.md``. These tests pin the pieces that
have historically been got wrong: the ``#``/string-safe brace scan, PROTO
base-node resolution (a robot declared as ``Husky { }`` is still a robot),
nested-parent transform accumulation, and the VRML larger-dimension
``fieldOfView`` semantics.
"""

from __future__ import annotations

import math

import pytest

from omniworld.validation import _wbt_scan as _scan
from omniworld.validation import _wbt_tree as _tree
from omniworld.validation import validate
from omniworld.validation._wbt_tree import ProtoIndex, parse_wbt
from omniworld.validation.viewpoint import (
    analyze_viewpoint,
    check_viewpoint,
    guess_subject_class,
    is_exempt,
)
from omniworld.viewpoint import (
    SUBJECT_PRESETS,
    format_orientation,
    format_position,
    hero_view,
)

HEADER = '#VRML_SIM R2025a utf8\n\nWorldInfo {\n  basicTimeStep 16\n}\n'


def _viewpoint(position, orientation, extra: str = "") -> str:
    return (
        "Viewpoint {\n"
        f"  orientation {orientation}\n"
        f"  position {position}\n"
        f"{extra}"
        "}\n"
    )


def _framed_viewpoint(center, radius) -> str:
    eye, orient = hero_view(center, radius)
    return _viewpoint(format_position(eye), format_orientation(orient))


def _urdf_robot(name="husky", translation="0 0 0.2", def_name="HUSKY") -> str:
    return (
        f"DEF {def_name} URDFRobot {{\n"
        f'  url "../../robots/clearpath/husky_description/urdf/husky.urdf"\n'
        f"  translation {translation}\n"
        f'  name "{name}"\n'
        f'  controller "omnilink_mobile_bridge"\n'
        "}\n"
    )


def _write(tmp_path, name, body):
    p = tmp_path / name
    p.write_text(body, encoding="utf-8")
    return p


# --------------------------------------------------------------------------
# Fixture: well-framed
# --------------------------------------------------------------------------


def test_well_framed_world_is_ok(tmp_path):
    body = HEADER + _framed_viewpoint((0, 0, 0.45), 1.2) + _urdf_robot()
    w = _write(tmp_path, "good.wbt", body)

    v = analyze_viewpoint(w)
    assert v.status == "ok", v.reason
    assert v.subject_kind == "robot"
    assert v.n_subjects == 1
    assert v.n_in_frame == 1
    assert v.angular_miss_deg == pytest.approx(0.0)
    assert 0.5 < v.fill_frac < 1.2


@pytest.mark.parametrize("urdf,klass", [
    ("ur5e.urdf", "arm"),
    ("g1_23dof.urdf", "humanoid"),
    ("omniquad.urdf", "quadruped"),
    ("husky.urdf", "mobile"),
])
def test_generated_framing_round_trips_for_every_preset(tmp_path, urdf, klass):
    """Anything ``omniworld.viewpoint`` emits must pass its own checker."""
    preset = SUBJECT_PRESETS[klass]
    body = (HEADER
            + _framed_viewpoint((3.0, -2.0, preset["look_z"]), preset["radius"])
            + f'DEF BOT URDFRobot {{\n  url "../urdf/{urdf}"\n'
              '  translation 3 -2 0\n  name "bot"\n}\n')
    v = analyze_viewpoint(_write(tmp_path, f"gen_{klass}.wbt", body))
    assert v.subjects[0].klass == klass
    assert v.status == "ok", (klass, v.reason)


# --------------------------------------------------------------------------
# Fixture: out of frame (the top-down-orientation copy-paste bug)
# --------------------------------------------------------------------------


def test_topdown_orientation_at_an_oblique_eye_is_broken(tmp_path):
    """The exact 160-world bug: a straight-down orientation at an oblique eye.

    ``-0.5773 0.5773 0.5773 2.0944`` decodes to forward ``(0, 0, -1)``. Correct
    for an overhead camera; with ``position`` moved off to the side it stares at
    empty floor metres from the robot.
    """
    body = (HEADER
            + _viewpoint("14 -14 9", "-0.5773 0.5773 0.5773 2.0944")
            + _urdf_robot())
    w = _write(tmp_path, "bad.wbt", body)

    v = analyze_viewpoint(w)
    assert v.status == "broken"
    assert v.subject_kind == "robot"
    assert v.n_in_frame == 0
    assert v.angular_miss_deg > 10.0
    assert "out of frame" in v.reason
    # The suggestion must be a runnable set_viewpoint.py command naming the DEF.
    assert "set_viewpoint.py" in v.suggestion
    assert "--subject HUSKY" in v.suggestion
    assert "--class mobile" in v.suggestion


def test_same_orientation_directly_overhead_is_fine(tmp_path):
    """The orientation itself is not the bug — the mismatched position is."""
    body = (HEADER
            + _viewpoint("0 0 6", "-0.5773 0.5773 0.5773 2.0944")
            + _urdf_robot())
    v = analyze_viewpoint(_write(tmp_path, "overhead.wbt", body))
    assert v.status in ("ok", "borderline"), v.reason
    assert v.angular_miss_deg == pytest.approx(0.0)


def test_camera_facing_away_is_broken(tmp_path):
    """A subject behind the camera must not sneak through the atan2 fold."""
    body = HEADER + _viewpoint("0 -4 1", "0 0 1 3.14159") + _urdf_robot()
    v = analyze_viewpoint(_write(tmp_path, "away.wbt", body))
    assert v.status == "broken"
    assert v.angular_miss_deg > 45.0


def test_distant_robot_is_a_speck(tmp_path):
    body = HEADER + _framed_viewpoint((0, 0, 0.45), 400.0) + _urdf_robot()
    v = analyze_viewpoint(_write(tmp_path, "speck.wbt", body))
    assert v.status == "broken"
    assert "speck" in v.reason


# --------------------------------------------------------------------------
# Fixture: no Viewpoint / no robot
# --------------------------------------------------------------------------


def test_missing_viewpoint_node_is_broken(tmp_path):
    v = analyze_viewpoint(_write(tmp_path, "novp.wbt", HEADER + _urdf_robot()))
    assert v.status == "broken"
    assert v.has_viewpoint is False
    assert "no Viewpoint node" in v.reason
    assert v.suggestion


def test_viewpoint_without_position_uses_engine_defaults(tmp_path):
    """A field-less Viewpoint is not an error: the engine substitutes
    ``position -10 0 0`` / identity orientation from Viewpoint.wrl."""
    body = HEADER + "Viewpoint {\n  near 0.05\n}\n" + _urdf_robot()
    v = analyze_viewpoint(_write(tmp_path, "bare.wbt", body))
    assert v.has_viewpoint is True
    assert v.eye == (-10.0, 0.0, 0.0)
    # Looking down +X from -10: the robot at the origin is actually in frame.
    assert v.status in ("ok", "borderline"), v.reason


def test_world_with_no_robot_falls_back_to_the_scene(tmp_path):
    body = (HEADER + _framed_viewpoint((0, 0, 0.5), 6.0)
            + 'Solid {\n  translation 2 2 0\n  children [ Shape { geometry Box { size 1 1 1 } } ]\n}\n')
    v = analyze_viewpoint(_write(tmp_path, "noRobot.wbt", body))
    assert v.subject_kind == "scene"
    assert v.n_subjects == 0
    # Scene framing is advisory: it can never fail the gate.
    assert check_viewpoint(v.world, v.world.read_text(encoding="utf-8")).passed


def test_empty_world_passes(tmp_path):
    v = analyze_viewpoint(_write(tmp_path, "empty.wbt", HEADER))
    assert v.subject_kind == "none"
    assert v.status == "ok"


# --------------------------------------------------------------------------
# Fixture: PROTO-instanced robot
# --------------------------------------------------------------------------


PROTO_SRC = """#VRML_SIM R2025a utf8
# A robot behind a PROTO — the checker must resolve the base node.
PROTO DemoBot [
  field SFVec3f    translation 0 0 0.2
  field SFRotation rotation    0 0 1 0
  field SFString   name        "demobot"
]
{
  %<
    // template block with unbalanced-looking braces: { { {
    const x = 1;
  >%
  Robot {
    translation IS translation
    rotation IS rotation
    name IS name
    controller "demo"
    children [
      Shape { geometry Box { size 0.6 0.4 0.3 } }
    ]
  }
}
"""

DERIVED_PROTO_SRC = """#VRML_SIM R2025a utf8
PROTO FancyBot [
  field SFVec3f translation 0 0 0.2
]
{
  DemoBot {
    translation IS translation
  }
}
"""


def _proto_tree(tmp_path):
    (tmp_path / "DemoBot.proto").write_text(PROTO_SRC, encoding="utf-8")
    (tmp_path / "FancyBot.proto").write_text(DERIVED_PROTO_SRC, encoding="utf-8")
    return tmp_path


def test_proto_index_resolves_base_and_derived(tmp_path):
    idx = ProtoIndex([_proto_tree(tmp_path)])
    assert idx.base_of("DemoBot") == "Robot"
    assert idx.resolve("DemoBot") == "Robot"
    assert idx.base_of("FancyBot") == "DemoBot"
    assert idx.resolve("FancyBot") == "Robot"


def test_proto_instanced_robot_is_found(tmp_path):
    root = _proto_tree(tmp_path)
    body = (HEADER + _viewpoint("14 -14 9", "-0.5773 0.5773 0.5773 2.0944")
            + 'DemoBot {\n  translation 0 0 0.2\n  name "demobot"\n}\n')
    w = _write(root, "proto.wbt", body)
    v = analyze_viewpoint(w, repo_root=root)
    assert v.subject_kind == "robot", "PROTO base node was not resolved"
    assert v.n_subjects == 1
    assert v.status == "broken"


def test_derived_proto_instanced_robot_is_found(tmp_path):
    root = _proto_tree(tmp_path)
    body = (HEADER + _framed_viewpoint((0, 0, 0.2), 1.0)
            + "FancyBot {\n  translation 0 0 0.2\n}\n")
    v = analyze_viewpoint(_write(root, "derived.wbt", body), repo_root=root)
    assert v.subject_kind == "robot"
    assert v.n_subjects == 1


# --------------------------------------------------------------------------
# Fixture: nested / parented robot
# --------------------------------------------------------------------------


NESTED = """DEF RIG Pose {
  translation 10 -4 0
  rotation 0 0 1 1.5707963
  children [
    Pose {
      translation 12 0 0.5
      children [
        DEF ARM URDFRobot {
          url "../../robots/universal_robots/ur_description/urdf/ur5e.urdf"
          translation 0 0 0
          name "ur5e"
        }
      ]
    }
  ]
}
"""


def test_nested_robot_position_accumulates_parent_transforms(tmp_path):
    # RIG at (10,-4,0) yawed +90 deg; child offset (12,0,0.5) rotates to
    # (0,12,0.5) -> the robot lands at (10, 8, 0.5).
    body = HEADER + _framed_viewpoint((10.0, 8.0, 0.95), 1.0) + NESTED
    v = analyze_viewpoint(_write(tmp_path, "nested.wbt", body))
    assert v.n_subjects == 1
    sx, sy, sz = v.subjects[0].center
    assert sx == pytest.approx(10.0, abs=1e-6)
    assert sy == pytest.approx(8.0, abs=1e-6)
    assert sz == pytest.approx(0.95, abs=1e-6)   # +0.45 arm look-height
    assert v.status == "ok", v.reason


def test_nested_robot_is_broken_when_the_camera_frames_the_parent_origin(tmp_path):
    body = HEADER + _framed_viewpoint((10.0, -4.0, 0.45), 1.0) + NESTED
    v = analyze_viewpoint(_write(tmp_path, "nested_bad.wbt", body))
    assert v.status == "broken"


# --------------------------------------------------------------------------
# Scene-helper exclusions
# --------------------------------------------------------------------------


def test_sun_marker_is_not_a_subject(tmp_path):
    body = (HEADER + _framed_viewpoint((0, 0, 0.5), 4.0)
            + "DEF SUN_MARKER OmniSimSunMarker { }\n")
    v = analyze_viewpoint(_write(tmp_path, "marker.wbt", body))
    assert v.subject_kind != "robot"


def test_bodiless_supervisor_shim_is_not_a_subject(tmp_path):
    body = (HEADER + _framed_viewpoint((0, 0, 0.5), 4.0)
            + 'Robot {\n  name "sup"\n  controller "warehouse_supervisor"\n'
              "  supervisor TRUE\n}\n")
    v = analyze_viewpoint(_write(tmp_path, "shim.wbt", body))
    assert v.subject_kind != "robot"


def test_passive_wrapper_robot_is_not_a_subject(tmp_path):
    """``controller "<none>"`` marks a fixture host (city traffic lights)."""
    body = (HEADER + _framed_viewpoint((0, 0, 0.5), 4.0)
            + 'DEF LIGHTS Robot {\n  name "lights"\n  controller "<none>"\n'
              "  children [ Shape { geometry Box { size 0.2 0.2 3 } } ]\n}\n")
    v = analyze_viewpoint(_write(tmp_path, "wrapper.wbt", body))
    assert v.subject_kind != "robot"


# --------------------------------------------------------------------------
# Lexical robustness
# --------------------------------------------------------------------------


def test_comments_and_strings_do_not_confuse_the_scan(tmp_path):
    body = (
        HEADER
        + "# a comment with a brace { and a quote \"\n"
        + _framed_viewpoint((0, 0, 0.45), 1.2)
        + "DEF HUSKY URDFRobot {\n"
        + '  url "../x.urdf"   # trailing comment }\n'
        + "  translation 0 0 0.2\n"
        + '  name "husky"\n'
        + '  customData "a } brace # hash inside a string"\n'
        + "}\n"
    )
    v = analyze_viewpoint(_write(tmp_path, "lex.wbt", body))
    assert v.n_subjects == 1
    assert v.status == "ok", v.reason


def test_parser_handles_multiline_mf_fields():
    nodes = parse_wbt(
        'WorldInfo {\n  info [\n    "line one"\n    "line { two"\n  ]\n'
        "  basicTimeStep 16\n}\nViewpoint {\n  position 1 2 3\n}\n"
    )
    assert [n.type_name for n in nodes] == ["WorldInfo", "Viewpoint"]
    assert nodes[1].floats("position", 3) == (1.0, 2.0, 3.0)


# --------------------------------------------------------------------------
# FOV semantics
# --------------------------------------------------------------------------


def test_fieldofview_is_the_larger_dimension(tmp_path):
    """A subject that fits horizontally but not vertically must be caught.

    ``fieldOfView`` is the angle on the LARGER viewport dimension (VRML), so on
    16:9 the binding axis is vertical. Offsetting the subject vertically by more
    than the *vertical* half-FOV — but less than the horizontal one — is only a
    miss if the tight axis is used.
    """
    fov = math.pi / 4
    half_h = fov / 2
    half_v = math.atan(math.tan(half_h) / (16 / 9))
    assert half_v < half_h
    # Camera at the origin looking down +X; robot 10 m out, raised so its
    # elevation sits between the vertical and horizontal half-angles.
    elev = (half_h + half_v) / 2
    z = 100.0 * math.tan(elev)
    body = (HEADER + _viewpoint("0 0 0", "0 0 1 0")
            + _urdf_robot(translation=f"100 0 {z:.4f}"))
    v = analyze_viewpoint(_write(tmp_path, "fov.wbt", body), aspect=16 / 9)
    assert v.status == "broken", v.reason
    # On a 1:1 viewport the same world frames fine (fov applies to both axes).
    v11 = analyze_viewpoint(_write(tmp_path, "fov.wbt", body), aspect=1.0)
    assert v11.angular_miss_deg == pytest.approx(0.0)


# --------------------------------------------------------------------------
# Class guessing / exemptions / harness wiring
# --------------------------------------------------------------------------


@pytest.mark.parametrize("hint,expected", [
    ("g1", "humanoid"),
    ("../robots/unitree/g1/urdf/g1_23dof.urdf", "humanoid"),
    ("omniquad", "quadruped"),
    ("go2_walk", "quadruped"),
    ("HUSKY", "mobile"),
    ("ur5e", "arm"),
    ("../robots/universal_robots/ur_description/urdf/ur5e.urdf", "arm"),
    ("something_unknown", "mobile"),
])
def test_guess_subject_class(hint, expected):
    assert guess_subject_class(hint) == expected


def test_exemptions_are_path_matched():
    assert is_exempt("distribution/generated_worlds/earth_night.wbt")
    assert is_exempt("projects/samples/demos/worlds/rendering/camera_wgpu_smoke.omniworld")
    assert is_exempt("projects/samples/demos/worlds/chat/omnilink_husky.omniworld") is None


def test_check_is_wired_into_omniworld_validate(tmp_path):
    body = (HEADER + _viewpoint("14 -14 9", "-0.5773 0.5773 0.5773 2.0944")
            + _urdf_robot())
    report = validate(_write(tmp_path, "wired.wbt", body))
    names = [r.name for r in report.results]
    assert "viewpoint_framing" in names
    result = next(r for r in report.results if r.name == "viewpoint_framing")
    assert result.passed is False
    assert "set_viewpoint.py" in result.detail
    assert report.ok is False


# --------------------------------------------------------------------------
# The per-text memos are PURE caches (2026-09-02): same text -> same tree,
# same placements, same report. Nothing here may change a verdict.
# --------------------------------------------------------------------------


def _memo_world(tmp_path, name="memo.wbt"):
    body = (HEADER + _framed_viewpoint((0, 0, 0.45), 1.2) + _urdf_robot()
            + "DEF CRATE Transform {\n  translation 3 0 0.5\n  children [\n"
            "    Shape { geometry Box { size 1 1 1 } }\n  ]\n}\n")
    return _write(tmp_path, name, body)


def test_validate_report_is_identical_cached_and_uncached(tmp_path):
    """The second ``validate()`` of one text answers from the parse and
    placement memos; its report must be exactly what the cold path produced,
    and a byte-different text of the same length must never be served."""
    path = _memo_world(tmp_path)
    _tree._PARSE_MEMO.clear()
    _scan._SCAN_MEMO.clear()

    cold = validate(path)
    assert (_tree._PARSE_MEMO.misses, _tree._PARSE_MEMO.hits) == (1, 0)
    # spawn + overlap scan the same text: one miss, then a hit.
    assert (_scan._SCAN_MEMO.misses, _scan._SCAN_MEMO.hits) == (1, 1)

    warm = validate(path)
    assert (_tree._PARSE_MEMO.misses, _tree._PARSE_MEMO.hits) == (1, 1)
    assert (_scan._SCAN_MEMO.misses, _scan._SCAN_MEMO.hits) == (1, 3)
    assert warm.results == cold.results
    assert warm.format() == cold.format()
    assert cold.ok, cold.format()

    text = path.read_text(encoding="utf-8")
    assert _tree.parse_wbt_cached(text) == parse_wbt(text)
    moved = text.replace("translation 3 0 0.5", "translation 3 0 0.6")
    assert len(moved) == len(text) and moved != text
    assert _tree.parse_wbt_cached(moved) == parse_wbt(moved)
    assert _tree._PARSE_MEMO.misses == 2
    assert _tree.parse_wbt_cached(moved)[-1].floats("translation", 3) == (3.0, 0.0, 0.6)
    assert _tree.parse_wbt_cached(text)[-1].floats("translation", 3) == (3.0, 0.0, 0.5)


def test_text_memo_is_bounded_lru():
    memo = _tree.TextMemo(max_entries=2)
    memo.put("a", 1)
    memo.put("b", 2)
    memo.put("c", 3)
    assert len(memo) == 2
    assert memo.get("a") is _tree.MISS
    assert memo.get("b") == 2          # refreshes b ...
    memo.put("d", 4)                   # ... so c is the one evicted
    assert memo.get("c") is _tree.MISS
    assert memo.get("b") == 2 and memo.get("d") == 4


@pytest.mark.parametrize("text, expected", [
    ("a # c\nb", ["a", "b"]),                        # comment runs to end of line
    ('"a # b" c', ['"a # b"', "c"]),                 # a hash inside a string is not a comment
    ("12%<x>%34", ["12", "34"]),                     # a blanked template block never fuses its neighbours
    ("DEF %<= n >% Pose {", ["DEF", "Pose", "{"]),   # the template-DEF shape (AdvertisingBoard.proto)
    ('"%<" x', ['"%<"', "x"]),                       # a template start inside a string
    ('%< x "y"', ["%", "<", "x", '"y"']),            # an unterminated template is ordinary text
    ('x "unterminated', ["x", '"unterminated']),
    ("-1.5e3 .5 +2 - .", ["-1.5e3", ".5", "+2", "-", "."]),
])
def test_tokenizer_lexical_rules(text, expected):
    assert _tree._tokenize(text) == expected


# --------------------------------------------------------------------------
# The retrofit tool (scripts/dev/set_viewpoint.py)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def set_viewpoint():
    import importlib.util
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "set_viewpoint", root / "scripts" / "dev" / "set_viewpoint.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ORIENT = (0.1, 0.2, 0.3, 1.5)
EYE = (1.0, 2.0, 3.0)


def test_rewrite_block_handles_the_single_line_form(set_viewpoint):
    """23 worlds write the whole Viewpoint on one line.

    The old line-oriented rewriter matched neither ``orientation`` nor
    ``position`` on such a line and then looked for a line *ending* in ``{`` to
    insert into — so it silently returned the block unchanged and reported
    "already framed" while leaving the camera broken.
    """
    block = ("Viewpoint { orientation -0.5573 0.5773 0.5973 2.10 "
             "position 1.95 -1.75 1.5 }")
    out = set_viewpoint.rewrite_block(block, ORIENT, EYE, None)
    assert out != block
    assert "orientation 0.100000 0.200000 0.300000 1.500000" in out
    assert "position 1.000 2.000 3.000" in out


def test_rewrite_block_preserves_other_fields_and_comments(set_viewpoint):
    block = ('Viewpoint {\n'
             '  orientation 0 0 1 0  # keep me\n'
             '  position 1 2 3\n'
             '  follow "g1"\n'
             '  followType "Tracking Shot"\n'
             '  bloomThreshold 21\n'
             '}')
    out = set_viewpoint.rewrite_block(block, ORIENT, EYE, None)
    assert '# keep me' in out
    assert 'follow "g1"' in out
    assert 'followType "Tracking Shot"' in out
    assert "bloomThreshold 21" in out
    assert "orientation 0.100000 0.200000 0.300000 1.500000" in out


def test_rewrite_block_inserts_missing_fields(set_viewpoint):
    out = set_viewpoint.rewrite_block("Viewpoint {\n  near 0.05\n}",
                                      ORIENT, EYE, None)
    assert "orientation " in out and "position " in out and "near 0.05" in out


def test_auto_target_resolves_the_subject(set_viewpoint, tmp_path):
    body = (HEADER + _viewpoint("14 -14 9", "-0.5773 0.5773 0.5773 2.0944")
            + _urdf_robot(translation="2 3 0.2"))
    w = _write(tmp_path, "auto.wbt", body)
    center, radius, mode, label = set_viewpoint.auto_target(
        w, w.read_text(encoding="utf-8"))
    assert mode == "hero"
    assert "HUSKY" in label and "mobile" in label
    assert center[0] == pytest.approx(2.0)
    assert center[1] == pytest.approx(3.0)


def test_auto_then_check_makes_a_broken_world_ok(set_viewpoint, tmp_path):
    """End-to-end: the tool's --auto output must satisfy the checker."""
    body = (HEADER + _viewpoint("14 -14 9", "-0.5773 0.5773 0.5773 2.0944")
            + _urdf_robot(translation="2 3 0.2"))
    w = _write(tmp_path, "roundtrip.wbt", body)
    assert analyze_viewpoint(w).status == "broken"
    set_viewpoint.apply_to_world(
        w, mode=None, center=None, radius=None, subject=None, klass=None,
        fov=None, auto=True, dry_run=False)
    assert analyze_viewpoint(w).status == "ok"


def test_check_passes_on_a_framed_world(tmp_path):
    body = HEADER + _framed_viewpoint((0, 0, 0.45), 1.2) + _urdf_robot()
    report = validate(_write(tmp_path, "wired_ok.wbt", body))
    result = next(r for r in report.results if r.name == "viewpoint_framing")
    assert result.passed is True
