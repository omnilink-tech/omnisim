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

"""Unit tests for the harness's spatial layer.

Two things are load-bearing here and both are asserted rather than assumed:

1. **There is exactly one framing implementation in the tree.** ``spatial``
   loads ``src/python/omniworld/viewpoint.py`` itself, so a harness-framed
   camera and a generator-baked ``.wbt`` camera cannot drift apart. The first
   test proves it is really that file, and the parity tests prove the numbers.
2. **Projection matches the engine.** ``spatial.project`` reproduces
   ``OmViewpoint::eyeToPixels`` (top-left pixel origin, +Y-left / +Z-up camera
   frame, ``fieldOfView`` on the LARGER viewport axis).
"""

from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))
sys.path.insert(0, str(REPO_ROOT / "projects" / "default" / "controllers" / "harness_supervisor"))

import geometry  # noqa: E402
import spatial  # noqa: E402
from omnisim_harness import (  # noqa: E402
    bounds_union,
    compute_look_at_orientation,
    png_size,
    screen_bbox,
)


# --- single-source-of-truth ---------------------------------------------------

def test_spatial_loads_the_real_omniworld_reference():
    """The framing math must come from the canonical file, not a copy."""
    assert Path(spatial.reference.__file__).resolve() == (
        REPO_ROOT / "src" / "python" / "omniworld" / "viewpoint.py"
    )
    assert spatial.look_at is spatial.reference.look_at
    assert spatial.frame_distance is spatial.reference._frame_distance


def test_look_at_parity_with_harness_copy():
    """The harness's own stdlib copy and the reference must agree bit-for-bit."""
    for position, target in [
        ([5, 5, 5], [0, 0, 0]),
        ([-10, -16, 10], [0, 0, 1]),
        ([2, 2, -3], [0, 0, 0]),
        ([0, 0, 10], [0, 0, 0]),
        ([-10, 0, 2], [0, 0, 2]),
    ]:
        assert compute_look_at_orientation(position, target) == pytest.approx(
            list(spatial.look_at(position, target)), abs=1e-12
        )


def test_hero_mode_reproduces_omniworld_hero_view():
    """`mode="hero"` must be the same shot the world generators bake."""
    center, radius = (1.0, -2.0, 0.35), 1.4
    eye, orientation, _meta = spatial.frame_pose(
        center, radius, mode="hero", aspect=16 / 9
    )
    ref_eye, ref_orientation = spatial.hero_view(center, radius, aspect=16 / 9)
    assert eye == pytest.approx(list(ref_eye), abs=1e-12)
    assert orientation == pytest.approx(list(ref_orientation), abs=1e-12)


def test_top_down_mode_reproduces_omniworld_top_down_view():
    center, radius = (0.0, 0.0, 0.0), 6.0
    eye, orientation, _meta = spatial.frame_pose(
        center, radius, mode="top_down", aspect=16 / 9, margin=1.15
    )
    ref_eye, ref_orientation = spatial.top_down_view(center, radius, aspect=16 / 9)
    assert eye == pytest.approx(list(ref_eye), abs=1e-12)
    assert orientation == pytest.approx(list(ref_orientation), abs=1e-12)


# --- field of view ------------------------------------------------------------

def test_fov_is_the_larger_axis_on_landscape():
    """VRML semantics (OmViewpoint::updateFieldOfViewY): on a landscape
    viewport `fieldOfView` is the HORIZONTAL angle and the vertical one is
    derived — narrower. Getting this backwards is what overflows tall subjects.
    """
    axes = spatial.fov_axes(math.pi / 4, 16 / 9)
    assert axes["fov_h_deg"] == pytest.approx(45.0, abs=1e-6)
    assert axes["fov_v_deg"] < axes["fov_h_deg"]
    assert axes["tan_half_v"] == pytest.approx(math.tan(math.pi / 8) / (16 / 9))


def test_fov_is_the_larger_axis_on_portrait():
    axes = spatial.fov_axes(math.pi / 4, 9 / 16)
    assert axes["fov_v_deg"] == pytest.approx(45.0, abs=1e-6)
    assert axes["fov_h_deg"] < axes["fov_v_deg"]


def test_fov_square_viewport_is_symmetric():
    axes = spatial.fov_axes(math.pi / 4, 1.0)
    assert axes["fov_h_deg"] == pytest.approx(axes["fov_v_deg"])


# --- projection ---------------------------------------------------------------

EYE = [0.0, 0.0, 0.0]
LOOK_X = list(spatial.look_at((0, 0, 0), (1, 0, 0)))  # identity: +X forward


def test_projection_centre_is_screen_centre():
    proj = spatial.project((10, 0, 0), EYE, LOOK_X, math.pi / 4, 16 / 9, 1920, 1080)
    assert proj["ndc_x"] == pytest.approx(0.5)
    assert proj["ndc_y"] == pytest.approx(0.5)
    assert proj["pixel"] == pytest.approx([960.0, 540.0])
    assert proj["in_frame"]


def test_projection_left_and_up_land_where_the_engine_puts_them():
    """+Y is LEFT and +Z is UP in the camera frame; the pixel origin is the
    TOP-left (OmViewpoint::eyeToPixels), so 'up' means a SMALLER pixel y.
    """
    left = spatial.project((10, 1, 0), EYE, LOOK_X, math.pi / 4, 16 / 9, 1920, 1080)
    assert left["ndc_x"] < 0.5
    assert left["yaw_deg"] > 0
    assert "to the left" in spatial.offset_hint(left)

    above = spatial.project((10, 0, 1), EYE, LOOK_X, math.pi / 4, 16 / 9, 1920, 1080)
    assert above["ndc_y"] < 0.5
    assert above["pitch_deg"] > 0
    assert " up" in spatial.offset_hint(above)


def test_projection_behind_camera():
    proj = spatial.project((-5, 0, 0), EYE, LOOK_X, math.pi / 4, 16 / 9)
    assert proj["behind_camera"]
    assert proj["in_frame"] is False
    assert proj["pixel"] is None
    assert spatial.offset_hint(proj) == "off-screen: behind the camera"


def test_projection_frame_edges_match_the_half_fov():
    """A point exactly at the horizontal half-FOV lands on the frame edge."""
    axes = spatial.fov_axes(math.pi / 4, 16 / 9)
    y = 10.0 * axes["tan_half_h"]
    edge = spatial.project((10, y, 0), EYE, LOOK_X, math.pi / 4, 16 / 9)
    assert edge["ndc_x"] == pytest.approx(0.0, abs=1e-9)
    z = 10.0 * axes["tan_half_v"]
    top = spatial.project((10, 0, z), EYE, LOOK_X, math.pi / 4, 16 / 9)
    assert top["ndc_y"] == pytest.approx(0.0, abs=1e-9)


def test_offscreen_hint_reports_direction_and_magnitude():
    proj = spatial.project((10, 7, 2), EYE, LOOK_X, math.pi / 4, 16 / 9)
    hint = spatial.offset_hint(proj)
    assert hint.startswith("off-screen:")
    assert "to the left" in hint and "up" in hint


def test_partial_hint_distinguishes_clipped_from_fully_offscreen():
    """An object whose centroid is outside the frame can still be half on
    screen; the hint must not call that 'off-screen'.
    """
    proj = spatial.project((10, 7, 0), EYE, LOOK_X, math.pi / 4, 16 / 9)
    assert proj["in_frame"] is False
    assert spatial.offset_hint(proj, partial=True).startswith("partly in frame")
    assert spatial.offset_hint(proj, partial=False).startswith("off-screen")


# --- framing round-trip -------------------------------------------------------

@pytest.mark.parametrize("aspect", [16 / 9, 4 / 3, 1.0, 9 / 16, 21 / 9])
@pytest.mark.parametrize("mode", ["hero", "top_down", "front", "back", "left", "right"])
@pytest.mark.parametrize("radius", [0.05, 0.6, 5.0, 40.0])
def test_frame_pose_actually_frames_the_subject(mode, aspect, radius):
    center = (3.0, -1.5, 0.75)
    eye, orientation, _meta = spatial.frame_pose(
        center, radius, mode=mode, aspect=aspect
    )
    verification = spatial.framing_verification(
        center, radius, eye, orientation, spatial.DEFAULT_FOV, aspect, 1920, 1080
    )
    assert verification["fits"], verification
    assert verification["center_projection"]["ndc_x"] == pytest.approx(0.5, abs=1e-6)
    assert verification["center_projection"]["ndc_y"] == pytest.approx(0.5, abs=1e-6)


def test_frame_pose_bbox_corners_are_all_on_screen():
    """The stronger check: every corner of the subject's AABB projects inside
    the viewport, not just the centre.
    """
    center = (0.0, 0.0, 0.4)
    half = (0.5, 0.29, 0.2)  # roughly a Husky chassis
    radius = math.sqrt(sum(h * h for h in half))
    eye, orientation, _meta = spatial.frame_pose(center, radius, aspect=16 / 9)
    lo = [center[i] - half[i] for i in range(3)]
    hi = [center[i] + half[i] for i in range(3)]
    box = screen_bbox(lo, hi, eye, orientation, spatial.DEFAULT_FOV, 16 / 9, 1920, 1080)
    assert box["corners_behind_camera"] == 0
    x0, y0, x1, y1 = box["ndc"]
    assert 0.0 < x0 and x1 < 1.0
    assert 0.0 < y0 and y1 < 1.0


def test_subject_relative_front_uses_the_subject_frame():
    """With a yawed subject, `front` follows the subject, not world +X."""
    # 90 deg yaw about +Z: subject's +X points along world +Y.
    yaw = geometry.axis_angle_to_matrix((0, 0, 1, math.pi / 2))
    eye, _o, meta = spatial.frame_pose(
        (0, 0, 0), 1.0, mode="front", aspect=16 / 9, subject_rotation=yaw
    )
    assert meta["direction"] == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
    assert eye[1] > 0 and abs(eye[0]) < 1e-6
    # Without the subject rotation it is the world +X side.
    _eye2, _o2, meta2 = spatial.frame_pose((0, 0, 0), 1.0, mode="front", aspect=16 / 9)
    assert meta2["direction"] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)


def test_unknown_mode_is_rejected_with_the_valid_list():
    with pytest.raises(ValueError) as exc:
        spatial.frame_pose((0, 0, 0), 1.0, mode="cinematic")
    assert "hero" in str(exc.value)


def test_mode_aliases_resolve():
    assert spatial.resolve_mode("topdown") == "top_down"
    assert spatial.resolve_mode("OVERVIEW") == "top_down"
    assert spatial.resolve_mode(None) == "hero"


# --- orbiting -----------------------------------------------------------------

def _pose_at(center, offset):
    eye = [center[i] + offset[i] for i in range(3)]
    return eye, list(spatial.look_at(eye, center))


def test_orbit_azimuth_preserves_radius_and_keeps_the_subject_centred():
    center = (1.0, 2.0, 0.5)
    eye, orientation = _pose_at(center, (5.0, 0.0, 3.0))
    new_eye, new_orientation, meta = spatial.orbit_pose(
        eye, orientation, center, azimuth_deg=90.0
    )
    assert meta["radius_after"] == pytest.approx(meta["radius_before"])
    assert meta["azimuth_deg_after"] - meta["azimuth_deg_before"] == pytest.approx(90.0)
    proj = spatial.project(center, new_eye, new_orientation,
                           spatial.DEFAULT_FOV, 16 / 9)
    assert proj["ndc_x"] == pytest.approx(0.5, abs=1e-6)
    assert proj["ndc_y"] == pytest.approx(0.5, abs=1e-6)


def test_orbit_dolly_multiplies_distance():
    center = (0.0, 0.0, 0.0)
    eye, orientation = _pose_at(center, (4.0, 0.0, 0.0))
    new_eye, _o, meta = spatial.orbit_pose(eye, orientation, center, dolly=1.5)
    assert meta["radius_after"] == pytest.approx(6.0)
    assert spatial.length(spatial.sub(new_eye, center)) == pytest.approx(6.0)


def test_orbit_elevation_is_clamped_below_the_pole():
    center = (0.0, 0.0, 0.0)
    eye, orientation = _pose_at(center, (5.0, 0.0, 0.0))
    _e, _o, meta = spatial.orbit_pose(eye, orientation, center, elevation_deg=200.0)
    assert meta["elevation_deg_after"] == pytest.approx(spatial.MAX_ELEVATION_DEG)
    assert meta["elevation_clamped"] is True


def test_orbit_pan_moves_eye_and_centre_together():
    center = (0.0, 0.0, 0.0)
    eye, orientation = _pose_at(center, (5.0, 0.0, 0.0))
    new_eye, _o, meta = spatial.orbit_pose(
        eye, orientation, center, pan=[2.0, 1.0]
    )
    # Camera sits at +X looking along -X with up +Z, so screen-right is world
    # +Y (right = forward x up) and screen-up is world +Z.
    assert meta["center"] == pytest.approx([0.0, 2.0, 1.0], abs=1e-6)
    assert meta["radius_after"] == pytest.approx(meta["radius_before"])
    assert new_eye == pytest.approx([5.0, 2.0, 1.0], abs=1e-6)


def test_orbit_zero_change_is_a_no_op():
    center = (2.0, -1.0, 0.3)
    eye, orientation = _pose_at(center, (3.0, 2.0, 1.5))
    new_eye, new_orientation, _meta = spatial.orbit_pose(eye, orientation, center)
    assert new_eye == pytest.approx(eye, abs=1e-9)
    assert new_orientation == pytest.approx(orientation, abs=1e-9)


# --- harness helpers ----------------------------------------------------------

def test_png_size_reads_the_ihdr():
    header = (b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IHDR"
              + struct.pack(">II", 1920, 1080) + b"\x08\x02\x00\x00\x00")
    assert png_size(header) == (1920, 1080)
    assert png_size(b"not a png") is None
    assert png_size(b"") is None


def test_bounds_union_merges_boxes_and_propagates_inexactness():
    a = {"bbox_min": [0, 0, 0], "bbox_max": [1, 1, 1], "exact": True}
    b = {"bbox_min": [-1, 2, 0], "bbox_max": [0, 3, 4], "exact": False}
    union = bounds_union([a, b])
    assert union["bbox_min"] == [-1, 0, 0]
    assert union["bbox_max"] == [1, 3, 4]
    assert union["center"] == pytest.approx([0.0, 1.5, 2.0])
    assert union["exact"] is False
    assert bounds_union([]) is None


def test_screen_bbox_flags_near_plane_clipping():
    center = (0.0, 0.0, 0.0)
    eye, orientation = _pose_at(center, (1.0, 0.0, 0.0))
    box = screen_bbox([-2, -2, -2], [2, 2, 2], eye, orientation,
                      spatial.DEFAULT_FOV, 16 / 9, 800, 600)
    assert box["corners_behind_camera"] > 0


# --- supervisor-side geometry (pure parts) ------------------------------------

class _FakeField:
    def __init__(self, value, kind):
        self._value = value
        self._kind = kind

    def getSFFloat(self):
        assert self._kind == "float"
        return self._value

    def getSFInt32(self):
        assert self._kind == "int"
        return self._value

    def getSFVec3f(self):
        assert self._kind == "vec3"
        return list(self._value)

    def getSFVec2f(self):
        assert self._kind == "vec2"
        return list(self._value)

    def getSFString(self):
        assert self._kind == "string"
        return self._value

    def getCount(self):
        return len(self._value)

    def getMFString(self, i):
        return self._value[i]

    def getMFFloat(self, i):
        return self._value[i]


class _FakeNode:
    def __init__(self, type_name, fields):
        self._type = type_name
        self._fields = fields

    def getTypeName(self):
        return self._type

    def getBaseTypeName(self):
        return self._type

    def getField(self, name):
        return self._fields.get(name)


def test_box_bounds_are_exact():
    node = _FakeNode("Box", {"size": _FakeField((2.0, 4.0, 6.0), "vec3")})
    lo, hi = geometry.local_geometry_bounds(node, "Box", [])
    assert lo == (-1.0, -2.0, -3.0)
    assert hi == (1.0, 2.0, 3.0)


def test_cylinder_height_is_on_the_z_axis():
    """OmniSim's Cylinder is Z-axis aligned (post-R2022b).

    `OmCylinder::rescale` still multiplies height by `scale.y()` — a stale
    pre-R2022b leftover. The authoritative code (`scaledHeight()` -> scale.z,
    the ODE geom, `computeFrictionDirection` testing localNormal[2],
    `recomputeBoundingSphere` offsetting caps along z) says Z. Reading the
    stale path made the Husky measure 0.885 m wide instead of 0.685 m, so this
    is pinned with a real URDF wheel's numbers.
    """
    node = _FakeNode("Cylinder", {"radius": _FakeField(0.1651, "float"),
                                  "height": _FakeField(0.1143, "float")})
    lo, hi = geometry.local_geometry_bounds(node, "Cylinder", [])
    assert hi == pytest.approx((0.1651, 0.1651, 0.05715))
    assert lo == pytest.approx((-0.1651, -0.1651, -0.05715))


def test_capsule_includes_its_caps():
    node = _FakeNode("Capsule", {"radius": _FakeField(0.5, "float"),
                                 "height": _FakeField(2.0, "float")})
    lo, hi = geometry.local_geometry_bounds(node, "Capsule", [])
    assert hi == pytest.approx((0.5, 0.5, 1.5))
    assert lo == pytest.approx((-0.5, -0.5, -1.5))


def test_cone_height_is_on_the_z_axis():
    node = _FakeNode("Cone", {"bottomRadius": _FakeField(1.0, "float"),
                              "height": _FakeField(3.0, "float")})
    lo, hi = geometry.local_geometry_bounds(node, "Cone", [])
    assert hi == pytest.approx((1.0, 1.0, 1.5))
    assert lo == pytest.approx((-1.0, -1.0, -1.5))


def test_sphere_and_plane_bounds():
    sphere = _FakeNode("Sphere", {"radius": _FakeField(1.25, "float")})
    assert geometry.local_geometry_bounds(sphere, "Sphere", []) == (
        (-1.25, -1.25, -1.25), (1.25, 1.25, 1.25))
    plane = _FakeNode("Plane", {"size": _FakeField((10.0, 4.0), "vec2")})
    lo, hi = geometry.local_geometry_bounds(plane, "Plane", [])
    assert lo == (-5.0, -2.0, 0.0)
    assert hi == (5.0, 2.0, 0.0)


def test_elevation_grid_spans_its_grid_and_heights():
    node = _FakeNode("ElevationGrid", {
        "xDimension": _FakeField(3, "int"),
        "yDimension": _FakeField(4, "int"),
        "xSpacing": _FakeField(2.0, "float"),
        "ySpacing": _FakeField(0.5, "float"),
        "height": _FakeField([0.0, 1.0, -0.5, 2.0], "mffloat"),
        "thickness": _FakeField(0.0, "float"),
    })
    lo, hi = geometry.local_geometry_bounds(node, "ElevationGrid", [])
    assert lo == pytest.approx((0.0, 0.0, -0.5))
    assert hi == pytest.approx((4.0, 1.5, 2.0))


def test_unsupported_geometry_reports_a_reason_instead_of_lying():
    node = _FakeNode("Fluid", {})
    result = geometry.local_geometry_bounds(node, "Fluid", [])
    assert result[0] is None
    assert "unsupported" in result[1]


def test_missing_mesh_url_is_reported_not_silently_zero():
    node = _FakeNode("Mesh", {"url": _FakeField(["nope.stl"], "mfstring")})
    result = geometry.local_geometry_bounds(node, "Mesh", ["/definitely/not/here"])
    assert result[0] is None
    assert "unreadable" in result[1]


# --- transform composition ----------------------------------------------------

def test_axis_angle_matrix_matches_a_known_rotation():
    m = geometry.axis_angle_to_matrix((0, 0, 1, math.pi / 2))
    assert geometry.mat_apply(m, (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-12)
    assert geometry.mat_apply(m, (0, 1, 0)) == pytest.approx((-1, 0, 0), abs=1e-12)


def test_frame_composition_applies_scale_then_rotation_then_translation():
    frame = geometry.Frame((1, 2, 3), geometry.axis_angle_to_matrix((0, 0, 1, math.pi / 2)),
                           (2.0, 2.0, 2.0))
    assert frame.to_world((1, 0, 0)) == pytest.approx((1, 4, 3), abs=1e-12)


def test_nested_frames_multiply_scale():
    root = geometry.Frame(scale=(2.0, 2.0, 2.0))
    child = root.compose((1, 0, 0), (0, 0, 1, 0.0), (3.0, 3.0, 3.0))
    assert child.origin == pytest.approx((2, 0, 0))
    assert child.scale == pytest.approx((6.0, 6.0, 6.0))


# --- mesh readers -------------------------------------------------------------

def test_stl_ascii_bounds(tmp_path):
    path = tmp_path / "tri.stl"
    path.write_text(
        "solid t\n facet normal 0 0 1\n  outer loop\n"
        "   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 2 3\n"
        "  endloop\n endfacet\nendsolid t\n"
    )
    assert geometry._mesh_file_bounds(str(path)) == ((0.0, 0.0, 0.0), (1.0, 2.0, 3.0))


def test_stl_binary_bounds(tmp_path):
    path = tmp_path / "tri.stl"
    body = b"\x00" * 80 + struct.pack("<I", 1)
    body += struct.pack("<12f", 0, 0, 1, -1, -1, -1, 2, 0, 0, 0, 3, 0)
    body += struct.pack("<H", 0)
    path.write_bytes(body)
    assert geometry._mesh_file_bounds(str(path)) == ((-1.0, -1.0, -1.0), (2.0, 3.0, 0.0))


def test_obj_bounds(tmp_path):
    path = tmp_path / "m.obj"
    path.write_text("# comment\nv 0 0 0\nv -1 2 0.5\nvn 0 0 1\nf 1 2 1\n")
    assert geometry._mesh_file_bounds(str(path)) == ((-1.0, 0.0, 0.0), (0.0, 2.0, 0.5))


def test_dae_bounds(tmp_path):
    path = tmp_path / "m.dae"
    path.write_text(
        '<?xml version="1.0"?>'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">'
        "<library_geometries><geometry><mesh>"
        '<source id="pos"><float_array count="9">'
        "0 0 0 1 0 0 0.5 2 -1</float_array></source>"
        '<vertices id="v"><input semantic="POSITION" source="#pos"/></vertices>'
        "</mesh></geometry></library_geometries></COLLADA>"
    )
    assert geometry._mesh_file_bounds(str(path)) == ((0.0, 0.0, -1.0), (1.0, 2.0, 0.0))


def test_gltf_bounds_use_the_declared_accessor_minmax(tmp_path):
    """glTF REQUIRES min/max on the POSITION accessor, so the bbox needs no
    vertex decoding at all.
    """
    path = tmp_path / "m.gltf"
    path.write_text(json.dumps({
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "accessors": [
            {"min": [-1.0, -2.0, 0.0], "max": [1.0, 2.0, 3.0]},
            {"min": [-99.0, -99.0, -99.0], "max": [99.0, 99.0, 99.0]},
        ],
    }))
    assert geometry._mesh_file_bounds(str(path)) == (
        (-1.0, -2.0, 0.0), (1.0, 2.0, 3.0))


def test_glb_container_bounds(tmp_path):
    doc = json.dumps({
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "accessors": [{"min": [0.0, 0.0, 0.0], "max": [0.5, 0.25, 2.0]}],
    }).encode()
    doc += b" " * ((4 - len(doc) % 4) % 4)
    blob = (b"glTF" + struct.pack("<II", 2, 12 + 8 + len(doc))
            + struct.pack("<I", len(doc)) + b"JSON" + doc)
    path = tmp_path / "m.glb"
    path.write_bytes(blob)
    assert geometry._mesh_file_bounds(str(path)) == (
        (0.0, 0.0, 0.0), (0.5, 0.25, 2.0))


def test_reset_caches_clears_everything():
    geometry._TYPE_CACHE[999] = "Box"
    geometry._MESH_CACHE[("x", 0, 0)] = None
    geometry._POSE_TYPE_MEMO["Whatever"] = True
    geometry.reset_caches()
    assert not geometry._TYPE_CACHE
    assert not geometry._MESH_CACHE
    assert not geometry._POSE_TYPE_MEMO


def test_mesh_bounds_are_cached_by_mtime(tmp_path):
    path = tmp_path / "m.obj"
    path.write_text("v 0 0 0\nv 1 1 1\n")
    first = geometry._mesh_file_bounds(str(path))
    assert first == ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    # Same stat -> cached object identity.
    assert geometry._mesh_file_bounds(str(path)) is first


def test_missing_mesh_file_returns_none():
    assert geometry._mesh_file_bounds(str(REPO_ROOT / "nope" / "nope.stl")) is None


# --- accumulator --------------------------------------------------------------

def test_accumulator_radius_is_the_half_diagonal():
    acc = geometry._Accum()
    acc.add_point((-1.0, -2.0, -3.0))
    acc.add_point((1.0, 2.0, 3.0))
    result = acc.result()
    assert result["center"] == [0.0, 0.0, 0.0]
    assert result["size"] == [2.0, 4.0, 6.0]
    assert result["radius"] == pytest.approx(math.sqrt(1 + 4 + 9), abs=1e-6)
    assert result["exact"] is True


def test_accumulator_reports_skipped_geometry():
    acc = geometry._Accum()
    acc.add_point((0.0, 0.0, 0.0))
    acc.note_skip("Mesh: unreadable url 'x.dae'")
    result = acc.result()
    assert result["exact"] is False
    assert result["skipped"] == ["Mesh: unreadable url 'x.dae'"]


def test_empty_accumulator_has_no_result():
    assert geometry._Accum().result() is None


# --- the engine-inversion oracle ----------------------------------------------

def test_move_viewpoint_inversion_is_algebraically_correct():
    """`probe_bounding_sphere` inverts OmViewpoint::moveViewpointToObject:

        distance = 1.05 * radius / (sin(fov/2) * min(aspect, 1/aspect))
        eye      = centre - view_direction * distance

    Two probes with different view directions give
    ``distance = |p1 - p2| / |d2 - d1|`` and then the centre and radius.
    This test replays that algebra against synthetic engine output, so the
    inversion is pinned even without a live simulator.
    """
    fov, aspect = math.pi / 4, 1.0
    true_center = (2.0, -3.0, 0.75)
    true_radius = 0.61
    tight = aspect if aspect <= 1.0 else 1.0 / aspect
    distance = 1.05 * true_radius / (math.sin(fov / 2.0) * tight)

    d1, d2 = (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    p1 = [true_center[i] - d1[i] * distance for i in range(3)]
    p2 = [true_center[i] - d2[i] * distance for i in range(3)]

    dd = math.sqrt(sum((d2[i] - d1[i]) ** 2 for i in range(3)))
    dp = math.sqrt(sum((p1[i] - p2[i]) ** 2 for i in range(3)))
    recovered_distance = dp / dd
    recovered_center = [p1[i] + d1[i] * recovered_distance for i in range(3)]
    recovered_radius = recovered_distance * math.sin(fov / 2.0) * tight / 1.05

    assert recovered_distance == pytest.approx(distance, rel=1e-12)
    assert recovered_center == pytest.approx(list(true_center), abs=1e-9)
    assert recovered_radius == pytest.approx(true_radius, rel=1e-12)
