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

"""SparkRover Mini's shell (projects/manufacturing/products/spark_rover/shell.py): the
contract skeleton.py asks of it, its design checks, its print files. Engine-free: the
geometry is built with OpenCascade once per session (about a minute with the checks)."""
import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))

import shell  # noqa: E402
import skeleton as sk  # noqa: E402

# The checks that report the skeleton's problems rather than the shell's.
SKELETON_ONLY = {"skeleton_issues"}


@pytest.fixture(scope="module")
def checks():
    return shell.checks()


@pytest.fixture(scope="module")
def solids():
    return shell.solids()


@pytest.fixture(scope="module")
def parts():
    return shell.parts()


# ---- pure numbers (no OpenCascade) -------------------------------------------------------------

def test_printable_fillet_profile():
    p = shell.Profile(16.0, 2.0)
    # the chamfer meets the round at 45 degrees, continuous, and the bed face is set in 0.586 R
    assert p.s(0.0) == pytest.approx(16.0 * (2 - math.sqrt(2)), abs=1e-9)
    assert p.s(p.a - 1e-9) == pytest.approx(p.s(p.a + 1e-9), abs=1e-6)
    assert p.s(p.R) == 0.0 and p.s(40.0) == 0.0
    # slope of the chamfer is exactly 45 degrees
    assert (p.s(0.0) - p.s(p.a / 2)) / (p.a / 2) == pytest.approx(1.0)
    # the inner surface is the wall's thickness in, everywhere on the plain wall
    assert p.si(p.R + 5.0) == p.t
    assert p.si(p.u1 - 1e-9) == pytest.approx(p.si(p.u1 + 1e-9), abs=1e-6)
    for u in np.linspace(p.t, p.R, 25):
        assert p.si(u) >= p.s(u) + p.t * 0.99


def test_teardrop_points_where_asked():
    td = shell.teardrop(0.0, 0.0, 4.0, (0.0, 1.0))
    x0, y0, x1, y1 = td.bounds
    assert y1 == pytest.approx(2.0 * math.sqrt(2.0), abs=1e-6) and y0 == pytest.approx(-2.0, abs=1e-3)


def test_face_opening_is_one_hole_with_the_camera():
    op = shell.face_opening()
    assert op.geom_type == "Polygon"
    lens = sk.CAMERA["lens_at"]
    assert op.contains(shell.Point(lens[1], lens[2]))
    fw = sk.FACE_WINDOW
    assert op.contains(shell.Point(fw["centre"][1], fw["centre"][2]))


def test_inlet_leaves_the_duct_seat_on_the_hatch():
    c = shell.Shell()
    seat = shell.inlet_seat_rect(c)
    hz0 = sk.REAR_HATCH["centre"][2] - sk.REAR_HATCH["size"][1] / 2
    assert seat.bounds[1] >= hz0 + c.inlet_land_min
    assert shell.inlet_open(c).area >= 800.0


def test_skeleton_issues_each_say_what_to_change():
    issues = shell.skeleton_issues(shell.Shell())
    ids = {i["id"] for i in issues}
    assert {"shoulder_zone_in_wall", "arm_zone_touches_skin", "hatch_vent_overlap", "top_slot_zone_is_the_socket",
            "inlet_raised", "shell_wall"} <= ids
    assert all(i.get("change") for i in issues)


# ---- the contract ---------------------------------------------------------------------------------

def test_every_design_check_passes(checks):
    bad = {k: v[1] for k, v in checks.items() if not v[0] and k not in SKELETON_ONLY}
    assert not bad, bad


def test_checks_cover_what_the_brief_asks(checks):
    for k in ("fits_bed", "min_wall", "insert_bosses", "overhang", "zones_gap", "mass", "internal_fit",
              "interface_face_window", "interface_camera", "interface_chest_touch", "interface_speaker_grille",
              "interface_mic_holes", "interface_led_strip", "interface_rear_hatch", "interface_rear_vent",
              "interface_rear_inlet", "interface_usb_c", "interface_power_switch", "interface_top_slot",
              "interface_shoulder_opening_left", "interface_shoulder_opening_right", "interface_bumper"):
        assert k in checks, k
    for m in sk.SHELL_MOUNTS:
        assert checks[f"interface_mount_{m.name}"][0], m.name


def test_solids_closed_inside_the_envelope(solids):
    import trimesh
    env = sk.ENVELOPE
    for label, tris in solids.items():
        assert tris.ndim == 3 and tris.shape[1:] == (3, 3), label
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
        m.merge_vertices(digits_vertex=5)
        assert m.is_watertight and m.volume > 0, label
        v = tris.reshape(-1, 3)
        assert v[:, 0].min() >= env.x0 - 1e-3 and v[:, 0].max() <= env.x1 + 1e-3, label
        assert v[:, 1].min() >= env.y0 - 1e-3 and v[:, 1].max() <= env.y1 + 1e-3, label
        assert v[:, 2].min() >= env.z0 - 1e-3 and v[:, 2].max() <= env.z1 + 1e-3, label


def test_parts_contract_and_mass(parts, solids):
    keys = {"label", "step", "kind", "process", "material", "mass_g", "centre_mm", "source"}
    for p in parts:
        assert keys <= set(p), p["label"]
        assert p["mass_g"] > 0
        assert p["label"] in solids
    made = [p for p in parts if p["kind"] == "made"]
    assert {p["process"] for p in made} <= {"print", "cnc"}
    printed = sum(p["mass_g"] for p in made if p["process"] == "print")
    assert 350.0 <= printed <= 500.0
    assert sum(p["mass_g"] for p in parts) <= shell.MASS_CEILING_G
    for label in shell.SECTIONS:
        assert any(p["label"] == label and p["material"] == "PETG" for p in made)


def test_mounts_are_the_shell_mounts():
    assert {m.name for m in shell.mounts()} == {m.name for m in sk.SHELL_MOUNTS}
    assert all(m.holder == "shell" for m in shell.mounts())


def test_jobs_are_print_files_and_a_cnc_program():
    jobs = shell.jobs()
    printed = [k for k, v in shell.MADE.items() if v[3] == "print"]
    for label in printed:
        data = jobs[label]
        assert isinstance(data, bytes)
        n = struct.unpack("<I", data[80:84])[0]
        assert len(data) == 84 + 50 * n and n > 0
        rec = np.frombuffer(data[84:], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        v = rec["v"].reshape(-1, 3)
        assert v[:, 2].min() == pytest.approx(0.0, abs=1e-3)               # on the bed
        assert np.all(v.max(0) - v.min(0) <= np.array(shell.BED_MM) + 1e-3)  # mm, and it fits
    prog = jobs["face_cover"]
    assert isinstance(prog, str) and prog.rstrip().endswith("M2")
    assert all(len(line) <= 127 for line in prog.splitlines())              # GRBL's line limit
