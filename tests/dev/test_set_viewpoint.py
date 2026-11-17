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

"""Tests for scripts/dev/set_viewpoint.py -- the viewpoint retrofit tool.

REGRESSION CONTEXT (2026-08-12). ``--radius`` is a *subject bounding-sphere*
radius, not a camera distance: the tool places the eye ~5.7x that far back
(hero) so the sphere fills the frame. That is correct, but the tool used to
print a bare ``position`` triple and nothing else, so an author who read
"radius 30" as "camera 30 m away" got a 172 m camera and had no way to notice
without rendering the world. Two cells in an agent diagnostic round were misled
by exactly this.

These tests pin the discoverability fix: the help text names the relationship,
and every run reports the resulting camera distance plus a framing check.

Run with:
    pytest tests/dev/test_set_viewpoint.py
"""

from __future__ import annotations

import importlib.util
import math
import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "python"))


def _load_set_viewpoint():
    path = REPO_ROOT / "scripts" / "dev" / "set_viewpoint.py"
    spec = importlib.util.spec_from_file_location("set_viewpoint_under_test", path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


sv = _load_set_viewpoint()

MINIMAL_WORLD = """#VRML_SIM R2025a utf8

WorldInfo {
  basicTimeStep 8
}
Viewpoint {
  orientation 0 0 1 0
  position -10 0 0
}
OmniSimSky {
}
DEF SUN OmniSimSun {
}
DEF CRATE Solid {
  translation 0 0 0.4
  children [
    Shape {
      geometry Box {
        size 0.8 0.8 0.8
      }
    }
  ]
  boundingObject Box {
    size 0.8 0.8 0.8
  }
}
"""


@pytest.fixture()
def world(tmp_path: Path) -> Path:
    p = tmp_path / "framing_probe.wbt"
    p.write_text(MINIMAL_WORLD, encoding="utf-8")
    return p


def _run(argv: list[str], capsys) -> str:
    sv.main(argv)
    return capsys.readouterr().out


# --------------------------------------------------------------------------
# 1. The help text must make the radius -> distance relationship discoverable.
# --------------------------------------------------------------------------


def test_radius_help_says_it_is_not_the_camera_distance(capsys):
    with pytest.raises(SystemExit):
        sv.main(["--help"])
    # argparse hard-wraps to the terminal width, so collapse whitespace before
    # matching -- otherwise this test fails on a narrow console, not on content.
    help_text = " ".join(capsys.readouterr().out.lower().split())
    # The old help was just "framing (bounding-sphere) radius, metres", which
    # is true and useless: nothing told the reader the eye lands ~5.7x further
    # out than the number they typed.
    assert "not the camera distance" in help_text, (
        "--radius help must say outright that it is not a camera distance"
    )
    assert re.search(r"5\.7", help_text), (
        "--radius help must quote the actual radius->distance multiplier"
    )


# --------------------------------------------------------------------------
# 2. Every run must report the resulting camera distance.
# --------------------------------------------------------------------------


def _expected_distance(radius: float, margin: float) -> float:
    half = sv.DEFAULT_FOV / 2.0
    tan_tight = math.tan(half) / sv.DEFAULT_ASPECT
    return radius / math.sin(math.atan(tan_tight)) * margin


@pytest.mark.parametrize(
    "mode,radius,margin",
    [("hero", 30.0, 1.3), ("hero", 10.0, 1.3), ("topdown", 24.0, 1.15),
     ("topdown", 12.0, 1.15)],
)
def test_reports_camera_distance(world, capsys, mode, radius, margin):
    out = _run([str(world), "--mode", mode, "--center", "0", "0", "0.3",
                "--radius", str(radius), "--dry-run"], capsys)
    m = re.search(r"camera distance\s+([0-9.]+)\s*m", out)
    assert m, f"no camera-distance line in output:\n{out}"
    reported = float(m.group(1))
    assert reported == pytest.approx(_expected_distance(radius, margin), rel=1e-3)
    # ...and the multiplier, so "radius 30 -> 172 m" is never a surprise.
    mult = re.search(r"([0-9.]+)x the --radius", out)
    assert mult, f"no radius multiplier in output:\n{out}"
    assert float(mult.group(1)) == pytest.approx(reported / radius, rel=1e-2)


def test_reported_distance_matches_the_written_position(world, capsys):
    out = _run([str(world), "--mode", "hero", "--center", "0", "0", "0.3",
                "--radius", "2.5", "--dry-run"], capsys)
    pos = re.search(r"position\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)", out)
    dist = re.search(r"camera distance\s+([0-9.]+)\s*m", out)
    assert pos and dist, out
    eye = tuple(float(g) for g in pos.groups())
    center = (0.0, 0.0, 0.3)
    actual = math.dist(eye, center)
    assert float(dist.group(1)) == pytest.approx(actual, rel=1e-3), (
        "the reported distance must be measured from the position it printed"
    )


# --------------------------------------------------------------------------
# 3. Every run must report a framing check, so a wrong radius is visible
#    without rendering the world.
# --------------------------------------------------------------------------


def test_reports_framing_fill(world, capsys):
    out = _run([str(world), "--mode", "hero", "--center", "0", "0", "0.3",
                "--radius", "1.0", "--dry-run"], capsys)
    assert "framing:" in out, f"no framing check in output:\n{out}"
    fill = re.search(r"fills\s+([0-9.]+)%", out)
    assert fill, f"framing check must quote a fill percentage:\n{out}"
    # margin 1.3 => the subject sphere subtends 1/1.3 of the tight half-angle
    # (asin/sin differ slightly at these angles, hence the loose tolerance).
    assert 70.0 <= float(fill.group(1)) <= 82.0


def test_framing_check_flags_a_subject_that_became_a_speck(world, capsys):
    """The exact failure the diagnostic round hit: an author reads --radius as
    a camera distance, asks for 30, and the 0.8 m crate becomes a speck. The
    tool must SAY so rather than printing a bare position triple."""
    out = _run([str(world), "--mode", "hero", "--center", "0", "0", "0.3",
                "--radius", "30", "--dry-run"], capsys)
    assert "framing:" in out
    low = out.lower()
    assert ("speck" in low or "small" in low or "broken" in low), (
        "a 0.8 m crate framed at --radius 30 must be flagged, got:\n" + out
    )


def test_framing_check_survives_a_world_it_cannot_analyse(tmp_path, capsys):
    """A world the scene walker cannot parse must still get the geometric
    report -- degrade, never crash, and never go silent."""
    p = tmp_path / "bare.wbt"
    p.write_text("#VRML_SIM R2025a utf8\n\nViewpoint {\n  orientation 0 0 1 0\n"
                 "  position -10 0 0\n}\n", encoding="utf-8")
    out = _run([str(p), "--center", "0", "0", "0", "--radius", "5", "--dry-run"],
               capsys)
    assert "camera distance" in out
    assert "framing:" in out


# --------------------------------------------------------------------------
# 4. Do not weaken what the tool already did.
# --------------------------------------------------------------------------


def test_still_rewrites_only_the_viewpoint_values(world, capsys):
    before = world.read_text(encoding="utf-8")
    _run([str(world), "--mode", "hero", "--center", "0", "0", "0.3",
          "--radius", "1.4"], capsys)
    after = world.read_text(encoding="utf-8")
    assert "DEF CRATE Solid" in after
    assert "OmniSimSky" in after
    assert before.count("\n") == after.count("\n")
    assert "position -10 0 0" not in after
