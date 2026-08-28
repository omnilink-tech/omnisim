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

"""Tests for `omnisim validate-urdf`.

Every check is exercised twice: once with a description that violates it and
once with one that does not. A checker only ever shown firing on bad input has
not been shown to be quiet on good input, and a noisy checker is worse than
none -- so the negative controls are the more important half of this file.

Thresholds are asserted against closed-form values (a solid sphere's
2/5*m*r^2, a box's m*(y^2+z^2)/12) rather than numbers copied out of a previous
run, so a regression in the maths cannot be absorbed by updating a golden.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest

from omnisim import validate_urdf as V


def _urdf(body: str, name: str = "test") -> str:
    return f'<?xml version="1.0"?>\n<robot name="{name}">\n{body}\n</robot>\n'


def _inertial(mass=1.0, ixx=1.0, iyy=1.0, izz=1.0, ixy=0.0, ixz=0.0, iyz=0.0) -> str:
    return (
        "    <inertial>\n"
        f'      <mass value="{mass}"/>\n'
        f'      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" '
        f'iyy="{iyy}" iyz="{iyz}" izz="{izz}"/>\n'
        "    </inertial>\n"
    )


_BOX = '    <collision><geometry><box size="1 1 1"/></geometry></collision>\n'


def _link(name: str, inertial=True, geometry=True, **kw) -> str:
    parts = [f'  <link name="{name}">\n']
    if inertial:
        parts.append(_inertial(**kw))
    if geometry:
        parts.append(_BOX)
    parts.append("  </link>\n")
    return "".join(parts)


def _joint(name, parent, child, jtype="revolute", effort=10.0, velocity=1.0,
           lower=-1.0, upper=1.0, mimic=None) -> str:
    limit = ""
    if jtype in ("revolute", "prismatic"):
        limit = (f'    <limit effort="{effort}" velocity="{velocity}" '
                 f'lower="{lower}" upper="{upper}"/>\n')
    mim = f'    <mimic joint="{mimic}" multiplier="1" offset="0"/>\n' if mimic else ""
    return (
        f'  <joint name="{name}" type="{jtype}">\n'
        f'    <parent link="{parent}"/>\n'
        f'    <child link="{child}"/>\n'
        '    <axis xyz="0 0 1"/>\n'
        f"{limit}{mim}"
        "  </joint>\n"
    )


def _write(tmp_path: Path, text: str, name: str = "m.urdf") -> Path:
    p = tmp_path / name
    p.write_text(text, encoding="utf-8")
    return p


def _checks(findings) -> set[str]:
    return {f.check for f in findings}


def _valid_two_link() -> str:
    """A minimal description that must produce no findings at all."""
    return _urdf(_link("base") + _link("arm") + _joint("j", "base", "arm"))


def _args(inputs, tiers=",".join(V.DEFAULT_TIERS), check_meshes=False,
          as_json=False, quiet=True) -> argparse.Namespace:
    return argparse.Namespace(input=[str(i) for i in inputs], tiers=tiers,
                              check_meshes=check_meshes, json=as_json, quiet=quiet)


# --------------------------------------------------------------------------
# the negative control: a clean description stays silent
# --------------------------------------------------------------------------


def test_clean_description_has_no_findings(tmp_path):
    assert V.validate_file(_write(tmp_path, _valid_two_link())) == []


def test_shipped_robots_are_clean():
    """OmniSim's own flagship descriptions must pass, or the checker is noise."""
    root = Path(V.REPO_ROOT)
    for rel in ("projects/robots/omnisim/omniarm6/omniarm6.urdf",
                "projects/robots/omnisim/omniquad/urdf/omniquad.urdf",
                "projects/robots/unitree/g1/urdf/g1_23dof.urdf"):
        path = root / rel
        if not path.is_file():  # pragma: no cover - corpus varies by checkout
            pytest.skip(f"{rel} not present in this checkout")
        assert V.validate_file(path) == [], f"{rel}: {[str(f) for f in V.validate_file(path)]}"


# --------------------------------------------------------------------------
# physics: inertia
# --------------------------------------------------------------------------


def test_impossible_inertia_is_reported(tmp_path):
    """(1, 1, 3) is positive definite and describes no body: 1 + 1 < 3."""
    text = _urdf(_link("base", ixx=1.0, iyy=1.0, izz=3.0)
                 + _link("arm") + _joint("j", "base", "arm"))
    assert "inertia_triangle_inequality" in _checks(V.validate_file(_write(tmp_path, text)))


def test_solid_box_inertia_is_accepted(tmp_path):
    """Control: a real box satisfies the triangle inequality with margin."""
    mass, (x, y, z) = 2.0, (0.3, 0.4, 0.5)
    text = _urdf(
        _link("base", mass=mass,
              ixx=mass * (y**2 + z**2) / 12.0,
              iyy=mass * (x**2 + z**2) / 12.0,
              izz=mass * (x**2 + y**2) / 12.0)
        + _link("arm") + _joint("j", "base", "arm"))
    assert V.validate_file(_write(tmp_path, text)) == []


def test_solid_sphere_inertia_is_accepted(tmp_path):
    """Control: every principal moment of a sphere is 2/5*m*r^2, and equal."""
    mass, radius = 3.0, 0.2
    i = 0.4 * mass * radius**2
    text = _urdf(_link("base", mass=mass, ixx=i, iyy=i, izz=i)
                 + _link("arm") + _joint("j", "base", "arm"))
    assert V.validate_file(_write(tmp_path, text)) == []


def test_negative_inertia_is_reported(tmp_path):
    text = _urdf(_link("base", ixx=-1.0) + _link("arm") + _joint("j", "base", "arm"))
    assert "inertia_not_positive_definite" in _checks(V.validate_file(_write(tmp_path, text)))


def test_rank_deficient_inertia_reports_singular_not_negative(tmp_path):
    """A placeholder tensor lands near -1e-18; calling that 'negative' is noise."""
    text = _urdf(_link("base", ixx=0.01, iyy=0.01, izz=0.01,
                       ixy=0.01, ixz=0.01, iyz=0.01)
                 + _link("arm") + _joint("j", "base", "arm"))
    found = _checks(V.validate_file(_write(tmp_path, text)))
    assert "inertia_singular" in found
    assert "inertia_not_positive_definite" not in found


def test_units_error_is_caught_by_radius_of_gyration(tmp_path):
    """An inertia left in g*mm^2 on a kg*m^2 model: gyration radius 1000 m."""
    text = _urdf(_link("base", mass=1.0, ixx=1.0e6, iyy=1.0e6, izz=1.0e6)
                 + _link("arm") + _joint("j", "base", "arm"))
    assert "implausible_radius_of_gyration" in _checks(V.validate_file(_write(tmp_path, text)))


def test_plausible_link_does_not_trigger_gyration_check(tmp_path):
    assert "implausible_radius_of_gyration" not in _checks(
        V.validate_file(_write(tmp_path, _valid_two_link())))


def test_missing_inertia_tag_is_distinguished_from_declared_zeros(tmp_path):
    """<mass> with no <inertia> is an omission, not an assertion of zero."""
    text = _urdf(
        '  <link name="base">\n    <inertial><mass value="1.0"/></inertial>\n'
        + _BOX + "  </link>\n"
        + _link("arm") + _joint("j", "base", "arm"))
    found = _checks(V.validate_file(_write(tmp_path, text)))
    assert "missing_inertia_tag" in found
    assert "zero_inertia_with_mass" not in found


def test_declared_zero_inertia_is_reported(tmp_path):
    text = _urdf(_link("base", mass=1.0, ixx=0.0, iyy=0.0, izz=0.0)
                 + _link("arm") + _joint("j", "base", "arm"))
    assert "zero_inertia_with_mass" in _checks(V.validate_file(_write(tmp_path, text)))


# --------------------------------------------------------------------------
# physics: mass and joints
# --------------------------------------------------------------------------


def test_zero_mass_with_geometry_is_reported(tmp_path):
    text = _urdf(_link("base", mass=0.0) + _link("arm") + _joint("j", "base", "arm"))
    assert "zero_mass_with_geometry" in _checks(V.validate_file(_write(tmp_path, text)))


def test_massless_frame_without_geometry_is_accepted(tmp_path):
    """Control: a zero-mass link with no geometry is the standard frame idiom."""
    text = _urdf(_link("base") + '  <link name="tcp"/>\n'
                 + _joint("j", "base", "tcp", jtype="fixed"))
    assert V.validate_file(_write(tmp_path, text)) == []


def _one_zero_effort_outlier() -> str:
    """One zero-effort joint among non-zero siblings.

    A file whose *only* actuated joint declares effort=0 is genuinely ambiguous
    -- convention and defect are indistinguishable at n=1 -- so the outlier needs
    a sibling to stand out against.
    """
    return _urdf(_link("a") + _link("b") + _link("c")
                 + _joint("j1", "a", "b", effort=10.0)
                 + _joint("j2", "b", "c", effort=0.0))


def test_zero_effort_outlier_is_reported(tmp_path):
    assert "zero_effort" in _checks(
        V.validate_file(_write(tmp_path, _one_zero_effort_outlier())))


def test_whole_file_zero_effort_is_a_convention_not_a_defect(tmp_path):
    """Control: if every actuated joint is 0, the field is unpopulated."""
    text = _urdf(_link("a") + _link("b") + _link("c")
                 + _joint("j1", "a", "b", effort=0.0)
                 + _joint("j2", "b", "c", effort=0.0))
    assert "zero_effort" not in _checks(V.validate_file(_write(tmp_path, text)))


def test_mimic_joint_may_declare_zero_effort(tmp_path):
    """Control: a kinematically driven joint legitimately applies no torque."""
    text = _urdf(_link("palm") + _link("fa") + _link("fb")
                 + _joint("drive", "palm", "fa", jtype="prismatic")
                 + _joint("follow", "palm", "fb", jtype="prismatic",
                          effort=0.0, mimic="drive"))
    assert "zero_effort" not in _checks(V.validate_file(_write(tmp_path, text)))


def test_zero_width_range_is_reported(tmp_path):
    text = _urdf(_link("base") + _link("arm")
                 + _joint("j", "base", "arm", lower=0.5, upper=0.5))
    assert "zero_width_range" in _checks(V.validate_file(_write(tmp_path, text)))


def test_inverted_limits_are_reported(tmp_path):
    text = _urdf(_link("base") + _link("arm")
                 + _joint("j", "base", "arm", lower=1.0, upper=-1.0))
    assert "inverted_limits" in _checks(V.validate_file(_write(tmp_path, text)))


def test_fixed_joint_needs_no_limit(tmp_path):
    text = _urdf(_link("base") + _link("arm")
                 + _joint("j", "base", "arm", jtype="fixed"))
    assert V.validate_file(_write(tmp_path, text)) == []


# --------------------------------------------------------------------------
# topology
# --------------------------------------------------------------------------


def test_broken_link_reference_is_reported(tmp_path):
    text = _urdf(_link("base") + _link("arm") + _joint("j", "base", "ghost"))
    assert "broken_link_reference" in _checks(V.validate_file(_write(tmp_path, text)))


def test_multiple_roots_are_reported(tmp_path):
    text = _urdf(_link("a") + _link("b"))
    assert "multiple_root_links" in _checks(V.validate_file(_write(tmp_path, text)))


def test_closed_loop_is_reported(tmp_path):
    """A loop leaves the whole world without physics under Newton."""
    text = _urdf(_link("a") + _link("b")
                 + _joint("j1", "a", "b") + _joint("j2", "b", "a"))
    found = _checks(V.validate_file(_write(tmp_path, text)))
    assert "cycle" in found or "no_root_link" in found, found


def test_link_with_two_parent_joints_is_reported(tmp_path):
    text = _urdf(_link("a") + _link("b") + _link("c")
                 + _joint("j1", "a", "c") + _joint("j2", "b", "c"))
    assert "multiple_parents" in _checks(V.validate_file(_write(tmp_path, text)))


def test_mimic_of_unknown_joint_is_reported(tmp_path):
    text = _urdf(_link("a") + _link("b") + _link("c")
                 + _joint("j1", "a", "b")
                 + _joint("j2", "b", "c", mimic="nope"))
    assert "mimic_broken_reference" in _checks(V.validate_file(_write(tmp_path, text)))


def test_well_formed_mimic_is_accepted(tmp_path):
    """Control: the ordinary parallel-jaw coupling must be silent."""
    text = _urdf(_link("palm") + _link("fa") + _link("fb")
                 + _joint("drive", "palm", "fa", jtype="prismatic")
                 + _joint("follow", "palm", "fb", jtype="prismatic", mimic="drive"))
    assert V.validate_file(_write(tmp_path, text)) == []


# --------------------------------------------------------------------------
# CLI contract
# --------------------------------------------------------------------------


def test_cli_returns_zero_for_a_clean_description(tmp_path):
    assert V.main(_args([_write(tmp_path, _valid_two_link())])) == 0


def test_cli_returns_one_for_a_finding(tmp_path):
    assert V.main(_args([_write(tmp_path, _one_zero_effort_outlier())])) == 1


def test_cli_returns_two_for_an_unreadable_file(tmp_path):
    assert V.main(_args([tmp_path / "absent.urdf"])) == 2


def test_cli_returns_two_for_an_unknown_tier(tmp_path):
    assert V.main(_args([_write(tmp_path, _valid_two_link())], tiers="nonsense")) == 2


def test_cli_checks_every_file_before_returning(tmp_path, capsys):
    """A finding in the first file must not stop the second being checked."""
    first = _write(tmp_path, _one_zero_effort_outlier(), "a.urdf")
    second = _write(tmp_path, _valid_two_link(), "b.urdf")
    assert V.main(_args([first, second], as_json=True, quiet=False)) == 1
    out = capsys.readouterr().out
    assert "a.urdf" in out and "b.urdf" in out


def test_physics_tier_can_be_skipped(tmp_path):
    """A visualisation user runs the tiers that apply to them."""
    text = _urdf(_link("base", ixx=1.0, iyy=1.0, izz=3.0)
                 + _link("arm") + _joint("j", "base", "arm"))
    path = _write(tmp_path, text)
    assert V.validate_file(path, tiers=("topology",)) == []
    assert V.validate_file(path, tiers=("physics",)) != []
