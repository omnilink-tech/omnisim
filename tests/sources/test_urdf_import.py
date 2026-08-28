#!/usr/bin/env python

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

"""Tests for the developer URDF importer helper."""

import importlib.util
import math
import os
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


def load_urdf_import_module():
    """Import scripts/dev/urdf_import.py without requiring it to be a package."""
    webots_home = Path(os.environ["OMNISIM_HOME"])
    module_path = webots_home / "scripts" / "dev" / "urdf_import.py"
    spec = importlib.util.spec_from_file_location("urdf_import", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TestUrdfImport(unittest.TestCase):
    """Unit tests for axis conversion and debug reporting."""

    @classmethod
    def setUpClass(cls):
        cls.urdf_import = load_urdf_import_module()

    def write_urdf(self, contents: str) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "robot.urdf"
        path.write_text(textwrap.dedent(contents), encoding="utf-8")
        return path

    def test_rotated_joint_axis_is_emitted_in_parent_frame(self):
        path = self.write_urdf(
            """\
            <?xml version="1.0"?>
            <robot name="axis_debug">
              <link name="base"/>
              <link name="arm">
                <collision>
                  <geometry>
                    <box size="0.1 0.1 0.1"/>
                  </geometry>
                </collision>
                <inertial>
                  <mass value="1.0"/>
                </inertial>
              </link>
              <joint name="shoulder" type="revolute">
                <parent link="base"/>
                <child link="arm"/>
                <origin xyz="0 0 0" rpy="0 0 1.57079632679"/>
                <axis xyz="1 0 0"/>
                <limit lower="-1.0" upper="1.0" velocity="2.5" effort="3.0"/>
                <dynamics damping="0.3" friction="0.1"/>
              </joint>
            </robot>
            """
        )

        robot = self.urdf_import.parse_urdf(path)
        axis = self.urdf_import.joint_axis_in_parent_frame(robot.joints[0])
        self.assertAlmostEqual(axis[0], 0.0, places=6)
        self.assertAlmostEqual(axis[1], 1.0, places=6)
        self.assertAlmostEqual(axis[2], 0.0, places=6)

        vrml = self.urdf_import.emit_robot(robot)
        self.assertIn("minStop -1.0", vrml)
        self.assertIn("maxStop 1.0", vrml)
        self.assertIn("dampingConstant 0.3", vrml)
        self.assertIn("staticFriction 0.1", vrml)

    def test_report_flags_unsupported_geometry_and_collision_collapse(self):
        path = self.write_urdf(
            """\
            <?xml version="1.0"?>
            <robot name="report_debug">
              <link name="base">
                <visual>
                  <geometry>
                    <mesh filename="package://debug/base_visual.stl"/>
                  </geometry>
                </visual>
                <collision>
                  <geometry>
                    <mesh filename="package://debug/base_collision.stl"/>
                  </geometry>
                </collision>
                <collision>
                  <geometry>
                    <box size="0.1 0.1 0.1"/>
                  </geometry>
                </collision>
                <inertial>
                  <mass value="1.0"/>
                  <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
                </inertial>
              </link>
            </robot>
            """
        )

        robot = self.urdf_import.parse_urdf(path)
        report = self.urdf_import.build_report(robot)
        warnings = "\n".join(report["warnings"])
        link_entry = report["links"][0]

        # REMOVED 2026-08-15: assertIn("Multiple collision elements are present").
        # The importer stopped COLLAPSING multi-collision links in dc7f57c1d and
        # now emits a composite boundingObject via emit_collision_group()
        # (urdf_import.py:721), so the warning it demanded is gone -- correctly.
        # The test was stale, not the importer; asserting a warning that no
        # longer fires would forbid the improvement that removed it.
        # The wording moved from a generic "Unsupported visual geometries are
        # skipped." to a per-link message that NAMES the file it could not
        # resolve -- strictly more useful. Assert the durable parts (which
        # geometry kind, and that it is skipped) rather than a literal
        # sentence, so a future improvement to the phrasing does not go red.
        self.assertIn("Visual mesh(es) could not be resolved", warnings)
        self.assertIn("will be skipped", warnings)
        self.assertIn("Collision mesh(es) could not be resolved", warnings)
        # The report now separates "geometry TYPE we cannot import"
        # (unsupported_*) from "mesh FILE we could not find on disk"
        # (unresolved_meshes_*). This fixture's package:// paths do not exist,
        # so they are the second kind; asserting the first found an empty list
        # and reported a failure that said nothing about the importer.
        self.assertIn("package://debug/base_visual.stl", link_entry["unresolved_meshes_visual"])
        self.assertIn("package://debug/base_collision.stl", link_entry["unresolved_meshes_collision"])

    def test_cylinder_with_zero_rpy_has_no_pose_rotation(self):
        """Both URDF and Webots cylinders are aligned with +Z, so a URDF
        cylinder with no rpy must be emitted without any extra axis-angle
        rotation wrapped around it."""
        path = self.write_urdf(
            """\
            <?xml version="1.0"?>
            <robot name="cylinder_alignment">
              <link name="base"/>
              <link name="wheel">
                <visual>
                  <geometry>
                    <cylinder radius="0.04" length="0.02"/>
                  </geometry>
                </visual>
                <inertial>
                  <mass value="0.1"/>
                </inertial>
              </link>
              <joint name="wheel_joint" type="fixed">
                <parent link="base"/>
                <child link="wheel"/>
              </joint>
            </robot>
            """
        )

        robot = self.urdf_import.parse_urdf(path)
        vrml = self.urdf_import.emit_robot(robot)

        self.assertIn("geometry Cylinder { height 0.02 radius 0.04 }", vrml)
        # The cylinder's visual uses its default rpy, so no Pose { rotation ... }
        # should appear around the cylinder Shape.
        wheel_section = vrml.split('name "wheel"', 1)[1].split("HingeJoint", 1)[0]
        self.assertNotIn("rotation ", wheel_section)

    def test_cylinder_rpy_is_emitted_verbatim(self):
        """A URDF cylinder with rpy="pi/2 0 0" should emit exactly that
        rotation, i.e. axis (1, 0, 0) angle pi/2. Previously the importer
        silently added a pi/2 cylinder-axis correction on top, doubling the
        angle to pi and making the wheel render as a flat disc."""
        half_pi = math.pi / 2.0
        path = self.write_urdf(
            f"""\
            <?xml version="1.0"?>
            <robot name="cylinder_rpy">
              <link name="base"/>
              <link name="wheel">
                <visual>
                  <origin xyz="0 0 0" rpy="{half_pi} 0 0"/>
                  <geometry>
                    <cylinder radius="0.04" length="0.02"/>
                  </geometry>
                </visual>
                <inertial>
                  <mass value="0.1"/>
                </inertial>
              </link>
              <joint name="wheel_joint" type="fixed">
                <parent link="base"/>
                <child link="wheel"/>
              </joint>
            </robot>
            """
        )

        robot = self.urdf_import.parse_urdf(path)
        vrml = self.urdf_import.emit_robot(robot)
        expected = self.urdf_import.rpy_to_axis_angle((half_pi, 0.0, 0.0))
        rotation = f"rotation {expected[0]} {expected[1]} {expected[2]} {expected[3]}"

        self.assertIn(rotation, vrml)
        # Guard against a regression where the cylinder axis correction
        # would double the angle to pi.
        self.assertNotIn(f"rotation 1 0 0 {math.pi}", vrml)

    def test_demo_omnibot_urdf_imports_with_rotated_wheels(self):
        """End-to-end fixture test on the real OmniBot URDF shipped in
        projects/samples/demos. Catches regressions in the importer that
        would silently break the demo robot."""
        webots_home = Path(os.environ["OMNISIM_HOME"])
        urdf = webots_home / "projects/samples/demos/robots/omnibot.urdf"
        self.assertTrue(urdf.exists(), urdf)

        robot = self.urdf_import.parse_urdf(urdf)
        vrml = self.urdf_import.emit_robot(robot)

        # Both wheels use rpy="pi/2 0 0" so a 1 0 0 <~1.5708> rotation
        # must appear at least twice (left and right wheel visuals).
        self.assertGreaterEqual(vrml.count("rotation 1.0 0.0 0.0 1.5708"), 2)
        # The chassis box must be present and two sphere bounding objects
        # (wheel colliders) must also survive the import.
        self.assertIn("geometry Box { size 0.18 0.1 0.04 }", vrml)
        self.assertGreaterEqual(vrml.count("boundingObject Sphere"), 2)

    def test_demo_cube_bot_urdf_imports_with_rotated_wheels(self):
        """Minimal cube-with-two-wheels fixture. Built during debugging of
        the rotation pipeline and kept as a regression guard."""
        webots_home = Path(os.environ["OMNISIM_HOME"])
        urdf = webots_home / "projects/samples/demos/robots/cube_bot.urdf"
        self.assertTrue(urdf.exists(), urdf)

        robot = self.urdf_import.parse_urdf(urdf)
        vrml = self.urdf_import.emit_robot(robot)

        self.assertGreaterEqual(vrml.count("rotation 1.0 0.0 0.0 1.5708"), 2)
        self.assertIn("geometry Box { size 0.2 0.2 0.2 }", vrml)


    # ------------------------------------------------------------------
    # Physical-plausibility checks (added 2026-08-27).
    #
    # These four defect classes all LOAD without error and are silently
    # wrong, which is why a load-succeeds smoke test cannot see them. Each
    # test asserts BOTH directions -- fires on the defect, silent on the
    # valid control -- because a check that cannot go green is as useless
    # as one that cannot go red.
    # ------------------------------------------------------------------

    def _warnings_for(self, body: str) -> str:
        path = self.write_urdf(body)
        robot = self.urdf_import.parse_urdf(path)
        return chr(10).join(self.urdf_import.build_report(robot)["warnings"])

    def test_triangle_inequality_violation_is_flagged(self):
        """A tensor can be positive definite and still describe no rigid body.

        Values are the real ones shipped by a public robot description whose
        ixy was copy-pasted from ixx; principal moments come out
        (3.382e-05, 4.145e-04, 6.392e-04), so a+b < c by ~30%.
        """
        warnings = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="bad_inertia">
              <link name="base">
                <inertial>
                  <mass value="0.55665538"/>
                  <inertia ixx="0.00030053" ixy="0.00030053" ixz="-0.0000017"
                           iyy="0.00037247" iyz="-0.00000005" izz="0.00041454"/>
                </inertial>
              </link>
            </robot>
            """
        )
        self.assertIn("triangle inequality", warnings)

        # Control: the same tensor with the copy-paste undone is valid and
        # must produce no such warning.
        ok = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="good_inertia">
              <link name="base">
                <inertial>
                  <mass value="0.55665538"/>
                  <inertia ixx="0.00030053" ixy="0" ixz="-0.0000017"
                           iyy="0.00037247" iyz="-0.00000005" izz="0.00041454"/>
                </inertial>
              </link>
            </robot>
            """
        )
        self.assertNotIn("triangle inequality", ok)

    def test_zero_mass_link_with_geometry_is_flagged(self):
        warnings = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="massless">
              <link name="finger">
                <visual><geometry><box size="0.01 0.01 0.05"/></geometry></visual>
                <collision><geometry><box size="0.01 0.01 0.05"/></geometry></collision>
                <inertial>
                  <mass value="0"/>
                  <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                </inertial>
              </link>
            </robot>
            """
        )
        self.assertIn("cannot be simulated", warnings)

        # Control: a massless link carrying NO geometry is a normal URDF
        # idiom (a pure frame) and must stay silent.
        ok = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="frame_only">
              <link name="imu_frame">
                <inertial>
                  <mass value="0"/>
                  <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                </inertial>
              </link>
            </robot>
            """
        )
        self.assertNotIn("cannot be simulated", ok)

    def test_zero_effort_and_velocity_limits_are_flagged(self):
        """effort="0" is a declared inability to move, not "unlimited" --
        URDF spells unlimited by omitting the attribute."""
        warnings = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="no_authority">
              <link name="base"/>
              <link name="arm"/>
              <joint name="shoulder" type="revolute">
                <parent link="base"/>
                <child link="arm"/>
                <axis xyz="0 0 1"/>
                <limit effort="0" velocity="0" lower="-1.0" upper="1.0"/>
              </joint>
            </robot>
            """
        )
        self.assertIn("zero authority", warnings)

        ok = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="has_authority">
              <link name="base"/>
              <link name="arm"/>
              <joint name="shoulder" type="revolute">
                <parent link="base"/>
                <child link="arm"/>
                <axis xyz="0 0 1"/>
                <limit effort="50" velocity="12.5664" lower="-1.0" upper="1.0"/>
              </joint>
            </robot>
            """
        )
        self.assertNotIn("zero authority", ok)

    def test_joint_range_excluding_zero_is_flagged(self):
        """All-zero is the only default configuration a URDF can express, so a
        range excluding it leaves every consumer starting in limit violation.

        Note this is INFORMATION, not automatically a defect: Franka's real
        panda_joint4 has range [-3.0718, -0.0698] and is a correct description
        of a real robot. The warning asks for a published home pose.
        """
        warnings = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="no_zero">
              <link name="base"/>
              <link name="shin"/>
              <joint name="knee" type="revolute">
                <parent link="base"/>
                <child link="shin"/>
                <axis xyz="0 1 0"/>
                <limit effort="150" velocity="12" lower="0.1745" upper="2.443"/>
              </joint>
            </robot>
            """
        )
        self.assertIn("excludes the zero position", warnings)

        ok = self._warnings_for(
            """            <?xml version="1.0"?>
            <robot name="spans_zero">
              <link name="base"/>
              <link name="shin"/>
              <joint name="knee" type="revolute">
                <parent link="base"/>
                <child link="shin"/>
                <axis xyz="0 1 0"/>
                <limit effort="150" velocity="12" lower="-0.1" upper="2.443"/>
              </joint>
            </robot>
            """
        )
        self.assertNotIn("excludes the zero position", ok)

    def test_principal_moments_matches_closed_form_on_a_diagonal_tensor(self):
        """Guards the no-numpy eigenvalue solver: a diagonal tensor's principal
        moments are its diagonal, and the solver must not perturb them."""
        moments = self.urdf_import.principal_moments(3.0, 0.0, 0.0, 1.0, 0.0, 2.0)
        for got, want in zip(moments, (1.0, 2.0, 3.0)):
            self.assertAlmostEqual(got, want, places=12)


    def test_gazebo_mu1_becomes_per_solid_friction(self):
        """URDF has no native schema for SURFACE friction; <gazebo><mu1> is the
        de-facto convention and we used to drop it silently.

        It now emits Solid.newtonFriction, which is the only way a robot whose
        gripper pads grip harder than the table it works on can state that in
        its own file -- friction was one global world value until 2026-08-27.
        """
        path = self.write_urdf(
            """            <?xml version="1.0"?>
            <robot name="gripper">
              <link name="base">
                <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
                <inertial><mass value="1.0"/>
                  <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
              </link>
              <link name="pad">
                <collision><geometry><box size="0.02 0.01 0.03"/></geometry></collision>
                <inertial><mass value="0.05"/>
                  <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
              </link>
              <joint name="slide" type="prismatic">
                <parent link="base"/><child link="pad"/><axis xyz="0 1 0"/>
                <limit effort="100" velocity="0.2" lower="0" upper="0.04"/>
              </joint>
              <gazebo reference="pad"><mu1>2.0</mu1><mu2>2.0</mu2></gazebo>
            </robot>
            """
        )
        robot = self.urdf_import.parse_urdf(path)
        self.assertEqual(robot.links["pad"].surface_friction, 2.0)
        self.assertIsNone(robot.links["base"].surface_friction)
        vrml = self.urdf_import.emit_robot(robot)
        self.assertIn("newtonFriction 2.0", vrml)
        # Exactly one link declared it, so exactly one emission.
        self.assertEqual(vrml.count("newtonFriction"), 1)

    def test_urdf_without_gazebo_friction_emits_none(self):
        """Control: a URDF that declares no surface friction must emit no
        newtonFriction at all, so every existing robot keeps inheriting the
        world value and nothing changes for it."""
        path = self.write_urdf(
            """            <?xml version="1.0"?>
            <robot name="plain">
              <link name="base">
                <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
                <inertial><mass value="1.0"/>
                  <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
              </link>
            </robot>
            """
        )
        robot = self.urdf_import.parse_urdf(path)
        vrml = self.urdf_import.emit_robot(robot)
        self.assertNotIn("newtonFriction", vrml)


    # ------------------------------------------------------------------
    # Cross-file mirror diff. A whole class of defect is invisible to a
    # single-file checker because nothing in the file is out of range -- the
    # value is only wrong RELATIVE TO the robot's other hand. The mirror is
    # also the only place the CORRECT value comes from.
    # ------------------------------------------------------------------

    HAND = """        <?xml version="1.0"?>
        <robot name="hand">
          <link name="palm"/>
          <link name="finger"/>
          <joint name="index_mcp" type="revolute">
            <parent link="palm"/><child link="finger"/><axis xyz="0 0 1"/>
            <limit effort="%s" velocity="%s" lower="0" upper="1.3"/>
          </joint>
        </robot>
        """

    def test_mirror_diff_catches_transposed_effort_and_velocity(self):
        """The real case: one hand ships (effort 100, velocity 1) and its twin
        ships (effort 1, velocity 100). Both are in range; only the pair is wrong."""
        left = self.write_urdf(self.HAND % ("100", "1"))
        right = self.write_urdf(self.HAND % ("1", "100"))
        rep = self.urdf_import.build_mirror_report(
            self.urdf_import.parse_urdf(left), self.urdf_import.parse_urdf(right))
        self.assertEqual(rep["matched_joint_pairs"], 1)
        joined = chr(10).join(rep["findings"])
        self.assertIn("effort differs across the mirror", joined)
        self.assertIn("velocity differs across the mirror", joined)

    def test_mirror_diff_is_silent_on_a_matching_pair(self):
        """Control: identical twins must produce no findings at all, otherwise
        the mode is noise and nobody will run it."""
        a = self.write_urdf(self.HAND % ("4.8", "1"))
        b = self.write_urdf(self.HAND % ("4.8", "1"))
        rep = self.urdf_import.build_mirror_report(
            self.urdf_import.parse_urdf(a), self.urdf_import.parse_urdf(b))
        self.assertEqual(rep["matched_joint_pairs"], 1)
        self.assertEqual(rep["findings"], [])

    def test_mirror_matching_handles_both_naming_conventions(self):
        """Side-in-the-name and side-in-the-path are both common. Missing the
        second made this checker match ZERO pairs on the descriptions it was
        written for, so both are asserted."""
        self.assertEqual(self.urdf_import.mirror_name("left_arm_joint2"), "right_arm_joint2")
        self.assertEqual(self.urdf_import.mirror_name("right_arm_joint2"), "left_arm_joint2")
        # No affix -> no mirror name; the caller falls back to identity matching.
        self.assertIsNone(self.urdf_import.mirror_name("index_mcp_roll"))


if __name__ == "__main__":
    unittest.main()
