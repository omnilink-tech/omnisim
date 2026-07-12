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

        self.assertIn("Multiple collision elements are present", warnings)
        self.assertIn("Unsupported visual geometries are skipped.", warnings)
        self.assertIn("Unsupported collision geometries are skipped.", warnings)
        self.assertIn("package://debug/base_visual.stl", link_entry["unsupported_visuals"])
        self.assertIn("package://debug/base_collision.stl", link_entry["unsupported_collisions"])

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


if __name__ == "__main__":
    unittest.main()
