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

"""Lightweight packaging / static checks for the omnisim_ros2_nav2 package.

This is deliberately a STATIC check, not a live Nav2 run: it needs no ROS, no
`launch`/`launch_ros`, no simulator and no network. It verifies that the pieces a
`colcon build` installs are actually present and well-formed — the package metadata,
the ament resource marker, the launch files (present + syntactically parseable), the
Nav2 / slam_toolbox params (present + valid YAML), the DDS config (valid XML), and the
`cmd_vel_relay` console-script target. The full live M1-M6 reproduction stays in the
package README / REPORT, not in a test.
"""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
PKG_NAME = "omnisim_ros2_nav2"

# Files the package promises to ship. A rename or deletion should fail here.
EXPECTED_LAUNCH = [
    "omnisim_slam.launch.py",
    "omnisim_nav2_bringup.launch.py",
    "omnisim_nav2_lean.launch.py",
]
EXPECTED_PARAMS = [
    "omnisim_nav2_params.yaml",
    "mapper_params_online_async.yaml",
]
EXPECTED_CONFIG = [
    "fastdds_profile.xml",
]


def test_resource_marker_present():
    """The ament index marker is what makes the package discoverable after install."""
    marker = PKG_ROOT / "resource" / PKG_NAME
    assert marker.is_file(), f"missing ament resource marker: {marker}"


def test_package_xml_is_well_formed():
    pkg_xml = PKG_ROOT / "package.xml"
    assert pkg_xml.is_file(), "package.xml is missing"
    root = ET.parse(pkg_xml).getroot()
    assert root.tag == "package"
    assert root.findtext("name") == PKG_NAME
    build_type = root.find("./export/build_type")
    assert build_type is not None and build_type.text == "ament_python"


def test_setup_py_declares_share_installs():
    """The launch/params/config trees must be wired into data_files, or an installed
    package resolves none of them via FindPackageShare."""
    setup_src = (PKG_ROOT / "setup.py").read_text(encoding="utf-8")
    for needed in (
        "resource_index/packages",       # ament index marker install target
        "package.xml",
        "launch/*.launch.py",
        "params/*.yaml",
        "config/*.xml",
    ):
        assert needed in setup_src, f"setup.py data_files does not install {needed!r}"


def test_launch_files_present_and_parseable():
    launch_dir = PKG_ROOT / "launch"
    for name in EXPECTED_LAUNCH:
        path = launch_dir / name
        assert path.is_file(), f"missing launch file: {path}"
        source = path.read_text(encoding="utf-8")
        # compile() validates syntax without importing ROS launch modules.
        compile(source, str(path), "exec")


def test_params_yaml_loads():
    yaml = pytest.importorskip("yaml")
    params_dir = PKG_ROOT / "params"
    for name in EXPECTED_PARAMS:
        path = params_dir / name
        assert path.is_file(), f"missing params file: {path}"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert isinstance(data, dict) and data, f"params YAML is empty/not a mapping: {path}"


def test_config_xml_parses():
    config_dir = PKG_ROOT / "config"
    for name in EXPECTED_CONFIG:
        path = config_dir / name
        assert path.is_file(), f"missing config file: {path}"
        ET.parse(path)  # raises on malformed XML


def test_cmd_vel_relay_entry_point_exists():
    """setup.py exposes `cmd_vel_relay = omnisim_ros2_nav2.cmd_vel_relay:main`."""
    module = PKG_ROOT / PKG_NAME / "cmd_vel_relay.py"
    assert module.is_file(), f"missing console-script module: {module}"
    tree = ast.parse(module.read_text(encoding="utf-8"))
    functions = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert "main" in functions, "cmd_vel_relay.py does not define main()"


def test_lean_launch_references_shipped_params():
    """The lean launch (the one that closed M6) must default to the shipped params
    via the package share, not an absolute path."""
    lean = (PKG_ROOT / "launch" / "omnisim_nav2_lean.launch.py").read_text(encoding="utf-8")
    assert "FindPackageShare" in lean
    assert "omnisim_nav2_params.yaml" in lean
