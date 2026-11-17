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

"""Static checks on the five Lane B ``initial_webots/`` task worlds.

WSL-free. The live half -- each world verified through the real launcher,
with the C1 world measured to FAIL to load -- is recorded in
``oracle_verdicts.json``'s notes and the committed evidence.
"""

import re
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

from agentbench.adapters.webots import launcher  # noqa: E402

TASKS = HERE.parent / "tasks"

WORLDS = {
    "B1_overlap_audit": "six_pioneers.wbt",
    "B2_subject_in_frame": "frame_the_cylinder.wbt",
    "B3_measure_and_report": "two_pioneers.wbt",
    "C1_parse_error_fix": "parse_error.wbt",
    "C2_fall_through_floor": "fall_through.wbt",
}

_EXTERNPROTO = re.compile(r'^EXTERNPROTO\s+"([^"]+)"', re.MULTILINE)


def _world(task):
    return TASKS / task / "initial_webots" / WORLDS[task]


def _text(task):
    return _world(task).read_text(encoding="utf-8")


class TestEveryWorld(unittest.TestCase):
    def test_exists_with_upstream_header(self):
        for task in WORLDS:
            with self.subTest(task=task):
                text = _text(task)
                self.assertTrue(text.startswith("#VRML_SIM R2025a utf8"))

    def test_no_rotation_at_exactly_pi(self):
        """Baseline sec. 5 trap 3: an axis-angle of exactly +/-pi sent a body
        non-finite within ten steps, reproducibly."""
        for task in WORLDS:
            with self.subTest(task=task):
                self.assertEqual(
                    launcher.pi_rotation_lines(_text(task)), [],
                    "%s carries a rotation within 1e-3 of +/-pi" % task)

    def test_externprotos_are_pinned_upstream_r2025a(self):
        """Every EXTERNPROTO must resolve from the pre-seeded R2025a asset
        cache: upstream-tag URLs only, so a cold cache never fetches an
        unpinned version (offline rule, BRINGUP.md sec. 1)."""
        prefix = ("https://raw.githubusercontent.com/cyberbotics/webots/"
                  "R2025a/")
        for task in WORLDS:
            with self.subTest(task=task):
                for url in _EXTERNPROTO.findall(_text(task)):
                    self.assertTrue(url.startswith(prefix),
                                    "%s imports %s" % (task, url))
                    self.assertNotIn("omnisim://", url)

    def test_basic_time_step_matches_omnisim_arm(self):
        for task in WORLDS:
            with self.subTest(task=task):
                self.assertEqual(
                    launcher.parse_basic_time_step_ms(_text(task)), 16.0)


class TestTaskSpecificSetup(unittest.TestCase):
    def test_b1_six_robots_one_close_pair(self):
        text = _text("B1_overlap_audit")
        self.assertEqual(text.count("Pioneer3at {"), 6)
        # The overlapping pair is 0.16 m apart -- far inside the ~0.5 m body.
        self.assertIn("translation 0 0 0.11", text)
        self.assertIn("translation 0.16 0.05 0.11", text)

    def test_b2_initial_camera_matches_omnisim_arm(self):
        text = _text("B2_subject_in_frame")
        omni = (TASKS / "B2_subject_in_frame" / "initial"
                / "frame_the_cylinder.wbt").read_text(encoding="utf-8")
        vp = "orientation 0.20664424 0.13429253 -0.96915617 2.01746471"
        self.assertIn(vp, text)
        self.assertIn(vp, omni)   # byte-identical initial aim on both arms
        for name in ("red_cylinder", "blue_cylinder", "green_cylinder",
                     "yellow_crate", "grey_sphere"):
            self.assertIn('name "%s"' % name, text)

    def test_b3_names_are_the_grader_contract(self):
        text = _text("B3_measure_and_report")
        self.assertIn('name "husky_ground"', text)
        self.assertIn('name "husky_plinth"', text)
        self.assertIn("translation 6.4 2.2 0.51", text)  # +0.4 m plinth

    def test_c1_carries_both_defects(self):
        text = _text("C1_parse_error_fix")
        self.assertIn("Soild", text)                      # undefined type
        self.assertEqual(text.count("{") - text.count("}"), 1,
                         "the unbalanced-brace defect must leave exactly "
                         "one unclosed block")
        for name in ("floor", "probe_bot", "pallet_a", "pallet_b"):
            self.assertIn('name "%s"' % name, text)

    def test_c2_floor_has_no_bounding_object_and_crate_is_first(self):
        text = _text("C2_fall_through_floor")
        floor = text.split("DEF FLOOR Solid")[1]
        self.assertNotIn("boundingObject", floor)
        # crate authored before floor: the webots adapter derives the
        # trajectory index for `crate_bot` from roster order.
        self.assertLess(text.index("DEF CRATE_BOT"), text.index("DEF FLOOR"))
        self.assertIn("physics Physics", text.split("DEF CRATE_BOT")[1]
                      .split("DEF FLOOR")[0])


if __name__ == "__main__":
    unittest.main()
