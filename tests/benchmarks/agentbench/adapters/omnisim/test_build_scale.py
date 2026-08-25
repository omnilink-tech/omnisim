"""Unit guards for the OmniSim BuildScale adapter boundary."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from agentbench.adapters.omnisim import build_scale
from agentbench.adapters.omnisim.build_scale_lane.generate_worlds import world_text


WORLD = '''#OMNISIM R2025a utf8
WorldInfo { }
Robot {
  name "r0"
  controller "fleet"
}
Robot {
  name "r1"
  controller "fleet"
}
'''


class TestBuildScaleOmniSimAdapter(unittest.TestCase):
    def project(self, controller_text="print('drive')\n", world=WORLD):
        tmp = tempfile.TemporaryDirectory()
        root = Path(tmp.name)
        (root / "worlds").mkdir()
        (root / "controllers" / "fleet").mkdir(parents=True)
        w = root / "worlds" / "scale.wbt"
        w.write_text(world, encoding="utf-8")
        (root / "controllers" / "fleet" / "fleet.py").write_text(
            controller_text, encoding="utf-8")
        return tmp, w

    def test_clean_project_has_no_missing_or_absolute_dependencies(self):
        tmp, world = self.project()
        try:
            got = build_scale.source_audit(world)
            self.assertFalse(got["missing"])
            self.assertFalse(got["absolute"])
            self.assertFalse(got["forbidden"])
            self.assertEqual(got["artifact_files"], [
                "controllers/fleet/fleet.py", "worlds/scale.wbt"])
        finally:
            tmp.cleanup()

    def test_missing_controller_is_a_portability_failure(self):
        tmp, world = self.project()
        try:
            (Path(tmp.name) / "controllers" / "fleet" /
             "fleet.py").unlink()
            (Path(tmp.name) / "controllers" / "fleet").rmdir()
            got = build_scale.source_audit(world)
            self.assertEqual(len(got["missing"]), 1)
        finally:
            tmp.cleanup()

    def test_absolute_host_literal_is_reported(self):
        tmp, world = self.project("ASSET = 'C:/author/private/wheel.obj'\n")
        try:
            got = build_scale.source_audit(world)
            self.assertTrue(got["absolute"])
        finally:
            tmp.cleanup()

    def test_pose_field_mutation_is_forbidden(self):
        tmp, world = self.project(
            "node.getField('translation').setSFVec3f([1, 2, 3])\n")
        try:
            got = build_scale.source_audit(world)
            self.assertTrue(any("setSFVec3f" in hit for hit in got["forbidden"]))
        finally:
            tmp.cleanup()

    def test_motion_controller_may_not_be_a_supervisor(self):
        tmp, world = self.project(world=WORLD.replace(
            'controller "fleet"', 'controller "fleet"\n  supervisor TRUE', 1))
        try:
            got = build_scale.source_audit(world)
            self.assertTrue(any("Supervisor" in hit for hit in got["forbidden"]))
        finally:
            tmp.cleanup()

    def test_clean_replay_copy_is_outside_source_and_keeps_project_layout(self):
        tmp, world = self.project()
        replay_tmp = tempfile.TemporaryDirectory()
        try:
            copied = build_scale._copy_clean_project(
                world, Path(replay_tmp.name) / "project")
            self.assertTrue(copied.is_file())
            self.assertTrue((copied.parent.parent / "controllers" / "fleet" /
                             "fleet.py").is_file())
        finally:
            replay_tmp.cleanup()
            tmp.cleanup()

    def test_large_world_generator_emits_exact_unique_counts(self):
        for level in (50, 100, 250):
            oracle = world_text(level, null=False)
            null = world_text(level, null=True)
            self.assertEqual(oracle.count("ScaleBot {"), level)
            self.assertEqual(null.count("ScaleBot {"), level)
            self.assertEqual(null.count('controller "build_scale_idle"'), level)
            for i in range(level):
                self.assertEqual(oracle.count(f'name "scale_{i:03d}"'), 1)

    def test_large_world_timestep_stays_inside_frozen_sampling_bound(self):
        for level in (50, 100, 250):
            self.assertIn("basicTimeStep 32", world_text(level, null=False))
            self.assertLessEqual(32 / 1000, 0.1)


if __name__ == "__main__":
    unittest.main()
def test_window_contact_calibration_proves_naming_without_inventing_a_pair():
    phase = type("Phase", (), {})()
    phase.run_contacts = {
        "supported": True, "steps_sampled": 625,
        "total_observed": 5000, "distinct_named": 0, "pairs": []}
    phase.phase_a = {"contact_witness": {
        "supported": True, "total_observed": 44, "distinct_named": 4}}
    phase.meta = {
        "recorded_s": 20.0,
        "contact_witness_defs": {
            "requested": ["A", "B"], "found": ["A", "B"], "missing": []}}

    observed = build_scale._window_contacts(phase)

    assert observed.supported
    assert observed.distinct_named == 4
    assert observed.can_name_a_robot_pair
    assert observed.robot_robot_pairs == []


def test_window_contact_calibration_cannot_pass_with_a_missing_witness():
    phase = type("Phase", (), {})()
    phase.run_contacts = {
        "supported": True, "steps_sampled": 625,
        "total_observed": 5000, "distinct_named": 0, "pairs": []}
    phase.phase_a = {"contact_witness": {
        "supported": True, "total_observed": 44, "distinct_named": 4}}
    phase.meta = {
        "recorded_s": 20.0,
        "contact_witness_defs": {
            "requested": ["A", "B"], "found": ["A"], "missing": ["B"]}}

    observed = build_scale._window_contacts(phase)

    assert observed.distinct_named == 0
    assert not observed.can_name_a_robot_pair
