"""Pure guards for the upstream-Webots BuildScale adapter boundary."""

from __future__ import annotations

import unittest
from pathlib import Path

from agentbench.adapters.omnisim.build_scale import source_audit
from agentbench.adapters.webots import build_scale
from agentbench.adapters.webots.build_scale_lane.generate_worlds import world_text
from agentbench.graders.evidence import ContactObservation, ContactPair


LANE = Path(__file__).resolve().parent / "build_scale_lane"


class TestWebotsBuildScale(unittest.TestCase):
    def test_worlds_have_exact_robot_counts_and_portable_sources(self):
        for role in ("", "_null"):
            world = LANE / "worlds" / f"build_scale_10{role}.wbt"
            text = world.read_text(encoding="utf-8")
            self.assertEqual(text.count("Pioneer3at {"), 10)
            audit = source_audit(world)
            self.assertFalse(audit["missing"])
            self.assertFalse(audit["absolute"])
            self.assertFalse(audit["forbidden"])

    def test_initial_contact_view_filters_by_recorded_step(self):
        observation = ContactObservation(
            pairs=[ContactPair("a", "b", True, True, step=5),
                   ContactPair("c", "d", True, True, step=20)],
            steps=1250, window_s=20, supported=True,
            total_observed=20, distinct_named=2, source="fixture")
        initial = build_scale._contacts(observation, first_steps=10)
        self.assertEqual([(p.a, p.b) for p in initial.pairs], [("a", "b")])
        self.assertEqual(initial.steps, 10)
        self.assertTrue(initial.can_name_a_robot_pair)

    def test_large_worlds_have_exact_unique_robot_names(self):
        for level in (25, 50, 100, 250):
            oracle = world_text(level, null=False)
            null = world_text(level, null=True)
            self.assertEqual(oracle.count("Pioneer3at {"), level)
            self.assertEqual(null.count("Pioneer3at {"), level)
            for i in range(level):
                self.assertEqual(oracle.count(f'name "scale_{i:03d}"'), 1)


if __name__ == "__main__":
    unittest.main()
