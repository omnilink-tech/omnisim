"""The BuildScale gate ledger must remain exhaustive and evidence-bound."""

from __future__ import annotations

import unittest

from agentbench.preregister.v1 import build_scale_gates


class TestBuildScaleGates(unittest.TestCase):
    def test_every_primary_comparator_and_level_is_visible(self):
        doc = build_scale_gates.build()
        self.assertEqual(doc["summary"]["cells_total"], 25)
        self.assertEqual(len(doc["rows"]), 25)
        self.assertEqual(
            {(r["sim"], r["level"]) for r in doc["rows"]},
            {(sim, level) for sim in doc["comparators"]
             for level in doc["levels"]})

    def test_ready_requires_a_live_record(self):
        for row in build_scale_gates.build()["rows"]:
            if row["state"] == "READY":
                self.assertTrue(row["evidence"])
                self.assertIn("oracle PASS and null FAIL", row["detail"])
            else:
                self.assertEqual(row["state"], "BLOCKED")

    def test_gate_output_cannot_be_misread_as_agent_score(self):
        doc = build_scale_gates.build()
        self.assertEqual(doc["summary"]["scored_model_rows"], 0)
        self.assertEqual(doc["summary"]["full_gate"], "BLOCKED")


if __name__ == "__main__":
    unittest.main()
