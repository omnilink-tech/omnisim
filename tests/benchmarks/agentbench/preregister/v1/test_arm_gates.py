"""Guards for the v1 all-primary publication ledger."""

from __future__ import annotations

import json
import unittest

from . import arm_gates


class TestArmGates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = arm_gates.build()
        cls.by_cell = {
            (row["task"], row["sim"]): row for row in cls.doc["rows"]
        }

    def test_every_frozen_primary_cell_is_visible_exactly_once(self):
        self.assertEqual(len(self.doc["tasks"]), 10)
        self.assertEqual(len(self.doc["comparators"]), 5)
        self.assertEqual(len(self.doc["rows"]), 50)
        self.assertEqual(len(self.by_cell), 50)

    def test_missing_primary_adapters_block_instead_of_disappearing(self):
        for sim in ("isaac", "gazebo", "genesis"):
            rows = [row for row in self.doc["rows"] if row["sim"] == sim]
            self.assertEqual(len(rows), 10)
            self.assertTrue(all(row["publication_state"] == "BLOCKED"
                                for row in rows))
            self.assertTrue(all(not row["adapter_implemented"] for row in rows))

    def test_only_explicit_role_bound_records_satisfy_oracle_null(self):
        for sim in ("omnisim", "webots"):
            row = self.by_cell[("R1_lidar_nav", sim)]
            self.assertEqual(row["gates"]["oracle_PASS"]["state"], "GREEN")
            self.assertEqual(row["gates"]["null_FAIL"]["state"], "GREEN")
            self.assertEqual(row["publication_state"], "READY")

        # The old scripted B1 records are positive oracles only.  They must
        # not silently borrow a generic null fixture and turn green per sim.
        b1 = self.by_cell[("B1_overlap_audit", "webots")]
        self.assertEqual(b1["gates"]["null_FAIL"]["state"], "BLOCKED")

    def test_live_webots_r4_record_closes_that_cell(self):
        row = self.by_cell[("R4_mobile_manipulation", "webots")]
        self.assertEqual(row["gates"]["oracle_PASS"]["state"], "GREEN")
        self.assertEqual(row["gates"]["null_FAIL"]["state"], "GREEN")
        self.assertEqual(row["publication_state"], "READY")

    def test_red_evidence_is_green_for_all_50_cells(self):
        self.assertTrue(all(row["red_evidence"]["state"] == "GREEN"
                            for row in self.doc["rows"]))
        self.assertEqual(self.doc["summary"]["red_assertions_validated"], 61)
        self.assertEqual(self.doc["summary"]["red_assertions_total"], 61)

    def test_checked_in_view_is_exactly_generated(self):
        recorded = json.loads(arm_gates.OUTPUT.read_text(encoding="utf-8"))
        self.assertEqual(recorded, self.doc)


if __name__ == "__main__":
    unittest.main()
