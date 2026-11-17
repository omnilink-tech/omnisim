#!/usr/bin/env python3
"""Offline checks for the cumulative v1 red-evidence coverage view."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from agentbench.preregister.v1 import red_coverage


HERE = Path(__file__).resolve().parent


class V1CoverageTests(unittest.TestCase):
    def test_r1_fixtures_advance_all_and_only_the_r1_rows(self) -> None:
        doc = red_coverage.build()
        self.assertEqual(doc["summary"]["total"], 61)
        self.assertEqual(doc["summary"]["validated"], 61)
        self.assertEqual(doc["summary"]["open"], 0)
        self.assertEqual(
            doc["summary"]["by_task"]["R1_lidar_nav"],
            {"validated": 6, "total": 6},
        )
        self.assertEqual(
            doc["summary"]["by_task"]["R2_arm_reach"],
            {"validated": 6, "total": 6},
        )
        self.assertEqual(
            doc["summary"]["by_task"]["R3_pick_and_place"],
            {"validated": 8, "total": 8},
        )
        self.assertEqual(
            doc["summary"]["by_task"]["R4_mobile_manipulation"],
            {"validated": 9, "total": 9},
        )
        for counts in doc["summary"]["by_task"].values():
            self.assertEqual(counts["validated"], counts["total"])

        newly_validated = {
            (row["task"], row["assertion"])
            for row in doc["rows"]
            if row.get("v1_observed_red")
        }
        self.assertEqual(
            newly_validated,
            {
                ("R1_lidar_nav", "R1.1"),
                ("R1_lidar_nav", "R1.2"),
                ("R1_lidar_nav", "R1.3"),
                ("R1_lidar_nav", "R1.4"),
                ("R1_lidar_nav", "R1.5"),
                ("R1_lidar_nav", "R1.6"),
                ("R2_arm_reach", "R2.1"),
                ("R2_arm_reach", "R2.2"),
                ("R2_arm_reach", "R2.3"),
                ("R2_arm_reach", "R2.4"),
                ("R2_arm_reach", "R2.5"),
                ("R2_arm_reach", "R2.6"),
                ("R3_pick_and_place", "R3.1"),
                ("R3_pick_and_place", "R3.2"),
                ("R3_pick_and_place", "R3.3"),
                ("R3_pick_and_place", "R3.4"),
                ("R3_pick_and_place", "R3.5"),
                ("R3_pick_and_place", "R3.6"),
                ("R3_pick_and_place", "R3.7"),
                ("R3_pick_and_place", "R3.8"),
                ("R4_mobile_manipulation", "R4.1"),
                ("R4_mobile_manipulation", "R4.2"),
                ("R4_mobile_manipulation", "R4.3"),
                ("R4_mobile_manipulation", "R4.4"),
                ("R4_mobile_manipulation", "R4.5"),
                ("R4_mobile_manipulation", "R4.6"),
                ("R4_mobile_manipulation", "R4.7"),
                ("R4_mobile_manipulation", "R4.8"),
                ("R4_mobile_manipulation", "R4.9"),
                ("A1_husky_swarm_10", "A1.10"),
                ("B2_subject_in_frame", "B2.1"),
                ("B2_subject_in_frame", "B2.2"),
                ("B2_subject_in_frame", "B2.4"),
                ("B2_subject_in_frame", "B2.5"),
                ("B2_subject_in_frame", "B2.6"),
                ("B3_measure_and_report", "B3.1"),
                ("B3_measure_and_report", "B3.2"),
                ("B3_measure_and_report", "B3.3"),
                ("B3_measure_and_report", "B3.4"),
                ("C2_fall_through_floor", "C2.1"),
                ("C2_fall_through_floor", "C2.2"),
                ("C2_fall_through_floor", "C2.4"),
                ("C2_fall_through_floor", "C2.5"),
            },
        )
        r15 = next(
            row for row in doc["rows"]
            if row["task"] == "R1_lidar_nav" and row["assertion"] == "R1.5"
        )
        self.assertIn(
            "preregister/v1/red_evidence/"
            "R1_lidar_nav.blind.omnisim.verdict.json",
            r15["v1_evidence"],
        )

    def test_checked_in_view_is_current(self) -> None:
        expected = json.dumps(red_coverage.build(), indent=2, sort_keys=True) + "\n"
        self.assertEqual(
            (HERE / "coverage.json").read_text(encoding="utf-8"),
            expected,
        )


if __name__ == "__main__":
    unittest.main()
