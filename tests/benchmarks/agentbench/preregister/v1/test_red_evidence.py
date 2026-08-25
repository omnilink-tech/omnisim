"""Offline integrity checks for committed AgenticSimBench v1 red evidence."""

import hashlib
import json
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
EVIDENCE = HERE / "red_evidence"
R1_BLIND = EVIDENCE / "R1_lidar_nav.blind.omnisim.verdict.json"
EXPECTED = {
    "altered_obstacle": ["R1.3", "R1.4", "R1.5", "R1.6"],
    "blind_non_null": ["R1.4", "R1.5", "R1.6"],
    "dirty": ["R1.1", "R1.4", "R1.5", "R1.6"],
    "undrivable": ["R1.2", "R1.4", "R1.5", "R1.6"],
}
R2_EXPECTED = {
    "below_ground": ["R2.3", "R2.4", "R2.5", "R2.6"],
    "dirty": ["R2.1", "R2.3", "R2.4", "R2.6"],
    "insufficient": ["R2.2", "R2.3", "R2.4", "R2.5", "R2.6"],
    "late": ["R2.6"],
    "static": ["R2.3", "R2.4", "R2.6"],
    "teleport": ["R2.4"],
    "wrong_order": ["R2.3", "R2.4", "R2.6"],
}
R3_EXPECTED = {
    "bad_start": ["R3.4", "R3.5", "R3.6", "R3.7"],
    "dirty": ["R3.1", "R3.5", "R3.6", "R3.7"],
    "insufficient_arm": ["R3.3", "R3.5", "R3.6", "R3.7"],
    "never_released": ["R3.6", "R3.7"],
    "no_lift": ["R3.5", "R3.6", "R3.7"],
    "scene_tamper": ["R3.2", "R3.5", "R3.6", "R3.7"],
    "teleport": ["R3.8"],
    "wrong_destination": ["R3.7"],
}
R4_EXPECTED = {
    "bad_start": ["R4.4", "R4.5", "R4.6", "R4.7", "R4.8", "R4.9"],
    "collision": ["R4.5"],
    "dirty": ["R4.1"],
    "insufficient": ["R4.3", "R4.5", "R4.6", "R4.7", "R4.8"],
    "no_carry": ["R4.6", "R4.7", "R4.8"],
    "reacquire": ["R4.7"],
    "scene_tamper": ["R4.2"],
    "teleport": ["R4.6", "R4.7", "R4.9"],
    "wrong_delivery": ["R4.8"],
}
FINAL_EXPECTED = {
    ("A1_husky_swarm_10", "a1_unattributed"): ["A1.10"],
    ("B2_subject_in_frame", "b2_no_claim"): ["B2.5", "B2.6"],
    ("B2_subject_in_frame", "b2_unchanged"):
        ["B2.1", "B2.2", "B2.4", "B2.5", "B2.6"],
    ("B3_measure_and_report", "b3_no_distance"): ["B3.1", "B3.2"],
    ("B3_measure_and_report", "b3_no_taller"): ["B3.3", "B3.4"],
    ("C2_fall_through_floor", "c2_dirty"): ["C2.1"],
    ("C2_fall_through_floor", "c2_fall"): ["C2.2", "C2.4", "C2.5"],
}


class TestCommittedRedEvidence(unittest.TestCase):
    def test_r1_blind_is_real_non_null_targeted_evidence(self):
        doc = json.loads(R1_BLIND.read_text(encoding="utf-8"))
        self.assertEqual(doc["schema"], "agenticsimbench/red-evidence/v1")
        self.assertEqual(doc["suite"], "agenticsimbench/v1")
        self.assertEqual(doc["task"], "R1_lidar_nav")
        self.assertEqual(doc["sim"], "omnisim")
        self.assertEqual(doc["fixture_kind"],
                         "scripted_controller_real_engine")
        self.assertTrue(doc["qualifies_as_non_null_red_evidence"])
        self.assertEqual(doc["expected_failures"],
                         ["R1.4", "R1.5", "R1.6"])
        self.assertEqual(doc["observed_failures"],
                         ["R1.4", "R1.5", "R1.6"])
        self.assertEqual(doc["outcome"], "FAIL")

    def test_every_r1_fixture_has_its_exact_registered_failure_set(self):
        docs = [
            json.loads(path.read_text(encoding="utf-8"))
            for path in sorted(EVIDENCE.glob("R1_lidar_nav.*.verdict.json"))
        ]
        self.assertEqual({doc["fixture"] for doc in docs}, set(EXPECTED))
        for doc in docs:
            expected = EXPECTED[doc["fixture"]]
            self.assertEqual(doc["expected_failures"], expected)
            self.assertEqual(doc["observed_failures"], expected)
            self.assertTrue(doc["qualifies_as_non_null_red_evidence"])
            self.assertEqual(doc["outcome"], "FAIL")

    def test_structural_failures_are_measured_not_inferred(self):
        dirty = json.loads(
            (EVIDENCE / "R1_lidar_nav.dirty.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        self.assertGreater(
            dirty["verdict"]["assertions"]["R1.1"]["measured"]["ERROR: lines"],
            0,
        )

        undrivable = json.loads(
            (EVIDENCE / "R1_lidar_nav.undrivable.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        r12 = undrivable["verdict"]["assertions"]["R1.2"]["measured"]
        self.assertEqual(r12["robot-class bodies"], 1)
        self.assertEqual(r12["of those, with >= 2 joints"], 0)

        altered = json.loads(
            (EVIDENCE / "R1_lidar_nav.altered_obstacle.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        r13 = altered["verdict"]["assertions"]["R1.3"]["measured"]
        self.assertEqual(r13["specified obstacles found"], 4)
        self.assertEqual(r13["specified obstacles missing"], ["OBSTACLE_5"])

    def test_every_r2_fixture_has_its_exact_registered_failure_set(self):
        docs = [
            json.loads(path.read_text(encoding="utf-8"))
            for path in sorted(EVIDENCE.glob("R2_arm_reach.*.verdict.json"))
        ]
        self.assertEqual({doc["fixture"] for doc in docs}, set(R2_EXPECTED))
        for doc in docs:
            expected = R2_EXPECTED[doc["fixture"]]
            self.assertEqual(doc["expected_failures"], expected)
            self.assertEqual(doc["observed_failures"], expected)
            self.assertTrue(doc["qualifies_as_non_null_red_evidence"])

    def test_r2_teleport_deadline_and_ground_clauses_are_independent(self):
        teleport = json.loads(
            (EVIDENCE / "R2_arm_reach.teleport.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        assertions = teleport["verdict"]["assertions"]
        self.assertTrue(assertions["R2.3"]["ok"])
        self.assertFalse(assertions["R2.4"]["ok"])
        self.assertTrue(assertions["R2.5"]["ok"])
        self.assertTrue(assertions["R2.6"]["ok"])
        measured = assertions["R2.4"]["measured"]
        self.assertGreater(measured["largest single-sample step (m)"], 0.125)
        self.assertGreater(
            measured["largest sample-to-sample speed (m/s)"], 2.5)

        late = json.loads(
            (EVIDENCE / "R2_arm_reach.late.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        self.assertEqual(late["observed_failures"], ["R2.6"])
        self.assertGreater(
            late["verdict"]["assertions"]["R2.6"]["measured"][
                "third hold satisfied at (s)"],
            30.0,
        )

        below = json.loads(
            (EVIDENCE / "R2_arm_reach.below_ground.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        clearance = below["verdict"]["assertions"]["R2.5"]["measured"][
            "lowest tip clearance over datum (m)"]
        self.assertLess(clearance, -0.02)

    def test_every_r3_fixture_has_its_exact_registered_failure_set(self):
        docs = [
            json.loads(path.read_text(encoding="utf-8"))
            for path in sorted(EVIDENCE.glob("R3_pick_and_place.*.verdict.json"))
        ]
        self.assertEqual({doc["fixture"] for doc in docs}, set(R3_EXPECTED))
        for doc in docs:
            expected = R3_EXPECTED[doc["fixture"]]
            self.assertEqual(doc["expected_failures"], expected)
            self.assertEqual(doc["observed_failures"], expected)
            self.assertTrue(doc["qualifies_as_non_null_red_evidence"])

    def test_r3_destination_and_teleport_fail_independently(self):
        wrong = json.loads(
            (EVIDENCE /
             "R3_pick_and_place.wrong_destination.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        self.assertEqual(wrong["observed_failures"], ["R3.7"])
        self.assertEqual(
            wrong["verdict"]["assertions"]["R3.7"]["measured"][
                "of those, outside the bin"],
            wrong["verdict"]["assertions"]["R3.7"]["measured"][
                "samples in the last 2.0 s"],
        )

        teleport = json.loads(
            (EVIDENCE / "R3_pick_and_place.teleport.omnisim.verdict.json")
            .read_text(encoding="utf-8"))
        self.assertEqual(teleport["observed_failures"], ["R3.8"])
        r38 = teleport["verdict"]["assertions"]["R3.8"]["measured"]
        self.assertGreater(r38["fastest the cube ever moved (m/s)"], 5.0)
        self.assertGreater(r38["largest single-sample step (m)"], 0.1)

    def test_every_r4_fixture_has_its_exact_registered_failure_set(self):
        docs = [
            json.loads(path.read_text(encoding="utf-8"))
            for path in sorted(
                EVIDENCE.glob("R4_mobile_manipulation.*.verdict.json"))
        ]
        self.assertEqual({doc["fixture"] for doc in docs}, set(R4_EXPECTED))
        for doc in docs:
            expected = R4_EXPECTED[doc["fixture"]]
            self.assertEqual(doc["expected_failures"], expected)
            self.assertEqual(doc["observed_failures"], expected)
            self.assertTrue(doc["qualifies_as_non_null_red_evidence"])

    def test_r4_collision_reacquire_and_delivery_are_independent(self):
        for fixture, assertion in (
            ("collision", "R4.5"),
            ("reacquire", "R4.7"),
            ("wrong_delivery", "R4.8"),
        ):
            doc = json.loads((
                EVIDENCE /
                f"R4_mobile_manipulation.{fixture}.omnisim.verdict.json"
            ).read_text(encoding="utf-8"))
            self.assertEqual(doc["observed_failures"], [assertion])

        collision = json.loads((
            EVIDENCE /
            "R4_mobile_manipulation.collision.omnisim.verdict.json"
        ).read_text(encoding="utf-8"))
        self.assertTrue(
            collision["verdict"]["assertions"]["R4.5"]["measured"]
            ["obstacles the track passes through"]
        )

        reacquire = json.loads((
            EVIDENCE /
            "R4_mobile_manipulation.reacquire.omnisim.verdict.json"
        ).read_text(encoding="utf-8"))
        self.assertEqual(
            reacquire["verdict"]["assertions"]["R4.7"]["measured"]
            ["separate airborne episodes of >= 0.5 s"],
            2,
        )

    def test_r1_collision_failure_names_both_participants(self):
        doc = json.loads(R1_BLIND.read_text(encoding="utf-8"))
        assertion = doc["verdict"]["assertions"]["R1.5"]
        self.assertFalse(assertion["ok"])
        measured = json.dumps(assertion["measured"], sort_keys=True)
        self.assertIn("rover", measured)
        self.assertIn("OBSTACLE", measured)
        self.assertGreater(
            assertion["measured"]["robot-obstacle/wall contacts"], 0)

    def test_final_seven_rows_have_live_targeted_evidence(self):
        for (task, fixture), expected in FINAL_EXPECTED.items():
            path = EVIDENCE / f"{task}.{fixture}.omnisim.verdict.json"
            doc = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(doc["expected_failures"], expected)
            self.assertEqual(doc["observed_failures"], expected)
            self.assertEqual(
                doc["fixture_kind"],
                "targeted_live_adapter_evidence_fixture",
            )
            self.assertTrue(doc["qualifies_as_non_null_red_evidence"])

    def test_final_controls_reach_the_intended_independent_channels(self):
        a1 = json.loads((
            EVIDENCE /
            "A1_husky_swarm_10.a1_unattributed.omnisim.verdict.json"
        ).read_text(encoding="utf-8"))
        self.assertIsNotNone(a1["attribution_before_deliberate_strip"])
        self.assertEqual(a1["outcome"], "INVALID")
        self.assertEqual(a1["observed_failures"], ["A1.10"])

        b2 = json.loads((
            EVIDENCE /
            "B2_subject_in_frame.b2_no_claim.omnisim.verdict.json"
        ).read_text(encoding="utf-8"))
        assertions = b2["verdict"]["assertions"]
        self.assertTrue(all(assertions[f"B2.{i}"]["ok"] for i in range(1, 5)))
        self.assertFalse(assertions["B2.5"]["ok"])

        c2 = json.loads((
            EVIDENCE /
            "C2_fall_through_floor.c2_fall.omnisim.verdict.json"
        ).read_text(encoding="utf-8"))
        self.assertTrue(c2["verdict"]["assertions"]["C2.3"]["ok"])
        self.assertLess(
            c2["verdict"]["assertions"]["C2.2"]["measured"]
            ["min z over all samples (m)"],
            -0.1,
        )

    def test_all_source_hashes_and_engine_attribution_are_present(self):
        for path in sorted(EVIDENCE.glob("*.verdict.json")):
            doc = json.loads(path.read_text(encoding="utf-8"))
            self.assertTrue(doc["engine_binary"]["sha256"], path)
            self.assertTrue(doc["sources"], path)
            self.assertTrue(all(item["sha256"] for item in doc["sources"]),
                            path)
            self.assertTrue(doc["run"]["sidecar"]["present"], path)
            for item in doc["sources"]:
                source = REPO / item["path"]
                self.assertTrue(source.is_file(), source)
                actual = hashlib.sha256(source.read_bytes()).hexdigest()
                self.assertEqual(actual, item["sha256"], source)


if __name__ == "__main__":
    unittest.main()
