"""Mechanical guard for the AgenticSimBench v1 claim pre-registration."""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
CONTRACT = HERE / "contract.json"
CLAIM = HERE / "CLAIM.md"
MANIFEST = HERE / "freeze_manifest.json"
FROZEN = (CONTRACT, CLAIM, HERE / "test_contract.py")


def _normal_bytes(path):
    return path.read_bytes().replace(b"\r\n", b"\n")


def _sha(path):
    return hashlib.sha256(_normal_bytes(path)).hexdigest()


def _contract():
    return json.loads(CONTRACT.read_text(encoding="utf-8"))


def _v1_rows():
    found = []
    for path in AGENTBENCH.rglob("*.jsonl"):
        try:
            for number, line in enumerate(
                    path.read_text(encoding="utf-8", errors="replace")
                    .splitlines(), 1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if (isinstance(row, dict)
                        and row.get("suite") == "agenticsimbench/v1"):
                    found.append("%s:%d" % (path, number))
        except OSError:
            continue
    return found


def build_manifest():
    return {
        "schema": "agenticsimbench/preregister-freeze/v1",
        "suite": "agenticsimbench/v1",
        "frozen_utc": "2026-08-13",
        "hash_rule": "sha256 over LF-normalised bytes",
        "files": {
            path.name: _sha(path) for path in FROZEN
        },
    }


class TestV1Contract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.doc = _contract()

    def test_identity_and_scope_are_exact(self):
        self.assertEqual(self.doc["suite"], "agenticsimbench/v1")
        self.assertEqual(self.doc["status"],
                         "preregistered_no_scored_rows")
        self.assertEqual(self.doc["comparators"]["primary"], [
            "omnisim", "isaac", "gazebo", "webots", "genesis"])
        self.assertEqual(self.doc["comparators"]["extended"], ["mujoco"])

    def test_models_are_pinned_not_inherited_from_cli_defaults(self):
        got = {(a["scaffold"], a["model"]) for a in self.doc["agents"]}
        self.assertEqual(got, {
            ("codex_cli", "gpt-5.6-sol"),
            ("claude_code", "claude-opus-5"),
        })
        self.assertEqual(
            self.doc["agent_identity"]["mixed_versions_within_campaign"],
            "forbidden")

    def test_track_order_matches_the_frozen_frontier(self):
        self.assertEqual(self.doc["tracks"], {
            "scene_reasoning": ["B1_overlap_audit", "B3_measure_and_report",
                                "B2_subject_in_frame"],
            "debug_loop": ["C1_parse_error_fix",
                           "C2_fall_through_floor"],
            "autonomy_scale": ["R1_lidar_nav", "A1_husky_swarm_10"],
            "manipulation": ["R2_arm_reach", "R3_pick_and_place",
                             "R4_mobile_manipulation"],
        })

    def test_repetition_and_solved_thresholds_measure_variance(self):
        rep = self.doc["repetitions"]
        self.assertEqual(rep["ordinary"], 5)
        self.assertEqual(rep["endpoint"], 10)
        self.assertEqual(rep["solved_fraction"], 0.8)
        self.assertLessEqual(rep["maximum_invalid_fraction"], 0.2)

    def test_scale_levels_and_physical_contract_are_frozen(self):
        scale = self.doc["build_scale"]
        self.assertEqual(scale["robots"], [10, 25, 50, 100, 250])
        self.assertTrue(scale["independently_controlled"])
        self.assertIn("no_teleport", scale["physical_requirements"])
        self.assertIn("replay_passes", scale["physical_requirements"])

    def test_publication_requires_red_evidence_and_arm_gates(self):
        gates = self.doc["publication_gates"]
        self.assertTrue(gates["red_evidence"].startswith("61/61"))
        self.assertIn("oracle_PASS", gates["per_task_simulator"])
        self.assertIn("null_FAIL", gates["per_task_simulator"])
        self.assertTrue(gates["artifact_and_trace_preserved"])

    def test_win_is_a_conjunction_and_cost_ceiling_is_125_percent(self):
        win = self.doc["win_rule"]
        self.assertTrue(win["all_clauses_required"])
        clauses = set(win["clauses"])
        for metric in ("agent_wall_s", "tokens_total", "tool_calls"):
            self.assertIn("median_normalized_%s_le_1.25" % metric, clauses)
        self.assertIn("independent_reproduction_passes", clauses)

    def test_missing_comparator_cannot_create_a_win(self):
        comp = self.doc["comparators"]
        self.assertEqual(comp["missing_primary_policy"], "blocks_full_claim")
        self.assertEqual(comp["post_result_removal"], "forbidden")

    def test_withdrawal_is_automatic(self):
        withdrawal = self.doc["withdrawal"]
        self.assertEqual(withdrawal["trigger"],
                         "any_win_clause_false_or_unevaluable")
        self.assertIn("forbid_best_claim", withdrawal["action"])
        self.assertEqual(withdrawal["post_first_row_changes"],
                         "new_suite_id_required")

    def test_no_v1_result_predates_the_freeze(self):
        self.assertEqual(_v1_rows(), [],
                         "v1 rows exist; the pre-run freeze is no longer "
                         "rewritable")

    def test_frozen_files_match_manifest(self):
        recorded = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(recorded, build_manifest())


def _write():
    rows = _v1_rows()
    if rows:
        raise SystemExit(
            "REFUSING: v1 result rows already exist; use a new suite id:\n%s"
            % "\n".join(rows))
    MANIFEST.write_text(json.dumps(build_manifest(), indent=2) + "\n",
                        encoding="utf-8")
    print("wrote %s" % MANIFEST)


if __name__ == "__main__":
    if "--write" in sys.argv[1:]:
        _write()
    else:
        unittest.main()
