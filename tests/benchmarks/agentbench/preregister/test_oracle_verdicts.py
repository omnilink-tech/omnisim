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

"""The committed oracle verdicts: freshness, cell coverage, and WSL-free
grader-PASS replays over the committed run evidence.

Three layers, weakest dependency first:

1. **arithmetic freshness** -- ``verdicts.compute`` over the committed
   ``runs/cells.json`` must reproduce the committed
   ``oracle_verdicts.json``, so the verdict file cannot drift from its own
   inputs;
2. **cell coverage** -- all 20 (task, sim, condition) cells exist, each
   cites its run artifact;
3. **grader replay** -- every WEBOTS cell's verdict is recomputed from the
   committed evidence copies with the real graders and real adapters,
   needing no WSL and no simulator. The OMNISIM cells' grading needs a live
   engine (phase-B recorder runs), so for them this file asserts the
   committed row itself (produced by ``run_agentbench.py`` on a real engine
   run and copied under ``evidence/``) -- re-running those cells is
   ``run_oracles.py``'s job, not a unit test's.
"""

import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

from agentbench.preregister import verdicts as verdicts_mod  # noqa: E402

RUNS = HERE / "runs"
EVIDENCE = HERE / "evidence"
VERDICTS = HERE / "oracle_verdicts.json"


def _cells():
    return json.loads((RUNS / "cells.json").read_text(
        encoding="utf-8"))["cells"]


def _verdicts_doc():
    return json.loads(VERDICTS.read_text(encoding="utf-8"))


def _evidence_dir(cell):
    return EVIDENCE / cell["sim"] / ("%s.%s" % (
        cell["task"], cell["condition"].replace("+", "_")))


class TestFreshness(unittest.TestCase):
    def test_verdicts_match_recomputation_from_cells(self):
        doc = _verdicts_doc()
        recomputed = verdicts_mod.compute(_cells())
        self.assertEqual(doc["verdicts"], recomputed,
                         "oracle_verdicts.json is stale: re-run "
                         "preregister/run_oracles.py --verdicts-only")

    def test_verdict_doc_cell_summaries_match_cells(self):
        doc = _verdicts_doc()
        by_key = {(c["task"], c["sim"], c["condition"]): c for c in _cells()}
        for c in doc["cells"]:
            src = by_key[(c["task"], c["sim"], c["condition"])]
            self.assertEqual(c["outcome"], src.get("outcome"))
            self.assertEqual(c["tool_calls"], src.get("tool_calls"))


class TestCoverage(unittest.TestCase):
    def test_all_twenty_cells_present_each_citing_artifacts(self):
        cells = _cells()
        keys = {(c["task"], c["sim"], c["condition"]) for c in cells}
        for task in verdicts_mod.TASKS:
            for sim in verdicts_mod.SIMS:
                for cond in verdicts_mod.CONDITIONS:
                    self.assertIn((task, sim, cond), keys)
        for c in cells:
            with self.subTest(cell=(c["task"], c["sim"], c["condition"])):
                self.assertTrue(c.get("run_dir"), "cell cites no run_dir")
                self.assertTrue(_evidence_dir(c).is_dir(),
                                "no committed evidence copy")
                self.assertIsNotNone(c.get("tool_calls"))

    def test_oracle_outcomes_are_pass(self):
        """The plan-2.1 lane test needs the oracle to COMPLETE every cell;
        a non-PASS committed cell is a loud change to the frozen design."""
        for c in _cells():
            with self.subTest(cell=(c["task"], c["sim"], c["condition"])):
                self.assertEqual(c.get("outcome"), "PASS")


class TestWebotsGraderReplay(unittest.TestCase):
    """Recompute every Webots cell's verdict from committed evidence.

    Real graders, real adapters, no WSL: the launcher/prober artifacts are
    committed under ``evidence/webots/<cell>/`` precisely so this replay can
    run anywhere. (The runs themselves happened on the WSL2 install recorded
    in BRINGUP.md; the cells.json rows cite them.)
    """

    @classmethod
    def _grade(cls, cell):
        from agentbench import tasks as task_registry
        from agentbench.adapters.webots import task_support

        dest = _evidence_dir(cell)
        task = task_registry.get(cell["task"])
        aabbs = None
        aabb_path = dest / "aabb" / "aabbs.json"
        if aabb_path.is_file():
            aabbs = json.loads(aabb_path.read_text(encoding="utf-8"))
        run, _merged = task_support.augment_run(dest / "webots", aabbs)
        artifacts = sorted(dest.glob("*.wbt"))
        answer = ""
        fm = dest / "final_message.txt"
        if fm.is_file():
            answer = fm.read_text(encoding="utf-8")
        kw = {"artifact": artifacts[0] if artifacts else None,
              "run": run, "sim": "webots",
              "self_verified": bool(cell.get("self_verified"))}
        if cell["task"] == "B2_subject_in_frame":
            init = json.loads((dest / "aabb_initial" / "aabbs.json")
                              .read_text(encoding="utf-8"))
            kw["view"] = task_support.view_evidence(
                init, aabbs, artifact=kw["artifact"])
        grader = task.grader()
        if cell["task"] == "C1_parse_error_fix":
            return grader.grade(dest, **kw)
        return grader.grade(dest, answer=answer, **kw)

    def test_every_webots_cell_regrades_to_its_committed_outcome(self):
        for cell in _cells():
            if cell["sim"] != "webots":
                continue
            with self.subTest(cell=(cell["task"], cell["condition"])):
                v = self._grade(cell)
                self.assertEqual(
                    v.outcome, cell.get("outcome"),
                    "grader replay over committed evidence disagrees with "
                    "the committed cell:\n%s" % v.summary())
                self.assertEqual(v.outcome, "PASS")


class TestOmnisimRows(unittest.TestCase):
    """The OmniSim cells' committed rows (regrading needs a live engine --
    marked here rather than silently skipped)."""

    NEEDS_ENGINE = ("re-grading an OmniSim cell repeats the phase-B "
                    "standalone recorder run and needs the engine binary; "
                    "run preregister/run_oracles.py for that. This test "
                    "checks the committed row instead.")

    def test_committed_rows_pass_and_are_attributed(self):
        for cell in _cells():
            if cell["sim"] != "omnisim":
                continue
            with self.subTest(cell=(cell["task"], cell["condition"])):
                rows = (_evidence_dir(cell) / "rows.jsonl").read_text(
                    encoding="utf-8")
                row = json.loads(
                    [ln for ln in rows.splitlines() if ln.strip()][-1])
                self.assertEqual(row["outcome"], "PASS")
                self.assertEqual(row["condition"], cell["condition"])
                self.assertEqual(row["sim"], "omnisim")
                self.assertEqual(
                    (row.get("metrics") or {}).get("tool_calls"),
                    cell["tool_calls"])
                # The scripted backend is honest in the row: the model id is
                # prefixed so it can never be mistaken for a model run.
                self.assertTrue(str((row.get("agent") or {}).get(
                    "model", "")).startswith("scripted:"))


if __name__ == "__main__":
    unittest.main()
