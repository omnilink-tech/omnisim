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

"""The Lane B oracle scripts: freshness, determinism, tool-set legality.

All WSL-free. The scripts' *live* behaviour (grader PASS on real runs) is
covered by ``test_oracle_verdicts.py`` over the committed evidence.
"""

import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

from agentbench.preregister import gen_oracle_scripts as gen  # noqa: E402
from agentbench.runner.backends.scripted import (  # noqa: E402
    ScriptedBackend, load_script)
from agentbench.runner.tools.shell import SHELL_TOOL_DEFS  # noqa: E402

SCRIPTS = HERE.parent / "runner" / "scripts"
SHELL_TOOL_NAMES = {name for name, _d, _s in SHELL_TOOL_DEFS}


def script_paths():
    return sorted(SCRIPTS.glob("oracle_b*.json")) + sorted(
        SCRIPTS.glob("oracle_c*.json"))


class TestFreshness(unittest.TestCase):
    def test_committed_scripts_match_regeneration(self):
        """The committed scripts are exactly what the generator emits.

        Guards the same drift the coverage table guards: an edit to a task
        world that silently invalidates an oracle's baked content must fail
        here, not at run time.
        """
        for name in gen.GENERATORS:
            with self.subTest(script=name):
                committed = (SCRIPTS / name).read_text(encoding="utf-8")
                self.assertEqual(committed, gen.render(name),
                                 "%s is stale: re-run preregister/"
                                 "gen_oracle_scripts.py" % name)

    def test_generator_covers_all_five_tasks_on_both_sims(self):
        names = set(gen.GENERATORS)
        for short in ("b1", "b2", "b3", "c1", "c2"):
            for sim in ("omnisim", "webots"):
                self.assertIn("oracle_%s_%s.json" % (short, sim), names)


class TestScriptShape(unittest.TestCase):
    def test_loads_under_the_runner_schema(self):
        for p in script_paths():
            with self.subTest(script=p.name):
                s = load_script(p)
                self.assertEqual(s["name"], p.stem)
                self.assertTrue(s["turns"])

    def test_minimality_reasoning_documented(self):
        for p in script_paths():
            with self.subTest(script=p.name):
                s = json.loads(p.read_text(encoding="utf-8"))
                self.assertTrue(s.get("minimality"),
                                "%s carries no minimality reasoning; the "
                                "oracle counts define the plan-2.2 verdicts "
                                "and an unreasoned count is unreviewable"
                                % p.name)

    def test_only_shell_tools_are_called(self):
        """Every call is a shell-set tool, so the same script is legal under
        BOTH conditions of its simulator (shell is a subset of shell+tools).
        This is the structural fact behind bridge==shell oracle counts; if a
        future oracle adds a bridge/harness verb, this test names the cells
        that can no longer share a script."""
        for p in script_paths():
            with self.subTest(script=p.name):
                s = load_script(p)
                called = {c["name"] for t in s["turns"]
                          for c in t.get("tool_calls") or []}
                self.assertLessEqual(called, SHELL_TOOL_NAMES)

    def test_final_turn_commits_an_answer(self):
        for p in script_paths():
            with self.subTest(script=p.name):
                s = load_script(p)
                last = s["turns"][-1]
                self.assertFalse(last.get("tool_calls"),
                                 "the last turn must be the final message")
                self.assertTrue((last.get("text") or "").strip())


class TestReplayDeterminism(unittest.TestCase):
    VARS = {"SCRATCH": "X:/scratch", "REPO": "X:/repo",
            "RUN_DIR": "X:/run", "HUSKY_URDF": "X:/h.urdf",
            "HARNESS_PORT": "7100", "SUPERVISOR_PORT": "7101",
            "HARNESS_URL": "http://127.0.0.1:7100", "LOG_PATH": "X:/log"}

    @staticmethod
    def _drain(backend):
        out = []
        while backend.remaining:
            turn = backend.turn("", [], [])
            out.append((turn.text,
                        [(c.name, json.dumps(c.arguments, sort_keys=True))
                         for c in turn.tool_calls],
                        turn.stop_reason))
        return out

    def test_two_replays_are_identical(self):
        for p in script_paths():
            with self.subTest(script=p.name):
                a = self._drain(ScriptedBackend(p, dict(self.VARS)))
                b = self._drain(ScriptedBackend(p, dict(self.VARS)))
                self.assertEqual(a, b)

    def test_no_unresolved_placeholders_after_substitution(self):
        for p in script_paths():
            with self.subTest(script=p.name):
                for text, calls, _stop in self._drain(
                        ScriptedBackend(p, dict(self.VARS))):
                    self.assertNotIn("{{", text)
                    for _n, args in calls:
                        self.assertNotIn("{{", args)


class TestOracleCallCounts(unittest.TestCase):
    # The counts the verdicts rest on, pinned: a change here changes the
    # plan-2.1/2.2 arithmetic and must be a deliberate re-measure.
    # B1 is 2 on both arms since the 2026-08-01 disclosure strip: the world
    # no longer names its overlapping pair, so the oracle must read BOTH the
    # scene (poses) and the robot model's geometry source (footprint).
    EXPECTED = {
        "oracle_b1_omnisim": 2, "oracle_b1_webots": 2,
        "oracle_b2_omnisim": 2, "oracle_b2_webots": 2,
        "oracle_b3_omnisim": 1, "oracle_b3_webots": 1,
        "oracle_c1_omnisim": 3, "oracle_c1_webots": 3,
        "oracle_c2_omnisim": 3, "oracle_c2_webots": 5,
    }

    def test_scripted_call_counts(self):
        for p in script_paths():
            s = load_script(p)
            n = sum(len(t.get("tool_calls") or []) for t in s["turns"])
            self.assertEqual(n, self.EXPECTED[p.stem],
                             "%s call count drifted" % p.name)


class TestWorldEditsStillAnchor(unittest.TestCase):
    """The oracle edits are anchored string replacements over the committed
    task worlds; if a world drifts, the generator must fail loudly rather
    than emit a stale fix."""

    def test_fix_builders_run(self):
        for sim in ("omnisim", "webots"):
            gen.b2_fixed(sim)
            fixed = gen.c1_fixed(sim)
            self.assertNotIn("Soild", fixed)
            self.assertEqual(fixed.count("}"), fixed.count("{"))
            c2 = gen.c2_fixed(sim)
            self.assertIn("boundingObject Box", c2.split("DEF FLOOR")[1])


if __name__ == "__main__":
    unittest.main()
