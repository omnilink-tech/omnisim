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

"""The pre-registration freeze: recompute every hash, fail on any drift.

``freeze_manifest.json`` is the SHA-256 manifest over the frozen content
(plan 2's SPEC-6.2.1 machinery; see ``FREEZE.md`` beside it). This file is
both the **generator** and the **guard**:

    python tests/benchmarks/agentbench/preregister/test_freeze.py --write
        (re)builds the manifest from the working tree -- legal ONLY before
        the freeze commit; afterwards a rebuild IS the campaign-voiding edit
        and must never be run casually;

    pytest tests/benchmarks/agentbench/preregister/test_freeze.py
        recomputes every hash from the tree and fails on any drift, and
        asserts the manifest has no silent omissions (every task dir and
        every grader module that exists is covered).

Hashes are computed over **LF-normalised bytes** (CRLF -> LF) so a
git-autocrlf checkout hashes identically to the LF working tree that froze
it; any real content change still changes the hash. The plan file carries
TWO hashes: the whole file (informational -- prose outside 2 may be
corrected) and 2 alone, whose drift voids the campaign (plan 2.4
no-escape). Section 2 is extracted as the byte span from the line
``## 2. The pre-registered falsification design`` up to (excluding) the line
``## 3. The phases``.
"""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
REPO = AGENTBENCH.parents[2]
sys.path.insert(0, str(AGENTBENCH.parent))

MANIFEST = HERE / "freeze_manifest.json"
PLAN = REPO / "docs" / "developer" / "agent-edge-validation-plan.md"

SECTION_2_START = "## 2. The pre-registered falsification design"
SECTION_2_END = "## 3. The phases"

# Frozen agents modules: the fixture registry + every scripted fixture/oracle
# module the coverage ledger discovers. The LLM loop scaffolding (llm.py,
# external.py, base.py) is deliberately NOT frozen -- it is instrumentation,
# not task/grader content, and FREEZE.md records that exclusion.
AGENT_MODULES = (
    "__init__.py", "null.py", "a1_fixtures_extra.py", "b1_fixtures.py",
    "b2_fixtures.py", "c1_fixtures.py", "oracle_a1.py", "oracle_b1.py",
    "oracle_b3.py", "oracle_c2.py",
)

LANE_ASSIGNMENT = {
    "lane_a": ["A1_husky_swarm_10"],
    "lane_b": ["B1_overlap_audit", "B2_subject_in_frame",
               "B3_measure_and_report", "C1_parse_error_fix",
               "C2_fall_through_floor"],
    "lane_c_frontier": [
        "five-edit iteration loop (SPEC annex task N3, re-expressed as a "
        "cross-sim outcome)",
        "snapshot/rollback experimentation",
        "structured-diagnostics triage of a deliberately broken world",
        "live spawn/delete",
    ],
}


def _norm(data: bytes) -> bytes:
    return data.replace(b"\r\n", b"\n")


#: Frozen files whose identity is their SEMANTIC content, not their bytes.
#:
#: `oracle_verdicts.json` is produced by the oracle/null gate (SPEC 7.1) --
#: the gate that MUST be re-run to prove the tasks still discriminate. Running
#: it restamps when it ran and which engine build was present, which would flip
#: the freeze red every single time the mandatory gate is executed. That
#: circularity (the gate you must run breaks the freeze you must have green) is
#: exactly what teaches people to ignore a red freeze test -- the failure SPEC
#: 8.4 exists to prevent. So these fields are dropped before hashing: a
#: re-run that REPRODUCES its verdicts leaves the freeze green, and a re-run
#: that CHANGES a verdict or a measurement still breaks it, which is the
#: property actually worth guarding.
VOLATILE_JSON_FIELDS = {
    "tests/benchmarks/agentbench/preregister/oracle_verdicts.json": (
        ("meta", "utc"),
        ("meta", "machine", "engine_build"),
    ),
}


def _strip_volatile(doc, keypaths):
    for kp in keypaths:
        node = doc
        for key in kp[:-1]:
            node = node.get(key) if isinstance(node, dict) else None
            if not isinstance(node, dict):
                break
        else:
            node.pop(kp[-1], None)
    return doc


def sha256_file(path: Path) -> str:
    """Hash a frozen file. Byte-exact, except where VOLATILE_JSON_FIELDS says
    the semantic content is what was frozen."""
    rel = _rel(path)
    drop = VOLATILE_JSON_FIELDS.get(rel)
    if drop:
        doc = _strip_volatile(
            json.loads(_norm(path.read_bytes()).decode("utf-8")), drop)
        payload = json.dumps(doc, indent=2, sort_keys=True,
                             ensure_ascii=False).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()
    return hashlib.sha256(_norm(path.read_bytes())).hexdigest()


def section_2_text() -> str:
    text = PLAN.read_text(encoding="utf-8").replace("\r\n", "\n")
    start = text.index("\n%s\n" % SECTION_2_START) + 1
    end = text.index("\n%s\n" % SECTION_2_END) + 1
    if not start < end:
        raise AssertionError("plan section 2 markers out of order")
    return text[start:end]


def sha256_section_2() -> str:
    return hashlib.sha256(section_2_text().encode("utf-8")).hexdigest()


def _rel(path: Path) -> str:
    return path.relative_to(REPO).as_posix()


def frozen_files():
    """Every frozen file, discovered from the tree (never hand-listed)."""
    # sims.py is frozen from v3: the comparator set IS design content, and a
    # comparator quietly appearing or vanishing between runs would change what
    # the leaderboard means without changing any task.
    files = [PLAN, AGENTBENCH / "SPEC.md", AGENTBENCH / "sims.py"]
    tasks_dir = AGENTBENCH / "tasks"
    for task in sorted(p for p in tasks_dir.iterdir() if p.is_dir()
                       and (p / "meta.json").is_file()):
        for sub in sorted(task.rglob("*")):
            if sub.is_file() and "__pycache__" not in sub.parts:
                files.append(sub)
    for p in sorted((AGENTBENCH / "graders").glob("*.py")):
        if not p.name.startswith("test"):
            files.append(p)
    for name in AGENT_MODULES:
        files.append(AGENTBENCH / "agents" / name)
    files.extend(sorted((AGENTBENCH / "runner" / "manifests").glob("*.json")))
    files.extend(sorted((AGENTBENCH / "runner" / "scripts")
                        .glob("oracle_*.json")))
    files.append(HERE / "oracle_verdicts.json")
    return files


def build_manifest() -> dict:
    return {
        "meta": {
            "schema": "agentbench/freeze/v1",
            "freeze_version": 3,
            "frozen_utc": "2026-08-09",
            "suite": "agenticsimbench/v0.3",
            "machine_id": "9722d23d12a3",
            "engine_build": "518a335e",
            "supersedes": {
                "freeze_version": 2,
                "manifest_sha256_lf_normalised":
                    "4661c893aae69a5f1577c7b784a781fa"
                    "95edb56268f9a72bcb23ed6f907be639",
                "reason": "SUITE VERSION BUMP with 35 scored runs already "
                          "recorded, so this is NOT a legal pre-run edit: "
                          "the model is repinned (claude-fable-5 -> "
                          "claude-opus-5), the wall-clock budget is capped "
                          "at 15 min/cell (was an unenforced per-task value "
                          "behind a 45 min global), and the comparator set "
                          "expands from 2 to 7 primary + 3 extended. The "
                          "v2-scored agentbench/v0 grid is preserved under "
                          "its own suite id and is never pooled with "
                          "agenticsimbench/v0.3 rows. Also records the "
                          "post-freeze drift of 5 frozen files (see "
                          "FREEZE.md, freeze v3).",
                "scored_runs_at_amendment_time": 35,
            },
            "generator": "preregister/test_freeze.py --write",
            "hash_rule": (
                "sha256 over LF-normalised bytes (CRLF -> LF); for the paths "
                "in volatile_json_fields, sha256 over the JSON re-serialised "
                "(indent=2, sort_keys) with those fields dropped, so re-running "
                "the mandatory oracle gate does not flip the freeze red merely "
                "by restamping when it ran"),
            "volatile_json_fields": {
                k: ["/".join(kp) for kp in v]
                for k, v in VOLATILE_JSON_FIELDS.items()},
            "section_2_rule": (
                "byte span of the LF-normalised plan file from the line "
                "'%s' up to (excluding) the line '%s', UTF-8 encoded"
                % (SECTION_2_START, SECTION_2_END)),
            "authority": "FREEZE.md (same directory)",
        },
        "plan": {
            "path": _rel(PLAN),
            "sha256_whole_file": sha256_file(PLAN),
            "sha256_section_2": sha256_section_2(),
        },
        "lane_assignment": LANE_ASSIGNMENT,
        "files": {_rel(p): sha256_file(p) for p in frozen_files()},
    }


def _manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


class TestFreezeIntegrity(unittest.TestCase):
    def test_manifest_exists(self):
        self.assertTrue(MANIFEST.is_file(),
                        "freeze_manifest.json is missing -- the campaign "
                        "has no frozen pre-registration")

    def test_every_hash_matches_the_tree(self):
        """Any drift in a frozen file voids the campaign (plan 2.4)."""
        doc = _manifest()
        for rel, want in sorted(doc["files"].items()):
            with self.subTest(file=rel):
                p = REPO / rel
                self.assertTrue(p.is_file(), "frozen file missing: %s" % rel)
                self.assertEqual(
                    sha256_file(p), want,
                    "FROZEN FILE DRIFTED: %s -- plan 2.4: a post-freeze "
                    "change to a hashed file voids the campaign (suite "
                    "version bump + full re-run)" % rel)

    def test_plan_section_2_hash_matches(self):
        doc = _manifest()
        self.assertEqual(sha256_section_2(),
                         doc["plan"]["sha256_section_2"],
                         "plan section 2 (the pre-registered design) "
                         "drifted -- this voids the campaign (plan 2.4)")
        self.assertEqual(sha256_file(PLAN),
                         doc["plan"]["sha256_whole_file"],
                         "the plan file drifted outside section 2; "
                         "re-freeze deliberately or revert")

    def test_manifest_covers_every_task_dir(self):
        """No silent omissions: every task that exists is frozen."""
        doc = _manifest()
        frozen = set(doc["files"])
        tasks_dir = AGENTBENCH / "tasks"
        for task in sorted(p for p in tasks_dir.iterdir() if p.is_dir()
                           and (p / "meta.json").is_file()):
            with self.subTest(task=task.name):
                self.assertIn(_rel(task / "meta.json"), frozen)
                self.assertIn(_rel(task / "prompt.txt"), frozen)
                for sub in task.rglob("*"):
                    if sub.is_file() and "__pycache__" not in sub.parts:
                        self.assertIn(_rel(sub), frozen,
                                      "task file not frozen: %s" % sub)

    def test_manifest_covers_every_grader(self):
        doc = _manifest()
        frozen = set(doc["files"])
        for p in sorted((AGENTBENCH / "graders").glob("*.py")):
            if p.name.startswith("test"):
                continue
            with self.subTest(grader=p.name):
                self.assertIn(_rel(p), frozen,
                              "grader module not frozen: %s" % p.name)

    def test_manifest_covers_manifests_scripts_and_verdicts(self):
        doc = _manifest()
        frozen = set(doc["files"])
        for p in sorted((AGENTBENCH / "runner" / "manifests")
                        .glob("*.json")):
            self.assertIn(_rel(p), frozen)
        for p in sorted((AGENTBENCH / "runner" / "scripts")
                        .glob("oracle_*.json")):
            self.assertIn(_rel(p), frozen)
        self.assertIn(_rel(HERE / "oracle_verdicts.json"), frozen)
        self.assertIn(_rel(AGENTBENCH / "SPEC.md"), frozen)

    def test_lane_assignment_is_the_frozen_one(self):
        doc = _manifest()
        self.assertEqual(doc["lane_assignment"], LANE_ASSIGNMENT)
        # Lane B is exactly the verdicts module's task tuple -- the lane
        # test instrument and the freeze must name the same five tasks.
        from agentbench.preregister import verdicts as verdicts_mod
        self.assertEqual(tuple(doc["lane_assignment"]["lane_b"]),
                         verdicts_mod.TASKS)

    def test_freeze_md_exists_and_names_the_gates(self):
        text = (HERE / "FREEZE.md").read_text(encoding="utf-8")
        for needle in ("2026-08-01", "9722d23d12a3", "518a335e",
                       "countersign", "credential", "voids the campaign",
                       # freeze v2 (plan revision 3): the supersession and
                       # the v1 audit-trail hash must stay on record
                       "Freeze v2", "supersedes v1",
                       "76c2739a5cbebc0619936a5f62c2b06f"
                       "3fa02feff04088de010a44588feefebd",
                       # freeze v3: the suite bump, its legality basis (the
                       # scored-run count), the v2 audit hash, and the
                       # recorded post-freeze drift must all stay on record
                       "Freeze v3", "supersedes v2",
                       "agenticsimbench/v0.3", "claude-opus-5",
                       "35 scored cells existed at amendment time",
                       "4661c893aae69a5f1577c7b784a781fa"
                       "95edb56268f9a72bcb23ed6f907be639"):
            self.assertIn(needle, text)

    def test_manifest_records_freeze_v3_supersession(self):
        """The manifest itself carries the audit trail back to freeze v2."""
        meta = _manifest()["meta"]
        self.assertEqual(meta.get("freeze_version"), 3)
        self.assertEqual(meta.get("suite"), "agenticsimbench/v0.3")
        sup = meta.get("supersedes") or {}
        self.assertEqual(sup.get("freeze_version"), 2)
        self.assertEqual(
            sup.get("manifest_sha256_lf_normalised"),
            "4661c893aae69a5f1577c7b784a781fa"
            "95edb56268f9a72bcb23ed6f907be639")
        # The count is what makes v3 a suite bump rather than a pre-run edit
        # -- an amendment that cannot state it has not been thought through.
        self.assertEqual(sup.get("scored_runs_at_amendment_time"), 35)

    def test_budget_ceiling_binds_every_task(self):
        """No task may declare a budget above the campaign cost ceiling."""
        import agentbench.tasks as tasks_mod
        # The VALUE is policy and moves (900 -> 1800 on 2026-08-09 when
        # every robotics cell exhausted 15 min). What must hold is that
        # a ceiling exists, is sane, and actually bounds every task.
        self.assertGreater(tasks_mod.TASK_HARD_CEILING_S, 0)
        for tid, task in sorted(tasks_mod.discover().items()):
            self.assertLessEqual(
                task.timeout_s, tasks_mod.TASK_HARD_CEILING_S,
                "%s escapes the ceiling" % tid)
            self.assertEqual(
                task.timeout_s, min(3 * task.par_s,
                                    tasks_mod.TASK_HARD_CEILING_S),
                "%s does not follow min(3 x par, ceiling)" % tid)

    def test_the_benchmark_model_is_pinned_in_code(self):
        """The model is the suite's, never the local CLI's default."""
        from agentbench.cc_lane import run_cc_cell
        self.assertEqual(run_cc_cell.DEFAULT_MODEL, "claude-opus-5")
        self.assertEqual(run_cc_cell.SUITE, "agenticsimbench/v0.3")


def _write():
    doc = build_manifest()
    MANIFEST.write_text(json.dumps(doc, indent=2, sort_keys=False) + "\n",
                        encoding="utf-8")
    print("wrote %s (%d files)" % (MANIFEST, len(doc["files"])))


if __name__ == "__main__":
    if "--write" in sys.argv[1:]:
        _write()
    else:
        unittest.main()
