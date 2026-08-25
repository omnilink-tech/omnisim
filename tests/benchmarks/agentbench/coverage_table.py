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

"""The red-evidence coverage table (agent-edge-validation-plan.md 5.5).

    python tests/benchmarks/agentbench/coverage_table.py            # write
    python tests/benchmarks/agentbench/coverage_table.py --check    # verify

One row per assertion id across every grader in the suite. The standing rule
it enforces, verbatim from the plan:

    No assertion may enter a scored campaign until it has been observed
    FAILING on a deliberately wrong artifact, with that negative fixture
    named in the assertion's record. [...] an assertion whose only red
    evidence is the ``null`` agent counts as having none.

So a row is **validated** only when a committed verdict from a non-null
scripted fixture shows the assertion red. ``expect_failures`` declarations are
*targeting*, not evidence; the ``null`` agent's all-red verdict is evidence of
nothing (that is exactly how A1.3 hid); and a core-unit red on a synthetic
bundle is recorded but does not count as end-to-end validation.

Discovery -- everything is found, nothing is hand-listed, so the B1/B2/C1
lanes appear automatically when their pieces land:

  * **assertion universe**: every ``graders/*_core.py`` module -- its ``TASK``
    plus ``_ALL`` when exported, else the ``v.add("<ID>", ...)`` call sites in
    its source (the same serve-from-the-code trick /capabilities uses);
    assertion ids seen in committed verdicts are unioned in as a backstop.
  * **fixture targeting**: ``agentbench.agents.REGISTRY``, plus the
    module-level REGISTRY-shaped exports of every ``agents/*_fixtures*.py``
    that is not yet merged into the package registry (the b1/b2/c1
    convention), package registry winning on overlap.
  * **observed reds**: ``phase0_validation/*.verdict.json`` -- graded output
    only, never written by hand. Filenames are ``<agent>.verdict.json`` or
    ``<task>.<agent>.verdict.json``; the task always comes from the JSON.
  * **extra evidence**: ``COVERAGE_EXTRA`` module exports, for red evidence
    that is structurally not reachable end-to-end (A1.10's core-unit red).

Outputs: ``phase0_validation/COVERAGE.md`` + ``coverage.json`` -- committed
evidence, deliberately not under the gitignored ``results/``.
"""

from __future__ import annotations

import argparse
import importlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))

GRADERS_DIR = HERE / "graders"
AGENTS_DIR = HERE / "agents"
PHASE0_DIR = HERE / "phase0_validation"

NULL_AGENTS = ("null",)

_VADD = re.compile(r'v\.add\(\s*"([A-Z]\d+\.\d+)"')


# --- discovery --------------------------------------------------------------


def discover_universe(graders_dir=GRADERS_DIR):
    """{task_id: {assertion_id: what}} from the grader cores themselves."""
    out = {}
    for path in sorted(Path(graders_dir).glob("*_core.py")):
        if path.name.startswith("test"):
            continue
        mod = importlib.import_module("agentbench.graders." + path.stem)
        task = getattr(mod, "TASK", None)
        if not task:
            continue
        table = out.setdefault(task, {})
        for aid, what in getattr(mod, "_ALL", []) or []:
            table[aid] = what
        # Cores without an _ALL export (b3_core): the v.add call sites are
        # the ground truth for which assertions exist.
        for aid in _VADD.findall(path.read_text(encoding="utf-8")):
            table.setdefault(aid, "")
    return out


def discover_registry(agents_dir=AGENTS_DIR):
    """The merged fixture registry: package REGISTRY + unmerged *_fixtures*
    module exports (which lose on key overlap), plus every COVERAGE_EXTRA."""
    agents_pkg = importlib.import_module("agentbench.agents")
    registry = {}
    extra = {}
    for path in sorted(Path(agents_dir).glob("*_fixtures*.py")):
        try:
            mod = importlib.import_module("agentbench.agents." + path.stem)
        except Exception:  # noqa: BLE001  (a sibling lane's WIP must not
            continue       # take the table down)
        registry.update(getattr(mod, "REGISTRY", {}) or {})
        extra.update(getattr(mod, "COVERAGE_EXTRA", {}) or {})
    registry.update(agents_pkg.REGISTRY)
    return registry, extra


def load_verdicts(phase0_dir=PHASE0_DIR):
    """[(agent_name, filename, verdict_dict)] from the committed evidence."""
    out = []
    for path in sorted(Path(phase0_dir).glob("*.verdict.json")):
        stem = path.name[:-len(".verdict.json")]
        agent = stem.rsplit(".", 1)[-1]     # "<task>.<agent>" or "<agent>"
        try:
            v = json.loads(path.read_text(encoding="utf-8"))
        except ValueError:
            continue
        out.append((agent, path.name, v))
    return out


# --- the table (pure function of its inputs) --------------------------------


def compute_coverage(universe, registry, verdicts, extra=None,
                     null_agents=NULL_AGENTS):
    """One row per (task, assertion). Pure; see module docstring for rules."""
    extra = extra or {}

    # Include assertions only seen in committed verdicts (universe backstop).
    seen = {}
    for agent, fname, v in verdicts:
        task = v.get("task")
        for aid, detail in (v.get("assertions") or {}).items():
            what = detail.get("what", "") if isinstance(detail, dict) else ""
            seen.setdefault(task, {}).setdefault(aid, what)
    merged = {}
    for src in (universe, seen):
        for task, table in src.items():
            dst = merged.setdefault(task, {})
            for aid, what in table.items():
                if aid not in dst or (what and not dst[aid]):
                    dst[aid] = what

    agents_by_task = {}
    for (t, agent) in registry:
        agents_by_task.setdefault(t, set()).add(agent)

    rows = []
    for task in sorted(merged):
        for aid in sorted(merged[task], key=_assertion_key):
            targeted = sorted(
                agent for (t, agent), entry in registry.items()
                if t == task and agent not in null_agents
                and aid in (entry.get("expect_failures") or ()))
            observed, null_hits, non_fixture = [], [], []
            for agent, fname, v in verdicts:
                if v.get("task") != task:
                    continue
                if aid in (v.get("failed_assertions") or []):
                    rec = {"agent": agent, "file": fname,
                           "outcome": v.get("outcome")}
                    if agent in null_agents:
                        null_hits.append(rec)
                    elif agent in agents_by_task.get(task, ()):
                        observed.append(rec)
                    else:
                        # A committed verdict from something that is NOT a
                        # registered scripted fixture (e.g. the A1.10
                        # core-level regeneration). Recorded, never counted:
                        # the rule demands a red on a wrong ARTIFACT.
                        non_fixture.append(rec)
            ex = extra.get((task, aid))
            validated = bool(observed)
            if validated:
                reason = None
            elif ex is not None:
                reason = ("no end-to-end red: %s"
                          % ex.get("reason", "see extra evidence"))
            elif null_hits:
                reason = ("only red evidence is the null agent, which "
                          "validates nothing (plan 5.5)")
            else:
                reason = "no committed red verdict from any fixture"
            rows.append({
                "task": task,
                "assertion": aid,
                "what": merged[task][aid],
                "targeted_fixtures": targeted,
                "observed_red": observed,
                "null_only_red": [r["file"] for r in null_hits],
                "non_fixture_red": non_fixture,
                "extra_evidence": ex,
                "validated": validated,
                "unvalidated_reason": reason,
            })
    summary = {
        "assertions": len(rows),
        "validated": sum(1 for r in rows if r["validated"]),
        "unvalidated": sum(1 for r in rows if not r["validated"]),
        "by_task": {},
    }
    for r in rows:
        t = summary["by_task"].setdefault(
            r["task"], {"assertions": 0, "validated": 0})
        t["assertions"] += 1
        t["validated"] += int(r["validated"])
    return {"rule": ("an assertion is validated only when a committed "
                     "verdict from a non-null scripted fixture shows it "
                     "red; null-only red evidence counts as none "
                     "(agent-edge-validation-plan.md 5.5)"),
            "rows": rows, "summary": summary}


def _assertion_key(aid):
    m = re.match(r"([A-Z]+)(\d+)\.(\d+)$", aid)
    return (m.group(1), int(m.group(2)), int(m.group(3))) if m else (aid, 0, 0)


# --- rendering --------------------------------------------------------------


def render_markdown(cov):
    s = cov["summary"]
    lines = [
        "# Red-evidence coverage table",
        "",
        "Generated by `coverage_table.py` -- do not edit by hand; regenerate "
        "after adding a fixture or committing a verdict.",
        "",
        "> %s" % cov["rule"],
        "",
        "**%d/%d assertions validated** (%d unvalidated)."
        % (s["validated"], s["assertions"], s["unvalidated"]),
        "",
        "| assertion | task | what | targeted fixture(s) | observed red "
        "(verdict file) | status |",
        "|---|---|---|---|---|---|",
    ]
    for r in cov["rows"]:
        if r["validated"]:
            status = "validated"
        else:
            status = "**UNVALIDATED** -- %s" % r["unvalidated_reason"]
        observed = "; ".join("`%s` (%s)" % (o["file"], o["agent"])
                             for o in r["observed_red"]) or "--"
        if not r["observed_red"] and r["null_only_red"]:
            observed = "null only: %s" % ", ".join(
                "`%s`" % f for f in r["null_only_red"])
        for nf in r.get("non_fixture_red") or []:
            observed += (" | non-fixture: `%s` (not counted)" % nf["file"])
        ex = r.get("extra_evidence")
        if ex:
            observed += (" | %s: `%s`" % (ex.get("kind", "extra"),
                                          ex.get("fixture", "?")))
        lines.append("| %s | %s | %s | %s | %s | %s |" % (
            r["assertion"], r["task"], r["what"] or "--",
            ", ".join("`%s`" % f for f in r["targeted_fixtures"]) or "--",
            observed, status))
    lines += [
        "",
        "Legend: *targeted fixture(s)* = scripted negative agents whose "
        "`expect_failures` declares this assertion; *observed red* = "
        "committed, grader-produced verdicts in `phase0_validation/` in "
        "which the assertion actually failed. A `core-unit` entry is red "
        "evidence produced by the real grader core on a synthetic evidence "
        "bundle (no engine run) and does **not** count as end-to-end "
        "validation.",
        "",
    ]
    return "\n".join(lines)


def build(phase0_dir=PHASE0_DIR):
    universe = discover_universe()
    registry, extra = discover_registry()
    verdicts = load_verdicts(phase0_dir)
    return compute_coverage(universe, registry, verdicts, extra)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="verify the committed COVERAGE.md/coverage.json "
                         "match a fresh regeneration (exit 1 on drift)")
    ap.add_argument("--out-dir", default=str(PHASE0_DIR))
    args = ap.parse_args(argv)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cov = build(out_dir)
    md = render_markdown(cov)
    js = json.dumps(cov, indent=2, default=str) + "\n"

    md_path = out_dir / "COVERAGE.md"
    js_path = out_dir / "coverage.json"
    if args.check:
        stale = []
        if not md_path.exists() or md_path.read_text(encoding="utf-8") != md:
            stale.append(str(md_path))
        if not js_path.exists() or js_path.read_text(encoding="utf-8") != js:
            stale.append(str(js_path))
        if stale:
            print("STALE (regenerate with coverage_table.py): %s"
                  % ", ".join(stale))
            return 1
        print("coverage table up to date: %d/%d validated"
              % (cov["summary"]["validated"], cov["summary"]["assertions"]))
        return 0

    md_path.write_text(md, encoding="utf-8")
    js_path.write_text(js, encoding="utf-8")
    print("wrote %s and %s" % (md_path, js_path))
    print("%d/%d assertions validated; unvalidated:"
          % (cov["summary"]["validated"], cov["summary"]["assertions"]))
    for r in cov["rows"]:
        if not r["validated"]:
            print("  %-6s %-24s %s"
                  % (r["assertion"], r["task"], r["unvalidated_reason"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
