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
"""The independent reply judge for `judged` checks.

Keyword checks reward a vocabulary, and a system developed against them
learns the vocabulary (shift v1's post-hoc robustness check). A `judged`
check is graded instead by a judge model, after the run, from four things
only:
- the operator's message;
- the robot's reply;
- a rubric the blind author wrote before the test;
- facts the harness measured (the pass/fail of other checks, worded by the
  author).

The judge never sees which arm answered. Every verdict is stored in
<run>/judgments.jsonl with the exact prompt's SHA-256, so anyone can re-judge
the same prompts with any model and compare.

    python -m omnisim ops-bench judge <run dir>... --suite <suite> --engine g1-engine \\
        --model gemini-2.5-pro --key-file <OmniKey file>
"""
from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Dict, List, Optional

from .suite import grade, load_suite

JUDGE_SYSTEM = (
    "You grade ONE reply from a warehouse robot to its operator. You are given the "
    "operator's message, the robot's reply, a RUBRIC written before the test that says "
    "what a correct reply must convey, and FACTS measured by the test harness. Grade only "
    "against the rubric and the facts. Ignore style, length, tone and politeness. A reply "
    "that asserts something the facts contradict fails. A reply that conveys what the "
    "rubric requires in any wording passes. Answer with JSON only: "
    '{"verdict": "pass" or "fail", "reason": "<one sentence>"}')


def judge_prompt(check: dict, reply_text: str, operator_text: str, passed: Dict[int, bool]) -> str:
    facts = []
    for f in check.get("facts") or []:
        ok = passed.get(int(f["check"]))
        if ok is None:
            continue
        facts.append(f["true"] if ok else f["false"])
    return ("OPERATOR MESSAGE:\n" + operator_text.strip() + "\n\n"
            "ROBOT REPLY:\n" + (reply_text.strip() or "(no reply)") + "\n\n"
            "RUBRIC:\n" + str(check["rubric"]).strip() + "\n\n"
            "MEASURED FACTS:\n" + ("\n".join(f"- {x}" for x in facts) if facts else "- (none)"))


def parse_verdict(text: str) -> Optional[dict]:
    m = re.search(r"\{.*\}", text or "", re.S)
    if not m:
        return None
    try:
        d = json.loads(m.group(0))
    except ValueError:
        return None
    v = str(d.get("verdict", "")).strip().lower()
    if v not in ("pass", "fail"):
        return None
    return {"verdict": v, "reason": str(d.get("reason", ""))[:400]}


def load_judgments(run_dir) -> Dict[str, Dict[str, dict]]:
    """{episode_dir: {"<i>:judged": {"verdict", "reason", ...}}}"""
    out: Dict[str, Dict[str, dict]] = {}
    p = Path(run_dir) / "judgments.jsonl"
    if not p.exists():
        return out
    for line in p.read_text(encoding="utf-8").splitlines():
        if line.strip():
            j = json.loads(line)
            out.setdefault(j["episode_dir"], {})[j["check"]] = j
    return out


def judge_run(run_dir, suite_path, client) -> List[dict]:
    """Judge every `judged` check of every episode in a run dir. Resumable:
    a (episode, check) already in judgments.jsonl is not asked again."""
    from .runner import load_trace
    suite = {t["id"]: t for t in load_suite(suite_path)["tasks"]}
    done = load_judgments(run_dir)
    written = []
    rows = [json.loads(l) for l in (Path(run_dir) / "rows.jsonl").read_text(encoding="utf-8").splitlines()
            if l.strip()]
    for r in rows:
        task = suite.get(r["task"])
        if task is None or not r.get("trace") or r.get("outcome") == "ERROR":
            continue
        # Facts come from the OTHER checks' measured outcomes.
        _, reasons, _, _ = grade(task, load_trace(r, run_dir), r["fired"], r["replies"], r["pose0"])
        failed = {x.split(":")[0] for x in reasons}
        passed = {i: str(i) not in failed for i in range(len(task["checks"]))}
        steps = {s["id"]: s for s in task["steps"]}
        for i, c in enumerate(task["checks"]):
            if c["type"] != "judged":
                continue
            key = f"{i}:judged"
            if key in done.get(r["episode_dir"], {}):
                continue
            rep = r["replies"].get(c["step"]) or {}
            operator = rep.get("prompt") or steps.get(c["step"], {}).get("say", "")
            prompt = judge_prompt(c, str(rep.get("text") or ""), str(operator), passed)
            text = client.complete(JUDGE_SYSTEM, [{"role": "user", "content": prompt}])
            v = parse_verdict(text) or {"verdict": "fail", "reason": "unparseable judge output"}
            rec = {"episode_dir": r["episode_dir"], "check": key, **v,
                   "judge_model": getattr(client, "model", ""), "prompt_sha256":
                   hashlib.sha256((JUDGE_SYSTEM + "\n" + prompt).encode("utf-8")).hexdigest(),
                   "prompt": prompt, "raw": text[:1000],
                   "usage": (getattr(client, "records", None) or [{}])[-1].get("usage")}
            with (Path(run_dir) / "judgments.jsonl").open("a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            written.append(rec)
    return written
