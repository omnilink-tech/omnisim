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
"""Cut a short probe out of a seen shift for every check OmniLink failed.

A 30-minute shift is the wrong unit for iterating on one failure. This keeps,
for each failed check:
  - the shift's opening (who is who, the stations, the standing rules);
  - a few steps of lead-in before the check's window;
  - the steps the check watches, and a short run-out after them;
and re-times the timeline so chat-only steps follow each other quickly while
any step that asks for motion keeps its original spacing. The check itself is
copied unchanged (same step ids), so the probe grades exactly what failed.

A probe is a FAST REPRODUCER, not a result: the robot starts at its home pose,
and context outside the kept steps is gone. Confirm every fix on the full
shift. Seen material only -- never point this at a sealed holdout.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

OPENING_S = 120.0       # steps before this (original at_s) set up people, stations and rules
LEAD_IN_STEPS = 4       # steps kept before the check's window
RUN_OUT_STEPS = 6       # steps kept after the window for "... to end" checks
CHAT_GAP_S = 8.0        # spacing between two chat-only steps after re-timing
PROBE_TAIL_S = 90.0     # time left after the last step for the check to pass


def _motion_step(step: Dict[str, Any]) -> bool:
    """Does the reference answer to this step move the robot (or is it a fixture)?"""
    if step.get("fixture"):
        return True
    tools = [c.get("tool") for c in step.get("calibration") or []]
    return any(t and t not in ("_reply",) for t in tools)


def failed_checks(rows: List[Dict[str, Any]]) -> Dict[int, List[str]]:
    """{check index: [reasons]} over OmniLink's rows."""
    out: Dict[int, List[str]] = {}
    for r in rows:
        if r.get("arm") != "omnilink":
            continue
        for reason in r.get("reasons") or []:
            head = reason.split(":")[0]
            if head.isdigit():
                out.setdefault(int(head), []).append(reason)
    return out


def _referenced_steps(check: Dict[str, Any], step_ids: Set[str]) -> List[str]:
    refs = []
    for v in check.values():
        if isinstance(v, str) and v in step_ids:
            refs.append(v)
        elif isinstance(v, list):
            refs += [x for x in v if isinstance(x, str) and x in step_ids]
    return refs


def _retime(steps: List[Dict[str, Any]], originals: Dict[str, float],
            order: Dict[str, int]) -> List[Dict[str, Any]]:
    """New at_s values. Two steps that were NEIGHBOURS in the shift keep their
    gap when either one asks for motion (the robot needs that time); every
    other gap -- chat-only neighbours, or a jump over dropped steps, during
    which nothing happens any more -- becomes CHAT_GAP_S."""
    out, t, prev_orig, prev = [], 0.0, None, None
    for s in steps:
        s = json.loads(json.dumps(s))
        when = s.get("when") or {}
        if "at_s" in when:
            orig = originals[s["id"]]
            if prev_orig is None:
                t = min(orig, 3.0)
            else:
                gap = max(0.0, orig - prev_orig)
                adjacent = prev is not None and order[s["id"]] == order[prev["id"]] + 1
                keep = adjacent and (_motion_step(s) or _motion_step(prev))
                t += gap if keep else min(gap, CHAT_GAP_S)
            when["at_s"] = round(t, 1)
            prev_orig = orig
        prev = s
        out.append(s)
    return out


def build_probe(task: Dict[str, Any], index: int) -> Optional[Dict[str, Any]]:
    steps = task["steps"]
    ids = [s["id"] for s in steps]
    pos = {sid: i for i, sid in enumerate(ids)}
    check = task["checks"][index]
    refs = _referenced_steps(check, set(ids))
    if not refs:
        return None
    first, last = min(pos[r] for r in refs), max(pos[r] for r in refs)
    if any(v == "end" for v in check.values()):
        last = min(len(steps) - 1, last + RUN_OUT_STEPS)
    originals = {s["id"]: float((s.get("when") or {}).get("at_s", 0.0)) for s in steps}
    keep: Set[int] = {i for i, s in enumerate(steps)
                      if "at_s" in (s.get("when") or {}) and originals[s["id"]] < OPENING_S}
    keep |= set(range(max(0, first - LEAD_IN_STEPS), last + 1))
    # A distance-triggered step needs the step before it, which started the
    # motion -- and that one may be distance-triggered too, so follow the chain.
    changed = True
    while changed:
        changed = False
        for i in sorted(keep):
            if "moved" in (steps[i].get("when") or {}) and i > 0 and i - 1 not in keep:
                keep.add(i - 1)
                changed = True
    kept = _retime([steps[i] for i in sorted(keep)], originals, pos)
    # A check that runs "to end" now ends PROBE_TAIL_S after the probe's last step.
    probe_check = json.loads(json.dumps(check))
    return {
        **{k: v for k, v in task.items() if k not in ("id", "steps", "checks", "tail_s")},
        "id": f"{task['id']}__c{index}",
        "family": f"failure_probe/{check.get('type')}",
        "steps": kept,
        "checks": [probe_check],
        "tail_s": max(float(task.get("tail_s") or 0.0), PROBE_TAIL_S),
        "probe_of": {"task": task["id"], "check": index, "original_steps": len(steps), "kept_steps": len(kept)},
    }


def build_suite(shift_suite: Path, rows: List[Dict[str, Any]], skip: Set[int] = frozenset()) -> Dict[str, Any]:
    suite = json.loads(shift_suite.read_text(encoding="utf-8"))
    task = suite["tasks"][0]
    probes = []
    for index, reasons in sorted(failed_checks(rows).items()):
        if index in skip or index >= len(task["checks"]):
            continue
        if task["checks"][index].get("type") == "judged":
            continue  # needs the judge pass; reproduce on the full shift
        p = build_probe(task, index)
        if p:
            p["probe_of"]["reasons"] = reasons
            probes.append(p)
    return {"schema": suite.get("schema"), "split": "development",
            "provenance": {"from_suite": str(shift_suite).replace("\\", "/"), "note": __doc__.split("\n")[0]},
            "tasks": probes}
