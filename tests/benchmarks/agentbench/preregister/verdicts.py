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

"""The three pre-freeze oracle verdicts, computed from measured cells.

Pure functions over the measured cell records ``run_oracles.py`` produces, so
a test can recompute the committed ``oracle_verdicts.json`` from the committed
cells and assert equality (freshness), and so the arithmetic is separable from
the runs. The three rules are the plan's, verbatim:

* **lane test** (plan 2.1): a task is Lane B only if the scripted oracle
  completes it (grader PASS) on every simulator, with the competitor
  (webots) best-condition oracle's tool_calls at most ``LANE_RATIO_MAX``
  (3x) the OmniSim best-condition oracle's;
* **granularity guard** (plan 2.2 guard 1): on every Lane B task, the
  oracle's tool_calls under the webots bridge condition must be <= its
  tool_calls under webots shell;
* **distinctness** (plan 2.2 guard 2): the bridge is distinct iff the oracle
  completes at least one Lane B task with >= ``DISTINCT_MIN_SAVING`` (15%)
  fewer tool_calls under the bridge than under shell. The verdict is
  rendered here as OPERATOR-MEASURED and is not final until the plan's
  6.2.4 non-OmniSim reviewer countersigns it (a human gate this code cannot
  close).
"""

from __future__ import annotations

LANE_RATIO_MAX = 3.0
DISTINCT_MIN_SAVING = 0.15

TASKS = ("B1_overlap_audit", "B2_subject_in_frame", "B3_measure_and_report",
         "C1_parse_error_fix", "C2_fall_through_floor")
SIMS = ("omnisim", "webots")
CONDITIONS = ("shell", "shell+tools")


def _cell(cells, task, sim, condition):
    for c in cells:
        if (c.get("task") == task and c.get("sim") == sim
                and c.get("condition") == condition):
            return c
    return None


def _passed(cell):
    return bool(cell and cell.get("outcome") == "PASS")


def best_condition(cells, task, sim):
    """The mechanical plan-2.4 rule at oracle scale (n=1 per cell): more
    passes wins; a tie goes to the cheaper condition (the direction that
    disfavours the bridge's author)."""
    ranked = []
    for cond in CONDITIONS:
        c = _cell(cells, task, sim, cond)
        if c is None:
            continue
        ranked.append((0 if _passed(c) else 1,
                       c.get("tool_calls") if c.get("tool_calls") is not None
                       else 10 ** 9,
                       cond, c))
    if not ranked:
        return None, None
    ranked.sort()
    _p, _n, cond, c = ranked[0]
    return cond, c


def lane_test(cells):
    out = {}
    for task in TASKS:
        o_cond, o = best_condition(cells, task, "omnisim")
        w_cond, w = best_condition(cells, task, "webots")
        completes = _passed(o) and _passed(w)
        ratio = None
        if (o and w and o.get("tool_calls") and w.get("tool_calls")
                is not None):
            ratio = w["tool_calls"] / float(o["tool_calls"])
        ok = bool(completes and ratio is not None
                  and ratio <= LANE_RATIO_MAX)
        out[task] = {
            "omnisim_best": {"condition": o_cond,
                             "tool_calls": o.get("tool_calls") if o else None,
                             "outcome": o.get("outcome") if o else None},
            "webots_best": {"condition": w_cond,
                            "tool_calls": w.get("tool_calls") if w else None,
                            "outcome": w.get("outcome") if w else None},
            "completes_on_both": completes,
            "ratio_webots_over_omnisim": (round(ratio, 4)
                                          if ratio is not None else None),
            "ratio_max": LANE_RATIO_MAX,
            "lane_b_membership": "CONFIRMED" if ok else "NOT CONFIRMED",
        }
    return out


def granularity_guard(cells):
    out = {}
    for task in TASKS:
        sh = _cell(cells, task, "webots", "shell")
        br = _cell(cells, task, "webots", "shell+tools")
        n_sh = sh.get("tool_calls") if sh else None
        n_br = br.get("tool_calls") if br else None
        ok = (n_sh is not None and n_br is not None and n_br <= n_sh)
        out[task] = {
            "webots_shell_tool_calls": n_sh,
            "webots_bridge_tool_calls": n_br,
            "bridge_le_shell": bool(ok),
            "verdict": "HOLDS" if ok else "VIOLATED",
        }
    return out


def distinctness(cells):
    per_task = {}
    winners = []
    for task in TASKS:
        sh = _cell(cells, task, "webots", "shell")
        br = _cell(cells, task, "webots", "shell+tools")
        saving = None
        if (sh and br and sh.get("tool_calls")
                and br.get("tool_calls") is not None):
            saving = 1.0 - br["tool_calls"] / float(sh["tool_calls"])
        qualifies = bool(saving is not None and _passed(br)
                         and saving >= DISTINCT_MIN_SAVING)
        per_task[task] = {
            "saving_fraction": (round(saving, 4)
                                if saving is not None else None),
            "qualifies": qualifies,
        }
        if qualifies:
            winners.append(task)
    return {
        "per_task": per_task,
        "min_saving": DISTINCT_MIN_SAVING,
        "distinct": bool(winners),
        "qualifying_tasks": winners,
        "verdict": ("DISTINCT" if winners else "NOT DISTINCT"),
        "status": ("OPERATOR-MEASURED, AWAITING REVIEWER COUNTERSIGNATURE "
                   "(the plan 6.2.4 non-OmniSim reviewer slot is open; this "
                   "verdict is not final until a non-OmniSim reviewer "
                   "renders it -- a human gate)"),
        "consequence_if_not_distinct": (
            "plan 2.2 guard 3: the Webots within-sim shell+tools - shell "
            "delta is published as descriptive only; nothing is removed "
            "from F-comparative, both Webots conditions still run, and the "
            "plan-2.4 best-condition rule selects among them mechanically."),
    }


def compute(cells):
    return {
        "lane_test": lane_test(cells),
        "granularity_guard": granularity_guard(cells),
        "distinctness": distinctness(cells),
    }


__all__ = ["CONDITIONS", "DISTINCT_MIN_SAVING", "LANE_RATIO_MAX", "SIMS",
           "TASKS", "best_condition", "compute", "distinctness",
           "granularity_guard", "lane_test"]
