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

"""The F evaluator -- agent-edge-validation-plan §2.4, implemented exactly.

Reads campaign ``rows.jsonl`` files (the SPEC §5 schema, optionally wrapped
with the ``campaign`` block ``campaign.py`` adds) and evaluates the frozen
withdrawal rule:

* **Eligibility**: a task enters a completion channel iff at least one of the
  two compared conditions passed it at least once. A task neither passed is
  excluded and reported as unresolvable by this design.
* **Definedness**: a cost ratio exists on a task only where BOTH compared
  conditions passed >= 3 of 5 runs. A median over one or two passing runs is
  a coin flip, not a statistic.
* **F-surface** (conjunct i, OmniSim cells only): per eligible task
  ``dpass = passes(shell+tools) - passes(shell)``; per defined task
  ``Rcalls = median tool_calls(shell+tools) / median tool_calls(shell)`` over
  passing runs. A task COUNTS for the cost channel only if
  ``Rcalls <= 0.85`` AND ``dpass >= 0`` (a cost win on a task the surface
  completes less is not a win).

  - Channel A (completion) holds iff ``dpass >= +2`` on >= 2 eligible tasks
    and no eligible task has ``dpass <= -2``.
  - Channel B (cost) holds iff >= 3 defined tasks count.

* **Unevaluable**: a completion channel with < 4 eligible tasks, or a cost
  channel with < 3 defined tasks, is unevaluable; an unevaluable channel does
  not hold, and the verdict's leading sentence names every unevaluable
  channel.
* **Verdict**: both channels hold -> the conjunct *survives* (and survival
  licenses nothing -- the note is emitted). Exactly one holds -> a *narrow
  finding* published in the holding channel's direction with the other
  channel's full per-task table printed regardless of what it shows. Neither
  holds -> the conjunct is *WITHDRAWN*.

  The four pre-written narrow templates, two of them adverse, and the
  mechanical selection used here:

  - cost holds, completion does not and shows no adverse task
    -> ``"same completions, materially fewer calls"``
  - completion holds, cost does not and is not adverse
    -> ``"more completions at no call saving"``
  - cost holds, completion does not because an eligible task has
    ``dpass <= -2`` -> ``"fewer completions"`` (adverse)
  - completion holds, cost is defined and materially against the surface
    (>= 3 defined tasks at ``Rcalls >= 1/0.85``)
    -> ``"materially more calls"`` (adverse)

* **Best-condition rule** (mechanical, identical for every simulator): the
  condition with more total Lane B passes out of 25; totals within 1 -> the
  lower sum of per-task median ``tool_calls`` over passing runs. Near-ties go
  to the cheaper condition on purpose.
* **F-comparative** (conjunct ii, best vs best): ``dx``/``Rx`` with the same
  gate and channel forms (channels C and D).
* **The death condition** (SPEC §6.1, unconditional): if total Lane B
  passes(competitor best) >= total Lane B passes(OmniSim best) AND
  median-of-per-task-median ``tool_calls``(competitor best) <= that of
  OmniSim best, the entire agent-native claim is dead and that is published
  as prominently as a win would have been.

The report emitter greps its own output for the Phase-W forbidden-sentence
patterns (plan §4) and refuses to write a report containing one. Every number
in the report is traceable to ``rows.jsonl`` source lines (the traceability
section lists ``path#L<n>`` per cell).

Usage::

    python tests/benchmarks/agentbench/f_eval.py \\
        --rows results_published/<campaign_id>/rows.jsonl \\
        --out  results_published/<campaign_id>/f_report.md
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------------------
# Frozen constants (plan §2.1, §2.4). Changing one after a freeze voids the
# campaign; they are named here once and referenced everywhere.
# ---------------------------------------------------------------------------

LANE_A_TASK = "A1_husky_swarm_10"
LANE_B_TASKS = (
    "B1_overlap_audit",
    "B2_subject_in_frame",
    "B3_measure_and_report",
    "C1_parse_error_fix",
    "C2_fall_through_floor",
)
N_LANE_B = 5                # runs per Lane B cell (SPEC §3.5)
N_LANE_A = 10               # A1's own floor (SPEC §3.5)

COST_GATE = 0.85            # Rcalls <= 0.85 counts  (conventional, §7 Q2)
COMPLETION_BAR = 2          # dpass >= +2 qualifies  (conventional, §7 Q2)
ADVERSE_BAR = -2            # dpass <= -2 is adverse
DEFINED_FLOOR = 3           # both conditions passed >= 3 of 5
MIN_ELIGIBLE = 4            # completion channel evaluable at >= 4 eligible
MIN_DEFINED = 3             # cost channel evaluable at >= 3 defined
QUALIFYING_TASKS = 2        # channel A/C needs dpass >= +2 on >= 2 tasks
COUNTING_TASKS = 3          # channel B/D needs >= 3 counting tasks
ADVERSE_COST_RATIO = 1.0 / COST_GATE   # the mirror of the counting gate

SHELL = "shell"
TOOLS = "shell+tools"
CONDITIONS = (SHELL, TOOLS)

SURVIVES = "survives"
NARROW = "narrow finding"
WITHDRAWN = "WITHDRAWN"

TEMPLATE_COST_ONLY = "same completions, materially fewer calls"
TEMPLATE_COMPLETION_ONLY = "more completions at no call saving"
TEMPLATE_FEWER_COMPLETIONS = "fewer completions"          # adverse
TEMPLATE_MORE_CALLS = "materially more calls"             # adverse
NARROW_TEMPLATES = (TEMPLATE_COST_ONLY, TEMPLATE_COMPLETION_ONLY,
                    TEMPLATE_FEWER_COMPLETIONS, TEMPLATE_MORE_CALLS)

ALL_SIM_NAMES = ("gazebo", "isaac", "mujoco", "webots")


class ForbiddenSentenceError(RuntimeError):
    """The rendered report contained a plan-§4 forbidden sentence."""


# ---------------------------------------------------------------------------
# Forbidden sentences (plan §4, "After Phase W"), encoded as patterns the
# emitter greps its own output for. Patterns for "than <sim>" are built per
# evaluation: naming a simulator after "than" is forbidden for every simulator
# that has no rows in the evaluated campaign (and for Gazebo/Isaac/MuJoCo it
# is forbidden in Phase W regardless -- they are not in any Phase W campaign).
# ---------------------------------------------------------------------------

_STATIC_FORBIDDEN = [
    ("faster-than-a-simulator", r"(?i)\bfaster\s+than\b"),
    ("most-agent-driveable", r"(?i)most\s+agent[- ]driveable"),
    ("agents-get-more-done-unqualified",
     r"(?i)agents\s+get\s+more\s+done\s+on\s+omnisim"),
    ("harness-pays-for-itself", r"(?i)harness\s+pays\s+for\s+itself"),
    ("claim-survived-F", r"(?i)claim\s+survived\s+F\b"),
    ("F-did-not-withdraw", r"(?i)\bF\s+did\s+not\s+withdraw"),
    ("anything-with-first", r"(?i)\bfirst\b"),
    ("lane-c-competitor-failure", r"(?i)failed\s+the\s+hot[- ]reload"),
    ("newton-default-claim", r"(?i)\bnewton\s+default\b"),
    ("lane-c-count", r"(?i)\b\d+\s+of\s+\d+\s+(frontier\s+)?capabilit"),
]


def forbidden_patterns(sims_present=()):
    """The compiled (label, regex) list for a campaign that ran ``sims``."""
    pats = [(label, re.compile(rx)) for label, rx in _STATIC_FORBIDDEN]
    present = {s.lower() for s in sims_present}
    for sim in ALL_SIM_NAMES:
        if sim not in present:
            pats.append(("than-%s-not-run" % sim,
                         re.compile(r"(?i)\bthan\s+(upstream\s+)?%s\b" % sim)))
    return pats


def check_forbidden(text, sims_present=()):
    """Return every forbidden-pattern hit in ``text`` as (label, excerpt)."""
    hits = []
    for label, rx in forbidden_patterns(sims_present):
        m = rx.search(text)
        if m:
            lo = max(0, m.start() - 40)
            hits.append((label, text[lo:m.end() + 40].replace("\n", " ")))
    return hits


# ---------------------------------------------------------------------------
# Row loading and per-cell statistics
# ---------------------------------------------------------------------------

def wilson(k, n, z=1.96):
    """Wilson 95% interval for k successes of n. Returns (lo, hi) or None."""
    if not n:
        return None
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


@dataclass
class RowRef:
    source: str            # "path#L<n>"
    outcome: str
    tool_calls: object
    t_agent_s: object
    t_total_s: object
    tokens_in: object
    tokens_out: object
    tokens_cache_read: object


@dataclass
class Cell:
    sim: str
    condition: str
    task: str
    rows: list = field(default_factory=list)

    def _of(self, *outcomes):
        return [r for r in self.rows if r.outcome in outcomes]

    @property
    def n_total(self):
        return len(self.rows)

    @property
    def passes(self):
        return len(self._of("PASS"))

    @property
    def fails(self):
        return len(self._of("FAIL"))

    @property
    def invalids(self):
        return len(self._of("INVALID"))

    @property
    def others(self):
        return len(self.rows) - self.passes - self.fails - self.invalids

    @property
    def scoreable(self):
        """PASS + FAIL -- what the pass rate is computed over (SPEC §3.3)."""
        return self.passes + self.fails

    def _median(self, attr, outcomes):
        vals = [getattr(r, attr) for r in self._of(*outcomes)
                if isinstance(getattr(r, attr), (int, float))]
        return statistics.median(vals) if vals else None

    @property
    def med_calls_pass(self):
        return self._median("tool_calls", ("PASS",))

    @property
    def med_calls_all(self):
        """All-runs (PASS+FAIL) median -- the §2.3 selection-effect
        sensitivity row, printed beside the passing-runs median."""
        return self._median("tool_calls", ("PASS", "FAIL"))

    @property
    def med_t_agent_pass(self):
        return self._median("t_agent_s", ("PASS",))

    @property
    def med_t_total_pass(self):
        return self._median("t_total_s", ("PASS",))

    @property
    def med_tokens_in(self):
        return self._median("tokens_in", ("PASS",))

    @property
    def med_tokens_out(self):
        return self._median("tokens_out", ("PASS",))

    @property
    def med_tokens_cache_read(self):
        return self._median("tokens_cache_read", ("PASS",))

    @property
    def sources(self):
        return [r.source for r in self.rows]


def load_rows(paths):
    """Yield ``(row_dict, "path#L<n>")`` for every non-blank JSONL line."""
    for p in paths:
        p = Path(p)
        with open(p, encoding="utf-8") as fh:
            for i, line in enumerate(fh, start=1):
                if line.strip():
                    yield json.loads(line), "%s#L%d" % (p, i)


def _row_sim(row):
    camp = row.get("campaign") or {}
    return camp.get("sim") or row.get("sim") or "omnisim"


def _excluded(row):
    """Rows the evaluation may not read: superseded retry attempts and
    exploratory rows (smoke / dry-run / n=1) -- SPEC §3.5 bars them."""
    camp = row.get("campaign") or {}
    if camp.get("superseded"):
        return "superseded"
    if camp.get("exploratory") or camp.get("dry_run"):
        return "exploratory"
    return None


@dataclass
class Collected:
    cells: dict = field(default_factory=dict)   # (sim, cond, task) -> Cell
    sims: list = field(default_factory=list)
    models: set = field(default_factory=set)
    machines: set = field(default_factory=set)
    excluded: dict = field(default_factory=dict)  # reason -> count
    sources: list = field(default_factory=list)
    #: Rows flagged ``measured_under_concurrency`` (plan §2.7): their TIME
    #: columns are nulled at load, so no latency statement can read them.
    #: The row itself stays -- verdicts, tool_calls and tokens are
    #: contention-insensitive.
    concurrency_time_excluded: int = 0

    def cell(self, sim, cond, task):
        key = (sim, cond, task)
        if key not in self.cells:
            self.cells[key] = Cell(sim=sim, condition=cond, task=task)
        return self.cells[key]


def collect(row_iter):
    col = Collected()
    for row, src in row_iter:
        reason = _excluded(row)
        if reason:
            col.excluded[reason] = col.excluded.get(reason, 0) + 1
            continue
        sim = _row_sim(row)
        cond = row.get("condition")
        task = row.get("task")
        if cond not in CONDITIONS or task is None:
            col.excluded["unattributed"] = (
                col.excluded.get("unattributed", 0) + 1)
            continue
        if sim not in col.sims:
            col.sims.append(sim)
        model = (row.get("agent") or {}).get("model")
        if model:
            col.models.add(model)
        mach = ((row.get("machine") or {}).get("machine") or {}).get("id")
        if mach:
            col.machines.add(mach)
        m = row.get("metrics") or {}
        # Plan §2.7/§2.3: wall clock measured under contention is not a
        # measurement (plan §5.3). A row flagged measured_under_concurrency
        # keeps its verdict and its contention-insensitive metrics; its time
        # columns are nulled HERE, so nothing downstream can quote them.
        t_agent_s, t_total_s = m.get("t_agent_s"), m.get("t_total_s")
        if (row.get("agent_artifacts") or {}).get(
                "measured_under_concurrency"):
            t_agent_s, t_total_s = None, None
            col.concurrency_time_excluded += 1
        col.cell(sim, cond, task).rows.append(RowRef(
            source=src, outcome=row.get("outcome"),
            tool_calls=m.get("tool_calls"),
            t_agent_s=t_agent_s, t_total_s=t_total_s,
            tokens_in=m.get("tokens_in"), tokens_out=m.get("tokens_out"),
            tokens_cache_read=m.get("tokens_cache_read")))
    return col


# ---------------------------------------------------------------------------
# Channel evaluation (shared by F-surface and F-comparative)
# ---------------------------------------------------------------------------

@dataclass
class TaskLine:
    task: str
    passes_a: int          # favoured side (shell+tools / OmniSim best)
    n_a: int
    passes_b: int          # baseline side (shell / competitor best)
    n_b: int
    delta: int
    eligible: bool
    defined: bool
    ratio: object          # float | None (undefined) | math.inf
    counts: bool
    med_a: object
    med_b: object
    med_all_a: object
    med_all_b: object


@dataclass
class PairEval:
    """One conjunct's evaluation: channels + verdict + templates."""
    name: str
    a_label: str
    b_label: str
    lines: list
    # completion channel
    eligible_n: int = 0
    qualifying_n: int = 0
    adverse_n: int = 0
    completion_unevaluable: bool = False
    completion_holds: bool = False
    # cost channel
    defined_n: int = 0
    counting_n: int = 0
    cost_adverse_n: int = 0
    cost_unevaluable: bool = False
    cost_holds: bool = False
    # verdict
    verdict: str = ""
    template: str = ""
    unevaluable_channels: list = field(default_factory=list)


def _ratio(med_a, med_b):
    if med_a is None or med_b is None:
        return None
    if med_b == 0:
        return 1.0 if med_a == 0 else math.inf
    return med_a / med_b


def evaluate_pair(name, a_label, b_label, cell_a, cell_b,
                  completion_names=("A", "B")):
    """Evaluate one conjunct. ``cell_a(task)`` / ``cell_b(task)`` return the
    :class:`Cell` for the favoured / baseline side. ``completion_names`` are
    the channel letters ((A, B) for F-surface, (C, D) for F-comparative)."""
    lines = []
    for task in LANE_B_TASKS:
        ca, cb = cell_a(task), cell_b(task)
        pa = ca.passes if ca else 0
        pb = cb.passes if cb else 0
        na = ca.n_total if ca else 0
        nb = cb.n_total if cb else 0
        eligible = (pa >= 1) or (pb >= 1)
        defined = (pa >= DEFINED_FLOOR) and (pb >= DEFINED_FLOOR)
        med_a = ca.med_calls_pass if ca else None
        med_b = cb.med_calls_pass if cb else None
        ratio = _ratio(med_a, med_b) if defined else None
        delta = pa - pb
        counts = bool(defined and ratio is not None
                      and ratio <= COST_GATE and delta >= 0)
        lines.append(TaskLine(
            task=task, passes_a=pa, n_a=na, passes_b=pb, n_b=nb,
            delta=delta, eligible=eligible, defined=defined, ratio=ratio,
            counts=counts, med_a=med_a, med_b=med_b,
            med_all_a=(ca.med_calls_all if ca else None),
            med_all_b=(cb.med_calls_all if cb else None)))

    pe = PairEval(name=name, a_label=a_label, b_label=b_label, lines=lines)
    elig = [ln for ln in lines if ln.eligible]
    defd = [ln for ln in lines if ln.defined]
    pe.eligible_n = len(elig)
    pe.defined_n = len(defd)
    pe.qualifying_n = sum(1 for ln in elig if ln.delta >= COMPLETION_BAR)
    pe.adverse_n = sum(1 for ln in elig if ln.delta <= ADVERSE_BAR)
    pe.counting_n = sum(1 for ln in defd if ln.counts)
    pe.cost_adverse_n = sum(
        1 for ln in defd
        if ln.ratio is not None and ln.ratio >= ADVERSE_COST_RATIO)

    pe.completion_unevaluable = pe.eligible_n < MIN_ELIGIBLE
    pe.cost_unevaluable = pe.defined_n < MIN_DEFINED
    pe.completion_holds = (not pe.completion_unevaluable
                           and pe.qualifying_n >= QUALIFYING_TASKS
                           and pe.adverse_n == 0)
    pe.cost_holds = (not pe.cost_unevaluable
                     and pe.counting_n >= COUNTING_TASKS)

    ch_a, ch_b = completion_names
    if pe.completion_unevaluable:
        pe.unevaluable_channels.append(
            "completion channel (%s): %d eligible task(s) of the %d required"
            % (ch_a, pe.eligible_n, MIN_ELIGIBLE))
    if pe.cost_unevaluable:
        pe.unevaluable_channels.append(
            "cost channel (%s): %d defined task(s) of the %d required"
            % (ch_b, pe.defined_n, MIN_DEFINED))

    # --- the three-way verdict + the four narrow templates -----------------
    if pe.completion_holds and pe.cost_holds:
        pe.verdict = SURVIVES
    elif pe.cost_holds and not pe.completion_holds:
        pe.verdict = NARROW
        pe.template = (TEMPLATE_FEWER_COMPLETIONS if pe.adverse_n > 0
                       else TEMPLATE_COST_ONLY)
    elif pe.completion_holds and not pe.cost_holds:
        pe.verdict = NARROW
        pe.template = (TEMPLATE_MORE_CALLS
                       if (not pe.cost_unevaluable
                           and pe.cost_adverse_n >= COUNTING_TASKS)
                       else TEMPLATE_COMPLETION_ONLY)
    else:
        pe.verdict = WITHDRAWN
    return pe


# ---------------------------------------------------------------------------
# Best condition and the death condition
# ---------------------------------------------------------------------------

@dataclass
class BestCondition:
    sim: str
    condition: str
    totals: dict                      # condition -> total Lane B passes
    call_sums: dict                   # condition -> sum of per-task medians
    call_sum_tasks: dict              # condition -> tasks contributing
    rule: str = ""


def best_condition(col, sim):
    """Plan §2.4: more total Lane B passes (out of 25); totals within 1 ->
    lower sum of per-task median tool_calls over passing runs. No tie-break
    toward shell+tools; a residual exact tie goes to plain ``shell`` (the
    cheaper-to-provide condition -- the direction that disfavours the
    surface's author)."""
    totals, call_sums, call_sum_tasks = {}, {}, {}
    for cond in CONDITIONS:
        total, csum, ctasks = 0, 0.0, 0
        for task in LANE_B_TASKS:
            cell = col.cells.get((sim, cond, task))
            if cell is None:
                continue
            total += cell.passes
            med = cell.med_calls_pass
            if med is not None:
                csum += med
                ctasks += 1
        totals[cond] = total
        call_sums[cond] = csum
        call_sum_tasks[cond] = ctasks
    bc = BestCondition(sim=sim, condition=SHELL, totals=totals,
                       call_sums=call_sums, call_sum_tasks=call_sum_tasks)
    if abs(totals[SHELL] - totals[TOOLS]) > 1:
        bc.condition = max(CONDITIONS, key=lambda c: totals[c])
        bc.rule = ("total Lane B passes: %d (%s) vs %d (%s)"
                   % (totals[TOOLS], TOOLS, totals[SHELL], SHELL))
    else:
        # near-tie -> the cheaper condition (lower sum of per-task medians)
        if call_sums[SHELL] == call_sums[TOOLS]:
            bc.condition = SHELL
        else:
            bc.condition = min(CONDITIONS, key=lambda c: call_sums[c])
        bc.rule = ("totals within 1 (%d vs %d) -> lower sum of per-task "
                   "median tool_calls over passing runs: %.1f (%s, %d tasks) "
                   "vs %.1f (%s, %d tasks)"
                   % (totals[TOOLS], totals[SHELL],
                      call_sums[TOOLS], TOOLS, call_sum_tasks[TOOLS],
                      call_sums[SHELL], SHELL, call_sum_tasks[SHELL]))
    return bc


@dataclass
class DeathCheck:
    evaluated: bool = False
    fired: bool = False
    detail: str = ""


def death_condition(col, omnisim_best, comp_sim, comp_best):
    """SPEC §6.1 via plan §2.4: aggregate totals + median-of-per-task-median
    tool_calls, competitor best vs OmniSim best."""
    def totals_and_mom(sim, cond):
        total, meds = 0, []
        for task in LANE_B_TASKS:
            cell = col.cells.get((sim, cond, task))
            if cell is None:
                continue
            total += cell.passes
            med = cell.med_calls_pass
            if med is not None:
                meds.append(med)
        mom = statistics.median(meds) if meds else None
        return total, mom, len(meds)

    dc = DeathCheck()
    o_total, o_mom, o_k = totals_and_mom("omnisim", omnisim_best)
    c_total, c_mom, c_k = totals_and_mom(comp_sim, comp_best)
    dc.evaluated = True
    passes_cond = c_total >= o_total
    if o_mom is None or c_mom is None:
        calls_cond = False
        calls_txt = ("median-of-medians undefined (omnisim: %s over %d "
                     "task(s); %s: %s over %d task(s))"
                     % (o_mom, o_k, comp_sim, c_mom, c_k))
    else:
        calls_cond = c_mom <= o_mom
        calls_txt = ("median-of-per-task-median tool_calls: %s %.1f vs "
                     "omnisim %.1f" % (comp_sim, c_mom, o_mom))
    dc.fired = bool(passes_cond and calls_cond)
    dc.detail = ("total Lane B passes: %s best (%s) %d/25 vs omnisim best "
                 "(%s) %d/25; %s"
                 % (comp_sim, comp_best, c_total, omnisim_best, o_total,
                    calls_txt))
    return dc


# ---------------------------------------------------------------------------
# The evaluation object
# ---------------------------------------------------------------------------

@dataclass
class Evaluation:
    col: Collected
    surface: object = None            # PairEval | None
    best: dict = field(default_factory=dict)      # sim -> BestCondition
    comparative: object = None        # PairEval | None
    comparative_sim: str = ""
    death: object = None              # DeathCheck | None
    notes: list = field(default_factory=list)

    @property
    def unevaluable_channels(self):
        out = []
        if self.surface is not None:
            out += ["F-surface " + u
                    for u in self.surface.unevaluable_channels]
        else:
            out.append("F-surface: no scored OmniSim Lane B rows")
        if self.comparative is not None:
            out += ["F-comparative " + u
                    for u in self.comparative.unevaluable_channels]
        else:
            out.append("F-comparative: no competitor rows in this campaign, "
                       "so conjunct (ii) is not evaluated")
        return out


def evaluate(row_iter):
    col = collect(row_iter)
    ev = Evaluation(col=col)

    if any(k[0] == "omnisim" for k in col.cells):
        ev.surface = evaluate_pair(
            "F-surface (conjunct i)", TOOLS, SHELL,
            cell_a=lambda t: col.cells.get(("omnisim", TOOLS, t)),
            cell_b=lambda t: col.cells.get(("omnisim", SHELL, t)),
            completion_names=("A", "B"))

    for sim in col.sims:
        ev.best[sim] = best_condition(col, sim)

    comp_sims = [s for s in col.sims if s != "omnisim"]
    if comp_sims and "omnisim" in col.sims:
        comp = comp_sims[0]
        if len(comp_sims) > 1:
            ev.notes.append("multiple competitor sims present (%s); "
                            "F-comparative evaluated against %s only"
                            % (", ".join(comp_sims), comp))
        ev.comparative_sim = comp
        o_best = ev.best["omnisim"].condition
        c_best = ev.best[comp].condition
        ev.comparative = evaluate_pair(
            "F-comparative (conjunct ii)",
            "omnisim best (%s)" % o_best, "%s best (%s)" % (comp, c_best),
            cell_a=lambda t: col.cells.get(("omnisim", o_best, t)),
            cell_b=lambda t: col.cells.get((comp, c_best, t)),
            completion_names=("C", "D"))
        ev.death = death_condition(col, o_best, comp, c_best)
    return ev


# ---------------------------------------------------------------------------
# Report rendering
# ---------------------------------------------------------------------------

def _fmt(v, spec="%.1f"):
    if v is None:
        return "n/a"
    if v is math.inf:
        return "inf"
    if isinstance(v, float):
        return spec % v
    return str(v)


def _wilson_str(k, n):
    ci = wilson(k, n)
    if ci is None:
        return "n/a"
    return "%d/%d = %.2f [%.2f, %.2f]" % (k, n, k / n, ci[0], ci[1])


def _verdict_para(pe, conjunct_txt):
    out = []
    if pe.unevaluable_channels:
        out.append("Unevaluable channels: %s. An unevaluable channel does "
                   "not hold." % "; ".join(pe.unevaluable_channels))
    else:
        out.append("No channel of this conjunct is unevaluable.")
    ch = ("completion holds=%s (dpass >= +%d on %d of %d eligible tasks, "
          "%d required; adverse tasks with dpass <= %d: %d); cost holds=%s "
          "(%d of %d defined tasks count, %d required)"
          % (pe.completion_holds, COMPLETION_BAR, pe.qualifying_n,
             pe.eligible_n, QUALIFYING_TASKS, ADVERSE_BAR, pe.adverse_n,
             pe.cost_holds, pe.counting_n, pe.defined_n, COUNTING_TASKS))
    out.append("Channels: %s." % ch)
    if pe.verdict == SURVIVES:
        out.append("**Verdict: %s survives.** Survival licenses nothing "
                   "beyond the phase text of plan §4 -- quoting survival as "
                   "an achievement is itself forbidden (plan §2.4); "
                   "confirmation is SPEC §9.3's business."
                   % conjunct_txt)
    elif pe.verdict == NARROW:
        out.append("**Verdict: narrow finding -- \"%s\".** Published in the "
                   "holding channel's direction; the other channel's full "
                   "per-task table is printed below regardless of what it "
                   "shows." % pe.template)
    else:
        out.append("**Verdict: %s is %s.**" % (conjunct_txt, WITHDRAWN))
    return "\n".join(out)


def _pair_tables(pe):
    """Both channels' per-task tables, printed regardless of direction."""
    out = []
    out.append("**Completion channel -- per task (passes, Wilson 95% CI, "
               "denominators printed):**\n")
    out.append("| task | %s passes | %s passes | dpass | eligible |"
               % (pe.a_label, pe.b_label))
    out.append("|---|---|---|---|---|")
    for ln in pe.lines:
        out.append("| %s | %s | %s | %+d | %s |"
                   % (ln.task, _wilson_str(ln.passes_a, ln.n_a),
                      _wilson_str(ln.passes_b, ln.n_b), ln.delta,
                      "yes" if ln.eligible else
                      "no -- unresolvable by this design"))
    out.append("")
    out.append("**Cost channel -- per task (median tool_calls over passing "
               "runs; the all-runs median beside it is the selection-effect "
               "sensitivity row, plan §2.3):**\n")
    out.append("| task | %s med calls (pass) | all-runs | %s med calls "
               "(pass) | all-runs | Rcalls | defined (>=%d passes both) | "
               "counts (R<=%.2f and dpass>=0) |"
               % (pe.a_label, pe.b_label, DEFINED_FLOOR, COST_GATE))
    out.append("|---|---|---|---|---|---|---|---|")
    for ln in pe.lines:
        out.append("| %s | %s | %s | %s | %s | %s | %s | %s |"
                   % (ln.task, _fmt(ln.med_a), _fmt(ln.med_all_a),
                      _fmt(ln.med_b), _fmt(ln.med_all_b),
                      _fmt(ln.ratio, "%.3f"),
                      "yes" if ln.defined else "no -- ratio undefined",
                      "yes" if ln.counts else "no"))
    out.append("")
    return out


def render(ev):
    col = ev.col
    L = []
    L.append("# AgentBench F evaluation (agent-edge-validation-plan §2.4)")
    L.append("")
    # The verdict's leading sentence names every unevaluable channel.
    unev = ev.unevaluable_channels
    if unev:
        L.append("Unevaluable channels in this evaluation: %s."
                 % "; ".join(unev))
    else:
        L.append("No channel in this evaluation is unevaluable.")
    L.append("")
    L.append("## Header and disclosures")
    L.append("")
    L.append("- Resolution limit: n = %d per Lane B cell (pass counts 0-%d; "
             "this design cannot distinguish a 0.6 pass rate from a 0.8 "
             "one); n = %d for %s. Wilson 95%% CIs are printed with n "
             "beside every pass count." % (N_LANE_B, N_LANE_B, N_LANE_A,
                                           LANE_A_TASK))
    L.append("- The effect-size constants -- Rcalls <= %.2f and the +%d "
             "completion bar -- are conventional, not power-derived "
             "(plan §7 Q2)." % (COST_GATE, COMPLETION_BAR))
    L.append("- Cost channels condition on passing runs, which is a "
             "selection effect whenever pass rates differ; per-condition "
             "pass counts are printed beside every cost ratio and the "
             "all-runs median is published beside the passing-runs median "
             "as a sensitivity row (plan §2.3).")
    L.append("- Tokens are reported separately (tokens_in / tokens_out / "
             "tokens_cache_read) and never summed (SPEC §3.1). Time is "
             "reported with t_agent_s leading and t_total_s beside it.")
    if col.concurrency_time_excluded:
        L.append("- %d row(s) were flagged `measured_under_concurrency` "
                 "(another lane active while they ran, plan §2.7); their "
                 "time columns are EXCLUDED from every time median above "
                 "and below -- wall clock measured under contention is not "
                 "a measurement (plan §5.3, §2.3). Their verdicts, "
                 "tool_calls and tokens stand."
                 % col.concurrency_time_excluded)
    L.append("- Machine id(s): %s." % (", ".join(sorted(col.machines))
                                       or "none recorded"))
    L.append("- Model id(s): %s." % (", ".join(sorted(col.models))
                                     or "none recorded"))
    L.append("- Simulators with rows: %s." % (", ".join(col.sims) or "none"))
    if ev.comparative is None:
        L.append("- Conjunct (ii) needs a competitor arm; this campaign "
                 "carries none, so F-comparative is not evaluated and no "
                 "comparative sentence of any kind is licensed.")
    if col.excluded:
        L.append("- Rows excluded from evaluation: %s."
                 % ", ".join("%s=%d" % kv
                             for kv in sorted(col.excluded.items())))
    L.append("")
    L.append("### n per cell")
    L.append("")
    L.append("| sim | condition | task | n | PASS | FAIL | INVALID | other |")
    L.append("|---|---|---|---|---|---|---|---|")
    for key in sorted(col.cells):
        c = col.cells[key]
        L.append("| %s | %s | %s | %d | %d | %d | %d | %d |"
                 % (c.sim, c.condition, c.task, c.n_total, c.passes,
                    c.fails, c.invalids, c.others))
    L.append("")

    # ---- F-surface -------------------------------------------------------
    L.append("## F-surface -- conjunct (i), OmniSim cells only")
    L.append("")
    if ev.surface is None:
        L.append("No scored OmniSim Lane B rows: conjunct (i) is not "
                 "evaluated.")
        L.append("")
    else:
        L.append(_verdict_para(ev.surface, "conjunct (i)"))
        L.append("")
        L.extend(_pair_tables(ev.surface))

    # ---- best condition --------------------------------------------------
    L.append("## Best condition per simulator (mechanical rule, identical "
             "for every simulator)")
    L.append("")
    for sim, bc in sorted(ev.best.items()):
        L.append("- **%s**: best condition = `%s` (%s)."
                 % (sim, bc.condition, bc.rule))
    if not ev.best:
        L.append("- no simulators with scoreable Lane B rows.")
    L.append("")

    # ---- F-comparative ---------------------------------------------------
    L.append("## F-comparative -- conjunct (ii), best vs best")
    L.append("")
    if ev.comparative is None:
        L.append("Not evaluated: this campaign has no competitor rows.")
        L.append("")
    else:
        L.append(_verdict_para(ev.comparative, "conjunct (ii)"))
        L.append("")
        L.extend(_pair_tables(ev.comparative))
        L.append("### The death condition (SPEC §6.1, evaluated "
                 "unconditionally)")
        L.append("")
        if ev.death is not None and ev.death.fired:
            L.append("**THE DEATH CONDITION FIRED.** %s. Per the "
                     "pre-registration, the entire agent-native claim is "
                     "dead, and this is published as prominently as a win "
                     "would have been (plan §2.4, §2.5)." % ev.death.detail)
        elif ev.death is not None:
            L.append("Did not fire. %s." % ev.death.detail)
        L.append("")

    # ---- Lane A ----------------------------------------------------------
    L.append("## Lane A (control, %s) -- reported separately, never in the "
             "decision set" % LANE_A_TASK)
    L.append("")
    a_cells = [c for k, c in sorted(col.cells.items())
               if k[2] == LANE_A_TASK]
    if a_cells:
        L.append("| sim | condition | passes (Wilson 95% CI) | median "
                 "tool_calls (pass) |")
        L.append("|---|---|---|---|")
        for c in a_cells:
            L.append("| %s | %s | %s | %s |"
                     % (c.sim, c.condition,
                        _wilson_str(c.passes, c.n_total),
                        _fmt(c.med_calls_pass)))
        L.append("")
        L.append("The expected Lane A result is a tie (both conditions "
                 "reduce to writing a file, plan §2.1); a surface loss here "
                 "is published. %s is the owner's demo task (SPEC §8.3) and "
                 "never enters any aggregate." % LANE_A_TASK)
    else:
        L.append("No Lane A rows in this evaluation.")
    L.append("")

    # ---- cost detail: time and tokens -----------------------------------
    L.append("## Time and tokens per cell (medians over passing runs; "
             "t_agent_s leading, t_total_s beside; tokens separate, "
             "never summed)")
    L.append("")
    if col.concurrency_time_excluded:
        L.append("(time medians exclude the %d concurrency-flagged row(s) "
                 "named in the header)" % col.concurrency_time_excluded)
        L.append("")
    L.append("| sim | condition | task | t_agent_s | t_total_s | tokens_in "
             "| tokens_out | tokens_cache_read |")
    L.append("|---|---|---|---|---|---|---|---|")
    for key in sorted(col.cells):
        c = col.cells[key]
        L.append("| %s | %s | %s | %s | %s | %s | %s | %s |"
                 % (c.sim, c.condition, c.task,
                    _fmt(c.med_t_agent_pass, "%.1f"),
                    _fmt(c.med_t_total_pass, "%.1f"),
                    _fmt(c.med_tokens_in, "%.0f"),
                    _fmt(c.med_tokens_out, "%.0f"),
                    _fmt(c.med_tokens_cache_read, "%.0f")))
    L.append("")

    # ---- traceability ----------------------------------------------------
    L.append("## Traceability -- every number above resolves to these rows")
    L.append("")
    for key in sorted(col.cells):
        c = col.cells[key]
        L.append("- `%s / %s / %s`: %s"
                 % (c.sim, c.condition, c.task, ", ".join(c.sources)))
    if not col.cells:
        L.append("- no cells.")
    L.append("")
    for note in ev.notes:
        L.append("Note: %s" % note)
    L.append("")
    return "\n".join(L)


def emit(ev, out_path):
    """Render, self-check against the forbidden-sentence list, then write.

    A report containing a plan-§4 forbidden pattern is never written --
    the emitter fails loudly instead (:class:`ForbiddenSentenceError`)."""
    text = render(ev)
    hits = check_forbidden(text, sims_present=ev.col.sims)
    if hits:
        raise ForbiddenSentenceError(
            "refusing to write the report: forbidden sentence pattern(s) in "
            "the rendered output -- %s"
            % "; ".join("%s: ...%s..." % h for h in hits))
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="utf-8")
    return out_path


#: Rows produced under the single-run protocol (SPEC §3.5, owner 2026-08-10)
#: carry this key. F cannot be evaluated over them -- see :func:`refuse_single_run`.
PROTOCOL_KEY = "protocol"


def refuse_single_run(pairs):
    """Return a refusal message if these rows cannot support an F verdict.

    Takes :func:`load_rows`' ``(row, source)`` pairs, so the refusal can name
    the file and line a single-run row came from.

    F (plan §2.4, frozen) is arithmetic over ``n = 5``: its completion channel
    needs ``dpass >= +2`` out of 5, and its cost channel is DEFINED only where
    both conditions passed ``>= 3 of 5``. Under the single-run protocol
    ``dpass`` lives in ``{-1, 0, +1}`` and the definedness floor is
    unreachable, so **both channels are structurally unevaluable and F would
    withdraw every time** -- on the arithmetic, not on evidence.

    A withdrawal that the design cannot avoid is not a finding, and printing
    one beside a header reading "n = 5 per Lane B cell" would be a false
    statement about the evidence in two places at once. So this refuses
    instead. The plan's §2 is frozen content and is NOT amended here: F needs
    re-deriving for a single-run design by its owner, and until then no F
    verdict may be computed, quoted, or read as a withdrawal.
    """
    pairs = list(pairs)
    single = [(r, src) for r, src in pairs
              if int(((r.get(PROTOCOL_KEY) or {}).get("runs_per_cell") or 0))
              == 1]
    if not single:
        return None
    ids = sorted({str((r.get(PROTOCOL_KEY) or {}).get("id"))
                  for r, _ in single})
    return (
        "REFUSED: %d of %d rows were produced under a single-run protocol "
        "(%s). F is defined on n = 5 (plan 2.4, frozen): its completion "
        "channel needs dpass >= +2 out of 5 and its cost channel needs >= 3 "
        "of 5 passes on both sides, so at one run per cell BOTH channels are "
        "unevaluable and F would withdraw on the arithmetic alone -- which "
        "is not evidence about the claim. See SPEC 3.5. F must be re-derived "
        "for a single-run design before this tool can produce a verdict; "
        "nothing here may be quoted as a withdrawal in the meantime. "
        "First such row: %s"
        % (len(single), len(pairs), ", ".join(ids), single[0][1]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rows", action="append", required=True,
                    help="rows.jsonl path (repeatable)")
    ap.add_argument("--out", required=True, help="output report path (.md)")
    args = ap.parse_args(argv)
    rows = list(load_rows(args.rows))
    refusal = refuse_single_run(rows)
    if refusal is not None:
        print(refusal, file=sys.stderr)
        return 2
    ev = evaluate(iter(rows))
    path = emit(ev, args.out)
    print("F evaluation written -> %s" % path)
    # The verdict, restated on stdout for the operator.
    for pe in (ev.surface, ev.comparative):
        if pe is not None:
            print("  %s: %s%s" % (pe.name, pe.verdict,
                                  " -- %s" % pe.template
                                  if pe.template else ""))
    if ev.death is not None and ev.death.fired:
        print("  DEATH CONDITION FIRED: %s" % ev.death.detail)
    return 0


if __name__ == "__main__":
    sys.exit(main())
