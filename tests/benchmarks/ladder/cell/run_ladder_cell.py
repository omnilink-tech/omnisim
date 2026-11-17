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

"""One graded capability-ladder cell, end to end.

    python tests/benchmarks/ladder/cell/run_ladder_cell.py \\
        --column omnisim --task T1_arrive --model claude-opus-5

    python tests/benchmarks/ladder/cell/run_ladder_cell.py \\
        --column mujoco --task T1_arrive --dry-run      # no quota spent

A **cell** is ``capability-ladder-plan.md`` §2's unit: one autonomous agent,
one sentence, the tier's ``container/``, no help. Everything the ladder has
measured so far came from scripted oracles, which are explicitly *not* cells.
The sequence, and where each piece comes from:

1. **Preflight** -- ``claude --version`` recorded; a trivial authenticated call
   proves the login and reports the CLI's own default model, which is recorded
   as drift evidence and **never** used as the pin. Reused from
   ``cc_lane.run_cc_cell``.
2. **Workspace** -- a fresh instance of the column's template
   (``stage_ladder_workspace``), the tier's declared ``container/`` staged into
   it, and the **quarantine verified before the session runs**. A hit here does
   not spend the quota: the cell is INVALID and says why.
3. **Session** -- ``claude -p "<prompt.txt>" --output-format json --model
   <PINNED> --dangerously-skip-permissions``, cwd = the workspace, scrubbed
   env. The model pin is an explicit flag with a package default of
   ``claude-opus-5`` and **no silent drift**: what was pinned and what the
   session reported are recorded as two separate fields in every row.
4. **Hygiene** -- the paid-for sweeps, all reused: workspace stragglers, the
   canonical harness ports, and anything the session wrote through the
   ``projects/`` junction into the real repo. ⚠ The junction sweep runs
   **after grading**, not before it: it preserves-then-DELETES, and running
   it first deletes the deliverable out of the junctioned tree (measured,
   2026-08-02 -- see :func:`discover_deliverable`).
5. **Deliverable** -- discovered by the column's declared convention (a
   ``.wbt`` for the engine columns, a ``scene.xml`` directory for MuJoCo),
   **in the agent's workspace**, links followed only as a second pass. A
   repo-swept artifact is evidence and is never gradeable output; a
   deliverable that cannot be identified unambiguously makes the cell
   ``INVALID`` with a named :data:`INSTRUMENT_BLOCKERS` code, never
   ``not_achieved``. A column with no declared convention is
   ``scaffolding_defect_ours``.
6. **Phase B + grading** -- the REAL tier grader
   (``ladder.graders.t1..t4.run_and_grade``), which re-runs the deliverable
   cold and standalone with the grader's own sampler and no agent present.
   Nothing here decides a threshold.
7. **Row** -- ``capability-ladder-plan.md`` §3's cell: the value
   (``achieved`` / ``not_achieved(blocker)`` / ``NOT_EXPRESSIBLE``), the cost
   fields with unmeasured as ``null``, the provenance sub-labels (``reuse_class``
   stays ``null`` and the row says a cell published with it null is not
   publishable), the tier's own required measurements (T4's
   ``distance_to_termination_m`` + ``termination_cause``, the support cell,
   T2's ``hold_mechanism``), and the grader's assertions verbatim. **Written
   and fsynced BEFORE any teardown** -- that ordering is a lesson somebody
   already paid for: a teardown that raised on a locked log file once lost a
   cell's earned row with the quota already spent.
8. **Publish** -- through ``run_agentbench.publish_run``, the standing "no row,
   no result" path (§3.4 rule 5).

Four rules decide code paths here, not just prose:

* **no fabricated metric** -- unmeasured is ``null`` with the reason beside it;
* **a channel WE never built is ``scaffolding_defect_ours``** and may never be
  published as somebody else's capability gap (§4's binding rule). Implemented
  literally: when every failed assertion is one the column's missing channels
  break, that is the blocker;
* **an answer key in front of the agent makes the cell INVALID** -- checked on
  the staged tree before the session and on the session's own transcript
  after it;
* **an instrument defect is never an agent-attributable verdict.** A cell whose
  deliverable we cannot identify is ``INVALID`` with a named
  :data:`INSTRUMENT_BLOCKERS` code, not ``not_achieved(<blocker>)``. Added
  2026-08-02 after the first paid cell published
  ``not_achieved(install_failed)`` for a phase B we had handed the wrong file.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LADDER = HERE.parent
BENCHMARKS = LADDER.parent
REPO = BENCHMARKS.parents[1]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import concurrency                 # noqa: E402
from agentbench.cc_lane import run_cc_cell as cc_cell      # noqa: E402
from agentbench.common.paths import ob_results             # noqa: E402
from ladder import tasks as ladder_tasks                   # noqa: E402
from ladder.cell import oracles                            # noqa: E402
from ladder.cell import stage_ladder_workspace as staging  # noqa: E402

SUITE = "ladder/v0"
CONDITION = "claude_code"
DRY_CONDITION = "scripted_oracle"
METRICS_SOURCE_AGENT = "claude_code_headless_json"
METRICS_SOURCE_DRY = "scripted_oracle_dry_run"

#: The pin. ``capability-ladder-plan.md`` §6.4 requires a model id recorded in
#: every row and forbids substituting an instrument to beat a quota. The
#: ladder has ZERO cells, so choosing the pin now is a pre-data choice and
#: legitimate; changing it later is a new campaign, and the campaign driver
#: refuses to resume across a change.
DEFAULT_MODEL = "claude-opus-5"

DEFAULT_RATE_LIMIT_BACKOFF_S = cc_cell.DEFAULT_RATE_LIMIT_BACKOFF_S
DEFAULT_MAX_RATE_LIMIT_RETRIES = cc_cell.DEFAULT_MAX_RATE_LIMIT_RETRIES

#: §3.3, verbatim. Nothing outside this tuple may be written as a blocker.
BLOCKERS = ("no_importer", "no_physics_capability", "no_measurement_surface",
            "agent_lost", "budget_exhausted", "resource_limit",
            "install_failed", "scaffolding_defect_ours", "unknown")
#: §3.1, verbatim.
CELL_VALUES = ("achieved", "not_achieved", "NOT_EXPRESSIBLE")

REUSE_CLASS_NOTE = (
    "reuse_class is decided by a HUMAN REVIEWER and never by a grader "
    "(capability-ladder-plan.md 9 Q3). It is null here. A cell published "
    "with it null is NOT PUBLISHABLE, and no prose sentence about this cell "
    "may omit the word (section 8).")

GPU_HOURS_NOTE = (
    "gpu_hours is null because nothing in this cell measured GPU time. Null "
    "is not zero: a session that trained on a GPU inside its own workspace "
    "would also read null here, and the cell says so rather than claiming 0.")

#: What each column's deliverable looks like. A column absent from this map
#: has no declared convention, and a cell on it blocks with
#: ``scaffolding_defect_ours`` rather than guessing at a file.
#:
#: ``workspace_path`` is the field that repairs the 2026-08-02 T2 defect: it
#: declares that this column's deliverable LIVES IN THE AGENT'S WORKSPACE, so
#: a file recovered from the real repo by the junction-artifact sweep is
#: EVIDENCE and never gradeable output. Every column in the ladder's roster
#: declares one; the field exists so the rule is a declaration a reader can
#: check rather than an assumption buried in a walk.
DELIVERABLE_RULES = {
    "omnisim": {"kind": "world", "suffix": ".wbt", "workspace_path": True,
                "what": "a .wbt scene the session authored"},
    "webots": {"kind": "world", "suffix": ".wbt", "workspace_path": True,
               "what": "a .wbt scene the session authored"},
    "mujoco": {"kind": "scene_dir", "marker": "scene.xml",
               "driver": "drive.py", "workspace_path": True,
               "what": "a directory holding scene.xml (and, where the tier "
                       "needs one, drive.py) -- the shape "
                       "ladder.adapters.mujoco.runner_t2/_t3 re-run"},
}

#: Named INSTRUMENT blockers. These are **not** ``capability-ladder-plan.md``
#: §3.3 blockers: §3.3's taxonomy explains a ``not_achieved`` cell, and an
#: instrument defect never produces one -- it produces an ``INVALID`` cell
#: whose value and blocker are both ``null`` (§3.1). What was missing until
#: 2026-08-02 was a NAME for the defect on an INVALID row, so a reviewer read
#: a paragraph instead of a code. These are those names; they ride in
#: ``cell.instrument_blocker`` and are owned by US, always.
INSTRUMENT_BLOCKERS = (
    # >1 candidate deliverable and no rule that separates them
    "deliverable_ambiguous",
    # the session's only deliverable-shaped output was recovered from the REAL
    # REPO by the junction sweep: preserved as evidence, not gradeable output
    "deliverable_not_in_workspace",
    # the column declares no workspace path at all, so nothing above applies
    "deliverable_convention_missing",
)

INSTRUMENT_BLOCKER_OWNER = (
    "US -- the instrument. An instrument blocker is never the simulator's and "
    "never the agent's, and a cell carrying one is INVALID (no cell value, no "
    "§3.3 blocker) rather than not_achieved, which would be an "
    "agent-attributable verdict on a measurement we broke.")

#: ``.wbt`` basenames that are TOOL OUTPUT rather than an authored scene, and
#: may never be collected as a deliverable.
#:
#: ⚠ **Measured, 2026-08-02, omnisim/T2 r0.** A session's own parameter sweep
#: wrote six ``.omnisim_probe_<cfg>.wbt`` variants; the newest of them was
#: newer than the world the agent actually authored, a newest-first rule
#: collected it, and phase B graded a throwaway probe. The dot-prefix is this
#: tree's own convention for a generated sibling -- ``run-headless
#: --fail-on-runaway`` writes ``.omnisim_runaway_<stem>.wbt`` (AGENTS.md §3b)
#: -- so a leading ``.`` is a positive signal that a file is machinery, not a
#: deliverable. ``_agentbench_`` is the injected-sampler prefix and was
#: already excluded.
def is_tool_generated_world(name):
    """True when this ``.wbt`` basename is machinery, not an authored scene."""
    name = str(name)
    return name.startswith(".") or name.startswith("_agentbench_")


#: Two candidates whose mtimes fall inside this window are not separated by
#: "newest": on a session that writes a scene and immediately copies it, the
#: order is a filesystem race. Inside the window the cell refuses to guess and
#: goes INVALID with :data:`INSTRUMENT_BLOCKERS` ``deliverable_ambiguous``,
#: which costs a re-run; outside it, newest is a real ordering.
DELIVERABLE_TIE_WINDOW_S = 2.0

#: Additional env prefixes the child must never see, on top of ``cc_lane``'s.
EXTRA_SCRUB_PREFIXES = ("LADDER_",)


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# --- pure helpers (unit-tested with no Claude, no engine, no GPU) ------------


def scrub_env(base=None):
    """``(child_env, removed)`` -- ``cc_lane``'s scrub plus ``LADDER_*``.

    ``CLAUDE_CONFIG_DIR`` stays (it locates the subscription login).
    ``ANTHROPIC_API_KEY`` stays absent by design.
    """
    env, removed = cc_cell.scrub_env(base)
    for k in list(env):
        if k.startswith(EXTRA_SCRUB_PREFIXES):
            env.pop(k)
            removed.append(k)
    return env, sorted(set(removed))


def _walk_world_candidates(workspace, start_ts, *, follow_links):
    """``[(mtime, path)]`` -- session-written ``.wbt`` under ``workspace``.

    ``follow_links=False`` prunes junctions explicitly (``os.walk`` descends
    Windows junctions even with ``followlinks=False``: they are not
    ``islink()``). ``follow_links=True`` is the junction pass -- the template
    junctions ``projects/`` into the real repo and an agent asked to author a
    world naturally saves it under ``projects/.../worlds/``. **The path it
    returns is still a WORKSPACE path**, which is the whole point: a scene's
    asset references (``url "../../../../robots/..."``) and its controller
    resolve relative to the file, so grading it where the agent wrote it is
    the only place they resolve.
    """
    ws = Path(workspace)
    cands, seen = [], set()
    for dirpath, dirnames, filenames in os.walk(ws, followlinks=follow_links):
        if follow_links:
            # never loop through a junction cycle, and never descend the
            # binary runtime tree (huge, holds no authored worlds)
            dirnames[:] = [d for d in dirnames
                           if d not in ("msys64", ".git")]
            try:
                key = os.path.realpath(dirpath)
            except OSError:
                key = dirpath
            if key in seen:
                dirnames[:] = []
                continue
            seen.add(key)
        else:
            dirnames[:] = [d for d in dirnames
                           if not staging.is_link(Path(dirpath) / d)]
        for f in filenames:
            if not f.endswith((".wbt", ".omniworld")) or is_tool_generated_world(f):
                continue
            p = Path(dirpath) / f
            try:
                mt = p.stat().st_mtime
            except OSError:
                continue
            if mt >= start_ts:
                cands.append((mt, p))
    cands.sort(reverse=True)
    return cands


def _pick(cands, pass_name):
    """``(path, ambiguity_or_None)`` from one pass's sorted candidate list."""
    if not cands:
        return None, None
    gap = (cands[0][0] - cands[1][0]) if len(cands) > 1 else None
    if gap is not None and gap < DELIVERABLE_TIE_WINDOW_S:
        tied = [str(p) for mt, p in cands
                if (cands[0][0] - mt) < DELIVERABLE_TIE_WINDOW_S]
        return cands[0][1], (
            "%d candidate deliverables in the %s pass are within %.1f s of "
            "each other, so 'newest' does not separate them: %s"
            % (len(tied), pass_name, DELIVERABLE_TIE_WINDOW_S,
               ", ".join(sorted(tied))))
    return cands[0][1], None


def swept_world_evidence(swept_roots, start_ts):
    """``.wbt`` files the junction sweep preserved. **Evidence, never output.**

    Kept as a separate function from the discovery walk so the distinction is
    structural rather than a comment: nothing here can be returned as a
    deliverable. Its only job is to let the runner say *"the session's output
    ended up in the real repo"* in the row instead of *"the session left no
    deliverable"*, which is a different -- and agent-attributable -- claim.
    """
    out = []
    for root in swept_roots or ():
        root = Path(root)
        if not root.is_dir():
            continue
        for p in sorted([*root.rglob("*.omniworld"), *root.rglob("*.wbt")]):
            if not p.is_file():
                continue
            try:
                mt = p.stat().st_mtime
            except OSError:
                continue
            if mt >= start_ts:
                out.append(str(p))
    return out


def discover_deliverable(column, workspace, start_ts, *, swept_roots=()):
    """``(path_or_None, kind, rule_note, diag)`` for one column's convention.

    ⚠ **Rewritten 2026-08-02 on a measured defect that cost a paid cell**
    (``results/ladder_cell/20260802_155111_omnisim_T2``). This used to
    delegate to ``cc_lane.discover_artifact`` with the cell's own
    ``repo_artifacts/`` passed as an ``extra_root``, which put a
    REPO-SWEPT ARTIFACT ahead of the workspace's own junction pass. On
    omnisim/T2 r0 the session authored ``omniarm6_block_in_bin.wbt`` and then
    ran a six-config parameter sweep that wrote six
    ``.omnisim_probe_*.wbt`` variants; all seven were swept out of the real
    repo, "newest" picked the last probe, phase B re-ran a throwaway, every
    T2 channel read unanswered on a column the readiness audit had just
    certified READY, and the cell recorded ``not_achieved(install_failed)``
    -- an agent-attributable verdict for a defect that was entirely ours.

    Three rules now, in order, each of which that cell falsifies:

    1. **The workspace outranks everything.** Pass 1 walks the workspace
       without following links; pass 2 walks it THROUGH its junctions. Both
       return workspace paths, so the deliverable is graded where the agent
       wrote it and its relative asset/controller references resolve.
    2. **A swept repo artifact is never the deliverable.** It is preserved as
       evidence (that preservation is how this defect was caught) and is
       reported in ``diag['swept_evidence']``, but the only column that could
       ever be graded from one is a column that declares no workspace path --
       and no column in the roster does, so the branch is a guarded refusal
       rather than a fallback.
    3. **Tool output is not a deliverable.** :func:`is_tool_generated_world`.

    ``diag`` carries ``{source, candidates, swept_evidence, ambiguous,
    ambiguity_reason, instrument_blocker}``. A non-null ``ambiguity_reason``
    means the cell must be INVALID with the named instrument blocker; see
    :func:`classify`.
    """
    diag = {"source": None, "candidates": [], "swept_evidence": [],
            "ambiguous": False, "ambiguity_reason": None,
            "instrument_blocker": None}
    rule = DELIVERABLE_RULES.get(column)
    if rule is None:
        diag["instrument_blocker"] = "deliverable_convention_missing"
        return None, None, (
            "no deliverable convention is declared for the %r column, so "
            "nothing could be collected. The blocker is OURS." % column), diag

    diag["swept_evidence"] = swept_world_evidence(swept_roots, start_ts)

    if rule["kind"] == "world":
        for pass_name, follow in (("workspace", False),
                                  ("workspace-through-junction", True)):
            cands = _walk_world_candidates(workspace, start_ts,
                                           follow_links=follow)
            if not cands:
                continue
            diag["candidates"] = [str(p) for _mt, p in cands]
            path, ambiguity = _pick(cands, pass_name)
            diag["source"] = pass_name
            if ambiguity:
                diag.update(ambiguous=True, ambiguity_reason=ambiguity,
                            instrument_blocker="deliverable_ambiguous")
                return path, "world", ambiguity, diag
            return path, "world", (
                "newest authored .wbt modified after session start, found in "
                "the %s pass (%d candidate(s)); tool-generated worlds "
                "(dot-prefixed, _agentbench_*) excluded, and repo-swept "
                "artifacts are evidence only" % (pass_name, len(cands))), diag
        return (None, "world", _no_workspace_note(diag, ".wbt"),
                _mark_swept(diag))

    if rule["kind"] == "scene_dir":
        marker = rule["marker"]
        cands = []
        for dirpath, dirnames, filenames in os.walk(Path(workspace)):
            dirnames[:] = [d for d in dirnames
                           if not staging.is_link(Path(dirpath) / d)]
            if marker not in filenames:
                continue
            p = Path(dirpath) / marker
            try:
                mt = p.stat().st_mtime
            except OSError:
                continue
            if mt >= start_ts:
                cands.append((mt, Path(dirpath)))
        cands.sort(reverse=True)
        if cands:
            diag["candidates"] = [str(p) for _mt, p in cands]
            path, ambiguity = _pick(cands, "workspace")
            diag["source"] = "workspace"
            if ambiguity:
                diag.update(ambiguous=True, ambiguity_reason=ambiguity,
                            instrument_blocker="deliverable_ambiguous")
                return path, "scene_dir", ambiguity, diag
            return path, "scene_dir", (
                "newest directory holding %s, modified after session start "
                "(%d candidate(s))" % (marker, len(cands))), diag
        return None, "scene_dir", (
            "no %s written during the session anywhere under the workspace"
            % marker), _mark_swept(diag)

    diag["instrument_blocker"] = "deliverable_convention_missing"
    return None, rule["kind"], (
        "the %r deliverable kind declared for column %r has no discovery "
        "implementation. The blocker is OURS." % (rule["kind"], column)), diag


def _no_workspace_note(diag, what):
    if diag["swept_evidence"]:
        return ("no authored %s under the workspace, but the junction sweep "
                "preserved %d from the REAL REPO -- the session wrote its "
                "output through the projects/ junction and the sweep removed "
                "it before discovery. A swept artifact is evidence, not "
                "gradeable output (its relative asset and controller "
                "references do not resolve from the quarantine copy), so "
                "nothing was collected" % (what, len(diag["swept_evidence"])))
    return ("no authored %s written during the session anywhere under the "
            "workspace" % what)


def _mark_swept(diag):
    """Swept-only evidence is an INSTRUMENT condition, not ``agent_lost``."""
    if diag["swept_evidence"]:
        diag.update(ambiguous=True,
                    instrument_blocker="deliverable_not_in_workspace",
                    ambiguity_reason=(
                        "the session's only deliverable-shaped output (%d "
                        "file(s)) was recovered from the real repo by the "
                        "junction-artifact sweep and is preserved as "
                        "evidence; it is not gradeable output, and the cell "
                        "cannot be attributed to the agent"
                        % len(diag["swept_evidence"])))
    return diag


def unanswered_assertion_ids(verdict):
    """Assertion ids that only OUR missing channels could have failed.

    Read off the verdict's own ``unanswered_channels`` measurement, which the
    tier entry points populate from the adapter shim. Empty when the column
    answered everything (or when the tier has no channel map, as T1 does not).
    """
    if verdict is None:
        return set()
    gaps = (verdict.measurements or {}).get("unanswered_channels") or {}
    out = set()
    for aids in gaps.values():
        out.update(aids or ())
    return out


def classify(*, session_status, deliverable_found, deliverable_rule_known,
             grade_error, verdict, phase_b_broke=False,
             deliverable_ambiguity=None):
    """``(cell_value, blocker, basis)`` per ``capability-ladder-plan.md`` §3.

    Deliberately a small, ordered, readable rule set rather than a judgement:
    a reviewer must be able to re-derive the label from the row, and §3.4
    lets them re-label it. Every branch names its own basis string, which
    rides in the row beside the code.

    ``(None, None, basis)`` means **INVALID**: no cell value and no §3.3
    blocker, because §3.3's codes explain a ``not_achieved`` cell and an
    INVALID one is not that. The runner turns the basis into
    ``cell.invalid_reason`` and pairs it with the named
    :data:`INSTRUMENT_BLOCKERS` code.
    """
    # ⚠ FIRST, deliberately -- ahead of every agent-attributable branch.
    # A deliverable we cannot identify is a measurement WE broke, and the
    # branch below it ("the session ended without leaving a deliverable")
    # reads as agent_lost, which blames the agent for our discovery walk.
    # Measured 2026-08-02 (omnisim/T2 r0): the runner graded a throwaway
    # probe world and published not_achieved(install_failed).
    if deliverable_ambiguity:
        return (None, None,
                "the deliverable could not be identified unambiguously, so "
                "the cell is INVALID rather than not_achieved: %s"
                % deliverable_ambiguity)
    if session_status == "timeout":
        return ("not_achieved", "budget_exhausted",
                "the session hit the timeout cap (SPEC 2: timeout = 3 x par); "
                "a FAIL, not INVALID -- and the row's session_end block says "
                "so in its own words: the agent was WORKING when the cap "
                "landed, it did not fail at the task, and the cap is the "
                "shared budget the tier pre-registered")
    if session_status == "launch_failed":
        return ("not_achieved", "scaffolding_defect_ours",
                "the agent process never produced a result document -- our "
                "harness, not the simulator")
    if session_status == "error":
        return ("not_achieved", "unknown",
                "the session returned an error that is not a usage limit; not "
                "diagnosed, and a reviewer may re-label it (3.4)")
    if session_status == "oracle_missing":
        return ("not_achieved", "scaffolding_defect_ours",
                "no committed scripted oracle for this (tier, column); a hole "
                "in OUR scaffolding")
    if session_status == "oracle_failed":
        return ("not_achieved", "scaffolding_defect_ours",
                "the scripted oracle itself failed; ours by construction")
    if not deliverable_found:
        if not deliverable_rule_known:
            return ("not_achieved", "scaffolding_defect_ours",
                    "no deliverable convention is declared for this column, "
                    "so nothing could be collected")
        return ("not_achieved", "agent_lost",
                "the session ended without leaving a deliverable of the "
                "column's own declared shape")
    if grade_error:
        return ("not_achieved", "scaffolding_defect_ours",
                "grading could not run on this column: %s" % grade_error)
    if verdict is None:
        return ("not_achieved", "unknown",
                "no verdict was produced and no cause was diagnosed")
    if verdict.outcome in ("INVALID", "ERROR"):
        return (None, None,
                "the run is unattributable or the stack broke; an "
                "unattributed physics result is not a result (T*.5), so this "
                "is INVALID rather than a blocker")
    if verdict.outcome == "NOT_EXPRESSIBLE":
        return ("NOT_EXPRESSIBLE", None,
                "the grader itself declared the tier inexpressible here")
    if verdict.outcome == "PASS":
        return ("achieved", None,
                "every tier assertion held in phase B on this run")
    failed = set(verdict.failed)
    ours = unanswered_assertion_ids(verdict)
    if failed and ours and failed <= ours:
        return ("not_achieved", "scaffolding_defect_ours",
                "every red assertion (%s) depends only on channels this "
                "column could not answer; a red that belongs to our "
                "scaffolding is never published as a finding about the "
                "simulator (4's binding rule)" % ", ".join(sorted(failed)))
    if phase_b_broke:
        return ("not_achieved", "install_failed",
                "the deliverable was collected but the standalone re-run "
                "never produced evidence; owned by our scaffolding until "
                "proven otherwise")
    if verdict.progress <= 1:            # no_artifact / artifact_invalid
        return ("not_achieved", "agent_lost",
                "the deliverable did not load as a valid scene (progress=%s)"
                % verdict.progress)
    if verdict.progress >= 3:            # artifact_runs / graded_pass
        return ("not_achieved", "no_physics_capability",
                "the deliverable ran and the tier's physical assertions "
                "measured a shortfall (%s)" % ", ".join(sorted(failed)))
    return ("not_achieved", "unknown",
            "not diagnosed; 3.3 caps a column at one third unknown before it "
            "is published as 'not diagnosed' rather than as findings")


def build_row(*, task, column, repeat, cell_value, blocker, blocker_basis,
              verdict, metrics, agent_block, workspace_block, artifacts,
              notes, deviations, machine, dry_run, t_total_s,
              answer_key, invalid_reason=None, not_expressible=None,
              phase_b=None, deliverable_note=None, grade_path=None,
              measured_under_concurrency=False, concurrency_block=None,
              instrument_blocker=None, deliverable_diag=None,
              session_end=None, session_cap=None, session_cap_source=None,
              post_cap_verdict=None):
    """One ladder cell row. Every §3.2 field, unmeasured as ``null``.

    ``post_cap_verdict`` is the grader's reading of a CAPPED session's
    deliverable. It rides in its own block and **never** in ``outcome``: the
    cell has no authoritative outcome, because the agent was cut off rather
    than finished, and a ``PASS`` in the outcome column would be counted as a
    win by the first person to scan the grid. It exists to separate "capped
    while flailing" from "capped after finishing", which is a real and large
    difference that the row could not previously express.
    """
    vd = verdict.as_dict() if verdict is not None else None
    meas = (vd or {}).get("measurements") or {}
    outcome = (vd or {}).get("outcome")
    notes = list(notes or [])
    if invalid_reason:
        outcome = "INVALID"
        cell_value, blocker = None, None
        if instrument_blocker:
            # An INVALID cell's own verdict block (unanswered channels, failed
            # assertions, the grader's notes) describes whatever the
            # instrument fed the grader, NOT the column. Say so first, in the
            # row, so nobody quotes it.
            notes.insert(0, (
                "INSTRUMENT BLOCKER %r: this cell is INVALID and every "
                "verdict field below it -- failed assertions, "
                "unanswered_channels, the grader's own notes about channels "
                "'this column could not answer' -- describes what the "
                "instrument handed the grader and is NOT a statement about "
                "the simulator, the column or the agent."
                % instrument_blocker))
    row = {
        "suite": SUITE,
        "schema": "ladder_cell/v0",
        "task": task.id,
        "rung": task.rung,
        "tier": task.meta.get("tier", "capability_ladder"),
        "column": column,
        "sim": column,                       # tools that key on `sim`
        "condition": DRY_CONDITION if dry_run else CONDITION,
        "dry_run": bool(dry_run),
        "repeat": repeat,
        "cell_key": "%s:%s:r%d" % (task.id, column, repeat),
        "scoring_class": task.meta.get("scoring_class", "exploratory"),
        "achieved_rule": task.meta.get("achieved_rule"),
        "par_s": task.par_s,
        "timeout_s": task.timeout_s,
        "session_cap_s": session_cap,
        "session_cap_source": session_cap_source,
        "session_end": session_end,

        # --- 3.1: the cell value ------------------------------------------
        "cell": {
            "value": cell_value,
            "blocker": blocker,
            "blocker_basis": blocker_basis,
            "blocker_owner": _BLOCKER_OWNER.get(blocker),
            "blocker_evidence": {
                "failed_assertions": (vd or {}).get("failed_assertions"),
                "deliverable": (artifacts or {}).get("deliverable"),
                "deliverable_rule": deliverable_note,
                "deliverable_discovery": deliverable_diag,
                "grade_path": grade_path,
                "phase_b": phase_b,
            },
            "not_expressible": not_expressible,
            "invalid_reason": invalid_reason,
            "instrument_blocker": instrument_blocker,
            "instrument_blocker_owner": (INSTRUMENT_BLOCKER_OWNER
                                         if instrument_blocker else None),
            "review": {"relabelled_by": None, "relabelled_utc": None,
                       "rule": "capability-ladder-plan.md 3.4 -- a reviewer "
                               "may re-label this cell; the original stays "
                               "visible with its date"},
        },

        # --- the grader's own verdict, verbatim ----------------------------
        "outcome": outcome,
        "progress": (vd or {}).get("progress"),
        "progress_name": (vd or {}).get("progress_name"),
        "assertions": {k: a["ok"] for k, a in
                       ((vd or {}).get("assertions") or {}).items()},
        "assertion_detail": (vd or {}).get("assertions"),
        "failed_assertion": (vd or {}).get("failed_assertion"),
        "vacuous_assertions": (vd or {}).get("vacuous_assertions"),
        "basis": (vd or {}).get("basis"),
        "attribution": (vd or {}).get("attribution"),
        "measurements": meas,
        "self_verified": (vd or {}).get("self_verified"),
        "unanswered_channels": meas.get("unanswered_channels"),

        # --- a CAPPED session's deliverable, graded but NOT the cell -------
        "post_cap_grading": (None if post_cap_verdict is None else {
            "outcome": post_cap_verdict.as_dict().get("outcome"),
            "failed_assertion":
                post_cap_verdict.as_dict().get("failed_assertion"),
            "assertions": {k: a["ok"] for k, a in
                           (post_cap_verdict.as_dict().get("assertions")
                            or {}).items()},
            "measurements": post_cap_verdict.as_dict().get("measurements"),
            "authority": "NOT the cell's outcome. The session was CUT OFF by "
                         "the cap, so this grades whatever the agent happened "
                         "to have on disk at that moment -- which may be "
                         "finished work, or may be mid-edit. The cell stays "
                         "not_achieved(budget_exhausted). Quote this only as "
                         "'the deliverable present when the cap landed graded "
                         "X', never as a tier result.",
        }),

        # --- 3.2: cost, unmeasured as null --------------------------------
        "metrics": dict(metrics),
        "metrics_source": (METRICS_SOURCE_DRY if dry_run
                           else METRICS_SOURCE_AGENT),
        "metrics_notes": {"gpu_hours": GPU_HOURS_NOTE,
                          "tokens": "tokens_in / tokens_out / "
                                    "tokens_cache_read are separate fields "
                                    "and are NEVER summed (3.2)"},
        "measured_under_concurrency": bool(measured_under_concurrency),
        "concurrency": concurrency_block,
        "interventions": 0,
        "interventions_rule": (
            "structurally 0 (SPEC 3.4): there is no human channel. The one "
            "scoped exception is operator provisioning of the venue BEFORE "
            "the first message, recorded in venue; an operator touching the "
            "run after it makes the cell INVALID."),

        # --- 3.2: the provenance sub-labels -------------------------------
        "provenance": {
            "reuse_class": meas.get("reuse_class"),
            "reuse_class_note": REUSE_CLASS_NOTE,
            "method": meas.get("method"),
            "hold_mechanism": meas.get("hold_mechanism"),
            "external_support": meas.get("external_support"),
            "support_attestation": meas.get("support_attestation"),
            "support_cell": meas.get("cell"),
            "support_cell_text": meas.get("cell_text"),
            "arena_attestation": meas.get("arena_attestation"),
            "distance_to_termination_m": meas.get("distance_to_termination_m"),
            "termination_cause": meas.get("termination_cause"),
            "termination": meas.get("termination"),
            "excluded_from_comparison": meas.get("excluded_from_comparison"),
            "comparison_exclusions": meas.get("comparison_exclusions"),
            "venue": machine,
        },

        # --- the instrument -------------------------------------------------
        "agent": agent_block,
        "workspace": workspace_block,
        "answer_key_exposure": answer_key,

        "expectation": task.meta.get("expectation"),
        "artifacts": artifacts,
        "deviations": list(deviations or []),
        # An instrument blocker's disclaimer goes FIRST, ahead of the
        # grader's own notes, because those notes are exactly what it warns
        # the reader not to quote.
        "notes": (list(notes) + list((vd or {}).get("notes") or [])
                  if instrument_blocker
                  else list((vd or {}).get("notes") or []) + list(notes)),
        "machine": machine,
        "t_total_s": t_total_s,
        "utc": _utc(),
    }
    return row


# --- the session cap, and how a session actually ended ------------------------

#: The cap used when a task declares no ``timeout_s`` of its own. **Named,
#: not a bare literal in two call sites** (it used to be ``3600.0``, inline,
#: in both the session launch and the phase-B launch, where nothing recorded
#: which one a row was run under). SPEC §2's rule is ``timeout = 3 x par``;
#: every ladder task declares ``par_s`` and ``timeout_s`` explicitly, so this
#: is a floor for a task that forgets, and the row always says which source
#: it used.
DEFAULT_SESSION_CAP_S = 5400.0

#: **Measured consumption, so the next reader sizes a cap instead of guessing
#: one.** One row per (tier, column) actually run. This is data, not a
#: setting: nothing reads it at run time.
MEASURED_SESSION_COST = {
    ("T2_transfer", "omnisim"): {
        "cell": "results/ladder_cell/20260802_155111_omnisim_T2",
        "par_s": 1800, "cap_s": 5400,
        "t_agent_s": 2266.8, "tool_calls": 63, "turns": 64, "usd": 6.36,
        "tool_wall_s": 1465.7,
        "engine_wall_s": 1455.8,          # 5 headless runs: 21.4 + 486.7 +
        #                                   335.1 + 12.2 + 600.4 (cut)
        "cap_hit": False,
        "what_ended_it": (
            "NOT the cap. At 2266.8 s of a 5400 s cap (42%) the session's "
            "sixth engine run passed the CLI's own 10-minute foreground "
            "limit, the CLI auto-backgrounded it ('Command running in "
            "background with ID: bdinwal2p ... You will be notified when it "
            "completes'), and the agent ended its turn to wait for a "
            "notification that headless `claude -p` never delivers. "
            "cc_stdout.json: terminal_reason=completed, stop_reason=end_turn, "
            "timed_out=false."),
        "projected_to_finish_s": "3200-3800",
        "reading": (
            "the 5400 s cap (3 x par) is ADEQUATE for T2/omnisim and is not "
            "what to change: an attempt that finishes projects to 3200-3800 s "
            "(the run had completed its authored world, its controller, an "
            "honest instrumented carry and two parameter sweeps by 2267 s, "
            "and had a sweep plus a negative control left). par_s = 1800 is "
            "the number this run strains -- it consumed 1.26 x par without "
            "finishing -- but par is pre-registered and changing it is a task "
            "decision, not a runner one."),
    },
}

#: A Claude Code tool result that hands a long-running command off to the
#: background. In headless ``-p`` there is no turn to be notified in, so an
#: agent that yields on one of these ENDS THE CELL while still working.
_BG_HANDOFF_RE = r"Command running in background with ID:\s*([A-Za-z0-9_-]+)"
#: Anything that would tell the agent the handed-off command finished.
_BG_DONE_RE = r"(?:Exit code|exited with code|has completed|completed with|" \
              r"Background task .* (?:completed|failed))"


def session_cap(task, override=None):
    """``(cap_s, source)`` -- the session cap, explicit and recorded.

    Order: an operator ``--timeout-s`` override, then the task's own
    ``timeout_s`` (SPEC §2's ``3 x par``), then :data:`DEFAULT_SESSION_CAP_S`.
    The source string goes in the row so "what cap was this run under" is
    answerable from the row alone.
    """
    if override is not None:
        return float(override), "operator --timeout-s override"
    if getattr(task, "timeout_s", None):
        return float(task.timeout_s), (
            "the task's own timeout_s (%s/meta.json; SPEC 2: 3 x par_s=%s)"
            % (getattr(task, "id", "?"), getattr(task, "par_s", "?")))
    return DEFAULT_SESSION_CAP_S, (
        "package default DEFAULT_SESSION_CAP_S -- the task declares no "
        "timeout_s, which is itself worth fixing in its meta.json")


def pending_background_tasks(transcript):
    """Background command ids the session handed off and never heard back on.

    ⚠ **The thing that actually ended the first paid ladder cell.** Reported,
    never used to change a cell value: a false positive here must cost a note
    and nothing else.
    """
    import re
    if not transcript:
        return []
    try:
        text = Path(transcript).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    handed = []
    for m in re.finditer(_BG_HANDOFF_RE, text):
        tid = m.group(1)
        tail = text[m.end():]
        done = any(tid in seg for seg in re.findall(
            r"[^\n]{0,200}" + _BG_DONE_RE + r"[^\n]{0,200}", tail))
        if not done and tid not in handed:
            handed.append(tid)
    return handed


def classify_session_end(session, cap_s, *, transcript=None):
    """A DISTINCT recorded outcome for how the session ended.

    ``status`` alone conflated three very different endings -- the cap
    landing on a working agent, the agent finishing, and the agent yielding
    to a background job that headless mode can never resume. All three read
    ``ok``/``timeout`` and none of them said whether the agent was still
    working. Returns the block the row carries verbatim.
    """
    meta = getattr(session, "meta", None) or {}
    wall = meta.get("wall_s")
    pending = pending_background_tasks(transcript)
    if getattr(session, "status", None) == "timeout" or meta.get("timed_out"):
        reason, working = "cap_reached", True
        detail = ("the %.0f s cap killed the session. The agent was WORKING, "
                  "not failing: SPEC 2 makes this budget_exhausted (a FAIL, "
                  "shared ownership), and nothing here is a statement about "
                  "the simulator or about whether the task is doable"
                  % float(cap_s))
    elif pending:
        reason, working = "background_work_pending", True
        detail = ("the session ended with %d handed-off background "
                  "command(s) (%s) still unreported. Claude Code backgrounds "
                  "a foreground command that outlives its own limit and tells "
                  "the agent it will be notified; in headless `claude -p` "
                  "there is no later turn, so the agent yields and the CELL "
                  "ENDS MID-WORK. This is NOT the cap and NOT a verdict on "
                  "the task: measured 2026-08-02 on omnisim/T2 r0 at 42%% of "
                  "the cap. A reviewer may re-label under 3.4"
                  % (len(pending), ", ".join(pending)))
    elif getattr(session, "status", None) == "ok":
        reason, working, detail = "completed", False, (
            "the agent ended its own turn with no work outstanding")
    else:
        reason, working = str(getattr(session, "status", "unknown")), False
        detail = "the session did not produce a result document"
    frac = None
    if wall and cap_s:
        frac = round(float(wall) / float(cap_s), 4)
    return {"reason": reason, "was_working": working, "wall_s": wall,
            "cap_s": float(cap_s), "fraction_of_cap": frac,
            "pending_background_tasks": pending, "detail": detail}


_BLOCKER_OWNER = {
    "no_importer": "the simulator (refutable in the correction window)",
    "no_physics_capability": "the simulator, or the state of the art",
    "no_measurement_surface": "the simulator",
    "agent_lost": "the product's docs, defaults and error messages",
    "budget_exhausted": "shared",
    "resource_limit": "us, or the hardware floor",
    "install_failed": "our scaffolding, until proven otherwise",
    "scaffolding_defect_ours": "US, and it is published as ours",
    "unknown": "us",
}


def empty_metrics():
    """Every cost field, all ``null``. Nothing here is ever defaulted to 0."""
    return {"tool_calls": None, "turns": None, "t_agent_s": None,
            "t_total_s": None, "tokens_in": None, "tokens_out": None,
            "tokens_cache_read": None, "tokens_cache_write": None,
            "gpu_hours": None, "usd": None, "interventions": 0}


def metrics_from_cc(cc, *, t_total_s=None):
    """The Claude Code figures, self-reported (``metrics_source`` says so)."""
    m = empty_metrics()
    m.update({"tool_calls": cc.get("tool_calls"),
              "turns": cc.get("num_turns"),
              "t_agent_s": cc.get("t_agent_s"),
              "t_total_s": t_total_s,
              "tokens_in": cc.get("tokens_in"),
              "tokens_out": cc.get("tokens_out"),
              "tokens_cache_read": cc.get("tokens_cache_read"),
              "tokens_cache_write": cc.get("tokens_cache_write"),
              "usd": cc.get("total_cost_usd")})
    return m


# --- writing ------------------------------------------------------------------


def write_row(run_dir, row):
    """Append + flush + fsync. Called BEFORE any teardown, always."""
    p = Path(run_dir) / "rows.jsonl"
    with open(p, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, default=str) + "\n")
        fh.flush()
        os.fsync(fh.fileno())
    return p


def write_report(run_dir, report):
    run_dir = Path(run_dir)
    (run_dir / "cell_report.json").write_text(
        json.dumps(report, indent=2, default=str), encoding="utf-8")
    lines = ["# capability-ladder cell report", ""]
    for k in ("task", "rung", "column", "condition", "dry_run",
              "model_pinned", "workspace", "prompt", "deliverable_rule",
              "blocker"):
        if k in report:
            lines.append("- **%s**: %s" % (k, report[k]))
    row = report.get("row_summary")
    if row:
        lines.append("")
        lines.append("## Cell")
        for k, v in row.items():
            lines.append("- %s: %s" % (k, v))
    (run_dir / "cell_report.md").write_text("\n".join(lines) + "\n",
                                            encoding="utf-8")


# --- the session (agent, or the dry-run oracle) -------------------------------


class SessionOutcome:
    """What the session step produced, whatever produced it."""

    def __init__(self, status, *, result=None, meta=None, cc=None,
                 oracle=None, deferrals=None, start_ts=None, end_ts=None):
        self.status = status          # ok | timeout | launch_failed | error |
        #                               oracle_missing | oracle_failed
        self.result = result
        self.meta = meta or {}
        self.cc = cc or {}
        self.oracle = oracle
        self.deferrals = deferrals or []
        self.start_ts = start_ts
        self.end_ts = end_ts


class RateLimited(SystemExit):
    """The quota is out. A PAUSE, never a failed run, never a row."""


def _recover_completed_session(workspace, run_dir):
    """``(synthetic_result, note)`` when a session ran but wrote no envelope.

    Two pieces of evidence, both required, because either alone is weak:

    * ``cc_stdout`` holds text -- the CLI printed an agent's final message, so
      an agent existed and reached the end of its turn;
    * a transcript exists for this workspace and contains assistant turns --
      the session is on disk and its cost can be counted.

    A launch that was refused produces neither. Returns ``None`` then, and the
    caller keeps ``launch_failed``, which is the truthful label for it.

    The synthesised document deliberately carries ONLY ``result`` and
    ``is_error``: the token and cost fields the real envelope would hold are
    not on disk, and inventing them would put made-up numbers in a row.
    """
    try:
        text = (Path(run_dir) / "cc_stdout.json").read_text(
            encoding="utf-8", errors="replace").strip()
    except OSError:
        return None
    if not text:
        return None
    tx = find_transcript_by_workspace(workspace)
    if tx is None:
        return None
    try:
        turns = sum(1 for line in tx.read_text(
            encoding="utf-8", errors="replace").splitlines()
            if '"role": "assistant"' in line or '"role":"assistant"' in line)
    except OSError:
        return None
    if turns < 2:
        return None
    return ({"result": text, "is_error": False, "subtype": "success",
             "_synthesised_by": "the runner, from cc_stdout + the transcript"},
            "the CLI wrote no --output-format json envelope; the session was "
            "recovered from its stdout (%d bytes) and its transcript (%d "
            "assistant turns). Token and cost figures are ABSENT rather than "
            "estimated." % (len(text), turns))


def find_transcript_by_workspace(workspace, claude_home=None):
    """The session transcript for a workspace, WITHOUT needing a session id.

    ``cc_cell.find_transcript`` keys on the ``session_id`` in the CLI's result
    document -- which a **killed** session never returns, so a capped cell used
    to lose its transcript, its tool-call count and its token totals all at
    once. The CLI also files the transcript under a slug derived from the
    session's cwd, and we know that path, so the id is recoverable from the
    workspace alone.

    Measured 2026-08-02 (omnisim/T2 r1): the cap landed 90 s after the agent
    said "task complete"; the row recorded ``metrics: all null`` while a 4.1 MB
    transcript sat on disk under this slug.
    """
    root = Path(claude_home or (Path.home() / ".claude")) / "projects"
    if not root.is_dir():
        return None
    # The CLI's slug: the absolute path with every non-alphanumeric run turned
    # into a single '-'. Derived rather than guessed at a prefix, so a
    # workspace whose name embeds a timestamp cannot match its neighbour.
    # ⚠ NO `+`. The CLI maps EACH non-alphanumeric character to its own dash
    # and does not collapse runs, so `C:\Users` becomes `C--Users` (two dashes:
    # one for the colon, one for the backslash). Collapsing produced `C-Users`,
    # which matched nothing -- measured 2026-08-03, when a 64-minute session
    # that had finished the task lost its transcript to this one character.
    slug = re.sub(r"[^A-Za-z0-9]", "-", str(Path(workspace).resolve()))
    d = root / slug
    if not d.is_dir():
        return None
    hits = sorted(d.glob("*.jsonl"), key=lambda p: p.stat().st_mtime,
                  reverse=True)
    return hits[0] if hits else None


def run_agent_session(*, prompt, workspace, env, run_dir, model, timeout_s,
                      rate_limit_backoff_s, max_rate_limit_retries,
                      version, on_defer=None, sleep=time.sleep,
                      reinstantiate=None):
    """The headless Claude Code child, with the deferral protocol.

    ``reinstantiate()`` is called before each retry to get a FRESH workspace
    (a deferred attempt must not reuse a workspace the previous try touched);
    it returns ``(workspace, prompt)``.
    """
    deferrals = []
    attempt = 0
    while True:
        start_ts = time.time()
        result, meta = cc_cell.run_claude_cell(prompt, workspace, env, run_dir,
                                               model=model, timeout_s=timeout_s)
        end_ts = time.time()
        stderr_text = meta.get("launch_error") or ""
        if result is None and not stderr_text:
            try:
                stderr_text = (Path(run_dir) / "cc_stderr.log").read_text(
                    encoding="utf-8", errors="replace")
            except OSError:
                pass
        limit = concurrency.rate_limit_reason(result, stderr_text)
        if limit is not None:
            deferrals.append({"utc": _utc(), "marker": limit,
                              "attempt": attempt,
                              "backoff_s": rate_limit_backoff_s})
            if attempt >= max_rate_limit_retries:
                raise RateLimited(
                    "usage/rate limit (%r) after %d deferred attempts: the "
                    "cell has NOT run and no row exists. This is a pause, not "
                    "a result (capability-ladder-plan.md 6.4)."
                    % (limit, len(deferrals)))
            if on_defer:
                on_defer(limit, attempt, deferrals)
            sleep(rate_limit_backoff_s)
            attempt += 1
            if reinstantiate is not None:
                workspace, prompt = reinstantiate()
            continue
        break

    if result is None:
        status = "timeout" if meta.get("timed_out") else "launch_failed"
        # ⚠ A SESSION THAT RAN IS NOT A LAUNCH FAILURE, whatever the CLI wrote.
        #
        # `run_claude_cell` returns None whenever stdout will not parse as the
        # `--output-format json` envelope. That is the right test for a launch
        # that never happened -- and the WRONG test for one that happened and
        # finished, because the CLI sometimes prints only the agent's final
        # message as plain text instead of the envelope. Measured twice on
        # 2026-08-03: a 64-minute session that BUILT the deliverable, graded
        # 4/5 when re-run by hand, and reported "The task is complete" was
        # recorded as launch_failed with every metric null and its deliverable
        # never discovered. The preflight, same flag, got proper JSON both
        # times, so this is not a mis-passed argument.
        #
        # So classify on evidence instead of on the envelope: a non-empty
        # stdout PLUS a session transcript means the CLI ran an agent to a
        # final message. Synthesise the envelope from what is on disk, keep
        # the deviation visible in the row, and let the deliverable be graded.
        if not meta.get("timed_out"):
            recovered = _recover_completed_session(workspace, run_dir)
            if recovered is not None:
                result, note = recovered
                meta = dict(meta, envelope_absent=note)
                status = "ok"
        # A killed CLI returns no result document, so the id-keyed transcript
        # lookup below cannot run -- but the transcript itself exists, and
        # everything the row wants is in it. Recover it from the workspace
        # BEFORE returning, or a capped cell reports `metrics: all null` while
        # its own record sits on disk (measured, omnisim/T2 r1, 2026-08-02).
        salvage = {"transcript": None, "tool_calls": None, "reason": None}
        try:
            tx = find_transcript_by_workspace(workspace)
            if tx is not None:
                import shutil as _sh
                _sh.copy2(tx, Path(run_dir) / "transcript.jsonl")
                salvage["transcript"] = str(Path(run_dir) / "transcript.jsonl")
                tc, tc_reason = cc_cell.count_tool_calls(tx)
                salvage["tool_calls"] = tc
                salvage["reason"] = tc_reason
            else:
                salvage["reason"] = (
                    "no transcript under ~/.claude/projects for the workspace "
                    "slug; nothing to salvage from a %s session" % status)
        except OSError as exc:
            salvage["reason"] = "transcript salvage failed: %r" % exc
        meta = dict(meta, salvage=salvage)
        cc = {"tool_calls": salvage["tool_calls"],
              "tool_calls_reason": salvage["reason"],
              "metrics_source": "salvaged from the transcript after a %s; the "
                                "CLI returned no result document, so token and "
                                "cost totals are absent by construction"
                                % status}
        return SessionOutcome(status, meta=meta, cc=cc, deferrals=deferrals,
                              start_ts=start_ts, end_ts=end_ts)

    transcript = cc_cell.find_transcript(result.get("session_id"))
    if transcript is not None:
        try:
            import shutil as _sh
            _sh.copy2(transcript, Path(run_dir) / "transcript.jsonl")
        except OSError:
            pass
        tool_calls, tc_reason = cc_cell.count_tool_calls(transcript)
    else:
        tool_calls, tc_reason = None, (
            "session transcript not found under ~/.claude/projects for "
            "session_id=%r" % result.get("session_id"))
    cc = cc_cell.cc_metrics_from_result(
        result, tool_calls=tool_calls, tool_calls_reason=tc_reason,
        version=version, permission_mode=meta.get("permission_mode"),
        transcript=transcript, cell_wall_s=meta.get("wall_s"),
        cli_command=meta.get("cli_command"))
    status = "error" if result.get("is_error") else "ok"
    return SessionOutcome(status, result=result, meta=meta, cc=cc,
                          deferrals=deferrals, start_ts=start_ts,
                          end_ts=end_ts)


def run_dry_session(*, task, column, workspace, run_dir, container_root):
    """The ``--dry-run`` stand-in: the tier's committed scripted oracle."""
    start_ts = time.time()
    res = oracles.run_oracle(task, column, workspace, run_dir, container_root)
    end_ts = time.time()
    if not res.ok and res.error and res.error.startswith("no committed"):
        status = "oracle_missing"
    elif not res.ok:
        status = "oracle_failed"
    else:
        status = "ok"
    return SessionOutcome(status, oracle=res, start_ts=start_ts, end_ts=end_ts)


# --- grading ------------------------------------------------------------------


#: ⚠ **A measured mis-routing hazard, guarded rather than silently graded.**
#: ``ladder.graders.t1`` and ``ladder.graders.t4`` call the column's GENERIC
#: ``run_standalone``. On the MuJoCo column that name is **T2's runner** -- its
#: own docstring says so ("This name is T2's; T3 has its own") -- so
#: ``t1.run_and_grade(sim="mujoco")`` and ``t4.run_and_grade(sim="mujoco")``
#: would re-run a T1 or T4 deliverable through the manipulation runner and
#: grade whatever came back. That is worse than a refusal: it produces a
#: number. Where a column's generic hook is known to belong to one tier, this
#: runner uses it ONLY for that tier and otherwise refuses with a named
#: blocker that is OURS.
GENERIC_RUNNER_TIER = {"mujoco": "T2"}


def phase_b_runner(column, rung):
    """``(callable, attr, refusal)`` -- the RIGHT phase-B runner, or a refusal.

    Preference order is the one ``ladder.graders.t3`` already established for
    itself: a tier-specific ``t<n>_run_standalone`` first, the generic
    ``run_standalone`` second, and -- new here -- a refusal when the generic
    one is known to be another tier's.
    """
    from ladder import adapters as ladder_adapters
    channels = ladder_adapters.resolve_ladder_channels(column)
    if channels is None:
        return None, None, ("the %r column ships no ladder-side channel "
                            "module, so there is no phase-B runner" % column)
    specific = "%s_run_standalone" % rung.lower()
    if hasattr(channels, specific):
        return getattr(channels, specific), specific, None
    owner = GENERIC_RUNNER_TIER.get(column)
    if owner and owner != rung:
        return None, None, (
            "the %r column's generic run_standalone is %s's runner (its own "
            "docstring says so) and there is no %s hook, so re-running a %s "
            "deliverable through it would grade the wrong tier. REFUSED: the "
            "blocker is OURS, not the simulator's." % (column, owner,
                                                       specific, rung))
    if hasattr(channels, "run_standalone"):
        return channels.run_standalone, "run_standalone", None
    return None, None, ("the %r column has no phase-B runner for %s"
                        % (column, rung))


def grade(task, column, *, deliverable, run_dir, phase_b_dir=None,
          timeout_s=None, backend=None):
    """Through the REAL tier grader. Returns ``(verdict, phase_b, error, path)``.

    Two ways in, both the grader's own:

    * ``phase_b_dir`` set -- the run already happened (the T1 MuJoCo oracle
      drives and records in one process, so there is nothing separable to
      re-run) and the grader reads it;
    * otherwise ``run_and_grade`` performs phase B: the deliverable re-run
      **cold and standalone, with no agent present**, which is the only thing
      a ladder cell may be scored from (SPEC 2.3).

    A column with no phase-B runner raises ``NotImplementedError`` inside the
    grader; that is caught and returned as an error string, because it is a
    hole in OUR scaffolding and must never read as a simulator failure. So is
    the mis-routing :func:`phase_b_runner` guards against.
    """
    grader = task.grader()
    try:
        if phase_b_dir is not None:
            verdict = grader.grade(str(phase_b_dir), task=task,
                                   artifact=(str(deliverable)
                                             if deliverable else None),
                                   phase_b=str(phase_b_dir), sim=column)
            return verdict, None, None, (
                "%s.grade(<oracle run dir>) -- the run was already performed "
                "standalone with the grader's own sampler" % grader.__name__)

        runner, attr, refusal = phase_b_runner(column, task.rung)
        if refusal:
            return None, None, refusal, None

        # Phase B is LAUNCHED here and GRADED by the grader, rather than both
        # inside ``run_and_grade``. The only difference is which hook is
        # called: ``t3.run_and_grade`` already prefers the tier-specific one
        # and its siblings do not, so delegating wholesale would send a T1 or
        # T4 deliverable into T2's runner on this column. The phase CONFIG
        # still comes from the task's own meta.json, exactly as
        # ``run_and_grade`` reads it, so the numbers a cell was graded under
        # stay in a file a reader can open.
        cfg = task.standalone
        res = runner(str(deliverable), str(Path(run_dir) / "phase_b"),
                     backend=backend,
                     duration=float(cfg.get("duration_s", 60.0)),
                     settle=float(cfg.get("settle_s", 0.5)),
                     stride=int(cfg.get("contact_stride", 2)),
                     surfaces=task.surfaces,
                     timeout_s=session_cap(task, timeout_s)[0])
        verdict = grader.grade(str(Path(run_dir) / "phase_b"), task=task,
                               artifact=str(deliverable), phase_b=res,
                               sim=column)
        verdict.artifacts["phase_b_run_dir"] = str(Path(run_dir) / "phase_b")
        return verdict, res, None, (
            "%s: phase B through the %r hook (a cold standalone re-run with "
            "the grader's own sampler and no agent), graded by %s.grade"
            % (grader.__name__, attr, grader.__name__))
    except NotImplementedError as exc:
        return None, None, repr(exc), None
    except Exception as exc:                              # noqa: BLE001
        return None, None, "the grader raised %r" % (exc,), None


def phase_b_broke(phase_b, verdict=None):
    """True only when the standalone re-run POSITIVELY produced no evidence.

    ⚠ A MISSING ATTRIBUTE IS NOT EVIDENCE OF EMPTINESS, and treating it as
    such was a fairness bug that ran in OUR favour. The old rule read
    ``getattr(phase_b, "t", None)`` and called a falsy answer broken -- but
    only one column's runner returns an object with a ``t``. MuJoCo's returns
    an exit code, so ``getattr`` yielded None on every one of its cells and
    every one was filed ``install_failed``: "our scaffolding, until proven
    otherwise".

    Measured 2026-08-04: a MuJoCo cell drove its scene with 30,250 control
    writes, answered every channel, and graded 4/5 with a real physical
    shortfall (the object was carried clear for 1.55 s of a required 3.0 s).
    That is a finding ABOUT MUJOCO, and it was being reported as a failure of
    ours -- which reads as generosity and is actually a benchmark that cannot
    see a competitor's result.

    So: an explicit error means broken; a ``t`` that is present and empty means
    broken; and a verdict carrying real measurements means NOT broken whatever
    shape the runner returned. Anything else is unknown, and unknown is not
    broken.
    """
    if phase_b is None:
        return False
    if getattr(phase_b, "error", None):
        return True
    # The grader got far enough to measure something, so the re-run ran.
    if verdict is not None:
        try:
            assertions = (verdict.as_dict() or {}).get("assertions") or {}
        except Exception:                                   # noqa: BLE001
            assertions = {}
        if any(not a.get("vacuous", False) for a in assertions.values()):
            return False
    t = getattr(phase_b, "t", "__absent__")
    if t == "__absent__":
        return False          # this runner does not report a series; unknown
    return t is None or (hasattr(t, "__len__") and len(t) == 0)


# --- the cell -----------------------------------------------------------------


def run_cell(column, task_id, *, root=staging.DEFAULT_ROOT, out_dir,
             model=DEFAULT_MODEL, repeat=0, dry_run=False, timeout_s=None,
             keep_workspace=False, lane=None,
             engine_slots=concurrency.DEFAULT_ENGINE_SLOTS, lock_root=None,
             use_locks=True,
             rate_limit_backoff_s=DEFAULT_RATE_LIMIT_BACKOFF_S,
             max_rate_limit_retries=DEFAULT_MAX_RATE_LIMIT_RETRIES,
             backend=None, session_fn=None, grade_fn=None,
             not_expressible=None, campaign=None):
    """One cell. Returns the row dict. Raises ``RateLimited`` on a quota pause.

    ``session_fn`` / ``grade_fn`` are injection points for the tests (and for
    the dry run, which swaps the session); both default to the real thing.
    """
    t_cell0 = time.monotonic()
    task = ladder_tasks.get(task_id)
    column = str(column)
    run_dir = Path(out_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    root = Path(root)
    lane = lane or column
    lock_root = Path(lock_root) if lock_root else (root / "locks")
    machine = ob_results.machine_fingerprint()
    report = {"task": task.id, "rung": task.rung, "column": column,
              "condition": DRY_CONDITION if dry_run else CONDITION,
              "dry_run": bool(dry_run), "repeat": repeat,
              "utc_start": _utc(), "lane": lane}
    notes, deviations = [], []

    # A pre-registered NOT_EXPRESSIBLE is a WRITTEN human judgement (3.1) and
    # spends no quota: the runner carries it, it never infers it.
    if not_expressible:
        return _record_not_expressible(task, column, repeat, run_dir,
                                       not_expressible, machine, dry_run,
                                       campaign, report)

    child_env, removed = scrub_env()
    report["env_scrubbed"] = removed
    guard_ok, guard_detail = concurrency.resource_guard()
    report["resource_guard"] = {"ok": guard_ok, **guard_detail}

    # 1. preflight (agent cells only; the dry run spends nothing) -----------
    version = None
    cli_default_model = None
    if not dry_run:
        pf = cc_cell.preflight(run_dir, child_env)
        report["preflight"] = pf
        (run_dir / "preflight.json").write_text(json.dumps(pf, indent=2),
                                                encoding="utf-8")
        if not pf["ok"]:
            raise SystemExit("preflight failed: %s" % pf["detail"])
        version = pf["version"]
        cli_default_model = pf["default_model"]
        if cli_default_model and model and cli_default_model != model:
            deviations.append(
                "the CLI's own default model is %r and this cell PINS %r; "
                "both are recorded and neither is inferred from the other"
                % (cli_default_model, model))
    report["model_pinned"] = model

    engine = concurrency.EngineSlots(lock_root, slots=engine_slots, lane=lane)
    tlock = concurrency.task_lock(lock_root, task.id, lane=lane)
    samples = []

    def _sample(tag):
        samples.append({"tag": tag, "utc": _utc(),
                        "others_active": engine.others_active()})

    if use_locks:
        tlock.acquire(on_wait=lambda p: print(
            "waiting on the same-task lock %s" % p.name))
    ws = None
    row = None
    try:
        tpl = staging.build_template(column, root)
        instance_manifest = {}
        container_record = {}

        def _instantiate():
            """A fresh workspace + the staged container. Returns (ws, prompt).

            A DEFERRED attempt calls this again, so the previous instance is
            torn down first: a product clone is ~200 MB and twelve deferrals
            over a quota reset would otherwise leave 2 GB of dead workspaces
            behind.
            """
            nonlocal ws, instance_manifest, container_record
            if ws is not None and Path(ws).exists():
                staging.sweep_workspace_processes(ws)
                staging.teardown_workspace_resilient(ws)
            name = "%s_%s_%s_r%d" % (time.strftime("%Y%m%d_%H%M%S"),
                                     column, task.rung, repeat)
            while (root / "instances" / name).exists():
                name += "_x"
            ws = root / "instances" / name
            instance_manifest = staging.instantiate(tpl, ws)
            prompt, container_record = staging.stage_container(ws, task)
            # Tell the agent what phase B will re-run. Every column gets its
            # own contract, so none is handed help the others lack -- see
            # staging.DELIVERABLE_CONTRACT for why the columns are not
            # symmetric and what assuming they were cost.
            report["deliverable_contract"] = bool(
                staging.write_deliverable_contract(ws, column))
            report["workspace"] = str(ws)
            report["prompt"] = prompt
            report["container"] = container_record
            if column == "omnisim":
                child_env["OMNISIM_HOME"] = str(ws)
            return ws, prompt

        ws, prompt = _instantiate()

        # 2. the quarantine, BEFORE the session -- a hit spends no quota ----
        pre_hits = staging.answer_key_violations(ws)
        report["quarantine_pre_session"] = {"hits": pre_hits,
                                            "clean": not pre_hits}
        if pre_hits:
            row = _invalid_row(
                task, column, repeat, machine, dry_run, campaign,
                reason=("answer-key material was found IN THE STAGED "
                        "WORKSPACE (%d hit(s)); the cell is INVALID and no "
                        "session was run" % len(pre_hits)),
                answer_key={"workspace_hits": pre_hits[:20],
                            "workspace_clean": False,
                            "transcript_hits": None,
                            "transcript_scan": "not run: the workspace "
                                               "failed before the session"},
                workspace_block=_workspace_block(instance_manifest,
                                                 container_record, pre_hits),
                t_total_s=round(time.monotonic() - t_cell0, 2))
            write_row(run_dir, row)          # the row FIRST, always
            report["row_summary"] = _row_summary(row)
            write_report(run_dir, report)
            if not keep_workspace:
                report["workspace_teardown"] = \
                    staging.teardown_workspace_resilient(ws)
                write_report(run_dir, report)
            return row

        # 3. the session ----------------------------------------------------
        if use_locks and column in ("omnisim", "webots"):
            engine.acquire("ladder %s cell %s: session" % (column, task.id),
                           on_wait=lambda p: print(
                               "waiting for a free engine slot..."))
        staging.reap_port_listeners()
        if column == "omnisim":
            staging.sweep_repo_junction_artifacts(
                run_dir / "quarantine_pre_session")
        _sample("session_start")
        sess_timeout, cap_source = session_cap(task, timeout_s)
        report["session_cap"] = {"cap_s": sess_timeout, "source": cap_source}
        if session_fn is not None:
            session = session_fn(task=task, column=column, workspace=ws,
                                 run_dir=run_dir, prompt=prompt,
                                 env=child_env, model=model,
                                 timeout_s=sess_timeout)
        elif dry_run:
            session = run_dry_session(
                task=task, column=column, workspace=ws, run_dir=run_dir,
                container_root=container_record.get("container_root"))
        else:
            session = run_agent_session(
                prompt=prompt, workspace=ws, env=child_env, run_dir=run_dir,
                model=model, timeout_s=sess_timeout,
                rate_limit_backoff_s=rate_limit_backoff_s,
                max_rate_limit_retries=max_rate_limit_retries,
                version=version, reinstantiate=_instantiate,
                on_defer=lambda marker, attempt, d: print(
                    "usage/rate limit (%r): DEFERRED (this is a pause, not a "
                    "run); retrying in %.0f s" % (marker,
                                                  rate_limit_backoff_s)))
        _sample("session_end")
        report["session"] = {"status": session.status,
                             "meta": session.meta,
                             "deferrals": session.deferrals,
                             "oracle": (session.oracle.as_dict()
                                        if session.oracle else None)}

        # 4. hygiene: workspace stragglers and ports -------------------------
        # ⚠ THE REPO-JUNCTION SWEEP MOVED (2026-08-02, measured). It used to
        # run HERE, before discovery -- so on omnisim it DELETED the
        # session's own deliverable out of the junctioned tree, discovery
        # found nothing in the workspace, and the runner graded a preserved
        # quarantine copy whose relative asset and controller references no
        # longer resolved (phase B: "Cannot open URDF file
        # .../repo_artifacts/.../../../../robots/omnisim/omniarm6/omniarm6_2f85.urdf",
        # 0 robots, 0 rows). It now runs AFTER grading, so the deliverable is
        # graded where the agent wrote it; the pre-session sweep is untouched
        # and is what protects the next cell from this one.
        staging.sweep_workspace_processes(ws)
        staging.reap_port_listeners()
        repo_sweep = []
        repo_swept = False

        def _sweep_repo(reason):
            """Preserve-then-delete, once, however this cell exits."""
            nonlocal repo_sweep, repo_swept
            # `is None`, not truthiness: a start_ts of 0.0 is a real timestamp
            # in a test and the old `and session.start_ts` guard skipped the
            # sweep entirely on it.
            if repo_swept or column != "omnisim" or session.start_ts is None:
                return
            repo_swept = True
            try:
                repo_sweep = staging.sweep_repo_junction_artifacts(
                    run_dir / "repo_artifacts",
                    window=(session.start_ts, time.time()))
            except Exception as exc:                      # noqa: BLE001
                repo_sweep = [{"rel": None, "action": "sweep_failed",
                               "detail": repr(exc)}]
            report["repo_artifact_sweep"] = repo_sweep
            report["repo_artifact_sweep_when"] = reason

        # 5. the deliverable -------------------------------------------------
        deliverable, kind, rule_note = None, None, None
        deliv_diag = {}
        phase_b_dir = None
        # "ok" OR "timeout": a capped session is graded too. Its cell value is
        # already fixed at not_achieved(budget_exhausted) by ``classify``,
        # which short-circuits on the status ABOVE any verdict branch -- so
        # grading here cannot promote a slow agent to `achieved`. What it buys
        # is the distinction the old code destroyed: "capped while flailing"
        # and "capped 90 seconds after finishing" were the same empty row.
        # Measured, omnisim/T2 r1: the agent said "task complete", the cap
        # landed, the workspace was deleted ungraded, and the row said
        # `deliverable: null`.
        if session.status in ("ok", "timeout"):
            if session.oracle is not None:
                deliverable = session.oracle.deliverable
                phase_b_dir = session.oracle.phase_b_dir
                kind = "oracle"
                rule_note = ("the scripted oracle's own output (dry run); "
                             + "; ".join(session.oracle.notes))
            else:
                deliverable, kind, rule_note, deliv_diag = \
                    discover_deliverable(
                        column, ws, session.start_ts,
                        swept_roots=(run_dir / "repo_artifacts",
                                     run_dir / "quarantine_pre_session"))
        report["deliverable_rule"] = rule_note
        report["deliverable_discovery"] = deliv_diag
        collected = None
        if deliverable:
            collected = oracles.copy_deliverable(deliverable,
                                                 run_dir / "deliverable")
        report["deliverable"] = {"found": str(deliverable) if deliverable
                                 else None, "collected": collected,
                                 "kind": kind}

        # 6. phase B + the real grader ---------------------------------------
        # Skipped outright when the deliverable is ambiguous: an engine run on
        # a file we cannot attribute produces a number nobody may use, and
        # that number is exactly what got published last time.
        verdict = phase_b = grade_error = grade_path = None
        ambiguity = (deliv_diag or {}).get("ambiguity_reason")
        if (deliverable or phase_b_dir) and not ambiguity:
            if use_locks:
                engine.acquire("ladder %s cell %s: phase B"
                               % (column, task.id))
            _sample("grade_start")
            fn = grade_fn or grade
            verdict, phase_b, grade_error, grade_path = fn(
                task, column, deliverable=deliverable, run_dir=run_dir,
                phase_b_dir=phase_b_dir, backend=backend)
            _sample("grade_end")
            staging.reap_port_listeners()
            if verdict is not None:
                (run_dir / "verdict.json").write_text(
                    json.dumps(verdict.as_dict(), indent=2, default=str),
                    encoding="utf-8")
        report["grade_path"] = grade_path
        report["grade_error"] = grade_error

        # 6b. NOW the repo sweep: the deliverable has been graded in place,
        # and the window runs to now so phase B's own injected sibling
        # (_agentbench_phaseB_*.wbt, written next to the world by design) is
        # swept too rather than left in the repo.
        _sweep_repo("after grading")

        # 6c. The safety net for the case the reorder is supposed to make
        # impossible: no deliverable in the workspace, but the sweep pulled
        # deliverable-shaped files out of the REAL REPO. That is our
        # discovery walk missing the agent's output, not the agent leaving
        # none -- so it is INVALID with an instrument blocker, never
        # agent_lost.
        if not deliverable and not phase_b_dir and not ambiguity:
            swept_worlds = [r.get("rel") for r in repo_sweep
                            if r.get("rel") and str(r["rel"]).endswith((".wbt", ".omniworld"))
                            and not is_tool_generated_world(
                                Path(str(r["rel"])).name)]
            if swept_worlds:
                deliv_diag = dict(deliv_diag or {})
                deliv_diag.update(
                    ambiguous=True,
                    instrument_blocker="deliverable_not_in_workspace",
                    swept_evidence=swept_worlds,
                    ambiguity_reason=(
                        "the workspace walk found no deliverable, but the "
                        "post-grading junction sweep preserved %d authored "
                        ".wbt from the REAL REPO (%s): the session's output "
                        "was written somewhere our walk does not reach"
                        % (len(swept_worlds), ", ".join(swept_worlds[:5]))))
                ambiguity = deliv_diag["ambiguity_reason"]
                report["deliverable_discovery"] = deliv_diag

        # 7. the row ----------------------------------------------------------
        cell_value, blocker, basis = classify(
            session_status=session.status,
            deliverable_found=bool(deliverable or phase_b_dir),
            deliverable_rule_known=column in DELIVERABLE_RULES,
            grade_error=grade_error, verdict=verdict,
            phase_b_broke=phase_b_broke(phase_b, verdict),
            deliverable_ambiguity=ambiguity)
        invalid_reason = None
        instrument_blocker = None
        if ambiguity:
            invalid_reason = basis
            instrument_blocker = (deliv_diag or {}).get("instrument_blocker")
        elif verdict is not None and verdict.outcome in ("INVALID", "ERROR"):
            invalid_reason = basis

        transcript = (session.cc or {}).get("transcript")
        tr_hits, tr_reason = staging.transcript_answer_key_hits(transcript)
        if tr_hits:
            invalid_reason = (
                "the session transcript quotes ladder answer-key material "
                "(%d hit(s)): the answer key was in front of the agent, so "
                "the cell is INVALID whatever the workspace scan said"
                % len(tr_hits))
        answer_key = {
            "workspace_clean": True, "workspace_hits": [],
            "transcript_hits": tr_hits,
            "transcript_scan": tr_reason or "scanned",
            "rule": ("a cell whose workspace contained -- or whose session "
                     "reached -- answer-key material is INVALID, never "
                     "graded"),
        }

        metrics = (metrics_from_cc(session.cc,
                                   t_total_s=round(time.monotonic() - t_cell0, 2))
                   if session.cc else empty_metrics())
        metrics["t_total_s"] = round(time.monotonic() - t_cell0, 2)
        if dry_run:
            metrics["t_agent_s"] = None
            notes.append(
                "DRY RUN: there was no agent, so every agent-cost column is "
                "null by construction. This row is NOT a ladder cell "
                "(capability-ladder-plan.md 2) and may not be published as "
                "one.")
        agent_block = {
            "kind": DRY_CONDITION if dry_run else "claude_code",
            "model_pinned": None if dry_run else model,
            "model_reported": (session.cc or {}).get("model"),
            "model_pin_rule": (
                "the pin is an explicit flag (default %r). The reported id is "
                "recorded SEPARATELY and never inferred from the pin: an "
                "alias resolves to a dated id, so the two legitimately "
                "differ and a reader must see both." % DEFAULT_MODEL),
            "cli_version": version,
            "cli_default_model": cli_default_model,
            "permission_mode": (session.cc or {}).get("permission_mode"),
            "session_id": (session.cc or {}).get("session_id"),
            "transcript": transcript,
            "tool_calls_reason": (session.cc or {}).get("tool_calls_reason"),
            "cli_command": (session.cc or {}).get("cli_command"),
            "rate_limit_deferrals": session.deferrals,
            "oracle": (session.oracle.as_dict() if session.oracle else None),
            "weakening": (
                "Claude Code is a moving product we cannot byte-hash, so "
                "replication means 'at this CLI version and this model id', "
                "and cost and turns are the instrument's SELF-REPORTED "
                "figures (agent-edge-validation-plan.md 5.11)."),
        }
        under_conc = any(s["others_active"] for s in samples)
        end_block = classify_session_end(session, sess_timeout,
                                         transcript=transcript)
        report["session_end"] = end_block
        if end_block["reason"] in ("cap_reached", "background_work_pending"):
            deviations.append(
                "session_end=%s: %s" % (end_block["reason"],
                                        end_block["detail"]))
        # A capped session's verdict is REROUTED out of `outcome` and into its
        # own block: the cell has no authoritative outcome (the agent was cut
        # off), but "what was on disk when the cap landed" is worth keeping.
        capped = (session.status == "timeout")
        row = build_row(
            task=task, column=column, repeat=repeat, cell_value=cell_value,
            blocker=blocker, blocker_basis=basis,
            verdict=(None if capped else verdict),
            post_cap_verdict=(verdict if capped else None),
            metrics=metrics, agent_block=agent_block,
            workspace_block=_workspace_block(instance_manifest,
                                             container_record, pre_hits),
            artifacts={"run_dir": str(run_dir),
                       "deliverable": collected,
                       "deliverable_found": (str(deliverable) if deliverable
                                             else None),
                       "phase_b_dir": (str(phase_b_dir) if phase_b_dir
                                       else None),
                       "verdict": str(run_dir / "verdict.json"),
                       "repo_artifacts_swept": repo_sweep},
            notes=notes, deviations=deviations, machine=machine,
            dry_run=dry_run,
            t_total_s=round(time.monotonic() - t_cell0, 2),
            answer_key=answer_key, invalid_reason=invalid_reason,
            phase_b=(phase_b.as_dict() if hasattr(phase_b, "as_dict")
                     else None),
            deliverable_note=rule_note, grade_path=grade_path,
            measured_under_concurrency=under_conc,
            concurrency_block={"lane": lane, "engine_slots": engine_slots,
                               "locks_used": bool(use_locks),
                               "samples": samples},
            instrument_blocker=instrument_blocker,
            deliverable_diag=deliv_diag or None,
            session_end=end_block, session_cap=sess_timeout,
            session_cap_source=cap_source)
        if campaign:
            row["campaign"] = dict(campaign)
    finally:
        if use_locks:
            engine.release()
            tlock.release()
        # The repo must not be left dirty by a cell that died mid-grade. The
        # sweep is idempotent (``_sweep_repo`` runs at most once) and is
        # defined only after the session, so it may not exist yet.
        try:
            _sweep_repo("teardown fallback")
        except (NameError, UnboundLocalError):
            pass
        # Leaving by an exception (a quota pause, a preflight refusal) means
        # NO row was earned, so there is nothing to protect and the workspace
        # must not be left behind. The success path deliberately does NOT tear
        # down here -- it happens after the row is on disk, below.
        if row is None and ws is not None and not keep_workspace:
            staging.sweep_workspace_processes(ws)
            staging.teardown_workspace_resilient(ws)

    # 7b. the agent's own working area, kept -------------------------------
    # `copy_deliverable` preserves the ONE discovered artifact (a world file);
    # the controllers, kinematics and proof bundle beside it are the substance
    # and used to die with the workspace. Cheap: this is the task container,
    # not the product clone. The out-of-container case is already covered by
    # `_sweep_repo` (cell r0 wrote into projects/ through the junction), and
    # anything an agent writes somewhere else entirely is still lost -- said
    # plainly here rather than implied to be complete.
    if ws is not None:
        try:
            src = Path(ws) / "container"
            if src.is_dir():
                dest = run_dir / "workspace_container"
                if dest.exists():
                    shutil.rmtree(dest, ignore_errors=True)
                shutil.copytree(src, dest, symlinks=True,
                                ignore=shutil.ignore_patterns(
                                    "*.exe", "*.dll", "__pycache__"))
                n = sum(1 for _ in dest.rglob("*") if _.is_file())
                report["workspace_container_preserved"] = {
                    "dest": str(dest), "files": n,
                    "scope": "the task container only; work written elsewhere "
                             "in the workspace is NOT here"}
        except (OSError, shutil.Error) as exc:
            report["workspace_container_preserved"] = {
                "dest": None, "error": repr(exc)}

    # 8. row FIRST, teardown after (the paid-for ordering) -------------------
    write_row(run_dir, row)
    report["row_summary"] = _row_summary(row)
    report["utc_end"] = _utc()
    write_report(run_dir, report)

    if not keep_workspace and ws is not None:
        staging.sweep_workspace_processes(ws)
        outcome = staging.teardown_workspace_resilient(ws)
        report["workspace_teardown"] = outcome
        if not outcome.get("ok"):
            print("workspace teardown failed (%s); marked for a later sweep: "
                  "%s" % (outcome.get("error"), outcome.get("pending")))
        write_report(run_dir, report)
    return row


def _workspace_block(manifest, container, hits):
    return {
        "staging_manifest_sha": manifest.get("filelist_sha256"),
        "staged_file_count": manifest.get("included_file_count"),
        "excluded_count": len(manifest.get("excluded") or []),
        "redactions": len(manifest.get("redactions") or []),
        "known_disclosures": manifest.get("known_disclosures") or [],
        "junctions": [j.get("name") for j in manifest.get("junctions") or []],
        "container": container,
        "quarantine_hits": len(hits or []),
    }


def _row_summary(row):
    return {"cell": (row.get("cell") or {}).get("value"),
            "blocker": (row.get("cell") or {}).get("blocker"),
            "instrument_blocker": (row.get("cell")
                                   or {}).get("instrument_blocker"),
            "outcome": row.get("outcome"),
            "session_end": (row.get("session_end") or {}).get("reason"),
            "failed_assertion": row.get("failed_assertion"),
            "metrics": row.get("metrics")}


def _invalid_row(task, column, repeat, machine, dry_run, campaign, *, reason,
                 answer_key, workspace_block, t_total_s,
                 instrument_blocker=None):
    row = build_row(
        task=task, column=column, repeat=repeat, cell_value=None,
        blocker=None, blocker_basis=None, verdict=None,
        metrics=empty_metrics(), agent_block={"kind": "none",
                                              "model_pinned": None},
        workspace_block=workspace_block,
        artifacts={}, notes=[reason], deviations=[], machine=machine,
        dry_run=dry_run, t_total_s=t_total_s, answer_key=answer_key,
        invalid_reason=reason, instrument_blocker=instrument_blocker)
    if campaign:
        row["campaign"] = dict(campaign)
    return row


def _record_not_expressible(task, column, repeat, run_dir, spec, machine,
                            dry_run, campaign, report):
    """Carry a REVIEWED ``NOT_EXPRESSIBLE`` judgement into a row.

    §3.1 requires a written justification citing *that simulator's own docs*
    and one of the ``REQUIRED_EVIDENCE`` lines, review inside the 30-day
    window, and a flip on a single credible counter-example. None of that is
    a measurement, so the runner never infers this label -- it only records
    one somebody wrote, and it spends no quota doing it.
    """
    missing = [k for k in ("justification", "sim_doc_citation",
                           "required_evidence_line") if not spec.get(k)]
    if missing:
        raise SystemExit(
            "NOT_EXPRESSIBLE needs %s (3.1): a justification citing the "
            "simulator's OWN docs or API listing, and the REQUIRED_EVIDENCE "
            "line naming the channel it cannot supply" % ", ".join(missing))
    row = build_row(
        task=task, column=column, repeat=repeat, cell_value="NOT_EXPRESSIBLE",
        blocker=None,
        blocker_basis="recorded from a written human judgement; never "
                      "inferred by the runner",
        verdict=None, metrics=empty_metrics(),
        agent_block={"kind": "none", "model_pinned": None},
        workspace_block={}, artifacts={},
        notes=["NOT_EXPRESSIBLE is excluded from every aggregate, is reported "
               "as its own count, and may never be rendered in a failure's "
               "colour in any chart (3.1). A single credible counter-example "
               "flips the label and the cell gets run."],
        deviations=[], machine=machine, dry_run=dry_run, t_total_s=0.0,
        answer_key={"workspace_clean": None, "workspace_hits": None,
                    "transcript_hits": None,
                    "transcript_scan": "no session was run"},
        not_expressible=dict(spec))
    if campaign:
        row["campaign"] = dict(campaign)
    write_row(run_dir, row)
    report["row_summary"] = _row_summary(row)
    write_report(run_dir, report)
    return row


# --- CLI ----------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--column", required=True,
                    help="roster column: " + ", ".join(staging.KNOWN_COLUMNS))
    ap.add_argument("--task", required=True,
                    help="T1_arrive | T2_transfer | T3_quadruped | "
                         "T4_humanoid (short ids work)")
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help="the PINNED model id, recorded in the row "
                         "(default %(default)s). There is no silent default "
                         "drift: the CLI's own default is recorded separately "
                         "and is never used as the pin.")
    ap.add_argument("--repeat", type=int, default=0)
    ap.add_argument("--root", default=str(staging.DEFAULT_ROOT))
    ap.add_argument("--out", default=None,
                    help="cell run dir (default results/ladder_cell/<utc>)")
    ap.add_argument("--dry-run", action="store_true",
                    help="replace the Claude session with the tier's "
                         "COMMITTED SCRIPTED ORACLE: the whole pipeline, no "
                         "quota. Rows are marked dry_run and are not cells.")
    ap.add_argument("--timeout-s", type=float, default=None,
                    help="session cap (default: the task's own timeout_s)")
    ap.add_argument("--backend", default=None, choices=(None, "ode", "newton"))
    ap.add_argument("--keep-workspace", action="store_true")
    ap.add_argument("--lane", default=None)
    ap.add_argument("--engine-slots", type=int,
                    default=concurrency.DEFAULT_ENGINE_SLOTS)
    ap.add_argument("--lock-root", default=None)
    ap.add_argument("--no-locks", action="store_true")
    ap.add_argument("--rate-limit-backoff-s", type=float,
                    default=DEFAULT_RATE_LIMIT_BACKOFF_S)
    ap.add_argument("--max-rate-limit-retries", type=int,
                    default=DEFAULT_MAX_RATE_LIMIT_RETRIES)
    ap.add_argument("--publish", action="store_true",
                    help="copy this cell's rows into results_published/")
    a = ap.parse_args(argv)

    out = (Path(a.out) if a.out else
           (LADDER / "results" / "ladder_cell"
            / ("%s_%s_%s" % (time.strftime("%Y%m%d_%H%M%S"), a.column,
                             a.task)))).resolve()
    row = run_cell(a.column, a.task, root=Path(a.root), out_dir=out,
                   model=a.model, repeat=a.repeat, dry_run=a.dry_run,
                   timeout_s=a.timeout_s, keep_workspace=a.keep_workspace,
                   lane=a.lane, engine_slots=a.engine_slots,
                   lock_root=a.lock_root, use_locks=not a.no_locks,
                   rate_limit_backoff_s=a.rate_limit_backoff_s,
                   max_rate_limit_retries=a.max_rate_limit_retries,
                   backend=a.backend)
    cell = row.get("cell") or {}
    print("\n=== %s / %s / %s ==="
          % (row["task"], row["column"], row["condition"]))
    print("cell      : %s%s" % (cell.get("value"),
                                ("(blocker=%s)" % cell.get("blocker"))
                                if cell.get("blocker") else ""))
    if cell.get("invalid_reason"):
        print("INVALID   : %s" % cell["invalid_reason"])
    if cell.get("instrument_blocker"):
        print("instrument: %s -- %s" % (cell["instrument_blocker"],
                                        INSTRUMENT_BLOCKER_OWNER))
    end = row.get("session_end") or {}
    if end:
        wall = end.get("wall_s")
        print("session   : ended %r after %s of a %.0f s cap (working=%s)"
              % (end.get("reason"),
                 ("%.0f s" % wall) if wall else "an unrecorded wall time",
                 end.get("cap_s") or 0.0, end.get("was_working")))
    print("outcome   : %s  failed=%s" % (row.get("outcome"),
                                         row.get("failed_assertion")))
    print("metrics   : %s" % json.dumps(row["metrics"], default=str))
    prov = row["provenance"]
    print("reuse_class: %s -- %s" % (prov["reuse_class"], REUSE_CLASS_NOTE))
    if row["rung"] in ("T3", "T4"):
        print("support   : %s / %s"
              % (prov["support_attestation"], prov["support_cell"]))
        print("distance_to_termination_m: %s  termination_cause: %s"
              % (prov["distance_to_termination_m"], prov["termination_cause"]))
    print("rows -> %s" % (out / "rows.jsonl"))
    if a.publish:
        from agentbench.run_agentbench import publish_run
        print("published -> %s" % publish_run(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
