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

"""The Phase W campaign driver (agent-edge-validation-plan §3).

One command drives the whole scored campaign the moment a credential exists:

    ANTHROPIC_API_KEY=... python tests/benchmarks/agentbench/campaign.py \\
        --model <exact-model-id> --sims omnisim

Schedule per simulator (plan §3.1): Lane A (``A1_husky_swarm_10`` x 2
conditions x n=10) + Lane B (5 tasks x 2 conditions x n=5) = **70 scored
runs**; ``--sims omnisim,webots`` doubles that to 140. Cells run **strictly
sequentially** (plan §5.3 -- wall clock measured under contention is not a
measurement); every run goes through the existing ``run_agentbench.py`` cell
path with per-run isolation (its own ``--out`` dir; the runner's sandbox owns
ports and log paths).

What the driver adds on top of ``run_agentbench.py``:

* **Flake handling** (plan §3 item 8, SPEC §3.3): an ``INVALID``/``ERROR``
  run is re-run up to 2 times; every attempt's row is appended (earlier
  attempts marked ``campaign.superseded``) so a retry is attributed, never
  silent. A cell over 20% INVALID after re-runs is marked
  ``published-as-unreliable``.
* **Credential gate**: scored runs refuse to start without
  ``ANTHROPIC_API_KEY``. ``--smoke`` runs exactly one n=1 cell end-to-end
  (marked exploratory, never aggregated) to validate the round trip, real
  token numbers and real usd before the campaign spends against them.
* **Model pinning**: the exact model id is a required flag, recorded in
  every row's ``campaign`` block; there is no default.
* **Resume**: ``state.json`` in the campaign dir; an interrupted campaign
  continues from the next incomplete run. Rows are append-only and a
  completed run is never re-run.
* **Publication**: on completion the campaign dir's ``rows.jsonl`` is
  published via ``run_agentbench.py``'s existing ``--publish`` mechanism into
  ``results_published/<campaign_id>/``, then the F evaluator (``f_eval.py``)
  runs on the published rows.
* ``--dry-run``: the full schedule walk on the scripted/fake-sim path -- no
  key, no model, no simulator -- proving the driver end-to-end. Dry-run rows
  are ``SKIPPED`` by construction and are marked so the F evaluator excludes
  them.

Known interface gaps (worked around, not patched -- ``run_agentbench.py``'s
CLI is the contract):

* no ``--sim`` flag: the child always writes ``sim: "omnisim"`` in the row.
  The driver sets ``AGENTBENCH_SIM`` in the child env (the tool registry
  reads it) and records the scheduled simulator in ``campaign.sim``, which
  the F evaluator prefers.
* no ``--repeat-offset``: each invocation runs ``--repeats 1`` so the child
  row always says ``repeat: 0``. The driver rewrites the row's ``repeat`` to
  the schedule index and keeps the child value in ``campaign.child_repeat``.
* the Webots execution path (plan Phase R item 4) is not built; ``webots``
  cells are schedulable so the campaign shape is real, but scoring them
  requires that path to land.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import f_eval  # noqa: E402  (sibling module; the evaluator this driver ends in)
import sims    # noqa: E402  (sibling module; the comparator registry)

REPO = _HERE.parents[2]
RUN_AGENTBENCH = _HERE / "run_agentbench.py"
NOOP_SCRIPT = _HERE / "runner" / "scripts" / "a1_noop.json"
CAMPAIGNS_ROOT = _HERE / "results" / "campaigns"

LANE_A_TASK = f_eval.LANE_A_TASK
LANE_B_TASKS = f_eval.LANE_B_TASKS
N_LANE_A = f_eval.N_LANE_A          # 10 (SPEC §3.5 flagship floor)
N_LANE_B = f_eval.N_LANE_B          # 5

CONDITIONS = (f_eval.SHELL, f_eval.TOOLS)      # "shell", "shell+tools"
# Condition -> the pinned agent. llm_shell / llm_tools pin their condition in
# code, so an A/B pair cannot be produced by a stale environment variable.
CONDITION_AGENTS = {f_eval.SHELL: "llm_shell", f_eval.TOOLS: "llm_tools"}

#: Comparators a campaign may schedule. Read from the registry (sims.py) so
#: the driver, the cell runner and the report cannot disagree about the
#: comparator set. Declared-but-unbuilt arms are deliberately NOT here -- a
#: campaign schedules only what can be scored -- but sims.DECLARED keeps them
#: nameable, so "missing from the campaign" stays distinguishable from
#: "missing from the design".
VALID_SIMS = sims.IMPLEMENTED

MAX_LAUNCH_RETRIES = 2              # plan §3 item 8 / SPEC §3.3
INVALID_CEILING = 0.20              # SPEC §3.3: > 20% INVALID -> unreliable
RETRY_OUTCOMES = ("INVALID", "ERROR")
SUBPROCESS_GRACE_S = 900.0          # on top of the task's own 3x-par timeout

SMOKE_DEFAULT_TASK = "B3_measure_and_report"


class CampaignError(RuntimeError):
    pass


class CredentialMissing(CampaignError):
    pass


def utcnow():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _cond_safe(condition):
    return condition.replace("+", "_plus_")


@dataclass(frozen=True)
class RunSpec:
    sim: str
    task: str
    condition: str          # SPEC spelling: "shell" | "shell+tools"
    lane: str               # "A" | "B" | "smoke"
    repeat: int
    n: int                  # scheduled repeats for this cell

    @property
    def agent(self):
        return CONDITION_AGENTS[self.condition]

    @property
    def cell(self):
        return "%s/%s/%s" % (self.sim, self.task, self.condition)

    @property
    def key(self):
        return "%s/r%d" % (self.cell, self.repeat)


def build_schedule(sims):
    """The pre-registered Phase W schedule, strictly ordered.

    Per sim: (A1 x 2 conditions x n=10) + (5 Lane B tasks x 2 conditions x
    n=5) = 70 runs; two sims = 140 (plan §3.1). The order is sim -> task ->
    condition -> repeat, and execution is strictly sequential (plan §5.3)."""
    for sim in sims:
        if sim not in VALID_SIMS:
            raise CampaignError("unknown sim %r (valid: %s)"
                                % (sim, ", ".join(VALID_SIMS)))
    specs = []
    plan = [(LANE_A_TASK, "A", N_LANE_A)] + [(t, "B", N_LANE_B)
                                             for t in LANE_B_TASKS]
    for sim in sims:
        for task, lane, n in plan:
            for cond in CONDITIONS:
                for rep in range(n):
                    specs.append(RunSpec(sim=sim, task=task, condition=cond,
                                         lane=lane, repeat=rep, n=n))
    return specs


def _task_timeout_s(task_id):
    """The task's own 3x-par timeout, read from the registry (read-only)."""
    try:
        from agentbench import tasks as task_registry
        return float(task_registry.get(task_id).timeout_s or 1800)
    except Exception:                                     # noqa: BLE001
        return 1800.0


def subprocess_executor(spec, attempt_dir, attempt, *, model, dry_run):
    """One run through ``run_agentbench.py``'s CLI (the contract).

    Returns the row dict the child wrote, or a synthesized attributed ERROR
    row when the child produced none -- a failure must be a row, not a log
    line."""
    attempt_dir = Path(attempt_dir)
    attempt_dir.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(RUN_AGENTBENCH),
           "--tasks", spec.task, "--agent", spec.agent,
           "--repeats", "1", "--out", str(attempt_dir),
           "--condition", spec.condition]
    env = os.environ.copy()
    env["AGENTBENCH_SIM"] = spec.sim
    if dry_run:
        cmd.append("--fake-sim")
        env["AGENTBENCH_BACKEND"] = "scripted"
        env["AGENTBENCH_SCRIPT"] = str(NOOP_SCRIPT)
        env.pop("AGENTBENCH_MODEL", None)
    else:
        env["AGENTBENCH_BACKEND"] = "anthropic"
        env["AGENTBENCH_MODEL"] = model
    timeout = _task_timeout_s(spec.task) + SUBPROCESS_GRACE_S
    log_path = attempt_dir / "driver.log"
    try:
        with open(log_path, "w", encoding="utf-8") as fh:
            proc = subprocess.run(cmd, cwd=str(REPO), env=env,
                                  stdout=fh, stderr=subprocess.STDOUT,
                                  timeout=timeout)
        exit_code = proc.returncode
        timed_out = False
    except subprocess.TimeoutExpired:
        exit_code, timed_out = None, True
    rows_file = attempt_dir / "rows.jsonl"
    if rows_file.is_file():
        lines = [ln for ln in
                 rows_file.read_text(encoding="utf-8").splitlines()
                 if ln.strip()]
        if lines:
            return json.loads(lines[-1])
    return {"suite": "agenticsimbench/v0.3", "task": spec.task,
            "sim": "omnisim", "condition": spec.condition,
            "repeat": 0, "agent": {"kind": spec.agent},
            "outcome": "ERROR",
            "notes": ["campaign driver: run_agentbench.py produced no "
                      "rows.jsonl (%s); see %s"
                      % ("timed out after %.0fs" % timeout if timed_out
                         else "exit code %s" % exit_code, log_path)],
            "utc": utcnow()}


class Campaign:
    """One campaign directory: schedule + state + append-only rows.

    ``executor(spec, attempt_dir, attempt)`` -> row dict is injectable so the
    driver logic is testable without a subprocess; the default shells out to
    ``run_agentbench.py`` (its CLI is the contract)."""

    def __init__(self, campaign_dir, *, model=None, sims=("omnisim",),
                 dry_run=False, executor=None, publish=True, quiet=False):
        self.dir = Path(campaign_dir)
        self.id = self.dir.name
        self.model = model
        self.sims = list(sims)
        self.dry_run = bool(dry_run)
        self.publish_enabled = bool(publish)
        self.quiet = quiet
        self.executor = executor or self._subprocess_executor
        self.rows_path = self.dir / "rows.jsonl"
        self.state_path = self.dir / "state.json"
        self.schedule = build_schedule(self.sims)
        self.state = self._init_or_resume()

    # -- state -----------------------------------------------------------
    def _init_or_resume(self):
        if self.state_path.exists():
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            for key, want in (("model", self.model), ("sims", self.sims),
                              ("dry_run", self.dry_run)):
                if state.get(key) != want:
                    raise CampaignError(
                        "resume refused: state.json has %s=%r but this "
                        "invocation asked for %r -- a campaign must not mix "
                        "configurations (re-run with the original flags, or "
                        "start a new --campaign-id)"
                        % (key, state.get(key), want))
            self._log("resuming campaign %s: %d/%d runs already complete"
                      % (self.id, len(state["completed"]), len(self.schedule)))
            return state
        self.dir.mkdir(parents=True, exist_ok=True)
        state = {"campaign_id": self.id, "model": self.model,
                 "sims": self.sims, "dry_run": self.dry_run,
                 "schedule_len": len(self.schedule),
                 "created_utc": utcnow(), "completed": {},
                 "retries": {}, "unreliable_cells": {}}
        self._save_state(state)
        return state

    def _save_state(self, state=None):
        state = state if state is not None else self.state
        tmp = self.state_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(state, indent=2), encoding="utf-8")
        os.replace(tmp, self.state_path)

    def _log(self, msg):
        if not self.quiet:
            print("[campaign %s] %s" % (self.id, msg), flush=True)

    # -- row plumbing ----------------------------------------------------
    def _append_row(self, row):
        with open(self.rows_path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(row, default=str) + "\n")

    def _annotate(self, row, spec, attempt, superseded, retry_reason):
        row["campaign"] = {
            "id": self.id, "sim": spec.sim, "lane": spec.lane,
            "cell": spec.cell, "repeat": spec.repeat,
            "child_repeat": row.get("repeat"), "scheduled_n": spec.n,
            "attempt": attempt + 1,
            "max_attempts": 1 + MAX_LAUNCH_RETRIES,
            "superseded": bool(superseded),
            "retry_reason": retry_reason,
            "model_flag": self.model, "dry_run": self.dry_run,
        }
        # The child always reports repeat=0 (one --repeats 1 invocation per
        # scheduled run); the schedule's index is the real one.
        row["repeat"] = spec.repeat
        return row

    # -- executors -------------------------------------------------------
    def _subprocess_executor(self, spec, attempt_dir, attempt):
        return subprocess_executor(spec, attempt_dir, attempt,
                                   model=self.model, dry_run=self.dry_run)

    # -- the run loop ----------------------------------------------------
    def run_one(self, spec):
        """One scheduled run: attempt + up to MAX_LAUNCH_RETRIES re-runs on
        INVALID/ERROR (plan §3 item 8), every attempt appended and
        attributed. Returns the final row."""
        retry_reason = None
        final_row = None
        for attempt in range(1 + MAX_LAUNCH_RETRIES):
            attempt_dir = (self.dir / "cells" / spec.sim / spec.task
                           / _cond_safe(spec.condition)
                           / ("r%d_a%d" % (spec.repeat, attempt)))
            row = self.executor(spec, attempt_dir, attempt)
            outcome = row.get("outcome")
            retryable = (outcome in RETRY_OUTCOMES
                         and attempt < MAX_LAUNCH_RETRIES)
            self._annotate(row, spec, attempt, superseded=retryable,
                           retry_reason=retry_reason)
            self._append_row(row)
            if not self.dry_run and outcome == "SKIPPED":
                # The backend refused (credential vanished mid-campaign).
                # Recorded, NOT marked complete -- resuming re-runs it.
                raise CredentialMissing(
                    "run %s came back SKIPPED (backend/credential refusal) "
                    "in a scored campaign; stopping so the remaining cells "
                    "are not burned. Fix the credential and re-run the same "
                    "command to resume." % spec.key)
            if retryable:
                retry_reason = outcome
                self.state["retries"][spec.key] = (
                    self.state["retries"].get(spec.key, 0) + 1)
                self._save_state()
                self._log("run %s attempt %d -> %s; retrying (%d/%d), "
                          "attributed in the rows"
                          % (spec.key, attempt + 1, outcome,
                             attempt + 1, MAX_LAUNCH_RETRIES))
                continue
            final_row = row
            break
        self.state["completed"][spec.key] = {
            "outcome": final_row.get("outcome"),
            "attempts": (final_row.get("campaign") or {}).get("attempt"),
            "utc": utcnow()}
        self._save_state()
        self._log("run %s -> %s (attempt %s)"
                  % (spec.key, final_row.get("outcome"),
                     (final_row.get("campaign") or {}).get("attempt")))
        return final_row

    def _maybe_flag_cell(self, spec):
        """When a cell's last repeat completes, apply the SPEC §3.3 INVALID
        ceiling: > 20% INVALID after re-runs -> published-as-unreliable."""
        done = [v for k, v in self.state["completed"].items()
                if k.startswith(spec.cell + "/")]
        if len(done) < spec.n:
            return
        n_invalid = sum(1 for v in done if v["outcome"] == "INVALID")
        frac = n_invalid / len(done)
        if frac > INVALID_CEILING:
            self.state["unreliable_cells"][spec.cell] = {
                "label": "published-as-unreliable",
                "invalid": n_invalid, "n": len(done),
                "rule": "SPEC §3.3: a cell over 20%% INVALID after re-runs "
                        "is published as unreliable, not as a score"}
            self._save_state()
            self._log("cell %s: %d/%d INVALID after re-runs -> "
                      "published-as-unreliable (SPEC §3.3)"
                      % (spec.cell, n_invalid, len(done)))

    def run(self):
        """Walk the schedule strictly sequentially; resume-safe."""
        for spec in self.schedule:
            if spec.key in self.state["completed"]:
                continue
            self.run_one(spec)
            self._maybe_flag_cell(spec)
        return self.finish()

    # -- completion ------------------------------------------------------
    def campaign_meta(self):
        hist = {}
        for v in self.state["completed"].values():
            hist[v["outcome"]] = hist.get(v["outcome"], 0) + 1
        return {"campaign_id": self.id, "model": self.model,
                "sims": self.sims, "dry_run": self.dry_run,
                "schedule_len": len(self.schedule),
                "completed": len(self.state["completed"]),
                "outcomes": hist,
                "launch_retries": dict(self.state["retries"]),
                "unreliable_cells": dict(self.state["unreliable_cells"]),
                "utc": utcnow()}

    def finish(self):
        meta = self.campaign_meta()
        (self.dir / "campaign_meta.json").write_text(
            json.dumps(meta, indent=2), encoding="utf-8")
        if self.dry_run or not self.publish_enabled:
            report = self._run_f_eval(self.rows_path,
                                      self.dir / "f_report.md")
            self._log("dry-run/unpublished campaign complete: rows=%s "
                      "report=%s" % (self.rows_path, report))
            return {"meta": meta, "published": None, "report": str(report)}
        dest = self._publish()
        shutil.copy2(self.dir / "campaign_meta.json",
                     dest / "campaign_meta.json")
        report = self._run_f_eval(dest / "rows.jsonl", dest / "f_report.md")
        self._log("campaign complete: published=%s report=%s"
                  % (dest, report))
        return {"meta": meta, "published": str(dest), "report": str(report)}

    def _publish(self):
        """The existing --publish mechanism, unmodified (run_agentbench.py
        owns it): copies rows.jsonl + provenance into
        results_published/<campaign_id>/ and refuses to overwrite."""
        import run_agentbench
        return Path(run_agentbench.publish_run(self.dir))

    def _run_f_eval(self, rows_path, out_path):
        ev = f_eval.evaluate(f_eval.load_rows([rows_path]))
        return f_eval.emit(ev, out_path)


# ---------------------------------------------------------------------------
# Smoke (Phase R item 1): one n=1 cell, end to end, marked exploratory.
# ---------------------------------------------------------------------------

def run_smoke(campaign_dir, *, model, sim="omnisim",
              task=SMOKE_DEFAULT_TASK, condition=f_eval.SHELL,
              executor=None, quiet=False):
    """One real credentialed run. Validates the HTTP round trip, real token
    numbers and real usd. n=1 -> ``exploratory``, never aggregated (SPEC
    §3.5); the row lands in ``<campaign_dir>/smoke/rows.jsonl``, never in
    the campaign's own rows -- and it deliberately touches no campaign
    ``state.json``, so the scored campaign starts clean afterwards."""
    spec = RunSpec(sim=sim, task=task, condition=condition, lane="smoke",
                   repeat=0, n=1)
    smoke_dir = Path(campaign_dir) / "smoke"
    if executor is None:
        row = subprocess_executor(spec, smoke_dir / "attempt_0", 0,
                                  model=model, dry_run=False)
    else:
        row = executor(spec, smoke_dir / "attempt_0", 0)
    row["campaign"] = {
        "id": Path(campaign_dir).name, "sim": spec.sim, "lane": "smoke",
        "cell": spec.cell, "repeat": 0, "child_repeat": row.get("repeat"),
        "scheduled_n": 1, "attempt": 1, "max_attempts": 1,
        "superseded": False, "retry_reason": None, "model_flag": model,
        "dry_run": False, "exploratory": True, "smoke": True}
    rows_path = smoke_dir / "rows.jsonl"
    rows_path.parent.mkdir(parents=True, exist_ok=True)
    with open(rows_path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(row, default=str) + "\n")
    m = row.get("metrics") or {}
    if not quiet:
        print("smoke row -> %s" % rows_path)
        print("  outcome=%s stop_reason=%s" % (row.get("outcome"),
                                               row.get("stop_reason")))
        print("  t_agent_s=%s t_total_s=%s turns=%s tool_calls=%s"
              % (m.get("t_agent_s"), m.get("t_total_s"), m.get("turns"),
                 m.get("tool_calls")))
        print("  tokens_in=%s tokens_out=%s tokens_cache_read=%s "
              "tokens_cache_write=%s usd=%s"
              % (m.get("tokens_in"), m.get("tokens_out"),
                 m.get("tokens_cache_read"), m.get("tokens_cache_write"),
                 m.get("usd")))
        print("  exploratory n=1: this row validates the round trip and the "
              "cost arithmetic; it is barred from every aggregate and every "
              "claim (SPEC §3.5).")
    return row


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_sims(raw):
    sims = [s.strip() for s in raw.split(",") if s.strip()]
    if not sims:
        raise CampaignError("--sims must name at least one simulator")
    for s in sims:
        if s not in VALID_SIMS:
            raise CampaignError("unknown sim %r (valid: %s)"
                                % (s, ", ".join(VALID_SIMS)))
    return sims


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", default=None,
                    help="EXACT model id for every scored cell (required "
                         "for scored runs and --smoke; recorded in every "
                         "row). There is deliberately no default.")
    ap.add_argument("--sims", default="omnisim",
                    help="comma list: omnisim (default) or omnisim,webots")
    ap.add_argument("--campaign-id", default=None,
                    help="campaign directory name; reusing an existing id "
                         "resumes it (default: campaign_<utc>)")
    ap.add_argument("--out-root", default=str(CAMPAIGNS_ROOT),
                    help="where campaign dirs live (default: "
                         "results/campaigns/ -- gitignored scratch; "
                         "publication copies rows to results_published/)")
    ap.add_argument("--dry-run", action="store_true",
                    help="full schedule walk on the scripted/fake-sim path: "
                         "no key, no model, no simulator")
    ap.add_argument("--smoke", action="store_true",
                    help="run exactly one n=1 cell end-to-end (exploratory, "
                         "never aggregated) to validate round trip, tokens "
                         "and usd")
    ap.add_argument("--smoke-task", default=SMOKE_DEFAULT_TASK)
    ap.add_argument("--smoke-condition", default=f_eval.SHELL,
                    choices=list(CONDITIONS))
    ap.add_argument("--no-publish", action="store_true",
                    help="skip the results_published/ copy (rows stay in "
                         "the campaign dir; the F report is still written)")
    args = ap.parse_args(argv)

    if args.dry_run and args.smoke:
        print("refusing: --dry-run and --smoke are different validations; "
              "run them separately", file=sys.stderr)
        return 2

    try:
        sims = _parse_sims(args.sims)
    except CampaignError as exc:
        print("refusing: %s" % exc, file=sys.stderr)
        return 2

    # -- the gates (scored + smoke) --------------------------------------
    if not args.dry_run:
        if not args.model or not args.model.strip():
            print("refusing: --model is required for scored runs. Model "
                  "pinning means the exact model id is a flag, recorded in "
                  "every row; this driver will not supply a default "
                  "(plan §2.3/§3, SPEC §4.4).", file=sys.stderr)
            return 2
        if not os.environ.get("ANTHROPIC_API_KEY"):
            print("refusing: ANTHROPIC_API_KEY is not set. Scored runs and "
                  "--smoke spend real tokens and need the real credential; "
                  "use --dry-run for the no-key schedule walk.",
                  file=sys.stderr)
            return 2

    cid = args.campaign_id or time.strftime("campaign_%Y%m%d_%H%M%SZ",
                                            time.gmtime())
    campaign_dir = Path(args.out_root) / cid

    if args.smoke:
        run_smoke(campaign_dir, model=args.model.strip(), sim=sims[0],
                  task=args.smoke_task, condition=args.smoke_condition)
        return 0

    try:
        camp = Campaign(campaign_dir,
                        model=(args.model.strip() if args.model else None),
                        sims=sims, dry_run=args.dry_run,
                        publish=not args.no_publish)
    except CampaignError as exc:
        print("refusing: %s" % exc, file=sys.stderr)
        return 2

    n_total = len(camp.schedule)
    n_done = len(camp.state["completed"])
    print("campaign %s: %d scheduled runs (%s), %d already complete%s"
          % (cid, n_total, ", ".join(sims), n_done,
             " [DRY RUN]" if args.dry_run else ""))
    try:
        result = camp.run()
    except CredentialMissing as exc:
        print("campaign interrupted: %s" % exc, file=sys.stderr)
        return 3
    print(json.dumps(result["meta"], indent=2))
    if result["published"]:
        print("published -> %s" % result["published"])
    print("F report -> %s" % result["report"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
