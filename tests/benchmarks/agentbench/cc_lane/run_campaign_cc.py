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

"""One LANE of the Phase W Claude Code campaign (plan §2.7, RUNBOOK §7).

    python tests/benchmarks/agentbench/cc_lane/run_campaign_cc.py \\
        --campaign-id phasew_cc_A --lane A \\
        --groups B1_overlap_audit:omnisim,B2_subject_in_frame:omnisim,...

A lane is a list of ``(task, sim)`` groups walked IN ORDER; within a group
the n cells run STRICTLY SEQUENTIALLY. **n defaults to 1** -- the protocol is
ONE run per (task, sim) under a wall-clock ceiling (owner, 2026-08-10;
``tasks.TASK_HARD_CEILING_S``), not the n = 5 / n = 10 repeats it used to be.
A single run measures an OUTCOME and estimates NO variance; every row says so
in its own ``protocol`` block. ``--n`` / ``--n-a1`` still accept more for a
deliberate, recorded variance experiment. Two lanes may run on
this machine at once, split by simulator (lane A = omnisim cells, lane B =
webots cells); cross-lane safety is not this driver's trust in the operator
but ``concurrency.py``'s locks, which every cell takes itself:

* same-task exclusion  -- a per-task lock held for the whole cell, so two
  lanes can never overlap on one task (plan §5.3);
* the engine semaphore -- max ``--engine-slots`` (default 2) engine-heavy
  phases machine-wide;
* the pre-cell resource guard -- a cell is SKIPPED-AND-RETRIED-LATER while
  free RAM < 4 GB or CPU load is high (psutil, else wmic/procfs);
* rate-limit deferral  -- a `claude -p` usage/rate-limit refusal is a
  deferred attempt inside the cell, never a consumed run.

State (``state.json`` in the campaign dir) records every cell's status;
**re-running the identical command resumes** from the first incomplete cell
and never re-runs a completed one. The driver refuses to resume under a
changed configuration (model / groups / n), same rule as the API-lane
driver -- which this file deliberately does NOT touch or reuse
(``campaign.py`` belongs to the API lane).

At the end of each finished group the group's ``rows.jsonl`` is published
through the EXISTING ``run_agentbench.py --publish`` mechanism (the "no row,
no result" path): ``results_published/<campaign-id>_<task>_<sim>/``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
BENCHMARKS = AGENTBENCH.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import concurrency                   # noqa: E402
from agentbench.cc_lane import run_cc_cell as cell_mod       # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging   # noqa: E402
from agentbench.run_agentbench import publish_run            # noqa: E402
from agentbench import sims                                  # noqa: E402

#: Cells per (task, sim). **ONE** -- the protocol, owner's decision
#: 2026-08-10: *"we are not doing n times anymore. We will do it so that we
#: set a maximum time that the agent should finish before."* Repeats were the
#: dominant token cost of a campaign (Lane B at 5, A1 at 10 = 35 cells for one
#: arm's core set); a single cell under a 45-minute ceiling is the trade that
#: bought the longer budget in ``tasks.TASK_HARD_CEILING_S``.
#:
#: ⚠ The consequence is not free and is not hidden: **there is no variance
#: estimate.** One cell per (task, arm) yields an OUTCOME, never a rate --
#: pass@1, a pass fraction and a confidence interval are undefined at one
#: sample, and every row says so (``run_cc_cell.PROTOCOL_ID`` and the row's
#: own ``protocol`` block). SPEC 3.5 is the contract.
#:
#: ``--n`` / ``--n-a1`` still take a larger number, deliberately: a repeated
#: cell remains possible for a recorded, deliberate variance experiment (SPEC
#: 9.3's re-run clause). It is simply not the default any more, and rows from
#: such a run carry the count they were scheduled under.
DEFAULT_N = 1
DEFAULT_N_A1 = 1
RESOURCE_POLL_S = 60.0
RESOURCE_WAIT_MAX_S = 30 * 60.0

LANE_B_TASKS = ("B1_overlap_audit", "B2_subject_in_frame",
                "B3_measure_and_report", "C1_parse_error_fix",
                "C2_fall_through_floor")


def parse_groups(spec):
    """``"TASK:sim,TASK:sim"`` -> ordered [(task, sim)]."""
    groups = []
    for part in (spec or "").split(","):
        part = part.strip()
        if not part:
            continue
        task, _, sim = part.partition(":")
        if sim not in sims.IMPLEMENTED:
            # Name the declared-but-unbuilt arms too: "not built yet" and
            # "not in the benchmark" are different facts (sims.py).
            raise ValueError(
                "group %r: sim must be one of %s (declared but not yet "
                "runnable: %s)"
                % (part, "|".join(sims.IMPLEMENTED),
                   "|".join(sims.DECLARED)))
        groups.append((task.strip(), sim))
    if not groups:
        raise ValueError("no groups given (--groups TASK:sim,...)")
    return groups


def default_groups(sim):
    """The standard lane assignment for one simulator arm: all five Lane B
    tasks + the A1 control, on that sim."""
    return [(t, sim) for t in LANE_B_TASKS] + [("A1_husky_swarm_10", sim)]


def n_for(task_id, n, n_a1):
    return n_a1 if task_id.startswith("A1") else n


def publish_staging_dir(campaign_dir, pub_name):
    """Where the publication-named copy of a group's rows is staged for
    ``publish_run`` (which names the destination after its source dir).
    Deliberately OUTSIDE ``groups/`` -- a copy in there masquerades as a
    second row set for the same group."""
    return Path(campaign_dir) / "publish_staging" / pub_name


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class Campaign:
    def __init__(self, args):
        self.args = args
        self.dir = (AGENTBENCH / "results" / "cc_lane" / "campaigns"
                    / args.campaign_id)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.state_path = self.dir / "state.json"
        self.groups = (parse_groups(args.groups) if args.groups
                       else default_groups(args.sim))
        self.config = {
            "campaign_id": args.campaign_id,
            "lane": args.lane,
            "groups": ["%s:%s" % g for g in self.groups],
            "n": args.n, "n_a1": args.n_a1,
            "model": args.model,
            "engine_slots": args.engine_slots,
        }
        self.state = self._load_state()

    def _load_state(self):
        if self.state_path.is_file():
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            # Refuse to resume under a different design: a campaign whose
            # model or schedule moved mid-way is two campaigns.
            prev = dict(state.get("config") or {})
            now = dict(self.config)
            if prev.get("model") is None and now.get("model") is not None:
                # the model may be pinned by the first cell (recorded below)
                prev["model"] = now["model"]
            if prev != now:
                raise SystemExit(
                    "state.json holds a different configuration:\n  was: %s"
                    "\n  now: %s\nresume with the identical command, or use "
                    "a new --campaign-id" % (prev, now))
            return state
        return {"config": dict(self.config), "cells": {},
                "published": {}, "created_utc": _utc()}

    def _save(self):
        self.state["updated_utc"] = _utc()
        self.state_path.write_text(json.dumps(self.state, indent=2),
                                   encoding="utf-8")

    def _cell_key(self, task, sim, rep):
        return "%s:%s:r%d" % (task, sim, rep)

    def _wait_for_resources(self):
        """The pre-cell resource guard: skip-and-retry-later while the
        machine is starved. Bounded so an unmeasurable/never-green guard
        cannot deadlock the lane -- after the cap it proceeds and says so."""
        t0 = time.monotonic()
        while True:
            ok, detail = concurrency.resource_guard(
                min_free_ram_gb=self.args.min_free_ram_gb,
                max_cpu_load_pct=self.args.max_cpu_load_pct)
            if ok:
                return detail
            waited = time.monotonic() - t0
            if waited > RESOURCE_WAIT_MAX_S:
                detail["guard_wait_exceeded_s"] = round(waited, 0)
                print("resource guard still red after %.0f s -- proceeding "
                      "and recording it (%s)" % (waited, detail))
                return detail
            print("resource guard: waiting (%s); retry in %.0f s"
                  % (detail, RESOURCE_POLL_S))
            time.sleep(RESOURCE_POLL_S)

    def run(self):
        # Reclaim workspaces a previous (crashed/stopped) run could not
        # delete: *.pending_delete dirs and markers under the staging root.
        swept = staging.sweep_pending_deletes(Path(self.args.root))
        if swept["deleted"] or swept["failed"]:
            print("pending-delete sweep: deleted=%s failed=%s"
                  % (swept["deleted"], swept["failed"]))
            self.state.setdefault("pending_delete_sweeps", []).append(
                {"utc": _utc(), **swept})
            self._save()
        pinned_model = self.args.model or self.state.get("pinned_model")
        for task, sim in self.groups:
            group_key = "%s:%s" % (task, sim)
            group_dir = self.dir / "groups" / ("%s_%s" % (task, sim))
            group_dir.mkdir(parents=True, exist_ok=True)
            n = n_for(task, self.args.n, self.args.n_a1)
            # An arm that cannot EXPRESS this task is refused for the whole
            # group, before a workspace or a token: SPEC 6.4. Its cells are
            # recorded as `not_expressible`, which is deliberately neither
            # `done` (they produce no row and enter no denominator) nor
            # `blocked` (nothing broke -- WE are missing that task's fixture
            # on that simulator, and calling it a blocker would read as the
            # comparator failing). MuJoCo covers A1/R1/R2 today; the other
            # six ship a `.wbt` fixture with no MJCF equivalent.
            if not sims.get(sim).expresses(task):
                reason = "%s cannot express %s" % (sim, task)
                try:
                    sims.require_implemented(sim, task)
                except NotImplementedError as exc:
                    reason = str(exc)
                for rep in range(n):
                    self.state["cells"][self._cell_key(task, sim, rep)] = {
                        "status": "not_expressible", "reason": reason,
                        "ended_utc": _utc()}
                self._save()
                print("group %s SKIPPED: %s" % (group_key, reason))
                continue
            group_done = True
            for rep in range(n):
                key = self._cell_key(task, sim, rep)
                rec = self.state["cells"].get(key) or {}
                if rec.get("status") == "done":
                    continue
                print("\n=== cell %s (%d/%d in group %s) ==="
                      % (key, rep + 1, n, group_key))
                guard = self._wait_for_resources()
                out_dir = self.dir / "cells" / key.replace(":", "_")
                rec = {"status": "running", "started_utc": _utc(),
                       "out_dir": str(out_dir), "resource_guard": guard}
                self.state["cells"][key] = rec
                self._save()
                try:
                    row = cell_mod.run_cell(
                        sim, task, root=Path(self.args.root),
                        out_dir=out_dir, model=pinned_model,
                        timeout_s=self.args.timeout_s,
                        lane=self.args.lane,
                        engine_slots=self.args.engine_slots,
                        lock_root=self.args.lock_root,
                        rate_limit_backoff_s=self.args.rate_limit_backoff_s,
                        max_rate_limit_retries=(
                            self.args.max_rate_limit_retries),
                        repeat=rep,
                        # What this campaign SCHEDULED per (task, sim), onto
                        # the row. At the protocol default of 1 it states the
                        # sample count that makes a rate undefined; above 1 it
                        # says a deliberate repeat experiment produced the row,
                        # so the two can never be confused after the fact.
                        runs_per_cell=n,
                        # R1's graded obstacle layout is drawn from THIS
                        # seed. It carries the campaign id and the repeat but
                        # not the simulator, so every arm's r<n> is scored on
                        # the same layout (a sim-vs-sim comparison must not
                        # also be a layout-vs-layout one) while two repeats,
                        # and two campaigns, get different ones. It is
                        # recorded on every row.
                        layout_seed="%s/%s/r%d"
                                    % (self.args.campaign_id, task, rep))
                except SystemExit as exc:
                    rec.update(status="blocked", blocker=str(exc),
                               ended_utc=_utc())
                    self._save()
                    group_done = False
                    print("cell %s blocked: %s -- continuing with the next "
                          "cell; re-run the same command to retry it later"
                          % (key, exc))
                    continue
                except NotImplementedError as exc:
                    # The group pre-check above catches this case; this is the
                    # belt-and-braces for a registry that says yes while the
                    # runner refuses. Same status, same reason: never a row.
                    rec.update(status="not_expressible", reason=str(exc),
                               ended_utc=_utc())
                    self._save()
                    group_done = False
                    print("cell %s not expressible on %s: %s"
                          % (key, sim, exc))
                    continue
                # Pin the model campaign-wide from the first successful cell
                # when none was passed, and hold every later cell to it.
                model = (row.get("agent") or {}).get("model")
                if pinned_model is None and model:
                    pinned_model = model
                    self.state["pinned_model"] = model
                    print("pinned model for this campaign: %s" % model)
                row.setdefault("campaign", {})
                row["campaign"].update(id=self.args.campaign_id,
                                       lane=self.args.lane, sim=sim)
                with open(group_dir / "rows.jsonl", "a",
                          encoding="utf-8") as fh:
                    fh.write(json.dumps(row, default=str) + "\n")
                rec.update(status="done", outcome=row.get("outcome"),
                           measured_under_concurrency=(
                               (row.get("agent_artifacts") or {})
                               .get("measured_under_concurrency")),
                           ended_utc=_utc())
                self._save()
            # -- group finished: publish through the existing path ---------
            if group_done and (group_dir / "rows.jsonl").is_file() \
                    and group_key not in self.state["published"]:
                pub_name = "%s_%s_%s" % (self.args.campaign_id, task, sim)
                # publish_run names the destination after the source dir, so
                # a staging copy with the publication name is needed -- but
                # it must NOT live under groups/ (it used to: groups/ then
                # held both <task>_<sim> and <campaign>_<task>_<sim> for the
                # same group, which read as duplicate row sets in a forensic
                # pass, 2026-08-01). It gets its own sibling dir.
                staged = publish_staging_dir(self.dir, pub_name)
                if staged != group_dir:
                    if not staged.exists():
                        staged.mkdir(parents=True)
                    (staged / "rows.jsonl").write_bytes(
                        (group_dir / "rows.jsonl").read_bytes())
                try:
                    dest = publish_run(staged)
                    self.state["published"][group_key] = str(dest)
                    print("published group %s -> %s" % (group_key, dest))
                except FileExistsError as exc:
                    print("publish skipped (already published): %s" % exc)
                    self.state["published"][group_key] = "pre-existing"
                self._save()
        skipped = sum(1 for c in self.state["cells"].values()
                      if c.get("status") == "not_expressible")
        print("\nlane complete: %d/%d cells done%s"
              % (sum(1 for c in self.state["cells"].values()
                     if c.get("status") == "done"),
                 sum(n_for(t, self.args.n, self.args.n_a1)
                     for t, _ in self.groups),
                 ("" if not skipped else
                  " (%d not expressible on this arm -- a missing FIXTURE of "
                  "ours, never a result of theirs)" % skipped)))
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--campaign-id", required=True)
    ap.add_argument("--lane", default="A",
                    help="lane label (A = omnisim cells, B = webots cells "
                         "by convention); recorded in locks and rows")
    ap.add_argument("--groups", default=None,
                    help="ordered lane assignment: TASK:sim,TASK:sim,... "
                         "(default: all five Lane B tasks + A1 on --sim)")
    ap.add_argument("--sim", default="omnisim",
                    choices=sims.IMPLEMENTED, metavar="SIM",
                    help="simulator for the default group set when --groups "
                         "is not given (runnable: %s; declared: %s)"
                         % (", ".join(sims.IMPLEMENTED),
                            ", ".join(sims.DECLARED)))
    ap.add_argument("--n", type=int, default=DEFAULT_N,
                    help="cells per Lane B task (default %(default)s -- the "
                         "protocol is ONE run under a wall-clock ceiling, "
                         "which measures an outcome and NO variance). A value "
                         "above 1 is a deliberate variance experiment and is "
                         "recorded as such on every row.")
    ap.add_argument("--n-a1", type=int, default=DEFAULT_N_A1,
                    help="cells for the A1 control (default %(default)s -- "
                         "same protocol; A1's old n = 10 is gone)")
    # Defaulted to the resolved id (not None) so state.json records WHAT RAN.
    # A campaign state reading '"model": null' cannot be audited from the
    # file alone -- the reader has to go find the runner's default.
    ap.add_argument("--model", default=cell_mod.DEFAULT_MODEL,
                    help="pin a model id for every cell (default: %(default)s). A campaign that mixes models is not a comparison; "
                         "changing this forces a suite version bump.")
    ap.add_argument("--root", default=str(staging.DEFAULT_ROOT))
    ap.add_argument("--lock-root", default=None,
                    help="shared lock dir; MUST be identical across lanes "
                         "(default <root>/locks)")
    ap.add_argument("--engine-slots", type=int,
                    default=concurrency.DEFAULT_ENGINE_SLOTS)
    ap.add_argument("--timeout-s", type=float,
                    default=cell_mod.DEFAULT_TIMEOUT_S)
    ap.add_argument("--rate-limit-backoff-s", type=float,
                    default=cell_mod.DEFAULT_RATE_LIMIT_BACKOFF_S)
    ap.add_argument("--max-rate-limit-retries", type=int,
                    default=cell_mod.DEFAULT_MAX_RATE_LIMIT_RETRIES)
    ap.add_argument("--min-free-ram-gb", type=float,
                    default=concurrency.MIN_FREE_RAM_GB)
    ap.add_argument("--max-cpu-load-pct", type=float,
                    default=concurrency.MAX_CPU_LOAD_PCT)
    args = ap.parse_args(argv)
    return Campaign(args).run()


if __name__ == "__main__":
    sys.exit(main())
