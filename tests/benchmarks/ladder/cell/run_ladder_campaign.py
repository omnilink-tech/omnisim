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

"""One LANE of a capability-ladder campaign: (tier, column) groups, n = 3.

    python tests/benchmarks/ladder/cell/run_ladder_campaign.py \\
        --campaign-id ladder_p1_A --groups T1_arrive:omnisim,T2_transfer:omnisim

    python tests/benchmarks/ladder/cell/run_ladder_campaign.py \\
        --campaign-id dryrun --groups T1_arrive:mujoco --dry-run   # no quota

A group is one **(tier, column)** pair. Within a group the cells run strictly
sequentially at ``n = repeats_default`` (3, ``exploratory``); groups are walked
in the order given, cheapest-decisive first, because
``capability-ladder-plan.md`` §6.4 says the quota -- not the dollar -- is the
scarcest resource in this programme.

What this driver owns, and what it deliberately does not:

* **owns**: the schedule, ``state.json`` resume, the pre-cell resource guard,
  the group summary under the tier's own ``≥ 1 of 3`` rule, and per-group
  publication through the standing "no row, no result" path;
* **does not own**: cross-lane safety. Every cell takes ``cc_lane``'s locks
  itself -- a per-task exclusion lock for the whole cell and the N-slot engine
  semaphore around engine-heavy phases -- so two lanes on this machine are
  safe by mechanism rather than by the operator's care.

**A quota refusal stops the lane rather than substituting an instrument.**
§6.4 is binding: a cell may never be run on a substitute model to beat the
quota, a usage limit is a *pause* recorded as deferred attempts, and it is
never a failed run. When a cell exhausts its deferrals the driver records the
cell as ``deferred`` (no row -- the cell did not run) and **stops**, so the
remaining cells are not burned against a limit that has not reset. Re-running
the identical command resumes from there.

**Resume refuses to run under a changed design.** A campaign whose model pin,
groups or n moved mid-way is two campaigns, and a grid assembled from both
would be unreadable. Same rule as the two drivers this one sits beside.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LADDER = HERE.parent
BENCHMARKS = LADDER.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import concurrency                   # noqa: E402
from agentbench.run_agentbench import publish_run            # noqa: E402
from ladder import tasks as ladder_tasks                     # noqa: E402
from ladder.cell import run_ladder_cell as cell_mod          # noqa: E402
from ladder.cell import stage_ladder_workspace as staging    # noqa: E402

RESOURCE_POLL_S = 60.0
RESOURCE_WAIT_MAX_S = 30 * 60.0

#: §6.1's order: cheapest-decisive first, and T4 last behind the cloud gate.
DEFAULT_TIER_ORDER = ("T1_arrive", "T2_transfer", "T3_quadruped",
                      "T4_humanoid")


def parse_groups(spec):
    """``"TASK:column,TASK:column"`` -> ordered ``[(task_id, column)]``."""
    groups = []
    for part in (spec or "").split(","):
        part = part.strip()
        if not part:
            continue
        task, _, column = part.partition(":")
        if not column:
            raise ValueError("group %r: expected TASK:column" % part)
        tid = ladder_tasks.get(task.strip()).id      # raises on an unknown id
        groups.append((tid, column.strip()))
    if not groups:
        raise ValueError("no groups given (--groups TASK:column,...)")
    return groups


def default_groups(column, tiers=DEFAULT_TIER_ORDER):
    return [(t, column) for t in tiers]


def n_for(task_id, override=None):
    if override:
        return int(override)
    return int(ladder_tasks.get(task_id).repeats_default or 3)


def dedupe_rows(rows):
    """One row per ``cell_key``, newest wins, order preserved.

    **A measured need, not a precaution.** A campaign killed mid-group leaves
    a cell recorded ``running``; the resume re-runs it, and the group's
    ``rows.jsonl`` -- appended to before the state is saved -- then holds the
    cell twice. Observed on the first full dry run: a three-cell group
    summarised itself as *"0/6 achieved"*, which is a wrong denominator in a
    published grid. Rows are keyed by ``task:column:rN``, so the fix is exact
    rather than heuristic.
    """
    out = {}
    for i, r in enumerate(rows):
        key = r.get("cell_key")
        if not key and r.get("task") is not None:
            key = "%s:%s:r%s" % (r.get("task"), r.get("column"),
                                 r.get("repeat"))
        # A row that identifies no cell is NOT collapsed into another: losing
        # a row is worse than keeping a duplicate, and every row this package
        # writes carries a cell_key.
        out[key or ("__unkeyed_%d" % i)] = r
    return list(out.values())


def summarise_group(rows, task_id=None):
    """The group's reading under the tier's own rule. **Never a pass rate.**

    ``achieved_rule`` in every ``meta.json`` is *"reached at least once in 3
    (solved_at_least_once); never quotable as a pass rate"*, and
    ``capability-ladder-plan.md`` §7 bars a published pass rate for an
    ``exploratory`` suite outright. So this returns the k/n counts as
    EVIDENCE and one boolean as the reading, with the bar printed beside it.
    """
    rows = dedupe_rows([r for r in rows if r])
    n = len(rows)
    achieved = [r for r in rows
                if (r.get("cell") or {}).get("value") == "achieved"]
    invalid = [r for r in rows if (r.get("cell") or {}).get("invalid_reason")]
    # ⚠ An INVALID row is NOT a measurement (3.1), so it may not be summed into
    # one. Measured 2026-08-02: a group holding exactly one INVALID cell --
    # the instrument had graded the wrong file -- summarised itself as
    # "not_achieved", which is an agent-attributable reading of a run in which
    # nothing about the agent was measured.
    measured = [r for r in rows if r not in invalid]
    blockers = {}
    for r in rows:
        b = (r.get("cell") or {}).get("blocker")
        if b:
            blockers[b] = blockers.get(b, 0) + 1
    modal = max(blockers, key=lambda k: blockers[k]) if blockers else None
    unknown_share = (blockers.get("unknown", 0) / float(n)) if n else 0.0
    return {
        "task": task_id,
        "n_runs": n,
        "n_measured": len(measured),
        "n_achieved": len(achieved),
        "n_invalid": len(invalid),
        "solved_at_least_once": bool(achieved),
        "cell_value": ("achieved" if achieved else
                       ("not_achieved" if measured else None)),
        "not_measured_warning": (
            "every row in this group is INVALID, so the group has NO cell "
            "value: an invalid cell is not a measurement and a group of them "
            "is not a not_achieved (3.1). The cells must be re-run."
            if rows and not measured else None),
        "modal_blocker": modal,
        "blockers": blockers,
        "all_blockers": [(r.get("cell") or {}).get("blocker") for r in rows],
        "unknown_share": round(unknown_share, 3),
        "not_diagnosed_warning": (
            "more than a third of this group's blockers are 'unknown', so it "
            "is published as NOT DIAGNOSED rather than as findings (3.3)"
            if unknown_share > 1.0 / 3 else None),
        "rule": ("reached at least once in %d (solved_at_least_once). NEVER "
                 "quotable as a pass rate: this suite is exploratory (SPEC "
                 "3.5) and 'achieved' means reached at least once, not "
                 "reliable (7)." % n),
    }


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class Campaign:
    def __init__(self, args, run_cell=None):
        self.args = args
        self.run_cell = run_cell or cell_mod.run_cell
        self.dir = (Path(args.results_root) if args.results_root
                    else (LADDER / "results" / "campaigns")) / args.campaign_id
        self.dir.mkdir(parents=True, exist_ok=True)
        self.state_path = self.dir / "state.json"
        self.groups = (parse_groups(args.groups) if args.groups
                       else default_groups(args.column))
        self.config = {
            "campaign_id": args.campaign_id,
            "lane": args.lane,
            "groups": ["%s:%s" % g for g in self.groups],
            "n": args.n,
            "model": args.model,
            "dry_run": bool(args.dry_run),
            "engine_slots": args.engine_slots,
        }
        self.state = self._load_state()

    def _load_state(self):
        if self.state_path.is_file():
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            prev = dict(state.get("config") or {})
            if prev != dict(self.config):
                raise SystemExit(
                    "state.json holds a different configuration -- a campaign "
                    "whose model, schedule or n moved mid-way is TWO "
                    "campaigns:\n  was: %s\n  now: %s\nresume with the "
                    "identical command, or use a new --campaign-id"
                    % (prev, self.config))
            return state
        return {"config": dict(self.config), "cells": {}, "groups": {},
                "published": {}, "created_utc": _utc()}

    def _save(self):
        self.state["updated_utc"] = _utc()
        self.state_path.write_text(json.dumps(self.state, indent=2,
                                              default=str), encoding="utf-8")

    @staticmethod
    def _append_group_row(group_dir, row):
        """Append the row and rewrite the file deduped by ``cell_key``.

        The per-cell ``rows.jsonl`` under ``cells/<key>/`` is the untouched
        original either way; this file is the group's assembled view, and a
        re-run cell must replace its own earlier attempt rather than sit
        beside it (see :func:`dedupe_rows`).
        """
        p = Path(group_dir) / "rows.jsonl"
        rows = []
        if p.is_file():
            rows = [json.loads(ln) for ln in
                    p.read_text(encoding="utf-8").splitlines() if ln.strip()]
        rows.append(row)
        rows = dedupe_rows(rows)
        p.write_text("".join(json.dumps(r, default=str) + "\n" for r in rows),
                     encoding="utf-8")
        return p

    def _wait_for_resources(self):
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
                      "and recording it" % waited)
                return detail
            print("resource guard: waiting (%s); retry in %.0f s"
                  % (detail, RESOURCE_POLL_S))
            time.sleep(RESOURCE_POLL_S)

    def run(self):
        swept = staging.sweep_pending_deletes(Path(self.args.root))
        if swept["deleted"] or swept["failed"]:
            print("pending-delete sweep: deleted=%s failed=%s"
                  % (swept["deleted"], swept["failed"]))
            self.state.setdefault("pending_delete_sweeps", []).append(
                {"utc": _utc(), **swept})
            self._save()

        for task_id, column in self.groups:
            group_key = "%s:%s" % (task_id, column)
            group_dir = self.dir / "groups" / ("%s_%s" % (task_id, column))
            group_dir.mkdir(parents=True, exist_ok=True)
            n = n_for(task_id, self.args.n)
            group_done = True
            for rep in range(n):
                key = "%s:r%d" % (group_key, rep)
                rec = self.state["cells"].get(key) or {}
                if rec.get("status") == "done":
                    continue
                print("\n=== cell %s (%d/%d in %s) ==="
                      % (key, rep + 1, n, group_key))
                guard = self._wait_for_resources()
                out_dir = self.dir / "cells" / key.replace(":", "_")
                self.state["cells"][key] = {
                    "status": "running", "started_utc": _utc(),
                    "out_dir": str(out_dir), "resource_guard": guard}
                self._save()
                try:
                    row = self.run_cell(
                        column, task_id, root=Path(self.args.root),
                        out_dir=out_dir, model=self.args.model, repeat=rep,
                        dry_run=self.args.dry_run,
                        timeout_s=self.args.timeout_s,
                        lane=self.args.lane,
                        engine_slots=self.args.engine_slots,
                        lock_root=self.args.lock_root,
                        rate_limit_backoff_s=self.args.rate_limit_backoff_s,
                        max_rate_limit_retries=(
                            self.args.max_rate_limit_retries),
                        backend=self.args.backend,
                        campaign={"id": self.args.campaign_id,
                                  "lane": self.args.lane, "group": group_key,
                                  "n": n})
                except cell_mod.RateLimited as exc:
                    self.state["cells"][key] = {
                        "status": "deferred", "detail": str(exc),
                        "ended_utc": _utc(), "out_dir": str(out_dir)}
                    self._save()
                    print("\nQUOTA PAUSE: %s\nThe lane STOPS here rather than "
                          "substituting an instrument (6.4). Re-run the "
                          "identical command to resume." % exc)
                    return 3
                except SystemExit as exc:
                    self.state["cells"][key] = {
                        "status": "blocked", "blocker": str(exc),
                        "ended_utc": _utc(), "out_dir": str(out_dir)}
                    self._save()
                    group_done = False
                    print("cell %s blocked: %s -- continuing; re-run the same "
                          "command to retry it" % (key, exc))
                    continue
                self._append_group_row(group_dir, row)
                cell = row.get("cell") or {}
                self.state["cells"][key] = {
                    "status": "done", "cell_value": cell.get("value"),
                    "blocker": cell.get("blocker"),
                    "invalid_reason": cell.get("invalid_reason"),
                    "outcome": row.get("outcome"),
                    "measured_under_concurrency":
                        row.get("measured_under_concurrency"),
                    "ended_utc": _utc(), "out_dir": str(out_dir)}
                self._save()

            # -- the group's reading + publication ------------------------
            rows_path = group_dir / "rows.jsonl"
            if rows_path.is_file():
                rows = [json.loads(ln) for ln in
                        rows_path.read_text(encoding="utf-8").splitlines()
                        if ln.strip()]
                summary = summarise_group(rows, task_id)
                summary["column"] = column
                summary["complete"] = group_done
                (group_dir / "group_summary.json").write_text(
                    json.dumps(summary, indent=2), encoding="utf-8")
                self.state["groups"][group_key] = summary
                self._save()
                print("group %s: %s (%d/%d achieved) -- %s"
                      % (group_key, summary["cell_value"],
                         summary["n_achieved"], summary["n_runs"],
                         summary["rule"]))
            # A DRY RUN NEVER PUBLISHES. ``results_published/`` is the tracked
            # tree "no row, no result" points at (§3.4 rule 5), and a scripted
            # oracle's row is not a cell -- it must not be able to arrive
            # there by a forgotten flag.
            if group_done and rows_path.is_file() \
                    and group_key not in self.state["published"] \
                    and not self.args.no_publish and not self.args.dry_run:
                pub = "%s_%s_%s" % (self.args.campaign_id, task_id, column)
                staged = self.dir / "publish_staging" / pub
                staged.mkdir(parents=True, exist_ok=True)
                (staged / "rows.jsonl").write_bytes(rows_path.read_bytes())
                if (group_dir / "group_summary.json").is_file():
                    (staged / "group_summary.json").write_bytes(
                        (group_dir / "group_summary.json").read_bytes())
                try:
                    dest = publish_run(staged)
                    self.state["published"][group_key] = str(dest)
                    print("published group %s -> %s" % (group_key, dest))
                except FileExistsError as exc:
                    print("publish skipped (already published): %s" % exc)
                    self.state["published"][group_key] = "pre-existing"
                self._save()

        done = sum(1 for c in self.state["cells"].values()
                   if c.get("status") == "done")
        total = sum(n_for(t, self.args.n) for t, _ in self.groups)
        print("\nlane complete: %d/%d cells done" % (done, total))
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--campaign-id", required=True)
    ap.add_argument("--lane", default="A",
                    help="lane label, recorded in locks and rows")
    ap.add_argument("--groups", default=None,
                    help="ordered lane assignment: TASK:column,TASK:column")
    ap.add_argument("--column", default="omnisim",
                    help="column for the default group set (all four tiers)")
    ap.add_argument("--n", type=int, default=None,
                    help="cells per group (default: the task's own "
                         "repeats_default, which is 3)")
    ap.add_argument("--model", default=cell_mod.DEFAULT_MODEL,
                    help="the PINNED model id for every cell in this "
                         "campaign (default %(default)s). Resume refuses to "
                         "run under a different one.")
    ap.add_argument("--dry-run", action="store_true",
                    help="walk the whole schedule through the tiers' "
                         "COMMITTED SCRIPTED ORACLES instead of Claude "
                         "sessions: no quota, no agent, real graders")
    ap.add_argument("--root", default=str(staging.DEFAULT_ROOT))
    ap.add_argument("--results-root", default=None)
    ap.add_argument("--lock-root", default=None)
    ap.add_argument("--engine-slots", type=int,
                    default=concurrency.DEFAULT_ENGINE_SLOTS)
    ap.add_argument("--timeout-s", type=float, default=None)
    ap.add_argument("--backend", default=None, choices=(None, "ode", "newton"))
    ap.add_argument("--rate-limit-backoff-s", type=float,
                    default=cell_mod.DEFAULT_RATE_LIMIT_BACKOFF_S)
    ap.add_argument("--max-rate-limit-retries", type=int,
                    default=cell_mod.DEFAULT_MAX_RATE_LIMIT_RETRIES)
    ap.add_argument("--min-free-ram-gb", type=float,
                    default=concurrency.MIN_FREE_RAM_GB)
    ap.add_argument("--max-cpu-load-pct", type=float,
                    default=concurrency.MAX_CPU_LOAD_PCT)
    ap.add_argument("--no-publish", action="store_true",
                    help="do not copy finished groups into "
                         "results_published/ (a dry run should not)")
    args = ap.parse_args(argv)
    return Campaign(args).run()


if __name__ == "__main__":
    sys.exit(main())
