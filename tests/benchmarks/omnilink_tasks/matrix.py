#!/usr/bin/env python3
# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Engine matrix: the whole suite x N engines x R repeats.

    # everything the account can serve, three samples each
    python tests/benchmarks/omnilink_tasks/matrix.py \\
        --engines g1-engine,g3-engine,local --repeat 3

    # prove the harness works with no sim, no account, no network
    python tests/benchmarks/omnilink_tasks/matrix.py --dry-run

    # what would run
    python tests/benchmarks/omnilink_tasks/matrix.py --list

WHY ``local`` IS IN THE DEFAULT ENGINE LIST
-------------------------------------------
``local`` is the no-LLM control: the prompt goes straight to a bridge's regex
intent router. It scores whatever the harness scores with no model in the
loop. Without it a suite cannot tell you how much of a result is the model —
a task that the control also passes is measuring the scaffolding.

MISSING CREDENTIALS ARE A FIRST-CLASS OUTCOME
---------------------------------------------
On an account with only some providers connected, most ``gN-engine`` routes
answer 402 BYOK_REQUIRED. That is recorded as ``SKIPPED: no credential`` for
every task on that engine and printed in the table as ``skipped``. It is not
a crash and it is not a zero.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

import ol_provenance as prov                                   # noqa: E402
import ol_suite                                                # noqa: E402
from ol_bridges import SWARM_PORTS, Stack                      # noqa: E402
from ol_costs import (DEFAULT_BASE_URL, AgentCostSampler,      # noqa: E402
                      NullCostSampler)
from ol_driver import LocalRouterDriver, OmniLinkDriver        # noqa: E402
from ol_runner import console_table, run_engine                # noqa: E402

AGENT_NAME = "HuskySwarm"
DEFAULT_TOOL_URL = "http://127.0.0.1:51520/tool"
CONTROL_BRIDGE = "husky_ne"
RESULTS_DIR = THIS_DIR / "results"


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def build_driver(engine: str, args: argparse.Namespace, stack: Stack) -> Any:
    if engine == "local":
        return LocalRouterDriver(
            bridge_url=f"http://127.0.0.1:{stack.ports[CONTROL_BRIDGE]}")
    return OmniLinkDriver(
        agent_name=args.agent, engine=engine, tool_url=args.tool_url,
        omni_key=os.environ.get("OMNI_KEY", "").strip(),
        model=args.model, max_turns=args.max_turns,
        chat_timeout_s=args.chat_timeout, tool_timeout_s=args.tool_timeout)


def build_cost_sampler(engine: str, args: argparse.Namespace) -> Any:
    if engine == "local":
        return NullCostSampler("local control: no model, no provider cost")
    key = os.environ.get("OMNI_KEY", "").strip()
    if not key:
        return NullCostSampler("OMNI_KEY not set — cost not measurable")
    return AgentCostSampler(agent_name=args.agent, omni_key=key,
                            base_url=args.base_url)


class StackLock:
    """Exclusive claim on the shared robot stack, for the duration of a run.

    WHY THIS EXISTS. On 2026-07-26 two `matrix.py` processes ran concurrently
    for 36 minutes against the same four bridges and the same coordinator. The
    rows interleave second-by-second and several share the same second. The
    damage was not a slow run, it was SILENTLY WRONG SCORES:

      * `ol_runner._run_one` disarms the e-stop at the start of EVERY task, so
        one process cleared the other's fault mid-episode 44 times and those
        honesty rows graded INVALID;
      * `/reset_to_home` from one run landed inside the other's episode, so a
        task that had genuinely arrived measured as still sitting at spawn;
      * every "the other robots must not move" assertion saw the other run's
        robots moving, and failed agents that had done exactly the right thing.

    Roughly half of all recorded failures in that campaign trace to the
    collision rather than to any model. A benchmark that can be quietly
    invalidated by a second copy of itself is not a benchmark, so this refuses
    to start rather than producing numbers nobody can trust.

    The lock is a file holding {pid, host, started_utc, argv}. A stale lock
    whose pid is gone is reclaimed automatically, so a killed run does not
    wedge the suite.
    """

    def __init__(self, path: Path) -> None:
        self.path = path
        self.held = False

    @staticmethod
    def _alive(pid: int) -> bool:
        if pid <= 0:
            return False
        try:
            if os.name == "nt":
                out = subprocess.run(
                    ["tasklist", "/FI", "PID eq %d" % pid, "/NH"],
                    capture_output=True, text=True, timeout=10).stdout
                return str(pid) in out
            os.kill(pid, 0)
            return True
        except (OSError, subprocess.SubprocessError):
            return False

    def acquire(self) -> Optional[Dict[str, Any]]:
        """None on success, or the holder's record if someone else has it."""
        if self.path.exists():
            try:
                held = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                held = {}
            pid = int(held.get("pid", 0) or 0)
            if pid != os.getpid() and self._alive(pid):
                return held
            print("[lock] reclaiming stale lock from pid %s (started %s)"
                  % (pid, held.get("started_utc", "?")))
        rec = {"pid": os.getpid(), "host": socket.gethostname(),
               "started_utc": datetime.now(timezone.utc).isoformat(),
               "argv": sys.argv[1:]}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(rec, indent=2), encoding="utf-8")
        self.held = True
        return None

    def release(self) -> None:
        if not self.held:
            return
        try:
            self.path.unlink()
        except OSError:
            pass
        self.held = False


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engines", default="g1-engine,g3-engine,local",
                    help="comma-separated. 'local' is the no-LLM control "
                         "(regex intent router). Default: %(default)s")
    ap.add_argument("--suite", choices=("core", "hard"), default="core",
                    help="benchmark contract to run. 'core' is "
                         "omnilink-tasks/v1; 'hard' is omnilink-hard/v1.")
    ap.add_argument("--repeat", type=int, default=1,
                    help="samples per (engine, task). LLM runs are "
                         "stochastic; 1 is not a result.")
    ap.add_argument("--tasks", default="", help="comma-separated task ids")
    ap.add_argument("--categories", default="",
                    help=f"comma-separated, from {', '.join(ol_suite.CATEGORIES)}")
    ap.add_argument("--model", default="",
                    help="pin the provider model (e.g. gemini-3.1-flash-lite). "
                         "gN-engine is a ROUTE, not a model — an unpinned run "
                         "cannot be labelled by model.")
    ap.add_argument("--agent", default=AGENT_NAME)
    ap.add_argument("--tool-url", default=os.environ.get(
        "HUSKY_SWARM_TOOL_URL", DEFAULT_TOOL_URL))
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL)
    ap.add_argument("--max-turns", type=int, default=10)
    ap.add_argument("--chat-timeout", type=float, default=180.0)
    ap.add_argument("--tool-timeout", type=float, default=180.0)
    ap.add_argument("--pace", type=float, default=0.0,
                    help="seconds between tasks. Providers throttle "
                         "back-to-back agent runs, and a throttled run looks "
                         "exactly like an incompetent one.")
    ap.add_argument("--no-reset", action="store_true",
                    help="skip the between-task arena reset (faster, and "
                         "cross-task contamination becomes your problem)")
    ap.add_argument("--out-dir", default="")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dry-run", action="store_true",
                    help="run the whole matrix against the in-memory fake "
                         "arena and scripted agents. No sim, no account, no "
                         "network. Proves the harness, measures nothing.")
    ap.add_argument("--dry-run-script", default="good",
                    choices=["good", "bad", "mixed"],
                    help="which scripted agent the dry run replays")
    args = ap.parse_args(argv)
    prov.enable_utf8_console()
    suite_module = ol_suite
    if args.suite == "hard":
        import ol_hard_suite
        suite_module = ol_hard_suite
    prov.SUITE = getattr(suite_module, "SUITE_ID", prov.SUITE)

    try:
        tasks = suite_module.select(
            [t for t in args.tasks.split(",") if t.strip()],
            [c for c in args.categories.split(",") if c.strip()])
    except KeyError as exc:
        print(f"ERROR: {exc}")
        return 2
    if not tasks:
        print("ERROR: no tasks selected")
        return 2

    engines = [e.strip() for e in args.engines.split(",") if e.strip()]

    if args.list:
        print(suite_module.task_table())
        print(f"\n{len(tasks)} task(s) x {len(engines)} engine(s) x "
              f"{args.repeat} repeat(s) = "
              f"{len(tasks) * len(engines) * args.repeat} runs")
        for e in engines:
            print(f"  engine: {e}" + ("   [no-LLM control]" if e == "local" else ""))
        return 0

    out_dir = Path(args.out_dir) if args.out_dir else \
        RESULTS_DIR / f"matrix-{_stamp()}"
    if args.dry_run:
        out_dir = Path(args.out_dir) if args.out_dir else \
            RESULTS_DIR / f"dryrun-{_stamp()}"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows_path = out_dir / "rows.jsonl"

    if args.dry_run:
        return _dry_run(args, tasks, engines, out_dir, rows_path)

    # ── live ─────────────────────────────────────────────────────────
    if any(e != "local" for e in engines) and not os.environ.get("OMNI_KEY"):
        print("ERROR: OMNI_KEY is not set, so no OmniLink engine can be run.\n"
              "       Get one: python -m omnisim key\n"
              "       Or run the control only:  --engines local")
        return 2

    # EXCLUSIVE CLAIM ON THE STACK. Taken before preflight: two runs can
    # both see a healthy stack and then destroy each other's episodes.
    lock = StackLock(RESULTS_DIR / ".stack.lock")
    holder = lock.acquire()
    if holder is not None:
        print(chr(10).join([
            'ERROR: another benchmark run holds the robot stack -- refusing to start.',
            '       pid {holder_pid} on {holder_host}, since {holder_started}',
            '       argv: {holder_argv}',
            '',
            '  Two runs against one stack do not merely interleave, they corrupt each',
            '  other: every task disarms the e-stop and resets the arena, so the other',
            "  run's faults get cleared and its robots teleported mid-episode. About",
            '  half the failures in the 2026-07-26 campaign came from exactly that.',
            '',
            '  Wait for it to finish. If that pid is dead, delete:',
            '    {lock_path}',
        ]).format(
            holder_pid=holder.get("pid"),
            holder_host=holder.get("host"),
            holder_started=holder.get("started_utc"),
            holder_argv=" ".join(holder.get("argv") or []) or "(none)",
            lock_path=RESULTS_DIR / ".stack.lock"))
        return 4

    try:
        return _run_live(args, tasks, engines, out_dir, rows_path)
    finally:
        lock.release()


def _run_live(args: argparse.Namespace, tasks: List[Any], engines: List[str],
              out_dir: Path, rows_path: Path) -> int:
    stack = Stack(bridge_token=os.environ.get("OMNISIM_BRIDGE_TOKEN", ""))
    pre = stack.preflight()
    print("[preflight] bridges: " + ", ".join(
        f"{h}={'up' if up else 'DOWN'}" for h, up in pre["bridges"].items()))
    print("[preflight] unit agents: " + ", ".join(
        f"{a}={'up' if up else 'down'}" for a, up in pre["units"].items()))
    if not pre["ok"]:
        print("\nERROR: the stack is not ready — refusing to run, because a "
              "dead stack scores as model failures.\n  " +
              "\n  ".join(pre["problems"]) +
              "\n\nStart it:\n"
              "  python -m omnisim run-agent --agent husky_swarm --headless\n"
              "  python agents/production/husky_swarm/swarm_agent.py")
        return 3

    all_rows: List[Dict[str, Any]] = []
    engine_cost: Dict[str, Dict[str, Any]] = {}
    for engine in engines:
        print(f"\n\n########## {engine} "
              f"{'(no-LLM control)' if engine == 'local' else ''} ##########")
        driver = build_driver(engine, args, stack)
        sampler = build_cost_sampler(engine, args)
        res = run_engine(tasks=tasks, driver=driver, stack=stack,
                         cost_sampler=sampler, engine=engine,
                         model=(args.model or None), repeats=args.repeat,
                         out_jsonl=rows_path, preflight=pre,
                         reset_between=not args.no_reset, pace_s=args.pace)
        all_rows.extend(res.rows)
        engine_cost[engine] = res.suite_cost

    return _report(all_rows, engine_cost, args, out_dir, rows_path)


def _dry_run(args: argparse.Namespace, tasks: List[Any], engines: List[str],
             out_dir: Path, rows_path: Path) -> int:
    """Exercise the entire pipeline offline."""
    from ol_driver import CredentialMissing
    from ol_fakes import (BAD, GOOD, FakeCostSampler, FakeStack, FakeWorld,
                          ScriptedDriver)

    os.environ.setdefault("OMNILINK_BENCH_FAST_FINGERPRINT", "1")
    prov._FINGERPRINT_CACHE = None  # honour the flag we just set

    scripts = {"good": GOOD, "bad": BAD}
    print(f"[dry-run] fake arena, scripted agents, no network. "
          f"Output: {out_dir}")

    all_rows: List[Dict[str, Any]] = []
    engine_cost: Dict[str, Dict[str, Any]] = {}
    # Three synthetic arms so the report shows a pass column, a fail column
    # and a no-credential column side by side.
    arms = [("dryrun-good", "good", None, "ok"),
            ("dryrun-bad", "bad", None, "credits_zero"),
            ("dryrun-nocred", "good",
             CredentialMissing("dryrun-nocred", "no provider key on this "
                                                "account"), "migration")]
    if args.dry_run_script != "mixed":
        arms = [a for a in arms if a[1] == args.dry_run_script or a[2]]

    for engine, which, raiser, cost_mode in arms:
        world = FakeWorld()
        stack = FakeStack(world)
        driver = ScriptedDriver(world, scripts.get(which, GOOD),
                                engine=engine, raise_on=raiser)
        sampler = FakeCostSampler(cost_mode)
        pre = stack.preflight()
        print(f"\n########## {engine} ##########")
        res = run_engine(tasks=tasks, driver=driver, stack=stack,
                         cost_sampler=sampler, engine=engine, model=None,
                         repeats=args.repeat, out_jsonl=rows_path,
                         preflight=pre, reset_between=True)
        all_rows.extend(res.rows)
        engine_cost[engine] = res.suite_cost

    rc = _report(all_rows, engine_cost, args, out_dir, rows_path,
                 dry_run=True)
    print("\n[dry-run] These numbers measure the HARNESS, not any model. "
          "They are scripted.")
    return rc


def _report(rows: List[Dict[str, Any]], engine_cost: Dict[str, Dict[str, Any]],
            args: argparse.Namespace, out_dir: Path, rows_path: Path,
            dry_run: bool = False) -> int:
    print(console_table(rows))

    header = ("# OmniLink agent benchmark — engine matrix\n"
              if not dry_run else
              "# OmniLink agent benchmark — DRY RUN (scripted, measures "
              "nothing)\n")
    md = prov.markdown_summary(rows, repeats=args.repeat,
                               engine_cost=engine_cost, header=header)
    md += "\n## Cost detail\n\n"
    md += "| engine | measured (credits=USD) | derived from measured units | source |\n"
    md += "|---|---|---|---|\n"
    for eng, c in engine_cost.items():
        measured = c.get("credits_usd")
        derived = c.get("derived_usd")
        md += (f"| {eng} | "
               f"{('$%.4f' % measured) if isinstance(measured, (int, float)) else '**unmeasured**'} | "
               f"{('$%.4f' % derived) if isinstance(derived, (int, float)) else '—'} | "
               f"{c.get('source', '?')} |\n")
    notes = {n for c in engine_cost.values() for n in (c.get("notes") or [])}
    if notes:
        md += "\n" + "\n".join(f"> {n}" for n in sorted(notes)) + "\n"

    (out_dir / "summary.md").write_text(md, encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps({
        "suite": prov.SUITE,
        "utc": prov.utc_now(),
        "git_sha": prov.git_sha(),
        "repeats": args.repeat,
        "dry_run": dry_run,
        "engine_cost": engine_cost,
        "aggregate": prov.aggregate(rows),
        "machine": prov.machine_fingerprint(),
    }, indent=2, default=str), encoding="utf-8")

    print(f"\nrows     -> {rows_path}")
    print(f"summary  -> {out_dir / 'summary.md'}")
    print(f"json     -> {out_dir / 'summary.json'}")

    graded = [r for r in rows if r["outcome"] in prov.GRADED]
    if not graded:
        print("\nNOTHING WAS GRADED. This run is not a result.")
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
