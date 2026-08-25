# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Run the suite against ONE engine and emit rows.

Per task, in this order, every time:

    clear any leftover fault  ->  reset the arena and prove it is still
    ->  task setup (direct to the bridge, never via the model)
    ->  arm + VERIFY the injected fault, if the task has one
    ->  snapshot ground-truth pose  ->  snapshot cost
    ->  drive the prompt
    ->  wait for motion to stop  ->  snapshot pose  ->  snapshot cost
    ->  grade  ->  disarm the fault, run teardown

The ordering is not cosmetic. Reading pose while the base is moving is
measurably wrong on this stack (the supervisor pose leads settled truth by
~0.2-0.3 m and snaps back the tick the wheels stop), and a compound tool's
background control thread will move a robot AFTER the next task's snapshot if
the reset does not wait for genuine idle.

FIVE OUTCOMES, AND ONLY TWO OF THEM ARE SCORES
----------------------------------------------
  PASS / FAIL  the model's result
  INVALID      the stack broke, or the task's precondition/fault was not
               established. Never counted in a pass rate.
  SKIPPED      no provider credential for this engine (402 BYOK_REQUIRED).
               Recorded once per task so the matrix shows the gap instead of
               silently omitting the column.
  ERROR        the harness itself failed.
"""

from __future__ import annotations

import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import ol_provenance as prov
from ol_bridges import Stack
from ol_costs import CostSnapshot
from ol_driver import (CredentialMissing, DriverError, EngineSubstituted,
                       Episode)
from ol_suite import GradeContext, SuiteTask, Verdict


@dataclass
class EngineResult:
    engine: str
    model: Optional[str] = None
    rows: List[Dict[str, Any]] = field(default_factory=list)
    suite_cost: Dict[str, Any] = field(default_factory=dict)
    aborted: str = ""            # non-empty => the rest was not attempted


def _fault_arm(stack: Stack, fault: Optional[str]) -> tuple:
    """(armed, description). ``armed`` is None when the task has no fault."""
    if not fault:
        return None, ""
    if fault == "estop":
        ok = stack.arm_estop("omnilink benchmark honesty lane")
        return ok, ("operator e-stop engaged and verified" if ok else
                    "could not engage/verify the operator e-stop")
    if fault.startswith("bridge_unreachable:"):
        husky = fault.split(":", 1)[1]
        ok = stack.verify_bridge_unreachable(husky)
        return ok, (f"{husky}'s bridge is unreachable, as required" if ok else
                    f"{husky}'s bridge is UP — this fault must be armed at "
                    f"launch (see the README: HUSKY_SWARM_BRIDGES with a dead "
                    f"port for {husky})")
    return False, f"unknown fault spec {fault!r}"


def _fault_disarm(stack: Stack, fault: Optional[str]) -> None:
    if fault == "estop":
        try:
            stack.clear_estop()
        except Exception:
            pass


def _team_ok(driver: Any) -> tuple:
    """Does the registered profile declare a 4-agent delegation roster?

    Delegation is executed server-side from ``settings.team``. Without a team
    the platform never offers ``delegate_to_agent``, so the delegation lane
    would score a model for a capability it was never given.
    """
    getter = getattr(driver, "settings", None)
    if not callable(getter):
        return True, ""          # stub/control drivers opt out of the check
    try:
        settings = getter() or {}
    except Exception as exc:  # noqa: BLE001
        return False, f"could not read the agent profile ({type(exc).__name__})"
    team = settings.get("team") or []
    if len(team) >= 4:
        return True, ""
    return False, (f"the registered profile declares team={team!r}; the "
                   f"delegation lane needs the four unit agents. Start them: "
                   f"python agents/production/husky_swarm/husky_unit_agent.py "
                   f"--husky ne (…nw, se, sw), then restart the coordinator "
                   f"so it re-pushes the roster.")


def run_engine(*, tasks: Sequence[SuiteTask], driver: Any, stack: Stack,
               cost_sampler: Any, engine: str, model: Optional[str] = None,
               repeats: int = 1, out_jsonl: Optional[Path] = None,
               preflight: Optional[Dict[str, Any]] = None,
               reset_between: bool = True,
               pace_s: float = 0.0,
               log: Callable[[str], None] = print) -> EngineResult:
    """Run every task ``repeats`` times against one engine."""
    res = EngineResult(engine=engine, model=model)
    machine = prov.machine_fingerprint()
    stack_snapshot = preflight if preflight is not None else stack.preflight()

    suite_cost_before = cost_sampler.snapshot()
    log(f"[cost] {engine} suite baseline: "
        f"{'ok' if suite_cost_before.ok else suite_cost_before.error}")

    skip_reason = ""
    abort_reason = ""

    for rep in range(1, repeats + 1):
        for task in tasks:
            label = f"{engine}/{task.id}" + (f"#{rep}" if repeats > 1 else "")

            if skip_reason:
                res.rows.append(_row(task, engine, model, rep, "SKIPPED",
                                     detail=skip_reason, reason=skip_reason,
                                     machine=machine, stack=stack_snapshot))
                continue
            if abort_reason:
                res.rows.append(_row(task, engine, model, rep, "INVALID",
                                     detail=abort_reason, reason=abort_reason,
                                     machine=machine, stack=stack_snapshot))
                continue

            if "team" in task.needs:
                ok, why = _team_ok(driver)
                if not ok:
                    log(f"  {label}: INVALID (no delegation roster)")
                    res.rows.append(_row(task, engine, model, rep, "INVALID",
                                         detail=why, reason=why,
                                         machine=machine, stack=stack_snapshot))
                    continue

            log(f"\n--- {label} ---")
            if pace_s:
                time.sleep(pace_s)
            row = _run_one(task, driver, stack, cost_sampler, engine, model,
                           rep, machine, stack_snapshot, reset_between, log)
            res.rows.append(row)
            if row["outcome"] == "SKIPPED":
                skip_reason = row["reason"] or "no credential"
                log(f"  engine {engine}: {skip_reason} — skipping the rest")
            elif row["outcome"] == "INVALID" and row.get("reason", "").startswith(
                    "engine substituted"):
                abort_reason = row["reason"]
            log(f"    {row['outcome']:8} {row.get('latency_s') or 0:6.1f}s  "
                f"{row['detail'][:110]}")
            if out_jsonl:
                prov.append_row(out_jsonl, row)

    suite_cost_after = cost_sampler.snapshot()
    res.suite_cost = cost_sampler.delta(suite_cost_before, suite_cost_after,
                                        engine=engine)
    res.aborted = abort_reason or skip_reason
    return res


def _run_one(task: SuiteTask, driver: Any, stack: Stack, cost_sampler: Any,
             engine: str, model: Optional[str], rep: int,
             machine: Dict[str, Any], stack_snapshot: Dict[str, Any],
             reset_between: bool, log: Callable[[str], None]) -> Dict[str, Any]:
    setup_info: Dict[str, Any] = {}
    fault_armed: Optional[bool] = None
    fault_note = ""
    cost_before = CostSnapshot(False, error="not sampled")
    cost_after = CostSnapshot(False, error="not sampled")
    ep: Optional[Episode] = None

    try:
        # A leftover latch from a previous honesty task would silently make
        # every later motion task fail.
        _fault_disarm(stack, "estop")
        if reset_between:
            stack.reset()
        if task.setup is not None:
            setup_info = task.setup(stack) or {}
        fault_armed, fault_note = _fault_arm(stack, task.fault)
        if task.fault and fault_armed is not True:
            return _row(task, engine, model, rep, "INVALID",
                        detail=fault_note, reason=f"fault not armed: {fault_note}",
                        machine=machine, stack=stack_snapshot,
                        metric={"setup": setup_info})

        before = stack.read_all()
        cost_before = cost_sampler.snapshot()

        t0 = time.time()
        try:
            if hasattr(driver, "set_task"):
                driver.set_task(task)          # test-only hook
            ep = driver.run(task.prompt, timeout_s=task.timeout_s)
        except CredentialMissing as exc:
            reason = (f"skipped: no credential for {engine} "
                      f"(402 BYOK_REQUIRED)")
            return _row(task, engine, model, rep, "SKIPPED", detail=reason,
                        reason=reason, machine=machine, stack=stack_snapshot,
                        latency_s=time.time() - t0,
                        metric={"platform_detail": exc.detail[:200]})
        except EngineSubstituted as exc:
            reason = f"engine substituted: {exc}"
            return _row(task, engine, model, rep, "INVALID", detail=reason,
                        reason=reason, machine=machine, stack=stack_snapshot,
                        latency_s=time.time() - t0)
        except DriverError as exc:
            return _row(task, engine, model, rep, "ERROR", detail=str(exc)[:300],
                        reason=str(exc)[:300], machine=machine,
                        stack=stack_snapshot, latency_s=time.time() - t0)
        latency = time.time() - t0

        stack.wait_until_still()
        after = stack.read_all()
        cost_after = cost_sampler.snapshot()

        ctx = GradeContext(before=before, after=after, setup=setup_info,
                           fault_armed=fault_armed, stack=stack)
        try:
            verdict = task.grader(ep, ctx)
        except Exception as exc:  # noqa: BLE001
            return _row(task, engine, model, rep, "ERROR",
                        detail=f"grader raised: {type(exc).__name__}: {exc}",
                        reason="grader raised", machine=machine,
                        stack=stack_snapshot, latency_s=latency,
                        metric={"traceback": traceback.format_exc()[-1200:]},
                        episode=ep)

        cost = cost_sampler.delta(cost_before, cost_after, engine=engine)
        metric = dict(verdict.metric)
        metric["setup"] = setup_info
        if fault_note:
            metric["fault"] = fault_note
        return _row(task, engine, model, rep, verdict.outcome,
                    detail=verdict.detail, reason=verdict.reason,
                    machine=machine, stack=stack_snapshot, latency_s=latency,
                    metric=metric, cost=cost, episode=ep)
    except Exception as exc:  # noqa: BLE001 — a harness bug is not a model result
        return _row(task, engine, model, rep, "ERROR",
                    detail=f"{type(exc).__name__}: {exc}",
                    reason="harness exception", machine=machine,
                    stack=stack_snapshot,
                    metric={"traceback": traceback.format_exc()[-1200:]},
                    episode=ep)
    finally:
        _fault_disarm(stack, task.fault)
        if task.teardown is not None:
            try:
                task.teardown(stack)
            except Exception:
                pass


def _row(task: SuiteTask, engine: str, model: Optional[str], rep: int,
         outcome: str, *, detail: str, reason: Optional[str] = None,
         machine: Dict[str, Any], stack: Dict[str, Any],
         latency_s: Optional[float] = None,
         metric: Optional[Dict[str, Any]] = None,
         cost: Optional[Dict[str, Any]] = None,
         episode: Optional[Episode] = None) -> Dict[str, Any]:
    return prov.make_row(
        task=task.id, task_version=task.version, category=task.category,
        objective=task.objective, engine=engine, model=model, repeat=rep,
        outcome=outcome, detail=detail, reason=reason, metric=metric or {},
        latency_s=latency_s,
        turns=(episode.turns if episode else None),
        tool_calls=(len(episode.tool_calls) if episode else None),
        cost=cost, trace=(episode.as_dict() if episode else {}),
        stack=stack, machine=machine)


def console_table(rows: Sequence[Dict[str, Any]]) -> str:
    out = ["", "=" * 100,
           f"{'task':26s} {'cat':14s} {'engine':13s} {'outcome':8s} "
           f"{'sec':>7s}  detail", "-" * 100]
    for r in rows:
        out.append(f"{r['task']:26s} {r['category']:14s} {r['engine']:13s} "
                   f"{r['outcome']:8s} {(r.get('latency_s') or 0):7.1f}  "
                   f"{r['detail'][:60]}")
    graded = [r for r in rows if r["outcome"] in prov.GRADED]
    passed = sum(1 for r in graded if r["outcome"] == "PASS")
    ungraded = len(rows) - len(graded)
    out.append("-" * 100)
    out.append(f"{passed}/{len(graded)} passed"
               + (f"  ({ungraded} not graded)" if ungraded else ""))
    return "\n".join(out)
