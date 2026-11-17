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

"""AgentBench runner -- Phase 0 (no LLM in the loop).

    python tests/benchmarks/agentbench/run_agentbench.py \\
        --tasks A1 --agent oracle --repeats 1 --out results/phase0

Per (task, agent, repeat) it:

  * builds an isolated run directory and a fresh scratch dir, materialising
    the task's ``initial/`` files into it (SPEC 4.3: "a task cannot leave an
    artifact that helps the next task");
  * picks per-run unique harness ports and a per-run ``OMNISIM_LOG_PATH``,
    because parallel runs otherwise clobber the shared ``omnisim_log.txt``;
  * runs the agent with a ``trace.jsonl`` open;
  * runs the harness pass (where the task asks for one) and the standalone
    recorder pass;
  * grades, and writes one SPEC-schema row with the omnibench machine
    fingerprint;
  * **never lets one task failure take the suite down** -- any exception
    becomes an ``ERROR`` row and the next cell starts.

Phase 0 also ships ``--fake-sim``: exercises the harness, isolation, trace
writer and results schema with no simulator, no GPU and no network (SPEC 7).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from agentbench import agents as agent_registry          # noqa: E402
from agentbench import tasks as task_registry            # noqa: E402
from agentbench.adapters.omnisim import harness as ad_harness  # noqa: E402
from agentbench.adapters.omnisim import headless as ad_headless  # noqa: E402
from agentbench.agents.base import AgentContext, AgentResult  # noqa: E402
from agentbench.common.paths import (                    # noqa: E402
    HUSKY_URDF, REPO, RESULTS_DIR, as_wbt_path, ob_results)
from agentbench.common.trace import Trace                # noqa: E402
from agentbench.common import worldtext                  # noqa: E402
from agentbench.graders.verdict import (                 # noqa: E402
    ERROR, INVALID, Verdict)
from agentbench.runner.isolation import parse_read_deny  # noqa: E402
from agentbench.runner.tools import (                    # noqa: E402
    CONDITION_ALIASES, SPEC_CONDITION_NAME, normalize_condition)

SUITE = "agenticsimbench/v0.3"
SIM = "omnisim"
CONDITION = "scripted"       # Phase 0 has no tool surface to compare
SCAFFOLD = "phase0"

# Campaign rows are COPIED here deliberately (--publish), reviewed, committed.
# results/ stays a gitignored scratch tree; this dir is the "no row, no
# result" mechanism (agent-edge-validation-plan §0.2, Phase R item 3).
RESULTS_PUBLISHED = _HERE / "results_published"


def is_scored_kind(entry) -> bool:
    """SCORED == the row is a measurement, not an expectation check.

    The registry's own convention (agents/__init__.py) is the definition: the
    Phase-0 scripted controls (oracle/null/wrong/parade) carry a bool
    ``expect_pass``; every unknown-outcome agent -- ``llm*``, ``external``, a
    future competitor -- carries ``expect_pass=None`` "by construction". So
    "scored" is derived from the entry rather than a hand-kept name list,
    and a new scripted control cannot silently dodge the guard by being
    forgotten here.
    """
    return entry.get("expect_pass") is None


def resolve_condition(entry, art_condition, cli_condition):
    """The row's ``condition``: agent artifact > --condition > module fallback.

    Returns ``(condition, guard_reason)``. ``guard_reason`` is non-None only
    when a SCORED row would have fallen back to the Phase-0 ``"scripted"``
    constant -- the exact defect that made the first A/B unqueryable (56/58
    rows read ``scripted``; agent-edge-validation-plan §0.2). Such a row gets
    ``condition=None`` and the caller marks it INVALID: SPEC 3.3 lists
    "missing attribution" as INVALID, never as a score.
    """
    if art_condition:
        return art_condition, None
    if cli_condition:
        # Normalised to the SPEC 5 spelling ("shell" / "shell+tools") so rows
        # from `--condition full_surface` and `--condition shell+tools` are
        # queryable as one condition.
        key = normalize_condition(cli_condition)
        return SPEC_CONDITION_NAME.get(key, key), None
    if not is_scored_kind(entry):
        return CONDITION, None
    return None, (
        "scored row has no condition: the agent recorded no "
        "artifacts['condition'] and no --condition was given, and a scored "
        "row may not fall back to the Phase-0 'scripted' constant "
        "(agent-edge-validation-plan §0.2, Phase R item 2)")


def utcnow():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def materialise_scratch(task, scratch: Path):
    """Copy the task's initial/ files in, substituting the URDF path."""
    scratch.mkdir(parents=True, exist_ok=True)
    src = task.initial_dir
    if not src.exists():
        return []
    written = []
    for p in sorted(src.rglob("*")):
        if p.is_dir() or p.name == ".gitkeep":
            continue
        # Harness-injected sibling debris (.harness_*.wbt /
        # ..harness_*.wbproj): gitignored leftovers from loading a task
        # world in place. Never task content -- and copying a stale
        # .wbproj beside the world makes the phase-B engine launch flaky
        # (measured 2026-08-01: 3/3 rc=1 with it, clean without).
        if worldtext.is_harness_sibling(p.name):
            continue
        dst = scratch / p.relative_to(src)
        dst.parent.mkdir(parents=True, exist_ok=True)
        if p.suffix in (".wbt", ".omniworld"):
            dst.write_text(
                p.read_text(encoding="utf-8").replace(
                    task_registry.SUBSTITUTION_TOKEN, as_wbt_path(HUSKY_URDF)),
                encoding="utf-8")
        else:
            shutil.copy2(p, dst)
        written.append(str(dst))
    return written


def run_cell(task, agent_name, repeat, out_dir, *, fake_sim=False,
             quiet=False, condition=None):
    """One (task, agent, repeat). Returns (row, verdict).

    ``condition`` is the --condition CLI value; the agent's own recorded
    condition still wins (see :func:`resolve_condition`).
    """
    cell = "%s.%s.r%d" % (task.id, agent_name, repeat)
    run_dir = Path(out_dir) / cell
    scratch = run_dir / "scratch"
    if run_dir.exists():
        shutil.rmtree(run_dir, ignore_errors=True)
    run_dir.mkdir(parents=True, exist_ok=True)

    t_total0 = time.perf_counter()
    t_agent = None               # stays None if the agent never ran
    trace = Trace(run_dir)
    entry = agent_registry.get(task.id, agent_name)
    verdict = Verdict(task.id)
    result = AgentResult()
    harness_probe = None
    phase_b = None
    deviations = []

    try:
        initial = materialise_scratch(task, scratch)
        trace.event("run_start", task=task.id, agent=agent_name,
                    repeat=repeat, prompt=task.prompt,
                    initial_files=initial, par_s=task.par_s,
                    timeout_s=task.timeout_s)

        ctx = AgentContext(task_id=task.id, prompt=task.prompt,
                           scratch_dir=scratch, run_dir=run_dir, repo=REPO,
                           trace=trace,
                           deadline_s=time.time() + (task.timeout_s or 0),
                           seed=repeat, fake_sim=fake_sim)

        t_agent0 = time.perf_counter()
        result = entry["fn"](ctx) or AgentResult()
        t_agent = time.perf_counter() - t_agent0
        trace.event("agent_done", t_agent_s=t_agent,
                    final=result.final_message)

        artifact = worldtext.pick_artifact(scratch)
        candidates = [str(p) for p in worldtext.find_world_artifacts(scratch)]
        trace.event("artifact_discovery", picked=str(artifact) if artifact
                    else None, candidates=candidates)

        # Could the artifact's own project supply the grader's recorder? The
        # engine searches <project>/controllers/ before the grader's
        # WEBOTS_EXTRA_PROJECT_PATH, so if it can, every "measured" number
        # this cell would report is the agent's to author. Detected once,
        # here, so BOTH phases are refused -- not just the standalone one.
        tamper = (ad_headless.controller_shadow_check(artifact)
                  if artifact is not None else [])
        if tamper:
            trace.event("tamper_detected", hits=tamper)

        # An agent that could not RUN (no credential, backend refused) is not
        # a failing agent -- grading it FAIL would put a zero in the results
        # that looks like a capability finding. SPEC 3.3: report SKIPPED.
        _err = (getattr(result, "error", None) or "")
        if _err and ("cannot run the" in _err or "credential" in _err.lower()):
            verdict = Verdict(task.id)
            verdict.note("agent could not start, so nothing was measured: %s"
                         % _err)
            verdict.outcome = "SKIPPED"
        elif fake_sim:
            verdict = Verdict(task.id)
            verdict.note("--fake-sim: no simulator was run; this row exercises "
                         "isolation, the trace writer and the results schema "
                         "only.")
            verdict.outcome = "SKIPPED"
            verdict.artifacts["world"] = str(artifact) if artifact else None
        elif tamper:
            # INVALID, never FAIL: nothing was measured, so nothing about the
            # artifact was established either way. A FAIL here would be a
            # capability claim we did not earn, and a PASS would be the agent
            # grading itself.
            verdict = Verdict(task.id)
            verdict.outcome = INVALID
            verdict.note(ad_headless.describe_tamper(tamper))
            verdict.artifacts["world"] = str(artifact) if artifact else None
            verdict.artifacts["tamper"] = tamper
            deviations.append(ad_headless.describe_tamper(tamper))
        else:
            # ---- Phase A: the live harness pass ------------------------
            if task.wants_harness and artifact is not None:
                trace.event("phase_a_start", world=str(artifact))
                harness_probe = ad_harness.probe_world(
                    artifact, run_dir / "harness")
                trace.event("phase_a_done", **harness_probe.as_dict())
                if not harness_probe.ok:
                    deviations.append("harness pass failed: %s"
                                      % harness_probe.error)

            # ---- Phase B: standalone, cold, grader-owned recorder ------
            if artifact is not None:
                sa = task.standalone
                trace.event("phase_b_start", world=str(artifact), **sa)
                phase_b = ad_headless.run_standalone(
                    artifact, run_dir / "phaseB",
                    duration=float(sa.get("duration_s", 30.0)),
                    settle=float(sa.get("settle_s", 1.0)),
                    contact_steps=int(sa.get("contact_steps", 10)),
                    solids=tuple(sa.get("solids", ())),
                    # Both OPT-IN and both absent from every already-frozen
                    # task meta, so nothing that ran before this key existed
                    # changes shape. ``links`` is the per-robot cap on link
                    # tracks (R2 needs them to see an end effector at all);
                    # ``solid_tracks`` decides which named Solids also get a
                    # pose series (default: the ones with a mass model).
                    links=int(sa.get("links", 0)),
                    solid_tracks=sa.get("solid_tracks"),
                    # The cap on the NAME-FREE t=0 scene scan. Absent from
                    # every frozen task meta, so the recorder's own default
                    # applies and the injected stanza is byte-identical; a
                    # task that wants a bigger or smaller budget (or none)
                    # says so and gets the arg emitted.
                    scan_solids=sa.get("scan_solids"),
                    # The cadence of the RUN-LONG contact watch. Absent from
                    # every frozen task meta, so the recorder's own default
                    # (ON) applies and the injected stanza is byte-identical;
                    # a task that wants it off or finer says so. It is ON by
                    # default because the alternative is a collision assertion
                    # that reports 0 whatever happened -- measured: R1.5
                    # passed a rover that finished outside its own walled
                    # arena.
                    run_contacts=sa.get("run_contacts"),
                    timeout_s=float(task.timeout_s or 1800),
                    tag="phaseB")
                trace.event("phase_b_done", **phase_b.as_dict())

            if phase_b is not None and phase_b.attempts_used > 1:
                deviations.append(
                    "standalone launch retried %d times (back-to-back-launch "
                    "flake)" % (phase_b.attempts_used - 1))

            grader = task.grader()
            verdict = grader.grade(
                run_dir, artifact=artifact, harness=harness_probe,
                phase_b=phase_b, answer=result.final_message,
                self_verified=result.self_verified, scratch_dir=scratch)

            # SPEC 3.3: a sim crash unrelated to the agent's edit is INVALID,
            # not FAIL. "The stack broke" is the ONLY thing that earns this
            # override, and ad_headless.stack_broke decides it from the
            # RECORDER'S OWN completion flag -- never from the row count. A
            # robot-free world makes the recorder write a header-only CSV and
            # quit cleanly; that is a legitimate FAIL, and calling it INVALID
            # turned an agent's empty world into "our harness broke".
            if ad_headless.stack_broke(phase_b):
                verdict.outcome = INVALID
                verdict.note(
                    "standalone run produced no samples after %d attempts and "
                    "the engine logged no ERROR: line -- the stack broke, the "
                    "artifact was not disproved."
                    % phase_b.attempts_used)
            elif (phase_b is not None and phase_b.xyz is None
                    and ad_headless.recorder_finished(phase_b)):
                verdict.note(
                    "the recorder ran to completion and found nothing to "
                    "record: the world loaded and stepped but contains no "
                    "trackable robot. That is a FAIL of the artifact, not a "
                    "broken harness.")

        verdict.self_verified = result.self_verified
        verdict.artifacts.setdefault("trace", str(trace.path))
        if phase_b is not None:
            verdict.artifacts.setdefault(
                "engine_log", str(phase_b.log_path))
            verdict.artifacts.setdefault(
                "recorder_csv", str(run_dir / "phaseB" / "phaseB.csv"))
    except Exception:                                     # noqa: BLE001
        verdict = Verdict(task.id)
        verdict.outcome = ERROR
        verdict.note("harness exception:\n" + traceback.format_exc())
        trace.event("harness_error", traceback=traceback.format_exc())
    finally:
        trace.close()

    t_total = time.perf_counter() - t_total0

    # ---- provenance + the row guard ------------------------------------
    # Agent-supplied provenance (empty for the scripted Phase-0 agents).
    _art = dict(getattr(result, "artifacts", None) or {})
    row_condition, cond_guard = resolve_condition(
        entry, _art.get("condition"), condition)
    if cond_guard is not None:
        deviations.append(cond_guard)
        if verdict.outcome in ("PASS", "FAIL"):
            # SPEC 3.3: missing attribution is INVALID, not a score. Done
            # BEFORE the expectation check so `got_pass` and the outcome
            # cannot disagree in the same row.
            verdict.outcome = INVALID
            verdict.note("condition unattributed -- %s. The verdict above was "
                         "measured but may not enter a pass rate." % cond_guard)

    # ---- expectation check (the Phase-0 point) -------------------------
    exp = {"expect_pass": entry.get("expect_pass"),
           "expect_failures": sorted(entry.get("expect_failures") or [])
           or None}
    got_pass = verdict.outcome == "PASS"
    exp["got_pass"] = got_pass
    exp["failed_assertions"] = verdict.failed
    # expect_pass=None means "no prior expectation" -- Phase 1 agents (an LLM,
    # a human, a competitor) have an UNKNOWN outcome by definition, and that is
    # the whole point of running them. Scoring them against a bool would report
    # every genuine measurement as a MISMATCH.
    if entry.get("expect_pass") is None:
        exp["pass_matches"] = None
    else:
        exp["pass_matches"] = (got_pass == bool(entry.get("expect_pass")))
    if entry.get("expect_failures") is not None:
        exp["failures_match"] = (set(verdict.failed)
                                 == set(entry["expect_failures"]))
    else:
        exp["failures_match"] = None
    exp["ok"] = bool(exp["pass_matches"] in (None, True)
                     and exp["failures_match"] in (None, True))
    if fake_sim:
        # Nothing was graded, so there is nothing to expect. Saying "OK" here
        # would be the benchmark lying to itself.
        exp = {"skipped": "--fake-sim: no verdict was produced", "ok": None}

    machine = ob_results.machine_fingerprint(
        engine_log_path=(phase_b.log_path if phase_b is not None else None))

    vd = verdict.as_dict()

    # SPEC 3.1: t_agent_s is the agent's OWN clock (the runner ledger's
    # agent_wall_s, stashed in artifacts by agents/llm.py), t_total_s the full
    # cell including sim phases + grading. Fallback is the wall this harness
    # measured around fn(ctx); None -- never a fabricated copy of t_total --
    # when the agent never ran. The two were written as the same number until
    # Phase R item 2 (agent-edge-validation-plan §0.2).
    _aw = _art.get("agent_wall_s")
    t_agent_s = (round(float(_aw), 3) if isinstance(_aw, (int, float))
                 else (round(t_agent, 3) if t_agent is not None else None))
    # tokens_cache_write: accounted by the runner Ledger, reaches the row via
    # LoopResult.artifacts (AgentResult has no field for it). Unmeasured is
    # None, never 0 (SPEC 3.1).
    _tcw = getattr(result, "tokens_cache_write", None)
    if _tcw is None:
        _tcw = _art.get("tokens_cache_write")

    row = {
        "suite": SUITE,
        "task": task.id, "tier": task.tier,
        # Provenance comes from the AGENT when it knows better than these
        # module constants. Without this a row cannot say which condition or
        # tool set produced it, which made the first A/B unqueryable: every
        # row read condition="scripted" and the headline survived only in a
        # commit message. See docs/developer/agent-edge-validation-plan.md.
        "sim": _art.get("sim") or SIM,
        "condition": row_condition,
        # The docs-ablation cell attribution (SPEC 6.3), read from the same
        # env var Sandbox.create enforces from, so the recorded and enforced
        # values agree by construction. Empty list == no ablation.
        "read_deny": list(parse_read_deny(
            os.environ.get("AGENTBENCH_READ_DENY") or "")),
        "repeat": repeat,
        "agent": {"model": _art.get("model") or "scripted:%s" % agent_name,
                  "temperature": _art.get("temperature"),
                  "scaffold_sha": SCAFFOLD,
                  "system_prompt_sha": _art.get("system_prompt_sha"),
                  "kind": agent_name,
                  "backend": _art.get("backend"),
                  "credential_source": _art.get("credential_source")},
        "tool_set": {"name": _art.get("tool_set"),
                     "tools_sha256": _art.get("tools_sha256"),
                     "manifest_sha256": _art.get("manifest_sha256")},
        "stop_reason": _art.get("stop_reason"),
        # A21: this key used to be "artifacts", which the LATER "artifacts"
        # (the verdict's) silently overwrote -- Python keeps the last one --
        # so external_label, hit_budget, stop_detail, port_survivors,
        # model_calls, system_prompt and leak_suspect_tool_calls never reached
        # rows.jsonl at all. leak_suspect_tool_calls is the entire recorded
        # mitigation for run_shell being unconfinable (runner/isolation.py
        # promises "a reviewer can grep one field"), so losing it made an
        # isolation claim unverifiable. Distinct key, both survive.
        "agent_artifacts": dict(_art),
        "outcome": vd["outcome"], "progress": vd["progress"],
        "progress_name": vd["progress_name"],
        "self_verified": vd["self_verified"],
        "assertions": {k: a["ok"] for k, a in vd["assertions"].items()},
        "assertion_detail": vd["assertions"],
        "failed_assertion": vd["failed_assertion"],
        "expectation": exp,
        "metrics": {"t_agent_s": t_agent_s,
                    "t_total_s": round(t_total, 3), "turns": result.turns,
                    "tool_calls": result.tool_calls,
                    "tokens_in": result.tokens_in,
                    "tokens_out": result.tokens_out,
                    "tokens_cache_read": result.tokens_cache_read,
                    "tokens_cache_write": _tcw,
                    "usd": result.usd},
        "measurements": vd["measurements"],
        "par_s": task.par_s, "timeout_s": task.timeout_s,
        "interventions": 0,
        "sim_version": _sim_version(phase_b, verdict),
        "image_digest": None,
        "artifacts": vd["artifacts"],
        "deviations": deviations + list(result.deviations),
        "notes": vd["notes"],
        "machine": machine,
        "utc": utcnow(),
    }
    (run_dir / "verdict.json").write_text(json.dumps(vd, indent=2,
                                                     default=str),
                                          encoding="utf-8")
    if not quiet:
        print(verdict.summary())
        # .get(): the --fake-sim exp dict has neither key, and ok=None is
        # "nothing to expect", not a MISMATCH. (Pre-Phase-R this line
        # KeyError'd, turning every non-quiet --fake-sim cell into ERROR.)
        print("  expectation: %s (pass_matches=%s failures_match=%s)"
              % ("-" if exp.get("ok") is None
                 else ("OK" if exp["ok"] else "MISMATCH"),
                 exp.get("pass_matches"), exp.get("failures_match")))
    return row, verdict


def publish_run(run_dir, dest_root=None):
    """Copy a finished run's rows into the tracked ``results_published/`` tree.

    The "no row, no result" mechanism (agent-edge-validation-plan §0.2, Phase R
    item 3): ``results/`` is gitignored scratch, so a campaign's rows die on
    the machine that produced them unless they are COPIED here, reviewed and
    committed. Publishing is a deliberate act -- nothing in the runner calls
    this -- and it refuses to overwrite an existing publication rather than
    silently replacing reviewed rows. Returns the destination directory.
    """
    src = Path(run_dir)
    rows = src / "rows.jsonl"
    if not rows.is_file():
        raise FileNotFoundError(
            "%s has no rows.jsonl -- only a finished run directory (the dir "
            "run_agentbench.py wrote rows into) can be published" % src)
    dest = Path(dest_root or RESULTS_PUBLISHED) / src.name
    if dest.exists():
        raise FileExistsError(
            "%s already exists -- published rows are reviewed artifacts; "
            "delete the old publication explicitly if you mean to replace it"
            % dest)
    data = rows.read_bytes()
    n_rows = sum(1 for line in data.splitlines() if line.strip())
    dest.mkdir(parents=True)
    shutil.copy2(rows, dest / "rows.jsonl")
    # Provenance of the copy itself. The rows carry their own machine/utc;
    # this records only what publishing added: where from, when, and a hash a
    # reviewer can check against the committed file.
    meta = {"run_id": src.name, "source": str(src), "n_rows": n_rows,
            "rows_sha256": hashlib.sha256(data).hexdigest(),
            "published_utc": utcnow()}
    (dest / "publish_meta.json").write_text(json.dumps(meta, indent=2),
                                            encoding="utf-8")
    return dest


def _sim_version(phase_b, verdict):
    a = verdict.attribution or {}
    out = {"name": "omnisim", "backend": a.get("backend"),
           "solver": a.get("solver"), "degraded": a.get("degraded")}
    if phase_b is not None:
        out["sidecar"] = phase_b.sidecar
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tasks", default="all",
                    help="comma list of task ids or prefixes (A1,B3,C2), or "
                         "'all'")
    ap.add_argument("--agent", default="oracle",
                    help="comma list: oracle,null,wrong,parade, or 'all'")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--out", default=None,
                    help="output dir (default results/<timestamp>)")
    ap.add_argument("--condition", default=None,
                    choices=sorted(CONDITION_ALIASES),
                    help="the ablation condition for this run (SPEC 5). Sets "
                         "AGENTBENCH_CONDITION for the agents and becomes the "
                         "row's condition when the agent records none. "
                         "Precedence: agent artifact > --condition > the "
                         "Phase-0 'scripted' fallback (non-scored rows only; "
                         "a scored row without a condition is INVALID).")
    ap.add_argument("--fake-sim", action="store_true",
                    help="dry run: exercise isolation/trace/schema with no "
                         "simulator")
    ap.add_argument("--publish", metavar="RUN_DIR", default=None,
                    help="copy RUN_DIR's rows.jsonl (+ a provenance meta) "
                         "into results_published/<run_id>/ -- the deliberate "
                         "'no row, no result' step; refuses to overwrite")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args(argv)

    if args.publish:
        try:
            dest = publish_run(args.publish)
        except (FileNotFoundError, FileExistsError) as exc:
            print("publish refused: %s" % exc, file=sys.stderr)
            return 2
        print("published -> %s" % dest)
        return 0

    if args.condition:
        # One knob, both consumers: the LLM agents read AGENTBENCH_CONDITION
        # via RunnerConfig.from_env() (llm_shell/llm_tools still pin theirs),
        # and run_cell uses it as the row fallback.
        os.environ["AGENTBENCH_CONDITION"] = args.condition

    known = task_registry.discover()
    if args.list:
        for tid, t in known.items():
            print("%-26s %-8s par=%ss agents=%s"
                  % (tid, t.tier, t.par_s,
                     ",".join(agent_registry.agents_for(tid))))
        return 0

    if args.tasks == "all":
        chosen = list(known.values())
    else:
        chosen = [task_registry.get(t.strip())
                  for t in args.tasks.split(",") if t.strip()]

    out_dir = Path(args.out) if args.out else (
        RESULTS_DIR / time.strftime("%Y%m%d_%H%M%S"))
    out_dir = out_dir if out_dir.is_absolute() else (Path.cwd() / out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows_path = out_dir / "rows.jsonl"

    rows = []
    for task in chosen:
        if args.agent == "all":
            names = agent_registry.agents_for(task.id)
        else:
            names = [a.strip() for a in args.agent.split(",") if a.strip()]
        for name in names:
            if (task.id, name) not in agent_registry.REGISTRY:
                print("skip: no %r agent for %s" % (name, task.id))
                continue
            for rep in range(args.repeats):
                print("\n=== %s / %s / repeat %d ==="
                      % (task.id, name, rep))
                try:
                    row, _ = run_cell(task, name, rep, out_dir,
                                      fake_sim=args.fake_sim,
                                      condition=args.condition)
                except Exception:                          # noqa: BLE001
                    traceback.print_exc()
                    # The --condition value, never the "scripted" fallback:
                    # an ERROR row is unattributed, not scripted.
                    row = {"suite": SUITE, "task": task.id, "sim": SIM,
                           "condition": (SPEC_CONDITION_NAME.get(
                               normalize_condition(args.condition))
                               if args.condition else None),
                           "agent": {"kind": name}, "repeat": rep,
                           "outcome": ERROR,
                           "notes": [traceback.format_exc()],
                           "utc": utcnow()}
                rows.append(row)
                with open(rows_path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(row, default=str) + "\n")

    print("\n" + "=" * 78)
    print("AgentBench Phase 0 -- %d rows -> %s" % (len(rows), rows_path))
    print("=" * 78)
    print("%-26s %-8s %-9s %-6s %-28s %s"
          % ("task", "agent", "outcome", "n/N", "failed", "expectation"))
    bad = 0
    checked = 0
    for r in rows:
        det = r.get("assertion_detail", {}) or {}
        npass = sum(1 for a in det.values() if a.get("ok"))
        exp = r.get("expectation", {}) or {}
        if exp.get("ok") is not None:
            checked += 1
            if not exp.get("ok"):
                bad += 1
        print("%-26s %-8s %-9s %-6s %-28s %s"
              % (r.get("task"), (r.get("agent") or {}).get("kind"),
                 r.get("outcome"), "%d/%d" % (npass, len(det)),
                 ",".join((r.get("expectation") or {})
                          .get("failed_assertions", []) or []) or "-",
                 "-" if exp.get("ok") is None
                 else ("OK" if exp.get("ok") else "MISMATCH")))
    print("\n%d/%d graded cells met their Phase-0 expectation."
          % (checked - bad, checked))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
