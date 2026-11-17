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
"""python -m omnisim ops-bench: validate, run (calibration or live), regrade."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from omnisim.paths import REPO_ROOT
from .agents import ARMS
from .accounting import episode_cost, request_cost
from .runner import run_task
from .suite import grade, load_suite

DEFAULT_SUITE = REPO_ROOT / "tests/benchmarks/robot_ops/suites/development.json"
SOURCES = ["omnisim/ops_bench/__init__.py", "omnisim/ops_bench/suite.py",
           "omnisim/ops_bench/session.py", "omnisim/ops_bench/agents.py",
           "omnisim/ops_bench/runner.py", "omnisim/ops_bench/cli.py",
           "omnisim/ops_bench/competitors.py", "omnisim/ops_bench/accounting.py",
           "omnisim/ops_bench/health.py", "omnisim/ops_bench/judge.py",
           "omnisim/ops_bench/codex_agent.py", "omnisim/ops_bench/claude_agent.py",
           "omnisim/ops_bench/claude_mcp_stdio.py"]


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _git(*args):
    try:
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True,
                              text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return None


def _competitor_stack():
    """What the competitor arms ran on, so a reproduction can match it."""
    out = {}
    try:
        from importlib.metadata import version
        for pkg in ("langgraph", "langchain-core", "requests"):
            try:
                out[pkg] = version(pkg)
            except Exception:
                out[pkg] = None
    except Exception:
        pass
    lob = REPO_ROOT / "tests/benchmarks/harness_comparison/vendor/lobster"
    try:
        out["lobster_commit"] = subprocess.run(["git", "-C", str(lob), "rev-parse", "HEAD"],
                                               capture_output=True, text=True, timeout=20).stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        out["lobster_commit"] = None
    try:
        out["node"] = subprocess.run(["node", "--version"], capture_output=True, text=True,
                                     timeout=20).stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        out["node"] = None
    out["python"] = sys.version.split()[0]
    return out


def run(suite_path, arms, out, key, only=None, repeats=1, model_cfg=None,
        cap_usd=None, rates=None, health_median_s=None, health_max_wait_h=12.0,
        parallel=1, repeat_base=0):
    if {"codex_full", "claude_full"} & set(arms) and (rates is not None or cap_usd is not None):
        raise ValueError("Signed-in billing is unavailable; --rates/--cap-usd cannot cover this arm")
    suite = load_suite(suite_path)
    tasks = [t for t in suite["tasks"] if not only or t["id"] in only or t["family"] in only]
    if not tasks: raise ValueError("No task selected")
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = {"schema": "omnisim-ops-bench-run/1", "suite": str(Path(suite_path)),
                "suite_sha256": _sha(suite_path), "split": suite["split"], "arms": arms,
                "tasks": [t["id"] for t in tasks], "repeats": repeats,
                "repeat_base": repeat_base,
                "calibration": all(a.startswith("oracle") for a in arms),
                "git_head": _git("rev-parse", "HEAD"),
                "dirty_sources": _git("status", "--porcelain", "--", *SOURCES,
                                      "packages/omnisim-bridges/src",
                                      "projects/samples/demos/controllers/omnilink_mobile_bridge"),
                "sources": {s: _sha(REPO_ROOT / s) for s in SOURCES},
                "model": {k: v for k, v in (model_cfg or {}).items() if k != "key"},
                "cap_usd": cap_usd, "rates_per_million": rates,
                "health_gate": ({"median_s": health_median_s, "max_wait_h": health_max_wait_h,
                                 "probe": "health.probe, 5 requests, before every task block"}
                                if health_median_s else None),
                "parallel": parallel,
                "competitor_stack": _competitor_stack(),
                "started_utc": datetime.now(timezone.utc).isoformat()}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    rows = out / "rows.jsonl"; rows.touch()
    n = 0
    spent, worst, biggest_episode, stopped = 0.0, 0.0, 0.0, None
    # repeat_base: this process runs repeats base..base+repeats-1. Several
    # processes, one per repeat, keep a long shift at real time: twenty
    # episodes in ONE process starved its pose samplers (2.5 s gaps, 0.7x
    # real time) while CPU sat at 26% (shift-calibration-holdout-01).
    for rep in range(repeat_base, repeat_base + repeats):
        if stopped:
            break
        for ti, task in enumerate(tasks):
            # Rotate the arm order per task (and per repeat), so no arm always
            # runs first on a fresh machine (PREREGISTRATION.md).
            k = (ti + rep) % len(arms)
            if stopped:
                break
            if health_median_s and (model_cfg or {}).get("engine"):
                # The gate runs before a whole task block, never inside one,
                # so every arm of a task meets the same provider window.
                from .health import wait_until_healthy
                ok, usd = wait_until_healthy(
                    key, model_cfg["engine"], model_cfg.get("model", ""), health_median_s,
                    out / "health.jsonl", health_max_wait_h * 3600,
                    cost=(lambda reqs: sum(request_cost(r.get("usage"), rates) or 0.0
                                           for r in reqs)) if rates else None)
                spent += usd
                if not ok:
                    stopped = {"reason": "provider unhealthy", "spent_usd": round(spent, 4),
                               "before": f"{task['id']} repeat {rep}"}
                    print("STOPPED: the provider stayed unhealthy past the wait limit", flush=True)
                    break
            block = [a for a in arms[k:] + arms[:k]
                     if a != "oracle_bad" or any("calibration_bad" in s for s in task["steps"])]
            # `parallel` episodes of the block start together (default 1:
            # strictly sequential). Arms of one wave meet the same provider
            # window and the same machine load at the same moment.
            for w0 in range(0, len(block), max(1, parallel)):
                wave = block[w0:w0 + max(1, parallel)]
                if cap_usd is not None and spent + len(wave) * max(0.05, biggest_episode) > cap_usd:
                    stopped = {"reason": "spend cap", "spent_usd": round(spent, 4),
                               "before": f"{wave[0]} {task['id']} repeat {rep}"}
                    print(f"STOPPED on the spend cap: ${spent:.4f} spent of ${cap_usd}", flush=True)
                    break
                dirs = [out / "episodes" / f"{n + i:04d}_{arm}_{task['id']}" for i, arm in enumerate(wave)]
                if len(wave) == 1:
                    recs = [run_task(task, wave[0], key, dirs[0], model_cfg)]
                else:
                    from concurrent.futures import ThreadPoolExecutor
                    with ThreadPoolExecutor(max_workers=len(wave)) as pool:
                        recs = list(pool.map(lambda ad: run_task(task, ad[0], key, ad[1], model_cfg),
                                             zip(wave, dirs)))
                for arm, rec in zip(wave, recs):
                    rec["repeat"] = rep
                    rec["wave_size"] = len(wave)
                    if rates:
                        usd, nreq, unk, worst = episode_cost(rec, rates, worst)
                        rec["cost"] = {"usd_estimated": round(usd, 6), "requests": nreq,
                                       "unknown_usage_requests": unk}
                        spent += usd
                        biggest_episode = max(biggest_episode, usd)
                    with rows.open("a", encoding="utf-8") as f:
                        f.write(json.dumps(rec, ensure_ascii=False) + "\n"); f.flush(); os.fsync(f.fileno())
                    n += 1
                    u = rec.get("relay_usage", {})
                    cost_label = ("Codex billed cost unavailable" if arm == "codex_full" else
                        "Claude billed cost unavailable" if arm == "claude_full" else
                        f"${(rec.get('cost') or {}).get('usd_estimated', 0):.4f} total ${spent:.4f}")
                    print(f"{n} {arm:10s} {task['id']:36s} {rec['outcome']:5s} "
                          f"unsafe={rec.get('unsafe')} rounds={u.get('rounds')} "
                          f"{cost_label} "
                          f"{';'.join(rec.get('reasons', []))[:150]} {rec.get('error', '')[:200]}", flush=True)
                if stopped:
                    break
    summary = summarise(out)
    summary["spent_usd_estimated"] = (None if {"codex_full", "claude_full"} & set(arms)
                                      else round(spent, 6))
    summary["stopped"] = stopped
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary["by_arm"], indent=2))
    return summary


def summarise(out):
    rows = [json.loads(l) for l in (Path(out) / "rows.jsonl").read_text(encoding="utf-8").splitlines() if l]
    by_arm, by_family = {}, {}
    for r in rows:
        a = by_arm.setdefault(r["arm"], {"PASS": 0, "FAIL": 0, "ERROR": 0, "unsafe": 0, "model_rounds": 0,
                                         "requests": 0, "usd_estimated": 0.0})
        a[r["outcome"]] += 1; a["unsafe"] += bool(r.get("unsafe"))
        if r["arm"] == "codex_full":
            a["usd_estimated"] = None
            a["billing"] = "signed-in Codex account; billed cost unavailable"
            a["model_rounds"] += (r.get("codex_usage") or {}).get("model_requests", 0)
            a["requests"] = None
        elif r["arm"] == "claude_full":
            a["usd_estimated"] = None
            a["billing"] = "signed-in Claude account; billed cost unavailable"
            a["model_rounds"] += (r.get("claude_usage") or {}).get("model_requests", 0)
            a["requests"] = None
        else:
            a["requests"] += (r.get("cost") or {}).get("requests", 0)
            a["usd_estimated"] = round(a["usd_estimated"] + (r.get("cost") or {}).get("usd_estimated", 0.0), 6)
            a["model_rounds"] += (r.get("relay_usage") or {}).get("rounds", 0)
        f = by_family.setdefault(r["arm"], {}).setdefault(r["family"], {"PASS": 0, "FAIL": 0, "ERROR": 0})
        f[r["outcome"]] += 1
    return {"episodes": len(rows), "by_arm": by_arm, "by_family": by_family}


def final_grades(out, suite_path):
    """Grade every row again WITH the judge's verdicts and write graded.jsonl:
    the scores a campaign is analysed on."""
    from .judge import load_judgments
    from .runner import load_trace
    suite = {t["id"]: t for t in load_suite(suite_path)["tasks"]}
    judged = load_judgments(out)
    lines = []
    for line in (Path(out) / "rows.jsonl").read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        rec = {"episode_dir": r.get("episode_dir"), "arm": r["arm"], "repeat": r.get("repeat"),
               "task": r["task"]}
        if r.get("outcome") == "ERROR" or "trace" not in r:
            rec.update(outcome=r.get("outcome"), reasons=r.get("reasons", []), unsafe=r.get("unsafe"),
                       checks=r.get("checks", {}))
        else:
            o, reasons, unsafe, detail = grade(suite[r["task"]], load_trace(r, out), r["fired"],
                                               r["replies"], r["pose0"],
                                               judged.get(r.get("episode_dir"), {}))
            rec.update(outcome=o, reasons=reasons, unsafe=unsafe, checks=detail)
        lines.append(json.dumps(rec, ensure_ascii=False))
    (Path(out) / "graded.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(lines)


def regrade(out, suite_path):
    """Recompute every grade from saved traces: no simulator, no model."""
    suite = {t["id"]: t for t in load_suite(suite_path)["tasks"]}
    mismatches = []
    for line in (Path(out) / "rows.jsonl").read_text(encoding="utf-8").splitlines():
        r = json.loads(line)
        if "trace" not in r: continue
        from .runner import load_trace
        o, reasons, unsafe, _ = grade(suite[r["task"]], load_trace(r, out), r["fired"], r["replies"],
                                      r["pose0"])
        if (o, reasons, unsafe) != (r["outcome"], r["reasons"], r["unsafe"]):
            mismatches.append({"task": r["task"], "arm": r["arm"], "saved": r["outcome"], "regraded": o})
    return mismatches


def main(argv=None):
    # Robot replies carry characters a cp1252 console cannot encode ("−", "…");
    # printing one crashed a crew take outright on 2026-10-20.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    v = sub.add_parser("validate"); v.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    r = sub.add_parser("run", help="Run arms over the suite in owned realtime engines")
    r.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    r.add_argument("--arms", nargs="+", choices=ARMS, required=True)
    r.add_argument("--out", type=Path, required=True)
    r.add_argument("--key-file", type=Path)
    r.add_argument("--only", nargs="*", help="Task ids or family names")
    r.add_argument("--repeats", type=int, default=1)
    r.add_argument("--repeat-base", type=int, default=0,
                   help="Number this process's repeats from here (one process per repeat)")
    r.add_argument("--engine", default="", help="OmniLink engine for EVERY model-using arm (e.g. g3-engine)")
    r.add_argument("--model", default="", help="Model id for every arm; empty = the engine's default")
    r.add_argument("--codex-model", default="gpt-6.1-sol", help="Exact model for the native Codex arm")
    r.add_argument("--codex-reasoning", default="high", choices=("low", "medium", "high", "xhigh", "max"))
    r.add_argument("--codex-workspace", type=Path,
                   help="Empty workspace root outside this repository, required for Codex")
    r.add_argument("--claude-model", default="claude-opus-5-5", help="Exact model for the native Claude Code arm")
    r.add_argument("--claude-effort", default="high", choices=("low", "medium", "high", "xhigh", "max"))
    r.add_argument("--claude-binary", default="", help="Claude Code executable; default: claude on PATH")
    r.add_argument("--claude-workspace", type=Path,
                   help="Empty workspace root outside this repository, required for Claude")
    r.add_argument("--cap-usd", type=float, default=None,
                   help="Stop before an episode that could take estimated spend past this")
    r.add_argument("--rates", default="",
                   help="USD per million tokens: input,cached,output (output includes thinking)")
    r.add_argument("--max-requests", type=int, default=60,
                   help="Per-episode cap on a competitor arm's model requests")
    r.add_argument("--health-median-s", type=float, default=None,
                   help="Before each task block, probe the model and wait until its median "
                        "latency is at most this many seconds")
    r.add_argument("--health-max-wait-h", type=float, default=12.0,
                   help="Stop the run if the provider stays unhealthy this long")
    r.add_argument("--parallel", type=int, default=1,
                   help="Episodes of one task block run concurrently, in waves of this size")
    r.add_argument("--sim-clock", action="store_true",
                   help="DEV LOOP ONLY, OmniLink arm only: run the world in fast mode on simulator "
                        "time, at real time while OmniLink is busy (omnisim_bridges.runtime_clock). "
                        "Never for a published result")
    r.add_argument("--sim-clock-speed", type=float, default=6.0,
                   help="Cap on simulated seconds per wall second while OmniLink is idle")
    r.add_argument("--replay-from", type=Path, default=None,
                   help="DEV LOOP ONLY: answer OmniLink's model calls from the replies recorded in an "
                        "earlier run's episodes when the conversation matches (omnisim_bridges.model_replay)")
    j = sub.add_parser("judge", help="Judge every `judged` check, then write graded.jsonl")
    j.add_argument("directories", nargs="+", type=Path)
    j.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    j.add_argument("--engine", default="g1-engine")
    j.add_argument("--model", default="gemini-2.5-pro")
    j.add_argument("--key-file", type=Path)
    g = sub.add_parser("regrade"); g.add_argument("directory", type=Path)
    g.add_argument("--suite", type=Path, default=DEFAULT_SUITE)
    args = p.parse_args(argv)
    try:
        if args.command == "validate":
            s = load_suite(args.suite)
            fams = sorted({t["family"] for t in s["tasks"]})
            print(json.dumps({"tasks": len(s["tasks"]), "families": fams, "split": s["split"]}))
            return 0
        if args.command == "run":
            key = (args.key_file.read_text().strip() if args.key_file
                   else os.environ.get("OMNI_KEY", "").strip())
            if not key: raise ValueError("Pass --key-file or set OMNI_KEY")
            cfg = {"engine": args.engine, "model": args.model, "max_requests": args.max_requests}
            if args.sim_clock:
                if set(args.arms) != {"omnilink"}:
                    raise ValueError("--sim-clock is OmniLink-only: competitors run in real time")
                cfg.update(sim_clock=True, sim_clock_speed=args.sim_clock_speed)
            if args.replay_from:
                if set(args.arms) != {"omnilink"}:
                    raise ValueError("--replay-from is OmniLink-only")
                cfg["replay_from"] = str(args.replay_from.resolve())
            if "codex_full" in args.arms:
                if not args.codex_workspace:
                    raise ValueError("Codex needs --codex-workspace outside the benchmark repository")
                workspace = args.codex_workspace.resolve()
                if workspace == REPO_ROOT or REPO_ROOT in workspace.parents:
                    raise ValueError("Codex workspace must be outside the benchmark repository")
                cfg.update(codex_model=args.codex_model, codex_reasoning=args.codex_reasoning,
                           codex_workspace=str(workspace))
            if "claude_full" in args.arms:
                if not args.claude_workspace:
                    raise ValueError("Claude needs --claude-workspace outside the benchmark repository")
                workspace = args.claude_workspace.resolve()
                if workspace == REPO_ROOT or REPO_ROOT in workspace.parents:
                    raise ValueError("Claude workspace must be outside the benchmark repository")
                cfg.update(claude_model=args.claude_model, claude_effort=args.claude_effort,
                           claude_binary=args.claude_binary, claude_workspace=str(workspace))
            rates = None
            if args.rates:
                i, c, o = (float(v) for v in args.rates.split(","))
                rates = {"input": i, "cached": c, "output": o}
            if args.cap_usd is not None and rates is None:
                raise ValueError("--cap-usd needs --rates")
            run(args.suite, args.arms, args.out, key, args.only, args.repeats, cfg,
                args.cap_usd, rates, args.health_median_s, args.health_max_wait_h,
                args.parallel, args.repeat_base)
            return 0
        if args.command == "judge":
            from .competitors import ModelClient
            from .judge import judge_run
            key = (args.key_file.read_text().strip() if args.key_file
                   else os.environ.get("OMNI_KEY", "").strip())
            if not key: raise ValueError("Pass --key-file or set OMNI_KEY")
            for d in args.directories:
                client = ModelClient(key, args.engine, args.model, "OmniSim-judge", 5000, log=[])
                n = len(judge_run(d, args.suite, client))
                g = final_grades(d, args.suite)
                print(json.dumps({"directory": str(d), "new_judgments": n, "graded_rows": g}))
            return 0
        mm = regrade(args.directory, args.suite)
        print(json.dumps({"regrade_mismatches": mm}, indent=2))
        return 0 if not mm else 1
    except (ValueError, OSError, KeyError) as exc:
        print(f"ops-bench: {exc}", file=sys.stderr); return 2


if __name__ == "__main__":
    raise SystemExit(main())
