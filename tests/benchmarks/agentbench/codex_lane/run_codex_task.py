"""Run one isolated Codex CLI task and grade the resulting artifact.

This is the Codex counterpart to ``cc_lane/run_cc_cell.py``.  It deliberately
reuses that lane's hardened workspace staging, artifact collection, controller
packaging, grade-time placement, and simulator-neutral graders.  Only the
agent process differs.

The runner follows the official non-interactive interface: ``codex exec`` with
``--json`` for an event stream, ``--ephemeral`` to avoid persisted rollouts,
an explicit ``workspace-write`` sandbox, approvals disabled, user config and
rules ignored, a pinned model, and ``--output-last-message`` for the answer
channel.  The exact argv and raw JSONL are preserved in every cell.

This lane is exploratory until its containment record says ``clean: true``
and the selected task's mechanical publication gates are green.  A current
builder session that has already read the benchmark must never be reported as
an uncontaminated scored cell.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
BENCHMARKS = AGENTBENCH.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench import frontier, readiness, sims, tasks  # noqa: E402
from agentbench.agents import external as external_agent  # noqa: E402
from agentbench.cc_lane import run_cc_cell as shared  # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging  # noqa: E402
from agentbench.common import r1_placement, worldtext  # noqa: E402


CONDITION = "codex_cli"
METRICS_SOURCE = "codex_exec_jsonl"
# Agent-authored worlds and controllers normally live below projects/.  The
# older Claude lane junctions that directory into the source checkout and
# relies on a preserve-and-sweep postscript.  A Codex workspace instead copies
# tracked projects/ content, so an ordinary edit is isolated by construction.
# Only large runtime trees that a build task should never modify remain links.
OMNISIM_RUNTIME_JUNCTIONS = ("msys64", "lib", "resources")
TOOL_ITEM_TYPES = frozenset({
    "command_execution", "file_change", "mcp_tool_call", "web_search",
    "plan_update",
})


def codex_command(executable, *, prompt, workspace, model, answer_path):
    """The documented, auditable ``codex exec`` argv for one task."""
    if not model:
        raise ValueError("a Codex frontier cell requires an explicit model")
    return [
        str(executable), "exec",
        "--ephemeral",
        "--json",
        "--sandbox", "workspace-write",
        "--ask-for-approval", "never",
        "--ignore-user-config",
        "--ignore-rules",
        "--skip-git-repo-check",
        "--cd", str(Path(workspace).resolve()),
        "--model", model,
        "--output-last-message", str(Path(answer_path).resolve()),
        prompt,
    ]


def parse_events(text):
    """Extract measured Codex session facts from a JSONL event stream."""
    events, invalid = [], 0
    for line in (text or "").splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except ValueError:
            invalid += 1
    thread_id = None
    final = None
    usage = {}
    tool_calls = 0
    errors = []
    for event in events:
        etype = event.get("type")
        if etype == "thread.started":
            thread_id = event.get("thread_id") or thread_id
        if etype in ("error", "turn.failed"):
            errors.append(event)
        item = event.get("item") or {}
        if etype == "item.completed":
            if item.get("type") == "agent_message":
                final = item.get("text") or final
            if item.get("type") in TOOL_ITEM_TYPES:
                tool_calls += 1
        if etype == "turn.completed" and isinstance(event.get("usage"), dict):
            usage = event["usage"]
    return {
        "events": len(events),
        "invalid_json_lines": invalid,
        "thread_id": thread_id,
        "final_message": final,
        "tool_calls": tool_calls,
        "tokens_in": usage.get("input_tokens"),
        "tokens_cache_read": usage.get("cached_input_tokens"),
        "tokens_out": usage.get("output_tokens"),
        "reasoning_tokens": usage.get("reasoning_output_tokens"),
        "errors": errors,
        "completed": bool(usage) and not errors,
    }


def _child_env(workspace, sim):
    env = dict(os.environ)
    for key in list(env):
        if (key.startswith("AGENTBENCH_") or key.startswith("OMNISIM_")
                or key in ("OPENAI_API_KEY", "CODEX_API_KEY", "PYTHONPATH",
                           "WEBOTS_HOME", "WEBOTS_PATH")):
            env.pop(key, None)
    if sim == "omnisim":
        env["OMNISIM_HOME"] = str(Path(workspace).resolve())
        env["OMNISIM_NO_WINDOW"] = "1"
    return env


def _terminate(proc):
    if proc.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                       capture_output=True, timeout=30)
    else:  # pragma: no cover - exercised on Linux campaign hosts
        proc.kill()


def codex_identity(executable):
    """Resolve and version the CLI so a campaign cannot drift silently."""
    found = shutil.which(str(executable))
    path = Path(found or executable).resolve()
    try:
        proc = subprocess.run(
            [str(path), "--version"], capture_output=True, text=True,
            encoding="utf-8", errors="replace", timeout=30)
    except OSError as exc:
        raise RuntimeError("Codex CLI is not executable: %s" % exc) from exc
    version = (proc.stdout or proc.stderr or "").strip()
    if proc.returncode != 0 or not version:
        raise RuntimeError("Codex CLI identity check failed (rc=%s): %s" % (
            proc.returncode, version or "no version output"))
    return {"path": str(path), "version": version}


def run_codex(executable, *, prompt, workspace, sim, model, answer_path,
              event_path, stderr_path, timeout_s):
    cmd = codex_command(executable, prompt=prompt, workspace=workspace,
                        model=model, answer_path=answer_path)
    started = time.time()
    try:
        proc = subprocess.Popen(
            cmd, cwd=str(workspace), env=_child_env(workspace, sim),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            encoding="utf-8", errors="replace",
            creationflags=(subprocess.CREATE_NEW_PROCESS_GROUP
                           if os.name == "nt" else 0))
    except OSError as exc:
        return {"argv": cmd, "started": False, "error": repr(exc),
                "wall_s": 0.0, "returncode": None, "timed_out": False}
    timed_out = False
    try:
        stdout, stderr = proc.communicate(timeout=float(timeout_s))
    except subprocess.TimeoutExpired:
        timed_out = True
        _terminate(proc)
        stdout, stderr = proc.communicate(timeout=30)
    ended = time.time()
    Path(event_path).write_text(stdout or "", encoding="utf-8")
    Path(stderr_path).write_text(stderr or "", encoding="utf-8")
    parsed = parse_events(stdout)
    if Path(answer_path).is_file():
        parsed["final_message"] = Path(answer_path).read_text(
            encoding="utf-8", errors="replace").strip()
    parsed.update({
        "argv": cmd, "started": True, "returncode": proc.returncode,
        "timed_out": timed_out, "wall_s": ended - started,
        "stderr_tail": (stderr or "")[-2000:],
    })
    return parsed


def containment_audit(event_text, *, workspace, repo=staging.REPO):
    """Post-hoc proof that the Codex session did not read the answer key.

    Codex's workspace-write sandbox constrains writes, not every possible read.
    The staged workspace removes the benchmark, and this audit inspects the
    machine-readable tool events for an attempt to reach the real checkout.
    A hit blocks grading.  No event stream is UNKNOWN, never clean.
    """
    parsed = parse_events(event_text)
    if parsed["events"] == 0:
        return {"clean": None, "reason": "no parseable Codex events",
                "hits": []}
    repo_forms = {
        str(Path(repo).resolve()).lower(),
        str(Path(repo).resolve()).replace("\\", "/").lower(),
    }
    ws_forms = {
        str(Path(workspace).resolve()).lower(),
        str(Path(workspace).resolve()).replace("\\", "/").lower(),
    }
    def strings(value):
        if isinstance(value, str):
            yield value
        elif isinstance(value, dict):
            for child in value.values():
                yield from strings(child)
        elif isinstance(value, list):
            for child in value:
                yield from strings(child)

    hits = []
    for line in (event_text or "").splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        for raw in strings(event):
            low = raw.lower()
            # A command event commonly records cwd as well as its actual
            # target. Remove the legitimate workspace spelling before
            # looking for the separate, unstaged checkout.
            for value in ws_forms:
                low = low.replace(value, "<workspace>")
            if any(value in low for value in repo_forms):
                hits.append(raw[:500])
                break
    return {
        "clean": not hits,
        "reason": (None if not hits else
                   "tool events referenced the unstaged repository checkout"),
        "hits": hits[:20],
    }


def _build_template(sim, root):
    if sim == "omnisim":
        return staging.build_omnisim_template(
            Path(root) / "codex_safe",
            junction_dirs=OMNISIM_RUNTIME_JUNCTIONS)
    if sim == "webots":
        return staging.build_webots_template(root)
    if sim == "mujoco":
        return staging.build_mujoco_template(root)
    raise ValueError(sim)


def _copy_deliverable(artifact, dest, *, authored_at):
    dest.parent.mkdir(parents=True, exist_ok=True)
    rebased = []
    if artifact.suffix.lower() in (".wbt", ".omniworld"):
        text = artifact.read_text(encoding="utf-8", errors="replace")
        text, rebased = worldtext.rebase_relative_urls(text, authored_at)
        dest.write_text(text, encoding="utf-8")
    else:
        shutil.copy2(artifact, dest)
    return rebased


def _grade(task_id, sim, artifact, *, answer, answer_path, run_dir,
           model, metrics, layout_seed):
    grade_project = Path(run_dir) / "grade"
    art_dir = Path(run_dir) / "artifact"
    if artifact is None:
        row = shared.no_artifact_row(
            sim, task_id, "the Codex session produced no deliverable")
    else:
        shared.stage_controllers(art_dir, grade_project)
        assets = shared.stage_task_assets(task_id, sim, grade_project)

        placement = None
        if task_id in r1_placement.TASKS:
            placement = r1_placement.place_and_declare(
                artifact, seed=layout_seed,
                declare_dirs=(Path(run_dir), grade_project))

        notes = "Codex CLI model=%s; metrics=%s" % (model, METRICS_SOURCE)
        if sim == "omnisim":
            row = shared.grade_omnisim(
                task_id,
                answer_path if task_id in shared.ANSWER_TASKS else artifact,
                grade_project / "worlds", notes,
                answer_path=(answer_path
                             if task_id in shared.WORLD_PLUS_ANSWER_TASKS
                             else None),
                layout_dir=(Path(run_dir) if placement else None))
        elif sim == "webots":
            row = shared.grade_webots(
                task_id, None if task_id in shared.ANSWER_TASKS else artifact,
                grade_project, answer,
                project_assets=[f["path"] for f in assets["files"]])
        else:
            row = shared.grade_mujoco(
                task_id, artifact, grade_project, answer)

    row["suite"] = shared.SUITE
    row["condition"] = CONDITION
    row.setdefault("agent", {}).update({
        "model": model, "kind": "codex_cli", "backend": "codex exec",
    })
    row.setdefault("tool_set", {}).update({"name": "codex_cli"})
    row.setdefault("metrics", {}).update({
        "t_agent_s": metrics.get("wall_s"),
        "tool_calls": metrics.get("tool_calls"),
        "tokens_in": metrics.get("tokens_in"),
        "tokens_out": metrics.get("tokens_out"),
        "tokens_cache_read": metrics.get("tokens_cache_read"),
    })
    ready, blockers = frontier.publication_ready(task_id, sim)
    artifacts = row.setdefault("agent_artifacts", {})
    artifacts.update({"condition": CONDITION, "external_label": CONDITION})
    artifacts["codex_cli"] = {
        "metrics_source": METRICS_SOURCE,
        "thread_id": metrics.get("thread_id"),
        "events": metrics.get("events"),
        "returncode": metrics.get("returncode"),
        "timed_out": metrics.get("timed_out"),
        "argv": metrics.get("argv"),
        "reasoning_tokens": metrics.get("reasoning_tokens"),
        "publication_ready": ready,
        "publication_blockers": blockers,
    }
    row["protocol"] = {
        "id": shared.PROTOCOL_ID,
        "runs_per_cell": 1,
        "samples": 1,
        "variance_measured": False,
        "is_rate": False,
        "hard_ceiling_s": tasks.TASK_HARD_CEILING_S,
        "task_budget_s": tasks.get(task_id).timeout_s,
        "cell_wall_bound_s": shared.cell_wall_bound_s(
            tasks.get(task_id).timeout_s),
    }
    return row


def run_task(*, sim, task_id, root, out_dir, model, codex=None,
             layout_seed=20260813):
    sims.require_implemented(sim, task_id)
    task = tasks.get(task_id)
    root = Path(root).resolve()
    out_dir = Path(out_dir).resolve()
    repo = staging.REPO.resolve()
    if root == repo or repo in root.parents or root in repo.parents:
        raise ValueError("--root must be outside the repository")
    executable = codex or shutil.which("codex")
    if not executable:
        raise RuntimeError("codex executable not found")
    identity = codex_identity(executable)
    out_dir.mkdir(parents=True, exist_ok=False)
    template = _build_template(sim, root)
    workspace = root / "instances" / out_dir.name
    manifest = staging.instantiate(template, workspace)
    prompt, staged = staging.stage_task(workspace, task_id, sim)
    start_ts = time.time()

    answer_path = out_dir / "answer.txt"
    event_path = out_dir / "codex_events.jsonl"
    stderr_path = out_dir / "codex_stderr.log"
    metrics = run_codex(
        identity["path"], prompt=prompt, workspace=workspace, sim=sim,
        model=model,
        answer_path=answer_path, event_path=event_path,
        stderr_path=stderr_path, timeout_s=task.timeout_s)
    metrics["cli_identity"] = identity
    event_text = (event_path.read_text(encoding="utf-8", errors="replace")
                  if event_path.is_file() else "")
    containment = containment_audit(event_text, workspace=workspace)

    report = {
        "task": task_id, "sim": sim, "model": model,
        "condition": CONDITION, "workspace": str(workspace),
        "staging_manifest_sha": manifest.get("filelist_sha256"),
        "staged": staged, "metrics": metrics,
        "containment": containment, "graded": False,
    }
    (out_dir / "session.json").write_text(
        json.dumps(report, indent=2, default=str), encoding="utf-8")
    if containment.get("clean") is not True:
        raise RuntimeError(
            "Codex containment is not proven clean; session preserved and "
            "the cell was not graded: %s" % containment.get("reason"))
    answer = metrics.get("final_message") or ""
    if not answer_path.is_file():
        answer_path.write_text(answer, encoding="utf-8")
    if task_id in shared.ANSWER_TASKS:
        artifact = answer_path
        rule = "the Codex final message is the task artifact"
    else:
        artifact, rule = shared.discover_artifact(
            workspace, staged, start_ts,
            suffixes=external_agent.artifact_suffixes(sim))
    if artifact is not None and not Path(artifact).is_file():
        artifact = None

    collected = None
    if task_id not in shared.ANSWER_TASKS and artifact is not None:
        name = external_agent.artifact_name(task_id, sim)
        collected = out_dir / "artifact" / name
        rebased = _copy_deliverable(
            Path(artifact), collected, authored_at=Path(artifact).parent)
        if collected.suffix.lower() in (".wbt", ".omniworld"):
            shared.collect_controllers(
                Path(artifact), out_dir / "artifact",
                search_roots=shared.candidate_project_roots(Path(artifact)))
        elif collected.suffix.lower() == ".xml":
            shared.collect_driver(Path(artifact), collected)
        report["rebased_urls"] = rebased

    row = _grade(
        task_id, sim, collected, answer=answer, answer_path=answer_path,
        run_dir=out_dir, model=model, metrics=metrics,
        layout_seed=layout_seed)
    report.update({"graded": True, "artifact_rule": rule,
                   "outcome": row.get("outcome")})
    (out_dir / "grader_row.json").write_text(
        json.dumps(row, indent=2, default=str), encoding="utf-8")
    (out_dir / "rows.jsonl").write_text(
        json.dumps(row, default=str) + "\n", encoding="utf-8")
    (out_dir / "session.json").write_text(
        json.dumps(report, indent=2, default=str), encoding="utf-8")
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sim", required=True, choices=sims.IMPLEMENTED)
    ap.add_argument("--task", required=True)
    ap.add_argument("--model", required=True,
                    help="pin the exact Codex model for this campaign")
    ap.add_argument("--root", required=True,
                    help="workspace staging root outside the repository")
    ap.add_argument("--out", required=True,
                    help="new output directory for this cell")
    ap.add_argument("--codex", help="Codex executable (default: PATH)")
    ap.add_argument("--layout-seed", type=int, default=20260813)
    args = ap.parse_args(argv)
    row = run_task(
        sim=args.sim, task_id=args.task, root=args.root, out_dir=args.out,
        model=args.model, codex=args.codex, layout_seed=args.layout_seed)
    print("%s %s %s" % (row.get("sim"), row.get("task"),
                         row.get("outcome")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
