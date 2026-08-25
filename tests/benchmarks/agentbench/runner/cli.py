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

"""``python -m agentbench.runner <cmd>`` -- inspect the runner without a run.

    dump-manifests [--out DIR]   write both tool-set manifests to JSON
    hashes                       print the two hashes per condition
    show-prompt [--condition C]  print the exact system prompt that is sent
    preflight [--model M]        offline validation of the anthropic backend
    run --task A1 [...]          one standalone run outside run_agentbench.py

``dump-manifests`` is the reviewer's entry point: the JSON is what each
condition is given, and diffing the two files is how you see what
``shell+tools`` adds. ``hashes`` is the fairness check -- the ``shell``
condition's ``tools_sha256`` must be identical on every simulator.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE.parents[2]) not in sys.path:
    sys.path.insert(0, str(_HERE.parents[2]))

from agentbench.common.paths import REPO                     # noqa: E402
from agentbench.runner.config import RunnerConfig            # noqa: E402
from agentbench.runner.isolation import Sandbox              # noqa: E402
from agentbench.runner.tools import (                        # noqa: E402
    CONDITIONS, get_tool_set, normalize_condition)


def _scratch_sandbox(tmp: Path, *, ports=True) -> Sandbox:
    return Sandbox.create(tmp / "run", tmp / "run" / "scratch", repo=REPO,
                          ports=ports)


def cmd_dump_manifests(args):
    out = Path(args.out) if args.out else (_HERE / "manifests")
    with tempfile.TemporaryDirectory(prefix="agentbench_manifest_") as tmp:
        sb = _scratch_sandbox(Path(tmp), ports=not args.no_ports)
        # (sim, condition, filename stem). sim=None == the default resolution
        # (OmniSim); the Webots shell+tools cell is its own file because the
        # bridge is a published condition (plan 2.2 / SPEC 4.1.1) -- and the
        # `shell` baseline is deliberately NOT dumped per sim: it is
        # byte-identical everywhere, which `hashes` can prove.
        cells = [(None, cond, cond) for cond in CONDITIONS]
        cells.append(("webots", "shell_plus_tools", "webots_shell_plus_tools"))
        for sim, cond, stem in cells:
            ts = get_tool_set(cond, sb, sim=sim)
            path = out / ("%s.json" % stem)
            sha = ts.dump(path)
            print("%-26s %3d tools  tools_sha256=%s  manifest_sha256=%s"
                  % (stem, len(ts), ts.tools_sha256[:16], sha[:16]))
            print("    -> %s" % path)
    return 0


def cmd_hashes(args):
    with tempfile.TemporaryDirectory(prefix="agentbench_hashes_") as tmp:
        sb = _scratch_sandbox(Path(tmp), ports=not args.no_ports)
        for cond in CONDITIONS:
            ts = get_tool_set(cond, sb)
            print(json.dumps({"condition": cond, "n_tools": len(ts),
                              "names": ts.names,
                              "tools_sha256": ts.tools_sha256,
                              "manifest_sha256": ts.manifest_sha256}))
    return 0


def cmd_show_prompt(args):
    from agentbench.runner import prompts as prompt_mod
    cond = normalize_condition(args.condition)
    with tempfile.TemporaryDirectory(prefix="agentbench_prompt_") as tmp:
        sb = _scratch_sandbox(Path(tmp), ports=not args.no_ports)
        ts = get_tool_set(cond, sb)
        composed = prompt_mod.compose(ts, sb)
        print(composed["text"])
        print("\n--- %s: base_sha=%s full_sha=%s ---"
              % (cond, composed["base_sha"][:16], composed["full_sha"][:16]))
    return 0


def cmd_preflight(args):
    from agentbench.runner.backends.anthropic_api import (
        DEFAULT_MODEL, preflight_offline)
    cond = normalize_condition(args.condition)
    with tempfile.TemporaryDirectory(prefix="agentbench_preflight_") as tmp:
        sb = _scratch_sandbox(Path(tmp), ports=not args.no_ports)
        ts = get_tool_set(cond, sb)
        report = preflight_offline(ts.as_anthropic_tools(),
                                   model=args.model or DEFAULT_MODEL)
    report["condition"] = cond
    print(json.dumps(report, indent=2))
    return 0 if report.get("ok") else 1


def cmd_run(args):
    """One standalone run. Authors the artifact; does NOT grade it.

    Grading goes through ``run_agentbench.py --agent llm`` so the phase-A
    harness pass, the grader-owned recorder and the row writer are the shipped
    ones. This subcommand exists for iterating on the loop itself.
    """
    from agentbench.agents.base import AgentContext
    from agentbench.common.trace import Trace
    from agentbench.runner.loop import run_agent
    from agentbench import tasks as task_registry

    task = task_registry.get(args.task)
    out = Path(args.out or (_HERE.parent / "results" /
                            ("runner_%s" % time.strftime("%Y%m%d_%H%M%S"))))
    run_dir = out / ("%s.%s" % (task.id, args.condition.replace("+", "_")))
    scratch = run_dir / "scratch"
    scratch.mkdir(parents=True, exist_ok=True)
    trace = Trace(run_dir)
    ctx = AgentContext(task_id=task.id, prompt=task.prompt,
                       scratch_dir=scratch, run_dir=run_dir, repo=REPO,
                       trace=trace,
                       deadline_s=time.time() + (task.timeout_s or 0),
                       seed=0)
    cfg = RunnerConfig.from_env(condition=normalize_condition(args.condition),
                                backend=args.backend, model=args.model,
                                script=args.script)
    try:
        res = run_agent(ctx, cfg)
    finally:
        trace.close()
    print(json.dumps(res.as_dict(), indent=2, default=str))
    print("\nartifacts under %s" % run_dir)
    return 0 if res.stop_reason == "model_stopped" else 1


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m agentbench.runner",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--no-ports", action="store_true",
                    help="skip port probing (offline / CI)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("dump-manifests")
    p.add_argument("--out", default=None)
    p.set_defaults(fn=cmd_dump_manifests)

    p = sub.add_parser("hashes")
    p.set_defaults(fn=cmd_hashes)

    p = sub.add_parser("show-prompt")
    p.add_argument("--condition", default="shell")
    p.set_defaults(fn=cmd_show_prompt)

    p = sub.add_parser("preflight")
    p.add_argument("--model", default=None)
    p.add_argument("--condition", default="shell")
    p.set_defaults(fn=cmd_preflight)

    p = sub.add_parser("run")
    p.add_argument("--task", default="A1")
    p.add_argument("--condition", default="shell")
    p.add_argument("--backend", default=None)
    p.add_argument("--model", default=None)
    p.add_argument("--script", default=None)
    p.add_argument("--out", default=None)
    p.set_defaults(fn=cmd_run)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
