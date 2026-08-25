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

"""Stable ``python -m omnisim policy`` command surface."""

from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from omnisim.paths import REPO_ROOT

from .artifacts import build_record, evaluate_promotion
from .compatibility import G1_ENV_BASELINES, env_digest, verify_g1_envs
from .matrix import load as load_matrix
from .matrix import validate as validate_matrix
from .motion_ir import MotionBinding, MotionIR
from .repository import PolicyRepository
from .skill_graph import SkillGraph


LEGACY = REPO_ROOT / "projects" / "policies" / "skills" / "skill_lib.py"
VERB_TABLE = LEGACY.parent / "skill_verbs.py"
MATRIX = REPO_ROOT / "projects" / "policies" / "benchmarks" / "matrix.json"
SUITE = REPO_ROOT / "projects" / "policies" / "benchmarks" / "suite.json"
# Implemented here rather than delegated. `audit` is BOTH: it runs the skill-library
# gates and then the public-core ones, so it is registered natively and skipped when
# the delegated verbs are registered.
NATIVE = ("graph", "ir", "matrix", "env-hash", "promote", "audit")


def _verb_table():
    """Load the shared skill-library verb table (argparse-only, no heavy deps).

    Returns None when the policies tree is absent from this install -- the native
    verbs still work, there is simply nothing to delegate to.
    """
    if not VERB_TABLE.exists() or not LEGACY.exists():
        return None
    spec = importlib.util.spec_from_file_location("_omnisim_skill_verbs", VERB_TABLE)
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_VERBS = _verb_table()
# Verbs whose implementation lives in skill_lib.py. DERIVED from its own table --
# a second hand-copied list here is exactly how this set came to be forwarded (all
# 15 of them ran) but never registered, so `--help` advertised only the 6 native
# verbs and the documented `list` / `preview` / `train` / `sequence` read as missing.
PASSTHROUGH = frozenset(_VERBS.names(skip=NATIVE) if _VERBS else ())


def _repo() -> PolicyRepository:
    return PolicyRepository(REPO_ROOT)


def _legacy(argv: list[str]) -> int:
    return subprocess.run([sys.executable, str(LEGACY), *argv], cwd=REPO_ROOT, check=False).returncode


def _legacy_module():
    skills_dir = LEGACY.parent
    if str(skills_dir) not in sys.path:
        sys.path.insert(0, str(skills_dir))
    spec = importlib.util.spec_from_file_location("_omnisim_legacy_skill_lib", LEGACY)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load compatibility runner at {LEGACY}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _assembled_env(name: str) -> dict[str, str]:
    legacy = _legacy_module()
    registry = legacy.M.Registry.discover()
    return legacy._assemble_env(registry.sequences[name], registry)


def _graphs(repo: PolicyRepository, name: str | None = None) -> list[SkillGraph]:
    skills, sequences = repo.skill_catalog(), repo.sequence_catalog()
    selected = [name] if name else sorted(sequences)
    unknown = [item for item in selected if item not in sequences]
    if unknown:
        raise KeyError(f"unknown sequence {unknown[0]!r}")
    return [SkillGraph.from_sequence(sequences[item], skills) for item in selected]


def _benchmark_cases(kind: str, name: str) -> list[dict[str, Any]]:
    suite = json.loads(SUITE.read_text(encoding="utf-8"))
    matches: list[dict[str, Any]] = []
    for case in suite.get("cases", []):
        target = case.get("target") or {"kind": "sequence", "name": case.get("sequence")}
        if target == {"kind": kind, "name": name}:
            matches.append(case)
    return matches


def _cmd_graph(args: argparse.Namespace) -> int:
    try:
        graphs = _graphs(_repo(), args.name)
    except KeyError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    issues = [(graph.name, issue) for graph in graphs for issue in graph.validate()]
    if args.json:
        print(json.dumps([graph.to_dict() for graph in graphs], indent=2))
    else:
        for graph in graphs:
            print(f"{graph.name}: {len(graph.nodes)} nodes, {len(graph.edges)} typed transitions, "
                  f"legacy round-trip={'PASS' if not graph.validate() else 'FAIL'}")
            for edge in graph.edges:
                print(f"  {edge.source} -> {edge.target}  {edge.gate}/{edge.handover}  "
                      f"recovery={edge.recovery}")
    for graph_name, issue in issues:
        print(f"[ERR ] {graph_name}: {issue}", file=sys.stderr)
    return 1 if issues else 0


def _cmd_ir(args: argparse.Namespace) -> int:
    repo = _repo()
    skills = repo.skill_catalog()
    motions = repo.motion_catalog()
    selected = [args.name] if args.name else sorted(skills)
    if any(name not in skills for name in selected):
        print(f"unknown skill {args.name!r}", file=sys.stderr)
        return 2
    values = []
    issues: list[str] = []
    for name in selected:
        motion = MotionIR.from_skill({**skills[name], "motion_ir": motions.get(name)})
        binding = MotionBinding.from_skill(skills[name], motion)
        issues.extend(f"{name}: {issue}" for issue in [*motion.validate(), *binding.validate()])
        values.append({"motion": motion.to_dict(), "binding": {
            "schema": binding.schema, "motion": binding.motion, "robot": binding.robot,
            "implementation": binding.implementation, "reference": binding.reference,
            "checkpoint": binding.checkpoint, "compiler": binding.compiler,
        }})
    if args.json or len(values) != 1:
        print(json.dumps(values, indent=2))
    else:
        print(json.dumps(values[0], indent=2))
    for issue in issues:
        print(f"[ERR ] {issue}", file=sys.stderr)
    return 1 if issues else 0


def _cmd_matrix(args: argparse.Namespace) -> int:
    matrix = load_matrix(MATRIX)
    issues = validate_matrix(_repo(), matrix)
    if args.json:
        print(json.dumps(matrix, indent=2))
    else:
        cases = matrix.get("cases", [])
        verified = sum(case.get("status") == "verified" for case in cases)
        print(f"policy benchmark matrix: {len(cases)} cases, {verified} verified, "
              f"{len(cases) - verified} explicit candidates")
        for case in cases:
            target = case["target"]
            print(f"  {case['name']:<30} {case['status']:<10} "
                  f"{target['kind']}:{target['name']}  {case['morphology']}")
    for issue in issues:
        print(f"[ERR ] {issue}", file=sys.stderr)
    return 1 if issues else 0


def _cmd_env_hash(args: argparse.Namespace) -> int:
    names = [args.name] if args.name else sorted(G1_ENV_BASELINES)
    failed = False
    for name in names:
        if name not in G1_ENV_BASELINES:
            print(f"no pinned G1 baseline for {name!r}", file=sys.stderr)
            failed = True
            continue
        env = _assembled_env(name)
        actual = (len(env), env_digest(env))
        expected = G1_ENV_BASELINES[name]
        ok = actual == expected
        print(f"{name}: {'PASS' if ok else 'FAIL'} keys={actual[0]} sha256={actual[1]}")
        failed |= not ok
    return 1 if failed else 0


def _cmd_promote(args: argparse.Namespace) -> int:
    repo = _repo()
    try:
        catalog = repo.skill_catalog() if args.kind == "skill" else repo.sequence_catalog()
        raw = catalog[args.name]
        cases = _benchmark_cases(args.kind, args.name)
        evidence_paths = [str((case.get("reference_evidence") or {}).get("result_file", ""))
                          for case in cases if case.get("status") == "verified"]
        env = _assembled_env(args.name) if args.kind == "sequence" else None
        record = build_record(repo, args.kind, args.name, assembled_env=env,
                              evidence_paths=evidence_paths)
    except (KeyError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    result = evaluate_promotion(record, raw, cases)
    print(json.dumps(result, indent=2))
    if args.write_lock:
        locks = repo.policies / "artifacts" / "locks"
        locks.mkdir(parents=True, exist_ok=True)
        path = locks / f"{args.kind}-{args.name}-{result['content_id'][:16]}.json"
        path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"promotion lock: {path.relative_to(repo.root).as_posix()}")
    required = {"catalogued": 0, "qualified": 1, "release": 2}[args.require]
    actual = {"catalogued": 0, "qualified": 1, "release": 2}[result["promotion_tier"]]
    return 0 if actual >= required else 1


def _native_audit() -> int:
    repo = _repo()
    issues: list[str] = []
    print("\n== public policy core ==")
    for graph in _graphs(repo):
        issues.extend(f"{graph.name}: {issue}" for issue in graph.validate())
    print(f"[{'ok  ' if not issues else 'ERR '}] typed graphs: "
          f"{len(repo.sequence_catalog())} sequence(s), lossless legacy compilation")
    matrix_issues = validate_matrix(repo, load_matrix(MATRIX))
    issues.extend(matrix_issues)
    print(f"[{'ok  ' if not matrix_issues else 'ERR '}] benchmark matrix")
    compatibility = verify_g1_envs(_assembled_env)
    issues.extend(compatibility)
    print(f"[{'ok  ' if not compatibility else 'ERR '}] pinned G1 deploy environments: "
          f"{len(G1_ENV_BASELINES)} unchanged")
    ir_issues: list[str] = []
    motions = repo.motion_catalog()
    for name, skill in repo.skill_catalog().items():
        if name not in motions:
            ir_issues.append(f"{name}: missing explicit motion IR catalog entry")
        motion = MotionIR.from_skill({**skill, "motion_ir": motions.get(name)})
        binding = MotionBinding.from_skill(skill, motion)
        ir_issues.extend(f"{name}: {issue}" for issue in [*motion.validate(), *binding.validate()])
    issues.extend(ir_issues)
    print(f"[{'ok  ' if not ir_issues else 'ERR '}] motion IR: "
          f"{len(repo.skill_catalog())} skill binding(s)")
    for issue in issues:
        print(f"[ERR ] {issue}")
    print(f"public policy core audit: {'PASS' if not issues else 'FAIL'}")
    return 1 if issues else 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="omnisim policy",
        description="Stable policy-brain API: the skill pipeline (design -> validate -> "
                    "preview -> train -> run -> sequence) plus typed BATON graphs, "
                    "benchmarks and promotion.",
    )
    sub = parser.add_subparsers(dest="command", metavar="<verb>")
    # The skill-pipeline verbs, registered from skill_lib.py's own table (so this
    # help screen can never disagree with what actually dispatches) and executed by
    # delegating to it. These come first: they are what a new user starts with.
    if _VERBS:
        _VERBS.register(sub, skip=NATIVE)
    graph = sub.add_parser("graph", help="Inspect/validate typed BATON graphs.")
    graph.add_argument("name", nargs="?")
    graph.add_argument("--json", action="store_true")
    motion = sub.add_parser("ir", help="Inspect robot-independent motion intent + robot binding.")
    motion.add_argument("name", nargs="?")
    motion.add_argument("--json", action="store_true")
    matrix = sub.add_parser("matrix", help="Inspect/validate the benchmark coverage matrix.")
    matrix.add_argument("--json", action="store_true")
    digest = sub.add_parser("env-hash", help="Verify pinned flagship G1 deploy environments.")
    digest.add_argument("name", nargs="?")
    promote = sub.add_parser("promote", help="Build a content-addressed promotion verdict.")
    promote.add_argument("kind", choices=("skill", "sequence"))
    promote.add_argument("name")
    promote.add_argument("--require", choices=("catalogued", "qualified", "release"),
                         default="catalogued")
    promote.add_argument("--write-lock", action="store_true",
                         help="Write the content-addressed verdict under projects/policies/artifacts/locks.")
    sub.add_parser("audit", help="Run legacy policy gates plus the public-core gates.")
    return parser


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] in PASSTHROUGH:
        # Parse against our own mirror of the verb table first: same table, so it
        # can only reject what skill_lib.py would reject, and `-h` / usage errors
        # name the front door the user typed instead of `skill_lib.py`. Then hand
        # the untouched argv to the implementation.
        _parser().parse_args(argv)
        return _legacy(argv)
    args = _parser().parse_args(argv)
    if args.command is None:
        _parser().print_help()
        return 0
    if args.command == "audit":
        legacy = _legacy(["audit"])
        native = _native_audit()
        return 1 if legacy or native else 0
    if args.command == "graph":
        return _cmd_graph(args)
    if args.command == "ir":
        return _cmd_ir(args)
    if args.command == "matrix":
        return _cmd_matrix(args)
    if args.command == "env-hash":
        return _cmd_env_hash(args)
    if args.command == "promote":
        return _cmd_promote(args)
    return 2
