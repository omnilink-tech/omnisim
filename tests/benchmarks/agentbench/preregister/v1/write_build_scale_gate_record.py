#!/usr/bin/env python3
"""Distil real oracle/null run directories into a hash-bound gate record."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
from datetime import date
from pathlib import Path


HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
REPO = AGENTBENCH.parents[2]
OMNISIM_SOURCES = (
    "preregister/v1/build_scale_spec.json",
    "preregister/v1/build_scale_core.py",
    "adapters/omnisim/build_scale.py",
    "adapters/omnisim/evidence.py",
    "adapters/omnisim/headless.py",
    "../../../projects/default/controllers/harness_supervisor/observe.py",
    "controllers/agentbench_recorder/agentbench_recorder.py",
    "adapters/omnisim/build_scale_lane/protos/ScaleBot.proto",
    "adapters/omnisim/build_scale_lane/controllers/build_scale_drive/build_scale_drive.py",
    "adapters/omnisim/build_scale_lane/controllers/build_scale_idle/build_scale_idle.py",
)

WEBOTS_SOURCES = (
    "preregister/v1/build_scale_spec.json",
    "preregister/v1/build_scale_core.py",
    "adapters/webots/build_scale.py",
    "adapters/webots/evidence.py",
    "adapters/webots/launcher.py",
    "adapters/webots/task_support.py",
    "adapters/webots/webots_lane/controllers/agentbench_webots_recorder/agentbench_webots_recorder.py",
    "adapters/webots/build_scale_lane/controllers/build_scale_drive/build_scale_drive.py",
    "adapters/webots/build_scale_lane/controllers/build_scale_idle/build_scale_idle.py",
)


def _load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--oracle-run", required=True, type=Path)
    parser.add_argument("--null-run", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--sim", default="omnisim", choices=("omnisim", "webots"))
    parser.add_argument("--level", default=10, type=int)
    parser.add_argument("--engine", type=Path)
    parser.add_argument("--engine-sha256")
    args = parser.parse_args()
    args.out = args.out.resolve()

    oracle = _load(args.oracle_run / "verdict.json")
    null = _load(args.null_run / "verdict.json")
    oracle_facts = _load(args.oracle_run / "facts.json")
    null_facts = _load(args.null_run / "facts.json")
    if oracle.get("outcome") != "PASS" or oracle.get("n_pass") != 9:
        raise SystemExit("refusing record: oracle must pass all nine assertions")
    if null.get("outcome") != "FAIL" or "BS.5" not in null.get(
            "failed_assertions", []):
        raise SystemExit("refusing record: null must fail physical movement BS.5")
    lane = f"adapters/{args.sim}/build_scale_lane/worlds"
    sources = (OMNISIM_SOURCES if args.sim == "omnisim" else WEBOTS_SOURCES) + (
        f"{lane}/build_scale_{args.level}.wbt",
        f"{lane}/build_scale_{args.level}_null.wbt",
    )
    missing = [relative for relative in sources
               if not (AGENTBENCH / relative).is_file()]
    if missing:
        raise SystemExit("missing bound sources: " + ", ".join(missing))

    def measured(aid):
        return oracle["assertions"][aid]["measured"]

    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    evidence_dir = args.out.parent / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    evidence_docs = {
        f"{args.out.stem}.oracle.verdict.json": oracle,
        f"{args.out.stem}.oracle.facts.json": oracle_facts,
        f"{args.out.stem}.null.verdict.json": null,
        f"{args.out.stem}.null.facts.json": null_facts,
    }
    evidence_paths = {}
    for name, doc in evidence_docs.items():
        path = evidence_dir / name
        path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        relative = str(path.relative_to(AGENTBENCH)).replace("\\", "/")
        evidence_paths[relative] = _sha(path)

    def wall(facts, role):
        rec = facts[role]
        return rec.get("wall_s", (rec.get("process") or {}).get("wall_s"))

    if args.engine:
        engine_name = str(args.engine.resolve())
        engine_hash = _sha(args.engine)
    else:
        if not args.engine_sha256:
            raise SystemExit("provide --engine or --engine-sha256")
        engine_name = "/opt/upstream-webots/R2025a/webots-bin"
        engine_hash = args.engine_sha256.lower()

    record = {
        "schema": "agenticsimbench/build-scale-live-gate/v1",
        "suite": "agenticsimbench/v1",
        "sim": args.sim,
        "level": args.level,
        "measured_utc": date.today().isoformat(),
        "outcomes": {"oracle": "PASS", "null": "FAIL"},
        "oracle": {
            "assertions_passed": oracle["n_pass"],
            "assertions_total": oracle["n_total"],
            "robot_count": measured("BS.2")["robot bodies"],
            "controller_instances": measured("BS.3")[
                "runtime controller instances started"],
            "minimum_clearance_m": measured("BS.4")[
                "minimum AABB clearance (m)"],
            "moving_robots": measured("BS.5")["moving robots"],
            "minimum_net_displacement_m": measured("BS.5")[
                "minimum net displacement (m)"],
            "minimum_path_length_m": measured("BS.5")[
                "minimum path length (m)"],
            "unique_robot_collision_pairs": measured("BS.7")[
                "unique robot collision pairs"],
            "maximum_speed_m_s": measured("BS.8")["maximum speed (m/s)"],
            "clean_replay_passes": measured("BS.9")["clean replay passes"],
            "primary_wall_s": wall(oracle_facts, "primary"),
            "clean_replay_wall_s": wall(oracle_facts, "clean_replay"),
        },
        "null": {
            "failed_assertions": null["failed_assertions"],
            "moving_robots": null["assertions"]["BS.5"]["measured"][
                "moving robots"],
            "primary_wall_s": wall(null_facts, "primary"),
            "clean_replay_wall_s": wall(null_facts, "clean_replay"),
        },
        "environment": {
            "machine": platform.node(),
            "host_os": platform.platform(),
            "engine": engine_name,
            "engine_sha256": engine_hash,
            "solver": oracle.get("attribution", {}).get("solver"),
            "runtime": ((oracle_facts["primary"].get("sidecar") or {}).get("runtime")
                        or {"webots_version": (oracle_facts["primary"].get("process")
                                               or {}).get("webots_version")}),
        },
        "source_commit": commit,
        "source_sha256": {relative: _sha(AGENTBENCH / relative)
                          for relative in sources},
        "run_evidence_sha256": evidence_paths,
        "claim_scope": ("Live BuildScale discriminator gate only; this is not "
                        "a scored coding-agent result or comparative win."),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
