"""Generate AgenticSimBench v1 R4 red evidence with the real OmniSim engine."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parents[1]
BENCHMARKS = AGENTBENCH.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.adapters.omnisim import evidence, headless  # noqa: E402
from agentbench.common.paths import REPO, engine_launch  # noqa: E402
from agentbench.graders import r4_core  # noqa: E402


SOURCE_WORLD = (AGENTBENCH / "adapters" / "webots" / "webots_lane" /
                "worlds" / "r4_mobile_manipulation.wbt")
FIXTURE_DIR = HERE / "fixtures" / "r4"
ROBOT_STANZA = FIXTURE_DIR / "robot_stanza.wbtfrag"
CONTROLLER_DIR = FIXTURE_DIR / "controllers" / "r4_fixture"
FIXTURES = {
    "dirty": {"expected": {"R4.1"}},
    "scene_tamper": {"expected": {"R4.2"}},
    "insufficient": {"expected": {"R4.3", "R4.5", "R4.6", "R4.7", "R4.8"}},
    "bad_start": {"expected": {"R4.4", "R4.5", "R4.6", "R4.7", "R4.8", "R4.9"}},
    "collision": {"expected": {"R4.5"}},
    "no_carry": {"expected": {"R4.6", "R4.7", "R4.8"}},
    "reacquire": {"expected": {"R4.7"}},
    "wrong_delivery": {"expected": {"R4.8"}},
    "teleport": {"expected": {"R4.6", "R4.7", "R4.9"}},
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def block_span(text, marker, start=0):
    begin = text.index(marker, start)
    opening = text.index("{", begin)
    depth = 0
    for pos in range(opening, len(text)):
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
            if depth == 0:
                return begin, pos + 1
    raise ValueError("unbalanced block after %r" % marker)


def stage(dest, fixture):
    dest = Path(dest)
    text = SOURCE_WORLD.read_text(encoding="utf-8")
    begin, _end = block_span(text, "DEF ROBOT Robot {")
    robot = ROBOT_STANZA.read_text(encoding="utf-8").replace(
        "__R4_MODE__", fixture)
    if fixture == "insufficient":
        spans, pos = [], 0
        while True:
            try:
                span = block_span(robot, "HingeJoint {", pos)
            except ValueError:
                break
            spans.append(span)
            pos = span[1]
        for a, b in reversed(spans[1:]):
            robot = robot[:a] + robot[b:]
    text = text[:begin] + robot + "\n"
    text = text.replace(
        "WorldInfo {\n", "WorldInfo {\n  newtonSolver \"mujoco\"\n", 1)
    if fixture == "dirty":
        text = text.replace(
            "WorldInfo {\n",
            "WorldInfo {\n  agentbenchIntentionalUnknownField 1\n", 1)
    if fixture == "scene_tamper":
        text = text.replace(
            "DEF OBSTACLE_5 Solid {\n  translation 4.1758 -0.8524 0.25",
            "DEF OBSTACLE_5 Solid {\n  translation 4.5 0.8 0.25",
            1)
    if fixture == "bad_start":
        text = text.replace(
            "DEF PAYLOAD Solid {\n  translation 3 3 0.625",
            "DEF PAYLOAD Solid {\n  translation 3.1 3 0.625",
            1)
    world = dest / "worlds" / "mobile_manipulation.wbt"
    world.parent.mkdir(parents=True, exist_ok=True)
    world.write_text(text, encoding="utf-8")
    shutil.copytree(CONTROLLER_DIR, dest / "controllers" / "r4_fixture",
                    dirs_exist_ok=True)
    return world


def generate(work_dir, fixture, output=None):
    if fixture not in FIXTURES:
        raise ValueError("unknown fixture")
    work_dir = Path(work_dir).resolve()
    allowed = (REPO / ".agentbench-v1-red-work").resolve()
    if allowed not in work_dir.parents:
        raise ValueError("work directory must be below %s" % allowed)
    if work_dir.exists():
        raise FileExistsError(work_dir)
    binary = engine_launch.resolve_binary(REPO)
    if binary is None:
        raise RuntimeError("built OmniSim binary required")
    python_dir = str(Path(sys.executable).resolve().parent)
    if python_dir.lower() not in {
        p.lower() for p in os.environ.get("PATH", "").split(os.pathsep) if p
    }:
        os.environ["PATH"] = python_dir + os.pathsep + os.environ.get("PATH", "")
    os.environ["WARP_CACHE_PATH"] = str(work_dir / "warp_cache")
    world = stage(work_dir / "project", fixture)
    phase = headless.run_standalone(
        world, work_dir / "phaseB", duration=45.0, settle=0.0,
        contact_steps=-1, links=16, solids=("payload", "table", "pad"),
        timeout_s=900.0, tag="phaseB")
    bundle = evidence.build_bundle(
        "R4_mobile_manipulation", robot_identity="any_robot",
        artifact=str(world), phase_b=phase, run_dir=work_dir / "phaseB",
        scene_inventory=True)
    verdict = r4_core.grade(bundle)
    failures = {a.id for a in verdict.assertions if not a.ok}
    expected = FIXTURES[fixture]["expected"]
    if failures != expected:
        raise RuntimeError("expected %s, observed %s\n%s" %
                           (sorted(expected), sorted(failures),
                            verdict.summary()))
    if output is None:
        output = HERE / "red_evidence" / (
            "R4_mobile_manipulation.%s.omnisim.verdict.json" % fixture)
    source_paths = [
        SOURCE_WORLD, ROBOT_STANZA, Path(__file__).resolve(),
        Path(r4_core.__file__).resolve(),
        *[p for p in CONTROLLER_DIR.rglob("*")
          if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"],
    ]
    doc = {
        "schema": "agenticsimbench/red-evidence/v1",
        "suite": "agenticsimbench/v1", "task": "R4_mobile_manipulation",
        "sim": "omnisim", "fixture": fixture,
        "fixture_kind": "real_supervisor_mobile_manipulation_fixture",
        "expected_failures": sorted(expected),
        "observed_failures": sorted(failures), "outcome": verdict.outcome,
        "qualifies_as_non_null_red_evidence": True,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "engine_binary": {"path": str(Path(binary).relative_to(REPO)),
                          "sha256": sha(binary)},
        "sources": [{"path": str(p.relative_to(REPO)), "sha256": sha(p)}
                    for p in source_paths],
        "staged_world_sha256": sha(world),
        "verdict": verdict.as_dict(), "run": phase.as_dict(),
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(doc, indent=2, default=str) + "\n",
                      encoding="utf-8")
    print("wrote %s" % output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", choices=sorted(FIXTURES), required=True)
    parser.add_argument("--work", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    generate(args.work, args.fixture, args.output)


if __name__ == "__main__":
    main()
