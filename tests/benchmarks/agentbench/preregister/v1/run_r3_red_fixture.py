"""Generate AgenticSimBench v1 R3 red evidence with the real OmniSim engine."""

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
from agentbench.graders import r3_core  # noqa: E402


TASK_DIR = AGENTBENCH / "tasks" / "R3_pick_and_place"
SOURCE_WORLD = TASK_DIR / "initial" / "pick_and_place.wbt"
SCENE_ASSET = TASK_DIR / "initial" / "benchmark_assets" / "scene.json"
META = TASK_DIR / "meta.json"
FIXTURE_DIR = HERE / "fixtures" / "r3"
ARM_STANZA = FIXTURE_DIR / "arm_stanza.wbtfrag"
CONTROLLER_DIR = FIXTURE_DIR / "controllers" / "r3_fixture"

FIXTURES = {
    "dirty": {"expected": {"R3.1", "R3.5", "R3.6", "R3.7"}},
    "scene_tamper": {"expected": {"R3.2", "R3.5", "R3.6", "R3.7"}},
    "insufficient_arm": {"expected": {"R3.3", "R3.5", "R3.6", "R3.7"}},
    "bad_start": {"expected": {"R3.4", "R3.5", "R3.6", "R3.7"}},
    "no_lift": {"expected": {"R3.5", "R3.6", "R3.7"}},
    "never_released": {"expected": {"R3.6", "R3.7"}},
    "wrong_destination": {"expected": {"R3.7"}},
    "teleport": {"expected": {"R3.8"}},
}


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _block_span(text, marker, start=0):
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


def _mutate(text, fixture):
    transforms = []
    if fixture == "dirty":
        text = text.replace(
            "WorldInfo {\n",
            "WorldInfo {\n  agentbenchIntentionalUnknownField 1\n",
            1,
        )
        transforms.append("added one intentional unknown WorldInfo field")
    if fixture == "scene_tamper":
        old = "DEF BIN Solid {\n  translation 0.3 0.35 0.8"
        new = "DEF BIN Solid {\n  translation 0.6 0.35 0.8"
        if text.count(old) != 1:
            raise ValueError("could not identify the shipped bin pose")
        text = text.replace(old, new)
        transforms.append("moved the bin 0.30 m in +x")

    arm = ARM_STANZA.read_text(encoding="utf-8").replace(
        "__R3_MODE__", fixture)
    if fixture == "insufficient_arm":
        spans = []
        pos = 0
        while True:
            try:
                span = _block_span(arm, "HingeJoint {", pos)
            except ValueError:
                break
            spans.append(span)
            pos = span[1]
        if len(spans) != 3:
            raise ValueError("expected three fixture-arm joints")
        for begin, end in reversed(spans[1:]):
            arm = arm[:begin] + arm[end:]
        transforms.append("removed two of the fixture arm's three joints")
    return text + arm, transforms


def _stage(dest, fixture):
    dest = Path(dest)
    world = dest / "worlds" / "pick_and_place.wbt"
    world.parent.mkdir(parents=True, exist_ok=True)
    text, transforms = _mutate(
        SOURCE_WORLD.read_text(encoding="utf-8"), fixture)
    world.write_text(text, encoding="utf-8")
    target = dest / "controllers" / "r3_fixture"
    shutil.copytree(CONTROLLER_DIR, target, dirs_exist_ok=True)
    controller_sources = sorted(
        path for path in CONTROLLER_DIR.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
        and path.suffix != ".pyc")
    return world, transforms, controller_sources


def generate(work_dir, fixture, output=None):
    if fixture not in FIXTURES:
        raise ValueError("unknown fixture %r" % fixture)
    config = FIXTURES[fixture]
    work_dir = Path(work_dir).resolve()
    allowed = (REPO / ".agentbench-v1-red-work").resolve()
    if work_dir != allowed and allowed not in work_dir.parents:
        raise ValueError("work directory must be below %s" % allowed)
    if work_dir.exists():
        raise FileExistsError("work directory already exists: %s" % work_dir)
    binary = engine_launch.resolve_binary(REPO)
    if binary is None:
        raise RuntimeError("a built OmniSim binary is required")
    if output is None:
        output = (HERE / "red_evidence" /
                  ("R3_pick_and_place.%s.omnisim.verdict.json" % fixture))
    output = Path(output).resolve()

    python_dir = str(Path(sys.executable).resolve().parent)
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    if python_dir.lower() not in {p.lower() for p in path_entries if p}:
        os.environ["PATH"] = python_dir + os.pathsep + os.environ.get("PATH", "")
    os.environ["WARP_CACHE_PATH"] = str(work_dir / "warp_cache")

    world, transforms, controller_sources = _stage(
        work_dir / "project", fixture)
    phase_b = headless.run_standalone(
        world,
        work_dir / "phaseB",
        duration=45.0,
        settle=1.0,
        contact_steps=5,
        solids=("table", "bin"),
        timeout_s=900.0,
        tag="phaseB",
    )
    bundle = evidence.build_bundle(
        "R3_pick_and_place",
        robot_identity="any_robot",
        artifact=str(world),
        phase_b=phase_b,
        run_dir=work_dir / "phaseB",
        scene_inventory=True,
    )
    verdict = r3_core.grade(bundle)
    failures = {a.id for a in verdict.assertions if not a.ok}
    expected = config["expected"]
    if failures != expected:
        raise RuntimeError(
            "fixture no longer discriminates as registered: expected %s, "
            "observed %s\n%s"
            % (sorted(expected), sorted(failures), verdict.summary())
        )

    source_paths = [
        SOURCE_WORLD,
        SCENE_ASSET,
        META,
        ARM_STANZA,
        Path(__file__).resolve(),
        Path(r3_core.__file__).resolve(),
        *controller_sources,
    ]
    sources = [
        {"path": str(path.relative_to(REPO)), "sha256": _sha(path)}
        for path in source_paths
    ]
    doc = {
        "schema": "agenticsimbench/red-evidence/v1",
        "suite": "agenticsimbench/v1",
        "task": "R3_pick_and_place",
        "sim": "omnisim",
        "fixture": fixture,
        "fixture_kind": "real_supervisor_cube_fixture",
        "fixture_transforms": transforms,
        "expected_failures": sorted(expected),
        "observed_failures": sorted(failures),
        "outcome": verdict.outcome,
        "qualifies_as_non_null_red_evidence": True,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "engine_binary": {
            "path": str(Path(binary).relative_to(REPO)),
            "sha256": _sha(binary),
        },
        "sources": sources,
        "staged_world_sha256": _sha(world),
        "verdict": verdict.as_dict(),
        "run": phase_b.as_dict(),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(doc, indent=2, default=str) + "\n",
                      encoding="utf-8")
    print("wrote %s (%s)" % (output, verdict.outcome))
    return doc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", choices=sorted(FIXTURES), required=True)
    parser.add_argument("--work", required=True)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    generate(args.work, args.fixture, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
