"""Generate committed v1 red evidence from real OmniSim fixtures.

Each fixture is a named, deterministic mutation of a shipped real-engine R1
world.  The runner accepts evidence only when the exact pre-registered
assertion set goes red; collateral failures are a fixture bug, not credit.
"""

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
from agentbench.graders import r1_core  # noqa: E402

LANE = (AGENTBENCH / "adapters" / "omnisim" / "omnisim_lane")
SOLIDS = ("OBSTACLE_1", "OBSTACLE_2", "OBSTACLE_3", "OBSTACLE_4",
          "OBSTACLE_5")

FIXTURES = {
    "blind": {
        "world": "r1_blind",
        "expected": {"R1.4", "R1.5", "R1.6"},
        "kind": "scripted_controller_real_engine",
        "purpose": {
            "R1.4": "a blind driver misses the goal",
            "R1.5": "a moving blind driver hits and names the blocking obstacle",
            "R1.6": "the blocked driver stops short of the shortest legal route",
        },
    },
    "dirty": {
        "world": "r1_null",
        "expected": {"R1.1", "R1.4", "R1.5", "R1.6"},
        "kind": "mutated_world_real_engine",
        "purpose": {
            "R1.1": "an unknown authored field makes the real engine run dirty",
            "R1.4": "the null driver does not reach the goal",
            "R1.5": "the null driver cannot earn collision-free motion credit",
            "R1.6": "the null driver does not produce a legal driven path",
        },
    },
    "undrivable": {
        "world": "r1_null",
        "expected": {"R1.2", "R1.4", "R1.5", "R1.6"},
        "kind": "mutated_world_real_engine",
        "purpose": {
            "R1.2": "a Robot with only one joint is not a drivable mobile robot",
            "R1.4": "an undrivable robot cannot reach the goal",
            "R1.5": "an undrivable robot cannot earn collision-free motion credit",
            "R1.6": "an undrivable robot cannot demonstrate a legal driven path",
        },
    },
    "altered_obstacle": {
        "world": "r1_null",
        "expected": {"R1.3", "R1.4", "R1.5", "R1.6"},
        "kind": "mutated_world_real_engine",
        "purpose": {
            "R1.3": "one required obstacle is displaced beyond the pose tolerance",
            "R1.4": "the null driver does not reach the goal",
            "R1.5": "the null driver cannot earn collision-free motion credit",
            "R1.6": "the null driver does not produce a legal driven path",
        },
    },
}


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _block_span(text, marker, start=0):
    """Return the balanced ``marker ... { ... }`` span."""
    begin = text.index(marker, start)
    opening = text.index("{", begin)
    depth = 0
    for pos in range(opening, len(text)):
        char = text[pos]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return begin, pos + 1
    raise ValueError("unbalanced block after %r" % marker)


def _mutate_world(text, fixture):
    if fixture == "undrivable":
        spans = []
        pos = 0
        while True:
            try:
                span = _block_span(text, "HingeJoint {", pos)
            except ValueError:
                break
            spans.append(span)
            pos = span[1]
        if len(spans) != 4:
            raise ValueError("expected four rover joints, found %d" % len(spans))
        for begin, end in reversed(spans[1:]):
            text = text[:begin] + text[end:]
        return text, "removed three of four rover joints"
    if fixture == "altered_obstacle":
        old = "DEF OBSTACLE_5 Solid {\n  translation 2.4 -2 0.25"
        new = "DEF OBSTACLE_5 Solid {\n  translation 3.6 -3.2 0.25"
        if text.count(old) != 1:
            raise ValueError("could not identify exactly one OBSTACLE_5 pose")
        return text.replace(old, new), (
            "moved OBSTACLE_5 from (2.4,-2.0) to (3.6,-3.2) metres")
    if fixture == "dirty":
        old = "WorldInfo {\n"
        new = "WorldInfo {\n  agentbenchIntentionalUnknownField 1\n"
        if text.count(old) != 1:
            raise ValueError("could not identify exactly one WorldInfo block")
        return text.replace(old, new), (
            "added one intentional unknown WorldInfo field")
    return text, None


def _mutate_controller(project, fixture):
    return None


def _stage(dest, fixture):
    """Stage and deterministically mutate one shipped R1 project."""
    dest = Path(dest)
    source = LANE / "worlds" / ("%s.wbt" % FIXTURES[fixture]["world"])
    text = source.read_text(encoding="utf-8")
    text, world_transform = _mutate_world(text, fixture)
    world = dest / "worlds" / "lidar_nav.wbt"
    world.parent.mkdir(parents=True, exist_ok=True)
    world.write_text(text, encoding="utf-8")
    controllers = []
    for line in text.splitlines():
        stripped = line.strip()
        if not (stripped.startswith("controller ") and '"' in stripped):
            continue
        name = stripped.split('"')[1]
        source_dir = LANE / "controllers" / name
        if source_dir.is_dir():
            target = dest / "controllers" / name
            shutil.copytree(source_dir, target, dirs_exist_ok=True)
            controllers.extend(sorted(p for p in target.rglob("*")
                                      if p.is_file()))
    controller_transform = _mutate_controller(dest, fixture)
    return (world, source, controllers,
            [v for v in (world_transform, controller_transform) if v])


def generate(work_dir, fixture="blind", output=None):
    work_dir = Path(work_dir).resolve()
    if fixture not in FIXTURES:
        raise ValueError("unknown fixture %r" % fixture)
    config = FIXTURES[fixture]
    if output is None:
        output = (HERE / "red_evidence" /
                  ("R1_lidar_nav.%s.omnisim.verdict.json" % fixture))
    output = Path(output).resolve()
    if REPO.resolve() in work_dir.parents or work_dir == REPO.resolve():
        # The work directory may be below the repo only when it is the narrow
        # caller-selected scratch tree, never a product or benchmark source
        # directory.  Keep the refusal list explicit.
        allowed = REPO / ".agentbench-v1-red-work"
        if work_dir != allowed.resolve() and allowed.resolve() not in work_dir.parents:
            raise ValueError("work directory inside the repo must be below %s"
                             % allowed)
    binary = engine_launch.resolve_binary(REPO)
    if binary is None:
        raise RuntimeError("a built OmniSim binary is required")
    if work_dir.exists():
        raise FileExistsError("work directory already exists: %s" % work_dir)
    # Controllers are separate child processes. A runner invoked with an
    # absolute Python path does not imply that child has `python.exe` on PATH.
    # Warp's default shared cache also races when another engine initializes
    # it. Give this evidence cell an explicit interpreter and isolated cache;
    # otherwise a broken host can look like a behavioural red fixture.
    python_dir = str(Path(sys.executable).resolve().parent)
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    if python_dir.lower() not in {p.lower() for p in path_entries if p}:
        os.environ["PATH"] = python_dir + os.pathsep + os.environ.get("PATH", "")
    os.environ["WARP_CACHE_PATH"] = str(work_dir / "warp_cache")
    world, source_world, controllers, transforms = _stage(
        work_dir / "project", fixture)
    phase_b = headless.run_standalone(
        world, work_dir / "phaseB", duration=60.0, settle=0.0,
        contact_steps=0, solids=SOLIDS, timeout_s=900.0, tag="phaseB")
    bundle = evidence.build_bundle(
        "R1_lidar_nav", robot_identity="any_robot", artifact=str(world),
        phase_b=phase_b, run_dir=work_dir / "phaseB", scene_inventory=True)
    verdict = r1_core.grade(bundle)
    failures = {a.id for a in verdict.assertions if not a.ok}
    expected = config["expected"]
    if failures != expected:
        raise RuntimeError(
            "fixture no longer discriminates as preregistered: expected %s, "
            "observed %s\n%s" % (sorted(expected), sorted(failures),
                                   verdict.summary()))
    if fixture == "blind":
        r15 = next(a for a in verdict.assertions if a.id == "R1.5")
        measured_text = json.dumps(r15.measured, sort_keys=True)
        if "OBSTACLE" not in measured_text or "rover" not in measured_text:
            raise RuntimeError(
                "R1.5 failed without naming rover/obstacle contact")

    sources = [{"path": str(source_world.relative_to(REPO)),
                "sha256": _sha(source_world)}]
    for staged in controllers:
        rel = staged.relative_to(work_dir / "project")
        original = LANE / rel
        sources.append({"path": str(original.relative_to(REPO)),
                        "sha256": _sha(original)})
    sources.append({
        "path": str(Path(__file__).resolve().relative_to(REPO)),
        "sha256": _sha(Path(__file__).resolve()),
    })
    doc = {
        "schema": "agenticsimbench/red-evidence/v1",
        "suite": "agenticsimbench/v1",
        "task": "R1_lidar_nav",
        "sim": "omnisim",
        "fixture": fixture,
        "fixture_kind": config["kind"],
        "fixture_transforms": transforms,
        "staged_world_sha256": _sha(world),
        "expected_failures": sorted(expected),
        "observed_failures": sorted(failures),
        "outcome": verdict.outcome,
        "qualifies_as_non_null_red_evidence": True,
        "purpose": config["purpose"],
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "engine_binary": {"path": str(Path(binary).relative_to(REPO)),
                          "sha256": _sha(binary)},
        "sources": sources,
        "verdict": verdict.as_dict(),
        "run": phase_b.as_dict(),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(doc, indent=2, default=str) + "\n",
                      encoding="utf-8")
    print("wrote %s (%s)" % (output, verdict.outcome))
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fixture", choices=sorted(FIXTURES), default="blind")
    ap.add_argument("--work", required=True,
                    help="new scratch directory for the real engine run")
    ap.add_argument("--output")
    args = ap.parse_args(argv)
    generate(args.work, args.fixture, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
