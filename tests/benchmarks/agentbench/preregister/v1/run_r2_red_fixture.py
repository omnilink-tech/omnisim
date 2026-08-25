"""Generate AgenticSimBench v1 R2 red evidence with the real OmniSim engine."""

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
from agentbench.preregister.v1 import r2_core_v1 as r2_core  # noqa: E402


FIXTURE_DIR = HERE / "fixtures" / "r2"
OMNIARM6_URDF = REPO / "projects" / "robots" / "omnisim" / "omniarm6" / "omniarm6.urdf"
FIXTURES = {
    "static": {
        "world": "r2_static.wbt",
        "expected": {"R2.3", "R2.4", "R2.6"},
        "purpose": {
            "R2.3": "a stationary arm reaches none of the ordered targets",
            "R2.4": "a stationary tip does not meet the actuation travel floor",
            "R2.6": "a stationary sequence never completes",
        },
    },
    "dirty": {
        "world": "r2_kinematic.wbt",
        "expected": {"R2.1", "R2.3", "R2.4", "R2.6"},
        "purpose": {
            "R2.1": "an unknown authored field makes the real engine run dirty",
            "R2.3": "the static tip reaches none of the ordered targets",
            "R2.4": "the static tip does not meet the actuation travel floor",
            "R2.6": "the static sequence never completes",
        },
    },
    "insufficient": {
        "world": "r2_kinematic.wbt",
        "expected": {"R2.2", "R2.3", "R2.4", "R2.5", "R2.6"},
        "purpose": {
            "R2.2": "the robot carries one joint rather than the required six",
            "R2.3": "there is no qualifying arm whose target sequence can be measured",
            "R2.4": "there is no qualifying arm whose actuation can be measured",
            "R2.5": "there is no qualifying arm whose clearance can be measured",
            "R2.6": "there is no qualifying arm whose completion can be measured",
        },
    },
    "below_ground": {
        "world": "r2_kinematic.wbt",
        "expected": {"R2.3", "R2.4", "R2.5", "R2.6"},
        "purpose": {
            "R2.3": "the below-ground static tip reaches none of the targets",
            "R2.4": "the below-ground static tip is not actuated",
            "R2.5": "the selected tip is measured below the mounting datum",
            "R2.6": "the target sequence never completes",
        },
    },
    "teleport": {
        "world": "r2_kinematic.wbt",
        "expected": {"R2.4"},
        "purpose": {
            "R2.4": "instant jumps complete the targets but exceed step and speed bounds",
        },
    },
    "wrong_order": {
        "world": "r2_kinematic.wbt",
        "expected": {"R2.3", "R2.4", "R2.6"},
        "purpose": {
            "R2.3": "the tip visits target two before target one",
            "R2.4": "the wrong-order programme uses instantaneous jumps",
            "R2.6": "the required ordered sequence never completes",
        },
    },
    "late": {
        "world": "r2_kinematic.wbt",
        "expected": {"R2.6"},
        "purpose": {
            "R2.6": "a smooth valid target sequence completes after 30 seconds",
        },
    },
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
    if "__R2_MODE__" in text:
        text = text.replace("__R2_MODE__", fixture)
    if fixture == "dirty":
        text = text.replace(
            "WorldInfo {\n",
            "WorldInfo {\n  agentbenchIntentionalUnknownField 1\n",
            1,
        )
        transforms.append("added one intentional unknown WorldInfo field")
    elif fixture == "insufficient":
        spans = []
        pos = 0
        while True:
            try:
                span = _block_span(text, "HingeJoint {", pos)
            except ValueError:
                break
            spans.append(span)
            pos = span[1]
        if len(spans) != 6:
            raise ValueError("expected six fixture joints, found %d" % len(spans))
        for begin, end in reversed(spans[:5]):
            text = text[:begin] + text[end:]
        transforms.append("removed five of six joints")
    elif fixture == "below_ground":
        old = "translation 0.20 0 0.35"
        new = "translation 0.80 0 -0.20"
        if text.count(old) != 1:
            raise ValueError("could not identify the TIP translation")
        text = text.replace(old, new)
        transforms.append("placed TIP at (0.80, 0, -0.20) metres")
    return text, transforms


def _stage(dest, fixture):
    dest = Path(dest)
    source = FIXTURE_DIR / FIXTURES[fixture]["world"]
    world = dest / "worlds" / "arm_reach.wbt"
    world.parent.mkdir(parents=True, exist_ok=True)
    text = source.read_text(encoding="utf-8")
    extra_sources = []
    marker = "__R2_OMNIARM6_URDF__"
    if marker in text:
        if text.count(marker) != 1:
            raise ValueError("fixture must contain exactly one URDF marker")
        text = text.replace(
            marker, str(OMNIARM6_URDF.resolve()).replace("\\", "/"))
        extra_sources.append(OMNIARM6_URDF)
    text, transforms = _mutate(text, fixture)
    world.write_text(text, encoding="utf-8")
    controller = FIXTURE_DIR / "controllers" / "r2_fixture"
    if 'controller "r2_fixture"' in text:
        target = dest / "controllers" / "r2_fixture"
        shutil.copytree(controller, target, dirs_exist_ok=True)
        extra_sources.extend(sorted(
            p for p in controller.rglob("*")
            if p.is_file() and "__pycache__" not in p.parts
            and p.suffix != ".pyc"))
    return world, source, extra_sources, transforms


def generate(work_dir, fixture="static", output=None):
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
                  ("R2_arm_reach.%s.omnisim.verdict.json" % fixture))
    output = Path(output).resolve()

    python_dir = str(Path(sys.executable).resolve().parent)
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    if python_dir.lower() not in {p.lower() for p in path_entries if p}:
        os.environ["PATH"] = python_dir + os.pathsep + os.environ.get("PATH", "")
    os.environ["WARP_CACHE_PATH"] = str(work_dir / "warp_cache")

    world, source_world, extra_sources, transforms = _stage(
        work_dir / "project", fixture)
    phase_b = headless.run_standalone(
        world,
        work_dir / "phaseB",
        duration=r2_core.RECORD_DURATION_S,
        settle=0.0,
        contact_steps=0,
        links=12,
        timeout_s=900.0,
        tag="phaseB",
    )
    bundle = evidence.build_bundle(
        "R2_arm_reach",
        robot_identity="any_robot",
        artifact=str(world),
        phase_b=phase_b,
        run_dir=work_dir / "phaseB",
        scene_inventory=True,
    )
    verdict = r2_core.grade(bundle)
    failures = {a.id for a in verdict.assertions if not a.ok}
    expected = config["expected"]
    if failures != expected:
        raise RuntimeError(
            "fixture no longer discriminates as registered: expected %s, "
            "observed %s\n%s"
            % (sorted(expected), sorted(failures), verdict.summary())
        )

    sources = [
        {"path": str(source_world.relative_to(REPO)), "sha256": _sha(source_world)},
        {
            "path": str(Path(__file__).resolve().relative_to(REPO)),
            "sha256": _sha(Path(__file__).resolve()),
        },
        {
            "path": str(Path(r2_core.__file__).resolve().relative_to(REPO)),
            "sha256": _sha(Path(r2_core.__file__).resolve()),
        },
        {
            "path": str(r2_core.LEGACY_SOURCE.relative_to(REPO)),
            "sha256": _sha(r2_core.LEGACY_SOURCE),
        },
    ]
    for source in extra_sources:
        sources.append(
            {"path": str(source.relative_to(REPO)), "sha256": _sha(source)}
        )
    doc = {
        "schema": "agenticsimbench/red-evidence/v1",
        "suite": "agenticsimbench/v1",
        "task": "R2_arm_reach",
        "sim": "omnisim",
        "fixture": fixture,
        "fixture_kind": (
            "real_urdfrobot_engine"
            if config["world"] == "r2_static.wbt"
            else "real_supervisor_link_fixture"
        ),
        "fixture_transforms": transforms,
        "expected_failures": sorted(expected),
        "observed_failures": sorted(failures),
        "outcome": verdict.outcome,
        "qualifies_as_non_null_red_evidence": True,
        "purpose": config["purpose"],
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
    parser.add_argument("--fixture", choices=sorted(FIXTURES), default="static")
    parser.add_argument("--work", required=True)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    generate(args.work, args.fixture, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
