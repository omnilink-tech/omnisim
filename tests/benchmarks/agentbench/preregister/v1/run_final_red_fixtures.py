"""Generate the seven remaining AgenticSimBench v1 red-evidence rows."""

from __future__ import annotations

import argparse
import dataclasses
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
REPO_FROM_HERE = HERE.parents[4]
if str(REPO_FROM_HERE) not in sys.path:
    sys.path.insert(0, str(REPO_FROM_HERE))

from agentbench import tasks  # noqa: E402
from agentbench.adapters.omnisim import evidence, headless  # noqa: E402
from agentbench.agents import a1_fixtures_extra, b2_fixtures, oracle_c2  # noqa: E402
from agentbench.common.paths import HUSKY_URDF, REPO, as_wbt_path, engine_launch  # noqa: E402
from agentbench.graders import a1_core, b2_core, b3_core, c2_core  # noqa: E402
from scripts.harness import spatial  # noqa: E402


FIXTURES = {
    "a1_unattributed": ("A1_husky_swarm_10", {"A1.10"}),
    "b2_unchanged": ("B2_subject_in_frame",
                      {"B2.1", "B2.2", "B2.4", "B2.5", "B2.6"}),
    "b2_no_claim": ("B2_subject_in_frame", {"B2.5", "B2.6"}),
    "b3_no_distance": ("B3_measure_and_report", {"B3.1", "B3.2"}),
    "b3_no_taller": ("B3_measure_and_report", {"B3.3", "B3.4"}),
    "c2_dirty": ("C2_fall_through_floor", {"C2.1"}),
    "c2_fall": ("C2_fall_through_floor", {"C2.2", "C2.4", "C2.5"}),
}
VIEW_PROBE = (HERE / "fixtures" / "final" / "controllers" /
              "agentbench_view_probe")
VIEW_SOURCE = ("live Supervisor Viewpoint fields after engine load; camera "
               "axes and per-axis FOV resolved by scripts/harness/spatial.py")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _stage_initial(task_id, dest):
    task = tasks.get(task_id)
    src = next(iter(sorted(task.initial_dir.glob("*.wbt"))))
    dest = Path(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    text = src.read_text(encoding="utf-8").replace(
        tasks.SUBSTITUTION_TOKEN, as_wbt_path(HUSKY_URDF))
    text = text.replace(
        "WorldInfo {\n", "WorldInfo {\n  newtonSolver \"mujoco\"\n", 1)
    dest.write_text(text, encoding="utf-8")
    return dest, src


def _run(world, work, task_id):
    spec = tasks.get(task_id).standalone
    return headless.run_standalone(
        world, work / "phaseB",
        duration=float(spec.get("duration_s", 1.0)),
        settle=float(spec.get("settle_s", 0.0)),
        contact_steps=int(spec.get("contact_steps", 1)),
        solids=tuple(spec.get("solids", ())), timeout_s=900.0, tag="phaseB")


def _install_view_probe(world, out_json):
    project = Path(world).parent.parent
    shutil.copytree(VIEW_PROBE, project / "controllers" /
                    "agentbench_view_probe", dirs_exist_ok=True)
    with Path(world).open("a", encoding="utf-8") as stream:
        stream.write("""
Robot {
  supervisor TRUE
  synchronization TRUE
  controller "agentbench_view_probe"
  controllerArgs [ "%s" ]
}
""" % as_wbt_path(out_json))


def _live_camera(payload):
    axes = spatial.camera_axes(payload["orientation"])
    fov = spatial.fov_axes(payload["fieldOfView"], 16.0 / 9.0)
    return b2_core.CameraPose(
        position=tuple(payload["position"]), forward=tuple(axes[0]),
        up=tuple(axes[2]), fov_h_rad=fov["fov_h_rad"],
        fov_v_rad=fov["fov_v_rad"], aspect=16.0 / 9.0,
        source=VIEW_SOURCE)


def _grade(fixture, work):
    task_id, _expected = FIXTURES[fixture]
    answer = None
    view = None
    original_attribution = None
    secondary_run = None

    if fixture == "a1_unattributed":
        world = work / "project" / "worlds" / "husky_swarm_10.wbt"
        a1_fixtures_extra._emit_ring_world(world)
        phase = _run(world, work, task_id)
        bundle = evidence.build_bundle(
            task_id, robot_identity="husky", live_expected=True,
            artifact=str(world), phase_b=phase, run_dir=work / "phaseB")
        original_attribution = (bundle.attribution.as_dict()
                                if bundle.attribution else None)
        # Deliberately wrong adapter artifact: a real attributed run whose
        # attribution record is stripped before the neutral core receives it.
        bundle.attribution = None
        verdict = a1_core.grade(bundle)
        source_extra = [Path(a1_fixtures_extra.__file__).resolve()]
        initial = None
    else:
        world, initial = _stage_initial(
            task_id, work / "project" / "worlds" /
            (task_id.split("_", 1)[1] + ".wbt"))

        if fixture == "b2_no_claim":
            spec = b2_fixtures._oracle_case()
            if not b2_fixtures.aim_camera(
                    world, spec["position"], spec["target"]):
                raise RuntimeError("could not aim staged B2 camera")
        if fixture == "c2_dirty":
            fixed, changed = oracle_c2.apply_fix(
                world.read_text(encoding="utf-8"))
            if not changed:
                raise RuntimeError("could not fix staged C2 floor")
            fixed = fixed.replace(
                "WorldInfo {\n",
                "WorldInfo {\n  agentbenchIntentionalUnknownField 1\n", 1)
            world.write_text(fixed, encoding="utf-8")

        source_extra = []

        if fixture.startswith("b2_"):
            # Probe the shipped and final Viewpoints through the same live
            # Supervisor API. This avoids the separate harness process, whose
            # engine/libController pair may be stale, without downgrading B2
            # to a source-file parser fixture.
            initial_world, _ = _stage_initial(
                task_id, work / "initial_probe" / "project" / "worlds" /
                "initial_subject_in_frame.wbt")
            initial_json = work / "initial_probe" / "viewpoint.json"
            final_json = work / "viewpoint.json"
            _install_view_probe(initial_world, initial_json)
            _install_view_probe(world, final_json)
            secondary_run = _run(initial_world, work / "initial_probe", task_id)
            phase = _run(world, work, task_id)
            if not initial_json.is_file() or not final_json.is_file():
                raise RuntimeError("live Viewpoint probe did not produce JSON")
            initial_camera = _live_camera(json.loads(
                initial_json.read_text(encoding="utf-8")))
            final_camera = _live_camera(json.loads(
                final_json.read_text(encoding="utf-8")))
            bundle = evidence.build_bundle(
                task_id, robot_identity="any_robot", artifact=str(world),
                phase_b=phase, run_dir=work / "phaseB")
            view = b2_core.ViewEvidence(
                initial=initial_camera, final=final_camera,
                artifact_parsed=True, artifact=str(world), source=VIEW_SOURCE)
            bundle.view = view
            if fixture == "b2_unchanged":
                answer = ("I measured the unchanged view: the red cylinder "
                          "is about 130.27 degrees off-axis and remains out "
                          "of frame.")
            else:
                answer = "I moved the camera to the requested subject."
            verdict = b2_core.grade(bundle, view=view, answer=answer)
            source_extra.extend([Path(b2_fixtures.__file__).resolve(),
                                 Path(spatial.__file__).resolve(),
                                 *(p for p in VIEW_PROBE.rglob("*.py"))])
        else:
            phase = _run(world, work, task_id)
            bundle = evidence.build_bundle(
                task_id, robot_identity="any_robot", artifact=str(world),
                phase_b=phase, run_dir=work / "phaseB")

        if fixture.startswith("b3_"):
            truth = b3_core.measure_ground_truth(bundle)
            if truth is None:
                raise RuntimeError("B3 ground truth unavailable")
            if fixture == "b3_no_distance":
                answer = ("husky_plinth is taller; I am deliberately "
                          "omitting the separation measurement.")
            else:
                answer = ("They are %.3f m apart centre to centre; I am "
                          "deliberately not identifying the taller body."
                          % truth["distance_m"])
            verdict = b3_core.grade(bundle, answer=answer)
        elif fixture.startswith("c2_"):
            verdict = c2_core.grade(bundle)
            source_extra.append(Path(oracle_c2.__file__).resolve())

    return {
        "task_id": task_id, "world": world, "initial": initial,
        "phase": phase, "verdict": verdict, "answer": answer, "view": view,
        "original_attribution": original_attribution,
        "source_extra": source_extra, "secondary_run": secondary_run,
    }


def generate(work_dir, fixture, output=None):
    if fixture not in FIXTURES:
        raise ValueError("unknown fixture")
    work = Path(work_dir).resolve()
    allowed = (REPO / ".agentbench-v1-red-work").resolve()
    if allowed not in work.parents:
        raise ValueError("work directory must be below %s" % allowed)
    if work.exists():
        raise FileExistsError(work)
    binary = engine_launch.resolve_binary(REPO)
    if binary is None:
        raise RuntimeError("built OmniSim binary required")
    python_dir = str(Path(sys.executable).resolve().parent)
    if python_dir.lower() not in {
        p.lower() for p in os.environ.get("PATH", "").split(os.pathsep) if p
    }:
        os.environ["PATH"] = python_dir + os.pathsep + os.environ.get("PATH", "")
    os.environ["WARP_CACHE_PATH"] = str(work / "warp_cache")

    result = _grade(fixture, work)
    verdict = result["verdict"]
    failures = {a.id for a in verdict.assertions if not a.ok}
    expected = FIXTURES[fixture][1]
    if failures != expected:
        raise RuntimeError("expected %s, observed %s\n%s" %
                           (sorted(expected), sorted(failures), verdict.summary()))

    task_id = result["task_id"]
    if output is None:
        output = HERE / "red_evidence" / (
            "%s.%s.omnisim.verdict.json" % (task_id, fixture))
    core = {"A1_husky_swarm_10": a1_core,
            "B2_subject_in_frame": b2_core,
            "B3_measure_and_report": b3_core,
            "C2_fall_through_floor": c2_core}[task_id]
    sources = [Path(__file__).resolve(), Path(core.__file__).resolve(),
               tasks.get(task_id).dir / "meta.json", *result["source_extra"]]
    if result["initial"] is not None:
        sources.append(result["initial"])
    sources = list(dict.fromkeys(Path(p).resolve() for p in sources))

    doc = {
        "schema": "agenticsimbench/red-evidence/v1",
        "suite": "agenticsimbench/v1", "task": task_id,
        "sim": "omnisim", "fixture": fixture,
        "fixture_kind": "targeted_live_adapter_evidence_fixture",
        "expected_failures": sorted(expected),
        "observed_failures": sorted(failures), "outcome": verdict.outcome,
        "qualifies_as_non_null_red_evidence": True,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "engine_binary": {"path": str(Path(binary).relative_to(REPO)),
                          "sha256": sha(binary)},
        "sources": [{"path": str(p.relative_to(REPO)), "sha256": sha(p)}
                    for p in sources],
        "staged_world_sha256": sha(result["world"]),
        "answer": result["answer"],
        "view": (dataclasses.asdict(result["view"])
                 if result["view"] is not None else None),
        "attribution_before_deliberate_strip": result["original_attribution"],
        "verdict": verdict.as_dict(), "run": result["phase"].as_dict(),
        "secondary_run": (result["secondary_run"].as_dict()
                          if result["secondary_run"] is not None else None),
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
