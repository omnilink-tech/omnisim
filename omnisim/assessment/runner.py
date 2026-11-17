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

"""Own a fresh engine per case; preserve every attempt and regrade offline."""
from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import re
import math

from omnisim.paths import REPO_ROOT, linux_runtime_env, resolve_omnisim_binary
from .schema import BrakingSpec
from .oracles import grade
from .report import write_report

WORLD = REPO_ROOT / "projects/assessments/worlds/husky_braking.omniworld"
MODEL = REPO_ROOT / "projects/robots/clearpath/husky_description"


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def source_hashes(robot="husky"):
    from .registry import get_profile
    model = get_profile(robot).model.parent.parent
    files = [REPO_ROOT / f"projects/samples/demos/worlds/chat/omnilink_{robot}.omniworld",
             REPO_ROOT / "projects/assessments/controllers/husky_braking/husky_braking.py",
             REPO_ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py"]
    if robot == "husky":
        files += [WORLD, REPO_ROOT / "projects/assessments/robots/husky.json"]
    files += sorted((REPO_ROOT / "omnisim/assessment").glob("*.py"))
    files += sorted(p for p in model.rglob("*") if p.is_file() and p.suffix != ".pyc")
    return {p.relative_to(REPO_ROOT).as_posix(): digest(p) for p in files}


def read_json(path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def regrade(directory: Path):
    spec = BrakingSpec.from_dict(read_json(directory / "spec.json"))
    try:
        rows = [json.loads(line) for line in (directory / "trace.jsonl").read_text().splitlines()]
    except (OSError, ValueError):
        rows = []
    verdict = grade(spec, rows, read_json(directory / "completion.json", {}),
                    read_json(directory / "engine.log.newton.json", {}))
    manifest = read_json(directory / "manifest.json", {})
    if spec.schema_version >= 2 and not read_json(directory / "adoption.json", {}).get("valid"):
        verdict["status"] = "ERROR"
        verdict["reasons"].append("physical_fixture_not_verified")
    for name, expected in manifest.get("artifacts_sha256", {}).items():
        path = directory / name
        if Path(name).name != name or not path.is_file() or digest(path) != expected:
            verdict["status"] = "ERROR"
            verdict["reasons"].append("artifact_hash_mismatch:" + name)
    if manifest.get("launch_error") or manifest.get("engine_exit_code") != 0 or manifest.get("cleanup_errors"):
        verdict["status"] = "ERROR"
        verdict["reasons"].append(manifest.get("launch_error") or "engine_exit_or_cleanup_error")
    return verdict


def _reap_owned(proc, children):
    import psutil
    processes = list(children.values())
    try:
        owner = psutil.Process(proc.pid)
        if proc.poll() is None:
            processes += owner.children(recursive=True) + [owner]
    except psutil.NoSuchProcess:
        pass
    # Wait for normal controller teardown first; captured Process objects retain
    # identity checks if a PID has since been reused. Never enumerate by name.
    _, processes = psutil.wait_procs(processes, timeout=1)
    errors = []
    for p in processes:
        try:
            p.terminate()
        except psutil.NoSuchProcess:
            pass
        except psutil.AccessDenied:
            errors.append(f"terminate denied for owned child {p.pid}")
    _, alive = psutil.wait_procs(processes, timeout=4)
    for p in alive:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            pass
        except psutil.AccessDenied:
            errors.append(f"kill denied for owned child {p.pid}")
    proc.wait(timeout=5)
    return errors


def runtime_hashes(binary):
    # Pin the bridge ABI and actual bundled runtime source as well as engine.
    # Windows AppData aliases/junctions can resolve outside the lexical install
    # prefix. Compare the physical path used by the module's resolved root.
    binary = Path(binary).resolve()
    candidates = [REPO_ROOT / "lib/controller/Controller.dll",
                  REPO_ROOT / "lib/controller/libController.so",
                  binary.parent / "newton-runtime/site-packages/omnisim_newton_runtime.py"]
    return {str(p.relative_to(REPO_ROOT)): digest(p) for p in candidates if p.is_file() and p.is_relative_to(REPO_ROOT)}


def run(spec: BrakingSpec, directory: Path, capture=False, parent=None, wall_timeout_s=90):
    directory = directory.resolve()
    binary = Path(os.environ.get("OMNISIM_BINARY") or resolve_omnisim_binary() or "").resolve()
    if not binary.is_file():
        raise ValueError("OmniSim binary not found; run python -m omnisim doctor")
    # Validate dependencies and reject preexisting destinations before launching anything.
    sources = source_hashes(spec.robot)
    directory.mkdir(parents=True, exist_ok=False)
    save(directory / "spec.json", spec.to_dict())
    world_path = WORLD
    fixture = None
    if spec.schema_version >= 2:
        from .scenarios import make_world
        contents, _, fixture = make_world(spec, directory)
        world_path = WORLD.with_name(f".assessment_{uuid.uuid4().hex}.omniworld")
        world_path.parent.mkdir(parents=True,exist_ok=True)
        world_path.write_text(contents, encoding="utf-8")
    else:
        (directory / "world.omniworld").write_bytes(WORLD.read_bytes())
    # Customer deliveries are relocatable folders, not necessarily Git checkouts.
    try:
        git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, capture_output=True, text=True, timeout=10)
        git_head = git.stdout.strip() if git.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        git_head = None
    env = linux_runtime_env(REPO_ROOT)
    # Do not inherit experimental physics/pose overrides into a qualified fixture.
    # Record this explicit configuration rather than dumping credentials from env.
    for key in list(env):
        if key.startswith(("OMNISIM_NEWTON_", "OMNISIM_INENGINE_", "OMNISIM_ASSESSMENT_")) or key == "OMNISIM_NO_WINDOW":
            env.pop(key)
    env.update(OMNISIM_HOME=str(REPO_ROOT), OMNISIM_LOG_PATH=str(directory / "engine.log"),
               OMNISIM_ASSESSMENT_OUT=str(directory), OMNISIM_ASSESSMENT_CAPTURE="1" if capture else "0")
    if fixture:
        env["OMNISIM_NEWTON_DUMP_MJMODEL"] = str(directory / "compiled-model.txt")
    runtime = binary.parent / "newton-runtime"
    env["PATH"] = os.pathsep.join([str(runtime), str(binary.parent), env.get("PATH", "")])
    manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                "git_head": git_head, "sources_sha256": sources,
                "binary": str(binary), "binary_sha256": digest(binary), "runtime_sha256": runtime_hashes(binary),
                "parent_case": str(parent) if parent else None, "capture": capture,
                "controller_kind": "standalone deterministic, no OmniLink/model calls",
                "world": "world.omniworld" if fixture else str(WORLD.relative_to(REPO_ROOT)),
                "runtime_settings": {"solver": "mujoco", "newtonGroundMu": spec.ground_mu, "newtonSubsteps": 8},
                "fixture": fixture,
                "motion": "wheel motor velocity only; no pose writes",
                "wall_watchdog_s": wall_timeout_s}
    save(directory / "manifest.json", manifest)
    args = [str(binary), "--batch", "--mode=fast", "--minimize", "--stdout", "--stderr"]
    if not capture:
        args.append("--no-rendering")
    args.append(str(world_path))
    proc = None
    children = {}
    started = time.monotonic()
    launch_error = None
    try:
        with (directory / "stdout.log").open("w", encoding="utf-8") as stdout:
            proc = subprocess.Popen(args, cwd=REPO_ROOT, env=env, stdout=stdout, stderr=subprocess.STDOUT,
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            manifest["owned_pid"] = proc.pid
            save(directory / "manifest.json", manifest)
            while proc.poll() is None:
                import psutil
                try:
                    for child in psutil.Process(proc.pid).children(recursive=True):
                        child.create_time()  # cache identity before the engine can exit
                        children[child.pid] = child
                except psutil.NoSuchProcess:
                    pass
                if time.monotonic() - started > wall_timeout_s:
                    launch_error = "wall_watchdog_expired"
                    break
                if (directory / "controller-error.txt").exists():
                    launch_error = "controller_error"
                    break
                time.sleep(.1)
            if proc.poll() is not None:
                manifest["engine_exit_code"] = proc.returncode
    except (OSError, subprocess.SubprocessError) as exc:
        launch_error = type(exc).__name__ + ": " + str(exc)
    finally:
        if proc is not None:
            manifest["cleanup_errors"] = _reap_owned(proc, children)
            manifest["owned_children"] = sorted(children)
        if world_path != WORLD:
            world_path.unlink(missing_ok=True)
    manifest["wall_elapsed_s"] = time.monotonic() - started
    manifest["launch_error"] = launch_error
    # Collect the existing machine/runtime fingerprint after the actual run.
    from projects.policies.common.env_fingerprint import collect
    fingerprint = collect(engine_log_path=directory / "engine.log", repo_root=REPO_ROOT, warm=False)
    save(directory / "fingerprint.json", fingerprint)
    save(directory / "manifest.json", manifest)
    if fixture:
        save(directory / "adoption.json", verify_fixture(directory, fixture))
    verdict = regrade(directory)
    manifest["artifacts_sha256"] = {p.name: digest(p) for p in directory.iterdir()
                                     if p.is_file() and p.name not in ("manifest.json", "verdict.json", "report.html", "braking.svg")}
    save(directory / "manifest.json", manifest)
    save(directory / "verdict.json", verdict)
    if (directory / "trace.jsonl").exists() and verdict["metrics"]:
        write_report(directory, [(directory.name, directory, verdict)], f"{spec.robot}: acquire and brake")
    return verdict


def rerun(case: Path, directory: Path, capture=False):
    manifest = read_json(case / "manifest.json")
    robot = read_json(case / "spec.json", {}).get("robot", "husky")
    if not manifest or manifest.get("sources_sha256") != source_hashes(robot):
        raise ValueError("Case source hashes differ from this checkout; restore the recorded sources before rerunning")
    binary = os.environ.get("OMNISIM_BINARY") or resolve_omnisim_binary()
    if not binary or digest(binary) != manifest["binary_sha256"]:
        raise ValueError("Case engine hash differs; restore the recorded engine before rerunning")
    if runtime_hashes(Path(binary)) != manifest.get("runtime_sha256"):
        raise ValueError("Case controller/runtime hashes differ; restore the recorded runtime before rerunning")
    return run(BrakingSpec.from_dict(read_json(case / "spec.json")), directory, capture, parent=case.resolve())


def verify_fixture(directory, fixture):
    """Check actual solver masses/friction, not just the generated XML."""
    path = directory / "compiled-model.txt"
    if not path.exists():
        return {"valid": False, "reason": "compiled_model_dump_missing"}
    text = path.read_text(encoding="utf-8")
    masses = [float(x) for x in re.findall(r"^body .*?mass=([\d.eE+-]+)", text, re.M)]
    floor = re.findall(r"^geom .*?body=0 .*?fric=\[([^\]]+)\].*?quat=\[([^\]]+)\]", text, re.M)
    mus = [float(x.split(',')[0]) for x,_ in floor]
    coefficients = [float(x.split(',')[0]) for x in re.findall(r"^geom .*?fric=\[([^\]]+)\]",text,re.M)]
    theta = math.radians(fixture["slope_deg"])/2
    expected_q = [math.cos(theta),0,math.sin(theta),0]
    slope_matches = all(all(abs(float(x)-y) < .0011 for x,y in zip(q.split(','),expected_q)) for _,q in floor)
    expected = fixture["expected_total_mass_kg"]
    return {"valid": bool(masses and mus) and abs(sum(masses)-expected) <= max(.02,expected*.001)
            and all(abs(x-fixture["ground_mu"]) < .0011 for x in coefficients) and slope_matches,
            "compiled_mass_kg": sum(masses), "expected_mass_kg": expected,
            "compiled_floor_friction": mus, "requested_ground_mu": fixture["ground_mu"],
            "compiled_all_geom_friction": coefficients, "slope_orientation_verified": slope_matches,
            "compiled_floor_quaternions": [q for _,q in floor]}


def prove(directory: Path, capture=False):
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    base = BrakingSpec()
    definitions = [(f"baseline-{i+1}", base, "PASS") for i in range(5)]
    definitions += [("delayed-stop", replace(base, brake_delay_s=.8), "FAIL"),
                    ("reproduced-delay", replace(base, brake_delay_s=.8), "FAIL"),
                    ("fixed-stop", base, "PASS"),
                    ("confirmation-0.5mps", replace(base, target_speed_m_s=.5), "PASS")]
    cases, results = [], []
    for name, spec, expected in definitions:
        path = directory / name
        # Rendering is explicit; do not pretend a later illustration is the original run.
        render = capture and name in ("delayed-stop", "reproduced-delay", "fixed-stop")
        verdict = (rerun(directory / "delayed-stop", path, render) if name == "reproduced-delay"
                   else run(spec, path, render))
        same_offline = regrade(path) == verdict
        results.append({"case": name, "expected": expected, "observed": verdict["status"],
                        "offline_regrade_matches": same_offline, "metrics": verdict["metrics"],
                        "reasons": verdict["reasons"]})
        cases.append((name, path, verdict))
        save(directory / "proof.json", {"complete": False, "cases": results})
        print(json.dumps(results[-1]), flush=True)
        if verdict["status"] in ("ERROR", "INCONCLUSIVE"):
            break
    checks = {}
    if len(results) == len(definitions) and all(r["metrics"] for r in results):
        metrics = {r["case"]: r["metrics"] for r in results}
        delta = abs(metrics["delayed-stop"]["max_forward_excursion_m"] - metrics["reproduced-delay"]["max_forward_excursion_m"])
        checks = {"reproduced_distance_error_m": delta, "reproduction_tolerance_m": .005,
                  "reproduction_within_tolerance": delta <= .005,
                  "fix_vs_baseline_distance_error_m": abs(metrics["fixed-stop"]["max_forward_excursion_m"] - metrics["baseline-1"]["max_forward_excursion_m"]),
                  "approach_speed_matched": abs(metrics["fixed-stop"]["initial_speed_m_s"] - metrics["delayed-stop"]["initial_speed_m_s"]) <= .005,
                  "requirement_max_stop_distance_m": base.max_stop_distance_m}
    success = (len(results) == len(definitions) and all(r["expected"] == r["observed"] and r["offline_regrade_matches"] for r in results)
               and checks.get("reproduction_within_tolerance", False) and checks.get("approach_speed_matched", False)
               and checks["fix_vs_baseline_distance_error_m"] <= .005)
    save(directory / "proof.json", {"complete": True, "verified": success, "cases": results,
         "checks": checks,
         "scope": "Controlled simulation defect, frozen requirement; not a vendor hardware failure"})
    if cases and all((p / "trace.jsonl").exists() and v["metrics"] for _, p, v in cases):
        # Plot the four central cases rather than nine overlapping traces.
        selected = [x for x in cases if x[0] in ("baseline-1", "delayed-stop", "reproduced-delay", "fixed-stop")]
        write_report(directory, selected or cases, "Husky braking: detect, reproduce, verify repair")
    return success
