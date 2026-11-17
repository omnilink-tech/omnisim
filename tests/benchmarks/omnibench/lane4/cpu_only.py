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

"""OmniBench lane 4c — does OmniSim simulate with NO CUDA DEVICE VISIBLE?

    python tests/benchmarks/omnibench/lane4/cpu_only.py

Why this exists, precisely
--------------------------
docs/developer/simulator-comparison.md carries a "hardware floor" claim: that
OmniSim runs on a CPU-only box. That claim's stated MECHANISM was "via ODE",
and ODE was deleted (bdc02139). The replacement mechanism — CPU `mj_step`, the
default solver since 2026-08-07 — is real, but the claim has not been
re-measured since the deletion, and the same sentence appears in a work-package
deliverable of an external funding commitment to a non-RTX/CPU fallback.
A third-party commitment whose stated mechanism changed and
whose replacement was never measured is the worst kind of claim to leave
standing.

What this measures, and what it does NOT
----------------------------------------
It hides every CUDA device from the engine process (`CUDA_VISIBLE_DEVICES=-1`)
and re-runs a world with a known analytic answer. That is a genuine test of
the CPU code path — warp defaults to `cuda:0` when a device exists, so with
none visible the whole Newton runtime must come up on the CPU or not at all.

It is **not** a test of GPU-less HARDWARE. The machine still has a driver, a
CUDA runtime and the GPU wheels installed; a box that never had them could
fail at import in a way this probe cannot see. The row says exactly that, and
the honest phrasing for any external claim is "runs with no CUDA device
visible to the process", never "verified on a CPU-only machine".

The third assertion is the interesting one: the no-CUDA run must produce a
trajectory IDENTICAL to the control run. If it does, the GPU was never
contributing to the default solver path in the first place — which is what
the 2026-08-09 step-cost campaign concluded from the other direction, when
pinning the model to the CPU device made the engine 2.1-3.6x faster because
newton's state had been living on the GPU while `mj_step` ran on the CPU.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OMNIBENCH = HERE.parent
REPO = HERE.parents[3]
for p in (str(HERE), str(OMNIBENCH), str(REPO)):
    if p not in sys.path:
        sys.path.insert(0, p)

import capabilities as caps                      # noqa: E402
import gen_worlds                                # noqa: E402
from common import engine_launch                 # noqa: E402
from common import results as results_mod        # noqa: E402

DEFAULT_OUT = HERE / "results" / "cpu_only.jsonl"
#: A probe world with a closed-form answer, reused rather than reinvented.
PROBE_ID = "object.rigid_box"
EXPECTED_REST_Z = 0.65


def log(msg):
    print(msg, flush=True)


def run(binpath, world, work, tag, timeout_s, hide_cuda):
    out_json = work / ("cpu_only_%s.json" % tag)
    elog = work / ("cpu_only_%s.engine.log" % tag)
    console = work / ("cpu_only_%s.console.log" % tag)
    for p in (out_json, elog, console, Path(str(elog) + ".newton.json")):
        if p.exists():
            p.unlink()
    extra = {"OMNIBENCH_OUT": str(out_json)}
    if hide_cuda:
        # -1 rather than "": an empty value is indistinguishable from unset on
        # Windows, and silently running WITH the GPU is the one failure this
        # probe must not have.
        extra["CUDA_VISIBLE_DEVICES"] = "-1"
    env = engine_launch.build_env("newton", elog, repo=REPO, extra=extra)
    t0 = time.perf_counter()
    rc, wall, timed_out = engine_launch.launch_once(
        binpath, world, env, console, timeout_s, cwd=REPO,
        extra_args=("--stdout", "--stderr"))
    verdict = engine_launch.newton_verdict(elog)
    engine, reason = engine_launch.engine_attribution(verdict)
    rec = None
    if out_json.exists():
        with open(out_json, "r", encoding="utf-8") as f:
            rec = json.load(f)
    return {"rc": rc, "wall_s": time.perf_counter() - t0,
            "timed_out": timed_out, "sidecar": verdict, "engine": engine,
            "reason": reason, "rec": rec, "engine_log": str(elog)}


def trajectory(rec):
    if not rec:
        return None
    return [list(map(float, p)) for p in (rec.get("pos_SUBJECT") or [])
            if p is not None]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--work-dir", type=Path, default=None)
    a = ap.parse_args(argv)

    binpath = engine_launch.resolve_binary(REPO)
    if binpath is None or not Path(binpath).exists():
        log("omnisim-bin not found -- build first (AGENTS.md section 2)")
        return 2
    gen_worlds.generate()
    world = gen_worlds.world_path(caps.get(PROBE_ID))
    work = a.work_dir or (HERE / "results" / "_work")
    work.mkdir(parents=True, exist_ok=True)
    out = Path(a.out)
    if out.exists():
        out.unlink()

    log("lane4c: control run (GPU visible) ...")
    ctrl = run(binpath, world, work, "control", a.timeout, hide_cuda=False)
    log("lane4c: treatment run (CUDA_VISIBLE_DEVICES=-1) ...")
    treat = run(binpath, world, work, "nocuda", a.timeout, hide_cuda=True)

    ctraj, ttraj = trajectory(ctrl["rec"]), trajectory(treat["rec"])
    rest_z = ttraj[-1][2] if ttraj else None
    identical = None
    max_dev = None
    if ctraj and ttraj and len(ctraj) == len(ttraj):
        max_dev = max(abs(c[i] - t[i])
                      for c, t in zip(ctraj, ttraj) for i in range(3))
        identical = max_dev == 0.0

    newton_ok = bool(treat["sidecar"].get("present")
                     and not treat["sidecar"].get("degraded")
                     and treat["sidecar"].get("finalised", True))
    physics_ok = rest_z is not None and abs(rest_z - EXPECTED_REST_Z) <= 0.005

    if not newton_ok:
        verdict, note = caps.BROKEN, (
            "with no CUDA device visible the Newton runtime did not finalise "
            "(sidecar=%s) -- there is no physics on this configuration"
            % json.dumps(treat["sidecar"]))
    elif not physics_ok:
        verdict, note = caps.DEGRADED, (
            "Newton finalised without CUDA but the analytic drop landed at "
            "z=%s instead of %.2f" % (rest_z, EXPECTED_REST_Z))
    elif identical is False:
        verdict, note = caps.DEGRADED, (
            "Newton runs without CUDA and the physics is right, but the "
            "trajectory differs from the GPU-visible run by %.3e m -- so the "
            "GPU was contributing something to the default solver path"
            % max_dev)
    else:
        verdict, note = caps.WORKS, None

    metrics = {
        "verdict": verdict,
        "claim": "OmniSim simulates with no CUDA device visible to the process",
        "note": note,
        "newton_finalised_without_cuda": newton_ok,
        "solver_without_cuda": treat["sidecar"].get("solver"),
        "rest_z_m": rest_z,
        "expected_rest_z_m": EXPECTED_REST_Z,
        "trajectory_identical_to_gpu_run": identical,
        "max_abs_trajectory_deviation_m": max_dev,
        "control_wall_s": ctrl["wall_s"],
        "nocuda_wall_s": treat["wall_s"],
        "control_engine": ctrl["engine"],
        "nocuda_engine": treat["engine"],
        "scope": ("CUDA devices hidden from the process; the machine still "
                  "has a driver, a CUDA runtime and the GPU wheels installed. "
                  "This is NOT a GPU-less-hardware result and must never be "
                  "quoted as one."),
    }
    row = results_mod.make_row(
        test="cpu_only_no_cuda_visible",
        engine=treat["engine"], dt_ms=4,
        metrics=metrics,
        wall_ms_per_step=float((treat["rec"] or {}).get("meta", {})
                               .get("wall_ms_per_step") or 0.0),
        steps=int((treat["rec"] or {}).get("meta", {}).get("steps") or 0),
        sim_seconds=float((treat["rec"] or {}).get("meta", {})
                          .get("sim_seconds") or 0.0),
        deviations=[d for d in (treat["reason"],) if d] + [
            "CPU-only is measured by HIDING CUDA from the process, not by "
            "running on a machine without a GPU. A clone whose GPU wheels are "
            "absent entirely could still fail at import, which this probe "
            "cannot see.",
        ],
        machine=results_mod.machine_fingerprint(
            engine_log_path=treat["engine_log"]))
    results_mod.append_row(out, row)

    log("")
    log("=" * 72)
    log("lane4c cpu-only: %s" % verdict.upper())
    log("  Newton finalised without CUDA : %s (solver=%s)"
        % (newton_ok, metrics["solver_without_cuda"]))
    log("  analytic drop rest z          : %s (expected %.2f)"
        % (rest_z, EXPECTED_REST_Z))
    log("  identical to GPU-visible run  : %s (max dev %s m)"
        % (identical, max_dev))
    if note:
        log("  note: %s" % note)
    log("  -> %s" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
