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

"""Launch the OmniQuad Newton demo headless and report what Newton did.

Pulls the trained policy from omniquad_omnisim_main (the ODE-validated walker
we shipped); whether it tracks well under Newton's velocity actuator is a
separate question, but the demo's job is to verify the Newton backend
actually loads OmniQuad's articulation (the multi-parent bug fix) and steps
without crashing.

Run from repo root:
    python projects/policies/research/tools/_run_newton_demo.py --duration 15 --view
"""
from __future__ import annotations
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
WORLD = REPO_ROOT / "projects" / "rl" / "worlds" / "omniquad_newton_demo.omniworld"
OMNISIM_BIN = REPO_ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
OMNISIM_GUI = REPO_ROOT / "msys64" / "mingw64" / "bin" / "webots.exe"
POLICY = REPO_ROOT / "projects" / "rl" / "inference" / "policies" / "omniquad_omnisim_main" / "policy.onnx"
LOG_DIR = REPO_ROOT / ".build_tmp"
SOLVER_LOG = LOG_DIR / "newton_solver.log"


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=10.0,
                   help="wall-clock seconds to let OmniSim run")
    p.add_argument("--view", action="store_true",
                   help="open GUI instead of headless")
    p.add_argument("--force-mujoco", action="store_true",
                   help="OMNISIM_NEWTON_FORCE_MUJOCO=1 (skip XPBD)")
    args = p.parse_args()

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    # Truncate the solver log so we know which lines belong to *this* run.
    if SOLVER_LOG.exists():
        SOLVER_LOG.unlink()

    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["OMNIQUAD_POLICY_ONNX"] = str(POLICY)
    env["OMNISIM_POLICY_ONNX"] = str(POLICY)
    env["OMNIQUAD_VX"] = "0.5"
    env["OMNIQUAD_VY"] = "0.0"
    env["OMNIQUAD_WZ"] = "0.0"
    env["OMNIQUAD_DEPLOY_TRACE"] = "1"
    env["OMNISIM_DEPLOY_TRACE"] = "1"
    env["OMNISIM_NEWTON_LOG"] = str(SOLVER_LOG)
    # Make OmUrdfImporter emit each URDF link's <inertia> tensor into
    # Webots' Physics.inertiaMatrix field. Off by default upstream
    # because it can crash ODE on small-mass child links; the Newton
    # path doesn't share that ODE failure mode.
    env["OMNISIM_URDF_USE_INERTIA"] = "1"
    # Default wrapper-placeholder behaviour: chassis is a 1mm sphere that
    # doesn't carry weight; ground contact happens through the descendant
    # foot spheres. Switching to wrapper-uses-own-shape makes the chassis
    # box overlap the in-spawn hip bodies (hip world position is INSIDE
    # the chassis envelope) -> XPBD penetration -> divergence. The
    # leg-segment mesh-AABB fix below is what gives OmniQuad proper ground
    # contact; the wrapper stays a placeholder for now.
    if args.force_mujoco:
        env["OMNISIM_NEWTON_FORCE_MUJOCO"] = "1"
    # Dump every controller stderr line to a file the harness can read.
    env["OMNIQUAD_DEPLOY_LOG"] = str(LOG_DIR / "omniquad_controller.log")
    print(f"[demo] world      = {WORLD.relative_to(REPO_ROOT)}")
    print(f"[demo] policy     = {POLICY.relative_to(REPO_ROOT)}")
    print(f"[demo] solver_log = {SOLVER_LOG.relative_to(REPO_ROOT)}")
    print(f"[demo] solver     = {'MuJoCo (forced)' if args.force_mujoco else 'XPBD primary'}")

    bin_path = OMNISIM_GUI if args.view else OMNISIM_BIN
    cmd = [str(bin_path), str(WORLD)]
    if not args.view:
        cmd += ["--batch", "--mode=fast", "--no-rendering", "--minimize",
                "--stdout", "--stderr"]
    stdout_capture = LOG_DIR / "omnisim_stdout.log"
    stderr_capture = LOG_DIR / "omnisim_stderr.log"
    print(f"[demo] omnisim stdout -> {stdout_capture.relative_to(REPO_ROOT)}")
    print(f"[demo] omnisim stderr -> {stderr_capture.relative_to(REPO_ROOT)}")

    t0 = time.time()
    with open(stdout_capture, "w") as fout, open(stderr_capture, "w") as ferr:
        proc = subprocess.Popen(cmd, env=env, stdout=fout, stderr=ferr,
                                cwd=str(REPO_ROOT))
        deadline = t0 + args.duration + 8
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            time.sleep(0.5)
        if proc.poll() is None:
            try:
                import psutil
                parent = psutil.Process(proc.pid)
                for c in parent.children(recursive=True):
                    try: c.kill()
                    except Exception: pass
                parent.kill()
            except Exception:
                proc.terminate()
        try:
            proc.wait(timeout=4)
        except Exception:
            pass
    elapsed = time.time() - t0
    print(f"[demo] omnisim exited after {elapsed:.1f}s wall")

    # Surface the solver log (the artifact that tells us Newton was used).
    print("\n=== newton_solver.log ===")
    if SOLVER_LOG.exists():
        print(SOLVER_LOG.read_text())
    else:
        print("(no log written — Newton init likely failed or fell back to ODE)")

    # And the tail of omnisim_stderr.log so any Python error is visible.
    print("\n=== omnisim stderr (last 40 lines) ===")
    if stderr_capture.exists():
        lines = stderr_capture.read_text(errors="replace").splitlines()
        for L in lines[-40:]:
            print(L)
    else:
        print("(no stderr captured)")

    print("\n=== omnisim stdout (Newton-relevant lines) ===")
    if stdout_capture.exists():
        for L in stdout_capture.read_text(errors="replace").splitlines():
            if "WbNewton" in L or "newton" in L.lower() or "Body" in L:
                print(L)

    return 0


if __name__ == "__main__":
    sys.exit(main())
