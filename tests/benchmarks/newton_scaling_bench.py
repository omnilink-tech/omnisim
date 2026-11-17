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

"""newton_scaling_bench — a RERUNNABLE Newton step-time / real-time-factor probe.

Why this exists
---------------
docs/benchmarks/performance-comparison.md §3.1 reports Newton single-world
physics numbers (ms/step, physics-fps, real-time factor). Those figures came
from a probe (scripts/xpbd_probes/bench_newton_scaling.py) that was since
*removed* — so the paper's flagship physics number had no rerunnable script,
which is exactly the "unreproducible headline" failure the same paper criticises
Genesis for (§4.8). This script closes that gap: a living, rerunnable benchmark
for the ms/step + RTF quantity.

What it measures (and how it differs from the retired probe)
------------------------------------------------------------
This reuses the *proven* GPU machinery from
projects/policies/research/tools/mjwarp_throughput_poc.py — build a
representative articulated robot in Newton's ModelBuilder, export the EXACT
MuJoCo model Newton simulates (SolverMuJoCo → MJCF), and step it under
mujoco_warp — so it is API-correct against the pinned newton==1.2.0 /
mujoco-warp==3.8.0.3 runtime.

It therefore measures the **MuJoCo-Warp solver step time** (the §3.2 / deploy
path), reported in the §3.1 *units* (ms/step, physics-fps, real-time factor vs a
60 Hz budget). It is NOT a byte-for-byte reproduction of the retired probe, which
timed the SolverXPBD path on Husky bodies — that solver/robot pairing is
recorded in the paper as a historical 2026-06-14 snapshot. Treat this as the
rerunnable successor for the step-time/RTF quantity going forward, not as a
re-measurement of the old XPBD figures.

Run
---
    python tests/benchmarks/newton_scaling_bench.py                 # default sweep
    python tests/benchmarks/newton_scaling_bench.py --worlds 1 256 4096 --json
    python tests/benchmarks/newton_scaling_bench.py --steps 500

Requires the Warp/Newton/mujoco_warp runtime and a CUDA GPU; it prints a clear
SKIP and exits 0 if they are absent (so CI on a CPU box stays green). Numbers
vary with GPU — always record the card alongside them, as the paper does.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

TICK_HZ = 60.0                     # the real-time budget the RTF is relative to
TICK_MS = 1000.0 / TICK_HZ         # 16.67 ms

REPO = next(_p for _p in Path(__file__).resolve().parents
            if (_p / "AGENTS.md").exists() or (_p / ".git").exists())
POC_TOOLS = REPO / "projects" / "policies" / "research" / "tools"


def _import_runtime():
    """Import the GPU runtime + the proven quad builder, or return the reason
    it is unavailable (so the caller can SKIP cleanly on a CPU box)."""
    sys.path.insert(0, str(POC_TOOLS))
    try:
        import numpy  # noqa: F401
        import warp as wp
        import newton
        import mujoco
        import mujoco_warp as mjw
        from newton_friction_probe import build
    except Exception as e:  # ImportError, or CUDA/driver init failure
        return None, f"{type(e).__name__}: {e}"
    return (wp, newton, mujoco, mjw, build), None


def _export_model(wp, newton, mujoco, build):
    """Build the representative robot in Newton and export the exact MuJoCo model
    it simulates (zero sim-to-sim gap), returning a loaded MjModel/MjData pair."""
    model = build(mu=2.0)[0]
    mjcf = str(Path(_tmpdir()) / "newton_scaling_export.xml")
    try:
        newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True, save_to_mjcf=mjcf)
    except TypeError:  # older positional-only signature
        newton.solvers.SolverMuJoCo(model, save_to_mjcf=mjcf)
    mjm = mujoco.MjModel.from_xml_path(mjcf)
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)
    return mjm, mjd


def _tmpdir() -> str:
    import tempfile
    d = Path(tempfile.gettempdir()) / "omnisim_newton_bench"
    d.mkdir(parents=True, exist_ok=True)
    return str(d)


def run(worlds, steps):
    rt, reason = _import_runtime()
    if rt is None:
        print(f"SKIP: Newton/Warp/mujoco_warp runtime unavailable ({reason}). "
              f"Install the pinned runtime (scripts/packaging/newton_runtime_pins.py) "
              f"and run on a CUDA GPU.")
        return None
    wp, newton, mujoco, mjw, build = rt

    wp.init()
    device = wp.get_device("cuda:0")
    print(f"newton {getattr(newton, '__version__', '?')}  warp {wp.__version__}  "
          f"mujoco {mujoco.__version__}  device {device}")

    mjm, mjd = _export_model(wp, newton, mujoco, build)
    print(f"model: nq={mjm.nq} nv={mjm.nv} nu={mjm.nu} nbody={mjm.nbody} "
          f"ngeom={mjm.ngeom}")

    rows = []
    for n in worlds:
        with wp.ScopedDevice(device):
            mw_m = mjw.put_model(mjm)
            mw_d = mjw.put_data(mjm, mjd, nworld=n)
            mjw.step(mw_m, mw_d)          # warmup + CUDA-graph compile
            wp.synchronize()
            t0 = time.perf_counter()
            for _ in range(steps):
                mjw.step(mw_m, mw_d)
            wp.synchronize()
            dt = time.perf_counter() - t0
        ms_per_step = dt / steps * 1000.0            # batched step time
        phys_fps = steps / dt                        # steps advanced per wall-second
        rtf = TICK_MS / ms_per_step                  # vs a 60 Hz tick budget
        env_steps_per_s = n * steps / dt
        rows.append({
            "worlds": n, "bodies": int(mjm.nbody) * n,
            "ms_per_step": round(ms_per_step, 3),
            "physics_fps": round(phys_fps, 1),
            "real_time_factor": round(rtf, 2),
            "env_steps_per_s": round(env_steps_per_s),
        })
        print(f"  worlds={n:5d}  {ms_per_step:7.3f} ms/step   "
              f"{phys_fps:8.1f} phys-fps   {rtf:6.2f}x RT   "
              f"{env_steps_per_s:,.0f} env-steps/s")
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worlds", type=int, nargs="+", default=[1, 2, 5, 10],
                    help="parallel-world counts to sweep (default echoes §3.1's 1/2/5/10)")
    ap.add_argument("--steps", type=int, default=200, help="timed steps per point")
    ap.add_argument("--json", action="store_true", help="emit the rows as JSON")
    args = ap.parse_args(argv)

    rows = run(args.worlds, args.steps)
    if rows is None:
        return 0  # clean skip
    if args.json:
        print(json.dumps({"tick_hz": TICK_HZ, "steps": args.steps, "rows": rows},
                         indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
