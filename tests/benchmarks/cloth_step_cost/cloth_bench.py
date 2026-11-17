# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Per-step cost of OmniSim's cloth path, decomposed, driven through the REAL runtime.

WHY A STANDALONE BENCH. The engine-level question ("what realtime factor does
newton_cloth_drape reach?") is answered by running the engine, but it cannot say
WHERE the milliseconds go, and every engine run costs a process launch, a world
parse, a renderer and ~4 C of GPU heating. This drives
``src/omnisim/physics/omnisim_newton_runtime.py`` -- the same module the embedded
interpreter imports -- directly, so a decomposition costs seconds and no GPU heat
beyond the physics itself.

⚠ IT IS NOT A SUBSTITUTE FOR THE ENGINE NUMBER. It excludes the per-step particle
readback the renderer needs, controller IPC, and the engine's own tracker walk.
Quote it for ATTRIBUTION (which phase owns the cost) and quote the engine for the
realtime factor. The two are cross-checked in the report this ships with.

MEASUREMENT PROTOCOL (this tree's standing rules):
  * warm up first -- warp JIT-compiles kernels on the first touch, and the CUDA
    kernel cache is cold after any runtime version bump.
  * ``wp.synchronize()`` before and after every timed window, or a GPU phase
    reports the time of whatever the CPU happened to be doing.
  * report the MEDIAN of per-step samples, never a cumulative average -- this
    repo has a documented case of a phase reading 1.17 ms/tick cumulative and
    0.0035 ms steady-state.

Usage:
    python.exe cloth_bench.py --scene drape --steps 60
    python.exe cloth_bench.py --scene drape --solver vbd --substeps 10
    python.exe cloth_bench.py --scale            # particle-count scaling sweep
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
RUNTIME_DIR = REPO / "src" / "omnisim" / "physics"
BUNDLE_SP = REPO / "msys64" / "mingw64" / "bin" / "newton-runtime" / "site-packages"

# The runtime module first, then the bundled newton/warp/mujoco it imports.
sys.path.insert(0, str(RUNTIME_DIR))
if BUNDLE_SP.is_dir():
    sys.path.append(str(BUNDLE_SP))


def _perf():
    return time.perf_counter()


# --------------------------------------------------------------------------
# scenes -- each mirrors a shipped .omniworld so a number here is traceable to
# a world the user can actually run.
# --------------------------------------------------------------------------

SCENES = {
    # projects/samples/demos/worlds/physics/newton_cloth_drape.omniworld
    "drape": dict(
        dim_x=16, dim_y=16, cell=0.05, mass=0.001, radius=0.01,
        pos=(-0.4, -0.4, 1.0), fix_top=True,
        boxes=[  # (half extents, centre) -- static
            ((3.0, 3.0, 0.05), (0.0, 0.0, 0.0)),      # floor
            ((0.25, 0.25, 0.25), (0.0, 0.0, 0.25)),   # table
        ],
        substeps=1, solver="mujoco+vbd", dt=0.008,
    ),
    # 616-particle garment stand-in: same particle count as the T-shirt asset,
    # authored as a grid because add_cloth_mesh needs the mesh file. Particle
    # COUNT is what scales the VBD cost; topology changes the colouring only.
    "tshirt": dict(
        dim_x=27, dim_y=21, cell=0.025, mass=0.001, radius=0.01,
        pos=(-0.34, -0.26, 1.0), fix_top=True,
        boxes=[
            ((3.0, 3.0, 0.05), (0.0, 0.0, 0.0)),
            ((0.25, 0.25, 0.25), (0.0, 0.0, 0.25)),
        ],
        substeps=1, solver="mujoco+vbd", dt=0.008,
    ),
    # projects/samples/demos/worlds/physics/newton_vbd_cloth_grasp.omniworld
    "grasp": dict(
        dim_x=20, dim_y=18, cell=0.02, mass=0.10, radius=0.005,
        pos=(-0.2, -0.18, 0.4), fix_top=False,
        boxes=[((3.0, 3.0, 0.05), (0.0, 0.0, -0.05))],
        substeps=10, solver="vbd", dt=0.008,
    ),
}


def build(scene, solver=None, substeps=None, dim=None, self_contact=None,
          device=None, log=None):
    """Construct + finalize a world through the real runtime. Returns (World, meta)."""
    import omnisim_newton_runtime as rt

    cfg = dict(SCENES[scene])
    if solver:
        cfg["solver"] = solver
    if substeps is not None:
        cfg["substeps"] = substeps
    if dim is not None:
        cfg["dim_x"] = cfg["dim_y"] = dim

    if self_contact is not None:
        os.environ["OMNISIM_CLOTH_SELF_CONTACT"] = "1" if self_contact else "0"
    if device:
        os.environ["OMNISIM_NEWTON_MODEL_DEVICE"] = device
    if log:
        os.environ["OMNISIM_NEWTON_LOG"] = str(log)

    w = rt.World()
    w.set_up_axis("ENU")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_contact_solver_params(1.0, 0.0, 0.0, 0, 0)
    w.set_solver_preference(cfg["solver"])
    w.set_substeps(cfg["substeps"])

    for half, centre in cfg["boxes"]:
        b = w.add_static_body(centre[0], centre[1], centre[2])
        w.add_shape_box(b, half[0], half[1], half[2])

    p0, p1 = w.add_cloth_grid(
        cfg["pos"][0], cfg["pos"][1], cfg["pos"][2],
        0.0, 0.0, 0.0, 1.0,
        cfg["dim_x"], cfg["dim_y"], cfg["cell"], cfg["cell"],
        cfg["mass"], cfg["radius"],
        100000.0, -1.0, -1.0, 0.01, -1.0,
        False, False, bool(cfg["fix_top"]), False,
        0.0, 0.0, 0.0, "bench_sheet")

    w.finalize()
    meta = dict(scene=scene, solver=cfg["solver"], substeps=cfg["substeps"],
                particles=int(p1 - p0), dt=cfg["dt"],
                dim="%dx%d" % (cfg["dim_x"] + 1, cfg["dim_y"] + 1))
    return w, meta


def _dev(w):
    try:
        return str(w.model.device)
    except Exception:
        return "?"


def _solver_desc(w):
    s = type(getattr(w, "solver", None)).__name__
    mjc = getattr(w, "solver_mjc", None)
    if mjc is not None:
        s += "(mjc cpu=%s)" % getattr(mjc, "use_mujoco_cpu", "?")
    return s


def run(w, meta, steps, warmup=12):
    """Time `steps` steps after `warmup`. Returns per-step millisecond samples."""
    import warp as wp

    dt = meta["dt"]
    for _ in range(warmup):
        w.step(dt)
    wp.synchronize()

    samples = []
    for _ in range(steps):
        wp.synchronize()
        t0 = _perf()
        w.step(dt)
        wp.synchronize()
        samples.append((_perf() - t0) * 1e3)
    return samples


def phase_decompose(w, meta, steps=40, warmup=12):
    """Attribute the step to collide / mjc / vbd / rest by monkeypatching the
    runtime's OWN seams -- _collide and the solver entries -- so no phase is
    inferred from a code reading."""
    import warp as wp

    dt = meta["dt"]
    acc = {"collide": 0.0, "mjc": 0.0, "vbd": 0.0}
    n = {"collide": 0, "mjc": 0, "vbd": 0}

    def wrap(obj, name, key, sync=True):
        orig = getattr(obj, name)

        def f(*a, **kw):
            if sync:
                wp.synchronize()
            t0 = _perf()
            r = orig(*a, **kw)
            if sync:
                wp.synchronize()
            acc[key] += (_perf() - t0) * 1e3
            n[key] += 1
            return r
        setattr(obj, name, f)
        return orig

    for _ in range(warmup):
        w.step(dt)
    wp.synchronize()

    wrap(w, "_collide", "collide")
    if getattr(w, "solver_mjc", None) is not None:
        wrap(w.solver_mjc, "step", "mjc")
    if getattr(w, "solver_soft", None) is not None:
        wrap(w.solver_soft, "step", "vbd")

    total = 0.0
    for _ in range(steps):
        wp.synchronize()
        t0 = _perf()
        w.step(dt)
        wp.synchronize()
        total += (_perf() - t0) * 1e3

    out = {k: acc[k] / steps for k in acc}
    out["total"] = total / steps
    out["other"] = out["total"] - out["collide"] - out["mjc"] - out["vbd"]
    out["_calls_per_step"] = {k: n[k] / steps for k in n}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="drape", choices=sorted(SCENES))
    ap.add_argument("--solver", default=None)
    ap.add_argument("--substeps", type=int, default=None)
    ap.add_argument("--dim", type=int, default=None)
    ap.add_argument("--steps", type=int, default=40)
    ap.add_argument("--warmup", type=int, default=12)
    ap.add_argument("--self-contact", dest="self_contact", default=None,
                    choices=["0", "1"])
    ap.add_argument("--device", default=None)
    ap.add_argument("--decompose", action="store_true")
    ap.add_argument("--json", default=None)
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    sc = None if a.self_contact is None else (a.self_contact == "1")
    w, meta = build(a.scene, a.solver, a.substeps, a.dim, sc, a.device)
    meta["device"] = _dev(w)
    meta["solver_obj"] = _solver_desc(w)
    meta["label"] = a.label

    if a.decompose:
        d = phase_decompose(w, meta, a.steps, a.warmup)
        meta["phases_ms"] = d
        med = d["total"]
    else:
        s = run(w, meta, a.steps, a.warmup)
        med = statistics.median(s)
        meta["ms_per_step"] = dict(
            median=round(med, 4), mean=round(statistics.mean(s), 4),
            p10=round(sorted(s)[len(s) // 10], 4),
            p90=round(sorted(s)[max(0, len(s) * 9 // 10 - 1)], 4),
            n=len(s))

    meta["realtime_factor"] = round(meta["dt"] * 1e3 / med, 4) if med else None
    meta["graph_armed"] = getattr(w, "_step_graph", None) is not None
    meta["graph_failed"] = bool(getattr(w, "_graph_failed", False))

    print(json.dumps(meta, indent=2, default=str))
    if a.json:
        with open(a.json, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(meta, default=str) + "\n")


if __name__ == "__main__":
    main()
