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

"""Prototype: is the XPBD substep sequence CUDA-graph-capturable?

The in-sim Newton runtime (WbNewtonBackend kNewtonRuntimeSource) CUDA-graphs
the clean-tick substep loop ONLY for the MuJoCo solver -- XPBD is excluded
(`not _solver_is_xpbd`) and also requires an even substep count. That leaves
the default XPBD path (substeps=1) paying full per-step kernel-launch overhead
every tick -- the bulk of the ~4.4 ms in-sim floor measured on a 25-omnibot
world (vs 0.19 ms for the same world on ODE).

This de-risks extending the graph to XPBD WITHOUT a binary rebuild. It builds
the same husky model the perf bench uses, then compares:

  (1) baseline : clear/collide/step/swap + per-step body_q readback (today's
                 XPBD path -- the buffers ping-pong each tick).
  (2) graphed  : capture [clear, collide, step(a->b), copy b->a] ONCE, replay
                 each tick. The in-graph copy-back keeps `a` canonical so a
                 single (odd) substep is graphable -- no even-count constraint.
  (3) correct  : assert graphed body_q matches baseline within tol after the
                 same number of steps.

If graphed is faster AND matches, the same capture+copy-back pattern goes into
the embedded runtime's _can_graph path. If capture throws or the trajectory
diverges, XPBD isn't graphable as-is and we learn it here, cheaply.

Usage:  python scripts/xpbd_probes/prototype_xpbd_cudagraph.py [N] [STEPS]
"""
# =============================================================================
# HISTORICAL (2026-07-09) — DOES NOT RUN AS-IS.
# This prototype imports bench_newton_scaling.py, which was REMOVED (the removal
# is recorded in docs/developer/engine-migration-plan.md). It is retained as a
# record of the XPBD CUDA-graph capture+copy-back prototype and its findings;
# do not expect it to execute without restoring that module.
# =============================================================================
import os
import sys
import time

import numpy as np
import warp as wp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_newton_scaling import build_n_huskies, WEBOTS_TICK_MS

DT = 1.0 / 60.0


def _readback(state):
    # Mirror WbNewtonBackend's per-tick pose sync (the base-divergence guard's
    # body_q.numpy() that also feeds the CPU scene graph).
    return state.body_q.numpy()


def run_baseline(n, steps, readback=True):
    model, s0, s1, control, contacts, solver, _ = build_n_huskies(n)
    # warmup (compile kernels)
    s0.clear_forces(); model.collide(s0, contacts)
    solver.step(s0, s1, control, contacts, DT)
    s0, s1 = s1, s0
    wp.synchronize()
    t0 = time.time()
    for _ in range(steps):
        s0.clear_forces()
        model.collide(s0, contacts)
        solver.step(s0, s1, control, contacts, DT)
        s0, s1 = s1, s0
        if readback:
            _ = _readback(s0)
    wp.synchronize()
    ms = (time.time() - t0) / steps * 1000.0
    return ms, s0.body_q.numpy()


def _state_arrays(state):
    # Every wp.array field of a State (body_q, body_qd, body_f, and for
    # generalized solvers joint_q/joint_qd, ...). Copying ALL of them back
    # makes the captured tick exactly equivalent to the ping-pong swap, with
    # no assumption about which fields the solver carries between steps.
    names = []
    for a in dir(state):
        if a.startswith("_"):
            continue
        try:
            v = getattr(state, a)
        except Exception:
            continue
        if isinstance(v, wp.array):
            names.append(a)
    return names


def run_graphed(n, steps, readback=True, copy_fields=None):
    model, s0, s1, control, contacts, solver, _ = build_n_huskies(n)
    device = model.device
    if "cuda" not in str(device).lower():
        raise RuntimeError(f"not on cuda (device={device}); graph capture needs a GPU")

    # Warmup so every kernel module is loaded before capture (capture cannot
    # JIT-compile). One real substep also seeds s0/s1 with live state.
    s0.clear_forces(); model.collide(s0, contacts)
    solver.step(s0, s1, control, contacts, DT)
    # leave s0 as the canonical "current" buffer (undo the implicit swap)
    s0, s1 = s1, s0
    wp.synchronize()

    fields = copy_fields if copy_fields is not None else _state_arrays(s1)
    fields = [f for f in fields
              if isinstance(getattr(s0, f, None), wp.array)
              and isinstance(getattr(s1, f, None), wp.array)]

    # Capture: advance s0 by one substep into s1, then copy the FULL state
    # back so s0 always holds the result and the SAME physical buffers are
    # read/written every replay (capture hard-codes buffer identity; the
    # copy-back removes the Python ping-pong that the even-substep constraint
    # exists to avoid -- making a single odd substep graphable).
    with wp.ScopedDevice(device):
        wp.synchronize()
        wp.capture_begin(force_module_load=False)
        try:
            s0.clear_forces()
            model.collide(s0, contacts)
            solver.step(s0, s1, control, contacts, DT)
            for name in fields:
                wp.copy(getattr(s0, name), getattr(s1, name))
        finally:
            graph = wp.capture_end()

    wp.synchronize()
    t0 = time.time()
    for _ in range(steps):
        with wp.ScopedDevice(device):
            wp.capture_launch(graph)
        if readback:
            _ = _readback(s0)
    wp.synchronize()
    ms = (time.time() - t0) / steps * 1000.0
    return ms, s0.body_q.numpy(), fields


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    steps = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    print(f"# XPBD CUDA-graph prototype: N={n} huskies, {steps} steps, dt={DT:.4f}")

    base_ms, base_q = run_baseline(n, steps, readback=True)
    print(f"baseline (swap + readback)      : {base_ms:7.3f} ms/step  "
          f"{1000.0/base_ms:6.1f} fps  {WEBOTS_TICK_MS/base_ms:5.2f}x rt")

    try:
        graph_ms, graph_q, fields = run_graphed(n, steps, readback=True)
    except Exception as e:
        print(f"graphed: CAPTURE FAILED -> XPBD not graphable as-is: {e!r}")
        return 1
    print(f"graphed  (capture + readback)   : {graph_ms:7.3f} ms/step  "
          f"{1000.0/graph_ms:6.1f} fps  {WEBOTS_TICK_MS/graph_ms:5.2f}x rt")
    print(f"  full-state copy-back fields    : {fields}")

    # Pure launch-overhead view: same loops without the per-step readback.
    base_nr, _ = run_baseline(n, steps, readback=False)
    graph_nr, _, _ = run_graphed(n, steps, readback=False)
    print(f"baseline (no readback)          : {base_nr:7.3f} ms/step")
    print(f"graphed  (no readback)          : {graph_nr:7.3f} ms/step")
    print(f"\nspeedup (with readback) : {base_ms/graph_ms:.2f}x")

    # GPU determinism floor: two independent baseline runs. If this is > 0 the
    # solver itself isn't bit-deterministic run-to-run, so any graph-vs-baseline
    # gap below this floor is noise, not a graphing bug.
    print("\n# divergence vs horizon (max |Δ body_q|), to separate a real")
    print("# missing-state bug from chaotic amplification of round-off:")
    print(f"#  {'H':>5} {'base-vs-base':>14} {'base-vs-graph':>14}")
    for H in (1, 10, 50, 100, steps):
        if H > steps:
            continue
        _, b1 = run_baseline(n, H, readback=False)
        _, b2 = run_baseline(n, H, readback=False)
        _, g1, _ = run_graphed(n, H, readback=False)
        bb = float(np.max(np.abs(b1 - b2)))
        bg = float(np.max(np.abs(b1 - g1)))
        print(f"   {H:>5} {bb:>14.3e} {bg:>14.3e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
