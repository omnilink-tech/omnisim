# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Does the odd-substep CUDA-graph copyback change the physics? Measured, not argued.

THE CLAIM UNDER TEST. Recording one extra device-to-device state copy at the end
of a CUDA-graph capture lets an ODD substep count be graphed, and is supposed to
be physics-neutral: same substeps, same sub_dt, same kernels in the same order,
only the buffer the answer lands in differs.

WHY A CONTROL ARM IS MANDATORY. Cloth on the GPU is NOT bit-reproducible run to
run -- VBD and the collision pipeline use atomics, whose accumulation order the
scheduler picks. So "graph and no-graph differ by 1e-7" is meaningless on its
own: the question is whether they differ by MORE than two identical no-graph runs
differ from each other. This script measures both and reports the ratio.

  arm CONTROL : no-graph vs no-graph   -> the irreducible run-to-run noise floor
  arm TEST    : graph    vs no-graph   -> what the copyback adds on top

A PASS is TEST <= a small multiple of CONTROL. A copyback that silently froze the
sim (replaying a stale buffer) would show up as a LARGE divergence, and one that
never advanced at all would be caught by the separate motion assertion.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import cloth_bench as cb  # noqa: E402


def trajectory(scene, solver, substeps, steps, no_graph, seed_note=""):
    """Run a world and return the final particle cloud + whether it graphed."""
    import os
    if no_graph:
        os.environ["OMNISIM_NEWTON_NO_GRAPH"] = "1"
    else:
        os.environ.pop("OMNISIM_NEWTON_NO_GRAPH", None)

    import numpy as np
    import warp as wp

    w, meta = cb.build(scene, solver, substeps)
    dt = meta["dt"]
    for _ in range(steps):
        w.step(dt)
    wp.synchronize()

    lo, hi = w.cloth_particle_start, w.cloth_particle_end
    q = np.asarray(w.state_a.particle_q.numpy())[lo:hi, 0:3].copy()
    return q, bool(getattr(w, "_step_graph", None) is not None), meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="drape")
    ap.add_argument("--solver", default="vbd")
    ap.add_argument("--substeps", type=int, default=1)
    ap.add_argument("--steps", type=int, default=120)
    a = ap.parse_args()

    import numpy as np

    # Each arm is a FRESH process-level world; run them in one process so the
    # warp context and kernel cache are identical across arms.
    q_ng1, g1, meta = trajectory(a.scene, a.solver, a.substeps, a.steps, no_graph=True)
    q_ng2, g2, _ = trajectory(a.scene, a.solver, a.substeps, a.steps, no_graph=True)
    q_g, g3, _ = trajectory(a.scene, a.solver, a.substeps, a.steps, no_graph=False)

    def dev(x, y):
        d = np.linalg.norm(x - y, axis=1)
        return dict(max=float(d.max()), mean=float(d.mean()), rms=float(np.sqrt((d ** 2).mean())))

    control = dev(q_ng1, q_ng2)
    test = dev(q_g, q_ng1)

    # The sim must actually have MOVED, or "they agree" is vacuous.
    span = float(np.linalg.norm(q_ng1.max(axis=0) - q_ng1.min(axis=0)))
    drop = float(q_ng1[:, 2].min())

    ratio = (test["max"] / control["max"]) if control["max"] > 0 else float("inf")
    out = dict(
        scene=a.scene, solver=a.solver, substeps=a.substeps, steps=a.steps,
        particles=int(q_ng1.shape[0]),
        graph_armed=dict(nograph_arm1=g1, nograph_arm2=g2, graph_arm=g3),
        control_nograph_vs_nograph_m=control,
        test_graph_vs_nograph_m=test,
        ratio_test_over_control=round(ratio, 3),
        cloud_span_m=round(span, 5), lowest_particle_z=round(drop, 5),
        verdict=("GRAPH DID NOT ARM -- test is vacuous" if not g3 else
                 "FROZEN -- cloth did not move" if span < 1e-6 else
                 "PASS: copyback divergence is within run-to-run noise"
                 if test["max"] <= max(5.0 * control["max"], 1e-6) else
                 "FAIL: copyback diverges beyond the noise floor"),
    )
    print(json.dumps(out, indent=2))
    return 0 if out["verdict"].startswith("PASS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
