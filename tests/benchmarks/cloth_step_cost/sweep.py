# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Run the cloth step-cost matrix in ONE process.

Each world build costs a warp JIT pass and each process costs a ~10 s import, so
a 12-cell matrix run as 12 processes is two minutes of GPU heat for seconds of
physics. This builds every cell in one interpreter instead. Worlds are
independent (each gets its own Model/State), which the copyback verifier already
exercises three-at-a-time.
"""

from __future__ import annotations

import datetime
import json
import os
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
import cloth_bench as cb  # noqa: E402


def _machine():
    """The box these numbers belong to.

    A cloth ms/step is a hardware measurement and the rows here carried no
    machine block at all until 2026-08-17, which is how "1.52-2.82 ms/step"
    ended up in the README with nothing saying it came from a laptop 3060. If
    the fingerprint cannot be imported, the row says so rather than silently
    claiming to be from nowhere.
    """
    try:
        sys.path.insert(0, str(REPO / "tests" / "benchmarks" / "omnibench"))
        from common import results as results_mod
        return results_mod.machine_fingerprint()
    except Exception as exc:                       # noqa: BLE001
        return {"error": "machine_fingerprint unavailable: %r" % (exc,)}


def cell(scene, solver, substeps, iters=None, self_contact=None, no_graph=False,
         steps=30, note=""):
    if iters is None:
        os.environ.pop("OMNISIM_CLOTH_VBD_ITERATIONS", None)
    else:
        os.environ["OMNISIM_CLOTH_VBD_ITERATIONS"] = str(iters)
    if no_graph:
        os.environ["OMNISIM_NEWTON_NO_GRAPH"] = "1"
    else:
        os.environ.pop("OMNISIM_NEWTON_NO_GRAPH", None)

    w, meta = cb.build(scene, solver, substeps, None,
                       None if self_contact is None else self_contact)
    s = cb.run(w, meta, steps)
    med = statistics.median(s)
    return dict(scene=scene, solver=solver, substeps=substeps,
                particles=meta["particles"], iters=iters,
                self_contact=self_contact, graph=bool(getattr(w, "_step_graph", None)),
                ms=round(med, 3), rt=round(meta["dt"] * 1e3 / med, 3), note=note)


def main():
    rows = []
    which = sys.argv[1] if len(sys.argv) > 1 else "all"

    if which in ("all", "garment"):
        # THE DELIVERABLE QUESTION: does a 616-particle garment reach realtime?
        for sc in ("drape", "tshirt"):
            rows.append(cell(sc, "mujoco+vbd", 1, note="AS SHIPPED (coupled, CPU mj_step)"))
            rows.append(cell(sc, "mujoco_warp", 1, note="coupled on GPU mj + odd-substep graph"))
            rows.append(cell(sc, "vbd", 1, note="whole-world VBD + odd-substep graph"))

    if which in ("all", "selfcontact"):
        for scv in (True, False):
            rows.append(cell("drape", "mujoco_warp", 1, self_contact=scv,
                             note="self_contact=%s" % scv))

    if which in ("all", "iters"):
        for it in (2, 5, 10):
            rows.append(cell("drape", "mujoco_warp", 1, iters=it,
                             note="VBD iterations sweep (graphed)"))

    print()
    print("%-8s %-12s %-4s %-6s %-6s %-6s %9s %8s  %s" %
          ("scene", "solver", "sub", "parts", "iters", "graph", "ms/step", "realtime", "note"))
    print("-" * 118)
    for r in rows:
        print("%-8s %-12s %-4s %-6s %-6s %-6s %9.3f %7.2fx  %s" %
              (r["scene"], r["solver"], r["substeps"], r["particles"],
               r["iters"] if r["iters"] else "def", "Y" if r["graph"] else "N",
               r["ms"], r["rt"], r["note"]))
    out = Path(os.environ.get("CLOTH_SWEEP_OUT") or (HERE / "sweep_results.jsonl"))
    out.parent.mkdir(parents=True, exist_ok=True)
    machine = _machine()
    utc = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")
    with open(out, "a", encoding="utf-8") as fh:
        for r in rows:
            r["machine"] = machine
            r["utc"] = utc
            fh.write(json.dumps(r) + "\n")
    print("\n-> %s (%d rows, machine %s)"
          % (out, len(rows),
             ((machine.get("machine") or {}).get("id") or machine.get("error"))))


if __name__ == "__main__":
    main()
