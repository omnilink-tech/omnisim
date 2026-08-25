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

"""lane3_recorder — OmniBench lane 3a trajectory recorder (SPEC.md lane 3).

Adapted from tests/physics/controllers/oracle_dumper/oracle_dumper.py, with two
changes for determinism grading:

  * records POSITION + VELOCITY (all 6 components) of every top-level DEF'd
    Solid, printed with %.17g so a double round-trips exactly — a "bitwise"
    verdict can be decided from the CSV text alone;
  * supports a two-phase COLD-vs-WARM mode: when OMNIBENCH_L3_MODE=coldwarm the
    controller writes run_w1.csv, calls worldReload() (same engine process, warm
    reload), and on its second life writes run_w2.csv and quits. Phase is
    detected by which output files already exist, since the reload restarts the
    controller process.

Env:
  OMNIBENCH_L3_OUT_DIR  directory for the CSVs (required)
  OMNIBENCH_L3_STEPS    sim steps to record (default 400)
  OMNIBENCH_L3_MODE     "single" (default; writes run.csv, quits) | "coldwarm"
"""

import os
import sys

try:  # `omnisim` is the only module name at HEAD (the `controller`
    from omnisim import Supervisor  # alias was deleted 2026-08-16)
except ImportError:  # older trees (e.g. a RunPod volume) still ship it
    from controller import Supervisor


def _targets(sv):
    """Top-level nodes with a translation field, excluding the recorder.

    ⚠️ HISTORY -- this function silently produced a FALSE POSITIVE. It used to
    skip any node without a DEF name. A 10-robot world whose robots are
    `URDFRobot { ... }` blocks with no DEF therefore recorded NOTHING but the
    sun marker, two runs of a static marker compared bitwise, and the lane
    reported "GPU physics is deterministic" while the engine's own step trace
    in the same directory showed the robots diverging by 0.33 m.

    Undeffed nodes are now recorded under a synthetic `IDX<i>_<type>` label,
    and the skip list is returned so the caller can refuse to grade a run that
    recorded nothing that moves.
    """
    out, skipped = [], []
    children = sv.getRoot().getField("children")
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node is None:
            continue
        if node.getField("translation") is None:
            skipped.append((i, "no translation field"))
            continue  # WorldInfo, Viewpoint, Background, lights...
        defname = node.getDef()
        if defname == "LANE3_RECORDER":
            continue
        if not defname:
            # Label by scene-tree index + type so the column is stable within a
            # run and comparable across runs of the SAME world.
            try:
                tname = node.getTypeName()
            except Exception:  # noqa: BLE001
                tname = "node"
            defname = "IDX%d_%s" % (i, tname)
        out.append((defname, node))
    return out, skipped


def _record(sv, out_path, max_steps):
    dt = int(sv.getBasicTimeStep())
    targets, skipped = _targets(sv)
    roster = ",".join(d for d, _ in targets)
    sys.stderr.write("[lane3_recorder] tracking %d bodies: %s\n"
                     % (len(targets), roster))
    if skipped:
        sys.stderr.write("[lane3_recorder] skipped %d non-positional nodes\n"
                         % len(skipped))
    cols = []
    for defname, _ in targets:
        cols += ["%s_%s" % (defname, c)
                 for c in ("x", "y", "z", "vx", "vy", "vz")]
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("step," + ",".join(cols) + "\n")
        step = 0
        while step < max_steps and sv.step(dt) != -1:
            vals = []
            for _, node in targets:
                p = node.getPosition()          # absolute world position
                v = node.getVelocity()          # [vx vy vz wx wy wz]
                vals += [p[0], p[1], p[2], v[0], v[1], v[2]]
            f.write(("%d," % step) + ",".join("%.17g" % x for x in vals) + "\n")
            f.flush()
            step += 1


def main():
    sv = Supervisor()
    out_dir = os.environ.get("OMNIBENCH_L3_OUT_DIR", ".")
    max_steps = int(os.environ.get("OMNIBENCH_L3_STEPS", "400"))
    mode = os.environ.get("OMNIBENCH_L3_MODE", "single")

    if mode == "coldwarm":
        w1 = os.path.join(out_dir, "run_w1.csv")
        w2 = os.path.join(out_dir, "run_w2.csv")
        done1 = os.path.join(out_dir, "run_w1.done")
        if not os.path.exists(done1):
            _record(sv, w1, max_steps)
            with open(done1, "w", encoding="utf-8") as f:
                f.write("ok\n")
            sv.worldReload()  # same process, warm reload -> phase 2
            # keep stepping until the reload tears this controller down
            while sv.step(int(sv.getBasicTimeStep())) != -1:
                pass
        else:
            _record(sv, w2, max_steps)
            sv.simulationQuit(0)
    else:
        _record(sv, os.path.join(out_dir, "run.csv"), max_steps)
        sv.simulationQuit(0)


main()
