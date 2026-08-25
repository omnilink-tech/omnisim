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

"""oracle_dumper — generic per-step pose recorder (was the physics dual-backend oracle).

A generic supervisor that records the absolute world position of every top-level DEF'd Solid,
once per simulation step, to a CSV.

⚠️ ITS SECOND ARM NO LONGER EXISTS (2026-08-08). This controller was the recording half of a
dual-backend oracle: scripts/dev/physics_oracle.py ran the SAME world twice — once forced to ODE
(OMNISIM_LEGACY=1), once on the migration default (Newton) — and diffed the two CSVs to show where
and by how much Newton diverged from the ODE reference. **src/ode was DELETED (commit bdc02139)**,
so there is no reference arm and physics_oracle.py refuses to pretend there is. The frozen numbers
that comparison last produced are in tests/goldens/ode_oracle_goldens.json.

The controller itself is UNAFFECTED and still useful: it is a backend-agnostic trajectory recorder
(it reads poses through the supervisor API and captures whatever physics actually produced), which
is what you want for an absolute check against analytic ground truth, for a cold-vs-warm parity
diff, or for run-to-run determinism on ONE backend. tests/benchmarks/omnibench/lane3's
lane3_recorder is adapted from it and does exactly that.

Env:
  OMNISIM_ORACLE_OUT    absolute path of the CSV to write (required; falls back to ./oracle_out.csv)
  OMNISIM_ORACLE_STEPS  number of sim steps to record (default 200)

The controller is backend-agnostic: it reads poses through the supervisor API, so it captures
whatever physics actually produced — ODE or Newton — without knowing or caring which.
"""

import os

from omnisim import Supervisor


def _targets(sv):
    """Every top-level node that has a `translation` field and a DEF name (so the two runs line up
    column-for-column), excluding the dumper robot itself."""
    out = []
    children = sv.getRoot().getField("children")
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node is None:
            continue
        if node.getField("translation") is None:
            continue  # Viewpoint, WorldInfo, Background, lights, etc.
        defname = node.getDef()
        if not defname or defname == "ORACLE_DUMPER":
            continue
        out.append((defname, node))
    return out


def main():
    sv = Supervisor()
    dt = int(sv.getBasicTimeStep())
    out_path = os.environ.get("OMNISIM_ORACLE_OUT", "oracle_out.csv")
    max_steps = int(os.environ.get("OMNISIM_ORACLE_STEPS", "200"))

    targets = _targets(sv)
    cols = []
    for defname, _ in targets:
        cols += ["%s_x" % defname, "%s_y" % defname, "%s_z" % defname]

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("step," + ",".join(cols) + "\n")
        step = 0
        while step < max_steps and sv.step(dt) != -1:
            vals = []
            for _, node in targets:
                p = node.getPosition()  # absolute world position, backend-agnostic
                vals += [p[0], p[1], p[2]]
            f.write(("%d," % step) + ",".join("%.9g" % v for v in vals) + "\n")
            f.flush()
            step += 1

    sv.simulationQuit(0)


main()
