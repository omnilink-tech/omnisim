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

"""Phase B on the upstream-Webots column: run the world, write the shared CSV.

Reuses AgentBench's launcher wholesale -- it already injects a recorder into a
sibling world and runs Webots headless inside WSL2 -- and adds only the format
shim: ``trajectory.json`` -> the wide-format ``phaseB.csv`` that
``ladder.adapters.omnisim`` and ``ladder.adapters.mujoco`` both write.

Writing the same file from all three columns is the point. The neutral graders
then receive identical input shapes, so a verdict that differs between columns
differs because the robots did, not because three parsers did.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.adapters.webots import launcher as wb_launcher   # noqa: E402
from agentbench.adapters.webots import recording as wb_recording  # noqa: E402


class Result:
    def __init__(self, rc, note, samples=0, bodies=0, attempts_used=1):
        self.rc = rc
        self.note = note
        self.samples = samples
        self.bodies = bodies
        self.attempts_used = attempts_used

    def as_dict(self):
        return {"rc": self.rc, "note": self.note, "samples": self.samples,
                "bodies": self.bodies, "attempts_used": self.attempts_used}


def _write_wide_csv(arrays, out_csv):
    """``(n_bodies, N, 3)`` + a shared clock -> ``t, r0_x, r0_y, r0_z, ...``."""
    xyz = getattr(arrays, "xyz", None)
    t = getattr(arrays, "t", None)
    if xyz is None or t is None or len(t) == 0:
        return 0, 0
    n_bodies = len(xyz)
    n = min(len(t), min((len(b) for b in xyz), default=0))
    if n_bodies == 0 or n == 0:
        return 0, 0
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    header = ["t"]
    for i in range(n_bodies):
        header += ["r%d_x" % i, "r%d_y" % i, "r%d_z" % i]
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for k in range(n):
            row = [float(t[k])]
            for i in range(n_bodies):
                p = xyz[i][k]
                row += [float(p[0]), float(p[1]), float(p[2])]
            w.writerow(row)
    return n, n_bodies


def run_standalone(world, run_dir, *, duration=60.0, settle=0.5, attempts=2,
                   **kw):
    """Run ``world`` cold under upstream Webots and write ``phaseB.csv``.

    Never raises: a simulator that will not run a world is a measurement, and
    it comes back as a Result whose rc says so and an absent CSV, which the
    graders read as an instrument outcome rather than a column failure.
    """
    out = Path(run_dir)
    out.mkdir(parents=True, exist_ok=True)
    try:
        launched = wb_launcher.launch(str(world), str(out),
                                      duration=float(duration),
                                      attempts=int(attempts))
    except Exception as exc:                                # noqa: BLE001
        return Result(9, "the launcher raised: %r" % (exc,))

    # ``launch`` returns ``(run_dir, facts)`` -- the run directory it actually
    # used and the process.json facts. It is NOT a bare dict, and assuming it
    # was is why this path had never run: the shim was type-correct on paper
    # and wrong on contact. Smoke-tested before spending a cell on it.
    run_dir_used, facts = (launched if isinstance(launched, tuple)
                           else (out, launched))
    rc = (facts or {}).get("exit_code") if hasattr(facts, "get") else None
    read_from = Path(run_dir_used) if run_dir_used else out
    try:
        run = wb_recording.read_run(str(read_from))
        arrays = wb_recording.to_arrays(getattr(run, "trajectory", None)
                                        or getattr(run, "traj", None))
    except Exception as exc:                                # noqa: BLE001
        return Result(8, "the recording could not be read: %r" % (exc,))

    if getattr(arrays, "fatal", None):
        return Result(7, "the trajectory is unusable: %s" % arrays.fatal)

    n, bodies = _write_wide_csv(arrays, out / "phaseB.csv")
    if n == 0:
        return Result(6, "the run produced no samples (launcher rc=%s)" % rc)
    return Result(0, "ok: %d samples x %d bodies (launcher rc=%s)"
                  % (n, bodies, rc), samples=n, bodies=bodies)


def p1_run_standalone(world, run_dir, **kw):
    """P1's phase B on this column."""
    return run_standalone(world, run_dir, **kw)


def l2_run_standalone(world, run_dir, **kw):
    """L2's phase B on this column -- the same pose series."""
    return run_standalone(world, run_dir, **kw)
