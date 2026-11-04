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

"""A Solid with no inertiaMatrix keeps its centre of mass where it is (2026-11-03).

THE DEFECT. With `inertiaMatrix` empty, the Newton path composes the body's tensor
from its boundingObject ABOUT THE SOLID ORIGIN and, until this fix, passed it with
the COM pinned AT the origin -- the declared `centerOfMass`, or the boundingObject's
own centroid, never reached the solver (MuJoCo `ipos` 0). Only an explicit
inertiaMatrix (and the URDF_INERTIA_FALLBACK marker) carried its COM. So a part
whose origin is off its material had its weight acting at the origin: found by the
desk-robot assembly (`projects/manufacturing/products/desk_robot/assemble.py`),
where a ball caster, gearmotors, a PCB and standoffs, built in the robot's frame,
tumbled off flat seats they should have rested on.

The probe is a 40 x 40 x 20 mm, 0.1 kg box on a static floor, its shape 30 mm off
its Solid origin along +x, `centerOfMass [ 0.03 0 0 ]`, no inertiaMatrix. 1 s:

  case                                       box centre drift   rotation   ipos
  origin at the box centre (reference)       0.05 mm            0 deg      0
  offset, fixed (default)                    0.05 mm            0 deg      0.03
  offset, OMNISIM_NEWTON_GEOM_INERTIA_COM=0  30.6 mm            86 deg     0

Two engine launches, ~10 s each, CPU `newtonSolver "mujoco"`. Honours
OMNISIM_BINARY, so a scratch build can be pinned without touching the live engine.

    python -m pytest tests/test_newton_geom_inertia_com.py -v
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
HATCH = "OMNISIM_NEWTON_GEOM_INERTIA_COM"

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

OFF = 0.03
CASES = (("CENTRED", 0.0, "density -1 mass 0.1"),
         ("OFFSET", OFF, "density -1 mass 0.1 centerOfMass [ %r 0 0 ]" % OFF))

SOLID = """DEF {d} Solid {{
  translation {x!r} 0 0.0101
  name "{n}"
  children [ Pose {{ translation {o!r} 0 0 children [ Shape {{ geometry Box {{ size 0.04 0.04 0.02 }} }} ] }} ]
  boundingObject Pose {{ translation {o!r} 0 0 children [ Box {{ size 0.04 0.04 0.02 }} ] }}
  physics Physics {{ {phys} }}
}}
"""

WORLD = """#VRML_SIM R2025a utf8
WorldInfo {{
  basicTimeStep 4
  gravity 9.81
  coordinateSystem "ENU"
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonStatics TRUE
}}
Viewpoint {{ orientation -0.15 0.05 0.99 1.2 position -2 -3 2 }}
DEF FLOOR Solid {{
  translation 0 0 -0.05
  name "floor"
  children [ DEF FLOOR_SHAPE Shape {{ geometry Box {{ size 4 2 0.1 }} }} ]
  boundingObject USE FLOOR_SHAPE
}}
{solids}
DEF PROBE Robot {{ name "probe" controller "comprobe" supervisor TRUE }}
"""

CONTROLLER = r'''
import math, os
from omnisim import Supervisor
CASES = %r
out = open(os.environ['PROBE_OUT'], 'w', buffering=1)
sup = Supervisor()
dt = int(sup.getBasicTimeStep())
def centre(n, o):
    p = n.getPosition(); R = n.getOrientation()
    return [p[i] + R[3 * i] * o for i in range(3)], R
nodes = {d: sup.getFromDef(d) for d, _ in CASES}
sup.step(dt)
start = {d: centre(nodes[d], o) for d, o in CASES}
for _ in range(int(round(1000.0 / dt))):
    if sup.step(dt) == -1:
        break
for d, o in CASES:
    c1, R1 = centre(nodes[d], o)
    c0, R0 = start[d]
    tr = sum(R0[k] * R1[k] for k in range(9))
    rot = math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))
    out.write('%%s %%.6f %%.6f\n' %% (d, 1000.0 * math.dist(c0, c1), rot))
out.close()
sup.simulationQuit(0)
'''


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override and Path(override).is_file():
        return Path(override)
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run(work: Path, tag: str, hatch: str | None = None):
    """One engine launch. Returns {def: (drift_mm, rot_deg)} and {body: ipos}."""
    root = work / tag
    (root / "worlds").mkdir(parents=True, exist_ok=True)
    (root / "controllers" / "comprobe").mkdir(parents=True, exist_ok=True)
    solids = "".join(SOLID.format(d=d, n=d.lower(), x=-1.0 + 1.0 * i - o, o=o, phys=phys)
                     for i, (d, o, phys) in enumerate(CASES))
    world = root / "worlds" / "com.wbt"
    world.write_text(WORLD.format(solids=solids), encoding="utf-8")
    (root / "controllers" / "comprobe" / "comprobe.py").write_text(
        CONTROLLER % ([(d, o) for d, o, _ in CASES],), encoding="utf-8")
    result, log, dump = root / "probe_out.txt", root / "engine.log", root / "mjmodel.txt"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log), OMNISIM_NEWTON_DUMP_MJMODEL=str(dump))
    for var in (HATCH, "OMNISIM_NEWTON_USE_LINK_COM", "OMNISIM_NEWTON_LEGACY_INERTIA_PRESET"):
        env.pop(var, None)
    if hatch is not None:
        env[HATCH] = hatch
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(world)], env=env, timeout=180, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not result.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up for %r (%r)" % (tag, sig))
        pytest.fail("the %r probe produced no output.\n%s" % (tag, text[-1500:]))
    motion = {}
    for line in result.read_text(encoding="utf-8").splitlines():
        d, drift, rot = line.split()
        motion[d] = (float(drift), float(rot))
    ipos = {}
    if dump.is_file():
        for m in re.finditer(r"^body \d+ (body_\d+)\s+mass=\S+ ipos=\[([^\]]*)\]",
                             dump.read_text(encoding="utf-8", errors="replace"), re.M):
            ipos[m.group(1)] = [float(v) for v in m.group(2).split(",")]
    return motion, ipos, text


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    work = tmp_path_factory.mktemp("geomcom")
    return {"default": _run(work, "default"), "hatch": _run(work, "hatch", hatch="0")}


def test_offset_origin_body_rests_like_a_centred_one(runs):
    motion, _, _ = runs["default"]
    drift, rot = motion["OFFSET"]
    assert rot < 0.5 and drift < 1.0, (
        "a box whose shape sits 30 mm off its Solid origin, centerOfMass declared at "
        "the shape centre and no inertiaMatrix, moved %.2f mm and turned %.1f deg in "
        "1 s on a flat floor (the centred reference: %r). Its COM did not reach the "
        "solver -- it was pinned at the origin, outside the box's own footprint."
        % (drift, rot, motion["CENTRED"]))


def test_solver_sees_the_com(runs):
    _, ipos, text = runs["default"]
    if not ipos:
        pytest.skip("OMNISIM_NEWTON_DUMP_MJMODEL wrote no body lines on this build")
    # body_0 = CENTRED, body_1 = OFFSET (registration order)
    assert ipos["body_0"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert ipos["body_1"] == pytest.approx([OFF, 0.0, 0.0], abs=1e-6), ipos
    assert "geometry COM: 1 body(ies)" in text, "the load log no longer reports the census line"


def test_hatch_zero_restores_the_com_at_the_origin(runs):
    motion, ipos, _ = runs["hatch"]
    assert motion["OFFSET"][1] > 30.0, (
        "OMNISIM_NEWTON_GEOM_INERTIA_COM=0 must restore the pre-fix COM at the origin, "
        "under which this box tips off its seat; it turned only %.1f deg" % motion["OFFSET"][1])
    if ipos:
        assert ipos["body_1"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
