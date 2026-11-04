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

"""Engine pin: a physics step that RAISES is counted and logged (2026-11-04).

The engine-free half (sidecar reader, harness delta, diagnostics, run-headless,
the arena hook) is tests/test_physics_step_errors.py, whose docstring has the
history. Here a slab of 900 one-centimetre convex pieces (one body) drops onto
a convex floor -- hull-against-hull, as on the SparkRover assembly that found
this -- three times, 1 s each, CPU `newtonSolver "mujoco"`:

  arena                               measured 2026-11-04              must be
  MuJoCo's own (13 MiB)               rests at z 0.005, 0 failures     clean
  WorldInfo.newtonArenaMB 1           falls THROUGH the floor, every   one ARENA FULL warning
                                      step "succeeds" (contacts dropped)
  OMNISIM_NEWTON_ARENA_MB=0.25        every step raises "mj_stackAlloc:  250 step_errors in the
                                      out of memory, stack overflow at  sidecar, one ERROR line,
                                      mj_narrowphase"; never moves       the clock still at 1 s

Honours OMNISIM_BINARY, so a scratch build can be pinned without touching the
live engine. ⚠ The engine finds its resources from the binary's own path: a
scratch build that adds a WorldInfo field must be launched through THIS
checkout's msys64/mingw64/bin, or it reads another tree's WorldInfo.wrl.

    python -m pytest tests/test_physics_step_errors_engine.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

SIDE = 30  # a 30 x 30 slab of 1 cm convex pieces, one body

WORLD = """#VRML_SIM R2025a utf8
WorldInfo {{
  basicTimeStep 4
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  {arena}
}}
Viewpoint {{ orientation -0.15 0.05 0.99 1.2 position -1 -1.5 1 }}
DEF FLOOR Solid {{
  translation 0 0 -0.05
  name "floor"
  boundingObject {floor}
}}
DEF SLAB Solid {{
  translation 0 0 0.03
  name "slab"
  boundingObject Group {{ children [ {boxes} ] }}
  physics Physics {{ density -1 mass 0.4 }}
}}
DEF PROBE Robot {{ name "probe" controller "stepprobe" supervisor TRUE }}
"""

CONTROLLER = r'''
import os
from omnisim import Supervisor
sup = Supervisor()
dt = int(sup.getBasicTimeStep())
slab = sup.getFromDef("SLAB")
steps = 0
for _ in range(int(round(1000.0 / dt))):
    if sup.step(dt) == -1:
        break
    steps += 1
with open(os.environ["PROBE_OUT"], "w") as f:
    f.write("%d %.6f %.4f\n" % (steps, slab.getPosition()[2], sup.getTime()))
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


def _hull_box(hx: float, hy: float, hz: float) -> str:
    """A box as an IndexedFaceSet, which the engine registers as a convex mesh."""
    pts = ", ".join("%g %g %g" % (x * hx, y * hy, z * hz)
                    for z in (-1, 1) for (x, y) in ((-1, -1), (1, -1), (1, 1), (-1, 1)))
    return ("IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ 0 3 2 1 -1 4 5 6 7 -1 "
            "0 1 5 4 -1 1 2 6 5 -1 2 3 7 6 -1 3 0 4 7 -1 ] }" % pts)


def _boxes() -> str:
    h = (SIDE - 1) * 0.005
    piece = _hull_box(0.005, 0.005, 0.005)
    return " ".join("Pose { translation %.3f %.3f 0 children [ %s ] }"
                    % (i * 0.01 - h, j * 0.01 - h, piece) for i in range(SIDE) for j in range(SIDE))


def _run(work: Path, tag: str, arena: str = "", env_arena: str | None = None):
    root = work / tag
    (root / "worlds").mkdir(parents=True, exist_ok=True)
    (root / "controllers" / "stepprobe").mkdir(parents=True, exist_ok=True)
    world = root / "worlds" / "steperr.omniworld"
    world.write_text(WORLD.format(arena=arena, boxes=_boxes(), floor=_hull_box(1, 1, 0.05)),
                     encoding="utf-8")
    (root / "controllers" / "stepprobe" / "stepprobe.py").write_text(CONTROLLER, encoding="utf-8")
    out, log = root / "probe_out.txt", root / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(out), OMNISIM_LOG_PATH=str(log))
    env.pop("OMNISIM_NEWTON_ARENA_MB", None)
    if env_arena is not None:
        env["OMNISIM_NEWTON_ARENA_MB"] = env_arena
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(world)], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not out.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up for %r (%r)" % (tag, sig))
        pytest.fail("the %r probe produced no output.\n%s" % (tag, text[-1500:]))
    steps, z, t = out.read_text(encoding="utf-8").split()
    sidecar = Path(str(log) + ".newton.json")
    verdict = json.loads(sidecar.read_text(encoding="utf-8")) if sidecar.is_file() else {}
    return {"steps": int(steps), "z": float(z), "t": float(t), "log": text, "sidecar": verdict}


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    if _binary() is None:
        pytest.skip("no simulator binary in this clone; build first")
    work = tmp_path_factory.mktemp("steperr")
    return {"stock": _run(work, "stock"),
            "full": _run(work, "full", arena="newtonArenaMB 1"),
            "raise": _run(work, "raise", env_arena="0.25")}


def _needs_counter(r):
    if "step_errors" not in r["sidecar"]:
        pytest.skip("this engine predates the step_errors sidecar field")


@pytest.mark.engine
def test_the_stock_arena_steps_cleanly(runs):
    r = runs["stock"]
    _needs_counter(r)
    assert r["sidecar"]["step_errors"] == 0, r["sidecar"]
    assert "RAISED and was SKIPPED" not in r["log"] and "MUJOCO ARENA FULL" not in r["log"]
    assert abs(r["z"] - 0.005) < 0.003, "the slab should rest on the floor (z %.4f)" % r["z"]
    assert "MuJoCo arena:" in r["log"]


@pytest.mark.engine
def test_a_raised_step_is_counted_not_executed(runs):
    r = runs["raise"]
    _needs_counter(r)
    sc = r["sidecar"]
    assert sc["step_errors"] > 0, (
        "a 0.25 MiB arena must overflow MuJoCo's narrowphase stack under this slab; the "
        "sidecar reports no failed step: %r" % (sc,))
    assert "mj_stackAlloc" in sc["step_first_error"] and sc["step_first_error_at"] >= 1
    assert sc["runtime"].get("mj_arena_mib") == pytest.approx(0.25, abs=0.06)
    # the clock and the supervisor advanced through every skipped step -- the defect
    assert r["steps"] == 250 and r["t"] == pytest.approx(1.0, abs=0.01)
    assert abs(r["z"] - 0.03) < 1e-4, "no step ran, so the slab must not have moved (z %.4f)" % r["z"]


@pytest.mark.engine
def test_a_raised_step_is_one_error_line_then_powers_of_two(runs):
    text = runs["raise"]["log"]
    assert text.count("ERROR: [OmNewtonBackend] physics step") == 1
    assert "RAISED and was SKIPPED" in text and "newtonArenaMB" in text
    assert 1 <= text.count("step raised again") <= 12, "never one line per tick"


@pytest.mark.engine
def test_a_full_arena_that_drops_contacts_is_reported(runs):
    r = runs["full"]
    _needs_counter(r)
    assert r["sidecar"]["runtime"].get("mj_arena_mib") == pytest.approx(1.0, abs=0.05),         "WorldInfo.newtonArenaMB did not reach the compiled model"
    assert r["sidecar"]["step_errors"] == 0, "dropped contacts do not raise"
    assert r["z"] < -0.5, "with its contacts dropped the slab should fall through (z %.3f)" % r["z"]
    assert r["log"].count("MUJOCO ARENA FULL") == 1, "one WARNING per world, naming the cause"
