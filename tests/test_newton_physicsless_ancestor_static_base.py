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

"""A Robot without Physics pins what hangs under it, as the physics reference says.

docs/reference/physics.md (inherited from Webots) states two rules:
  * a Solid with Physics whose Solid ancestors all lack Physics "is attached to
    the static environment with a fixed joint";
  * a subtree of physics-less Solids under the top Solid is a "larger static
    base", and the joints hanging off it articulate from the world.

The defects this pins (2026-10-09, found on isento's pib Webots twin
2026-10-04): OmniSim 9.2.0 broke both. pib's Robot has no Physics and its one
child, urdf_body, does: OmniSim made urdf_body a FREE body, so pib dropped
22.6 mm onto its desk, slid 0.40 m during a replay and fell off the desk.
Removing urdf_body's Physics (the documented static-base form) FATALed at load
("parent body 'urdf_body' resolved to Newton but never registered").

Probe: three robots spawned 1 m above a floor, each a body with a motorised
arm on a hinge.
  A  Robot (no Physics) > body (Physics) > hinge > arm  -- the body stays put
  B  Robot (no Physics) > body (no Physics) > hinge > arm -- loads, stays put
  C  Robot (Physics) > hinge > arm                       -- control: free, falls
In A and B the arm must still follow its motor.

    python -m pytest tests/test_newton_physicsless_ancestor_static_base.py -v

OMNISIM_BINARY selects a scratch build. Three engine launches, a few seconds each.
"""

from __future__ import annotations

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
TARGET = 0.5

ARM = """HingeJoint {
        jointParameters HingeJointParameters { axis 0 1 0 }
        device [ RotationalMotor { name "m" maxTorque 50 } PositionSensor { name "s" } ]
        endPoint Solid {
          translation 0.2 0 0
          name "arm"
          boundingObject Box { size 0.2 0.04 0.04 }
          physics Physics { density -1 mass 0.3 }
        }
      }"""

BODIES = {
    # A Robot without Physics, a body WITH Physics under it.
    "A": """Robot {
  translation 0 0 1
  name "A"
  controller "probe"
  children [
    Solid {
      name "body"
      boundingObject Box { size 0.2 0.2 0.2 }
      physics Physics { density -1 mass 2 }
      children [ %s ]
    }
  ]
}""" % ARM,
    # B the same body WITHOUT Physics: a larger static base.
    "B": """Robot {
  translation 0 0 1
  name "B"
  controller "probe"
  children [
    Solid {
      name "body"
      boundingObject Box { size 0.2 0.2 0.2 }
      children [ %s ]
    }
  ]
}""" % ARM,
    # C control: the Robot itself has Physics, so it is free and must fall.
    "C": """Robot {
  translation 0 0 1
  name "C"
  controller "probe"
  boundingObject Box { size 0.2 0.2 0.2 }
  physics Physics { density -1 mass 2 }
  children [ %s ]
}""" % ARM,
}

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
Solid { translation 0 0 -0.5 name "floor" boundingObject Box { size 10 10 1 } }
%s
"""

CONTROLLER = """import os
from omnisim import Robot
r = Robot(); dt = int(r.getBasicTimeStep())
m, s = r.getDevice("m"), r.getDevice("s")
s.enable(dt)
m.setPosition(%r)
for _ in range(375):
    r.step(dt)
with open(os.environ["PROBE_OUT"], "w") as f:
    f.write("%%.9g\\n" %% s.getValue())
""" % TARGET


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run(tmp_path, case, extra_env=None):
    root = tmp_path / case
    worlds, ctrl = root / "worlds", root / "controllers" / "probe"
    worlds.mkdir(parents=True)
    ctrl.mkdir(parents=True)
    (worlds / "probe.omniworld").write_text(WORLD % BODIES[case], encoding="utf-8")
    (ctrl / "probe.py").write_text(CONTROLLER, encoding="utf-8")
    out, log, traj = root / "out.txt", root / "engine.log", root / "traj.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), PROBE_OUT=str(out),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="1600",
               **(extra_env or {}))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(worlds / "probe.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in text:
            pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
    assert "FATAL:" not in text, "case %s FATALed at load:\n%s" % (
        case, text[-2000:])
    # Lowest z the body (A, B) or the Robot root (C) reached after the first 0.1 s.
    name = "body" if case in "AB" else case
    zs = []
    for row in traj.read_text(encoding="utf-8").splitlines() if traj.is_file() else []:
        r = row.split("\t")
        if len(r) == 6 and r[1] == name and float(r[0]) >= 100.0:
            zs.append(float(r[4]))
    assert zs, "no trajectory for %r in case %s:\n%s" % (name, case, text[-1500:])
    sensor = float(out.read_text(encoding="utf-8").strip()) if out.is_file() else None
    return min(zs), sensor, text


@pytest.mark.parametrize("case", ["A", "B"])
def test_body_under_physicsless_robot_is_pinned_and_arm_moves(tmp_path, case):
    zmin, sensor, _ = _run(tmp_path, case)
    assert zmin > 0.99, "case %s: the body must stay at its 1 m spawn height; it reached z=%.3f" % (case, zmin)
    assert sensor is not None and abs(sensor - TARGET) < 0.02, (
        "case %s: the arm must still follow its motor to %.2f rad; sensor %r" % (case, TARGET, sensor))


def test_control_robot_with_physics_still_falls(tmp_path):
    zmin, sensor, _ = _run(tmp_path, "C")
    assert zmin < 0.2, "a Robot with its own Physics is free and must fall to the floor; lowest z %.3f" % zmin
