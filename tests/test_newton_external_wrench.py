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

"""External forces and torques on a Newton body, against analytic ground truth.

This class of test was MISSING, and its absence is exactly why a defect shipped.
A control-conversion "optimisation" skipped `_apply_mjc_control` on ticks where
the world had nothing to drive -- but that call does not merely write control,
it OVERWRITES `mj_data.ctrl` / `qfrc_applied` / `xfrc_applied` wholesale. So the
tick after a wrench was applied, nothing cleared it, and MuJoCo kept applying
the previous wrench for ever.

Measured on the shipped build: 1 N for 1 s of a 2 s run left the body at
v = 1.996 m/s and x = 1.996 m instead of 1.0 and 1.5 -- the force never stopped.
The same shape on torque: 704 rad/s against an analytic 353.

Nothing caught it. The smoke suite passed, and so did all of lane-1 T3/T6/T7,
because not one of those scenes applies an external force. Analytic ground truth
on a single free body catches it in about ten seconds, which is the whole point:
a rigid body under a known wrench in a gravity-free, contact-free world has a
closed-form answer, so there is no reference implementation to disagree with.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

#: Known intermittent embedded-interpreter bring-up failure. Since 85fa6bde a
#: world that asked for Newton is refused rather than quietly run on ODE, so
#: such a run yields no data at all -- an instrument outcome, not a physics
#: result, and reported as a skip naming the defect.
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

WORLD = """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 4
  gravity 0
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
}
Viewpoint { orientation 0 0 1 0 position -4 0 2 }
DEF BOX Solid {
  translation 0 0 2
  children [ Shape { geometry Box { size 0.4 0.2 0.1 } } ]
  name "box"
  boundingObject Box { size 0.4 0.2 0.1 }
  physics Physics { mass 1 }
}
DEF PROBE Robot { name "probe" controller "wrenchprobe" supervisor TRUE }
"""

CONTROLLER = '\n'.join([
    "import os",
    "from omnisim import Supervisor",
    "",
    "out = open(os.environ['PROBE_OUT'], 'w', buffering=1)",
    "sup = Supervisor()",
    "dt = int(sup.getBasicTimeStep())",
    "box = sup.getFromDef('BOX')",
    "mode = os.environ.get('PROBE_MODE', 'force')",
    "mag = float(os.environ.get('PROBE_MAG', '1.0'))",
    "for k in range(500):",              # 2 s total
    "    if k < 250:",                   # excite for the first second only
    "        if mode == 'force':",
    "            box.addForce([mag, 0.0, 0.0], False)",
    "        else:",
    "            box.addTorque([0.0, mag, 0.0], False)",
    "    if sup.step(dt) == -1:",
    "        break",
    "v = box.getVelocity()",
    "out.write('lin %.9f\\n' % v[0])",
    "out.write('ang %.9f\\n' % v[4])",
    "out.close()",
    "sup.simulationQuit(0)",   # exit promptly; otherwise the run waits out the timeout
    "",
])


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run(tmp_path, mode, mag):
    worlds = tmp_path / "worlds"
    ctrl = tmp_path / "controllers" / "wrenchprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / "wrench.wbt").write_text(WORLD, encoding="utf-8")
    (ctrl / "wrenchprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = tmp_path / "probe_out.txt"
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               PROBE_MODE=mode, PROBE_MAG=str(mag), OMNISIM_LOG_PATH=str(log))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "wrench.wbt")],
                       env=env, timeout=90, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    if not result.is_file():
        text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no "
                            "data, so this says nothing about the physics" % sig)
        pytest.fail("the wrench probe produced no output:\n%s" % text[-1200:])

    values = {}
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            values[parts[0]] = float(parts[1])
    return values


def test_external_force_stops_when_the_controller_stops(tmp_path):
    """F = 1 N for 1 s on 1 kg -> v = 1 m/s, and the force must then STOP."""
    v = _run(tmp_path, "force", 1.0)["lin"]
    # One basic timestep of discretisation (249/250 = 0.4%) is expected.
    # Roughly 2x is the defect: the wrench was never cleared.
    assert 0.99 <= v <= 1.01, (
        "external force wrong: v = %.6f m/s against an analytic 1.0. Close to "
        "2x means the wrench kept being applied after the controller stopped, "
        "i.e. mj_data.xfrc_applied was never cleared." % v)


def test_external_torque_matches_analytic(tmp_path):
    """tau = 5 N*m for 1 s about y, I_yy = (1/12)*m*(0.4^2 + 0.1^2)."""
    w = _run(tmp_path, "torque", 5.0)["ang"]
    expected = 5.0 / ((1.0 / 12.0) * (0.4 ** 2 + 0.1 ** 2))
    assert abs(w - expected) / expected < 0.01, (
        "external torque wrong: omega = %.3f rad/s against an analytic %.3f "
        "(ratio %.4f)" % (w, expected, w / expected))
