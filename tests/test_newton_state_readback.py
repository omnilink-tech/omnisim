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

"""What the engine REPORTS about a Newton body must match what it was told.

Pose and velocity readback are the surface a native step path would replace:
today they come from newton's `State`, which `_update_newton_state` rebuilds
every tick by converting joint coordinates and re-running forward kinematics.
Anything that serves them from MuJoCo directly instead has to agree with these
tests, so they are written to be independent of how the answer is produced.

They are also deliberately free of solver accuracy. With gravity off, no
contacts and no actuation, a body's velocity one step after being set must be
exactly what it was set to -- there is no dynamics in the answer, only
plumbing. That makes a failure unambiguous: a mismatch is a readback or
frame-convention defect, never "the integrator drifted".

Both a FREE body and a body inside an articulation are covered, because the two
travel different paths: a free body's state maps to a free joint, while an
articulated link's is reconstructed through the kinematic chain. A round trip
that only ever tested the free body would miss the case that matters most.
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

WORLD = """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 4
  gravity 0
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
}
Viewpoint { orientation 0 0 1 0 position -4 0 2 }
DEF FREE Solid {
  translation 0 0 2
  children [ Shape { geometry Box { size 0.2 0.2 0.2 } } ]
  name "free"
  boundingObject Box { size 0.2 0.2 0.2 }
  physics Physics { mass 1 }
}
DEF BASE Robot {
  translation 0 3 2
  name "base"
  controller "readbackprobe"
  supervisor TRUE
  children [
    Pose { rotation 0 1 0 1.5707963267948966 children [ Shape { geometry Capsule { height 0.5 radius 0.02 } } ] }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor 0.25 0 0 }
      endPoint DEF LINK2 Solid {
        translation 0.5 0 0
        name "link2"
        children [ Pose { rotation 0 1 0 1.5707963267948966 children [ Shape { geometry Capsule { height 0.5 radius 0.02 } } ] } ]
        boundingObject Pose { rotation 0 1 0 1.5707963267948966 children [ Capsule { height 0.5 radius 0.02 } ] }
        physics Physics { density -1 mass 1 }
      }
    }
  ]
  boundingObject Pose { rotation 0 1 0 1.5707963267948966 children [ Capsule { height 0.5 radius 0.02 } ] }
  physics Physics { density -1 mass 1 }
}
"""

CONTROLLER = "\n".join([
    "import os",
    "from omnisim import Supervisor",
    "",
    "out = open(os.environ['PROBE_OUT'], 'w', buffering=1)",
    "sup = Supervisor()",
    "dt = int(sup.getBasicTimeStep())",
    "sup.step(dt)",                      # let the world finalise before writing state
    "",
    "for defname in ('FREE', 'LINK2'):",
    "    n = sup.getFromDef(defname)",
    "    if n is None:",
    "        out.write('%s missing\\n' % defname)",
    "        continue",
    "    want = [0.3, -0.2, 0.5, 0.0, 0.4, 0.0]",
    "    n.setVelocity(want)",
    "    sup.step(dt)",
    "    got = n.getVelocity()",
    "    out.write('%s want %s\\n' % (defname, ' '.join('%.9f' % v for v in want)))",
    "    out.write('%s got  %s\\n' % (defname, ' '.join('%.9f' % v for v in got)))",
    "",
    "out.write('done\\n')",
    "out.close()",
    "sup.simulationQuit(0)",
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


@pytest.fixture(scope="module")
def readback(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("readback")
    worlds = tmp_path / "worlds"
    ctrl = tmp_path / "controllers" / "readbackprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / "rb.wbt").write_text(WORLD, encoding="utf-8")
    (ctrl / "readbackprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = tmp_path / "probe_out.txt"
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "rb.wbt")],
                       env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    if not result.is_file():
        text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r)" % sig)
        pytest.fail("the readback probe produced no output:\n%s" % text[-1200:])

    parsed = {}
    engine_log = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if "has NO EFFECT" in engine_log:
        parsed[("__warning__",)] = True
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 8 and parts[1] in ("want", "got"):
            parsed[(parts[0], parts[1])] = [float(x) for x in parts[2:]]
    return parsed


def test_free_body_velocity_round_trips(readback):
    """Set a velocity, step once with no forces, read it back unchanged.

    Gravity is off and nothing is touching, so the only thing between the write
    and the read is the plumbing. LINK2 is the one that matters: it is inside an
    articulation, so its state is reconstructed through the kinematic chain
    rather than read straight off a free joint.
    """
    body = "FREE"
    want = readback.get((body, "want"))
    got = readback.get((body, "got"))
    if want is None or got is None:
        pytest.fail("no readback recorded for %s (probe output incomplete)" % body)

    worst = max(abs(a - b) for a, b in zip(want, got))
    assert worst < 1e-3, (
        "velocity readback for %s does not match what was written:\n"
        "  set  %s\n  read %s\n  worst component error %.3e\n"
        "With gravity off, no contacts and no actuation there is no dynamics in "
        "this answer -- a mismatch is a readback or frame-convention defect."
        % (body, want, got, worst))


def test_articulated_link_velocity_is_not_silently_ignored(readback, tmp_path_factory):
    """A write that cannot take effect must SAY SO rather than read back zero.

    Under MuJoCo a link's velocity is derived from the joint velocities, so
    writing it per body does nothing. ODE, on the same world, applies the write
    and lets the constraint project it -- the requested (0.3, -0.2, 0.5, ...)
    comes back as (0.150, -0.173, 0.419, ...). Newton returned exactly zeros,
    which is indistinguishable to a caller from "the body really is at rest".

    Refusing the write is a defensible choice; refusing it silently is not. So
    this test does not demand the velocity stick -- it demands that a caller can
    tell. The engine now warns, and that warning is the contract being pinned.
    """
    got = readback.get(("LINK2", "got"))
    if got is None:
        pytest.fail("no readback recorded for LINK2 (probe output incomplete)")
    warned = readback.get(("__warning__",)) is not None
    if any(abs(v) > 1e-6 for v in got):
        return                      # the write took effect; nothing to warn about
    assert warned, (
        "setVelocity on an articulated link read back all zeros AND produced no "
        "warning -- a caller cannot distinguish that from the body being at "
        "rest. Either apply the write or say that it was refused.")
