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

"""The Newton raycast service must answer Receiver IR occlusion like ODE did.

WHY THIS EXISTS
---------------
The infra-red Receiver drops a packet when the emitter->receiver segment is
blocked, answered under ODE by a ray geom per pending packet (OmReceiver
Transmission). The Newton replacement re-answers the same segment via
OmNewtonBackend::raycastBatch (mj_ray over the LIVE mjModel), excluding both
endpoint robots' Newton bodies, behind the value-parsed OMNISIM_NEWTON_RAYCAST
gate.

THE MEASUREMENT
---------------
One generated world, two IR emitter/receiver pairs on two body-less robots
(newtonStatics TRUE so the wall exists):

    channel 1: emitter (0,0,0.5)  -> receiver (2,0,0.5)  -- clear path
    channel 2: emitter (0,2,0.5)  -> receiver (2,2,0.5)  -- wall at x=1

THE ODE ARM IS GONE
-------------------
This test used to run OMNISIM_FORCE_ODE=1 as a live oracle beside the Newton arm
and require the delivered/dropped verdicts to match. src/ode is being deleted,
so the ODE arm has been DELETED from this file. Its verdicts were measured first
and frozen into

    tests/goldens/ode_oracle_goldens.json  ->  families.receiver_occlusion

(measured: rx_clear delivered, rx_blocked dropped). The frozen verdicts and the
ABSOLUTE expectation are asserted separately, even though they coincide here --
the absolute one is the physics and does not depend on either backend.

    python -m pytest tests/test_newton_receiver_occlusion_parity.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"

#: receiver -> should packets arrive? (absolute, backend-independent)
EXPECTED = {"rx_clear": True, "rx_blocked": False}

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["receiver_occlusion"]["measurements"]


def _world_text():
    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_receiver_occlusion_parity.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  gravity 0
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 1 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF EMITTER_BOT Robot {
  translation 0 0 0
  name "emitter_bot"
  controller "receiver_occlusion_probe"
  children [
    Emitter { translation 0 0 0.5 name "em_clear" type "infra-red" channel 1 }
    Emitter { translation 0 2 0.5 name "em_blocked" type "infra-red" channel 2 }
  ]
}
DEF RECEIVER_BOT Robot {
  translation 2 0 0
  name "receiver_bot"
  controller "receiver_occlusion_probe"
  children [
    Receiver { translation 0 0 0.5 name "rx_clear" type "infra-red" channel 1 }
    Receiver { translation 0 2 0.5 name "rx_blocked" type "infra-red" channel 2 }
  ]
}
DEF WALL Solid {
  translation 1 2 0.5
  name "wall"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.3 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.2 1 1 }
    }
  ]
  boundingObject Box { size 0.2 1 1 }
}
"""


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(tmp_path, attempt):
    """Run the Newton arm. -> {receiver: received bool} or None (bring-up flake)."""
    world = WORLDS / ".receiver_occlusion_parity.wbt"
    world.write_text(_world_text(), encoding="utf-8")
    out = tmp_path / ("probe_newton_%d.json" % attempt)
    log = tmp_path / ("engine_newton_%d.log" % attempt)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_RECEIVER_PROBE_OUT"] = str(out)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_NEWTON_RAYCAST"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(120):          # the probe writes after ~24 ticks; poll
            if out.exists():
                break
            try:
                proc.wait(timeout=1)
                break
            except subprocess.TimeoutExpired:
                continue
    finally:
        if proc.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                               capture_output=True)
            else:
                proc.kill()
            proc.wait()
        try:
            world.unlink()
        except OSError:
            pass
    blob = log.read_text(errors="replace") if log.exists() else ""
    if any(sig in blob for sig in _BRINGUP):
        return None
    if not out.exists():
        pytest.fail("the Newton arm produced no probe output\n%s" % blob[-1200:])
    return {k: v["received"] for k, v in json.loads(out.read_text()).items()}


def _run_with_retry(tmp_path):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(tmp_path, attempt)
        if got is not None:
            return got
    return None


def test_newton_receiver_occlusion_matches_frozen_ode_goldens(tmp_path):
    newton = _run_with_retry(tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    goldens = _goldens()

    problems = []
    for name, expected in EXPECTED.items():
        n = newton.get(name)
        if n is None:
            problems.append("%s: missing from the Newton probe output" % name)
            continue
        # (1) absolute IR-occlusion physics -- backend-independent
        if n != expected:
            problems.append("%s: Newton received=%s, expected %s" % (name, n, expected))
        # (2) the frozen ODE verdict
        g = goldens[name]["ode_value"]
        if n != g:
            problems.append("%s: Newton received=%s vs the FROZEN ODE-oracle verdict %s"
                            % (name, n, g))
    assert not problems, (
        "receiver occlusion parity against the frozen ODE goldens failed:\n  "
        + "\n  ".join(problems) +
        "\n\nThe reference verdicts are FROZEN ODE-ORACLE VALUES, measured before src/ode was "
        "removed and committed to tests/goldens/ode_oracle_goldens.json "
        "(families.receiver_occlusion). THE ODE ARM NO LONGER EXISTS. Newton answers via mj_ray "
        "on the live mjModel behind OMNISIM_NEWTON_RAYCAST=1; a mismatch is a Newton raycast "
        "regression, not a stale golden. Do not retune the golden.")
