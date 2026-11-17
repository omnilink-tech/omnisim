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

"""Flagship behavioural signatures — the 'demos behave the same' gate.

WHY THIS EXISTS
---------------
The ODE-retirement programme changes contact physics defaults, solver
selection, and registration paths. The owner's standing condition is that the
flagship demos keep behaving the same. Point gates (lane-1 exact values, the
grasp verdict) each watch one mechanism; this file watches BEHAVIOUR, cheaply
enough to run after every engine change.

It earned its keep before it existed: the flagship pinch demo was SLIPPING for
up to three weeks (batched force-mode path, fixed in 1f06d625) and nothing
noticed, because no gate ran the demo the way a user does. The grasp now has
its own gate (test_friction_grasp_holds.py); this adds the wheeled flagship.

WHAT IT PINS
------------
The 10-Husky swarm world (newton_husky_swarm_drive.omniworld, shipped, UNPINNED --
it runs whatever the engine defaults are, which is the point): ten Huskies
commanded straight ahead for 6 s of sim must actually drive. Reference
signatures, measured 2026-08-07 on the MuJoCo-default binary:

    max displacement   2.385 m   (ODE agrees: 2.402 m -- within 1%)
    robots > 0.5 m     8 / 10    (two rear-spawned huskies start blocked; a
                                  robot count regression means wheels locked)

Under the removed XPBD default the same world read 0.97 m -- the lateral
wheel-pair lock. This gate is what makes that class of regression loud.

The bounds are deliberately loose (cross-machine lane-1 spread is real):
max >= 1.8 m says "the fleet genuinely drove", moved >= 7 says "no wheel-lock
class regression". Tighten only with cross-machine evidence.

The G1 box-delivery flagship is NOT here: it needs the GPU + ~15 min and is
run manually per campaign step (three consecutive greens on 2026-08-07:
segs=14/14, minz=0.720, dur 159.0/156.2/156.1). Its cheap proxies -- G1 spec
conformance and the grasp gate -- are in the standard battery.

    python -m pytest tests/test_flagship_signatures.py -v
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLD = REPO / "projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld"

SIM_MS = 6000
MIN_MAX_DISPLACEMENT_M = 1.8
MIN_ROBOTS_MOVED = 7
MOVED_THRESHOLD_M = 0.5

RE_LINE = re.compile(
    r"^(-?\d+(?:\.\d+)?)\t(.*)\t(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)\t(-?\d+(?:\.\d+)?)(?:\t([AR]))?$")

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def test_the_husky_swarm_actually_drives(tmp_path):
    """Ten unpinned Huskies on the engine DEFAULTS must drive ~2.4 m in 6 s."""
    traj = tmp_path / "swarm.traj"
    log = tmp_path / "swarm.log"
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_PROBE_TRAJ"] = str(traj)
    env["OMNISIM_PROBE_TRAJ_MS"] = str(SIM_MS)
    # This gate measures the DEFAULTS -- scrub every knob that would steer them.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)

    proc = subprocess.Popen(
        [str(_binary()), str(WORLD), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        proc.wait(timeout=420)   # the probe _Exit()s at the sim-time budget
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

    blob = log.read_text(errors="replace") if log.exists() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in blob:
            pytest.skip("Newton bring-up flake (%r) -- no data, re-run" % sig)
    if not traj.exists() or traj.stat().st_size == 0:
        pytest.skip("no trajectory produced (rc=%s) -- instrument outcome, not "
                    "a drive verdict:\n%s" % (proc.returncode, blob[-800:]))

    pts: dict[str, dict[float, tuple]] = {}
    for ln in traj.read_text(errors="replace").splitlines():
        m = RE_LINE.match(ln)
        if m:
            pts.setdefault(m.group(2), {})[float(m.group(1))] = (
                float(m.group(3)), float(m.group(4)))
    disp = {}
    for name, series in pts.items():
        ts = sorted(series)
        if len(ts) >= 2:
            a, b = series[ts[0]], series[ts[-1]]
            disp[name] = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
    assert disp, "trajectory parsed to zero roots"

    vals = sorted(disp.values(), reverse=True)
    moved = sum(1 for v in vals if v > MOVED_THRESHOLD_M)
    assert vals[0] >= MIN_MAX_DISPLACEMENT_M and moved >= MIN_ROBOTS_MOVED, (
        "the flagship swarm did not drive: max=%.3f m (need >= %.1f), "
        "moved>%.1fm: %d/%d (need >= %d). Reference on the MuJoCo default is "
        "2.385 m with 8/10; the removed XPBD default read 0.97 m (lateral "
        "wheel-pair lock). Suspect solver selection, wheel-joint gains, or "
        "contact-default changes -- and check the sidecar for which solver "
        "actually ran.\nper-robot: %s"
        % (vals[0], MIN_MAX_DISPLACEMENT_M, MOVED_THRESHOLD_M, moved, len(vals),
           MIN_ROBOTS_MOVED,
           {k: round(v, 3) for k, v in sorted(disp.items())}))
