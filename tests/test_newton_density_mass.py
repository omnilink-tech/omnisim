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

"""A density-defined body must reach the solver with its density-derived mass.

WHY THIS EXISTS
---------------
Until 2026-09-27 every dynamic Solid whose mass came from `density` -- a bare
`Physics {}`, or `Physics { density 250 }` -- was registered on Newton at
0.25 kg, whatever its size or density. The Newton body build summed the raw
`Physics.mass` FIELD, which is -1 for those bodies, so they counted as 0 and
fell through to a 0.25 kg fallback (OmSolid.cpp, the registration after
rolledUpMass). The inertia was scaled to that mass, so the bodies were
consistent but tiny: a 67 kg steel-ish ball bounced off a stack of 2 kg blocks
instead of knocking it over, and nothing warned.

test_newton_native_inertia_parity.py did not catch it: it checks the load-time
composer (OmSolid::mass() / mNativeInertia), whose density mass was always
right. This test checks the other end -- the MuJoCo model the solver actually
steps -- via the OMNISIM_NEWTON_DUMP_MJMODEL introspection dump.

    python -m pytest tests/test_newton_density_mass.py -v
"""
from __future__ import annotations

import math
import os
import re
import subprocess
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

#: x position -> (label, expected registered mass in kg)
EXPECTED = {
    -1.5: ("box 0.2 m, density 250", 0.2 ** 3 * 250),
    -0.5: ("box 0.2 m, mass 2 (control: declared mass)", 2.0),
    0.5: ("sphere r 0.2 m, density 2000", 4.0 / 3.0 * math.pi * 0.2 ** 3 * 2000),
    1.5: ("box 0.5 m, bare Physics {} (density 1000)", 0.5 ** 3 * 1000),
    2.5: ("box 0.2 m, mass 3 AND density 7800 (mass wins)", 3.0),
}


def _solid(x, z, geometry, physics):
    return f"""Solid {{
  translation {x} 0 {z}
  name "s{int((x + 2) * 10)}"
  children [ Shape {{ geometry DEF G{int((x + 2) * 10)} {geometry} }} ]
  boundingObject USE G{int((x + 2) * 10)}
  physics {physics}
}}
"""


def _world_text():
    return ("#OMNISIM R2025a utf8\n\nWorldInfo { basicTimeStep 8 }\n"
            "Viewpoint { position 0 -6 3 }\n"
            + _solid(-1.5, 0.1, "Box { size 0.2 0.2 0.2 }", "Physics { density 250 }")
            + _solid(-0.5, 0.1, "Box { size 0.2 0.2 0.2 }", "Physics { density -1 mass 2 }")
            + _solid(0.5, 0.2, "Sphere { radius 0.2 }", "Physics { density 2000 }")
            + _solid(1.5, 0.25, "Box { size 0.5 0.5 0.5 }", "Physics { }")
            + _solid(2.5, 0.1, "Box { size 0.2 0.2 0.2 }", "Physics { density 7800 mass 3 }"))


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no omnisim-bin in this clone")

_BODY = re.compile(r"^body \d+ \S+\s+mass=(\S+) ipos=\[[^\]]*\] pos=\[([^\]]*)\]", re.M)


def _run_once(tmp_path, attempt):
    world = WORLDS / ".newton_density_mass.omniworld"
    world.write_text(_world_text(), encoding="utf-8")
    dump = tmp_path / f"mjmodel_{attempt}.txt"
    env = {k: v for k, v in os.environ.items()
           if not (k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"))}
    env.update(OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(tmp_path / f"engine_{attempt}.log"),
               OMNISIM_NEWTON_DUMP_MJMODEL=str(dump))
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering", "--minimize",
         "--stdout", "--stderr"],
        cwd=str(REPO), env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    masses = {}
    try:
        deadline = time.time() + 120
        while time.time() < deadline and proc.poll() is None:
            if dump.exists():
                time.sleep(0.5)  # let the one-shot dump finish writing
                for m in _BODY.finditer(dump.read_text(errors="replace")):
                    x = round(float(m.group(2).split(",")[0]), 2)
                    masses[x] = float(m.group(1))
                if set(EXPECTED) <= set(masses):
                    break
            time.sleep(0.5)
    finally:
        if proc.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)
            else:
                proc.kill()
            proc.wait()
        try:
            world.unlink()
        except OSError:
            pass
    return masses


def test_density_defined_bodies_reach_the_solver_with_their_real_mass(tmp_path):
    masses = {}
    for attempt in (1, 2):  # the Newton FFI bring-up flakes on a few % of launches
        masses = _run_once(tmp_path, attempt)
        if set(EXPECTED) <= set(masses):
            break
    missing = sorted(set(EXPECTED) - set(masses))
    assert not missing, f"no registered body found at x={missing}; bodies seen: {masses}"
    wrong = []
    for x, (label, want) in EXPECTED.items():
        got = masses[x]
        if abs(got - want) > 1e-3 * want:
            wrong.append(f"{label}: registered {got:.6g} kg, expected {want:.6g} kg")
    assert not wrong, "density-derived mass lost on the way to the solver:\n  " + "\n  ".join(wrong)
