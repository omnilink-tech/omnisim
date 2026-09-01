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

"""A multi-child compound boundingObject must not be silently truncated.

WHY THIS EXISTS (internal parity plan, item W1.6)
--------------------------------------------
`boundingObject Group { children [ A B C ] }` registers ONLY `A` with the
physics engine unless WorldInfo.newtonCompoundColliders is TRUE (the default is
FALSE). B and C are dropped -- no warning, no error, world loads clean, the
body has a collider. ~108 compound objects with >= 2 children across ~77 tracked
files are in this state (regex recount 2026-09-01; originally 120/91); a
Chair.proto collides today as a floating seat slab with no legs, and this just
bit a shipped sample world.

WHAT THIS TEST DOES AND DOES NOT PIN. It pins the DISCLOSURE, not a fix. The
default is deliberately NOT flipped: the same flag also selects the inertia
source for every dynamic multi-collider body (OmSolid.cpp, the
newtonCompoundColliders branch in the inertia path), so flipping it silently
changes the dynamics of far more than the colliders. Decoupling those two is
separate, larger work. Until then a named warning is the honest half, and this
file is what stops it regressing back to silence.

ARMS
----
  default (flag unset)                 -> warns, names the OWNING SOLID
  WorldInfo.newtonCompoundColliders TRUE -> silent (all colliders registered)
  single-child Group                   -> silent (nothing is dropped)

The second and third arms are what make the first evidence rather than
assertion: a rig that warned unconditionally fails both.

    python -m pytest tests/test_newton_compound_collider_warning.py -v
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

#: The engine's own wording (OmSolid.cpp).
WARN_RE = re.compile(r"The boundingObject of '([^']+)' is a Group of (\d+) collision shapes")

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")

#: Two boxes side by side on one rigid body -- the shape a table, a chair or a
#: C-channel has. Only the first is registered by default.
TWO_CHILD_BO = """boundingObject Group {
    children [
      Box { size 0.4 0.4 0.2 }
      Pose {
        translation 0.6 0 0
        children [ Box { size 0.4 0.4 0.2 } ]
      }
    ]
  }"""

ONE_CHILD_BO = """boundingObject Group {
    children [
      Box { size 0.4 0.4 0.2 }
    ]
  }"""


def _world_text(bo, compound_field):
    field = "\n  newtonCompoundColliders TRUE" if compound_field else ""
    return """#OMNISIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_compound_collider_warning.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  newtonStatics TRUE
  coordinateSystem "ENU"
  newtonSolver "mujoco"%(FIELD)s
}
Viewpoint { position -4 0 1.5 }
Background { skyColor [ 0.15 0.18 0.24 ] }
DEF FLOOR Solid {
  translation 0 0 -0.05
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.4 0.4 0.45 roughness 1 metalness 0 }
      geometry Box { size 8 8 0.1 }
    }
  ]
  boundingObject USE FLOOR_SHAPE
}
DEF BENCH Solid {
  translation 0 0 1
  name "bench"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.25 roughness 1 metalness 0 }
      geometry Box { size 0.4 0.4 0.2 }
    }
  ]
  %(BO)s
  physics Physics { density -1 mass 5 }
}
""" % {"BO": bo, "FIELD": field}


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run(tmp_path, tag, bo, compound_field):
    world = WORLDS / (".compound_collider_%s.omniworld" % tag)
    world.write_text(_world_text(bo, compound_field), encoding="utf-8")
    log = tmp_path / ("engine_%s.log" % tag)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(90):
            blob = log.read_text(errors="replace") if log.exists() else ""
            if "world finalised" in blob:
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
    if "world finalised" not in blob:
        pytest.fail("the world never finalised, so nothing registered:\n%s" % blob[-1500:])
    return blob


def test_dropped_compound_children_are_named(tmp_path):
    blob = _run(tmp_path, "default", TWO_CHILD_BO, compound_field=False)
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    hits = WARN_RE.findall(blob)
    assert hits, (
        "a 2-child Group boundingObject registered only its first collider and said NOTHING. "
        "That is the silent-truncation defect: the world loads clean, the body has a collider, "
        "and only its geometry is wrong.")
    owner, count = hits[0]
    assert "bench" in owner.lower(), (
        "the warning named %r; it must name the OWNING SOLID, because 'a Group' is not "
        "something an author can find in a world file." % (owner,))
    assert count == "2", "the warning reported %s shapes, want 2" % count


def test_no_warning_when_the_world_opts_in(tmp_path):
    """WorldInfo.newtonCompoundColliders TRUE registers all of them -- nothing dropped."""
    blob = _run(tmp_path, "optin", TWO_CHILD_BO, compound_field=True)
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    assert not WARN_RE.findall(blob), (
        "the world declared newtonCompoundColliders TRUE, so every collider IS registered and "
        "there is nothing to warn about. Warning anyway makes the message noise.")


def test_no_warning_for_a_single_child_group(tmp_path):
    """The other half of the control: one child, nothing dropped, nothing said."""
    blob = _run(tmp_path, "single", ONE_CHILD_BO, compound_field=False)
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    assert not WARN_RE.findall(blob), (
        "a single-child Group warned about dropped colliders; nothing was dropped.")
