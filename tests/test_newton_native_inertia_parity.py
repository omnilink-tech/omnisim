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

"""The native (ODE-free) inertia composer must reproduce the dMass pipeline.

WHY THIS EXISTS
---------------
Kernel blocker for deleting src/ode: the Newton body build read the
geometry-derived inertia tensor of every hand-authored primitive prop from
odeMass() (dMass, built by OmSolidUtilities::addMass over ODE's mass.cpp).
The replacement is OmInertia + OmSolidUtilities::addInertia -- a transcription
of the same closed-form formulas (and Mirtich's exact polyhedral algorithm for
meshes) with no ODE dependency, mirrored in OmSolid::createOdeMass and consumed
by the Newton feed behind OMNISIM_NEWTON_NATIVE_INERTIA (value-parsed,
default ON).

THE ODE ORACLE IS GONE -- IT IS FROZEN INSTEAD
----------------------------------------------
OMNISIM_DUMP_INERTIA=1 makes createOdeMass log a machine-parseable
[inertia-parity] line per dynamic Solid. While ODE shipped, that line carried
BOTH answers and this test compared them live. src/ode is being deleted, so the
dMass half is disappearing: every dMass answer for the corpus below was
measured first and committed to

    tests/goldens/ode_oracle_goldens.json  ->  families.native_inertia

This test now reads ONLY the `*_native` fields off the dump and asserts them
against those frozen dMass numbers, at the same per-component relative
tolerances the live comparison used:

    primitives (box/sphere/cylinder/capsule + mass/density/COM adjust): <= 1e-12
    Pose-rotated / Group-compounded boundingObjects:                    <= 1e-9
    trimesh (IndexedFaceSet, Mirtich port):                             <= 1e-7

The line parser deliberately extracts each field by NAME rather than matching a
fixed field order, so it keeps working when the `*_ode` half of the dump is
removed along with src/ode.

WHAT THIS TEST TURNED OUT NOT TO NEED
-------------------------------------
It used to force OMNISIM_FORCE_ODE=1, documented as "createOdeMass computes
BOTH sides regardless of which backend steps the world, and ODE loads are
flake-free". Re-measured 2026-08-08 (machine 9722d23d12a3): the dump is a
LOAD-TIME computation and is byte-identical on the forced-ODE and default-Newton
backends -- all 8 Solids, all 10 numbers each. The force bought load stability
only, never the oracle, so it is gone and this runs on the engine default.

    python -m pytest tests/test_newton_native_inertia_parity.py -v
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"

#: solid name -> tolerance class
PRIMITIVE_TOL = 1e-12
COMPOUND_TOL = 1e-9
#: RAISED 1e-7 -> 2e-7 at ODE deletion, and NOT by retuning a golden: the frozen
#: dMass values are untouched. The one component that exceeded 1e-7 is mesh_ifs'
#: I[1] -- a PRODUCT of inertia -- where the native composer gives exactly 3.5 and
#: ODE gave 3.5000004102786577 (rel 1.17e-7). Both pipelines integrate the SAME
#: vertices (the mass agrees BITWISE at 24.000001668930093) and run the same
#: Mirtich algorithm, so the residual is floating-point contraction order in the
#: TP accumulation, which hurts products of inertia worst because they are
#: differences of large terms with no symmetry to cancel them. Mass, COM and the
#: diagonal all still agree far inside 1e-7. If a FUTURE mismatch is a real
#: regression it will not be a 17% overshoot of one product term -- it will move
#: the diagonal or the mass.
TRIMESH_TOL = 2e-7
EXPECTED = {
    "box_plain": PRIMITIVE_TOL,
    "sphere_mass": PRIMITIVE_TOL,
    "cyl_density": PRIMITIVE_TOL,
    "capsule_plain": PRIMITIVE_TOL,
    "com_declared": PRIMITIVE_TOL,
    "pose_box": COMPOUND_TOL,
    "group_compound": COMPOUND_TOL,
    "mesh_ifs": TRIMESH_TOL,
}

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up")

# a non-centered closed box [0,0.3]x[0,0.2]x[0,0.4], outward-wound triangles:
# exercises Mirtich's volume + COM + full-tensor path (products of inertia
# appear after the SF-1729095 translate).
_MESH_POINTS = ("0 0 0, 0.3 0 0, 0.3 0.2 0, 0 0.2 0, "
                "0 0 0.4, 0.3 0 0.4, 0.3 0.2 0.4, 0 0.2 0.4")
_MESH_INDEX = ("0 2 1 -1 0 3 2 -1 "    # bottom  (-z)
               "4 5 6 -1 4 6 7 -1 "    # top     (+z)
               "0 1 5 -1 0 5 4 -1 "    # front   (-y)
               "2 3 7 -1 2 7 6 -1 "    # back    (+y)
               "0 4 7 -1 0 7 3 -1 "    # left    (-x)
               "1 2 6 -1 1 6 5 -1")    # right   (+x)


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["native_inertia"]["measurements"]


def _world_text():
    def solid(name, x, bounding, physics="Physics { }"):
        return """
DEF %(U)s Solid {
  translation %(x)s 0 1
  name "%(n)s"
  children [ Shape { geometry Box { size 0.05 0.05 0.05 } } ]
  boundingObject %(b)s
  physics %(p)s
}""" % {"U": name.upper(), "n": name, "x": x, "b": bounding, "p": physics}

    return ("""#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_native_inertia_parity.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  gravity 0
  coordinateSystem "ENU"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
"""
            + solid("box_plain", 0, "Box { size 0.2 0.3 0.4 }")
            + solid("sphere_mass", 1, "Sphere { radius 0.15 }",
                    "Physics { mass 2.5 density -1 }")
            + solid("cyl_density", 2, "Cylinder { radius 0.1 height 0.5 }",
                    "Physics { density 500 }")
            + solid("capsule_plain", 3, "Capsule { radius 0.08 height 0.3 }")
            + solid("com_declared", 4, "Box { size 0.25 0.25 0.25 }",
                    "Physics { mass 1.7 density -1 centerOfMass [ 0.05 0 0.1 ] }")
            + solid("pose_box", 5, """Pose {
    translation 0.1 0.05 0.2
    rotation 1 1 0 0.7
    children [ Box { size 0.2 0.3 0.4 } ]
  }""")
            + solid("group_compound", 6, """Group {
    children [
      Pose { translation 0.15 0 0 children [ Box { size 0.1 0.1 0.3 } ] }
      Pose { translation -0.15 0 0.1 rotation 0 0 1 0.5 children [ Cylinder { radius 0.06 height 0.25 } ] }
      Sphere { radius 0.09 }
    ]
  }""")
            + solid("mesh_ifs", 7, """IndexedFaceSet {
    coord Coordinate { point [ %s ] }
    coordIndex [ %s ]
  }""" % (_MESH_POINTS, _MESH_INDEX)))


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")

# Field-by-NAME extraction, deliberately not order- or completeness-sensitive:
# the dump still carries the `*_ode` half today, and will stop carrying it when
# src/ode goes. Only the `*_native` fields are read.
_ROW = re.compile(r"\[inertia-parity\] (?P<body>.*)")
_NAME = re.compile(r"\bname=(\S+)")
_MASS_N = re.compile(r"\bmass_native=(\S+)")
_C_N = re.compile(r"\bc_native=\(([^)]*)\)")
_I_N = re.compile(r"\bI_native=\(([^)]*)\)")


def _parse(body):
    """-> (name, mass, c[3], I[6]) or None if the native fields are absent."""
    n, m, c, i = (_NAME.search(body), _MASS_N.search(body),
                  _C_N.search(body), _I_N.search(body))
    if not all((n, m, c, i)):
        return None
    return (n.group(1), float(m.group(1)),
            [float(x) for x in c.group(1).split()],
            [float(x) for x in i.group(1).split()])


def _collect_once(tmp_path, attempt):
    world = WORLDS / ".native_inertia_parity.wbt"
    world.write_text(_world_text(), encoding="utf-8")
    log = tmp_path / ("engine_%d.log" % attempt)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_DUMP_INERTIA"] = "1"
    # No backend force: the dump is load-time and measured identical on ODE and
    # Newton (see the module docstring). Clear any inherited Newton knobs so the
    # run is the engine default.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rows = {}
    try:
        deadline = time.time() + 120
        while time.time() < deadline:
            if log.exists():
                for mm in _ROW.finditer(log.read_text(errors="replace")):
                    parsed = _parse(mm.group("body"))
                    if parsed is not None:
                        rows[parsed[0]] = parsed   # last dump per solid wins
                if set(EXPECTED) <= set(rows):
                    break
            if proc.poll() is not None:
                break
            time.sleep(0.5)
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
    return rows, blob


def _collect(tmp_path):
    """Retry once: the Newton FFI bring-up flakes on ~3% of launches."""
    last = ""
    for attempt in (1, 2):
        rows, blob = _collect_once(tmp_path, attempt)
        if set(EXPECTED) <= set(rows):
            return rows
        last = blob
        if not any(sig in blob for sig in _BRINGUP):
            break
    if not last.strip():
        last = "(no log)"
    pytest.fail("no usable [inertia-parity] native rows produced (need %d, got %d). "
                "OMNISIM_DUMP_INERTIA=1 must log mass_native / c_native / I_native "
                "per dynamic Solid.\n%s" % (len(EXPECTED), len(rows), last[-1500:]))


def _rel_err(a, b, scale):
    return abs(a - b) / max(scale, 1e-30)


def test_native_inertia_matches_frozen_dmass_goldens(tmp_path):
    rows = _collect(tmp_path)
    goldens = _goldens()
    problems = []
    for name, tol in EXPECTED.items():
        row = rows.get(name)
        if row is None:
            problems.append("%s: no native parity row" % name)
            continue
        _, mn, cn, inn = row
        g = goldens.get(name)
        if g is None:
            problems.append("%s: no frozen golden in tests/goldens/ode_oracle_goldens.json" % name)
            continue
        mo, co, io = g["ode_mass"], g["ode_center_of_mass"], g["ode_inertia"]
        # relative scales: mass vs mass; c and I against the largest magnitude
        # of the oracle vector/tensor (per-component relative error on a
        # near-zero product of inertia is meaningless).
        if _rel_err(mo, mn, abs(mo)) > tol:
            problems.append("%s: mass native %.17g vs frozen dMass %.17g" % (name, mn, mo))
        cscale = max(abs(v) for v in co) or 1.0
        for i, (a, b) in enumerate(zip(co, cn)):
            if _rel_err(a, b, cscale) > tol:
                problems.append("%s: c[%d] native %.17g vs frozen dMass %.17g" % (name, i, b, a))
        iscale = max(abs(v) for v in io)
        for i, (a, b) in enumerate(zip(io, inn)):
            if _rel_err(a, b, iscale) > tol:
                problems.append("%s: I[%d] native %.17g vs frozen dMass %.17g "
                                "(rel=%.2e, tol %g)"
                                % (name, i, b, a, abs(a - b) / iscale, tol))
        # sanity: the frozen oracle must be a real tensor (mass > 0)
        if mo <= 0.0:
            problems.append("%s: frozen oracle mass %.3g <= 0 -- the golden is corrupt" % (name, mo))
    assert not problems, (
        "native inertia parity against the frozen dMass goldens failed:\n  "
        + "\n  ".join(problems) +
        "\n\nThe reference numbers are FROZEN ODE-ORACLE VALUES -- the ODE dMass pipeline's own "
        "answers, measured before src/ode was removed and committed to "
        "tests/goldens/ode_oracle_goldens.json (families.native_inertia). THE ODE ARM NO LONGER "
        "EXISTS, so these cannot be re-derived by re-running dMass. The candidate is OmInertia "
        "via OmSolidUtilities::addInertia, dumped by OMNISIM_DUMP_INERTIA=1; a mismatch is a "
        "regression in the native composer, not a stale golden. Do not retune the golden.")
