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

"""ContactProperties.bounce is never read, and the engine must say so.

WHY THIS EXISTS (internal parity plan, item W1.5)
--------------------------------------------
`ContactProperties.bounce` / `bounceVelocity` are ODE-path fields. Their
accessors on OmContactProperties had ZERO callers, and unlike `coulombFriction`
there is no `newton*` field to migrate them to -- MuJoCo has no coefficient of
restitution at all. Our contact defaults compile to the stock, critically
damped `solref (0.02, 1.0)`, i.e. e ~= 0.

So this is a HARD LIMIT, not a gap to close, and the only honest engineering
response is to name it: a world that authors a bounce is authoring something the
engine will never read, and silence there reads as "it worked".

ARMS
----
  bounce authored (0.9)   -> warns, quoting the value
  bounce left at default  -> silent  (the .wrl default is 0.5, and warning on it
                             would fire on all 322 worlds carrying a
                             ContactProperties node and be muted rather than read)

FREEBIE IN THE SAME CHANGE, also pinned here: the neighbouring coulombFriction
warning used to advise pinning `physicsBackend "ode"`. Since bdc02139 deleted
src/ode, a Solid pinned to "ode" is registered with NO solver -- no gravity, no
contact -- so the engine's advice for "my friction is ignored" was "have no
physics". A static scan asserts no engine source offers that workaround again.

    python -m pytest tests/test_newton_restitution_is_declared_never_read.py -v
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"
SRC = REPO / "src" / "omnisim"

WARN_RE = re.compile(r"ContactProperties declares bounce ([0-9.]+) / bounceVelocity ([0-9.]+)")

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _world_text(bounce_line):
    return """#OMNISIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_restitution_is_declared_never_read.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  newtonStatics TRUE
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  contactProperties [
    ContactProperties {
      material1 "default"
      material2 "default"%(B)s
    }
  ]
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
DEF BALL Solid {
  translation 0 0 1
  name "ball"
  children [
    DEF BALL_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.85 0.3 0.25 roughness 1 metalness 0 }
      geometry Sphere { radius 0.1 }
    }
  ]
  boundingObject USE BALL_SHAPE
  physics Physics { density -1 mass 1 }
}
""" % {"B": bounce_line}


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


def _run(tmp_path, tag, bounce_line):
    world = WORLDS / (".restitution_%s.omniworld" % tag)
    world.write_text(_world_text(bounce_line), encoding="utf-8")
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
        pytest.fail("the world never finalised:\n%s" % blob[-1500:])
    return blob


# --------------------------------------------------------------- static scan

#: "pin/set/use physicsBackend "ode"" -- the ADVICE, not the identifier, which
#: legitimately appears all over backend resolution and inside warnings ABOUT
#: the value. Applied to comment-stripped, literal-joined source (see _messages).
ODE_ADVICE_RE = re.compile(r"(pin|set|use)\s[^\"]{0,40}physicsBackend \\\"ode\\\"", re.IGNORECASE)


def _messages(text):
    """C++ source reduced to something a message-level regex can be run on.

    Two transformations, and both are load-bearing:
      * comment lines are dropped -- otherwise a comment that QUOTES a removed
        message (which is exactly how this repo records a retraction) reads as
        the message itself;
      * adjacent string literals are joined -- a wrapped `tr(...)` splits its
        own sentence across source lines, so the phrase this test is looking for
        does not exist on any single line of the file it came from.
    """
    lines = [ln for ln in text.splitlines() if not ln.lstrip().startswith(("//", "*", "/*"))]
    return re.sub(r'"\s*\n\s*"', "", "\n".join(lines))


def _ode_advice_offenders():
    out = []
    for path in sorted(SRC.rglob("*.cpp")) + sorted(SRC.rglob("*.hpp")):
        joined = _messages(path.read_text(encoding="utf-8", errors="replace"))
        for m in ODE_ADVICE_RE.finditer(joined):
            out.append("%s: ...%s..." % (path.relative_to(REPO).as_posix(),
                                         joined[m.start():m.start() + 70]))
    return out


def test_the_ode_advice_scan_can_go_red():
    """A green that cannot be made to go red is not evidence.

    Feeds the scanner the message this change removed, in its original wrapped
    form, and asserts it is caught -- so the green above means "the advice is
    gone", not "the regex never matched anything".
    """
    original = (
        '        OmLog::warning(QObject::tr("... the effective friction is "\n'
        '                                   "1.0. Set WorldInfo.newtonGroundMu %1 to get the friction you asked "\n'
        '                                   "for, or pin physicsBackend \\"ode\\" on the Solids you are tuning "\n'
        '                                   "through contactProperties."));\n')
    assert ODE_ADVICE_RE.search(_messages(original)), (
        "the scanner no longer catches the exact message this change removed, so its green "
        "proves nothing")


def test_no_engine_source_advises_pinning_physicsbackend_ode():
    """ODE was deleted; a Solid pinned to "ode" is registered with NO solver.

    A source scan rather than a run, because the failure is a STRING: any
    message offering `physicsBackend "ode"` as a workaround tells a user to fix
    a wrong answer by removing their physics entirely.
    """
    offenders = _ode_advice_offenders()
    assert not offenders, (
        "engine source still advises pinning physicsBackend \"ode\" as a workaround:\n  %s\n"
        "src/ode was deleted in bdc02139; that pin now means the Solid is registered with no "
        "solver at all -- no gravity and no contact." % "\n  ".join(offenders))


# ------------------------------------------------------------------ live arms

@pytest.mark.skipif(_binary() is None, reason="no omnisim-bin in this clone")
def test_authored_bounce_is_disclosed(tmp_path):
    blob = _run(tmp_path, "authored", "\n      bounce 0.9\n      bounceVelocity 0.2")
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    hits = WARN_RE.findall(blob)
    assert hits, (
        "a world authored bounce 0.9 and the engine said nothing. Restitution is not "
        "implemented and structurally cannot be -- MuJoCo has no coefficient of restitution -- "
        "so silence here reads as 'your bouncy contact is configured', which it is not.")
    assert hits[0][0].startswith("0.9"), "the warning quoted bounce %s, want 0.9" % hits[0][0]


@pytest.mark.skipif(_binary() is None, reason="no omnisim-bin in this clone")
def test_default_bounce_is_not_warned_about(tmp_path):
    """The control. 322 worlds carry a ContactProperties node at the .wrl
    default bounce 0.5; warning on all of them trains readers to mute the
    message, which costs more than it buys."""
    blob = _run(tmp_path, "default", "")
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    assert not WARN_RE.findall(blob), (
        "the restitution disclosure fired on a ContactProperties that never authored a "
        "bounce at all.")
