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

# D1.5 (WREN deleted at D1.4, commit 976b9449d): the WREN renderer no longer exists, so
# every WREN arm this tool could drive is RETIRED -- the engine warns about and ignores
# the retired selectors, and a "WREN arm" run renders wgpu. The frozen WREN reference
# images live in tests/rendering/wren_reference/ (captured pre-deletion). The tool itself
# is kept: its wgpu arms and its A/B harness remain useful.
"""reversibility_check — the RENDER arm's revert lever, and a guard that the PHYSICS lever stays dead.

⚠️ THIS SCRIPT CHANGED MEANING ON 2026-08-08. It used to assert that `OMNISIM_LEGACY=1` reverts the whole
process to ODE+WREN. **src/ode was DELETED (commit bdc02139) and Newton is the only physics backend**, so
there is nothing for the physics arm to revert TO, and the old assertion was worse than useless: measured
on the post-deletion build it still printed

    [reversibility] legacy       -> render=Wren physics=Ode
    [reversibility] GATE PASS: OMNISIM_LEGACY=1 -> (Wren, Ode); per-arm FORCE levers confirmed.

i.e. a GREEN GATE CERTIFYING A REVERT PATH THAT CANNOT SIMULATE ANYTHING. The registry still *registers*
an Ode backend and resolve() still returns it, so a user who trusts that line and sets OMNISIM_LEGACY=1
gets a world with no physics implementation behind it and no warning.

So the physics assertion is INVERTED rather than dropped. This gate now proves:

  config              render     physics    asserts
  ------------------  ---------  ---------  -------------------------------------------------------------
  (default)           Vulkan*    Newton*    the new stack IS the default
  OMNISIM_FORCE_WREN  Wren       (Newton)   the render-only revert lever STILL WORKS (WREN is the default
                                            main-view renderer; wgpu is compiled in but opt-in)
  OMNISIM_LEGACY=1    Wren       NOT Ode    the retired physics lever must no longer select a deleted
  OMNISIM_FORCE_ODE   (Vulkan)   NOT Ode    backend -- if either still resolves to Ode, that is the
                                            DEFECT this gate exists to catch, and it FAILS naming it.
  (* Vulkan needs a wgpu build; Newton needs warp/newton importable by the embedded interpreter.)

STATUS: the physics rows PASS as of the 2026-08-08 18:02 engine build -- the engine-side cleanup
landed during the same session that inverted this gate, and both knobs now resolve physics to Newton.
So this half is no longer a to-do; it is a standing regression guard against the deleted backend
becoming selectable again. It genuinely caught the bug first: on the 16:49 build of the same day the
probe still reported physics=Ode under both knobs, and the OLD assertion PASSED on that.

Run from PowerShell (so warp resolves and the default/force_wren physics rows show Newton).
"""

import os
import re
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
from omnisim.paths import resolve_omnisim_binary  # noqa: E402

# Cross-platform binary resolution (Windows msys64, Linux bin/, macOS bundle).
# Fall back to the legacy Windows path so the exists() check still prints a
# helpful location on an unbuilt checkout.
BIN = resolve_omnisim_binary() or os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")

RE_RENDER = re.compile(r"^render=\S+ kind=(\S+) available=(\d)")
RE_PHYSICS = re.compile(r"^physics=\S+ kind=(\S+) available=(\d)")

# Render resolves fast (a ~100ms wgpu device probe, or instant under FORCE/LEGACY). Physics may init
# Newton (embedded Python + warp kernel compile -- tens of seconds cold), so allow generously for the
# physics line; the render line is flushed first so it's never lost to a slow/killed physics resolve.
RENDER_WAIT_S = 25
PHYSICS_EXTRA_WAIT_S = 75


def run_probe(label, levers):
    """Launch `omnisim-bin --help` with OMNISIM_PROBE_BACKENDS under the given lever env; return
    {'render': kind|None, 'physics': kind|None}."""
    out = os.path.join(REPO, "_scratch", "probe_be_%s.txt" % label)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    try:
        os.remove(out)
    except OSError:
        pass
    env = dict(os.environ)
    for k in ("OMNISIM_LEGACY", "OMNISIM_FORCE_ODE", "OMNISIM_FORCE_WREN"):
        env.pop(k, None)
    env.update(levers)
    env["OMNISIM_PROBE_BACKENDS"] = out
    env.setdefault("WEBOTS_HOME", REPO)
    env.setdefault("OMNISIM_HOME", REPO)
    proc = subprocess.Popen([BIN, "--help"], cwd=REPO, env=env)

    def parsed():
        res = {"render": None, "physics": None}
        try:
            with open(out, "r", errors="replace") as fh:
                for line in fh:
                    m = RE_RENDER.match(line.strip())
                    if m:
                        res["render"] = m.group(1)
                    m = RE_PHYSICS.match(line.strip())
                    if m:
                        res["physics"] = m.group(1)
        except OSError:
            pass
        return res

    deadline = time.time() + RENDER_WAIT_S + PHYSICS_EXTRA_WAIT_S
    res = {"render": None, "physics": None}
    while time.time() < deadline:
        res = parsed()
        if res["render"] is not None and res["physics"] is not None:
            break
        if proc.poll() is not None:
            res = parsed()
            break
        time.sleep(1)
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
    res = parsed() if (res["render"] is None and res["physics"] is None) else res
    print("[reversibility] %-12s -> render=%s physics=%s" % (label, res["render"], res["physics"]))
    return res


def main():
    if not os.path.exists(BIN):
        print("[reversibility] omnisim-bin not found at %s -- build it first." % BIN)
        return 0

    print("[reversibility] proving the RENDER revert lever works, and that the retired PHYSICS")
    print("[reversibility] lever no longer selects the DELETED ODE backend (see the module docstring)")
    print("[reversibility] (run from PowerShell so the default/force_wren physics rows can show Newton)\n")

    legacy = run_probe("legacy", {"OMNISIM_LEGACY": "1"})
    force_ode = run_probe("force_ode", {"OMNISIM_FORCE_ODE": "1"})
    force_wren = run_probe("force_wren", {"OMNISIM_FORCE_WREN": "1"})
    default = run_probe("default", {})

    print("")
    failures = []

    # RENDER arm -- still a real, live lever: WREN is the default main-view renderer and
    # wgpu is compiled in but opt-in, so the revert has somewhere to go.
    if legacy["render"] != "Wren":
        failures.append("OMNISIM_LEGACY=1 did not resolve render to Wren: got %s" % legacy["render"])

    # PHYSICS arm -- INVERTED (2026-08-08, src/ode deleted in bdc02139). Selecting a backend
    # whose implementation no longer exists must not be reported as a successful revert.
    for label, res in (("OMNISIM_LEGACY=1", legacy), ("OMNISIM_FORCE_ODE=1", force_ode)):
        if res["physics"] == "Ode":
            failures.append(
                "%s still resolves physics to Ode, but src/ode was DELETED (bdc02139) -- the "
                "engine is offering a backend it cannot implement. Fix in "
                "src/omnisim/physics/OmPhysicsBackend.cpp (forceOde() short-circuits resolve()); "
                "the retired knob should be ignored with a warning, not honoured." % label)
    if force_wren["render"] != "Wren":
        failures.append("OMNISIM_FORCE_WREN=1 did not resolve render to Wren: got %s" % force_wren["render"])

    # CONTRAST (informational): show the default actually uses the new render stack.
    if default["render"] == "Vulkan":
        print("[reversibility] CONTRAST: default render is Vulkan -> FORCE_WREN is a real revert "
              "(render Vulkan->Wren).")
    else:
        print("[reversibility] CONTRAST: default render is %s, not Vulkan -- a WREN-only build. The "
              "FORCE_WREN lever above is then a no-op contrast, not a proven revert."
              % default["render"])
    if default["physics"] != "Newton":
        print("[reversibility] NOTE: default physics resolved to %s, not Newton -- warp/newton is not "
              "importable by the embedded interpreter here, so this run says nothing about the Newton "
              "arm. There is no second backend to fall back to any more (src/ode deleted): see "
              "docs/developer/newton-runtime-bundle.md." % default["physics"])

    print("")
    if failures:
        for f in failures:
            print("[reversibility] FAIL: %s" % f)
        print("[reversibility] GATE FAIL")
        return 1
    print("[reversibility] GATE PASS: the render revert lever (OMNISIM_FORCE_WREN / OMNISIM_LEGACY -> "
          "Wren) works, and neither retired physics knob resolves to the deleted ODE backend.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
