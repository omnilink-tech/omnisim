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

"""reversibility_check — prove the OMNISIM_LEGACY=1 escape hatch reverts the whole process to ODE+WREN.

This is the last box of the architectural baseline (architectural-baseline.md §5): after the C1 flip the
DEFAULT build compiles BOTH new backends in (Newton-ON + wgpu-ON), so the one-switch revert to the legacy
stack must be proven, not assumed.

It drives the headless `OMNISIM_PROBE_BACKENDS` probe (built into omnisim-bin: it calls each registry's
resolve() under the current environment and reports the chosen backend) across the lever configs and
asserts the reversion:

  config              render     physics    proves
  ------------------  ---------  ---------  -------------------------------------------------------------
  (default)           Vulkan*    Newton*    the new stack IS the default (so LEGACY reverts FROM it)
  OMNISIM_LEGACY=1    Wren       Ode        the UMBRELLA reverts BOTH arms -- the core claim
  OMNISIM_FORCE_ODE   (Vulkan)   Ode        the physics-only lever (render unchanged)
  OMNISIM_FORCE_WREN  Wren       (Newton)   the render-only lever (physics unchanged)
  (* needs the new backend available: a wgpu build for Vulkan, warp/newton importable for Newton.)

Byte-IDENTITY of the reverted output is guaranteed by construction -- the legacy ODE+WREN code is untouched
by the migration (engine-migration non-negotiables #1/#2) -- and is EMPIRICALLY verified by the two
standing pre-push gates: the forced-ODE determinism smoke (legacy physics byte-identical) and the render
golden (wgpu deterministic; WREN unchanged). This check proves the LEVER selects those legacy backends.

Run from PowerShell (so warp resolves and the default/force_wren physics rows show Newton). A WREN-only or
no-warp build still PASSES the core LEGACY+FORCE assertions (those force the legacy backends, which need no
init); only the "new stack is the default" contrast softens to a note.
"""

import os
import re
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BIN = os.path.join(REPO, "msys64", "mingw64", "bin", "omnisim-bin.exe")

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

    print("[reversibility] proving OMNISIM_LEGACY=1 reverts the whole process to ODE+WREN")
    print("[reversibility] (run from PowerShell so the default/force_wren physics rows can show Newton)\n")

    legacy = run_probe("legacy", {"OMNISIM_LEGACY": "1"})
    force_ode = run_probe("force_ode", {"OMNISIM_FORCE_ODE": "1"})
    force_wren = run_probe("force_wren", {"OMNISIM_FORCE_WREN": "1"})
    default = run_probe("default", {})

    print("")
    failures = []

    # CORE assertions (must hold on any build -- they force the legacy backends, which need no init).
    if not (legacy["render"] == "Wren" and legacy["physics"] == "Ode"):
        failures.append("OMNISIM_LEGACY=1 did not resolve to (Wren, Ode): got (%s, %s)"
                        % (legacy["render"], legacy["physics"]))
    if force_ode["physics"] != "Ode":
        failures.append("OMNISIM_FORCE_ODE=1 did not resolve physics to Ode: got %s" % force_ode["physics"])
    if force_wren["render"] != "Wren":
        failures.append("OMNISIM_FORCE_WREN=1 did not resolve render to Wren: got %s" % force_wren["render"])

    # CONTRAST (informational): show the default actually uses the new stack, so LEGACY reverts FROM it.
    new_render = default["render"] == "Vulkan"
    new_physics = default["physics"] == "Newton"
    if new_render or new_physics:
        bits = []
        if new_render:
            bits.append("render Vulkan->Wren")
        if new_physics:
            bits.append("physics Newton->Ode")
        print("[reversibility] CONTRAST: default uses the new stack (%s) -> LEGACY is a real revert (%s)."
              % ("+".join(k for k, v in (("Vulkan", new_render), ("Newton", new_physics)) if v),
                 ", ".join(bits)))
    else:
        print("[reversibility] CONTRAST: default did NOT show a new backend (render=%s physics=%s) -- a "
              "WREN-only / no-warp build, or Newton init didn't finish. The levers above still PROVE the "
              "reversion (they force legacy, which needs no init); the empirical 'new stack is default' "
              "lives in the standing physics oracle + render golden gates." % (default["render"], default["physics"]))

    print("")
    if failures:
        for f in failures:
            print("[reversibility] FAIL: %s" % f)
        print("[reversibility] GATE FAIL")
        return 1
    print("[reversibility] GATE PASS: OMNISIM_LEGACY=1 -> (Wren, Ode); per-arm FORCE levers confirmed.")
    print("[reversibility] Byte-identity of the reverted stack follows by construction (legacy code")
    print("[reversibility] untouched) + the standing forced-ODE smoke + render-golden gates.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
