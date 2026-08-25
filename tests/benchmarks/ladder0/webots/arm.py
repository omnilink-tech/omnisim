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

"""The upstream-Webots arm: R2025a in WSL2, driven from Windows.

This is the ladder's CONTROL arm.  It is the same scenes, the same recorded
quantities and the same shared judge as the OmniSim arm -- the only things
that differ are the dialect the scene is spelled in (``worldgen.py`` documents
each spelling and where it was verified) and the engine underneath.

It implements CONTRACT.md section 5 and nothing more: it launches, it records,
and it hands the sample document over.  **No verdict is computed here.**

Mechanics worth knowing:

* The WSL side is a script FILE (``run_cell.sh``) invoked as
  ``wsl.exe bash <path> <args...>``, with every argument a separate argv entry.
  A multi-line command handed to ``wsl.exe`` gets mangled by the Windows
  command-line round trip.
* Wall-clock marks (``T0``/``T1``) are taken inside WSL, in the same clock
  domain as the controller's own ``time.time()``, so "spawn -> first step" is a
  subtraction within one clock rather than across the Windows/WSL boundary.
* The TCP port is pinned to 1604 -- outside OmniSim's 1234-1294 auto-scan range
  and away from the sibling arms -- so a ladder run cannot collide with an
  engine another lane is running on this shared machine.
* Nothing is ever killed by name or by pattern.  The only bound on a run is the
  ``timeout`` wrapping this arm's own child, so a run of this arm cannot leave
  a simulator behind and cannot touch another lane's processes.
* Rungs 9 and 11 are MULTI-RUN cells (CONTRACT.md amendment A): one
  ``run()`` call, several runs of one contract-owned scene family, **one
  engine launch each**, sequentially on the same pinned port.  That is not an
  implementation convenience on rung 9 -- ``distinct_processes`` asserts it,
  from the pid and process-start time each driver records for ITSELF, because
  a determinism rung whose replicas share a process measures the arm.

``run()`` never raises: a broken simulator is a measurement, and a raised
exception would abort the row, which is indistinguishable from a row that was
never run.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)

import rungs                                        # noqa: E402  (shared)


def _load_sibling(name, filename):
    """Import a module from THIS arm's directory, by path and under a unique
    name.

    Deliberately not ``import worldgen``.  All three arms have a module called
    ``worldgen``, so a bare import resolves to whichever arm ``run_ladder.py``
    happened to load first -- this arm would then generate, and measure, the
    wrong simulator's scenes.  A path load under an arm-qualified name cannot
    collide.  (A bare import also only works when this file is executed
    directly, which is how it escaped notice until the shared runner imported
    the arm by path.)
    """
    path = os.path.join(HERE, filename)
    sp = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(sp)
    sys.modules[name] = mod
    sp.loader.exec_module(mod)
    return mod


worldgen = _load_sibling("ladder0_webots_worldgen", "worldgen.py")

NAME = "webots"
WEBOTS_HOME = os.environ.get("LADDER0_WEBOTS_HOME",
                             "/opt/upstream-webots/R2025a")
PORT = 1604
DEFAULT_TIMEOUT_S = 240.0

# Faults this arm implements end to end (CONTRACT.md section 6).  Two are
# scene-level and live in worldgen.py; three are driver-level and live in the
# controller.  Anything else is reported as unsupported rather than silently
# run as a control -- a fault that did not happen must never read as a fault
# that produced no failure.
SUPPORTED_FAULTS = {"none", "short_run", "no_floor", "half_gravity",
                    "ignore_zero", "slide",
                    # rungs 5-8.  CONTRACT.md section 6 does not name faults
                    # for these rungs yet, so these are this arm's own, chosen
                    # to the same standard: each reddens the assertion it
                    # targets and leaves the rest of the rung green.
                    "frozen_sensor", "late_stop", "crosstalk", "weak_grip",
                    # rungs 9 and 11, and these ARE the contract's
                    # (selftest.LIVE_FAULTS).  ``seed_nudge`` and ``frozen``
                    # are scene faults; ``short_b`` is a driver fault; the two
                    # rung-11 faults are spawn-time and apply at
                    # RUNG11_FAULT_N only.
                    "seed_nudge", "frozen", "short_b",
                    "stalled_robot", "lane_offset"}

#: Rungs this arm has NOT BUILT, and why.  ``run_ladder.py`` reports these as
#: not run, which is an UNKNOWN -- and an unknown is not a pass.
#:
#: This is deliberately NOT ``NOT_EXPRESSIBLE``.  N/E means the SIMULATOR
#: structurally cannot express the quantity, declared per check with the
#: missing capability named and cited (CONTRACT.md amendment B); using it here
#: would charge an engine for this lane's backlog.  ODE could express rung 18
#: perfectly well.  Nobody has written the arm.
UNIMPLEMENTED = {
    18: ("ODE's agreement with the lane1r cube-toss recording has not been "
         "measured by us and, as far as this ladder knows, by anyone -- the "
         "published baselines rung 18 is judged against are MuJoCo, Drake, "
         "Bullet and Dart, not ODE.  An arm for it is real work (per-toss "
         "initial-condition injection, orientation readback in the "
         "recording's quaternion convention, and one engine launch per toss) "
         "and it has not been done, so this row is an UNKNOWN rather than a "
         "failure and rather than NOT_EXPRESSIBLE."),
}

_MARKER = re.compile(r"^(RC|T0|T1|VERSION|BINSHA|RUNG|TAG)=(.*)$")


def win_to_wsl(p):
    """``O:\\omnisim\\x`` -> ``/mnt/o/omnisim/x``."""
    p = os.path.abspath(p)
    drive, rest = os.path.splitdrive(p)
    if not drive:
        return p.replace("\\", "/")
    return "/mnt/%s%s" % (drive[0].lower(), rest.replace("\\", "/"))


def _wsl(args, timeout):
    """Run one command inside WSL.  ``args`` are separate argv entries."""
    return subprocess.run(["wsl.exe", "-e"] + args, capture_output=True,
                          text=True, timeout=timeout, encoding="utf-8",
                          errors="replace")


def available():
    """(True, None) if this arm can run here; (False, why) otherwise."""
    try:
        r = _wsl(["bash", "-lc",
                  'test -x "%s/webots" && echo OK' % WEBOTS_HOME], 60)
    except FileNotFoundError:
        return False, "wsl.exe not found (this arm needs WSL2)"
    except Exception as exc:                        # noqa: BLE001
        return False, "WSL probe failed: %r" % (exc,)
    if "OK" in (r.stdout or ""):
        return True, None
    return False, ("upstream Webots not executable at %s inside WSL"
                   % WEBOTS_HOME)


def _markers(text):
    out = {}
    for line in (text or "").splitlines():
        m = _MARKER.match(line.strip())
        if m:
            out[m.group(1)] = m.group(2).strip()
    return out


#: Per-rung sample stride, read from the CONTRACT (amendment D) and carried to
#: the driver.  The arm never picks one: a stride chosen here would decimate
#: the very series rung 9 measures, at a rate nothing in the table records.
STRIDE = {9: getattr(rungs, "RUNG9_SAMPLE_EVERY", 1),
          11: getattr(rungs, "RUNG11_SAMPLE_EVERY", 1)}


def run(rung, out_dir, fault="none", timeout_s=DEFAULT_TIMEOUT_S,
        subset=None, **_kw):
    """Run one cell.  Returns ``(samples, meta)`` and never raises.

    THE SCENE IS NOT A PARAMETER: the world is derived from
    ``(rung, fault, tag)`` and there is no override, because a row produced
    from a scene the contract did not describe is not comparable with the other
    arms' rows and nothing in the table would say which scene it came from.

    ``subset`` is rung 18's and this arm does not implement rung 18 (see
    :data:`UNIMPLEMENTED`); it is accepted and ignored so the shared runner and
    self-test can pass it uniformly.
    """
    rung = int(rung)
    fault = fault or "none"
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    meta = {
        "sim": NAME, "rung": rung, "fault": fault,
        "engine": "upstream Webots R2025a", "webots_home": WEBOTS_HOME,
        "port": PORT, "error": None, "exit_code": None, "timed_out": False,
        "proc_t0": None, "proc_t1": None,
    }
    empty = {"rung": rung, "sim": NAME, "t": [], "steps": 0}

    if fault not in SUPPORTED_FAULTS:
        meta["error"] = ("this arm does not implement fault %r (supported: %s)"
                         % (fault, ", ".join(sorted(SUPPORTED_FAULTS))))
        return empty, meta
    if rung in UNIMPLEMENTED:
        meta["error"] = "rung %d not implemented on this arm: %s" % (
            rung, UNIMPLEMENTED[rung])
        return empty, meta

    # Check every scene against the contract before every cell: a committed
    # world that had drifted from rungs.py would be measured against the wrong
    # expectation.  ``write_all`` compares first and writes only what differs
    # -- see its docstring; rewriting the files unconditionally raced the WSL
    # side's open handles and cost two rung-11 fault cells.
    try:
        worldgen.write_all(verbose=False)
        specs = worldgen.run_specs(rung, fault)
    except Exception as exc:                        # noqa: BLE001
        meta["error"] = "worldgen failed: %r" % (exc,)
        return empty, meta

    if len(specs) == 1 and specs[0].get("tag") is None:
        return _launch(rung, out_dir, fault, specs[0], timeout_s, meta)
    return _multi_run(rung, out_dir, fault, specs, timeout_s, meta)


def _multi_run(rung, out_dir, fault, specs, timeout_s, meta):
    """One cell, several runs, ONE ENGINE LAUNCH EACH.  Never raises.

    Sequential, on the one pinned port: a parallel launch would put two
    engines on this shared machine at once for no measurement gain, and on
    rung 9 it would also make the two replicas contend for the same CPU, which
    is a difference between them that the rung would then be measuring.
    """
    runs = []
    meta["multi_run"] = True
    meta["run_tags"] = [s.get("tag") for s in specs]
    meta["runs"] = []
    for spec in specs:
        tag = str(spec.get("tag"))
        sub = {}
        rec, mt = _launch(rung, os.path.join(out_dir, tag), fault, spec,
                          timeout_s, sub)
        rec["tag"] = spec.get("tag")
        rec["params"] = {k: v for k, v in spec.items() if k != "tag"}
        runs.append(rec)
        meta["runs"].append({k: v for k, v in mt.items()
                             if k in ("world", "exit_code", "error",
                                      "timed_out", "proc_t0", "proc_t1",
                                      "engine_version", "engine_sha256_16",
                                      "world_info")})
        if mt.get("error") and not meta["error"]:
            meta["error"] = "run %s: %s" % (tag, mt["error"])
        if mt.get("exit_code"):
            meta["exit_code"] = mt["exit_code"]
        meta["timed_out"] = meta["timed_out"] or bool(mt.get("timed_out"))
        for k in ("engine_version", "engine_sha256_16"):
            if meta.get(k) is None:
                meta[k] = mt.get(k)
    if meta["exit_code"] is None:
        meta["exit_code"] = 0
    stamps0 = [m.get("proc_t0") for m in meta["runs"] if m.get("proc_t0")]
    stamps1 = [m.get("proc_t1") for m in meta["runs"] if m.get("proc_t1")]
    meta["proc_t0"] = min(stamps0) if stamps0 else None
    meta["proc_t1"] = max(stamps1) if stamps1 else None
    # WHAT CONFIGURATION DID THIS ROW COME FROM.  Read back out of every
    # loaded scene by the recorder itself, then collapsed: a list rather than a
    # scalar, so two runs that somehow disagreed could not average into a
    # number neither of them used.
    meta["world_info"] = _collapse_world_info(runs)
    samples = {"rung": rung, "sim": NAME, "fault": fault, "runs": runs,
               # The wall block is the CELL's, spanning every run in it.
               "wall": {"t_start": meta["proc_t0"],
                        "t_first_step": _first_step(runs),
                        "t_end": meta["proc_t1"]}}
    return samples, meta


def _collapse_world_info(runs):
    seen = []
    for r in runs:
        wi = r.get("world_info")
        if wi and wi not in seen:
            seen.append(wi)
    if not seen:
        return None
    return seen[0] if len(seen) == 1 else seen


def _first_step(runs):
    stamps = [(r.get("wall") or {}).get("t_first_step") for r in runs]
    stamps = [s for s in stamps if s]
    return min(stamps) if stamps else None


def _launch(rung, out_dir, fault, spec, timeout_s, meta):
    """One engine launch of one world.  Returns ``(sample document, meta)``."""
    os.makedirs(out_dir, exist_ok=True)
    tag = spec.get("tag")
    empty = {"rung": rung, "sim": NAME, "t": [], "steps": 0}
    meta.setdefault("sim", NAME)
    meta.setdefault("rung", rung)
    meta.setdefault("fault", fault)
    meta.setdefault("error", None)
    meta.setdefault("exit_code", None)
    meta.setdefault("timed_out", False)
    meta["tag"] = tag

    world = worldgen.world_path(rung, fault, tag)
    meta["world"] = world
    if not os.path.exists(world):
        meta["error"] = "world missing: %s (run --regen-worlds)" % world
        return empty, meta
    samples_path = os.path.join(out_dir, "samples.json")
    console_path = os.path.join(out_dir, "console.log")
    for stale in (samples_path, console_path):
        if os.path.exists(stale):
            try:
                os.remove(stale)
            except OSError:
                pass

    argv = ["bash", win_to_wsl(os.path.join(HERE, "run_cell.sh")),
            str(rung), fault, win_to_wsl(out_dir), win_to_wsl(world),
            str(int(timeout_s)), str(PORT),
            # The per-RUN values.  They reach the controller as environment,
            # never as controllerArgs -- rung 9's replicas a and b are the same
            # world file on purpose.
            "" if tag is None else str(tag),
            str(int(STRIDE.get(rung, 1))),
            repr(float(spec.get("short") or 1.0))]
    meta["cmd"] = "wsl.exe -e " + " ".join(argv)

    t0 = time.time()
    try:
        proc = _wsl(argv, timeout_s + 120.0)
        text = (proc.stdout or "") + (proc.stderr or "")
    except subprocess.TimeoutExpired as exc:
        text = "OUTER-TIMEOUT %r" % (exc,)
        meta["timed_out"] = True
    except Exception as exc:                        # noqa: BLE001
        text = "WSL-LAUNCH-FAILED %r" % (exc,)
        meta["error"] = text
    t1 = time.time()

    mk = _markers(text)
    meta["exit_code"] = int(mk["RC"]) if mk.get("RC", "").lstrip("-").isdigit() \
        else None
    meta["timed_out"] = meta["timed_out"] or meta["exit_code"] == 124
    meta["engine_version"] = mk.get("VERSION")
    meta["engine_sha256_16"] = mk.get("BINSHA")
    # Prefer the in-WSL marks; fall back to the Windows-side wall clock only if
    # the script never got far enough to print them.
    meta["proc_t0"] = float(mk["T0"]) if "T0" in mk else t0
    meta["proc_t1"] = float(mk["T1"]) if "T1" in mk else t1
    meta["launcher_output"] = text[-2000:]

    samples = empty
    if os.path.exists(samples_path):
        try:
            with open(samples_path, "r", encoding="utf-8") as f:
                samples = json.load(f)
            samples["sim"] = NAME
            # The configuration the ENGINE ran, read back out of the loaded
            # scene by the recorder.  Lifted into meta so the row says which
            # configuration it came from -- ``optimalThreadCount`` above 1 is
            # a parallel ODE island solve, which is exactly the class of
            # mechanism rung 9 exists to catch.
            if samples.get("world_info"):
                meta["world_info"] = samples["world_info"]
        except Exception as exc:                    # noqa: BLE001
            meta["error"] = "samples unreadable: %r" % (exc,)
    else:
        tail = ""
        if os.path.exists(console_path):
            with open(console_path, "r", encoding="utf-8",
                      errors="replace") as f:
                tail = "".join(f.readlines()[-25:])
        meta["error"] = ("no sample document was written; engine console "
                         "tail:\n%s" % tail)

    # Engine complaints are CONTEXT for a red row, never a verdict: every
    # verdict on this ladder comes from a physical quantity.  Upstream Webots
    # prints its complaints and exits 0 regardless, so they are worth carrying.
    if os.path.exists(console_path):
        with open(console_path, "r", encoding="utf-8", errors="replace") as f:
            console = f.read()
        meta["engine_complaints"] = [
            ln.strip() for ln in console.splitlines()
            if ("WARNING" in ln or "ERROR" in ln) and "ALSA" not in ln][:20]

    return samples, meta


__all__ = ["NAME", "available", "run", "PORT", "WEBOTS_HOME", "win_to_wsl",
           "SUPPORTED_FAULTS", "UNIMPLEMENTED", "STRIDE"]
