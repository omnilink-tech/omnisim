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

"""The OmniSim arm: one headless ``omnisim-bin`` per rung.

Launch shape is the repo's proven headless one -- ``--batch --mode=fast
--no-rendering --minimize --stdout --stderr``, no ``--port`` (a local
controller does not inherit one and then cannot connect), stdout to a real
file, wall-clock capped with a whole-tree kill on expiry.

Three pieces of hygiene that are not optional here:

* a UNIQUE ``OMNISIM_LOG_PATH`` per run.  The default is the shared
  ``$OMNISIM_HOME/omnisim_log.txt`` and this tree is worked in by several lanes
  at once; sharing it would also make the Newton verdict sidecar ambiguous.
* ``OMNISIM_REQUIRE_NEWTON=1``.  Newton is the only physics backend, and a
  clone whose runtime is merely absent otherwise takes a one-shot ERROR and
  keeps going.  A ladder row from a run with no physics would be worse than
  no row.
* the retired ODE selectors are POPPED from the child env, so a stale export
  in the operator's shell cannot steer a measurement.

The run is verified by the race-free ``<log>.newton.json`` sidecar rather than
by the exit code: exit code 0 proves nothing here (a world that never stepped,
a controller hung at zero ticks by a stale libController, and a clean run all
exit 0).  The sample file's own step count is the second gate.
"""

from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))       # .../ladder0/omnisim
LADDER0 = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(LADDER0, os.pardir, os.pardir, os.pardir))
if LADDER0 not in sys.path:                     # the SHARED contract only
    sys.path.insert(0, LADDER0)


def _load_sibling(stem):
    """Load one of this arm's own modules BY PATH, under an arm-qualified name.

    Never ``import worldgen``.  ``run_ladder.py`` loads all three arms into one
    process and ``sys.modules`` is global to it; all three ship a module called
    ``worldgen``, so a bare import in the second arm loaded resolves to the
    FIRST arm's generator.  That arm would then generate AND measure another
    simulator's scenes while reporting them as its own -- a row that is wrong in
    a way no assertion in this ladder can see, because every number in it is
    internally consistent.  ``run_ladder.load_arm`` refuses the collision
    outright, and reported this arm's bare import as an IMPORT HAZARD until it
    was fixed here.
    """
    key = "ladder0_omnisim_%s" % stem
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "%s.py" % stem))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


import rungs                                     # noqa: E402  (shared contract)

worldgen = _load_sibling("worldgen")

NAME = "omnisim"
IS_WIN = os.name == "nt"
BASE_ARGS = ("--batch", "--mode=fast", "--no-rendering", "--minimize",
             "--stdout", "--stderr")
DEFAULT_TIMEOUT_S = 180.0


def resolve_binary(explicit=None):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    cands = ([os.path.join(REPO, "msys64", "mingw64", "bin",
                           "omnisim-bin.exe")] if IS_WIN else
             [os.path.join(REPO, "bin", "omnisim-bin"),
              os.path.join(REPO, "omnisim-bin")])
    for c in cands:
        if os.path.exists(c):
            return c
    return None


def available(binary=None):
    b = resolve_binary(binary)
    return (b is not None), (None if b else "omnisim-bin not found under %s"
                             % REPO)


def _env(log_path, out_path):
    env = dict(os.environ)
    env["OMNISIM_HOME"] = REPO
    env["WEBOTS_HOME"] = REPO          # a stale machine-scope value points at
    env["OMNISIM_LOG_PATH"] = log_path  # a phantom install; override it
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    env["LADDER0_OUT"] = out_path
    for k in ("OMNISIM_LEGACY", "OMNISIM_FORCE_ODE",
              "OMNISIM_ALLOW_ODE_FALLBACK"):
        env.pop(k, None)
    return env


def _kill_tree(proc):
    if IS_WIN:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)
    else:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:                        # noqa: BLE001
            proc.kill()


def spawn(argv, env, console, timeout_s):
    """One engine process, wall-clock capped, whole tree killed on expiry.

    Extracted so rung 18 -- which drives lane1r's world and reads an npz rather
    than this arm's own sample file -- carries EXACTLY the same launch hygiene
    as a ladder row: unique log path, ``OMNISIM_REQUIRE_NEWTON``, the retired
    ODE selectors popped from the child env, and the verdict sidecar read back.
    A measurement taken under weaker hygiene is not comparable with the row it
    is supposed to explain.
    """
    kwargs = {} if IS_WIN else {"start_new_session": True}
    timed_out = False
    t0 = time.time()
    with open(console, "w", encoding="utf-8", errors="replace") as logf:
        proc = subprocess.Popen(argv, env=env, cwd=REPO, stdout=logf,
                                stderr=subprocess.STDOUT, **kwargs)
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            _kill_tree(proc)
            rc = proc.wait()
    return rc, t0, time.time(), timed_out, proc.pid


def _sidecar(log_path):
    p = log_path + ".newton.json"
    if not os.path.exists(p):
        return {"present": False}
    try:
        with open(p, "r", encoding="utf-8", errors="replace") as f:
            d = json.load(f)
        d["present"] = True
        return d
    except Exception as exc:                     # noqa: BLE001
        return {"present": True, "unreadable": repr(exc)}


def _tail(path, n=30):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except OSError:
        return ""


#: Per-rung sample stride, read from the contract (amendment D).  The arm
#: carries it to the driver and never picks one.
STRIDE = {9: getattr(rungs, "RUNG9_SAMPLE_EVERY", 1),
          11: getattr(rungs, "RUNG11_SAMPLE_EVERY", 1)}


def run(rung, out_dir, fault="none", binary=None,
        timeout_s=DEFAULT_TIMEOUT_S, subset=None, **_kw):
    """-> (samples, meta).  Never raises; a failure comes back in meta.

    THE SCENE IS NOT A PARAMETER.  ``world`` is derived from ``(rung, fault)``
    and there is no override, because a row produced from a scene the contract
    did not describe is not comparable with the other arms' rows and nothing in
    the table would say which scene it came from.  Attribution runs against
    non-default engine settings go through ``variants.py``, which calls
    :func:`launch` directly and labels its output as NOT a ladder row.

    Rungs 9, 11 and 18 are MULTI-RUN cells (CONTRACT.md amendment A): one
    ``arm.run()`` call, several runs of one contract-owned scene family, each
    from ITS OWN PROCESS.  The replicas of rung 9 must be distinct processes or
    the rung measures this arm rather than the engine, and the reducer asserts
    it from the pid each driver records for itself.
    """
    rung = int(rung)
    out_dir = os.path.abspath(out_dir)
    if rung == 18:
        return _load_sibling("rung18").run(
            out_dir, fault=fault, binary=binary, timeout_s=timeout_s,
            subset=subset)
    specs = worldgen.run_specs(rung, fault)
    if len(specs) == 1 and specs[0].get("tag") is None:
        return launch(rung, out_dir, worldgen.world_path(rung, fault),
                      fault=fault, binary=binary, timeout_s=timeout_s)
    return _multi_run(rung, out_dir, specs, fault=fault, binary=binary,
                      timeout_s=timeout_s)


def _multi_run(rung, out_dir, specs, fault="none", binary=None,
               timeout_s=DEFAULT_TIMEOUT_S):
    """One cell, several runs, one process each.  Never raises."""
    runs, metas = [], []
    meta = {"sim": NAME, "rung": rung, "fault": fault, "error": None,
            "exit_code": 0, "timed_out": False, "multi_run": True,
            "run_tags": [s.get("tag") for s in specs]}
    for spec in specs:
        tag = spec.get("tag")
        d = os.path.join(out_dir, str(tag))
        world = worldgen.world_path(rung, fault, tag)
        env_extra = {"LADDER0_TAG": str(tag),
                     "LADDER0_STRIDE": str(STRIDE.get(rung, 1))}
        if spec.get("short"):
            env_extra["LADDER0_SHORT"] = repr(float(spec["short"]))
        rec, mt = launch(rung, d, world, fault=fault, binary=binary,
                         timeout_s=timeout_s, env_extra=env_extra)
        rec["tag"] = tag
        rec["params"] = {k: v for k, v in spec.items() if k != "tag"}
        runs.append(rec)
        metas.append({k: v for k, v in mt.items()
                      if k in ("world", "exit_code", "error", "timed_out",
                               "newton", "binary_sha256", "binary_mtime",
                               "proc_t0", "proc_t1")})
        if mt.get("error") and not meta["error"]:
            meta["error"] = "run %s: %s" % (tag, mt["error"])
        if mt.get("exit_code"):
            meta["exit_code"] = mt["exit_code"]
        meta["timed_out"] = meta["timed_out"] or bool(mt.get("timed_out"))
        for k in ("binary", "binary_sha256", "binary_mtime"):
            meta.setdefault(k, mt.get(k))
    meta["runs"] = metas
    meta["proc_t0"] = metas[0].get("proc_t0") if metas else None
    meta["proc_t1"] = metas[-1].get("proc_t1") if metas else None
    meta["newton"] = metas[0].get("newton") if metas else None
    samples = {"rung": rung, "sim": NAME, "fault": fault, "runs": runs,
               # The wall block is the CELL's, spanning every run in it.
               "wall": {"t_start": meta["proc_t0"],
                        "t_first_step": _first_step(runs),
                        "t_end": meta["proc_t1"]}}
    return samples, meta


def _first_step(runs):
    stamps = [(r.get("wall") or {}).get("t_first_step") for r in runs]
    stamps = [s for s in stamps if s]
    return min(stamps) if stamps else None


def launch(rung, out_dir, world, fault="none", binary=None,
           timeout_s=DEFAULT_TIMEOUT_S, env_extra=None):
    """One headless engine run of ``world``, sampled by ``ladder0_probe``.

    Shared by :func:`run` and by ``variants.py`` so an attribution run carries
    the same hygiene as a ladder row: unique log path, ``OMNISIM_REQUIRE_NEWTON``,
    the retired ODE selectors popped, the verdict sidecar read back.  A variant
    measured under weaker hygiene would not be comparable with the row it is
    supposed to explain.
    """
    rung = int(rung)
    # Absolute, defensively: LADDER0_OUT is read by a child process the engine
    # starts from the CONTROLLER's directory, so a relative path here writes
    # the run into the controller tree and reads back as "no samples".
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    binpath = resolve_binary(binary)
    samples_path = os.path.join(out_dir, "samples.json")
    log_path = os.path.join(out_dir, "engine.log")
    console = os.path.join(out_dir, "console.log")
    meta = {"sim": NAME, "rung": rung, "world": world, "fault": fault,
            "binary": binpath, "error": None, "exit_code": None,
            "timed_out": False}
    if binpath is None:
        meta["error"] = "omnisim-bin not found"
        return {"rung": rung, "sim": NAME, "t": [], "steps": 0}, meta
    meta["binary_sha256"] = _sha256(binpath)
    meta["binary_mtime"] = _mtime(binpath)

    if os.path.exists(samples_path):
        os.remove(samples_path)
    if not os.path.exists(world):
        meta["error"] = ("world missing: %s (run --regen-worlds)" % world)
        return {"rung": rung, "sim": NAME, "t": [], "steps": 0}, meta
    argv = [binpath, world] + list(BASE_ARGS)

    env = _env(log_path, samples_path)
    if env_extra:
        env.update({str(k): str(v) for k, v in env_extra.items()})
        meta["env_extra"] = dict(env_extra)
    rc, t0, t1, timed_out, pid = spawn(argv, env, console, timeout_s)
    meta["timed_out"] = timed_out
    meta["exit_code"] = rc
    meta["proc_t0"] = t0
    meta["proc_t1"] = t1
    meta["pid"] = pid
    meta["newton"] = _sidecar(log_path)
    meta["command"] = " ".join(argv)

    rec = {"rung": rung, "sim": NAME, "t": [], "steps": 0}
    if os.path.exists(samples_path):
        try:
            with open(samples_path, "r", encoding="utf-8") as f:
                rec = json.load(f)
            rec["sim"] = NAME
        except Exception as exc:                 # noqa: BLE001
            meta["error"] = "samples unreadable: %r" % (exc,)
    else:
        meta["error"] = ("no samples file; console tail:\n%s\nengine tail:\n%s"
                         % (_tail(console, 20), _tail(log_path, 20)))
    # Provenance gate.  The sidecar is written at world FINALIZE, and rung 0's
    # world has no dynamic body at all, so Newton has nothing to build and
    # writes none -- measured, and correct.  Requiring it there would red a
    # rung for the absence of physics the rung deliberately does not contain,
    # so the gate applies only to the rungs whose verdict depends on a solver.
    needs_physics = rung != 0
    if not meta["newton"].get("present"):
        msg = ("no .newton.json verdict sidecar: Newton did not finalize "
               "this world")
        if needs_physics:
            meta["error"] = (meta["error"] or "") + " | " + msg
        else:
            meta["note"] = msg + " (expected: rung 0 has no dynamic body)"
    elif meta["newton"].get("degraded"):
        meta["error"] = ((meta["error"] or "") + " | Newton reports degraded")
    return rec, meta


def _sha256(path):
    import hashlib
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()[:16]
    except OSError:
        return None


def _mtime(path):
    try:
        return time.strftime("%Y-%m-%d %H:%M:%S",
                             time.localtime(os.path.getmtime(path)))
    except OSError:
        return None


def which_binaries():
    """Every ``omnisim-bin*`` next to the canonical one, for A/B work."""
    d = os.path.dirname(resolve_binary() or "")
    if not d or not os.path.isdir(d):
        return []
    out = []
    for n in sorted(os.listdir(d)):
        if n.startswith("omnisim-bin") and n.endswith(".exe"):
            p = os.path.join(d, n)
            out.append({"name": n, "path": p, "mtime": _mtime(p),
                        "sha256": _sha256(p),
                        "size": os.path.getsize(p)})
    return out


__all__ = ["NAME", "available", "run", "launch", "resolve_binary",
           "which_binaries"]
