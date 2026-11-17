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

"""Compile a world for the core, with the ENGINE as the compiler, and keep the result.

One short headless engine load (scripts/core/compile_hook.py) saves the exact
MuJoCo model, the starting state and the clamp tables. The world file supplies
the names (names.py). Nothing about how the engine builds physics is repeated
here, so the core cannot drift from the engine on what a world IS.

THE CACHE. A machine twin runs many jobs, and every job's world is the same
machine with a different stock, a different program and the Z axis parked at a
different height. Compiled worlds are kept under .tmp/omnisim-core-cache, keyed
by a signature of the world with exactly those per-job parts left out:

  * MillingStock nodes (the stock is carved by the core, it is not a body);
  * controllerArgs of controllers known to send the same first command
    whatever their arguments (`ARGS_FREE_CONTROLLERS`);
  * a slider's rest translation and its motor's travel limits, which
    `names.retarget` writes into the cached model -- and ONLY when the engine's
    own model for the compiled world held exactly the values `retarget` would
    have written (`retarget_is_exact`). Otherwise the entry serves that one
    world and nothing else.

Everything else in the file, every file it refers to, the engine binary, the
physics runtime and every OMNISIM_* setting is part of the key: a change to
any of them is a new compile, never a stale model.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from . import names as nm
from . import worldfile as wf
from .errors import CoreDeclined
from .runtime import HOME, bundle_dir, engine_binary

CACHE = HOME / ".tmp" / "omnisim-core-cache"
HOOK = "scripts.core.compile_hook:tick"
FILES = ("model.mjb", "state0.npy", "clamp.npz", "engine.json")

# Controllers whose first command to the machine does not depend on their arguments (the engine's starting
# state includes the effect of that first command, so only these may share a compile across argument lists).
ARGS_FREE_CONTROLLERS = frozenset(("grbl_twin",))

# OMNISIM_* settings that name where a run writes, not how the world is built (compared without the prefix).
_PER_RUN = frozenset(("LOG_PATH", "MILLING_STOCK_DUMP", "CNC_TWIN_TRACE", "CNC_TWIN_TRACE_PERIOD_S", "CNC_TWIN_METER_EVERY",
                      "CNC_TWIN_WAIT_SENDER", "CNC_TWIN_PROFILE",
                      "CORE_COMPILE_OUT", "INENGINE_PYMOD", "TWIN_BACKEND", "HOME"))
_PREFIX = "OMNISIM" + "_"


def _stat(path):
    try:
        st = os.stat(path)
        return [str(path).replace("\\", "/"), st.st_mtime_ns, st.st_size]
    except OSError:
        return None


def _environment_stamp():
    env = sorted((k, v) for k, v in os.environ.items() if k.startswith(_PREFIX) and k[len(_PREFIX):] not in _PER_RUN)
    b = bundle_dir()
    return {"engine": _stat(engine_binary()) if engine_binary() else None,
            "runtime": _stat(b / "omnisim_newton_runtime.py") if b else None,
            "hook": _stat(HOME / "scripts" / "core" / "compile_hook.py"), "env": env}


def signature(world_path, machine=True):
    """A key for the compiled form of this world. machine=True leaves out what a job changes on a machine."""
    world_path = os.path.abspath(str(world_path))
    wdir = os.path.dirname(world_path)
    with open(world_path, encoding="utf-8") as f:
        text = f.read()
    ext, nodes = wf.parse_file(text)
    files = []

    def note_file(s):
        if "\n" in s or len(s) > 300:
            return
        try:
            p = wf.resolve_url(s, wdir)
        except Exception:
            return
        st = _stat(p) if os.path.isfile(p) else None
        if st:
            files.append(st)

    def canon(v, drop=()):
        if isinstance(v, wf.Node):
            if machine and v.type == "MillingStock":
                return "~MillingStock"
            if v.type == "Viewpoint":
                return "~Viewpoint"
            ctl = str(v.get("controller", [""])[0]) if v.type in ("Robot", "URDFRobot") else None
            out = {"": v.type + ((" " + v.defname) if v.defname else "")}
            for k in sorted(v.fields):
                if k in drop:
                    continue
                if machine and k == "controllerArgs" and ctl in ARGS_FREE_CONTROLLERS:
                    continue
                if v.type == "WorldInfo" and k in ("title", "info"):
                    continue
                sub = ()
                if machine and v.type == "SliderJoint":
                    sub = ("translation",) if k == "endPoint" else (("minPosition", "maxPosition") if k == "device" else ())
                out[k] = canon(v.fields[k], sub)
            return out
        if isinstance(v, list):
            return [canon(x, drop) for x in v]
        if isinstance(v, wf.Ref):
            return "%s %s" % (v.kind, v.name)
        if isinstance(v, wf.Str):
            note_file(str(v))
            return "s:" + str(v)
        return v

    tree = [canon(n) for n in nodes] if machine else text
    for url in ext.values():
        note_file(str(url))
    blob = json.dumps({"tree": tree, "ext": sorted(ext.items()), "files": sorted(files), "stamp": _environment_stamp(),
                       "machine": bool(machine), "format": 1}, sort_keys=True, default=str)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:24]


def engine_compile(world, out_dir, timeout_s=300.0):
    """One short headless engine load of `world`; the hook writes the compiled world into `out_dir`."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        (out_dir / name).unlink(missing_ok=True)
    env = dict(os.environ)
    env["OMNISIM_INENGINE_PYMOD"] = HOOK
    env["OMNISIM_CORE_COMPILE_OUT"] = str(out_dir)
    env["OMNISIM_LOG_PATH"] = str(out_dir / "compile_engine.log")
    env["OMNISIM_MILLING_STOCK_DUMP"] = str(out_dir / "compile_discard.zmap")
    env["OMNISIM_CNC_TWIN_TRACE"] = str(out_dir / "compile_discard_trace.jsonl")
    last = ""
    # a plain load check stops shortly after the first physics step; a slow world gets a longer second try
    for extra in ([], ["--duration", "45"]):
        try:
            r = subprocess.run([sys.executable, "-m", "omnisim", "run-headless", str(world)] + extra, cwd=HOME, env=env,
                               capture_output=True, text=True, timeout=timeout_s)
            last = (r.stdout or "")[-300:] + (r.stderr or "")[-300:]
        except subprocess.TimeoutExpired:
            last = "the engine load did not finish in %d s" % timeout_s
        if (out_dir / "model.mjb").exists() and (out_dir / "engine.json").exists():
            break
    for junk in ("compile_discard.zmap", "compile_discard_trace.jsonl"):
        (out_dir / junk).unlink(missing_ok=True)
    if not (out_dir / "engine.json").exists():
        raise CoreDeclined("the engine did not compile this world for the core: " + (last.strip() or "no physics step was reached"))
    info = json.loads((out_dir / "engine.json").read_text(encoding="utf-8"))
    if not info.get("ok") or not (out_dir / "model.mjb").exists():
        raise CoreDeclined(info.get("why") or "the engine did not reach a physics step")
    return info


def _entry(key):
    p = CACHE / key / "entry.json"
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _lock(key, wait_s=240.0):
    """-> True when this process holds the compile lock for `key`; False when another process finished the entry."""
    CACHE.mkdir(parents=True, exist_ok=True)
    lock = CACHE / (key + ".lock")
    end = time.time() + wait_s
    while True:
        try:
            fd = os.open(str(lock), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.write(fd, str(os.getpid()).encode())
            os.close(fd)
            return True
        except FileExistsError:
            if _entry(key) is not None:
                return False
            try:
                if time.time() - lock.stat().st_mtime > 600.0:      # a compile that died with its lock
                    lock.unlink(missing_ok=True)
                    continue
            except OSError:
                continue
            if time.time() > end:
                raise CoreDeclined("another process has been compiling this machine for over %d s" % wait_s)
            time.sleep(0.5)


def _store(key, src, entry):
    tmp = CACHE / ("%s.tmp-%d" % (key, os.getpid()))
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    for name in FILES:
        if (Path(src) / name).exists():
            shutil.copy2(Path(src) / name, tmp / name)
    (tmp / "entry.json").write_text(json.dumps(entry, indent=1), encoding="utf-8")
    try:
        os.replace(tmp, CACHE / key)
    except OSError:                 # another process stored the same key first: theirs is as good
        shutil.rmtree(tmp, ignore_errors=True)


def _take(key, job_dir):
    for name in FILES:
        src = CACHE / key / name
        if src.exists():
            shutil.copy2(src, Path(job_dir) / name)


def compiled_for(world, job_dir, use_cache=True):
    """Put the compiled form of `world` into `job_dir` (model, state, clamp tables, names). -> meta dict.

    Raises CoreDeclined when the engine cannot compile it or a joint of the world cannot be found in the model.
    """
    job_dir = Path(job_dir)
    job_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    names = nm.world_names(world)
    mkey, xkey = signature(world, machine=True), signature(world, machine=False)
    how, held = None, None

    def usable(key):
        e = _entry(key)
        if e is None:
            return False
        return e.get("exact_key") == xkey or (key == mkey and e.get("retarget_exact"))

    try:
        if use_cache:
            for key in (mkey, xkey):
                if usable(key):
                    _take(key, job_dir)
                    how = "cache"
                    break
            if how is None and _lock(mkey):
                held = mkey
            elif how is None and usable(mkey):       # another process compiled it while this one waited
                _take(mkey, job_dir)
                how = "cache"
        if how is None:
            engine_compile(world, job_dir)
            how = "compiled"
        rep = nm.match(names, job_dir / "model.mjb")
        if rep["unmatched"]:
            raise CoreDeclined("joints of the world not found in the engine's model: %s" % ", ".join(rep["unmatched"][:4]))
        if how == "compiled":
            exact = nm.retarget_is_exact(names, job_dir / "model.mjb")
            if use_cache:
                entry = {"exact_key": xkey, "retarget_exact": bool(exact), "world": str(world), "compiled_at": time.strftime("%Y-%m-%d %H:%M:%S")}
                _store(mkey if _entry(mkey) is None else xkey, job_dir, entry)
            changed = []
        else:
            changed = nm.retarget(names, job_dir)
    finally:
        if held:
            (CACHE / (held + ".lock")).unlink(missing_ok=True)
    engine = json.loads((job_dir / "engine.json").read_text(encoding="utf-8"))
    meta = {"world": str(world), "names": names, "engine": engine,
            "compile": {"how": how, "seconds": round(time.time() - t0, 2), "machine_key": mkey, "retargeted": changed,
                        "joints_matched": rep["joints"], "worst_match_mm": round(rep["worst_mm"], 4)}}
    (job_dir / "meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    return meta


def clear_cache():
    n = sum(1 for p in CACHE.glob("*") if p.is_dir()) if CACHE.exists() else 0
    shutil.rmtree(CACHE, ignore_errors=True)
    return n
