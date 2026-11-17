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

"""Run a compiled world in ONE process: every robot's unmodified controller on the core.

Each controller script runs as a greenlet. Its `robot.step(ms)` hands control
back to this loop, which wakes every controller that is due, steps the core
once, carves the milling stocks, and repeats: the engine's order, without the
per-step message between processes.

Exit status: 0 the run ended (tick limit, stop file, or every controller
returned); 3 a controller asked for something the core does not model; 4 a
controller raised or exited on its own with an error.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import runpy
import sys
import time
import types
from pathlib import Path

from . import names as nm
from .errors import CoreUnsupported
from .runtime import HOME

EXIT_OK, EXIT_UNSUPPORTED, EXIT_CONTROLLER = 0, 3, 4
CONTROLLER_DIRS = ("projects/default/controllers", "projects/samples/demos/controllers")


def find_controller(name, world_path=None):
    """The controller script, looked up as the engine does: beside the world's project first, then the shared ones."""
    dirs = []
    if world_path:
        dirs.append(Path(world_path).resolve().parent.parent / "controllers")
    dirs += [HOME / d for d in CONTROLLER_DIRS]
    for d in dirs:
        p = d / name / (name + ".py")
        if p.is_file():
            return p
    return None


def _stall_watchdog(core, path, quiet_s=30.0):
    """When simulated time stops advancing for `quiet_s` of wall time, write every thread's stack to `path`.

    A controller is ordinary Python running in this process: one that blocks (a socket, a lock, a wait)
    stops the whole world, and from outside that looks like a machine that never goes idle. The file says
    where it is standing. -> an Event; set it to stop the watchdog.
    """
    import faulthandler
    import threading
    stop = threading.Event()
    if path is None:
        return stop

    def watch():
        last, since, dumped = core.ticks, time.time(), False
        while not stop.wait(5.0):
            if core.ticks != last:
                last, since, dumped = core.ticks, time.time(), False
            elif not dumped and time.time() - since >= quiet_s:
                with open(path, "w", encoding="utf-8") as f:
                    f.write("no simulation tick for %.0f s of wall time; stopped at tick %d\n\n" % (time.time() - since, core.ticks))
                    f.flush()
                    faulthandler.dump_traceback(file=f, all_threads=True)
                dumped = True

    threading.Thread(target=watch, name="core-stall-watchdog", daemon=True).start()
    return stop


def _stand_in(core_mod):
    mod = types.ModuleType("omnisim")
    mod.Supervisor = core_mod.Supervisor
    mod.Robot = core_mod.Robot
    mod.__file__ = core_mod.__file__

    def refuse(name):
        if name.startswith("__"):
            raise AttributeError(name)
        raise CoreUnsupported("the core does not model omnisim.%s" % name)

    mod.__getattr__ = refuse
    return mod


def run(cdir, ticks=None, stock_dump=None, stop_file=None, strict=True, quiet=False, record=None, report=None,
        timeline_dir=None, keyframe_every=250, intervention=None, against=None):
    """Run the compiled world in `cdir`. -> (exit status, report dict).

    timeline_dir: keep a timeline of the run there (timeline.py: replay, inspect, branch, diff).
    intervention: (tick, [edits]) applied to the live run at that tick, the controllers still in the loop.
    against: a recording of the same world; the report says up to which tick this run matched it.
    """
    t_start = time.perf_counter()
    import greenlet
    import numpy as np

    from . import core as core_mod
    from . import milling

    cdir = Path(cdir)
    meta = json.loads((cdir / core_mod.META).read_text(encoding="utf-8"))
    names = meta["names"]
    core = core_mod.Core.from_compiled(cdir)
    max_ticks = int(ticks) if ticks else 10 ** 12
    dt = float(names["dt_ms"])
    # before a controller commands it, a position servo holds the joint's authored angle (not zero)
    for rob in names["robots"]:
        for mo in rob.get("motors", []):
            if mo["servo"] and mo.get("a_pos") is not None:       # an EFFORT joint has no servo to hold it
                core.ctrl[mo["a_pos"]] = float(np.float32(core.d.qpos[core.m.jnt_qposadr[mo["jid"]]]))

    recorder = interv = None
    if timeline_dir or intervention or against:
        from . import timeline
        if intervention:
            interv = timeline.Intervention(intervention[0], intervention[1], names)
        if timeline_dir or against:
            recorder = timeline.Recorder(core, keyframe_every)
            core.recorder = recorder

    stocks = [milling.MillingStock(ms, core, stock_dump) for ms in names.get("milling", [])]
    objects = {st.name: st for st in stocks}
    main_g = greenlet.getcurrent()
    ctls, not_run = [], []

    def scheduler(sup, ms):
        c = sup.cfg["ctl"]
        c["wake"] = core.ticks + max(1, int(round(ms / dt)))
        return main_g.switch()

    defs = {dn: info["body"] for dn, info in names["defs"].items() if "body" in info}
    frames = {}                    # DEF'd Pose/Transform frames fixed to a robot's Solid, with the body that carries them
    for rob in names["robots"]:
        for dn, fr in (rob.get("frames") or {}).items():
            if fr["solid"] == "":
                body = rob.get("root_body") if rob.get("bodies") else None
            else:
                sol = (rob.get("solids") or {}).get(fr["solid"])
                body = rob["joints"][sol["joint"]].get("body") if sol is not None else None
            if body is not None and int(body) > 0 and dn not in defs:
                frames[dn] = dict(fr, body=int(body))
    for rob in names["robots"]:
        ctl_name = rob["controller"]
        if ctl_name in nm.NO_CONTROLLER:
            continue
        if ctl_name in nm.INERT_CONTROLLERS:
            not_run.append("%s (%s only changes what is drawn)" % (rob["name"], ctl_name))
            continue
        path = find_controller(ctl_name, meta.get("world"))
        if path is None:
            if strict:
                raise CoreUnsupported("the controller %r of %s (no Python script found for it)" % (ctl_name, rob["name"]))
            not_run.append("%s (%s: no Python script)" % (rob["name"], ctl_name))
            continue
        bodies = {b: rob.get("link_of_body", {}).get(str(b), "link_%d" % b) for b in rob["bodies"]}
        for dn, b in defs.items():
            bodies.setdefault(b, dn.lower())
        cfg = dict(core=core, dt_ms=dt, max_ticks=max_ticks, motors=rob["motors"], bodies=bodies,
                   robot_bodies=[rob["root_body"]] + [b for b in rob["bodies"] if b != rob["root_body"]] if rob["bodies"] else [],
                   robot_def=rob["def"] or rob["name"], robot_name=rob["name"], custom_data=rob["custom_data"], defs=dict(defs),
                   scheduler=scheduler, objects=objects, strict=strict, notes={}, all_defs=names.get("all_defs", {}),
                   defs_complete=not names.get("skipped"), physical_types=nm.PHYSICAL_TYPES,
                   other_devices=rob.get("other_devices", []), devices_complete=rob.get("kind") == "vrml",
                   frames=frames,
                   static_defs={dn: info for dn, info in names["defs"].items()
                                if info.get("static") and "body" not in info and "quat" in info})
        c = {"cfg": cfg, "wake": 0, "alive": True, "path": str(path), "argv": [ctl_name] + list(rob["args"]), "exit": None, "name": rob["name"]}
        cfg["ctl"] = c

        def body(*_ignored, c=c):
            d0 = os.path.dirname(c["path"])
            if d0 not in sys.path:
                sys.path.insert(0, d0)          # a controller imports its sibling modules
            try:
                runpy.run_path(c["path"], run_name="__main__")
                c["exit"] = ("returned", 0)
            except core_mod.SimQuit as q:
                c["exit"] = ("quit", q.args[0] if q.args else 0)
            except CoreUnsupported as u:
                c["exit"] = ("unsupported", str(u))
            except SystemExit as q:
                c["exit"] = ("exit", q.code) if q.code in (None, 0) else ("error", "exited: %s" % (str(q.code)[:300],))
            except BaseException as e:          # noqa: BLE001 - the report names it; the run stops
                if isinstance(e, greenlet.GreenletExit):
                    raise
                c["exit"] = ("error", "%s: %s" % (type(e).__name__, str(e)[:300]))
            c["alive"] = False

        c["g"] = greenlet.greenlet(body)
        ctls.append(c)

    sys.modules["omnisim"] = _stand_in(core_mod)
    # The gearbox declaration channel (gears/runtime_gearbox.py): the engine names it after its pid, which a
    # controller reads from OMNISIM_IPC_NONCE; on the core there is no engine pid, so the run names it itself.
    gb_env_old = os.environ.get("OMNISIM_NEWTON_GEARBOX_CMD")
    if not gb_env_old:
        import tempfile
        os.environ["OMNISIM_NEWTON_GEARBOX_CMD"] = os.path.join(tempfile.mkdtemp(prefix="omnisim_core_gearbox_"), "channel.jsonl")
    watch_stop = _stall_watchdog(core, Path(str(report) + ".stalled.txt") if report else None)
    rec = {"state": [], "ctrl": []}
    sink = io.StringIO() if quiet else None
    stop_file = str(stop_file) if stop_file else None
    t_run = time.perf_counter()
    t_core, stopped_by = 0.0, "controllers returned"
    saved_argv = sys.argv
    try:
        with contextlib.redirect_stdout(sink) if sink else contextlib.nullcontext():
            while True:
                done = core.ticks >= max_ticks
                if stop_file and (core.ticks & 63) == 0 and os.path.exists(stop_file):
                    done, stopped_by = True, "stop file"
                elif done:
                    stopped_by = "tick limit"
                for c in ctls:
                    if c["alive"] and c["wake"] <= core.ticks:
                        core_mod.Supervisor._next_cfg = c["cfg"]
                        sys.argv = c["argv"]
                        c["g"].switch(-1 if done else 0)
                failed = [c for c in ctls if c["exit"] and c["exit"][0] in ("unsupported", "error")]
                if failed:
                    stopped_by = "a controller stopped"
                    break
                if done or not any(c["alive"] for c in ctls):
                    break
                if interv is not None:
                    interv.before_step(core)
                t = time.perf_counter()
                core.step()
                for st in stocks:
                    st.update()
                t_core += time.perf_counter() - t
                time.sleep(0)             # let a controller's own threads (a G-code server) run
                if record:
                    rec["state"].append(core.state())
                    rec["ctrl"].append(core.ctrl.copy())
    finally:
        sys.argv = saved_argv
        watch_stop.set()
        if not gb_env_old:
            os.environ.pop("OMNISIM_NEWTON_GEARBOX_CMD", None)
    t_end = time.perf_counter()
    for st in stocks:
        st.dump()
    if record:
        np.savez(record, state=np.array(rec["state"]), ctrl=np.array(rec["ctrl"]))
    notes = {}
    for c in ctls:
        for k, v in c["cfg"]["notes"].items():
            notes[k] = notes.get(k, 0) + v
    unsupported = [c["exit"][1] for c in ctls if c["exit"] and c["exit"][0] == "unsupported"]
    errors = [[c["name"], c["exit"][1]] for c in ctls if c["exit"] and c["exit"][0] == "error"]
    status = EXIT_UNSUPPORTED if unsupported else (EXIT_CONTROLLER if errors else EXIT_OK)
    rep = {"status": status, "stopped_by": stopped_by, "ticks": core.ticks, "sim_s": round(core.ticks * dt / 1000.0, 4),
           "substeps": core.substeps, "clamp_ticks": core.clamp_ticks,
           "controllers": [os.path.basename(c["path"]) for c in ctls], "not_run": not_run,
           "unsupported": unsupported, "errors": errors, "lenient_notes": notes,
           "wall_run_s": round(t_end - t_run, 3), "wall_process_s": round(t_end - t_start, 3),
           "ms_per_tick": round((t_end - t_run) / max(1, core.ticks) * 1e3, 5),
           "ms_per_tick_core_only": round(t_core / max(1, core.ticks) * 1e3, 5),
           "stocks": [{"def": st.name, "removed_mm3": st.removed * 1e9, "deepest_mm": st.deepest * 1e3} for st in stocks]}
    if core.force_on or core.gearboxes:
        rep["force_mode"] = {"gearboxes": [{"tick": t, "dof": dof, "name": spec.get("name", "")} for t, dof, spec in core.attach_log]}
    if recorder is not None:
        core.recorder = None
        if interv is not None and interv.done:
            recorder.edit_tick, recorder.edits = interv.tick, interv.edits
        if against:
            ref = timeline.Recording(against)
            n = min(len(recorder.hash), len(ref.hash))
            differ = np.nonzero(np.array(recorder.hash[:n], dtype=np.uint64) != ref.hash[:n])[0]
            rep["against"] = {"reference": str(against), "ticks_compared": int(n),
                              "first_tick_that_differs": int(differ[0]) if len(differ) else None,
                              "change_at_tick": interv.tick if interv is not None else None}
        if timeline_dir:
            recorder.save(timeline_dir, cdir, {"kind": "run", "world": meta.get("world"), "controllers_in_the_loop": True,
                                               "controllers": rep["controllers"], "strict": bool(strict)})
            rep["timeline"] = {"path": str(timeline_dir), "ticks": core.ticks, "keyframes": len(recorder.key_tick)}
    if report:
        Path(report).write_text(json.dumps(rep, indent=1), encoding="utf-8")
    return status, rep


def main(argv=None):
    p = argparse.ArgumentParser(prog="omnisim core run-compiled", description=__doc__.split("\n\n")[0])
    p.add_argument("compiled", help="a directory written by omnisim_core.compile.compiled_for")
    p.add_argument("--ticks", type=int, default=None, help="stop after this many simulation ticks")
    p.add_argument("--stock-dump", default=None, help="write the MillingStock height map here (the engine's OMZMAP1 file)")
    p.add_argument("--stop-file", default=None, help="stop cleanly when this file appears")
    p.add_argument("--lenient", action="store_true", help="count calls the core does not model instead of refusing them")
    p.add_argument("--quiet", action="store_true", help="drop the controllers' own output")
    p.add_argument("--record", default=None, help="save every tick's state and commands (.npz)")
    p.add_argument("--report", default=None, help="write the run report here (JSON)")
    a = p.parse_args(argv)
    try:
        status, rep = run(a.compiled, a.ticks, a.stock_dump, a.stop_file, strict=not a.lenient, quiet=a.quiet,
                          record=a.record, report=a.report)
    except CoreUnsupported as u:
        rep = {"status": EXIT_UNSUPPORTED, "unsupported": [str(u)], "errors": [], "ticks": 0}
        if a.report:
            Path(a.report).write_text(json.dumps(rep, indent=1), encoding="utf-8")
        status = EXIT_UNSUPPORTED
    print("\n[core] %s" % json.dumps({k: rep.get(k) for k in ("status", "stopped_by", "ticks", "sim_s", "ms_per_tick", "controllers",
                                                               "not_run", "unsupported", "errors", "lenient_notes") if rep.get(k) not in (None, [], {})}),
          flush=True)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
