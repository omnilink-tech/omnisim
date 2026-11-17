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

"""`python -m omnisim core ...`: the in-process core from the command line.

    compile <world> --out DIR           the engine compiles the world for the core (cached per machine)
    run <world> [--seconds S]           run the world's controllers on the core
    cache [--clear]                     the compiled-machine cache

Time control (docs/developer/core-timeline.md): a recorded run can be replayed exactly, opened at any tick,
branched with a change, and compared with another run.

    record <world> --out DIR [--seconds S]            run the world on the core and keep its timeline
           [--at-time S --set kind:target=value ...]  ... with a change scheduled into the live run
    replay DIR                                        step it again from the recorded commands; every tick checked
    inspect DIR --time S | --tick N                   joints, bodies and contacts with forces at that moment
    events DIR [--from-time S --to-time S]            contacts that began and ended, joints that reached a limit
    branch DIR --at-time S --set ... --out DIR2       the rest of the run again with a change (recorded commands)
    diff DIR_A DIR_B                                  where two runs part, why, and how far

Changes (--set): friction:BODY=mu  mass:BODY=kg  gain:MOTOR=scale  qpos:MOTOR=v  qvel:MOTOR=v
                 pos:BODY=x,y,z  vel:BODY=vx,vy,vz  ctrl:MOTOR=v (held from that tick on)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .errors import CoreDeclined, CoreUnsupported
from .runtime import HOME




def _compile(a):
    from .compile import compiled_for
    try:
        meta = compiled_for(a.world, a.out, use_cache=not a.no_cache)
    except CoreDeclined as d:
        print("[core] not compiled: %s" % d)
        return 1
    print(json.dumps({"out": str(a.out), "robots": [(r["name"], r["controller"], len(r.get("motors", []))) for r in meta["names"]["robots"]],
                      **meta["compile"]}, indent=1))
    return 0


def _run(a):
    from . import run as runner
    from .compile import compiled_for
    out = Path(a.out) if a.out else HOME / ".tmp" / "omnisim-core-run" / Path(a.world).stem
    try:
        meta = compiled_for(a.world, out, use_cache=not a.no_cache)
        ticks = a.ticks or max(1, int(round(a.seconds * 1000.0 / float(meta["names"]["dt_ms"]))))
        status, rep = runner.run(out, ticks=ticks, strict=not a.lenient, quiet=a.quiet, report=out / "core_report.json",
                                 stock_dump=(out / "sheet.zmap") if meta["names"]["milling"] else None)
    except (CoreDeclined, CoreUnsupported) as d:
        print("[core] cannot run this world: %s" % d)
        return 3
    print(json.dumps({"compile": meta["compile"]["how"], **{k: rep[k] for k in ("status", "stopped_by", "ticks", "sim_s", "ms_per_tick",
                      "controllers", "not_run", "unsupported", "errors", "lenient_notes")}}, indent=1))
    return status




def _tick(rec_dt_ms, total, tick, seconds, default=None):
    if tick is not None:
        return int(tick)
    if seconds is not None:
        return max(0, min(total, int(round(float(seconds) * 1000.0 / rec_dt_ms))))
    return default


def _record(a):
    from . import run as runner
    from .compile import compiled_for
    out = Path(a.out)
    try:
        meta = compiled_for(a.world, out, use_cache=not a.no_cache)
        dt = float(meta["names"]["dt_ms"])
        ticks = a.ticks or max(1, int(round(a.seconds * 1000.0 / dt)))
        intervention = None
        if a.set:
            from . import timeline
            at = _tick(dt, ticks, a.at, a.at_time)
            if at is None:
                print("[core] --set needs --at TICK or --at-time S")
                return 2
            intervention = (at, [timeline.parse_edit(x) for x in a.set])
        status, rep = runner.run(out, ticks=ticks, strict=not a.lenient, quiet=a.quiet, report=out / "core_report.json",
                                 timeline_dir=out, keyframe_every=a.keyframe_every, intervention=intervention, against=a.against)
    except (CoreDeclined, CoreUnsupported, ValueError) as d:
        print("[core] cannot record this world: %s" % d)
        return 3
    print(json.dumps({k: rep.get(k) for k in ("status", "stopped_by", "ticks", "sim_s", "ms_per_tick", "controllers", "not_run",
                                               "unsupported", "errors", "lenient_notes", "timeline", "against")}, indent=1))
    return status


def _replay(a):
    from . import timeline
    rec = timeline.Recording(a.recording)
    t0 = time.perf_counter()
    rep = rec.verify()
    rep["wall_s"] = round(time.perf_counter() - t0, 2)
    rep["sim_s"] = round(rec.ticks * rec.dt_ms / 1000.0, 3)
    print(json.dumps(rep, indent=1))
    return 0 if rep["exact"] else 1


def _inspect(a):
    from . import timeline
    rec = timeline.Recording(a.recording)
    tick = _tick(rec.dt_ms, rec.ticks, a.tick, a.time, default=rec.ticks)
    snap = timeline.snapshot(rec, rec.at(tick))
    if a.touching:
        snap = {"tick": snap["tick"], "time_s": snap["time_s"], "contacts": snap["contacts"]}
    print(json.dumps(snap, indent=1))
    return 0


def _events(a):
    from . import timeline
    rec = timeline.Recording(a.recording)
    print(json.dumps(timeline.events(rec, _tick(rec.dt_ms, rec.ticks, a.from_tick, a.from_time, default=0),
                                     _tick(rec.dt_ms, rec.ticks, a.to_tick, a.to_time, default=rec.ticks)), indent=1))
    return 0


def _branch(a):
    from . import timeline
    rec = timeline.Recording(a.recording)
    at = _tick(rec.dt_ms, rec.ticks, a.at, a.at_time)
    if at is None or not a.set:
        print("[core] a branch needs --at TICK or --at-time S, and at least one --set")
        return 2
    try:
        out = timeline.branch(rec, at, [timeline.parse_edit(x) for x in a.set], a.out)
    except (CoreUnsupported, ValueError) as d:
        print("[core] cannot branch: %s" % d)
        return 3
    print(json.dumps({"out": str(out), "from": str(rec.path), "at_tick": at, "at_time_s": round(at * rec.dt_ms / 1000.0, 4), "changes": a.set,
                      "controllers_in_the_loop": False,
                      "note": "the rest of the run was stepped with the RECORDED commands; record the world again with the same "
                              "--at-time and --set to let its controller react"}, indent=1))
    return 0


def _diff(a):
    from . import timeline
    try:
        rep = timeline.diff(timeline.Recording(a.a), timeline.Recording(a.b))
    except CoreUnsupported as d:
        print("[core] cannot compare: %s" % d)
        return 3
    print(json.dumps(rep, indent=1))
    return 0


def _cache(a):
    from . import compile as comp
    if a.clear:
        print("removed %d compiled machine(s) from %s" % (comp.clear_cache(), comp.CACHE))
        return 0
    entries = sorted(p for p in comp.CACHE.glob("*") if p.is_dir()) if comp.CACHE.exists() else []
    for p in entries:
        e = comp._entry(p.name) or {}
        print("%s  %s  any job on this machine: %s  (%s)" % (p.name, e.get("compiled_at"), bool(e.get("retarget_exact")), e.get("world")))
    print("%d compiled machine(s) in %s" % (len(entries), comp.CACHE))
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    p = argparse.ArgumentParser(prog="python -m omnisim core", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="verb", required=True)
    s = sub.add_parser("compile", help="compile a world for the core (one engine load, cached per machine)")
    s.add_argument("world")
    s.add_argument("--out", required=True)
    s.add_argument("--no-cache", action="store_true")
    s.set_defaults(fn=_compile)
    s = sub.add_parser("run", help="run a world's controllers on the core")
    s.add_argument("world")
    s.add_argument("--seconds", type=float, default=10.0, help="simulated seconds to run (default 10)")
    s.add_argument("--ticks", type=int, default=None)
    s.add_argument("--out", default=None)
    s.add_argument("--lenient", action="store_true", help="count calls the core does not model instead of refusing them")
    s.add_argument("--quiet", action="store_true")
    s.add_argument("--no-cache", action="store_true")
    s.set_defaults(fn=_run)
    s = sub.add_parser("record", help="run a world on the core and keep its timeline")
    s.add_argument("world")
    s.add_argument("--out", required=True)
    s.add_argument("--seconds", type=float, default=10.0)
    s.add_argument("--ticks", type=int, default=None)
    s.add_argument("--keyframe-every", type=int, default=250, help="ticks between full-state keyframes (default 250)")
    s.add_argument("--at", type=int, default=None, help="tick at which --set changes are applied to the live run")
    s.add_argument("--at-time", type=float, default=None, help="the same, in simulated seconds")
    s.add_argument("--set", action="append", default=[], help="a change, kind:target=value (repeatable)")
    s.add_argument("--against", default=None, help="a recording of the same world: report up to which tick this run matches it")
    s.add_argument("--lenient", action="store_true")
    s.add_argument("--quiet", action="store_true")
    s.add_argument("--no-cache", action="store_true")
    s.set_defaults(fn=_record)
    s = sub.add_parser("replay", help="step a recording again from its commands and check every tick")
    s.add_argument("recording")
    s.set_defaults(fn=_replay)
    s = sub.add_parser("inspect", help="joints, bodies and contacts at one moment of a recording")
    s.add_argument("recording")
    s.add_argument("--tick", type=int, default=None)
    s.add_argument("--time", type=float, default=None)
    s.add_argument("--touching", action="store_true", help="contacts only")
    s.set_defaults(fn=_inspect)
    s = sub.add_parser("events", help="contacts that began and ended, joints that reached a limit")
    s.add_argument("recording")
    s.add_argument("--from-tick", type=int, default=None)
    s.add_argument("--to-tick", type=int, default=None)
    s.add_argument("--from-time", type=float, default=None)
    s.add_argument("--to-time", type=float, default=None)
    s.set_defaults(fn=_events)
    s = sub.add_parser("branch", help="the rest of a recording again, with a change, on the recorded commands")
    s.add_argument("recording")
    s.add_argument("--out", required=True)
    s.add_argument("--at", type=int, default=None)
    s.add_argument("--at-time", type=float, default=None)
    s.add_argument("--set", action="append", default=[])
    s.set_defaults(fn=_branch)
    s = sub.add_parser("diff", help="where two recordings part, why, and how far")
    s.add_argument("a")
    s.add_argument("b")
    s.set_defaults(fn=_diff)
    s = sub.add_parser("cache", help="list or clear the compiled machines")
    s.add_argument("--clear", action="store_true")
    s.set_defaults(fn=_cache)
    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    raise SystemExit(main())
