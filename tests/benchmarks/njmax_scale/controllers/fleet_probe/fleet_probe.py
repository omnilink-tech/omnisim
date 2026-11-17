"""fleet_probe -- supervisor that measures whether N wheeled robots ACTUALLY drive.

Not a log check. Per robot it records, over the whole run:
  * chassis world position (start / end / trace)  -> displacement
  * chassis linear speed trace                    -> jitter / stall detection
  * per-wheel angular speed |w| trace             -> "did the wheels turn"
  * chassis z trace                               -> tunnelling / sinking / launch

Writes one JSON blob to $FLEET_PROBE_OUT and quits the simulation, so the
run is bounded by simulated time, not by wall clock.

Env:
  FLEET_PROBE_OUT        output json path (required)
  FLEET_PROBE_DURATION_MS  simulated ms to run (default 20000)
  FLEET_PROBE_SAMPLE_MS    sample period ms (default 500)
  FLEET_PROBE_DUMPTREE   if set, dump the first robot's subtree names
"""

from __future__ import annotations

import json
import math
import os
import sys

from omnisim import Supervisor


def walk(node, depth, out, maxdepth=6):
    if node is None or depth > maxdepth:
        return
    try:
        tname = node.getTypeName()
    except Exception:
        return
    nm = ""
    try:
        f = node.getField("name")
        if f is not None:
            nm = f.getSFString()
    except Exception:
        pass
    out.append((depth, tname, nm, node))
    for fname in ("children", "endPoint"):
        try:
            fld = node.getField(fname)
        except Exception:
            fld = None
        if fld is None:
            continue
        try:
            if fname == "children":
                n = fld.getCount()
                for i in range(n):
                    walk(fld.getMFNode(i), depth + 1, out, maxdepth)
            else:
                walk(fld.getSFNode(), depth + 1, out, maxdepth)
        except Exception:
            pass


def main() -> int:
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    out_path = os.environ.get("FLEET_PROBE_OUT", "")
    duration_ms = int(os.environ.get("FLEET_PROBE_DURATION_MS", "20000"))
    sample_ms = int(os.environ.get("FLEET_PROBE_SAMPLE_MS", "500"))

    root = sup.getRoot()
    kids = root.getField("children")
    robots = []
    for i in range(kids.getCount()):
        n = kids.getMFNode(i)
        try:
            tn = n.getTypeName()
        except Exception:
            continue
        nm = ""
        try:
            f = n.getField("name")
            if f is not None:
                nm = f.getSFString()
        except Exception:
            pass
        if not nm.startswith("bot_"):
            continue
        subtree = []
        walk(n, 0, subtree)
        wheels = [(w_nm, w_node) for (_d, _t, w_nm, w_node) in subtree
                  if "wheel" in w_nm.lower()]
        robots.append({"name": nm, "type": tn, "node": n, "wheels": wheels,
                       "trace": [], "wheel_trace": []})
        if os.environ.get("FLEET_PROBE_DUMPTREE") and len(robots) == 1:
            for (d, t, w_nm, _n) in subtree:
                sys.stderr.write("[tree] %s%s name=%r\n" % ("  " * d, t, w_nm))

    sys.stderr.write("[fleet_probe] robots=%d wheels_per_robot=%s ts=%d\n"
                     % (len(robots),
                        [len(r["wheels"]) for r in robots][:3], ts))
    sys.stderr.flush()

    def sample():
        t = sup.getTime()
        for r in robots:
            try:
                p = list(r["node"].getPosition())
            except Exception:
                p = [float("nan")] * 3
            try:
                v = list(r["node"].getVelocity())
            except Exception:
                v = [float("nan")] * 6
            speed = math.sqrt(v[0] ** 2 + v[1] ** 2) if v[0] == v[0] else float("nan")
            ws = []
            for (_wn, wnode) in r["wheels"]:
                try:
                    wv = wnode.getVelocity()
                    ws.append(math.sqrt(wv[3] ** 2 + wv[4] ** 2 + wv[5] ** 2))
                except Exception:
                    ws.append(float("nan"))
            r["trace"].append([round(t, 3), round(p[0], 5), round(p[1], 5),
                               round(p[2], 5), round(speed, 5)])
            r["wheel_trace"].append([round(t, 3)] + [round(x, 4) for x in ws])

    sample()
    next_sample = sample_ms
    while sup.step(ts) != -1:
        t_ms = sup.getTime() * 1000.0
        if t_ms >= next_sample:
            sample()
            next_sample += sample_ms
        if t_ms >= duration_ms:
            break
    sample()

    blob = {
        "n_robots": len(robots),
        "basic_time_step_ms": ts,
        "sim_time_s": sup.getTime(),
        "robots": [{"name": r["name"], "n_wheels": len(r["wheels"]),
                    "trace": r["trace"], "wheel_trace": r["wheel_trace"]}
                   for r in robots],
    }
    if out_path:
        try:
            with open(out_path, "w") as f:
                json.dump(blob, f)
            sys.stderr.write("[fleet_probe] wrote %s\n" % out_path)
        except Exception as exc:
            sys.stderr.write("[fleet_probe] write failed: %r\n" % (exc,))
    sys.stderr.flush()
    sup.simulationQuit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
