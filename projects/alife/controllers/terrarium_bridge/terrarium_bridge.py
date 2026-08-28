#!/usr/bin/env python3
"""Agent-driveable terrarium bridge.

A live terrarium an LLM agent can operate over HTTP: read the census, perturb a
creature's gait, cull one, respawn one, reset the lap. The creatures are the
evolved champions; the agent is the ecosystem operator.

THREADING CONTRACT (the thing that makes this correct)
------------------------------------------------------
The supervisor API is NOT thread-safe and every call costs an IPC round trip
serviced at a step boundary. So the HTTP thread NEVER touches the supervisor.
It pushes a command onto a queue and blocks on a per-command Event; the sim
thread drains the queue once per tick, executes, and sets the result. Reads are
served from an immutable census snapshot the sim thread republishes every tick,
so a GET never blocks the simulation at all.

TOOL-DESIGN RULES (docs/developer/tool-design-for-agents.md)
------------------------------------------------------------
An LLM has no independent access to the world -- every belief it holds came from
a tool result, so a tool that echoes its own argument back installs a false
belief the agent then reports confidently. Therefore:
  * every result carries MEASURED values, never the argument echoed back
  * anything not actually measured is null, never a plausible number
  * an action completes before returning, or says plainly that it did not
  * a command arriving while another is in flight is REJECTED with 409,
    never silently clobbered

Env: BRIDGE_PORT (default 8790), LOOP_TICKS, SETTLE_TICKS
"""
import json
import math
import os
import threading
import queue
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.normpath(os.path.join(HERE, "..", "..", "_run"))
POP_PATH = os.path.join(RUN, "showcase_population.json")

PORT = int(os.environ.get("BRIDGE_PORT", "8790"))
LOOP = int(os.environ.get("LOOP_TICKS", "0"))
SETTLE = int(os.environ.get("SETTLE_TICKS", "60"))

with open(POP_PATH, encoding="utf-8") as f:
    POP = json.load(f)

# ---------------------------------------------------------------- sim state
_cmdq = queue.Queue()
_busy = threading.Lock()
_census = {"ready": False}
_census_lock = threading.Lock()


class Cmd:
    __slots__ = ("verb", "args", "done", "result")

    def __init__(self, verb, args):
        self.verb, self.args = verb, args
        self.done = threading.Event()
        self.result = None


def submit(verb, args, timeout=10.0):
    """Marshal one command onto the sim thread. Never call the supervisor from
    an HTTP thread."""
    if not _busy.acquire(blocking=False):
        return 409, {"ok": False, "error": "busy",
                     "detail": "another command is in flight; retry"}
    try:
        c = Cmd(verb, args)
        _cmdq.put(c)
        if not c.done.wait(timeout):
            return 504, {"ok": False, "error": "timeout",
                         "detail": "sim thread did not execute within %.1fs" % timeout}
        code, body = c.result
        return code, body
    finally:
        _busy.release()


# ---------------------------------------------------------------- HTTP layer
CAPABILITIES = {
    "service": "omnisim-alife-terrarium",
    "protocol": "OmniSim Wire Protocol",
    "verbs": ["/capabilities", "/census", "/perturb", "/cull", "/revive",
              "/reset", "/healthz"],
    "measured_fields": ["position", "displacement_m", "alive"],
    "notes": [
        "Every result reports MEASURED state read back from the simulator; "
        "arguments are never echoed as if they were outcomes.",
        "Runtime creation of physical bodies is impossible in this engine "
        "(a spawned node has no physics), so /revive recycles a POOLED "
        "creature rather than creating one. /cull parks rather than deletes.",
    ],
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"          # keep-alive; the harness is 1.0

    def log_message(self, *a):             # keep the sim log readable
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/healthz":
            return self._send(200, {"ok": True})
        if path == "/capabilities":
            return self._send(200, CAPABILITIES)
        if path == "/census":
            with _census_lock:
                return self._send(200, dict(_census))
        return self._send(404, {"ok": False, "error": "unknown_verb", "path": path})

    def do_POST(self):
        path = self.path.split("?")[0]
        n = int(self.headers.get("Content-Length") or 0)
        try:
            args = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._send(400, {"ok": False, "error": "bad_json"})

        if path == "/census":
            with _census_lock:
                return self._send(200, dict(_census))
        if path in ("/perturb", "/cull", "/revive", "/reset"):
            code, body = submit(path[1:], args)
            return self._send(code, body)
        return self._send(404, {"ok": False, "error": "unknown_verb", "path": path})


def serve():
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    srv.serve_forever()


# ---------------------------------------------------------------- sim thread
r = Supervisor()
# Derive the tick from the world -- never hardcode it. The CPG phase
# computation (t = tick*DT) breaks silently if this drifts from
# WorldInfo.basicTimeStep.
DT = int(r.getBasicTimeStep())
roots, fields = {}, {}
for i, g in enumerate(POP):
    node = r.getFromDef("CREATURE_%d" % i)
    if node is None:
        continue
    roots[i] = node
    for j in range(len(g["limbs"])):
        p = r.getFromDef("C%d_J%d_PARAMS" % (i, j))
        fld = p.getField("position") if p is not None else None
        if fld is not None:
            fields[(i, j)] = fld

home = {i: list(roots[i].getField("translation").getSFVec3f()) for i in roots}
alive = {i: True for i in roots}
# Parking must be a real slab-free spot BELOW nothing: there is no implicit
# ground plane any more, so a parked body would fall forever. Park them high on
# a shelf far to the side instead, where they simply rest out of frame.
CRYPT = {i: [60.0 + 2.0 * i, 60.0, 0.5] for i in roots}
gait_scale = {i: 1.0 for i in roots}

threading.Thread(target=serve, daemon=True).start()
print("[bridge] listening on http://127.0.0.1:%d  (%d creatures, %d joints)"
      % (PORT, len(roots), len(fields)), flush=True)

start, lap = {}, 0


def do_cmd(c):
    """Execute one agent command ON THE SIM THREAD, then measure the result."""
    a = c.args
    try:
        if c.verb == "reset":
            for i in roots:
                roots[i].getField("translation").setSFVec3f(home[i])
                roots[i].getField("rotation").setSFRotation([0, 0, 1, 0])
                roots[i].resetPhysics()
                alive[i] = True
            return 200, {"ok": True, "reset": len(roots), "lap_ticks_elapsed": lap}

        idx = int(a.get("creature", -1))
        if idx not in roots:
            return 404, {"ok": False, "error": "no_such_creature",
                         "creature": idx, "valid": sorted(roots)}

        if c.verb == "perturb":
            # Scale this creature's gait amplitude. Reported back as the value
            # actually in force, read from our own state, plus the measured pose.
            s = float(a.get("amplitude_scale", 1.0))
            s = max(0.0, min(3.0, s))
            gait_scale[idx] = s
            p = list(roots[idx].getPosition())
            return 200, {"ok": True, "creature": idx,
                         "amplitude_scale_applied": s,
                         "clamped": s != float(a.get("amplitude_scale", 1.0)),
                         "position_measured": p}

        if c.verb == "cull":
            roots[idx].getField("translation").setSFVec3f(CRYPT[idx])
            roots[idx].resetPhysics()
            alive[idx] = False
            return 200, {"ok": True, "creature": idx, "alive": False,
                         "position_measured": list(roots[idx].getPosition())}

        if c.verb == "revive":
            roots[idx].getField("translation").setSFVec3f(home[idx])
            roots[idx].getField("rotation").setSFRotation([0, 0, 1, 0])
            roots[idx].resetPhysics()
            alive[idx] = True
            return 200, {"ok": True, "creature": idx, "alive": True,
                         "position_measured": list(roots[idx].getPosition())}

        return 400, {"ok": False, "error": "unknown_verb", "verb": c.verb}
    except Exception as exc:                       # never kill the sim thread
        return 500, {"ok": False, "error": "exception", "detail": repr(exc)}


while r.step(DT) != -1:
    t = lap * (DT / 1000.0)

    for (i, j), fld in fields.items():
        if not alive[i]:
            continue
        g = POP[i]
        lb = g["limbs"][j]
        fld.setSFFloat(lb["bias"] + gait_scale[i] * lb["amp"]
                       * math.sin(2.0 * math.pi * g["freq"] * t + lb["phase"]))

    if lap == SETTLE:
        for i in roots:
            start[i] = list(roots[i].getPosition())

    # drain agent commands (bounded, so one burst cannot stall the sim)
    for _ in range(8):
        try:
            c = _cmdq.get_nowait()
        except queue.Empty:
            break
        c.result = do_cmd(c)
        c.done.set()

    # republish the census snapshot -- reads never block the sim
    snap = {"ready": True, "tick": lap, "sim_time_s": round(t, 3),
            "creatures": {}}
    for i in roots:
        p = list(roots[i].getPosition())
        s = start.get(i)
        snap["creatures"][str(i)] = {
            "id": POP[i]["id"],
            "generation": POP[i].get("_gen"),
            "evolved_fitness_m": POP[i].get("_fitness"),
            "alive": alive[i],
            "position": [round(v, 4) for v in p],
            # null, not 0.0, until the settle window has actually elapsed --
            # an unmeasured value must never look like a measurement
            "displacement_m": round(math.dist(p[:2], s[:2]), 4) if s else None,
            "amplitude_scale": gait_scale[i],
            "limbs": len(POP[i]["limbs"]),
        }
    with _census_lock:
        _census.clear()
        _census.update(snap)

    lap += 1
    if LOOP and lap >= LOOP:
        for i in roots:
            if alive[i]:
                roots[i].getField("translation").setSFVec3f(home[i])
                roots[i].getField("rotation").setSFRotation([0, 0, 1, 0])
                roots[i].resetPhysics()
        start, lap = {}, 0
