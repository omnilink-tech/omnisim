#!/usr/bin/env python3
"""RoboLife world supervisor: the referee for a fleet of pooled Huskies.

Every robot runs its OWN controller (`robolife_robot` [B]); this supervisor
never drives a wheel. It reads each robot's pose and its customData reply,
applies the pure rules in `rl/energy.py` (battery drain by mass and speed,
pad charging, impact detection by one-tick |dv|, death -> stop -> release ->
park, fabrication at the bay), keeps the module ledger (docked / loose /
parked, positions), and writes each robot's bus JSON back into its
customData every BUS_EVERY ticks. Births and deaths are TELEPORTS of pooled
slots (runtime spawn/delete have no physics): translation + rotation +
resetPhysics and NEVER setVelocity (it freezes the body ~2 s).

THREADING CONTRACT (same as alife's terrarium_life.py)
------------------------------------------------------
The supervisor API is not thread-safe and every call is an IPC round trip.
The HTTP thread NEVER touches the supervisor: it queues a command and blocks
on an Event; the sim thread drains the queue once per tick, executes, and --
for a teleport -- completes the command only after the engine has stepped
and the new pose has been READ BACK, so the result is measured, not the
argument echoed. A command arriving while one is in flight is rejected with
409. Reads (`/census`) come from an immutable snapshot republished by the
sim thread.

Files (relative to projects/robolife/_run/robolife/):
  fleet.json        [in]  {config, robots, modules} written by robolife.py
  telemetry.json    [out] every 250 ticks
  epoch_result.json [out] at epoch_s, then simulationQuit(0) unless watch

Env: ROBOLIFE_PORT (default 8790).
"""
import json
import math
import os
import queue
import random
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from omnisim import Supervisor

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOLIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
RUN = os.path.join(ROBOLIFE, "_run", "robolife")
FLEET_PATH = os.path.join(RUN, "fleet.json")
TELEMETRY_PATH = os.path.join(RUN, "telemetry.json")
RESULT_PATH = os.path.join(RUN, "epoch_result.json")

sys.path.insert(0, ROBOLIFE)
from rl import energy as E  # noqa: E402

PORT = int(os.environ.get("ROBOLIFE_PORT", "8790"))
BUS_EVERY = 5              # ticks between customData read/write (contract)
MODULE_EVERY = 10          # ticks between loose-module pose reads
PARKED_BUS_EVERY = 50      # parked slots only need "stay dead" now and then
CENSUS_EVERY = 25          # /census snapshot republish (0.2 s)
VERIFY_TICKS = 2           # steps before a teleport's pose is read back
FAB_EVERY = 25             # fabrication check cadence (0.2 s)


def log(msg):
    print("[world] %s" % msg, flush=True)


# ------------------------------------------------------------------ inputs
with open(FLEET_PATH, encoding="utf-8") as f:
    FLEET_IN = json.load(f)
CFG = FLEET_IN.get("config", {})
ARENA = float(CFG.get("arena", 24.0))
EPOCH_S = float(CFG.get("epoch_s", 240.0))
WATCH = bool(CFG.get("watch", False))
TELEMETRY_EVERY = int(CFG.get("telemetry_every", 250))
RNG = random.Random(int(CFG.get("seed", 0)) * 1000003 + int(CFG.get("epoch", 0)))

# --------------------------------------------------------------- sim state
r = Supervisor()
RADIO = r.getDevice("radio")
if RADIO is None:
    print("[world] FATAL: DIRECTOR has no Emitter 'radio' -- regenerate the world", flush=True)
    sys.exit(1)
DT = int(r.getBasicTimeStep())
DT_S = DT / 1000.0

missing = []
robot_node, robot_tr, robot_rot, robot_cd = {}, {}, {}, {}
module_node, module_tr, module_rot = {}, {}, {}
pad_node, pad_tr = {}, {}

robots = []
for idx, entry in enumerate(FLEET_IN.get("robots", [])):
    slot = int(entry.get("slot", idx))
    node = r.getFromDef("ROBOT_%d" % slot)
    if node is None:
        missing.append("ROBOT_%d" % slot)
        continue
    tr, rot, cd = (node.getField("translation"), node.getField("rotation"),
                   node.getField("customData"))
    if tr is None or rot is None or cd is None:
        missing.append("ROBOT_%d.translation/rotation/customData" % slot)
        continue
    robot_node[slot], robot_tr[slot], robot_rot[slot], robot_cd[slot] = node, tr, rot, cd
    alive = bool(entry.get("alive_at_start", entry.get("alive", False)))
    robots.append(E.Robot(slot, entry.get("genome") or E.default_genome(),
                          entry.get("lineage"), entry.get("id", "r%d" % slot),
                          charge_frac=float(entry.get("charge_frac", E.START_FRAC)),
                          alive=alive, docked=entry.get("docked")))

modules = []
for idx, entry in enumerate(FLEET_IN.get("modules", [])):
    j = int(entry.get("id", idx))
    node = r.getFromDef("MODULE_%d" % j)
    tr = node.getField("translation") if node is not None else None
    rot = node.getField("rotation") if node is not None else None
    if tr is None or rot is None:
        missing.append("MODULE_%d" % j)
        continue
    module_node[j], module_tr[j], module_rot[j] = node, tr, rot
    m = dict(entry)
    m["id"] = j
    m["loose"] = bool(entry.get("loose_at_start", entry.get("loose", False)))
    m["pos"] = list(tr.getSFVec3f())
    modules.append(m)

for k in range(len(CFG.get("pads", []))):
    node = r.getFromDef("PAD_%d" % k)
    tr = node.getField("translation") if node is not None else None
    if tr is None:
        missing.append("PAD_%d" % k)
        continue
    pad_node[k], pad_tr[k] = node, tr

if missing:
    shown = missing[:60]
    log("MISSING %d DEF(s): %s%s" % (len(missing), " ".join(shown),
                                     " ..." if len(missing) > 60 else ""))
    log("MISSING -> expected ROBOT_{i} (translation, rotation, customData), "
        "MODULE_{j} (translation, rotation), PAD_{k}; check fleet.json against the world")

if not robots:
    log("FATAL no robot resolved -- nothing to run")
    os.makedirs(RUN, exist_ok=True)
    with open(RESULT_PATH, "w", encoding="utf-8") as f:
        json.dump({"error": "no_robots_resolved", "missing": missing}, f, indent=1)
    r.simulationQuit(1)
    sys.exit(1)

FLEET = E.Fleet(robots, modules, CFG, RNG)


# --------------------------------------------------------------- teleports
def teleport_robot(slot, xyz, yaw=None):
    robot_tr[slot].setSFVec3f([float(xyz[0]), float(xyz[1]), float(xyz[2])])
    if yaw is not None:
        robot_rot[slot].setSFRotation([0.0, 0.0, 1.0, float(yaw)])
    robot_node[slot].resetPhysics()
    # NO setVelocity: it freezes a Newton body for ~2 s (alife probe).
    _last_pos.pop(slot, None)
    _last_v.pop(slot, None)


def teleport_module(j, xyz, yaw=None):
    module_tr[j].setSFVec3f([float(xyz[0]), float(xyz[1]), float(xyz[2])])
    if yaw is not None:
        module_rot[j].setSFRotation([0.0, 0.0, 1.0, float(yaw)])
    module_node[j].resetPhysics()


def park_robot(slot):
    teleport_robot(slot, E.robot_park_translation(slot), yaw=0.0)


def apply(acts):
    for a in acts:
        if a[0] == "park":
            park_robot(a[1])
        elif a[0] == "module":
            teleport_module(a[1], a[2], a[3])
        elif a[0] == "revive":
            teleport_robot(a[1], a[2], a[3])


_last_pos, _last_v = {}, {}

# Slots not alive at start rest on the crypt before the first step (the
# world authors them there already; this is idempotent and cheap).
for rb in FLEET.slots.values():
    if rb.parked:
        park_robot(rb.slot)

log("driving %d robots (%d alive) / %d modules (%d loose) / %d pads, bay %s"
    % (len(FLEET.slots), len(FLEET.alive()), len(FLEET.modules),
       len(FLEET.loose_modules()), len(FLEET.pads), FLEET.bay))
log("epoch_s=%.0f watch=%s dt=%d ms  modules=%s" % (EPOCH_S, WATCH, DT, E.module_source()))

# --------------------------------------------------------------- HTTP layer
_cmdq = queue.Queue()
_busy = threading.Lock()
_census = {"ready": False}
_census_lock = threading.Lock()
_pending = []


class Cmd:
    __slots__ = ("verb", "args", "done", "result", "due_tick", "finish")

    def __init__(self, verb, args):
        self.verb, self.args = verb, args
        self.done = threading.Event()
        self.result = None
        self.due_tick = None
        self.finish = None


def submit(verb, args, timeout=10.0):
    """Marshal one command onto the sim thread. Never call the supervisor
    from an HTTP thread."""
    if not _busy.acquire(blocking=False):
        return 409, {"ok": False, "error": "busy",
                     "detail": "another command is in flight; retry"}
    try:
        c = Cmd(verb, args)
        _cmdq.put(c)
        if not c.done.wait(timeout):
            return 504, {"ok": False, "error": "timeout",
                         "detail": "sim thread did not complete within %.1fs "
                                   "(is the simulation paused?)" % timeout}
        return c.result
    finally:
        _busy.release()


CAPABILITIES = {
    "service": "omnisim-robolife-world",
    "protocol": "OmniSim Wire Protocol",
    "verbs": ["/capabilities", "/census", "/scatter", "/cull", "/fabricate", "/pad",
              "/healthz"],
    "frames": {"position": "world ENU metres", "yaw": "rad about +z"},
    "measured_fields": ["position_measured", "alive", "charge_wh", "docked", "loose"],
    "notes": [
        "Every result reports MEASURED state read back from the simulator after "
        "the engine stepped; arguments are never echoed as if they were outcomes.",
        "Runtime creation of physical bodies is impossible in this engine, so "
        "/fabricate revives a POOLED parked slot and /cull parks (teleports to "
        "the crypt) rather than deletes. /scatter drops PARKED modules into the "
        "arena; it reports how many it could.",
        "/cull, /fabricate, /scatter and /pad complete only after the teleport has "
        "been stepped and the new pose read back (%d ticks)." % VERIFY_TICKS,
        "/fabricate is FORCED: it skips the charge and bay-distance preconditions "
        "but still needs a free pooled slot, and the parent still pays 45 %% of "
        "its capacity (floored at 0).",
        "Robots are driven by their own controllers; this service never commands "
        "a wheel. Orders reach a robot through its customData bus.",
    ],
    "config": {"arena": ARENA, "robots": len(FLEET.slots), "modules": len(FLEET.modules),
               "pads": FLEET.pads, "bay": FLEET.bay, "epoch_s": EPOCH_S, "watch": WATCH,
               "basic_time_step_ms": DT, "bus_every_ticks": BUS_EVERY},
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
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
        if path in ("/scatter", "/cull", "/fabricate", "/pad"):
            code, body = submit(path[1:], args)
            return self._send(code, body)
        return self._send(404, {"ok": False, "error": "unknown_verb", "path": path})


def serve():
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    srv.serve_forever()


threading.Thread(target=serve, daemon=True).start()
log("listening on http://127.0.0.1:%d" % PORT)


# --------------------------------------------------------------- commands
def _measure_robot(slot):
    return [round(v, 4) for v in robot_node[slot].getPosition()]


def _measure_module(j):
    return [round(v, 4) for v in module_node[j].getPosition()]


def _robot_state(slot):
    rb = FLEET.slots[slot]
    cap = FLEET.capacity(slot)
    return {"robot": slot, "id": rb.id, "lineage": rb.lineage, "alive": rb.alive,
            "parked": rb.parked, "charge_wh": round(rb.charge_wh, 2),
            "batt": round(rb.frac(cap), 3), "docked": dict(rb.docked),
            "age_s": round(rb.age_s, 2)}


def do_cmd(c, tick, now_s):
    """Execute one agent command ON THE SIM THREAD. Returns (code, body) for
    an immediate result, or None after arming c.finish for c.due_tick."""
    a = c.args
    try:
        if c.verb == "scatter":
            n = int(a.get("n", 1))
            acts = FLEET.scatter(n)
            apply(acts)
            ids = [x[1] for x in acts]
            if not ids:
                return 409, {"ok": False, "error": "no_parked_module", "requested": n,
                             "loose": len(FLEET.loose_modules())}

            def fin(ids=ids, n=n):
                meas = {str(j): _measure_module(j) for j in ids}
                for j in ids:
                    FLEET.set_module_pos(j, meas[str(j)])
                return 200, {"ok": True, "requested": n, "scattered": len(ids),
                             "modules": ids, "position_measured": meas,
                             "loose": len(FLEET.loose_modules())}
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        if c.verb == "pad":
            if "x" not in a or "y" not in a:
                return 400, {"ok": False, "error": "missing_args", "need": ["x", "y"]}
            k = int(a.get("pad", 0))
            if k not in pad_tr:
                return 404, {"ok": False, "error": "no_such_pad", "pad": k,
                             "valid": sorted(pad_tr)}
            h = ARENA / 2.0 - 1.5
            x, y = E.clamp(float(a["x"]), -h, h), E.clamp(float(a["y"]), -h, h)
            FLEET.pads[k] = [x, y]
            pad_tr[k].setSFVec3f([x, y, 0.01])

            def fin(k=k, x=x, y=y):
                return 200, {"ok": True, "pad": k,
                             "position_measured": [round(v, 4) for v in pad_node[k].getPosition()],
                             "clamped": [x, y] != [float(a["x"]), float(a["y"])],
                             "pads": FLEET.pads}
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        slot = int(a.get("robot", -1))
        if slot not in FLEET.slots:
            return 404, {"ok": False, "error": "no_such_robot", "robot": slot,
                         "valid": sorted(FLEET.slots)}

        if c.verb == "cull":
            rb = FLEET.slots[slot]
            if rb.parked:
                return 409, {"ok": False, "error": "already_parked", **_robot_state(slot)}
            if rb.alive:
                FLEET.kill(slot, now_s, cause="culled")
            apply(FLEET.death_tick(now_s, force=True))
            write_bus(slot, now_s, _last_pos.get(slot, (0.0, 0.0, 0.0)))

            def fin(slot=slot):
                body = _robot_state(slot)
                body.update({"ok": True, "position_measured": _measure_robot(slot)})
                return 200, body
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        if c.verb == "fabricate":
            rb = FLEET.slots[slot]
            if not rb.alive:
                return 409, {"ok": False, "error": "parent_dead", **_robot_state(slot)}
            pos = _last_pos.get(slot) or robot_node[slot].getPosition()
            why = FLEET.can_fabricate(slot, pos)
            res = FLEET.fabricate(slot, pos, now_s, force=True)
            if res is None:
                return 409, {"ok": False, "error": "no_free_slot", "reason": why,
                             **_robot_state(slot)}
            child, xyz, yaw = res
            teleport_robot(child.slot, xyz, yaw)
            write_bus(child.slot, now_s, xyz)

            def fin(slot=slot, child=child, why=why):
                body = {"ok": True, "forced_past": why, "parent": _robot_state(slot),
                        "child": _robot_state(child.slot)}
                body["child"]["position_measured"] = _measure_robot(child.slot)
                return 200, body
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        return 400, {"ok": False, "error": "unknown_verb", "verb": c.verb}
    except Exception as exc:                       # never kill the sim thread
        return 500, {"ok": False, "error": "exception", "detail": repr(exc)}


# ------------------------------------------------------------------ output
def write_json(path, obj):
    os.makedirs(RUN, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, path)


engine_ms = []
bus_stats = {"writes": 0, "reads": 0, "replies": 0, "trimmed": 0, "max_bytes": 0}


def _step_median():
    w = sorted(engine_ms[40:] or engine_ms)
    return round(w[len(w) // 2], 3) if w else None


def write_bus(slot, now_s, pos):
    msg = FLEET.bus_for(slot, now_s, pos)
    text, kept = E.encode_bus(msg)
    if kept < len(msg["modules"]):
        bus_stats["trimmed"] += 1
    n = len(text.encode("utf-8"))
    bus_stats["max_bytes"] = max(bus_stats["max_bytes"], n)
    bus_stats["writes"] += 1
    # Radio out: the packet carries its addressee first so a robot can filter
    # cheaply. Slot 0 prefix must be the very first key (the robot checks the
    # head of the packet).
    RADIO.send(('{"slot":%d,' % slot + text[1:]).encode("utf-8"))


def read_bus(slot):
    """The robot's reply, if the field holds one. The field is shared both
    ways, so it may still hold OUR last message (no reply since): that is
    recognised by its keys and ignored."""
    bus_stats["reads"] += 1
    d = E.decode_status(robot_cd[slot].getSFString())
    if d and "state" in d and "orders" not in d:
        bus_stats["replies"] += 1
        return d
    return None


def finish_epoch(sim_s):
    res = FLEET.epoch_result(sim_s)
    res.update({"ticks": tick, "dt_ms": DT, "missing_defs": missing,
                "engine_ms_per_step_median": _step_median(), "bus": dict(bus_stats),
                "epoch": CFG.get("epoch"), "seed": CFG.get("seed")})
    write_json(RESULT_PATH, res)
    log("epoch done at %.1fs: births=%d deaths=%d docks=%d releases=%d -> %s"
        % (sim_s, FLEET.births, FLEET.deaths, FLEET.docks, FLEET.releases, RESULT_PATH))


# --------------------------------------------------------------- main loop
tick = 0
positions = {}
epoch_written = False

while True:
    ts = time.perf_counter()
    if r.step(DT) == -1:
        break
    engine_ms.append((time.perf_counter() - ts) * 1000.0)
    if len(engine_ms) > 4000:
        del engine_ms[:2000]

    t = tick * DT_S
    positions.clear()
    died = []

    # 1. every present robot: pose, speed, one-tick dv -> energy + impact
    for rb in FLEET.present():
        slot = rb.slot
        pos = tuple(robot_node[slot].getPosition())
        positions[slot] = list(pos)
        if FLEET.watchdog(slot, pos):
            if FLEET.kill(slot, t, cause="watchdog"):
                died.append(slot)
            apply(FLEET.death_tick(t, force=True))
            positions.pop(slot, None)
            continue
        lp = _last_pos.get(slot)
        v = math.hypot(pos[0] - lp[0], pos[1] - lp[1]) / DT_S if lp is not None else 0.0
        _last_pos[slot] = pos
        lv = _last_v.get(slot)
        dv = (v - lv) if lv is not None else 0.0
        _last_v[slot] = v
        if not rb.alive:
            continue
        yaw_rate = float(rb.status.get("w", 0.0) or 0.0)
        if FLEET.energy_tick(slot, DT_S, v, yaw_rate, pos):
            FLEET.kill(slot, t, cause="empty")
            died.append(slot)
            continue
        FLEET.impact_tick(slot, dv)

    # 2. dying robots past 20 s -> modules loose, chassis parked
    apply(FLEET.death_tick(t))

    # 3. bus: read replies, then write orders/state (read BEFORE write, the
    #    field is shared); module poses on their own cadence
    if tick % BUS_EVERY == 0:
        for rb in FLEET.present():
            st = read_bus(rb.slot)
            if st is not None:
                FLEET.report(rb.slot, st)
        if tick % MODULE_EVERY == 0:
            for m in FLEET.loose_modules():
                j = m["id"]
                p = module_node[j].getPosition()
                R9 = module_node[j].getOrientation()
                FLEET.set_module_pos(j, p, math.atan2(R9[3], R9[0]))
            for m in FLEET.modules.values():
                if m["holder"] is not None and m["holder"] in positions:
                    FLEET.set_module_pos(m["id"], positions[m["holder"]])
            apply(FLEET.module_tick())
        if tick % FAB_EVERY == 0:
            for rb in FLEET.alive():
                if rb.slot in positions and FLEET.can_fabricate(rb.slot, positions[rb.slot]) is None:
                    res = FLEET.fabricate(rb.slot, positions[rb.slot], t)
                    if res is None:
                        continue
                    child, xyz, yaw = res
                    teleport_robot(child.slot, xyz, yaw)
                    write_bus(child.slot, t, xyz)
                    log("t=%.1fs FABRICATION %s -> %s (slot %d) at (%.2f, %.2f)"
                        % (t, rb.id, child.id, child.slot, xyz[0], xyz[1]))
        for rb in FLEET.slots.values():
            if rb.parked and tick % PARKED_BUS_EVERY != 0:
                continue
            write_bus(rb.slot, t, positions.get(rb.slot) or E.robot_park_translation(rb.slot))

    for slot in died:
        log("t=%.1fs DEATH %s (slot %d) cause=%s" % (t, FLEET.slots[slot].id, slot,
                                                      FLEET.slots[slot].cause))

    # 4. agent commands (bounded) + deferred read-backs
    for _ in range(8):
        try:
            c = _cmdq.get_nowait()
        except queue.Empty:
            break
        res = do_cmd(c, tick, t)
        if res is None:
            _pending.append(c)
        else:
            c.result = res
            c.done.set()
    if _pending:
        still = []
        for c in _pending:
            if tick >= c.due_tick:
                try:
                    c.result = c.finish()
                except Exception as exc:               # noqa: BLE001
                    c.result = (500, {"ok": False, "error": "exception", "detail": repr(exc)})
                c.done.set()
            else:
                still.append(c)
        _pending[:] = still

    # 5. telemetry + census snapshot (reads never block the sim)
    if tick % CENSUS_EVERY == 0 or tick % TELEMETRY_EVERY == 0:
        snap = FLEET.snapshot(tick, t, positions)
        snap["ready"] = True
        snap["engine_ms_per_step_median"] = _step_median()
        snap["bus"] = dict(bus_stats)
        with _census_lock:
            _census.clear()
            _census.update(snap)
        if tick % TELEMETRY_EVERY == 0:
            write_json(TELEMETRY_PATH, snap)
            log("t=%7.1fs tick=%6d alive=%d births=%d deaths=%d docks=%d loose=%d "
                "replies=%d/%d step=%s ms"
                % (t, tick, len(FLEET.alive()), FLEET.births, FLEET.deaths, FLEET.docks,
                   len(FLEET.loose_modules()), bus_stats["replies"], bus_stats["reads"],
                   _step_median()))

    tick += 1
    if not epoch_written and t >= EPOCH_S:
        finish_epoch(t)
        epoch_written = True
        if not WATCH:
            r.simulationQuit(0)
            break
