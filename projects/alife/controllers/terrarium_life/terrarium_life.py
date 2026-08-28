#!/usr/bin/env python3

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

"""Life director: one supervisor runs the whole alife v2 ecosystem.

Creatures are `controller "<none>"` Robots with no process of their own. This
supervisor senses for them, steers them, writes every hinge's
`HingeJointParameters.position` field (POSTPONED writes, drained as ONE batch
right before the engine pushes motor targets into Newton -- one round trip per
tick for the whole population; never `setJointPosition`, which is a blocking
flush ~1400x costlier under load), burns their energy, feeds them, breeds them
and buries them.

Births and deaths are TELEPORTS of pooled slots: runtime spawn/delete have no
physics in this engine, so every body is authored at load. A dead creature is
parked at (60 + 2i, 60, 5) where nothing lies beneath it -- it free-falls with
zero contacts, which is what makes a parked slot nearly free. Food is
visual-only (no physics); eaten = teleported to z -3, respawn = teleported to
a random arena point at z 0.09.

All of the ecology RULES live in `alife/ecology.py` (pure, unit-tested). This
file owns the supervisor round trips, the clock, the files and the HTTP bridge.

THREADING CONTRACT (same as terrarium_bridge.py)
------------------------------------------------
The supervisor API is not thread-safe and every call is an IPC round trip. The
HTTP thread NEVER touches the supervisor: it queues a command and blocks on an
Event; the sim thread drains the queue once per tick, executes, and -- for a
teleport -- completes the command only after the engine has stepped and the
new pose has been READ BACK, so the result is measured, not the argument
echoed. A command arriving while one is in flight is rejected with 409. Reads
(`/census`) come from an immutable snapshot republished by the sim thread.

Files (relative to projects/alife/_run/life/):
  population.json   [in]  list of genome v2 + "slot" + "alive_at_start"
  config.json       [in]  {arena, food_pool, food_active_max,
                           food_respawn_s: [lo, hi], epoch_s, watch,
                           energy_start?, seed?, telemetry_every?}
  telemetry.json    [out] every 250 ticks
  epoch_result.json [out] at epoch_s, then simulationQuit(0) unless watch

Env: LIFE_PORT (default 8790).
"""
import copy
import inspect
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
ALIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
RUN = os.path.join(ALIFE, "_run", "life")
POP_PATH = os.path.join(RUN, "population.json")
CFG_PATH = os.path.join(RUN, "config.json")
TELEMETRY_PATH = os.path.join(RUN, "telemetry.json")
RESULT_PATH = os.path.join(RUN, "epoch_result.json")

sys.path.insert(0, ALIFE)
from alife import ecology as eco  # noqa: E402

PORT = int(os.environ.get("LIFE_PORT", "8790"))
CENSUS_EVERY = 25          # ticks between /census snapshot republishes (0.2 s)
VERIFY_TICKS = 2           # steps before a teleport's pose is read back


def log(msg):
    print("[life] %s" % msg, flush=True)


# ------------------------------------------------------------------ inputs
with open(POP_PATH, encoding="utf-8") as f:
    POP = json.load(f)
with open(CFG_PATH, encoding="utf-8") as f:
    CFG = json.load(f)

ARENA = float(CFG.get("arena", 14.0))
FOOD_POOL = int(CFG.get("food_pool", 24))
FOOD_ACTIVE_MAX = int(CFG.get("food_active_max", 12))
RESPAWN = CFG.get("food_respawn_s", [4.0, 12.0])
EPOCH_S = float(CFG.get("epoch_s", 120.0))
WATCH = bool(CFG.get("watch", False))
ENERGY_START = float(CFG.get("energy_start", eco.ENERGY_START))
TELEMETRY_EVERY = int(CFG.get("telemetry_every", 250))
# One stream per epoch: the driver writes the same `seed` every epoch and an
# identical stream would replay identical respawn points and child yaws.
RNG = random.Random(int(CFG.get("seed", 0)) * 1000003 + int(CFG.get("epoch", 0)))

# The brain mutation operator belongs to genome2 [A]. Fall back to the pure
# creep in ecology so a missing module degrades to a running (and logged)
# ecosystem rather than a dead one.
try:
    from alife.genome2 import mutate_brain as _g2_mutate_brain
    _N_MUTATE_ARGS = len(inspect.signature(_g2_mutate_brain).parameters)
    MUTATE_SOURCE = "genome2.mutate_brain"
except Exception as exc:                                   # noqa: BLE001
    _g2_mutate_brain, _N_MUTATE_ARGS = None, 0
    MUTATE_SOURCE = "ecology.mutate_brain_fallback (genome2 unavailable: %r)" % exc


def mutate_brain(genome, rng, child_id):
    """Adapter for Ecology.try_reproduce: genome2.mutate_brain(g, rng, gid)
    returns a whole genome (id + parent stamped); older shapes may take fewer
    arguments or return a bare brain. Normalise to a brain dict."""
    if _g2_mutate_brain is None:
        return eco.mutate_brain_fallback(genome["brain"], rng)
    try:
        if _N_MUTATE_ARGS >= 3:
            out = _g2_mutate_brain(genome, rng, child_id)
        elif _N_MUTATE_ARGS == 2:
            out = _g2_mutate_brain(genome, rng)
        else:
            out = _g2_mutate_brain(genome)
    except Exception as exc:                               # noqa: BLE001
        log("WARNING genome2.mutate_brain raised %r; using fallback" % exc)
        return eco.mutate_brain_fallback(genome["brain"], rng)
    return out["brain"] if isinstance(out, dict) and "brain" in out else out


# --------------------------------------------------------------- sim state
r = Supervisor()
# Derive the tick from the world -- never hardcode it. The CPG phase
# computation (t = tick*DT) breaks silently if this drifts from
# WorldInfo.basicTimeStep.
DT = int(r.getBasicTimeStep())
DT_S = DT / 1000.0

# ---- resolve every DEF once; anything unresolved is reported LOUDLY --------
missing = []
roots = {}          # slot -> Robot node
tr_field = {}       # slot -> translation field
rot_field = {}      # slot -> rotation field
joint_fields = {}   # slot -> {(pair_k, side, 'H'|'K'): position field}
creatures = []

for idx, entry in enumerate(POP):
    slot = int(entry.get("slot", idx))
    node = r.getFromDef("CREATURE_%d" % slot)
    if node is None:
        missing.append("CREATURE_%d" % slot)
        continue
    roots[slot] = node
    tr_field[slot] = node.getField("translation")
    rot_field[slot] = node.getField("rotation")
    if tr_field[slot] is None or rot_field[slot] is None:
        missing.append("CREATURE_%d.translation/rotation" % slot)
        del roots[slot]
        continue
    jf = {}
    for k, pair in enumerate(entry["body"]["pairs"]):
        for side in ("L", "R"):
            joints = ["H"] if len(pair["segments"]) < 2 else ["H", "K"]
            for jn in joints:
                d = "C%d_P%d_%s_%s_PARAMS" % (slot, k, side, jn)
                p = r.getFromDef(d)
                fld = p.getField("position") if p is not None else None
                if fld is None:
                    missing.append(d)
                else:
                    jf[(k, side, jn)] = fld
    joint_fields[slot] = jf
    genome = {k: v for k, v in entry.items() if k not in ("slot", "alive_at_start")}
    creatures.append(eco.Creature(slot, genome, ENERGY_START, born_at=0.0,
                                  alive=bool(entry.get("alive_at_start", True))))

food_nodes, food_tr, foods = [], [], []
for j in range(FOOD_POOL):
    n = r.getFromDef("FOOD_%d" % j)
    fld = n.getField("translation") if n is not None else None
    if fld is None:
        missing.append("FOOD_%d" % j)
        continue
    food_nodes.append(n)
    food_tr.append(fld)
    foods.append(list(fld.getSFVec3f()))

if missing:
    shown = missing[:60]
    log("MISSING %d DEF(s): %s%s" % (len(missing), " ".join(shown),
                                     " ..." if len(missing) > 60 else ""))
    log("MISSING -> expected CREATURE_{i}, C{i}_P{k}_{L|R}_{H|K}_PARAMS, FOOD_{j}; "
        "check population.json slots / body.pairs against the world")

if not creatures:
    log("FATAL no creature resolved -- nothing to run")
    os.makedirs(RUN, exist_ok=True)
    with open(RESULT_PATH, "w", encoding="utf-8") as f:
        json.dump({"error": "no_creatures_resolved", "missing": missing}, f, indent=1)
    r.simulationQuit(1)
    sys.exit(1)

# Food indices in `foods` are dense (0..len-1) even when FOOD_{j} DEFs are
# missing; food_tr[j] is the field that corresponds to foods[j].
ECO = eco.Ecology(creatures, foods, ARENA, FOOD_ACTIVE_MAX, RESPAWN, RNG)
SPAWN_Z = {c.slot: eco.spawn_height(c.genome) for c in creatures}
NOISE_SEED = {c.slot: c.slot * 7919 + 13 for c in creatures}

# Steering polarity by CALIBRATION WIGGLE. Whether "left limbs down, right
# limbs up" turns a given body left or right is a property of its evolved
# gait that no genome field predicts (probe_steer.py), and a body also has an
# intrinsic spin even when told to go straight (-0.77 rad/m measured), which
# fooled a naive online estimator into steering AWAY from food (aimed 1%,
# below chance). So a newborn does what an animal does: it wiggles. 1 s of
# full-left, 1 s of full-right, ignoring food; the DIFFERENCE of the two yaw
# responses is the steering polarity (the drift cancels), the SUM is the
# drift. Repeated every RECAL_TICKS to track posture changes.
CAL_TICKS = 125                  # 1 s per trial
RECAL_TICKS = 2500               # 20 s
STEER_SIGN = {c.slot: 1.0 for c in creatures}
# phase 2 + next = CAL_DELAY: the first wiggle waits for the gait ramp-in
# (RAMP_S), otherwise it measures a half-strength gait and mis-signs.
CAL_DELAY = 300
_cal = {c.slot: {"phase": 2, "t0": 0, "acc": 0.0, "dp": 0.0, "dm": 0.0,
                 "b": None, "a": None, "next": CAL_DELAY} for c in creatures}
_last_yaw = {}

# Righting reflex. A creature on its side or back (torso up-axis z below
# UPRIGHT_MIN for RIGHT_AFTER ticks) rights itself: it is re-posed upright at
# its own position, at a small energy cost. Without it a tipped body is dead
# for the rest of the epoch while still burning energy (measured: whole
# populations lying on their backs by t = 6 s).
UPRIGHT_MIN = 0.35
RIGHT_AFTER = 125                # 1 s
RIGHT_COST = 4.0
_fallen = {c.slot: 0 for c in creatures}
RAMP_S = 1.5                     # gait fade-in after birth / righting
_ramp_t0 = {}
_rightings = {c.slot: 0 for c in creatures}

# Heading = the direction the body is actually TRAVELLING, not where its nose
# points. An evolved gait may carry a body sideways or backwards; steering the
# nose at the food then walks the creature away from it. Travel heading is
# read from ~1 s of position history; below a minimum displacement the body
# yaw (+ the genome's heading_offset) is the fallback.
TRAVEL_WINDOW = 125              # ticks (1 s)
WALL_MARGIN = 3.0                # m; the walker's turn radius is ~3 m at the capped gain
TRAVEL_MIN_M = 0.06
_pos_hist = {c.slot: [] for c in creatures}     # [(tick, x, y)]
# steering diagnostics per slot: ticks food was sensed / ticks aimed within
# the deadband while sensed -- reported per species in epoch_result
_diag = {c.slot: [0, 0, 0, 0.0, 0.0] for c in creatures}  # [alive_ticks, sensed, aimed, path_m, abs_yaw_rad]
_last_pos = {}
_target_buf = {c.slot: {} for c in creatures}     # reused per tick, no alloc


def teleport(slot, xyz, yaw=None):
    """Move a creature root and zero its velocities. Pose-wrapped roots make
    this invisible to every other creature (README P3)."""
    tr_field[slot].setSFVec3f([float(xyz[0]), float(xyz[1]), float(xyz[2])])
    if yaw is not None:
        rot_field[slot].setSFRotation([0.0, 0.0, 1.0, float(yaw)])
    roots[slot].resetPhysics()
    # NO velocity reset here. resetPhysics() does not zero a Newton body's
    # velocity, but Node.setVelocity([0]*6) measurably FREEZES the body for
    # ~2 s after the teleport (probe terrarium_probe_tp: the control moved at
    # once, the reset body sat still for 240 ticks). Parking is on a slab
    # where bodies are at rest, so nothing needs zeroing.


tick = 0                         # defined here too: park() runs during setup


def park(slot):
    teleport(slot, eco.park_translation(slot), yaw=0.0)
    _last_yaw.pop(slot, None)
    _last_pos.pop(slot, None)
    _pos_hist[slot].clear()
    _cal[slot].update({"phase": 2, "t0": 0, "acc": 0.0, "next": tick + CAL_DELAY})


def revive(slot, x, y, yaw):
    h = ARENA / 2.0 - 1.0
    teleport(slot, [eco.clamp(x, -h, h), eco.clamp(y, -h, h), SPAWN_Z[slot]], yaw)


def move_food(j, xyz):
    food_tr[j].setSFVec3f([float(xyz[0]), float(xyz[1]), float(xyz[2])])


# slots not alive at start go to the pit before the first step
for c in creatures:
    if not c.alive:
        park(c.slot)

# Apply the initial active-count cap (an over-authored pool) right away.
for j, xyz in ECO.food_tick(0.0):
    move_food(j, xyz)

log("driving %d creatures (%d alive) / %d joints / %d food (%d active, max %d)"
    % (len(creatures), len(ECO.alive()), sum(len(v) for v in joint_fields.values()),
       len(foods), ECO.food_active(), FOOD_ACTIVE_MAX))
log("species %s  epoch_s=%.0f watch=%s dt=%d ms  mutate=%s"
    % (ECO.population(), EPOCH_S, WATCH, DT, MUTATE_SOURCE))

# --------------------------------------------------------------- HTTP layer
_cmdq = queue.Queue()
_busy = threading.Lock()
_census = {"ready": False}
_census_lock = threading.Lock()
_pending = []       # commands waiting for a read-back after a teleport


class Cmd:
    __slots__ = ("verb", "args", "done", "result", "due_tick", "finish")

    def __init__(self, verb, args):
        self.verb, self.args = verb, args
        self.done = threading.Event()
        self.result = None
        self.due_tick = None
        self.finish = None


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
                         "detail": "sim thread did not complete within %.1fs "
                                   "(is the simulation paused?)" % timeout}
        return c.result
    finally:
        _busy.release()


CAPABILITIES = {
    "service": "omnisim-alife-life",
    "protocol": "OmniSim Wire Protocol",
    "verbs": ["/capabilities", "/census", "/feed", "/cull", "/spawn", "/perturb",
              "/healthz"],
    "frames": {"position": "world ENU metres", "yaw": "rad about +z"},
    "measured_fields": ["position_measured", "alive", "energy", "food_active"],
    "notes": [
        "Every result reports MEASURED state read back from the simulator after "
        "the engine stepped; arguments are never echoed as if they were outcomes.",
        "Runtime creation of physical bodies is impossible in this engine, so "
        "/spawn revives a POOLED free slot of that species and /cull parks "
        "(free-falls in a pit) rather than deletes. /feed raises a parked food "
        "item; it fails with 409 when none is parked.",
        "/cull, /spawn and /feed complete only after the teleport has been "
        "stepped and the new pose read back (%d ticks)." % VERIFY_TICKS,
        "Census positions are the poses read this tick for ALIVE creatures; a "
        "parked slot reports pos null, never a stale number.",
    ],
    "config": {"arena": ARENA, "food_pool": len(foods), "food_active_max": FOOD_ACTIVE_MAX,
               "food_respawn_s": RESPAWN, "epoch_s": EPOCH_S, "watch": WATCH,
               "basic_time_step_ms": DT},
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
        if path in ("/feed", "/cull", "/spawn", "/perturb"):
            code, body = submit(path[1:], args)
            return self._send(code, body)
        return self._send(404, {"ok": False, "error": "unknown_verb", "path": path})


def serve():
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    srv.serve_forever()


threading.Thread(target=serve, daemon=True).start()
log("listening on http://127.0.0.1:%d" % PORT)


# --------------------------------------------------------------- commands
def _measure_slot(slot):
    p = roots[slot].getPosition()
    return [round(v, 4) for v in p]


def _slot_state(slot):
    c = ECO.slots[slot]
    return {"creature": slot, "id": c.genome["id"], "species": c.species,
            "alive": c.alive, "energy": round(c.energy, 3), "age_s": round(c.age_s, 3)}


def do_cmd(c, tick, now_s):
    """Execute one agent command ON THE SIM THREAD. Returns (code, body) for an
    immediate result, or None after arming c.finish to run at c.due_tick."""
    a = c.args
    try:
        if c.verb == "feed":
            if "x" not in a or "y" not in a:
                return 400, {"ok": False, "error": "missing_args", "need": ["x", "y"]}
            placed = ECO.place_food(float(a["x"]), float(a["y"]))
            if placed is None:
                return 409, {"ok": False, "error": "no_parked_food",
                             "food_active": ECO.food_active(), "food_pool": len(foods)}
            j, xyz = placed
            move_food(j, xyz)

            def fin(j=j):
                return 200, {"ok": True, "food": j,
                             "position_measured": [round(v, 4) for v in food_nodes[j].getPosition()],
                             "clamped": [xyz[0], xyz[1]] != [float(a["x"]), float(a["y"])],
                             "food_active": ECO.food_active()}
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        if c.verb == "spawn":
            sp = a.get("species")
            if sp not in ECO.species_ids():
                return 404, {"ok": False, "error": "no_such_species", "species": sp,
                             "valid": ECO.species_ids()}
            slot = ECO.free_slot(sp)
            if slot is None:
                return 409, {"ok": False, "error": "no_free_slot", "species": sp,
                             "population": ECO.population()}
            best = ECO.best_creature(sp)
            genome = copy.deepcopy(ECO.slots[slot].genome)
            genome["brain"] = copy.deepcopy(best.brain)
            genome["parent"] = best.genome["id"]
            genome["id"] = "%s_spawn%d" % (best.genome["id"], tick)
            child = ECO.revive(slot, genome, ENERGY_START, now_s)
            h = ARENA / 2.0 - 1.0
            x = float(a.get("x", RNG.uniform(-h, h)))
            y = float(a.get("y", RNG.uniform(-h, h)))
            yaw = RNG.uniform(-math.pi, math.pi)
            revive(slot, x, y, yaw)

            def fin(slot=slot, child=child):
                body = _slot_state(slot)
                body.update({"ok": True, "position_measured": _measure_slot(slot),
                             "brain_source": child.genome["parent"]})
                return 200, body
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        slot = int(a.get("creature", -1))
        if slot not in ECO.slots:
            return 404, {"ok": False, "error": "no_such_creature",
                         "creature": slot, "valid": sorted(ECO.slots)}

        if c.verb == "cull":
            if not ECO.slots[slot].alive:
                return 409, {"ok": False, "error": "already_dead", **_slot_state(slot)}
            ECO.kill(slot, now_s, cause="culled")
            park(slot)

            def fin(slot=slot):
                body = _slot_state(slot)
                body.update({"ok": True, "position_measured": _measure_slot(slot)})
                return 200, body
            c.finish, c.due_tick = fin, tick + VERIFY_TICKS
            return None

        if c.verb == "perturb":
            brain = ECO.slots[slot].brain
            applied, clamped = {}, {}
            if "steer_gain" in a:
                v = float(a["steer_gain"])
                brain["steer_gain"] = eco.clamp(v, 0.0, 1.0)
                applied["steer_gain"] = brain["steer_gain"]
                clamped["steer_gain"] = brain["steer_gain"] != v
            if "freq" in a:
                v = float(a["freq"])
                brain["freq"] = eco.clamp(v, 0.5, 3.0)
                applied["freq"] = brain["freq"]
                clamped["freq"] = brain["freq"] != v
            if not applied:
                return 400, {"ok": False, "error": "missing_args",
                             "need_one_of": ["steer_gain", "freq"]}
            body = _slot_state(slot)
            body.update({"ok": True, "applied": applied, "clamped": clamped,
                         "position_measured": _measure_slot(slot) if ECO.slots[slot].alive else None})
            return 200, body

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


def finish_epoch(sim_s):
    res = ECO.epoch_result(sim_s)
    # steering diagnostics: how often food was in range, and how often the
    # creature was actually pointed at it while it was
    for sp, v in res["species"].items():
        tot = [0, 0, 0, 0.0, 0.0]
        for c in creatures:
            if c.species == sp:
                for i in range(5):
                    tot[i] += _diag[c.slot][i]
        v["path_m"] = round(tot[3], 2)
        v["abs_yaw_rad"] = round(tot[4], 2)
        v["alive_s"] = round(tot[0] * DT_S, 1)
        v["sensed_frac"] = round(tot[1] / tot[0], 3) if tot[0] else 0.0
        v["aimed_frac"] = round(tot[2] / tot[1], 3) if tot[1] else 0.0
        v["steer_sign"] = [STEER_SIGN[c.slot] for c in creatures if c.species == sp]
        v["rightings"] = sum(_rightings[c.slot] for c in creatures if c.species == sp)
        v["steer_b_rad_per_s"] = [round(_cal[c.slot]["b"], 3) if _cal[c.slot]["b"] is not None else None
                                  for c in creatures if c.species == sp]
        v["drift_a_rad_per_s"] = [round(_cal[c.slot]["a"], 3) if _cal[c.slot]["a"] is not None else None
                                  for c in creatures if c.species == sp]
    res.update({"ticks": tick, "dt_ms": DT, "missing_defs": missing,
                "mutate_source": MUTATE_SOURCE,
                "engine_ms_per_step_median": _step_median()})
    write_json(RESULT_PATH, res)
    log("epoch done at %.1fs: births=%d deaths=%d eats=%d wd=%d -> %s"
        % (sim_s, ECO.births, ECO.deaths, ECO.eats, ECO.watchdog_kills, RESULT_PATH))


engine_ms = []


def _step_median():
    w = sorted(engine_ms[40:] or engine_ms)
    return round(w[len(w) // 2], 3) if w else None


# --------------------------------------------------------------- main loop
tick = 0
positions = {}          # slot -> [x, y, z] read THIS tick (alive only)
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
    dead_now, born_now = [], []

    # 1-3. sense -> steer -> actuate, one pose read per alive creature (getPose
    #      gives position AND heading in one round trip), one postponed field
    #      write per joint -- all drained as one batch before the motor push.
    for c in ECO.alive():
        slot = c.slot
        # getPosition + getOrientation, NOT getPose: measured in this world,
        # getPose's translation tracked the physics body but its rotation block
        # did not -- every creature's yaw read constant, so the calibration
        # wiggle measured b = 0.000 for all of them. getOrientation is the call
        # probe_steer.py proved live (it read +-500 deg turns).
        pos = tuple(roots[slot].getPosition())
        positions[slot] = list(pos)
        # 8. watchdog: MuJoCo's instability channel is read by nothing, so a
        #    NaN or a launched body would otherwise steer forever
        if ECO.watchdog(slot, pos):
            dead_now.append((slot, "watchdog"))
            continue
        brain = c.brain
        R9 = roots[slot].getOrientation()
        yaw = eco.yaw_from_orientation(R9)
        # righting reflex: R9[8] is the z component of the body's up axis
        if R9[8] < UPRIGHT_MIN:
            _fallen[slot] += 1
            if _fallen[slot] >= RIGHT_AFTER:
                revive(slot, pos[0], pos[1], yaw)
                _ramp_t0[slot] = t
                c.energy -= RIGHT_COST
                _fallen[slot] = 0
                _rightings[slot] += 1
                _pos_hist[slot].clear()
                _last_yaw.pop(slot, None)
                continue
        else:
            _fallen[slot] = 0
        hist = _pos_hist[slot]
        hist.append((tick, pos[0], pos[1]))
        while hist and hist[0][0] < tick - TRAVEL_WINDOW:
            hist.pop(0)
        heading = yaw + brain.get("heading_offset", 0.0)
        if len(hist) > 1:
            dx, dy = pos[0] - hist[0][1], pos[1] - hist[0][2]
            if dx * dx + dy * dy > TRAVEL_MIN_M * TRAVEL_MIN_M:
                heading = math.atan2(dy, dx)
        hit = eco.sense(pos, ECO.foods, brain.get("sense_radius", 4.0))
        # wall avoidance: a body pinned against a wall flips (measured), so
        # within WALL_MARGIN of one the target bearing becomes "the arena
        # centre" and overrides food.
        wall_gap = ARENA / 2.0 - max(abs(pos[0]), abs(pos[1]))
        if wall_gap < WALL_MARGIN:
            hit = (math.atan2(-pos[1], -pos[0]), wall_gap, -1)
        err = None
        d = _diag[slot]
        d[0] += 1
        lp = _last_pos.get(slot)
        if lp is not None:
            d[3] += math.hypot(pos[0] - lp[0], pos[1] - lp[1])
        _last_pos[slot] = pos
        if hit is not None:
            err = eco.heading_error(hit[0], heading, 0.0)
            d[1] += 1
            if abs(err) <= eco.STEER_DEADBAND:
                d[2] += 1
        cal = _cal[slot]
        ly = _last_yaw.get(slot)
        dyaw = eco.wrap_angle(yaw - ly) if ly is not None else 0.0
        d[4] += abs(dyaw)
        if cal["phase"] < 2:
            # calibration trial in progress: force the command, ignore food
            if cal["t0"] == 0:
                cal["t0"], cal["acc"] = tick, 0.0
            cal["acc"] += dyaw
            trial = 1.0 if cal["phase"] == 0 else -1.0
            if tick - cal["t0"] >= CAL_TICKS:
                if cal["phase"] == 0:
                    cal["dp"] = cal["acc"]
                else:
                    cal["dm"] = cal["acc"]
                    b = 0.5 * (cal["dp"] - cal["dm"])      # response to command
                    a = 0.5 * (cal["dp"] + cal["dm"])      # intrinsic drift
                    cal["b"], cal["a"] = b, a
                    STEER_SIGN[slot] = 1.0 if b >= 0.0 else -1.0
                    cal["next"] = tick + RECAL_TICKS
                cal["phase"] += 1
                cal["t0"], cal["acc"] = 0, 0.0
            left, right, _turn = eco.steer(trial * (math.pi / 2.0), brain.get("steer_gain", 0.5),
                                           0.0, t, NOISE_SEED[slot], 1.0)
        else:
            if tick >= cal["next"]:
                cal["phase"], cal["t0"] = 0, 0
            left, right, _turn = eco.steer(err, brain.get("steer_gain", 0.5),
                                           brain.get("wander", 0.4), t, NOISE_SEED[slot],
                                           STEER_SIGN[slot])
        _last_yaw[slot] = yaw
        targets = eco.joint_targets(brain, t, left, right, _target_buf[slot],
                                    ramp=min(1.0, max(0.0, t - _ramp_t0.get(slot, 0.0)) / RAMP_S))
        jf = joint_fields[slot]
        for key, v in targets.items():
            fld = jf.get(key)
            if fld is not None:
                fld.setSFFloat(v)

        # 4. metabolism
        if c.step_energy(DT_S, ECO.mass(slot)):
            dead_now.append((slot, "starved"))
            continue

        # 5. eat
        ate = ECO.eat_check(c, pos)
        if ate is not None:
            move_food(ate[0], ate[1])

        # 6. reproduce
        if c.can_reproduce():
            res = ECO.try_reproduce(c, mutate_brain, t)
            if res is not None:
                child, (dx, dy), yaw = res
                born_now.append((child.slot, pos[0] + dx, pos[1] + dy, yaw))

    for slot, cause in dead_now:
        if ECO.kill(slot, t, cause) is not None:
            park(slot)
            positions.pop(slot, None)
    for slot, x, y, yaw in born_now:
        revive(slot, x, y, yaw)

    # 7. food respawn
    for j, xyz in ECO.food_tick(DT_S):
        move_food(j, xyz)

    # agent commands (bounded, so one burst cannot stall the sim) + deferred
    # completions whose teleport has now been stepped and can be read back
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

    # 9. telemetry + census snapshot (reads never block the sim)
    if tick % CENSUS_EVERY == 0 or tick % TELEMETRY_EVERY == 0:
        snap = ECO.snapshot(tick, t, positions)
        snap["ready"] = True
        # steering debug: calibration state + live yaw per slot
        snap["steer_debug"] = {
            str(slot): {"yaw": round(_last_yaw[slot], 4) if slot in _last_yaw else None,
                        "sign": STEER_SIGN[slot],
                        "cal": {k: (round(v, 4) if isinstance(v, float) else v)
                                for k, v in _cal[slot].items()}}
            for slot in roots}
        snap["engine_ms_per_step_median"] = _step_median()
        with _census_lock:
            _census.clear()
            _census.update(snap)
        if tick % TELEMETRY_EVERY == 0:
            write_json(TELEMETRY_PATH, snap)
            log("t=%7.1fs tick=%6d pop=%s food=%d/%d births=%d deaths=%d eats=%d wd=%d step=%s ms"
                % (t, tick, ECO.population(), ECO.food_active(), len(foods), ECO.births,
                   ECO.deaths, ECO.eats, ECO.watchdog_kills, _step_median()))

    tick += 1
    if not epoch_written and t >= EPOCH_S:
        finish_epoch(t)
        epoch_written = True
        if not WATCH:
            # End the run NOW: `run-headless --duration N` is a wall-clock
            # SLEEP, and the result file is already flushed above.
            r.simulationQuit(0)
            break
