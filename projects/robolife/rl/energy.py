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

"""Energy, impact, lifecycle, fabrication and genome rules for RoboLife.

PURE Python on purpose -- no `omnisim` import, no engine, no I/O -- so every
rule the supervisor applies per tick is unit-testable in milliseconds
(`tests/test_energy.py`). The supervisor (`controllers/robolife_world/`) owns
the round trips: it reads poses and the robots' customData, hands them here,
and applies the teleports / bus writes this module hands back. Nothing in
this file knows what a DEF is.

Contract: `projects/robolife/DESIGN.md` ("Energy and lifecycle", "Modules",
"Supervisor <-> robot bus"). The constants below are the numbers written
there; change them there first.

Battery time scale
------------------
The contract's draw formula is in WATTS on a REAL Husky (idle 2 W, cruise
~25 W) against a 200 Wh pack -- that is hours of runtime, and an epoch is
240 s. The contract therefore asks for the draw to be "reported in Wh" and
TUNED so an idle robot lasts ~25 min and a cruising bare robot ~4 min. One
knob does that: BATTERY_TIME_SCALE books each simulated second of draw as
that many seconds of battery time (Wh -= W * dt_s * SCALE / 3600). At 100:
idle (2 W) on the 120 Wh start charge lasts 3600 s (60 min), a bare robot at
0.8 m/s (22.2 W) lasts 195 s (3.2 min), and a pad (+60 W) refills 60 -> 85 %
in 30 s. The two contract targets are inconsistent under the formula's own
idle:cruise ratio (1:11), so both land within ~1.5x; the cruise figure is
the one selection pressure depends on, so it is the one that was tuned to.
"""
import copy
import math

TAU = 2.0 * math.pi

# --- contract numbers (DESIGN.md) ------------------------------------------
CAP_BASE_WH = 200.0            # C = 200 Wh * (1 + 0.5 * n_battery)
CAP_PER_BATTERY = 0.5
START_FRAC = 0.60              # alive-at-start robots
IDLE_W = 2.0
BATTERY_IDLE_W = 0.5           # per battery module
ROLL_COEF = 0.9 * 0.02         # W per (kg * m/s)
WHEEL_W = 40.0 / 10.0          # W per rad/s of mean |wheel omega|
SOLAR_W = 6.0                  # per solar module, always sunlit in v1
PAD_W = 60.0
PAD_RADIUS = 0.9               # m, robot centre to pad centre (XY)
IMPACT_DV = 0.9                # m/s change in ONE tick
ARMOR_FACTOR = 2.0
DEATH_RELEASE_S = 20.0         # stop -> (release + park) after this long
FAB_MIN_FRAC = 0.85
FAB_BAY_DIST = 1.5             # m from the bay centre (XY)
FAB_COST_FRAC = 0.45
CHILD_FRAC = 0.40
BATTERY_TIME_SCALE = 100.0     # see module docstring
DETECT_RANGE = 6.0
DETECT_RANGE_MAST = 12.0
MAX_DOCKED = 2                 # one per socket
BUS_LIMIT = 1024               # bytes, customData JSON
ORDER_TTL = 3                  # bus writes an order rides unless acked
RUNAWAY_LIMIT = 1e4            # watchdog: |coordinate| beyond this = diverged
SCORE_FAB = 10.0
SCORE_CHARGE_DIV = 50.0

# Husky: 46.03 kg inertial_link + 4 x 2.637 kg wheels (urdf_import output).
HUSKY_CHASSIS_KG = 46.03
HUSKY_WHEEL_KG = 2.637
HUSKY_MASS_KG = HUSKY_CHASSIS_KG + 4 * HUSKY_WHEEL_KG      # 56.578
WHEEL_RADIUS = 0.1651
TRACK = 0.555

# Module table from DESIGN.md -- the FALLBACK when rl/modules.py [A] is not
# importable. `module_mass()` prefers A's catalogue so the two cannot drift
# once it lands.
MODULE_TABLE = {
    "battery": {"mass": 6.0},
    "solar": {"mass": 2.5},
    "mast": {"mass": 1.5},
    "armor": {"mass": 5.0},
}
MODULE_TYPES = ("battery", "solar", "mast", "armor")
SOCKETS = ("front", "rear")

# Crypt: a static slab far outside the arena where parked slots rest at zero
# velocity (revive = plain teleport; setVelocity freezes a body ~2 s). Robots
# on one row (y = CRYPT_Y), modules on the other (y = CRYPT_Y + 2). These
# MATCH rl/worldgen.py [A] (robot_park_position / module_park_position), which
# authors the parked bodies there at load; the supervisor's park teleports
# must land on the same spots or a revived slot's neighbour is on top of it.
# scene.py builds the slab from these (it covers A's fallback slab).
CRYPT_X, CRYPT_Y = 60.0, 60.0
CRYPT_SIZE = (44.0, 10.0)                # centred at (CRYPT_X + 20, CRYPT_Y)
ROBOT_PARK_PITCH = 2.0
MODULE_PARK_PITCH = 1.0
# Husky origin (base_footprint) sits 0.13228 m above the ground (A measured);
# a module's origin is the centre of its footprint ON the floor.
HUSKY_ORIGIN_Z = 0.13228
ROBOT_PARK_Z = 0.16
ROBOT_SPAWN_Z = 0.15
MODULE_PARK_Z = 0.01
MODULE_SPAWN_Z = 0.01

_MODULES = None                          # rl.modules [A], resolved lazily
MODULE_SOURCE = "DESIGN.md table"


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    w = a - TAU * math.floor((a + math.pi) / TAU)
    return w + TAU if w <= -math.pi else w


# ================================================================== modules
def _catalogue():
    """rl.modules [A] if importable. Accepts either a `MODULES` dict
    (type -> {mass, ...}) or a `mass(type)` function; anything else falls
    back to the DESIGN.md table and says so in MODULE_SOURCE."""
    global _MODULES, MODULE_SOURCE
    if _MODULES is not None:
        return _MODULES
    try:
        from . import modules as M           # noqa: WPS433
    except Exception:                        # noqa: BLE001  (absent / broken)
        try:
            import modules as M              # noqa: F811  (sys.path form)
        except Exception:                    # noqa: BLE001
            _MODULES = {}
            return _MODULES
    table = {}
    src = getattr(M, "MODULES", None) or getattr(M, "CATALOGUE", None)
    if isinstance(src, dict):
        for t, rec in src.items():
            m = rec.get("mass", rec.get("mass_kg")) if isinstance(rec, dict) else None
            if m is not None:
                table[t] = {"mass": float(m)}
    elif callable(getattr(M, "mass", None)):
        for t in MODULE_TYPES:
            try:
                table[t] = {"mass": float(M.mass(t))}
            except Exception:                # noqa: BLE001
                pass
    if table:
        MODULE_SOURCE = "rl.modules"
    _MODULES = table
    return _MODULES


def module_source():
    """Where module masses come from ("rl.modules" once A's catalogue is
    importable, else "DESIGN.md table"). Resolves the catalogue first, so
    it is never stale."""
    _catalogue()
    return MODULE_SOURCE


def module_mass(mtype):
    """Mass of one module of `mtype` (kg): A's catalogue, else the table."""
    rec = _catalogue().get(mtype) or MODULE_TABLE.get(mtype)
    if rec is None:
        raise KeyError("unknown module type %r" % (mtype,))
    return float(rec["mass"])


def mass_total(docked_types):
    """Husky plus every docked module."""
    return HUSKY_MASS_KG + sum(module_mass(t) for t in docked_types if t)


# =================================================================== energy
def capacity_wh(n_battery):
    return CAP_BASE_WH * (1.0 + CAP_PER_BATTERY * int(n_battery))


def drain_w(m_total_kg, v_mps, wheel_omega_mean, n_battery, n_solar):
    """Net electrical draw in watts (positive = the pack is emptying):
    2 idle + 0.5/battery + 0.9*m*|v|*0.02 + 40*|omega|/10 - 6/solar."""
    return (IDLE_W + BATTERY_IDLE_W * int(n_battery)
            + ROLL_COEF * float(m_total_kg) * abs(float(v_mps))
            + WHEEL_W * abs(float(wheel_omega_mean))
            - SOLAR_W * int(n_solar))


def wh_per_s(watts):
    """Wh booked per simulated second of `watts` draw (see docstring)."""
    return float(watts) * BATTERY_TIME_SCALE / 3600.0


def pad_charge_w(dist_m):
    return PAD_W if float(dist_m) <= PAD_RADIUS else 0.0


def wheel_omega_mean(v_mps, yaw_rate):
    """Mean |wheel omega| of a skid-steer base from its measured body speed
    and yaw rate: (|v| + |w|*track/2) / r."""
    return (abs(float(v_mps)) + abs(float(yaw_rate)) * TRACK / 2.0) / WHEEL_RADIUS


def impact(dv, has_armor):
    """True when a one-tick speed change exceeds the threshold (x2 armored)."""
    thr = IMPACT_DV * (ARMOR_FACTOR if has_armor else 1.0)
    return abs(float(dv)) > thr


def nearest_pad_dist(pos, pads):
    best = float("inf")
    for p in pads:
        d = math.hypot(pos[0] - p[0], pos[1] - p[1])
        if d < best:
            best = d
    return best


# =================================================================== genome
GENOME_RANGES = {
    "cruise_speed": (0.4, 1.2),
    "charge_at": (0.15, 0.5),
    "greed": (0.0, 1.0),
    "caution": (0.3, 2.0),
    "explore_radius": (3.0, 12.0),
}
PREF_RANGE = (0.0, 1.0)


def default_genome():
    return {
        "cruise_speed": 0.8,
        "charge_at": 0.3,
        "module_pref": {"battery": 0.5, "solar": 0.5, "mast": 0.5, "armor": 0.5},
        "greed": 0.5,
        "caution": 1.0,
        "explore_radius": 8.0,
    }


def random_genome(rng):
    g = default_genome()
    for k, (lo, hi) in GENOME_RANGES.items():
        g[k] = rng.uniform(lo, hi)
    g["module_pref"] = {t: rng.uniform(*PREF_RANGE) for t in MODULE_TYPES}
    return g


def clamp_genome(g):
    """Fill defaults and clamp every field into its contract range."""
    out = default_genome()
    out.update({k: v for k, v in (g or {}).items() if k in out})
    for k, (lo, hi) in GENOME_RANGES.items():
        out[k] = clamp(float(out[k]), lo, hi)
    pref = dict(default_genome()["module_pref"])
    src = out.get("module_pref") or {}
    for t in MODULE_TYPES:
        pref[t] = clamp(float(src.get(t, pref[t])), *PREF_RANGE)
    out["module_pref"] = pref
    return out


def validate(g):
    """List of problems ([] = ok): missing keys or values outside range."""
    problems = []
    for k, (lo, hi) in GENOME_RANGES.items():
        v = g.get(k) if isinstance(g, dict) else None
        if not isinstance(v, (int, float)):
            problems.append("%s missing" % k)
        elif not (lo <= v <= hi):
            problems.append("%s=%r outside [%g, %g]" % (k, v, lo, hi))
    pref = g.get("module_pref") if isinstance(g, dict) else None
    if not isinstance(pref, dict):
        problems.append("module_pref missing")
    else:
        for t in MODULE_TYPES:
            v = pref.get(t)
            if not isinstance(v, (int, float)) or not (PREF_RANGE[0] <= v <= PREF_RANGE[1]):
                problems.append("module_pref.%s=%r" % (t, v))
    return problems


def mutate(genome, rng, sigma=0.15):
    """Gaussian creep, relative to each field's range width, clamped. Always
    returns a NEW dict that passes validate()."""
    g = clamp_genome(copy.deepcopy(genome))
    for k, (lo, hi) in GENOME_RANGES.items():
        g[k] = clamp(g[k] + rng.gauss(0.0, sigma * (hi - lo)), lo, hi)
    for t in MODULE_TYPES:
        g["module_pref"][t] = clamp(g["module_pref"][t] + rng.gauss(0.0, sigma), *PREF_RANGE)
    return g


# ===================================================================== bus
def encode_bus(msg, limit=BUS_LIMIT):
    """Compact JSON for customData. If it exceeds `limit` bytes the trailing
    `modules` entries are dropped (the caller sorts them nearest-first) until
    it fits; the genome and orders are never dropped. Returns (text, n_kept)."""
    import json
    mods = list(msg.get("modules", []))
    while True:
        m = dict(msg)
        m["modules"] = mods
        text = json.dumps(m, separators=(",", ":"))
        if len(text.encode("utf-8")) <= limit or not mods:
            return text, len(mods)
        mods = mods[:-1]


def decode_status(text):
    """Robot -> supervisor customData. Never raises; a blank or malformed
    string is an empty dict (the robot has not spoken yet)."""
    import json
    if not text:
        return {}
    try:
        d = json.loads(text)
    except ValueError:
        return {}
    return d if isinstance(d, dict) else {}


# ==================================================================== robot
class Robot:
    """One pooled Husky slot. A slot is reused across lifetimes: when this
    robot dies it stays in `Fleet.history` and a fabricated child later takes
    the slot as a NEW Robot."""

    __slots__ = ("slot", "genome", "lineage", "id", "parent", "charge_wh",
                 "alive", "parked", "docked", "born_at", "died_at", "dying_since",
                 "cause", "orders", "status", "age_s", "fabrications",
                 "modules_docked", "charge_collected", "distance_m", "impacts",
                 "last_v")

    def __init__(self, slot, genome, lineage, rid, charge_frac=START_FRAC,
                 alive=True, born_at=0.0, parent=None, docked=None):
        self.slot = int(slot)
        self.genome = clamp_genome(genome)
        self.lineage = lineage
        self.id = rid
        self.parent = parent
        self.docked = {"front": None, "rear": None}
        if docked:
            self.docked.update({k: docked.get(k) for k in SOCKETS})
        self.charge_wh = float(charge_frac) * self.capacity() if charge_frac is not None else 0.0
        self.alive = bool(alive)
        self.parked = not self.alive
        self.born_at = float(born_at)
        self.died_at = None
        self.dying_since = None
        self.cause = None
        self.orders = {}              # order -> bus sends left (sticky, see order())
        self.status = {}              # last decoded robot -> supervisor report
        self.age_s = 0.0
        self.fabrications = 0
        self.modules_docked = 0
        self.charge_collected = 0.0
        self.distance_m = 0.0
        self.impacts = 0
        self.last_v = None

    # ------------------------------------------------------------ derived
    def docked_ids(self):
        return [j for j in self.docked.values() if j is not None]

    def n_docked(self):
        return len(self.docked_ids())

    def has_free_socket(self):
        return any(self.docked[s] is None for s in SOCKETS)

    def capacity(self, n_battery=None):
        return capacity_wh(n_battery if n_battery is not None else 0)

    def frac(self, capacity):
        return clamp(self.charge_wh / capacity, 0.0, 1.0) if capacity > 0 else 0.0

    def state_hint(self):
        if not self.alive:
            return "dead"
        return "ok"

    # ------------------------------------------------------------- energy
    def step(self, dt_s, m_total_kg, v_mps, wheel_omega, n_battery, n_solar,
             pad_dist_m, capacity):
        """Age, drain, charge for dt_s. Returns True when THIS step emptied
        the pack (the caller then orders `stop` and books the death)."""
        if not self.alive:
            return False
        self.age_s += dt_s
        self.distance_m += abs(float(v_mps)) * dt_s
        draw = drain_w(m_total_kg, v_mps, wheel_omega, n_battery, n_solar)
        gain = pad_charge_w(pad_dist_m)
        net_w = gain - draw                      # positive = charging
        delta = wh_per_s(net_w) * dt_s
        before = self.charge_wh
        self.charge_wh = clamp(self.charge_wh + delta, 0.0, capacity)
        if self.charge_wh > before:
            self.charge_collected += self.charge_wh - before
        return self.charge_wh <= 0.0

    def order(self, o, ttl=ORDER_TTL):
        """Queue an order. It rides the next ORDER_TTL bus writes (the field
        is shared both ways, so a single write can be overwritten by the
        robot's own reply before it is read) and is retired early by
        report() once the robot's status shows it applied."""
        self.orders[o] = max(int(ttl), self.orders.get(o, 0))

    def take_orders(self):
        out = list(self.orders)
        for o in out:
            self.orders[o] -= 1
            if self.orders[o] <= 0:
                del self.orders[o]
        return out or ["none"]

    def ack_orders(self, status):
        """Retire orders the robot's report shows applied."""
        docked = status.get("docked") if isinstance(status, dict) else None
        if isinstance(docked, dict):
            for s in SOCKETS:
                if docked.get(s) is None:
                    self.orders.pop("release_" + s, None)
        if status.get("state") == "stopped":
            self.orders.pop("stop", None)


# ==================================================================== fleet
def robot_park_translation(slot):
    return [CRYPT_X + ROBOT_PARK_PITCH * int(slot), CRYPT_Y, ROBOT_PARK_Z]


def module_park_translation(j):
    return [CRYPT_X + MODULE_PARK_PITCH * int(j), CRYPT_Y + 2.0, MODULE_PARK_Z]


class Fleet:
    """Bookkeeping over robot slots and the module pool. No engine access:
    every method that moves something RETURNS the action for the supervisor
    to apply -- ("park", slot) / ("revive", slot, xyz, yaw) /
    ("module", j, xyz, yaw) -- and orders go onto the robot, delivered by
    the next bus."""

    def __init__(self, robots, modules, config, rng):
        """
        robots   iterable of Robot (alive or parked); one per slot
        modules  iterable of {id, type, loose, holder?, pos?, yaw?}
        config   {arena, pads: [[x,y],..], bay: [x,y], margin?}
        rng      random.Random -- the ONLY source of randomness here
        """
        self.slots = {r.slot: r for r in robots}
        self.history = list(robots)
        self.modules = {}
        for m in modules:
            j = int(m["id"])
            self.modules[j] = {
                "id": j, "type": m["type"], "loose": bool(m.get("loose", True)),
                "holder": m.get("holder"), "socket": m.get("socket"),
                "pos": list(m.get("pos") or module_park_translation(j)),
                "yaw": float(m.get("yaw", 0.0)),
            }
        # A module a robot starts with is neither loose nor parked.
        for r in self.slots.values():
            for s, j in r.docked.items():
                if j is not None and j in self.modules:
                    self.modules[j].update({"loose": False, "holder": r.slot, "socket": s})
        self.arena = float(config.get("arena", 24.0))
        self.margin = float(config.get("margin", 1.5))
        self.pads = [list(p) for p in config.get("pads", [])]
        self.bay = list(config.get("bay", [0.0, 0.0]))
        self.rng = rng
        self.births = self.deaths = self.releases = self.docks = self.scatters = 0
        self.watchdog_kills = 0
        self.peak_alive = sum(1 for r in self.slots.values() if r.alive)
        self._next_child = {}

    # ---------------------------------------------------------------- queries
    def alive(self):
        return [r for r in self.slots.values() if r.alive]

    def present(self):
        """Robots in the arena: alive or dying (dead but not yet parked)."""
        return [r for r in self.slots.values() if not r.parked]

    def free_slot(self):
        for slot in sorted(self.slots):
            if self.slots[slot].parked:
                return slot
        return None

    def lineages(self):
        return sorted({r.lineage for r in self.history if r.lineage})

    def docked_types(self, slot):
        r = self.slots[slot]
        return [self.modules[j]["type"] for j in r.docked_ids() if j in self.modules]

    def n_of(self, slot, mtype):
        return sum(1 for t in self.docked_types(slot) if t == mtype)

    def mass(self, slot):
        return mass_total(self.docked_types(slot))

    def capacity(self, slot):
        return capacity_wh(self.n_of(slot, "battery"))

    def detection_range(self, slot):
        return DETECT_RANGE_MAST if self.n_of(slot, "mast") else DETECT_RANGE

    def loose_modules(self):
        return [m for m in self.modules.values() if m["loose"]]

    def parked_modules(self):
        return [m for m in self.modules.values() if not m["loose"] and m["holder"] is None]

    def visible_modules(self, slot, pos):
        """Loose modules within the robot's detection range, nearest first."""
        rng2 = self.detection_range(slot) ** 2
        out = []
        for m in self.loose_modules():
            d2 = (m["pos"][0] - pos[0]) ** 2 + (m["pos"][1] - pos[1]) ** 2
            if d2 <= rng2:
                out.append((d2, m))
        out.sort(key=lambda t: t[0])
        return [m for _, m in out]

    # ----------------------------------------------------------- positions
    def random_arena_point(self, z=MODULE_SPAWN_Z):
        h = self.arena / 2.0 - self.margin
        return [self.rng.uniform(-h, h), self.rng.uniform(-h, h), z]

    def in_arena(self, pos, slack=0.5):
        h = self.arena / 2.0 + slack
        return (abs(pos[0]) <= h and abs(pos[1]) <= h and -1.0 < pos[2] < 5.0
                and all(math.isfinite(v) for v in pos))

    def set_module_pos(self, j, pos, yaw=None):
        m = self.modules[j]
        m["pos"] = [float(pos[0]), float(pos[1]), float(pos[2])]
        if yaw is not None:
            m["yaw"] = float(yaw)

    def module_tick(self):
        """Loose modules that ended off the arena (knocked over a wall,
        fallen through, launched) are re-teleported to a random arena point.
        Returns [("module", j, xyz, yaw), ...]."""
        acts = []
        for m in self.loose_modules():
            if not self.in_arena(m["pos"]):
                xyz = self.random_arena_point()
                yaw = self.rng.uniform(-math.pi, math.pi)
                m["pos"], m["yaw"] = xyz, yaw
                acts.append(("module", m["id"], xyz, yaw))
        return acts

    def scatter(self, n):
        """Drop up to n PARKED modules into the arena. Returns the teleports."""
        acts = []
        for m in self.parked_modules()[:max(0, int(n))]:
            xyz = self.random_arena_point()
            yaw = self.rng.uniform(-math.pi, math.pi)
            m.update({"loose": True, "holder": None, "socket": None,
                      "pos": xyz, "yaw": yaw})
            self.scatters += 1
            acts.append(("module", m["id"], xyz, yaw))
        return acts

    # --------------------------------------------------------------- report
    def report(self, slot, status):
        """Ingest one robot -> supervisor bus message. Reconciles `docked`
        against the module table: a module the robot now holds is not loose;
        one it held and no longer reports is loose where it lies (its pos is
        refreshed by the supervisor's next module read)."""
        r = self.slots[slot]
        if not isinstance(status, dict):
            return
        r.status = status
        r.ack_orders(status)
        docked = status.get("docked")
        if not isinstance(docked, dict):
            return
        new = {}
        for s in SOCKETS:
            j = docked.get(s)
            if j is None:
                new[s] = None
                continue
            try:
                j = int(j)
            except (TypeError, ValueError):
                new[s] = None
                continue
            m = self.modules.get(j)
            if m is None:
                new[s] = None
                continue
            # A module another robot holds cannot also be here; the earlier
            # holder keeps it until it reports otherwise.
            if m["holder"] is not None and m["holder"] != slot:
                new[s] = None
                continue
            new[s] = j
        for s in SOCKETS:
            old, nj = r.docked[s], new[s]
            if old == nj:
                continue
            if old is not None and old not in new.values():
                self._release(old)
            if nj is not None:
                m = self.modules[nj]
                if m["loose"] or m["holder"] != slot:
                    r.modules_docked += 1
                    self.docks += 1
                m.update({"loose": False, "holder": slot, "socket": s})
        r.docked = new

    def _release(self, j):
        m = self.modules[j]
        m.update({"loose": True, "holder": None, "socket": None})
        self.releases += 1

    def release_all(self, slot):
        r = self.slots[slot]
        for s in SOCKETS:
            j = r.docked[s]
            if j is not None:
                self._release(j)
            r.docked[s] = None

    # ---------------------------------------------------------------- tick
    def energy_tick(self, slot, dt_s, v_mps, yaw_rate, pos):
        """Apply one tick of drain/charge to an alive robot. Returns True when
        it just died (the caller books it with kill())."""
        r = self.slots[slot]
        if not r.alive:
            return False
        cap = self.capacity(slot)
        return r.step(dt_s, self.mass(slot), v_mps, wheel_omega_mean(v_mps, yaw_rate),
                      self.n_of(slot, "battery"), self.n_of(slot, "solar"),
                      nearest_pad_dist(pos, self.pads), cap)

    def impact_tick(self, slot, dv):
        """Order the release of one module (rear first) on an impact. Returns
        the order issued or None."""
        r = self.slots[slot]
        if not r.alive or not impact(dv, self.n_of(slot, "armor") > 0):
            return None
        r.impacts += 1
        for s in ("rear", "front"):
            if r.docked[s] is not None:
                o = "release_" + s
                r.order(o)
                return o
        return None

    def watchdog(self, slot, pos):
        r = self.slots[slot]
        if r.parked:
            return False
        return any((not math.isfinite(v)) or abs(v) > RUNAWAY_LIMIT for v in pos)

    # ---------------------------------------------------------------- death
    def kill(self, slot, now_s, cause="empty"):
        """charge <= 0: order `stop`. The chassis stays where it is for
        DEATH_RELEASE_S, then death_tick() releases its modules and parks it.
        Returns True if this call killed it."""
        r = self.slots[slot]
        if not r.alive:
            return False
        r.alive = False
        r.died_at = now_s
        r.dying_since = now_s
        r.cause = cause
        r.charge_wh = 0.0
        r.order("stop")
        self.deaths += 1
        if cause == "watchdog":
            self.watchdog_kills += 1
        return True

    def death_tick(self, now_s, force=False):
        """Dying robots past DEATH_RELEASE_S: modules loose, chassis parked.
        Returns [("park", slot), ...]. `force` skips the wait (watchdog)."""
        acts = []
        for r in self.slots.values():
            if r.alive or r.parked:
                continue
            if force or r.dying_since is None or now_s - r.dying_since >= DEATH_RELEASE_S:
                for s in SOCKETS:
                    if r.docked[s] is not None:
                        r.order("release_" + s)
                self.release_all(r.slot)
                r.parked = True
                acts.append(("park", r.slot))
        return acts

    # ---------------------------------------------------------- fabrication
    def can_fabricate(self, slot, pos):
        """Why a robot may not fabricate now, or None when it may."""
        r = self.slots[slot]
        if not r.alive:
            return "dead"
        cap = self.capacity(slot)
        if r.charge_wh < FAB_MIN_FRAC * cap:
            return "charge %.0f%% < %.0f%%" % (100 * r.frac(cap), 100 * FAB_MIN_FRAC)
        d = math.hypot(pos[0] - self.bay[0], pos[1] - self.bay[1])
        if d > FAB_BAY_DIST:
            return "bay %.2f m away > %.1f" % (d, FAB_BAY_DIST)
        if self.free_slot() is None:
            return "no free slot"
        return None

    def fabricate(self, slot, pos, now_s, force=False):
        """Parent pays 45 % of ITS capacity; the child revives at the bay with
        40 % of its own (bare) capacity and mutate(genome). Returns
        (child, xyz, yaw) or None."""
        r = self.slots[slot]
        why = self.can_fabricate(slot, pos)
        if why is not None and not (force and why not in ("dead", "no free slot")):
            return None
        child_slot = self.free_slot()
        if child_slot is None:
            return None
        cap = self.capacity(slot)
        r.charge_wh = max(0.0, r.charge_wh - FAB_COST_FRAC * cap)
        r.fabrications += 1
        n = self._next_child.get(r.id, 0) + 1
        self._next_child[r.id] = n
        child = Robot(child_slot, mutate(r.genome, self.rng), r.lineage,
                      "%s_c%d" % (r.id, n), charge_frac=CHILD_FRAC, alive=True,
                      born_at=now_s, parent=r.id)
        self.slots[child_slot] = child
        self.history.append(child)
        self.births += 1
        self.peak_alive = max(self.peak_alive, len(self.alive()))
        # Opposite side of the bay from the parent, 2.2 m out, facing away.
        a = math.atan2(self.bay[1] - pos[1], self.bay[0] - pos[0])
        if not math.isfinite(a) or (pos[0] == self.bay[0] and pos[1] == self.bay[1]):
            a = self.rng.uniform(-math.pi, math.pi)
        xyz = [self.bay[0] + 2.2 * math.cos(a), self.bay[1] + 2.2 * math.sin(a), ROBOT_SPAWN_Z]
        return child, xyz, a

    def revive(self, slot, genome, lineage, rid, now_s, charge_frac=START_FRAC):
        """Put a fresh robot into a parked slot (driver seeding / agent). Not
        a fabrication; not scored as one."""
        old = self.slots[slot]
        if not old.parked:
            raise ValueError("slot %d is occupied" % slot)
        r = Robot(slot, genome, lineage, rid, charge_frac=charge_frac, alive=True, born_at=now_s)
        self.slots[slot] = r
        self.history.append(r)
        self.peak_alive = max(self.peak_alive, len(self.alive()))
        return r

    # ------------------------------------------------------------------ bus
    def bus_for(self, slot, now_s, pos):
        """Supervisor -> robot message (a dict; encode with encode_bus)."""
        r = self.slots[slot]
        cap = self.capacity(slot)
        frac = r.frac(cap)
        hint = "dead" if not r.alive else ("low" if frac < r.genome["charge_at"] else "ok")
        mods = [{"id": m["id"], "type": m["type"], "x": round(m["pos"][0], 2),
                 "y": round(m["pos"][1], 2), "yaw": round(m["yaw"], 2), "loose": True}
                for m in self.visible_modules(slot, pos)]
        g = r.genome
        return {
            "t": round(now_s, 2),
            "batt": round(frac, 3),
            "cap_wh": round(cap, 1),
            "state_hint": hint,
            "pads": [[round(p[0], 2), round(p[1], 2)] for p in self.pads],
            "bay": [round(self.bay[0], 2), round(self.bay[1], 2)],
            "modules": mods,
            "orders": r.take_orders(),
            "genome": {
                "cruise_speed": round(g["cruise_speed"], 3),
                "charge_at": round(g["charge_at"], 3),
                "module_pref": {t: round(v, 2) for t, v in g["module_pref"].items()},
                "greed": round(g["greed"], 3),
                "caution": round(g["caution"], 3),
                "explore_radius": round(g["explore_radius"], 2),
            },
        }

    # ------------------------------------------------------------ reporting
    def best_robot(self, lineage):
        best = None
        for r in self.history:
            if r.lineage != lineage:
                continue
            key = (r.fabrications, r.modules_docked, r.charge_collected, r.age_s)
            if best is None or key > best[0]:
                best = (key, r)
        return best[1] if best else None

    def lineage_scores(self):
        per = {}
        for r in self.history:
            if not r.lineage:
                continue
            rec = per.setdefault(r.lineage, {
                "score": 0.0, "fabrications": 0, "modules_docked": 0,
                "charge_collected_wh": 0.0, "deaths": 0, "impacts": 0,
                "distance_m": 0.0, "robots": 0, "alive": 0, "lifespans_s": []})
            rec["fabrications"] += r.fabrications
            rec["modules_docked"] += r.modules_docked
            rec["charge_collected_wh"] += r.charge_collected
            rec["impacts"] += r.impacts
            rec["distance_m"] += r.distance_m
            rec["robots"] += 1
            if r.alive:
                rec["alive"] += 1
            if r.died_at is not None:
                rec["deaths"] += 1
                rec["lifespans_s"].append(r.died_at - r.born_at)
        for lin, rec in per.items():
            rec["score"] = (SCORE_FAB * rec["fabrications"] + rec["modules_docked"]
                            + rec["charge_collected_wh"] / SCORE_CHARGE_DIV)
            ls = rec.pop("lifespans_s")
            rec["mean_lifespan_s"] = round(sum(ls) / len(ls), 2) if ls else None
            rec["charge_collected_wh"] = round(rec["charge_collected_wh"], 2)
            rec["distance_m"] = round(rec["distance_m"], 2)
            rec["score"] = round(rec["score"], 3)
            b = self.best_robot(lin)
            rec["best_genome"] = copy.deepcopy(b.genome) if b else None
            rec["best_id"] = b.id if b else None
        return per

    def snapshot(self, tick, sim_s, positions):
        """Telemetry dict. `positions` maps slot -> measured [x, y, z] for the
        robots the supervisor read this tick; parked slots report pos null."""
        robots = {}
        for slot in sorted(self.slots):
            r = self.slots[slot]
            cap = self.capacity(slot)
            robots[str(slot)] = {
                "id": r.id, "lineage": r.lineage, "alive": r.alive, "parked": r.parked,
                "charge_wh": round(r.charge_wh, 2), "batt": round(r.frac(cap), 3),
                "cap_wh": cap, "mass_kg": round(self.mass(slot), 2),
                "docked": dict(r.docked), "state": r.status.get("state"),
                "pos": [round(v, 3) for v in positions[slot]] if slot in positions else None,
                "age_s": round(r.age_s, 1), "fabrications": r.fabrications,
                "modules_docked": r.modules_docked, "impacts": r.impacts,
                "cause": r.cause,
            }
        modules = {str(j): {"type": m["type"], "loose": m["loose"], "holder": m["holder"],
                            "pos": [round(v, 3) for v in m["pos"]]}
                   for j, m in sorted(self.modules.items())}
        return {
            "tick": tick, "sim_s": round(sim_s, 3),
            "alive": len(self.alive()), "present": len(self.present()),
            "births": self.births, "deaths": self.deaths, "docks": self.docks,
            "releases": self.releases, "scatters": self.scatters,
            "loose_modules": len(self.loose_modules()),
            "parked_modules": len(self.parked_modules()),
            "robots": robots, "modules": modules,
            "pads": self.pads, "bay": self.bay,
        }

    def epoch_result(self, sim_s):
        return {
            "sim_s": round(sim_s, 3),
            "births": self.births, "deaths": self.deaths, "docks": self.docks,
            "releases": self.releases, "scatters": self.scatters,
            "watchdog_kills": self.watchdog_kills, "peak_alive": self.peak_alive,
            "alive_at_end": len(self.alive()),
            "lineages": self.lineage_scores(),
            "module_source": module_source(),
        }
