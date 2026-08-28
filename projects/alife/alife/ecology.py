#!/usr/bin/env python3
"""Ecology for the alife v2 terrarium: energy, sensing, steering, reproduction.

PURE Python on purpose -- no `omnisim` import, no engine, no I/O -- so every
rule the director applies per tick is unit-testable in milliseconds
(`tests/test_ecology.py`). The director (`controllers/terrarium_life/`) owns
the supervisor round trips: it reads poses, hands them here, and applies the
teleports and joint targets this module hands back. Nothing in this file knows
what a DEF is.

Contract: `projects/alife/DESIGN_v2.md` ("Director" section). The constants
below are the numbers written there; change them there first.

Frames and units
----------------
World is ENU, z up. A creature's torso capsule lies along its body +x at rest,
so its heading is the yaw of the body +x axis. `Node.getOrientation()` returns
a 3x3 ROW-major rotation matrix whose first COLUMN is the body +x axis in world
coordinates -- yaw = atan2(R[3], R[0]). Bearings are world-frame angles from
the torso to a food item; the steering error is bearing - heading, wrapped.
Distances are XY (the torso rides ~0.3 m above a food sphere at z 0.09, so a
3-D distance would put a bias on every eat radius).

Food
----
The food pool is authored at load (runtime spawn has no physics, and food has
no physics anyway). An item is ACTIVE iff its z >= 0; parked items sit at
z = -3 under the floor. This module is the authority on where every food item
is -- food never moves except by our own teleports, so the director never has
to read a food position back.
"""
import copy
import math

TAU = 2.0 * math.pi

# --- contract numbers (DESIGN_v2.md, "Director") ---------------------------
GAIT_CEIL = 1.55           # |bias| + |amp| ceiling, inside the authored ±1.8
DENSITY = 250.0            # kg/m^3, matches worldgen2's inertia derivation
# Metabolism. The contract's 0.012/0.02 burned ~5 energy in 120 s, so nothing
# ever starved and there was no selection pressure inside an epoch. At 10x a
# creature that never eats dies in ~90 s and must find food every ~40 s.
COST_MASS = 0.12           # energy/s per kg
COST_GAIT = 0.2            # energy/s per unit of sum(amp * freq)
STEER_DEADBAND = 0.12      # rad; inside it the creature goes "straight"
STEER_GAIN_MAX = 0.32      # measured on the walker archetype: amp asymmetry 0.3
                           # turns cleanly (-0.30 rad/m, still 0.79 m/s, no flip);
                           # 0.6 collapses the gait (0.18 m/s, flips). Never exceed.
STEER_ERR_FULL = 0.6       # rad of heading error at which the command saturates
EAT_ENERGY = 45.0
ENERGY_CAP = 200.0
REPRO_THRESHOLD = 140.0    # strictly greater than
REPRO_COST = 55.0
CHILD_ENERGY = 55.0
EAT_MARGIN = 0.25          # eat radius = torso.length/2 + EAT_MARGIN
SPAWN_OFFSET = 0.8         # child revived this far beside the parent
FOOD_Z = 0.09              # active food sphere centre
FOOD_PARK_Z = -3.0         # parked food, under the floor
RUNAWAY_LIMIT = 1e4        # watchdog: |coordinate| beyond this = diverged
ENERGY_START = 100.0       # alive_at_start creatures (not in the contract;
                           # config.json may override with "energy_start")

# Deterministic wander: two incommensurate slow sines. The ratio is sqrt(2) so
# the sum never repeats; the base rate is slow enough (period ~20 s) that a
# foodless creature drifts in wide arcs rather than jittering.
WANDER_W1 = 0.31
WANDER_W2 = 0.31 * math.sqrt(2.0)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ============================================================ geometry / sense
def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    w = a - TAU * math.floor((a + math.pi) / TAU)
    return w + TAU if w <= -math.pi else w


def yaw_from_orientation(R9):
    """Yaw (rad, ENU) of the body +x axis given the 9-float row-major rotation
    matrix from `Node.getOrientation()`. Column 0 is the body x axis in world:
    (R[0], R[3], R[6])."""
    return math.atan2(R9[3], R9[0])


def yaw_from_pose(P16):
    """Same, from the 16-float row-major 4x4 of `Node.getPose()` -- one round
    trip yields both position (P[3], P[7], P[11]) and heading."""
    return math.atan2(P16[4], P16[0])


def sense(pos, foods, sense_radius):
    """Nearest ACTIVE food within `sense_radius` (XY distance).

    `foods` is a sequence of [x, y, z]; z < 0 means parked and is ignored.
    Returns (bearing_world_rad, dist_m, food_index) or None.
    """
    px, py = pos[0], pos[1]
    best, best_d2 = None, sense_radius * sense_radius
    for j, f in enumerate(foods):
        if f[2] < 0.0:
            continue
        dx, dy = f[0] - px, f[1] - py
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2, best = d2, (dx, dy, j)
    if best is None:
        return None
    dx, dy, j = best
    return math.atan2(dy, dx), math.sqrt(best_d2), j


def heading_error(bearing, yaw, heading_offset=0.0):
    """err = wrap(bearing - (yaw + heading_offset)); positive = food is to the
    creature's LEFT (counter-clockwise, ENU)."""
    return wrap_angle(bearing - (yaw + heading_offset))


# ================================================================== steering
def wander_noise(t, seed):
    """Slow deterministic noise in [-1, 1], distinct per seed."""
    p1 = (seed * 0.7548776662) % 1.0 * TAU        # golden-ratio-ish spreads
    p2 = (seed * 0.5698402910) % 1.0 * TAU
    return 0.5 * (math.sin(WANDER_W1 * t + p1) + math.sin(WANDER_W2 * t + p2))


def steer(err, steer_gain, wander, t, noise_seed, sign=1.0):
    """Differential stride from a heading error -- PROPORTIONAL, small gains.

    err        wrapped bearing error (rad) or None when no food is sensed
    sign       +1/-1 steering polarity (the director calibrates it)
    Returns (left_scale, right_scale, turn) with turn in [-1, 1].

    Measured on the designed walker (probe_steer2): amplitude asymmetry of
    0.3 (left 0.7 / right 1.3) turns at -0.30 rad/m while the body keeps
    walking at 0.79 m/s with no flip; 0.6 collapses it. So the gain is capped
    at STEER_GAIN_MAX and the command is proportional with a small deadband.
    Sign convention: a POSITIVE turn (food to the left, CCW) enlarges the LEFT
    stride -- on the archetype left-0.7/right-1.3 yawed clockwise, so the
    opposite asymmetry yaws CCW. The wiggle calibration corrects bodies that
    answer the other way.
    """
    if err is None:
        turn = wander * wander_noise(t, noise_seed)
    elif abs(err) <= STEER_DEADBAND:
        turn = 0.0
    else:
        turn = clamp(err / STEER_ERR_FULL, -1.0, 1.0)
    g = clamp(abs(steer_gain), 0.0, STEER_GAIN_MAX)
    cmd = sign * turn
    left = max(0.0, 1.0 + g * cmd)
    right = max(0.0, 1.0 - g * cmd)
    return left, right, turn


# ================================================================= actuation
def _target(joint, phase_total, scale, bias_add=0.0, ramp=1.0):
    """bias + amp*scale*sin(phase), with |bias|+|amp*scale| clamped to the
    ceiling so a hard steer (scale up to 2) can never reach the motor limit
    and trigger the one-WARNING-per-joint-per-tick log storm. bias_add is a
    per-side stride offset (a second steering channel)."""
    bias = clamp(joint["bias"] + bias_add, -GAIT_CEIL, GAIT_CEIL)
    amp = abs(joint["amp"]) * scale
    room = GAIT_CEIL - abs(bias)
    if amp > room:
        amp = room
    return ramp * (bias + amp * math.sin(phase_total))


def joint_targets(brain, t, left_scale=1.0, right_scale=1.0, out=None,
                  left_bias=0.0, right_bias=0.0, ramp=1.0):
    """Every joint's position target at time t.

    Returns {(pair_k, 'L'|'R', 'H'|'K'): target}. `ramp` in [0, 1] fades the
    whole gait (bias AND amplitude) in: measured, a newborn whose knees snap
    from 0 to their bias in one tick is thrown 10 cm into the air and lands on
    its side (probe_steer2 archetype: torso z 0.215 -> 0.319 -> 0.050). The right side carries
    `mirror_phase` in addition to the joint's own phase. Pairs without a knee
    entry (single-segment limbs) produce hip targets only. Pass `out` to reuse
    a dict across ticks (the director does, to avoid per-tick allocation).
    """
    if out is None:
        out = {}
    w = TAU * brain["freq"] * t
    mirror = brain.get("mirror_phase", math.pi)
    for k, pair in enumerate(brain["pairs"]):
        for side, scale, add, badd in (("L", left_scale, 0.0, left_bias),
                                       ("R", right_scale, mirror, right_bias)):
            hip = pair.get("hip")
            if hip is not None:
                out[(k, side, "H")] = _target(hip, w + hip["phase"] + add, scale, badd, ramp)
            knee = pair.get("knee")
            if knee is not None:
                out[(k, side, "K")] = _target(knee, w + knee["phase"] + add, scale, 0.0, ramp)
    return out


# ================================================================ metabolism
def gait_activity(brain):
    """sum(amp * freq) over the brain's joint entries (hip + knee per pair, as
    written -- the mirrored right side is the same entry, not a second one)."""
    f = brain["freq"]
    s = 0.0
    for pair in brain["pairs"]:
        for key in ("hip", "knee"):
            j = pair.get(key)
            if j is not None:
                s += abs(j["amp"]) * f
    return s


def metabolic_cost_per_s(mass_kg, brain):
    """cost/s = 0.012*mass + 0.02*sum(amp*f)."""
    return COST_MASS * mass_kg + COST_GAIT * gait_activity(brain)


def _capsule_volume(length, radius):
    return math.pi * radius * radius * length + (4.0 / 3.0) * math.pi * radius ** 3


def mass_of(genome):
    """Total mass at 250 kg/m^3: torso capsule + every limb segment capsule on
    both sides (the head is visual-only and weighs nothing)."""
    body = genome["body"]
    v = _capsule_volume(body["torso"]["length"], body["torso"]["radius"])
    for pair in body["pairs"]:
        for seg in pair["segments"]:
            v += 2.0 * _capsule_volume(seg["length"], seg["radius"])
    return DENSITY * v


def spawn_height(genome):
    """Rest height of the torso centre: legdrop + R + 0.05 (worldgen2's spawn
    z), with legdrop = sum(segment length * cos(splay)) over the LONGEST limb.
    """
    body = genome["body"]
    legdrop = 0.0
    for pair in body["pairs"]:
        c = math.cos(pair.get("splay", 0.0))
        legdrop = max(legdrop, sum(s["length"] for s in pair["segments"]) * c)
    return legdrop + body["torso"]["radius"] + 0.05


def mutate_brain_fallback(brain, rng, sigma=0.08):
    """Gaussian creep on the brain only. The real operator is genome2's
    `mutate_brain` [A]; this exists so the director still runs (and says so)
    when that module is not importable."""
    b = copy.deepcopy(brain)
    b["freq"] = clamp(b["freq"] * (1.0 + rng.gauss(0.0, sigma)), 0.5, 3.0)
    for pair in b["pairs"]:
        for key in ("hip", "knee"):
            j = pair.get(key)
            if j is None:
                continue
            j["amp"] = clamp(j["amp"] + rng.gauss(0.0, sigma), 0.1, 1.0)
            j["bias"] = clamp(j["bias"] + rng.gauss(0.0, sigma), -0.9, 0.9)
            j["phase"] = wrap_angle(j["phase"] + rng.gauss(0.0, 3.0 * sigma))
            tot = abs(j["bias"]) + abs(j["amp"])
            if tot > GAIT_CEIL:
                j["amp"] *= GAIT_CEIL / tot
                j["bias"] *= GAIT_CEIL / tot
    b["steer_gain"] = clamp(b.get("steer_gain", 0.5) + rng.gauss(0.0, sigma), 0.0, 1.0)
    b["heading_offset"] = wrap_angle(b.get("heading_offset", 0.0) + rng.gauss(0.0, sigma))
    b["sense_radius"] = clamp(b.get("sense_radius", 4.0) + rng.gauss(0.0, 4.0 * sigma), 1.5, 7.0)
    b["wander"] = clamp(b.get("wander", 0.4) + rng.gauss(0.0, sigma), 0.0, 1.0)
    return b


# ================================================================== creature
class Creature:
    """One occupant of a pooled body slot. A slot is reused across lifetimes:
    when this creature dies it stays in `Ecology.history` and a child later
    takes the slot as a NEW Creature."""

    __slots__ = ("slot", "genome", "energy", "alive", "age_s", "eaten",
                 "offspring", "born_at", "died_at", "cause")

    def __init__(self, slot, genome, energy, born_at=0.0, alive=True):
        self.slot = slot
        self.genome = genome
        self.energy = float(energy)
        self.alive = bool(alive)
        self.age_s = 0.0
        self.eaten = 0
        self.offspring = 0
        self.born_at = float(born_at)
        self.died_at = None
        self.cause = None

    @property
    def species(self):
        return self.genome["species"]

    @property
    def brain(self):
        return self.genome["brain"]

    def step_energy(self, dt_s, mass_kg):
        """Age and burn energy for dt_s. Returns True when this step killed it
        (energy <= 0); the caller parks the body and books the death."""
        if not self.alive:
            return False
        self.age_s += dt_s
        self.energy -= metabolic_cost_per_s(mass_kg, self.brain) * dt_s
        return self.energy <= 0.0

    def can_reproduce(self):
        return self.alive and self.energy > REPRO_THRESHOLD

    def eat(self):
        self.energy = min(ENERGY_CAP, self.energy + EAT_ENERGY)
        self.eaten += 1


# =================================================================== ecology
class Ecology:
    """Bookkeeping over creature slots and the food pool. No engine access:
    every method that moves something RETURNS the teleport for the director to
    apply, as (slot_or_food_index, [x, y, z]) pairs."""

    def __init__(self, creatures, foods, arena, food_active_max, respawn_range,
                 rng, food_margin=0.6):
        """
        creatures     iterable of Creature (alive or parked); one per slot
        foods         list of [x, y, z] as authored in the world (z < 0 parked)
        arena         floor side S (m); food respawns inside |x|,|y| <= S/2 - margin
        respawn_range [lo, hi] seconds a parked item waits before it may return
        rng           random.Random -- the ONLY source of randomness here
        """
        self.slots = {c.slot: c for c in creatures}
        self.history = list(creatures)
        self.foods = [list(f) for f in foods]
        self.food_timer = [0.0] * len(self.foods)
        self.arena = float(arena)
        self.food_margin = float(food_margin)
        self.food_active_max = int(food_active_max)
        self.respawn_lo, self.respawn_hi = float(respawn_range[0]), float(respawn_range[1])
        self.rng = rng
        self.births = self.deaths = self.eats = self.watchdog_kills = 0
        self.species_births, self.species_deaths = {}, {}
        self.peak_pop = {}
        self._mass = {}
        for c in self.history:
            self._mass[c.slot] = mass_of(c.genome)
        # Stagger authored-parked items so the arena does not fill in one tick.
        for j, f in enumerate(self.foods):
            if f[2] < 0.0:
                self.food_timer[j] = rng.uniform(0.0, self.respawn_hi)
        self._recount()

    # ---------------------------------------------------------------- queries
    def mass(self, slot):
        return self._mass[slot]

    def alive(self):
        return [c for c in self.slots.values() if c.alive]

    def species_ids(self):
        return sorted({c.species for c in self.history})

    def population(self):
        pop = {}
        for c in self.slots.values():
            if c.alive:
                pop[c.species] = pop.get(c.species, 0) + 1
        return pop

    def food_active(self):
        return sum(1 for f in self.foods if f[2] >= 0.0)

    def free_slot(self, species):
        """Lowest free slot whose lineage is `species` (a slot's body plan is
        authored per species, so a child can only take a same-species slot)."""
        for slot in sorted(self.slots):
            c = self.slots[slot]
            if not c.alive and c.species == species:
                return slot
        return None

    def _recount(self):
        for sp, n in self.population().items():
            if n > self.peak_pop.get(sp, 0):
                self.peak_pop[sp] = n

    # ------------------------------------------------------------------ eat
    def eat_check(self, creature, pos):
        """If an active food lies within torso.length/2 + 0.25 (XY) of `pos`,
        eat it: energy up, food parked with a respawn timer. Returns
        (food_index, park_translation) or None. At most one item per call."""
        if not creature.alive:
            return None
        reach = creature.genome["body"]["torso"]["length"] / 2.0 + EAT_MARGIN
        hit = sense(pos, self.foods, reach)
        if hit is None:
            return None
        j = hit[2]
        creature.eat()
        self.eats += 1
        f = self.foods[j]
        f[2] = FOOD_PARK_Z
        self.food_timer[j] = self.rng.uniform(self.respawn_lo, self.respawn_hi)
        return j, list(f)

    # ------------------------------------------------------------ reproduce
    def try_reproduce(self, parent, mutate_brain_fn, now_s):
        """Spend the parent's energy on a child in a free same-species slot.

        mutate_brain_fn(parent_genome, rng, child_id) -> a brain dict, or a
        whole genome carrying one (genome2.mutate_brain returns the latter).
        Returns (child, (dx, dy), yaw) -- the director revives the child's slot
        at parent + (dx, dy) facing `yaw` -- or None (not eligible / no slot).
        """
        if not parent.can_reproduce():
            return None
        slot = self.free_slot(parent.species)
        if slot is None:
            return None
        child_id = "%s_c%d" % (parent.genome["id"], parent.offspring + 1)
        mutated = mutate_brain_fn(parent.genome, self.rng, child_id)
        brain = mutated["brain"] if "brain" in mutated else mutated
        # The BODY is the slot's: the engine built that body at load and a
        # teleport cannot change it. Same-species slots share a body plan, so
        # this equals the parent's body -- but the slot is the ground truth.
        genome = copy.deepcopy(self.slots[slot].genome)
        genome["species"] = parent.species
        genome["brain"] = copy.deepcopy(brain)
        genome["parent"] = parent.genome["id"]
        genome["id"] = child_id
        child = Creature(slot, genome, CHILD_ENERGY, born_at=now_s, alive=True)
        parent.energy -= REPRO_COST
        parent.offspring += 1
        self.slots[slot] = child
        self.history.append(child)
        self._mass[slot] = mass_of(genome)
        self.births += 1
        sp = parent.species
        self.species_births[sp] = self.species_births.get(sp, 0) + 1
        self._recount()
        a = self.rng.uniform(0.0, TAU)
        yaw = self.rng.uniform(-math.pi, math.pi)
        return child, (SPAWN_OFFSET * math.cos(a), SPAWN_OFFSET * math.sin(a)), yaw

    def revive(self, slot, genome, energy, now_s):
        """Put a fresh creature (agent /spawn) into a free slot. The director
        places the body. Not a birth; not counted as one."""
        old = self.slots[slot]
        if old.alive:
            raise ValueError("slot %d is occupied" % slot)
        c = Creature(slot, genome, energy, born_at=now_s, alive=True)
        self.slots[slot] = c
        self.history.append(c)
        self._mass[slot] = mass_of(genome)
        self._recount()
        return c

    # ----------------------------------------------------------------- death
    def kill(self, slot, now_s, cause="starved"):
        """Mark the slot's occupant dead. Returns the park translation
        (60 + 2*slot, 60, 5) for the director to apply, or None if it was
        already dead."""
        c = self.slots[slot]
        if not c.alive:
            return None
        c.alive = False
        c.died_at = now_s
        c.cause = cause
        self.deaths += 1
        self.species_deaths[c.species] = self.species_deaths.get(c.species, 0) + 1
        if cause == "watchdog":
            self.watchdog_kills += 1
        return park_translation(slot)

    def watchdog(self, slot, pos):
        """True when an ALIVE creature's position is non-finite or has left
        the world -- the director then kills it with cause 'watchdog'.
        MuJoCo's instability channel is read by nothing, so this is the only
        detector. Parked slots free-fall in the pit and are never checked."""
        c = self.slots[slot]
        if not c.alive:
            return False
        for v in pos:
            if not math.isfinite(v) or abs(v) > RUNAWAY_LIMIT:
                return True
        return False

    # ------------------------------------------------------------------ food
    def random_arena_point(self):
        h = self.arena / 2.0 - self.food_margin
        return [self.rng.uniform(-h, h), self.rng.uniform(-h, h), FOOD_Z]

    def food_tick(self, dt_s):
        """Advance respawn timers and keep the active count <= food_active_max.
        Returns [(food_index, [x, y, z]), ...] teleports to apply: ripe parked
        items come back at a random arena point; if more are active than
        allowed (an over-authored pool), the surplus is parked."""
        moves = []
        active = 0
        for j, f in enumerate(self.foods):
            if f[2] >= 0.0:
                active += 1
            else:
                self.food_timer[j] -= dt_s
        # surplus -> park (highest indices first, deterministic)
        j = len(self.foods) - 1
        while active > self.food_active_max and j >= 0:
            f = self.foods[j]
            if f[2] >= 0.0:
                f[2] = FOOD_PARK_Z
                self.food_timer[j] = self.rng.uniform(self.respawn_lo, self.respawn_hi)
                moves.append((j, list(f)))
                active -= 1
            j -= 1
        # deficit -> respawn ripe items
        for j, f in enumerate(self.foods):
            if active >= self.food_active_max:
                break
            if f[2] < 0.0 and self.food_timer[j] <= 0.0:
                f[:] = self.random_arena_point()
                self.food_timer[j] = 0.0
                moves.append((j, list(f)))
                active += 1
        return moves

    def place_food(self, x, y):
        """Agent /feed: bring a parked item up at (x, y) (clamped to the
        arena), ignoring its timer. Returns (index, [x, y, z]) or None when no
        item is parked."""
        h = self.arena / 2.0 - self.food_margin
        for j, f in enumerate(self.foods):
            if f[2] < 0.0:
                f[:] = [clamp(float(x), -h, h), clamp(float(y), -h, h), FOOD_Z]
                self.food_timer[j] = 0.0
                return j, list(f)
        return None

    # ------------------------------------------------------------- reporting
    def best_creature(self, species):
        """Alive-or-dead creature of `species` with the most offspring, then
        the most eaten, then the longest life. None if the species never
        existed."""
        best = None
        for c in self.history:
            if c.species != species:
                continue
            key = (c.offspring, c.eaten, c.age_s)
            if best is None or key > best[0]:
                best = (key, c)
        return best[1] if best else None

    def snapshot(self, tick, sim_s, positions):
        """Telemetry dict per the contract. `positions` maps slot -> measured
        [x, y, z] for the creatures the director read this tick; slots not in
        it (parked) report pos null -- never a stale or invented number."""
        per_slot = {}
        for slot in sorted(self.slots):
            c = self.slots[slot]
            p = positions.get(slot)
            per_slot[str(slot)] = {
                "id": c.genome["id"],
                "species": c.species,
                "alive": c.alive,
                "energy": round(c.energy, 3),
                "age_s": round(c.age_s, 3),
                "eaten": c.eaten,
                "offspring": c.offspring,
                "pos": [round(v, 4) for v in p] if p is not None else None,
                "brain": c.brain,
            }
        return {
            "tick": tick,
            "sim_s": round(sim_s, 3),
            "population": self.population(),
            "food_active": self.food_active(),
            "births": self.births,
            "deaths": self.deaths,
            "eats": self.eats,
            "watchdog_kills": self.watchdog_kills,
            "slots": per_slot,
        }

    def epoch_result(self, sim_s):
        """Per-species summary for the epoch driver. `mean_lifespan_s` is over
        every creature that ever lived: age at death, or age so far for the
        survivors (censored at epoch end -- stated so the driver knows a
        long-lived survivor is not being under-counted)."""
        out = {"sim_s": round(sim_s, 3), "births": self.births, "deaths": self.deaths,
               "eats": self.eats, "watchdog_kills": self.watchdog_kills,
               "species": {}}
        for sp in self.species_ids():
            members = [c for c in self.history if c.species == sp]
            # a slot parked since load (alive_at_start false, never revived)
            # never lived and must not drag the mean down
            lifespans = [c.age_s for c in members if c.alive or c.died_at is not None]
            best = self.best_creature(sp)
            out["species"][sp] = {
                "births": self.species_births.get(sp, 0),
                "deaths": self.species_deaths.get(sp, 0),
                "eaten": sum(c.eaten for c in members),
                "peak_pop": self.peak_pop.get(sp, 0),
                "alive_at_end": sum(1 for c in members if c.alive),
                "mean_lifespan_s": round(sum(lifespans) / len(lifespans), 3) if lifespans else 0.0,
                "lifespan_censored": any(c.alive for c in members),
                "best_brain": copy.deepcopy(best.brain) if best else None,
                "best_id": best.genome["id"] if best else None,
                "best_offspring": best.offspring if best else 0,
                "best_eaten": best.eaten if best else 0,
            }
        return out


def park_translation(slot):
    """The crypt: (60 + 2i, 60, 0.6), dropped onto a static slab where the
    body comes to rest. Free-fall parking was tried and rejected: reviving a
    falling body needs setVelocity(), which freezes it for ~2 s (measured)."""
    return [60.0 + 2.0 * slot, 60.0, 0.6]
