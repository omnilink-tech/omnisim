#!/usr/bin/env python3
"""Ecology for Metazoa: charge, light, recruit / divide / shed / death / recycle.

PURE Python on purpose -- no `omnisim` import, no engine, no I/O -- so every
rule the supervisor applies per tick is unit-testable in milliseconds
(`tests/test_ecology.py`). The supervisor (`controllers/metazoa_world/`, D)
owns the round trips: it reads cell poses, hands them to `Reef.step()`, and
executes the ACTIONS this module hands back. Nothing in this file knows what
a DEF is.

Contract: `projects/metazoa/DESIGN.md` ("Ecology (C)"). The constants below
are the numbers written there; change them there first.

Action shapes (the whole supervisor-facing vocabulary; every action is a
one-key dict so the supervisor can `for a in actions: k, v = next(iter(a.items()))`):

    {"lock":     (i, face, j)}   write cell i's Connector `face`.isLocked TRUE;
                                 j is the partner cell (its face is implied:
                                 f_nose for a spine junction, f_left / f_right
                                 for a branch -- see Reef.junctions()).
    {"unlock":   (i, face, j)}   the same tuple, isLocked FALSE.
    {"limp":     i}              motor maxTorque -> 0 (dead cell).
    {"unlimp":   i}              motor maxTorque -> back to 0.6 (recycled cell).
    {"teleport": (i, x, y, yaw)} move cell i (the supervisor picks z: the
                                 rest height of the tail block, 0.03).
    {"ring":     (i, (r, g, b))} write CELL_<i>_RING_APP.emissiveColor.
    {"target":   (org_id, (x, y))} the organism's steering goal this tick
                                 (a light patch or a free cell to recruit).
    {"light":    (k, x, y)}      move light patch k's disc (every tick).

`limp`/`unlimp` and `light` are extensions of the DESIGN.md list: a recycled
cell must get its torque back, and the supervisor has to move the drifting
patches somewhere.

Units and time
--------------
Charge is in Wh. Power is in W on the cell's real battery (12 Wh); each
simulated second books TIME_SCALE seconds of battery time
(Wh -= W * dt_s * TIME_SCALE / 3600) so charge matters in minutes of demo.
`t` everywhere is SIMULATED seconds. Positions are XY metres, ENU, arena
centred at the origin.

Cell frame (DESIGN.md): the tail block is the Robot root, the nose block is
at +x. A chain is nose-to-tail along +x: the rear cell's f_nose meets the
front cell's f_tail. `Organism.spine` is ordered TAIL -> HEAD: spine[0] is
the rearmost cell (its f_tail is free, it is the one shed), spine[-1] is the
head (its f_nose is free; a recruit docks its f_tail onto it, so recruits are
APPENDED). Junction k joins spine[k] (rear) and spine[k+1] (front), and the
active locking side is the front cell's f_tail: lock = (spine[k+1], "f_tail",
spine[k]). A branch at (k, side) is a cell whose f_tail meets spine[k]'s
f_left / f_right: lock = (branch_cell, "f_tail", spine[k]).
"""
import copy
import logging
import math

log = logging.getLogger("metazoa.ecology")

TAU = 2.0 * math.pi

# --- contract numbers (DESIGN.md, "Ecology") ---------------------------------
CAP_WH = 12.0                 # battery capacity per cell
START_WH = 5.4                # 45 % at load: one patch visit away from recruiting
RECYCLE_WH = 5.4              # a recycled cell comes back at 45 %
IDLE_W = 0.05                 # every living cell, always
WORK_W = 0.4                  # x A * omega / (2 pi)  (gait work per cell)
FLIP_WORK_W = 0.2             # lone-cell flip work term (idle is added on top)
LIGHT_W = 4.0                 # per lit cell, into its organism's pool (2.0 barely beat gait cost: charge sat at 50 % for a whole epoch)
PATCH_RADIUS = 1.2
N_PATCHES = 5
PATCH_DRIFT_MPS = 0.05        # constant speed along the Lissajous
TIME_SCALE = 30.0             # battery seconds per simulated second (first live reef: at 20 nothing changed state in 120 s)
DEBRIS_RECYCLE_S = 20.0       # simulated seconds a dead cell lies as debris
SEEK_LIGHT_FRAC = 0.40
RECRUIT_FRAC = 0.55
DIVIDE_FRAC = 0.80
SHED_FRAC = 0.10
SHED_COOLDOWN_S = 10.0        # at most one autotomy per organism per this
EDGE_INSET = 0.6              # the recycling edge ring, inside the walls (D)
WALL_MARGIN = 0.3             # patch centre keeps this clear of the walls
SCORE_DIVISION = 10.0
SCORE_LIGHT_DIV = 10.0
RUNAWAY_LIMIT = 1e4           # watchdog: |coordinate| beyond this = diverged

# Charge ring colours (emissiveColor), by state-of-charge band.
RING_GREEN = (0.0, 1.0, 0.6)
RING_AMBER = (1.0, 0.65, 0.0)
RING_RED = (1.0, 0.08, 0.08)
RING_DEBRIS = (0.15, 0.15, 0.15)
RING_GREEN_FRAC = 0.60
RING_AMBER_FRAC = 0.25

# Genome / body-plan ranges (DESIGN.md "The organism"). The FALLBACK operator
# below creeps inside these; B's `mz.organism.mutate` is preferred when it is
# importable, and MUTATION_SOURCE says which one ran.
GENOME_RANGES = {
    "A": (0.3, 1.2), "omega": (2.0, 8.0), "dphi": (0.6, 2.4),
    "bias_pitch": (-0.6, 0.6), "bias_yaw": (-0.6, 0.6),
    "branch_phase": (-math.pi, math.pi), "branch_scale": (0.0, 1.0),   # wrapped, not clamped
    "steer_gain": (0.0, 0.6),
}
TARGET_LENGTH_RANGE = (2, 8)

_MUTATE_GENOME = None          # resolved lazily by _resolve_mutation()
_MUTATE_BODYPLAN = None
MUTATION_SOURCE = "unresolved"


class ConservationError(AssertionError):
    """Raised by Reef when the cell census does not add up. An explicit
    exception class (not a bare `assert`) so `python -O` cannot silence it."""


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    w = a - TAU * math.floor((a + math.pi) / TAU)
    return w + TAU if w <= -math.pi else w


def wh_per_s(watts):
    """Wh drained/charged per simulated second by `watts`, time-scaled."""
    return watts * TIME_SCALE / 3600.0


def ring_colour(frac, alive=True):
    if not alive:
        return RING_DEBRIS
    if frac >= RING_GREEN_FRAC:
        return RING_GREEN
    if frac >= RING_AMBER_FRAC:
        return RING_AMBER
    return RING_RED


def _xy(p):
    return float(p[0]), float(p[1])


def _pos_of(positions, i):
    """positions is a dict idx -> (x, y[, z]) or a sequence indexed by cell;
    None when the supervisor did not read that cell this tick."""
    if positions is None:
        return None
    if isinstance(positions, dict):
        p = positions.get(i)
    else:
        p = positions[i] if 0 <= i < len(positions) else None
    return None if p is None else _xy(p)


# ============================================================ genome / plan
def default_genome():
    return {"A": 0.8, "omega": 5.0, "dphi": 1.57, "bias_pitch": 0.0,
            "bias_yaw": 0.0, "branch_phase": 0.0, "branch_scale": 0.5,
            "steer_gain": 0.3}


def default_bodyplan(target_length=4):
    return {"target_length": int(target_length),
            "dock_rotation_pattern": [0, 1], "branch_rule": "none"}


def clamp_genome(g):
    out = default_genome()
    for k in out:
        if g and k in g:
            out[k] = float(g[k])
    for k, (lo, hi) in GENOME_RANGES.items():
        out[k] = wrap_angle(out[k]) if k == "branch_phase" else clamp(out[k], lo, hi)
    return out


def clamp_bodyplan(bp):
    out = default_bodyplan()
    if bp:
        out.update({k: v for k, v in bp.items() if k in out})
    lo, hi = TARGET_LENGTH_RANGE
    out["target_length"] = int(clamp(int(out["target_length"]), lo, hi))
    pat = [int(r) % 4 for r in (out.get("dock_rotation_pattern") or [0])]
    out["dock_rotation_pattern"] = pat or [0]
    br = out.get("branch_rule")
    if not (isinstance(br, dict) and "at" in br):
        out["branch_rule"] = "none"
    else:
        sides = [s for s in br.get("sides", ["L", "R"]) if s in ("L", "R")]
        out["branch_rule"] = {"at": int(br["at"]), "sides": sides or ["L", "R"]}
    return out


def mutate_genome_fallback(genome, rng, sigma=0.1):
    """Gaussian creep relative to each range width, clamped. Always a NEW
    dict. The real operator is B's `mz.organism.mutate`."""
    g = clamp_genome(genome)
    for k, (lo, hi) in GENOME_RANGES.items():
        v = g[k] + rng.gauss(0.0, sigma * (hi - lo))
        g[k] = wrap_angle(v) if k == "branch_phase" else clamp(v, lo, hi)
    return g


def mutate_bodyplan_fallback(bodyplan, rng):
    """target_length +-1 (p 0.25), one rotation re-drawn (p 0.2), branch rule
    toggled (p 0.1). Always a NEW dict."""
    bp = clamp_bodyplan(bodyplan)
    lo, hi = TARGET_LENGTH_RANGE
    if rng.random() < 0.25:
        bp["target_length"] = int(clamp(bp["target_length"] + rng.choice((-1, 1)), lo, hi))
    if rng.random() < 0.2:
        pat = list(bp["dock_rotation_pattern"])
        pat[rng.randrange(len(pat))] = rng.randrange(4)
        bp["dock_rotation_pattern"] = pat
    if rng.random() < 0.1:
        if bp["branch_rule"] == "none":
            bp["branch_rule"] = {"at": rng.randrange(1, max(2, bp["target_length"])),
                                 "sides": rng.choice((["L"], ["R"], ["L", "R"]))}
        else:
            bp["branch_rule"] = "none"
    return bp


def _resolve_mutation():
    """B's `mz.organism` (written in parallel) if importable, else the local
    creep -- and say so once in the log."""
    global _MUTATE_GENOME, _MUTATE_BODYPLAN, MUTATION_SOURCE
    if _MUTATE_GENOME is not None:
        return
    try:
        import importlib
        _org = importlib.import_module("mz.organism")     # lazy on purpose (B, in parallel)
        fn = getattr(_org, "mutate", None) or getattr(_org, "mutate_genome", None)
        if not callable(fn):
            raise ImportError("mz.organism has no mutate()")
        _MUTATE_GENOME = fn
        bfn = getattr(_org, "mutate_bodyplan", None)
        _MUTATE_BODYPLAN = bfn if callable(bfn) else mutate_bodyplan_fallback
        MUTATION_SOURCE = "mz.organism.%s" % fn.__name__ + (
            "" if callable(bfn) else " (+ local bodyplan creep)")
    except ImportError as e:
        _MUTATE_GENOME = mutate_genome_fallback
        _MUTATE_BODYPLAN = mutate_bodyplan_fallback
        MUTATION_SOURCE = "local gaussian creep (mz.organism not importable: %s)" % e
        log.warning("metazoa ecology: %s", MUTATION_SOURCE)


def mutation_source():
    _resolve_mutation()
    return MUTATION_SOURCE


# ================================================================ light
class LightPatch:
    """One drifting light disc. Drifts at a CONSTANT speed along a Lissajous
    figure around `home`, using an arc-length parametrisation (u advances by
    speed*dt/|dP/du|), and is clamped so the whole disc stays inside the
    arena -- the amplitude is sized from the home point so the clamp is a
    guard, not the mechanism."""

    __slots__ = ("k", "home", "ax", "ay", "wx", "wy", "px", "py", "u", "pos",
                 "bound", "radius")

    def __init__(self, k, home, arena, radius=PATCH_RADIUS, phase=0.0):
        self.k = int(k)
        self.radius = float(radius)
        self.bound = arena / 2.0 - radius - WALL_MARGIN
        self.wx, self.wy = 2.0, 3.0           # frequency ratio 2:3 (never closes early)
        self.px, self.py = phase, phase * 1.7 + 0.4
        self.u = 0.0
        self.set_home(home)

    def set_home(self, home):
        x, y = _xy(home)
        b = self.bound
        self.home = (clamp(x, -b, b), clamp(y, -b, b))
        # Largest amplitude that keeps the figure inside the bound, up to 2.5 m.
        self.ax = clamp(b - abs(self.home[0]), 0.0, 2.5)
        self.ay = clamp(b - abs(self.home[1]), 0.0, 2.5)
        self.pos = self._point(self.u)

    def _point(self, u):
        x = self.home[0] + self.ax * math.sin(self.wx * u + self.px)
        y = self.home[1] + self.ay * math.sin(self.wy * u + self.py)
        b = self.bound
        return (clamp(x, -b, b), clamp(y, -b, b))

    def _du_speed(self, u):
        dx = self.ax * self.wx * math.cos(self.wx * u + self.px)
        dy = self.ay * self.wy * math.cos(self.wy * u + self.py)
        return math.hypot(dx, dy)

    def step(self, dt_s, speed=PATCH_DRIFT_MPS):
        """Advance `speed * dt_s` metres of arc along the figure, in
        sub-steps of at most 0.02 in u so the chord matches the arc to ~1e-3
        even where the parametric speed changes fast. At a cusp (|dP/du|
        below 1e-3 of the peak) the disc pauses rather than jumping."""
        remaining = speed * dt_s
        floor = 1e-3 * (self.ax * self.wx + self.ay * self.wy)
        guard = 0
        while remaining > 1e-12 and guard < 200:
            guard += 1
            sp = self._du_speed(self.u)
            if sp <= floor or sp < 1e-12:
                if floor > 0.0:          # a cusp: step past it; zero amplitude: stay
                    self.u += 0.02
                break
            du = min(remaining / sp, 0.02)
            sp = max(self._du_speed(self.u + 0.5 * du), 0.5 * sp)   # midpoint rule
            du = min(remaining / sp, 0.02)
            self.u += du
            remaining -= du * sp
        self.pos = self._point(self.u)
        return self.pos

    def contains(self, xy):
        return math.hypot(xy[0] - self.pos[0], xy[1] - self.pos[1]) <= self.radius


def default_patch_homes(arena, n=N_PATCHES):
    """A pentagon (or n-gon) at 0.55 of the usable half-width."""
    b = arena / 2.0 - PATCH_RADIUS - WALL_MARGIN
    r = 0.55 * b
    return [(r * math.cos(TAU * k / n), r * math.sin(TAU * k / n)) for k in range(n)]


def edge_point(arena, s):
    """Point on the recycling edge (a square ring EDGE_INSET inside the walls)
    at perimeter fraction s in [0, 1), plus the inward-facing yaw."""
    r = arena / 2.0 - EDGE_INSET
    s = s % 1.0
    side, f = divmod(s * 4.0, 1.0)
    side = int(side)
    if side == 0:
        x, y = r, -r + 2.0 * r * f
    elif side == 1:
        x, y = r - 2.0 * r * f, r
    elif side == 2:
        x, y = -r, r - 2.0 * r * f
    else:
        x, y = -r + 2.0 * r * f, -r
    return x, y, math.atan2(-y, -x)


# ================================================================== cell
class Cell:
    """One robot. Three states: free (alive, organism None), member (alive,
    organism set), debris (not alive, debris_since set). The index `i` is the
    CELL_<i> DEF and never changes -- cells are conserved."""

    __slots__ = ("i", "charge_wh", "alive", "organism", "debris_since", "limp",
                 "ring", "recruited", "deaths", "light_wh")

    def __init__(self, i, charge_wh=START_WH, organism=None):
        self.i = int(i)
        self.charge_wh = float(charge_wh)
        self.alive = True
        self.organism = organism
        self.debris_since = None
        self.limp = False
        self.ring = None            # last colour band emitted
        self.recruited = 0          # times this cell joined an organism
        self.deaths = 0
        self.light_wh = 0.0

    @property
    def frac(self):
        return self.charge_wh / CAP_WH

    @property
    def free(self):
        return self.alive and self.organism is None

    @property
    def debris(self):
        return not self.alive


# ============================================================== organism
class Organism:
    __slots__ = ("id", "spine", "branches", "genome", "bodyplan", "lineage",
                 "parent", "born_at", "died_at", "cause", "state", "target",
                 "recruit_target", "divisions", "recruited", "sheds",
                 "light_wh", "last_shed_at", "generation")

    def __init__(self, oid, spine, genome, bodyplan, lineage, born_at=0.0,
                 parent=None, branches=None, generation=0):
        self.id = str(oid)
        self.spine = list(spine)
        self.branches = dict(branches or {})     # (spine_index, "L"|"R") -> cell
        self.genome = clamp_genome(genome)
        self.bodyplan = clamp_bodyplan(bodyplan)
        self.lineage = str(lineage)
        self.parent = parent
        self.born_at = float(born_at)
        self.died_at = None
        self.cause = None
        self.state = "roam"
        self.target = None
        self.recruit_target = None
        self.divisions = 0
        self.recruited = 0
        self.sheds = 0
        self.light_wh = 0.0
        self.last_shed_at = -1e9
        self.generation = int(generation)

    def members(self):
        """Every cell index: spine tail->head, then branches in key order."""
        return list(self.spine) + [self.branches[k] for k in sorted(self.branches)]

    def __len__(self):
        return len(self.spine) + len(self.branches)

    @property
    def head(self):
        return self.spine[-1]

    @property
    def tail(self):
        return self.spine[0]

    @property
    def target_length(self):
        return self.bodyplan["target_length"]

    def work_w(self, cell):
        """Gait work term for one member: 0.4*A*omega/(2 pi), scaled by
        branch_scale on a branch cell."""
        g = self.genome
        w = WORK_W * g["A"] * g["omega"] / TAU
        if cell not in self.spine:
            w *= g["branch_scale"]
        return w

    def open_branch_slot(self):
        """(k, side) of the first empty branch face the body plan asks for,
        or None."""
        br = self.bodyplan["branch_rule"]
        if br == "none":
            return None
        k = br["at"]
        if k < 0 or k >= len(self.spine):
            return None
        for side in br["sides"]:
            if (k, side) not in self.branches:
                return k, side
        return None

    def junctions(self):
        """[(active_cell, active_face, partner_cell, partner_face), ...] for
        every weld in the body."""
        out = []
        for k in range(len(self.spine) - 1):
            out.append((self.spine[k + 1], "f_tail", self.spine[k], "f_nose"))
        for (k, side), c in sorted(self.branches.items()):
            out.append((c, "f_tail", self.spine[k], "f_left" if side == "L" else "f_right"))
        return out


# =================================================================== reef
class Reef:
    """Bookkeeping over a CONSERVED set of cells, the organisms they form, and
    the light patches. No engine access: `step()` returns the actions the
    supervisor must execute.

    cell_positions (to step/snapshot): dict idx -> (x, y[, z]) or a list
    indexed by cell, as MEASURED by the supervisor this tick. A cell absent
    from it is treated as unlit and invisible to recruitment -- never as
    being at a stale or invented position.
    """

    def __init__(self, n_cells, rng, arena=18.0, organisms=(), n_patches=N_PATCHES,
                 patch_homes=None, dim=1.0, dock_reach=None, mutate_genome=None,
                 mutate_bodyplan=None, t0=0.0):
        """
        n_cells     the conserved population
        rng         random.Random -- the ONLY source of randomness here
        organisms   iterable of (spine_cells, genome, bodyplan) seeds, or
                    dicts {"spine", "genome", "bodyplan", "branches"?, "id"?}
        dock_reach  if set (m), step() auto-recruits a free cell that close
                    (XY) to the head; None (default) leaves docking to the
                    supervisor, which calls recruit() once the faces mate.
        """
        self.n_cells = int(n_cells)
        self.rng = rng
        self.arena = float(arena)
        self.dim = max(0.0, float(dim))
        self.dock_reach = dock_reach
        self._mutate_genome = mutate_genome
        self._mutate_bodyplan = mutate_bodyplan
        self.t = float(t0)
        self.tick = 0
        self.cells = [Cell(i) for i in range(self.n_cells)]
        self.organisms = {}
        self.history = []
        self._next_org = 0
        self._edge_s = 0.0
        self.divisions = self.recruits = self.sheds = self.deaths = self.recycles = 0
        self.watchdog_kills = 0
        self.light_wh = 0.0
        self.peak_length = 0
        homes = list(patch_homes) if patch_homes is not None else default_patch_homes(self.arena, n_patches)
        self.patches = [LightPatch(k, h, self.arena, phase=TAU * k / max(1, len(homes)))
                        for k, h in enumerate(homes)]
        for seed in organisms:
            if isinstance(seed, dict):
                spine = seed.get("spine", seed.get("members"))
                self._add_organism(spine, seed.get("genome"), seed.get("bodyplan"),
                                   lineage=seed.get("lineage"), parent=seed.get("parent"),
                                   branches=seed.get("branches"), oid=seed.get("id"))
            else:
                spine, genome, bodyplan = seed
                self._add_organism(spine, genome, bodyplan)
        self._check_conservation()

    @classmethod
    def from_reef_dict(cls, reef, rng, **kw):
        """Build from the driver's `reef.json` (metazoa.py build_reef): cells
        [{id, charge_wh, parked, ...}], organisms [{id, lineage, members,
        genome, bodyplan, parent}]. A PARKED cell (on the crypt, charge 0)
        enters as DEBRIS at t0, so it joins the reef through the recycling
        edge after DEBRIS_RECYCLE_S at 50 % -- the count is conserved from the
        first tick and nothing is spawned."""
        n = int(reef["n_cells"])
        self = cls(n, rng, arena=float(reef.get("arena", kw.pop("arena", 18.0))),
                   organisms=reef.get("organisms", ()), **kw)
        for c in reef.get("cells", ()):
            cell = self.cells[int(c["id"])]
            if c.get("parked"):
                if cell.organism is not None:
                    raise ValueError("parked cell %d is an organism member" % cell.i)
                cell.alive = False
                cell.limp = True
                cell.charge_wh = 0.0
                cell.debris_since = self.t
            elif "charge_wh" in c:
                cell.charge_wh = clamp(float(c["charge_wh"]), 0.0, CAP_WH)
        self._check_conservation()
        return self

    # ------------------------------------------------------------- queries
    def cell(self, i):
        return self.cells[i]

    def free_cells(self):
        return [c for c in self.cells if c.free]

    def debris_cells(self):
        return [c for c in self.cells if c.debris]

    def member_cells(self):
        return [c for c in self.cells if c.alive and c.organism is not None]

    def organism_of(self, i):
        oid = self.cells[i].organism
        return self.organisms.get(oid) if oid is not None else None

    def lineages(self):
        return sorted({o.lineage for o in self.history})

    def patch_positions(self):
        return [p.pos for p in self.patches]

    def junctions(self, oid=None):
        if oid is not None:
            return self.organisms[oid].junctions()
        out = []
        for o in self.organisms.values():
            out.extend(o.junctions())
        return out

    def pool_frac(self, org):
        n = len(org)
        return sum(self.cells[i].charge_wh for i in org.members()) / (n * CAP_WH) if n else 0.0

    def _check_conservation(self):
        """Every cell is in exactly one of {free, member of exactly one living
        organism, debris}, and the total is n_cells. Raises ConservationError."""
        seen = {}
        for o in self.organisms.values():
            for i in o.members():
                if i in seen:
                    raise ConservationError("cell %d is in organisms %s and %s" % (i, seen[i], o.id))
                seen[i] = o.id
                c = self.cells[i]
                if not c.alive or c.organism != o.id:
                    raise ConservationError("cell %d listed in %s but alive=%s organism=%s"
                                            % (i, o.id, c.alive, c.organism))
        free = debris = 0
        for c in self.cells:
            if c.i in seen:
                continue
            if c.alive and c.organism is None:
                free += 1
            elif not c.alive:
                debris += 1
            else:
                raise ConservationError("cell %d claims organism %s, which does not hold it"
                                        % (c.i, c.organism))
        total = free + len(seen) + debris
        if total != self.n_cells or len(self.cells) != self.n_cells:
            raise ConservationError("census %d free + %d members + %d debris = %d != %d"
                                    % (free, len(seen), debris, total, self.n_cells))

    # ------------------------------------------------------ organism admin
    def _new_id(self):
        oid = "org_%d" % self._next_org
        self._next_org += 1
        return oid

    def _add_organism(self, spine, genome, bodyplan, lineage=None, parent=None,
                      branches=None, oid=None, generation=0, born_at=None):
        oid = oid or self._new_id()
        if oid in self.organisms:
            raise ValueError("organism id %s already exists" % oid)
        o = Organism(oid, spine, genome or default_genome(), bodyplan or default_bodyplan(),
                     lineage or oid, born_at=self.t if born_at is None else born_at,
                     parent=parent, branches=branches, generation=generation)
        for i in o.members():
            c = self.cells[i]
            if not c.alive:
                raise ValueError("cell %d is debris" % i)
            if c.organism is not None:
                raise ValueError("cell %d already belongs to %s" % (i, c.organism))
            c.organism = oid
        self.organisms[oid] = o
        self.history.append(o)
        self.peak_length = max(self.peak_length, len(o))
        return o

    def _retire(self, o, cause):
        for i in o.members():
            if self.cells[i].organism == o.id:
                self.cells[i].organism = None
        o.died_at = self.t
        o.cause = cause
        o.state = "dead"
        o.target = None
        del self.organisms[o.id]

    def _ring_action(self, c, out, force=False):
        col = ring_colour(c.frac, c.alive)
        if force or col != c.ring:
            c.ring = col
            out.append({"ring": (c.i, col)})

    def initial_actions(self):
        """Everything the supervisor applies once after placement: every
        seeded weld, every ring, every patch position."""
        out = []
        for o in self.organisms.values():
            for a, face, b, _pf in o.junctions():
                out.append({"lock": (a, face, b)})
        for c in self.cells:
            self._ring_action(c, out, force=True)
        for p in self.patches:
            out.append({"light": (p.k, p.pos[0], p.pos[1])})
        return out

    # --------------------------------------------------------- mutation
    def _mutate(self, genome, bodyplan):
        _resolve_mutation()
        mg = self._mutate_genome or _MUTATE_GENOME
        mb = self._mutate_bodyplan or _MUTATE_BODYPLAN
        g = mg(copy.deepcopy(genome), self.rng)
        if isinstance(g, dict) and "genome" in g and "A" not in g:
            g = g["genome"]
        bp = mb(copy.deepcopy(bodyplan), self.rng)
        return clamp_genome(g), clamp_bodyplan(bp)

    # ------------------------------------------------------------- energy
    def _energy_tick(self, dt_s, positions, moving_free):
        """Drain every living cell, feed lit cells into their organism's pool,
        equalise each organism. Returns the set of cells that hit 0."""
        dead = set()
        lit = [p.pos for p in self.patches]
        rate = self.dim * LIGHT_W

        def is_lit(i):
            xy = _pos_of(positions, i)
            if xy is None:
                return False
            for (px, py) in lit:
                if math.hypot(xy[0] - px, xy[1] - py) <= PATCH_RADIUS:
                    return True
            return False

        # free cells
        for c in self.cells:
            if not c.free:
                continue
            moving = True if moving_free is None else (c.i in moving_free)
            c.charge_wh -= wh_per_s(IDLE_W + (FLIP_WORK_W if moving else 0.0)) * dt_s
            if is_lit(c.i):
                gain = min(wh_per_s(rate) * dt_s, max(0.0, CAP_WH - c.charge_wh))
                c.charge_wh += gain
                c.light_wh += gain
                self.light_wh += gain
            if c.charge_wh <= 0.0:
                c.charge_wh = 0.0
                dead.add(c.i)
        # organisms: drain per member, pool the light, equalise (mean)
        for o in self.organisms.values():
            members = o.members()
            n = len(members)
            pool = 0.0
            for i in members:
                c = self.cells[i]
                c.charge_wh -= wh_per_s(IDLE_W + o.work_w(i)) * dt_s
                pool += c.charge_wh
            gain = 0.0
            for i in members:
                if is_lit(i):
                    gain += wh_per_s(rate) * dt_s
            gain = min(gain, max(0.0, n * CAP_WH - pool))
            pool += gain
            o.light_wh += gain
            self.light_wh += gain
            per = max(0.0, pool / n)
            for i in members:
                self.cells[i].charge_wh = per
            if per <= 0.0:
                dead.update(members)
        return dead

    # ------------------------------------------------------------ transitions
    def _kill_cell(self, i, out, cause="starved"):
        """A cell at 0: limp, unlocked from its body, debris."""
        c = self.cells[i]
        if not c.alive:
            return
        o = self.organism_of(i)
        if o is not None:
            self._detach(o, i, out, cause)
        c.alive = False
        c.limp = True
        c.charge_wh = 0.0
        c.debris_since = self.t
        c.deaths += 1
        self.deaths += 1
        out.append({"limp": i})
        self._ring_action(c, out)

    def _detach(self, o, i, out, cause):
        """Remove cell i from organism o, emitting the unlocks. A tail or head
        cell just leaves; a branch leaves; an INTERIOR spine cell splits the
        body -- the front part keeps the id, the rear part becomes a new
        organism of the same genome. A body left with no cells retires."""
        for a, face, b, _pf in o.junctions():
            if a == i or b == i:
                out.append({"unlock": (a, face, b)})
        self.cells[i].organism = None
        if i in o.spine:
            k = o.spine.index(i)
            rear, front = o.spine[:k], o.spine[k + 1:]
            rear_br = {kk: c for kk, c in o.branches.items() if kk[0] < k}
            front_br = {(kk[0] - k - 1, kk[1]): c for kk, c in o.branches.items() if kk[0] > k}
            for kk, c in o.branches.items():
                if kk[0] == k:          # branches hanging off the dead cell go free
                    self.cells[c].organism = None
                    self._ring_action(self.cells[c], out)
            if front:
                o.spine, o.branches = front, front_br
                if rear:
                    for c in list(rear) + list(rear_br.values()):
                        self.cells[c].organism = None
                    self._add_organism(rear, o.genome, o.bodyplan, lineage=o.lineage, parent=o.id,
                                       branches=rear_br, generation=o.generation)
            elif rear:
                o.spine, o.branches = rear, rear_br
            else:
                o.spine, o.branches = [], {}
        else:
            for kk, c in list(o.branches.items()):
                if c == i:
                    del o.branches[kk]
        if len(o) == 0:
            self._retire(o, cause)

    def _kill_cells(self, dead, out, cause):
        """Kill a set of cells at once. An organism whose EVERY member is in
        the set (the normal case -- equalised charge hits 0 together) is
        retired whole: one unlock per junction, one limp per cell, no
        transient split organisms in the history."""
        dead = set(dead)
        for oid in sorted(self.organisms):
            o = self.organisms[oid]
            members = o.members()
            if not members or not all(i in dead for i in members):
                continue
            for a, face, b, _pf in o.junctions():
                out.append({"unlock": (a, face, b)})
            self._retire(o, cause)
        for i in sorted(dead):
            self._kill_cell(i, out, cause)

    def kill_cell(self, i, cause="watchdog"):
        """Supervisor-facing: kill cell i now (runaway, agent). Returns the
        actions. Its body loses the cell as `_detach` describes."""
        out = []
        if cause == "watchdog":
            self.watchdog_kills += 1
        self._kill_cell(i, out, cause)
        self._check_conservation()
        return out

    def watchdog(self, i, pos):
        """True when a living cell's measured position is non-finite or has
        left the world."""
        c = self.cells[i]
        if not c.alive or pos is None:
            return False
        return any((not math.isfinite(v)) or abs(v) > RUNAWAY_LIMIT for v in pos)

    def _recycle_tick(self, out):
        for c in self.cells:
            if c.alive or c.debris_since is None:
                continue
            if self.t - c.debris_since < DEBRIS_RECYCLE_S:
                continue
            x, y, yaw = edge_point(self.arena, self._edge_s)
            self._edge_s = (self._edge_s + 0.6180339887498949) % 1.0
            c.alive = True
            c.limp = False
            c.organism = None
            c.debris_since = None
            c.charge_wh = RECYCLE_WH
            self.recycles += 1
            out.append({"teleport": (c.i, x, y, yaw)})
            out.append({"unlimp": c.i})
            self._ring_action(c, out, force=True)

    def recruit(self, oid, i):
        """Dock free cell i onto organism oid at the next open face (a body-plan
        branch face if one is open, else the head's nose). Returns the actions
        (lock + ring), or raises ValueError when the rule forbids it."""
        o = self.organisms.get(oid)
        if o is None:
            raise ValueError("no organism %s" % oid)
        c = self.cells[i]
        if not c.free:
            raise ValueError("cell %d is not free" % i)
        out = []
        slot = o.open_branch_slot()
        if slot is not None:
            k, side = slot
            o.branches[(k, side)] = i
            out.append({"lock": (i, "f_tail", o.spine[k])})
        else:
            partner = o.head
            o.spine.append(i)
            out.append({"lock": (i, "f_tail", partner)})
        c.organism = o.id
        c.recruited += 1
        o.recruited += 1
        o.recruit_target = None
        self.recruits += 1
        self.peak_length = max(self.peak_length, len(o))
        self._check_conservation()
        return out

    def shed(self, oid):
        """Autotomy: the TAIL spine cell (spine[0]) goes free with its share of
        charge. A one-cell organism cannot shed. Returns the actions."""
        o = self.organisms[oid]
        if len(o.spine) < 2:
            return []
        out = []
        tail = o.spine[0]
        out.append({"unlock": (o.spine[1], "f_tail", tail)})
        for (k, side), c in list(o.branches.items()):
            if k == 0:                   # a branch on the tail cell leaves with it
                out.append({"unlock": (c, "f_tail", tail)})
                del o.branches[(k, side)]
                self.cells[c].organism = None
        o.spine.pop(0)
        o.branches = {(k - 1, side): c for (k, side), c in o.branches.items()}
        self.cells[tail].organism = None
        o.sheds += 1
        o.last_shed_at = self.t
        self.sheds += 1
        self._check_conservation()
        return out

    def divide(self, oid):
        """Split at the spine midpoint: rear half spine[:n//2], front half
        spine[n//2:]. Both children get a mutated genome + body plan and the
        parent's lineage; the parent retires (cause 'divided'). Returns
        (rear_child, front_child, actions), or (None, None, []) when the body
        has fewer than two spine cells."""
        o = self.organisms[oid]
        n = len(o.spine)
        if n < 2:
            return None, None, []
        k = n // 2
        out = [{"unlock": (o.spine[k], "f_tail", o.spine[k - 1])}]
        rear, front = o.spine[:k], o.spine[k:]
        rear_br = {kk: c for kk, c in o.branches.items() if kk[0] < k}
        front_br = {(kk[0] - k, kk[1]): c for kk, c in o.branches.items() if kk[0] >= k}
        o.divisions += 1
        self.divisions += 1
        self._retire(o, "divided")
        kids = []
        for spine, br in ((rear, rear_br), (front, front_br)):
            g, bp = self._mutate(o.genome, o.bodyplan)
            kids.append(self._add_organism(spine, g, bp, lineage=o.lineage, parent=o.id,
                                           branches=br, generation=o.generation + 1))
        self._check_conservation()
        return kids[0], kids[1], out

    # --------------------------------------------------------------- step
    def _nearest(self, xy, candidates, positions):
        best, best_d = None, None
        for i in candidates:
            p = _pos_of(positions, i)
            if p is None:
                continue
            d = math.hypot(p[0] - xy[0], p[1] - xy[1])
            if best_d is None or d < best_d:
                best, best_d = i, d
        return best, best_d

    def _decide(self, o, positions, out, claimed):
        """One organism's state for this tick; emits its target."""
        frac = self.pool_frac(o)
        head_xy = _pos_of(positions, o.head)
        o.target = None
        o.recruit_target = None
        if frac < SHED_FRAC and len(o.spine) >= 2 and self.t - o.last_shed_at >= SHED_COOLDOWN_S:
            o.state = "shed"
            out.extend(self.shed(o.id))
        if frac < SEEK_LIGHT_FRAC:
            o.state = "seek_light"
            if head_xy is not None and self.patches:
                p = min(self.patches, key=lambda q: math.hypot(q.pos[0] - head_xy[0], q.pos[1] - head_xy[1]))
                o.target = (p.pos[0], p.pos[1])
        elif len(o) >= o.target_length and frac > DIVIDE_FRAC and len(o.spine) >= 2:
            o.state = "divide"
            _rear, _front, acts = self.divide(o.id)
            out.extend(acts)
            return
        elif frac > RECRUIT_FRAC and len(o) < o.target_length:
            o.state = "recruit"
            if head_xy is not None:
                free = [c.i for c in self.cells if c.free and c.i not in claimed]
                j, d = self._nearest(head_xy, free, positions)
                if j is not None:
                    claimed.add(j)
                    o.recruit_target = j
                    o.target = _pos_of(positions, j)
                    if self.dock_reach is not None and d <= self.dock_reach:
                        out.extend(self.recruit(o.id, j))
                        o.state = "docked"
                        o.target = None
        else:
            # roam = amble toward the nearest patch: light is what every other
            # transition needs, so a body with nothing better to do collects it
            o.state = "roam"
            if self.patches:
                o.target = self._nearest_patch_xy(o, positions)
        if o.target is not None:
            out.append({"target": (o.id, o.target)})

    def _nearest_patch_xy(self, o, positions):
        hx, hy = _pos_of(positions, o.head) or (0.0, 0.0)
        best, bd = None, float("inf")
        for p in self.patches:
            px, py = p.pos[0], p.pos[1]
            d = (px - hx) ** 2 + (py - hy) ** 2
            if d < bd:
                best, bd = (px, py), d
        return best

    def step(self, dt_s, cell_positions=None, moving_free=None):
        """Advance the ecology by dt_s simulated seconds. Returns the action
        list (see the module docstring for every shape). `moving_free` is the
        set of free cells the supervisor is currently flipping (they pay the
        flip work term); None means all of them are."""
        if dt_s <= 0.0:
            return []
        out = []
        self.t += dt_s
        self.tick += 1
        for p in self.patches:
            x, y = p.step(dt_s)
            out.append({"light": (p.k, x, y)})
        dead = self._energy_tick(dt_s, cell_positions, moving_free)
        self._kill_cells(dead, out, "starved")
        self._recycle_tick(out)
        claimed = set()
        for oid in sorted(self.organisms):
            o = self.organisms.get(oid)
            if o is None:            # retired earlier this tick
                continue
            self._decide(o, cell_positions, out, claimed)
        for c in self.cells:
            self._ring_action(c, out)
        self._check_conservation()
        return out

    # ------------------------------------------------------------ agent verbs
    def place_light(self, k, x, y):
        """/light: move patch k's home (the drift continues around it, inside
        the arena). Returns the clamped (x, y) it now sits at."""
        p = self.patches[int(k)]
        p.u = 0.0
        p.px = p.py = 0.0            # sin(0) = 0: the disc sits AT the new home
        p.set_home((x, y))
        return p.pos

    def set_dim(self, factor):
        self.dim = max(0.0, float(factor))
        return self.dim

    # ------------------------------------------------------------- reporting
    def best_organism(self, lineage):
        best = None
        for o in self.history:
            if o.lineage != lineage:
                continue
            key = (o.divisions, o.recruited, o.light_wh, len(o))
            if best is None or key > best[0]:
                best = (key, o)
        return best[1] if best else None

    def lineage_scores(self):
        """DESIGN.md: divisions*10 + cells recruited + light collected/10 +
        mean organism length (over the lineage's organisms ALIVE at scoring
        time; an extinct lineage scores 0 there)."""
        per = {}
        for o in self.history:
            rec = per.setdefault(o.lineage, {
                "score": 0.0, "divisions": 0, "recruited": 0, "sheds": 0,
                "light_wh": 0.0, "organisms": 0, "alive": 0, "deaths": 0,
                "lengths": [], "max_generation": 0})
            rec["divisions"] += o.divisions
            rec["recruited"] += o.recruited
            rec["sheds"] += o.sheds
            rec["light_wh"] += o.light_wh
            rec["organisms"] += 1
            rec["max_generation"] = max(rec["max_generation"], o.generation)
            if o.died_at is None:
                rec["alive"] += 1
                rec["lengths"].append(len(o))
            elif o.cause != "divided":
                rec["deaths"] += 1
        for lin, rec in per.items():
            ls = rec.pop("lengths")
            rec["mean_length"] = round(sum(ls) / len(ls), 3) if ls else 0.0
            rec["score"] = round(SCORE_DIVISION * rec["divisions"] + rec["recruited"]
                                 + rec["light_wh"] / SCORE_LIGHT_DIV + rec["mean_length"], 3)
            rec["light_wh"] = round(rec["light_wh"], 3)
            b = self.best_organism(lin)
            rec["best_id"] = b.id if b else None
            rec["best_genome"] = copy.deepcopy(b.genome) if b else None
            rec["best_bodyplan"] = copy.deepcopy(b.bodyplan) if b else None
        return per

    def snapshot(self, cell_positions=None):
        """Telemetry dict (the /census body). Positions are the MEASURED ones
        the supervisor passed; a cell it did not read reports pos null."""
        cells = {}
        for c in self.cells:
            p = _pos_of(cell_positions, c.i)
            cells[str(c.i)] = {
                "alive": c.alive, "organism": c.organism, "charge_wh": round(c.charge_wh, 3),
                "frac": round(c.frac, 3), "limp": c.limp,
                "debris_since": None if c.debris_since is None else round(c.debris_since, 3),
                "pos": [round(p[0], 4), round(p[1], 4)] if p is not None else None,
                "ring": c.ring,
            }
        orgs = {}
        for oid in sorted(self.organisms):
            o = self.organisms[oid]
            orgs[oid] = {
                "spine": list(o.spine),
                "branches": {"%d%s" % k: v for k, v in sorted(o.branches.items())},
                "length": len(o), "genome": copy.deepcopy(o.genome),
                "bodyplan": copy.deepcopy(o.bodyplan), "lineage": o.lineage,
                "parent": o.parent, "generation": o.generation,
                "state": o.state, "target": list(o.target) if o.target else None,
                "recruit_target": o.recruit_target,
                "charge_frac": round(self.pool_frac(o), 3),
                "divisions": o.divisions, "recruited": o.recruited, "sheds": o.sheds,
                "light_wh": round(o.light_wh, 3), "born_at": round(o.born_at, 3),
                "age_s": round(self.t - o.born_at, 3),
            }
        return {
            "tick": self.tick, "sim_s": round(self.t, 3), "n_cells": self.n_cells,
            "free": len(self.free_cells()), "members": len(self.member_cells()),
            "debris": len(self.debris_cells()), "organisms_alive": len(self.organisms),
            "divisions": self.divisions, "recruits": self.recruits, "sheds": self.sheds,
            "deaths": self.deaths, "recycles": self.recycles,
            "watchdog_kills": self.watchdog_kills,
            "light_wh": round(self.light_wh, 3), "dim": self.dim,
            "patches": [[round(p.pos[0], 4), round(p.pos[1], 4)] for p in self.patches],
            "cells": cells, "organisms": orgs,
        }

    def epoch_result(self):
        """Per-lineage summary for the epoch driver. `score_kind` is
        'evolution_time': these scores were earned under drifting light and
        a changing population, so they are comparable only to a deterministic
        re-score of the same genomes (the alife lesson) -- never across
        epochs."""
        return {
            "sim_s": round(self.t, 3), "ticks": self.tick, "n_cells": self.n_cells,
            "divisions": self.divisions, "recruits": self.recruits, "sheds": self.sheds,
            "deaths": self.deaths, "recycles": self.recycles,
            "watchdog_kills": self.watchdog_kills, "peak_length": self.peak_length,
            "organisms_alive": len(self.organisms), "light_wh": round(self.light_wh, 3),
            "dim": self.dim, "lineages": self.lineage_scores(),
            "score_kind": "evolution_time", "mutation_source": mutation_source(),
        }


# ================================================================ seeding
def seed_reef(rng, n_cells=24, n_organisms=6, arena=18.0, genomes=None, bodyplans=None, **kw):
    """The PLAN.md reef: n_organisms two-cell organisms on cells 0..2n-1
    (tail = 2k, head = 2k+1), the rest free. `genomes`/`bodyplans` per
    organism, else random genomes and the default plan."""
    seeds = []
    for k in range(n_organisms):
        g = genomes[k] if genomes else random_genome(rng)
        bp = bodyplans[k] if bodyplans else default_bodyplan()
        seeds.append(([2 * k, 2 * k + 1], g, bp))
    return Reef(n_cells, rng, arena=arena, organisms=seeds, **kw)


def random_genome(rng):
    return {k: rng.uniform(lo, hi) for k, (lo, hi) in GENOME_RANGES.items()}
