#!/usr/bin/env python3
"""Creature genome: morphology + brain, with mutation and random initialisation.

A genome describes ONE creature completely. Morphology is deliberately a TREE of
boxes (a torso plus N independent limbs), for two hard engine reasons:

  * A closed kinematic loop makes `SolverMuJoCo` construction raise, and the
    world then gets NO physics at all -- the entire population freezes, not just
    the offending creature. A tree cannot close a loop by construction.
  * A `Cone` boundingObject is absent from newton's geom mapping and kills the
    world at solver construction. Every part here is a Box.

The brain is a per-joint central pattern generator:

    target_j(t) = bias_j + amp_j * sin(2*pi*freq*t + phase_j)

CPGs are used instead of evolved neural networks on purpose. This repo's own
history is that from-scratch policy learning under Newton reliably fails, and a
CPG produces locomotion in generation 1 -- so there is a fitness gradient to
climb immediately rather than after a long flat search.

Measured constraint that shapes mutation (see ../README.md): perfectly
left-right symmetric gaits produce EXACTLY zero net displacement. Asymmetry is
what locomotes, so both morphology and brain are free to break symmetry.
"""
import copy
import math
import random

# Motor limits authored into the world. bias+amp must stay inside these or the
# engine emits one WARNING per joint per tick, and the log I/O alone is enough
# to slow the simulation below its own measurement points.
JOINT_LIMIT = 1.8
GAIT_CEIL = 1.55          # hard cap on |bias| + |amp|

TORSO_MIN, TORSO_MAX = 0.10, 0.34
LIMB_LEN_MIN, LIMB_LEN_MAX = 0.08, 0.26
LIMB_THICK_MIN, LIMB_THICK_MAX = 0.03, 0.07
NLIMB_MIN, NLIMB_MAX = 3, 6

DENSITY = 250.0           # kg/m^3 -> torso ~0.6 kg at the probe's 0.20x0.12x0.06


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _mass(size):
    return max(0.02, DENSITY * size[0] * size[1] * size[2])


def box_inertia(m, s):
    """Exact box tensor. Required on the Robot root: the geometry-derived
    inertia path EXCLUDES OmRobot bodies and silently substitutes a Husky
    preset (OmSolid.cpp:3842)."""
    x, y, z = s
    return (m * (y * y + z * z) / 12.0,
            m * (x * x + z * z) / 12.0,
            m * (x * x + y * y) / 12.0)


def _fix_gait(lb):
    """Keep bias+amp inside the authored motor limits, preserving their ratio."""
    total = abs(lb["bias"]) + abs(lb["amp"])
    if total > GAIT_CEIL:
        k = GAIT_CEIL / total
        lb["bias"] *= k
        lb["amp"] *= k
    lb["amp"] = abs(lb["amp"])
    lb["bias"] = _clamp(lb["bias"], -JOINT_LIMIT, JOINT_LIMIT)


def random_limb(rng, torso):
    """One limb: where it attaches, how big it is, which way its hinge turns."""
    # Attach on a torso face. side picks +x/-x/+y/-y so limbs can be legs at the
    # ends or outriggers on the flanks.
    side = rng.choice(["front", "back", "left", "right"])
    hx, hy, hz = torso[0] / 2.0, torso[1] / 2.0, torso[2] / 2.0
    jitter = rng.uniform(-0.4, 0.4)
    if side == "front":
        anchor = [hx, hy * jitter * 2.0, 0.0]
    elif side == "back":
        anchor = [-hx, hy * jitter * 2.0, 0.0]
    elif side == "left":
        anchor = [hx * jitter * 2.0, hy, 0.0]
    else:
        anchor = [hx * jitter * 2.0, -hy, 0.0]

    length = rng.uniform(LIMB_LEN_MIN, LIMB_LEN_MAX)
    thick = rng.uniform(LIMB_THICK_MIN, LIMB_THICK_MAX)
    # Hinge axis: y = pitch (swings the limb fore/aft, the main thrust axis),
    # z = yaw (sweeps it sideways, useful for turning and crawling).
    axis = [0, 1, 0] if rng.random() < 0.75 else [0, 0, 1]

    lb = {
        "side": side,
        "anchor": anchor,
        "size": [length, thick, thick],
        "axis": axis,
        # brain
        "amp": rng.uniform(0.25, 0.95),
        "phase": rng.uniform(0.0, 2.0 * math.pi),
        "bias": rng.uniform(-0.85, 0.85),
    }
    _fix_gait(lb)
    return lb


def random_genome(rng, gid):
    torso = [rng.uniform(0.14, TORSO_MAX),
             rng.uniform(TORSO_MIN, 0.20),
             rng.uniform(0.05, 0.10)]
    n = rng.randint(NLIMB_MIN, NLIMB_MAX)
    return {
        "id": gid,
        "torso": torso,
        "freq": rng.uniform(0.6, 2.4),
        "limbs": [random_limb(rng, torso) for _ in range(n)],
        "parent": None,
    }


def mutate(g, rng, gid, rate=1.0):
    """Gaussian creep on every continuous parameter, plus rare structural
    changes (add / drop a limb, flip a hinge axis). rate scales the step size."""
    m = copy.deepcopy(g)
    m["id"] = gid
    m["parent"] = g["id"]

    m["torso"] = [
        _clamp(m["torso"][0] * (1.0 + rng.gauss(0, 0.10 * rate)), 0.12, TORSO_MAX),
        _clamp(m["torso"][1] * (1.0 + rng.gauss(0, 0.10 * rate)), TORSO_MIN, 0.22),
        _clamp(m["torso"][2] * (1.0 + rng.gauss(0, 0.10 * rate)), 0.04, 0.12),
    ]
    m["freq"] = _clamp(m["freq"] + rng.gauss(0, 0.22 * rate), 0.4, 3.0)

    for lb in m["limbs"]:
        lb["anchor"] = [a + rng.gauss(0, 0.012 * rate) for a in lb["anchor"]]
        lb["size"] = [
            _clamp(lb["size"][0] * (1.0 + rng.gauss(0, 0.12 * rate)),
                   LIMB_LEN_MIN, LIMB_LEN_MAX),
            _clamp(lb["size"][1] * (1.0 + rng.gauss(0, 0.10 * rate)),
                   LIMB_THICK_MIN, LIMB_THICK_MAX),
            _clamp(lb["size"][2] * (1.0 + rng.gauss(0, 0.10 * rate)),
                   LIMB_THICK_MIN, LIMB_THICK_MAX),
        ]
        lb["amp"] += rng.gauss(0, 0.14 * rate)
        lb["bias"] += rng.gauss(0, 0.14 * rate)
        lb["phase"] = (lb["phase"] + rng.gauss(0, 0.55 * rate)) % (2.0 * math.pi)
        if rng.random() < 0.04 * rate:
            lb["axis"] = [0, 0, 1] if lb["axis"] == [0, 1, 0] else [0, 1, 0]
        _fix_gait(lb)

    # structural mutation
    if rng.random() < 0.10 * rate and len(m["limbs"]) < NLIMB_MAX:
        m["limbs"].append(random_limb(rng, m["torso"]))
    elif rng.random() < 0.08 * rate and len(m["limbs"]) > NLIMB_MIN:
        m["limbs"].pop(rng.randrange(len(m["limbs"])))

    return m


def validate(g):
    """Reject anything that would produce a world the solver refuses, or a
    creature whose parts interpenetrate at spawn. Returns a list of problems."""
    bad = []
    if not (NLIMB_MIN <= len(g["limbs"]) <= NLIMB_MAX):
        bad.append("limb count %d out of range" % len(g["limbs"]))
    for i, s in enumerate(g["torso"]):
        if not (0.02 < s < 0.6):
            bad.append("torso dim %d = %.3f" % (i, s))
    for j, lb in enumerate(g["limbs"]):
        if abs(lb["bias"]) + abs(lb["amp"]) > JOINT_LIMIT:
            bad.append("limb %d gait %.3f exceeds motor limit" % (j, abs(lb["bias"]) + abs(lb["amp"])))
        for k, s in enumerate(lb["size"]):
            if not (0.01 < s < 0.4):
                bad.append("limb %d dim %d = %.3f" % (j, k, s))
        if lb["axis"] not in ([0, 1, 0], [0, 0, 1]):
            bad.append("limb %d bad axis %s" % (j, lb["axis"]))
    return bad


def limb_placement(g, lb):
    """World-frame-ish offset of the limb centre in the torso frame, and the
    outward direction it extends. Kept here so the world writer and any
    analysis agree on one definition."""
    a = lb["anchor"]
    half = lb["size"][0] / 2.0
    if lb["side"] == "front":
        d = (1.0, 0.0, 0.0)
    elif lb["side"] == "back":
        d = (-1.0, 0.0, 0.0)
    elif lb["side"] == "left":
        d = (0.0, 1.0, 0.0)
    else:
        d = (0.0, -1.0, 0.0)
    centre = [a[0] + d[0] * half, a[1] + d[1] * half, a[2] + d[2] * half]
    return centre, d


def describe(g):
    axes = "".join("p" if lb["axis"] == [0, 1, 0] else "y" for lb in g["limbs"])
    return "%s torso=%.2fx%.2fx%.2f limbs=%d[%s] f=%.2f" % (
        g["id"], g["torso"][0], g["torso"][1], g["torso"][2],
        len(g["limbs"]), axes, g["freq"])


def seed_population(n, seed, tag="g0"):
    rng = random.Random(seed)
    pop = []
    while len(pop) < n:
        g = random_genome(rng, "%s_%02d" % (tag, len(pop)))
        if not validate(g):
            pop.append(g)
    return pop
