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

"""Unit tests for the pure ecology module (no engine, no omnisim import).

Run: python -m pytest projects/alife/tests -q
"""
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from alife import ecology as E  # noqa: E402


# ------------------------------------------------------------------ fixtures
def make_genome(species="sp0", gid="sp0_g0_00", pairs=2, segments=2):
    body_pairs, brain_pairs = [], []
    for k in range(pairs):
        segs = [{"length": 0.16, "radius": 0.022}]
        if segments == 2:
            segs.append({"length": 0.14, "radius": 0.018})
        body_pairs.append({"x": 0.7 - 1.4 * k / max(1, pairs - 1) if pairs > 1 else 0.0,
                           "z": 0.0, "segments": segs, "splay": 0.35})
        bp = {"hip": {"amp": 0.6, "bias": 0.1, "phase": 0.0}}
        if segments == 2:
            bp["knee"] = {"amp": 0.4, "bias": -0.5, "phase": 1.2}
        brain_pairs.append(bp)
    return {
        "id": gid, "species": species, "parent": None,
        "body": {"torso": {"length": 0.30, "radius": 0.06},
                 "head": {"radius": 0.055}, "pairs": body_pairs, "hue": 0.62},
        "brain": {"freq": 1.6, "pairs": brain_pairs, "mirror_phase": math.pi,
                  "steer_gain": 0.5, "heading_offset": 0.0,
                  "sense_radius": 4.0, "wander": 0.4},
    }


def make_ecology(n_species=2, slots=3, alive=2, foods=None, active_max=4,
                 respawn=(1.0, 2.0), seed=1):
    creatures = []
    slot = 0
    for s in range(n_species):
        sp = "sp%d" % s
        for m in range(slots):
            g = make_genome(sp, "%s_g0_%02d" % (sp, m))
            creatures.append(E.Creature(slot, g, E.ENERGY_START, alive=(m < alive)))
            slot += 1
    if foods is None:
        foods = [[1.0, 0.0, E.FOOD_Z], [-2.0, 0.5, E.FOOD_Z], [0.0, 0.0, E.FOOD_PARK_Z]]
    return E.Ecology(creatures, foods, arena=14.0, food_active_max=active_max,
                     respawn_range=respawn, rng=random.Random(seed))


def no_mutation(genome, rng, child_id):
    """Stand-in for genome2.mutate_brain(g, rng, gid): returns a genome."""
    m = dict(genome)
    m["id"] = child_id
    return m


# ---------------------------------------------------------------- wrap / yaw
def test_wrap_angle():
    assert E.wrap_angle(0.0) == 0.0
    assert abs(E.wrap_angle(3 * math.pi) - math.pi) < 1e-12
    assert abs(E.wrap_angle(-3 * math.pi) - math.pi) < 1e-12
    assert abs(E.wrap_angle(2 * math.pi + 0.3) - 0.3) < 1e-12
    assert abs(E.wrap_angle(-2 * math.pi - 0.3) + 0.3) < 1e-12
    for a in (-10.0, -1.0, 0.5, 7.0, 100.0):
        w = E.wrap_angle(a)
        assert -math.pi < w <= math.pi
        assert abs(math.sin(w) - math.sin(a)) < 1e-9


def test_yaw_from_orientation_rotation_about_z():
    for yaw in (0.0, 0.7, -2.0, math.pi / 2):
        c, s = math.cos(yaw), math.sin(yaw)
        R9 = [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]     # row-major Rz(yaw)
        assert abs(E.wrap_angle(E.yaw_from_orientation(R9) - yaw)) < 1e-12
        P16 = [c, -s, 0, 1.0, s, c, 0, 2.0, 0, 0, 1, 0.3, 0, 0, 0, 1]
        assert abs(E.wrap_angle(E.yaw_from_pose(P16) - yaw)) < 1e-12


# --------------------------------------------------------------------- sense
def test_sense_picks_nearest_active_within_radius_and_ignores_parked():
    foods = [[3.0, 0.0, E.FOOD_Z],          # 3 m away
             [0.5, 0.0, E.FOOD_PARK_Z],     # nearest but PARKED (z < 0)
             [0.0, 1.0, E.FOOD_Z],          # 1 m away -> the answer
             [0.0, 9.0, E.FOOD_Z]]          # out of radius
    hit = E.sense((0.0, 0.0, 0.3), foods, 4.0)
    assert hit is not None
    bearing, dist, idx = hit
    assert idx == 2
    assert abs(dist - 1.0) < 1e-12
    assert abs(bearing - math.pi / 2) < 1e-12
    assert E.sense((0.0, 0.0, 0.3), foods, 0.9) is None
    assert E.sense((0.0, 0.0, 0.3), [[0.5, 0.0, -3.0]], 10.0) is None


# --------------------------------------------------------------------- steer
def test_steer_food_to_the_left_turns_left():
    # creature at origin facing +x, food at +y -> bearing pi/2, err +pi/2
    bearing, _, _ = E.sense((0, 0, 0), [[0.0, 2.0, E.FOOD_Z]], 4.0)
    err = E.heading_error(bearing, 0.0, 0.0)
    left, right, turn = E.steer(err, 0.5, 0.4, 0.0, 0)
    assert turn > 0
    assert left > right
    assert abs(turn - 1.0) < 1e-12          # pi/2 saturates exactly
    g = min(0.5, E.STEER_GAIN_MAX)                     # the gain is capped
    assert abs(left - (1 + g)) < 1e-12 and abs(right - (1 - g)) < 1e-12


def test_steer_food_to_the_right_turns_right():
    bearing, _, _ = E.sense((0, 0, 0), [[0.0, -2.0, E.FOOD_Z]], 4.0)
    err = E.heading_error(bearing, 0.0, 0.0)
    left, right, turn = E.steer(err, 0.5, 0.4, 0.0, 0)
    assert turn < 0
    assert left < right


def test_steer_straight_ahead_is_nearly_zero():
    bearing, _, _ = E.sense((0, 0, 0), [[3.0, 0.01, E.FOOD_Z]], 4.0)
    err = E.heading_error(bearing, 0.0, 0.0)
    left, right, turn = E.steer(err, 0.5, 0.4, 0.0, 0)
    assert abs(turn) < 0.05
    assert abs(left - right) < 0.05


def test_steer_uses_yaw_and_heading_offset():
    # facing +y already, food at +y -> no turn
    bearing, _, _ = E.sense((0, 0, 0), [[0.0, 2.0, E.FOOD_Z]], 4.0)
    assert abs(E.heading_error(bearing, math.pi / 2, 0.0)) < 1e-12
    # a heading_offset rotates the perceived "forward"
    assert E.heading_error(bearing, 0.0, math.pi / 2) < 1e-12
    # wrap: facing -x (yaw pi), food at +y -> err = -pi/2 (turn right), not +3pi/2
    err = E.heading_error(bearing, math.pi, 0.0)
    assert abs(err + math.pi / 2) < 1e-12


def test_steer_wander_is_deterministic_bounded_and_per_creature():
    a = [E.steer(None, 0.5, 1.0, t * 0.1, 7)[2] for t in range(500)]
    b = [E.steer(None, 0.5, 1.0, t * 0.1, 7)[2] for t in range(500)]
    c = [E.steer(None, 0.5, 1.0, t * 0.1, 8)[2] for t in range(500)]
    assert a == b
    assert a != c
    assert all(-1.0 <= v <= 1.0 for v in a)
    assert max(a) > 0.3 and min(a) < -0.3         # it actually wanders
    # wander 0 -> no turn at all
    assert E.steer(None, 0.5, 0.0, 3.3, 7) == (1.0, 1.0, 0.0)


def test_steer_scales_never_negative():
    left, right, turn = E.steer(math.pi, 1.0, 0.0, 0.0, 0)
    assert left >= 0.0 and right >= 0.0


# ------------------------------------------------------------- joint targets
def test_joint_targets_shape_mirror_and_ceiling():
    g = make_genome(pairs=2, segments=2)
    b = g["brain"]
    out = E.joint_targets(b, 0.0)
    assert set(out) == {(k, s, j) for k in range(2) for s in "LR" for j in "HK"}
    # right = left with mirror_phase added (pi): sin flips sign around bias
    t = 0.123
    out = E.joint_targets(b, t)
    hip = b["pairs"][0]["hip"]
    w = 2 * math.pi * b["freq"] * t + hip["phase"]
    assert abs(out[(0, "L", "H")] - (hip["bias"] + hip["amp"] * math.sin(w))) < 1e-12
    assert abs(out[(0, "R", "H")] - (hip["bias"] + hip["amp"] * math.sin(w + math.pi))) < 1e-12
    # a hard steer scales one side up to 2x; ceiling keeps every target inside
    for tt in range(0, 200):
        out = E.joint_targets(b, tt * 0.017, 0.0, 2.0)
        for v in out.values():
            assert abs(v) <= E.GAIT_CEIL + 1e-9
    # single-segment pairs -> hips only
    g1 = make_genome(pairs=1, segments=1)
    assert set(E.joint_targets(g1["brain"], 0.0)) == {(0, "L", "H"), (0, "R", "H")}
    # dict reuse
    d = {}
    assert E.joint_targets(b, 0.0, out=d) is d


# ---------------------------------------------------------------- mass / cost
def test_mass_of_and_metabolic_cost():
    g = make_genome(pairs=2, segments=2)
    torso = math.pi * 0.06 ** 2 * 0.30 + 4 / 3 * math.pi * 0.06 ** 3
    seg1 = math.pi * 0.022 ** 2 * 0.16 + 4 / 3 * math.pi * 0.022 ** 3
    seg2 = math.pi * 0.018 ** 2 * 0.14 + 4 / 3 * math.pi * 0.018 ** 3
    expect = 250.0 * (torso + 2 * 2 * (seg1 + seg2))     # 2 pairs x 2 sides
    assert abs(E.mass_of(g) - expect) < 1e-9
    cost = E.metabolic_cost_per_s(expect, g["brain"])
    act = 2 * (0.6 * 1.6 + 0.4 * 1.6)
    assert abs(cost - (E.COST_MASS * expect + E.COST_GAIT * act)) < 1e-12
    assert abs(E.spawn_height(g) - (0.30 * math.cos(0.35) + 0.06 + 0.05)) < 1e-12


# ------------------------------------------------------------------- energy
def test_energy_death():
    g = make_genome()
    c = E.Creature(0, g, 1.0)
    m = E.mass_of(g)
    died = False
    steps = 0
    while not died and steps < 100000:
        died = c.step_energy(0.008, m)
        steps += 1
    assert died and c.energy <= 0.0 and steps > 1
    assert abs(c.age_s - steps * 0.008) < 1e-6
    # a dead creature neither ages nor burns
    c.alive = False
    assert c.step_energy(1.0, m) is False
    # eat caps at 200
    c2 = E.Creature(1, g, 190.0)
    c2.eat()
    assert c2.energy == E.ENERGY_CAP and c2.eaten == 1
    assert not E.Creature(2, g, 140.0).can_reproduce()
    assert E.Creature(2, g, 140.01).can_reproduce()


# ------------------------------------------------------------------ eating
def test_eat_check_parks_food_and_arms_timer():
    eco = make_ecology()
    c = eco.slots[0]
    reach = 0.30 / 2 + 0.25
    # too far
    assert eco.eat_check(c, (1.0 + reach + 0.01, 0.0, 0.3)) is None
    res = eco.eat_check(c, (1.0 + reach - 0.01, 0.0, 0.3))
    assert res is not None
    j, park = res
    assert j == 0 and park[2] == E.FOOD_PARK_Z
    assert eco.foods[0][2] == E.FOOD_PARK_Z
    assert 1.0 <= eco.food_timer[0] <= 2.0
    assert c.eaten == 1 and c.energy == E.ENERGY_START + E.EAT_ENERGY
    assert eco.eats == 1
    # parked food cannot be eaten again
    assert eco.eat_check(c, (1.0, 0.0, 0.3)) is None


# ------------------------------------------------------------ reproduction
def test_reproduction_consumes_free_same_species_slot_and_refuses_when_none():
    eco = make_ecology(n_species=2, slots=3, alive=2)
    parent = eco.slots[0]                     # sp0, slots 0,1 alive, 2 free
    parent.energy = 150.0
    res = eco.try_reproduce(parent, no_mutation, now_s=10.0)
    assert res is not None
    child, (dx, dy), yaw = res
    assert child.slot == 2 and child.species == "sp0"
    assert abs(math.hypot(dx, dy) - E.SPAWN_OFFSET) < 1e-12
    assert -math.pi <= yaw <= math.pi
    assert child.energy == E.CHILD_ENERGY and child.alive
    assert child.genome["parent"] == parent.genome["id"]
    assert child.genome["id"] != parent.genome["id"]
    assert parent.energy == 150.0 - E.REPRO_COST
    assert parent.offspring == 1 and eco.births == 1
    assert eco.population() == {"sp0": 3, "sp1": 2}
    assert eco.peak_pop["sp0"] == 3
    # no free sp0 slot left -> refused even with energy (sp1 has a free slot 5)
    parent.energy = 190.0
    assert eco.free_slot("sp0") is None
    assert eco.free_slot("sp1") == 5
    assert eco.try_reproduce(parent, no_mutation, now_s=11.0) is None
    assert parent.energy == 190.0 and eco.births == 1
    # not eligible below the threshold
    parent.energy = 100.0
    eco.kill(2, 12.0)
    assert eco.try_reproduce(parent, no_mutation, now_s=12.0) is None


def test_kill_parks_and_frees_the_slot_for_a_new_child():
    eco = make_ecology(n_species=1, slots=2, alive=2)
    assert eco.kill(1, 5.0) == E.park_translation(1)
    assert eco.kill(1, 5.0) is None           # already dead
    assert eco.deaths == 1 and not eco.slots[1].alive
    old = eco.slots[1]
    p = eco.slots[0]
    p.energy = 150.0
    child, _, _ = eco.try_reproduce(p, no_mutation, 6.0)
    assert child.slot == 1 and eco.slots[1] is child and old in eco.history


def test_watchdog_flags_only_alive_runaways():
    eco = make_ecology(n_species=1, slots=2, alive=1)
    assert not eco.watchdog(0, [1.0, 2.0, 0.3])
    assert eco.watchdog(0, [1.0, float("nan"), 0.3])
    assert eco.watchdog(0, [1.0, 2.0, -20000.0])
    assert not eco.watchdog(1, [float("inf"), 0, 0])      # parked: exempt
    eco.kill(0, 1.0, cause="watchdog")
    assert eco.watchdog_kills == 1 and eco.deaths == 1


# --------------------------------------------------------------------- food
def test_food_respawn_respects_active_max():
    foods = [[0, 0, E.FOOD_PARK_Z] for _ in range(6)]
    eco = make_ecology(foods=foods, active_max=3, respawn=(0.5, 0.5))
    for j in range(6):
        eco.food_timer[j] = 0.0                # all ripe now
    moves = eco.food_tick(0.008)
    assert len(moves) == 3 and eco.food_active() == 3
    for j, p in moves:
        assert p[2] == E.FOOD_Z and abs(p[0]) <= 7.0 and abs(p[1]) <= 7.0
        assert eco.foods[j] == p
    assert eco.food_tick(0.008) == []          # full
    # eat one -> parked with a 0.5 s timer -> respawns only after it elapses
    # (the three still-parked items get a long timer so only the eaten one
    # can come back)
    for j in range(6):
        if eco.foods[j][2] < 0.0:
            eco.food_timer[j] = 100.0
    c = eco.slots[0]
    j, _ = eco.eat_check(c, eco.foods[moves[0][0]])
    assert eco.food_active() == 2
    assert eco.food_tick(0.1) == []
    assert eco.food_tick(0.45) != [] and eco.food_active() == 3
    # over-authored pool: surplus is parked
    over = [[float(k), 0.0, E.FOOD_Z] for k in range(5)]
    eco2 = make_ecology(foods=over, active_max=2)
    moves = eco2.food_tick(0.008)
    assert len(moves) == 3 and eco2.food_active() == 2
    assert all(p[2] == E.FOOD_PARK_Z for _, p in moves)


def test_place_food_uses_a_parked_item_and_clamps_to_arena():
    eco = make_ecology()                        # index 2 parked
    j, p = eco.place_food(100.0, -100.0)
    assert j == 2 and p[2] == E.FOOD_Z and abs(p[0]) <= 7.0 and abs(p[1]) <= 7.0
    assert eco.place_food(0, 0) is None         # nothing parked now


# ------------------------------------------------------------- epoch result
def test_epoch_result_best_brain_ordering_and_counts():
    eco = make_ecology(n_species=1, slots=4, alive=3)
    a, b, c = eco.slots[0], eco.slots[1], eco.slots[2]
    # b eats the most; a reproduces once (fewer eats) -> a wins on offspring
    b.eaten = 5
    a.eaten = 1
    a.energy = 150.0
    child, _, _ = eco.try_reproduce(a, no_mutation, 20.0)
    a.age_s, b.age_s, c.age_s, child.age_s = 30.0, 30.0, 10.0, 10.0
    eco.kill(c.slot, 10.0)
    res = eco.epoch_result(40.0)
    sp = res["species"]["sp0"]
    assert sp["best_id"] == a.genome["id"]
    assert sp["best_offspring"] == 1 and sp["best_eaten"] == 1
    assert sp["best_brain"] == a.brain and sp["best_brain"] is not a.brain
    assert sp["births"] == 1 and sp["deaths"] == 1 and sp["eaten"] == 6
    assert sp["peak_pop"] == 4 and sp["alive_at_end"] == 3
    assert abs(sp["mean_lifespan_s"] - (30 + 30 + 10 + 10) / 4) < 1e-9
    assert sp["lifespan_censored"] is True
    # with no reproduction, the most eaten wins
    eco2 = make_ecology(n_species=1, slots=2, alive=2)
    eco2.slots[1].eaten = 3
    assert eco2.epoch_result(1.0)["species"]["sp0"]["best_id"] == eco2.slots[1].genome["id"]
    # a parked-since-load slot never lived: not in the lifespan mean
    eco3 = make_ecology(n_species=1, slots=2, alive=1)
    eco3.slots[0].age_s = 8.0
    assert eco3.epoch_result(8.0)["species"]["sp0"]["mean_lifespan_s"] == 8.0


def test_snapshot_reports_null_for_unmeasured_positions():
    eco = make_ecology(n_species=1, slots=2, alive=1)
    snap = eco.snapshot(250, 2.0, {0: [0.1, 0.2, 0.3]})
    assert snap["population"] == {"sp0": 1} and snap["food_active"] == 2
    assert snap["slots"]["0"]["pos"] == [0.1, 0.2, 0.3]
    assert snap["slots"]["1"]["pos"] is None and snap["slots"]["1"]["alive"] is False


def test_mutate_brain_fallback_stays_in_range():
    g = make_genome()
    rng = random.Random(3)
    for _ in range(200):
        b = E.mutate_brain_fallback(g["brain"], rng)
        assert 0.5 <= b["freq"] <= 3.0
        for pair in b["pairs"]:
            for j in (pair["hip"], pair["knee"]):
                assert abs(j["bias"]) + abs(j["amp"]) <= E.GAIT_CEIL + 1e-9
                assert 0.0 < j["amp"] <= 1.0
    assert g["brain"]["freq"] == 1.6                  # input untouched
