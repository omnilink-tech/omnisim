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

"""Unit tests for the pure Metazoa ecology (no engine, no omnisim import).

Run: python -m pytest projects/metazoa/tests/test_ecology.py -q
"""
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from mz import ecology as E  # noqa: E402

DT = 0.008
ARENA = 18.0


# ------------------------------------------------------------------ helpers
def census(reef):
    """Independent recount (never trusts Reef's own checker)."""
    free = sum(1 for c in reef.cells if c.alive and c.organism is None)
    debris = sum(1 for c in reef.cells if not c.alive)
    members = sum(len(o.members()) for o in reef.organisms.values())
    return free, members, debris


def assert_conserved(reef):
    free, members, debris = census(reef)
    assert free + members + debris == reef.n_cells
    seen = [i for o in reef.organisms.values() for i in o.members()]
    assert len(seen) == len(set(seen))
    for o in reef.organisms.values():
        for i in o.members():
            assert reef.cells[i].organism == o.id and reef.cells[i].alive


def kinds(actions):
    return [next(iter(a)) for a in actions]


def only(actions, kind):
    return [a[kind] for a in actions if kind in a]


def far_positions(n, x0=-6.0):
    """Every cell far from every patch (the default pentagon), 0.5 m apart."""
    return {i: (x0 + 0.5 * (i % 6), -7.5 + 0.5 * (i // 6)) for i in range(n)}


def make_reef(n_cells=8, spines=((0, 1, 2, 3),), target=4, seed=1, **kw):
    seeds = [(list(sp), E.default_genome(), E.default_bodyplan(target)) for sp in spines]
    return E.Reef(n_cells, random.Random(seed), arena=ARENA, organisms=seeds, **kw)


def set_charge(reef, frac, cells=None):
    for c in reef.cells if cells is None else (reef.cells[i] for i in cells):
        c.charge_wh = frac * E.CAP_WH


# --------------------------------------------------------------- structure
def test_seed_reef_is_the_plan_population():
    reef = E.seed_reef(random.Random(0))
    assert reef.n_cells == 24
    assert len(reef.organisms) == 6
    assert len(reef.free_cells()) == 12
    assert all(len(o) == 2 for o in reef.organisms.values())
    assert all(c.charge_wh == E.START_WH for c in reef.cells)
    assert_conserved(reef)
    # one weld per seeded junction, one ring per cell, one light per patch
    acts = reef.initial_actions()
    assert len(only(acts, "lock")) == 6
    assert len(only(acts, "ring")) == 24
    assert len(only(acts, "light")) == 5
    assert only(acts, "lock")[0] == (1, "f_tail", 0)


def test_junction_shapes_spine_and_branch():
    reef = make_reef(n_cells=6, spines=((0, 1, 2),))
    o = reef.organisms["org_0"]
    o.bodyplan["branch_rule"] = {"at": 1, "sides": ["L", "R"]}
    acts = reef.recruit("org_0", 3)
    assert acts[0] == {"lock": (3, "f_tail", 1)}
    assert o.branches == {(1, "L"): 3}
    assert (3, "f_tail", 1, "f_left") in o.junctions()
    acts = reef.recruit("org_0", 4)
    assert o.branches == {(1, "L"): 3, (1, "R"): 4}
    # both branch faces filled -> the next recruit goes onto the head's nose
    acts = reef.recruit("org_0", 5)
    assert acts[0] == {"lock": (5, "f_tail", 2)}
    assert o.spine == [0, 1, 2, 5]
    assert len(o) == 6
    assert_conserved(reef)


# ------------------------------------------------------------- conservation
def test_conservation_recruit():
    reef = make_reef(n_cells=6, spines=((0, 1),))
    reef.recruit("org_0", 2)
    assert reef.organisms["org_0"].spine == [0, 1, 2]
    assert census(reef) == (3, 3, 0)
    with pytest.raises(ValueError):
        reef.recruit("org_0", 2)          # already a member
    assert_conserved(reef)


def test_conservation_divide():
    reef = make_reef(n_cells=6, spines=((0, 1, 2, 3),))
    rear, front, acts = reef.divide("org_0")
    assert census(reef) == (2, 4, 0)
    assert len(reef.organisms) == 2
    assert_conserved(reef)


def test_conservation_shed():
    reef = make_reef(n_cells=6, spines=((0, 1, 2, 3),))
    reef.shed("org_0")
    assert census(reef) == (3, 3, 0)
    assert_conserved(reef)


def test_conservation_death_and_recycle():
    reef = make_reef(n_cells=6, spines=((0, 1, 2, 3),))
    set_charge(reef, 1e-7)
    reef.step(DT, far_positions(6))
    assert census(reef) == (0, 0, 6)
    assert_conserved(reef)
    for _ in range(int(E.DEBRIS_RECYCLE_S / DT) + 2):
        reef.step(DT, far_positions(6))
    assert census(reef) == (6, 0, 0)
    assert_conserved(reef)


def test_conservation_checker_catches_tampering():
    reef = make_reef(n_cells=6, spines=((0, 1),))
    reef.cells[4].organism = "org_0"           # claims a body that does not hold it
    with pytest.raises(E.ConservationError):
        reef.step(DT, far_positions(6))
    reef = make_reef(n_cells=6, spines=((0, 1),))
    reef.organisms["org_0"].spine.append(1)    # a cell listed twice
    with pytest.raises(E.ConservationError):
        reef.step(DT, far_positions(6))
    reef = make_reef(n_cells=6, spines=((0, 1),))
    reef.cells.pop()                           # a cell vanished
    with pytest.raises(E.ConservationError):
        reef.step(DT, far_positions(6))


def test_conservation_long_random_run_exercises_every_transition():
    """Random charges, cells clustered under drifting light, auto-docking:
    over 4000 ticks every transition fires and the count never moves."""
    rng = random.Random(7)
    reef = E.seed_reef(rng, n_cells=24, n_organisms=6, dock_reach=0.5,
                       bodyplans=[E.default_bodyplan(2 + k % 3) for k in range(6)])
    reef.set_dim(3.0)                             # bright reef: fast cycles
    for c in reef.cells:
        c.charge_wh = rng.uniform(0.0, E.CAP_WH)
    for k, o in enumerate(reef.organisms.values()):
        if k % 2:
            o.bodyplan["branch_rule"] = {"at": 0, "sides": ["L", "R"]}
    dark = [10, 11] + list(range(16, 24))          # one body + 8 free cells never see light
    for i in dark:
        reef.cells[i].charge_wh = rng.uniform(0.0, 1.5)
    pos = {i: [rng.uniform(-8, 8), rng.uniform(-8, 8)] for i in range(24)}
    total = 0
    for tick in range(8000):
        if tick % 40 == 0:                        # re-cluster every 10 s of sim
            for i in pos:
                if i in dark:
                    pos[i] = far_positions(24)[i]
                    continue
                p = rng.choice(reef.patches).pos if rng.random() < 0.6 else (0.0, 0.0)
                pos[i] = [p[0] + rng.uniform(-0.8, 0.8), p[1] + rng.uniform(-0.8, 0.8)]
        acts = reef.step(0.25, pos)
        total += len(acts)
        assert_conserved(reef)
    assert reef.recruits > 0 and reef.divisions > 0 and reef.deaths > 0
    assert reef.recycles > 0 and reef.sheds > 0
    assert total > 0


# -------------------------------------------------------------- energy
def test_drain_rates_idle_work_and_flip():
    reef = make_reef(n_cells=3, spines=((0, 1),))
    reef.organisms["org_0"].genome.update({"A": 0.8, "omega": 5.0})
    set_charge(reef, 0.5)
    reef.step(1.0, far_positions(3))
    work = 0.4 * 0.8 * 5.0 / (2 * math.pi)
    expect_member = 6.0 - (0.05 + work) * E.TIME_SCALE / 3600.0
    expect_free = 6.0 - (0.05 + 0.2) * E.TIME_SCALE / 3600.0
    assert reef.cells[0].charge_wh == pytest.approx(expect_member, rel=1e-9)
    assert reef.cells[1].charge_wh == pytest.approx(expect_member, rel=1e-9)
    assert reef.cells[2].charge_wh == pytest.approx(expect_free, rel=1e-9)
    # a resting free cell pays idle only
    reef.step(1.0, far_positions(3), moving_free=set())
    assert reef.cells[2].charge_wh == pytest.approx(expect_free - 0.05 * E.TIME_SCALE / 3600.0, rel=1e-9)


def test_equalisation_is_the_mean():
    reef = make_reef(n_cells=5, spines=((0, 1, 2, 3),))
    for i, f in enumerate((0.9, 0.3, 0.6, 0.2)):
        reef.cells[i].charge_wh = f * E.CAP_WH
    reef.cells[4].charge_wh = 0.9 * E.CAP_WH
    o = reef.organisms["org_0"]
    drain = sum(E.wh_per_s(E.IDLE_W + o.work_w(i)) for i in o.spine) * DT
    reef.step(DT, far_positions(5))
    vals = [reef.cells[i].charge_wh for i in range(4)]
    assert max(vals) - min(vals) < 1e-12
    assert vals[0] == pytest.approx((0.9 + 0.3 + 0.6 + 0.2) * E.CAP_WH / 4 - drain / 4, rel=1e-9)
    assert reef.cells[4].charge_wh < 0.9 * E.CAP_WH     # the free cell is not in the pool


def test_light_charges_whole_pool_through_one_member_and_dim_scales():
    def run(dim, lit_members):
        reef = make_reef(n_cells=4, spines=((0, 1, 2),), dim=dim)
        set_charge(reef, 0.5)
        pos = far_positions(4)
        p = reef.patches[0]
        p.ax = p.ay = 0.0                           # hold the disc still at its home
        px, py = p.home
        for i in lit_members:
            pos[i] = (px + 0.3, py)
        o = reef.organisms["org_0"]
        drain = sum(E.wh_per_s(E.IDLE_W + o.work_w(i)) for i in o.spine) * 1.0
        reef.step(1.0, pos)
        pool = sum(reef.cells[i].charge_wh for i in range(3))
        return pool - (18.0 - drain)          # net light gain into the pool

    one = run(1.0, [1])
    assert one == pytest.approx(E.LIGHT_W * E.TIME_SCALE / 3600.0, rel=1e-9)
    assert run(1.0, []) == pytest.approx(0.0, abs=1e-12)
    assert run(0.5, [1]) == pytest.approx(one * 0.5, rel=1e-9)
    assert run(0.0, [1]) == pytest.approx(0.0, abs=1e-12)
    assert run(1.0, [0, 1]) == pytest.approx(2.0 * one, rel=1e-9)
    # every member shares the gain even though only one stood in the light
    reef = make_reef(n_cells=4, spines=((0, 1, 2),))
    set_charge(reef, 0.5)
    pos = far_positions(4)
    pos[2] = reef.patches[0].pos
    reef.step(1.0, pos)
    assert reef.cells[0].charge_wh == pytest.approx(reef.cells[2].charge_wh)


def test_charge_never_exceeds_capacity():
    reef = make_reef(n_cells=2, spines=((0, 1),))
    set_charge(reef, 0.999)
    pos = {0: reef.patches[0].pos, 1: reef.patches[0].pos}
    for _ in range(50):
        reef.step(5.0, pos)
        pos = {0: reef.patches[0].pos, 1: reef.patches[0].pos}
    assert all(c.charge_wh <= E.CAP_WH + 1e-9 for c in reef.cells)


# ------------------------------------------------------------- division
@pytest.mark.parametrize("n,rear,front", [(2, [0], [1]), (4, [0, 1], [2, 3]),
                                          (5, [0, 1], [2, 3, 4])])
def test_division_splits_spine_at_midpoint(n, rear, front):
    reef = make_reef(n_cells=n, spines=(tuple(range(n)),), target=n)
    parent = reef.organisms["org_0"]
    pg, pbp = dict(parent.genome), dict(parent.bodyplan)
    r, f, acts = reef.divide("org_0")
    assert acts == [{"unlock": (front[0], "f_tail", rear[-1])}]
    assert r.spine == rear and f.spine == front
    assert "org_0" not in reef.organisms and parent.cause == "divided"
    assert parent.divisions == 1 and reef.divisions == 1
    for kid in (r, f):
        assert kid.lineage == parent.lineage == "org_0"
        assert kid.parent == "org_0"
        assert kid.generation == 1
        assert kid.genome != pg                 # mutated
        assert kid.genome is not parent.genome
        assert set(kid.genome) == set(pg)
        for k, (lo, hi) in E.GENOME_RANGES.items():
            assert lo - 1e-9 <= kid.genome[k] <= hi + 1e-9
        assert kid.bodyplan["target_length"] >= 2
    assert r.genome != f.genome
    assert parent.genome == pg and parent.bodyplan == pbp   # parent untouched
    assert_conserved(reef)


def test_division_with_local_fallback_operator(caplog):
    reef = make_reef(n_cells=4, spines=((0, 1, 2, 3),), mutate_genome=E.mutate_genome_fallback,
                     mutate_bodyplan=E.mutate_bodyplan_fallback)
    parent = reef.organisms["org_0"]
    pg = dict(parent.genome)
    r, f, _ = reef.divide("org_0")
    assert r.genome != pg and f.genome != pg
    assert E.mutation_source()             # resolves without raising either way


def test_division_reassigns_branches_to_the_half_that_holds_them():
    reef = make_reef(n_cells=6, spines=((0, 1, 2, 3),))
    o = reef.organisms["org_0"]
    o.branches = {(0, "L"): 4, (3, "R"): 5}
    reef.cells[4].organism = reef.cells[5].organism = "org_0"
    r, f, _ = reef.divide("org_0")
    assert r.branches == {(0, "L"): 4}
    assert f.branches == {(1, "R"): 5}
    assert_conserved(reef)


def test_division_needs_two_spine_cells():
    reef = make_reef(n_cells=2, spines=((0,),))
    assert reef.divide("org_0") == (None, None, [])


# ------------------------------------------------------------------ shed
def test_shed_removes_the_tail_first():
    reef = make_reef(n_cells=5, spines=((0, 1, 2, 3),))
    o = reef.organisms["org_0"]
    o.branches = {(0, "L"): 4}
    reef.cells[4].organism = "org_0"
    acts = reef.shed("org_0")
    assert acts[0] == {"unlock": (1, "f_tail", 0)}
    assert {"unlock": (4, "f_tail", 0)} in acts    # the tail's branch leaves with it
    assert o.spine == [1, 2, 3] and o.branches == {}
    assert reef.cells[0].free and reef.cells[4].free
    assert reef.shed("org_0")[0] == {"unlock": (2, "f_tail", 1)}
    assert o.spine == [2, 3]
    assert reef.shed("org_0") and o.spine == [3]
    assert reef.shed("org_0") == []                 # one cell cannot shed
    assert_conserved(reef)


def test_shed_rule_fires_below_10_percent_with_cooldown():
    reef = make_reef(n_cells=4, spines=((0, 1, 2, 3),))
    set_charge(reef, 0.09)
    acts = reef.step(DT, far_positions(4))
    o = reef.organisms["org_0"]
    assert {"unlock": (1, "f_tail", 0)} in acts
    assert o.spine == [1, 2, 3] and o.state == "seek_light"
    acts = reef.step(DT, far_positions(4))
    assert not only(acts, "unlock")                 # cooldown
    reef.t += E.SHED_COOLDOWN_S
    acts = reef.step(DT, far_positions(4))
    assert {"unlock": (2, "f_tail", 1)} in acts


# ---------------------------------------------------------- death / recycle
def test_death_emits_limp_and_unlock_then_recycles_at_50_percent():
    reef = make_reef(n_cells=5, spines=((0, 1, 2),))
    set_charge(reef, 0.5)
    set_charge(reef, 1e-7, cells=[0, 1, 2, 4])
    pos = far_positions(5)
    acts = reef.step(DT, pos)
    assert sorted(only(acts, "limp")) == [0, 1, 2, 4]
    assert sorted(only(acts, "unlock")) == [(1, "f_tail", 0), (2, "f_tail", 1)]
    assert "org_0" not in reef.organisms
    assert reef.history[0].cause == "starved" and reef.history[0].died_at == pytest.approx(DT)
    assert len(reef.history) == 1                   # no transient split bodies
    for i in (0, 1, 2, 4):
        c = reef.cells[i]
        assert not c.alive and c.limp and c.organism is None and c.charge_wh == 0.0
        assert c.debris_since == pytest.approx(DT)
        assert (i, E.RING_DEBRIS) in only(acts, "ring")
    assert reef.cells[3].alive and reef.cells[3].free
    assert reef.deaths == 4
    assert census(reef) == (1, 0, 4)
    # nothing happens for 20 s ...
    t_end = reef.t + E.DEBRIS_RECYCLE_S
    while reef.t + DT < t_end:
        acts = reef.step(DT, pos)
        assert not only(acts, "teleport")
        assert all(reef.cells[i].charge_wh == 0.0 for i in (0, 1, 2, 4))
    # ... then every debris cell comes back on the edge at 50 %
    acts = reef.step(2 * DT, pos)
    tel = only(acts, "teleport")
    assert sorted(t[0] for t in tel) == [0, 1, 2, 4]
    assert sorted(only(acts, "unlimp")) == [0, 1, 2, 4]
    r = ARENA / 2 - E.EDGE_INSET
    for i, x, y, yaw in tel:
        assert max(abs(x), abs(y)) == pytest.approx(r)          # on the ring
        assert abs(E.wrap_angle(yaw - math.atan2(-y, -x))) < 1e-9  # facing inward
        c = reef.cells[i]
        assert c.alive and c.free and not c.limp and c.debris_since is None
        assert c.charge_wh == E.RECYCLE_WH
        assert (i, E.RING_AMBER) in only(acts, "ring")
    assert len({(round(x, 3), round(y, 3)) for _, x, y, _ in tel}) == 4   # spread out
    assert reef.recycles == 4
    assert census(reef) == (5, 0, 0)
    assert_conserved(reef)


def test_kill_interior_cell_splits_the_body():
    reef = make_reef(n_cells=4, spines=((0, 1, 2, 3),))
    acts = reef.kill_cell(1, cause="watchdog")
    assert {"limp": 1} in acts
    assert sorted(only(acts, "unlock")) == [(1, "f_tail", 0), (2, "f_tail", 1)]
    o = reef.organisms["org_0"]
    assert o.spine == [2, 3]
    others = [x for x in reef.organisms.values() if x.id != "org_0"]
    assert len(others) == 1 and others[0].spine == [0] and others[0].lineage == "org_0"
    assert reef.watchdog_kills == 1
    assert reef.watchdog(2, (1e5, 0.0, 0.0)) and not reef.watchdog(1, (1e5, 0.0, 0.0))
    assert_conserved(reef)


# ----------------------------------------------------------------- light
def test_light_drift_stays_in_the_arena_at_the_contract_speed():
    reef = E.seed_reef(random.Random(3))
    half = ARENA / 2
    prev = [p.pos for p in reef.patches]
    for _ in range(20000):
        acts = reef.step(0.05, None)
        lights = only(acts, "light")
        assert [k for k, _, _ in lights] == [0, 1, 2, 3, 4]
        for k, x, y in lights:
            assert abs(x) + E.PATCH_RADIUS <= half and abs(y) + E.PATCH_RADIUS <= half
            d = math.hypot(x - prev[k][0], y - prev[k][1])
            assert d <= E.PATCH_DRIFT_MPS * 0.05 * 1.01     # chord <= arc (sub-stepped)
            prev[k] = (x, y)
    # it really moves (drift, not a dead patch), at the contract speed
    reef2 = E.seed_reef(random.Random(3))
    p = reef2.patches[0]
    a = p.pos
    for _ in range(100):
        p.step(0.1)
    assert math.hypot(p.pos[0] - a[0], p.pos[1] - a[1]) > 0.3
    assert math.hypot(p.pos[0] - a[0], p.pos[1] - a[1]) <= 0.5 + 1e-9


def test_place_light_clamps_into_the_arena_and_keeps_drifting():
    reef = E.seed_reef(random.Random(0))
    xy = reef.place_light(2, 3.0, -2.0)
    assert xy == pytest.approx((3.0, -2.0))
    xy = reef.place_light(2, 100.0, -100.0)
    b = ARENA / 2 - E.PATCH_RADIUS - E.WALL_MARGIN
    assert xy == pytest.approx((b, -b))
    for _ in range(2000):
        reef.step(0.1, None)
    p = reef.patches[2].pos
    assert abs(p[0]) + E.PATCH_RADIUS <= ARENA / 2 and abs(p[1]) + E.PATCH_RADIUS <= ARENA / 2
    assert reef.set_dim(-1.0) == 0.0 and reef.set_dim(0.5) == 0.5


# ---------------------------------------------------------------- states
def test_seek_light_targets_the_nearest_patch():
    reef = make_reef(n_cells=3, spines=((0, 1),))
    set_charge(reef, 0.39)
    pos = far_positions(3)
    acts = reef.step(DT, pos)
    o = reef.organisms["org_0"]
    assert o.state == "seek_light"
    head = pos[1]
    nearest = min(reef.patches, key=lambda p: math.hypot(p.pos[0] - head[0], p.pos[1] - head[1]))
    assert only(acts, "target") == [("org_0", nearest.pos)]


def test_recruit_targets_nearest_free_cell_and_auto_docks_in_reach():
    reef = make_reef(n_cells=5, spines=((0, 1),), dock_reach=0.2)
    set_charge(reef, 0.7)
    pos = {0: (0.0, 0.0), 1: (0.12, 0.0), 2: (3.0, 0.0), 3: (1.0, 0.0), 4: (2.0, 2.0)}
    acts = reef.step(DT, pos)
    o = reef.organisms["org_0"]
    assert o.state == "recruit" and o.recruit_target == 3
    assert only(acts, "target") == [("org_0", (1.0, 0.0))]
    pos[3] = (0.25, 0.0)
    acts = reef.step(DT, pos)
    assert o.state == "docked" and o.spine == [0, 1, 3]
    assert only(acts, "lock") == [(3, "f_tail", 1)]
    assert not only(acts, "target")
    assert reef.recruits == 1 and o.recruited == 1 and reef.cells[3].recruited == 1
    # full at target 4: one more recruit, then the body divides above 80 %
    pos[2] = (0.4, 0.0)
    reef.step(DT, pos)
    assert len(o) == 4 and o.state == "docked"
    set_charge(reef, 0.9, cells=o.spine)
    acts = reef.step(DT, pos)
    assert "org_0" not in reef.organisms and reef.divisions == 1
    assert only(acts, "unlock") == [(3, "f_tail", 1)]
    assert sorted(x.spine for x in reef.organisms.values()) == [[0, 1], [3, 2]]
    assert_conserved(reef)


def test_roam_when_nothing_applies_and_two_bodies_do_not_claim_one_cell():
    reef = make_reef(n_cells=5, spines=((0, 1), (2, 3)))
    set_charge(reef, 0.5)
    acts = reef.step(DT, far_positions(5))
    assert all(o.state == "roam" for o in reef.organisms.values())
    # roaming bodies amble toward the nearest light patch (one target each)
    patch_xy = {tuple(p.pos[:2]) for p in reef.patches}
    roam_targets = only(acts, "target")
    assert len(roam_targets) == len(reef.organisms)
    assert all(tuple(xy) in patch_xy for _oid, xy in roam_targets)
    set_charge(reef, 0.7)
    acts = reef.step(DT, far_positions(5))
    targets = only(acts, "target")
    assert len(targets) == 1                        # only one free cell to claim
    assert sorted(o.recruit_target for o in reef.organisms.values() if o.recruit_target is not None) == [4]


def test_recruit_target_without_supervisor_docking_needs_recruit_call():
    reef = make_reef(n_cells=3, spines=((0, 1),))      # dock_reach None (default)
    set_charge(reef, 0.7)
    pos = {0: (0.0, 0.0), 1: (0.12, 0.0), 2: (0.25, 0.0)}
    acts = reef.step(DT, pos)
    assert not only(acts, "lock") and only(acts, "target") == [("org_0", (0.25, 0.0))]
    assert reef.recruit("org_0", 2) == [{"lock": (2, "f_tail", 1)}]


# ------------------------------------------------------------------ rings
def test_ring_colour_bands_and_change_only_emission():
    assert E.ring_colour(0.6) == E.RING_GREEN
    assert E.ring_colour(0.599) == E.RING_AMBER
    assert E.ring_colour(0.25) == E.RING_AMBER
    assert E.ring_colour(0.249) == E.RING_RED
    assert E.ring_colour(1.0, alive=False) == E.RING_DEBRIS
    reef = make_reef(n_cells=2, spines=((0, 1),))
    reef.initial_actions()
    assert not only(reef.step(DT, None), "ring")    # 50 % stays amber
    set_charge(reef, 0.7)
    assert sorted(only(reef.step(DT, None), "ring")) == [(0, E.RING_GREEN), (1, E.RING_GREEN)]
    assert not only(reef.step(DT, None), "ring")


# --------------------------------------------------------------- reporting
def test_snapshot_reports_measured_positions_or_null():
    reef = make_reef(n_cells=3, spines=((0, 1),))
    reef.step(DT, {0: (1.0, 2.0, 0.03)})
    s = reef.snapshot({0: (1.0, 2.0, 0.03)})
    assert s["n_cells"] == 3 and s["free"] == 1 and s["members"] == 2 and s["debris"] == 0
    assert s["cells"]["0"]["pos"] == [1.0, 2.0] and s["cells"]["1"]["pos"] is None
    assert s["organisms"]["org_0"]["spine"] == [0, 1]
    assert s["organisms"]["org_0"]["charge_frac"] == pytest.approx(E.START_WH / E.CAP_WH, abs=1e-3)
    assert len(s["patches"]) == 5 and s["dim"] == 1.0
    assert s["tick"] == 1 and s["sim_s"] == pytest.approx(DT, abs=1e-3)


def test_epoch_result_scores_per_lineage():
    reef = make_reef(n_cells=8, spines=((0, 1, 2, 3), (4, 5)))
    o1 = reef.organisms["org_1"]
    reef.recruit("org_1", 6)                    # org_1: 1 recruit, length 3
    reef.organisms["org_0"].light_wh = 25.0
    reef.divide("org_0")                        # org_0 lineage: 1 division, two 2-chains
    res = reef.epoch_result()
    assert res["score_kind"] == "evolution_time"
    assert res["n_cells"] == 8 and res["divisions"] == 1 and res["recruits"] == 1
    l0, l1 = res["lineages"]["org_0"], res["lineages"]["org_1"]
    assert l0["divisions"] == 1 and l0["recruited"] == 0 and l0["light_wh"] == 25.0
    assert l0["alive"] == 2 and l0["mean_length"] == 2.0 and l0["max_generation"] == 1
    assert l0["score"] == pytest.approx(10.0 + 0 + 2.5 + 2.0)
    assert l1["divisions"] == 0 and l1["recruited"] == 1 and l1["mean_length"] == 3.0
    assert l1["score"] == pytest.approx(0 + 1 + 0 + 3.0)
    assert l0["best_id"] == "org_0" and l1["best_id"] == "org_1"
    assert l0["best_genome"] == reef.history[0].genome
    # an extinct lineage keeps its earned score minus the length term
    set_charge(reef, 1e-6, cells=o1.members())
    reef.step(DT, far_positions(8))
    l1 = reef.epoch_result()["lineages"]["org_1"]
    assert l1["alive"] == 0 and l1["deaths"] == 1 and l1["mean_length"] == 0.0
    assert l1["score"] == pytest.approx(1.0)
    assert res["mutation_source"]


# ------------------------------------------------------------- misc rules
def test_step_ignores_non_positive_dt_and_genome_helpers_clamp():
    reef = make_reef(n_cells=2, spines=((0, 1),))
    assert reef.step(0.0, None) == [] and reef.tick == 0
    g = E.clamp_genome({"A": 9.0, "omega": -1.0, "branch_phase": 7.0})
    assert g["A"] == 1.2 and g["omega"] == 2.0 and -math.pi < g["branch_phase"] <= math.pi
    bp = E.clamp_bodyplan({"target_length": 99, "dock_rotation_pattern": [5, -1],
                           "branch_rule": {"at": 1, "sides": ["L", "X"]}})
    assert bp["target_length"] == 8 and bp["dock_rotation_pattern"] == [1, 3]
    assert bp["branch_rule"] == {"at": 1, "sides": ["L"]}
    assert E.clamp_bodyplan({"branch_rule": {"sides": ["L"]}})["branch_rule"] == "none"
    rng = random.Random(0)
    for _ in range(50):
        m = E.mutate_bodyplan_fallback(bp, rng)
        assert 2 <= m["target_length"] <= 8
        assert all(0 <= r <= 3 for r in m["dock_rotation_pattern"])


def test_edge_point_walks_the_square_ring():
    r = ARENA / 2 - E.EDGE_INSET
    assert E.edge_point(ARENA, 0.0)[:2] == pytest.approx((r, -r))
    assert E.edge_point(ARENA, 0.25)[:2] == pytest.approx((r, r))
    assert E.edge_point(ARENA, 0.5)[:2] == pytest.approx((-r, r))
    assert E.edge_point(ARENA, 0.75)[:2] == pytest.approx((-r, -r))
    for s in (0.1, 0.37, 0.61, 0.9):
        x, y, _ = E.edge_point(ARENA, s)
        assert max(abs(x), abs(y)) == pytest.approx(r)


def test_mutation_falls_back_to_local_creep_when_organism_module_is_absent(monkeypatch, caplog):
    import logging
    monkeypatch.setitem(sys.modules, "mz.organism", None)     # makes the import raise
    monkeypatch.setattr(E, "_MUTATE_GENOME", None)
    monkeypatch.setattr(E, "_MUTATE_BODYPLAN", None)
    monkeypatch.setattr(E, "MUTATION_SOURCE", "unresolved")
    with caplog.at_level(logging.WARNING, logger="metazoa.ecology"):
        src = E.mutation_source()
    assert src.startswith("local gaussian creep")
    assert any("local gaussian creep" in r.getMessage() for r in caplog.records)
    reef = make_reef(n_cells=4, spines=((0, 1, 2, 3),))
    pg = dict(reef.organisms["org_0"].genome)
    r, f, _ = reef.divide("org_0")
    assert r.genome != pg and f.genome != pg and r.genome != f.genome
    assert reef.epoch_result()["mutation_source"] == src


def test_mutation_prefers_b_operator_when_present():
    pytest.importorskip("mz.organism")
    E._MUTATE_GENOME = None
    E.MUTATION_SOURCE = "unresolved"
    assert E.mutation_source().startswith("mz.organism.")


def test_from_reef_dict_matches_the_driver_shape():
    reef_d = {
        "n_cells": 8, "arena": 12.0,
        "cells": [{"id": i, "charge_wh": 6.0, "parked": i >= 6} for i in range(8)],
        "organisms": [{"id": "L0_e0", "lineage": "L0", "members": [0, 1], "parent": None,
                       "genome": E.default_genome(), "bodyplan": E.default_bodyplan(3)},
                      {"id": "L1_e0", "lineage": "L1", "members": [2, 3], "parent": "L1_x",
                       "genome": E.default_genome(), "bodyplan": E.default_bodyplan(2)}],
        "free": [4, 5], "parked": [6, 7],
    }
    reef = E.Reef.from_reef_dict(reef_d, random.Random(0))
    assert reef.arena == 12.0 and reef.n_cells == 8
    assert census(reef) == (2, 4, 2)
    o = reef.organisms["L1_e0"]
    assert o.lineage == "L1" and o.parent == "L1_x" and o.spine == [2, 3]
    assert reef.lineages() == ["L0", "L1"]
    assert reef.cells[6].debris and reef.cells[6].charge_wh == 0.0
    pos = far_positions(8)
    acts = []
    while reef.t < E.DEBRIS_RECYCLE_S + 1.0:
        acts += reef.step(0.5, pos)
    assert sorted(t[0] for t in only(acts, "teleport")) == [6, 7]
    assert census(reef) == (4, 4, 0)
    assert reef.cells[6].charge_wh == pytest.approx(E.RECYCLE_WH, abs=0.01)   # one tick of drain since
    assert set(reef.epoch_result()["lineages"]) == {"L0", "L1"}
    assert_conserved(reef)
    bad = dict(reef_d, cells=[dict(c, parked=(c["id"] == 1)) for c in reef_d["cells"]])
    with pytest.raises(ValueError):
        E.Reef.from_reef_dict(bad, random.Random(0))
