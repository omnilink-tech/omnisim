"""Unit tests for the pure RoboLife rules (no engine, no omnisim import).

Run: python -m pytest projects/robolife/tests/test_energy.py -q
"""
import json
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from rl import energy as E  # noqa: E402


# ------------------------------------------------------------------ fixtures
def make_fleet(n_robots=6, alive=4, modules=None, arena=24.0, seed=1,
               pads=((6.0, 0.0), (-3.0, 5.2), (-3.0, -5.2)), bay=(0.0, 0.0)):
    robots = [E.Robot(i, E.default_genome(), "L%d" % i if i < alive else None,
                      "r%d" % i, alive=(i < alive)) for i in range(n_robots)]
    if modules is None:
        modules = []
        for j in range(8):
            modules.append({"id": j, "type": E.MODULE_TYPES[j % 4],
                            "loose": j < 6, "pos": [1.0 + j, 0.0, 0.2]})
    cfg = {"arena": arena, "pads": [list(p) for p in pads], "bay": list(bay)}
    return E.Fleet(robots, modules, cfg, random.Random(seed))


def run_for(fleet, slot, seconds, v=0.0, w=0.0, pos=(10.0, 10.0, 0.2), dt=0.008):
    """Advance one robot's energy for `seconds` at constant speed. Returns
    the sim time at which it died, or None."""
    t, n = 0.0, int(round(seconds / dt))
    for _ in range(n):
        t += dt
        if fleet.energy_tick(slot, dt, v, w, pos):
            fleet.kill(slot, t)
            return t
    return None


# ------------------------------------------------------------------ capacity
def test_capacity_grows_with_battery_modules():
    assert E.capacity_wh(0) == 200.0
    assert E.capacity_wh(1) == 300.0
    assert E.capacity_wh(2) == 400.0


# --------------------------------------------------------------------- drain
def test_drain_grows_with_mass():
    bare = E.drain_w(E.HUSKY_MASS_KG, 1.0, 6.0, 0, 0)
    loaded = E.drain_w(E.HUSKY_MASS_KG + 11.0, 1.0, 6.0, 0, 0)
    assert loaded > bare
    assert loaded - bare == pytest.approx(0.9 * 11.0 * 1.0 * 0.02)


def test_drain_grows_with_speed():
    slow = E.drain_w(E.HUSKY_MASS_KG, 0.4, E.wheel_omega_mean(0.4, 0.0), 0, 0)
    fast = E.drain_w(E.HUSKY_MASS_KG, 1.2, E.wheel_omega_mean(1.2, 0.0), 0, 0)
    assert fast > slow
    idle = E.drain_w(E.HUSKY_MASS_KG, 0.0, 0.0, 0, 0)
    assert idle == pytest.approx(2.0)


def test_drain_formula_terms():
    # 2 idle + 0.5/battery + 0.9*m*|v|*0.02 + 40*|w|/10 - 6/solar
    w = E.drain_w(100.0, 1.0, 10.0, 2, 1)
    assert w == pytest.approx(2.0 + 1.0 + 0.9 * 100.0 * 1.0 * 0.02 + 40.0 - 6.0)
    # speed sign never matters
    assert E.drain_w(100.0, -1.0, -10.0, 0, 0) == E.drain_w(100.0, 1.0, 10.0, 0, 0)


def test_mass_total_uses_module_table():
    assert E.mass_total([]) == pytest.approx(56.578)
    assert E.mass_total(["battery", "armor"]) == pytest.approx(56.578 + 6.0 + 5.0)
    with pytest.raises(KeyError):
        E.module_mass("wings")


def test_lifetimes_are_in_the_epoch_regime():
    """The contract asks for ~25 min idle / ~4 min cruising (bare, 60 %).
    The formula's own idle:cruise ratio cannot satisfy both; assert the
    regime, not the exact minutes: idle in [20, 90] min, cruise in [2, 6]."""
    f = make_fleet()
    t_idle = run_for(f, 0, 6000.0, v=0.0)
    assert t_idle is not None and 20 * 60 <= t_idle <= 90 * 60
    f = make_fleet()
    t_cruise = run_for(f, 0, 6000.0, v=0.8, w=0.0)
    assert t_cruise is not None and 2 * 60 <= t_cruise <= 6 * 60
    assert t_idle > 4 * t_cruise


# ----------------------------------------------------------------------- pad
def test_pad_charges_only_within_radius():
    assert E.pad_charge_w(0.0) == 60.0
    assert E.pad_charge_w(0.9) == 60.0
    assert E.pad_charge_w(0.91) == 0.0
    assert E.pad_charge_w(5.0) == 0.0


def test_pad_charging_raises_charge_and_counts_it():
    f = make_fleet()
    r = f.slots[0]
    before = r.charge_wh
    # sit ON pad 0 (6, 0)
    for _ in range(125):
        f.energy_tick(0, 0.008, 0.0, 0.0, (6.0, 0.2, 0.2))
    assert r.charge_wh > before
    assert r.charge_collected == pytest.approx(r.charge_wh - before)
    # 1 s of net (60 - 2) W at the time scale
    assert r.charge_wh - before == pytest.approx(E.wh_per_s(58.0) * 1.0, rel=1e-6)
    # off the pad the charge only falls
    mid = r.charge_wh
    for _ in range(125):
        f.energy_tick(0, 0.008, 0.0, 0.0, (6.0, 1.0, 0.2))
    assert r.charge_wh < mid


def test_charge_never_exceeds_capacity():
    f = make_fleet()
    r = f.slots[0]
    r.charge_wh = 199.9
    for _ in range(250):
        f.energy_tick(0, 0.008, 0.0, 0.0, (6.0, 0.0, 0.2))
    assert r.charge_wh == pytest.approx(200.0)


def test_solar_offsets_idle():
    f = make_fleet(modules=[{"id": 0, "type": "solar", "loose": False},
                            {"id": 1, "type": "solar", "loose": False}])
    f.report(0, {"docked": {"front": 0, "rear": 1}})
    r = f.slots[0]
    before = r.charge_wh
    f.energy_tick(0, 1.0, 0.0, 0.0, (10.0, 10.0, 0.2))
    assert r.charge_wh > before                     # 12 W in > 2 W out


# -------------------------------------------------------------------- impact
def test_armor_doubles_impact_threshold():
    assert not E.impact(0.9, False)
    assert E.impact(0.91, False)
    assert E.impact(-1.0, False)
    assert not E.impact(1.5, True)
    assert E.impact(1.81, True)


def test_impact_orders_release_rear_first_then_front():
    f = make_fleet()
    f.report(0, {"docked": {"front": 0, "rear": 1}})
    assert f.impact_tick(0, 0.5) is None
    assert f.impact_tick(0, 1.2) == "release_rear"
    bus = f.bus_for(0, 1.0, (0.0, 0.0, 0.2))
    assert bus["orders"] == ["release_rear"]
    # the robot reports it let go -> module 1 is loose again
    f.report(0, {"docked": {"front": 0, "rear": None}})
    assert f.modules[1]["loose"] and f.modules[1]["holder"] is None
    assert f.impact_tick(0, 1.2) == "release_front"
    # nothing left to shed: an impact is still counted, no order
    f.report(0, {"docked": {"front": None, "rear": None}})
    assert f.impact_tick(0, 1.2) is None
    assert f.slots[0].impacts == 3


def test_armored_robot_shrugs_off_a_moderate_hit():
    f = make_fleet(modules=[{"id": 0, "type": "armor", "loose": False},
                            {"id": 1, "type": "battery", "loose": False}])
    f.report(0, {"docked": {"front": 0, "rear": 1}})
    assert f.impact_tick(0, 1.2) is None
    assert f.impact_tick(0, 2.0) == "release_rear"


# ---------------------------------------------------------------------- death
def test_death_ordering_stop_release_park():
    f = make_fleet()
    f.report(0, {"docked": {"front": 0, "rear": 1}})
    r = f.slots[0]
    r.charge_wh = 0.01
    # module 1 is a solar panel: idle it CHARGES (12 - 2.5 W), so drive
    assert f.energy_tick(0, 1.0, 1.0, 0.0, (10.0, 10.0, 0.2))     # this tick empties it
    assert f.kill(0, 100.0)
    assert not r.alive and not r.parked and r.charge_wh == 0.0
    assert f.deaths == 1
    # 1. stop is the first order out
    assert f.bus_for(0, 100.0, (10.0, 10.0, 0.2))["orders"] == ["stop"]
    assert f.bus_for(0, 100.0, (10.0, 10.0, 0.2))["state_hint"] == "dead"
    # modules still held while dying
    assert not f.modules[0]["loose"] and not f.modules[1]["loose"]
    # 2. nothing happens before 20 s
    assert f.death_tick(119.9) == []
    assert not r.parked
    # 3. at 20 s: release orders + park
    acts = f.death_tick(120.0)
    assert acts == [("park", 0)]
    assert r.parked
    assert f.modules[0]["loose"] and f.modules[1]["loose"]
    assert f.modules[0]["holder"] is None
    orders = f.bus_for(0, 120.0, (10.0, 10.0, 0.2))["orders"]
    assert "release_front" in orders and "release_rear" in orders
    # a dead robot no longer drains
    assert not f.energy_tick(0, 1.0, 1.0, 1.0, (10.0, 10.0, 0.2))
    assert f.kill(0, 130.0) is False
    # the slot is now free for fabrication
    assert f.free_slot() == 0


def test_watchdog_flags_runaway_only_when_present():
    f = make_fleet()
    assert f.watchdog(0, (1e5, 0.0, 0.2))
    assert f.watchdog(0, (float("nan"), 0.0, 0.2))
    assert not f.watchdog(0, (1.0, 0.0, 0.2))
    assert not f.watchdog(5, (1e9, 0.0, 0.2))       # parked slot: never checked


# ---------------------------------------------------------------- fabrication
def test_fabrication_refused_below_85_percent():
    f = make_fleet()
    f.slots[0].charge_wh = 0.84 * 200.0
    assert f.can_fabricate(0, (0.0, 0.0, 0.2)).startswith("charge")
    assert f.fabricate(0, (0.0, 0.0, 0.2), 10.0) is None
    assert f.births == 0


def test_fabrication_refused_far_from_bay():
    f = make_fleet()
    f.slots[0].charge_wh = 0.9 * 200.0
    assert f.can_fabricate(0, (1.6, 0.0, 0.2)).startswith("bay")
    assert f.fabricate(0, (1.6, 0.0, 0.2), 10.0) is None
    assert f.can_fabricate(0, (1.4, 0.0, 0.2)) is None


def test_fabrication_refused_without_free_slot():
    f = make_fleet(n_robots=4, alive=4)
    f.slots[0].charge_wh = 0.9 * 200.0
    assert f.can_fabricate(0, (0.0, 0.0, 0.2)) == "no free slot"
    assert f.fabricate(0, (0.0, 0.0, 0.2), 10.0) is None
    # force cannot conjure a slot either
    assert f.fabricate(0, (0.0, 0.0, 0.2), 10.0, force=True) is None


def test_fabrication_pays_45_and_child_gets_40():
    f = make_fleet()
    parent = f.slots[0]
    parent.charge_wh = 0.9 * 200.0
    res = f.fabricate(0, (1.0, 0.0, 0.2), 12.0)
    assert res is not None
    child, xyz, yaw = res
    assert parent.charge_wh == pytest.approx((0.9 - 0.45) * 200.0)
    assert child.charge_wh == pytest.approx(0.4 * 200.0)
    assert child.alive and not child.parked
    assert child.slot == 4                       # lowest parked slot
    assert child.lineage == "L0" and child.parent == "r0"
    assert child.born_at == 12.0
    assert parent.fabrications == 1 and f.births == 1
    assert E.validate(child.genome) == []
    # revived on the far side of the bay, 2.2 m out
    assert math.hypot(xyz[0], xyz[1]) == pytest.approx(2.2)
    assert xyz[0] < 0                            # parent was at +x
    assert xyz[2] == E.ROBOT_SPAWN_Z
    assert -math.pi <= yaw <= math.pi
    # the parent is now below 85 % -> no chain fabrication
    assert f.can_fabricate(0, (1.0, 0.0, 0.2)).startswith("charge")


def test_fabrication_cost_scales_with_capacity():
    f = make_fleet(modules=[{"id": 0, "type": "battery", "loose": False}])
    f.report(0, {"docked": {"front": 0, "rear": None}})
    assert f.capacity(0) == 300.0
    p = f.slots[0]
    p.charge_wh = 0.9 * 300.0
    child, _, _ = f.fabricate(0, (0.0, 0.0, 0.2), 1.0)
    assert p.charge_wh == pytest.approx(0.45 * 300.0)
    assert child.charge_wh == pytest.approx(0.4 * 200.0)   # bare child


def test_force_fabricate_bypasses_charge_and_bay():
    f = make_fleet()
    p = f.slots[1]
    p.charge_wh = 10.0
    res = f.fabricate(1, (9.0, 9.0, 0.2), 5.0, force=True)
    assert res is not None
    assert p.charge_wh == 0.0                    # floored, never negative


# --------------------------------------------------------------------- genome
def test_mutate_stays_in_range_and_is_valid():
    rng = random.Random(3)
    g = E.default_genome()
    for _ in range(500):
        g = E.mutate(g, rng, sigma=0.5)
        assert E.validate(g) == []
        for k, (lo, hi) in E.GENOME_RANGES.items():
            assert lo <= g[k] <= hi
        for t in E.MODULE_TYPES:
            assert 0.0 <= g["module_pref"][t] <= 1.0


def test_mutate_returns_new_dict_and_changes_something():
    rng = random.Random(4)
    g = E.default_genome()
    m = E.mutate(g, rng)
    assert m is not g and m["module_pref"] is not g["module_pref"]
    assert g == E.default_genome()               # parent untouched
    assert m != g


def test_clamp_genome_fills_and_clamps():
    g = E.clamp_genome({"cruise_speed": 9.0, "module_pref": {"mast": 3.0}})
    assert g["cruise_speed"] == 1.2
    assert g["module_pref"]["mast"] == 1.0 and g["module_pref"]["solar"] == 0.5
    assert E.validate(g) == []
    assert E.validate({"cruise_speed": 5.0}) != []


# ----------------------------------------------------------------------- bus
def test_bus_lists_only_loose_modules_in_range_nearest_first():
    f = make_fleet()
    # modules 0..5 loose at x = 1..6; 6, 7 parked at the crypt
    bus = f.bus_for(0, 3.0, (0.0, 0.0, 0.2))
    ids = [m["id"] for m in bus["modules"]]
    assert ids == [0, 1, 2, 3, 4, 5]             # all within 6 m, sorted by distance
    assert all(m["loose"] for m in bus["modules"])
    bus = f.bus_for(0, 3.0, (7.5, 0.0, 0.2))
    assert [m["id"] for m in bus["modules"]] == [5, 4, 3, 2, 1]   # 0 is 6.5 m away
    for k in ("t", "batt", "cap_wh", "state_hint", "pads", "bay", "modules", "orders", "genome"):
        assert k in bus
    assert bus["orders"] == ["none"]
    assert bus["state_hint"] == "ok"


def test_mast_extends_detection_range():
    f = make_fleet(modules=[{"id": 0, "type": "mast", "loose": False},
                            {"id": 1, "type": "battery", "loose": True, "pos": [9.0, 0.0, 0.2]}])
    assert f.detection_range(0) == 6.0
    assert f.visible_modules(0, (0.0, 0.0, 0.2)) == []
    f.report(0, {"docked": {"front": 0, "rear": None}})
    assert f.detection_range(0) == 12.0
    assert [m["id"] for m in f.visible_modules(0, (0.0, 0.0, 0.2))] == [1]


def test_bus_encodes_under_1kb():
    mods = [{"id": j, "type": "battery", "loose": True, "pos": [0.1 * j, 0.0, 0.2]}
            for j in range(40)]
    f = make_fleet(modules=mods)
    bus = f.bus_for(0, 100.0, (0.0, 0.0, 0.2))
    assert len(bus["modules"]) == 40
    text, kept = E.encode_bus(bus)
    assert len(text.encode("utf-8")) <= 1024
    assert 0 < kept < 40
    d = json.loads(text)
    assert d["genome"]["cruise_speed"] == 0.8 and d["orders"] == ["none"]
    assert [m["id"] for m in d["modules"]] == list(range(kept))     # nearest kept


def test_decode_status_tolerates_garbage():
    assert E.decode_status("") == {}
    assert E.decode_status("not json") == {}
    assert E.decode_status("[1,2]") == {}
    assert E.decode_status('{"state":"explore"}') == {"state": "explore"}


def test_report_reconciles_docking_and_counts_it():
    f = make_fleet()
    f.report(0, {"state": "dock", "docked": {"front": 2, "rear": None}})
    assert f.modules[2] == {"id": 2, "type": "mast", "loose": False, "holder": 0,
                            "socket": "front", "pos": [3.0, 0.0, 0.2], "yaw": 0.0}
    assert f.slots[0].modules_docked == 1 and f.docks == 1
    # repeating the same report is not a second dock
    f.report(0, {"docked": {"front": 2, "rear": None}})
    assert f.slots[0].modules_docked == 1
    # a second robot cannot claim it
    f.report(1, {"docked": {"front": 2, "rear": None}})
    assert f.slots[1].docked == {"front": None, "rear": None}
    assert f.modules[2]["holder"] == 0
    # garbage ids are ignored
    f.report(1, {"docked": {"front": "x", "rear": 99}})
    assert f.slots[1].docked == {"front": None, "rear": None}
    assert f.mass(0) == pytest.approx(56.578 + 1.5)


# ------------------------------------------------------------------- modules
def test_scatter_and_off_arena_return():
    f = make_fleet()
    assert len(f.parked_modules()) == 2
    acts = f.scatter(5)
    assert len(acts) == 2 and all(a[0] == "module" for a in acts)
    assert f.parked_modules() == []
    for _, j, xyz, yaw in acts:
        assert f.in_arena(xyz) and abs(xyz[0]) <= 12 - 1.5
        assert f.modules[j]["loose"]
    # a loose module that ended outside the walls comes back
    f.set_module_pos(0, [30.0, 0.0, 0.2])
    f.set_module_pos(1, [0.0, 0.0, -5.0])
    acts = f.module_tick()
    assert sorted(a[1] for a in acts) == [0, 1]
    assert f.in_arena(f.modules[0]["pos"]) and f.in_arena(f.modules[1]["pos"])
    assert f.module_tick() == []


def test_park_translations_sit_on_the_crypt():
    for i in range(6):
        x, y, z = E.robot_park_translation(i)
        assert x >= E.CRYPT_X and z > 0
    for j in range(14):
        x, y, z = E.module_park_translation(j)
        assert x >= E.CRYPT_X and z > 0
    assert E.robot_park_translation(0)[1] != E.module_park_translation(0)[1]


# ------------------------------------------------------------------- scoring
def test_epoch_scoring_per_lineage():
    f = make_fleet()
    r0 = f.slots[0]
    r0.charge_wh = 0.9 * 200.0
    child, _, _ = f.fabricate(0, (0.0, 0.0, 0.2), 10.0)
    f.report(0, {"docked": {"front": 0, "rear": 1}})
    f.report(1, {"docked": {"front": 2, "rear": None}})
    r0.charge_collected = 100.0
    child.charge_collected = 25.0
    f.kill(1, 50.0, "empty")
    res = f.epoch_result(240.0)
    L = res["lineages"]
    assert L["L0"]["fabrications"] == 1 and L["L0"]["modules_docked"] == 2
    assert L["L0"]["score"] == pytest.approx(10.0 + 2 + 125.0 / 50.0)
    assert L["L0"]["robots"] == 2 and L["L0"]["alive"] == 2
    assert L["L0"]["best_id"] == "r0"
    assert L["L1"]["score"] == pytest.approx(1.0)
    assert L["L1"]["deaths"] == 1 and L["L1"]["mean_lifespan_s"] == 50.0
    assert L["L2"]["score"] == 0.0
    assert "L4" not in L                          # never-born slots have no lineage
    assert res["births"] == 1 and res["deaths"] == 1 and res["alive_at_end"] == 4
    snap = f.snapshot(10, 0.08, {0: [1.0, 2.0, 0.2]})
    assert snap["robots"]["0"]["pos"] == [1.0, 2.0, 0.2]
    assert snap["robots"]["5"]["pos"] is None and snap["robots"]["5"]["parked"]
    json.dumps(snap)
    json.dumps(res)
