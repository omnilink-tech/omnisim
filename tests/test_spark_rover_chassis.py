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

"""SparkRover Mini's chassis and drive (projects/manufacturing/products/spark_rover/chassis.py)
against its skeleton: the drive's sizing, the fits along each axle, the zones, the wheels,
the mount holes, the fasteners, the masses and the machine files. Engine-free."""
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))

import chassis  # noqa: E402
import skeleton as sk  # noqa: E402

R = chassis.Chassis()


@pytest.fixture(scope="module")
def checks():
    return chassis.checks(R)


@pytest.fixture(scope="module")
def solids():
    return chassis.solids(R)


def test_the_scrub_load_is_the_textbook_one():
    """mu W B / (4 T) per wheel, times the wheel's radius: the skeleton's 2.4 kg ceiling, mu 1.0,
    B 110, T 170, r 32.5."""
    d = chassis.drive(R)
    assert R.mass_kg == pytest.approx(sk.MASS_TARGET_G[1] / 1000)
    want = 1.0 * R.mass_kg * 9.81 * 110 / (4 * 170) * 0.0325 / 0.0980665
    assert d["loads_kgcm_per_wheel"]["spin_mu_hi"] == pytest.approx(want, abs=1e-3)
    assert d["loads_kgcm_per_wheel"]["spin_mu_hi"] == pytest.approx(1.262, abs=0.002)
    # Turning is possible at all only while that force is under what a tyre can grip: B < T.
    assert sk.WHEELBASE < sk.TRACK


def test_the_motor_drives_the_robot_at_every_charge(checks):
    for name in ("drive_spin_torque", "drive_ramp_10deg", "drive_speed", "drive_continuous_current", "gearbox_load"):
        assert checks[name][0], checks[name][1]
    d = chassis.drive(R)
    assert d["margins"]["spin_stall_over_load_at_6V"] > 4.0
    # A spin-turn (intermittent) at 2.4 kg: inside Pololu's 25 %-of-stall-current rule at mu 0.8,
    # a little over it at mu 1.0.
    assert d["operating"]["spin_mu_lo"]["7.2V"]["a_frac_of_stall"] <= 0.25
    assert d["operating"]["spin_mu_hi"]["7.2V"]["a_frac_of_stall"] <= 0.30
    assert 0.3 <= d["operating"]["cruise"]["6V"]["m_s"] and d["operating"]["cruise"]["8.4V"]["m_s"] < 1.5
    # Pololu's 6 V row is the 12 V row scaled: the model reproduces it.
    m = chassis.motor_at(6.0)
    assert m["no_load_rpm"] == pytest.approx(150.0) and m["stall_kgcm"] == pytest.approx(5.5)
    assert m["stall_a"] == pytest.approx(2.5)


def test_the_currents_handed_to_the_electrical_design():
    c = chassis.drive(R)["currents_per_motor_a"]
    assert c["stall"]["8.4V"] == pytest.approx(3.5)          # the worst a driver channel sees
    assert c["cruise"] < c["ramp_10deg"] < c["spin_turn"] < 1.0


def test_the_drive_line_fits_between_the_zones(checks):
    """Two 66.5 mm motors face to face on each axle, the gearbox faces on the base zone's edge
    (|y| 67), 1 mm between the encoders; the adapter clears the face screws' heads and its
    far end sits on the wheel's web."""
    assert checks["drive_line_fit"][0], checks["drive_line_fit"][1]
    assert checks["foot_screw_heads_clear_motor"][0], checks["foot_screw_heads_clear_motor"][1]
    assert checks["axle_height"][0]
    assert R.motor_l == pytest.approx(66.5)
    assert 2 * (R.face_y - R.motor_l) == pytest.approx(1.0)
    assert R.face_y == pytest.approx(sk.ZONES["chassis"][0][1].y1)


def test_every_solid_is_closed(solids):
    import trimesh
    for label, t in solids.items():
        vol, _ = chassis.mass_props(t)
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(t))
        m.merge_vertices()
        assert vol > 0 and m.is_watertight, label


def test_the_zones(checks):
    """Every solid inside the union of the chassis zones (integrate.zones' test). The spine's
    hold-down screws cross the base/spine boundary at z 60, the drive line the base/well one
    at |y| 67: allowed, both are our own zones."""
    assert checks["zones_union"][0], checks["zones_union"][1]
    zr = chassis.zone_report(R)
    assert not zr["outside"]
    assert {f"screw_spine_{s}" for s in ("lb", "lf", "rb", "rf")} <= set(zr["union_only"])
    import integrate
    assert integrate.zones({"chassis": chassis.solids(R)}) == []


def test_the_wheels_stand_where_the_skeleton_puts_them(checks, solids):
    assert checks["wheels_at_positions"][0], checks["wheels_at_positions"][1]
    assert checks["ground_clearance"][0], checks["ground_clearance"][1]
    for name, (x, y, z) in sk.WHEELS.items():
        v = solids[f"wheel_{chassis.WHEEL_LABEL[name]}"].reshape(-1, 3)
        assert v[:, 2].min() == pytest.approx(0.0, abs=1e-9)
        assert (v[:, 0].min() + v[:, 0].max()) / 2 == pytest.approx(x)
        assert (v[:, 1].min() + v[:, 1].max()) / 2 == pytest.approx(y)


def test_the_mount_holes(checks):
    assert checks["shell_mount_holes"][0], checks["shell_mount_holes"][1]
    assert checks["electrical_mount_holes"][0], checks["electrical_mount_holes"][1]
    assert checks["shoulder_bolt_holes"][0], checks["shoulder_bolt_holes"][1]
    names = {m.name for m in chassis.mounts(R)}
    assert {m.name for m in sk.SHELL_MOUNTS + sk.SHOULDER_BOLTS} <= names
    # The interface screws run along each mount's axis, head on our side, into the holder.
    every = sk.SHELL_MOUNTS + sk.SHOULDER_BOLTS + chassis.electrical_mounts(R)
    for j in chassis.interface_joints(R):
        m = next(m for m in every if m.name == j["mount"])
        assert j["axis"] == pytest.approx(m.axis)
        ours = j["grip_mm"] - (chassis.PI5_BAY_PLATE_T if j["mount"].startswith("bay_pi5") else 0.0)
        assert np.asarray(j["at"]) + np.asarray(j["axis"]) * ours == pytest.approx(np.asarray(m.at))


def test_no_arm_post_screws_and_the_key_reaches_every_head(checks):
    """The arms fasten their post to their own bracket from outside (skeleton.ARM_POST_MOUNTS is
    empty): the side plates carry no post holes or screws. An L-key still reaches every head on
    the side plates (2 shell, 4 shoulder) through the spine's open front or rear."""
    assert sk.ARM_POST_MOUNTS == []
    assert not any("arm_post" in m.name for m in chassis.mounts(R))
    assert not any("arm_post" in j["label"] for j in chassis.interface_joints(R))
    assert "arm_post_holes" not in checks
    assert len(chassis.side_plate(R).interiors) == len(chassis.side_plate(chassis.Chassis()).interiors)
    holes = [h for h in chassis.side_plate(R).interiors if abs(chassis.Polygon(h).area - math.pi * 1.7 ** 2) < 0.2]
    assert len(holes) == 2 + 4                         # the upper shell mounts and the shoulder bolts
    assert checks["key_access"][0], checks["key_access"][1]
    heads = chassis.key_access(R)["heads"]
    assert len(heads) == 6 and all(h["swing_deg"] >= 60 for h in heads.values())


def test_the_fasteners(checks):
    assert checks["fasteners"][0], checks["fasteners"][1]
    by = {}
    for j in chassis.joints(R):
        by.setdefault(j["joint"], set()).add((j["size"], j["length"]))
    assert by["gearbox_face"] == {("M3", 8)}         # 2 mm flange + 0.5 washer, 5.5 mm into the 6.0 mm hole
    assert by["bracket_foot"] == {("M3", 12)}        # foot + spacer + lower deck layer, nut in the upper's trap
    assert by["spine_tslot"] == {("M3", 16)}
    assert by["wheel_hub"] == {("M4", 8)}            # the screws Pololu supplies with #2684
    face = next(j for j in chassis.joints(R) if j["joint"] == "gearbox_face")
    assert 4.5 <= face["thread_in_mm"] <= 5.5
    for j in chassis.interface_joints(R):
        if j["mount"].startswith("bay_pi5"):
            continue
        assert j["thread_in_mm"] >= 4.5 and j["size"] == "M3"


def test_the_electrical_requests(checks):
    """electrical.py's asks: the sealed 2S pack fits the reserved volume; the Pi 5's M2.5 screws
    through the rear plate and the 3 mm bay plate, 1.5 d into F-F standoffs; the front motors'
    leads routed inside the spine through the mid rung's slot; the spine open at z 136..180
    for the harness and two FPC ribbons; the carriers' holes kept."""
    for name in ("battery_pack_fits", "harness_pass", "front_lead_routes", "electrical_mount_holes"):
        assert checks[name][0], checks[name][1]
    pi = chassis.pi5_joints(R)
    assert len(pi) == 4
    for j in pi:
        assert (j["size"], j["length"], j["grip_mm"]) == ("M2.5", 16, 9.0)
        assert j["thread_in_mm"] == pytest.approx(7.0) and j["thread_in_mm"] >= 1.5 * 2.5
        assert j["thread_in_mm"] + j["pi_thread_mm"] + 0.5 <= j["standoff_mm"] + 1e-9
        assert j["at"][0] == pytest.approx(-R.spine_x + R.spine_t) and j["ok"]
        assert j["label"] in chassis.interface_solids(R)
    # An 8 mm standoff cannot hold both: ISO 4762 has no M2.5 between x 12 (3.0 mm) and x 16 (7.0).
    assert not any(j["ok"] for j in chassis.pi5_joints(R, standoff_mm=8.0))
    assert chassis.PI5_STANDOFF["len_mm"] == 12.0
    hp = chassis.harness_pass(R)
    assert hp["z"] == (136.0, 180.0) and hp["clear_y"] >= 40.0
    lr = chassis.lead_routes(R)
    assert lr["to_bay_mm"] < chassis.MOTOR_LEAD_MM - chassis.LEAD_SLACK_MM
    assert lr["joint_mm"][0] < -R.spine_x                      # the joint lies in the bay, behind the spine
    # The carriers' holes: the bay's five-hole M3 row at z 64, the front grid's (+-40, 190) and (+-40, 250).
    names = {(m.at[0], m.at[1], m.at[2]) for m in chassis.electrical_mounts(R) if m.size == "M3"}
    for y in (-40.0, -20.0, 0.0, 20.0, 40.0):
        assert (-R.spine_x, y, 64.0) in names
    for y in (-40.0, 40.0):
        for z in (190.0, 250.0):
            assert (R.spine_x, y, z) in names


def test_the_battery_volume_is_kept_free(checks):
    """The 2S2P pack (75 x 72 x 20) with 1.5 mm round it, between the hung motors (|x| >= 42.5);
    the strap slots outboard of it, the lead hole beyond its -y end, its underside above the
    skeleton's ground clearance."""
    assert checks["battery_volume_clear"][0], checks["battery_volume_clear"][1]
    assert checks["battery_pack_fits"][0], checks["battery_pack_fits"][1]
    hb, pb = chassis.BATTERY_HOLDER["box"], chassis.BATTERY_HOLDER["pack_box"]
    assert hb.x1 - pb.x1 == pytest.approx(chassis.BATTERY_MARGIN) and hb.z1 == pytest.approx(R.deck_z)
    assert hb.z0 >= sk.GROUND_CLEARANCE
    sx = chassis.BATTERY_HOLDER["strap_x"]
    assert sx - chassis.STRAP_SLOT_W / 2 >= hb.x1 - 1e-9
    deck = chassis.deck_layer(R, False)
    assert not deck.contains(chassis.Point(sx, 0.0)) and not deck.contains(chassis.Point(-sx, 0.0))
    lx, ly, _ = chassis.BATTERY_HOLDER["lead_hole"]
    assert not deck.contains(chassis.Point(lx, ly)) and abs(ly) > pb.y1


def test_masses():
    ms = chassis.mass_summary(R)
    assert 700.0 < ms["total_g"] < 1200.0
    assert abs(ms["com_mm"][0]) < 2.0 and abs(ms["com_mm"][1]) < 1.0      # symmetric
    ps = chassis.parts(R)
    assert len({p["label"] for p in ps}) == len(ps)
    for p in ps:
        assert p["kind"] in ("made", "bought") and p["mass_g"] >= 0.0 and len(p["centre_mm"]) == 3
        if p["kind"] == "bought":
            assert p["source"], p["label"]
    motors = [p for p in ps if re.fullmatch(r"motor_(fl|fr|rl|rr)", p["label"])]
    assert len(motors) == 4 and all(p["mass_g"] == 101.0 for p in motors)


def test_the_parts_and_solids_match(solids):
    labels = {p["label"] for p in chassis.parts(R)}
    assert set(solids) <= labels


def test_the_laser_jobs():
    jobs = chassis.jobs(R)
    for name in ("deck_lower", "deck_upper", "spacers", "spine_side_l", "spine_side_r", "spine_rear",
                 "spine_front", "spine_rungs"):
        text = jobs[name]
        passes = len(re.findall(r"^; cut pass", text, flags=re.M))
        t = 3.0 if name.startswith(("deck", "spacers")) else 6.0
        assert passes == chassis.LASER[t]["passes"], name
        assert "M4 S0" in text and text.rstrip().endswith("M2")
        assert f"F{chassis.LASER[t]['mm_s'] * 60:g}" in text


def test_the_spine_foot_program_preflights():
    sys.path.insert(0, str(ROOT / "projects/manufacturing/cnc"))
    import machine as machine_mod
    import preflight
    import tool_table
    mdir = ROOT / "projects/manufacturing/machines/longmill_mk2_30x30"
    rep = preflight.preflight(machine_mod.load(mdir / "longmill_mk2_30x30.grbl"), chassis.jobs(R)["spine_foot"], None,
                              tools=tool_table.load(mdir / "longmill_mk2_tools_aluminium_fine.json"))
    assert rep["error_count"] == 0 and rep["program_end"]
    assert rep["bounds"]["z"]["work"][0] == pytest.approx(-(R.foot_t + chassis.THROUGH_MM))


def test_no_part_interferes_with_another():
    """The integrator's FCL method on the chassis alone: every contact a seat or a thread."""
    pytest.importorskip("fcl")
    rep = chassis.interference(R)
    assert rep["interference"] == [], rep["interference"]
