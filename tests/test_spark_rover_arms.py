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

"""SparkRover Mini's arms (projects/manufacturing/products/spark_rover/arms.py) against its
skeleton: the servo's sizing, the swept arm in its zones, the clearances, the shoulder
interface, the fasteners, the simulator's joint data, the machine files. Engine-free."""
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import Point, box

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))

import arms  # noqa: E402
import skeleton as sk  # noqa: E402

A = arms.A


@pytest.fixture(scope="module")
def checks():
    return arms.checks()


@pytest.fixture(scope="module")
def solids():
    return arms.solids()


def test_every_design_rule_holds(checks):
    bad = {k: v[1] for k, v in checks.items() if not v[0]}
    assert not bad, bad


def test_the_servo_needs_a_regulated_rail_not_the_pack():
    lo, hi = arms.SERVO["voltage_v"]
    assert hi < 8.4 and hi < 7.4                       # 2S runs 6.0-8.4 V: above the XC330's 6.0 V
    assert arms.joints()[0]["supply_v"] == 5.0


def test_torque_and_speed_margins():
    s = arms.sizing()
    for name, row in s["joints"].items():
        assert row["torque_margin"] >= 3.0, name
        assert row["speed_margin"] >= 1.2, name       # 90 deg in 0.5 s with a trapezoid profile
        assert row["push_over_stall"] < 1.0, name     # a 10 N push stays under the metal gear train's stall load
    # the wave's peak speed is what the profile says: 90 deg over two thirds of 0.5 s
    assert s["joints"]["shoulder"]["wave_peak_rad_s"] == pytest.approx(math.radians(90) / (0.5 * 2 / 3), abs=1e-3)


def test_the_swept_arm_stays_in_its_zone_at_every_5_degrees():
    sw = arms.sweep()
    assert sw["samples"] == (len(arms._angles(A.shoulder_deg)) * len(arms._angles(A.elbow_deg)))
    assert not sw["outside"]
    assert min(sw["margins_mm"].values()) >= 0.0
    # the hanging arm is the lowest pose and clears the zone's floor (z 88: above the lower body)
    assert sw["extent_mm"]["z0"] == pytest.approx(sk.SHOULDER_AXIS_Z - A.L1 - A.fore_len, abs=0.05)


def test_the_limits_are_the_widest_the_zone_allows():
    lo, hi = arms.feasible_shoulder_range()
    assert (lo, hi) == A.shoulder_deg
    assert A.elbow_deg == (-90.0, 90.0)
    a_wide = arms.Arm(shoulder_deg=(lo - 5.0, hi))
    assert arms.sweep(a_wide)["outside"]               # 5 deg further back leaves arm_l (x -70)


def test_links_clear_each_other_and_the_post():
    sw = arms.sweep()
    assert sw["elbow_clearance_mm"] >= 0.9             # 1 mm nominal, polygons of 40-gon circles
    assert sw["shoulder_clearance_mm"] >= 0.9


def test_the_post_passes_the_shell_opening_and_the_arm_stays_off_the_skin():
    ps = arms.arm_parts()
    skin = sk.SHOULDERS["left"][1]
    for p in ps:
        for y0, y1, poly in p.bands:
            if y1 > skin - sk.SHELL_WALL and y0 < skin:         # crosses the skin's slab
                assert p.link == "static"                         # only the post, which does not turn
                assert poly.buffer(1.0).within(Point(0, 0).buffer(sk.SHELL_ARM_OPENING_D / 2))
    assert min(b[0] for p in ps if p.link != "static" for b in p.bands) >= skin + 1.0


def test_solids_are_closed_and_in_the_arms_zones(solids):
    import trimesh
    assert len(solids) > 40
    for k, t in solids.items():
        assert t.shape[1:] == (3, 3), k
        assert trimesh.triangles.mass_properties(t)["volume"] > 0, k     # outward normals
        assert arms._in_zones(t), k
    # the integrator's own check
    import integrate
    assert integrate.zones({"arms": solids}) == []


def test_no_part_interferes_with_another(solids):
    import integrate
    r = integrate.interference({"arms": solids})
    assert r["interference"] == []


def test_a_raised_arm_is_still_in_its_zone():
    s = arms.solids(q={"shoulder_left": 90.0, "elbow_left": 90.0, "shoulder_right": -70.0, "elbow_right": -90.0})
    for k, t in s.items():
        assert arms._in_zones(t), k


def test_the_right_arm_is_the_left_mirrored(solids):
    for k, t in solids.items():
        if k.endswith("_left") or k.endswith("_left_in_body") or k.endswith("_left_outside"):
            r = solids[k.replace("_left", "_right")]
            lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
            rlo, rhi = r.reshape(-1, 3).min(0), r.reshape(-1, 3).max(0)
            assert np.allclose([lo[0], hi[0], lo[2], hi[2]], [rlo[0], rhi[0], rlo[2], rhi[2]], atol=1e-6), k
            assert np.allclose([lo[1], hi[1]], [-rhi[1], -rlo[1]], atol=1e-6), k


def test_the_bracket_takes_the_skeletons_shoulder_bolts():
    assert arms.mounts()[:len(sk.SHOULDER_BOLTS)] == list(sk.SHOULDER_BOLTS)
    br = next(p for p in arms.arm_parts() if p.label == "shoulder_bracket")
    (y0, y1, poly), = br.bands
    assert y0 == pytest.approx(55.0)                    # on the spine's side face
    assert y1 - y0 >= arms.SPINE_THREAD_MIN             # >= 6.5 mm of thread (the chassis' M3 x 12 puts 6 in)
    for t, s in arms.shoulder_bolts_ts():
        assert not poly.contains(Point(t, s))           # a hole there
        assert poly.buffer(-0.01).boundary.distance(Point(t, s)) < arms.TAP["M3_alu"] / 2 + 0.05
        assert Point(t, s).buffer(4.0).difference(Point(t, s).buffer(1.3)).within(poly)  # metal round it
    for m in sk.SHOULDER_BOLTS:
        assert m.holder == "arms" and m.part == "chassis"
        assert m.axis[1] * m.at[1] > 0                  # from inside the spine, outward, into our thread
    zone = dict(sk.ZONES["arms"])["shoulder_l"]
    x0, z0, x1, z1 = poly.bounds
    assert zone.x0 <= x0 and x1 <= zone.x1             # (t = x at the shoulder)


def test_fasteners():
    rows = arms.fastener_joints()
    names = {r["joint"] for r in rows}
    assert names == {"post_flange", "shoulder_horn", "elbow_horn", "upper_halves", "fore_halves_in", "fore_halves_root"}
    for r in rows:
        assert r["thread_in_mm"] >= r["min_thread_mm"] - 1e-9, r
        assert r["thread_in_mm"] <= r.get("max_thread_mm", r["hole_mm"]) + 1e-9, r
    horn = [r for r in rows if r["joint"].endswith("_horn")]
    assert all(r["size"] == "M2" and r["length"] == 6 and r["thread_in_mm"] == 3.0 for r in horn)  # ROBOTIS' rule
    post = next(r for r in rows if r["joint"] == "post_flange")
    assert post["grip_mm"] == A.flange_t and post["dir"] == -1                 # from outside, into the bracket
    assert post["thread_in_mm"] <= A.bracket_t                                # the tip stays in the bracket


def test_the_arm_module_is_fitted_from_outside_last():
    # shell and electronics are in; nothing is reached from inside the spine
    assert arms.mounts() == list(sk.SHOULDER_BOLTS)
    assert not hasattr(arms, "post_mounts")
    fl = arms.flange_outline(A)
    assert fl.within(Point(0, 0).buffer(sk.SHELL_ARM_OPENING_D / 2 - 1.0))     # the flange passes the opening
    acc = arms.post_access()
    assert acc["ok"], acc
    for row in acc["rows"]:
        assert row["clearance_mm"] >= 0.2 and row["in_opening"]
        assert row["reach_mm"] + 5.0 <= arms.POST_ACCESS["blade_mm"]
    # the bracket carries the flange screws' threads, clear of the shoulder bolts
    br = next(p for p in arms.arm_parts() if p.label == "shoulder_bracket")
    poly = br.bands[0][2]
    for t, s_ in arms.FLANGE_SCREWS:
        assert not poly.contains(Point(t, s_))
        assert Point(t, s_).buffer(3.0).difference(Point(t, s_).buffer(1.3)).within(poly)
        assert min(math.dist((t, s_), b) for b in arms.shoulder_bolts_ts()) >= 6.0
    # the access bores go through every layer of the upper arm, outside the servo's pocket
    U = arms.upper_outlines(A)
    for p in arms.arm_parts():
        if p.label in ("upper_arm_inner", "upper_arm_outer", "shoulder_cap"):
            for _, _, poly in p.bands:
                for t, s_ in arms.FLANGE_SCREWS:
                    assert poly.distance(Point(t, s_)) >= arms.POST_ACCESS["bore_d_mm"] / 2 - 0.05
    for t, s_ in arms.FLANGE_SCREWS:
        assert U["pocket"].distance(Point(t, s_)) >= arms.POST_ACCESS["bore_d_mm"] / 2 + 0.8
        assert U["body"].exterior.distance(Point(t, s_)) >= arms.POST_ACCESS["bore_d_mm"] / 2 + 0.8


def test_joint_data_for_the_simulator():
    js = {j["name"]: j for j in arms.joints()}
    assert set(js) == {"shoulder_left", "elbow_left", "shoulder_right", "elbow_right"}
    for side, sy in arms.SIDES.items():
        sh, el = js[f"shoulder_{side}"], js[f"elbow_{side}"]
        x0, y0, z0 = sk.SHOULDERS[side]
        assert sh["axis"] == (0.0, -1.0, 0.0) and el["axis"] == (0.0, -1.0, 0.0)
        assert sh["anchor_mm"][0] == x0 and sh["anchor_mm"][2] == z0     # on the skeleton's shoulder axis
        assert sh["anchor_mm"][1] * sy > abs(y0)                          # moved along it into the arm's material
        assert el["anchor_mm"][2] == pytest.approx(z0 - A.L1)
        assert sh["limits_deg"] == A.shoulder_deg and el["limits_deg"] == A.elbow_deg
        assert sh["max_torque_nm"] == 0.93 and sh["max_velocity_rad_s"] == pytest.approx(81 * 2 * math.pi / 60, abs=1e-3)
    links = arms.links()
    for name, li in links.items():
        I = np.asarray(li["inertia_kgm2"])
        assert np.allclose(I, I.T)
        assert np.all(np.linalg.eigvalsh(I) > 0), name
        ev = np.linalg.eigvalsh(I)
        assert ev[0] + ev[1] >= ev[2] - 1e-12           # triangle inequality of a real body
        assert 0.02 < li["mass_kg"] < 0.1, name
        assert np.linalg.norm(li["com_from_origin_m"]) < 0.03, name   # the origin is near the material


def test_masses_add_up():
    rows = arms.parts()
    links = arms.links()
    for side in arms.SIDES:
        for link, name in (("upper", "upper_arm"), ("fore", "forearm")):
            g = sum(r["mass_g"] for r in rows if r["link"] == f"{link}_{side}")
            assert g == pytest.approx(links[f"{name}_{side}"]["mass_kg"] * 1000, abs=0.05)
    total = sum(r["mass_g"] for r in rows)
    assert total <= 2 * arms.ARM_MASS_BUDGET_G
    servos = [r for r in rows if r["label"].startswith("servo_") and "horn" not in r["label"]]
    assert len(servos) == 4 and all(r["mass_g"] == 22.0 for r in servos)


def test_cable_slack_covers_the_range():
    c = arms.cable()
    assert c["shoulder"]["slack_mm"] < 2 * math.pi * c["shoulder"]["wrap_radius_mm"]
    assert c["elbow"]["slack_mm"] <= 55.0


def test_jobs():
    j = arms.jobs()
    prints = [p for p in arms.arm_parts() if p.process == "print"]
    assert set(j) == {"shoulder_brackets"} | {f"{p.label}_{s}" for p in prints for s in arms.SIDES}
    nc = j["shoulder_brackets"]
    assert "T3 M6" in nc and "T1 M6" in nc and nc.rstrip().endswith("M2")
    zs = [float(z) for z in re.findall(r"Z(-?\d+\.\d+)", nc)]
    assert min(zs) == pytest.approx(-(A.bracket_t + arms.THROUGH_MM), abs=1e-3)
    t3 = nc[nc.index("T3 M6"):nc.index("T1 M6")]
    assert min(float(z) for z in re.findall(r"Z(-?\d+\.\d+)", t3)) >= -arms.T3_FLUTE_MM   # within the 1 mm tool's flute
    for k, text in j.items():
        if k == "shoulder_brackets":
            continue
        assert text.startswith("solid ") and text.rstrip().endswith(f"endsolid spark_rover {k}")
        v = np.array([[float(x) for x in m] for m in re.findall(r"vertex (\S+) (\S+) (\S+)", text)])
        assert v[:, 2].min() == pytest.approx(0.0, abs=1e-3)       # on the bed
        assert np.all(v.max(0) - v.min(0) <= np.array(arms.SV08_BED_MM))


def test_print_check_catches_an_overhang():
    p = arms.Part("t", "static", "made", "print", "PETG", "", [(0, 2, box(0, 0, 10, 10)), (2, 3, box(0, 0, 20, 10))],
                  density=arms.PETG_G_MM3, bed="min")
    assert not arms.print_check(p)["ok"]                 # 9 mm sticking out over air
    ok = arms.Part("t", "static", "made", "print", "PETG", "",
                   [(0, 2, box(0, 0, 30, 10).difference(box(5, -1, 25, 11))), (2, 3, box(0, 0, 30, 10))],
                   density=arms.PETG_G_MM3, bed="min")
    assert arms.print_check(ok)["ok"]                    # a 20 mm bridge between two piers


def test_the_post_crosses_the_skin_inside_the_opening_zones():
    zl = dict(sk.ZONES["arms"])["opening_l"]
    for p in arms.arm_parts():
        for y0, y1, poly in p.bands:
            if min(y1, zl.y1) - max(y0, zl.y0) > 1e-9:       # in the skin's slab (|y| 79..83)
                assert p.label in ("shoulder_post",), p.label
                x0, s0, x1, s1 = poly.bounds
                z0, z1 = sk.SHOULDER_AXIS_Z - s1, sk.SHOULDER_AXIS_Z - s0
                assert zl.x0 - 0.05 <= x0 and x1 <= zl.x1 + 0.05 and zl.z0 - 0.05 <= z0 and z1 <= zl.z1 + 0.05
