"""The desk robot's bought parts: their drawn models, the fasteners and the assembled clearance."""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "projects/manufacturing/products/desk_robot"))

import bought_parts  # noqa: E402
import design  # noqa: E402

R = design.Robot()


def test_every_screw_is_long_enough_and_no_longer_than_it_must_be():
    for j in bought_parts.joints(R):
        assert j["length"] >= j["needs_mm"]
        shorter = [L for L in bought_parts.SHCS_LENGTHS if L < j["length"]]
        assert not shorter or shorter[-1] < j["needs_mm"], j


def test_the_bom_lists_the_screws_the_joints_need():
    by = {}
    for j in bought_parts.joints(R):
        by.setdefault(j["joint"].split("_")[0], set()).add(f"{j['size']} x {j['length']}")
    spec = {k: design.BOUGHT[k]["spec"] for k in ("screw_m3", "screw_m2_clamp", "screw_m16_caster", "screw_m2_tray")}
    assert by["standoff"] == {"M3 x 8"} and spec["screw_m3"].startswith("M3 x 8")
    assert by["motor"] == {"M2 x 8"} and spec["screw_m2_clamp"].startswith("M2 x 8")
    assert by["caster"] == {"M1.6 x 8"} and spec["screw_m16_caster"].startswith("M1.6 x 8")
    assert by["tray"] == {"M2 x 8"} and spec["screw_m2_tray"].startswith("M2 x 8")
    counts = {"standoff": 8, "motor": 4, "caster": 2, "tray": 2}
    for k, n in counts.items():
        assert sum(1 for j in bought_parts.joints(R) if j["joint"].startswith(k)) == n
        assert n == sum(design.BOUGHT[b]["qty"] for b in
                        {"standoff": ["screw_m3"], "motor": ["screw_m2_clamp"], "caster": ["screw_m16_caster"],
                         "tray": ["screw_m2_tray"]}[k])


def test_the_caster_head_fits_its_notch_and_the_clamp_nuts_clear_the_walls():
    # Pololu's housing notches are 3.0 mm wide (measured on its STEP); a cap head must fit.
    assert bought_parts.SHCS["M1.6"]["head_d"] <= 3.0 < bought_parts.SHCS["M2"]["head_d"]
    wall = (R.motor["body_mm"][0] + 2 * R.clamp_clear) / 2 + R.clamp_wall
    nut_corner = bought_parts.NUT["M2"]["s"] / 3 ** 0.5
    assert R.clamp_hole_spacing / 2 - nut_corner >= wall + 0.2
    assert R.clamp_hole_spacing / 2 + nut_corner <= R.clamp_span / 2


@pytest.mark.parametrize("model", sorted(bought_parts.drawn_models()))
def test_drawn_models_are_closed_solids(model):
    import trimesh
    for _, tris in bought_parts.drawn_models()[model]:
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
        m.merge_vertices()
        assert len(m.faces) > 0 and m.is_watertight, model


def test_the_assembled_robot_has_no_interference_with_the_drawn_models():
    res = bought_parts.clearance(R, prefer_vendor=False)
    assert res["interference"] == []
    assert res["pressed"] and all(p["ok"] for p in res["pressed"])
