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

"""SparkRover Mini's make_print.py without an engine: the printability check on shapes
whose answer is known (a block, a cantilever, a bridge, an overhang over the part), the
orientation choice, the plate layout and packing, the material profiles, and the plan the
release records. Engine-free."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
sys.path.insert(0, str(PRODUCT))

import make_print as mp  # noqa: E402


def _box(x0, y0, z0, x1, y1, z1):
    m = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    m.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return m.triangles


def _part(*boxes):
    return np.concatenate([_box(*b) for b in boxes])


# ---- printability -------------------------------------------------------------------------

def test_a_block_needs_nothing():
    pa = mp.printability(_part((-10, -10, 0, 10, 10, 10)))
    assert pa["overhang_mm2"] == 0 and pa["bridge_mm2"] == 0 and pa["support"] is False
    assert pa["height_mm"] == 10.0


def test_a_cantilever_needs_build_plate_support():
    # a 10 mm post with a 20 x 20 slab out to one side at 8 mm: the slab overhangs the bed
    pa = mp.printability(_part((0, 0, 0, 10, 10, 10), (10, 0, 8, 30, 10, 10)))
    assert pa["overhang_mm2"] > 150 and pa["support"] == "buildplate"
    assert pa["overhang_over_part_mm2"] == 0


def test_a_lid_between_two_posts_is_a_bridge():
    pa = mp.printability(_part((0, 0, 0, 5, 20, 10), (15, 0, 0, 20, 20, 10), (0, 0, 10, 20, 20, 12)))
    assert pa["support"] is False and pa["overhang_mm2"] < mp.NEEDS_SUPPORT_MM2
    assert 9.0 < pa["widest_bridge_mm"] < 11.5


def test_a_bridge_too_wide_is_an_overhang():
    pa = mp.printability(_part((0, 0, 0, 5, 20, 10), (45, 0, 0, 50, 20, 10), (0, 0, 10, 50, 20, 12)))
    assert pa["support"] == "buildplate"


def test_an_overhang_over_the_part_needs_support_everywhere():
    # a wide base, a post, and a slab over the base: build-plate support cannot reach under it
    pa = mp.printability(_part((0, 0, 0, 40, 40, 2), (0, 0, 2, 8, 40, 20), (8, 0, 18, 40, 40, 20)))
    assert pa["support"] == "everywhere" and pa["overhang_over_part_mm2"] > 500


def test_a_rib_thinner_than_a_line_is_measured():
    pa = mp.printability(_part((0, 0, 0, 20, 20, 2), (0, 0, 2, 0.3, 20, 6)))
    assert pa["thinner_than_line_mm3"] > 10


def test_orientation_turns_a_part_off_its_overhang():
    # an upside-down T: as designed the slab is on top (an overhang); turned over it is on the bed
    t = _part((-3, -3, 0, 3, 3, 20), (-15, -15, 20, 15, 15, 23))
    t = t - np.array([0, 0, t[:, :, 2].min()])
    ch = mp.choose_orientation({"tris": t})
    assert ch["printability"]["support"] is False and ch["turn"] == "x 180"
    lo = ch["printability"]
    assert lo["height_mm"] == pytest.approx(23.0, abs=0.01)


def test_turned_keeps_the_part_on_the_bed_and_centred():
    t = _part((5, 5, 3, 25, 15, 11))
    for turn in mp._TURNS:
        v = mp.turned(t, turn).reshape(-1, 3)
        assert v[:, 2].min() == pytest.approx(0.0, abs=1e-9)
        assert (v[:, :2].min(0) + v[:, :2].max(0)) == pytest.approx([0.0, 0.0], abs=1e-9)


# ---- layout and packing -----------------------------------------------------------------------

def _overlap(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def test_layout_keeps_brims_apart_and_on_the_bed():
    sizes = [(160, 187), (54, 210), (30, 132), (35, 78), (35, 46), (35, 46), (52, 52)]
    pos = mp.layout(sizes)
    assert pos is not None
    rects = []
    for (w, h), (cx, cy, t) in zip(sizes, pos):
        w, h = (h, w) if t else (w, h)
        r = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        assert r[0] >= mp.EDGE_MM - 1e-9 and r[1] >= mp.EDGE_MM - 1e-9
        assert r[2] <= mp.BED_MM[0] - mp.EDGE_MM + 1e-9 and r[3] <= mp.BED_MM[1] - mp.EDGE_MM + 1e-9
        g = mp.GAP_MM / 2 - 1e-6
        rects.append((r[0] - g, r[1] - g, r[2] + g, r[3] + g))
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            assert not _overlap(rects[i], rects[j])


def test_layout_refuses_what_does_not_fit():
    assert mp.layout([(200, 200), (200, 200)]) is None


def _row(label, t, material="PETG", support=False, fp=(40, 40)):
    return {"label": label, "time_s": t, "material": material, "support": support, "footprint": list(fp)}


def test_pack_balances_and_keeps_materials_and_support_apart():
    rows = [_row("big1", 13000, fp=(160, 187)), _row("big2", 13000, fp=(160, 187)),
            _row("tpu1", 4000, "TPU 95A", fp=(30, 132)), _row("tpu2", 4000, "TPU 95A", fp=(30, 132)),
            _row("sup", 6000, support="buildplate", fp=(108, 113))] + \
           [_row(f"s{i}", 1500) for i in range(12)]
    plates = mp.pack(rows, engines=4)
    labels = [lab for pl in plates for lab in (q["label"] for q in pl["parts"])] if isinstance(plates[0]["parts"][0], dict) \
        else [lab for pl in plates for lab in pl["parts"]]
    assert sorted(labels) == sorted(r["label"] for r in rows)
    by = {r["label"]: r for r in rows}
    for pl in plates:
        names = [q["label"] if isinstance(q, dict) else q for q in pl["parts"]]
        assert len({(by[n]["material"], by[n]["support"]) for n in names}) == 1
        assert mp.layout([by[n]["footprint"] for n in names]) is not None
    loads = {}
    for pl in plates:
        loads[pl["engine"]] = loads.get(pl["engine"], 0) + pl["time_s"]
    total = sum(r["time_s"] for r in rows)
    assert max(loads.values()) <= max(13000, total / 4) * 1.35


# ---- profiles -----------------------------------------------------------------------------------

def test_material_profiles():
    base = json.loads(mp.BASE_PROFILE.read_text(encoding="utf-8"))
    petg, tpu = mp.material_profile("PETG"), mp.material_profile("TPU 95A")
    assert petg["hotend"]["max_volumetric_mm3_s"] == 17.0
    assert tpu["hotend"]["max_volumetric_mm3_s"] == 3.6
    assert "Generic%20TPU%20%40Sovol%20SV08" in tpu["hotend"]["max_volumetric_source"]
    assert petg["slicer"]["cura"]["material_print_temperature"] == 235
    assert tpu["slicer"]["cura"]["material_bed_temperature"] == 65
    # the machine is the same machine
    assert petg["klipper"] == base["klipper"] == tpu["klipper"]
    # TPU's first layer and brim stay under the melt limit at their wide, thick lines
    c = tpu["slicer"]["cura"]
    w0 = c["line_width"] * c["initial_layer_line_width_factor"] / 100.0
    for key in ("speed_print_layer_0", "skirt_brim_speed"):
        assert c[key] * w0 * c["layer_height_0"] <= 3.6 + 1e-6
    # the base profile is not changed by making a material copy
    assert json.loads(mp.BASE_PROFILE.read_text(encoding="utf-8")) == base


# ---- the recorded plan ---------------------------------------------------------------------------

PRINTED = PRODUCT / "release/make/print/printed.json"


@pytest.mark.skipif(not PRINTED.exists(), reason="no release record yet (make_print.py --plan)")
def test_the_recorded_plan_prints_every_part_once():
    rec = json.loads(PRINTED.read_text(encoding="utf-8"))
    current = [p for p in rec["parts"] if not p.get("superseded")]
    # one current drawing per part, each current on exactly one plate that holds that drawing
    assert len({p["label"] for p in current}) == len(current)
    plates = {pl["name"]: pl for pl in rec["plates"]}
    on_plates = [lab for pl in rec["plates"] for lab in pl["current_parts"]]
    assert sorted(p["label"] for p in current) == sorted(on_plates)
    for p in current:
        pl = plates[p["current_plate"]]
        assert p["label"] in pl["current_parts"] and pl["part_sha"][p["label"]] == p["design_tris_sha"]
        assert p["material"] == pl["material"]
    # a superseded drawing is current nowhere; an older plate of a replanned part says where it went
    for p in rec["parts"]:
        if p.get("superseded"):
            assert p["current_plate"] is None
    for pl in rec["plates"]:
        for lab, to in pl["replanned_by"].items():
            assert lab in plates[to]["current_parts"]
        assert pl["mode"] in (None, "twin", "ideal")
    assert {p["material"] for p in current} == {"PETG", "TPU 95A"}


def test_a_superseded_drawing_is_known_by_its_hash():
    old_sha = next(k for k, v in mp.SUPERSEDED_SHA.items() if v[0] == "el_bay_carrier")
    assert mp.superseded({"label": "el_bay_carrier", "design_tris_sha": old_sha})
    assert mp.superseded({"label": "el_bay_carrier", "design_tris_sha": "a revised drawing"}) is None
    assert mp.superseded({"label": "shell_upper_left", "design_tris_sha": old_sha}) is None
