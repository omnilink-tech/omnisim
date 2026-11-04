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

"""The clothes rack made on the CNC twin (projects/manufacturing/products/clothes_rack):
its design, its connector program, its assembly plan and its load-test world. Engine-free."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SERVICE = ROOT / "projects/manufacturing/cnc"
MACHINES = SERVICE.parent / "machines"
PRODUCT = ROOT / "projects/manufacturing/products/clothes_rack"
for p in (SERVICE, PRODUCT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import assembly  # noqa: E402
import design  # noqa: E402
import preflight  # noqa: E402
import machine as machine_mod  # noqa: E402
import tool_table as tools_mod  # noqa: E402

RACK = design.Rack()


def test_the_tubes_meet_the_connectors():
    assert RACK.upright_mm == pytest.approx(1419.0)
    assert RACK.bar_z == 1485.0 and RACK.base_z == 35.0
    t = {x["name"]: x for x in design.tubes(RACK)}
    # The upright stops 0.5 mm short of each tube it meets.
    up = t["upright_right"]
    assert up["centre"][2] - up["length"] / 2 == pytest.approx(RACK.base_z + RACK.tube_r + RACK.end_gap_mm)
    assert up["centre"][2] + up["length"] / 2 == pytest.approx(RACK.bar_z - RACK.tube_r - RACK.end_gap_mm)


def test_the_connector_half_clamps_a_30_mm_tube():
    z = lambda x, y: float(design.connector_surface(RACK, np.array(x), np.array(y)))  # noqa: E731
    # Bore bottoms: the bore radius below its centre, which stands relief_mm above the face.
    assert z(0.0, RACK.through_y_mm) == pytest.approx(RACK.relief_mm - RACK.bore_r_mm)
    assert z(0.0, -30.0) == pytest.approx(RACK.relief_mm - RACK.bore_r_mm)
    # A closed pair: 2 * 0.5 mm between the faces, so any tube from 29.2 to 30.2 mm is gripped.
    assert 2 * RACK.bore_r_mm - 2 * RACK.relief_mm <= 29.2 + 1e-9 and 2 * RACK.bore_r_mm >= 30.2 - 1e-9
    for hx, hy in RACK.holes_xy:
        assert z(hx, hy) == -RACK.block_t_mm                   # through
    # Both halves are the same part: the face is symmetric about the upright's axis.
    zz, _, _ = design.half_grid(RACK, 0.5)
    assert np.array_equal(zz, zz[:, ::-1])


def test_the_connectors_close_around_both_tubes():
    for c in design.connectors(RACK):
        R = c["R"]
        assert np.allclose(R.T @ R, np.eye(3)) and np.linalg.det(R) == pytest.approx(1.0)
        # The upright leaves along connector -y, which points along the upright toward the other end.
        away = -R[:, 1]
        assert away[2] == pytest.approx(1.0 if c["name"].startswith("foot") else -1.0)


def test_the_bill_of_materials_adds_up():
    rows = design.bom(RACK)
    assert sum(r["qty"] for r in rows if r["item"] == "Connector half") == 8
    assert sum(r["qty"] for r in rows if r["item"] == "Bolt") == 16
    mp = design.mass_properties(RACK, 20.0)
    assert mp["mass_kg"] == pytest.approx(sum(r["kg"] for r in rows) + 20.0, abs=2e-3)   # every BOM line
    assert mp["com_m"][1] == pytest.approx(0.0)
    I = np.array(mp["inertia_kg_m2"])
    assert np.allclose(I, I.T) and np.all(np.linalg.eigvalsh(I) > 0)


def test_the_hand_calculations():
    c = design.checks(RACK, 20.0)
    g = 9.81
    assert c["tip_force_at_bar_n"] == pytest.approx((c["rack_kg"] + 20.0) * g * RACK.foot_y_mm / RACK.bar_z, abs=0.2)
    assert c["bar_safety_factor"] > 5
    assert design.checks(RACK, 0.0)["bar_safety_factor"] is None


def test_the_program_fits_grbl_and_the_machine():
    text, info = design.program(RACK, res=0.5)
    lines = text.splitlines()
    assert max(len(l) for l in lines) + 1 <= 127            # GRBL's line buffer, comments included
    assert info["holes"] == 4
    assert sum(1 for l in lines if l.startswith("G2 ")) > 4 * 60          # helical, not plunged
    m = machine_mod.load(MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl")
    rep = preflight.preflight(m, text, wco=[-400.0, -440.0, -60.0], tools=tools_mod.load(design.TOOLS))
    assert rep["error_count"] == 0 and rep["violation_count"] == 0
    assert rep["warning_counts"]["rapid_below_z0"] == 0                   # pecks feed back down
    assert [t["tool"]["number"] for t in rep["tool_changes"]] == [1, 2]


def test_preflight_refuses_a_line_grbl_cannot_take():
    m = machine_mod.load(MACHINES / "longmill_mk2_30x30/longmill_mk2_30x30.grbl")
    rep = preflight.preflight(m, "(" + "x" * 130 + ")\nG0 X1\n")
    assert rep["errors"][0]["code"] == "line_too_long"


def test_the_assembly_plan_ends_assembled():
    p = assembly.plan(RACK)
    shapes = p.pop("_shapes")
    assert set(shapes) == set(p["parts"])
    mm = 1e-3
    conns = {c["name"]: c for c in design.connectors(RACK)}
    end = lambda name: p["parts"][name][-1]  # noqa: E731
    for cname, c in conns.items():
        for side in ("a", "b"):
            assert end(f"{cname}_{side}")[1:4] == pytest.approx(list(np.asarray(c["origin"]) * mm), abs=1e-6)
    for t in design.tubes(RACK):
        assert end(t["name"])[1:4] == pytest.approx(list(np.asarray(t["centre"]) * mm), abs=1e-6)
    # Every bolt ends in a hole of its connector, against half A's back face.
    for k in range(16):
        b = np.asarray(end(f"bolt_{k}")[1:4]) / mm
        c = conns[("foot_left", "foot_right", "top_left", "top_right")[k // 4]]
        local = c["R"].T @ (b - np.asarray(c["origin"]))
        hx, hy = RACK.holes_xy[k % 4]
        assert local == pytest.approx([hx, hy - RACK.through_y_mm, -(RACK.block_t_mm + RACK.relief_mm)], abs=1e-3)
    # Captions follow one another with no gap, and the camera keys run forward in time.
    for a, b in zip(p["captions"], p["captions"][1:]):
        assert a[1] == b[0]
    assert all(a[0] <= b[0] for a, b in zip(p["camera"], p["camera"][1:]))


def test_the_committed_plan_and_worlds_are_current():
    p = assembly.plan(RACK)
    p.pop("_shapes")
    committed = json.loads(assembly.PLAN.read_text(encoding="utf-8"))
    assert committed["parts"] == json.loads(json.dumps(p["parts"]))
    lt = (ROOT / "projects/samples/demos/worlds/manufacturing/clothes_rack_load_test.omniworld").read_text(encoding="utf-8")
    assert "DEF RACK Solid" in lt and "inertiaMatrix" in lt and "newtonGroundMu 0.6" in lt
    al = (ROOT / "projects/samples/demos/worlds/manufacturing/cnc_longmill_mk2_aluminium.omniworld").read_text(encoding="utf-8")
    assert '"--material=aluminium_alloy"' in al and "longmill_mk2_tools_aluminium.json" in al
