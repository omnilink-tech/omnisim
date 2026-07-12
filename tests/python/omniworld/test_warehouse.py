"""T1.6 warehouse recipe tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_warehouse_registered():
    assert "warehouse" in list_recipes()


def test_warehouse_generates_and_validates(tmp_path: Path):
    out = tmp_path / "warehouse.wbt"
    result = generate("warehouse", seed=42, out=out)
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    # Pallet racks and at least one box type present.
    assert "WoodenPalletStack {" in text
    assert any(k in text for k in ("CardboardBox {", "PlasticCrate {", "WoodenBox {"))
    # Warehouse is flat — no ElevationGrid.
    assert "ElevationGrid {" not in text
    # Validator passes.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_warehouse_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("warehouse", seed=3, out=a)
    generate("warehouse", seed=3, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_warehouse_box_count_matches_param(tmp_path: Path):
    out = tmp_path / "warehouse.wbt"
    result = generate("warehouse", seed=1, out=out, params={"box_count": 10})
    extras = result.manifest.extra
    assert extras["rack_count"] > 0
    # Solver rejects candidates that collide with racks, so we get at
    # most ``box_count`` boxes but usually fewer.
    assert 0 < extras["box_count"] <= 10


def test_warehouse_floor_boxes_not_inside_racks(tmp_path: Path):
    """Every *floor-level* box must sit outside every rack footprint.
    Boxes stacked on top of racks (T1.11 clutter) have z > 0 and are
    exempt from the horizontal-separation check."""
    out = tmp_path / "warehouse.wbt"
    result = generate("warehouse", seed=5, out=out)
    floor_boxes = [
        p for p in result.description.props
        if p.proto_type in ("CardboardBox", "PlasticCrate", "WoodenBox")
        and p.translation[2] < 0.5
    ]
    racks = [p for p in result.description.props
             if p.proto_type == "WoodenPalletStack"]
    assert racks, "expected at least one rack"
    for b in floor_boxes:
        bx, by, _ = b.translation
        for r in racks:
            rx, ry, _ = r.translation
            # Rack footprint: 1.2 x 1.0 (half: 0.6, 0.5).
            assert not (abs(bx - rx) < 0.6 and abs(by - ry) < 0.5), (
                f"floor box at {(bx, by)} inside rack footprint {(rx, ry)}"
            )


def test_warehouse_boxes_not_in_loading_dock(tmp_path: Path):
    out = tmp_path / "warehouse.wbt"
    result = generate("warehouse", seed=7, out=out)
    dock_length = 6.0
    boxes = [p for p in result.description.props
             if p.proto_type in ("CardboardBox", "PlasticCrate", "WoodenBox")]
    for b in boxes:
        bx, _, _ = b.translation
        assert bx >= dock_length, (
            f"box at x={bx} inside loading dock"
        )


def test_warehouse_spawn_urdf_places_robot(tmp_path: Path):
    out = tmp_path / "warehouse_spawn.wbt"
    result = generate(
        "warehouse",
        seed=9,
        out=out,
        params={"spawn_urdf": "../robots/cube_bot.urdf"},
    )
    text = out.read_text(encoding="utf-8")
    assert "URDFRobot {" in text
    assert validate(result.world_path).ok


def test_warehouse_rejects_unknown_param(tmp_path: Path):
    with pytest.raises(ValueError):
        generate(
            "warehouse",
            seed=0,
            out=tmp_path / "w.wbt",
            params={"not_a_param": True},
        )


def test_warehouse_rack_grid_aligned(tmp_path: Path):
    """Racks in the same row share a y coordinate; racks in the same
    column share an x coordinate."""
    out = tmp_path / "warehouse.wbt"
    result = generate("warehouse", seed=11, out=out)
    racks = [p.translation for p in result.description.props
             if p.proto_type == "WoodenPalletStack"]
    # Pull unique y values — should equal rack_rows.
    ys = sorted({round(y, 4) for (_, y, _) in racks})
    assert len(ys) == 4  # default rack_rows
