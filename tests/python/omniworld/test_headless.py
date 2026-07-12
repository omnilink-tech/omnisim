"""T1.7 headless-simulator validator tests.

These tests actually spawn OmniSim. When OmniSim is not discoverable
on the test host they skip, so CI lanes that run without OmniSim
stay green. The repo ships its own OmniSim under ``msys64/`` so
local runs from a fresh checkout usually cover this.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate
from omniworld.validation import discover_webots_bin, run_headless


REPO_ROOT = Path(__file__).resolve().parents[3]


def _webots_available() -> bool:
    return discover_webots_bin() is not None


requires_webots = pytest.mark.skipif(
    not _webots_available(),
    reason="OmniSim binary not discoverable (set OMNIWORLD_WEBOTS_BIN or OMNISIM_HOME)",
)


@requires_webots
def test_discover_webots_returns_executable():
    bin_path = discover_webots_bin()
    assert bin_path is not None
    assert bin_path.exists()


@requires_webots
@pytest.mark.parametrize("recipe", [
    "flat_ground",
    "outdoor_forest",
    "outdoor_desert",
    "warehouse",
    "urban_block",
    "indoor_apartment",
])
def test_every_shipped_biome_loads_headless(tmp_path: Path, recipe: str):
    """Each biome, at the default parameter set, must load under OmniSim
    with zero ERROR: lines. Warnings are tolerated by default because
    some inherited PROTOs warn on non-fatal issues."""
    assert recipe in list_recipes()
    out = tmp_path / f"{recipe}.wbt"
    generate(recipe, seed=42, out=out)
    result = run_headless(
        out,
        timeout_s=20.0,
        load_time_budget_s=15.0,
    )
    assert result.passed, (
        f"{recipe} failed headless load: "
        f"errors={result.errors[:3]}, load_time={result.load_time_s:.2f}s"
    )


@requires_webots
def test_headless_rejects_missing_world():
    with pytest.raises(FileNotFoundError):
        run_headless("/definitely/does/not/exist.wbt")


@requires_webots
def test_validate_with_sim_runs_headless_check(tmp_path: Path):
    out = tmp_path / "w.wbt"
    generate("flat_ground", seed=0, out=out)
    report = validate(out, with_sim=True, headless_timeout_s=20.0)
    names = [r.name for r in report.results]
    assert "headless_load" in names
    assert report.ok


def test_validate_without_sim_skips_headless(tmp_path: Path):
    """Default ``validate()`` must not spawn OmniSim — other tests use
    it heavily and cannot eat the process-spawn cost."""
    out = tmp_path / "w.wbt"
    generate("flat_ground", seed=0, out=out)
    report = validate(out, with_sim=False)
    names = [r.name for r in report.results]
    assert "headless_load" not in names


def test_validate_with_sim_reports_missing_webots(tmp_path: Path, monkeypatch):
    """If Webots is not discoverable the headless check must report
    that, not raise."""
    # Point every discovery path at nothing.
    monkeypatch.setenv("OMNIWORLD_WEBOTS_BIN", "/does/not/exist/webots")
    monkeypatch.delenv("OMNISIM_HOME", raising=False)

    # Suppress the walk-up-from-module-path fallback by monkeypatching
    # the discovery function to return None.
    import omniworld.validation.headless as headless_mod
    monkeypatch.setattr(headless_mod, "discover_webots_bin", lambda: None)

    out = tmp_path / "w.wbt"
    generate("flat_ground", seed=0, out=out)
    report = validate(out, with_sim=True)
    headless_check = [r for r in report.results if r.name == "headless_load"][0]
    assert not headless_check.passed
    assert "webots" in headless_check.detail.lower()


@requires_webots
def test_headless_catches_bad_world(tmp_path: Path):
    """A world with a broken EXTERNPROTO URL must fail the headless
    validator (it would pass the in-process ones because the URL is
    still a omnisim:// scheme, just pointing at nothing real)."""
    path = tmp_path / "bad.wbt"
    path.write_text(
        "#VRML_SIM R2025a utf8\n"
        "EXTERNPROTO \"omnisim://does/not/exist/Nothing.proto\"\n"
        "WorldInfo {\n"
        "  basicTimeStep 16\n"
        "  title \"broken\"\n"
        "}\n"
        "Viewpoint {\n"
        "  orientation 0 0 1 0\n"
        "  position 0 0 1\n"
        "}\n"
        "Nothing {\n"
        "}\n",
        encoding="utf-8",
    )
    result = run_headless(path, timeout_s=15.0, load_time_budget_s=10.0)
    assert not result.passed, "expected broken world to fail headless load"
    assert result.errors
