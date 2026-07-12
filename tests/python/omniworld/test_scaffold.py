"""T1.1 scaffold tests.

Covers:

- Package imports, ``__version__`` surfaces.
- Registry: ``flat_ground`` is registered; unknown names raise.
- ``generate``: writes a ``.wbt`` and a ``.seed.json`` next to it.
- Determinism: two runs with the same seed+params produce byte-identical
  output.
- Seed manifest: SHA256 matches the file on disk; schema version set.
- Validation: a default-stub world passes all in-process checks.
- Validation: a world with a spoofed ``https://`` reference fails
  ``asset_locality``.
- CLI: ``--version``, ``list-recipes``, ``generate``, ``validate`` all
  exit 0 on the happy path.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

import omniworld
from omniworld import __version__, generate, list_recipes, validate
from omniworld.core.manifest import MANIFEST_SCHEMA_VERSION, manifest_path_for
from omniworld.core.registry import get_recipe
from omniworld.core.rng import derive_seed


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = REPO_ROOT / "scripts" / "dev" / "omniworld.py"


def test_package_imports():
    assert __version__
    assert isinstance(omniworld.list_recipes(), list)


def test_flat_ground_registered():
    recipes = list_recipes()
    assert "flat_ground" in recipes


def test_unknown_recipe_raises():
    with pytest.raises(KeyError):
        get_recipe("does_not_exist")


def test_generate_writes_both_artifacts(tmp_path: Path):
    out = tmp_path / "w.wbt"
    result = generate("flat_ground", seed=7, out=out)
    assert out.exists()
    assert result.manifest_path.exists()
    assert result.manifest_path == manifest_path_for(out)
    # World starts with the VRML signature.
    text = out.read_text(encoding="utf-8")
    assert text.startswith("#VRML_SIM")
    # Manifest is valid JSON matching our schema version.
    payload = json.loads(result.manifest_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == MANIFEST_SCHEMA_VERSION
    assert payload["library_version"] == __version__
    assert payload["recipe"] == "flat_ground"
    assert payload["seed"] == 7


def test_generate_is_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("flat_ground", seed=42, out=a)
    generate("flat_ground", seed=42, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_manifest_is_location_independent(tmp_path: Path):
    """Same (recipe, seed, params, output basename) => byte-identical
    manifest, regardless of parent directory. Required for CI regression
    diffs to compare cleanly."""
    d1 = tmp_path / "machine_a"
    d2 = tmp_path / "a_totally_different_path" / "nested"
    d1.mkdir()
    d2.mkdir(parents=True)
    r1 = generate("flat_ground", seed=5, out=d1 / "w.wbt")
    r2 = generate("flat_ground", seed=5, out=d2 / "w.wbt")
    assert (
        r1.manifest_path.read_bytes() == r2.manifest_path.read_bytes()
    ), "manifests diverged with different parent dirs"


def test_generate_seed_affects_manifest_but_not_stub_output(tmp_path: Path):
    """The flat_ground stub ignores its seed (Tier 1 behavior), so the
    world bytes are identical for different seeds. The manifest still
    records the seed, so the manifests differ."""
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("flat_ground", seed=1, out=a)
    generate("flat_ground", seed=2, out=b)
    assert a.read_bytes() == b.read_bytes()
    ma = json.loads(manifest_path_for(a).read_text(encoding="utf-8"))
    mb = json.loads(manifest_path_for(b).read_text(encoding="utf-8"))
    assert ma["seed"] != mb["seed"]


def test_manifest_sha_matches_file(tmp_path: Path):
    import hashlib

    out = tmp_path / "w.wbt"
    result = generate("flat_ground", seed=3, out=out)
    file_sha = hashlib.sha256(out.read_bytes()).hexdigest()
    assert result.manifest.world_sha256 == file_sha


def test_derive_seed_stable_and_varies_by_name():
    assert derive_seed(42, "heightmap") == derive_seed(42, "heightmap")
    assert derive_seed(42, "heightmap") != derive_seed(42, "scatter")
    # Different parent seeds produce different children for the same name.
    assert derive_seed(1, "x") != derive_seed(2, "x")


def test_validate_stub_world(tmp_path: Path):
    out = tmp_path / "w.wbt"
    generate("flat_ground", seed=0, out=out)
    report = validate(out)
    assert report.ok, report.format()


def test_validate_catches_remote_reference(tmp_path: Path):
    out = tmp_path / "bad.wbt"
    out.write_text(
        "#VRML_SIM R2025a utf8\n"
        "EXTERNPROTO \"https://example.com/thing.proto\"\n"
        "WorldInfo {\n"
        "}\n",
        encoding="utf-8",
    )
    report = validate(out)
    assert not report.ok
    names = [r.name for r in report.results if not r.passed]
    assert "asset_locality" in names


def test_generate_with_spawn_urdf(tmp_path: Path):
    out = tmp_path / "w.wbt"
    result = generate(
        "flat_ground",
        seed=11,
        out=out,
        params={
            "spawn_urdf": "../robots/cube_bot.urdf",
            "spawn_controller": "omnibot_agent",
        },
    )
    text = out.read_text(encoding="utf-8")
    assert "URDFRobot {" in text
    assert "cube_bot.urdf" in text
    # Validate still passes with a spawn present.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_unknown_param_rejected(tmp_path: Path):
    with pytest.raises(ValueError):
        generate(
            "flat_ground",
            seed=0,
            out=tmp_path / "w.wbt",
            params={"this_key_does_not_exist": True},
        )


# --- CLI smoke tests ---------------------------------------------------


def _run_cli(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(LAUNCHER), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def test_cli_version():
    r = _run_cli("--version")
    assert r.returncode == 0
    assert r.stdout.strip() == __version__


def test_cli_list_recipes():
    r = _run_cli("list-recipes")
    assert r.returncode == 0
    assert "flat_ground" in r.stdout.splitlines()


def test_cli_generate_then_validate(tmp_path: Path):
    out = tmp_path / "cli.wbt"
    r = _run_cli("generate", "flat_ground", "--seed", "9", "--out", str(out))
    assert r.returncode == 0, r.stderr
    assert out.exists()
    assert manifest_path_for(out).exists()

    r2 = _run_cli("validate", str(out))
    assert r2.returncode == 0, r2.stderr
    assert "ok" in r2.stdout


def test_cli_describe():
    r = _run_cli("describe", "flat_ground")
    assert r.returncode == 0, r.stderr
    payload = json.loads(r.stdout)
    assert payload["name"] == "flat_ground"
    assert "default_params" in payload
