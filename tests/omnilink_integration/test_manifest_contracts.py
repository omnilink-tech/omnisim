# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Contract checks for the `agents/production/<name>/omnilink.json` manifests.

These are static-analysis tests — they do **not** launch Webots and
they do not require an Omni Key. They protect against the silent
drift modes that hurt most:

* a manifest references a `.wbt` world that was renamed or deleted
* a manifest's `bridge_port` no longer matches any
  `*_omnilink_bridge` controller's `BRIDGE_PORT` literal
* an agent script no longer parses as Python (would crash run-agent)
* an agent script has no `__main__` block (would no-op when launched)

Adding a new manifest under `agents/production/*/omnilink.json`
automatically picks up coverage here — there is no per-agent test to
keep in sync.
"""

from __future__ import annotations

import ast
import json
import re
import sys
from pathlib import Path
from typing import Dict, Iterator, Tuple

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
AGENTS_DIR = REPO_ROOT / "agents" / "production"
CONTROLLERS_DIR = REPO_ROOT / "projects" / "samples" / "demos" / "controllers"

# Manifests with `tags` containing "first-party" are held to a stricter
# contract: every field must be present, including bridge_port + agent_port_env.
_FIRST_PARTY_TAG = "first-party"


def _discover_manifests() -> Iterator[Tuple[str, Path, Dict]]:
    for path in sorted(AGENTS_DIR.glob("*/omnilink.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        name = data.get("name") or path.parent.name
        yield name, path, data


_MANIFESTS = list(_discover_manifests())


# ---------------------------------------------------------------------------
# Per-manifest contract checks
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_has_required_keys(name: str, path: Path, data: Dict):
    for key in ("name", "description", "agent_script", "agent_port", "display_name"):
        assert key in data, f"{path}: missing required key {key!r}"
    assert isinstance(data["description"], str) and data["description"], (
        f"{path}: description must be non-empty string"
    )


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_world_resolves(name: str, path: Path, data: Dict):
    world = data.get("world")
    if world is None:
        pytest.skip(f"{name} is orchestrator-only")
    full = REPO_ROOT / world
    assert full.exists(), f"{name}: world missing at {full}"
    assert full.suffix in (".wbt", ".omniworld"), f"{name}: world is not a .wbt file"


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_agent_script_parses(name: str, path: Path, data: Dict):
    script = data.get("agent_script")
    if script is None:
        pytest.skip(f"{name} has no agent_script")
    full = REPO_ROOT / script
    assert full.exists(), f"{name}: agent_script missing at {full}"
    try:
        ast.parse(full.read_text(encoding="utf-8"), filename=str(full))
    except SyntaxError as exc:
        pytest.fail(f"{name}: agent_script has syntax error: {exc}")


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_agent_script_has_main_block(name: str, path: Path, data: Dict):
    script = data.get("agent_script")
    if script is None:
        pytest.skip(f"{name} has no agent_script")
    src = (REPO_ROOT / script).read_text(encoding="utf-8")
    assert '__name__ == "__main__"' in src or "__name__ == '__main__'" in src, (
        f"{name}: agent_script has no __main__ block — run-agent would be a no-op"
    )


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_bridge_port_in_range(name: str, path: Path, data: Dict):
    port = data.get("bridge_port")
    if port is None:
        pytest.skip(f"{name} is orchestrator-only (no bridge)")
    assert isinstance(port, int) and 1024 <= port <= 65535, (
        f"{name}: bridge_port {port!r} out of valid range"
    )


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_agent_port_in_range(name: str, path: Path, data: Dict):
    port = data.get("agent_port")
    assert isinstance(port, int) and 1024 <= port <= 65535, (
        f"{name}: agent_port {port!r} out of valid range"
    )


# ---------------------------------------------------------------------------
# Cross-repo contract: manifest bridge_port matches a real controller
# ---------------------------------------------------------------------------


def _bridge_ports_declared_in_controllers() -> Dict[int, Path]:
    """Walk every *_bridge controller and harvest its bridge port.

    Returns ``{port: controller_path}``. Multiple controllers may share
    a port (e.g. several Husky scenes), which is fine — the test only
    cares whether the manifest's port appears at all.

    Controllers declare their port in three patterns, so the heuristic
    is intentionally broad: harvest any 4-digit number in the
    OmniSim-bridge ranges 6000–6999 and 8000–8999 that appears anywhere
    in the controller source (``BRIDGE_PORT = 6060``,
    ``parser.add_argument("--port", default=6070)``,
    ``BRIDGE_PORT = int(os.environ.get("X", "6080"))``).

    Source literals alone are not sufficient. A shared controller like
    ``omnilink_mobile_bridge`` hardcodes one default (8765) but is driven
    onto other ports by a world's ``controllerArgs [ "--port" "8865" ]`` --
    which is exactly how any multi-robot world assigns a port per robot.
    So worlds are harvested too; otherwise every such manifest fails a
    check whose stated intent it actually satisfies.
    """
    result: Dict[int, Path] = {}
    port_pattern = re.compile(r"\b([68][0-9]{3})\b")

    if CONTROLLERS_DIR.exists():
        for ctrl_dir in CONTROLLERS_DIR.iterdir():
            if not ctrl_dir.is_dir():
                continue
            if "bridge" not in ctrl_dir.name and "twin" not in ctrl_dir.name:
                continue
            for py in ctrl_dir.glob("*.py"):
                try:
                    src = py.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    continue
                for match in port_pattern.finditer(src):
                    result.setdefault(int(match.group(1)), py)

    # Ports a world hands to a bridge controller via controllerArgs.
    args_pattern = re.compile(r'"--port"\s+"(\d{4})"')
    worlds_root = REPO_ROOT / "projects"
    if worlds_root.exists():
        for wbt in [*worlds_root.rglob("*.omniworld"), *worlds_root.rglob("*.wbt")]:
            try:
                src = wbt.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            if "bridge" not in src:
                continue
            for match in args_pattern.finditer(src):
                result.setdefault(int(match.group(1)), wbt)
    return result


_PORTS_IN_CONTROLLERS = _bridge_ports_declared_in_controllers()


@pytest.mark.parametrize("name,path,data", _MANIFESTS, ids=lambda t: t if isinstance(t, str) else "")
def test_manifest_bridge_port_matches_a_controller(name: str, path: Path, data: Dict):
    port = data.get("bridge_port")
    if port is None:
        pytest.skip(f"{name} is orchestrator-only (no bridge)")
    if not _PORTS_IN_CONTROLLERS:
        pytest.skip("no *_omnilink_bridge controllers found in projects/samples/demos/controllers")
    assert port in _PORTS_IN_CONTROLLERS, (
        f"{name}: manifest bridge_port={port} doesn't match any controller's BRIDGE_PORT.\n"
        f"  Controllers declare: {sorted(_PORTS_IN_CONTROLLERS)}.\n"
        f"  Did the controller's port literal change without updating the manifest?"
    )


# ---------------------------------------------------------------------------
# Cross-repo contract: agent ports don't collide with other agents
# ---------------------------------------------------------------------------


def test_no_two_manifests_share_an_agent_port():
    by_port: Dict[int, str] = {}
    for name, _, data in _MANIFESTS:
        port = data["agent_port"]
        if port in by_port:
            pytest.fail(
                f"agent_port {port} is claimed by both {by_port[port]!r} and {name!r} — "
                "two agents cannot bind the same tool-callback port at the same time"
            )
        by_port[port] = name


def test_first_party_agents_carry_full_env_var_names():
    for name, path, data in _MANIFESTS:
        if _FIRST_PARTY_TAG not in (data.get("tags") or []):
            continue
        for key in ("agent_port_env", "omnilink_agent_name"):
            assert data.get(key), (
                f"{name}: first-party agent must declare {key!r} in {path.name}"
            )
