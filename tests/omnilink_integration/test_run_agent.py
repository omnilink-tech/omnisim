# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Regression tests for `scripts/dev/omnisim_run_agent.py`.

Catches the most likely silent breakage modes of the agent registry:

* registry references a world `.wbt` that no longer exists
* registry references an agent script that no longer exists
* `--list-json` output stops being parseable

These tests do NOT actually launch Webots — they only check that the
registry stays in sync with the on-disk layout. Adding a new entry to
`AGENTS` automatically picks up coverage here.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "dev" / "omnisim_run_agent.py"

sys.path.insert(0, str(SCRIPT.parent))

from omnisim_run_agent import AGENTS, AgentSpec  # noqa: E402


def test_registry_is_non_empty():
    assert AGENTS, "agent registry should not be empty"


@pytest.mark.parametrize("name,spec", sorted(AGENTS.items()))
def test_world_path_exists_when_set(name: str, spec: AgentSpec):
    if spec.world is None:
        pytest.skip(f"{name} is orchestrator-only (no world)")
    world_path = REPO_ROOT / spec.world
    assert world_path.exists(), f"{name}: world file missing at {world_path}"


@pytest.mark.parametrize("name,spec", sorted(AGENTS.items()))
def test_agent_script_exists_when_set(name: str, spec: AgentSpec):
    if spec.agent_script is None:
        pytest.skip(f"{name} has no agent runner script in registry")
    script_path = REPO_ROOT / spec.agent_script
    assert script_path.exists(), f"{name}: agent script missing at {script_path}"


def test_list_json_is_parseable():
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--list-json"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
        timeout=30,
    )
    payload = json.loads(proc.stdout)
    # Every entry must carry the four registry fields callers depend on.
    for name, entry in payload.items():
        for required in ("world", "bridge_port", "agent_script", "agent_port", "description"):
            assert required in entry, f"{name}: missing field {required!r}"


def test_help_runs_without_args():
    proc = subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    # No --agent given: prints help, returns 1.
    assert proc.returncode == 1
    assert "omnisim-runner" in proc.stdout or "omnisim-runner" in proc.stderr
