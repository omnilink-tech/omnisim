# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.
"""Live HTTP smoke test for every production agent on the shared runner.

Auto-discovers each ``agents/production/<name>/<name>_agent.py`` that adopts
``OmniLinkAgentRunner`` and runs it through ``_runner_smoke_child.py`` in its
own subprocess (one per agent — they each ship a ``tools`` package, so they
cannot share an interpreter). The child boots the real tool-callback HTTP
server and exercises /tool, /status, /activity. No OMNI_KEY / network needed.

This is the regression guard for the runner-consolidation migrations: if a
migrated agent stops wiring dispatch/status/tools correctly, its subprocess
fails and so does this test.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[2]
_PROD = _REPO / "agents" / "production"
_CHILD = Path(__file__).resolve().parent / "_runner_smoke_child.py"


def _discover_runner_agents() -> list[Path]:
    found: list[Path] = []
    for agent_py in sorted(_PROD.glob("*/*_agent.py")):
        try:
            text = agent_py.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "OmniLinkAgentRunner" in text:
            found.append(agent_py)
    return found


_AGENTS = _discover_runner_agents()


@pytest.mark.skipif(not _AGENTS, reason="no agents on the shared runner yet")
@pytest.mark.parametrize("agent_py", _AGENTS, ids=lambda p: p.parent.name)
def test_runner_agent_http_smoke(agent_py: Path) -> None:
    proc = subprocess.run(
        [sys.executable, str(_CHILD), str(agent_py)],
        capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode == 0, (
        f"{agent_py.parent.name} smoke failed (rc={proc.returncode})\n"
        f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    assert "SMOKE_PASS" in proc.stdout, proc.stdout
