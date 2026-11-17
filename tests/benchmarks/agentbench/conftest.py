"""Pytest boundaries for AgenticSimBench.

The benchmark preserves complete campaign workspaces under ``results/``.
Those workspaces contain copies of repository tests and controller programs;
collecting them recursively re-runs copied tests under the wrong import root.

Four files are genuine live simulator gates.  They launch long (up to
15-minute) engine runs and are deliberately opt-in so a normal benchmark
unit-test pass remains bounded and useful.
"""

from __future__ import annotations

from pathlib import Path

import pytest


collect_ignore = ["results"]

LIVE_FILES = frozenset({
    "adapters/omnisim/test_r1_discriminates_omnisim.py",
    "adapters/omnisim/test_r1_placement_omnisim.py",
    "adapters/webots/test_r1_discriminates_webots.py",
    "adapters/webots/test_r4_discriminates_webots.py",
})


def pytest_addoption(parser):
    parser.addoption(
        "--agentbench-live",
        action="store_true",
        default=False,
        help="run AgenticSimBench tests that launch long-lived simulators",
    )


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "agentbench_live: launches a real simulator and may take many minutes",
    )


def pytest_collection_modifyitems(config, items):
    root = Path(__file__).resolve().parent
    live = pytest.mark.agentbench_live
    skip = pytest.mark.skip(
        reason="live simulator gate; pass --agentbench-live to run it",
    )
    enabled = config.getoption("--agentbench-live")
    for item in items:
        try:
            rel = Path(item.path).resolve().relative_to(root).as_posix()
        except (OSError, ValueError):
            continue
        if rel not in LIVE_FILES:
            continue
        item.add_marker(live)
        if not enabled:
            item.add_marker(skip)
