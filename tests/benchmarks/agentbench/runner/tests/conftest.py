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

"""Test fixtures for the runner.

Puts ``tests/benchmarks`` on ``sys.path`` (the ``agentbench`` package root) and
provides a real :class:`Sandbox` over a tmp dir. ``ports=False`` everywhere:
these tests must not probe TCP ports, both because it is slow on Windows (a
closed port takes ~2 s to refuse) and because a unit test that needs the
network is not a unit test.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[3]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.runner.isolation import Sandbox          # noqa: E402
from agentbench.runner.tools import get_tool_set         # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


@pytest.fixture()
def sandbox(tmp_path):
    return Sandbox.create(tmp_path / "run", tmp_path / "run" / "scratch",
                          ports=False)


@pytest.fixture()
def shell_tools(sandbox):
    return get_tool_set("shell", sandbox)


@pytest.fixture()
def full_tools(sandbox):
    return get_tool_set("shell+tools", sandbox)


@pytest.fixture()
def scripts_dir():
    return SCRIPTS
