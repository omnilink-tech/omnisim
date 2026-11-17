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

"""Tool-set registry (AgentBench SPEC 4.1).

Two conditions ship, and the difference between them is the measurement:

``shell``
    A POSIX shell, ``read_file``, ``write_file``, ``list_dir``. **Nothing
    simulator-specific** -- not in the tool names, not in the descriptions, not
    in the schemas. This set is byte-identical for every simulator, which is
    what lets us answer "you gave our simulator a worse interface" with a hash
    instead of a promise.

``shell_plus_tools``
    The same four tools plus OmniSim's first-party agent surface, generated
    from the shipped MCP server's registry. The delta between the two
    conditions *measures* the value of that surface instead of asserting it --
    including the possibility that the delta is small, which would be a real
    and publishable result about our own harness.

A ``gazebo`` or ``webots`` set is a new module that builds a
:class:`~agentbench.runner.tools.manifest.ToolSet` and one line in
:data:`BUILDERS`. The rule from SPEC 4.2 binds every competitor as well: their
definitions must be generated from *their* published API, not paraphrased by
us.
"""

from __future__ import annotations

from dataclasses import dataclass, field

CONDITIONS = ("shell", "shell_plus_tools")

# The row's `condition` field uses the SPEC's spelling ("shell+tools"); the
# registry key is identifier-safe. Both are accepted on input.
CONDITION_ALIASES = {
    "shell": "shell",
    "shell_only": "shell",
    "shell+tools": "shell_plus_tools",
    "shell_plus_tools": "shell_plus_tools",
    "full_surface": "shell_plus_tools",
}

SPEC_CONDITION_NAME = {"shell": "shell", "shell_plus_tools": "shell+tools"}


@dataclass
class ToolResult:
    """What a tool handler returns.

    ``text`` is what the model sees; ``data`` is the full structured result for
    the trace. They differ on purpose: a 4 MB scene tree is useless in a
    prompt and essential in an audit.
    """

    text: str = ""
    data: object = None
    is_error: bool = False
    meta: dict = field(default_factory=dict)


def normalize_condition(name) -> str:
    key = (name or "shell").strip().lower()
    if key in CONDITION_ALIASES:
        return CONDITION_ALIASES[key]
    raise ValueError("unknown condition %r (have: %s)"
                     % (name, ", ".join(sorted(CONDITION_ALIASES))))


def resolve_sim(sim=None) -> str:
    """The simulator whose tool packaging a cell uses.

    Same resolution the adapter registry uses (``adapters.resolve``): the
    explicit argument, else ``$AGENTBENCH_SIM``, else ``omnisim``. The
    ``shell`` condition is byte-identical across simulators by construction
    (the fairness floor), so only ``shell_plus_tools`` ever branches on this.
    """
    import os
    return sim or os.environ.get("AGENTBENCH_SIM") or "omnisim"


def _load_builder(spec):
    """``"module.path:fn"`` -> the callable."""
    import importlib
    mod_name, fn_name = spec.split(":")
    return getattr(importlib.import_module(mod_name), fn_name)


def get_tool_set(condition, sandbox, sim=None):
    """Build the tool set for ``condition`` on ``sim``, bound to ``sandbox``.

    Resolution is registry-driven: ``BUILDERS["<sim>:<condition>"]`` wins over
    ``BUILDERS["<condition>"]``, so the Webots cell's ``shell+tools`` is the
    upstream-API bridge while every other simulator falls through to the
    default (OmniSim's MCP-generated surface). The ``shell`` baseline never
    branches -- it is the same four tools, byte-identical, on every simulator.
    """
    key = normalize_condition(condition)
    from agentbench.runner.tools import shell as shell_mod
    ts = shell_mod.build(sandbox)
    if key == "shell":
        return ts
    spec = (BUILDERS.get("%s:%s" % (resolve_sim(sim), key))
            or BUILDERS[key])
    return _load_builder(spec)(sandbox, base=ts)


BUILDERS = {
    "shell": "agentbench.runner.tools.shell:build",
    "shell_plus_tools": "agentbench.runner.tools.omnisim_harness:build",
    # The Webots cell's shell+tools condition (plan 2.2 / SPEC 4.1.1): the
    # shell set plus a bridge wrapping upstream R2025a's entire published
    # Supervisor + Robot function reference. The handler resolves its
    # endpoint per run from sandbox.webots_bridge_port, then
    # AGENTBENCH_WEBOTS_BRIDGE_PORT, then sandbox.harness_port (see
    # webots_bridge.bridge_endpoint); nothing per-run enters the manifest.
    "webots:shell_plus_tools": "agentbench.runner.tools.webots_bridge:build",
}

__all__ = ["BUILDERS", "CONDITIONS", "CONDITION_ALIASES",
           "SPEC_CONDITION_NAME", "ToolResult", "get_tool_set",
           "normalize_condition", "resolve_sim"]
