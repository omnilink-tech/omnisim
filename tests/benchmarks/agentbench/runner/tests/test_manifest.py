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

"""Tool-set manifests: hashing, diffability, and the SPEC 4.2 equality check.

The load-bearing test in this file is
:func:`test_harness_tools_equal_shipped_mcp_registry`. SPEC 4.2 promises that
OmniSim's ``shell+tools`` definitions are generated from the shipped MCP
server's registry, so "whatever helps the benchmark agent is what ships to
users, and vice versa". Without this assertion that promise is unenforceable
and we could quietly tune our own tool descriptions for the benchmark.
"""

from __future__ import annotations

import json

from agentbench.runner.tools import normalize_condition
from agentbench.runner.tools.manifest import ToolSet, ToolSpec, sha256_of
from agentbench.runner.tools.omnisim_harness import shipped_tool_definitions


def _spec(name="t", desc="d"):
    return ToolSpec(name=name, description=desc,
                    input_schema={"type": "object", "properties": {}},
                    handler=lambda a, t: None)


def test_hash_is_deterministic_and_handler_free():
    a = ToolSet(id="x", description="d", specs=[_spec()])
    b = ToolSet(id="x", description="d", specs=[_spec()])
    assert a.tools_sha256 == b.tools_sha256
    # A different handler must NOT change the hash: implementations may differ
    # per simulator while the model-facing definition stays identical.
    b.specs[0].handler = lambda a, t: "other"
    assert a.tools_sha256 == b.tools_sha256


def test_description_change_changes_the_hash():
    a = ToolSet(id="x", description="d", specs=[_spec()])
    b = ToolSet(id="x", description="d", specs=[_spec(desc="d2")])
    assert a.tools_sha256 != b.tools_sha256


def test_env_policy_is_outside_tools_sha_but_inside_manifest_sha():
    a = ToolSet(id="x", description="d", specs=[_spec()],
                env_policy={"mode": "allowlist"})
    b = ToolSet(id="x", description="d", specs=[_spec()],
                env_policy={"mode": "inherit"})
    assert a.tools_sha256 == b.tools_sha256      # fairness hash: tools only
    assert a.manifest_sha256 != b.manifest_sha256  # identity hash: everything


def test_manifest_sha_covers_the_dump_and_is_recomputable():
    ts = ToolSet(id="x", description="d", specs=[_spec()], notes=["n"])
    payload = ts.to_json()
    claimed = payload.pop("manifest_sha256")
    assert claimed == sha256_of(payload)


def test_dump_writes_valid_json(tmp_path):
    ts = ToolSet(id="x", description="d", specs=[_spec()])
    sha = ts.dump(tmp_path / "m.json")
    loaded = json.loads((tmp_path / "m.json").read_text(encoding="utf-8"))
    assert loaded["manifest_sha256"] == sha
    assert loaded["tools"][0]["name"] == "t"


# -- the shell baseline ----------------------------------------------------

def test_shell_set_is_the_four_baseline_tools(shell_tools):
    assert shell_tools.names == ["run_shell", "read_file", "write_file",
                                 "list_dir"]


def test_shell_definitions_mention_no_simulator(shell_tools):
    """SPEC 4.1: byte-identical across simulators means simulator-free text."""
    blob = json.dumps(shell_tools.definitions()).lower()
    for word in ("omnisim", "webots", "gazebo", "isaac", "mujoco", "wbt",
                 "harness", "robot", "simulator", "husky"):
        assert word not in blob, "%r leaked into the shell tool set" % word


def test_shell_hash_is_stable_across_sandboxes(tmp_path):
    """The fairness hash must not depend on the run's paths or ports."""
    from agentbench.runner.isolation import Sandbox
    from agentbench.runner.tools import get_tool_set
    a = get_tool_set("shell", Sandbox.create(tmp_path / "a",
                                             tmp_path / "a" / "s",
                                             ports=False))
    b = get_tool_set("shell", Sandbox.create(tmp_path / "b",
                                             tmp_path / "b" / "s",
                                             ports=False))
    assert a.tools_sha256 == b.tools_sha256
    # ...and so must the manifest hash, or a row's manifest id is per-run
    # noise instead of an identity.
    assert a.manifest_sha256 == b.manifest_sha256


# -- shell+tools -----------------------------------------------------------

def test_full_set_is_shell_plus_the_harness_tools(full_tools, shell_tools):
    assert full_tools.names[:4] == shell_tools.names
    assert len(full_tools) == len(shell_tools) + len(shipped_tool_definitions())


def test_harness_tools_equal_shipped_mcp_registry(full_tools):
    """SPEC 4.2's integrity property, asserted rather than promised."""
    shipped = {t["name"]: t for t in shipped_tool_definitions()}
    ours = {s.name: s.definition() for s in full_tools.specs
            if s.name in shipped}
    assert set(ours) == set(shipped), "tool NAMES drifted from the MCP server"
    for name, want in shipped.items():
        assert ours[name]["description"] == want["description"], (
            "description of %r differs from the shipped MCP server -- the "
            "benchmark must not tune its own tool text (SPEC 4.2)" % name)
        assert ours[name]["input_schema"] == want["input_schema"], (
            "input schema of %r differs from the shipped MCP server" % name)


def test_conditions_normalize_both_spellings():
    assert normalize_condition("shell+tools") == "shell_plus_tools"
    assert normalize_condition("SHELL") == "shell"


def test_tool_surface_text_lists_every_tool(full_tools):
    block = full_tools.tool_surface_text()
    for name in full_tools.names:
        assert name in block


# -- the checked-in manifests -----------------------------------------------

def test_checked_in_manifests_are_not_stale(shell_tools, full_tools, sandbox):
    """``runner/manifests/*.json`` is what a reviewer diffs, so it must match
    what the code actually builds. Regenerate with::

        python -m agentbench.runner dump-manifests
    """
    from pathlib import Path
    from agentbench.runner.tools import get_tool_set
    root = Path(__file__).resolve().parents[1] / "manifests"
    webots_tools = get_tool_set("shell+tools", sandbox, sim="webots")
    for ts, stem in ((shell_tools, "shell"),
                     (full_tools, "shell_plus_tools"),
                     (webots_tools, "webots_shell_plus_tools")):
        path = root / ("%s.json" % stem)
        if not path.is_file():
            continue          # not dumped in this checkout; nothing to be stale
        on_disk = json.loads(path.read_text(encoding="utf-8"))
        assert on_disk["tools_sha256"] == ts.tools_sha256, (
            "%s is stale -- re-run `python -m agentbench.runner "
            "dump-manifests`" % path)
        assert on_disk["manifest_sha256"] == ts.manifest_sha256, (
            "%s is stale -- re-run `python -m agentbench.runner "
            "dump-manifests`" % path)


def test_webots_cell_resolves_the_bridge_and_shell_stays_identical(tmp_path):
    """The sim-aware BUILDERS lookup: ``webots:shell_plus_tools`` is the
    upstream-API bridge; the ``shell`` baseline never branches on sim, so its
    fairness hash is one hash everywhere (SPEC 6.2.2)."""
    from agentbench.runner.isolation import Sandbox
    from agentbench.runner.tools import get_tool_set
    sb = Sandbox.create(tmp_path / "r", tmp_path / "r" / "s", ports=False)
    webots = get_tool_set("shell+tools", sb, sim="webots")
    omni = get_tool_set("shell+tools", sb, sim="omnisim")
    assert "wb_robot_step" in webots.names
    assert "wb_robot_step" not in omni.names
    assert webots.names[:4] == omni.names[:4]      # the shared shell floor
    assert (get_tool_set("shell", sb, sim="webots").tools_sha256
            == get_tool_set("shell", sb, sim="omnisim").tools_sha256)
