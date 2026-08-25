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

"""Gates for the failure mode that keeps getting through: a SILENT DOWNGRADE.

Every check I had was green while the warehouse demo had no LLM in it at
all. The bridges catch relay-setup failures broadly and fall back to the
offline regex router, so a fatal wiring bug does not crash, does not log an
error a casual run would notice, and leaves a demo that still answers chat --
just stupidly. `python -m py_compile` passed, the unit suite passed, and the
smoke worlds passed, because none of them constructs the relay.

The specific bug: a commit added a module-scope `import profile_sync` but left
the function-local ones in place. A function-local import binds that name as a
local for the WHOLE function body, so the new first use at the top of the
function read an unbound local and raised on every launch, in both bridges.

These tests are cheap, need no simulator, and fail loudly on that class.
"""

from __future__ import annotations

import ast
import os
import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parents[3]
BRIDGES = [
    REPO / "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py",
    REPO / "projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py",
]

# The platform rejects an agentName outside this set with INVALID_AGENT_NAME,
# which surfaces as a robot that exists but cannot speak.
AGENT_NAME_RE = re.compile(r"[A-Za-z0-9 _.-]{1,64}")


def _functions(tree: ast.AST):
    return [n for n in ast.walk(tree)
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]


@pytest.mark.parametrize("path", BRIDGES, ids=lambda p: p.name)
def test_no_use_before_function_local_import(path: pathlib.Path) -> None:
    """A name used above its own function-local import is an UnboundLocalError.

    This is the exact shape that shipped a regex-only demo to main.
    """
    if not path.exists():                       # pragma: no cover
        pytest.skip(f"{path} not present")
    tree = ast.parse(path.read_text(encoding="utf-8"))
    defects = []
    for fn in _functions(tree):
        first_local_import: dict[str, int] = {}
        for node in ast.walk(fn):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                for alias in node.names:
                    name = alias.asname or alias.name.split(".")[0]
                    first_local_import.setdefault(name, node.lineno)
        for node in ast.walk(fn):
            if (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
                    and node.id in first_local_import
                    and node.lineno < first_local_import[node.id]):
                defects.append(
                    f"{path.name}:{node.lineno} uses {node.id!r} before its "
                    f"function-local import on line {first_local_import[node.id]}"
                )
    assert not defects, "UnboundLocalError waiting to happen:\n" + "\n".join(defects)


@pytest.mark.parametrize("path", BRIDGES, ids=lambda p: p.name)
def test_profile_sync_is_imported_at_module_scope(path: pathlib.Path) -> None:
    """setup_omnilink_relay() calls profile_sync.agent_name_for() near its top,
    so the name must exist at module scope.

    The first fix for the UnboundLocalError deleted the function-local imports
    with a regex that matched any indentation -- and removed the module-scope
    one too, turning the UnboundLocalError into a plain NameError with the
    same silent-downgrade symptom. Caught only by running the code. This gate
    is the cheap version of that.
    """
    if not path.exists():                       # pragma: no cover
        pytest.skip(f"{path} not present")
    tree = ast.parse(path.read_text(encoding="utf-8"))

    def module_level(node):
        """Yield statements reachable without entering a def/class.

        Walking the whole tree and excluding function bodies is quadratic on a
        5000-line controller (it hung for minutes); descending selectively is
        linear and is what "module scope" actually means.
        """
        for stmt in getattr(node, "body", []):
            yield stmt
            if isinstance(stmt, (ast.Try, ast.If, ast.With)):
                yield from module_level(stmt)
                for extra in ("handlers", "orelse", "finalbody"):
                    for sub in getattr(stmt, extra, []) or []:
                        yield from module_level(sub) if hasattr(sub, "body") else ()
                        if isinstance(sub, ast.stmt) and not hasattr(sub, "body"):
                            yield sub

    at_module_scope = any(
        isinstance(node, ast.ImportFrom)
        and node.module == "_omnilink_relay"
        and any((a.asname or a.name) == "profile_sync" for a in node.names)
        for node in module_level(tree)
    )
    assert at_module_scope, (
        f"{path.name} never imports profile_sync at module scope; "
        "setup_omnilink_relay() would raise NameError and silently downgrade "
        "the whole demo to the regex router."
    )


@pytest.mark.parametrize("path", BRIDGES, ids=lambda p: p.name)
def test_profile_push_uses_the_relay_tool_list(path: pathlib.Path) -> None:
    """The profile must advertise the tools the RELAY registers, not the
    bridge's own pre-relay list.

    The relay appends get_action_history after the bridge builds its tools, so
    pushing the bridge's list silently drops the anti-fabrication tool from the
    web UI and from every delegation round trip.
    """
    if not path.exists():                       # pragma: no cover
        pytest.skip(f"{path} not present")
    src = path.read_text(encoding="utf-8")
    assert "tool_defs=[t.to_definition() for t in tools]" not in src, (
        f"{path.name} pushes its pre-relay tool list; use relay.tool_defs so "
        "relay-registered tools (get_action_history) reach the platform."
    )


def test_relay_registers_action_history_and_exposes_it() -> None:
    """get_action_history must be both callable AND advertised."""
    from omnisim_bridges.relay import OmniLinkRelay
    from omnisim_bridges.tool import Tool

    probe = Tool("drive_forward", "move", {"type": "object", "properties": {}},
                 lambda a: {"ok": True})
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="WiringProbe",
        main_task="test",
        tools=[probe],
        usage_enabled=False,
        memory_enabled=False,
    )
    assert "get_action_history" in relay.tools, "journal tool not callable"
    names = [d["name"] for d in relay.tool_defs]
    assert "get_action_history" in names, (
        "journal tool missing from the pushed definitions — the fix would work "
        "on /prompt and be invisible everywhere else"
    )
    assert "drive_forward" in names, "relay dropped a bridge tool"


@pytest.mark.parametrize("tag,expect_suffix", [
    ("", False),
    ("hardtest", True),
    ("weird/tag with spaces!", True),
])
def test_agent_name_is_always_platform_legal(tag: str, expect_suffix: bool,
                                             monkeypatch) -> None:
    """A tagged identity must still be accepted by the chat API.

    An earlier separator ('~') passed profile CREATE and was rejected by chat,
    so the tag produced a robot that existed and could not talk.
    """
    from omnisim_bridges import profile_sync
    monkeypatch.setenv("OMNILINK_AGENT_TAG", tag)
    name = profile_sync.agent_name_for("tug_a")
    assert AGENT_NAME_RE.fullmatch(name), f"{name!r} would be INVALID_AGENT_NAME"
    assert name.startswith("OmniSim-tug_a")
    assert (name != "OmniSim-tug_a") == expect_suffix


def test_tag_isolates_intent_state_too(monkeypatch) -> None:
    """The tag must isolate commitments, not just identity and memory.

    Otherwise a tagged test can leave a standing constraint in the shipped
    demo's state file and take its tug offline on the next plain launch.
    """
    from omnisim_bridges import intents

    monkeypatch.delenv("OMNILINK_AGENT_TAG", raising=False)
    production = intents._default_state_path("tug_a")
    monkeypatch.setenv("OMNILINK_AGENT_TAG", "hardtest")
    tagged = intents._default_state_path("tug_a")
    assert production != tagged, "a tagged run shares the demo's intent store"
    assert os.path.basename(production) == "intents_tug_a.json"
    assert "hardtest" in os.path.basename(tagged)
