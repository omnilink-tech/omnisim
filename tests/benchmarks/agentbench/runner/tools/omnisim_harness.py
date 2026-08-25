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

"""``shell_plus_tools``: shell + OmniSim's harness surface as native tools.

SPEC 4.2's integrity property, implemented literally: the tool definitions are
**generated from the shipped first-party MCP server's own registry**
(``packages/omnisim-mcp`` ``TOOLS``), and the handlers are the shipped handlers,
called with the module's harness URL pointed at this run's port. We therefore
cannot quietly hand-tune our own tool descriptions for the benchmark -- whatever
helps the benchmark agent is what ships to users, and a test asserts the names,
descriptions and schemas are *equal* to the shipped ones.

Two honest deviations from what an MCP client would see, both recorded in the
manifest's ``notes`` so they show up in a reviewer's diff:

1. **Images are not passed through as images.** ``screenshot`` with no ``path``
   returns an inline base64 PNG over MCP. Here the bytes are written next to
   the run and the model gets a pointer. A vision-capable agent is therefore
   *worse off* in this condition than a real MCP user -- that is a limitation
   of this runner, not of the shipped surface, and it biases the
   ``shell+tools`` delta **downward** (against us), which is the safe
   direction for a vendor-run benchmark.
2. **The harness is not started for us.** SPEC 2.3 makes starting it part of
   task A1, so these tools point at a port that nothing is listening on until
   the agent starts a harness there. ``harness_status`` reports exactly that,
   and the assigned port is stated factually in the system prompt's tool
   surface block.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path

from agentbench.common.paths import REPO
from agentbench.runner.tools import ToolResult
from agentbench.runner.tools.manifest import ToolSet, ToolSpec

MCP_SERVER_PY = (REPO / "packages" / "omnisim-mcp" / "src" / "omnisim_mcp"
                 / "server.py")

_server = None


def load_mcp_server():
    """Import the shipped MCP server module by path (cached).

    By path rather than by package name because ``packages/omnisim-mcp/src`` is
    not on ``sys.path`` in a fresh clone and adding it would be a side effect
    on the whole process.
    """
    global _server
    if _server is not None:
        return _server
    import importlib.util
    import sys
    if not MCP_SERVER_PY.is_file():
        raise FileNotFoundError(
            "the shipped MCP server is missing at %s; the shell_plus_tools "
            "condition generates its tool definitions from it and must not "
            "fall back to hand-written copies (SPEC 4.2)" % MCP_SERVER_PY)
    spec = importlib.util.spec_from_file_location(
        "agentbench_omnisim_mcp_server", MCP_SERVER_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("agentbench_omnisim_mcp_server", mod)
    spec.loader.exec_module(mod)
    _server = mod
    return mod


def shipped_tool_definitions() -> list[dict]:
    """``[{name, description, input_schema}]`` from the shipped registry."""
    server = load_mcp_server()
    return [{"name": name, "description": desc, "input_schema": schema}
            for name, (_h, desc, schema) in server.TOOLS.items()]


def _render(name, blocks, run_dir: Path):
    """MCP content blocks -> (text for the model, data for the trace)."""
    texts, saved = [], []
    for i, b in enumerate(blocks or []):
        if b.get("type") == "text":
            texts.append(b.get("text", ""))
        elif b.get("type") == "image":
            raw = base64.b64decode(b.get("data", "") or "")
            dest = run_dir / "harness_images"
            dest.mkdir(parents=True, exist_ok=True)
            out = dest / ("%s_%d.png" % (name, len(list(dest.glob("*.png")))))
            out.write_bytes(raw)
            saved.append(str(out))
            texts.append("[image: %d bytes written to %s -- this runner does "
                         "not forward images into the conversation]"
                         % (len(raw), out))
        else:
            texts.append(json.dumps(b, default=str)[:2000])
    text = "\n".join(t for t in texts if t)
    return text, {"blocks": len(blocks or []), "images_saved": saved,
                  "text": text}


def _make_handler(name, sandbox):
    def handler(args, timeout_s):
        server = load_mcp_server()
        # The shipped module resolves both of these at call time, so pointing
        # them per-run needs no fork of the server.
        server.DEFAULT_HARNESS = sandbox.harness_url
        server.HTTP_TIMEOUT_S = float(max(1.0, timeout_s))
        fn = server.TOOLS[name][0]
        try:
            blocks = fn(dict(args or {}))
        except KeyError as exc:
            return ToolResult(text="missing required argument: %s" % exc,
                              is_error=True)
        except Exception as exc:                          # noqa: BLE001
            return ToolResult(text="%s: %s" % (type(exc).__name__, exc),
                              is_error=True)
        text, data = _render(name, blocks, Path(sandbox.run_dir))
        low = text.lower()
        is_error = ("cannot reach the omnisim harness" in low
                    or '"reachable": false' in low.replace(" ", ""))
        return ToolResult(text=text or "(empty result)", data=data,
                          is_error=is_error)
    return handler


def build(sandbox, base: ToolSet | None = None) -> ToolSet:
    """``base`` (the shell set) + the 18 shipped harness tools."""
    if base is None:
        from agentbench.runner.tools import shell as shell_mod
        base = shell_mod.build(sandbox)
    server = load_mcp_server()
    version = (server.SERVER_INFO.get("version")
               if isinstance(getattr(server, "SERVER_INFO", None), dict)
               else None)
    specs = list(base.specs)
    for name, (_fn, desc, schema) in server.TOOLS.items():
        specs.append(ToolSpec(
            name=name, description=desc, input_schema=schema,
            handler=_make_handler(name, sandbox),
            source="omnisim-mcp:%s (generated from the shipped TOOLS "
                   "registry)" % (version or "unknown")))
    return ToolSet(
        id="shell_plus_tools",
        description=("The shell baseline plus OmniSim's first-party "
                     "agent-facing harness surface, generated from the "
                     "shipped MCP server's own registry (SPEC 4.2)."),
        specs=specs,
        env_policy=sandbox.env_policy(),
        notes=list(base.notes) + [
            "The %d harness tool definitions are GENERATED from "
            "packages/omnisim-mcp TOOLS (version %s) and a test asserts "
            "equality, so they cannot be tuned for the benchmark."
            % (len(server.TOOLS), version),
            # Deliberately no port numbers here: the manifest hash must be
            # stable across runs, and the per-run pair is recorded in the
            # trace's sandbox block instead.
            "The harness is NOT pre-started: SPEC 2.3 makes starting it part "
            "of the task. These tools point at the run's assigned harness "
            "port on 127.0.0.1 and fail until the agent starts a harness "
            "there.",
            "Images are NOT forwarded into the conversation: a screenshot is "
            "written next to the run and the model gets a path. A "
            "vision-capable agent is worse off here than a real MCP user, "
            "which biases the shell+tools delta downward (against us).",
        ])
