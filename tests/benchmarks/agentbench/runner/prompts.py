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

"""System prompt assembly (AgentBench SPEC 4.4).

"The system prompt is one shared file; the only per-sim text is a factual
'here is your tool surface' block generated from §4.2." So:

* :data:`SYSTEM_PROMPT_PATH` is that one shared file. It is simulator-neutral
  and task-neutral -- **no per-simulator prompt tuning, ever**, because that is
  the mechanism by which vendor benchmarks cheat.
* the tool-surface block is generated from the tool-set manifest, not written;
* the environment block states per-run facts (working directory, repository
  root, assigned ports) in a fixed template.

Both hashes go in the row: ``system_prompt_sha`` for the shared file (so a
reviewer can confirm two cells shared one prompt) and ``system_full_sha`` for
the composed text (so a reviewer can confirm what was actually sent).

The task prompt is **never** touched here. It arrives verbatim from
``tasks/<id>/prompt.txt`` and is sent as the first user message, byte-identical
across simulators.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

SYSTEM_PROMPT_PATH = Path(__file__).resolve().parent / "prompts" / "system.txt"

TOOL_SURFACE_TEMPLATE = """
## Your tools

You have exactly these {n} tools, and nothing else:

{tools}

There is no tool for asking a person a question. There is no web access.
"""

ENVIRONMENT_TEMPLATE = """
## Your environment

- Your working directory (writable, initially {initial}): {scratch}
- The repository (readable, do not modify it): {repo}
- Reserved for you if you need to start a service: TCP ports {port_a} and
  {port_b} on 127.0.0.1. Nothing is listening on them yet.
- There is no network access beyond localhost.
"""


def sha256_text(text) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def base_system_prompt() -> str:
    return SYSTEM_PROMPT_PATH.read_text(encoding="utf-8").strip()


def tool_surface_block(tool_set) -> str:
    return TOOL_SURFACE_TEMPLATE.format(
        n=len(tool_set), tools=tool_set.tool_surface_text()).strip()


def environment_block(sandbox, *, initial_files=()) -> str:
    n = len(list(initial_files))
    return ENVIRONMENT_TEMPLATE.format(
        scratch=sandbox.scratch_dir,
        initial=("empty" if n == 0 else
                 "contains %d file%s" % (n, "" if n == 1 else "s")),
        repo=sandbox.repo, port_a=sandbox.harness_port,
        port_b=sandbox.supervisor_port).strip()


def compose(tool_set, sandbox, *, initial_files=(), extra_blocks=()) -> dict:
    """Return ``{"text", "base_sha", "full_sha", "blocks"}``."""
    base = base_system_prompt()
    blocks = [base, tool_surface_block(tool_set),
              environment_block(sandbox, initial_files=initial_files)]
    blocks.extend(b for b in extra_blocks if b)
    text = "\n\n".join(blocks)
    return {"text": text, "base_sha": sha256_text(base),
            "full_sha": sha256_text(text),
            "blocks": ["shared_system_prompt", "tool_surface", "environment"]
                      + ["extra"] * len([b for b in extra_blocks if b])}
