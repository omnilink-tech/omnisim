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

"""The agent contract Phase 0 exercises without an LLM.

Phase 0 (SPEC 7) runs two scripted agents over every task: an **oracle** that
performs the known-good solution and a **null** agent that does nothing.
Required outcome: oracle PASSes every task, null PASSes none. That is
``omnilink_tasks``' rule 6 made structural, and it must be green before a
single token is spent.

Scripted agents get the same context an LLM loop will (scratch dir, prompt,
ports, deadline, a trace to write into) and return the same result object, so
Phase 1 can drop a model in behind the same interface.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class AgentContext:
    task_id: str
    prompt: str
    scratch_dir: Path
    run_dir: Path
    repo: Path
    trace: object
    deadline_s: float = 0.0
    seed: int = 0
    # --fake-sim: author artifacts, skip anything that launches the engine.
    fake_sim: bool = False

    @property
    def out_of_time(self):
        return self.deadline_s and time.time() > self.deadline_s


@dataclass
class AgentResult:
    final_message: str = ""
    self_verified: bool = False
    turns: int = 0
    tool_calls: int = 0
    # Unmeasured cost is None, never 0.0 (SPEC 3.1).
    tokens_in: int | None = None
    tokens_out: int | None = None
    tokens_cache_read: int | None = None
    usd: float | None = None
    artifacts: dict = field(default_factory=dict)
    deviations: list = field(default_factory=list)
    error: str | None = None
