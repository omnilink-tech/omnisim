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

"""The trace writer: nothing lost, everything parseable, leaks flagged.

SPEC 4.5 requires the trace file itself to be non-truncating, so the spill test
checks the *whole* result is recoverable from disk and that the JSONL record
carries a verifiable sha256 -- not that a big result was dropped politely.
"""

from __future__ import annotations

import hashlib
import json

from agentbench.common.trace import Trace
from agentbench.runner.backends.base import Usage
from agentbench.runner.budget import Budget, Ledger, StopReason
from agentbench.runner.trace import RunnerTrace


def _records(run_dir):
    return [json.loads(line) for line in
            (run_dir / "trace.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip()]


def _rt(tmp_path, **kw):
    t = Trace(tmp_path)
    return RunnerTrace(t, tmp_path, **kw), t


def test_turn_writes_a_message_plus_usage_metadata(tmp_path):
    rt, t = _rt(tmp_path)
    rt.turn("hello", usage=Usage(tokens_in=10, tokens_out=3),
            stop_reason="tool_use", tool_names=["run_shell"])
    t.close()
    recs = _records(tmp_path)
    msg = [r for r in recs if r.get("kind") == "message"][0]
    meta = [r for r in recs if r.get("kind") == "turn_meta"][0]
    assert msg["text"] == "hello" and msg["turn"] == 1
    assert meta["usage"]["tokens_in"] == 10
    assert meta["provider_stop_reason"] == "tool_use"
    assert meta["tool_calls"] == ["run_shell"]
    assert rt.turns == 1


def test_tool_records_full_arguments(tmp_path):
    rt, t = _rt(tmp_path)
    rt.tool("write_file", {"path": "a.wbt", "content": "x" * 50},
            result={"bytes": 50}, dt_s=0.01)
    t.close()
    call = [r for r in _records(tmp_path) if r.get("kind") == "tool_call"][0]
    assert call["args"]["content"] == "x" * 50, "arguments must not be elided"
    assert call["result"] == {"bytes": 50}
    assert rt.tool_calls == 1


def test_big_results_spill_to_a_file_with_a_verifiable_hash(tmp_path):
    rt, t = _rt(tmp_path, max_result_bytes=100)
    payload = "y" * 5000
    rt.tool("run_shell", {"command": "cat big"}, result=payload)
    t.close()
    call = [r for r in _records(tmp_path) if r.get("kind") == "tool_call"][0]
    spill = call["result"]
    assert spill["spilled_to_file"]
    assert spill["bytes"] == 5000
    from pathlib import Path
    on_disk = Path(spill["spilled_to_file"]).read_text(encoding="utf-8")
    assert on_disk == payload, "the full result must survive on disk"
    assert spill["sha256"] == hashlib.sha256(
        payload.encode()).hexdigest()
    assert spill["head"] == payload[:2000]
    assert rt.spilled == 1


def test_small_results_are_inline(tmp_path):
    rt, t = _rt(tmp_path, max_result_bytes=10_000)
    rt.tool("list_dir", {"path": "."}, result="two entries")
    t.close()
    call = [r for r in _records(tmp_path) if r.get("kind") == "tool_call"][0]
    assert call["result"] == "two entries"
    assert rt.spilled == 0


def test_leak_suspicion_is_flagged(tmp_path, sandbox):
    rt, t = _rt(tmp_path, guard=sandbox.guard)
    rt.tool("run_shell", {"command": "ls /somewhere"}, result="fine")
    rt.tool("run_shell",
            {"command": "cat %s/graders/a1.py" % sandbox.guard.deny_roots[0]},
            result="thresholds")
    t.close()
    metas = [r for r in _records(tmp_path) if r.get("kind") == "tool_meta"]
    assert [m["leak_suspect"] for m in metas] == [False, True]
    assert rt.leak_suspects == 1


def test_budget_trip_is_recorded_loudly(tmp_path):
    rt, t = _rt(tmp_path)
    led = Ledger()
    led.turns = 60
    rt.limit(StopReason.MAX_TURNS, "hit the cap", led, Budget(max_turns=60))
    t.close()
    rec = [r for r in _records(tmp_path)
           if r.get("kind") == "budget_exhausted"][0]
    assert rec["reason"] == StopReason.MAX_TURNS
    assert rec["ledger"]["turns"] == 60
    assert rec["budget"]["max_turns"] == 60


def test_every_record_is_valid_jsonl_with_timestamps(tmp_path):
    rt, t = _rt(tmp_path)
    rt.event("runner_start", tool_set="shell")
    rt.turn("t")
    rt.tool("list_dir", {"path": "."}, result="ok")
    rt.final("done", True)
    t.close()
    recs = _records(tmp_path)
    assert len(recs) >= 4
    for r in recs:
        assert "utc" in r and "t_s" in r
    assert recs[-1]["kind"] == "final" and recs[-1]["self_verified"] is True
