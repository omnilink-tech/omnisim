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
"""Record model replies in one run, replay them in the next (dev loop only).

OMNILINK_MODEL_RECORD=<file.jsonl>  append every successful /api/chat reply
OMNILINK_MODEL_REPLAY=<file.jsonl>  answer a request from the recording when
                                    one matches, else call the model live

A change that does not touch what the model is asked -- the parser, the
safety gate, the scheduler, the robot code -- can then be re-run without
paying for or waiting on the model.

The match key is the conversation, the standing instructions, the tools,
the engine and the model, with every number rounded to one decimal: the
conversation carries MEASURED values (a drive reports 0.998 m one run and
1.002 m the next) that would otherwise break the match after the first tool
call. The per-turn liveContext (odometer, visits) is left out of the key
for the same reason. A replayed reply may therefore have been produced for
a slightly different world: replay is a FAST REPRODUCER, never a result.
Each recorded reply is served at most once, in recorded order.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import threading
from collections import defaultdict, deque
from typing import Any, Deque, Dict, Optional

RECORD_PATH = os.environ.get("OMNILINK_MODEL_RECORD", "").strip()
REPLAY_PATH = os.environ.get("OMNILINK_MODEL_REPLAY", "").strip()

_NUMBER = re.compile(r"-?\d+\.\d+")
_lock = threading.Lock()
_replay: Optional[Dict[str, Deque[Dict[str, Any]]]] = None


def active() -> bool:
    return bool(RECORD_PATH or REPLAY_PATH)


def _round_numbers(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 1)
    if isinstance(value, str):
        return _NUMBER.sub(lambda m: f"{float(m.group(0)):.1f}", value)
    if isinstance(value, list):
        return [_round_numbers(v) for v in value]
    if isinstance(value, dict):
        return {k: _round_numbers(v) for k, v in value.items()}
    return value


def key(messages: Any, system_instruction: Dict[str, Any], engine: str, model: str) -> str:
    stable = {k: v for k, v in (system_instruction or {}).items() if k != "liveContext"}
    blob = json.dumps(_round_numbers({"m": messages, "s": stable, "e": engine or "", "model": model or ""}),
                      sort_keys=True, default=str)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def _load() -> Dict[str, Deque[Dict[str, Any]]]:
    global _replay
    if _replay is None:
        table: Dict[str, Deque[Dict[str, Any]]] = defaultdict(deque)
        if REPLAY_PATH and os.path.exists(REPLAY_PATH):
            with open(REPLAY_PATH, encoding="utf-8") as fh:
                for line in fh:
                    try:
                        row = json.loads(line)
                        table[row["key"]].append(row["data"])
                    except (ValueError, KeyError):
                        continue
        _replay = table
    return _replay


def lookup(k: str) -> Optional[Dict[str, Any]]:
    if not REPLAY_PATH:
        return None
    with _lock:
        queue = _load().get(k)
        return queue.popleft() if queue else None


def record(k: str, data: Dict[str, Any]) -> None:
    if not RECORD_PATH:
        return
    line = json.dumps({"key": k, "data": data}, default=str)
    with _lock:
        with open(RECORD_PATH, "a", encoding="utf-8") as fh:
            fh.write(line + "\n")


def _reset_for_tests(record_path: str = "", replay_path: str = "") -> None:
    global RECORD_PATH, REPLAY_PATH, _replay
    RECORD_PATH, REPLAY_PATH, _replay = record_path, replay_path, None
