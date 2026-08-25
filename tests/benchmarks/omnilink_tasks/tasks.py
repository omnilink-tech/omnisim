# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Task definitions for the OmniLink agent benchmark suite.

Each Task is the minimum specification needed to evaluate one
single-prompt instruction:

    id            stable identifier (file-name safe)
    world         relative path to a world under projects/samples/demos/worlds/
    bridge_port   port the bridge listens on (8765 for all single-robot demos)
    prompt        instruction text fed to /prompt
    timeout_s     max wall-clock seconds the runner waits for grader=pass
    grader        callable(state_history) -> (passed: bool, metric: float, note: str)
                  state_history is a list of /get_robot_state responses
                  collected at ~5 Hz from /prompt onward.

Adding a task: define a new grader (a small lambda or function) and a
Task instance and append it to TASKS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Tuple


GraderResult = Tuple[bool, float, str]  # (passed, metric, note)
Grader = Callable[[List[Dict[str, Any]]], GraderResult]


@dataclass
class Task:
    id: str
    world: str
    bridge_port: int
    prompt: str
    timeout_s: float
    grader: Grader


# ── Graders ──────────────────────────────────────────────────────────


def _last_pose(history: List[Dict[str, Any]]) -> Tuple[float, float, float]:
    for st in reversed(history):
        if all(k in st for k in ("x", "y", "yaw")):
            return float(st["x"]), float(st["y"]), float(st["yaw"])
    return 0.0, 0.0, 0.0


def grader_mobile_drive_1m(history: List[Dict[str, Any]]) -> GraderResult:
    """Husky travels >= 0.9 m from spawn (x[0], y[0])."""
    if not history:
        return False, 0.0, "no state"
    spawn = (float(history[0].get("x", 0.0)), float(history[0].get("y", 0.0)))
    last = _last_pose(history)
    d = math.hypot(last[0] - spawn[0], last[1] - spawn[1])
    return d >= 0.9, d, f"travelled {d:.2f} m"


# ── Tasks ────────────────────────────────────────────────────────────


TASKS: List[Task] = [
    Task(
        id="mobile_drive_1m",
        # The chat worlds moved into worlds/chat/ and this path was never
        # updated, so run.py's WORLDS_DIR join missed and the lane returned
        # "world not found" BEFORE launching anything -- i.e. the legacy lane has
        # been unable to score at all, silently, rather than failing a grader.
        world="chat/omnilink_husky.omniworld",
        bridge_port=8765,
        prompt="forward 1 meter",
        timeout_s=20.0,
        grader=grader_mobile_drive_1m,
    ),
]


def get_task(task_id: str) -> Task:
    for t in TASKS:
        if t.id == task_id:
            return t
    raise KeyError(f"unknown task {task_id!r}; known: {[t.id for t in TASKS]}")
