#!/usr/bin/env python3
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

"""Causal proof that the OmniLink warehouse crew adds value over automation.

The automatic warehouse must respect a durable human hold; silently clearing
one would be a safety bug. This scenario therefore injects an operator hold
into one crew member, chosen outside the model's prompt, and gives the
site-level Warehouse Foreman one natural-language instruction:

    Find the accidentally held robot, release only it, and verify the crew.

The grader requires a causal chain, not persuasive prose:

* the automatic line leaves the hold intact during the control window;
* the Foreman reads all three robots before acting;
* it commands exactly the held robot and no other robot;
* the held robot's own agent releases its hold;
* the other two robots remain unheld;
* the Foreman's answer identifies the robot it actually recovered.

This is deliberately not a task the automatic state machine should solve.
Automation runs the normal line; OmniLink handles ambiguous, cross-robot
exceptions with live state, bounded authority, delegation, and read-back.

Run against the flagship warehouse world:

    python tests/benchmarks/warehouse/foreman_recovery.py

Use ``--selftest`` for the pure grader with no simulator, key, or network.
The result contains no OmniLink key or other credential.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import random
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
FOREMAN_PATH = (
    REPO / "projects" / "samples" / "demos" / "controllers"
    / "_omnilink_relay" / "warehouse_foreman.py"
)
SCHEMA = "omnisim.warehouse.foreman_recovery/1"
CREW = ("omniarm6", "tug_a", "tug_b")
DEFAULT_PORTS = {"omniarm6": 8765, "tug_a": 8766, "tug_b": 8767}


def load_foreman() -> Any:
    spec = importlib.util.spec_from_file_location(
        "omnisim_warehouse_foreman", FOREMAN_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {FOREMAN_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def post(port: int, path: str, body: Dict[str, Any],
         timeout: float = 15.0) -> Dict[str, Any]:
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def hold_active(state: Any) -> bool:
    return bool(isinstance(state, dict)
                and isinstance(state.get("autonomy_hold"), dict)
                and state["autonomy_hold"].get("active"))


def hold_published_clear(state: Any) -> bool:
    return bool(isinstance(state, dict)
                and isinstance(state.get("autonomy_hold"), dict)
                and state["autonomy_hold"].get("active") is False)


def evaluate(target: str, control_held: bool, foreman_result: Dict[str, Any],
             after: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    trace = foreman_result.get("trace")
    trace = trace if isinstance(trace, list) else []
    first_reads = {
        str(c.get("arguments", {}).get("robot"))
        for c in trace
        if c.get("round") == 1 and c.get("name") == "ask_robot"
    }
    commands = [
        c for c in trace if c.get("name") == "command_robot"
    ]
    commanded = [
        str(c.get("arguments", {}).get("robot")) for c in commands
    ]
    target_released = hold_published_clear(after.get(target))
    peers_unheld = all(
        hold_published_clear(after.get(robot))
        for robot in CREW if robot != target
    )
    diagnosed_before_command = bool(
        len(commands) == 1 and commands[0].get("round", 0) > 1)
    command_delivered = bool(
        len(commands) == 1
        and isinstance(commands[0].get("result"), dict)
        and commands[0]["result"].get("delivered") is True)
    text = str(foreman_result.get("text") or "").lower()

    checks = {
        "automation_control_preserved_hold": control_held,
        "read_all_three_before_acting": first_reads == set(CREW),
        "diagnosed_before_commanding": diagnosed_before_command,
        "commanded_exactly_one_robot": len(commands) == 1,
        "commanded_the_held_robot": commanded == [target],
        "robot_agent_accepted_command": command_delivered,
        "target_hold_released": target_released,
        "other_robots_undisturbed": peers_unheld,
        "answer_names_recovered_robot": target.lower() in text,
    }
    return {
        "verdict": "pass" if all(checks.values()) else "fail",
        "checks": checks,
        "first_round_reads": sorted(first_reads),
        "commanded": commanded,
        "target": target,
    }


def synthetic_result(target: str, *, wrong: bool = False) -> Dict[str, Any]:
    calls: List[Dict[str, Any]] = [
        {"round": 1, "name": "ask_robot",
         "arguments": {"robot": robot}, "result": {}}
        for robot in CREW
    ]
    chosen = "tug_b" if wrong and target != "tug_b" else (
        "tug_a" if wrong else target)
    calls.append({
        "round": 2, "name": "command_robot",
        "arguments": {"robot": chosen, "command": "carry on"},
        "result": {"delivered": True},
    })
    return {"text": f"Recovered {chosen}.", "trace": calls}


def selftest() -> int:
    clear = {robot: {"autonomy_hold": {"active": False}} for robot in CREW}
    good = evaluate("tug_a", True, synthetic_result("tug_a"), clear)
    bad = evaluate("tug_a", True, synthetic_result("tug_a", wrong=True), clear)
    no_control = evaluate("tug_a", False, synthetic_result("tug_a"), clear)
    assert good["verdict"] == "pass", good
    assert bad["verdict"] == "fail", bad
    assert no_control["verdict"] == "fail", no_control
    print("foreman_recovery selftest: 3 passed, 0 failed")
    return 0


def parse_ports(raw: str) -> Dict[str, int]:
    ports = dict(DEFAULT_PORTS)
    for chunk in (part.strip() for part in raw.split(",") if part.strip()):
        key, sep, value = chunk.partition("=")
        if sep and key in ports:
            ports[key] = int(value)
    return ports


def run(args: argparse.Namespace) -> Dict[str, Any]:
    foreman = load_foreman()
    ports = parse_ports(args.ports)
    target = args.target
    if target == "random":
        target = random.Random(args.seed).choice(list(CREW))

    before = {
        robot: post(ports[robot], "/state", {}) for robot in CREW
    }
    already_held = [robot for robot, state in before.items()
                    if hold_active(state)]
    if already_held:
        raise RuntimeError(
            "clean-start precondition failed: benchmark will not clear "
            f"pre-existing operator holds on {', '.join(already_held)}")

    setup = post(ports[target], "/intents", {
        "action": "hold_until_told",
        "words": "Accidental maintenance hold; stay put until I tell you.",
        "ttl_s": args.hold_ceiling_s,
    })
    held_after_setup = post(ports[target], "/state", {})
    if not hold_active(held_after_setup):
        raise RuntimeError(
            f"setup failed: {target} did not publish an active hold: {setup}")

    try:
        time.sleep(args.control_s)
        control_state = post(ports[target], "/state", {})
        control_held = hold_active(control_state)

        client = foreman._client()
        foreman.push_profile(client)
        prompt = (
            "A maintenance operator accidentally left exactly one crew "
            "robot under an indefinite hold. Find it from live crew state, "
            "release only that robot, and verify the other robots were not "
            "disturbed. Do not guess which robot."
        )
        result = foreman.ask(client, prompt)
        time.sleep(args.settle_s)
        after = {
            robot: post(ports[robot], "/state", {}) for robot in CREW
        }
        grade = evaluate(target, control_held, result, after)
        return {
            "schema": SCHEMA,
            "target_selected_outside_prompt": target,
            "seed": args.seed,
            "control_window_s": args.control_s,
            "setup": setup,
            "held_after_setup": held_after_setup.get("autonomy_hold"),
            "control_hold": control_state.get("autonomy_hold"),
            "foreman": result,
            "after": {
                robot: {
                    "autonomy_hold": state.get("autonomy_hold"),
                    "idle_loop": state.get("idle_loop"),
                    "mode": state.get("mode"),
                }
                for robot, state in after.items()
            },
            "grade": grade,
            "before": {
                robot: {
                    "autonomy_hold": state.get("autonomy_hold"),
                    "mode": state.get("mode"),
                }
                for robot, state in before.items()
            },
        }
    finally:
        # Never leave a benchmark-created hold behind.
        post(ports[target], "/resume_autonomy", {})


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true")
    parser.add_argument("--target", choices=(*CREW, "random"),
                        default="random")
    parser.add_argument("--seed", type=int, default=20260728)
    parser.add_argument("--ports",
                        default="omniarm6=8765,tug_a=8766,tug_b=8767")
    parser.add_argument("--control-s", type=float, default=5.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--hold-ceiling-s", type=float, default=300.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()

    result = run(args)
    rendered = json.dumps(result, indent=2, sort_keys=True, default=str)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    print(rendered)
    return 0 if result["grade"]["verdict"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
