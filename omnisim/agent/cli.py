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

"""Scaffold a production-shaped OmniLink agent for an OmniSim robot bridge."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from string import Template
from typing import Dict

from ..paths import REPO_ROOT


_CLASSES: Dict[str, Dict[str, str]] = {
    "mobile": {
        "world": "projects/samples/demos/worlds/chat/omnilink_husky.omniworld",
        "tool": "drive_forward",
        "description": "Drive the mobile base forward by a bounded distance.",
        "parameters": '{"type":"object","properties":{"distance":{"type":"number","description":"Metres; negative reverses."}},"required":["distance"],"additionalProperties":false}',
        "method": "POST",
        "path": "/drive_forward",
    },
    "arm": {
        "world": "projects/samples/demos/worlds/chat/omnilink_ur5e.omniworld",
        "tool": "set_joint_positions",
        "description": "Move the arm to a joint-space target in radians.",
        "parameters": '{"type":"object","properties":{"q":{"type":"array","items":{"type":"number"}},"duration_s":{"type":"number"}},"required":["q"],"additionalProperties":false}',
        "method": "POST",
        "path": "/set_joint_positions",
    },
    "quadruped": {
        "world": "projects/samples/demos/worlds/chat/omnilink_omniquad.omniworld",
        "tool": "ask_robot",
        "description": "Send a natural-language motion request to the quadruped bridge.",
        "parameters": '{"type":"object","properties":{"text":{"type":"string"}},"required":["text"],"additionalProperties":false}',
        "method": "POST",
        "path": "/prompt",
    },
    "flying": {
        "world": "projects/samples/demos/worlds/chat/omnilink_mavic.omniworld",
        "tool": "goto_waypoint",
        "description": "Fly to a world-frame waypoint after takeoff.",
        "parameters": '{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"altitude":{"type":"number"}},"required":["x","y"],"additionalProperties":false}',
        "method": "ACTION",
        "path": "/action",
    },
}


_AGENT_SOURCE = Template(r'''"""Generated OmniLink agent for an OmniSim $robot_class bridge."""
from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR.parent))
from _lib import OmniLinkAgentRunner, locate_omnilink_lib  # noqa: E402

AGENT_NAME = "$display_name"
BRIDGE_URL = os.environ.get("$bridge_env", "http://127.0.0.1:$bridge_port").rstrip("/")
BRIDGE_TOKEN = os.environ.get("OMNISIM_BRIDGE_TOKEN", "").strip()
locate_omnilink_lib(env_var="$lib_env")

QUERY_TOOLS: List[Dict[str, Any]] = [
    {"name": "get_robot_state", "description": "Read current robot telemetry.",
     "parameters": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "stop_robot", "description": "Immediately halt robot motion.",
     "parameters": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "$tool", "description": "$tool_description", "parameters": $parameters},
]
REGISTRY = {item["name"]: item for item in QUERY_TOOLS}


def _call(path: str, payload: Dict[str, Any] | None = None, method: str = "POST") -> Dict[str, Any]:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    if BRIDGE_TOKEN:
        headers["Authorization"] = f"Bearer {BRIDGE_TOKEN}"
    request = urllib.request.Request(BRIDGE_URL + path, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            return json.loads(exc.read().decode("utf-8"))
        except Exception:
            return {"error": f"bridge HTTP {exc.code}"}
    except Exception as exc:
        return {"error": f"bridge unavailable: {exc}"}


def dispatch(tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    if tool_name == "get_robot_state":
        return _call("/state", None, "GET")
    if tool_name == "stop_robot":
        return _call("/stop_robot", {})
    if tool_name == "$tool":
        payload = dict(args)
$action_line        return _call("$tool_path", payload, "$tool_method")
    return {"error": f"unknown tool: {tool_name}", "known_tools": sorted(REGISTRY)}


def classify_result(tool: str, args: Dict[str, Any], result: Any):
    if isinstance(result, dict) and (result.get("error") or result.get("ok") is False):
        return "warning", f"{tool}: {result.get('message') or result.get('error')}"[:120]
    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {"agent": AGENT_NAME, "robot_class": "$robot_class",
            "bridge_url": BRIDGE_URL, "tools_registered": len(REGISTRY),
            "activity_log_size": len(activity_log)}


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name=AGENT_NAME,
        profile_path=THIS_DIR / "profile.json",
        port=$agent_port,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="$port_env",
        lib_env="$lib_env",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
''')


def _slug(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")
    if not slug or not re.fullmatch(r"[a-z][a-z0-9_]{1,62}", slug):
        raise ValueError("name must produce a 2-63 character lowercase agent slug")
    return slug


def scaffold(args: argparse.Namespace) -> int:
    slug = _slug(args.name)
    cfg = _CLASSES[args.robot_class]
    display_name = args.display_name or " ".join(part.capitalize() for part in slug.split("_"))
    root = Path(args.output_root).resolve() if args.output_root else REPO_ROOT / "agents" / "production"
    target = root / slug
    if target.exists():
        raise FileExistsError(f"refusing to overwrite existing agent directory: {target}")
    target.mkdir(parents=True)

    env_prefix = re.sub(r"[^A-Z0-9]", "_", slug.upper())
    world = args.world or cfg["world"]
    manifest = {
        "name": slug,
        "display_name": display_name,
        "omnilink_agent_name": display_name,
        "world": world,
        "bridge_port": args.bridge_port,
        "agent_script": f"agents/production/{slug}/{slug}_agent.py",
        "agent_port": args.agent_port,
        "agent_port_env": f"{env_prefix}_PORT",
        "bridge_url_env": f"{env_prefix}_BRIDGE_URL",
        "description": args.description or f"{display_name} controls an OmniSim {args.robot_class} bridge through OmniLink.",
        "tags": [args.robot_class, "generated"],
    }
    profile = {
        "name": display_name,
        "settings": {
            "agentName": display_name,
            "mainTask": (
                f"You are {display_name}, a careful {args.robot_class} robot operator in OmniSim. "
                "Use only registered tools, inspect state before motion, stop on stale telemetry or faults, "
                "and never claim success without reading state that confirms it."
            ),
            "availableCommands": f"get_robot_state, stop_robot, {cfg['tool']}",
            "availableActions": "inspect, act, verify, stop_on_fault",
            "engine": args.engine,
            "temperature": 0.1,
        },
    }
    action_line = "        payload['action'] = 'goto_waypoint'\n" if cfg["method"] == "ACTION" else ""
    method = "POST" if cfg["method"] == "ACTION" else cfg["method"]
    source = _AGENT_SOURCE.substitute(
        robot_class=args.robot_class,
        display_name=display_name,
        bridge_env=f"{env_prefix}_BRIDGE_URL",
        bridge_port=args.bridge_port,
        lib_env=f"{env_prefix}_OMNILINK_LIB",
        tool=cfg["tool"],
        tool_description=cfg["description"],
        parameters=cfg["parameters"],
        action_line=action_line,
        tool_path=cfg["path"],
        tool_method=method,
        agent_port=args.agent_port,
        port_env=f"{env_prefix}_PORT",
    )
    readme = (
        f"# {display_name}\n\nGenerated OmniLink agent for an OmniSim {args.robot_class} bridge.\n\n"
        f"Run from the OmniSim repository root:\n\n"
        f"```bash\nexport OMNI_KEY=olink_YOUR_KEY\npython -m omnisim run-agent {slug}\n```\n\n"
        "Edit `profile.json` to change behavior and edit the `QUERY_TOOLS`/`dispatch` pair in the agent script "
        "to add capabilities. Keep tool schemas and dispatch handlers in sync.\n"
    )
    (target / "omnilink.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (target / "profile.json").write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    (target / f"{slug}_agent.py").write_text(source, encoding="utf-8")
    (target / "README.md").write_text(readme, encoding="utf-8")
    print(f"Created {target}")
    print(f"Next: set OMNI_KEY and run `python -m omnisim run-agent {slug}`")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="omnisim agent", description="Build OmniLink agents for OmniSim.")
    sub = parser.add_subparsers(dest="command", required=True)
    new = sub.add_parser("new", help="Create a complete OmniLink agent scaffold.")
    new.add_argument("name")
    new.add_argument("--robot-class", choices=sorted(_CLASSES), required=True)
    new.add_argument("--display-name")
    new.add_argument("--description")
    new.add_argument("--world", help="Repository-relative .wbt path; defaults by robot class.")
    new.add_argument("--bridge-port", type=int, default=8765)
    new.add_argument("--agent-port", type=int, default=51530)
    new.add_argument("--engine", default="g2-engine")
    new.add_argument("--output-root", help=argparse.SUPPRESS)
    new.set_defaults(func=scaffold)
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (FileExistsError, ValueError) as exc:
        parser.error(str(exc))
    return 2
