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

# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Register the OmniSim-Roomba profile with the OmniLink platform.

Same pattern as the Picker: profile-only specialist that reuses an
existing mobile bridge's /tool endpoint. Run once to register, then
chat from the OmniLink web UI.

Run:

    export OMNI_KEY="olink_..."
    # default ROOMBA_BRIDGE points at the single-mobile-base demo:
    # export ROOMBA_BRIDGE="http://127.0.0.1:8765"
    pip install omnilink
    python agents/templates/roomba/register.py

Delete the profile with --delete.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import truststore  # type: ignore
    truststore.inject_into_ssl()
except Exception:
    pass

try:
    from omnilink.client import OmniLinkClient  # type: ignore
except ImportError:
    print("ERROR: omnilink-lib is not installed.\n    pip install omnilink")
    sys.exit(1)


THIS_DIR = Path(__file__).resolve().parent
PROFILE_PATH = THIS_DIR / "profile.json"
BASE_URL = os.environ.get("OMNILINK_BASE_URL", "https://www.omnilink-agents.com")
ROOMBA_BRIDGE = os.environ.get("ROOMBA_BRIDGE", "http://127.0.0.1:8765").rstrip("/")


def build_mobile_tools() -> List[Dict[str, Any]]:
    """Mirror of `build_mobile_tools()` in
    projects/samples/demos/controllers/omnilink_mobile_bridge/."""
    return [
        {"name": "get_robot_state", "description": "Read x, y, yaw, measured linear/angular velocity, and current mode.", "parameters": {"type": "object", "properties": {}}},
        {"name": "drive_forward", "description": "Drive a specified distance along the current heading. Positive=forward, negative=reverse. Optional speed override.", "parameters": {"type": "object", "properties": {"distance": {"type": "number"}, "speed": {"type": "number"}}, "required": ["distance"]}},
        {"name": "turn", "description": "Turn in place by a signed angle (radians). Positive=counter-clockwise.", "parameters": {"type": "object", "properties": {"angle_rad": {"type": "number"}}, "required": ["angle_rad"]}},
        {"name": "set_velocity", "description": "Continuous (linear, angular) velocity. Robot keeps moving until next command. Use stop_robot to halt.", "parameters": {"type": "object", "properties": {"linear": {"type": "number"}, "angular": {"type": "number"}}, "required": ["linear", "angular"]}},
        {"name": "stop_robot", "description": "Emergency halt -- zero both wheel velocities. ALWAYS available.", "parameters": {"type": "object", "properties": {}}},
        {"name": "reset_to_home", "description": "Teleport back to the spawn pose (supervisor reset).", "parameters": {"type": "object", "properties": {}}},
    ]


def find_profile_id(client: Any, name: str) -> Optional[str]:
    for p in client.list_profiles():
        if (p.get("name") or "").lower() == name.lower():
            return p.get("id")
    return None


def confirm_bridge(bridge_url: str) -> Optional[Dict[str, Any]]:
    try:
        body = json.dumps({}).encode("utf-8")
        req = urllib.request.Request(f"{bridge_url}/list_robots", data=body, method="POST")
        req.add_header("Content-Type", "application/json")
        with urllib.request.urlopen(req, timeout=5) as r:
            data = json.loads(r.read().decode("utf-8") or "[]")
        if isinstance(data, list) and data:
            return data[0]
    except Exception as e:
        print(f"  [warn] could not reach bridge at {bridge_url} ({e}). Registering profile anyway.")
    return None


def load_profile() -> Dict[str, Any]:
    with open(PROFILE_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--delete", action="store_true")
    args = parser.parse_args()

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        print("ERROR: OMNI_KEY not set.")
        return 1

    profile = load_profile()
    agent_name = profile["name"]
    client = OmniLinkClient(omni_key=omni_key, base_url=BASE_URL, timeout=30)

    if args.delete:
        pid = find_profile_id(client, agent_name)
        if not pid:
            print(f"  No profile named {agent_name!r} found.")
            return 0
        try:
            client.delete_profile(pid)
            print(f"  Deleted profile {agent_name!r} (id={pid}).")
        except Exception as e:
            print(f"  Delete failed: {e}")
            return 1
        return 0

    info = confirm_bridge(ROOMBA_BRIDGE)
    if info:
        print(f"  Bridge online: id={info.get('id')}, model={info.get('model')}")
    else:
        print(f"  Bridge not reachable at {ROOMBA_BRIDGE} (yet). Profile will still be created.")

    tool_defs = build_mobile_tools()
    settings: Dict[str, Any] = dict(profile["settings"])
    settings["availableTools"] = ", ".join(t["name"] for t in tool_defs)
    settings["availableToolDetails"] = tool_defs
    settings["allowToolUse"] = True
    settings["toolCallbackUrl"] = f"{ROOMBA_BRIDGE}/tool"
    settings.setdefault("engine", "g2-engine")

    pid = find_profile_id(client, agent_name)
    if pid:
        client.update_profile(pid, name=agent_name, settings=settings)
        print(f"  Updated profile {agent_name!r} (id={pid}) -> toolCallbackUrl={settings['toolCallbackUrl']}")
    else:
        result = client.create_profile(agent_name, settings=settings)
        pid = result.get("id", "")
        print(f"  Created profile {agent_name!r} (id={pid}) -> toolCallbackUrl={settings['toolCallbackUrl']}")

    print()
    print(f"  Open https://www.omnilink-agents.com/agents and pick "
          f"{agent_name!r}.")
    print(f"  Make sure the mobile bridge at {ROOMBA_BRIDGE} is running before you chat.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
