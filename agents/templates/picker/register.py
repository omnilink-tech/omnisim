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

"""Register the OmniSim-Picker profile with the OmniLink platform.

The Picker is a profile-only specialist. It doesn't run its own tool
server; it reuses an existing arm bridge's /tool endpoint. So all this
script does is:

  1. Load profile.json (with the picker-specialised mainTask).
  2. Probe the configured arm bridge for its tool surface and turn the
     bridge's relay tools into availableToolDetails.
  3. Push the profile to OmniLink (create or update) with the bridge's
     /tool URL as toolCallbackUrl.

Run:

    export OMNI_KEY="olink_..."
    # default PICKER_BRIDGE points at the single-arm OmniArm 6 demo:
    # export PICKER_BRIDGE="http://127.0.0.1:8765"
    pip install omnilink
    python agents/templates/picker/register.py

Then open https://omnilink-agents.com, pick "OmniSim-Picker", and chat.
Make sure the arm bridge is running (e.g. open omnilink_omniarm6.omniworld in
OmniSim) so the platform's tool callbacks have somewhere to land.

To delete the profile, pass --delete:

    python agents/templates/picker/register.py --delete
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
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
PICKER_BRIDGE = os.environ.get("PICKER_BRIDGE", "http://127.0.0.1:8765").rstrip("/")


def probe_bridge_tools(bridge_url: str) -> List[Dict[str, Any]]:
    """Inspect the bridge's /list_robots + /capabilities so we can hand
    the platform a tool surface that matches what the bridge actually
    accepts. The bridge doesn't expose a tool catalog endpoint of its
    own (its /tool dispatch is data-driven from relay.tools), so we
    hardcode the Axis-normalised tool set the arm bridge always
    supports."""
    # Hardcoded mirror of what `build_arm_tools()` in the arm bridge
    # produces. Keep in sync if the arm bridge grows new tools.
    # Source: projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py
    return [
        {"name": "get_robot_state", "description": "Read freshest joint state, TCP pose, motion mode, and last-tick timestamp.", "parameters": {"type": "object", "properties": {}}},
        {"name": "read_joints", "description": "Read just the joint angles q.", "parameters": {"type": "object", "properties": {}}},
        {"name": "read_tcp_pose", "description": "Read just the TCP xyz pose.", "parameters": {"type": "object", "properties": {}}},
        {"name": "solve_ik", "description": "Preview-only IK -- returns {q, err_norm} for a candidate TCP target without commanding motion. Use BEFORE set_tcp_target.", "parameters": {"type": "object", "properties": {"xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}}, "required": ["xyz"]}},
        {"name": "set_tcp_target", "description": "Command TCP target via DLS IK. Bridge rejects unreachable targets and clamps the resulting q.", "parameters": {"type": "object", "properties": {"xyz": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}}, "required": ["xyz"]}},
        {"name": "set_joint_positions", "description": "Direct joint-space command. Bridge clamps to joint_limits. Prefer set_tcp_target for normal pick-and-place.", "parameters": {"type": "object", "properties": {"q": {"type": "array", "items": {"type": "number"}}}, "required": ["q"]}},
        {"name": "open_gripper", "description": "Open the gripper. Fails with effector_unavailable on arms without one.", "parameters": {"type": "object", "properties": {}}},
        {"name": "close_gripper", "description": "Close the gripper.", "parameters": {"type": "object", "properties": {}}},
        {"name": "reset_to_home", "description": "Interpolate to the configured home pose.", "parameters": {"type": "object", "properties": {}}},
        {"name": "stop_robot", "description": "Emergency halt -- freeze at current joint angles. ALWAYS available.", "parameters": {"type": "object", "properties": {}}},
    ]


def confirm_bridge(bridge_url: str) -> Optional[Dict[str, Any]]:
    """Hit /list_robots to confirm the bridge is up. Returns the first
    robot's record on success, None on failure (best-effort -- we still
    register the profile so the operator can launch the bridge later)."""
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


def find_profile_id(client: Any, name: str) -> Optional[str]:
    for p in client.list_profiles():
        if (p.get("name") or "").lower() == name.lower():
            return p.get("id")
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--delete", action="store_true", help="Delete the profile and exit.")
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

    robot_info = confirm_bridge(PICKER_BRIDGE)
    if robot_info:
        print(f"  Bridge online: id={robot_info.get('id')}, model={robot_info.get('model')}")
    else:
        print(f"  Bridge not reachable at {PICKER_BRIDGE} (yet). Profile will still be created.")

    tool_defs = probe_bridge_tools(PICKER_BRIDGE)
    settings: Dict[str, Any] = dict(profile["settings"])
    settings["availableTools"] = ", ".join(t["name"] for t in tool_defs)
    settings["availableToolDetails"] = tool_defs
    settings["allowToolUse"] = True
    settings["toolCallbackUrl"] = f"{PICKER_BRIDGE}/tool"
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
    print(f"  Open https://omnilink-agents.com and pick {agent_name!r} in the profile dropdown.")
    print(f"  Make sure the bridge at {PICKER_BRIDGE} is running before you chat.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
