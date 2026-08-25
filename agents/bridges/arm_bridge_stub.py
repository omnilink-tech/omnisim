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

"""Real-robot arm bridge stub.

Subclasses `BridgeBase` to expose the same HTTP surface OmniSim arms
expose (/list_robots, /get_robot_state, /set_joint_positions,
/set_tcp_target, /open_gripper, /stop_robot, ..., /tool, /prompt). The
act_* methods delegate to a `MockArmDriver` that prints what it would
have done. To wire this to a real arm:

  1. Replace `MockArmDriver` with your robot SDK's client.
  2. Replace each driver-method body with the equivalent SDK call.
  3. Restart this script.

That's it. The OmniLink-side stays byte-identical -- same agents, same
profile-callback URLs, same chat prompts.

Run:

    python agents/bridges/arm_bridge_stub.py
    # then in another terminal:
    curl -X POST -H "Content-Type: application/json" \\
        -d '{"text":"go home"}' http://127.0.0.1:8765/prompt
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

# Allow running this file directly without pip-installing the example.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from bridge_base import BridgeBase, serve_http  # noqa: E402
from grippers import make_driver, GRIPPER_SPECS  # noqa: E402
from grippers.driver_base import RealGripperDriver  # noqa: E402


class MockArmDriver:
    """Stand-in for your robot SDK's client. Replace with the real thing.

    Methods deliberately match the action surface of the OmniSim arm
    bridge so the act_* mapping in `RealArmBridge` is a one-line each.
    """

    JOINT_NAMES = [
        "shoulder_pan", "shoulder_lift", "elbow",
        "wrist_1", "wrist_2", "wrist_3",
    ]
    HOME_POSE = [0.0, -1.0, 1.4, -1.2, -1.57, 0.0]

    def __init__(self) -> None:
        self.q: List[float] = list(self.HOME_POSE)
        self.gripper_state = "open"
        self.faulted = False
        self.last_tick = time.time()

    def stop(self) -> Dict[str, Any]:
        print(f"  [MOCK_ARM] STOP  (freezing q={self.q})")
        return {"halted_at": time.time(), "q": list(self.q)}

    def set_joint_positions(self, q: List[float]) -> Dict[str, Any]:
        if len(q) != len(self.JOINT_NAMES):
            return {"error": f"q must have {len(self.JOINT_NAMES)} entries"}
        print(f"  [MOCK_ARM] set_joint_positions: {[round(v, 3) for v in q]}")
        self.q = list(q)
        return {"accepted": True, "clamped_q": list(q)}

    def set_tcp_target(self, xyz: List[float]) -> Dict[str, Any]:
        if len(xyz) != 3:
            return {"error": "xyz must have 3 entries"}
        print(f"  [MOCK_ARM] set_tcp_target: {xyz}")
        return {"accepted": True, "solved_q": list(self.q), "err_norm": 0.0}

    def reset_to_home(self) -> Dict[str, Any]:
        print(f"  [MOCK_ARM] reset_to_home")
        self.q = list(self.HOME_POSE)
        return {"q": list(self.q)}

    def open_gripper(self) -> Dict[str, Any]:
        print(f"  [MOCK_ARM] gripper -> open")
        self.gripper_state = "open"
        return {"state": "open"}

    def close_gripper(self) -> Dict[str, Any]:
        print(f"  [MOCK_ARM] gripper -> closed")
        self.gripper_state = "closed"
        return {"state": "closed"}

    def get_state(self) -> Dict[str, Any]:
        return {
            "q": list(self.q),
            "tcp": None,  # mock skips IK
            "gripper": self.gripper_state,
            "fault": "fault" if self.faulted else None,
            "last_tick_at": self.last_tick,
        }


class RealArmBridge(BridgeBase):
    """One-to-one mapping from BridgeBase action surface to the driver.

    In production this is the only file that knows about the robot
    SDK. Everything else (OmniLink agents, the chat panel, the HTTP
    surface, the tool dispatch) stays generic.
    """

    def __init__(self, driver: MockArmDriver, robot_id: str = "real_arm",
                 gripper: Optional[RealGripperDriver] = None) -> None:
        self.driver = driver
        self.gripper = gripper
        self.robot_id = robot_id
        self.model = "MockArm"
        self.capabilities = {
            "joint_names": MockArmDriver.JOINT_NAMES,
            "home_pose": list(MockArmDriver.HOME_POSE),
            "has_gripper": gripper is not None,
        }
        if gripper is not None:
            self.capabilities["gripper"] = gripper.capabilities()

    def act_stop(self) -> Dict[str, Any]:
        return self.driver.stop()

    def act_reset_to_home(self) -> Dict[str, Any]:
        return self.driver.reset_to_home()

    def act_set_joint_positions(self, q: List[float], duration_s: float = 1.2) -> Dict[str, Any]:
        return self.driver.set_joint_positions(q)

    def act_set_tcp_target(self, xyz: List[float]) -> Dict[str, Any]:
        return self.driver.set_tcp_target(xyz)

    def act_open_gripper(self) -> Dict[str, Any]:
        if self.gripper is None:
            return {"error": "open_gripper not supported (no gripper attached)"}
        return {"state": "open", "gripper": self.gripper.open()}

    def act_close_gripper(self) -> Dict[str, Any]:
        if self.gripper is None:
            return {"error": "close_gripper not supported (no gripper attached)"}
        return {"state": "closed", "gripper": self.gripper.close()}

    def act_set_gripper_width(self, width: float) -> Dict[str, Any]:
        if self.gripper is None:
            return {"error": "no gripper attached"}
        return {"gripper": self.gripper.set_width(float(width))}

    def act_grasp(self, force: Optional[float] = None,
                  width: Optional[float] = None) -> Dict[str, Any]:
        if self.gripper is None:
            return {"error": "no gripper attached"}
        return {"gripper": self.gripper.grasp(force=force, width=width)}

    def act_release(self) -> Dict[str, Any]:
        if self.gripper is None:
            return {"error": "no gripper attached"}
        return {"gripper": self.gripper.release()}

    def get_state(self) -> Dict[str, Any]:
        st = self.driver.get_state()
        st.setdefault("id", self.robot_id)
        st.setdefault("model", self.model)
        st["gripper"] = self.gripper.state() if self.gripper else None
        return st

    def act_prompt(self, text: str) -> Dict[str, Any]:
        """Tiny offline router so /prompt does something useful without
        an OmniLink relay attached. Maps a handful of natural-language
        intents onto the action surface. In a real deployment you'd
        plug in the same OmniLinkRelay used by the OmniSim bridges,
        with the same Tool definitions -- nothing else changes."""
        s = text.strip().lower()
        if not s:
            return {"response": "(empty prompt)", "actions": []}
        if re.search(r"\b(stop|halt|freeze)\b", s):
            self.act_stop()
            return {"response": "Stopping.", "actions": [{"tool": "stop_robot", "result": "ok"}]}
        if re.search(r"\b(home|reset|park)\b", s):
            self.act_reset_to_home()
            return {"response": "Moving to home pose.", "actions": [{"tool": "reset_to_home", "result": "ok"}]}
        if re.search(r"\b(grasp|grab|pick (it )?up|take it)\b", s):
            res = self.act_grasp()
            ok = "error" not in res
            return {"response": "Grasping." if ok else "No gripper attached.",
                    "actions": [{"tool": "grasp", "result": "ok" if ok else "err"}]}
        if re.search(r"\b(release|let go|drop|put it down)\b", s):
            res = self.act_release()
            ok = "error" not in res
            return {"response": "Releasing." if ok else "No gripper attached.",
                    "actions": [{"tool": "release", "result": "ok" if ok else "err"}]}
        if re.search(r"\bopen\b.*\bgrip", s) or s.strip() == "open":
            self.act_open_gripper()
            return {"response": "Opening the gripper.", "actions": [{"tool": "open_gripper", "result": "ok"}]}
        if re.search(r"\bclose\b.*\bgrip", s) or s.strip() == "close":
            self.act_close_gripper()
            return {"response": "Closing the gripper.", "actions": [{"tool": "close_gripper", "result": "ok"}]}
        m = re.search(r"(?:go|move)[^-\d]*\(?\s*(-?\d+\.?\d*)[ ,]+(-?\d+\.?\d*)[ ,]+(-?\d+\.?\d*)", s)
        if m:
            xyz = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
            self.act_set_tcp_target(xyz)
            return {"response": f"Moving TCP to {xyz}.", "actions": [{"tool": "set_tcp_target", "result": "ok"}]}
        return {
            "response": (
                "I don't recognise that. Try: 'home', 'stop', "
                "'open the gripper', 'go to 0.4 0.0 0.3'."
            ),
            "actions": [],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765, help="HTTP port (default 8765).")
    parser.add_argument("--id", default="real_arm", help="Robot id reported in /list_robots.")
    parser.add_argument("--gripper", default=None,
                        help=("Real gripper to attach, one of "
                              f"{sorted(GRIPPER_SPECS.keys())}. Omit for no gripper. "
                              "Drivers default to a DryTransport (no hardware); "
                              "inject a real transport for deployment."))
    args = parser.parse_args()

    driver = MockArmDriver()
    gripper = None
    if args.gripper:
        gripper = make_driver(args.gripper)
        info = gripper.connect()
        print(f"  Gripper: {gripper.model} ({gripper.driver_id}) -> {info}")
    bridge = RealArmBridge(driver, robot_id=args.id, gripper=gripper)
    server = serve_http(bridge, port=args.port)

    print(f"  Real-robot arm stub ready on http://127.0.0.1:{args.port}")
    print(f"  Try:")
    print(f"    curl -X POST -H 'Content-Type: application/json' \\")
    print(f"      -d '{{\"text\":\"go home\"}}' http://127.0.0.1:{args.port}/prompt")
    print(f"  Ctrl-C to exit.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("  shutting down")
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
