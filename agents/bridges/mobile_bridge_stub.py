# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Real-robot mobile-base bridge stub.

Mirror of `arm_bridge_stub.py` for wheeled bases. Replace `MockMobileDriver`
with your fleet's SDK and you have an OmniLink-controlled real Roomba /
Husky / Jackal / custom AGV.

Run:

    python agents/bridges/mobile_bridge_stub.py
    curl -X POST -H "Content-Type: application/json" \\
        -d '{"text":"forward 1 m"}' http://127.0.0.1:8765/prompt
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bridge_base import BridgeBase, serve_http  # noqa: E402


class MockMobileDriver:
    """Replace with your AGV / mobile-base SDK client."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_lin = 0.0
        self.v_ang = 0.0
        self.last_tick = time.time()

    def stop(self) -> Dict[str, Any]:
        print(f"  [MOCK_MOBILE] STOP")
        self.v_lin = 0.0
        self.v_ang = 0.0
        return {"halted_at": time.time()}

    def set_velocity(self, linear: float, angular: float) -> Dict[str, Any]:
        print(f"  [MOCK_MOBILE] set_velocity: linear={linear:+.2f} angular={angular:+.2f}")
        self.v_lin = linear
        self.v_ang = angular
        return {"accepted": True, "linear": linear, "angular": angular}

    def drive_forward(self, distance: float, speed: Optional[float] = None) -> Dict[str, Any]:
        actual = 0.5 if speed is None else float(speed)
        eta = abs(distance) / max(abs(actual), 1e-6)
        print(f"  [MOCK_MOBILE] drive_forward: distance={distance:+.2f}m eta={eta:.1f}s")
        return {"accepted": True, "distance": distance, "eta_s": eta}

    def turn(self, angle_rad: float) -> Dict[str, Any]:
        eta = abs(angle_rad) / 1.0  # 1 rad/s assumed
        print(f"  [MOCK_MOBILE] turn: angle={math.degrees(angle_rad):+.0f} deg eta={eta:.1f}s")
        return {"accepted": True, "angle_rad": angle_rad, "eta_s": eta}

    def reset_to_home(self) -> Dict[str, Any]:
        print(f"  [MOCK_MOBILE] reset_to_home (teleport to spawn)")
        self.x = self.y = self.yaw = 0.0
        return {"accepted": True}

    def get_state(self) -> Dict[str, Any]:
        return {
            "x": self.x, "y": self.y, "yaw": self.yaw,
            "v_linear": self.v_lin, "v_angular": self.v_ang,
            "last_tick_at": self.last_tick,
        }


class RealMobileBridge(BridgeBase):
    def __init__(self, driver: MockMobileDriver, robot_id: str = "real_mobile") -> None:
        self.driver = driver
        self.robot_id = robot_id
        self.model = "MockMobile"
        self.capabilities = {"max_linear_m_s": 1.0, "max_angular_rad_s": 2.0}

    def act_stop(self) -> Dict[str, Any]:
        return self.driver.stop()

    def act_set_velocity(self, linear: float, angular: float) -> Dict[str, Any]:
        return self.driver.set_velocity(linear, angular)

    def act_drive_forward(self, distance: float, speed: Optional[float] = None) -> Dict[str, Any]:
        return self.driver.drive_forward(distance, speed)

    def act_turn(self, angle_rad: float) -> Dict[str, Any]:
        return self.driver.turn(angle_rad)

    def act_reset_to_home(self) -> Dict[str, Any]:
        return self.driver.reset_to_home()

    def get_state(self) -> Dict[str, Any]:
        st = self.driver.get_state()
        st.setdefault("id", self.robot_id)
        st.setdefault("model", self.model)
        return st

    def act_prompt(self, text: str) -> Dict[str, Any]:
        s = text.strip().lower()
        if re.search(r"\b(stop|halt|brake)\b", s):
            self.act_stop()
            return {"response": "Stopping wheels.", "actions": [{"tool": "stop_robot", "result": "ok"}]}
        if re.search(r"\b(home|reset|dock)\b", s):
            self.act_reset_to_home()
            return {"response": "Teleporting home.", "actions": [{"tool": "reset_to_home", "result": "ok"}]}
        m = re.search(r"\b(forward|back|reverse)\b[^-\d]*(-?\d+\.?\d*)", s)
        if m:
            val = float(m.group(2))
            if m.group(1).startswith("back") or m.group(1) == "reverse":
                val = -val
            self.act_drive_forward(val)
            return {"response": f"Driving {val:+.2f} m.", "actions": [{"tool": "drive_forward", "result": "ok"}]}
        m = re.search(r"\bturn\s+(left|right)?\s*(-?\d+\.?\d*)\s*(deg|rad)?", s)
        if m:
            v = float(m.group(2))
            if (m.group(3) or "deg").startswith("deg"):
                v = math.radians(v)
            if m.group(1) == "right":
                v = -abs(v)
            self.act_turn(v)
            return {"response": f"Turning {math.degrees(v):+.0f} deg.", "actions": [{"tool": "turn", "result": "ok"}]}
        return {
            "response": "I don't recognise that. Try: 'forward 1 m', 'turn left 90', 'stop', 'home'.",
            "actions": [],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--id", default="real_mobile")
    args = parser.parse_args()

    bridge = RealMobileBridge(MockMobileDriver(), robot_id=args.id)
    server = serve_http(bridge, port=args.port)
    print(f"  Real-robot mobile stub on http://127.0.0.1:{args.port}")
    print(f"  Ctrl-C to exit.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
