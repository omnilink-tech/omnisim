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

"""Does a robot event reach the platform as a wake? Against the REAL platform.

WHAT THIS PROVES, AND WHAT IT DOES NOT
--------------------------------------
The wake path is: an event lands on the bridge's ring -> the relay's poll sees it
-> `may_wake` admits it -> the rate limiter admits it -> ONE relay turn is
dispatched through the same cascade as an operator sentence -> the platform ->
a model -> a reply to the event sink.

Every link up to and including "the platform" is exercised here for real: a
real `EventRing`, a real `OmniLinkRelay` on a real OmniKey, a real POST to the
live platform. The last link -- a MODEL answering -- is reported separately as
`model_answered`. A platform refusal (a 402 of any kind) still proves the wake
travelled the whole way, so it counts as `reached_platform`, but never as an
answer. Anything else (no reply at all, a reply to an event that should not
wake, more than one reply to a burst) is a real failure. Verified end to end on
2026-09-23: the model answered naming the event's commanded and achieved figures.

No engine is launched. The ring is fed directly, the way a detector feeds it.

Side effects, stated: heartbeat rows and a memory write under the scratch agent
name `OmniSim-trackd-wakeprobe`; the heartbeat row is deleted at the end.
No profile is created, so the account's profile limit is untouched.
"""
from __future__ import annotations

import os
import pathlib
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3]
                       / "packages" / "omnisim-bridges" / "src"))

AGENT = os.environ.get("TRACKD_WAKE_AGENT", "OmniSim-trackd-wakeprobe")
# ⚠️ Since 2026-09-23 the platform refuses to run any agent outside the plan's
# allowance (402 AGENT_LIMIT_REACHED), and a new scratch name counts as a new
# agent. On a full account set TRACKD_WAKE_AGENT to an EXISTING profile, e.g.
# OmniSim-husky, to get a real model answer. That writes one probe turn into
# that agent's memory; the heartbeat row is removed at the end as before.
os.environ["OMNILINK_EDGE"] = "0"          # no edge socket for a probe
os.environ["OMNILINK_USAGE"] = "0"
os.environ.setdefault("OMNILINK_PRESENCE", "1")   # the event poll rides this thread
os.environ.setdefault("OMNILINK_EVENT_WAKE", "1")

from omnisim_bridges import relay as R                 # noqa: E402
from omnisim_bridges.events import EventRing           # noqa: E402

KEY = os.environ["OMNI_KEY"]


class _Bridge:
    """The minimum a relay reads off a bridge: a ring and a few state fields."""

    def __init__(self) -> None:
        self.events = EventRing(robot="husky")
        self.sim_time = 0.0
        self.sim_step = 0
        self.held = False
        self.fault = None
        self.world = "wake_probe"
        self.robot_id = "husky"


def main() -> int:
    bridge = _Bridge()
    replies: list = []
    lock = threading.Lock()

    def sink(kind: str, payload: dict) -> None:
        if kind in ("agent", "error"):
            with lock:
                replies.append((time.time(), kind, str(payload.get("text", ""))[:200]))

    relay = R.OmniLinkRelay(omni_key=KEY, agent_name=AGENT,
                            main_task="wake probe", tools=[], bridge=bridge)
    relay.set_event_sink(sink)

    def wait_for(n: int, timeout: float) -> list:
        end = time.time() + timeout
        while time.time() < end:
            with lock:
                if len(replies) >= n:
                    return list(replies)
            time.sleep(0.25)
        with lock:
            return list(replies)

    results = {}

    # 1. A NON-waking event must cost nothing.
    bridge.events.emit("motion.unsettled", sim_time=1.0, step=62, robot="husky",
                       tool="drive_forward", error_m=0.04)
    time.sleep(4.0)
    with lock:
        results["non_waking"] = len(replies)
    print(f"[1] motion.unsettled (policy: no wake) -> replies {results['non_waking']} (want 0)")

    # 2. ONE qualifying event must produce exactly ONE turn that reaches the platform.
    t0 = time.time()
    bridge.events.emit("motion.timed_out", sim_time=2.0, step=125, robot="husky",
                       tool="drive_forward", commanded=5.0, achieved=1.2)
    got = wait_for(1, 60.0)
    results["one_event"] = len(got)
    if got:
        ts, kind, text = got[0]
        print(f"[2] motion.timed_out -> reply after {ts - t0:.1f}s, kind={kind!r}")
        print(f"      {text!r}")
        results["reached_platform"] = ("402" in text or "BYOK" in text
                                       or kind == "agent")
        # The last link, reported separately so a 402 can never read as it:
        # did a MODEL answer, in words, about the event?
        results["model_answered"] = (kind == "agent")
    else:
        print("[2] motion.timed_out -> NO reply within 60 s")
        results["reached_platform"] = False
        results["model_answered"] = False

    # 3. A BURST inside the rate-limit window must not buy a second turn.
    before = len(got)
    for i in range(20):
        bridge.events.emit("motion.timed_out", sim_time=3.0 + i * 0.1,
                           step=190 + i, robot="husky", tool="drive_forward",
                           commanded=5.0, achieved=1.0)
    time.sleep(6.0)
    with lock:
        results["burst_extra"] = len(replies) - before
    print(f"[3] burst of 20 within the window -> extra replies {results['burst_extra']} (want 0)")

    relay.close()
    try:
        import urllib.request, json
        req = urllib.request.Request(
            f"{R.BASE_URL}/api/relay-heartbeat",
            data=json.dumps({"agent": AGENT}).encode(), method="DELETE",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {KEY}"})
        urllib.request.urlopen(req, timeout=10).read()
        print("[cleanup] scratch heartbeat row deleted")
    except Exception as e:
        print(f"[cleanup] heartbeat delete skipped: {e}")

    ok = (results["non_waking"] == 0 and results["one_event"] >= 1
          and results["reached_platform"] and results["burst_extra"] == 0)
    print()
    print("=== VERDICT:", "WAKE REACHES THE PLATFORM, BOUNDED" if ok else "FAILED", results)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
