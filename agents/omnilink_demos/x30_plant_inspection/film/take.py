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

"""Record one take of the X30 plant-inspection OmniLink demo.

    python -m omnisim capture --port 6893          # with OMNI_KEY set, and
    #   OMNISIM_CAPTURE_SUPERVISOR_PORT_OVERRIDE=6894 OMNILINK_PROFILE_SYNC=0
    python agents/omnilink_demos/x30_plant_inspection/film/take.py <take_dir>

PROMPTS below is the whole script of the film: (sentence, show the robot's
answer on screen?). "__STOP_AFTER__<s>" sends "Stop." <s> sim-seconds into
the order before it. CAMERA_AT re-aims the follow camera. Takes ~1 h on a
laptop: the capture camera renders 1600x900 on sim time (~0.1-0.15x).
X30_CAPTURE / X30_BRIDGE override the two service URLs.

Drives the robot ONLY through its real POST /prompt endpoint and records
native OmniSim frames on sim time (capture service, follow camera). Writes
events.jsonl (every prompt: dispatch sim time, reply, reply sim time, wall
latency, robot state) beside the frames, for the edit and as evidence.
"""
import json
import os
import threading
import sys
import time
import urllib.request

CAP = os.environ.get("X30_CAPTURE", "http://127.0.0.1:6893")
BRIDGE = os.environ.get("X30_BRIDGE", "http://127.0.0.1:8796")
TAKE = sys.argv[1]
WORLD = "projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld"
PROMPTS = [
    ("Walk forward 2 metres.", False),
    ("Head over to the feed pump and check on it.", False),
    ("Go to the tank.", True),
    ("Tank T-2, please.", False),
    ("Now go to valve manifold V-7.", False),
    ("__STOP_AFTER__6", False),           # 'Stop.' 6 s of walking into the previous order
    ("Where are you?", True),
    ("Change of plan: inspect tank T-1, then return to the dock.", False),
]
NE = [3.2, 3.4, 2.2]          # camera north-east of the robot (clear of the control room)
ESE = [3.8, -2.4, 2.3]        # east-south-east, for the tank farm
CAMERA_AT = {3: ESE}          # prompt number (1-based) -> offset to ease to when it is sent


def http(method, url, body=None, timeout=660):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def state():
    return http("GET", BRIDGE + "/state", timeout=10)


os.makedirs(TAKE, exist_ok=True)
ev = open(os.path.join(TAKE, "events.jsonl"), "w", encoding="utf-8")


def log(**kw):
    kw["wall"] = time.time()
    ev.write(json.dumps(kw) + "\n")
    ev.flush()
    print(json.dumps(kw)[:400], flush=True)


print(http("POST", CAP + "/world/load", {"path": WORLD, "width": 1600, "height": 900, "fov": 0.9},
           timeout=300).get("ok"))
for _ in range(120):
    try:
        s = state()
        if s.get("mode") == "driver":
            break
    except Exception:
        pass
    time.sleep(1)
time.sleep(2)
rec = http("POST", CAP + "/capture/record/start", {
    "dir": os.path.abspath(os.path.join(TAKE, "frames")), "period_ms": 40, "quality": 100,
    "track": ["X30"],
    "follow": {"def": "X30", "offset": NE, "look": [0.0, 0.0, 0.25], "smooth_s": 1.2}})
log(event="record_start", rec=rec, state=state())
time.sleep(3)

import threading
pending = {}


def camera(offset, move_s=4.0):
    log(event="camera", offset=offset, move_s=move_s,
        res=http("POST", CAP + "/capture/record/follow", {"offset": offset, "move_s": move_s}))


def watch_home_leg():
    """Swing the camera north-east once the robot leaves tank T-1 for home."""
    t0 = time.time()
    while time.time() - t0 < 900:
        lm = (state().get("last_motion") or {})
        if lm.get("label") == "tank T-1" and lm.get("settled") is not None:
            camera(NE, 5.0)
            return
        time.sleep(0.5)


def send(i, text, show_answer=False):
    if i in CAMERA_AT:
        camera(CAMERA_AT[i])
    if text.startswith("Change of plan"):
        threading.Thread(target=watch_home_leg, daemon=True).start()
    s0 = state()
    log(event="dispatch", i=i, text=text, show_answer=show_answer, sim=s0["sim_time"],
        pose=[s0["x"], s0["y"], s0["yaw"]])
    t0 = time.time()
    out = http("POST", BRIDGE + "/prompt", {"text": text, "timeout_s": 600})
    s1 = state()
    log(event="reply", i=i, text=text, sim=s1["sim_time"], wall_s=round(time.time() - t0, 2),
        reply=out, pose=[s1["x"], s1["y"], s1["yaw"]], last_motion=s1.get("last_motion"))
    return out


def wait_idle(limit=400):
    t0 = time.time()
    while time.time() - t0 < limit:
        if not state().get("busy"):
            return
        time.sleep(0.4)


i = 0
bg = None
for text, answer in PROMPTS:
    i += 1
    if text.startswith("__STOP_AFTER__"):
        # interrupt the order running in the background thread
        delay = float(text.split("__")[-1])
        t0 = time.time()
        s0 = state()["sim_time"]
        while state()["sim_time"] - s0 < delay and time.time() - t0 < 120:
            time.sleep(0.2)
        send(i, "Stop.")
        if bg is not None:
            bg.join(300)
            bg = None
        wait_idle()
        time.sleep(2.5)
        continue
    nxt = PROMPTS[i][0] if i < len(PROMPTS) else ""
    if nxt.startswith("__STOP_AFTER__"):
        # this order will be interrupted: run it in the background, and wait
        # until the robot is actually walking before the stop is timed
        bg = threading.Thread(target=send, args=(i, text, answer))
        bg.start()
        t0 = time.time()
        while time.time() - t0 < 120:
            st = state()
            if st.get("busy") and abs(st.get("speed_mps") or 0) > 0.15:
                break
            time.sleep(0.2)
        continue
    send(i, text, answer)
    wait_idle()
    time.sleep(2.5)

time.sleep(2)
log(event="record_stop", rec=http("POST", CAP + "/capture/record/stop", {}), state=state())
