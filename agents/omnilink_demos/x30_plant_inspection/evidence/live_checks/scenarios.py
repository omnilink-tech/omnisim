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

"""Live scenario checks against the X30 plant world's bridge (port 8796)."""
import json
import sys
import threading
import time
import urllib.request

BASE = "http://127.0.0.1:8796"
LOG = open(sys.argv[1] if len(sys.argv) > 1 else "scenarios.jsonl", "a", encoding="utf-8")


def post(path, body, timeout=620):
    req = urllib.request.Request(BASE + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            out = json.loads(r.read())
    except urllib.error.HTTPError as e:
        out = {"http_error": e.code, "body": json.loads(e.read() or b"{}")}
    return out, time.time() - t0


def state():
    with urllib.request.urlopen(BASE + "/state", timeout=10) as r:
        return json.loads(r.read())


def rec(name, **kw):
    s = state()
    kw.update(name=name, sim_time=s["sim_time"], pose=[round(s["x"], 3), round(s["y"], 3), round(s["yaw"], 3)],
              busy=s.get("busy"), speed=s.get("speed_mps"))
    LOG.write(json.dumps(kw) + "\n"); LOG.flush()
    print(json.dumps(kw)[:700], flush=True)


def wait_idle(limit=300):
    t0 = time.time()
    while time.time() - t0 < limit:
        if not state().get("busy"):
            return True
        time.sleep(0.5)
    return False


def prompt(text, name, timeout_s=600):
    out, dt = post("/prompt", {"text": text, "timeout_s": timeout_s})
    rec(name, text=text, wall_s=round(dt, 2), reply=out)
    return out


which = sys.argv[2].split(",") if len(sys.argv) > 2 else ["A", "B", "C", "D", "E", "F", "G"]

if "A" in which:
    prompt("turn left 90 degrees", "A_turn")
    wait_idle(); rec("A_after", last=state().get("last_motion"))
if "B" in which:
    prompt("Where are you right now?", "B_where")
if "C" in which:
    prompt("Go to the tank.", "C_ambiguous")
    wait_idle()
if "D" in which:
    res = {}
    th = threading.Thread(target=lambda: res.update(r=post("/prompt", {"text": "go to electrical cabinet E-3", "timeout_s": 600})))
    th.start()
    # wait until it is actually walking
    t0 = time.time()
    while time.time() - t0 < 60 and not (state().get("busy") and abs(state().get("speed_mps") or 0) > 0.15):
        time.sleep(0.3)
    rec("D_walking")
    out, dt = post("/prompt", {"text": "stop"})
    rec("D_stop", text="stop", wall_s=round(dt, 2), reply=out)
    th.join(120)
    rec("D_first_reply", reply=res.get("r"))
if "E" in which:
    res = {}
    th = threading.Thread(target=lambda: res.update(r=post("/prompt", {"text": "Go to tank T-1.", "timeout_s": 600})))
    th.start()
    t0 = time.time()
    while time.time() - t0 < 60 and not (state().get("busy") and abs(state().get("speed_mps") or 0) > 0.15):
        time.sleep(0.3)
    time.sleep(3)
    rec("E_walking")
    out, dt = post("/prompt", {"text": "Actually, change of plan: go to the control room instead.", "timeout_s": 600})
    rec("E_replan", wall_s=round(dt, 2), reply=out)
    th.join(120)
    rec("E_first_reply", reply=res.get("r"))
    wait_idle(); rec("E_after", last=state().get("last_motion"))
if "F" in which:
    prompt("walk forward 80 metres", "F_rail")
if "G" in which:
    prompt("Go to the boiler house.", "G_unknown_place")
