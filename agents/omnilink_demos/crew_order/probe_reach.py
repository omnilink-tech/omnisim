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
"""Can the set be worked at all? The whole job, by typed tool calls, no model.

    python agents/omnilink_demos/crew_order/probe_reach.py <out_dir> [--key-file ...]

Hitch, tow to the pick pad, unhitch, pick the three parts onto the cart, hitch,
tow to shipping, unhitch, pick them into the crate -- every step through the
robots' own /tool door (the gate still vets each call), each result printed
with what was measured. If this fails, no agent can succeed on this layout.
"""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import world  # noqa: E402

import argparse  # noqa: E402
import os  # noqa: E402
import subprocess  # noqa: E402
import urllib.request  # noqa: E402

REPO = HERE.parents[2]
sys.path.insert(0, str(REPO))
from omnisim.paths import linux_runtime_env, resolve_omnisim_binary  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--key-file", default="O:/omnilink-keys/omni_key.txt")
a = ap.parse_args()
out = Path(a.out).resolve()
out.mkdir(parents=True, exist_ok=True)
wp = REPO / "projects/samples/demos/worlds/chat/.harness_crew_order.omniworld"
wp.write_text(world.world_text(500), encoding="utf-8")
binary = resolve_omnisim_binary()
env = linux_runtime_env(str(REPO), None)
env.update(OMNISIM_HOME=str(REPO), OMNISIM_NO_WINDOW="1", OMNISIM_LOG_PATH=str(out / "engine.log"),
           OMNISIM_FILM_DIR=str(out / "frames"), OMNILINK_INTENT_STATE_DIR=str(out / "intents"),
           OMNILINK_MEMORY="0", OMNILINK_PROFILE_SYNC="0", OMNILINK_EDGE="0", OMNILINK_USAGE="0",
           OMNI_KEY=Path(a.key_file).read_text(encoding="utf-8").strip())
env["PATH"] = os.pathsep.join([str(Path(binary).parent / "newton-runtime"), str(Path(binary).parent), env.get("PATH", "")])
log = open(out / "stdout.log", "w", encoding="utf-8")
proc = subprocess.Popen([binary, "--batch", "--mode=realtime", "--no-rendering", "--minimize", "--stdout", "--stderr",
                         str(wp)], cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT)
P = world.PORTS
ok = True


def post(port, path, body, timeout=120):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def get(port, path, timeout=10):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def tool(robot, name, **args):
    global ok
    r = post(P[robot], "/tool", {"tool": name, **args}, timeout=120)
    res = r.get("result", r)
    # `error` is a per-axis list on success (commanded - achieved); a failure carries a string.
    bad = r.get("status") not in (None, "ok") or (isinstance(res, dict) and (
        isinstance(res.get("error"), str) or res.get("accepted") is False))
    ok = ok and not bad
    keep = {k: res.get(k) for k in ("accepted", "error", "refused", "carrying", "released", "at", "aboard", "achieved", "error_m",
                                    "achieved_xy", "arrived", "picked", "held") if isinstance(res, dict) and k in res}
    print(f"{'FAIL' if bad else 'ok  '} {robot:6} {name} {args} -> {keep or str(r)[:300]}", flush=True)
    return res


try:
    t0 = time.monotonic()
    while True:
        try:
            [get(p, "/state") for p in P.values()]
            break
        except Exception:
            if proc.poll() is not None or time.monotonic() - t0 > 180:
                raise SystemExit("bridges never came up")
            time.sleep(2)
    px, sx = world.PLACES["pick cell"][0], world.PLACES["shipping"][0]
    tool("Tug", "attach_trolley", **{"def": "CART"})
    tool("Tug", "drive_to", x=px, y=0.0)
    cart = tool("Tug", "detach_trolley").get("at") or [px - world.CART_BEHIND_M, 0, 0]
    for i, c in enumerate(("red", "blue", "green")):
        tool("Picker", "pick", object=c)
        tool("Picker", "place", xyz=[cart[0] + world.CART_SLOTS[i][0], cart[1] + world.CART_SLOTS[i][1], 0.40])
    def on_deck(cart_at):
        objs = post(P["Picker"], "/tool", {"tool": "locate_objects"}).get("result", {}).get("objects", [])
        return {o["def"]: (o["world_xyz"][0] - cart_at[0], o["world_xyz"][1] - cart_at[1]) for o in objs}
    before = on_deck(cart)
    tool("Tug", "attach_trolley", **{"def": "CART"})
    tool("Tug", "drive_to", x=sx, y=0.0)
    cart = tool("Tug", "detach_trolley").get("at")
    after = on_deck(cart)
    slip = {k: round(((after[k][0] - before[k][0]) ** 2 + (after[k][1] - before[k][1]) ** 2) ** 0.5, 4) for k in before}
    print("slip on the deck over the tow (m):", slip, flush=True)
    for i, c in enumerate(("red", "blue", "green")):
        tool("Packer", "pick", object=c)
        tool("Packer", "place", xyz=[world.CRATE[0], world.CRATE[1] + (i - 1) * 0.10, 0.40])
    objs = tool("Packer", "locate_objects")
    for o in objs.get("objects", []):
        print("   ", o.get("def"), o.get("world_xyz"))
    print("PROBE", "PASS" if ok else "FAIL")
finally:
    subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
    proc.wait()
    wp.unlink(missing_ok=True)
