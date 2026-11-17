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
"""Record one take of the crew-order film.

    python agents/omnilink_demos/crew_order/take.py <take_dir> [--model claude-opus-5-5]
    python agents/omnilink_demos/crew_order/take.py <dir> --smoke      look-check: no robots, a few frames

One person (Sam, the shift lead) gives ONE order to ONE robot (Picker). From
there the three robots, each its own OmniLink agent, work it out by talking
to each other over the crew channel. Nothing else is sent to any robot after
the order: the take script only watches.

Before the order each robot hears one briefing from Sam (its job and the
facts of the floor) and the tug is hitched to the cart. Those are recorded too.

Writes <take>/crew.jsonl (every operator line, crew message and reply, with
simulator time), <take>/frames/<cam>/<sim ms>.jpg, the relay trace, the model
replies and the engine log.
"""
from __future__ import annotations

import argparse
import json
import re
import os
import shutil
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO))
import world  # noqa: E402

from omnisim.paths import linux_runtime_env, resolve_omnisim_binary  # noqa: E402

OPERATOR = "Sam"
ORDER = ("Sam: Order 42 is one red, one blue and one green part, packed in the shipping crate. "
         "Work it out with Tug and Packer, and tell me when it's packed.")
PX, PY = world.PLACES["pick cell"]
SX, SY = world.PLACES["shipping"]
CX, CY = world.CRATE
BRIEFINGS = {
    "Picker": (f"Sam: Briefing, nothing to do yet. You're Picker, the arm at the parts shelf. The red, blue and "
               f"green parts sit on your shelf. Tug brings an empty cart to the yellow pad in front of you and "
               f"must unhitch it before you load it. You can't see the cart yourself, so load it where Tug tells "
               f"you it is. Drop the three parts at z 0.40 at these offsets from the cart's centre: (0, 0), "
               f"(+0.10 m in x, 0) and (+0.05 m in x, +0.10 m in y); the cart's west end is out of your reach. "
               f"When all three parts are on the cart, ask Tug to take it to Packer at shipping; Packer "
               f"unloads it there into the crate and tells Sam when the order is packed."),
    "Tug": (f"Sam: Briefing, nothing to do yet. You're Tug. You tow the cart, called CART, which is hitched behind "
            f"you now, its centre {world.CART_BEHIND_M:.2f} m behind yours. There are two yellow pads on the floor: the "
            f"cart must stand on the pick pad at x {PX - world.CART_BEHIND_M:.1f}, y {PY:.1f} for Picker to load it, "
            f"and on the shipping pad at x {SX - world.CART_BEHIND_M:.1f}, y {SY:.1f} for Packer to unload it. So you "
            f"stop at x {PX:.2f} for the pick pad and at x {SX:.2f} for the shipping pad. Unhitch the cart at a pad so "
            f"an arm can work on it, and hitch it again (attach_trolley CART) before you drive off. The arms can't see "
            f"the cart, so tell them where you left it."),
    "Packer": (f"Sam: Briefing, nothing to do yet. You're Packer, the arm at shipping. Parts arrive on a cart Tug "
               f"brings to the yellow pad in front of you; locate_objects shows where they are. The shipping crate "
               f"is beside you at x {CX:.2f}; drop parts into it at z 0.40, at y {CY - 0.10:.2f}, {CY:.2f} and "
               f"{CY + 0.10:.2f}. When every part is in the crate, tell Sam the order is packed, with the "
               f"positions you measured."),
}


# The briefings without their spoken preamble: what a STANDING briefing says.
PREAMBLE = "Sam: Briefing, nothing to do yet. "
TUG_PLACES = {"pick pad stop": (PX, PY), "shipping pad stop": (SX, SY)}


def standing_briefings() -> dict:
    out = {n: t[len(PREAMBLE):] if t.startswith(PREAMBLE) else t for n, t in BRIEFINGS.items()}
    out["Tug"] += (" Your two stops are saved as places: 'pick pad stop' and 'shipping pad stop'.")
    return out


def post(port, path, body, timeout=240):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def get(port, path, timeout=10):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


BRIDGE_SCRIPTS = ("omnilink_arm_bridge.py", "omnilink_mobile_bridge.py", "film_camera.py")


def reap_bridges(ports: dict) -> int:
    """Kill bridge controllers left on THIS run's ports by an engine that died
    without them. An orphan keeps its port and answers for the next run (2026-10-21:
    three orphans on 8800-8802 failed every later run in that slot to start)."""
    try:
        import psutil
    except ImportError:
        return 0
    wanted = {str(p) for p in ports.values()}
    n = 0
    for proc in psutil.process_iter(["pid", "cmdline"]):
        cmd = proc.info.get("cmdline") or []
        if not any(c.endswith(BRIDGE_SCRIPTS) for c in cmd):
            continue
        if "--port" in cmd and cmd.index("--port") + 1 < len(cmd) and cmd[cmd.index("--port") + 1] in wanted:
            try:
                proc.kill()
                n += 1
            except Exception:
                pass
    return n


RATES = {"claude-opus-5-5": (4.0, 0.20, 20.0), "claude-sonnet-5-5": (2.0, 0.20, 10.0)}   # $/MTok in, cached, out


def in_crate(xyz) -> bool:
    cx, cy = world.CRATE
    return abs(xyz[0] - cx) < 0.15 and abs(xyz[1] - cy) < 0.20 and world.CRATE_FLOOR_Z - 0.01 < xyz[2] < 0.46


def _jsonl(path: Path) -> list:
    """Rows of a JSONL file, skipping a line still being written (campaign
    run 3: the relay was mid-write when the verdict read its trace)."""
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


_DONE = re.compile(r"(?i)\bpacked\b|\b(?:in|into) the (?:shipping )?crate\b")
_NOT_DONE = re.compile(r"(?i)not (?:yet |been )?packed|isn't packed|is not packed|not packed yet|not (?:yet )?in the")


def report_time(rows: list):
    """Simulator time Sam was told the order is DONE, or None: a message to
    him, sent after Tug delivered the cart to shipping (its last message to
    Packer), saying the parts are packed or in the crate. A progress update
    before the delivery does not count, whatever it promises ("Packer will
    unload into the crate")."""
    msgs = [m for m in rows if m.get("kind") == "message"]
    delivered = max((m["sim_s"] for m in msgs if m.get("from") == "Tug" and m.get("to") == "Packer"), default=None)
    for m in msgs:
        if m.get("to") != OPERATOR or (delivered is not None and m["sim_s"] < delivered):
            continue
        said = f"{m.get('text') or ''} {m.get('details') or ''}"
        if _DONE.search(said) and not _NOT_DONE.search(said):
            return m["sim_s"]
    return None


def reported_packed(rows: list) -> bool:
    return report_time(rows) is not None


def judge(take: Path, wall_s: float, model: str = "claude-opus-5-5") -> dict:
    """One run's verdict, from what was measured: every part in the crate
    (the final scan), a report to Sam, and every error the crew log saw."""
    rows = _jsonl(take / "crew.jsonl")
    final = next((r for r in reversed(rows) if r["kind"] == "final"), {})
    objs = (((final.get("locate_objects") or {}).get("result") or {}).get("objects")) or []
    parts = {o["def"]: o["world_xyz"] for o in objs}
    packed = len(parts) == 3 and all(in_crate(p) for p in parts.values())
    msgs = [r for r in rows if r["kind"] == "message"]
    reported = reported_packed(rows)
    # A gate refusal reads "<tool>: <rule>[; <tool>: <rule>]"; anything else
    # (a connection failure, an exception) is a delivery error.
    errs = [str(r["error"]) for r in rows if r["kind"] == "reply" and r.get("error")]
    refusals = [e for e in errs if re.fullmatch(r"[a-z_]+: [a-z_]+(?:; [a-z_]+: [a-z_]+)*", e)]
    delivery = [e for e in errs if e not in refusals]
    P = C = W = O = 0
    rounds = 0
    if (take / "relay_trace.jsonl").exists():
        for r in _jsonl(take / "relay_trace.jsonl"):
            if r.get("kind") == "round":
                rounds += 1
                P += r.get("prompt_tokens") or 0
                C += r.get("cached_tokens") or 0
                W += r.get("cache_write_tokens") or 0
                O += r.get("output_tokens") or 0
    rin, rc, rout = RATES.get(model, RATES["claude-opus-5-5"])
    cost = ((P - C - W) * rin + C * rc + W * rin * 1.25 + O * rout) / 1e6
    order_t = next((r["sim_s"] for r in rows if r["kind"] == "operator" and "Order" in r["text"]), None)
    report_t = next((m["sim_s"] for m in msgs if m["to"] == OPERATOR), None)
    return {"ok": packed and reported and not delivery and "error" not in final,
            "packed": packed, "reported": reported, "parts": parts,
            "messages": len(msgs), "refusals": refusals, "delivery_errors": delivery,
            "final_error": final.get("error"), "model_rounds": rounds, "cost_usd": round(cost, 3),
            "sim_s_order_to_report": (round(report_t - order_t, 1) if order_t is not None and report_t is not None else None),
            "wall_s": round(wall_s, 1)}


def main():
    # Robots write "−" and "…"; a cp1252 console must not crash the watch loop
    # (take 7 died here, engine and all).
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("take")
    ap.add_argument("--model", default="claude-opus-5-5")
    ap.add_argument("--engine", default="g4-engine")
    ap.add_argument("--key-file", default="O:/omnilink-keys/omni_key.txt")
    ap.add_argument("--frame-ms", type=int, default=64,
                    help="simulated ms between frames: 64 = every second 32 ms step, smooth at 2x playback")
    ap.add_argument("--wall-limit-s", type=float, default=900)
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--port-base", type=int, default=8765, help="Picker, Tug, Packer on base, +1, +2")
    ap.add_argument("--cams", default=",".join(world.FILM_CAMERAS),
                    help="comma-separated cameras to film (world.CAMERAS); empty = none")
    ap.add_argument("--tag", default="", help="OMNILINK_AGENT_TAG: this run's own agent identity and memory")
    ap.add_argument("--quiet-s", type=float, default=60.0, help="wall s of crew silence after the report that ends a take")
    ap.add_argument("--briefing", choices=("standing", "prompt"), default="standing",
                    help="standing: each robot starts with its briefing in its own instructions (no model turn); "
                         "prompt: Sam says each briefing as a chat turn first (the original takes)")
    a = ap.parse_args()
    ports = world.ports_from(a.port_base)
    cams = tuple(c for c in a.cams.split(",") if c)

    take = Path(a.take).resolve()
    if take.exists():
        shutil.rmtree(take)
    (take / "frames").mkdir(parents=True)
    log_path = take / "crew.jsonl"
    world_path = REPO / f"projects/samples/demos/worlds/chat/.harness_crew_order_{a.port_base}.omniworld"
    world_path.write_text(world.world_text(500 if a.smoke else a.frame_ms, controllers=not a.smoke,
                                           ports=ports, cameras=cams), encoding="utf-8")

    binary = resolve_omnisim_binary()
    env = linux_runtime_env(str(REPO), None)
    env.update(OMNISIM_HOME=str(REPO), OMNISIM_NO_WINDOW="1", OMNISIM_LOG_PATH=str(take / "engine.log"),
               OMNISIM_FILM_DIR=str(take / "frames"),
               OMNILINK_CREW=world.crew_env(ports), OMNILINK_CREW_OPERATOR=OPERATOR, OMNILINK_CREW_LOG=str(log_path),
               OMNILINK_ENGINE=a.engine, OMNILINK_MODEL=a.model,
               OMNILINK_TRACE=str(take / "relay_trace.jsonl"),
               OMNILINK_MODEL_RECORD=str(take / "model_replies.jsonl"),
               OMNILINK_INTENT_STATE_DIR=str(take / "intents"),
               **({"OMNILINK_CREW_BRIEFINGS": str(take / "briefings.json")} if a.briefing == "standing" else {}),
               OMNILINK_PRESENCE="1", OMNILINK_EVENT_WAKE="1", OMNILINK_MEMORY="0",
               OMNILINK_PROFILE_SYNC="0", OMNILINK_EDGE="0", OMNILINK_USAGE="0")
    if a.tag:
        env["OMNILINK_AGENT_TAG"] = a.tag
    if not a.smoke:
        env["OMNI_KEY"] = Path(a.key_file).read_text(encoding="utf-8").strip()
    env["PATH"] = os.pathsep.join([str(Path(binary).parent / "newton-runtime"), str(Path(binary).parent), env.get("PATH", "")])
    (take / "briefings.json").write_text(json.dumps(standing_briefings(), indent=1), encoding="utf-8")
    (take / "meta.json").write_text(json.dumps({"model": a.model, "engine": a.engine, "briefing": a.briefing,
                                                "port_base": a.port_base, "tag": a.tag}, indent=1), encoding="utf-8")
    reaped = reap_bridges(ports)
    if reaped:
        print(f"reaped {reaped} orphaned bridge(s) on ports {sorted(ports.values())}", flush=True)
    out = open(take / "stdout.log", "w", encoding="utf-8")
    proc = subprocess.Popen([binary, "--batch", "--mode=realtime", "--no-rendering", "--minimize", "--stdout",
                             "--stderr", str(world_path)], cwd=REPO, env=env, stdout=out, stderr=subprocess.STDOUT,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def note(record):
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

    try:
        if a.smoke:
            t0 = time.monotonic()
            while time.monotonic() - t0 < 120 and sum(1 for _ in (take / "frames").rglob("*.jpg")) < 8:
                time.sleep(1)
            print(sorted(str(f) for f in (take / "frames").rglob("*.jpg"))[-2:])
            return
        t0 = time.monotonic()
        while True:
            try:
                for p in ports.values():
                    get(p, "/state")
                break
            except Exception:
                if proc.poll() is not None or time.monotonic() - t0 > 180:
                    raise SystemExit("bridges never came up; see stdout.log")
                time.sleep(2)
        sim = lambda: float((get(ports["Tug"], "/state") or {}).get("sim_time") or 0.0)
        hitch = post(ports["Tug"], "/tool", {"tool": "attach_trolley", "def": "CART"})
        note({"kind": "setup", "sim_s": sim(), "wall": time.time(), "to": "Tug", "text": "attach_trolley CART", "result": hitch})
        print("hitch:", hitch.get("result", hitch), flush=True)
        if a.briefing == "standing":
            note({"kind": "setup", "sim_s": sim(), "wall": time.time(), "to": "all",
                  "text": "standing briefings", "briefings": standing_briefings()})
            for place, (x, y) in TUG_PLACES.items():
                res = post(ports["Tug"], "/tool", {"tool": "remember_place", "name": place, "x": x, "y": y})
                note({"kind": "setup", "sim_s": sim(), "wall": time.time(), "to": "Tug",
                      "text": f"remember_place {place} ({x:.2f}, {y:.2f})", "result": res})
        for name, text in (BRIEFINGS.items() if a.briefing == "prompt" else ()):
            note({"kind": "operator", "sim_s": sim(), "wall": time.time(), "from": OPERATOR, "to": name, "text": text})
            r = post(ports[name], "/prompt", {"text": text, "timeout_s": 200})
            note({"kind": "reply", "sim_s": sim(), "wall": time.time(), "from": name, "to": OPERATOR,
                  "text": r.get("response", ""), "via": r.get("via"), "actions": r.get("actions"), "error": r.get("error", "")})
            print(f"briefed {name}: {r.get('response', '')[:140]!r}", flush=True)
        note({"kind": "operator", "sim_s": sim(), "wall": time.time(), "from": OPERATOR, "to": "Picker", "text": ORDER})
        r = post(ports["Picker"], "/prompt", {"text": ORDER, "timeout_s": 200})
        note({"kind": "reply", "sim_s": sim(), "wall": time.time(), "from": "Picker", "to": OPERATOR,
              "text": r.get("response", ""), "via": r.get("via"), "actions": r.get("actions"), "error": r.get("error", "")})
        print(f"order -> Picker: {r.get('response', '')[:200]!r}", flush=True)
        # Watch only. Done once every part is MEASURED in the crate, someone has
        # told Sam it is packed, and the crew has been quiet for a while -- or,
        # packed but never reported, after a longer silence (300 s: under four
        # engines a crewmate's last turn can run past two minutes, pilot v3
        # run 2). A progress update
        # to Sam ("Order 42 under way") ends nothing (Sonnet pilot run 2 was
        # cut off mid-load by exactly that).
        seen, quiet_since, last_scan, packed_now = 0, time.monotonic(), 0.0, False
        while time.monotonic() - t0 < a.wall_limit_s and proc.poll() is None:
            lines = log_path.read_text(encoding="utf-8").splitlines()
            for line in lines[seen:]:
                rec = json.loads(line)
                print(f"[{rec.get('sim_s', 0):7.1f}] {rec.get('kind'):8} {rec.get('from')} -> {rec.get('to')}: {str(rec.get('text'))[:160]}", flush=True)
                quiet_since = time.monotonic()
            seen = len(lines)
            try:  # the tug's track, for the edit's name tag (not part of the crew log)
                st = get(ports["Tug"], "/state")
                with open(take / "tug_track.jsonl", "a", encoding="utf-8") as tf:
                    tf.write(json.dumps({"sim_s": st.get("sim_time"), "x": st.get("x"), "y": st.get("y"),
                                         "carrying": st.get("carrying")}) + "\n")
            except Exception:
                pass
            if time.monotonic() - last_scan > 10:
                last_scan = time.monotonic()
                try:
                    objs = post(ports["Picker"], "/tool", {"tool": "locate_objects"}, timeout=8)
                    parts = (objs.get("result") or {}).get("objects") or []
                    packed_now = len(parts) == 3 and all(in_crate(o["world_xyz"]) for o in parts)
                except Exception:
                    pass
            quiet = time.monotonic() - quiet_since
            if packed_now and ((reported_packed(_jsonl(log_path)) and quiet > a.quiet_s) or quiet > 300):
                break
            time.sleep(2)
        try:
            objs = post(ports["Packer"], "/tool", {"tool": "locate_objects"})
            note({"kind": "final", "sim_s": sim(), "wall": time.time(), "locate_objects": objs})
        except Exception as e:
            note({"kind": "final", "error": str(e)})
        verdict = judge(take, time.monotonic() - t0, a.model)
        (take / "result.json").write_text(json.dumps(verdict, indent=1), encoding="utf-8")
        print("RESULT", json.dumps({k: verdict[k] for k in ("ok", "packed", "reported", "messages", "refusals",
                                                            "delivery_errors", "cost_usd", "wall_s")}), flush=True)
    finally:
        # Our own engine: take its controllers with it, or an orphaned bridge
        # keeps its port and answers for the next take.
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
        proc.wait()
        reap_bridges(ports)
        out.close()
        world_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
