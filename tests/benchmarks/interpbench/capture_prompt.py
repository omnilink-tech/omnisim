"""Stand in for the model server and record what the bridge actually posts.

The costbench figure (5,592 tokens) came from a capture like this, but the
payload itself was never saved -- only the number. So it could not be reused
and had to be re-captured. This one is written to disk.

⚠️ Kills its own recorder. A leftover server on 11434 was once adopted as
the relay by a soak run, which then reported 25 perfect laps while the robot
sat still.
"""
import json, os, pathlib, subprocess, sys, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

ROOT = pathlib.Path("O:/omnisim")
# ⚠️ THE PAYLOAD IS PER ROBOT, AND IT WAS A CONSTANT.
#
# What this captures is the bridge's REAL system instruction and its REAL
# tool declarations -- for the world it was pointed at. That was hardcoded
# to the Husky, so `bridge_payload.json` opens "You drive a Clearpath
# Husky mobile base" and declares seventeen MOBILE tools.
#
# Every benchmark that loads it inherits that. Running the shift demo
# against a UR5e on 2026-09-21 told the model it was driving a Husky and
# offered it `drive_forward` / `turn`, which the arm bridge does not
# serve: 29 commands dispatched, 0.00 m of TCP motion, and a per-kind
# table that looked clean because four of its six rows only require the
# robot to stay still. The run was VOIDed by the harness rather than
# published, which is the only reason this is a note and not a result.
#
#     python tests/benchmarks/interpbench/capture_prompt.py #         --world projects/samples/demos/worlds/chat/omnilink_ur5e.omniworld #         --out tests/benchmarks/interpbench/bridge_payload_arm.json
import argparse

_ap = argparse.ArgumentParser()
_ap.add_argument("--world",
                 default="projects/samples/demos/worlds/chat/omnilink_husky.omniworld")
_ap.add_argument("--port", type=int, default=8765)
_ap.add_argument("--out",
                 default=str(ROOT / "tests/benchmarks/interpbench/bridge_payload.json"))
_ap.add_argument("--text", default="drive forward 2 metres",
                 help="one prompt, only to make the bridge build a payload")
_A = _ap.parse_args()

OUT = pathlib.Path(_A.out)
WORLD = _A.world
PORT = _A.port
captured = {}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n)
        try:
            payload = json.loads(body)
            if "messages" in payload and not captured:
                captured.update(payload)
        except Exception:
            pass
        out = json.dumps({"model": payload.get("model", "x"),
                          "message": {"role": "assistant",
                                      "content": "ok", "tool_calls": []},
                          "done": True, "done_reason": "stop"}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(out)))
        self.end_headers()
        self.wfile.write(out)

    def do_GET(self):
        out = json.dumps({"models": [{"name": "qwen3:8b"}]}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(out)))
        self.end_headers()
        self.wfile.write(out)


srv = HTTPServer(("127.0.0.1", 11434), H)
threading.Thread(target=srv.serve_forever, daemon=True).start()
print("recorder up on 11434")

env = {**os.environ, "OMNISIM_OLLAMA": "1",
       "OMNISIM_LOG_PATH": str(pathlib.Path(os.environ.get("TEMP", "/tmp")) / "cap.log")}
env.pop("OMNI_KEY", None)
proc = subprocess.Popen([sys.executable, "-m", "omnisim", "run-headless", WORLD,
                         "--duration", "120"], cwd=str(ROOT),
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
try:
    for _ in range(120):
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{PORT}/state", timeout=2); break
        except Exception:
            time.sleep(1)
    else:
        print("bridge never came up"); raise SystemExit(1)
    time.sleep(2)
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}/prompt", method="POST",
        data=json.dumps({"text": _A.text}).encode(),
        headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=90).read()
    except Exception as exc:
        print("prompt:", type(exc).__name__)
    if captured:
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(captured, indent=2), encoding="utf-8")
        sysmsg = next((m for m in captured["messages"] if m.get("role") == "system"), {})
        print(f"CAPTURED -> {OUT}")
        print(f"  system instruction: {len(sysmsg.get('content',''))} chars")
        print(f"  tools: {len(captured.get('tools') or [])}")
        print(f"  messages: {len(captured['messages'])}")
    else:
        print("NOTHING CAPTURED -- the relay never reached the recorder")
finally:
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)
    srv.shutdown()
    print("recorder down")
