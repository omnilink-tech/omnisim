"""Deterministic probe of the fixed surfaces, one arm at a time.

Not the A/B -- a CONTROL for it. Three agent cells per arm cannot prove the
product changed; this can, because it is the same request sequence against
each arm's own harness/capture service with no model in the loop.

Run from inside an arm worktree:
    python probe_endpoints.py --arm armA --out armA_probe.json
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

HOST = "127.0.0.1"


def http(method, url, body=None, timeout=90):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
            return {"status": r.status, "bytes": len(raw),
                    "ctype": r.headers.get("Content-Type"),
                    "body": _peek(raw), "ms": round((time.time() - t0) * 1000)}
    except urllib.error.HTTPError as e:
        raw = e.read()
        return {"status": e.code, "bytes": len(raw),
                "ctype": e.headers.get("Content-Type"),
                "body": _peek(raw), "ms": round((time.time() - t0) * 1000)}
    except Exception as e:                                     # noqa: BLE001
        return {"status": None, "error": "%s: %s" % (type(e).__name__, e),
                "ms": round((time.time() - t0) * 1000)}


def _peek(raw, n=900):
    if raw[:8] == b"\x89PNG\r\n\x1a\n":
        return {"png": True, "bytes": len(raw)}
    try:
        return json.loads(raw.decode("utf-8", "replace"))
    except Exception:                                          # noqa: BLE001
        return raw[:n].decode("utf-8", "replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True)
    ap.add_argument("--hport", type=int, default=6889)
    ap.add_argument("--cport", type=int, default=6891)
    ap.add_argument("--world", default="projects/samples/demos/worlds/"
                                       "physics/newton_husky_swarm_drive.wbt")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    root = os.path.abspath(".")
    env = dict(os.environ)
    env["OMNISIM_HOME"] = root
    env.pop("WEBOTS_HOME", None)
    env["OMNISIM_NO_WINDOW"] = "1"
    env["OMNISIM_LOG_PATH"] = os.path.join(root, "_probe_log.txt")

    res = {"arm": a.arm, "root": root, "world": a.world, "probes": {}}
    hp = subprocess.Popen(
        [sys.executable, "scripts/harness/omnisim_harness.py",
         "--port", str(a.hport), "--supervisor-port", str(a.hport + 1)],
        cwd=root, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base = "http://%s:%d" % (HOST, a.hport)
    try:
        for _ in range(60):
            if http("GET", base + "/healthz", timeout=3).get("status") == 200:
                break
            time.sleep(1)
        res["probes"]["healthz"] = http("GET", base + "/healthz")
        # 1. capabilities on a harness with NO WORLD LOADED (fix 4)
        res["probes"]["capabilities_no_world"] = http(
            "GET", base + "/capabilities")
        # 2. load
        res["probes"]["world_load"] = http(
            "POST", base + "/world/load", {"path": a.world, "wait_s": 180},
            timeout=300)
        # 3. screenshot -- zero-byte body? (fix 3)
        res["probes"]["screenshot"] = http(
            "POST", base + "/world/screenshot", {}, timeout=180)
        # 4. /robots -- does it count the injected supervisor?
        res["probes"]["robots"] = http("GET", base + "/robots", timeout=120)
        # 5. step, then reset, then step, then read poses (fix 1)
        res["probes"]["step_200"] = http(
            "POST", base + "/sim/step", {"steps": 200}, timeout=300)
        res["probes"]["robots_after_step"] = http("GET", base + "/robots",
                                                  timeout=120)
        res["probes"]["reset"] = http("POST", base + "/sim/reset", {},
                                      timeout=180)
        res["probes"]["step_after_reset"] = http(
            "POST", base + "/sim/step", {"steps": 400}, timeout=300)
        res["probes"]["robots_after_reset"] = http("GET", base + "/robots",
                                                   timeout=120)
        res["probes"]["sim_state"] = http("GET", base + "/sim/state")
    finally:
        try:
            hp.terminate()
            hp.wait(timeout=20)
        except Exception:                                      # noqa: BLE001
            hp.kill()

    # --- capture service: the path forms (2010888cc) -------------------------
    cp = subprocess.Popen(
        [sys.executable, "scripts/capture/omnisim_capture.py",
         "--port", str(a.cport), "--supervisor-port", str(a.cport + 1)],
        cwd=root, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    cbase = "http://%s:%d" % (HOST, a.cport)
    try:
        for _ in range(60):
            if http("GET", cbase + "/healthz", timeout=3).get("status"):
                break
            time.sleep(1)
        res["probes"]["capture_load"] = http(
            "POST", cbase + "/world/load",
            {"path": a.world, "width": 640, "height": 360}, timeout=300)
        res["probes"]["capture_shot_bare"] = http(
            "POST", cbase + "/capture/screenshot", {}, timeout=180)
        res["probes"]["capture_shot_relname"] = http(
            "POST", cbase + "/capture/screenshot", {"path": "probe_bare.png"},
            timeout=180)
        res["probes"]["capture_shot_relsubdir"] = http(
            "POST", cbase + "/capture/screenshot",
            {"path": "probe_out/probe_sub.png"}, timeout=180)
        res["probes"]["capture_shot_bad"] = http(
            "POST", cbase + "/capture/screenshot",
            {"path": "Z:/definitely/not/writable/x.png"}, timeout=180)
    finally:
        try:
            cp.terminate()
            cp.wait(timeout=20)
        except Exception:                                      # noqa: BLE001
            cp.kill()

    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    for k, v in res["probes"].items():
        print("%-26s status=%s bytes=%s %s"
              % (k, v.get("status"), v.get("bytes"),
                 str(v.get("body"))[:130].replace("\n", " ")))
    print("\nwrote %s" % a.out)


if __name__ == "__main__":
    main()
