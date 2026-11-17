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

"""OmniBench lane 3c — agent-driveability benchmark.

Machine-scores the agent-facing surface of the simulator: 10 capability probes
against a live validation harness, each pass/fail + latency. Score = passed/N.
The probe list is documented in DRIVEABILITY.md so other simulators can be
hand-scored against the same rubric.

The script starts its OWN harness on non-default ports (default 6889/6890, so a
user's harness on 6789 is untouched), runs the probes, tears the harness (and
its engine child) down, and writes one SPEC row per probe plus a summary row.

Windows notes: run with the SYSTEM python (Pillow needed for PNG verification);
the script prepends the bundled msys64/mingw64/bin (and C:/msys64/mingw64/bin
when present) to the harness's PATH so the engine finds its Qt DLLs.

Usage:
    python tests/benchmarks/omnibench/lane3/driveability.py
"""
from __future__ import annotations

import argparse
import io
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _row  # noqa: E402  (also puts OMNIBENCH on sys.path for `common`)
from common import engine_launch as _launch  # noqa: E402

REPO = _row.repo_root()
WORLD_SRC = _row.LANE3 / "worlds" / "lane3_drive.wbt"
LOAD_TIMEOUT_S = 240        # cold loads on slow disks measured 46-79 s in-tree
HTTP_TIMEOUT_S = 90
HOT_RELOAD_THRESHOLD_S = 30.0
STEP_DET_TOL = 1e-9

BROKEN_WBT = """#VRML_SIM R2025a utf8
WorldInfo {
Viewpoint { position 0 0 5 }
DEF X Solid { translation [ this is deliberately not valid VRML
"""


# ---------------------------------------------------------------------------
# tiny HTTP client (stdlib only)
# ---------------------------------------------------------------------------
class Http:
    def __init__(self, base):
        self.base = base

    def _req(self, method, path, body=None, timeout=HTTP_TIMEOUT_S):
        url = self.base + path
        data = None
        headers = {}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read()
                ctype = resp.headers.get("Content-Type", "")
                status = resp.status
        except urllib.error.HTTPError as e:
            raw = e.read()
            ctype = e.headers.get("Content-Type", "") if e.headers else ""
            status = e.code
        latency = (time.perf_counter() - t0) * 1000.0
        parsed = None
        if "json" in ctype or (raw[:1] in (b"{", b"[")):
            try:
                parsed = json.loads(raw.decode("utf-8"))
            except Exception:
                parsed = None
        return status, parsed, raw, ctype, latency

    def get(self, path, timeout=HTTP_TIMEOUT_S):
        return self._req("GET", path, None, timeout)

    def post(self, path, body=None, timeout=HTTP_TIMEOUT_S):
        return self._req("POST", path, body if body is not None else {}, timeout)


# ---------------------------------------------------------------------------
# harness lifecycle
# ---------------------------------------------------------------------------
def _port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def pick_ports(base):
    for cand in (base, base + 20, base + 40, base + 60):
        if _port_free(cand) and _port_free(cand + 1):
            return cand, cand + 1
    raise RuntimeError("no free port pair found near %d" % base)


def start_harness(port, sup_port, tmp):
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = os.path.join(tmp, "harness_engine.log.txt")
    if os.name == "nt":
        prefixes = [str(REPO / "msys64" / "mingw64" / "bin")]
        if Path("C:/msys64/mingw64/bin").is_dir():
            prefixes.append("C:\\msys64\\mingw64\\bin")
        env["PATH"] = os.pathsep.join(prefixes) + os.pathsep + env.get("PATH", "")
    out = open(os.path.join(tmp, "harness_stdout.log"), "w",
               encoding="utf-8", errors="replace")
    kwargs = {}
    if os.name == "nt":
        # own process group so CTRL_BREAK reaches harness + engine child only
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    proc = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "harness",
         "--port", str(port), "--supervisor-port", str(sup_port)],
        cwd=str(REPO), env=env, stdout=out, stderr=subprocess.STDOUT, **kwargs)
    return proc, env["OMNISIM_LOG_PATH"]


def wait_healthy(http, proc, timeout_s=60):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if proc.poll() is not None:
            return False
        try:
            status, parsed, _, _, _ = http.get("/healthz", timeout=3)
            if status == 200:
                return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def stop_harness(proc, tmp):
    """Graceful signal first (harness terminates its engine child in its
    handler), then a hard tree-kill, then reap engines the harness itself
    orphaned. The broken-world probe exercises the harness's engine-relaunch
    machinery, which has been observed to leave a superseded omnisim-bin alive
    past the harness's own shutdown — identify OURS by the unique tmp dir in
    the engine's command line and kill only those (never someone else's sim)."""
    if proc.poll() is None:
        try:
            if os.name == "nt":
                proc.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                proc.send_signal(signal.SIGTERM)
            proc.wait(timeout=15)
        except Exception:
            try:
                if os.name == "nt":
                    subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                                   timeout=30, capture_output=True)
                else:
                    proc.kill()
            except Exception:
                pass
    # reap orphaned engines belonging to THIS run only. Marker: the unique tmp
    # dir BASENAME (e.g. omnibench_l3_drv_ab12cd) — basename, so no backslash
    # escaping games between WQL and -like.
    if os.name == "nt":
        marker = os.path.basename(str(tmp)).replace("'", "''")
        ps = ("Get-CimInstance Win32_Process -Filter \"Name='omnisim-bin.exe'\" | "
              "Where-Object { $_.CommandLine -like '*%s*' } | "
              "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" % marker)
        try:
            subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                           timeout=60, capture_output=True)
        except Exception:
            pass
    else:
        try:
            subprocess.run(["pkill", "-f", "omnisim-bin.*%s" % tmp],
                           timeout=30, capture_output=True)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# probes — each returns (passed: bool, latency_ms: float, extra: dict)
# ---------------------------------------------------------------------------
def _tree_pos(http, defname):
    status, parsed, _, _, lat = http.get("/scene/tree")
    if status != 200 or not isinstance(parsed, dict):
        return None, lat
    for node in parsed.get("nodes", parsed.get("tree", [])) or []:
        if node.get("def") == defname or node.get("def_name") == defname:
            return node.get("position"), lat
    return None, lat


def probe_load_valid(http, world_path):
    t0 = time.perf_counter()
    status, parsed, _, _, _ = http.post(
        "/world/load", {"path": str(world_path), "wait_s": LOAD_TIMEOUT_S},
        timeout=LOAD_TIMEOUT_S + 30)
    # if the caller window expired mid-load, poll /sim/state
    if status == 200 and isinstance(parsed, dict) and parsed.get("load_state") == "in_progress":
        while time.perf_counter() - t0 < LOAD_TIMEOUT_S:
            s2, p2, _, _, _ = http.get("/sim/state", timeout=10)
            if s2 == 200 and isinstance(p2, dict) and p2.get("supervisor_connected"):
                parsed = p2
                break
            time.sleep(1)
    lat = (time.perf_counter() - t0) * 1000.0
    ok = status == 200 and isinstance(parsed, dict) and bool(
        parsed.get("ok", parsed.get("supervisor_connected")))
    return ok, lat, {"status": status,
                     "load_state": (parsed or {}).get("load_state")}


def probe_hot_reload(http, world_path):
    """Edit the (temp-copy) world, re-POST /world/load, require < threshold."""
    txt = Path(world_path).read_text(encoding="utf-8")
    assert "translation 1 0 1" in txt, "BALL translation marker missing"
    Path(world_path).write_text(
        txt.replace("translation 1 0 1", "translation 1.1 0 1.2"),
        encoding="utf-8")
    t0 = time.perf_counter()
    status, parsed, _, _, _ = http.post(
        "/world/load", {"path": str(world_path), "wait_s": LOAD_TIMEOUT_S},
        timeout=LOAD_TIMEOUT_S + 30)
    lat = (time.perf_counter() - t0) * 1000.0
    ok = (status == 200 and isinstance(parsed, dict)
          and bool(parsed.get("ok"))
          and lat / 1000.0 < HOT_RELOAD_THRESHOLD_S)
    # verify the edit actually took: BALL now starts at x=1.1
    pos, _ = _tree_pos(http, "BALL")
    edit_took = pos is not None and abs(pos[0] - 1.1) < 0.2
    return ok and edit_took, lat, {
        "threshold_s": HOT_RELOAD_THRESHOLD_S, "edit_visible": edit_took}


def probe_scene_tree(http):
    status, parsed, _, _, lat = http.get("/scene/tree")
    ok = False
    n = 0
    if status == 200 and isinstance(parsed, dict):
        nodes = parsed.get("nodes", parsed.get("tree", [])) or []
        n = len(nodes)
        with_pose = [nd for nd in nodes
                     if isinstance(nd.get("position"), list) and len(nd["position"]) == 3]
        ok = n > 0 and len(with_pose) > 0
    return ok, lat, {"nodes": n}


def probe_scene_tree_bounds(http):
    status, parsed, _, _, lat = http.get("/scene/tree?bounds=1")
    ok = False
    n_bounds = 0
    if status == 200 and isinstance(parsed, dict):
        nodes = parsed.get("nodes", parsed.get("tree", [])) or []
        for nd in nodes:
            b = nd.get("bounds")
            if isinstance(b, dict) and "bbox_min" in b and "bbox_max" in b:
                n_bounds += 1
        ok = n_bounds > 0
    return ok, lat, {"nodes_with_bounds": n_bounds}


def probe_step_deterministic(http, world_path):
    """Two trials of (reload world -> POST /sim/step 50 -> read BALL) must land
    the falling ball at the SAME rest position (<1e-9), and that rest position
    must differ from the authored start pose (i.e. stepping really advanced the
    dynamics).

    Design note (measured findings on OmniSim, recorded as deviations):
      * /sim/reset rewinds sim TIME but does NOT restore node state, and the
        injected supervisor free-runs the sim between RPCs — so a reset-based
        "step N, compare trajectories" probe is impossible over this surface.
        The state-reset primitive that works is a world (re)load, and the
        settled rest state absorbs the free-run timing noise. Lane 3a measures
        strict trajectory determinism separately (bitwise, controller-side).
    """
    t0 = time.perf_counter()

    def one_pass():
        s, p, _, _, _ = http.post(
            "/world/load", {"path": str(world_path), "wait_s": LOAD_TIMEOUT_S},
            timeout=LOAD_TIMEOUT_S + 30)
        if s != 200 or not isinstance(p, dict) or not p.get("ok"):
            return None
        s, p, _, _, _ = http.post("/sim/step", {"steps": 50}, timeout=300)
        if s != 200:
            return None
        pos, _ = _tree_pos(http, "BALL")
        return pos

    a = one_pass()
    b = one_pass()
    lat = (time.perf_counter() - t0) * 1000.0
    if a is None or b is None:
        return False, lat, {"error": "reload/step/tree read failed"}
    # authored start pose of BALL in the (hot-reload-edited) temp world
    start_z = 1.2
    moved = abs(a[2] - start_z) > 1e-3
    max_dev = max(abs(a[i] - b[i]) for i in range(3))
    ok = moved and max_dev < STEP_DET_TOL
    return ok, lat, {"moved": moved, "max_dev_m": max_dev, "tol": STEP_DET_TOL,
                     "_deviations": [
                         "/sim/reset used to rewind time WITHOUT restoring node "
                         "state, so this probe was written around a world reload. "
                         "That is fixed: reset now restores the engine's "
                         "parse-time state (verified both backends). The reload "
                         "path is kept here so the probe keeps measuring the same "
                         "thing across the fix boundary -- switching it to reset "
                         "would silently change what the score means.",
                         "sim free-runs between harness RPCs; rest-state comparison "
                         "absorbs the timing noise (strict trajectory determinism is "
                         "lane 3a's job)"]}


def probe_events_stream(http):
    status, parsed, _, _, lat = http.get("/sim/events?since=0&log_since=0")
    ok = (status == 200 and isinstance(parsed, dict)
          and "next_since" in parsed and "next_log_since" in parsed
          and isinstance(parsed.get("events"), list))
    n = len(parsed.get("events", [])) if isinstance(parsed, dict) else 0
    # cursor advances: poll again from next_since; must not error
    if ok:
        s2, p2, _, _, _ = http.get("/sim/events?since=%s&log_since=%s"
                                   % (parsed["next_since"], parsed["next_log_since"]))
        ok = s2 == 200 and isinstance(p2, dict)
    return ok, lat, {"events_first_poll": n}


def probe_robot_joints(http):
    status, parsed, _, _, lat = http.get("/robot/BOT/joints")
    ok = False
    n = 0
    if status == 200 and isinstance(parsed, dict):
        joints = parsed.get("joints") or []
        n = len(joints)
        ok = n >= 1 and all("position" in j for j in joints)
    return ok, lat, {"joints": n}


def probe_scene_frame(http):
    status, parsed, _, _, lat = http.post("/scene/frame", {"def": "TARGET"})
    ok = False
    fits = None
    headroom = None
    if status == 200 and isinstance(parsed, dict):
        ver = parsed.get("verification")
        if isinstance(ver, dict):
            fits = ver.get("fits")
            headroom = ver.get("headroom_h_deg")
            # numeric proof required, not just a boolean claim
            numeric = all(isinstance(ver.get(k), (int, float))
                          for k in ("headroom_h_deg", "headroom_v_deg",
                                    "subject_angular_radius_deg"))
            ok = fits is True and numeric
    return ok, lat, {"fits": fits, "headroom_h_deg": headroom}


def probe_screenshot(http):
    status, parsed, raw, ctype, lat = http.post("/world/screenshot", {})
    ok = status == 200 and raw[:8] == b"\x89PNG\r\n\x1a\n"
    size = None
    if ok:
        try:
            from PIL import Image
            img = Image.open(io.BytesIO(raw))
            img.verify()
            size = list(Image.open(io.BytesIO(raw)).size)
        except Exception:
            ok = False
    return ok, lat, {"png_bytes": len(raw), "size": size}


def probe_load_broken(http, tmp):
    """A syntactically broken .wbt must yield a STRUCTURED diagnostic code,
    not free text. Run LAST — it leaves the harness in a failed-load state."""
    broken = os.path.join(tmp, "lane3_broken.wbt")
    with open(broken, "w", encoding="utf-8") as f:
        f.write(BROKEN_WBT)
    # A failed load can take several minutes to resolve on this surface: the
    # harness retries the load on a cold engine relaunch, then waits out its
    # supervisor-bind stall detector before reporting. Budget the full window
    # (300 s harness hard ceiling + margin) — the measured latency IS the
    # result being benchmarked.
    status, parsed, _, _, lat = http.post(
        "/world/load", {"path": broken, "wait_s": 120}, timeout=420)
    codes = []

    def collect(p):
        if isinstance(p, dict):
            for d in p.get("diagnostics") or []:
                if isinstance(d, dict) and d.get("code"):
                    codes.append(d["code"])

    collect(parsed)
    if not codes:
        # diagnostics may only be available from the dedicated endpoint
        s2, p2, _, _, _ = http.get("/world/diagnostics", timeout=60)
        if s2 == 200:
            collect(p2)
    # Pass = the failure is REPORTED STRUCTURED (a machine-branchable code, not
    # free text) and the harness does not report the broken world as a healthy
    # load (supervisor connected).
    structured = any(c and c != "UNKNOWN" for c in codes)
    healthy_claim = False
    s3, p3, _, _, _ = http.get("/sim/state", timeout=60)
    if s3 == 200 and isinstance(p3, dict):
        healthy_claim = bool(p3.get("supervisor_connected")) and \
            p3.get("load_state") == "complete"
    ok = structured and not healthy_claim
    return ok, lat, {"codes": sorted(set(codes))[:6],
                     "resp_ok": (parsed or {}).get("ok") if isinstance(parsed, dict) else None,
                     "resp_load_state": (parsed or {}).get("load_state") if isinstance(parsed, dict) else None,
                     "falsely_healthy": healthy_claim}


# ---------------------------------------------------------------------------
def engine_attribution(engine_log):
    """(engine label, reason-or-None) from the engine's own backend verdict.

    The label a row may carry is decided ONLY by `<engine-log>.newton.json`, the
    race-free sidecar the engine writes at world-finalize. THE FALLBACK IS NOT
    `"omnisim-ode"` -- see `common.engine_launch.engine_attribution`, which now
    owns the rule.

    This wrapper survives because lane 3 has a log PATH, not a verdict, and its
    call site reads the sidecar at a specific moment (before the broken-world
    probe truncates it). The RULE moved to `common/` when lane 1 was found still
    publishing `omnisim-newton` unconditionally: one lane's fix is not the
    suite's, and two copies of an attribution rule is how they diverge.
    """
    return _launch.engine_attribution(_launch.newton_verdict(engine_log))


def main():
    ap = argparse.ArgumentParser(description="OmniBench lane 3c agent-driveability")
    ap.add_argument("--port", type=int, default=6889)
    ap.add_argument("--out", default=str(_row.default_out("driveability.jsonl")))
    args = ap.parse_args()

    try:
        from PIL import Image  # noqa: F401
    except ImportError:
        print("[l3-drv] Pillow required (PNG verification + harness render_stats): pip install Pillow")
        return 2
    if not _row.omnisim_binary().exists():
        print("[l3-drv] omnisim-bin not found — build first.")
        return 2

    port, sup_port = pick_ports(args.port)
    tmp = tempfile.mkdtemp(prefix="omnibench_l3_drv_")
    world = os.path.join(tmp, "lane3_drive.wbt")
    shutil.copyfile(WORLD_SRC, world)

    http = Http("http://127.0.0.1:%d" % port)
    print("[l3-drv] starting harness on :%d (supervisor :%d), tmp=%s" % (port, sup_port, tmp))
    proc, engine_log = start_harness(port, sup_port, tmp)
    rows = []
    try:
        if not wait_healthy(http, proc, 60):
            print("[l3-drv] FATAL: harness did not become healthy (see %s)" % tmp)
            return 1

        plan = [
            ("load_valid_world", lambda: probe_load_valid(http, world)),
            ("hot_reload_edited_world", lambda: probe_hot_reload(http, world)),
            ("scene_tree_poses", lambda: probe_scene_tree(http)),
            ("scene_tree_bounds", lambda: probe_scene_tree_bounds(http)),
            ("sim_step_deterministic", lambda: probe_step_deterministic(http, world)),
            ("events_cursor_stream", lambda: probe_events_stream(http)),
            ("robot_joints_state", lambda: probe_robot_joints(http)),
            ("scene_frame_verified", lambda: probe_scene_frame(http)),
            ("screenshot_png", lambda: probe_screenshot(http)),
        ]

        results = []

        def run_probe(name, fn):
            try:
                passed, lat, extra = fn()
            except Exception as e:  # a crashing probe is a fail, not a crash
                passed, lat, extra = False, 0.0, {"exception": repr(e)}
            results.append((name, passed, lat, extra))
            print("[l3-drv]   %-36s %s  %8.1f ms  %s"
                  % (name, "PASS" if passed else "FAIL", lat, extra))

        for name, fn in plan:
            run_probe(name, fn)

        # Engine attribution: the .newton.json sidecar next to the harness's
        # engine log. Read it NOW, while the last load was a VALID world that
        # reached finalize — the broken-world probe below truncates the log
        # and deletes the sidecar (its load never finalizes), which would
        # mis-attribute the whole suite.
        engine, engine_why = engine_attribution(engine_log)

        run_probe("broken_world_structured_diagnostic",
                  lambda: probe_load_broken(http, tmp))

        # Carried into EVERY row so an unverified backend travels with the
        # measurement instead of only being visible in this process's stdout.
        attribution_devs = [engine_why] if engine_why else []
        if engine_why:
            print("[l3-drv] " + engine_why)

        machine = _row.fingerprint()
        for name, passed, lat, extra in results:
            extra = dict(extra)
            probe_devs = extra.pop("_deviations", [])
            rows.append(_row.make_row(
                test="driveability_%s" % name, engine=engine, dt_ms=16.0,
                metrics={"pass": bool(passed), "latency_ms": round(lat, 1), **extra},
                wall_ms_per_step=0.0, steps=0, sim_seconds=0.0,
                deviations=["latency includes all HTTP round-trips the probe makes"]
                + attribution_devs + list(probe_devs),
                machine=machine))
        n_pass = sum(1 for _, p, _, _ in results if p)
        score = n_pass / len(results)
        rows.append(_row.make_row(
            test="driveability_summary", engine=engine, dt_ms=16.0,
            metrics={"probes_passed": n_pass, "probes_total": len(results),
                     "score": round(score, 3)},
            wall_ms_per_step=0.0, steps=0, sim_seconds=0.0,
            deviations=["probe list: see tests/benchmarks/omnibench/lane3/DRIVEABILITY.md"]
            + attribution_devs,
            machine=machine))
        for r in rows:
            _row.write_row(args.out, r)
        print("[l3-drv] SCORE: %d/%d = %.2f  (rows -> %s)"
              % (n_pass, len(results), score, args.out))
        return 0 if n_pass == len(results) else 1
    finally:
        print("[l3-drv] stopping harness ...")
        stop_harness(proc, tmp)


if __name__ == "__main__":
    sys.exit(main())
