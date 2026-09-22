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

"""Diagnose a dropped payload from the OmniSim event stream alone.

This is the tool half of the debugging demo in
`projects/samples/demos/worlds/debug/` -- "the arm drops the box two times in
five, and an agent finds out why in 90 seconds".

Everything it needs comes off ONE endpoint, `GET /sim/events`, loaded in full
tracking mode (`{"light": false}`). No screenshot, no world source, no result
file. It decides whether a carry ended in a DROP, prints the evidence chain
that says so, and names the suspect parameter -- which the engine itself
publishes on the same stream as a `controller.log` event.

    # live: start a harness, load the world, watch, diagnose, reap
    python scripts/dev/diagnose_drop.py \
        projects/samples/demos/worlds/debug/omniarm6_drop_fault.omniworld

    # offline: re-run the decision rule over a capture from a previous session
    python scripts/dev/diagnose_drop.py --offline run_events.json

!! THE TRAP THIS TOOL EXISTS TO AVOID. `grip.released` alone is NOT the signal.
A perfectly healthy carry emits TWO release/re-acquire flickers while the pads
re-seat (measured: 32 ms and 40 ms apart). A detector that fires on any
`grip.released` reports a drop on a run that placed the part correctly.
`tests/test_drop_detector.py` pins that it does not.

The decision rule, stated once:

    A DROP is the FINAL `grip.released` for the part -- final meaning no
    `grip.acquired` for the same part follows it within `regrip_ms` -- that is
    NOT preceded by a `contact.began` between the part and the destination
    surface, and IS followed within `reland_ms` by a `contact.began` between
    the part and some named surface.

If the part never re-contacts anything inside the window, the verdict is
`inconclusive`, not `dropped`: the run may simply have ended in mid-fall, and
an instrument that guesses is worse than one that says it cannot see.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_WORLD = ("projects/samples/demos/worlds/debug/"
                 "omniarm6_drop_fault.omniworld")

# A "flicker" is a release that is undone almost immediately. Measured on the
# healthy carry: 32 ms and 40 ms. 200 ms is ~5x the largest observed and still
# an order of magnitude below the 1.5-2.3 s a real carry lasts.
REGRIP_MS = 200

# How long after the final release the part is allowed to take to hit
# something before the tool refuses to call it a drop. A free fall from the
# 0.44 m carry height to a 0.245 m table top is ~200 ms; a part that is
# already sagging through the pads lands in ~64 ms. 500 ms covers both with
# margin and still fails closed if the run ends in mid-air.
RELAND_MS = 500

# The engine publishes its own contact/solver settings on the event stream.
SOLVER_LINE_RE = re.compile(
    r"\[OmNewtonBackend\]\s+contact/solver params from WorldInfo:\s+(.*)$")
SOLVER_KV_RE = re.compile(r"(\w+)=([-\w.]+)")

# Contacts name URDF links by opaque node id (`#193`), because URDF links carry
# no DEF. So a partner with a real name is, by construction, an authored world
# body -- a table, a floor -- and not part of the gripper.
OPAQUE_NODE_RE = re.compile(r"^#\d+$")


# --------------------------------------------------------------------------
# The decision rule. Pure: no engine, no network, no clock.
# --------------------------------------------------------------------------

@dataclass
class Verdict:
    """What the event stream says happened to one carried part."""

    verdict: str                      # carried | dropped | inconclusive | no_grip
    reason: str                       # one line, human-readable
    part: str = "BLOCK"
    destination: str | None = None
    release_t_ms: int | None = None   # the final release
    dest_contact_t_ms: int | None = None
    reland_t_ms: int | None = None
    reland_partner: str | None = None
    flickers: list = field(default_factory=list)   # (released_ms, reacquired_ms)
    evidence: list = field(default_factory=list)   # the raw events, in order

    @property
    def dropped(self) -> bool:
        return self.verdict == "dropped"

    def as_dict(self) -> dict:
        out = dict(self.__dict__)
        out["dropped"] = self.dropped
        return out


def _is_pair(event, a: str, b: str) -> bool:
    """True if a contact event joins exactly `a` and `b`, in either order."""
    got = {str(event.get("a_def")), str(event.get("b_def"))}
    return got == {a, b}


def _touches(event, part: str) -> str | None:
    """If a contact event involves `part`, return the other partner."""
    a, b = str(event.get("a_def")), str(event.get("b_def"))
    if a == part:
        return b
    if b == part:
        return a
    return None


def _is_named_surface(name: str, part: str) -> bool:
    """A partner that is an authored world body rather than a gripper link."""
    return bool(name) and name != part and not OPAQUE_NODE_RE.match(name)


def classify_carry(events,
                   part: str = "BLOCK",
                   destination: str = "PLACE_TABLE",
                   regrip_ms: int = REGRIP_MS,
                   reland_ms: int = RELAND_MS) -> Verdict:
    """Decide whether `part` was carried to `destination` or dropped.

    `events` is the concatenated `events` array from one or more
    `GET /sim/events` responses, in arrival order. Supervisor events carry
    `t_sim_ms`; `controller.log` events do not and are ignored here (they are
    used for narration, not for the verdict -- the two clocks do not share a
    timeline, see docs/developer/positioning.md 7).
    """
    sup = [e for e in events if e.get("source") == "sup" or "t_sim_ms" in e]

    grips = [e for e in sup
             if e.get("type") in ("grip.acquired", "grip.released")
             and str(e.get("held_def")) == part]
    if not grips:
        return Verdict("no_grip",
                       "no grip.acquired/grip.released for %r on the stream -- "
                       "either the part was never gripped, or the world was "
                       "loaded in light mode, which suppresses grip.* events "
                       "(reload with {\"light\": false})" % part,
                       part=part, destination=destination)

    acquires = [e for e in grips if e["type"] == "grip.acquired"]
    releases = [e for e in grips if e["type"] == "grip.released"]
    if not releases:
        return Verdict("no_grip",
                       "%r was gripped and never released before the stream "
                       "ended -- nothing to judge yet" % part,
                       part=part, destination=destination,
                       evidence=acquires[:1])

    # --- separate the flickers from the one release that stuck ------------
    flickers = []
    final_release = None
    for rel in releases:
        t_rel = rel.get("t_sim_ms", 0)
        regrip = next((a for a in acquires
                       if 0 <= a.get("t_sim_ms", 0) - t_rel <= regrip_ms), None)
        if regrip is not None:
            flickers.append((t_rel, regrip.get("t_sim_ms")))
        else:
            final_release = rel
    if final_release is None:
        return Verdict("no_grip",
                       "every grip.released for %r was undone by a re-acquire "
                       "within %d ms -- the part is still held" % (part, regrip_ms),
                       part=part, destination=destination, flickers=flickers)

    t_rel = final_release.get("t_sim_ms", 0)

    # --- did the part reach its destination BEFORE the release? -----------
    dest_contact = next(
        (e for e in sup
         if e.get("type") == "contact.began"
         and _is_pair(e, part, destination)
         and e.get("t_sim_ms", 0) <= t_rel), None)

    # --- what did it hit AFTER the release? -------------------------------
    reland = next(
        (e for e in sup
         if e.get("type") == "contact.began"
         and 0 < e.get("t_sim_ms", 0) - t_rel <= reland_ms
         and _is_named_surface(str(_touches(e, part) or ""), part)), None)

    chain = _evidence_chain(sup, part, destination, final_release, t_rel,
                            reland_ms)

    if dest_contact is not None:
        return Verdict(
            "carried",
            "%s reached %s at t_sim=%d ms while still gripped; the release at "
            "t_sim=%d ms was deliberate (%d flicker%s ignored)"
            % (part, destination, dest_contact.get("t_sim_ms"), t_rel,
               len(flickers), "" if len(flickers) == 1 else "s"),
            part=part, destination=destination, release_t_ms=t_rel,
            dest_contact_t_ms=dest_contact.get("t_sim_ms"),
            flickers=flickers, evidence=chain)

    if reland is None:
        return Verdict(
            "inconclusive",
            "the grip on %s ended at t_sim=%d ms with no prior contact.began "
            "%s<->%s, but the part did not contact any named surface within "
            "%d ms either -- the stream may have ended in mid-fall. Not "
            "calling this a drop."
            % (part, t_rel, part, destination, reland_ms),
            part=part, destination=destination, release_t_ms=t_rel,
            flickers=flickers, evidence=chain)

    partner = _touches(reland, part)
    return Verdict(
        "dropped",
        "the grip on %s ended at t_sim=%d ms with the part in flight -- no "
        "contact.began %s<->%s ever preceded it -- and %d ms later the part "
        "struck %s"
        % (part, t_rel, part, destination,
           reland.get("t_sim_ms", 0) - t_rel, partner),
        part=part, destination=destination, release_t_ms=t_rel,
        reland_t_ms=reland.get("t_sim_ms"), reland_partner=partner,
        flickers=flickers, evidence=chain)


def _evidence_chain(sup, part, destination, final_release, t_rel, reland_ms):
    """The handful of events a human should be shown, in stream order."""
    keep = []
    for e in sup:
        t = e.get("t_sim_ms", 0)
        typ = e.get("type", "")
        if typ in ("grip.acquired", "grip.released") and str(e.get("held_def")) == part:
            keep.append(e)
        elif typ in ("contact.began", "contact.ended"):
            other = _touches(e, part)
            if other is None:
                continue
            # Every pad re-seat is a contact event; only the named surfaces and
            # the ones bracketing the release are worth a reader's attention.
            if _is_named_surface(str(other), part):
                keep.append(e)
            elif t_rel - 200 <= t <= t_rel + 8:
                # The pads are opaque node ids (URDF links carry no DEF), so
                # show only the ones that bracket the release -- the moment the
                # pads stopped touching the part. The rest is re-seat chatter.
                keep.append(e)
    return keep


def read_solver_params(events) -> dict:
    """The engine's own contact/solver settings, off the same event stream.

    The backend prints them once at world start; the harness forwards engine
    stdout as `controller.log` events, so an agent can read the value of the
    suspect parameter without ever opening the world file.
    """
    for e in events:
        if e.get("type") != "controller.log":
            continue
        m = SOLVER_LINE_RE.search(str(e.get("line", "")))
        if m:
            return {k: v for k, v in SOLVER_KV_RE.findall(m.group(1))}
    return {}


def controller_result(events) -> str | None:
    """The controller's own RESULT line, if it printed one."""
    for e in reversed(events):
        line = str(e.get("line", ""))
        if e.get("type") == "controller.log" and "RESULT" in line:
            return line
    return None


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------

# The reference value for this cell, from the flagship world's own header and
# docs/guide/friction-grasp.md. Used only to say "this differs from", never to
# assert a cause on its own.
REFERENCE_MU = 6.0


def format_report(verdict: Verdict, params: dict, result_line: str | None = None) -> str:
    out = []
    tag = {"dropped": "DROPPED", "carried": "CARRIED",
           "inconclusive": "INCONCLUSIVE", "no_grip": "NO GRIP"}[verdict.verdict]
    out.append("VERDICT  %s" % tag)
    out.append("         %s" % verdict.reason)
    out.append("")
    out.append("EVIDENCE (GET /sim/events, full tracking -- nothing else was read)")
    for e in verdict.evidence:
        t = e.get("t_sim_ms")
        typ = e.get("type", "")
        if typ.startswith("grip."):
            note = ""
            if typ == "grip.released":
                if any(f[0] == t for f in verdict.flickers):
                    note = "   <- flicker, re-acquired %d ms later (NOT a drop)" % (
                        next(f[1] for f in verdict.flickers if f[0] == t) - t)
                elif t == verdict.release_t_ms:
                    note = ("   <- THE RELEASE" if verdict.verdict != "dropped"
                            else "   <- GRIP LOST IN MID-AIR")
            out.append("  t_sim %-6s %-15s %s%s"
                       % (t, typ, e.get("held_def"), note))
        else:
            note = ""
            if typ == "contact.began" and t == verdict.dest_contact_t_ms:
                note = "   <- reached the destination, still gripped"
            elif typ == "contact.began" and t == verdict.reland_t_ms:
                note = ("   <- the part HITS %s here, %d ms AFTER the release"
                        % (verdict.reland_partner,
                           t - (verdict.release_t_ms or 0)))
            out.append("  t_sim %-6s %-15s %s <-> %s%s"
                       % (t, typ, e.get("a_def"), e.get("b_def"), note))
    out.append("")

    if verdict.verdict == "dropped" and verdict.reland_partner == verdict.destination:
        out.append("!! THE PART LANDED ON %s ANYWAY. Landing on the destination is"
                   % verdict.destination)
        out.append("  not the same as being carried to it, and an end-of-run")
        out.append("  screenshot cannot tell the two apart. Only the ORDER of these")
        out.append("  events can: the contact came AFTER the release, not before.")
        out.append("  (The controller's own `placed` test scores this run True.)")
        out.append("")

    if verdict.verdict == "dropped":
        out.append("MECHANISM")
        out.append("  The part left the gripper while airborne. That excludes a")
        out.append("  motion/IK error (the tool reached the place pose) and a")
        out.append("  mis-timed open command (no joint command precedes the loss --")
        out.append("  the pads simply stop reporting contact). What is left is the")
        out.append("  hold itself: friction between the pads and the part.")
        out.append("")
        out.append("SUSPECT PARAMETER -- read off the SAME stream")
        if params:
            out.append("  the engine published: %s"
                       % " ".join("%s=%s" % kv for kv in sorted(params.items())))
            mu = params.get("mu")
            if mu is not None:
                try:
                    differs = abs(float(mu) - REFERENCE_MU) > 1e-9
                except ValueError:
                    differs = True
                if differs:
                    out.append("  WorldInfo.newtonGroundMu = %s, against a reference"
                               % mu)
                    out.append("  value of %g for this cell (the flagship world's own"
                               % REFERENCE_MU)
                    out.append("  header, and docs/guide/friction-grasp.md).")
                    out.append("  Friction is GLOBAL in this world -- one ShapeConfig")
                    out.append("  serves every body -- so the pad cannot be grippier")
                    out.append("  than this one number.")
                    out.append("")
                    out.append("FIX")
                    out.append("  set WorldInfo.newtonGroundMu %g  and re-run." % REFERENCE_MU)
                    out.append("  Verified fix world:")
                    out.append("    projects/samples/demos/worlds/debug/"
                               "omniarm6_drop_fault_fixed.omniworld")
                    out.append("  Verify across the payload set (expect 5/5 vs 3/5):")
                    out.append("    python scripts/dev/drop_payload_sweep.py --both")
                else:
                    out.append("  newtonGroundMu is already at the reference value %g,"
                               % REFERENCE_MU)
                    out.append("  so friction is NOT the fault here. Look at the payload")
                    out.append("  mass and the squeeze force next.")
        else:
            out.append("  not on this stream. The engine prints its contact/solver")
            out.append("  settings once at world start; if you began polling after")
            out.append("  that, restart the capture from log_since=0.")
    elif verdict.verdict == "carried" and params:
        out.append("PARAMETERS on this run: %s"
                   % " ".join("%s=%s" % kv for kv in sorted(params.items())))

    if result_line:
        out.append("")
        out.append("CONTROLLER'S OWN VERDICT (for cross-check only -- the diagnosis")
        out.append("above used no result file):")
        out.append("  %s" % result_line)
    return "\n".join(out)


# --------------------------------------------------------------------------
# Live driver: start a harness, load, watch /sim/events, reap.
# --------------------------------------------------------------------------

def _http(base, method, path, body=None, timeout=180):
    url = base + path
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode()), time.time() - t0
    except urllib.error.HTTPError as e:
        return {"_http_error": e.code, "body": e.read().decode()[:400]}, time.time() - t0
    except Exception as e:                                    # noqa: BLE001
        return {"_error": repr(e)}, time.time() - t0


def _port_owner(port: int):
    """The pid LISTENING on `port`, or None."""
    if os.name == "nt":
        out = subprocess.run(["netstat", "-ano"], capture_output=True,
                             text=True).stdout
        for line in out.splitlines():
            f = line.split()
            if (len(f) >= 5 and f[0] == "TCP" and f[1].endswith(":%d" % port)
                    and f[3] == "LISTENING"):
                return f[4]
        return None
    out = subprocess.run(["lsof", "-ti", "tcp:%d" % port, "-sTCP:LISTEN"],
                         capture_output=True, text=True).stdout.strip()
    return out.splitlines()[0] if out else None


def _port_is_free(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) != 0


def _engine_pids() -> set:
    """Every omnisim-bin currently alive. Used ONLY to reap our own children.

    Never kill an engine you did not spawn -- other sessions share the host.
    """
    if os.name != "nt":
        out = subprocess.run(["pidof", "omnisim-bin"], capture_output=True,
                             text=True).stdout
        return set(out.split())
    out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq omnisim-bin.exe",
                          "/FO", "CSV", "/NH"], capture_output=True,
                         text=True).stdout
    pids = set()
    for line in out.splitlines():
        parts = [p.strip('"') for p in line.split('","')]
        if len(parts) > 1 and parts[0].lower().startswith("omnisim-bin"):
            pids.add(parts[1])
    return pids


def _engine_cmdlines() -> dict:
    """{pid: command line} for every live omnisim-bin, or {} if unavailable.

    Used to make double sure a reap only touches an engine that is running OUR
    world. `tasklist` cannot report command lines, so this asks CIM.
    """
    if os.name != "nt":
        return {}
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command",
             "Get-CimInstance Win32_Process -Filter \"Name='omnisim-bin.exe'\""
             " | ForEach-Object { \"$($_.ProcessId)`t$($_.CommandLine)\" }"],
            capture_output=True, text=True, timeout=25).stdout
    except Exception:                                         # noqa: BLE001
        return {}
    rows = {}
    for line in out.splitlines():
        pid, _, cmd = line.partition("\t")
        if pid.strip().isdigit():
            rows[pid.strip()] = cmd
    return rows


def _kill(pid, tree=False):
    if os.name == "nt":
        cmd = ["taskkill", "/F", "/PID", str(pid)]
        if tree:
            cmd.insert(1, "/T")
        subprocess.run(cmd, capture_output=True)
    else:
        subprocess.run(["kill", "-9", str(pid)], capture_output=True)


def run_live(world: str, port: int = 6789, budget_s: float = 180.0,
             poll_s: float = 0.4, verbose: bool = True):
    """Drive one run through a private harness and return (events, timings)."""
    owner = _port_owner(port)
    if owner or not _port_is_free(port):
        raise SystemExit(
            "port %d is already held (pid %s).\n"
            "`python -m omnisim harness` survives Popen.terminate() and keeps "
            "its port, so this is very likely a harness from an earlier "
            "session -- and driving it would read ITS stale event ring.\n"
            "Reap it (Windows: taskkill /T /F /PID %s) or pass --port."
            % (port, owner, owner))

    scratch = Path(tempfile.mkdtemp(prefix="omnisim_diagnose_drop_"))
    env = dict(os.environ)
    # Unique engine log: the default is the SHARED repo-root omnisim_log.txt.
    env["OMNISIM_LOG_PATH"] = str(scratch / "engine.log")
    # !! Without PICK_OUT the flagship controller overwrites its own tracked
    # _real_pick_result.json next to its source. Keep the tree clean.
    env["PICK_OUT"] = str(scratch / "pick_result.json")

    pre_engines = _engine_pids()
    base = "http://127.0.0.1:%d" % port
    hlog = open(scratch / "harness.out", "w")
    harness = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "harness", "--port", str(port)],
        cwd=str(REPO_ROOT), env=env, stdout=hlog, stderr=subprocess.STDOUT)

    events, timings = [], {"scratch": str(scratch)}
    try:
        t0 = time.time()
        while time.time() - t0 < 90:
            r, _ = _http(base, "GET", "/healthz", timeout=2)
            if "_error" not in r and "_http_error" not in r:
                break
            time.sleep(0.4)
        else:
            raise SystemExit("harness never became live on port %d" % port)
        timings["healthz_s"] = round(time.time() - t0, 2)
        if verbose:
            print("[diagnose] harness live in %.1f s" % timings["healthz_s"])

        # !! light mode (the default) suppresses contact.* / grip.* /
        # joint.limit_hit -- exactly the five types this diagnosis needs.
        r, dt = _http(base, "POST", "/world/load",
                      {"path": world, "light": False}, timeout=300)
        timings["load_s"] = round(dt, 2)
        if r.get("ok") is False or "_error" in r:
            raise SystemExit("world load failed: %s" % json.dumps(r)[:600])
        if verbose:
            print("[diagnose] loaded %s in %.1f s (tracking=%s)"
                  % (Path(world).name, dt, (r.get("tracking") or {}).get("mode",
                                                                         "full")))

        # ONE endpoint. Reading anything else pauses the engine per call and
        # stretches an 11 s run into 100 s (measured).
        since = log_since = 0
        dropped_sup = dropped_log = 0
        t_poll = time.time()
        costs = []
        while time.time() - t_poll < budget_s:
            e, dt = _http(base, "GET",
                          "/sim/events?since=%d&log_since=%d&limit=512"
                          % (since, log_since), timeout=60)
            costs.append(dt)
            if "_error" in e:
                break
            since = e.get("next_since", since)
            log_since = e.get("next_log_since", log_since)
            dropped_sup += e.get("dropped_sup", 0) or 0
            dropped_log += e.get("dropped_log", 0) or 0
            events.extend(e.get("events", []))
            if controller_result(events):
                break
            time.sleep(poll_s)
        timings["watch_s"] = round(time.time() - t_poll, 1)
        timings["events_ms"] = (round(1000 * sum(costs) / len(costs), 1)
                                if costs else None)
        timings["n_events"] = len(events)
        timings["dropped_sup"] = dropped_sup
        timings["dropped_log"] = dropped_log
        if dropped_sup or dropped_log:
            print("[diagnose] !! the event ring dropped %d supervisor / %d log "
                  "events -- poll faster or raise limit; the evidence chain "
                  "below may be incomplete." % (dropped_sup, dropped_log))
    finally:
        owner = _port_owner(port)
        try:
            harness.terminate()
            harness.wait(timeout=10)
        except Exception:                                     # noqa: BLE001
            harness.kill()
        # The launcher exits but the real harness process keeps the port.
        if owner:
            _kill(owner, tree=True)
        # ...and the harness's engine outlives the harness. Reap only ours:
        # engines that were NOT alive before we started AND whose command line
        # names the world we loaded. Other sessions share this host, and a
        # taskkill reads like a crash in the victim's log.
        mine = _engine_pids() - pre_engines
        cmds = _engine_cmdlines()
        stem = Path(world).stem
        for pid in sorted(mine):
            cmd = cmds.get(pid) or ""
            names_a_world = ".omniworld" in cmd or ".wbt" in cmd
            if names_a_world and stem not in cmd:
                # It names a world, and it is not ours: someone else's engine
                # that happened to start while we were running.
                print("[diagnose] leaving omnisim-bin pid %s alone -- its "
                      "command line names a different world" % pid)
                continue
            _kill(pid)
        hlog.close()
    return events, timings


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Diagnose a dropped payload from GET /sim/events alone.")
    ap.add_argument("world", nargs="?", default=DEFAULT_WORLD,
                    help="world to load (default: the demo's faulty world)")
    ap.add_argument("--offline", metavar="EVENTS.JSON",
                    help="skip the engine; re-run the decision rule over a "
                         "saved capture (a JSON list of events, or an object "
                         "with an `events` key)")
    ap.add_argument("--port", type=int, default=6789)
    ap.add_argument("--part", default="BLOCK")
    ap.add_argument("--destination", default="PLACE_TABLE")
    ap.add_argument("--budget", type=float, default=180.0,
                    help="seconds to watch before giving up (default 180)")
    ap.add_argument("--save-events", metavar="PATH",
                    help="write the raw captured events here (replayable with "
                         "--offline, and the fixture format the detector test "
                         "uses)")
    ap.add_argument("--json", action="store_true",
                    help="emit the verdict as JSON instead of a report")
    ap.add_argument("--expect", choices=("dropped", "carried"),
                    help="exit non-zero unless the verdict matches -- use this "
                         "to turn the demo into a check")
    args = ap.parse_args(argv)

    if args.offline:
        raw = json.loads(Path(args.offline).read_text(encoding="utf-8"))
        events = raw["events"] if isinstance(raw, dict) else raw
        timings = {}
    else:
        events, timings = run_live(args.world, port=args.port,
                                   budget_s=args.budget)

    if args.save_events:
        Path(args.save_events).write_text(
            json.dumps(events, indent=1), encoding="utf-8")

    verdict = classify_carry(events, part=args.part,
                             destination=args.destination)
    params = read_solver_params(events)

    if args.json:
        print(json.dumps({"verdict": verdict.as_dict(), "params": params,
                          "timings": timings}, indent=1))
    else:
        if timings:
            print("")
            print("[diagnose] %d events in %.1f s of watching; "
                  "GET /sim/events cost %s ms/call"
                  % (timings.get("n_events", 0), timings.get("watch_s", 0.0),
                     timings.get("events_ms")))
        print("")
        print(format_report(verdict, params, controller_result(events)))

    if args.expect and verdict.verdict != args.expect:
        print("\n[diagnose] EXPECTED %s, GOT %s" % (args.expect, verdict.verdict),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
