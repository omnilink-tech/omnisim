"""Wait until the machine is safe to measure on, then run an arm.

The naive gate ("no listener on 6789-6792") never opens against an idle
harness whose engine has exited -- and that is exactly the leaked service the
cc_lane port sweep exists to reap. But an engine agent mid-debug will
hot-reload the same harness rather than restart it, and reaping THAT destroys
their run. So the gate distinguishes:

  BUSY   a compiler is running, or an omnisim-bin is simulating, or a harness
         reports running:true                      -> block, reset the clock
  LEAKED every harness listener reports running:false AND nothing has changed
         for GRACE_S                               -> safe to proceed

Anything else keeps waiting.
"""
import json
import os
import re
import subprocess
import sys
import time
import urllib.request

PORTS = (6789, 6790, 6791, 6792)
GRACE_S = 120.0          # quiet window before we call the machine safe
POLL_S = 30.0


def listeners():
    out = subprocess.run(["netstat", "-ano"], capture_output=True, text=True)
    hits = {}
    for line in out.stdout.splitlines():
        m = re.search(r":(\d+)\s+\S+\s+LISTENING\s+(\d+)", line)
        if m and int(m.group(1)) in PORTS:
            hits[int(m.group(1))] = int(m.group(2))
    return hits


def procs(name):
    out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq %s" % name,
                          "/FO", "CSV", "/NH"], capture_output=True, text=True)
    return sum(1 for line in out.stdout.splitlines()
               if line.lower().startswith('"%s' % name.lower()))


def builders():
    out = subprocess.run(["tasklist", "/FO", "CSV", "/NH"],
                         capture_output=True, text=True)
    n = 0
    for line in out.stdout.splitlines():
        low = line.lower()
        if low.startswith(('"make.exe', '"cc1plus.exe', '"g++.exe',
                           '"ld.exe', '"cc1.exe')):
            n += 1
    return n


def sim_state(port):
    try:
        with urllib.request.urlopen(
                "http://127.0.0.1:%d/sim/state" % port, timeout=5) as r:
            return json.loads(r.read().decode("utf-8", "replace"))
    except Exception:                                          # noqa: BLE001
        return None


def main():
    arm, sha, n = sys.argv[1], sys.argv[2], sys.argv[3]
    max_wait_s = float(sys.argv[4]) * 60 if len(sys.argv) > 4 else 5400.0
    t_start = time.time()
    leaked_since = None
    leaked_fp = None
    prev_builders = 0
    while True:
        b = builders()
        eng = procs("omnisim-bin.exe")
        ls = listeners()
        states = {p: sim_state(p) for p in ls}
        live_sim = any((s or {}).get("running") for s in states.values())
        # A transient headless omnisim-bin is NOT a hazard: those bind
        # [1234,1294], not 6789-6792, so our sweep cannot reach them. The
        # hazards are a build (CPU contention) and a harness listener
        # (which the port sweep would reap out from under its owner).
        # An incremental build here is BURSTY -- make.exe lives for a second
        # or two per compile -- so a single sample resets a strict clock for
        # ever. Only a build seen on two CONSECUTIVE polls counts as
        # sustained contention. A listener is always a hazard.
        sustained_build = bool(b and prev_builders)
        busy = bool(sustained_build or ls or live_sim)
        prev_builders = b
        fp = json.dumps({str(p): [(s or {}).get("load_started_at"),
                                  (s or {}).get("running")]
                         for p, s in sorted(states.items())}, sort_keys=True)
        if busy:
            leaked_since, leaked_fp = None, None
        elif not ls:
            leaked_since = leaked_since or time.time()
            leaked_fp = fp
        else:
            if fp != leaked_fp:
                leaked_since, leaked_fp = time.time(), fp
        held = 0.0 if leaked_since is None else time.time() - leaked_since
        print("[gate2] %s builders=%d omnisim-bin=%d listeners=%s live_sim=%s "
              "stable=%.0f/%.0fs waited=%.0fm"
              % (time.strftime("%FT%TZ", time.gmtime()), b, eng,
                 sorted(ls), live_sim, held, GRACE_S,
                 (time.time() - t_start) / 60.0), flush=True)
        if leaked_since is not None and held >= GRACE_S:
            print("[gate2] machine safe (%s); starting %s"
                  % ("idle" if not ls else
                     "only engine-less harness listeners %s" % sorted(ls), arm),
                  flush=True)
            break
        if time.time() - t_start > max_wait_s:
            print("[gate2] GAVE UP after %.0f min" % ((time.time() - t_start) / 60),
                  flush=True)
            return 3
        time.sleep(POLL_S)
    os.execvp("bash", ["bash", "/o/omnisim/_scratch/ab_beforeafter/run_arm.sh",
                       arm, sha, n])


if __name__ == "__main__":
    sys.exit(main())
