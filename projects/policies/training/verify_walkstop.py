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

"""Automated PASS/FAIL verdict for a walk-stop-walk live deploy log.

Owner rule (2026-07-04): NOTHING goes on the owner's screen until this says PASS.
Parses the `deploy t= x= y= z= ... cmd= vx= glat=` telemetry emitted by the
g1_walk_recipe deploy hook and checks, over the whole run:

  walking_ok : some window with cmd>0.4, forward rate>0.2 m/s, base upright (z>0.6)
  stopped_ok : some window with cmd<0.05, |forward rate|<0.05 m/s, base upright
  fell       : any sample with z<0.4
  PASS       = walking_ok and stopped_ok and not fell

Usage: python verify_walkstop.py <mpc_log.txt> [--quiet]
Exit code 0 on PASS, 1 on FAIL (so shell chains can gate on it).
"""
import re
import sys


def verdict(path, quiet=False):
    rows = []
    pat = re.compile(r"deploy t=(\d+) x=([-\d.]+) y=([-\d.]+) z=([-\d.]+) "
                     r"roll=([-\d.]+) pit=([-\d.]+) cmd=([-\d.]+) vx=([-\d.]+) glat=([-\d.]+)")
    for ln in open(path, encoding="utf-8", errors="ignore"):
        m = pat.search(ln)
        if m:
            rows.append([float(g) for g in m.groups()])
    if len(rows) < 10:
        print("VERDICT: no telemetry (%d rows) -- FAIL" % len(rows))
        return False
    walking_ok = stopped_ok = fell = False
    if not quiet:
        print("t      x     z    cmd   vx  glat")
    for i in range(3, len(rows)):
        t, x, y, z, roll, pit, cmd, vx, glat = rows[i]
        t0, x0 = rows[i - 3][0], rows[i - 3][1]
        rate = (x - x0) / max(1e-6, (t - t0) * 0.016)
        if z < 0.4:
            fell = True
        if cmd > 0.4 and rate > 0.2 and z > 0.6:
            walking_ok = True
        if cmd < 0.05 and abs(rate) < 0.05 and z > 0.6:
            stopped_ok = True
        if not quiet and i % 15 == 0:
            print("%5d %5.2f %.3f %5.2f %5.2f %5.2f" % (t, x, z, cmd, vx, glat))
    print("VERDICT: walking_ok=%s stopped_ok=%s fell=%s" % (walking_ok, stopped_ok, fell))
    ok = walking_ok and stopped_ok and not fell
    print("PASS -- walk AND clean stop observed, no fall" if ok else "FAIL -- do not show")
    return ok


if __name__ == "__main__":
    quiet = "--quiet" in sys.argv
    sys.exit(0 if verdict(sys.argv[1], quiet) else 1)
