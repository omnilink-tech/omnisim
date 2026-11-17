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

"""Machine-vs-machine behavioral conformance for deploy policies.

The cross-machine guarantee for a chaotic plant (a balancing biped) is
STATISTICAL, never bitwise: two machines are conformant when the same skill,
run over the same seeds, produces outcomes inside agreed tolerance bands --
same no-fall record, same pace, same posture envelope. (Bitwise short-horizon
identity is golden_compare.py's job; the version/stack audit is
env_fingerprint's. This tool sits on top of both.)

One command per machine, then one compare anywhere:

    python projects/policies/training/machine_conformance.py run \
        --skill g1_walk --seeds 1,2,3 --duration 90 --out conf_thispc.json
    # ...same on the other machine -> conf_otherpc.json ...
    python projects/policies/training/machine_conformance.py compare \
        conf_thispc.json conf_otherpc.json

`run` drives the skill through the SKILL LIBRARY (the canonical launch env,
not a hand-rolled one), one headless run per seed via DEPLOY_IC_SEED, parses
the deploy telemetry, and attaches this machine's env fingerprint.

Verdict logic in `compare`:
  * a fall-record mismatch is an automatic DIFFERENT (outcome changed);
  * pace within --tol-pace (relative), pelvis height within --tol-z (m),
    roll/pitch std within --tol-att (rad) -> metric passes;
  * fingerprint stack mismatches are REPORTED but only [sys]-resolved
    packages make the verdict DIFFERENT (bundle packages are pinned by
    the repo; a differing bundle is a staging bug the fingerprint already
    flags on its own machine).
Exit codes: 0 EQUIVALENT, 2 DIFFERENT, 4 unusable input.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_TELEM = re.compile(
    r"walk-recipe deploy t=(\d+) x=(-?[\d.]+) y=(-?[\d.]+) z=([\d.]+) "
    r"roll=(-?[\d.]+) pit=(-?[\d.]+)")


def _analyze_log(text: str) -> dict | None:
    rows = [tuple(float(g) for g in m.groups()) for m in _TELEM.finditer(text)]
    if len(rows) < 10:
        return None
    t0, x0, y0 = rows[0][0], rows[0][1], rows[0][2]
    t1, x1, y1 = rows[-1][0], rows[-1][1], rows[-1][2]
    zs = [r[3] for r in rows]
    rolls = [r[4] for r in rows]
    pits = [r[5] for r in rows]

    def _std(v):
        mu = sum(v) / len(v)
        return math.sqrt(sum((a - mu) ** 2 for a in v) / len(v))

    sim_s = max(1e-9, (t1 - t0) * 0.016)
    dist = math.hypot(x1 - x0, y1 - y0)
    return {
        "ticks": int(t1),
        "dist_m": round(dist, 3),
        "pace_mps": round(dist / sim_s, 4),
        "z_mean": round(sum(zs) / len(zs), 4),
        "z_min": round(min(zs), 4),
        # z_min < 0.5 is a collapsed pelvis on every humanoid this recipe runs
        "fell": bool(min(zs) < 0.5),
        "roll_std": round(_std(rolls), 4),
        "pitch_std": round(_std(pits), 4),
    }


def cmd_run(args) -> int:
    seeds = [s.strip() for s in args.seeds.split(",") if s.strip()]
    rl_log = _REPO / "_scratch/foot_redesign" / f"{args.skill}_solo_rl.txt"
    runs = []
    for seed in seeds:
        # One retry per seed: cold-load launches have a known per-launch flake
        # (engine exits ~20 s in, before the recipe starts -- same class
        # verify_box_grasp.sh retries). A flaked launch produces no telemetry
        # at all, so retrying cannot mistake a bad RUN for a bad LAUNCH.
        metrics, rc, t0 = None, None, time.time()
        for attempt in (1, 2):
            if rl_log.exists():
                rl_log.unlink()
            env = dict(os.environ, DEPLOY_IC_SEED=seed)
            # MSYS bash rebuilds PATH with its own dirs first, so a bare
            # `python` inside run_walk_rl.sh does not resolve to THIS
            # interpreter (or to any). The launcher honors OMNISIM_PYTHON.
            env["OMNISIM_PYTHON"] = sys.executable
            cmd = [sys.executable,
                   str(_REPO / "projects/policies/skills/skill_lib.py"),
                   "run", args.skill, "--duration", str(args.duration),
                   "--gui", "headless"]
            print(f"[conformance] seed={seed} attempt {attempt}: "
                  f"{' '.join(cmd[1:])}", flush=True)
            r = subprocess.run(cmd, cwd=_REPO, env=env)
            rc = r.returncode
            metrics = _analyze_log(
                rl_log.read_text(encoding="utf-8", errors="replace")
            ) if rl_log.exists() else None
            if metrics is not None:
                break
            print(f"[conformance] seed={seed} attempt {attempt}: NO TELEMETRY "
                  f"-- launch flake or dead run"
                  + ("; retrying once" if attempt == 1 else "; giving up"),
                  file=sys.stderr)
        runs.append({"seed": seed, "exit": rc,
                     "wall_s": round(time.time() - t0, 1), "metrics": metrics})

    sys.path.insert(0, str(_REPO / "projects/policies/common"))
    import env_fingerprint as ef
    out = {
        "host": socket.gethostname(),
        "skill": args.skill,
        "duration_s": args.duration,
        "fingerprint": ef.collect(repo_root=_REPO),
        "runs": runs,
    }
    Path(args.out).write_text(json.dumps(out, indent=2, default=str),
                              encoding="utf-8")
    ok = [r for r in runs if r["metrics"] is not None]
    print(f"[conformance] wrote {args.out}: {len(ok)}/{len(runs)} runs usable, "
          f"falls={sum(1 for r in ok if r['metrics']['fell'])}")
    return 0 if len(ok) == len(runs) else 1


def _agg(doc) -> dict | None:
    ms = [r["metrics"] for r in doc["runs"] if r.get("metrics")]
    if not ms:
        return None
    n = len(ms)
    return {
        "n": n,
        "falls": sum(1 for m in ms if m["fell"]),
        "pace": sum(m["pace_mps"] for m in ms) / n,
        "z_mean": sum(m["z_mean"] for m in ms) / n,
        "roll_std": sum(m["roll_std"] for m in ms) / n,
        "pitch_std": sum(m["pitch_std"] for m in ms) / n,
    }


def cmd_compare(args) -> int:
    a = json.loads(Path(args.a).read_text(encoding="utf-8"))
    b = json.loads(Path(args.b).read_text(encoding="utf-8"))
    if a.get("skill") != b.get("skill"):
        print(f"unusable: different skills ({a.get('skill')} vs {b.get('skill')})")
        return 4
    ga, gb = _agg(a), _agg(b)
    if ga is None or gb is None:
        print("unusable: one side has no usable runs")
        return 4

    print(f"skill={a['skill']}  A={a['host']} (n={ga['n']})  B={b['host']} (n={gb['n']})")
    bad = []

    # fingerprint stack: report every mismatch; only [sys] packages gate
    fa, fb = a.get("fingerprint", {}), b.get("fingerprint", {})
    sa = (fa.get("stack") or {}).get("versions") or {}
    sb = (fb.get("stack") or {}).get("versions") or {}
    oa = (fa.get("stack") or {}).get("origin") or {}
    ob = (fb.get("stack") or {}).get("origin") or {}
    for k in sorted(set(sa) | set(sb)):
        if sa.get(k) != sb.get(k):
            gated = "system" in (oa.get(k), ob.get(k))
            print(f"  stack   {k}: {sa.get(k)} vs {sb.get(k)}"
                  + ("   <- [sys]-resolved: MATCH THESE FIRST" if gated else ""))
            if gated:
                bad.append(f"stack:{k}")
    ba = (fa.get("binary") or {}).get("sha256")
    bb = (fb.get("binary") or {}).get("sha256")
    if ba != bb:
        print(f"  binary  sha256 {ba} vs {bb} (expected to differ across machines "
              f"unless both run the same packaged release)")

    def check(name, va, vb, tol, rel=False):
        d = abs(va - vb)
        lim = tol * max(abs(va), abs(vb), 1e-9) if rel else tol
        flag = "ok" if d <= lim else "OVER"
        print(f"  {name:10s} {va:.4f} vs {vb:.4f}   |d|={d:.4f} lim={lim:.4f} {flag}")
        if d > lim:
            bad.append(name)

    if ga["falls"] != gb["falls"]:
        print(f"  falls      {ga['falls']}/{ga['n']} vs {gb['falls']}/{gb['n']}   OUTCOME MISMATCH")
        bad.append("falls")
    else:
        print(f"  falls      {ga['falls']}/{ga['n']} vs {gb['falls']}/{gb['n']}   ok")
    check("pace", ga["pace"], gb["pace"], args.tol_pace, rel=True)
    check("z_mean", ga["z_mean"], gb["z_mean"], args.tol_z)
    check("roll_std", ga["roll_std"], gb["roll_std"], args.tol_att)
    check("pitch_std", ga["pitch_std"], gb["pitch_std"], args.tol_att)

    if bad:
        print(f"VERDICT: DIFFERENT ({', '.join(bad)})")
        return 2
    print("VERDICT: EQUIVALENT (within tolerance bands)")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    rp = sub.add_parser("run", help="run K seeded headless runs and emit a conformance JSON")
    rp.add_argument("--skill", default="g1_walk")
    rp.add_argument("--seeds", default="1,2,3")
    rp.add_argument("--duration", type=int, default=90)
    rp.add_argument("--out", default=f"conformance_{socket.gethostname()}.json")
    apz = sub.add_parser("analyze", help="metrics from one deploy RL log (debug)")
    apz.add_argument("rl_log")
    cp = sub.add_parser("compare", help="compare two conformance JSONs")
    cp.add_argument("a"); cp.add_argument("b")
    cp.add_argument("--tol-pace", type=float, default=0.25,
                    help="relative pace tolerance (default 0.25)")
    cp.add_argument("--tol-z", type=float, default=0.02,
                    help="pelvis height tolerance, m (default 0.02)")
    cp.add_argument("--tol-att", type=float, default=0.02,
                    help="roll/pitch std tolerance, rad (default 0.02)")
    args = ap.parse_args(argv)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "analyze":
        m = _analyze_log(Path(args.rl_log).read_text(encoding="utf-8",
                                                     errors="replace"))
        print(json.dumps(m, indent=2))
        return 0 if m else 4
    return cmd_compare(args)


if __name__ == "__main__":
    sys.exit(main())
