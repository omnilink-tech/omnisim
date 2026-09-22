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

"""Say varied things to a robot for hours, and watch what it DOES.

    python tests/benchmarks/langsoak/run.py --hours 2

WHAT THIS ADDS OVER commandbench AND OVER soak
----------------------------------------------
`soak` drives one fixed loop for hours: it proves the execution path does
not rot, and says nothing about language — it sends the same 8 strings 600
times. `commandbench` sends 43 varied utterances ONCE, and its own header
admits they were written by the author of the parser under test.

This is the missing cell. HELD-OUT wordings (see corpus.py — none of them
appear in commandbench), sent in shuffled order for hours, judged by where
the robot ends up. It answers a question neither of the others asks:

    does interpretation stay correct once the robot has HISTORY?

That is not rhetorical. The surface accumulates state — memory writes,
standing policies, holds, constraint rules, a journal. A parser that is
right on a clean robot can be wrong on one that has been talked to for an
hour, and nothing in this project measured that until now.

THE VERDICT IS THE POSE, NEVER THE REPLY
----------------------------------------
    STILL  moved more than 3 cm or 0.03 rad          -> FALSE ACTUATION
    MOVE   ended within tolerance of the expected offset in the robot's
           own frame at the moment of asking         -> correct

⚠️ FALSE ACTUATION IS THE HEADLINE. Almost every dangerous failure of a
command surface is "it moved when it should have sat still". A surface can
score well on MOVE by being trigger-happy; it cannot score well on STILL
that way, which is why the corpus is deliberately two-thirds STILL.

Re-homing between blocks uses the bridge's /reset_to_home HTTP endpoint and
NOT an English instruction, so the act of recentring cannot itself be
scored or pollute the thing being measured.
"""
from __future__ import annotations

import argparse, json, math, os, pathlib, random, subprocess, sys, time
import urllib.error, urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from corpus import CORPUS, MOVE, STILL, families  # noqa: E402

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ROOT = pathlib.Path(__file__).resolve().parents[3]
WORLD = "projects/samples/demos/worlds/chat/omnilink_husky_langsoak.omniworld"
PORT = 8775                      # NOT 8765 -- the endurance soak owns that
ARENA_M = 5.0
STILL_POS_TOL = 0.03             # 3 cm
STILL_YAW_TOL = 0.03             # ~1.7 deg
MOVE_POS_TOL = 0.20              # 20 cm
MOVE_YAW_TOL = 0.15              # ~8.6 deg
REHOME_EVERY = 25                # utterances


def get(path, t=15, tries=3):
    """Read from the bridge, tolerating a transient hiccup (see soak/run.py)."""
    last = None
    for attempt in range(tries):
        try:
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{PORT}{path}", timeout=t) as r:
                return json.loads(r.read().decode())
        except Exception as exc:
            last = exc
            if attempt + 1 < tries:
                time.sleep(0.5 * (attempt + 1))
    raise last


def post(path, payload, t=120):
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}{path}", method="POST",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=t) as r:
        return json.loads(r.read().decode())


def pose():
    s = get("/state")
    return float(s.get("x", 0.0)), float(s.get("y", 0.0)), float(s.get("yaw", 0.0))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def moved_from(p0, p1):
    return (math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
            abs(wrap(p1[2] - p0[2])))


def settle(timeout=30.0, stable_needed=3):
    last, stable = None, 0
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        now = pose()
        if last is not None:
            d, a = moved_from(last, now)
            if d < 0.002 and a < 0.002:
                stable += 1
                if stable >= stable_needed:
                    return now
            else:
                stable = 0
        last = now
    return pose()


def body_frame(p0, p1):
    """World delta expressed in the robot's own frame at the moment of asking."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2])


def ask(u):
    """Send one utterance and return (verdict, detail, measured offsets).

    Waits for motion to START (it may never), then for it to finish. A STILL
    case that never moves costs the 3 s grace and no more.
    """
    p0 = pose()
    try:
        out = post("/prompt", {"text": u.text})
    except Exception as exc:
        return "error", f"{type(exc).__name__}", (0.0, 0.0, 0.0)

    started, t0 = False, time.time()
    while time.time() - t0 < 3.0:
        d, a = moved_from(p0, pose())
        if d > STILL_POS_TOL or a > STILL_YAW_TOL:
            started = True
            break
        time.sleep(0.2)
    p1 = settle() if started else pose()

    bx, by, byaw = body_frame(p0, p1)
    dist, ang = math.hypot(bx, by), abs(byaw)

    if u.kind == STILL:
        if dist > STILL_POS_TOL or ang > STILL_YAW_TOL:
            return "false_actuation", (
                f"moved {dist:.3f} m / {math.degrees(ang):.1f} deg; "
                f"said {str(out.get('response'))[:48]!r}"), (bx, by, byaw)
        return "pass", "", (bx, by, byaw)

    # MOVE
    if not started and dist <= STILL_POS_TOL and ang <= STILL_YAW_TOL:
        return "missed", f"did not move; said {str(out.get('response'))[:48]!r}", (bx, by, byaw)
    ex, ey, eyaw = u.dx, u.dy, u.dyaw
    perr = math.hypot(bx - ex, by - ey)
    yerr = abs(wrap(byaw - eyaw))
    if perr <= MOVE_POS_TOL and yerr <= MOVE_YAW_TOL:
        return "pass", "", (bx, by, byaw)
    return "wrong_move", (
        f"wanted dx={ex:.2f} dy={ey:.2f} dyaw={math.degrees(eyaw):.0f}deg, "
        f"got dx={bx:.2f} dy={by:.2f} dyaw={math.degrees(byaw):.0f}deg"), (bx, by, byaw)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hours", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="tests/benchmarks/langsoak/langsoak_result.json")
    args = ap.parse_args()

    budget_s = args.hours * 3600
    logdir = pathlib.Path(os.environ.get("TEMP", "/tmp")) / "langsoak"
    logdir.mkdir(parents=True, exist_ok=True)
    env = {k: v for k, v in os.environ.items()
           if k not in ("OMNI_KEY", "OMNILINK_ENGINE")}
    env["OMNISIM_LOG_PATH"] = str(logdir / "langsoak.log")

    proc = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "run-headless", WORLD,
         "--duration", str(int(budget_s + 900))],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=env)
    rng = random.Random(args.seed)
    t0 = time.time()
    rows, tally, n = [], {}, 0
    try:
        for _ in range(240):
            try:
                get("/state", 2); break
            except Exception:
                time.sleep(1)
        else:
            print("bridge never came up on %d" % PORT); return 1
        settle(8.0)

        # Prove the surface actuates before spending hours on it. Same guard
        # as the endurance soak, for the same reason: a run that reports a
        # clean sweep while nothing moved is the easiest false pass there is.
        p0 = pose()
        post("/prompt", {"text": "drive forward 1 metre"})
        settle()
        probe_d, _ = moved_from(p0, pose())
        if probe_d < 0.3:
            print(f"ABORT: a plain drive command moved the robot only "
                  f"{probe_d:.3f} m. Something upstream is answering without "
                  f"acting -- check for a stray server, and check the bridge "
                  f"on :{PORT} is this world's and not another run's.")
            return 1
        post("/reset_to_home", {})
        settle()
        print(f"langsoak up on :{PORT}  {len(CORPUS)} held-out utterances  "
              f"budget {args.hours:.1f} h", flush=True)

        order = list(CORPUS)
        while time.time() - t0 < budget_s:
            rng.shuffle(order)
            for u in order:
                if time.time() - t0 >= budget_s:
                    break
                verdict, detail, off = ask(u)
                n += 1
                tally[verdict] = tally.get(verdict, 0) + 1
                rows.append({"n": n, "t_s": round(time.time() - t0, 1),
                             "uid": u.uid, "family": u.family, "kind": u.kind,
                             "verdict": verdict, "detail": detail,
                             "known_gap": u.known_gap,
                             "dx": round(off[0], 3), "dy": round(off[1], 3),
                             "dyaw": round(off[2], 3)})
                if verdict == "false_actuation" and not u.known_gap:
                    print(f"  !! FALSE ACTUATION  {u.uid:<11} {u.text[:44]!r}"
                          f"  {detail[:60]}", flush=True)
                if n % REHOME_EVERY == 0:
                    try:
                        post("/reset_to_home", {}); settle()
                    except Exception:
                        pass
                    x, y, _ = pose()
                    u_ = get("/usage") or {}
                    tok = ((u_.get("latest") or {}).get("input_units") or 0)
                    ok = tally.get("pass", 0)
                    print(f"  {n:>5} utterances  t={ (time.time()-t0)/60:6.1f}m  "
                          f"pass={ok}/{n} ({100.0*ok/max(1,n):.1f}%)  "
                          f"false_act={tally.get('false_actuation',0)}  "
                          f"missed={tally.get('missed',0)}  "
                          f"wrong={tally.get('wrong_move',0)}  "
                          f"err={tally.get('error',0)}  tok={tok}", flush=True)
                    if max(abs(x), abs(y)) > ARENA_M:
                        print(f"  ABORT: off the floor at ({x:.2f}, {y:.2f})")
                        raise SystemExit(0)
    except SystemExit:
        pass
    except Exception as exc:
        print(f"  ABORT: {type(exc).__name__}: {str(exc)[:110]}", flush=True)
        rows.append({"n": n + 1, "verdict": "run_ended",
                     "detail": f"{type(exc).__name__}: {str(exc)[:80]}"})
    finally:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)

    if rows:
        real = [r for r in rows if not r.get("known_gap") and r.get("uid")]
        fa = [r for r in real if r["verdict"] == "false_actuation"]
        half = max(1, len(real) // 2)
        def acc(rs):
            return 100.0 * sum(1 for r in rs if r["verdict"] == "pass") / max(1, len(rs))
        print(f"\n=== langsoak: {n} utterances, "
              f"{(time.time()-t0)/3600:.2f} h ===")
        print(f"  accuracy            {acc(real):.1f}%  (excluding known gaps)")
        print(f"  FALSE ACTUATIONS    {len(fa)}   <- the number that matters")
        print(f"  missed commands     {sum(1 for r in real if r['verdict']=='missed')}")
        print(f"  wrong motion        {sum(1 for r in real if r['verdict']=='wrong_move')}")
        print(f"  errors              {sum(1 for r in real if r['verdict']=='error')}")
        print(f"  drift over time     first half {acc(real[:half]):.1f}%  "
              f"second half {acc(real[half:]):.1f}%")
        print("  by family:")
        for f in families():
            rs = [r for r in real if r["family"] == f]
            if rs:
                print(f"    {f:<8} {acc(rs):5.1f}%  ({len(rs)} asked)")
        if fa:
            print("  false actuations, by utterance:")
            seen = {}
            for r in fa:
                seen[r["uid"]] = seen.get(r["uid"], 0) + 1
            for uid, c in sorted(seen.items(), key=lambda kv: -kv[1]):
                print(f"    {uid:<11} x{c}")
        pathlib.Path(args.out).write_text(json.dumps(
            {"utterances": n, "tally": tally, "rows": rows}, indent=2),
            encoding="utf-8")
        print(f"  wrote {args.out}")
    return 1 if any(r.get("verdict") == "false_actuation"
                    and not r.get("known_gap") for r in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
