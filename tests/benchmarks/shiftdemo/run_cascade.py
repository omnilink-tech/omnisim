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

"""A Husky works a 100-prompt shift on the full cascade, and we count it.

    python tests/benchmarks/shiftdemo/run_cascade.py --dry
    python tests/benchmarks/shiftdemo/run_cascade.py

    parser (free)  ->  model, only if the parser declines  ->  gate (free)
                                                            ->  robot

WHY THIS EXISTS SEPARATELY FROM run.py
--------------------------------------
`run.py` routes interpretation through the OmniLink cloud relay. On
2026-09-21 that returned `invalid_grant` on all 82 prompts it reached:
the account's provider grant is lapsed. The run LOOKED like a triumph --
"$0.0000, 100 prompts" -- and was worth nothing, because no model was ever
reached and 39 of 54 commands silently failed. A zero that comes from a
dead credential is not the same zero as a zero that comes from not needing
a model, and a demo that cannot tell them apart is a demo that lies.

This drives the same cascade with a credential that works, so the cost is
real. The stages are identical; only the transport differs.

WHAT IS MEASURED
    parser hits     prompts answered with no model call at all -- free
    model calls     only what the parser declined, tokens from the
                    provider's own usageMetadata
    gate refusals   with the rule that fired, separated from transport
                    failures, which is the distinction run.py got wrong
    robot           judged by pose, because a reply is not a robot
"""
from __future__ import annotations

import argparse, importlib, json, math, os, pathlib, subprocess, sys, time
import urllib.error, urllib.request

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "packages/omnisim-bridges/src"))
sys.path.insert(0, str(ROOT / "tests/benchmarks/interpbench"))
from profiles import PROFILES, verify_observed, EXPECT_MOVE  # noqa: E402
from omnisim_bridges import gate  # noqa: E402
from omnisim_bridges.interpret import interpret  # noqa: E402

import importlib.util as _ilu  # noqa: E402
_spec = _ilu.spec_from_file_location(
    "ib", ROOT / "tests/benchmarks/interpbench/run.py")
ib = _ilu.module_from_spec(_spec); _spec.loader.exec_module(ib)

# ⚠️ SET BY --profile IN main(). They were module constants naming one
# Husky world and one port; every one of them is a per-robot-class fact.
PROFILE = PROFILES["husky"]
WORLD = PROFILE.world
PORT = PROFILE.port
USD_IN_PER_M, USD_OUT_PER_M = 0.50, 3.00
PER_CALL_USD = (5592 * USD_IN_PER_M + 84 * USD_OUT_PER_M) / 1e6


def get(path, t=20, tries=3):
    last = None
    for i in range(tries):
        try:
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{PORT}{path}", timeout=t) as r:
                return json.loads(r.read().decode())
        except Exception as exc:
            last = exc
            if i + 1 < tries:
                time.sleep(0.4 * (i + 1))
    raise last


def post_tool(tool, args, t=120):
    req = urllib.request.Request(
        f"http://127.0.0.1:{PORT}/tool", method="POST",
        data=json.dumps({"tool": tool, **args}).encode(),
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=t) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode())
        except Exception:
            return e.code, {}
    except Exception as exc:
        return 0, {"transport": type(exc).__name__}


def pose():
    """The comparable pose, via the profile. RAISES if the bridge has none.

    ⚠️ THIS USED TO BE `float(s.get("x", 0))` AND THAT WAS THE BUG.
    An arm publishes q/tcp/gripper and a quadruped published position, so
    the defaulting read returned (0,0,0) for every sample on any robot but
    a mobile base. Every `cmd` then scored "did not move" and every ask /
    chat / refuse / halt scored CORRECT, because those four only require
    stillness -- and the demo's headline, zero false actuations, came out
    perfect on a robot nobody could see. See profiles.PoseUnavailable.
    """
    return PROFILE.pose(get("/state"))


def settle(timeout=None):
    """Watch until the robot is still, or until the profile's window ends.

    ⚠️ The window is per robot class. Waiting for stillness assumes the
    motion terminates, and a quadruped's `walk` does not -- it is a mode.
    """
    timeout = PROFILE.settle_s if timeout is None else timeout
    last, stable, t0 = pose(), 0, time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        now = pose()
        if all(abs(a - b) < PROFILE.settle_eps for a, b in zip(now, last)):
            stable += 1
            if stable >= 3:
                return now
        else:
            stable = 0
        last = now
    return pose()


# Frames the parser emits, mapped onto the bridge's HTTP tool names.
# Per profile: the quadruped has no `turn` tool, the drone's stop is on a
# different verb, and a table that assumed the Husky's names silently
# posted to endpoints the other bridges do not serve.


def parser_frames(text):
    """(frames, confident). Tier 0 gets first refusal and costs nothing."""
    try:
        # The profile's surface, and `interpret` now RAISES on an unknown
        # one. It used to accept anything: "MOBILE" instead of "mobile"
        # degraded every parse to conversation/0.3/no-frames, and on that
        # basis interpbench published the parser at 68.3% against the
        # model's 91.7%. Corrected, it is 90.0%. Fanning out across four
        # surfaces multiplies that trap by four, so it is closed.
        r = interpret(text, surface=PROFILE.surface)
    except Exception:
        return [], False
    conf = getattr(r, "confidence", 0.0) or 0.0
    intent = str(getattr(r, "intent", "") or "")
    frames = [(f.tool, dict(f.args)) for f in (r.frames or [])]
    # Only a confident COMMAND is answered without a model. Everything
    # else -- questions, chat, anything the parser is unsure of -- is
    # exactly what we are paying the model for.
    return frames, bool(frames) and conf >= 0.7 and "COMMAND" in intent.upper()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="husky", choices=sorted(PROFILES),
                    help="which robot class works the shift")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--limit", type=int, default=0,
                    help="run only the first N prompts")
    ap.add_argument("--budget-usd", type=float, default=0.30)
    ap.add_argument("--parser-only", action="store_true",
                    help="tier 0 + gate only: no model, no credential, $0.00")
    ap.add_argument("--out", default=str(HERE / "cascade_result.json"))
    a = ap.parse_args()

    global PROFILE, WORLD, PORT
    PROFILE = PROFILES[a.profile]
    WORLD, PORT = PROFILE.world, PROFILE.port
    SHIFT = importlib.import_module(PROFILE.script_module).SHIFT

    # Price the worst case before spending anything.
    n_parser = sum(1 for t, _ in SHIFT if parser_frames(t)[1])
    print(f"shift: {PROFILE.name} ({PROFILE.surface}) -- {len(SHIFT)} prompts")
    print(f"  world: {PROFILE.world}")
    print(f"  parser answers (free):   {n_parser}")
    print(f"  would reach the model:   {len(SHIFT) - n_parser}"
          f"  ~= ${(len(SHIFT) - n_parser) * PER_CALL_USD:.3f}")
    print(f"  budget cap: ${a.budget_usd:.2f}")
    if a.dry:
        return 0

    # ── The FREE configuration: tier 0 + gate, no model ──────────────
    #
    # This is the configuration the release notes call the headline, and
    # it is the only one every robot class can run today: the quadruped
    # and the flying bridge have no Ollama path, so capture_prompt.py
    # cannot record their system instruction and tool declarations, and
    # running them against the Husky's would measure the wrong robot.
    #
    # It needs no credential, spends $0.00, and its remaining failures are
    # MISSED commands rather than wrong ones -- it declines to act rather
    # than acting wrongly. That is the property worth measuring per class.
    system, decls, tok = None, None, None
    if a.parser_only:
        # ASCII only: this prints to a Windows console at cp1252, where a
        # warning glyph raises UnicodeEncodeError and takes the whole run
        # with it. The comments may carry it; the output may not.
        print("  model tier: DISABLED (--parser-only). Tier 0 + gate, $0.00.")
        print("  !! A prompt the parser declines does NOTHING here. Missed")
        print("     commands are expected; wrong ones are not.")
    else:
        pass

    # The profile's OWN system instruction and tool declarations. Handing
    # every robot the Husky's is how the arm's first run dispatched 29
    # commands and moved 0.00 m.
    payload_path = ROOT / "tests/benchmarks/interpbench" / PROFILE.payload
    if not a.parser_only and not payload_path.is_file():
        print(f"\n  MISSING PAYLOAD for {PROFILE.name}: {payload_path}")
        print("  capture it with:")
        print("    python tests/benchmarks/interpbench/capture_prompt.py"
              f" --world {PROFILE.world}"
              f" --port {PROFILE.port}"
              f" --out {payload_path}")
        # ⚠️ REFUSE rather than fall back to the Husky's. A model told it
        # is driving the wrong robot, and offered tools this bridge does
        # not serve, produces a run that looks complete and measures
        # nothing -- 29 commands dispatched, 0.00 m moved.
        print("  Refusing to run: another robot's prompt is not a default.")
        return 1
    if not a.parser_only:
        system, decls = ib.load_bridge_payload(payload_path)
        print(f"  model tier: {PROFILE.payload} "
              f"({len(system)} chars, {len(decls)} tools)")
        tok = ib.token()

    logdir = pathlib.Path(os.environ.get("TEMP", "/tmp")) / "cascade"
    logdir.mkdir(parents=True, exist_ok=True)
    # ⚠️ KEEP OMNI_KEY. /tool dispatches through `relay.tools[...]`, so
    # with no relay the endpoint is 503 and every command silently fails.
    # The first cascade run stripped the key "to be safe", dispatched 72
    # commands, moved the robot 0.0 m and still printed a cost. The
    # relay's PROVIDER grant being lapsed does not matter here -- nothing
    # in this demo asks the relay to interpret; it only needs the tool
    # registry that /tool resolves names against.
    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = str(logdir / "cascade.log")
    # Whatever this world needs to come up at all. See ShiftProfile.env:
    # the Mavic's controller exits 0 without OMNISIM_URDF_USE_SENSORS=1,
    # which looks like success and leaves nothing listening.
    env.update(PROFILE.env)
    proc = subprocess.Popen(
        [sys.executable, "-m", "omnisim", "run-headless", WORLD,
         "--duration", "5400"], cwd=str(ROOT), stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, env=env)

    rows, tin, tout, calls, t0 = [], 0, 0, 0, time.time()
    dist_total, robot_cmds = 0.0, 0
    try:
        for _ in range(200):
            try:
                get("/state", 2); break
            except Exception:
                time.sleep(1)
        else:
            print("bridge never came up"); return 1
        settle(8.0)
        origin = pose()
        print(f"\n{'#':>3} {'kind':<7} {'via':<7} {'moved':>7}  prompt")
        print("-" * 76)

        shift = SHIFT[:a.limit] if a.limit else SHIFT
        for i, (text, kind) in enumerate(shift, 1):
            p0 = pose()
            frames, confident = parser_frames(text)
            via = "parser"
            if not confident and a.parser_only:
                # ⚠️ DECLINED, AND NOTHING HAPPENS. Recorded as its own
                # `via` so a missed command is never confused with a
                # parser that answered: the free configuration's failures
                # are misses, and a table that hid them behind "parser"
                # would claim coverage it does not have.
                frames, via = [], "declined"
            elif not confident:
                via = "model"
                try:
                    resp = ib.call(tok, system, decls, text)
                    um = resp.get("usageMetadata") or {}
                    tin += int(um.get("promptTokenCount", 0))
                    tout += int(um.get("candidatesTokenCount", 0))
                    calls += 1
                    frames = ib.tools_from(resp)
                except Exception as exc:
                    frames, via = [], f"ERR:{type(exc).__name__}"

            # ── the deterministic veto, on whatever produced the frames ──
            #
            # ⚠️ PASS THE SURFACE. Without it `move_body` falls back to
            # the ground rail, and this runner is a SEPARATE PROCESS from
            # the engine -- no bridge has registered anything here, so
            # there is not even a recorded surface to fall back to.
            # Measured 2026-09-21 on the drone: "climb another 2 metres",
            # "fly forward 5 metres" and "go forward 4 metres" were all
            # refused `implausible` against a quadruped's 1 m body-shift
            # limit. Three false refusals of ordinary flying, produced by
            # the one argument this call did not pass.
            rej = gate.check(text, [{"tool": n, "args": ar}
                                    for n, ar in frames],
                             surface=PROFILE.surface) if frames else []
            blocked = {r.tool for r in rej}
            survivors = [(n, ar) for n, ar in frames
                         if gate.normalise(n, ar)[0] not in blocked]

            sent = []
            for name, args in survivors:
                http_name = PROFILE.tool_http.get(name, name)
                code, _body = post_tool(http_name, args)
                sent.append((http_name, code))
                robot_cmds += 1
            p1 = settle()
            moved, d = PROFILE.moved(p0, p1)
            dist_total += d
            # EXPECT_MOVE, not `kind == "cmd"`. `hover`, `stand` and a
            # gripper closing are positive orders whose correct execution
            # is near-zero displacement; scored as cmd they fail for
            # succeeding, and scored as halt they pass for being ignored.
            ok = moved == EXPECT_MOVE[kind]
            rows.append({
                "n": i, "prompt": text, "kind": kind, "via": via,
                "frames": [n for n, _ in frames],
                # ARGS too: without them no iteration on the gate is free,
                # and this file has now been re-billed twice for that.
                "calls": [[n, ar] for n, ar in frames], "sent": sent,
                "gate_rules": sorted({r.rule for r in rej}),
                "moved": moved, "dist_m": round(d, 3),
                "as_expected": ok, "pose": [round(v, 3) for v in p1]})
            if i % 10 == 0 or not ok:
                print(f"{i:>3} {kind:<7} {via:<7} {d:>6.2f}m  {text[:42]}"
                      f"{'' if ok else '   <-- unexpected'}")
            spend = (tin * USD_IN_PER_M + tout * USD_OUT_PER_M) / 1e6
            if spend > a.budget_usd:
                print(f"\n  ABORT: ${spend:.3f} passed the cap at {i}"); break
            # Per profile: a 12x12 floor for the Husky, 6x6 for the
            # quadruped, an altitude ceiling for the drone, and NO planar
            # rail for the arm -- its bound is a reach envelope nothing
            # here enforces, and inventing a number would read as one.
            oob = PROFILE.out_of_bounds(p1)
            if oob:
                print(f"\n  ABORT: {oob}"); break
    finally:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                       capture_output=True)

    dt = time.time() - t0
    spend = (tin * USD_IN_PER_M + tout * USD_OUT_PER_M) / 1e6
    by_parser = sum(1 for r in rows if r["via"] == "parser")
    gated = [r for r in rows if r["gate_rules"]]
    errs = [r for r in rows if r["via"].startswith("ERR")]
    # ⚠️ A DISPATCH THAT WAS REFUSED BY HTTP IS NOT A TRANSPORT ERROR AND
    # IS NOT A GATE REFUSAL, AND IT USED TO BE NEITHER.
    #
    # `errs` counts rows whose `via` starts with ERR:, which only happens
    # when the MODEL call raises. A POST to /tool that comes back 503 or
    # 400 is a recorded status, not an exception, so it landed in no
    # column at all. Measured 2026-09-21 on the quadruped: four of
    # fourteen dispatches returned 503 `tool_not_registered` -- the parser
    # emits `turn` on the quadruped surface and that bridge serves six
    # tools without it -- and the summary printed "transport errors: 0".
    # The command never ran and nothing said so.
    bad_dispatch = [(r["n"], t, c) for r in rows
                    for t, c in r["sent"] if not (200 <= c < 300)]
    per = {}
    for r in rows:
        dd = per.setdefault(r["kind"], [0, 0])
        dd[1] += 1; dd[0] += 1 if r["as_expected"] else 0
    end = tuple(rows[-1]["pose"]) if rows else tuple(0.0 for _ in origin)

    print("\n" + "=" * 76)
    print(f"SHIFT COMPLETE  {len(rows)} prompts in {dt/60:.1f} min")
    print("=" * 76)
    print(f"  robot commands dispatched   {robot_cmds}")
    # ⚠️ THE EFFORT METRIC IS NAMED BY THE PROFILE, NOT HARDCODED.
    # "distance driven" on an arm is 0.0 m and always will be: its base is
    # bolted down. A label that travels unchanged between robot classes is
    # how a number that measures nothing gets published as a result.
    print(f"  {PROFILE.effort_label:<27} {dist_total:.2f}")
    print(f"  ended at                    "
          f"({', '.join(f'{v:.2f}' for v in end)})   "
          f"origin ({', '.join(f'{v:.2f}' for v in origin)})")
    print(f"  pose read as                {'/'.join(PROFILE.pose_keys)}")
    print("  behaved as expected, by kind:")
    for k in ("cmd", "halt", "hold", "ask", "refuse", "chat"):
        if k in per:
            o, n = per[k]
            print(f"    {k:<8} {o:>3}/{n:<3}")
    print(f"  refused by the GATE         {len(gated)}"
          f"   (transport errors, separately: {len(errs)})")
    if bad_dispatch:
        by_code = {}
        for _n, t, c in bad_dispatch:
            by_code.setdefault(c, set()).add(t)
        print(f"  !! DISPATCHES THAT DID NOT RUN  {len(bad_dispatch)}")
        for c in sorted(by_code):
            why = ("tool not served by this bridge" if c == 503
                   else "refused by the gate" if c == 400 else "")
            print(f"       HTTP {c}: {', '.join(sorted(by_code[c]))}"
                  f"{'  -- ' + why if why else ''}")
    print()
    print(f"  answered by the parser      {by_parser}   free, no model call")
    print(f"  model calls                 {calls}")
    print(f"  tokens                      {tin} in / {tout} out")
    print(f"  ** MEASURED COST            ${spend:.4f} **")
    print(f"  per prompt                  ${spend/max(1,len(rows)):.5f}")

    # ⚠️ THE EVIDENCE CHECK. A scoreboard in which nothing moved is not a
    # pass, it is a broken instrument -- and it is one that scores WELL,
    # because every row but `cmd` only requires stillness. This is the
    # last thing printed so it cannot be missed, and it makes the run
    # FAIL rather than publish.
    void = verify_observed(rows)
    if void:
        print()
        print("=" * 76)
        print(f"  {void}")
        print("=" * 76)
    # ⚠️ NO COUNTERFACTUAL RATIO HERE, ON PURPOSE.
    #
    # An earlier version printed "a model deciding each robot command
    # would cost $X (Yx this shift)" and it came out at 0.9x -- i.e.
    # cheaper. That is not a defect in the architecture, it is a category
    # error in the comparison: in a CONVERSATION each prompt is roughly
    # one command, so there is nothing to amplify, and the thesis ratio
    # does not apply. Printing it anyway would have invited exactly the
    # wrong conclusion from the flagship demo.
    #
    # The amplification case is a repetitive job, where one instruction
    # drives thousands of commands. That is measured separately in
    # tests/benchmarks/soak: 4,648 commands over two hours for $0.00.
    print()
    if not void:
        print(f"  what this measures: the cost of TALKING to a robot for a")
        print(f"  shift -- ${spend/max(1,len(rows)):.5f} per thing said.")
        print(f"  It does NOT show the 26,916x figure, because conversation")
        print(f"  does not amplify: each prompt here is about one command.")
        print(f"  The amplification case is tests/benchmarks/soak -- 4,648")
        print(f"  robot commands from one repeated instruction, for $0.00.")

    # ⚠️ THE EVIDENCE IS WRITTEN EVEN WHEN THE RUN IS VOID, and especially
    # then. A void run is the most diagnostic artefact this script
    # produces -- it names 29 dispatched commands against 0.00 m of
    # motion, which is what you need to find out WHY. Returning early
    # threw that away and left nothing on disk to look at.
    pathlib.Path(a.out).write_text(json.dumps({
        "void": void or None,
        "prompts": len(rows), "minutes": round(dt/60, 2),
        "robot_commands": robot_cmds, "distance_m": round(dist_total, 2),
        "by_parser": by_parser, "model_calls": calls,
        "tokens_in": tin, "tokens_out": tout, "usd": round(spend, 6),
        "gate_refusals": len(gated), "transport_errors": len(errs),
        "dispatches_that_did_not_run": bad_dispatch,
        "profile": PROFILE.name, "surface": PROFILE.surface,
        "effort_label": PROFILE.effort_label,
        "rows": rows}, indent=2), encoding="utf-8")
    print(f"\n  wrote {a.out}")
    # ⚠️ A VOID RUN EXITS NON-ZERO. The per-kind table it printed looks
    # like a pass -- four of the six rows score correct whenever the robot
    # does not move -- so anything reading the exit code must not be told
    # this succeeded.
    return 1 if void else 0


if __name__ == "__main__":
    raise SystemExit(main())
