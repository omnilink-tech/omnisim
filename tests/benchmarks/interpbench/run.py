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

"""Does a MODEL interpret these sentences better than the parser does?

    python tests/benchmarks/interpbench/run.py --dry     # count tokens, spend nothing
    python tests/benchmarks/interpbench/run.py           # ~$0.20

THE QUESTION THIS SETTLES
-------------------------
The thesis argues the deterministic path "is not a downgrade". Until now
that compared the parser against the OLD KEYWORD LADDER, never against a
model. So the central claim rested on a comparison nobody had run.

langsoak measured the parser on 64 held-out wordings: 78.2%, with 22% of
must-not-move cases actuating. Whether that is good or bad depends entirely
on a number that did not exist — what a model scores on the SAME sentences.

It also decides an architecture. If the model interprets far better, the
right design is model-interprets / deterministic-executes: the model emits
a typed frame, the gate vets it, the executor runs it. If the model is no
better, that design buys 1.88 s of latency, an API key requirement and
nondeterminism for nothing.

FAIRNESS, DELIBERATELY TILTED TOWARD THE MODEL
----------------------------------------------
Both sides are judged on the INTENT THEY EMIT, not on where a robot ended
up. That removes execution error, settling tolerance and actuator drift —
all of which can only hurt the parser, since the parser is the one that
actually drove a robot in langsoak. The model is also given the real
9,066-character system instruction and all 17 tool definitions captured
verbatim from the bridge (`bridge_payload.json`), not a reconstruction.

If the model still does not win here, it will not win in production.

THE VERDICT
-----------
    STILL  emitted ANY physical tool                 -> false actuation
    MOVE   emitted the right tool with args in range -> pass

`get_robot_state` and the other read-only tools are not motion, so a STILL
case that answers by reading state is correct, for either side.

ENVIRONMENT (the model arm only; the parser arm needs nothing)
    OMNILINK_SA_FILE  path to a Google service-account JSON with Vertex AI
                      access to the project below. GOOGLE_APPLICATION_
                      CREDENTIALS is read as the fallback, since every other
                      google-auth tool already honours it. No default path:
                      this file ships.
"""
from __future__ import annotations

import argparse, json, math, pathlib, statistics, sys, time
import urllib.error, urllib.request

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "tests/benchmarks/langsoak"))
sys.path.insert(0, str(ROOT / "packages/omnisim-bridges/src"))
from corpus import CORPUS, MOVE, STILL  # noqa: E402

# The Google service-account JSON the Vertex call authenticates with. Taken
# from the environment: OMNILINK_SA_FILE, or the standard
# GOOGLE_APPLICATION_CREDENTIALS that every google-auth tool already reads.
# There is no default path -- this file ships, and where an operator keeps
# their credentials is not a public fact about the benchmark.
SA_FILE_ENVS = ("OMNILINK_SA_FILE", "GOOGLE_APPLICATION_CREDENTIALS")
PROJECT = "omnilink-test"
REGION = "us-central1"
MODEL = "gemini-2.5-flash"
PAYLOAD = HERE / "bridge_payload.json"

# Gemini Flash list rates, per COSTS.md — the same ones costbench priced with.
USD_IN_PER_M, USD_OUT_PER_M = 0.50, 3.00

PHYSICAL = {
    "drive_to", "drive_forward", "turn", "set_velocity", "stop_robot",
    "reset_to_home", "resume_autonomy",
}
# Emitting these is not motion: reading state, or scheduling/declining.
READONLY = {
    "get_robot_state", "list_pending_intents", "estimate_time_remaining",
}


def sanitise(schema):
    """Gemini rejects JSON-Schema keys the Ollama tool defs carry."""
    if not isinstance(schema, dict):
        return schema
    drop = {"additionalProperties", "$schema", "default", "examples", "title"}
    out = {}
    for k, v in schema.items():
        if k in drop:
            continue
        if k == "properties" and isinstance(v, dict):
            out[k] = {pk: sanitise(pv) for pk, pv in v.items()}
        elif k == "items":
            out[k] = sanitise(v)
        elif isinstance(v, dict):
            out[k] = sanitise(v)
        else:
            out[k] = v
    return out


def load_bridge_payload(path=None):
    """(system instruction, tool declarations) as the BRIDGE builds them.

    ⚠️ `path` is per ROBOT CLASS. The default is the Husky's, captured
    from the mobile bridge, and it opens "You drive a Clearpath Husky
    mobile base" with seventeen mobile tools. Handing that to a model
    driving an arm tells it the wrong robot and offers it tools the arm
    bridge does not serve -- measured 2026-09-21: 29 commands dispatched,
    0.00 m of TCP motion. Capture one per class with capture_prompt.py.
    """
    d = json.loads(pathlib.Path(path or PAYLOAD).read_text(encoding="utf-8"))
    sysmsg = next(m for m in d["messages"] if m.get("role") == "system")
    decls = []
    for t in d.get("tools") or []:
        f = t.get("function") or {}
        decl = {"name": f["name"], "description": (f.get("description") or "")[:1024]}
        params = sanitise(f.get("parameters") or {})
        if params.get("properties"):
            decl["parameters"] = params
        decls.append(decl)
    return sysmsg["content"], decls


def service_account_file() -> str:
    """The credentials path from the environment, or a legible refusal."""
    import os
    for name in SA_FILE_ENVS:
        value = (os.environ.get(name) or "").strip()
        if value:
            if not pathlib.Path(value).exists():
                raise SystemExit(f"{name} points at {value}, which does not exist")
            return value
    raise SystemExit(
        "the model arm needs Google credentials: set "
        + " or ".join(SA_FILE_ENVS)
        + " to a service-account JSON with Vertex AI access "
          f"(project {PROJECT}, region {REGION}). --dry needs them too: "
          "counting tokens is a Vertex call.")


def token():
    from google.oauth2 import service_account
    import google.auth.transport.requests as gr
    c = service_account.Credentials.from_service_account_file(
        service_account_file(),
        scopes=["https://www.googleapis.com/auth/cloud-platform"])
    c.refresh(gr.Request())
    return c.token


def call(tok, system, decls, text, count_only=False):
    verb = "countTokens" if count_only else "generateContent"
    url = (f"https://{REGION}-aiplatform.googleapis.com/v1/projects/{PROJECT}"
           f"/locations/{REGION}/publishers/google/models/{MODEL}:{verb}")
    body = {
        "contents": [{"role": "user", "parts": [{"text": text}]}],
        "systemInstruction": {"parts": [{"text": system}]},
        "tools": [{"functionDeclarations": decls}],
    }
    if not count_only:
        body["generationConfig"] = {"temperature": 0.0, "maxOutputTokens": 512}
    req = urllib.request.Request(
        url, method="POST", data=json.dumps(body).encode(),
        headers={"Authorization": f"Bearer {tok}",
                 "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read().decode())


def tools_from(resp):
    out = []
    for cand in resp.get("candidates") or []:
        for part in ((cand.get("content") or {}).get("parts") or []):
            fc = part.get("functionCall")
            if fc:
                out.append((fc.get("name"), fc.get("args") or {}))
    return out


# ⚠️ A HALT IS NOT MOTION. `stop_robot` is physical but de-escalating:
# invoking it on a stationary robot changes nothing, and it is the correct
# answer to "stop turning left" and to a retraction. Counting it as a false
# actuation scored the model as dangerous for doing the safe thing, and the
# metric this benchmark exists to measure is "it MOVED when it should have
# sat still". langsoak does not need this rule because it judges by pose,
# where a stop is invisible by construction.
MOTION = PHYSICAL - {"stop_robot"}


def judge(u, calls):
    """Same standard for both interpreters. Returns (verdict, detail)."""
    physical = [(n, a) for n, a in calls if n in MOTION]
    if u.kind == STILL:
        if physical:
            return "false_actuation", ", ".join(n for n, _ in physical)
        return "pass", ""
    if not physical:
        return "missed", ("emitted " + (", ".join(n for n, _ in calls) or "nothing"))
    name, args = physical[0]
    want_lin, want_yaw = math.hypot(u.dx, u.dy), u.dyaw
    if abs(want_yaw) > 1e-6 and abs(want_lin) < 1e-6:
        if name != "turn":
            return "wrong_move", f"expected turn, got {name}"
        got = float(args.get("angle_rad", args.get("angle", 0)) or 0)
        if "degrees" in str(args) or abs(got) > 7:
            got = math.radians(got)
        return ("pass", "") if abs(got - want_yaw) < 0.15 else \
               ("wrong_move", f"turn {got:.3f} rad, wanted {want_yaw:.3f}")
    if abs(want_lin) > 1e-6 and abs(want_yaw) < 1e-6:
        if name not in ("drive_forward", "drive_to"):
            return "wrong_move", f"expected drive, got {name}"
        got = float(args.get("distance", args.get("distance_m", 0)) or 0)
        signed = math.copysign(abs(got), u.dx if u.dx else 1.0)
        return ("pass", "") if abs(signed - u.dx) < 0.2 else \
               ("wrong_move", f"drive {got}, wanted {u.dx}")
    # Compound or net-zero expectations: accept any physical emission as
    # "acted"; the geometry is not decidable from a single frame.
    return "pass", ""


def parser_calls(u):
    """What tier 0 emits for the same sentence, as (tool, args) pairs."""
    from omnisim_bridges.interpret import interpret
    try:
        # ⚠️ LOWERCASE. The signature is `surface: str = "mobile"`, and
        # "MOBILE" is not rejected -- it silently degrades every parse to
        # conversation/0.3/no-frames. Measured with the wrong case, tier 0
        # scored 68.3% with 19 missed commands, and this benchmark
        # published "the model is the better interpreter" on the strength
        # of it. A string argument that accepts anything and means
        # something different is a trap; the fix is here, the lesson is
        # that the parser column must be spot-checked against a known
        # command before it is believed.
        r = interpret(u.text, surface="mobile")
    except Exception as exc:
        return [], f"{type(exc).__name__}"
    return [(f.tool, dict(f.args)) for f in (r.frames or [])], ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true",
                    help="count tokens and price the run; call nothing")
    ap.add_argument("--budget-usd", type=float, default=1.00)
    ap.add_argument("--out", default=str(HERE / "interpbench_result.json"))
    ap.add_argument("--replay", action="store_true",
                    help="re-judge cached model output through the current "
                         "gate; calls nothing, costs nothing")
    a = ap.parse_args()

    if a.replay:
        # Free. Re-judges CACHED model output through the current gate, so
        # the combined architecture can be iterated without paying again.
        d = json.loads(pathlib.Path(a.out).read_text(encoding="utf-8"))
        # ⚠️ RE-JUDGE THE MODEL COLUMN TOO. Its verdicts were computed by
        # whatever judge was current when the API was last called, and the
        # combined column is judged fresh here. Reporting one against the
        # other means comparing two different rules in the same table --
        # which is how the model kept showing 6 false actuations after a
        # halt stopped counting as motion.
        # Re-score the PARSER column too: it was cached from a run that
        # called interpret() with the wrong surface case.
        for r in d["parser"]:
            u = next((x for x in CORPUS if x.uid == r["uid"]), None)
            if u is None:
                continue
            calls, err = parser_calls(u)
            v, det = (judge(u, calls) if not err else ("error", err))
            r["verdict"], r["detail"] = v, det
            r["emitted"] = [n for n, _ in calls]
        for r in d["llm"]:
            u = next((x for x in CORPUS if x.uid == r["uid"]), None)
            if u is None or not r.get("calls"):
                continue
            v, det = judge(u, [(n, ar) for n, ar in r["calls"]])
            r["verdict"], r["detail"] = v, det
        report(d["parser"], d["llm"], d.get("usd", 0.0), 0.0,
               combined(d["llm"]))
        return 0

    system, decls = load_bridge_payload()
    print(f"bridge payload: {len(system)} chars of system instruction, "
          f"{len(decls)} tools, {len(CORPUS)} utterances")

    # ---- the parser, offline and free -------------------------------
    prows = []
    for u in CORPUS:
        calls, err = parser_calls(u)
        v, d = judge(u, calls) if not err else ("error", err)
        prows.append({"uid": u.uid, "family": u.family, "kind": u.kind,
                      "known_gap": u.known_gap, "verdict": v, "detail": d,
                      "emitted": [n for n, _ in calls]})

    tok = token()
    est_in = call(tok, system, decls, CORPUS[0].text, count_only=True)
    per = int(est_in.get("totalTokens", 0))
    cost = len(CORPUS) * (per * USD_IN_PER_M + 120 * USD_OUT_PER_M) / 1e6
    print(f"per-call input: {per} tokens -> estimated {len(CORPUS)} calls "
          f"= ${cost:.3f}")
    if cost > a.budget_usd:
        print(f"ABORT: over the ${a.budget_usd:.2f} budget."); return 1
    if a.dry:
        print("dry run: nothing called."); return 0

    # ---- the model ---------------------------------------------------
    lrows, tin, tout, t0 = [], 0, 0, time.time()
    for i, u in enumerate(CORPUS, 1):
        try:
            resp = call(tok, system, decls, u.text)
        except urllib.error.HTTPError as e:
            lrows.append({"uid": u.uid, "family": u.family, "kind": u.kind,
                          "known_gap": u.known_gap, "verdict": "error",
                          "detail": f"HTTP {e.code} {e.read().decode()[:90]}",
                          "emitted": []})
            continue
        except Exception as exc:
            lrows.append({"uid": u.uid, "family": u.family, "kind": u.kind,
                          "known_gap": u.known_gap, "verdict": "error",
                          "detail": type(exc).__name__, "emitted": []})
            continue
        um = resp.get("usageMetadata") or {}
        tin += int(um.get("promptTokenCount", 0))
        tout += int(um.get("candidatesTokenCount", 0))
        calls = tools_from(resp)
        v, d = judge(u, calls)
        lrows.append({"uid": u.uid, "family": u.family, "kind": u.kind,
                      "known_gap": u.known_gap, "verdict": v, "detail": d,
                      "emitted": [n for n, _ in calls],
                      # ⚠️ ARGS, not just names. Without them the gate
                      # cannot be re-run offline and every iteration on the
                      # combined architecture costs another API bill. The
                      # first version of this file stored names only and
                      # had to be re-queried.
                      "calls": [[n, a] for n, a in calls]})
        if i % 16 == 0:
            print(f"  {i}/{len(CORPUS)}  ${(tin*USD_IN_PER_M + tout*USD_OUT_PER_M)/1e6:.3f}",
                  flush=True)

    spend = (tin * USD_IN_PER_M + tout * USD_OUT_PER_M) / 1e6
    report(prows, lrows, spend, time.time() - t0, combined(lrows))
    pathlib.Path(a.out).write_text(json.dumps(
        {"model": MODEL, "usd": round(spend, 4), "tokens_in": tin,
         "tokens_out": tout, "parser": prows, "llm": lrows}, indent=2),
        encoding="utf-8")
    print(f"  wrote {a.out}")
    return 0


def combined(lrows):
    """MODEL INTERPRETS -> DETERMINISTIC GATE VETS -> survivors execute.

    The architecture under test. The model's frames are filtered by
    gate.check(); anything rejected never reaches an actuator, so the
    robot does nothing for that utterance. Judged by the same standard as
    the other two, which means a gate rejection on a STILL case is a pass
    and on a MOVE case is a miss -- the gate is not free, and refusing a
    legitimate order is a real cost, not a rounding error.
    """
    from omnisim_bridges import gate
    out = []
    for r in lrows:
        u = next(x for x in CORPUS if x.uid == r["uid"])
        calls = [(n, a) for n, a in (r.get("calls") or [])]
        frames = [{"tool": n, "args": a} for n, a in calls]
        rej = gate.check(u.text, frames) if frames else []
        blocked = {x.tool for x in rej}
        survivors = [(n, a) for n, a in calls
                     if gate.normalise(n, a)[0] not in blocked]
        v, d = judge(u, survivors)
        out.append({**r, "verdict": v, "detail": d,
                    "gate_blocked": sorted(blocked),
                    "emitted": [n for n, _ in survivors]})
    return out


def acc(rows):
    real = [r for r in rows if not r["known_gap"]]
    return 100.0 * sum(1 for r in real if r["verdict"] == "pass") / max(1, len(real))


def fa(rows):
    return sum(1 for r in rows
               if not r["known_gap"] and r["verdict"] == "false_actuation")


def report(p, l, spend, secs, c=None):
    print()
    print("=" * 62)
    print("PARSER vs MODEL, same 64 held-out sentences, same judge")
    print("=" * 62)
    still = sum(1 for r in p if not r["known_gap"] and r["kind"] == STILL)
    cc = f"{'model+gate':>12}" if c else ""
    print(f"{'':22} {'parser':>10} {'model':>10}{cc}")
    e = (lambda f: f"{f(c):11.1f}%") if c else (lambda f: "")
    ei = (lambda v: f"{v:12d}") if c else (lambda v: "")
    print(f"{'accuracy':22} {acc(p):9.1f}% {acc(l):9.1f}%{e(acc)}")
    print(f"{'false actuations':22} {fa(p):10d} {fa(l):10d}"
          f"{ei(fa(c)) if c else ''}   of {still} STILL")
    for v in ("missed", "wrong_move", "error"):
        cp = sum(1 for r in p if not r["known_gap"] and r["verdict"] == v)
        cl = sum(1 for r in l if not r["known_gap"] and r["verdict"] == v)
        cg = sum(1 for r in (c or []) if not r["known_gap"] and r["verdict"] == v)
        print(f"{v:22} {cp:10d} {cl:10d}{ei(cg) if c else ''}")
    print(f"{'cost':22} {'$0.00':>10} {'$'+format(spend,'.3f'):>10}"
          f"{('$'+format(spend,'.3f')):>12}" if c else "")
    print(f"{'wall seconds':22} {'<1':>10} {secs:10.0f}")
    if c:
        print()
        print("  what the GATE stopped (model -> model+gate):")
        for rl, rc in zip(l, c):
            if rl["verdict"] != rc["verdict"]:
                sign = "FIXED " if rc["verdict"] == "pass" else "BROKE "
                print(f"    {sign}{rc['uid']:<11} {rl['verdict']:<16} -> "
                      f"{rc['verdict']:<16} blocked={rc.get('gate_blocked')}")

    pm = {r["uid"]: r for r in p}
    print()
    print("  where they DIFFER:")
    for r in l:
        q = pm[r["uid"]]
        if q["verdict"] != r["verdict"]:
            who = "MODEL wins" if r["verdict"] == "pass" else \
                  ("parser wins" if q["verdict"] == "pass" else "both wrong")
            print(f"    {r['uid']:<11} {who:<12} parser={q['verdict']:<16} "
                  f"model={r['verdict']}")


if __name__ == "__main__":
    raise SystemExit(main())
