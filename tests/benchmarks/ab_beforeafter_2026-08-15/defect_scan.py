#!/usr/bin/env python3
"""Count DEFECT ENCOUNTERS in a cc_lane cell, by signature.

The A/B's primary metric. Pass/fail at n=3 is far too noisy to carry a claim;
"how many times did the agent hit a thing we fixed" is direct and countable.

Reads a cell dir's `transcript.jsonl` (preferred: it carries timestamps) or
`cc_stream.jsonl`, pairs every assistant `tool_use` with its `tool_result`,
and matches each pair against the signature table below. Also reads
`cell_report.json` for the cross-cell / grading-hygiene signatures, which are
properties of the RUN rather than of the agent's transcript.

A scanner that cannot go red is worthless, so `--self-test` runs the table
against synthetic pairs, and the round-1 cells (which are known to contain
these defects) are the live calibration set.

Usage:
    python defect_scan.py <cell_dir> [<cell_dir> ...] [--json out.json]
    python defect_scan.py --self-test
"""
import argparse
import json
import os
import re
import sys
from datetime import datetime

# --- the signature table -----------------------------------------------------
#
# Each entry: (id, where, input_re, result_re, note)
#   where = "pair"   -> input_re must match the tool INPUT and result_re the RESULT
#           "result" -> result_re alone (input_re may still narrow it, or be None)
#           "input"  -> input_re alone (the agent ACTING on a stale doc claim)
#
# Signatures are deliberately narrow: a false positive here would manufacture
# the very improvement the round is trying to measure.

SIGNATURES = [
    # 1. /world/screenshot returning a zero-byte body
    dict(id="screenshot_zero_bytes", where="pair",
         input_re=r"(?i)/world/screenshot|world/screenshot",
         result_re=r"(?i)(\b0 bytes\b|zero[- ]byte|empty (?:png|image|file|body)"
                   r"|Length:\s*0\b|size[:= ]+0\b|\b0\s*$)",
         note="/world/screenshot wrote an empty body"),
    # 2. /sim/reset leaving the scene frozen
    dict(id="reset_freeze", where="pair",
         input_re=r"(?i)/sim/reset|simulationReset",
         result_re=r"(?i)(0\.00+\s*m\b|displacement[^0-9]{0,20}0\.0{2,}"
                   r"|did ?n[o']t move|no(?:t| ) moved|frozen|stuck at)",
         note="/sim/reset left the robots frozen"),
    # 2b. THE SYMPTOM, however it was caused: the agent measured a fleet that
    #     did not move. Round-1 c1 hit this twice (calls #27, #31: "min net
    #     displacement: 0.00 m  max: 0.00 m") before working around it and
    #     reading 5.14-30.27 m at #35 -- but the reset that froze the scene was
    #     several calls earlier, so a same-call pairing cannot see it.
    #     Symmetric across arms and cheap to audit by hand.
    dict(id="zero_motion_reading", where="result",
         input_re=None,
         result_re=r"(?i)(displacement[^\n]{0,60}\b0\.00+\b[^\n]{0,60}\b0\.00+\b"
                   r"|\b(?:max|mean)[^\n]{0,25}displacement[^\n]{0,15}[:= ]\s*0\.00+\b"
                   r"|\b0\s*/\s*10 moved\b|no robots? moved"
                   r"|none of the (?:10 )?robots moved)",
         note="the agent measured a fleet that did not move"),
    # 2c. the agent AUTHORING a workaround for the reset freeze -- round-1 c1
    #     wrote "NOTE: POST /sim/reset was tried first and leaves the scene
    #     inert" into its own proof script's docstring. Counted separately from
    #     `zero_motion_reading`: after the fix the harness DISCLOSES the
    #     mechanism, so an arm-B agent can write the same note without having
    #     paid to discover it. Encounter, not cost.
    dict(id="reset_workaround_authored", where="input",
         input_re=r"(?i)(/sim/reset[^\n]{0,160}(inert|does not restart|freez|"
                  r"leaves the scene|not re-?issue|brake)"
                  r"|(inert|freez|does not restart)[^\n]{0,80}/sim/reset)",
         result_re=None,
         note="agent wrote a workaround/warning about the /sim/reset freeze"),
    # 3. a 503 where a 4xx belonged
    dict(id="http_503", where="pair",
         input_re=r"(?i)(:678[89]|:679[12]|/world/|/sim/|/scene/|/capture/|/robot)",
         result_re=r"(?<![0-9])503(?![0-9])|Service Unavailable",
         note="the service answered 503"),
    # 4. /robots counting the injected supervisor
    dict(id="robots_counts_supervisor", where="pair",
         input_re=r"(?i)/robots\b|GET /robots",
         result_re=r"(?i)(AGENTBENCH_SUP|HARNESS_SUP|_SUPERVISOR\b|\"supervisor\""
                   r"|OMNISIM_SUP|SUP_ROBOT)",
         note="/robots included the harness's own supervisor"),
    # 5. /capture/screenshot with a path
    dict(id="capture_path_failed", where="pair",
         input_re=r"(?i)/capture/screenshot",
         result_re=r"(?i)(503|error|failed|no such file|not found|denied)",
         note="/capture/screenshot with a path failed"),
    # 6. acting on stale ODE-era doc claims.
    #    ⚠ COMMENTS ARE STRIPPED FIRST, and that correction was made after
    #    both arms had run. Raw, this fired 1x on arm A and 2x on arm B -- and
    #    inspection showed ALL THREE were the agent doing the RIGHT thing:
    #    each wrote `newtonGroundMu` and added a comment saying
    #    "`contactProperties`/`coulombFriction` is the dead ODE-path field and
    #    is NOT read". The regex was matching the agent's own explanation of
    #    the defect. Counting that as an encounter would have scored correct
    #    behaviour as a defect on both arms. Note the correction removes MORE
    #    from arm B (2) than from arm A (1), so it is not a flattering edit --
    #    it is a wrong one being undone. Both raw and corrected are reported.
    dict(id="stale_ode_doc_claim", where="input", strip_comments=True,
         input_re=r"(?i)(coulombFriction|ContactProperties\s*\{|physicsDisableTime"
                  r"|physicsDisableLinearThreshold|WorldInfo[^}]{0,200}\bCFM\b"
                  r"|\bERP\b\s+[0-9]|ImmersionProperties|physicsBackend\s+\"ode\")",
         result_re=None,
         note="agent wrote/tuned an ODE-era field the engine does not read"),
    # 7. harness/capture service refused to start or died under the agent
    dict(id="service_unreachable", where="pair",
         input_re=r"(?i)(:6789|:6791|omnisim_harness|omnisim_capture|127\.0\.0\.1:679)",
         result_re=r"(?i)(connection refused|failed to connect|couldn't connect"
                   r"|could not connect|Connection reset|actively refused"
                   r"|EADDRINUSE|address already in use)",
         note="the harness/capture service was unreachable"),
    # 8. the run-headless log-only PASS / engine crash markers
    dict(id="engine_crash", where="result",
         input_re=None,
         result_re=r"(0xC0000005|0xC0000135|Segmentation fault|Access violation"
                   r"|SIMULATOR_EXITED_NONZERO)",
         note="the engine crashed under the agent"),
]

# Signatures read off the cell REPORT, not the transcript.
#
# ⚠ `cross_cell_interference` was HERE and has been REMOVED from the defect
# count, after arm A c1 and before any arm B cell existed. It counted every
# kill the cell's sweeps made -- and on arm A c1 both were the cell reaping its
# OWN shell and its OWN harness at teardown, which is correct behaviour, not
# interference. Genuine cross-cell interference is also structurally
# unmeasurable in this A/B: the isolation fix that prevents it is part of the
# INSTRUMENT, which is identical on both arms by design. Sweep kills are still
# reported, as context, by `sweep_kills` below.
REPORT_SIGNATURES = [
    ("graded_on_dot_sibling",
     lambda r: 1 if re.search(r"[\\/]\.(?:capture|harness|omnisim)_",
                              str(((r.get("artifact") or {}).get("found")) or "")) else 0,
     "the graded artifact is a dot-prefixed sibling the agent did not write"),
    ("killed_read_as_rate_limit",
     lambda r: sum(1 for d in (r.get("rate_limit_deferrals") or [])
                   if "4294967295" in json.dumps(d)),
     "a killed session was deferred as a rate limit"),
]

# Context, NOT defects: reported per cell but never summed into the headline.
CONTEXT_SIGNALS = [
    ("measured_under_concurrency",
     lambda r: 1 if r.get("measured_under_concurrency") else 0),
    ("sweep_kills",
     lambda r: sum(len(s.get("kills") or [])
                   for s in (r.get("process_sweeps") or []))),
    ("sweep_skipped_other_cell",
     lambda r: sum(1 for s in (r.get("process_sweeps") or [])
                   for k in (s.get("kills") or [])
                   if k.get("action") == "skipped_other_cell")),
]

ERROR_RE = re.compile(
    r"(?i)(\btraceback\b|\berror\b|\bexception\b|\bfailed\b|not recognized"
    r"|no such file|cannot find|refused|timed? out|\b4\d\d\b|\b5\d\d\b"
    r"|is not defined|command not found)")

WORKING_SCENE_RE = re.compile(
    r"(?i)(world finalis?zed|world finalised|\[OmNewtonBackend\] world finalis"
    r"|0 errors, 0 warnings|\bPASS\b|\"ok\"\s*:\s*true|\"loaded\"\s*:\s*true"
    r"|Simulation loaded|loaded successfully)")


def _txt(o, limit=200000):
    if o is None:
        return ""
    if isinstance(o, str):
        return o[:limit]
    try:
        return json.dumps(o)[:limit]
    except Exception:
        return str(o)[:limit]


def load_pairs(cell):
    """[(idx, ts, tool, input_text, result_text, is_error)] for one cell."""
    tpath = os.path.join(cell, "transcript.jsonl")
    spath = os.path.join(cell, "cc_stream.jsonl")
    path = tpath if os.path.exists(tpath) and os.path.getsize(tpath) else spath
    if not os.path.exists(path):
        return [], None, path
    calls, results, t0 = {}, {}, None
    order = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except ValueError:
                continue
            ts = d.get("timestamp")
            if ts and t0 is None:
                t0 = ts
            msg = d.get("message") or {}
            content = msg.get("content")
            if not isinstance(content, list):
                continue
            for b in content:
                if not isinstance(b, dict):
                    continue
                if b.get("type") == "tool_use":
                    calls[b["id"]] = (b.get("name"), _txt(b.get("input")), ts)
                    order.append(b["id"])
                elif b.get("type") == "tool_result":
                    rid = b.get("tool_use_id")
                    results[rid] = (_txt(b.get("content")),
                                    bool(b.get("is_error")), ts)
    pairs = []
    for i, cid in enumerate(order):
        name, inp, ts = calls[cid]
        res, is_err, rts = results.get(cid, ("", False, None))
        pairs.append((i, ts or rts, name, inp, res, is_err))
    return pairs, t0, path


def _sec(t0, ts):
    if not t0 or not ts:
        return None
    try:
        a = datetime.fromisoformat(t0.replace("Z", "+00:00"))
        b = datetime.fromisoformat(ts.replace("Z", "+00:00"))
        return round((b - a).total_seconds(), 1)
    except Exception:
        return None


def scan_pairs(pairs, t0):
    hits = []
    for (i, ts, tool, inp, res, is_err) in pairs:
        blob_in = "%s %s" % (tool or "", inp)
        # A .wbt / script written by the agent arrives JSON-escaped, so its
        # newlines are the two characters \ and n. Dropping every line whose
        # first non-space char is '#' leaves only code the engine will read.
        blob_nocomment = " ".join(
            ln for ln in re.split(r"\\n|\n", blob_in)
            if not ln.lstrip().startswith("#"))
        for sig in SIGNATURES:
            src = blob_nocomment if sig.get("strip_comments") else blob_in
            ire, rre = sig["input_re"], sig["result_re"]
            if sig["where"] == "input":
                if ire and re.search(ire, src):
                    hits.append(dict(sig=sig["id"], call=i, t_s=_sec(t0, ts),
                                     tool=tool, note=sig["note"],
                                     excerpt=inp[:220]))
            else:
                if ire and not re.search(ire, src):
                    continue
                if rre and not re.search(rre, res):
                    continue
                if not rre:
                    continue
                hits.append(dict(sig=sig["id"], call=i, t_s=_sec(t0, ts),
                                 tool=tool, note=sig["note"],
                                 excerpt=(inp[:110] + " => " + res[:160])))
    return hits


def scan_cell(cell):
    pairs, t0, src = load_pairs(cell)
    hits = scan_pairs(pairs, t0)
    report = {}
    rp = os.path.join(cell, "cell_report.json")
    if os.path.exists(rp):
        try:
            report = json.load(open(rp, encoding="utf-8"))
        except Exception:
            report = {}
    rhits = []
    for name, fn, note in REPORT_SIGNATURES:
        try:
            n = fn(report) or 0
        except Exception:
            n = 0
        if n:
            rhits.append(dict(sig=name, count=int(n), note=note))
    context = {}
    for name, fn in CONTEXT_SIGNALS:
        try:
            context[name] = int(fn(report) or 0)
        except Exception:
            context[name] = None

    err_calls = [p for p in pairs if p[5] or ERROR_RE.search(p[4][:4000])]

    # "tool calls lost to defects" -- a PROXY, defined so it is reproducible
    # rather than judged: a call is counted as lost when it either matches a
    # defect signature itself, or is an error call within LOST_WINDOW calls
    # after one. It cannot separate "diagnosing the tool" from "diagnosing my
    # own mistake", so read it as an upper bound on defect-driven waste.
    LOST_WINDOW = 4
    err_idx = {p[0] for p in err_calls}
    hit_idx = sorted({h["call"] for h in hits})
    lost = set(hit_idx)
    for i in hit_idx:
        for j in range(i + 1, i + 1 + LOST_WINDOW):
            if j in err_idx:
                lost.add(j)
    calls_lost = len(lost)
    first_scene = None
    for (i, ts, tool, inp, res, _e) in pairs:
        if WORKING_SCENE_RE.search(res[:20000]):
            first_scene = dict(call=i, t_s=_sec(t0, ts), tool=tool,
                               excerpt=res[:160])
            break

    by_sig = {}
    for h in hits:
        by_sig[h["sig"]] = by_sig.get(h["sig"], 0) + 1
    for h in rhits:
        by_sig[h["sig"]] = by_sig.get(h["sig"], 0) + h["count"]

    row = (report.get("row") or {})
    met = (report.get("row_metrics") or {})
    n_pass = n_tot = None
    gr = os.path.join(cell, "grader_row.json")
    if os.path.exists(gr):
        try:
            g = json.load(open(gr, encoding="utf-8"))
            asserts = g.get("assertions") or {}
            if isinstance(asserts, dict):
                n_tot = len(asserts)
                n_pass = sum(1 for v in asserts.values()
                             if v is True or (isinstance(v, dict)
                                              and v.get("ok") is True))
        except Exception:
            pass
    return dict(
        cell=os.path.basename(cell), path=os.path.abspath(cell), source=src,
        tool_calls=len(pairs),
        defect_encounters=len(hits) + sum(h["count"] for h in rhits),
        transcript_encounters=len(hits),
        by_signature=by_sig,
        hits=hits, report_hits=rhits, context=context,
        error_calls=len(err_calls), calls_lost_to_defects=calls_lost,
        error_call_pct=(round(100.0 * len(err_calls) / len(pairs), 1)
                        if pairs else None),
        time_to_first_working_scene_s=(first_scene or {}).get("t_s"),
        first_working_scene=first_scene,
        outcome=row.get("outcome"), failed_assertion=row.get("failed_assertion"),
        assertions_passed=n_pass, assertions_total=n_tot,
        t_agent_s=met.get("t_agent_s"), usd=met.get("usd"),
        engine_sha=(((report.get("engine") or {}).get("engine") or {})
                    .get("sha256")),
        pinned_model=report.get("pinned_model"),
    )


def self_test():
    """The scanner must be able to go RED. Synthetic pairs, one per signature."""
    cases = [
        ("screenshot_zero_bytes",
         "curl -X POST http://127.0.0.1:6789/world/screenshot -o shot.png",
         "wrote shot.png: 0 bytes"),
        ("reset_freeze",
         "curl -X POST http://127.0.0.1:6789/sim/reset -d '{}'",
         "husky_0 displacement 0.000 m after 200 steps"),
        ("http_503",
         "curl http://127.0.0.1:6791/capture/screenshot",
         "HTTP/1.0 503 Service Unavailable"),
        ("robots_counts_supervisor",
         "curl http://127.0.0.1:6789/robots",
         '{"robots":[{"def":"AGENTBENCH_SUP"},{"def":"HUSKY_0"}]}'),
        ("capture_path_failed",
         'POST /capture/screenshot {"path":"out.png"}',
         "503 the capture service is not serving"),
        ("stale_ode_doc_claim",
         'edit world: contactProperties [ ContactProperties { coulombFriction 1.2 } ]',
         "ok"),
        ("service_unreachable",
         "curl http://127.0.0.1:6789/healthz",
         "curl: (7) Failed to connect to 127.0.0.1 port 6789: Connection refused"),
        ("engine_crash", "run world", "omnisim-bin.exe exited 0xC0000005"),
        ("zero_motion_reading", "python proof.py",
         "min net displacement: 0.00 m   max: 0.00 m   mean: 0.00 m"),
    ]
    ok = True
    for want, inp, res in cases:
        pairs = [(0, None, "Bash", inp, res, False)]
        got = {h["sig"] for h in scan_pairs(pairs, None)}
        mark = "RED " if want in got else "MISS"
        if want not in got:
            ok = False
        print("%s %-28s %s" % (mark, want, sorted(got)))
    # and a clean pair must stay green
    clean = [(0, None, "Bash", "ls projects/samples", "world.wbt controllers")]
    clean = [(0, None, "Bash", "ls projects/samples", "world.wbt controllers", False)]
    got = {h["sig"] for h in scan_pairs(clean, None)}
    print("%s %-28s %s" % ("GREEN" if not got else "FALSE+", "clean pair", sorted(got)))
    if got:
        ok = False
    print("\nself-test: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cells", nargs="*")
    ap.add_argument("--json", default=None)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    out = [scan_cell(c) for c in a.cells]
    for r in out:
        print("== %s  outcome=%s  tool_calls=%s  defects=%s  err_calls=%s(%s%%)  "
              "t_first_scene=%ss" %
              (r["cell"], r["outcome"], r["tool_calls"], r["defect_encounters"],
               r["error_calls"], r["error_call_pct"],
               r["time_to_first_working_scene_s"]))
        for k, v in sorted(r["by_signature"].items(), key=lambda kv: -kv[1]):
            print("     %-28s %d" % (k, v))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        print("\nwrote %s" % a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
