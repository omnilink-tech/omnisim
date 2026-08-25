"""The containment guard: a ``PreToolUse`` hook that REFUSES a tool call
before it runs, instead of noticing afterwards that it ran.

Why this exists
---------------

``stage_workspaces`` hands the session a FILTERED copy of the product. The
filter is real and it works -- and it is enforced on the copy only. On
2026-08-11 the first R4/omnisim cell ran ``ls -la /o/omnisim/``, found the
source tree at its real path, and made 56 tool calls into it: the graders
(``graders/r4_core.py``, ``graders/r4.py``), the task's own ``meta.json``,
and the Webots oracle's controller and world. ``run_cc_cell.audit_containment``
DETECTS that and blocks the cell before grading, which is the right last
resort and the wrong first one: it costs a whole cell (and the operator's
quota) to learn something we could have refused in 30 ms.

This module is the first resort. It is installed as a ``PreToolUse`` hook
through ``claude --settings <file>``, sees every tool call's input before the
tool runs, and returns ``permissionDecision: "deny"`` for the ones that reach
outside the cell.

THE RULE (and why it is drawn here)
-----------------------------------

**Admitted: everything inside the cell's own staged workspace.** That is the
product as a real user has it -- ``AGENTS.md``, ``docs/``, ``scripts/``, the
engine binary and runtime through the ``projects/`` ``msys64/`` ``lib/``
``resources/`` junctions, and the **engine C++ source under ``src/``**.

The ``src/`` inclusion is a deliberate ruling, not an oversight. A previous
session read ``src/omnisim/nodes/OmBasicJoint.cpp`` to work out why a motor
behaved as it did, and the question is whether that is legitimate use or
contamination. It is legitimate:

* OmniSim is Apache 2.0 and ships its source; every real user diagnosing a
  motor *can* open that file. Removing it would measure a product we do not
  ship.
* It is not answer key. It contains no task, no threshold, no grader and no
  oracle solution -- nothing that tells an agent what it is being measured on
  or what "correct" is. The answer key is the benchmark tree, and that is what
  is excluded.
* For a DIAGNOSTIC round the read is itself the finding. An agent that has to
  read C++ to learn what a documented API does has found a documentation
  defect, and suppressing the read would suppress the signal.

The honest cost, stated rather than buried: the upstream-Webots arm's
workspace is a binary install with no equivalent C++ tree, so a cross-sim
comparison that turns on source reading is asymmetric. That asymmetry belongs
in the comparison round's caveats (and is fixable there by staging upstream's
source, which is equally public). It does not belong in a single-sim
diagnostic round, which this is.

**Refused: everything else on the machine that is ours.** Specifically:

1. the real repo checkout by any spelling (``O:\\omnisim``, ``/o/omnisim``,
   ``/mnt/o/omnisim``) -- *except* the four junction directories, whose bytes
   are already in the workspace by design;
2. anything matching ``stage_workspaces``' own exclusion list by name
   (``tests/benchmarks/agentbench/``, ``tests/benchmarks/ladder/``,
   ``tests/benchmarks/loopbench/``, ``social/``, ``cloud/``, and the named
   answer-key documents), wherever it appears;
3. the lane's scratch root -- other cells' workspaces, the staging templates
   and manifests;
4. the cell's own run directory, which holds this guard, the stream, the
   preserved workspace mirror and the grading output.

Rule 1 is stated as "the whole checkout" rather than "the benchmark subtree
inside it" on purpose. Denying only the subtree leaves ``ls /o/omnisim`` legal,
and that single call is how the last session learned the benchmark existed;
it also leaves ``git log``, ``CHANGELOG.md`` and the other redacted documents
reachable, each of which the staging filter removed for a reason. Everything
legitimate in the checkout is already in the workspace at the same relative
path, so the denial costs the agent nothing it is entitled to.

What this canNOT do, stated plainly
-----------------------------------

The hook sees the tool call's INPUT. A path the agent composes at run time
inside a script it wrote -- ``open(os.environ["HOMEDRIVE"] + "/omnisim/...")``
-- is not in any tool input and is not seen. So this is prevention for the
direct route (which is the measured one) and not a sandbox. That is why
``audit_containment`` stays: two independent layers, one before and one after,
and the report carries both.

Refusal text is deliberately NEUTRAL. It says "outside this project
workspace"; it never says "benchmark", "grader" or "answer key". A refusal
that names the benchmark tells the agent it is being measured, which changes
the thing being measured.
"""

from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path

#: Written next to this file when the guard is installed into a cell.
CONFIG_NAME = "guard_config.json"

#: Repo subdirectories that the workspace attaches as junctions. Their bytes
#: ARE the workspace's bytes, so naming them through the real repo path is
#: the same read as naming them through the workspace and is allowed. Kept in
#: sync with ``stage_workspaces.OMNISIM_JUNCTION_DIRS`` by the installer,
#: which passes the value rather than re-declaring it.
DEFAULT_JUNCTION_DIRS = ("msys64", "projects", "lib", "resources")

#: The refusal the agent sees. Product language only.
DENY_TEMPLATE = (
    "Denied: %(what)s is outside this project workspace. This session is "
    "confined to %(ws)s -- work from the files there (the runtime "
    "directories are attached inside it). Paths elsewhere on this machine "
    "are not available."
)

# --- machine-wide process termination -----------------------------------------
#
# CONTAINMENT SCOPED FILES AND SAID NOTHING ABOUT PROCESSES, AND THAT COST TWO
# RUNS. Measured 2026-08-12, `20260812_round1_a1_omnisim_c1`: the agent's own
# cleanup matched processes by IMAGE NAME (`Name -eq 'omnisim-bin.exe'`,
# `taskkill /IM`), which reaps every engine on the machine. It killed a
# concurrent lane's engine twice -- its kill list names that lane's
# `instances\...\_R4\...` path -- then reported "remaining omnisim-bin: 0",
# and also killed its own PowerShell. The other lane's agent mis-diagnosed the
# resulting failure as its own reload bug.
#
# So: terminating by NAME/PATTERN is refused, terminating by PID is not. That
# is the same line the harness's own sweeps now draw for themselves, it costs
# an honest agent nothing (it knows the pids it started), and the refusal is
# product-shaped -- a real multi-user machine is exactly where this rule
# belongs.
#
# Read-only enumeration (`Get-Process -Name x`, `tasklist`, `pgrep`) is
# deliberately NOT refused: finding out what is running is legitimate, and the
# diagnostic value of watching an agent look is worth keeping.
_KILL_BY_NAME = tuple(re.compile(p) for p in (
    # taskkill /IM <image>   (and /F /T in any order)
    r"\btaskkill\b[^\n|;&]*\s/im\b",
    # Stop-Process -Name x   /  Stop-Process -processname x
    r"\bstop-process\b[^\n|;&]*\s-(?:name|processname)\b",
    # ... | Stop-Process     -- the pipeline form: whatever produced the
    # objects, this terminates a SET the agent did not enumerate by pid
    r"\|\s*stop-process\b",
    # posix pattern kills
    r"\bpkill\b", r"\bkillall\b",
    r"\bkill\b[^\n|;&]*\$\(\s*(?:pgrep|pidof)\b",
))

#: The COMPOUND form, and the one actually measured: select a process SET by
#: image name, then terminate each member by its pid. Every member pid is
#: legitimate-looking and the set is still machine-wide -- this is verbatim
#: what `a1_omnisim_c1` ran before reporting "remaining omnisim-bin: 0".
_KILL_VERB = re.compile(r"(?:\bstop-process\b|\btaskkill\b|(?<![a-z])kill\b"
                        r"|\.kill\(|\.terminate\(\))")
_NAME_SELECTOR = re.compile(
    r"(?:\$_\.name\s*-(?:eq|like|match|in)"          # Where-Object { $_.Name -eq
    r"|\bget-process\b[^\n|;&]*\s-name\b"            # Get-Process -Name x | ...
    r"|\bget-process\b\s+[\"']?[a-z0-9_.\-]*omnisim" # Get-Process omnisim-bin
    r"|-filter\s*[\"'][^\"']*\bname\s*="             # -Filter "name='x.exe'"
    r"|\bprocessname\b)")

DENY_KILL_TEMPLATE = (
    "Denied: terminating processes by name or pattern (%(what)s) is not "
    "available in this session -- it would reap every matching process on "
    "this machine, including ones this session did not start. Terminate the "
    "processes you started by PID instead (e.g. taskkill /PID <pid> /T /F, "
    "Stop-Process -Id <pid>, kill <pid>); listing processes is unrestricted."
)


def _norm(text):
    """Lowercase, slash-normalised, so a Windows path, a Git-Bash path and a
    WSL path all read the same."""
    return (text or "").replace("\\\\", "/").replace("\\", "/").lower()


def path_spellings(path):
    """Every spelling of an absolute path a tool call could plausibly carry.

    ``O:\\omnisim`` -> ``o:/omnisim``, ``/o/omnisim``, ``/mnt/o/omnisim``,
    ``//o/omnisim``. Generated from the path itself rather than hard-coded, so
    a clone at another location is covered without an edit.
    """
    raw = _norm(str(path)).rstrip("/")
    out = {raw}
    m = re.match(r"^([a-z]):(/.*)$", raw)
    if m:
        drive, rest = m.group(1), m.group(2)
        out.add("/%s%s" % (drive, rest))          # Git Bash / MSYS
        out.add("//%s%s" % (drive, rest))         # doubled mount
        out.add("/mnt/%s%s" % (drive, rest))      # WSL
        out.add("/cygdrive/%s%s" % (drive, rest))
    return tuple(sorted(out, key=len, reverse=True))


def _strings(obj, out, depth=0):
    """Every string anywhere in the tool input. Total: a tool we have never
    seen still gets scanned, because the field names are not enumerated."""
    if depth > 8:
        return
    if isinstance(obj, str):
        out.append(obj)
    elif isinstance(obj, dict):
        for v in obj.values():
            _strings(v, out, depth + 1)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            _strings(v, out, depth + 1)


def _needle_patterns(prefixes, files):
    """``(label, pattern)`` for the staging exclusion list.

    Same matching discipline as ``run_cc_cell._containment_needles``: a
    multi-component prefix matches without its trailing slash but with a
    boundary after it (so both ``.../agentbench/graders/x.py`` and
    ``find ... /agentbench -name`` are caught while ``agentbench_notes`` is
    not); a single-component prefix keeps its slash and demands a boundary
    before it (so ``cd cloud/x`` matches and the English word in
    ``grep -r "cloud" docs/`` does not).
    """
    out = []
    for p in prefixes:
        s = _norm(p).strip("/")
        if not s:
            continue
        core = re.escape(s)
        pat = (core + r"(?![a-z0-9_.\-])" if "/" in s
               else r"(?<![a-z0-9_.\-])" + core + "/")
        out.append((s + "/", re.compile(pat)))
    for f in files:
        s = _norm(f).strip("/")
        if s:
            out.append((s, re.compile(re.escape(s))))
    return tuple(out)


def _escapes_root(blob, spellings, allow_first=(), allow_paths=()):
    """Does ``blob`` name something under ``spellings`` that is not allowed?

    Returns the offending substring, or None. ``allow_first`` names first path
    components that are permitted under the root (the junction dirs);
    ``allow_paths`` are already-normalised absolute prefixes that are
    permitted wholesale (the cell's own workspace, when it lives under the
    root being guarded).
    """
    for root in spellings:
        for m in re.finditer(re.escape(root) + r"(?![a-z0-9_.\-])", blob):
            tail = blob[m.end():]
            # Just the offending path, cut at the first whitespace: the raw
            # window used to drag the tool's `description` field into the
            # refusal text, which read as noise to the agent.
            hit = re.split(r"[\s'\"]", blob[m.start():m.end() + 80])[0]
            if any(blob[m.start():].startswith(a) for a in allow_paths):
                continue
            if not tail.startswith("/"):
                # The bare root: `ls -la /o/omnisim`. This is the discovery
                # call the whole guard exists to stop.
                return hit
            comp = re.match(r"/([a-z0-9_.\-]+)", tail)
            if comp and comp.group(1) in allow_first:
                continue
            return hit
    return None


def decide(tool_name, tool_input, cfg):
    """``(allow: bool, reason: str|None, matched: str|None)``.

    Pure: no I/O, so the unit tests exercise exactly what the hook runs.
    """
    blobs = []
    _strings(tool_input, blobs)
    blob = _norm(" \n ".join(blobs))
    # HTTP(S) urls are dropped before matching. They are not local paths, and
    # a url whose path segment happens to read ``/cloud/`` is not a read of
    # our private ops tree -- a needle that fires on ``http://x/cloud/y``
    # trains the reader to ignore the gate. Local harness urls
    # (``http://127.0.0.1:6789/...``) are the common case and must stay legal;
    # any local path in the same command is a separate token and still scans.
    blob = re.sub(r"https?://[^\s'\"]+", " ", blob)
    if not blob.strip():
        return True, None, None

    if cfg.get("deny_machine_wide_kill", True):
        m = next((h for h in (p.search(blob) for p in _KILL_BY_NAME) if h),
                 None)
        if m is None:
            sel = _NAME_SELECTOR.search(blob)
            if sel is not None and _KILL_VERB.search(blob) is not None:
                m = sel
        if m is not None:
            return (False,
                    DENY_KILL_TEMPLATE % {
                        "what": "%r" % blob[max(0, m.start() - 10):
                                            m.end() + 30].strip()},
                    "machine_wide_kill")

    for label, pat in _needle_patterns(cfg.get("exclude_prefixes") or (),
                                       cfg.get("exclude_files") or ()):
        m = pat.search(blob)
        if m:
            return (False,
                    DENY_TEMPLATE % {"what": "%r" % blob[max(0, m.start() - 20):
                                                         m.end() + 20].strip(),
                                     "ws": cfg.get("workspace", "?")},
                    label)

    ws_norm = _norm(cfg.get("workspace") or "")
    ws_spellings = path_spellings(ws_norm) if ws_norm else ()

    repo = cfg.get("repo")
    if repo:
        hit = _escapes_root(blob, path_spellings(repo),
                            allow_first=tuple(cfg.get("junction_dirs")
                                              or DEFAULT_JUNCTION_DIRS))
        if hit:
            return (False,
                    DENY_TEMPLATE % {"what": "%r" % hit.strip(),
                                     "ws": cfg.get("workspace", "?")},
                    "repo_checkout")

    scratch = cfg.get("scratch_root")
    if scratch:
        hit = _escapes_root(blob, path_spellings(scratch),
                            allow_paths=ws_spellings)
        if hit:
            return (False,
                    DENY_TEMPLATE % {"what": "%r" % hit.strip(),
                                     "ws": cfg.get("workspace", "?")},
                    "other_cell_or_template")

    for prot in cfg.get("protect") or ():
        for sp in path_spellings(prot):
            if sp in blob:
                return (False,
                        DENY_TEMPLATE % {"what": "%r" % sp,
                                         "ws": cfg.get("workspace", "?")},
                        "cell_run_dir")
    return True, None, None


def load_config(here=None):
    here = Path(here or __file__).resolve().parent
    p = here / CONFIG_NAME
    return json.loads(p.read_text(encoding="utf-8"))


def main(argv=None):
    """Hook entry point. Reads the event on stdin, writes the decision on
    stdout, always exits 0.

    FAIL CLOSED. If the guard cannot parse its input or its config, it DENIES.
    A guard that fails open is a guard that silently stops guarding, and the
    whole reason this file exists is that the last silent failure cost a cell.
    A guard that fails closed breaks the cell loudly on its first tool call,
    which is a harness defect you find in seconds.
    """
    started = time.time()
    rec, cfg, decision = {}, {}, None
    try:
        rec = json.loads(sys.stdin.read() or "{}")
        cfg = load_config()
        allow, reason, matched = decide(rec.get("tool_name"),
                                        rec.get("tool_input") or {}, cfg)
        decision = (allow, reason, matched)
    except Exception as exc:                     # noqa: BLE001 - fail closed
        decision = (False,
                    "Denied: the workspace guard could not evaluate this call "
                    "(%s). Retry with a path inside your working directory."
                    % exc.__class__.__name__,
                    "guard_error:%s" % exc)
    allow, reason, matched = decision
    log = cfg.get("log")
    if log:
        try:
            with open(log, "a", encoding="utf-8") as fh:
                fh.write(json.dumps({
                    "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "tool": rec.get("tool_name"),
                    "allow": allow, "matched": matched,
                    "ms": round((time.time() - started) * 1000, 1),
                    "input": json.dumps(rec.get("tool_input") or {},
                                        default=str)[:600],
                }) + "\n")
        except OSError:
            pass
    if allow:
        # Say NOTHING and exit 0: "no decision, carry on". Returning an
        # explicit ``allow`` would override the session's own permission
        # handling for every call, which is a different (and larger) change
        # than the one this guard is for.
        pass
    else:
        print(json.dumps({"hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "permissionDecision": "deny",
            "permissionDecisionReason": reason}}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
