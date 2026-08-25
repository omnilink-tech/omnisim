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

"""Stage clean product workspaces for the Claude Code lane (Phase W).

One workspace per simulator arm, built OUTSIDE the repo (default: a dir under
``%TEMP%``), so a headless Claude Code session dropped into it sees **the
product and only the product** -- never the benchmark's answer key.

OmniSim arm
    The repo's *tracked* files (``git ls-files``, so build scratch, gitignored
    results and stray CSVs can never leak), MINUS the answer key
    (``tests/benchmarks/agentbench/**``, the validation-plan and
    control-baseline docs -- see ``OMNISIM_EXCLUDE_*`` below), with the
    multi-GB binary/runtime dirs (``msys64/``, ``projects/``, ``lib/``,
    ``resources/``) attached as **directory junctions** into the real tree
    instead of copies. Junctioned dirs are runtime/product material only and
    contain no answer-key files; the write-through risk (a session inside the
    workspace CAN write through a junction into the real tree) is recorded in
    the manifest rather than pretended away.

Webots arm
    A small dir exposing the verified upstream R2025a install in WSL2
    (``adapters/webots/BRINGUP.md``) via a directory symlink to
    ``\\\\wsl$\\<distro>/opt/upstream-webots/R2025a``. Upstream's own ``docs/``
    ship inside that install tree, so they are reachable through the link.
    Nothing authored by us goes into the workspace beyond the task files --
    our text there would contaminate the product comparison.

Layout under ``--root`` (never inside the repo)::

    <root>/templates/<sim>/                 the template tree (plain files only)
    <root>/templates/<sim>.manifest.json    staging manifest (outside the ws)
    <root>/instances/<name>/                per-cell instances (run_cc_cell.py)
    <root>/instances/<name>.manifest.json

Templates hold **no links**; junctions/symlinks are recorded in the manifest
and materialised per instance, so instantiating a workspace can never recurse
through a junction and copy gigabytes -- and teardown severs links with
``rmdir`` semantics (``os.rmdir``) before any recursive delete, so nothing is
ever deleted *through* a junction.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import textwrap
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../agentbench/cc_lane
AGENTBENCH = HERE.parent
REPO = AGENTBENCH.parents[2]

DEFAULT_ROOT = Path(os.environ.get("TEMP") or os.environ.get("TMP")
                    or "/tmp") / "agentbench_cc"

# --- the OmniSim workspace recipe -------------------------------------------

# Binary/runtime dirs attached as junctions (no answer-key material in any of
# them; sizes measured 2026-08-01: msys64 1165 MB, projects 1699 MB).
# lib/ is junctioned because libController (lib/controller/Controller.dll) is
# a BUILD OUTPUT -- untracked, so a tracked-files copy would miss it and
# `python -m omnisim doctor` would report an incoherent install.
# resources/ likewise carries built runtime pieces (qt_utils) the engine loads.
OMNISIM_JUNCTION_DIRS = ("msys64", "projects", "lib", "resources")

# The answer key, and private trees that are not part of the public product.
OMNISIM_EXCLUDE_PREFIXES = (
    "tests/benchmarks/agentbench/",   # the benchmark itself: graders, oracle
    #                                   solutions, thresholds, task worlds
    "tests/benchmarks/ladder/",       # the capability ladder: same reason
    "tests/benchmarks/loopbench/",    # LOOPBENCH: graders, thresholds, tasks
    "social/",                        # deny-listed from the public snapshot
    "cloud/",                         # private ops tree, not in the snapshot
)
OMNISIM_EXCLUDE_FILES = (
    # The two docs the lane design names explicitly:
    "docs/developer/agent-edge-validation-plan.md",
    "docs/developer/webots-control-baseline.md",
    # Additional leak found by grep while staging (2026-08-01): this latency
    # ledger names the AgentBench task worlds (incl. C2 fall_through) and
    # their backend pins -- benchmark internals, not product docs.
    "docs/developer/harness-latency-2026-07-31.md",
    # CHANGELOG.md names the C2 defect verbatim ("a world whose floor Solid
    # has no boundingObject" under the task id). Not on the required include
    # list, so it is excluded rather than doctored.
    "CHANGELOG.md",
    # Internal roadmap doc AGENTS.md itself says is excluded from the public
    # snapshot.
    "docs/developer/omnilink-roadmap.md",
    # --- benchmark write-ups (2026-08-02) ------------------------------------
    # These are the ANSWER, not documentation. ladder-findings carries the
    # tuned friction recipe that solves T2 outright (elliptic cone, impratio,
    # contact ke, a constant-force squeeze, the mu that works); the others name
    # tasks, thresholds, cells and verdicts. They were written the same day the
    # cells ran, so leaving them staged would hand a later agent the answer to
    # the task it is being measured on -- and would silently invalidate any
    # before/after against a cell that ran before they existed. The quarantine
    # caught all of them on 2026-08-03 and refused to spend a session, which is
    # exactly what it is for.
    "docs/developer/ladder-findings-2026-08-02.md",
    "docs/developer/ladder-grid-2026-08-02.md",
    "docs/developer/loopbench-plan.md",
    "docs/developer/capability-ladder-plan.md",
    "docs/developer/cold-launch-failure-2026-08-02.md",
)

# Answer-key redaction ruling (ratified by the owner, 2026-08-01; plan §2.7,
# FREEZE.md Amendment 2): benchmark SELF-REFERENCES in staged docs are
# answer-key and are removed; CAPABILITY documentation stays. Concretely: a
# sentence/clause naming an agentbench task id (A1_/B1_/B2_/B3_/C1_/C2_
# patterns, or the task-shaped "C2 pair"/"C2 fall-through" spellings) or
# naming AgentBench itself is redacted from every staged .md; the sentence
# documenting what --fail-on-runaway does is capability documentation and is
# untouched. Every redaction is recorded in the staging manifest as an exact
# before/after diff. This supersedes the earlier "ships verbatim" known
# disclosure for AGENTS.md §3b.
KNOWN_DISCLOSURES = [
    "AGENTS.md ships with its benchmark self-references REDACTED per the "
    "2026-08-01 answer-key ruling (its sec 3b used to name AgentBench task "
    "C2_fall_through_floor as the --fail-on-runaway motivation). Capability "
    "documentation -- including everything --fail-on-runaway does -- is "
    "preserved; every removed sentence/clause is recorded in this "
    "manifest's `redactions` as an exact before/after diff.",
]

# --- the Webots workspace recipe ---------------------------------------------

WEBOTS_DISTRO = "Ubuntu-22.04"
WEBOTS_WSL_HOME = "/opt/upstream-webots/R2025a"
WEBOTS_UNC = "\\\\wsl$\\%s%s" % (WEBOTS_DISTRO,
                                 WEBOTS_WSL_HOME.replace("/", "\\"))
WEBOTS_LINK_NAME = "webots-R2025a"


# --- helpers ------------------------------------------------------------------


def git_tracked_files(repo=REPO):
    """Every tracked path (posix, relative). The product = what git ships."""
    out = subprocess.run(["git", "-C", str(repo), "ls-files", "-z"],
                         capture_output=True, check=True)
    return [p for p in out.stdout.decode("utf-8", "replace").split("\0") if p]


def select_omnisim_files(file_list,
                         junction_dirs=OMNISIM_JUNCTION_DIRS,
                         exclude_prefixes=OMNISIM_EXCLUDE_PREFIXES,
                         exclude_files=OMNISIM_EXCLUDE_FILES):
    """Split a tracked-file list into (copied, excluded) for the OmniSim arm.

    Pure so the include/exclude rules are testable on a synthetic tree with
    no git and no multi-GB repo.
    """
    junction_prefixes = tuple(d.rstrip("/") + "/" for d in junction_dirs)
    copied, excluded = [], []
    for rel in file_list:
        rel = rel.replace("\\", "/")
        if rel.startswith(junction_prefixes):
            continue                      # reached via the junction instead
        if rel in exclude_files or rel.startswith(exclude_prefixes):
            excluded.append(rel)
            continue
        copied.append(rel)
    return copied, excluded


def filelist_sha256(relpaths):
    """Content hash of the file LIST (sorted relative paths)."""
    blob = "\n".join(sorted(relpaths)).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# --- answer-key redaction (the 2026-08-01 ruling; see KNOWN_DISCLOSURES) ------

# What marks a sentence/clause as a benchmark SELF-REFERENCE. Deliberately
# conservative: bare "B2"/"C1"/"A1" are real robot names (Unitree B2) and real
# baseline-plan item ids all over the product docs, so only the full task-id
# spelling, the word AgentBench itself, and the task-shaped short spellings
# ("the C2 pair", "C2 fall-through") match. Capability docs never trip these.
REDACT_PATTERNS = (
    re.compile(r"\b(?:A1|B1|B2|B3|C1|C2)_[A-Za-z0-9_]+"),
    # \b after "bench" so "agent-benchmarks.md" (the OmniLink agent suite, a
    # different product doc) never trips it; "AgentBench", "agentbench/" do.
    re.compile(r"agent[\s_-]?bench\b", re.IGNORECASE),
    re.compile(r"\b(?:A1|B1|B2|B3|C1|C2)[ -](?:task|pair|cell|row|defect)\b"),
    re.compile(r"\b(?:A1|B1|B2|B3|C1|C2)[ -]fall[- _]through\b"),
)

#: End of sentence: terminal punctuation, optionally closed by markdown
#: emphasis/quote/bracket characters ("means.**", 'said."'), then whitespace.
_SENT_END = re.compile(r"[.!?][\"'*`)\]]*(?=\s)")
_CLAUSE_SPLIT = re.compile(r";\s+")
_PARENTHETICAL = re.compile(r"\s*\([^()]*\)")


def _split_sentences(text):
    """Sentence spans, keeping trailing emphasis/quote marks with their
    sentence (a naive ``(?<=[.!?])\\s`` split severs ``...means.**`` and lets
    a matching neighbour drag a clean sentence down with it)."""
    parts, last = [], 0
    for m in _SENT_END.finditer(text):
        parts.append(text[last:m.end()].strip())
        last = m.end()
    tail = text[last:].strip()
    if tail:
        parts.append(tail)
    return [p for p in parts if p]


def _matches(text):
    return any(p.search(text) for p in REDACT_PATTERNS)


def _drop_matching_parentheticals(sentence):
    """Remove only the (...) groups that carry the match, if that clears it."""
    def _sub(m):
        return "" if _matches(m.group(0)) else m.group(0)
    return _PARENTHETICAL.sub(_sub, sentence)


def _filter_sentence(sentence):
    """The sentence minus its self-reference, or None to drop it whole.

    Escalation: (1) strip only the matching parenthetical(s); (2) strip only
    the matching semicolon clause(s); (3) drop the sentence. Whichever first
    leaves a non-matching remainder wins -- "sentences/clauses" per the
    ruling, never more than the reference itself.
    """
    if not _matches(sentence):
        return sentence
    stripped = _drop_matching_parentheticals(sentence)
    if stripped.strip() and not _matches(stripped):
        return stripped.strip()
    clauses = _CLAUSE_SPLIT.split(sentence)
    if len(clauses) > 1:
        kept = [c for c in clauses if not _matches(c)]
        if kept:
            out = "; ".join(kept).rstrip("; ")
            if not out.rstrip().endswith((".", "!", "?", ":")):
                out = out.rstrip(",;: ") + "."
            return out
    return None


def _block_kind(line):
    ls = line.lstrip()
    if ls.startswith("|"):
        return "table_row"
    if ls.startswith("#"):
        return "heading"
    return "text"


def redact_markdown(text):
    """``(new_text, redactions)`` -- the ruling applied to one document.

    Works block-wise: a markdown table row or heading that self-references is
    dropped whole (a half-row is not markdown); a prose paragraph/bullet is
    re-assembled, filtered sentence-by-sentence (parenthetical -> clause ->
    sentence escalation) and re-wrapped. Untouched blocks are emitted
    byte-identically. Each redaction records the exact before/after.
    """
    lines = text.split("\n")
    out = []
    redactions = []
    block = []            # accumulated prose lines

    def _flush_block():
        if not block:
            return
        raw = "\n".join(block)
        if not _matches(raw):
            out.extend(block)
            block.clear()
            return
        first = block[0]
        indent = first[:len(first) - len(first.lstrip())]
        marker = ""
        body = first.lstrip()
        m = re.match(r"([-*>]\s+|\d+\.\s+)", body)
        if m:
            marker = m.group(1)
        joined = " ".join(ln.strip() for ln in block)
        prose = joined[len(marker):] if marker and joined.startswith(marker) \
            else joined
        kept = []
        for s in _split_sentences(prose):
            f = _filter_sentence(s)
            if f:
                kept.append(f)
        new = " ".join(kept).strip()
        if new:
            wrapped = textwrap.fill(
                new, width=79,
                initial_indent=indent + marker,
                subsequent_indent=indent + " " * len(marker),
                break_long_words=False, break_on_hyphens=False)
            out.extend(wrapped.split("\n"))
            after = wrapped
        else:
            after = ""
        redactions.append({"kind": "paragraph", "before": raw,
                           "after": after})
        block.clear()

    item_start = re.compile(r"\s*(?:[-*>]\s+|\d+\.\s+)")
    for line in lines:
        kind = _block_kind(line)
        if not line.strip():
            _flush_block()
            out.append(line)
            continue
        if kind in ("table_row", "heading"):
            _flush_block()
            if _matches(line):
                redactions.append({"kind": kind, "before": line,
                                   "after": ""})
            else:
                out.append(line)
            continue
        # A new list item / blockquote line starts its OWN block, so a
        # redaction in one bullet can never rewrap (or delete) its siblings.
        if block and item_start.match(line):
            _flush_block()
        block.append(line)
    _flush_block()
    return "\n".join(out), redactions


def redact_staged_docs(tpl: Path):
    """Apply the ruling to every staged ``.md``; returns the manifest list.

    Each entry: ``{"file": <relpath>, "kind", "before", "after"}`` -- exact
    text diffs, published with the campaign so a reviewer can re-derive every
    removal.
    """
    tpl = Path(tpl)
    all_redactions = []
    for p in sorted(tpl.rglob("*.md")):
        if is_link(p) or not p.is_file():
            continue
        try:
            text = p.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        new, reds = redact_markdown(text)
        if reds:
            p.write_text(new, encoding="utf-8")
            rel = p.relative_to(tpl).as_posix()
            for r in reds:
                all_redactions.append({"file": rel, **r})
    return all_redactions


# --- links: create / detect / sever -------------------------------------------


def make_junction(link: Path, target: Path):
    """Directory junction (local volumes; no admin needed)."""
    link = Path(link)
    link.parent.mkdir(parents=True, exist_ok=True)
    try:
        import _winapi
        _winapi.CreateJunction(str(target), str(link))
        return
    except (ImportError, OSError):
        pass
    proc = subprocess.run(["cmd", "/c", "mklink", "/J", str(link),
                           str(target)], capture_output=True, text=True)
    if proc.returncode != 0:
        raise OSError("mklink /J failed for %s -> %s: %s"
                      % (link, target, proc.stderr.strip()
                         or proc.stdout.strip()))


def make_dir_symlink(link: Path, target: str):
    """Directory symlink -- the only link kind that can point at a ``\\\\wsl$``
    UNC path (junctions cannot). Needs Developer Mode or admin; verified
    working on this machine 2026-08-01."""
    link = Path(link)
    link.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(["cmd", "/c", "mklink", "/D", str(link),
                           str(target)], capture_output=True, text=True)
    if proc.returncode != 0:
        raise OSError("mklink /D failed for %s -> %s: %s"
                      % (link, target, proc.stderr.strip()
                         or proc.stdout.strip()))


def is_link(p: Path) -> bool:
    """Junction or symlink (either must be severed, never recursed into)."""
    try:
        return p.is_junction() or p.is_symlink()
    except OSError:
        return False


def sever_links(root: Path):
    """Remove every junction/symlink under ``root`` with ``os.rmdir``/unlink
    semantics -- the link dies, the target is untouched. Returns the severed
    paths. First pass of every teardown."""
    root = Path(root)
    severed = []
    for dirpath, dirnames, filenames in os.walk(root):
        for d in list(dirnames):
            p = Path(dirpath) / d
            if is_link(p):
                os.rmdir(p)               # severs the link only
                severed.append(str(p))
                dirnames.remove(d)        # never walk into (or past) it
        for f in filenames:
            p = Path(dirpath) / f
            if p.is_symlink():
                p.unlink()
                severed.append(str(p))
    return severed


def teardown_workspace(root: Path):
    """Delete a staged workspace WITHOUT ever deleting through a link.

    Two passes: sever every junction/symlink first (rmdir semantics), then
    ``shutil.rmtree`` what remains -- which by then contains plain files only.
    Refuses to touch the repo or anything above it.
    """
    root = Path(root).resolve()
    if not root.exists():
        return []
    repo = REPO.resolve()
    if root == repo or root in repo.parents or repo in root.parents \
            or root == Path(root.anchor) or (root / ".git").exists():
        raise ValueError("refusing to tear down %s: it is (or contains, or "
                         "is inside) the real repo" % root)
    severed = sever_links(root)
    # Belt and braces: verify no link survived before the recursive delete.
    for dirpath, dirnames, _files in os.walk(root):
        for d in dirnames:
            if is_link(Path(dirpath) / d):
                raise RuntimeError("link survived the sever pass: %s"
                                   % (Path(dirpath) / d))
    shutil.rmtree(root)
    # The sibling manifest, if any.
    m = root.parent / (root.name + ".manifest.json")
    try:
        m.unlink()
    except OSError:
        pass
    return severed


# --- resilient teardown + the pending-delete sweep -----------------------------

#: A workspace instance that could not be deleted (a file still held open by
#: some straggler) is renamed to ``<name>.pending_delete`` and reclaimed by
#: :func:`sweep_pending_deletes` at the next campaign start. A locked temp
#: file must never kill a campaign (the phasew_cc_v1 teardown crash,
#: 2026-08-01: a lingering omnisim-bin held omnisim_log.txt open, rmtree hit
#: WinError 32 and the cell's already-earned row was lost).
PENDING_DELETE_SUFFIX = ".pending_delete"
#: When even the rename is refused (Windows refuses to rename a directory
#: while a file under it is open), a sibling marker file records the dir for
#: the same later sweep.
PENDING_MARKER_SUFFIX = ".pending_delete.marker"


def teardown_workspace_resilient(root: Path, *, tries=3, backoff_s=1.0,
                                 sleep=time.sleep):
    """:func:`teardown_workspace` that retries and NEVER raises for a locked
    file.

    Returns ``{"ok", "severed", "attempts", "error", "pending"}``. On final
    failure the links are severed (best effort), the dir is renamed to
    ``*.pending_delete`` (or, when the rename itself is refused, marked with
    a ``*.pending_delete.marker`` sibling) and the caller CONTINUES; the
    campaign-start sweep reclaims it later. The repo-refusal ``ValueError``
    is deliberately NOT swallowed -- that guard must stay fatal.
    """
    root = Path(root)
    last_exc = None
    for attempt in range(1, max(1, int(tries)) + 1):
        try:
            severed = teardown_workspace(root)
            return {"ok": True, "severed": severed, "attempts": attempt,
                    "error": None, "pending": None}
        except (OSError, RuntimeError) as exc:
            last_exc = exc
            if attempt < tries:
                sleep(backoff_s * (2 ** (attempt - 1)))
    # Final failure: make the leftover safe (no live links) and mark it.
    try:
        severed = sever_links(root)
    except OSError:
        severed = []
    pending = root.parent / (root.name + PENDING_DELETE_SUFFIX)
    result = {"ok": False, "severed": severed, "attempts": tries,
              "error": repr(last_exc), "pending": None}
    try:
        os.rename(root, pending)
        result["pending"] = str(pending)
        man = root.parent / (root.name + ".manifest.json")
        if man.is_file():
            try:
                os.rename(man, root.parent
                          / (pending.name + ".manifest.json"))
            except OSError:
                pass
    except OSError as exc:
        # Windows refuses to rename a dir while a file under it is open --
        # fall back to a marker file the sweep also understands.
        marker = root.parent / (root.name + PENDING_MARKER_SUFFIX)
        try:
            marker.write_text(json.dumps({
                "path": str(root), "utc": _utc(),
                "teardown_error": repr(last_exc),
                "rename_error": repr(exc)}), encoding="utf-8")
            result["pending"] = str(marker)
        except OSError as exc2:
            result["error"] += "; marker write failed: %r" % (exc2,)
    return result


def sweep_pending_deletes(root=DEFAULT_ROOT):
    """Reclaim workspaces a previous run could not delete. Campaign start
    calls this; safe to call any time (teardown severs links first, so a
    pending dir is never deleted through a junction).

    Returns ``{"deleted": [...], "failed": [{"path", "error"}]}``.
    """
    root = Path(root)
    deleted, failed = [], []
    candidates = []
    for sub in ("instances", "templates"):
        d = root / sub
        if not d.is_dir():
            continue
        for p in sorted(d.iterdir()):
            if p.is_dir() and p.name.endswith(PENDING_DELETE_SUFFIX):
                candidates.append(p)
            elif p.is_file() and p.name.endswith(PENDING_MARKER_SUFFIX):
                try:
                    target = Path(json.loads(
                        p.read_text(encoding="utf-8"))["path"])
                except (OSError, ValueError, KeyError):
                    target = p.parent / p.name[:-len(PENDING_MARKER_SUFFIX)]
                candidates.append(target)
                candidates.append(p)          # the marker itself, afterwards
    for p in candidates:
        try:
            if p.is_file():
                p.unlink()
            elif p.is_dir():
                teardown_workspace(p)
            elif not p.exists():
                continue
            deleted.append(str(p))
        except (OSError, RuntimeError, ValueError) as exc:
            failed.append({"path": str(p), "error": repr(exc)})
    return {"deleted": deleted, "failed": failed}


# --- process sweep: kill stragglers still referencing an instance ---------------
#
# A headless Claude session may launch the engine/harness inside its
# workspace and exit without killing it (measured 2026-08-01: an omnisim-bin
# lingered after the B1 session, held omnisim_log.txt open, and the teardown
# crash cost the cell its row). Any process whose COMMAND LINE references the
# instance path is ours by construction -- the instance name is a fresh
# timestamped dir nothing else knows about. Only such processes are ever
# touched; there is never a bare image-name kill.


def cmdline_references(cmdline, instance_path):
    """True when ``cmdline`` references the instance (full path either slash
    flavour, or the unique timestamped instance dir name -- which is also how
    a WSL-side process referencing the translated /mnt/... path matches)."""
    if not cmdline:
        return False
    hay = cmdline.lower() if os.name == "nt" else cmdline
    inst = Path(instance_path)
    name = inst.name.lower() if os.name == "nt" else inst.name
    if not name:
        return False
    for cand in (str(inst), str(inst).replace("\\", "/")):
        if os.name == "nt":
            cand = cand.lower()
        if cand and cand in hay:
            return True
    return name in hay


def _iter_processes():
    """Yield ``(pid, name, cmdline)`` for every visible process."""
    try:
        import psutil
    except ImportError:
        psutil = None
    if psutil is not None:
        for p in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                yield (p.info["pid"], p.info["name"] or "",
                       " ".join(p.info["cmdline"] or []))
            except (psutil.Error, TypeError):
                continue
        return
    if os.name == "nt":
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-CimInstance Win32_Process | "
             "Select-Object ProcessId,Name,CommandLine | "
             "ConvertTo-Json -Compress"],
            capture_output=True, text=True, timeout=60)
        try:
            doc = json.loads(out.stdout or "[]")
        except ValueError:
            return
        if isinstance(doc, dict):
            doc = [doc]
        for rec in doc:
            yield (rec.get("ProcessId"), rec.get("Name") or "",
                   rec.get("CommandLine") or "")
    else:
        out = subprocess.run(["ps", "-eo", "pid=,comm=,args="],
                             capture_output=True, text=True, timeout=60)
        for line in (out.stdout or "").splitlines():
            parts = line.split(None, 2)
            if len(parts) >= 2 and parts[0].isdigit():
                yield (int(parts[0]), parts[1],
                       parts[2] if len(parts) > 2 else "")


def _kill_pid(pid, grace_s=1.5):
    """Terminate, then kill. Returns ``(action, detail)``."""
    try:
        import psutil
    except ImportError:
        psutil = None
    if psutil is not None:
        try:
            p = psutil.Process(pid)
            p.terminate()
            try:
                p.wait(timeout=grace_s)
                return "terminated", None
            except psutil.TimeoutExpired:
                p.kill()
                p.wait(timeout=grace_s)
                return "killed", None
        except psutil.NoSuchProcess:
            return "already_gone", None
        except psutil.Error as exc:
            last = repr(exc)
    else:
        last = "psutil unavailable"
    if os.name == "nt":
        out = subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"],
                             capture_output=True, text=True, timeout=60)
        if out.returncode == 0:
            return "killed", None
        return "kill_failed", (out.stderr or out.stdout or last).strip()
    try:
        os.kill(pid, 9)
        return "killed", None
    except ProcessLookupError:
        return "already_gone", None
    except OSError as exc:
        return "kill_failed", repr(exc)


def sweep_workspace_processes(instance_path, *, wsl_distro=None,
                              grace_s=1.5):
    """Terminate every process whose command line references the instance.

    Windows-side: enumerated with psutil (CIM fallback), matched with
    :func:`cmdline_references`, terminated then killed. WSL-side (webots
    arm): ``pgrep -af <instance name>`` inside ``wsl_distro``, then
    ``pkill``, because WSL2 processes are invisible to Windows enumeration.
    Our own process (and parents) are never touched. Returns one record per
    process acted on -- log it into the cell report.
    """
    instance_path = Path(instance_path)
    records = []
    protected = {os.getpid()}
    try:
        import psutil
        p = psutil.Process()
        for anc in p.parents():
            protected.add(anc.pid)
    except Exception:                                     # noqa: BLE001
        pass
    try:
        for pid, name, cmdline in _iter_processes():
            if pid in protected or not cmdline_references(cmdline,
                                                          instance_path):
                continue
            action, detail = _kill_pid(pid, grace_s=grace_s)
            records.append({"where": "host", "pid": pid, "name": name,
                            "cmdline": cmdline[:300], "action": action,
                            "detail": detail})
    except (OSError, subprocess.SubprocessError) as exc:
        records.append({"where": "host", "pid": None, "name": None,
                        "cmdline": None, "action": "enumeration_failed",
                        "detail": repr(exc)})
    if wsl_distro:
        frag = instance_path.name
        try:
            out = subprocess.run(
                ["wsl", "-d", wsl_distro, "--", "pgrep", "-af", frag],
                capture_output=True, text=True, timeout=60)
            victims = [ln.strip() for ln in (out.stdout or "").splitlines()
                       if ln.strip()]
            if victims:
                subprocess.run(
                    ["wsl", "-d", wsl_distro, "--", "pkill", "-TERM",
                     "-f", frag],
                    capture_output=True, text=True, timeout=60)
                time.sleep(grace_s)
                subprocess.run(
                    ["wsl", "-d", wsl_distro, "--", "pkill", "-KILL",
                     "-f", frag],
                    capture_output=True, text=True, timeout=60)
            for v in victims:
                pid_s, _, rest = v.partition(" ")
                records.append({"where": "wsl:%s" % wsl_distro,
                                "pid": int(pid_s) if pid_s.isdigit()
                                else pid_s,
                                "name": None, "cmdline": rest[:300],
                                "action": "killed", "detail": None})
        except (OSError, subprocess.SubprocessError) as exc:
            records.append({"where": "wsl:%s" % wsl_distro, "pid": None,
                            "name": None, "cmdline": None,
                            "action": "enumeration_failed",
                            "detail": repr(exc)})
    return records


# --- port hygiene: reap harness services leaked onto the well-known ports -------
#
# Root cause measured 2026-08-01 (phasew_cc_v1 B2:omnisim r0-r2): a benchmark
# agent (B1:omnisim r4) launched `omnisim_harness.py --port 6789` with
# `cd /o/omnisim` -- ESCAPING its workspace -- so the cmdline-references-the-
# instance sweep above never matched it and the harness outlived the cell.
# Every later B2 agent probed the canonical port first (exactly what the
# staged AGENTS.md teaches), found a live harness serving a FOREIGN world
# (`warehouse_omnilink.omniworld` from the real repo), concluded that was its task
# world, and did all its work there: the staged deliverable was never touched
# and three cells were silently poisoned. The fix is port-scoped, not
# name-scoped: between cells, the cc lane OWNS the default harness/capture
# ports -- ANY listener found on them is cross-cell contamination and is
# reaped, with a full record (pid + cmdline) in the cell report. This is
# deliberately narrow: only processes actually BOUND to these four localhost
# ports (plus their child engines) are ever touched.

#
# ⚠ 2026-08-12 AMENDMENT -- THE SWEEP ABOVE WAS TOO WIDE AND IT DESTROYED A
# CONCURRENT CELL. Measured, `20260812_round1_r4_omnisim_c1`: "between cells
# the cc lane OWNS these ports" is only true when no OTHER cell is running, and
# two lanes on this machine is the supported shape. That cell's `post_session`
# port sweep terminated a live `A1` cell's harness on 6789, both engines (one
# of them named `instances\20260812_102807_omnisim_A1` in its own argv), ten
# `husky_random.py` controllers and three supervisors; the victim's agent then
# spent four tool calls diagnosing a capture-service bug that did not exist.
#
# The rule is now OWNERSHIP, not the port: a listener is reaped when it is
# ours (our own process tree, or its argv names our workspace) or when NO live
# cell claims it (the original leak case, still reaped). A listener claimed by
# another LIVE cell -- `concurrency.active_cells` -- is recorded as
# `skipped_other_cell` and left alone. Lineage is the load-bearing test: the
# victim harness was launched as `python scripts/harness/omnisim_harness.py
# --port 6789` with a relative path, so its argv names no workspace at all and
# only its ancestry identifies it.

DEFAULT_HARNESS_PORTS = (6789, 6790, 6791, 6792)


def process_ancestry(pid, *, depth=12):
    """``[parent, grandparent, ...]`` for ``pid``; ``[]`` when unknowable.

    An empty list is not "no parents" -- it is "we could not tell", and the
    ownership rules treat it that way (they fall back to path matching and,
    failing that, to the unclaimed-leak rule).
    """
    try:
        import psutil
    except ImportError:
        return []
    try:
        return [p.pid for p in psutil.Process(int(pid)).parents()][:depth]
    except Exception:                                     # noqa: BLE001
        return []


def claim_covers(claim, pid, cmdline, ancestry):
    """Does the cell described by ``claim`` own this process?

    Two independent witnesses, either sufficient:

    * **lineage** -- the process descends from the cell's runner pid (this is
      what identifies a harness the agent started with a relative path);
    * **path** -- its command line references the cell's workspace (this is
      what identifies an engine when psutil is unavailable and no ancestry
      can be read).
    """
    if not claim:
        return False
    cpid = claim.get("pid")
    if cpid is not None and (pid == cpid or cpid in (ancestry or ())):
        return True
    ws = claim.get("workspace")
    return bool(ws) and cmdline_references(cmdline or "", ws)


def port_listeners(ports=DEFAULT_HARNESS_PORTS):
    """``[(port, pid)]`` for every listening TCP socket on ``ports``.

    psutil when available; on Windows falls back to ``netstat -ano`` parsing
    (the only fields read are proto/local-address/state/pid)."""
    ports = set(int(p) for p in ports)
    hits = []
    try:
        import psutil
    except ImportError:
        psutil = None
    if psutil is not None:
        try:
            for c in psutil.net_connections(kind="tcp"):
                if (c.status == psutil.CONN_LISTEN and c.laddr
                        and c.laddr.port in ports and c.pid):
                    hits.append((c.laddr.port, c.pid))
            return sorted(set(hits))
        except (psutil.Error, OSError):
            pass
    if os.name == "nt":
        out = subprocess.run(["netstat", "-ano", "-p", "TCP"],
                             capture_output=True, text=True, timeout=60)
        for line in (out.stdout or "").splitlines():
            parts = line.split()
            if len(parts) >= 5 and parts[0].upper() == "TCP" \
                    and parts[3].upper() == "LISTENING":
                try:
                    port = int(parts[1].rsplit(":", 1)[1])
                    pid = int(parts[4])
                except (ValueError, IndexError):
                    continue
                if port in ports and pid > 0:
                    hits.append((port, pid))
    return sorted(set(hits))


def reap_port_listeners(ports=DEFAULT_HARNESS_PORTS, grace_s=1.5, *,
                        mine=None, others=(), listeners=None, processes=None,
                        ancestry=None, kill=None):
    """Reap the harness/capture ports -- **but never another live cell's**.

    ``mine`` is this cell's claim (``concurrency.register_cell``'s payload) and
    ``others`` the claims of every OTHER live cell. A listener is killed when
    it is ours, or when no live claim covers it (the leaked-harness case the
    sweep was written for); a listener covered by another live claim is
    recorded ``skipped_other_cell`` -- naming the owner -- and left running.

    With no claims passed the behaviour is the pre-2026-08-12 one (reap every
    listener), so a standalone cell on an idle machine is unchanged.

    ``listeners`` / ``processes`` / ``ancestry`` / ``kill`` are injection
    points for the unit tests; nothing else passes them. Returns one record
    per process acted on -- an empty list is the clean-ports witness.
    """
    records = []
    protected = {os.getpid()}
    try:
        import psutil
        for anc in psutil.Process().parents():
            protected.add(anc.pid)
    except Exception:                                     # noqa: BLE001
        psutil = None
    else:
        pass
    cmdlines = dict(processes or {})
    if processes is None:
        try:
            for pid, name, cmdline in _iter_processes():
                cmdlines[pid] = (name, cmdline)
        except (OSError, subprocess.SubprocessError):
            pass
    kill = kill or _kill_pid
    hits = port_listeners(ports) if listeners is None else list(listeners)

    def _ancestry(pid):
        if ancestry is not None:
            return ancestry.get(pid, [])
        return process_ancestry(pid)

    for port, pid in hits:
        name, cmdline = cmdlines.get(pid, ("", ""))
        if pid in protected:
            records.append({"where": "port:%d" % port, "pid": pid,
                            "name": name, "cmdline": cmdline[:300],
                            "action": "skipped_protected", "detail": None})
            continue
        anc = _ancestry(pid)
        owner = next((c for c in (others or ())
                      if claim_covers(c, pid, cmdline, anc)), None)
        if owner is not None and not claim_covers(mine, pid, cmdline, anc):
            records.append({"where": "port:%d" % port, "pid": pid,
                            "name": name, "cmdline": cmdline[:300],
                            "action": "skipped_other_cell",
                            "owner": {k: owner.get(k) for k in
                                      ("pid", "lane", "sim", "task",
                                       "workspace")},
                            "detail": ("a live cell owns this listener; "
                                       "reaping it would destroy its run")})
            continue
        # children first (the engine the harness spawned), then the listener
        kids = []
        try:
            import psutil
            kids = [c.pid for c in
                    psutil.Process(pid).children(recursive=True)]
        except Exception:                                 # noqa: BLE001
            pass
        action, detail = kill(pid, grace_s=grace_s)
        records.append({"where": "port:%d" % port, "pid": pid, "name": name,
                        "cmdline": cmdline[:300], "action": action,
                        "detail": detail})
        for kpid in kids:
            if kpid in protected:
                continue
            kname, kcmd = cmdlines.get(kpid, ("", ""))
            kaction, kdetail = kill(kpid, grace_s=grace_s)
            records.append({"where": "port:%d:child" % port, "pid": kpid,
                            "name": kname, "cmdline": kcmd[:300],
                            "action": kaction, "detail": kdetail})
    return records


# --- junction-artifact hygiene: reclaim session writes through projects/ --------
#
# The omnisim workspace template junctions ``projects/`` into the REAL repo,
# so a session that saves its deliverable under ``projects/`` writes into the
# repo itself -- and the next same-task cell then sees the predecessor's
# answer through its own junction (measured 2026-08-01, phasew_cc_v1 A1
# r1/r3/r6: r1 authored NO world of its own, "verified" r0's
# husky_random_x10.wbt through the shared junction, and stayed blocked at
# re-adjudication -- see the cells' recovered/RECOVERY.json). Between cells
# the repo's projects/ tree must therefore hold no session-authored files.
# The sweep is deliberately narrow -- refusal is the default:
#
# * only files git neither tracks nor ignores (two-witness check against
#   ``git ls-files`` and ``git ls-files --others --exclude-standard``;
#   tracked files are NEVER touched, and gitignored build scratch is not
#   session evidence);
# * only benchmark-deliverable shapes: ``*.wbt``, ``*.py``, or any file
#   inside a ``controllers/`` dir;
# * with a ``window``, only mtimes STRICTLY inside it (the cell's session);
# * every file is preserved (``copy2`` + sha256) into ``dest_dir`` BEFORE
#   its repo copy is deleted; a refused delete (open handle) keeps the
#   preserved copy and is recorded, never raised past the caller;
# * directories are never recursively deleted -- only now-empty parents are
#   ``os.rmdir``'d, which by construction cannot remove a dir that still
#   holds a tracked (or any other) file.
#
# Residual risk, recorded rather than pretended away: the check-then-delete
# window is not atomic, so a CONCURRENT writer into the repo's projects/
# tree (a second omnisim lane -- the supported lane split is A=omnisim,
# B=webots, which never writes here) could in principle race the mtime gate.

REPO_SWEEP_SUBDIR = "projects"
REPO_SWEEP_SUFFIXES = (".omniworld", ".wbt", ".py")


def _git_z(repo, args):
    out = subprocess.run(["git", "-C", str(repo)] + args,
                         capture_output=True, check=True)
    return [p for p in out.stdout.decode("utf-8", "replace").split("\0") if p]


def repo_sweep_candidates(repo=REPO, subdir=REPO_SWEEP_SUBDIR):
    """``(untracked, tracked)`` posix-relative path sets under ``subdir``.

    ``untracked`` is ``git ls-files --others --exclude-standard`` (so
    gitignored files never appear) minus anything tracked -- the two-witness
    verification the sweep's never-touch-tracked rail rests on.
    """
    spec = subdir.rstrip("/") + "/"
    tracked = set(_git_z(repo, ["ls-files", "-z", "--", spec]))
    others = set(_git_z(repo, ["ls-files", "-z", "--others",
                               "--exclude-standard", "--", spec]))
    return others - tracked, tracked


def _sweep_eligible(rel, suffixes=REPO_SWEEP_SUFFIXES):
    rel = rel.replace("\\", "/")
    return rel.endswith(tuple(suffixes)) or "/controllers/" in rel


def sweep_repo_junction_artifacts(dest_dir, *, window=None, repo=REPO,
                                  subdir=REPO_SWEEP_SUBDIR,
                                  suffixes=REPO_SWEEP_SUFFIXES,
                                  protect_after_ts=None):
    """Preserve-then-delete session-shaped untracked files under the real
    repo's ``subdir`` tree (see the section comment for the full rails).

    ``window=(start_ts, end_ts)`` keeps only files whose mtime is STRICTLY
    inside it (the post-session evidence sweep); ``window=None`` takes every
    eligible untracked file (the pre-session quarantine sweep -- leftovers
    from crashed prior cells have arbitrary mtimes, and quarantine is a
    move, so nothing is destroyed). Returns one record per eligible
    candidate (and per pruned empty dir); an empty list is the clean-tree
    witness. Never deletes a file it failed to preserve.

    ``protect_after_ts`` is the 2026-08-12 rail and it is not optional in a
    two-lane campaign. "Quarantine is a move, so nothing is destroyed" was
    only true for a FINISHED cell's leftovers. MEASURED that day: with
    ``window=None`` this sweep `preserved_and_deleted`
    ``projects/samples/demos/worlds/showcase/husky_random_10.wbt`` -- a
    CONCURRENTLY RUNNING cell's deliverable, which it was still iterating on
    -- into the sweeping cell's own directory. Passing the oldest live cell's
    start time (``concurrency.oldest_active_start``) makes any file newer than
    it untouchable: it may still be being written, and a file that is somebody
    else's live work is not a leftover.
    """
    repo = Path(repo)
    dest_dir = Path(dest_dir)
    sub_root = (repo / subdir).resolve()
    d = dest_dir.resolve()
    if d == sub_root or sub_root in d.parents:
        raise ValueError("dest_dir %s must live outside the swept tree %s"
                         % (dest_dir, sub_root))
    try:
        untracked, _tracked = repo_sweep_candidates(repo, subdir)
    except (OSError, subprocess.SubprocessError) as exc:
        return [{"rel": None, "action": "enumeration_failed",
                 "detail": repr(exc)}]
    records = []
    deleted_parents = []
    for rel in sorted(untracked):
        if not _sweep_eligible(rel, suffixes):
            continue                      # not deliverable-shaped: untouched
        src = repo / rel
        try:
            st = src.stat()
        except OSError as exc:
            records.append({"rel": rel, "action": "stat_failed",
                            "detail": repr(exc)})
            continue
        if window is not None:
            t0, t1 = window
            if not (t0 < st.st_mtime < t1):
                records.append({"rel": rel, "mtime": st.st_mtime,
                                "action": "skipped_outside_window",
                                "detail": None})
                continue
        if protect_after_ts is not None and st.st_mtime >= protect_after_ts:
            records.append({"rel": rel, "mtime": st.st_mtime,
                            "action": "skipped_owned_by_active_cell",
                            "protect_after_ts": protect_after_ts,
                            "detail": ("written after the oldest live cell "
                                       "started; it may be that cell's live "
                                       "deliverable, so it is left alone")})
            continue
        dst = dest_dir / rel
        try:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            digest = hashlib.sha256(dst.read_bytes()).hexdigest()
        except OSError as exc:
            records.append({"rel": rel, "mtime": st.st_mtime,
                            "action": "preserve_failed",
                            "detail": repr(exc)})
            continue                      # never delete what isn't preserved
        try:
            os.remove(src)
            action, detail = "preserved_and_deleted", None
            deleted_parents.append(src.parent)
        except OSError as exc:
            action, detail = "preserved_delete_failed", repr(exc)
        records.append({"rel": rel, "repo_path": str(src),
                        "preserved_to": str(dst), "sha256": digest,
                        "mtime": st.st_mtime, "action": action,
                        "detail": detail})
    # Prune now-empty dirs the deletions left behind (deepest first), never
    # past the swept subtree root. os.rmdir refuses a non-empty dir, so a
    # dir still holding a tracked -- or any -- file survives by construction.
    for parent in sorted(set(deleted_parents),
                         key=lambda p: -len(str(p))):
        cur = Path(parent)
        while True:
            try:
                rcur = cur.resolve()
            except OSError:
                break
            if rcur == sub_root or sub_root not in rcur.parents:
                break
            try:
                os.rmdir(cur)
            except OSError:
                break
            records.append({"rel": str(rcur), "action": "removed_empty_dir",
                            "detail": None})
            cur = cur.parent
    return records


# --- template build -------------------------------------------------------------


def omnisim_template_reusable(tpl, manifest_path, copied, *,
                              exclude_prefixes=OMNISIM_EXCLUDE_PREFIXES,
                              exclude_files=OMNISIM_EXCLUDE_FILES):
    """``(bool, why_not)`` -- may this cached template be handed to a cell?

    "The directory exists and has a manifest" was the whole rule, and it is
    not enough in either direction.

    MEASURED 2026-08-12. The cached template was built 2026-08-01, its
    manifest recorded 4799 files, and the directory on disk held **255** --
    zero top-level files (no ``AGENTS.md``, no ``README.md``, no ``Makefile``)
    and a ``src/omnisim/nodes/`` still on the pre-rename ``Wb*`` names. Every
    cell staged from it in between handed its agent a nearly empty tree
    missing the product's own entry point, and nothing said so: the manifest
    described the build, and the build had since been partly swept.

    So the cache is checked against what it CLAIMS and against what the
    product IS:

    * the tracked-file selection's sha256 -- the repo moves, and a template
      built eleven days ago is not this product;
    * the exclusion lists -- a needle added to the answer key must invalidate
      every template built before it, or the next cell is staged with the leak
      the list was extended to close;
    * the file count on disk vs the count the manifest recorded -- the failure
      above, which no amount of sha checking would have caught because the
      manifest was honest about a tree that no longer existed.
    """
    try:
        m = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return False, "the manifest is unreadable (%s)" % exc.__class__.__name__
    want = filelist_sha256(copied)
    if m.get("filelist_sha256") != want:
        return False, ("the repo's tracked-file selection has changed since "
                       "it was built at %s (%s != %s)"
                       % (m.get("utc"), m.get("filelist_sha256"), want))
    if list(m.get("exclude_prefixes") or ()) != list(exclude_prefixes) or \
            list(m.get("exclude_files") or ()) != list(exclude_files):
        return False, ("the answer-key exclusion list has changed since it "
                       "was built at %s" % m.get("utc"))
    on_disk = sum(1 for p in Path(tpl).rglob("*") if p.is_file())
    claimed = int(m.get("included_file_count") or 0)
    if on_disk < claimed:
        return False, ("the template on disk holds %d files and its manifest "
                       "records %d -- it has been partly deleted since it was "
                       "built at %s" % (on_disk, claimed, m.get("utc")))
    return True, None


def build_omnisim_template(root=DEFAULT_ROOT, *, repo=REPO, file_list=None,
                           junction_dirs=OMNISIM_JUNCTION_DIRS,
                           exclude_prefixes=OMNISIM_EXCLUDE_PREFIXES,
                           exclude_files=OMNISIM_EXCLUDE_FILES,
                           force=False):
    """Build (or reuse) the OmniSim workspace template. Returns its path.

    The template holds plain copied files only; the junctions are recorded in
    the manifest and materialised per instance by :func:`instantiate`.
    """
    root = Path(root)
    tpl = root / "templates" / "omnisim"
    manifest_path = root / "templates" / "omnisim.manifest.json"

    files = file_list if file_list is not None else git_tracked_files(repo)
    copied, excluded = select_omnisim_files(
        files, junction_dirs=junction_dirs,
        exclude_prefixes=exclude_prefixes, exclude_files=exclude_files)

    if tpl.exists():
        reusable, why = omnisim_template_reusable(
            tpl, manifest_path, copied,
            exclude_prefixes=exclude_prefixes, exclude_files=exclude_files)
        if reusable and not force:
            return tpl
        if not reusable:
            print("[stage] rebuilding the omnisim template: %s" % why)
        teardown_workspace(tpl)
    tpl.mkdir(parents=True, exist_ok=True)

    missing = []
    for rel in copied:
        src = Path(repo) / rel
        dst = tpl / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        try:
            shutil.copy2(src, dst)
        except OSError:
            missing.append(rel)           # tracked but absent on disk
    copied = [r for r in copied if r not in set(missing)]

    junctions = [{"name": d, "target": str(Path(repo) / d),
                  "kind": "junction"} for d in junction_dirs
                 if (Path(repo) / d).is_dir()]

    # The answer-key redaction pass (the 2026-08-01 ruling): benchmark
    # self-references out of every staged .md, capability docs untouched,
    # every removal recorded below as an exact before/after diff.
    redactions = redact_staged_docs(tpl)

    manifest = {
        "sim": "omnisim",
        "repo": str(repo),
        "utc": _utc(),
        "included_file_count": len(copied),
        "filelist_sha256": filelist_sha256(copied),
        "redaction_ruling": (
            "benchmark SELF-REFERENCES in staged docs are answer-key and "
            "are removed (sentences/clauses naming agentbench task ids or "
            "AgentBench itself); CAPABILITY documentation stays (e.g. the "
            "--fail-on-runaway documentation). Ratified by the owner "
            "2026-08-01; plan §2.7 / FREEZE.md Amendment 2."),
        "redactions": redactions,
        "junctions": junctions,
        "junction_note": ("junctioned dirs are runtime/product material with "
                          "no answer-key files; a session inside the "
                          "workspace CAN write through a junction into the "
                          "real tree -- reviewed as a residual risk, not "
                          "eliminated"),
        "excluded": sorted(excluded),
        "exclude_prefixes": list(exclude_prefixes),
        "exclude_files": list(exclude_files),
        "tracked_but_missing_on_disk": sorted(missing),
        "known_disclosures": list(KNOWN_DISCLOSURES),
        "source": "git ls-files (tracked files only; untracked scratch can "
                  "never leak into the workspace)",
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2),
                             encoding="utf-8")
    return tpl


def build_webots_template(root=DEFAULT_ROOT, *, unc=WEBOTS_UNC,
                          wsl_home=WEBOTS_WSL_HOME, distro=WEBOTS_DISTRO,
                          force=False):
    """Build (or reuse) the Webots workspace template. Returns its path.

    The template is deliberately almost empty: the upstream install is
    exposed per instance as a directory symlink (recorded in the manifest),
    upstream's own docs live inside that install tree, and no file authored
    by us enters the workspace beyond the task files staged later.
    """
    root = Path(root)
    tpl = root / "templates" / "webots"
    manifest_path = root / "templates" / "webots.manifest.json"
    if tpl.exists():
        if not force and manifest_path.is_file():
            return tpl
        teardown_workspace(tpl)
    tpl.mkdir(parents=True, exist_ok=True)

    manifest = {
        "sim": "webots",
        "utc": _utc(),
        "included_file_count": 0,
        "redaction_ruling": ("nothing authored by us is staged, so there is "
                             "nothing to redact (the ruling applies to "
                             "staged docs; upstream's own docs are the "
                             "product and are never touched)"),
        "redactions": [],
        "links": [{"name": WEBOTS_LINK_NAME, "target": unc,
                   "kind": "dir_symlink",
                   "wsl_path": wsl_home, "distro": distro}],
        "note": ("upstream R2025a exposed via a directory symlink; the "
                 "install's own docs/ ship inside it. Nothing authored by "
                 "us is staged beyond the task files (a README from us "
                 "would contaminate the product comparison)."),
        "known_disclosures": [],
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2),
                             encoding="utf-8")
    return tpl


def build_mujoco_template(root=DEFAULT_ROOT, *, force=False):
    """Build (or reuse) the MuJoCo workspace template. Returns its path.

    Empty, and for a reason that is not laziness: on this arm the *product* is
    a Python package inside the cell's interpreter (``mujoco`` 3.8.1, resolved
    by ``$AGENTBENCH_MUJOCO_PYTHON`` or the parent's own ``sys.executable``),
    not a tree on disk. There is no install to expose, no docs directory to
    symlink and nothing authored by us to stage -- so there is nothing to
    redact either, and a README from us would contaminate the comparison
    exactly as it would on the Webots arm.

    The agent therefore starts in a bare directory holding only the task's own
    staged files, which is what ``adapters/mujoco/BRINGUP.md`` §5 means when it
    says the three expressible tasks "stage correctly for this arm with no
    task-side change at all": they begin from an empty or world-free
    ``initial/``.
    """
    root = Path(root)
    tpl = root / "templates" / "mujoco"
    manifest_path = root / "templates" / "mujoco.manifest.json"
    if tpl.exists():
        if not force and manifest_path.is_file():
            return tpl
        teardown_workspace(tpl)
    tpl.mkdir(parents=True, exist_ok=True)

    manifest = {
        "sim": "mujoco",
        "utc": _utc(),
        "included_file_count": 0,
        "redaction_ruling": ("nothing authored by us is staged, so there is "
                             "nothing to redact (the ruling applies to "
                             "staged docs; MuJoCo's own documentation is "
                             "published by its vendor and is not in the "
                             "workspace at all)"),
        "redactions": [],
        "junctions": [],
        "links": [],
        "note": ("the MuJoCo 'install' is the `mujoco` package in the cell's "
                 "interpreter, not a tree on disk -- so the workspace holds "
                 "only the task's staged files. The deliverable convention "
                 "for this arm is a PAIR: an MJCF .xml and the .py program "
                 "that steps it (agents/external.ARTIFACT_NAME_BY_SIM)."),
        "known_disclosures": [],
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2),
                             encoding="utf-8")
    return tpl


# --- instantiation ----------------------------------------------------------------


def _copy_tree_no_links(src: Path, dst: Path):
    """Copy a template tree that is guaranteed link-free; refuse links."""
    for dirpath, dirnames, filenames in os.walk(src):
        rel = Path(dirpath).relative_to(src)
        for d in dirnames:
            if is_link(Path(dirpath) / d):
                raise RuntimeError("template contains a link (%s) -- "
                                   "templates must be plain files only"
                                   % (Path(dirpath) / d))
        (dst / rel).mkdir(parents=True, exist_ok=True)
        for f in filenames:
            shutil.copy2(Path(dirpath) / f, dst / rel / f)


def instantiate(template: Path, dest: Path):
    """A fresh per-cell workspace instance from a template: copy the plain
    files, then materialise the links the manifest records. Returns the
    instance manifest (also written beside the instance)."""
    template = Path(template)
    dest = Path(dest)
    if dest.exists():
        raise FileExistsError("instance dir already exists: %s (cells never "
                              "share state; tear it down first)" % dest)
    tpl_manifest = json.loads(
        (template.parent / (template.name + ".manifest.json"))
        .read_text(encoding="utf-8"))
    dest.mkdir(parents=True)
    _copy_tree_no_links(template, dest)

    made = []
    for j in tpl_manifest.get("junctions", []):
        make_junction(dest / j["name"], Path(j["target"]))
        made.append({**j, "path": str(dest / j["name"])})
    for l in tpl_manifest.get("links", []):
        make_dir_symlink(dest / l["name"], l["target"])
        made.append({**l, "path": str(dest / l["name"])})

    inst_manifest = dict(tpl_manifest)
    inst_manifest.update({
        "instance": str(dest),
        "template": str(template),
        "instantiated_utc": _utc(),
        "links_materialised": made,
    })
    (dest.parent / (dest.name + ".manifest.json")).write_text(
        json.dumps(inst_manifest, indent=2), encoding="utf-8")
    return inst_manifest


# --- task staging -----------------------------------------------------------------


def task_initial_dir(task_id: str, sim: str, *,
                     tasks_dir=AGENTBENCH / "tasks"):
    """The per-arm ``initial`` tree a task publishes into the workspace.

    One rule, one place. ``initial_webots`` on the upstream arm (its fixtures
    are authored against upstream's node set), ``initial`` everywhere else.
    """
    return (Path(tasks_dir) / task_id
            / ("initial_webots" if sim == "webots" else "initial"))


def published_task_files(task_id: str, sim: str, *,
                         tasks_dir=AGENTBENCH / "tasks"):
    """``[(relative posix path, absolute source path)]`` -- EXACTLY the files
    this lane plants in the agent's workspace for ``(task_id, sim)``.

    Factored out of :func:`stage_task` rather than restated, because a second
    consumer arrived: the graded project has to be able to re-materialise the
    same set at the same relative paths (``run_cc_cell.stage_task_assets``),
    and two copies of "what did we publish" is exactly how a graded project
    ends up missing a file the agent was handed. The source is always the
    frozen task tree in the repo -- never a workspace -- which is what makes
    the grade-time copy safe (see ``stage_task_assets``).

    Sorted, so callers get a stable, reproducible order.
    """
    src = task_initial_dir(task_id, sim, tasks_dir=tasks_dir)
    sys.path.insert(0, str(AGENTBENCH.parent))
    from agentbench.common.worldtext import is_harness_sibling
    out = []
    if not src.is_dir():
        return out
    for p in sorted(src.rglob("*")):
        if p.is_dir() or p.name == ".gitkeep" or is_harness_sibling(p.name):
            continue
        out.append((p.relative_to(src).as_posix(), p))
    return out


def stage_task(workspace: Path, task_id: str, sim: str, *,
               tasks_dir=AGENTBENCH / "tasks"):
    """Copy the task's initial world(s) into the workspace root and return
    ``(prompt_text, [staged paths])``.

    The prompt is returned, NOT planted as a file: it is passed to
    ``claude -p`` verbatim (identical on both arms), and a prompt file in the
    workspace would be a second, non-identical delivery channel.
    """
    task_dir = Path(tasks_dir) / task_id
    prompt = (task_dir / "prompt.txt").read_text(encoding="utf-8").strip()
    staged = []
    for rel, p in published_task_files(task_id, sim, tasks_dir=tasks_dir):
        dst = Path(workspace) / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dst)
        staged.append(str(dst))
    return prompt, staged


# --- CLI --------------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sim", choices=("omnisim", "webots", "mujoco", "both"),
                    default="both")
    ap.add_argument("--root", default=str(DEFAULT_ROOT),
                    help="scratch root for templates/instances (never inside "
                         "the repo); default %(default)s")
    ap.add_argument("--force", action="store_true",
                    help="rebuild the template even if one exists")
    args = ap.parse_args(argv)

    root = Path(args.root).resolve()
    if REPO.resolve() in root.parents or root == REPO.resolve():
        print("refusing: --root must be outside the repo", file=sys.stderr)
        return 2

    sims = ("omnisim", "webots") if args.sim == "both" else (args.sim,)
    for sim in sims:
        if sim == "omnisim":
            tpl = build_omnisim_template(root, force=args.force)
        elif sim == "mujoco":
            tpl = build_mujoco_template(root, force=args.force)
        else:
            tpl = build_webots_template(root, force=args.force)
        man = json.loads((tpl.parent / (tpl.name + ".manifest.json"))
                         .read_text(encoding="utf-8"))
        print("%s template -> %s" % (sim, tpl))
        print("  files copied : %s" % man.get("included_file_count"))
        print("  junctions    : %s" % [j["name"] for j in
                                       man.get("junctions", [])])
        print("  links        : %s" % [l["name"] for l in
                                       man.get("links", [])])
        print("  excluded     : %d paths (answer key + private trees)"
              % len(man.get("excluded", [])))
        reds = man.get("redactions", [])
        print("  redactions   : %d (in %d file(s)) -- exact before/after "
              "diffs in the manifest"
              % (len(reds), len({r["file"] for r in reds})))
    return 0


if __name__ == "__main__":
    sys.exit(main())
