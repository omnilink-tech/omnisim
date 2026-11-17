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

"""Concurrency protocol for the Claude Code lane (plan §2.7, RUNBOOK §7).

The rules this module makes mechanical rather than aspirational:

* **Max N engine lanes on one machine** (default 2 — lane A drives omnisim
  cells, lane B webots cells): a *file-lock semaphore* of N slot files under
  a shared lock root. A lane acquires a slot around its **engine-heavy
  phases** — for an omnisim cell that is the whole headless Claude session
  (the agent may launch the engine at any point) plus the grading pass; for a
  webots cell the grading/recorder pass only. The grading pass **always**
  holds a slot.
* **Same-task cells never overlap** (plan §5.3: `husky_random` traces land in
  a hardcoded global dir; two concurrent same-task recorders corrupt each
  other's evidence): a per-task exclusive lock held for the whole cell.
* **Attribution over prevention for the rest**: while a lane holds a slot it
  can *see* whether any other slot is held by a live process. A row produced
  while another lane was active is flagged
  ``measured_under_concurrency: true`` — the report side then excludes that
  row's time columns from any latency statement (plan §2.3 note). Tool
  calls, tokens and verdicts are contention-insensitive and stay.
* **A usage/rate limit is not a failed run**: :func:`rate_limit_reason`
  recognises the limit-shaped errors so the worker can record the attempt as
  *deferred*, wait, and retry the same cell without burning its one scored
  session.

Lock mechanics: a slot/lock is a file created with ``O_CREAT | O_EXCL``
holding ``{"pid", "lane", "purpose", "utc"}``. Liveness is checked with the
signal-0 probe; a lock whose pid is dead is stale and reclaimed. This is
advisory locking by agreement between the lanes on this machine — exactly the
scope the protocol needs, and it needs no daemon and no extra dependency.
"""

from __future__ import annotations

import ctypes
import json
import os
import re
import subprocess
import time
from pathlib import Path

DEFAULT_ENGINE_SLOTS = 2
MIN_FREE_RAM_GB = 4.0
MAX_CPU_LOAD_PCT = 85.0

_ENGINE_PREFIX = "engine_slot_"
_TASK_PREFIX = "task_"


def _pid_alive(pid):
    """Best-effort liveness. Unknown (permission etc.) counts as alive.

    ⚠ Never ``os.kill(pid, 0)`` on Windows — CPython implements non-console
    signals there with ``TerminateProcess``, so the "probe" would KILL the
    other lane. Signal-0 is POSIX-only; Windows gets an OpenProcess probe.
    """
    if not isinstance(pid, int) or pid <= 0:
        return False
    if os.name == "nt":
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        ERROR_INVALID_PARAMETER = 87
        h = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION,
                                 False, pid)
        if not h:
            # invalid-parameter -> no such pid; access-denied etc. -> assume
            # alive rather than steal a live lane's lock.
            return ctypes.get_last_error() != ERROR_INVALID_PARAMETER
        code = ctypes.c_ulong()
        ok = kernel32.GetExitCodeProcess(h, ctypes.byref(code))
        kernel32.CloseHandle(h)
        STILL_ACTIVE = 259
        return bool(ok) and code.value == STILL_ACTIVE
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except OSError:
        return True


class FileLock:
    """One exclusive advisory lock file (create-exclusive semantics)."""

    def __init__(self, path: Path, *, lane="", purpose=""):
        self.path = Path(path)
        self.lane = lane
        self.purpose = purpose
        self.held = False

    def try_acquire(self):
        if self.held:
            return True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._reclaim_if_stale()
        try:
            fd = os.open(str(self.path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        except FileExistsError:
            return False
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump({"pid": os.getpid(), "lane": self.lane,
                       "purpose": self.purpose,
                       "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                            time.gmtime())}, fh)
        self.held = True
        return True

    def acquire(self, *, poll_s=2.0, timeout_s=None, on_wait=None):
        t0 = time.monotonic()
        waited = False
        while not self.try_acquire():
            if timeout_s is not None and time.monotonic() - t0 > timeout_s:
                raise TimeoutError("lock %s not acquired within %ss"
                                   % (self.path.name, timeout_s))
            if on_wait and not waited:
                on_wait(self.path)
                waited = True
            time.sleep(poll_s)
        return self

    def release(self):
        if not self.held:
            return
        self.held = False
        try:
            self.path.unlink()
        except OSError:
            pass

    def _reclaim_if_stale(self):
        info = read_lock(self.path)
        if info is None:
            return
        if not _pid_alive(info.get("pid")):
            try:
                self.path.unlink()
            except OSError:
                pass

    def __enter__(self):
        return self.acquire()

    def __exit__(self, *exc):
        self.release()


def read_lock(path: Path):
    """The lock file's payload, or None (absent / unreadable / malformed)."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


class EngineSlots:
    """The N-slot engine semaphore (max concurrent engine-heavy phases)."""

    def __init__(self, lock_root: Path, *, slots=DEFAULT_ENGINE_SLOTS,
                 lane=""):
        self.root = Path(lock_root)
        self.slots = max(1, int(slots))
        self.lane = lane
        self._held = None            # the FileLock currently held, if any

    def _slot_paths(self):
        return [self.root / ("%s%d.lock" % (_ENGINE_PREFIX, i))
                for i in range(self.slots)]

    def try_acquire(self, purpose=""):
        if self._held is not None:
            return True
        for p in self._slot_paths():
            lock = FileLock(p, lane=self.lane, purpose=purpose)
            if lock.try_acquire():
                self._held = lock
                return True
        return False

    def acquire(self, purpose="", *, poll_s=2.0, timeout_s=None,
                on_wait=None):
        t0 = time.monotonic()
        waited = False
        while not self.try_acquire(purpose):
            if timeout_s is not None and time.monotonic() - t0 > timeout_s:
                raise TimeoutError("no engine slot free within %ss"
                                   % timeout_s)
            if on_wait and not waited:
                on_wait(self.root)
                waited = True
            time.sleep(poll_s)
        return self

    def release(self):
        if self._held is not None:
            self._held.release()
            self._held = None

    def others_active(self):
        """True when any *other* live lane holds an engine slot right now.

        This is the ``measured_under_concurrency`` witness: sampled while a
        cell runs, any True sample flags the row (plan §2.7).
        """
        me = self._held.path if self._held is not None else None
        for p in self._slot_paths():
            if me is not None and p == me:
                continue
            info = read_lock(p)
            if info and info.get("pid") != os.getpid() \
                    and _pid_alive(info.get("pid")):
                return True
        return False

    def __enter__(self):
        return self.acquire()

    def __exit__(self, *exc):
        self.release()


def task_lock(lock_root: Path, task_id: str, *, lane=""):
    """The same-task exclusion lock (held for the WHOLE cell)."""
    safe = "".join(c if c.isalnum() or c in "._-" else "_" for c in task_id)
    return FileLock(Path(lock_root) / ("%s%s.lock" % (_TASK_PREFIX, safe)),
                    lane=lane, purpose="same-task exclusion")


# --- the pre-cell resource guard ---------------------------------------------


def _free_ram_cpu_psutil():
    import psutil
    free_gb = psutil.virtual_memory().available / (1024.0 ** 3)
    cpu_pct = psutil.cpu_percent(interval=1.0)
    return free_gb, cpu_pct, "psutil"


def _free_ram_cpu_windows():
    """Cheap fallback: wmic (no psutil, no heavy deps)."""
    out = subprocess.run(
        ["wmic", "OS", "get", "FreePhysicalMemory", "/value"],
        capture_output=True, text=True, timeout=30)
    free_kb = None
    for line in (out.stdout or "").splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            if k.strip() == "FreePhysicalMemory" and v.strip().isdigit():
                free_kb = int(v.strip())
    out2 = subprocess.run(
        ["wmic", "cpu", "get", "LoadPercentage", "/value"],
        capture_output=True, text=True, timeout=30)
    loads = []
    for line in (out2.stdout or "").splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            if k.strip() == "LoadPercentage" and v.strip().isdigit():
                loads.append(int(v.strip()))
    if free_kb is None:
        raise RuntimeError("wmic reported no FreePhysicalMemory")
    cpu = (sum(loads) / len(loads)) if loads else None
    return free_kb / (1024.0 ** 2), cpu, "wmic"


def _free_ram_cpu_posix():
    free_kb = None
    with open("/proc/meminfo", encoding="ascii", errors="replace") as fh:
        for line in fh:
            if line.startswith("MemAvailable:"):
                free_kb = int(line.split()[1])
                break
    load1 = os.getloadavg()[0]
    ncpu = os.cpu_count() or 1
    cpu_pct = 100.0 * load1 / ncpu
    if free_kb is None:
        raise RuntimeError("/proc/meminfo has no MemAvailable")
    return free_kb / (1024.0 ** 2), cpu_pct, "/proc+loadavg"


def resource_guard(*, min_free_ram_gb=MIN_FREE_RAM_GB,
                   max_cpu_load_pct=MAX_CPU_LOAD_PCT):
    """``(ok, detail)`` — may this lane start an engine-heavy cell now?

    psutil when importable; a cheap wmic (Windows) / procfs (POSIX) fallback
    otherwise — deliberately no new hard dependency. A guard that cannot
    measure says so and does NOT block (an unmeasurable guard silently
    blocking every cell would be worse than no guard), but the detail is
    recorded so the gap is loud.
    """
    detail = {"min_free_ram_gb": min_free_ram_gb,
              "max_cpu_load_pct": max_cpu_load_pct}
    for probe in (_free_ram_cpu_psutil,
                  _free_ram_cpu_windows if os.name == "nt"
                  else _free_ram_cpu_posix):
        try:
            free_gb, cpu_pct, source = probe()
        except Exception as exc:                          # noqa: BLE001
            detail.setdefault("probe_errors", []).append(
                "%s: %r" % (getattr(probe, "__name__", "probe"), exc))
            continue
        detail.update(free_ram_gb=round(free_gb, 2),
                      cpu_load_pct=(round(cpu_pct, 1)
                                    if cpu_pct is not None else None),
                      source=source)
        ok = free_gb >= min_free_ram_gb and (
            cpu_pct is None or cpu_pct <= max_cpu_load_pct)
        return ok, detail
    detail["unmeasurable"] = True
    return True, detail


# --- rate-limit / usage-limit recognition ------------------------------------

# Substrings (lowercased match) that mark a `claude -p` failure as a
# usage/rate limit rather than a failed run. Deliberately narrow: a false
# positive would retry (and re-bill) a genuinely failed session.
_RATE_LIMIT_MARKERS = (
    "rate limit", "rate_limit", "rate-limit",
    "usage limit", "usage_limit", "usage-limit",
    "session limit",
    "limit reached", "limit will reset", "out of extra usage",
    "overloaded_error", "server overloaded",
    "too many requests",
    "quota exceeded",
)

# `429` USED TO LIVE IN THE LIST ABOVE AS A BARE SUBSTRING, AND IT COST A CELL.
#
# MEASURED 2026-08-12, `20260812_round1_a1_omnisim_c2`: the operator killed the
# cell's Claude child. Windows reports a killed process as
# **rc=4294967295** (0xFFFFFFFF), `run_claude_cell` writes that number into its
# own `launch_error` prose, `run_cell` fed that prose to this function -- and
# "4294967295" CONTAINS "429". The cell recorded
# `rate_limit_deferrals: [{"marker": "429"}]`, tore down the workspace it had
# just preserved, and slept 900 s holding the task lock: no artifact, no grade,
# no repo sweep, no row. That is defect 4 of the diagnostic round, and it is
# what made early-stopping a cell unusable.
#
# So the HTTP status is matched as a status: digit-bounded, never as a
# substring of a longer number. (`\b` is not enough -- `\b429\b` does not match
# inside "4294967295" either, but it DOES match "1429" nowhere and "429," yes;
# the explicit digit lookarounds say the intent out loud.)
_HTTP_429 = re.compile(r"(?<![0-9])429(?![0-9])")

# The SUBSCRIPTION cap arrives with a non-ASCII separator and its own
# wording: "You've hit your session limit · resets 7:10pm
# (Europe/Berlin)" (measured 2026-08-01: phasew_cc_v1 A1:omnisim r8 and
# phasew_cc_v1_B A1:webots r7 -- both misclassified as artifact-discovery
# blockers and burned, because no marker above matched). The patterns below
# run on the same ERROR-shaped, lowercased blob as the markers and tolerate
# the separator char whatever it decodes to (U+00B7, U+FFFD, mojibake --
# any non-alphanumeric run bridges it), spacing/hyphen variants of the
# limit wordings, and the "resets ..."/"resets at ..." tail.
_RATE_LIMIT_PATTERNS = tuple(re.compile(p) for p in (
    r"\b(?:session|usage|rate)[\s_-]{1,3}limit\b",
    r"\bhit your\b[^.\n]{0,40}\blimit\b",
    r"\blimit\b[^a-z0-9\n]{1,8}\bresets?\b",
    r"\bresets? at\b",
))


def rate_limit_reason(result_json, stderr_text=""):
    """The matching marker/pattern when the attempt was limit-refused, else
    None.

    Reads the headless JSON result (``is_error`` + ``result`` text) and the
    stderr of a launch that produced no JSON at all. Only an ERROR-shaped
    outcome can be a rate limit: a clean result containing the words "rate
    limit" (an agent discussing limits) is never deferred.
    """
    texts = []
    if isinstance(result_json, dict) and result_json.get("is_error"):
        texts.append(str(result_json.get("result") or ""))
        texts.append(str(result_json.get("subtype") or ""))
    if stderr_text:
        texts.append(str(stderr_text))
    blob = "\n".join(texts).lower()
    if not blob.strip():
        return None
    for marker in _RATE_LIMIT_MARKERS:
        if marker in blob:
            return marker
    if _HTTP_429.search(blob):
        return "429"
    for pat in _RATE_LIMIT_PATTERNS:
        m = pat.search(blob)
        if m:
            return m.group(0)
    return None


# --- the ACTIVE-CELL REGISTRY: who else on this machine is a live cell? -------
#
# WHY THIS EXISTS. Until 2026-08-12 the lane's containment scoped FILES and said
# nothing about PROCESSES, so a cell's hygiene sweeps ran machine-wide.
# MEASURED that day, `20260812_round1_r4_omnisim_c1`: its `post_session` port
# sweep terminated the harness, BOTH engines, ten controllers and three
# supervisors of a CONCURRENT `A1` cell -- one of them an `omnisim-bin.exe`
# whose argv names `instances\20260812_102807_omnisim_A1` -- and its repo sweep
# `preserved_and_deleted` that cell's LIVE deliverable into its own directory.
# The victim's agent then spent four tool calls hunting a capture-service bug
# that did not exist.
#
# The sweeps were right to exist (a harness leaked by a FINISHED cell poisons
# every later cell -- measured, phasew_cc_v1 B2 r0-r2). What they lacked was a
# way to tell a leak from somebody else's live work. This registry is that
# way: every running cell publishes a claim, and a sweep spares anything a
# LIVE claim covers while still reaping the unclaimed.
#
# Deliberately the same mechanics as the locks above -- a JSON file per cell
# under the shared lock root, liveness by the same pid probe, stale entries
# pruned on read. No daemon, no dependency, and a crashed cell disarms nothing
# (its claim stops protecting the moment its pid is gone).

ACTIVE_DIRNAME = "active"


def _active_dir(lock_root):
    p = Path(lock_root) / ACTIVE_DIRNAME
    p.mkdir(parents=True, exist_ok=True)
    return p


def register_cell(lock_root, *, lane, sim, task, workspace, run_dir,
                  pid=None, started_ts=None):
    """Publish this cell's claim. Returns the claim file path.

    Written as early as the workspace is known and refreshed whenever it
    changes (a rate-limit deferral instantiates a new one), so the window in
    which another lane's sweep could see an unclaimed process of ours is the
    instantiation itself.
    """
    pid = int(pid if pid is not None else os.getpid())
    rec = {"pid": pid, "lane": lane, "sim": sim, "task": task,
           "workspace": str(workspace) if workspace else None,
           "run_dir": str(run_dir) if run_dir else None,
           "started_ts": float(started_ts if started_ts is not None
                               else time.time()),
           "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    path = _active_dir(lock_root) / ("cell_%d.json" % pid)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(rec, indent=2), encoding="utf-8")
    os.replace(tmp, path)
    return path


def unregister_cell(path):
    """Drop a claim. Never raises -- a claim that outlives its process is
    pruned on the next read anyway."""
    try:
        Path(path).unlink()
        return True
    except OSError:
        return False


def active_cells(lock_root, *, exclude_pid=None, alive=None):
    """Every LIVE cell claim under ``lock_root``, newest first.

    ``alive`` is injectable so the ownership rules can be tested without
    spawning processes. Claims whose pid is dead are pruned from disk: a
    crashed cell must not protect a leaked harness for the rest of the
    campaign.
    """
    alive = alive or _pid_alive
    out = []
    d = Path(lock_root) / ACTIVE_DIRNAME
    if not d.is_dir():
        return out
    for p in sorted(d.glob("cell_*.json")):
        try:
            rec = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if not alive(rec.get("pid")):
            try:
                p.unlink()
            except OSError:
                pass
            continue
        if exclude_pid is not None and rec.get("pid") == exclude_pid:
            continue
        rec["claim_file"] = str(p)
        out.append(rec)
    out.sort(key=lambda r: r.get("started_ts") or 0.0, reverse=True)
    return out


def oldest_active_start(cells):
    """The earliest ``started_ts`` among ``cells``, or None.

    This is the timestamp the repo sweep protects behind: any untracked file
    in the real tree that is NEWER than it may belong to a cell that is still
    running, and a preserve-then-delete of somebody's live deliverable is not
    hygiene, it is destruction (measured: r4_c1 took A1's `husky_random_10.wbt`).
    """
    stamps = [c.get("started_ts") for c in cells
              if isinstance(c.get("started_ts"), (int, float))]
    return min(stamps) if stamps else None
